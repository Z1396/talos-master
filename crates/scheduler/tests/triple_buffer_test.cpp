#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <latch>
#include <string>
#include <thread>
#include <vector>

#include "primitive/spmc_triple_buffer.hpp"
#include "primitive/spsc_triple_buffer.hpp"

using namespace talos::primitive;

// Data structure for integrity testing - all fields should have same value
struct IntegrityData {
    static constexpr size_t kNumFields = 64;
    std::array<uint64_t, kNumFields> fields{};

    IntegrityData() = default;
    explicit IntegrityData(uint64_t val) { fields.fill(val); }

    [[nodiscard]] bool is_consistent() const {
        if (fields[0] == 0)
            return true; // uninitialized is ok
        for (size_t i = 1; i < kNumFields; ++i) {
            if (fields[i] != fields[0])
                return false;
        }
        return true;
    }

    [[nodiscard]] uint64_t value() const { return fields[0]; }
};

// ============================================================================
// SPSC Tests
// ============================================================================

TEST(SpscTest, BasicWriteRead) {
    auto [w, r] = SpscTripleBuffer<int>::create();

    w.write(42);

    const auto val = r.read();
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 42);
}

TEST(SpscTest, NoDataReturnsNullopt) {
    auto [w, r] = SpscTripleBuffer<int>::create();
    EXPECT_FALSE(r.read().has_value());
}

TEST(SpscTest, MultipleWrites) {
    auto [w, r] = SpscTripleBuffer<int>::create();

    w.write(1);
    w.write(2);
    w.write(3);

    const auto val = r.read();
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 3);

    // No more new data
    EXPECT_FALSE(r.read().has_value());
}

TEST(SpscTest, BorrowMut) {
    auto [w, r] = SpscTripleBuffer<int>::create();

    w.borrow_mut() = 99;
    w.publish();

    const auto val = r.read();
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 99);
}

TEST(SpscTest, MoveSemantics) {
    auto [w, r] = SpscTripleBuffer<std::string>::create();

    std::string data = "hello world";
    w.write(std::move(data));

    const auto val = r.read();
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, "hello world");
}

TEST(SpscTest, Concurrent) {
    auto [w, r] = SpscTripleBuffer<int>::create();

    std::atomic stop{false};
    std::atomic last_read{-1};
    constexpr int kIterations = 100000;

    std::thread producer([&w, &stop] {
        for (int i = 0; i < kIterations; ++i) {
            w.write(i);
        }
        stop.store(true, std::memory_order_release);
    });

    std::thread consumer([&r, &stop, &last_read] {
        while (!stop.load(std::memory_order_acquire)) {
            if (auto val = r.read()) {
                last_read.store(*val, std::memory_order_relaxed);
            }
        }
        if (const auto val = r.read()) {
            last_read.store(*val, std::memory_order_relaxed);
        }
    });

    producer.join();
    consumer.join();

    EXPECT_GT(last_read.load(), kIterations / 2);
}

TEST(SpscTest, DataIntegrity) {
    auto [w, r] = SpscTripleBuffer<IntegrityData>::create();

    std::atomic stop{false};
    std::atomic<uint64_t> corruptions{0};
    std::atomic<uint64_t> reads{0};
    constexpr int kDurationMs = 200;

    std::thread producer([&w, &stop] {
        uint64_t counter = 1;
        while (!stop.load(std::memory_order_acquire)) {
            w.write(IntegrityData(counter++));
        }
    });

    std::thread consumer([&r, &stop, &corruptions, &reads] {
        while (!stop.load(std::memory_order_acquire)) {
            if (const auto val = r.read()) {
                reads.fetch_add(1, std::memory_order_relaxed);
                if (!val->is_consistent()) {
                    corruptions.fetch_add(1, std::memory_order_relaxed);
                }
            }
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(kDurationMs));
    stop.store(true, std::memory_order_release);

    producer.join();
    consumer.join();

    EXPECT_EQ(corruptions.load(), 0) << "Data corruption detected!";
    EXPECT_GT(reads.load(), 0) << "No data was read";
}

// ============================================================================
// SPMC Tests
// ============================================================================

TEST(SpmcTest, BasicWriteRead) {
    auto [w, r] = SpmcTripleBuffer<int>::create();

    w.write(42);

    const auto val = r.read();
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 42);
}

TEST(SpmcTest, NoDataReturnsNullopt) {
    auto [w, r] = SpmcTripleBuffer<int>::create();
    EXPECT_FALSE(r.read().has_value());
}

TEST(SpmcTest, MultipleWrites) {
    auto [w, r] = SpmcTripleBuffer<int>::create();

    w.write(1);
    w.write(2);
    w.write(3);

    const auto val = r.read();
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 3);

    // No more new data
    EXPECT_FALSE(r.read().has_value());
}

TEST(SpmcTest, ReadHandleCopyable) {
    auto [w, r] = SpmcTripleBuffer<int>::create();

    w.write(42);

    auto r2 = r.clone();
    auto r3 = r;

    const auto v1 = r.read();
    const auto v2 = r2.read();
    const auto v3 = r3.read();

    ASSERT_TRUE(v1.has_value());
    ASSERT_TRUE(v2.has_value());
    ASSERT_TRUE(v3.has_value());

    EXPECT_EQ(*v1, 42);
    EXPECT_EQ(*v2, 42);
    EXPECT_EQ(*v3, 42);
}
TEST(SpmcTest, MoveInCopyOutAndReadersAdvanceIndependently) {
    auto [w, r] = SpmcTripleBuffer<std::string>::create();

    auto r2 = r.clone();

    std::string payload = "hello world";
    w.write(std::move(payload));

    const auto v1 = r.read();
    const auto v2 = r2.read();

    ASSERT_TRUE(v1.has_value());
    ASSERT_TRUE(v2.has_value());
    EXPECT_EQ(*v1, "hello world");
    EXPECT_EQ(*v2, "hello world");

    EXPECT_FALSE(r.has_new());
    EXPECT_FALSE(r2.has_new());
}

TEST(SpmcTest, IndependentGeneration) {
    auto [w, r] = SpmcTripleBuffer<int>::create();

    auto r2 = r.clone();

    w.write(1);

    const auto v1 = r.read();
    ASSERT_TRUE(v1.has_value());

    EXPECT_TRUE(r2.has_new());
    const auto v2 = r2.read();
    ASSERT_TRUE(v2.has_value());

    EXPECT_FALSE(r.has_new());
    EXPECT_FALSE(r2.has_new());
}

TEST(SpmcTest, HasNew) {
    auto [w, r] = SpmcTripleBuffer<int>::create();

    EXPECT_FALSE(r.has_new());
    w.write(1);
    EXPECT_TRUE(r.has_new());
    static_cast<void>(r.read());
    EXPECT_FALSE(r.has_new());
}

TEST(SpmcTest, ConcurrentMultipleConsumers) {
    auto [w, r] = SpmcTripleBuffer<int>::create();

    std::atomic stop{false};
    std::atomic total_reads{0};
    constexpr int kIterations   = 10000;
    constexpr int kNumConsumers = 4;

    std::latch start_latch(kNumConsumers + 1);

    std::thread producer([&w, &stop, &start_latch] {
        start_latch.arrive_and_wait();
        for (int i = 0; i < kIterations; ++i) {
            w.write(i);
        }
        stop.store(true, std::memory_order_release);
    });

    std::vector<std::thread> consumers;
    consumers.reserve(kNumConsumers);
    for (int c = 0; c < kNumConsumers; ++c) {
        auto reader = r.clone();
        consumers.emplace_back(
            [reader = std::move(reader), &stop, &total_reads, &start_latch]() mutable {
                start_latch.arrive_and_wait();
                while (!stop.load(std::memory_order_acquire)) {
                    if (auto val = reader.read()) {
                        total_reads.fetch_add(1, std::memory_order_relaxed);
                        [[maybe_unused]] volatile int x = *val;
                    }
                }
            });
    }

    producer.join();
    for (auto& t : consumers) {
        t.join();
    }

    EXPECT_GT(total_reads.load(), 0);
}

TEST(SpmcTest, StressTest) {
    auto [w, r] = SpmcTripleBuffer<std::string>::create();

    std::atomic stop{false};
    constexpr int kDurationMs   = 100;
    constexpr int kNumConsumers = 3;

    std::latch start_latch(kNumConsumers + 1);

    std::thread producer([&w, &stop, &start_latch] {
        start_latch.arrive_and_wait();
        int counter = 0;
        while (!stop.load(std::memory_order_acquire)) {
            w.write("data_" + std::to_string(counter++));
        }
    });

    std::vector<std::thread> consumers;
    consumers.reserve(kNumConsumers);
    for (int c = 0; c < kNumConsumers; ++c) {
        auto reader = r.clone();
        consumers.emplace_back([reader = std::move(reader), &stop, &start_latch]() mutable {
            start_latch.arrive_and_wait();
            while (!stop.load(std::memory_order_acquire)) {
                if (const auto val = reader.read()) {
                    EXPECT_TRUE(val->starts_with("data_"));
                }
            }
        });
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kDurationMs));
    stop.store(true, std::memory_order_release);

    producer.join();
    for (auto& t : consumers) {
        t.join();
    }
}

TEST(SpmcTest, DataIntegrity) {
    auto [w, r] = SpmcTripleBuffer<IntegrityData>::create();

    std::atomic stop{false};
    std::atomic<uint64_t> corruptions{0};
    std::atomic<uint64_t> total_reads{0};
    constexpr int kDurationMs   = 200;
    constexpr int kNumConsumers = 4;

    std::latch start_latch(kNumConsumers + 1);

    std::thread producer([&w, &stop, &start_latch] {
        start_latch.arrive_and_wait();
        uint64_t counter = 1;
        while (!stop.load(std::memory_order_acquire)) {
            w.write(IntegrityData(counter++));
        }
    });

    std::vector<std::thread> consumers;
    consumers.reserve(kNumConsumers);
    for (int c = 0; c < kNumConsumers; ++c) {
        auto reader = r.clone();
        consumers.emplace_back([reader = std::move(reader), &stop, &corruptions, &total_reads,
                                &start_latch]() mutable {
            start_latch.arrive_and_wait();
            while (!stop.load(std::memory_order_acquire)) {
                if (const auto val = reader.read()) {
                    total_reads.fetch_add(1, std::memory_order_relaxed);
                    if (!val->is_consistent()) {
                        corruptions.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
        });
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kDurationMs));
    stop.store(true, std::memory_order_release);

    producer.join();
    for (auto& t : consumers) {
        t.join();
    }

    EXPECT_EQ(corruptions.load(), 0) << "Data corruption detected!";
    EXPECT_GT(total_reads.load(), 0) << "No data was read";
}
