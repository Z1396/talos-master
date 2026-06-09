#include "chiral/navigation.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#include <gtest/gtest.h>
#include <sys/mman.h>

using namespace talos::chiral;
using namespace talos::chiral::ipc;

namespace chiral_test {

using TalosData = navigation::TalosData;

struct IntegrityData {
    static constexpr size_t kNumFields = 64;
    std::array<uint64_t, kNumFields> fields{};

    IntegrityData() = default;
    explicit IntegrityData(uint64_t value) noexcept { fields.fill(value); }

    [[nodiscard]] bool is_consistent() const noexcept {
        if (fields[0] == 0) {
            return true;
        }
        for (size_t i = 1; i < kNumFields; ++i) {
            if (fields[i] != fields[0]) {
                return false;
            }
        }
        return true;
    }
};

} // namespace chiral_test

namespace talos::chiral::ipc {

template <>
struct ShmName<chiral_test::IntegrityData> {
    static constexpr const char* value = "/chiral_test_integrity";
};

} // namespace talos::chiral::ipc

namespace chiral_test {

void cleanup_test_shm() noexcept {
    (void)::shm_unlink(ShmName<TalosData>::value);
    (void)::shm_unlink(ShmName<IntegrityData>::value);
}

class TripleBufferSimpleTest : public ::testing::Test {
protected:
    void SetUp() override { cleanup_test_shm(); }
    void TearDown() override { cleanup_test_shm(); }
};

TEST_F(TripleBufferSimpleTest, BasicWriteRead) {
    auto writer = ChannelWriter<TalosData>::create();
    ASSERT_TRUE(writer.has_value());

    TalosData data{};
    data.gimbal_link.translation.x = 1.5;
    data.gimbal_link.translation.y = 2.5;
    data.gimbal_link.translation.z = 3.5;
    writer->write(data);

    auto reader = ChannelReader<TalosData>::open();
    ASSERT_TRUE(reader.has_value());

    auto read_data = reader->read_new();
    ASSERT_TRUE(read_data.has_value());
    EXPECT_DOUBLE_EQ(read_data->gimbal_link.translation.x, 1.5);
    EXPECT_DOUBLE_EQ(read_data->gimbal_link.translation.y, 2.5);
    EXPECT_DOUBLE_EQ(read_data->gimbal_link.translation.z, 3.5);
}

TEST_F(TripleBufferSimpleTest, ReadNewReturnsNulloptWhenNoData) {
    auto writer = ChannelWriter<TalosData>::create();
    ASSERT_TRUE(writer.has_value());

    auto reader = ChannelReader<TalosData>::open();
    ASSERT_TRUE(reader.has_value());

    EXPECT_FALSE(reader->read_new().has_value());
}

TEST_F(TripleBufferSimpleTest, ReadLatestReturnsZeroInitializedSnapshotBeforeFirstWrite) {
    auto writer = ChannelWriter<TalosData>::create();
    ASSERT_TRUE(writer.has_value());

    auto reader = ChannelReader<TalosData>::open();
    ASSERT_TRUE(reader.has_value());

    const auto latest = reader->read_latest();
    EXPECT_DOUBLE_EQ(latest.gimbal_link.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(latest.gimbal_link.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(latest.gimbal_link.translation.z, 0.0);
}

TEST_F(TripleBufferSimpleTest, MultipleWritesExposeLatestSnapshot) {
    auto writer = ChannelWriter<TalosData>::create();
    ASSERT_TRUE(writer.has_value());

    TalosData data{};
    data.gimbal_link.translation.x = 1.0;
    writer->write(data);

    data.gimbal_link.translation.x = 2.0;
    writer->write(data);

    data.gimbal_link.translation.x = 3.0;
    writer->write(data);

    auto reader = ChannelReader<TalosData>::open();
    ASSERT_TRUE(reader.has_value());

    auto latest = reader->read_new();
    ASSERT_TRUE(latest.has_value());
    EXPECT_DOUBLE_EQ(latest->gimbal_link.translation.x, 3.0);
    EXPECT_FALSE(reader->read_new().has_value());
}

TEST_F(TripleBufferSimpleTest, DataIntegrityNoTearing) {
    auto writer = ChannelWriter<IntegrityData>::create();
    ASSERT_TRUE(writer.has_value());

    auto reader = ChannelReader<IntegrityData>::open();
    ASSERT_TRUE(reader.has_value());

    std::atomic<bool> stop{false};
    std::atomic<uint64_t> reads{0};
    std::atomic<uint64_t> corruptions{0};

    std::thread producer([&]() {
        uint64_t counter = 1;
        while (!stop.load(std::memory_order_acquire)) {
            writer->write(IntegrityData(counter++));
        }
    });

    std::thread consumer([&]() {
        while (!stop.load(std::memory_order_acquire)) {
            if (const auto snapshot = reader->read_new()) {
                reads.fetch_add(1, std::memory_order_relaxed);
                if (!snapshot->is_consistent()) {
                    corruptions.fetch_add(1, std::memory_order_relaxed);
                }
            }
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    stop.store(true, std::memory_order_release);

    producer.join();
    consumer.join();

    EXPECT_GT(reads.load(std::memory_order_relaxed), 0U) << "No data captured in integrity test";
    EXPECT_EQ(corruptions.load(std::memory_order_relaxed), 0U)
        << "Data tearing detected in shared-memory triple buffer";
}

} // namespace chiral_test
