#pragma once
// GoogleTest 单元测试框架
#include <gtest/gtest.h>

// 固定大小数组
#include <array>
// 多线程原子同步
#include <atomic>
// 线程同步门闩，多线程同时启动
#include <latch>
// 字符串
#include <string>
// 标准线程库
#include <thread>
// 动态数组存储线程
#include <vector>

// SPSC单生产者单消费者三缓冲无锁通道
#include "primitive/spsc_triple_buffer.hpp"
// SPMC单生产者多消费者三缓冲无锁通道
#include "primitive/spmc_triple_buffer.hpp"

// 简化命名空间，省略talos::primitive前缀
using namespace talos::primitive;

/**
 * @brief 数据完整性校验结构体，用于检测三缓冲快照撕裂
 * 内部64个uint64_t字段全部填充同一个数值，读取时校验全部字段相等
 * 若存在半写快照，会出现部分字段旧值、部分新值，判定为数据损坏
 */
struct IntegrityData {
    // 字段总数量
    static constexpr size_t kNumFields = 64;
    std::array<uint64_t, kNumFields> fields{};

    // 默认构造，全0初始化
    IntegrityData() = default;
    /**
     * @brief 构造：所有字段统一填充传入val
     * @param val 统一填充的版本序列号
     */
    explicit IntegrityData(uint64_t val) { fields.fill(val); }

    /**
     * @brief 校验快照完整无撕裂
     * @return true 全部字段一致，快照完整；false 存在新旧混合，数据损坏
     */
    [[nodiscard]] bool is_consistent() const {
        // 全0未初始化数据直接判定合法
        if (fields[0] == 0)
            return true;
        // 遍历所有字段，必须与首字段相等
        for (size_t i = 1; i < kNumFields; ++i) {
            if (fields[i] != fields[0])
                return false;
        }
        return true;
    }

    /**
     * @brief 获取当前快照统一版本号
     * @return 首字段存储的序列号
     */
    [[nodiscard]] uint64_t value() const { return fields[0]; }
};

// ============================================================================
// SPSC TripleBuffer 单生产者单消费者 测试用例
// ============================================================================
/**
 * @brief 基础读写测试：写入一条数据，读取校验值匹配
 */
TEST(SpscTest, BasicWriteRead) {
    // 创建SPSC三缓冲，返回写入句柄w、读取句柄r
    auto [w, r] = SpscTripleBuffer<int>::create();

    // 写入整数42
    w.write(42);

    // 读取快照
    const auto val = r.read();
    // 必须存在有效数据
    ASSERT_TRUE(val.has_value());
    // 数值等于写入值
    EXPECT_EQ(*val, 42);
}

/**
 * @brief 无数据时read返回std::nullopt空值
 */
TEST(SpscTest, NoDataReturnsNullopt) {
    auto [w, r] = SpscTripleBuffer<int>::create();
    // 无写入，读取无有效值
    EXPECT_FALSE(r.read().has_value());
}

/**
 * @brief 连续多次写入，读取仅获取最新快照，旧数据被覆盖丢弃
 */
TEST(SpscTest, MultipleWrites) {
    auto [w, r] = SpscTripleBuffer<int>::create();

    w.write(1);
    w.write(2);
    w.write(3);

    // 只能读到最后写入的3
    const auto val = r.read();
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 3);

    // 读完后无剩余新数据
    EXPECT_FALSE(r.read().has_value());
}

/**
 * @brief borrow_mut 临时独占缓冲区修改 + publish发布快照
 * 不直接write，而是获取可变引用修改后手动提交
 */
TEST(SpscTest, BorrowMut) {
    auto [w, r] = SpscTripleBuffer<int>::create();

    // 获取缓冲区可变引用赋值
    w.borrow_mut() = 99;
    // 提交快照，对外可见
    w.publish();

    const auto val = r.read();
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 99);
}

/**
 * @brief 移动语义测试：std::string右值写入，无拷贝
 */
TEST(SpscTest, MoveSemantics) {
    auto [w, r] = SpscTripleBuffer<std::string>::create();

    std::string data = "hello world";
    // 移动传入，不拷贝字符串
    w.write(std::move(data));

    const auto val = r.read();
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, "hello world");
}

/**
 * @brief 单生产者单消费者并发压测，多线程无锁读写
 */
TEST(SpscTest, Concurrent) {
    auto [w, r] = SpscTripleBuffer<int>::create();

    // 线程停止标记
    std::atomic stop{false};
    // 读取到的最后序列号
    std::atomic last_read{-1};
    // 总写入迭代次数
    constexpr int kIterations = 100000;

    // 生产者线程：循环写入0~99999
    std::thread producer([&w, &stop] {
        for (int i = 0; i < kIterations; ++i) {
            w.write(i);
        }
        // 写入完成，置停止标记release同步
        stop.store(true, std::memory_order_release);
    });

    // 消费者线程：循环读取快照更新last_read
    std::thread consumer([&r, &stop, &last_read] {
        while (!stop.load(std::memory_order_acquire)) {
            if (auto val = r.read()) {
                last_read.store(*val, std::memory_order_relaxed);
            }
        }
        // 退出循环后再读一次剩余数据
        if (const auto val = r.read()) {
            last_read.store(*val, std::memory_order_relaxed);
        }
    });

    // 等待线程回收
    producer.join();
    consumer.join();

    // 至少读到一半数据，证明并发读写正常工作
    EXPECT_GT(last_read.load(), kIterations / 2);
}

/**
 * @brief SPSC并发数据完整性测试，校验无撕裂快照
 * 多线程持续读写200ms，统计损坏快照数量，预期0损坏
 */
TEST(SpscTest, DataIntegrity) {
    auto [w, r] = SpscTripleBuffer<IntegrityData>::create();

    std::atomic stop{false};
    // 损坏快照计数
    std::atomic<uint64_t> corruptions{0};
    // 总读取次数
    std::atomic<uint64_t> reads{0};
    // 并发运行时长
    constexpr int kDurationMs = 200;

    // 生产者：持续递增版本写入完整填充结构体
    std::thread producer([&w, &stop] {
        uint64_t counter = 1;
        while (!stop.load(std::memory_order_acquire)) {
            w.write(IntegrityData(counter++));
        }
    });

    // 消费者：读取校验完整性，损坏则计数+1
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

    // 并发运行200ms
    std::this_thread::sleep_for(std::chrono::milliseconds(kDurationMs));
    // 发送停止信号
    stop.store(true, std::memory_order_release);

    producer.join();
    consumer.join();

    // 损坏数必须为0
    EXPECT_EQ(corruptions.load(), 0) << "Data corruption detected!";
    // 至少读到过数据
    EXPECT_GT(reads.load(), 0) << "No data was read";
}

// ============================================================================
// SPMC TripleBuffer 单生产者多消费者 测试用例
// ============================================================================
/**
 * @brief SPMC基础读写，单写单读
 */
TEST(SpmcTest, BasicWriteRead) {
    auto [w, r] = SpmcTripleBuffer<int>::create();

    w.write(42);

    const auto val = r.read();
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 42);
}

/**
 * @brief 无数据返回空optional
 */
TEST(SpmcTest, NoDataReturnsNullopt) {
    auto [w, r] = SpmcTripleBuffer<int>::create();
    EXPECT_FALSE(r.read().has_value());
}

/**
 * @brief 多次写入仅保留最新快照
 */
TEST(SpmcTest, MultipleWrites) {
    auto [w, r] = SpmcTripleBuffer<int>::create();

    w.write(1);
    w.write(2);
    w.write(3);

    const auto val = r.read();
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 3);

    EXPECT_FALSE(r.read().has_value());
}

/**
 * @brief SPMC读取句柄可拷贝clone，多句柄独立维护读取游标
 */
TEST(SpmcTest, ReadHandleCopyable) {
    auto [w, r] = SpmcTripleBuffer<int>::create();

    w.write(42);

    // 拷贝生成两个独立读取句柄
    auto r2 = r.clone();
    auto r3 = r;

    // 三个句柄均可读到同一份快照，互不影响
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

/**
 * @brief 写入移动语义，多读取句柄独立推进游标
 */
TEST(SpmcTest, MoveInCopyOutAndReadersAdvanceIndependently) {
    auto [w, r] = SpmcTripleBuffer<std::string>::create();

    auto r2 = r.clone();

    std::string payload = "hello world";
    w.write(std::move(payload));

    // 两个读取器各自读取，互不干扰对方游标
    const auto v1 = r.read();
    const auto v2 = r2.read();

    ASSERT_TRUE(v1.has_value());
    ASSERT_TRUE(v2.has_value());
    EXPECT_EQ(*v1, "hello world");
    EXPECT_EQ(*v2, "hello world");

    // 全部读完无新数据
    EXPECT_FALSE(r.has_new());
    EXPECT_FALSE(r2.has_new());
}

/**
 * @brief 多读取器版本游标独立，一个读完不影响另一个仍能读到旧快照
 */
TEST(SpmcTest, IndependentGeneration) {
    auto [w, r] = SpmcTripleBuffer<int>::create();

    auto r2 = r.clone();

    w.write(1);

    // r读取后游标前进
    const auto v1 = r.read();
    ASSERT_TRUE(v1.has_value());

    // r2仍存在未读快照
    EXPECT_TRUE(r2.has_new());
    const auto v2 = r2.read();
    ASSERT_TRUE(v2.has_value());

    // 两者都无新数据
    EXPECT_FALSE(r.has_new());
    EXPECT_FALSE(r2.has_new());
}

/**
 * @brief has_new() 判断是否存在未读取快照
 */
TEST(SpmcTest, HasNew) {
    auto [w, r] = SpmcTripleBuffer<int>::create();

    EXPECT_FALSE(r.has_new());
    w.write(1);
    EXPECT_TRUE(r.has_new());
    static_cast<void>(r.read());
    EXPECT_FALSE(r.has_new());
}

/**
 * @brief 多消费者并发压测：4个读取线程同时消费同一生产者
 */
TEST(SpmcTest, ConcurrentMultipleConsumers) {
    auto [w, r] = SpmcTripleBuffer<int>::create();

    std::atomic stop{false};
    // 总读取次数统计
    std::atomic total_reads{0};
    // 写入总迭代
    constexpr int kIterations   = 10000;
    // 并发消费者数量
    constexpr int kNumConsumers = 4;

    // 同步门闩：生产者+4消费者共5线程同时启动
    std::latch start_latch(kNumConsumers + 1);

    // 生产者线程
    std::thread producer([&w, &stop, &start_latch] {
        start_latch.arrive_and_wait();
        for (int i = 0; i < kIterations; ++i) {
            w.write(i);
        }
        stop.store(true, std::memory_order_release);
    });

    std::vector<std::thread> consumers;
    consumers.reserve(kNumConsumers);
    // 批量创建4个读取线程，每个持有独立clone读取句柄
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

    // 至少读到部分数据
    EXPECT_GT(total_reads.load(), 0);
}

/**
 * @brief SPMC字符串高并发压力测试，3个消费者持续读取校验字符串前缀
 */
TEST(SpmcTest, StressTest) {
    auto [w, r] = SpmcTripleBuffer<std::string>::create();

    std::atomic stop{false};
    constexpr int kDurationMs   = 100;
    constexpr int kNumConsumers = 3;

    std::latch start_latch(kNumConsumers + 1);

    // 生产者持续写入data_数字字符串
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
                    // 校验字符串前缀合法
                    EXPECT_TRUE(val->starts_with("data_"));
                }
            }
        });
    }

    // 压测运行100ms
    std::this_thread::sleep_for(std::chrono::milliseconds(kDurationMs));
    stop.store(true, std::memory_order_release);

    producer.join();
    for (auto& t : consumers) {
        t.join();
    }
}

/**
 * @brief SPMC多消费者数据完整性测试，4线程并发读取无撕裂快照
 */
TEST(SpmcTest, DataIntegrity) {
    auto [w, r] = SpmcTripleBuffer<IntegrityData>::create();

    std::atomic stop{false};
    std::atomic<uint64_t> corruptions{0};
    std::atomic<uint64_t> total_reads{0};
    constexpr int kDurationMs   = 200;
    constexpr int kNumConsumers = 4;

    std::latch start_latch(kNumConsumers + 1);

    // 生产者持续写入递增完整填充结构体
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

    // 并发运行200ms
    std::this_thread::sleep_for(std::chrono::milliseconds(kDurationMs));
    stop.store(true, std::memory_order_release);

    producer.join();
    for (auto& t : consumers) {
        t.join();
    }

    // 损坏快照数量必须为0
    EXPECT_EQ(corruptions.load(), 0) << "Data corruption detected!";
    // 至少读取到数据
    EXPECT_GT(total_reads.load(), 0) << "No data was read";
}