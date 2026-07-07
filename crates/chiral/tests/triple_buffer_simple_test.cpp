// 引入导航模块消息结构体、IPC通道定义头文件
#include "chiral/navigation.hpp"

// 定长数组容器，用于完整性校验结构体存储批量校验字段
#include <array>
// 原子操作，多线程无锁同步标记（启停、计数）
#include <atomic>
// 标准时间库，线程休眠毫秒延时
#include <chrono>
// 固定宽度无符号整数 uint64_t
#include <cstdint>
// 跨平台线程库，生产者消费者并发测试线程
#include <thread>

// Google Test 单元测试框架
#include <gtest/gtest.h>
// Linux 共享内存映射系统调用头文件
#include <sys/mman.h>

// 简化命名空间，省去长前缀重复书写
using namespace talos::chiral;
using namespace talos::chiral::ipc;

// 三缓冲单测隔离命名空间
namespace chiral_test {

// 导航模块上行消息类型别名
using TalosData = navigation::TalosData;

/**
 * @brief 内存完整性校验结构体，用于并发读写撕裂（data tearing）检测
 * 填充64个uint64统一值，若读取出存在不一致数字，证明读写并发撕裂
 */
struct IntegrityData {
    // 固定字段数量64个64位无符号整数
    static constexpr size_t kNumFields = 64;
    // 定长数组，默认零初始化
    std::array<uint64_t, kNumFields> fields{};

    // 默认构造，全部字段自动置0
    IntegrityData() = default;

    /**
     * @brief 带值构造：全部数组字段填充同一个传入value
     * @param value 统一填充的64位数字
     */
    explicit IntegrityData(uint64_t value) noexcept { fields.fill(value); }

    /**
     * @brief 判断当前数组所有字段是否完全一致，无撕裂
     * @return true 全部字段相同（数据完整无撕裂）；false 存在不一致（读写撕裂）
     */
    [[nodiscard]] bool is_consistent() const noexcept {
        // 首字段为0代表空初始化，直接判定合法
        if (fields[0] == 0) {
            return true;
        }
        // 遍历剩余63个字段，全部必须等于首字段
        for (size_t i = 1; i < kNumFields; ++i) {
            if (fields[i] != fields[0]) {
                return false;
            }
        }
        return true;
    }
};

} // namespace chiral_test

// IPC共享内存名称模板特化命名空间
namespace talos::chiral::ipc {

/**
 * @brief 为完整性测试结构体特化共享内存文件名
 * 独立SHM，和导航业务数据隔离，互不干扰
 */
template <>
struct ShmName<chiral_test::IntegrityData> {
    static constexpr const char* value = "/chiral_test_integrity";
};

} // namespace talos::chiral::ipc

namespace chiral_test {

/**
 * @brief 全局清理函数：删除两套测试用共享内存，清除测试残留脏文件
 */
void cleanup_test_shm() noexcept {
    // 导航业务TalosData共享内存
    (void)::shm_unlink(ShmName<TalosData>::value);
    // 完整性测试专用共享内存
    (void)::shm_unlink(ShmName<IntegrityData>::value);
}

/**
 * @brief 三缓冲基础功能测试夹具，所有TripleBuffer测试共用
 * 每个用例执行前后自动清理共享内存，用例环境完全隔离
 */
class TripleBufferSimpleTest : public ::testing::Test {
protected:
    // 每个TEST_F执行前回调，清理残留SHM
    void SetUp() override { cleanup_test_shm(); }
    // 每个TEST_F执行完毕后回调，再次清理防止进程异常残留
    void TearDown() override { cleanup_test_shm(); }
};

/**
 * @brief 基础单线程读写测试：写入一帧TalosData，读取并校验平移数值
 */
TEST_F(TripleBufferSimpleTest, BasicWriteRead) {
    // 创建TalosData写入生产者通道，新建对应共享内存
    auto writer = ChannelWriter<TalosData>::create();
    // 断言创建共享内存成功
    ASSERT_TRUE(writer.has_value());

    // 构造测试数据，填充云台平移三轴
    TalosData data{};
    data.gimbal_link.translation.x = 1.5;
    data.gimbal_link.translation.y = 2.5;
    data.gimbal_link.translation.z = 3.5;
    // 写入三缓冲共享内存
    writer->write(data);

    // 打开读消费者通道，映射已创建的共享内存
    auto reader = ChannelReader<TalosData>::open();
    ASSERT_TRUE(reader.has_value());

    // 读取最新未消费新帧
    auto read_data = reader->read_new();
    // 断言读取到有效数据
    ASSERT_TRUE(read_data.has_value());
    // 浮点等值校验，允许极小浮点误差
    EXPECT_DOUBLE_EQ(read_data->gimbal_link.translation.x, 1.5);
    EXPECT_DOUBLE_EQ(read_data->gimbal_link.translation.y, 2.5);
    EXPECT_DOUBLE_EQ(read_data->gimbal_link.translation.z, 3.5);
}

/**
 * @brief 无数据场景测试：写入端创建但未写入任何数据，read_new返回空std::nullopt
 */
TEST_F(TripleBufferSimpleTest, ReadNewReturnsNulloptWhenNoData) {
    auto writer = ChannelWriter<TalosData>::create();
    ASSERT_TRUE(writer.has_value());

    auto reader = ChannelReader<TalosData>::open();
    ASSERT_TRUE(reader.has_value());

    // 无任何写入，无新帧可读取，返回空
    EXPECT_FALSE(reader->read_new().has_value());
}

/**
 * @brief 首次写入前调用read_latest，返回全零初始化快照
 * 共享内存创建时全部内存清零，无旧脏数据
 */
TEST_F(TripleBufferSimpleTest, ReadLatestReturnsZeroInitializedSnapshotBeforeFirstWrite) {
    auto writer = ChannelWriter<TalosData>::create();
    ASSERT_TRUE(writer.has_value());

    auto reader = ChannelReader<TalosData>::open();
    ASSERT_TRUE(reader.has_value());

    // 获取当前最新缓冲区快照（尚未写入，全零）
    const auto latest = reader->read_latest();
    EXPECT_DOUBLE_EQ(latest.gimbal_link.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(latest.gimbal_link.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(latest.gimbal_link.translation.z, 0.0);
}

/**
 * @brief 多次连续写入测试：三缓冲自动覆盖旧帧，读取仅能拿到最新一帧
 */
TEST_F(TripleBufferSimpleTest, MultipleWritesExposeLatestSnapshot) {
    auto writer = ChannelWriter<TalosData>::create();
    ASSERT_TRUE(writer.has_value());

    // 第1帧 x=1
    TalosData data{};
    data.gimbal_link.translation.x = 1.0;
    writer->write(data);

    // 第2帧 x=2
    data.gimbal_link.translation.x = 2.0;
    writer->write(data);

    // 第3帧 x=3（最新帧）
    data.gimbal_link.translation.x = 3.0;
    writer->write(data);

    auto reader = ChannelReader<TalosData>::open();
    ASSERT_TRUE(reader.has_value());

    // 读取新帧，只能拿到最后写入的x=3
    auto latest = reader->read_new();
    ASSERT_TRUE(latest.has_value());
    EXPECT_DOUBLE_EQ(latest->gimbal_link.translation.x, 3.0);
    // 已经消费完最新帧，再次read_new无新数据
    EXPECT_FALSE(reader->read_new().has_value());
}

/**
 * @brief 并发完整性撕裂测试：多线程高速读写，验证无锁三缓冲无半写撕裂
 * 原理：生产者持续写入全统一数字的结构体，消费者循环读取校验所有字段一致
 * 若出现字段不一致，代表读取时写入正在进行，发生data tearing，无锁实现失效
 */
TEST_F(TripleBufferSimpleTest, DataIntegrityNoTearing) {
    // 创建完整性测试专用写通道
    auto writer = ChannelWriter<IntegrityData>::create();
    ASSERT_TRUE(writer.has_value());

    // 打开对应读通道
    auto reader = ChannelReader<IntegrityData>::open();
    ASSERT_TRUE(reader.has_value());

    // 原子标记：线程停止开关，多线程同步无锁
    std::atomic<bool> stop{false};
    // 成功读取帧数计数器
    std::atomic<uint64_t> reads{0};
    // 检测到撕裂损坏数据计数器
    std::atomic<uint64_t> corruptions{0};

    // 生产者写入线程：循环生成自增计数器，填充完整结构体写入共享内存
    std::thread producer([&]() {
        uint64_t counter = 1;
        // 读取停止标记，获取语义acquire保证可见性
        while (!stop.load(std::memory_order_acquire)) {
            writer->write(IntegrityData(counter++));
        }
    });

    // 消费者读取校验线程：循环读取快照，校验字段一致性
    std::thread consumer([&]() {
        while (!stop.load(std::memory_order_acquire)) {
            // 读取新生成帧
            if (const auto snapshot = reader->read_new()) {
                // 读取计数+1，宽松内存序，无需同步
                reads.fetch_add(1, std::memory_order_relaxed);
                // 校验结构体所有字段是否统一
                if (!snapshot->is_consistent()) {
                    // 发现撕裂损坏，损坏计数+1
                    corruptions.fetch_add(1, std::memory_order_relaxed);
                }
            }
        }
    });

    // 并发读写持续250毫秒，高压竞争测试
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    // 置位停止标记，release语义保证两个线程可见停止信号
    stop.store(true, std::memory_order_release);

    // 阻塞等待生产者线程执行完毕退出
    producer.join();
    // 阻塞等待消费者线程执行完毕退出
    consumer.join();

    // 断言至少读取到一帧数据，验证线程正常工作
    EXPECT_GT(reads.load(std::memory_order_relaxed), 0U) << "No data captured in integrity test";
    // 关键断言：损坏撕裂数量必须为0，证明三缓冲无锁读写安全无撕裂
    EXPECT_EQ(corruptions.load(std::memory_order_relaxed), 0U)
        << "Data tearing detected in shared-memory triple buffer";
}

} // namespace chiral_test