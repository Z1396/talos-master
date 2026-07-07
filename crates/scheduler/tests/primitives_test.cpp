// 标准算法库（ranges::all_of）
#include <algorithm>
// GoogleTest 单元测试框架
#include <gtest/gtest.h>

// 固定大小数组
#include <array>
// 多线程原子同步
#include <atomic>
// 基础整数类型
#include <cstdint>
// 标准输出流 std::cout
#include <iostream>
// 线程库 std::thread
#include <thread>
// 动态数组存储线程
#include <vector>

// 性能埋点延迟直方图
#include "primitive/performance_probe.hpp"
// SPMC三缓冲无锁多生产者单消费者通道
#include "primitive/spmc_triple_buffer.hpp"

// 简化命名空间
using namespace talos;
// 时间字面量 100ms
using namespace std::chrono_literals;

namespace {

/**
 * @brief RAII 标准输出重定向守卫
 * 构造时劫持std::cout输出到指定流，析构自动恢复原始输出缓冲区
 * 禁用拷贝/移动，单一作用域独占生命周期
 */
class CoutRedirect {
public:
    /**
     * @brief 构造：保存原始cout缓冲区，替换为目标流缓冲区
     * @param to 输出重定向目标流
     */
    explicit CoutRedirect(const std::ostream& to)
        : old_(std::cout.rdbuf(to.rdbuf())) {}

    /**
     * @brief 析构：RAII自动恢复cout原始输出缓冲区
     */
    ~CoutRedirect() { std::cout.rdbuf(old_); }

    // 禁用拷贝构造
    CoutRedirect(const CoutRedirect&)            = delete;
    // 禁用拷贝赋值
    CoutRedirect& operator=(const CoutRedirect&) = delete;
    // 禁用移动构造
    CoutRedirect(CoutRedirect&&)                 = delete;
    // 禁用移动赋值
    CoutRedirect& operator=(CoutRedirect&&)      = delete;

private:
    // 保存原始std::cout缓冲区指针
    std::streambuf* old_;
};

} // 匿名测试隔离命名空间

/**
 * @brief 三缓冲测试数据结构体
 * 携带版本号gen + 32个填充数组，用于校验快照完整一致性
 */
struct GenData {
    // 数据生成版本号，每次write自增
    std::uint64_t gen{0};
    // 32位填充数组，全部填充与gen相同的值，校验快照无撕裂
    std::array<std::uint64_t, 32> pad{};

    /**
     * @brief 构造：用指定版本填充全部pad数组
     * @param g 当前生成版本号
     */
    explicit GenData(const std::uint64_t g)
        : gen(g) {
        pad.fill(g);
    }

    /**
     * @brief 校验当前快照数据完整无撕裂
     * @return true 全部pad等于gen，快照完整；false 存在部分旧值，数据撕裂
     */
    [[nodiscard]] bool consistent() const noexcept {
        // 遍历全部数组，所有元素必须等于gen
        return std::ranges::all_of(pad, [gen = this->gen](const auto v) { return v == gen; });
    }
};

/**
 * @brief 测试：多并发读取器读取SPMC三缓冲，快照保证完整一致无撕裂
 * 业务约束：SPMC TripleBuffer允许多读单写，任意时刻读取到完整未撕裂快照
 * 测试流程：
 * 1. 创建三缓冲读写句柄对
 * 2. 启动1个写入线程，持续递增版本写入GenData
 * 3. 启动4个并发读取线程，循环读取快照做两层校验
 *    ① 快照内部pad全部匹配gen，无半更新撕裂
 *    ② 版本号单调不回退，不会读到更旧的历史快照
 * 4. 运行100ms后停止所有线程，校验无任何违规标记
 */
TEST(SpmcTripleBufferTest, ConcurrentReadersSeeConsistentSnapshots) {
    // 创建SPMC三缓冲，返回写入句柄w、读取句柄r
    auto [w, r] = primitive::SpmcTripleBuffer<GenData>::create();

    // 全局停止标记，原子多线程同步
    std::atomic stop{false};
    // 并发读取线程数量
    constexpr int kReaders = 4;

    // 写入线程：持续生成递增版本数据写入缓冲
    std::thread writer([&] {
        std::uint64_t g = 1;
        // acquire读取停止标记，同步可见stop.store
        while (!stop.load(std::memory_order_acquire)) {
            w.write(GenData(g++));
        }
    });

    // 全局违规标记，任意读取线程发现数据撕裂/版本回退置true
    std::atomic violated{false};
    std::vector<std::thread> readers;
    readers.reserve(kReaders);
    // 循环创建4个读取线程
    for (int i = 0; i < kReaders; ++i) {
        // 克隆读取句柄（SPMC多读支持clone复制）
        auto reader = r.clone();
        readers.emplace_back([reader = std::move(reader), &stop, &violated]() mutable {
            // 记录上一次读到的版本号
            std::uint64_t last = 0;
            while (!stop.load(std::memory_order_acquire)) {
                // 读取当前快照，有新数据返回optional包含数据
                if (const auto v = reader.read()) {
                    // 校验1：快照内部填充数据不一致，存在撕裂
                    if (!v->consistent()) {
                        violated.store(true, std::memory_order_relaxed);
                    }
                    // 校验2：当前版本小于上一次，快照版本回退，时序错误
                    if (v->gen < last) {
                        violated.store(true, std::memory_order_relaxed);
                    }
                    // 更新上一次版本
                    last = v->gen;
                }
            }
        });
    }

    // 读写并发运行100毫秒
    std::this_thread::sleep_for(100ms);
    // release写入停止标记，所有线程acquire可见
    stop.store(true, std::memory_order_release);

    // 等待写入线程退出
    writer.join();
    // 等待全部读取线程退出
    for (auto& th : readers) {
        th.join();
    }

    // 断言全程无数据撕裂、版本回退违规
    EXPECT_FALSE(violated.load(std::memory_order_acquire));
}

/**
 * @brief 测试：延迟直方图总采样计数持续累加，窗口仅保留最近样本
 * 逻辑：
 * 1. LatencyHistogram 存在滑动采样窗口SAMPLE_WINDOW，窗口内样本用于统计分位数；
 * 2. 全局总计数count持续累加，不受窗口截断影响；
 * 3. 写入样本总数 = 窗口大小 + 257，校验：
 *    - 总计数 = 全部写入样本；
 *    - 参与统计样本 = 窗口最大值；
 *    - min=1，max=总样本数
 */
TEST(LatencyHistogramTest, CountKeepsGrowingPastSampleWindow) {
    primitive::LatencyHistogram hist;

    // 总采样数 = 窗口容量 + 257，超出窗口长度
    const auto total_samples = primitive::LatencyHistogram::SAMPLE_WINDOW + 257U;
    for (size_t i = 0; i < total_samples; ++i) {
        // 记录延迟值 1 ~ total_samples
        hist.record(static_cast<std::uint64_t>(i + 1));
    }

    // 计算统计指标
    const auto stats = hist.compute();
    // 总采样计数等于全部写入样本，不截断
    EXPECT_EQ(stats.count, total_samples);
    // 参与分位数计算的样本仅保留窗口内最新数据
    EXPECT_EQ(stats.sample_count, primitive::LatencyHistogram::SAMPLE_WINDOW);
    // 最小延迟为第一条样本1
    EXPECT_EQ(stats.min_ns, 1U);
    // 最大延迟等于最后一条样本
    EXPECT_EQ(stats.max_ns, static_cast<std::uint64_t>(total_samples));
}

} // namespace talos::scheduler::rclcompat