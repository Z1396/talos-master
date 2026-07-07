// 自研性能延时探针、延时直方图统计头文件声明
#include "primitive/performance_probe.hpp"

// 标准算法库：排序、最值
#include <algorithm>
// 高精度稳定时钟
#include <chrono>
// 数学运算 标准差开方
#include <cmath>
// 控制台标准输出打印统计
#include <iostream>
// 数值累加求和
#include <numeric>
// 动态数组存储采样用于分位数计算
#include <vector>

namespace talos::primitive {

// ===================== 单次延时计时器 LatencyProbe =====================
/**
 * @brief 单段代码块耗时探针，一次性计时工具
 * 基于 steady_clock 稳定时钟，不受系统时间修改影响
 * 仅记录起点时间戳，提供两种读取耗时方式：
 * 1. elapsed_ns() 不重置起点，持续计算从创建至今总耗时
 * 2. snapshot_and_reset() 读取耗时并重置起点，分段计时
 */
LatencyProbe::LatencyProbe() noexcept
    : start_ns_(now_ns()) {}

/**
 * @brief 获取从探针创建到当前的总纳秒耗时，不重置起点
 * @return 时间差 纳秒 uint64_t
 */
auto LatencyProbe::elapsed_ns() const noexcept -> std::uint64_t { return now_ns() - start_ns_; }

/**
 * @brief 快照式读取耗时，同时重置计时起点，用于分段循环计时
 * @return 上一次重置到本次调用之间的纳秒耗时
 */
auto LatencyProbe::snapshot_and_reset() noexcept -> std::uint64_t {
    // 获取当前高精度时间戳
    const auto now     = now_ns();
    // 计算上一次起点到当前的间隔
    const auto elapsed = now - start_ns_;
    // 更新起点为当前时间，下一轮计时从此刻开始
    start_ns_          = now;
    return elapsed;
}

/**
 * @brief 静态工具函数：获取系统稳定时钟当前纳秒时间戳
 * @return 自系统开机稳态时钟纪元起总纳秒数 uint64_t
 */
auto LatencyProbe::now_ns() noexcept -> std::uint64_t {
    // 稳定时钟 → 强制转换为纳秒精度 → 取纪元差值计数
    return std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now())
        .time_since_epoch()
        .count();
}

// ===================== 环形窗口延时直方图统计 LatencyHistogram =====================
/**
 * @brief 多线程安全延时采样直方图统计器
 * 核心特性：
 * 1. 固定大小环形滑动窗口 SAMPLE_WINDOW，仅保留最新N个采样
 * 2. 无锁原子操作实现多生产者单读取（SPSC）线程安全
 * 3. 实时维护全局最小/最大延时，原子CAS更新无锁竞争
 * 4. 支持计算：均值、标准差、P50/P95/P99/P99.9分位数
 * 5. 移动构造/移动赋值，内部原子数组完整拷贝
 */
// 移动构造，转移/拷贝另一个直方图全部采样与统计状态
LatencyHistogram::LatencyHistogram(LatencyHistogram&& other) noexcept { copy_from(other); }

/**
 * @brief 移动赋值重载，安全自判避免自身赋值
 */
auto LatencyHistogram::operator=(LatencyHistogram&& other) noexcept -> LatencyHistogram& {
    if (this != &other) {
        copy_from(other);
    }
    return *this;
}

/**
 * @brief 记录单次延时采样，多线程无锁安全写入环形缓冲区
 * @param latency_ns 本次延时纳秒值
 * 逻辑：
 * 1. 原子读取当前写入索引
 * 2. 写入环形数组对应位置（取模循环覆盖旧采样）
 * 3. 原子更新全局最小、最大延时
 * 4. 原子自增写入索引与总采样计数
 */
void LatencyHistogram::record(const std::uint64_t latency_ns) noexcept {
    // 松弛序读取当前写入下标，无同步开销
    const auto index = write_index_.load(std::memory_order_relaxed);
    // 环形覆盖：index % 窗口大小，原子写入采样值
    samples_[index % SAMPLE_WINDOW].value.store(latency_ns, std::memory_order_relaxed);
    // 原子更新全局最小、最大值
    update_min(latency_ns);
    update_max(latency_ns);
    // 写入索引+1，松弛序存储
    write_index_.store(index + 1, std::memory_order_relaxed);
    // 总采样计数同步更新，release序保证读取端可见最新采样
    count_.store(index + 1, std::memory_order_release);
}

/**
 * @brief 读取全部有效采样，计算完整统计指标结构体Stats
 * @return Stats 包含 min/max/mean/p50/p95/p99/p999/stddev/总采样数/窗口有效采样数
 */
auto LatencyHistogram::compute() const -> Stats {
    // 获取总写入采样数，acquire同步确保采样全部可见
    const auto total_count = count_.load(std::memory_order_acquire);
    // 无任何采样，返回全零空统计
    if (total_count == 0) {
        return {0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0};
    }

    // 有效采样数量：总采样数与窗口容量取最小值，避免读取未覆盖旧数据
    const auto sample_count =
        static_cast<std::size_t>(std::min<std::uint64_t>(total_count, SAMPLE_WINDOW));

    // 临时容器存储当前窗口全部有效采样，用于排序分位数计算
    std::vector<std::uint64_t> snapshot;
    snapshot.reserve(sample_count);
    // 遍历窗口全部存储单元，原子读取采样值
    for (std::size_t i = 0; i < sample_count; ++i) {
        snapshot.push_back(samples_[i].value.load(std::memory_order_relaxed));
    }

    // 升序排序采样，用于分位数索引取值
    std::ranges::sort(snapshot);

    // 读取全局最小值，若初始极大值代表无有效采样，置0
    auto min_val = min_ns_.load(std::memory_order_relaxed);
    if (min_val == std::numeric_limits<std::uint64_t>::max()) {
        min_val = 0;
    }
    // 全局最大值
    const auto max_val  = max_ns_.load(std::memory_order_relaxed);
    // 全部采样求和，uint64无溢出累加
    const auto sum      = std::accumulate(snapshot.begin(), snapshot.end(), 0ULL);
    // 算术平均值
    const auto mean_val = sum / sample_count;

    // 计算各分位数下标，防止下标越界
    const auto p50_idx  = std::min(sample_count * 50 / 100, sample_count - 1);
    const auto p95_idx  = std::min(sample_count * 95 / 100, sample_count - 1);
    const auto p99_idx  = std::min(sample_count * 99 / 100, sample_count - 1);
    const auto p999_idx = std::min(sample_count * 999 / 1000, sample_count - 1);

    // 计算标准差：平方差累加
    double sum_sq       = 0.0;
    const double mean_d = static_cast<double>(mean_val);
    for (std::size_t i = 0; i < sample_count; i++) {
        const double diff = static_cast<double>(snapshot[i]) - mean_d;
        sum_sq += diff * diff;
    }
    // 总体标准差
    const double stddev = std::sqrt(sum_sq / static_cast<double>(sample_count));

    // 填充统计结果结构体返回
    return {min_val,           max_val,           mean_val,           snapshot[p50_idx],
            snapshot[p95_idx], snapshot[p99_idx], snapshot[p999_idx], stddev,
            total_count,       sample_count};
}

/**
 * @brief 控制台格式化打印完整延时统计报表
 */
void LatencyHistogram::print_stats() const {
    const auto stats = compute();
    std::cout << "Latency Statistics (ns):\n"
              << "  Count:  " << stats.count << "\n"
              << "  Min:    " << stats.min_ns << " ns\n"
              << "  P50:    " << stats.p50_ns << " ns\n"
              << "  P95:    " << stats.p95_ns << " ns\n"
              << "  P99:    " << stats.p99_ns << " ns\n"
              << "  P99.9:  " << stats.p999_ns << " ns\n"
              << "  Mean:   " << stats.mean_ns << " ns\n"
              << "  Max:    " << stats.max_ns << " ns\n"
              << "  StdDev: " << stats.stddev_ns << " ns\n";
}

/**
 * @brief 内部拷贝辅助函数：完整复制另一个直方图全部原子状态与采样数组
 * @param other 源直方图常量引用
 * 所有原子变量使用relaxed序读写，无锁拷贝内部状态
 */
void LatencyHistogram::copy_from(const LatencyHistogram& other) noexcept {
    write_index_.store(
        other.write_index_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    count_.store(other.count_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    min_ns_.store(other.min_ns_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    max_ns_.store(other.max_ns_.load(std::memory_order_relaxed), std::memory_order_relaxed);

    // 逐个拷贝环形窗口内所有采样原子值
    for (std::size_t i = 0; i < SAMPLE_WINDOW; ++i) {
        samples_[i].value.store(
            other.samples_[i].value.load(std::memory_order_relaxed), std::memory_order_relaxed);
    }
}

/**
 * @brief 原子无锁更新全局最小延时，CAS自旋循环保证最终写入最小值
 * @param latency_ns 新采样延时
 */
void LatencyHistogram::update_min(const std::uint64_t latency_ns) noexcept {
    auto current = min_ns_.load(std::memory_order_relaxed);
    // 自旋循环：当前最小值大于新采样则尝试交换，失败重读重试
    while (latency_ns < current
           && !min_ns_.compare_exchange_weak(
               current, latency_ns, std::memory_order_relaxed, std::memory_order_relaxed)) {}
}

/**
 * @brief 原子无锁更新全局最大延时，CAS自旋循环保证最终写入最大值
 * @param latency_ns 新采样延时
 */
void LatencyHistogram::update_max(const std::uint64_t latency_ns) noexcept {
    auto current = max_ns_.load(std::memory_order_relaxed);
    while (latency_ns > current
           && !max_ns_.compare_exchange_weak(
               current, latency_ns, std::memory_order_relaxed, std::memory_order_relaxed)) {}
}

} // namespace talos::primitive