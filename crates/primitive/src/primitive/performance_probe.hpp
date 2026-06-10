#pragma once

#include <array>       // 固定大小数组容器，用于存放采样数据
#include <atomic>      // 原子类型，实现无锁并发安全
#include <cstddef>     // 标准基础类型，如 std::size_t
#include <cstdint>     // 固定宽度整型，如 uint64_t
#include <limits>      // 数值极限模板，获取类型最大/最小值

// talos 框架基础原生组件命名空间
namespace talos::primitive {

/**
 * @brief 高性能时延探测工具类
 *
 * 底层使用 std::chrono::steady_clock，多数平台下性能优于 high_resolution_clock
 * 特点：时钟单调递增、无时区修正，适合耗时统计场景。
 *
 * 设计特性：
 * - 构造时自动记录起始时间戳
 * - 支持查询自创建以来的耗时
 * - 无锁设计：临界路径无内存分配、无系统调用
 * - 跨平台：兼容 x86-64、ARM64 等主流架构
 *
 * 性能开销：编译优化后单次调用耗时约 20~40 纳秒
 */
class LatencyProbe {
public:
    /**
     * @brief 构造函数，自动记录当前时间作为计时起点
     * @note noexcept 保证构造过程不会抛出异常
     */
    LatencyProbe() noexcept;

    /**
     * @brief 析构函数，使用默认实现
     */
    ~LatencyProbe() = default;

    /**
     * @brief 获取从对象构造至今的耗时（纳秒）
     * @return 耗时，单位：纳秒
     * @note [[nodiscard]] 强制要求调用方接收返回值，避免误用
     * @note noexcept 保证函数无异常抛出
     */
    [[nodiscard]] std::uint64_t elapsed_ns() const noexcept;

    /**
     * @brief 获取当前累计耗时，并重置计时起点
     * @return 重置前的耗时，单位：纳秒
     * @note 适用于分段计时、循环轮次计时场景
     */
    std::uint64_t snapshot_and_reset() noexcept;

private:
    // 存储计时起点的时间戳，单位：纳秒
    std::uint64_t start_ns_;

    /**
     * @brief 静态工具函数，获取当前系统单调时钟时间（纳秒）
     * @return 当前时间戳，单位：纳秒
     * @note 内部封装时钟逻辑，统一时间获取入口
     */
    static std::uint64_t now_ns() noexcept;
};

/**
 * @brief 低开销时延统计直方图，用于调度器运行时指标统计
 *
 * 设计思路：
 * 1. 热路径仅维护总计数 + 固定大小滑动采样窗口
 * 2. 全程无互斥锁，规避多线程锁竞争开销
 * 3. 采样容量有界，不会随运行时间无限占用内存
 * 4. 可计算均值、中位数、分位数、标准差等指标，满足长周期任务统计需求
 */
class LatencyHistogram {
public:
    // 采样窗口最大容量：最多保存 8192 个时延样本
    static constexpr std::size_t SAMPLE_WINDOW = 8192;

    /**
     * @brief 默认构造函数
     */
    LatencyHistogram() noexcept = default;

    /**
     * @brief 移动构造函数
     * @param other 待转移资源的源对象
     * @note 支持对象资源转移，无异常抛出
     */
    LatencyHistogram(LatencyHistogram&& other) noexcept;

    /**
     * @brief 移动赋值运算符
     * @param other 待转移资源的源对象
     * @return 当前对象引用
     * @note 支持对象资源转移，无异常抛出
     */
    LatencyHistogram& operator=(LatencyHistogram&& other) noexcept;

    /**
     * @brief 拷贝构造函数 显式禁用
     * @note 原子类型不建议浅拷贝，禁止拷贝语义
     */
    LatencyHistogram(const LatencyHistogram&)            = delete;

    /**
     * @brief 拷贝赋值运算符 显式禁用
     * @note 原子类型不建议浅拷贝，禁止拷贝语义
     */
    LatencyHistogram& operator=(const LatencyHistogram&) = delete;

    /**
     * @brief 记录单个时延采样值
     * @param latency_ns 时延数值，单位：纳秒
     * @note 多线程安全，无锁设计，热路径高效写入
     */
    void record(std::uint64_t latency_ns) noexcept;

    /**
     * @brief 统计结果结构体，汇总各项时延指标
     */
    struct Stats {
        std::uint64_t min_ns;      // 最小时延（纳秒）
        std::uint64_t max_ns;      // 最大时延（纳秒）
        std::uint64_t mean_ns;     // 平均时延（纳秒）
        std::uint64_t p50_ns;      // P50 分位数（中位数，纳秒）
        std::uint64_t p95_ns;      // P95 分位数（纳秒）
        std::uint64_t p99_ns;      // P99 分位数（纳秒）
        std::uint64_t p999_ns;     // P999 分位数（纳秒）
        double stddev_ns;          // 标准差（纳秒）
        std::uint64_t count;       // 总采样次数
        std::size_t sample_count;  // 当前窗口内有效样本数量
    };

    /**
     * @brief 根据已采集样本计算完整统计指标
     * @return 包含各类时延指标的 Stats 结构体
     * @note [[nodiscard]] 强制接收返回结果；只读操作
     */
    [[nodiscard]] Stats compute() const;

    /**
     * @brief 格式化打印所有统计指标到输出流
     * @note 便于日志输出、现场调试查看时延数据
     */
    void print_stats() const;

private:
    /**
     * @brief 采样槽位结构体，单个采样存储单元
     */
    struct SampleSlot {
        // 原子类型存储时延值，保证多线程读写安全，初始值为 0
        std::atomic<std::uint64_t> value{0};
    };

    /**
     * @brief 内部辅助函数，从另一个直方图拷贝数据
     * @param other 数据源对象
     * @note 供移动语义内部复用逻辑
     */
    void copy_from(const LatencyHistogram& other) noexcept;

    /**
     * @brief 更新全局最小时延
     * @param latency_ns 新的采样时延值
     */
    void update_min(std::uint64_t latency_ns) noexcept;

    /**
     * @brief 更新全局最大时延
     * @param latency_ns 新的采样时延值
     */
    void update_max(std::uint64_t latency_ns) noexcept;

    // 固定大小采样数组，循环覆盖写入，构成滑动窗口
    std::array<SampleSlot, SAMPLE_WINDOW> samples_{};
    // 原子变量：当前写入位置索引，循环递增
    std::atomic<std::uint64_t> write_index_{0};
    // 原子变量：累计总采样次数
    std::atomic<std::uint64_t> count_{0};
    // 原子变量：全局最小时延，初始化为 uint64 类型最大值
    std::atomic<std::uint64_t> min_ns_{std::numeric_limits<std::uint64_t>::max()};
    // 原子变量：全局最大时延，初始化为 0
    std::atomic<std::uint64_t> max_ns_{0};
};

} // namespace talos::primitive