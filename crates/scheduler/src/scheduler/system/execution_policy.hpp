#pragma once

/**
 * @file
 * @brief Execution policies for systems
 *
 * This module defines policies that specify how systems are executed:
 * - **FixedRate**: Runs on a dedicated thread (single-producer)
 * - **Pool compute**: Runs on the TBB thread pool (multi-consumer)
 *
 * ## Usage
 *
 * ```cpp
 * // Camera: fixed_rate, runs at 30Hz, pinned to core 0
 * scheduler.add_system<fixed_rate<30, 0, 0>>("camera", [](auto input, auto output) {
 *     // ...
 * });
 *
 * // Tracker: pool compute (default)
 * scheduler.add_system("tracker", [](auto input, auto output) {
 *     // ...
 * });
 * ```
 */

#include <cstdint>
#include <type_traits>
#include <utility>

// Talos框架 调度器模块 系统执行策略子命名空间
namespace talos::scheduler::system {

// 内部实现细节命名空间，对外隐藏
namespace detail {

// 标记类型：执行完成后主动通知调度器
struct NotifyingTopic {};
// 标记类型：执行完成后静默运行，不通知调度器
struct SilentTopic {};

/**
 * @brief 固定频率执行策略基础模板
 * @tparam FreqHz 执行频率(赫兹)，传0表示以最大速度运行
 * @tparam Affinity CPU亲和性，-1表示不绑定核心
 * @tparam Priority 线程优先级，0为默认优先级
 * @tparam Topic 通知行为标记类型，区分主动通知/静默模式
 */
template <
    std::uint32_t FreqHz = 0, std::int32_t Affinity = -1, std::int32_t Priority = 0,
    typename Topic = NotifyingTopic>
struct fixed_rate_base {
    // 运行频率，单位Hz
    static constexpr std::uint32_t frequency_hz   = FreqHz;
    // CPU核心亲和性编号
    static constexpr std::int32_t cpu_affinity    = Affinity;
    // 线程优先级
    static constexpr std::int32_t thread_priority = Priority;
    // 是否主动通知调度器：根据Topic类型自动推导
    static constexpr bool notifies                = std::is_same_v<Topic, NotifyingTopic>;
};

} // namespace detail

/**
 * @brief 主动通知型固定频率策略：独占线程，产出数据后通知调度器
 * 适用场景：数据源产生新数据后，需要立即触发后续计算流程
 *
 * @tparam FreqHz 执行频率(Hz)，0 = 全速运行
 * @tparam Affinity CPU核心绑定，-1 = 不绑定
 * @tparam Priority 线程优先级，0 = 默认优先级
 *
 * ## Example
 *
 * ```cpp
 * // 30Hz相机任务，绑定到0号核心，默认优先级
 * add_system<fixed_rate<30, 0, 0>>("camera", [](auto in, auto out) {
 *     // ...
 * });
 * ```
 */
template <std::uint32_t FreqHz = 0, std::int32_t Affinity = -1, std::int32_t Priority = 0>
using fixed_rate = detail::fixed_rate_base<FreqHz, Affinity, Priority, detail::NotifyingTopic>;

/**
 * @brief 静默型固定频率策略：独占线程，运行后不主动通知调度器
 * 适用场景：高频数据源（如IMU），持续刷新数据，无需每次都触发下游
 *
 * @tparam FreqHz 执行频率(Hz)，0 = 全速运行
 * @tparam Affinity CPU核心绑定，-1 = 不绑定
 * @tparam Priority 线程优先级，0 = 默认优先级
 *
 * ## Example
 *
 * ```cpp
 * // 500Hz IMU任务，绑定到1号核心
 * add_system<fixed_rate_silent<500, 1, 0>>("imu", [](auto in, auto out) {
 *     // ...
 * });
 * ```
 */
template <std::uint32_t FreqHz = 0, std::int32_t Affinity = -1, std::int32_t Priority = 0>
using fixed_rate_silent = detail::fixed_rate_base<FreqHz, Affinity, Priority, detail::SilentTopic>;

/**
 * @brief 线程池计算策略：基于TBB线程池执行
 * 由调度器根据数据就绪状态统一唤醒调度，是最常用的默认策略
 */
struct pool_compute {};

/**
 * @brief 可视化专用线程池策略：基于TBB线程池，由固定频率任务的通知触发执行
 * 预留类型，专门用于可视化、界面渲染类任务（待完善）
 * TODO
 */
struct pool_visualization {};

/// 系统默认执行策略
using default_policy = pool_compute;

// ============================================================================
// 模板特征判断（Traits）：编译期类型检测工具
// ============================================================================

namespace detail {

/// 通用固定频率策略检测基类，默认返回false
template <typename T>
struct is_fixed_rate_policy_base : std::false_type {};

/// 偏特化：匹配所有 fixed_rate_base 派生的策略类型，标记为true
template <std::uint32_t F, std::int32_t A, std::int32_t P, typename Topic>
struct is_fixed_rate_policy_base<fixed_rate_base<F, A, P, Topic>> : std::true_type {};

/// 判断类型是否为固定频率执行策略（包含通知/静默两种子类）
template <typename T>
struct is_fixed_rate_policy : is_fixed_rate_policy_base<T> {};

/// 判断类型是否为普通线程池计算策略
template <typename T>
struct is_pool_policy : std::is_same<T, pool_compute> {};

/// 判断类型是否为可视化线程池策略
template <typename T>
struct is_visualization_policy : std::is_same<T, pool_visualization> {};

/// 判断类型是否为「主动通知」策略，默认false
template <typename T>
struct is_notifying_policy : std::false_type {};

/// 偏特化：匹配 fixed_rate 主动通知策略，标记为true
template <std::uint32_t F, std::int32_t A, std::int32_t P>
struct is_notifying_policy<fixed_rate<F, A, P>> : std::true_type {};

/// 判断类型是否为「静默不通知」策略，默认false
template <typename T>
struct is_silent_policy : std::false_type {};

/// 偏特化：匹配 fixed_rate_silent 静默策略，标记为true
template <std::uint32_t F, std::int32_t A, std::int32_t P>
struct is_silent_policy<fixed_rate_silent<F, A, P>> : std::true_type {};

} // namespace detail

/**
 * @brief 编译期常量：判断指定类型是否为固定频率执行策略
 * @tparam T 待检测的策略类型
 */
template <typename T>
inline constexpr bool is_fixed_rate_policy_v = detail::is_fixed_rate_policy<T>::value;

/**
 * @brief 编译期常量：判断指定类型是否为普通线程池计算策略
 * @tparam T 待检测的策略类型
 */
template <typename T>
inline constexpr bool is_pool_policy_v = detail::is_pool_policy<T>::value;

/**
 * @brief 编译期常量：判断指定类型是否为可视化线程池策略
 * @tparam T 待检测的策略类型
 */
template <typename T>
inline constexpr bool is_visualization_policy_v = detail::is_visualization_policy<T>::value;

/**
 * @brief 编译期常量：判断策略执行后是否会主动通知调度器
 * @tparam T 待检测的策略类型
 */
template <typename T>
inline constexpr bool is_notifying_policy_v = detail::is_notifying_policy<T>::value;

/**
 * @brief 编译期常量：判断策略是否为静默模式（不通知调度器）
 * @tparam T 待检测的策略类型
 */
template <typename T>
inline constexpr bool is_silent_policy_v = detail::is_silent_policy<T>::value;

// ============================================================================
// 运行时策略信息：将编译期模板配置转为运行时可用的数据结构
// ============================================================================

/**
 * @brief 策略类型枚举，运行时区分不同执行策略大类
 */
enum class PolicyKind : std::uint8_t {
    FixedRate,         ///< 独占线程的固定频率任务
    PoolCompute,       ///< 普通TBB线程池任务
    PoolVisualization, ///< 可视化专用TBB线程池任务
};

/**
 * @brief 运行时策略信息结构体
 * 存储从编译期模板中解析出的全部配置参数，供调度器运行时使用
 */
struct PolicyInfo {
    PolicyKind kind              = PolicyKind::PoolCompute; // 策略大类，默认普通线程池
    std::uint32_t frequency_hz   = 0;                        // 运行频率(Hz)
    std::int32_t cpu_affinity    = -1;                       // CPU核心亲和性
    std::int32_t thread_priority = 0;                        // 线程优先级
    bool notifies                = false;                   // 是否主动通知调度器

    /**
     * @brief 判断当前策略是否为指定类型
     * @param kind 待比对的策略枚举值
     * @return 匹配返回true，否则false
     * @note constexpr 编译期/运行时均可调用，noexcept 无异常
     */
    [[nodiscard]] constexpr bool is_kind(PolicyKind kind) const noexcept {
        return this->kind == kind;
    }

    /**
     * @brief 判断是否为固定频率策略
     * @return true = 固定频率独占线程
     */
    [[nodiscard]] constexpr bool is_fixed_rate() const noexcept {
        return is_kind(PolicyKind::FixedRate);
    }

    /**
     * @brief 判断是否为普通线程池策略
     * @return true = 普通TBB线程池任务
     */
    [[nodiscard]] constexpr bool is_pool() const noexcept {
        return is_kind(PolicyKind::PoolCompute);
    }

    /**
     * @brief 判断是否为可视化线程池策略
     * @return true = 可视化专用线程池任务
     */
    [[nodiscard]] constexpr bool is_visualization() const noexcept {
        return is_kind(PolicyKind::PoolVisualization);
    }
};

/**
 * @brief 编译期模板转运行时策略信息的转换函数
 * @tparam Policy 编译期定义的策略模板类型
 * @return 填充完成的 PolicyInfo 运行时结构体
 * @note 基于if constexpr 分支匹配不同策略类型，std::unreachable 标记不可执行分支
 */
template <typename Policy>
[[nodiscard]] constexpr auto make_policy_info() noexcept -> PolicyInfo {
    // 匹配固定频率类策略
    if constexpr (is_fixed_rate_policy_v<Policy>) {
        return PolicyInfo{
            .kind            = PolicyKind::FixedRate,
            .frequency_hz    = Policy::frequency_hz,
            .cpu_affinity    = Policy::cpu_affinity,
            .thread_priority = Policy::thread_priority,
            .notifies        = Policy::notifies,
        };
    }
    // 匹配普通线程池策略
    if constexpr (is_pool_policy_v<Policy>) {
        return PolicyInfo{.kind = PolicyKind::PoolCompute};
    }
    // 匹配可视化线程池策略
    if constexpr (is_visualization_policy_v<Policy>) {
        return PolicyInfo{.kind = PolicyKind::PoolVisualization};
    }
    // 理论上不会执行到此处，标记代码不可达
    std::unreachable();
}

} // namespace talos::scheduler::system