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

namespace talos::scheduler::system {

namespace detail {

struct NotifyingTopic {};
struct SilentTopic {};

/**
 * @brief Base template for fixed_rate execution policies
 *
 * @tparam FreqHz Execution frequency in Hz, 0 = run as fast as possible
 * @tparam Affinity CPU core affinity, -1 = no binding
 * @tparam Priority Thread priority, 0 = default
 * @tparam Topic Notification behavior topic
 */
template <
    std::uint32_t FreqHz = 0, std::int32_t Affinity = -1, std::int32_t Priority = 0,
    typename Topic = NotifyingTopic>
struct fixed_rate_base {
    static constexpr std::uint32_t frequency_hz   = FreqHz;
    static constexpr std::int32_t cpu_affinity    = Affinity;
    static constexpr std::int32_t thread_priority = Priority;
    static constexpr bool notifies                = std::is_same_v<Topic, NotifyingTopic>;
};

} // namespace detail

/**
 * @brief FixedRate with notify: dedicated thread, notifies scheduler after writes
 *
 * Use for: data sources that should trigger compute when they actually publish new data
 *
 * ## Template parameters
 *
 * - `FreqHz`: Execution frequency in Hz, 0 = run as fast as possible
 * - `Affinity`: CPU core affinity, -1 = no binding
 * - `Priority`: Thread priority, 0 = default
 *
 * ## Example
 *
 * ```cpp
 * // 30Hz camera, pinned to core 0, normal priority
 * add_system<fixed_rate<30, 0, 0>>("camera", [](auto in, auto out) {
 *     // ...
 * });
 * ```
 */
template <std::uint32_t FreqHz = 0, std::int32_t Affinity = -1, std::int32_t Priority = 0>
using fixed_rate = detail::fixed_rate_base<FreqHz, Affinity, Priority, detail::NotifyingTopic>;

/**
 * @brief FixedRate silent: dedicated thread, does NOT notify scheduler
 *
 * Use for: high-frequency data sources (e.g., IMU) that update silently
 *
 * ## Template parameters
 *
 * - `FreqHz`: Execution frequency in Hz, 0 = run as fast as possible
 * - `Affinity`: CPU core affinity, -1 = no binding
 * - `Priority`: Thread priority, 0 = default
 *
 * ## Example
 *
 * ```cpp
 * // 500Hz IMU, pinned to core 1
 * add_system<fixed_rate_silent<500, 1, 0>>("imu", [](auto in, auto out) {
 *     // ...
 * });
 * ```
 */
template <std::uint32_t FreqHz = 0, std::int32_t Affinity = -1, std::int32_t Priority = 0>
using fixed_rate_silent = detail::fixed_rate_base<FreqHz, Affinity, Priority, detail::SilentTopic>;

/**
 * @brief Pool compute: TBB thread pool, triggered by the scheduler ready set
 *
 * Default policy. Systems run on the TBB thread pool and are selectively
 * woken when an external source or upstream compute system produces data.
 */
struct pool_compute {};

/**
 * @brief Pool compute: TBB thread pool, triggered by fixed_rate notify
 *
 * For visualization.
 * TODO
 */
struct pool_visualization {};

/// Default policy
using default_policy = pool_compute;

// ============================================================================
// Policy Traits
// ============================================================================

namespace detail {

/// Unified fixed_rate policy checker
template <typename T>
struct is_fixed_rate_policy_base : std::false_type {};

// Both fixed_rate and fixed_rate_silent are fixed_rate policies (unified check)
template <std::uint32_t F, std::int32_t A, std::int32_t P, typename Topic>
struct is_fixed_rate_policy_base<fixed_rate_base<F, A, P, Topic>> : std::true_type {};

/// Check if policy is fixed_rate (both notifying and silent)
template <typename T>
struct is_fixed_rate_policy : is_fixed_rate_policy_base<T> {};

/// Check if policy is pool_compute
template <typename T>
struct is_pool_policy : std::is_same<T, pool_compute> {};

/// Check if policy is pool_visualization
template <typename T>
struct is_visualization_policy : std::is_same<T, pool_visualization> {};

/// Check if policy notifies (only fixed_rate, not fixed_rate_silent)
template <typename T>
struct is_notifying_policy : std::false_type {};

template <std::uint32_t F, std::int32_t A, std::int32_t P>
struct is_notifying_policy<fixed_rate<F, A, P>> : std::true_type {};

/// Check if policy is silent (only fixed_rate_silent)
template <typename T>
struct is_silent_policy : std::false_type {};

template <std::uint32_t F, std::int32_t A, std::int32_t P>
struct is_silent_policy<fixed_rate_silent<F, A, P>> : std::true_type {};

} // namespace detail

/**
 * @brief Check if a policy is fixed_rate
 *
 * ## Template parameters
 *
 * - `T`: policy type to check
 */
template <typename T>
inline constexpr bool is_fixed_rate_policy_v = detail::is_fixed_rate_policy<T>::value;

/**
 * @brief Check if a policy is pool compute
 *
 * ## Template parameters
 *
 * - `T`: policy type to check
 */
template <typename T>
inline constexpr bool is_pool_policy_v = detail::is_pool_policy<T>::value;

template <typename T>
inline constexpr bool is_visualization_policy_v = detail::is_visualization_policy<T>::value;

/**
 * @brief Check if a policy notifies after execution
 *
 * ## Template parameters
 *
 * - `T`: policy type to check
 */
template <typename T>
inline constexpr bool is_notifying_policy_v = detail::is_notifying_policy<T>::value;

/**
 * @brief Check if a policy is silent (no notification)
 *
 * ## Template parameters
 *
 * - `T`: policy type to check
 */
template <typename T>
inline constexpr bool is_silent_policy_v = detail::is_silent_policy<T>::value;

// ============================================================================
// Runtime Policy Info
// ============================================================================

/**
 * @brief Runtime representation of a policy's kind
 */
enum class PolicyKind : std::uint8_t {
    FixedRate,         ///< FixedRate system (dedicated thread)
    PoolCompute,       ///< Pool compute system (TBB thread pool)
    PoolVisualization, ///< Pool compute system (TBB thread pool)
};

/**
 * @brief Runtime policy information extracted from compile-time policy
 */
struct PolicyInfo {
    PolicyKind kind              = PolicyKind::PoolCompute;
    std::uint32_t frequency_hz   = 0;
    std::int32_t cpu_affinity    = -1;
    std::int32_t thread_priority = 0;
    bool notifies                = false;

    /**
     * @brief Unified policy kind checker
     *
     * ## Parameters
     *
     * - `kind`: the PolicyKind to check against
     *
     * ## Returns
     *
     * `true` if this policy matches the given kind
     */
    [[nodiscard]] constexpr bool is_kind(PolicyKind kind) const noexcept {
        return this->kind == kind;
    }

    /**
     * @brief Check if this is an fixed_rate policy
     *
     * ## Returns
     *
     * `true` if fixed_rate, `false` if pool compute
     */
    [[nodiscard]] constexpr bool is_fixed_rate() const noexcept {
        return is_kind(PolicyKind::FixedRate);
    }

    /**
     * @brief Check if this is a pool compute policy
     *
     * ## Returns
     *
     * `true` if pool compute, `false` if fixed_rate
     */
    [[nodiscard]] constexpr bool is_pool() const noexcept {
        return is_kind(PolicyKind::PoolCompute);
    }

    [[nodiscard]] constexpr bool is_visualization() const noexcept {
        return is_kind(PolicyKind::PoolVisualization);
    }
};

/**
 * @brief Convert compile-time policy to runtime PolicyInfo
 *
 * ## Template parameters
 *
 * - `Policy`: compile-time policy type
 *
 * ## Returns
 *
 * PolicyInfo with extracted configuration
 */
template <typename Policy>
[[nodiscard]] constexpr auto make_policy_info() noexcept -> PolicyInfo {
    if constexpr (is_fixed_rate_policy_v<Policy>) {
        return PolicyInfo{
            .kind            = PolicyKind::FixedRate,
            .frequency_hz    = Policy::frequency_hz,
            .cpu_affinity    = Policy::cpu_affinity,
            .thread_priority = Policy::thread_priority,
            .notifies        = Policy::notifies,
        };
    }
    if constexpr (is_pool_policy_v<Policy>) {
        return PolicyInfo{.kind = PolicyKind::PoolCompute};
    }
    if constexpr (is_visualization_policy_v<Policy>) {
        return PolicyInfo{.kind = PolicyKind::PoolVisualization};
    }
    std::unreachable();
}

} // namespace talos::scheduler::system
