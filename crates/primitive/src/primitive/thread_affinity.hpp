#pragma once

#include <cstdint>
#include <expected>
#include <string>
#include <thread>

namespace talos::primitive {

using CoreIdResult   = std::expected<std::uint32_t, std::string>;
using AffinityResult = std::expected<void, std::string>;

/**
 * @brief 线程亲和性工具 - CPU绑定
 *
 * 性能优化原理:
 * - 减少缓存失效 (线程固定在同一核心)
 * - 减少上下文切换开销
 * - 更好的L1/L2缓存利用率
 *
 * 平台支持:
 * - Linux: pthread_setaffinity_np (精确绑核)
 * - macOS: thread_policy_set (QoS提示，非严格绑定)
 */
class ThreadAffinity {
public:
    ThreadAffinity() = delete; // 纯静态工具类，禁止实例化

    /**
     * @brief 将当前线程绑定到指定CPU核心
     *
     * @param core_id CPU核心ID (0-based)
     * @return AffinityResult 成功返回void，失败返回具体错误
     */
    [[nodiscard]] static auto pin_to_core(std::uint32_t core_id) noexcept -> AffinityResult;

    [[nodiscard]] static auto
        pin_thread_to_core(std::thread& thread, std::uint32_t core_id) noexcept -> AffinityResult;

    /**
     * @brief 实时优先级配置
     */
    struct RealtimeConfig {
        std::uint8_t priority        = 50;     // Linux: 1-99, 99最高
        std::uint32_t computation_ns = 50000;  // macOS: 计算时间（纳秒）
        std::uint32_t constraint_ns  = 100000; // macOS: 约束时间（纳秒）
        bool preemptible             = false;  // macOS: 是否可抢占
    };

    /**
     * @brief 设置线程为实时优先级 (需要root权限)
     *
     * @param config 实时优先级配置
     * @return AffinityResult 成功返回void，失败返回具体错误
     */
    [[nodiscard]] static auto set_realtime_priority(
        const RealtimeConfig config = RealtimeConfig{50, 50000, 100000, false}) noexcept
        -> AffinityResult;

    /**
     * @brief 获取可用CPU核心数
     */
    [[nodiscard]] static auto get_num_cores() noexcept -> std::uint32_t;

    /**
     * @brief 获取当前线程运行的CPU核心ID
     *
     * @return CoreIdResult 成功返回核心ID，失败返回错误
     */
    [[nodiscard]] static auto get_current_core() noexcept -> CoreIdResult;
};

} // namespace talos::primitive
