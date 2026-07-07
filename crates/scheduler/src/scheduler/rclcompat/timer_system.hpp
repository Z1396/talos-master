#pragma once
// 头文件保护宏，防止重复包含重定义

// 调度执行策略 fixed_rate 定义
#include "../system/execution_policy.hpp"
// RclSubSystemBase 订阅系统基类、CallbackContextScope RAII守卫
#include "system.hpp"

// 固定宽度整数
#include <cstdint>
// 定时器回调函数包装器
#include <functional>
// 系统名称字符串
#include <string>
// std::move 移动语义
#include <utility>

namespace talos::scheduler::rclcompat {

/**
 * @brief 固定频率定时器系统 RclTimerSystem
 *
 * 运行在独立 fixed_rate 专用周期线程，按模板参数指定频率稳定执行回调；
 * 继承 RclSubSystemBase，复用线程本地上下文自动绑定 Publisher 所有权校验机制。
 *
 * ## 模板参数
 *
 * - `FrequencyHz`: 定时器执行频率，单位Hz，编译期常量
 * - `Affinity`: CPU亲和核心编号，-1=不绑定固定CPU核心
 * - `Priority`: 线程静态优先级，0=系统默认基础优先级
 */
template <std::uint32_t FrequencyHz, std::int32_t Affinity = -1, std::int32_t Priority = 0>
class RclTimerSystem : public RclSubSystemBase {
public:
    // 回调函数类型：无入参无返回值周期回调
    using Callback = std::function<void()>;
    // 绑定当前定时器参数的固定频率调度策略类型
    using Policy   = fixed_rate<FrequencyHz, Affinity, Priority>;

    /**
     * @brief 构造定时器系统
     * @param name 定时器系统唯一名称，移动语义
     * @param callback 每周期触发的用户回调，移动捕获闭包
     */
    explicit RclTimerSystem(std::string name, Callback callback) noexcept
        : callback_(std::move(callback)) {
        // 调用基类静态工具构建系统元数据
        meta_ = build_meta<Policy>(std::move(name), [](auto&) {
            // 定时器无SPMC/SPSC读写通道，无需添加任何ChannelMeta
        });
    }

    /**
     * @brief 调度器初始化绑定World资源接口重写
     * 定时器不使用消息通道，无需绑定任何SPMC/SPSC读写器，空实现
     * @param world 全局资源容器World，[[maybe_unused]]消除未使用参数告警
     */
    void bind([[maybe_unused]] World& world) noexcept override {
        // Timer typically doesn't need to bind channels
    }

    /**
     * @brief 调度周期执行入口函数
     * 每固定周期自动触发，执行逻辑：
     * 1. RAII 切换线程本地回调上下文，标记当前系统为本定时器
     * 2. 执行业务回调，回调内部publish会校验上下文匹配
     * 3. 返回false：单次执行，无持续待处理任务
     */
    bool run([[maybe_unused]] World& world) noexcept override {
        // 设置线程本地回调上下文，开启Publisher自动所有权绑定校验
        CallbackContextScope scope(this);
        callback_();

        return false;
    }

private:
    // 用户周期回调存储
    Callback callback_;
};

} // namespace talos::scheduler::rclcompat