// 定时器工厂头文件
#include "scheduler/rclcompat/timer_factory.hpp"

// 定时器频率枚举常量定义
#include "scheduler/rclcompat/timer_constants.hpp"
// RclTimerSystem 定时器系统模板定义
#include "timer_system.hpp"
// std::move 完美移动语义
#include <utility>

namespace talos::scheduler::rclcompat {

/**
 * @brief 根据指定频率创建固定周期定时器系统
 * @param name 定时器系统唯一名称，右值移动传入，避免拷贝
 * @param frequency 定时周期频率枚举（Hz_1 ~ Hz_1000）
 * @param callback 每周期触发的用户回调函数，移动捕获
 * @return std::unique_ptr<system::SystemBase> 多态定时器系统基类智能指针
 *
 * 逻辑：
 * 1. switch匹配频率枚举值，实例化对应模板参数的 RclTimerSystem<N>
 * 2. N为固定频率，编译期常量，调度器可基于编译期常量做周期调度优化
 * 3. 全部参数使用std::move移动，无字符串/函数对象拷贝开销
 * 4. 枚举无匹配分支执行std::unreachable()，编译器标记该分支不可达，消除未返回警告
 */
std::unique_ptr<system::SystemBase> create_timer_system(
    std::string&& name, const Frequency frequency, std::function<void()> callback) {
    switch (frequency) {
    // 1Hz 定时器模板实例
    case Frequency::Hz_1: return std::make_unique<RclTimerSystem<1>>(name, std::move(callback));
    // 2Hz
    case Frequency::Hz_2: return std::make_unique<RclTimerSystem<2>>(name, std::move(callback));
    // 5Hz
    case Frequency::Hz_5: return std::make_unique<RclTimerSystem<5>>(name, std::move(callback));
    // 10Hz
    case Frequency::Hz_10: return std::make_unique<RclTimerSystem<10>>(name, std::move(callback));
    // 20Hz
    case Frequency::Hz_20: return std::make_unique<RclTimerSystem<20>>(name, std::move(callback));
    // 27Hz
    case Frequency::Hz_27: return std::make_unique<RclTimerSystem<27>>(name, std::move(callback));
    // 30Hz
    case Frequency::Hz_30: return std::make_unique<RclTimerSystem<30>>(name, std::move(callback));
    // 50Hz
    case Frequency::Hz_50: return std::make_unique<RclTimerSystem<50>>(name, std::move(callback));
    // 60Hz
    case Frequency::Hz_60: return std::make_unique<RclTimerSystem<60>>(name, std::move(callback));
    // 100Hz
    case Frequency::Hz_100: return std::make_unique<RclTimerSystem<100>>(name, std::move(callback));
    // 120Hz
    case Frequency::Hz_120: return std::make_unique<RclTimerSystem<120>>(name, std::move(callback));
    // 150Hz
    case Frequency::Hz_150: return std::make_unique<RclTimerSystem<150>>(name, std::move(callback));
    // 200Hz
    case Frequency::Hz_200: return std::make_unique<RclTimerSystem<200>>(name, std::move(callback));
    // 250Hz
    case Frequency::Hz_250: return std::make_unique<RclTimerSystem<250>>(name, std::move(callback));
    // 500Hz
    case Frequency::Hz_500: return std::make_unique<RclTimerSystem<500>>(name, std::move(callback));
    // 1000Hz
    case Frequency::Hz_1000:
        return std::make_unique<RclTimerSystem<1000>>(name, std::move(callback));
    }
    // 所有枚举值均已覆盖，此处代码永远不会执行，消除编译器缺少return告警
    std::unreachable();
}

} // namespace talos::scheduler::rclcompat