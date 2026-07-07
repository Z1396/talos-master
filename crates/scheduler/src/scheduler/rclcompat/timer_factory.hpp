#pragma once
// 头文件保护宏，防止重复包含造成重定义

// 系统任务基类 SystemBase 定义
#include "../system/system.hpp"
// 定时器频率枚举 Frequency 常量定义
#include "timer_constants.hpp"
// 标准回调函数包装器 std::function
#include <functional>
// 独占智能指针 std::unique_ptr
#include <memory>
// 系统名称字符串 std::string
#include <string>

namespace talos::scheduler::rclcompat {

/**
 * @brief 创建周期定时器系统（编译期频率分发工厂函数声明）
 *
 * 根据传入的 Frequency 枚举值，分发实例化对应模板特化 RclTimerSystem<N>，
 * 不同频率对应独立编译期模板实例，便于调度器静态时序优化。
 *
 * ## 支持频率分类
 *
 * **低频监控类:** 1, 2, 5 Hz
 *
 * **常规控制周期:** 10, 20, 27 Hz
 *
 * **通用相机/传感器标准帧率:** 30, 50, 60, 100 Hz
 *
 * **高频传感器/高速控制:** 120, 150, 200, 250, 500, 1000 Hz
 *
 * ## 入参说明
 *
 * - `name`: 定时器系统唯一标识名称，右值引用，支持移动语义零拷贝
 * - `frequency`: 定时器周期频率枚举，限定预定义固定Hz档位
 * - `callback`: 每周期触发执行的回调函数对象
 *
 * ## 返回值
 *
 * 类型擦除为 SystemBase 的独占智能指针，可统一存入Node延迟系统缓存，
 * 调度器统一管理所有类型系统多态运行。
 */
[[nodiscard]] std::unique_ptr<system::SystemBase>
    create_timer_system(std::string&& name, Frequency frequency, std::function<void()> callback);

} // namespace talos::scheduler::rclcompat