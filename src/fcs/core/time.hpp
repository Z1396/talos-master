// 头文件保护宏：保证该头文件只会被编译一次，替代传统 #ifndef / #define / #endif 头卫兵
#pragma once

// 引入C++标准时间库，提供时钟、时间点、时长相关类与转换逻辑
#include <chrono>
// 引入固定宽度整数头文件，提供 uint64_t 无符号64位整型
#include <cstdint>

// 自定义命名空间 fcs::clock，隔离时钟相关工具函数，避免全局命名污染
namespace fcs::clock {

/**
 * @brief 获取当前系统Unix时间戳，单位：纳秒(ns)
 * @return uint64_t 从1970-01-01 00:00:00 UTC纪元至今的总纳秒数
 * @note 属性说明：
 *       1. [[nodiscard]]：强制要求调用方接收返回值，忽略返回值会触发编译警告，防止误用
 *       2. static：静态函数，仅属于当前翻译单元，不会产生多重定义链接错误
 *       3. noexcept：函数保证不会抛出任何异常，编译器可做异常优化
 */
[[nodiscard]] static uint64_t now_ns() noexcept {
    // 1. 获取系统实时时钟的当前时间点（wall-clock 墙上时钟，对应现实世界UTC时间）
    // std::chrono::system_clock：系统时钟，会受NTP校时、手动改时间影响，适合时间戳、日志时间
    const auto now = std::chrono::system_clock::now();

    // 2. time_since_epoch()：获取该时间点距离Unix纪元(1970-01-01 UTC)的时长对象
    // 3. duration_cast<>：时长强制类型转换，把默认精度时长转为【纳秒】单位
    // 4. .count()：取出时长内部存储的原始整数数值
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

} // namespace fcs::clock