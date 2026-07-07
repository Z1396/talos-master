#pragma once
// 头文件保护宏，防止重复包含导致宏重定义编译报错

/**
 * @file
 * @brief 低延迟忙等待自旋提示指令封装
 *
 * 本模块提供CPU架构专属自旋提示指令，用于原子变量忙等循环降低延迟、减少功耗。
 *
 * ## 使用方法
 * 在等待原子状态的紧循环内部调用 SPIN_HINT()：
 *
 * ```cpp
 * while (!ready.load(std::memory_order_acquire)) {
 *     SPIN_HINT();
 * }
 * ```
 *
 * ## 架构适配说明
 *
 * - **x86/x64 平台**：使用 _mm_pause() 内置函数，对应CPU PAUSE 指令
 * - **ARM64 平台**：内联汇编 yield 指令
 * - **其余未知架构**：空操作无任何指令
 *
 * ## 收益作用
 *
 * 1. 忙等待期间降低CPU功耗，避免满负载发热
 * 2. 阻止CPU激进乱序执行、流水线过度占用，减少不必要上下文切换
 * 3. 提升缓存一致性协议效率，减少核间缓存风暴、降低总线争抢延迟
 */

// 低延迟自旋提示宏，作用：避免频繁上下文切换、降低CPU功耗
// ===================== x86 / x86_64 平台分支 =====================
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
// 引入Intel SSE内置函数头文件，提供 _mm_pause
# include <immintrin.h>
// 定义自旋提示宏，调用内置PAUSE指令封装函数
# define SPIN_HINT() _mm_pause()
// ===================== ARM / AArch64 平台分支 =====================
#elif defined(__aarch64__) || defined(__arm__)
// 内联汇编：执行ARM yield 自旋提示指令，memory 内存屏障阻止编译器优化跨循环重排
# define SPIN_HINT() __asm__ volatile("yield" ::: "memory")
// ===================== 其余未知架构兜底分支 =====================
#else
// 空操作，强制void转换消除未使用表达式警告
# define SPIN_HINT() ((void)0)
#endif