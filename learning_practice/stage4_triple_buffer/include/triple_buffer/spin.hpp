#pragma once
// 头文件保护宏，防止重复包含导致宏重定义编译报错

// ===========================================================================
// spin.hpp - 低延迟忙等待自旋提示指令封装
// 与真实版 crates/primitive/src/primitive/spin.hpp 保持一致
//
// 作用：在等待原子状态的紧循环内部调用 SPIN_HINT()：
//   1. 忙等待期间降低 CPU 功耗，避免满负载发热
//   2. 阻止 CPU 激进乱序执行，减少流水线过度占用
//   3. 提升缓存一致性协议效率，降低核间缓存风暴
// ===========================================================================

// ===================== x86 / x86_64 平台分支 =====================
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
// Intel SSE 内置函数头文件，提供 _mm_pause
# include <immintrin.h>
// 调用内置 PAUSE 指令封装函数
# define SPIN_HINT() _mm_pause()
// ===================== ARM / AArch64 平台分支 =====================
#elif defined(__aarch64__) || defined(__arm__)
// 内联汇编：ARM yield 自旋提示指令，memory 屏障阻止编译器跨循环重排
# define SPIN_HINT() __asm__ volatile("yield" ::: "memory")
// ===================== 其余未知架构兜底分支 =====================
#else
// 空操作，强制 void 转换消除未使用表达式警告
# define SPIN_HINT() ((void)0)
#endif
