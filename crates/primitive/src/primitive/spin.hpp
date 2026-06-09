#pragma once

/**
 * @file
 * @brief Low-latency spin hints for busy-waiting
 *
 * This module provides CPU-specific spin hints to reduce latency
 * when waiting on atomic variables.
 *
 * ## Usage
 *
 * Use `SPIN_HINT()` in tight loops waiting for atomic operations:
 *
 * ```cpp
 * while (!ready.load(std::memory_order_acquire)) {
 *     SPIN_HINT();
 * }
 * ```
 *
 * ## Architecture support
 *
 * - **x86/x64**: Uses `_mm_pause()` (PAUSE instruction)
 * - **ARM64**: Uses `yield` instruction
 * - **Other**: No-op
 *
 * ## Benefits
 *
 * - Reduces power consumption during spin-wait
 * - Avoids unnecessary context switches
 * - Improves cache coherency protocol efficiency
 */

// Low-latency spin hint (avoid context switch, reduce power)
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
# include <immintrin.h>
# define SPIN_HINT() _mm_pause()
#elif defined(__aarch64__) || defined(__arm__)
# define SPIN_HINT() __asm__ volatile("yield" ::: "memory")
#else
# define SPIN_HINT() ((void)0)
#endif
