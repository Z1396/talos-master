#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace talos::primitive {
/**
 * @brief High-performance latency probe
 *
 * Uses std::chrono::steady_clock which is faster than high_resolution_clock
 * on most platforms (no time zone adjustment, monotonic).
 *
 * Design:
 * - Records start timestamp on construction
 * - Supports elapsed time queries
 * - Lock-free: no allocations, no system calls in critical path
 * - Portable: works on x86-64, ARM64, etc.
 *
 * Overhead: ~20-40 nanoseconds per call (optimized)
 */
class LatencyProbe {
public:
    LatencyProbe() noexcept;

    ~LatencyProbe() = default;

    /**
     * @brief Get elapsed nanoseconds since construction
     */
    [[nodiscard]] std::uint64_t elapsed_ns() const noexcept;

    /**
     * @brief Get elapsed nanoseconds and reset
     */
    std::uint64_t snapshot_and_reset() noexcept;

private:
    std::uint64_t start_ns_;

    /**
     * @brief Get current time in nanoseconds
     * Uses steady_clock which is faster than high_resolution_clock
     */
    static std::uint64_t now_ns() noexcept;
};

/**
 * @brief Low-overhead latency tracker for online scheduler statistics
 *
 * The hot path keeps an exact total count plus a bounded recent sample window.
 * This avoids mutex contention and unbounded per-system storage while still
 * providing useful percentile snapshots for long-running workloads.
 */
class LatencyHistogram {
public:
    static constexpr std::size_t SAMPLE_WINDOW = 8192;

    LatencyHistogram() noexcept = default;

    LatencyHistogram(LatencyHistogram&& other) noexcept;

    LatencyHistogram& operator=(LatencyHistogram&& other) noexcept;

    LatencyHistogram(const LatencyHistogram&)            = delete;
    LatencyHistogram& operator=(const LatencyHistogram&) = delete;

    /**
     * @brief Record a latency sample (nanoseconds)
     */
    void record(std::uint64_t latency_ns) noexcept;

    /**
     * @brief Compute statistics from recorded samples
     */
    struct Stats {
        std::uint64_t min_ns;
        std::uint64_t max_ns;
        std::uint64_t mean_ns;
        std::uint64_t p50_ns; // Median
        std::uint64_t p95_ns;
        std::uint64_t p99_ns;
        std::uint64_t p999_ns;
        double stddev_ns;
        std::uint64_t count;
        std::size_t sample_count;
    };

    [[nodiscard]] Stats compute() const;

    /**
     * @brief Pretty-print statistics
     */
    void print_stats() const;

private:
    struct SampleSlot {
        std::atomic<std::uint64_t> value{0};
    };

    void copy_from(const LatencyHistogram& other) noexcept;

    void update_min(std::uint64_t latency_ns) noexcept;

    void update_max(std::uint64_t latency_ns) noexcept;

    std::array<SampleSlot, SAMPLE_WINDOW> samples_{};
    std::atomic<std::uint64_t> write_index_{0};
    std::atomic<std::uint64_t> count_{0};
    std::atomic<std::uint64_t> min_ns_{std::numeric_limits<std::uint64_t>::max()};
    std::atomic<std::uint64_t> max_ns_{0};
};
} // namespace talos::primitive
