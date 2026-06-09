#include "primitive/performance_probe.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace talos::primitive {

LatencyProbe::LatencyProbe() noexcept
    : start_ns_(now_ns()) {}

auto LatencyProbe::elapsed_ns() const noexcept -> std::uint64_t { return now_ns() - start_ns_; }

auto LatencyProbe::snapshot_and_reset() noexcept -> std::uint64_t {
    const auto now     = now_ns();
    const auto elapsed = now - start_ns_;
    start_ns_          = now;
    return elapsed;
}

auto LatencyProbe::now_ns() noexcept -> std::uint64_t {
    return std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now())
        .time_since_epoch()
        .count();
}

LatencyHistogram::LatencyHistogram(LatencyHistogram&& other) noexcept { copy_from(other); }

auto LatencyHistogram::operator=(LatencyHistogram&& other) noexcept -> LatencyHistogram& {
    if (this != &other) {
        copy_from(other);
    }
    return *this;
}

void LatencyHistogram::record(const std::uint64_t latency_ns) noexcept {
    const auto index = write_index_.load(std::memory_order_relaxed);
    samples_[index % SAMPLE_WINDOW].value.store(latency_ns, std::memory_order_relaxed);
    update_min(latency_ns);
    update_max(latency_ns);
    write_index_.store(index + 1, std::memory_order_relaxed);
    count_.store(index + 1, std::memory_order_release);
}

auto LatencyHistogram::compute() const -> Stats {
    const auto total_count = count_.load(std::memory_order_acquire);
    if (total_count == 0) {
        return {0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0};
    }

    const auto sample_count =
        static_cast<std::size_t>(std::min<std::uint64_t>(total_count, SAMPLE_WINDOW));

    std::vector<std::uint64_t> snapshot;
    snapshot.reserve(sample_count);
    for (std::size_t i = 0; i < sample_count; ++i) {
        snapshot.push_back(samples_[i].value.load(std::memory_order_relaxed));
    }

    std::ranges::sort(snapshot);

    auto min_val = min_ns_.load(std::memory_order_relaxed);
    if (min_val == std::numeric_limits<std::uint64_t>::max()) {
        min_val = 0;
    }
    const auto max_val  = max_ns_.load(std::memory_order_relaxed);
    const auto sum      = std::accumulate(snapshot.begin(), snapshot.end(), 0ULL);
    const auto mean_val = sum / sample_count;

    const auto p50_idx  = std::min(sample_count * 50 / 100, sample_count - 1);
    const auto p95_idx  = std::min(sample_count * 95 / 100, sample_count - 1);
    const auto p99_idx  = std::min(sample_count * 99 / 100, sample_count - 1);
    const auto p999_idx = std::min(sample_count * 999 / 1000, sample_count - 1);

    double sum_sq       = 0.0;
    const double mean_d = static_cast<double>(mean_val);
    for (std::size_t i = 0; i < sample_count; i++) {
        const double diff = static_cast<double>(snapshot[i]) - mean_d;
        sum_sq += diff * diff;
    }
    const double stddev = std::sqrt(sum_sq / static_cast<double>(sample_count));

    return {min_val,           max_val,           mean_val,           snapshot[p50_idx],
            snapshot[p95_idx], snapshot[p99_idx], snapshot[p999_idx], stddev,
            total_count,       sample_count};
}

void LatencyHistogram::print_stats() const {
    const auto stats = compute();
    std::cout << "Latency Statistics (ns):\n"
              << "  Count:  " << stats.count << "\n"
              << "  Min:    " << stats.min_ns << " ns\n"
              << "  P50:    " << stats.p50_ns << " ns\n"
              << "  P95:    " << stats.p95_ns << " ns\n"
              << "  P99:    " << stats.p99_ns << " ns\n"
              << "  P99.9:  " << stats.p999_ns << " ns\n"
              << "  Mean:   " << stats.mean_ns << " ns\n"
              << "  Max:    " << stats.max_ns << " ns\n"
              << "  StdDev: " << stats.stddev_ns << " ns\n";
}

void LatencyHistogram::copy_from(const LatencyHistogram& other) noexcept {
    write_index_.store(
        other.write_index_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    count_.store(other.count_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    min_ns_.store(other.min_ns_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    max_ns_.store(other.max_ns_.load(std::memory_order_relaxed), std::memory_order_relaxed);

    for (std::size_t i = 0; i < SAMPLE_WINDOW; ++i) {
        samples_[i].value.store(
            other.samples_[i].value.load(std::memory_order_relaxed), std::memory_order_relaxed);
    }
}

void LatencyHistogram::update_min(const std::uint64_t latency_ns) noexcept {
    auto current = min_ns_.load(std::memory_order_relaxed);
    while (latency_ns < current
           && !min_ns_.compare_exchange_weak(
               current, latency_ns, std::memory_order_relaxed, std::memory_order_relaxed)) {}
}

void LatencyHistogram::update_max(const std::uint64_t latency_ns) noexcept {
    auto current = max_ns_.load(std::memory_order_relaxed);
    while (latency_ns > current
           && !max_ns_.compare_exchange_weak(
               current, latency_ns, std::memory_order_relaxed, std::memory_order_relaxed)) {}
}

} // namespace talos::primitive
