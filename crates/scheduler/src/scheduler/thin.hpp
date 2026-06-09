#pragma once

#include <cstddef>

namespace talos {
namespace scheduler {
class Scheduler;
class World;

struct SchedulerConfig {
    /**
     * # TBB arena concurrency
     *
     * Number of threads in the compute pool. 0 = hardware concurrency.
     *
     * ## Default
     *
     * 0 (use all available cores)
     */
    std::size_t compute_concurrency = 0;
    bool print_stats{true};
};

} // namespace scheduler
using namespace talos::scheduler;
} // namespace talos
