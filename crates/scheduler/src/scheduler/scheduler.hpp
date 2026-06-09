#pragma once

/**
 * @file
 * @brief Scheduler for data-driven task execution
 *
 * This module provides a scheduler that manages fixed_rate (single-threaded)
 * and compute (thread-pool) systems with dependency-aware execution.
 *
 * ## Architecture
 *
 * - **FixedRate systems**: Run on dedicated threads, can notify the scheduler
 * - **Compute systems**: Run on TBB thread pool, triggered by notifications
 * - **Dependency graph**: Built from channel connections, enables selective wakeup
 * - **Resources**: Shared side state only; resources do not contribute scheduling edges
 *
 * ## Execution model
 *
 * 1. FixedRate systems run on their own threads
 * 2. When a notifying fixed_rate system writes output, it triggers dependent compute systems
 * 3. Compute systems drain the ready set to a fixed point in topological order (levels)
 * 4. TBB arena manages thread pool concurrency
 *
 * ## Build errors
 *
 * The scheduler validates the system graph during `build()`:
 * - Single writer per channel (both SPSC and SPMC)
 * - No cycles in dependency graph
 * - Orphaned readers detected
 * - Max 64 compute systems for bitmask scheduling
 */

#include "primitive/performance_probe.hpp"
#include "scheduler/error.hpp"
#include "scheduler/system/execution_policy.hpp"
#include "scheduler/system/system.hpp"
#include "scheduler/thin.hpp"
#include "scheduler/world.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace tbb::detail::d1 {
class task_arena;
} // namespace tbb::detail::d1
namespace tbb {
using task_arena = tbb::detail::d1::task_arena;
} // namespace tbb

namespace talos::scheduler {

// ============================================================================
// System Entry
// ============================================================================

/**
 * @brief Internal storage for a registered system
 */
struct SystemEntry {
    std::unique_ptr<SystemBase> system;
    PolicyInfo policy;
    bool bound = false;
};

using namespace system;

// ============================================================================
// Scheduler
// ============================================================================

/**
 * @brief Main scheduler for data-driven task execution
 *
 * The scheduler manages fixed_rate systems (single-threaded, dedicated threads)
 * and compute systems (thread-pool based) with dependency-aware execution.
 *
 * ## Usage
 *
 * ```cpp
 * World world;
 * Scheduler scheduler(world);
 *
 * scheduler.add_system<fixed_rate<30>>("camera", [](talos::spmc_mut<fcs::ImageFrame,
 * fcs::ImageChannelTopic> cam_out) {
 *     // Process camera input, write to channel
 *     cam_out.write(...);
 * });
 *
 * auto result = scheduler.build();
 * if (!result) {
 *     // Handle build error
 * }
 *
 * scheduler.run();  // Blocks until stop() is called
 * ```
 *
 * ## Thread safety
 *
 * The scheduler is not thread-safe after construction. Configure and build
 * in a single-threaded context before calling `run()`. A successful `build()`
 * also freezes resource structure: existing resource objects may be mutated,
 * but safe-path insertion or replacement after build is rejected because systems
 * and accessors may cache raw `res/res_mut` pointers.
 */
class Scheduler {
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    using Duration  = Clock::duration;

    enum class LifecycleState : std::uint8_t {
        Configuring, ///< Topology/resources may still change; must build before run
        Built,       ///< Graph is valid and ready to run
        Running,     ///< Fixed-rate threads and compute loop are active
    };

    /**
     * @brief Construct a scheduler
     *
     * ## Parameters
     *
     * - `world`: the world containing resources and channels
     * - `config`: optional scheduler configuration
     */
    explicit Scheduler(SchedulerConfig config = {}) noexcept;
    ~Scheduler() noexcept;

    Scheduler(const Scheduler&)            = delete;
    Scheduler& operator=(const Scheduler&) = delete;
    Scheduler(Scheduler&&)                 = delete;
    Scheduler& operator=(Scheduler&&)      = delete;

    // ========================================================================
    // System Registration
    // ========================================================================

    /**
     * @brief Add a system to the scheduler
     *
     * Systems are analyzed at build time to extract channel dependencies
     * and determine the execution policy.
     *
     * ## Template parameters
     *
     * - `Policy`: execution policy (fixed_rate, fixed_rate_silent, pool_compute)
     * - `F`: callable type for the system function
     *
     * ## Parameters
     *
     * - `name`: human-readable system name (for debugging)
     * - `func`: the system function to execute
     */
    template <typename Policy = default_policy, typename F>
    auto add_system(std::string&& name, F&& func) -> void;

    /**
     * @brief Add a pre-built custom system to the scheduler
     *
     * This overload allows advanced users to provide their own SystemBase implementation,
     * including custom bind/run behavior and runtime-constructed SystemMeta.
     *
     * The system's execution policy is taken from `system->meta().policy`.
     *
     * ## Notes
     *
     * - Registration is only valid while the scheduler is not running.
     * - The scheduler's dependency analysis and validation still uses SystemMeta.
     * - Custom systems are trusted to keep `meta()`, `bind()`, and `run()` consistent.
     * - Declared channel handles must be acquired in `bind()`. `run()` must not dynamically
     *   acquire channels from `World` or mutate scheduler topology.
     */
    [[nodiscard]] auto add_system(std::unique_ptr<SystemBase> system)
        -> std::expected<uint64_t, SchedulerError>;

    /**
     * @brief Get the world
     *
     * ## Returns
     *
     * Reference to the World containing resources and channels
     *
     * ## Contract
     *
     * Structural mutation of resources should happen before the first successful `build()`.
     * After that point, safe-path insertion or replacement is rejected to preserve bound
     * pointer stability. Channel acquisition through `World` is reserved for scheduler
     * bind-time; external code must not lazily create or claim channel endpoints after build.
     */
    [[nodiscard]] World& world() noexcept;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Build dependency graph and prepare for execution
     *
     * Validates the system graph:
     * - No dependency cycles
     * - Channel constraints (SPSC single writer, etc.)
     * - Orphaned readers detected
     * - All compute systems must be reachable from an external wake source
     *
     * ## Resource semantics
     *
     * Resources (`res<T>` / `res_mut<T>`) do not participate in dependency analysis
     * or wake-up propagation. Their thread-safety contract is defined by the resource type.
     * After the first successful build, resource structure is frozen on the safe path: the
     * value behind an existing resource may change, but safe-path insertion or replacement is
     * rejected.
     *
     * Can be called multiple times for dynamic system addition (idempotent).
     * Each call re-validates all channels and rebuilds wake-up chains.
     *
     * ## Returns
     *
     * `std::expected<void, BuildError>` - success or detailed error
     *
     * ## Thread safety
     *
     * Must be called from a single thread and is rejected while `run()` is active.
     */
    [[nodiscard]] auto build() -> BuildResult;

    /**
     * @brief Start scheduler, blocks until stop() is called
     *
     * Launches fixed_rate system threads and enters the compute loop.
     *
     * ## Returns
     *
     * `std::expected<void, SchedulerError>` - success or lifecycle error
     */
    [[nodiscard]] auto run() -> std::expected<void, SchedulerError>;

    /**
     * @brief Request graceful shutdown
     *
     * Transitions the scheduler from `Running` back to `Built`.
     * FixedRate threads will exit after their current tick completes.
     */
    void stop() noexcept;

    /**
     * @brief Register a callback invoked during stop(), before lifecycle transition
     *
     * Hooks run in registration order. Use for cleanup that must happen before the
     * scheduler stops executing systems (e.g. stopping camera streaming).
     */
    void add_shutdown_hook(std::function<void()> hook);

    /**
     * @brief Add a system and rebuild on the safe path
     *
     * Safe-path hot-add is only supported before `run()`. Once the scheduler is running,
     * callers must opt in to the explicitly unsafe escape hatch.
     *
     * ## Parameters
     *
     * - `system`: the system to add (moved)
     *
     * ## Returns
     *
     * BuildResult indicating success or failure
     *
     * ## Thread safety
     *
     * Must be called from a single thread before `run()`.
     *
     * ## Notes
     *
     * Equivalent to `add_system()` + `build()` while not running. Calling this while the
     * scheduler is active aborts and points callers at `unsafe_hot_add_system()`.
     */
    [[nodiscard]] auto hot_add_system(std::unique_ptr<SystemBase> system) -> BuildResult;

    /**
     * @brief Experimental runtime hot-add escape hatch
     *
     * This preserves the legacy running hot-add path for experiments. It is intentionally
     * marked unsafe because it does not provide a full scheduler-wide thread-safety proof.
     *
     * ## Notes
     *
     * - While not running, this behaves like the safe `hot_add_system()`
     * - While running, it only pauses the compute loop before rebuilding topology
     * - Callers are responsible for any additional synchronization they require
     */
    [[nodiscard]] auto unsafe_hot_add_system(std::unique_ptr<SystemBase> system) -> BuildResult;

    /**
     * @brief Check if running
     *
     * ## Returns
     *
     * `true` if the scheduler is currently running
     */
    [[nodiscard]] auto is_running() const noexcept -> bool;

    [[nodiscard]] auto lifecycle_state() const noexcept -> LifecycleState;

    // ========================================================================
    // Notification (called by fixed_rate threads)
    // ========================================================================

    /**
     * @brief Notify scheduler that a system has produced data
     *
     * Called by fixed_rate systems to trigger dependent compute systems.
     * The notification uses a bitmask for efficient selective wakeup.
     *
     * ## Parameters
     *
     * - `system_index`: index of the fixed_rate system in fixed_rate_systems_
     *
     * ## Thread safety
     *
     * Safe to call from any fixed_rate system thread.
     */
    void notify(std::size_t system_index) noexcept;

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /**
     * @brief Runtime statistics
     */
    struct Stats {
        std::uint64_t notify_count        = 0; ///< Total notifications received
        std::uint64_t compute_cycle_count = 0; ///< Total compute cycles executed
    };

    /**
     * @brief Get runtime statistics
     *
     * ## Returns
     *
     * Current scheduler statistics
     */
    [[nodiscard]] auto stats() const noexcept -> Stats;

    /**
     * @brief Print registered systems info
     *
     * Outputs system names, policies, and channel counts to stdout.
     */
    void print_systems() const;

    /**
     * @brief Print wake chains as Mermaid diagram
     *
     * Outputs the wake graph showing which systems can actually trigger compute work
     * in Mermaid flowchart syntax.
     * Requires a successful `build()` / `Node::finalize()` so cached wake masks are current.
     *
     * ## Edge labels
     *
     * - Edge labels show the wake-causing channel; payload type is appended when topic names
     *   alone would be ambiguous
     * - Shared resources (`res<T>` / `res_mut<T>`) are intentionally excluded
     * - `fixed_rate_silent` systems are shown as side inputs, not wake sources
     * - External compute sources (for example `RclPubSystem`) are grouped separately
     */
    void print_mermaid_wake_chains() const;

    /**
     * @brief Print data flow DAG as Mermaid diagram
     *
     * Outputs the complete data flow graph with channels and shared resources.
     * Shows both fixed_rate and compute systems with their data dependencies.
     *
     * ## Visualization style
     *
     * - Nodes: systems (grouped by semantic role)
     * - Nodes also include shared resource objects when accessed via `res<T>` / `res_mut<T>`
     * - Solid colored edges: channel flow (writer → reader)
     * - Dashed grey edges: shared resource access only; these do not imply wake-up causality
     * - `res<T>` is rendered as resource → system; `res_mut<T>` is rendered in both directions
     *   to show mutable shared-state access
     * - This graph is broader than `print_mermaid_wake_chains()`: it includes data dependencies
     *   that do not participate in scheduling
     * - Invalid channel topologies are annotated instead of rendered as misleading edges
     */
    void print_mermaid_data_flow() const;

    /**
     * @brief Print execution levels as Mermaid diagram
     *
     * Outputs the topologically sorted compute levels plus external wake sources.
     * Levels describe the compute closure stages the scheduler drains within one round.
     * Requires a successful `build()` / `Node::finalize()` so cached levels and wake masks are
     * current.
     *
     * ## Visualization style
     *
     * - Subgraphs: execution levels (systems within can run in parallel)
     * - Nodes: systems within each level, labeled with channel I/O counts
     *   (reader endpoints vs writer endpoints; resources excluded)
     * - Edges: all direct wake-causing dependencies across compute levels
     * - `fixed_rate_silent` systems are shown as side inputs, not level-entry wakers
     * - Analysis: pipeline depth and max parallelism printed as comments
     */
    void print_mermaid_execution_levels() const;

    /**
     * @brief Print performance statistics
     *
     * Outputs latency histograms and throughput metrics to stdout.
     */
    void print_stats() const;

    /**
     * @brief Get performance statistics as JSON string
     *
     * ## Returns
     *
     * JSON-formatted statistics string
     */
    [[nodiscard]] std::string get_stats_json() const;

private:
    // ========================================================================
    // Internal Types
    // ========================================================================

    struct FixedRateContext {
        SystemBase* system;
        PolicyInfo policy;
        std::atomic<LifecycleState>* lifecycle;
        Scheduler* scheduler;
        World* world;
        std::size_t system_index;
    };

    /**
     * @brief Performance statistics for fixed_rate systems
     */
    struct alignas(std::hardware_destructive_interference_size) FixedRateStats {
        std::atomic<std::uint64_t> notify_count{0};
        std::atomic<std::uint64_t> execution_count{0};
        std::atomic<std::uint64_t> record_count{0};
        primitive::LatencyHistogram latency_hist;

        FixedRateStats() = default;

        // Move constructor: atomics are not movable, so we load and store
        FixedRateStats(FixedRateStats&& other) noexcept;

        // Non-copyable, non-movable-assignable
        FixedRateStats(const FixedRateStats&)            = delete;
        FixedRateStats& operator=(const FixedRateStats&) = delete;
        FixedRateStats& operator=(FixedRateStats&&)      = delete;
    };

    /**
     * @brief Performance statistics for compute systems
     */
    struct ComputeStats {
        std::atomic<std::uint64_t> run_count{0};
        primitive::LatencyHistogram latency_hist;

        ComputeStats() = default;
        ComputeStats(ComputeStats&& other) noexcept;
        ComputeStats(const ComputeStats&)            = delete;
        ComputeStats& operator=(const ComputeStats&) = delete;
        ComputeStats& operator=(ComputeStats&&)      = delete;
    };

    // ========================================================================
    // Internal Methods
    // ========================================================================

    struct ComputeRoundResult {
        std::uint64_t written_mask  = 0;
        std::uint64_t deferred_mask = 0;
    };

    struct TopologySnapshot {
        std::vector<std::vector<std::size_t>> levels;
        std::vector<std::uint64_t> fixed_rate_affects;
        std::vector<std::uint64_t> compute_affects;
    };

    static void run_fixed_rate_thread(FixedRateContext ctx);
    void run_compute_loop();
    [[nodiscard]] ComputeRoundResult run_compute_selective(std::uint64_t ready_mask);

    [[nodiscard]] auto build_topology_snapshot() const
        -> std::expected<TopologySnapshot, BuildError>;
    void commit_topology(TopologySnapshot snapshot) noexcept;
    void bind_external_compute_sources() noexcept;
    static bool is_running_state(LifecycleState state) noexcept;

    // Pause/resume helpers for hot-add
    void request_compute_loop_pause() noexcept;
    void resume_compute_loop() noexcept;

    // Helper: sum notify counts from all fixed_rate systems
    [[nodiscard]] std::uint64_t sum_notify_counts() const noexcept;

    // Helper: check if scheduler is running and abort if so (eliminates duplicate checks)
    void ensure_not_running() const noexcept;

    void mark_topology_dirty() noexcept;

    // ========================================================================
    // Members
    // ========================================================================

    World world_{};
    SchedulerConfig config_;

    // Systems
    std::vector<SystemEntry> fixed_rate_systems_;
    std::vector<SystemEntry> compute_systems_;

    // Execution levels for compute systems (topologically sorted)
    std::vector<std::vector<std::size_t>> levels_;

    // Threads
    std::vector<std::jthread> fixed_rate_threads_;

    // TBB
    std::unique_ptr<tbb::task_arena> compute_arena_;

    // Lifecycle
    std::atomic<LifecycleState> lifecycle_{LifecycleState::Configuring};

    // Selective notification: wake chains (waker -> direct wakee only)
    // Transitive propagation is handled by compute-round closure.
    std::vector<std::uint64_t>
        fixed_rate_affects_; // [fixed_rate_idx] -> bitmask of compute systems that directly read
                             // this fixed_rate's channels
    std::vector<std::uint64_t> compute_affects_; // [compute_idx] -> bitmask of compute systems that
                                                 // directly depend on this system via channels
    std::atomic<std::uint64_t> ready_systems_{0}; // Bitmask of compute systems pending execution

    // Stats
    std::atomic<std::uint64_t> compute_cycles_{0};
    std::atomic<std::uint64_t> compute_total_time_ns_{0};
    std::atomic<std::uint64_t> compute_last_time_ns_{0};
    std::vector<FixedRateStats> fixed_rate_stats_;
    std::vector<ComputeStats> compute_stats_;
    TimePoint stats_start_time_;

    // Hot-add pause state
    std::atomic<bool> pause_requested_{false};
    std::atomic<bool> paused_{false};
    std::condition_variable pause_cv_;
    std::mutex pause_mutex_;

    // Shutdown hooks (executed in registration order during stop())
    std::vector<std::function<void()>> shutdown_hooks_;
};

// ============================================================================
// Template Implementation
// ============================================================================

template <typename Policy, typename F>
auto Scheduler::add_system(std::string&& name, F&& func) -> void {
    ensure_not_running();
    mark_topology_dirty();

    auto system = make_system<F, Policy>(std::move(name), std::forward<F>(func));
    auto policy = make_policy_info<Policy>();

    SystemEntry entry{
        .system = std::move(system),
        .policy = policy,
        .bound  = false,
    };

    if constexpr (is_fixed_rate_policy_v<Policy>) {
        fixed_rate_systems_.push_back(std::move(entry));
        return;
    }
    if constexpr (is_pool_policy_v<Policy> || is_visualization_policy_v<Policy>) {
        compute_systems_.push_back(std::move(entry));
        return;
    }
    std::unreachable();
}

} // namespace talos::scheduler

// ============================================================================
// Convenience namespace aliases for talos::
// ============================================================================

namespace talos {
using namespace talos::scheduler::system;
using talos::scheduler::BuildError;
using talos::scheduler::ChannelKindConflict;
using talos::scheduler::DependencyCycleError;
using talos::scheduler::MultipleReadersError;
using talos::scheduler::MultipleWritersError;
using talos::scheduler::OrphanedReaderError;
using talos::scheduler::Scheduler;
using talos::scheduler::SchedulerError;
using talos::scheduler::TooManyComputeSystemsError;
using talos::scheduler::UnreachableComputeSystemsError;
using talos::scheduler::World;
} // namespace talos
