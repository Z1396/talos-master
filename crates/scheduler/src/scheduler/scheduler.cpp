#include "scheduler.hpp"

#include <tbb/task_arena.h>
#include <tbb/task_group.h>

#include "demangle.hpp"
#include "primitive/performance_probe.hpp"
#include "primitive/spin.hpp"
#include "primitive/thread_affinity.hpp"

#include <algorithm>
#include <bit>
#include <cinttypes>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <unordered_map>

namespace talos::scheduler {

// ============================================================================
// Internal Helpers
// ============================================================================

/// Helper: Print wake chains for a given set of systems
/// @param title Section title to print
/// @param source_systems Vector of source systems to iterate
/// @param affects_mask Vector of bitmasks indicating which target systems are affected
/// @param target_systems Vector of target systems for name lookup
template <typename SourceSystemEntry>
static void print_wake_chains(
    const char* title, const std::vector<SourceSystemEntry>& source_systems,
    const std::vector<std::uint64_t>& affects_mask,
    const std::vector<SystemEntry>& target_systems) {
    printf("%s\n", title);
    for (std::size_t i = 0; i < source_systems.size(); ++i) {
        const auto& src_name = source_systems[i].system->meta().name;
        const auto affects   = i < affects_mask.size() ? affects_mask[i] : 0U;

        printf("  [%s] -> ", src_name.c_str());
        if (affects == 0) {
            printf("(none)");
        } else {
            bool first = true;
            for (std::size_t j = 0; j < target_systems.size(); ++j) {
                if (affects & (1UL << j)) {
                    if (!first)
                        printf(", ");
                    printf("[%s]", target_systems[j].system->meta().name.c_str());
                    first = false;
                }
            }
        }
        printf("\n");
    }
}

struct ChannelIoCounts {
    std::size_t inputs  = 0;
    std::size_t outputs = 0;
};

template <typename Channels>
static void
    accumulate_channel_io_counts(const Channels& channels, ChannelIoCounts& counts) noexcept {
    for (const auto& channel : channels) {
        switch (channel.kind) {
        case channel_kind::spsc_reader:
        case channel_kind::spmc_reader: counts.inputs++; break;
        case channel_kind::spsc_writer:
        case channel_kind::spmc_writer: counts.outputs++; break;
        default: std::unreachable();
        }
    }
}

static auto count_channel_io(const SystemMeta& meta) noexcept -> ChannelIoCounts {
    ChannelIoCounts counts;
    accumulate_channel_io_counts(meta.spsc_channels, counts);
    accumulate_channel_io_counts(meta.spmc_channels, counts);
    return counts;
}

static constexpr auto is_writer_channel_kind(const channel_kind kind) noexcept -> bool {
    return kind == channel_kind::spsc_writer || kind == channel_kind::spmc_writer;
}

template <typename Channels>
static auto has_writer_channels(const Channels& channels) noexcept -> bool {
    return std::any_of(channels.begin(), channels.end(), [](const auto& channel) {
        return is_writer_channel_kind(channel.kind);
    });
}

static auto counts_written_calls(const SystemMeta& meta) noexcept -> bool {
    return has_writer_channels(meta.spsc_channels) || has_writer_channels(meta.spmc_channels);
}

struct ChannelLabelNames {
    std::string topic_name;
    std::string type_name;
};

template <typename Systems>
static auto build_resource_label_map(const Systems& systems)
    -> std::map<std::type_index, std::string> {
    std::map<std::type_index, std::string> resource_labels;
    for (const auto& entry : systems) {
        const auto& meta = entry.system->meta();
        for (const auto& type : meta.reads) {
            resource_labels.try_emplace(type, talos::scheduler::detail::demangle(type.name()));
        }
        for (const auto& type : meta.writes) {
            resource_labels.try_emplace(type, talos::scheduler::detail::demangle(type.name()));
        }
    }
    return resource_labels;
}

template <typename Systems>
static void collect_channel_label_names(
    const Systems& systems, std::map<ChannelKey, ChannelLabelNames>& channel_names) {
    for (const auto& entry : systems) {
        const auto& meta = entry.system->meta();
        auto collect     = [&](const auto& channels) {
            for (const auto& channel : channels) {
                const ChannelKey key{channel.type, channel.topic};
                channel_names.try_emplace(
                    key, ChannelLabelNames{
                                 .topic_name = talos::scheduler::detail::demangle(channel.topic.name()),
                                 .type_name  = talos::scheduler::detail::demangle(channel.type.name()),
                         });
            }
        };
        collect(meta.spsc_channels);
        collect(meta.spmc_channels);
    }
}

template <typename FixedSystems, typename ComputeSystems>
static auto build_channel_label_map(
    const FixedSystems& fixed_systems, const ComputeSystems& compute_systems)
    -> std::map<ChannelKey, std::string> {
    std::map<ChannelKey, ChannelLabelNames> channel_names;
    collect_channel_label_names(fixed_systems, channel_names);
    collect_channel_label_names(compute_systems, channel_names);

    std::unordered_map<std::string, std::size_t> topic_name_counts;
    for (const auto& [_, names] : channel_names) {
        topic_name_counts[names.topic_name]++;
    }

    std::map<ChannelKey, std::string> channel_labels;
    for (const auto& [key, names] : channel_names) {
        std::string label = names.topic_name;
        if (topic_name_counts[names.topic_name] > 1) {
            label += " [";
            label += names.type_name;
            label += "]";
        }
        channel_labels.emplace(key, std::move(label));
    }

    return channel_labels;
}

static void print_mermaid_rebuild_hint(const char* message) {
    printf("```mermaid\n");
    printf("flowchart LR\n\n");
    printf("    BuildRequired[\"%s\"]\n", message);
    printf("```\n");
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

Scheduler::Scheduler(const SchedulerConfig config) noexcept
    : config_(config)
    , stats_start_time_(Clock::now()) {}

Scheduler::~Scheduler() noexcept {
    stop();

    for (auto& t : fixed_rate_threads_) {
        if (t.joinable()) {
            t.join();
        }
    }
}

Scheduler::FixedRateStats::FixedRateStats(FixedRateStats&& other) noexcept
    : notify_count(other.notify_count.load(std::memory_order_relaxed))
    , execution_count(other.execution_count.load(std::memory_order_relaxed))
    , record_count(other.record_count.load(std::memory_order_relaxed))
    , latency_hist(std::move(other.latency_hist)) {}

Scheduler::ComputeStats::ComputeStats(ComputeStats&& other) noexcept
    : run_count(other.run_count.load(std::memory_order_relaxed))
    , latency_hist(std::move(other.latency_hist)) {}

World& Scheduler::world() noexcept { return world_; }

bool Scheduler::is_running_state(const LifecycleState state) noexcept {
    return state == LifecycleState::Running;
}

void Scheduler::ensure_not_running() const noexcept {
    if (is_running_state(lifecycle_.load(std::memory_order_acquire))) {
        panic("add_system: system already running!");
    }
}

void Scheduler::mark_topology_dirty() noexcept {
    lifecycle_.store(LifecycleState::Configuring, std::memory_order_release);
}

// ============================================================================
// Lifecycle
// ============================================================================

auto Scheduler::add_system(std::unique_ptr<SystemBase> system)
    -> std::expected<uint64_t, SchedulerError> {
    if (!system) {
        std::abort();
    }
    ensure_not_running();
    mark_topology_dirty();

    const auto policy = system->meta().policy;
    SystemEntry entry{
        .system = std::move(system),
        .policy = policy,
        .bound  = false,
    };

    if (policy.is_fixed_rate()) {
        fixed_rate_systems_.push_back(std::move(entry));
        return static_cast<uint64_t>(fixed_rate_systems_.size() - 1);
    }
    if (policy.is_pool() || policy.is_visualization()) {
        compute_systems_.push_back(std::move(entry));
        return static_cast<uint64_t>(compute_systems_.size() - 1);
    }
    std::unreachable();
}

auto Scheduler::build() -> BuildResult {
    if (is_running()) {
        return std::unexpected(BuildError{SchedulerError::AlreadyRunning});
    }

    auto topology_result = build_topology_snapshot();
    if (!topology_result) {
        return std::unexpected(topology_result.error());
    }

    // Create TBB arena only on first call
    if (!compute_arena_) {
        if (config_.compute_concurrency > 0) {
            compute_arena_ =
                std::make_unique<tbb::task_arena>(static_cast<int>(config_.compute_concurrency));
        } else {
            compute_arena_ = std::make_unique<tbb::task_arena>();
        }
    }

    // Ensure stats capacity matches system count
    fixed_rate_stats_.resize(fixed_rate_systems_.size());
    compute_stats_.resize(compute_systems_.size());

    // Bind any newly-added systems (pre-create channel readers/writers, single-threaded)
    world_.open_channel_binding();
    auto bind_all_systems = [&](auto& systems) {
        for (auto& entry : systems) {
            if (!entry.bound) {
                entry.system->bind(world_);
                entry.bound = true;
            }
        }
    };
    bind_all_systems(fixed_rate_systems_);
    bind_all_systems(compute_systems_);
    world_.close_channel_binding();
    bind_external_compute_sources();
    world_.freeze_resource_structure();

    commit_topology(std::move(*topology_result));
    lifecycle_.store(LifecycleState::Built, std::memory_order_release);
    return {};
}

auto Scheduler::run() -> std::expected<void, SchedulerError> {
    auto expected = LifecycleState::Built;
    if (!lifecycle_.compare_exchange_strong(
            expected, LifecycleState::Running, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
        if (expected == LifecycleState::Running) {
            return std::unexpected(SchedulerError::AlreadyRunning);
        }
        return std::unexpected(SchedulerError::NotBuilt);
    }

    // Start fixed_rate threads
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        auto& entry = fixed_rate_systems_[i];
        FixedRateContext ctx{
            .system       = entry.system.get(),
            .policy       = entry.policy,
            .lifecycle    = &lifecycle_,
            .scheduler    = this,
            .world        = &world_,
            .system_index = i,
        };

        fixed_rate_threads_.emplace_back([ctx]() mutable { run_fixed_rate_thread(ctx); });
    }

    // Run compute loop on main thread (blocking)
    run_compute_loop();

    // Wait for fixed_rate threads
    for (auto& t : fixed_rate_threads_) {
        if (t.joinable()) {
            t.join();
        }
    }
    fixed_rate_threads_.clear();

    return {};
}

void Scheduler::stop() noexcept {
    auto expected = LifecycleState::Running;
    lifecycle_.compare_exchange_strong(
        expected, LifecycleState::Built, std::memory_order_acq_rel, std::memory_order_acquire);

    // Wake up compute loop if it's paused
    {
        std::lock_guard lock(pause_mutex_);
        pause_requested_.store(false, std::memory_order_release);
    }
    pause_cv_.notify_all();
}

void Scheduler::add_shutdown_hook(std::function<void()> hook) {
    shutdown_hooks_.push_back(std::move(hook));
}

auto Scheduler::hot_add_system(std::unique_ptr<SystemBase> system) -> BuildResult {
    if (!system) {
        std::abort();
    }

    if (is_running()) {
        panic(
            "hot_add_system() while scheduler is running is unsafe; use "
            "unsafe_hot_add_system() if you need the explicit escape hatch");
    }

    if (auto result = add_system(std::move(system)); !result) {
        return std::unexpected(BuildError{result.error()});
    }
    return build();
}

auto Scheduler::unsafe_hot_add_system(std::unique_ptr<SystemBase> system) -> BuildResult {
    if (!system) {
        std::abort();
    }

    if (!is_running()) {
        return hot_add_system(std::move(system));
    }

    // Hot-add: request pause
    request_compute_loop_pause();

    // Now safely add the system
    const auto policy = system->meta().policy;
    SystemEntry entry{
        .system = std::move(system),
        .policy = policy,
        .bound  = false,
    };

    const bool is_fixed_rate = policy.is_fixed_rate();
    if (is_fixed_rate) {
        fixed_rate_systems_.push_back(std::move(entry));
    } else if (policy.is_pool() || policy.is_visualization()) {
        compute_systems_.push_back(std::move(entry));
    } else {
        std::unreachable();
    }

    auto topology_result = build_topology_snapshot();
    if (!topology_result) {
        // Rollback: remove the just-added system
        if (is_fixed_rate) {
            fixed_rate_systems_.pop_back();
        } else {
            compute_systems_.pop_back();
        }

        // Resume compute loop
        resume_compute_loop();

        return std::unexpected(topology_result.error());
    }

    // Update stats capacity
    fixed_rate_stats_.resize(fixed_rate_systems_.size());
    compute_stats_.resize(compute_systems_.size());

    // Bind the new system
    world_.open_channel_binding();
    if (is_fixed_rate) {
        auto& new_entry = fixed_rate_systems_.back();
        if (!new_entry.bound) {
            new_entry.system->bind(world_);
            new_entry.bound = true;
        }

        // Launch new fixed_rate thread
        const auto idx = fixed_rate_systems_.size() - 1;
        FixedRateContext ctx{
            .system       = new_entry.system.get(),
            .policy       = new_entry.policy,
            .lifecycle    = &lifecycle_,
            .scheduler    = this,
            .world        = &world_,
            .system_index = idx,
        };
        fixed_rate_threads_.emplace_back([ctx]() mutable { run_fixed_rate_thread(ctx); });
    } else {
        auto& new_entry = compute_systems_.back();
        if (!new_entry.bound) {
            new_entry.system->bind(world_);
            new_entry.bound = true;
        }
    }
    world_.close_channel_binding();
    bind_external_compute_sources();
    commit_topology(std::move(*topology_result));

    // Resume compute loop
    resume_compute_loop();

    return {};
}

auto Scheduler::is_running() const noexcept -> bool {
    return is_running_state(lifecycle_.load(std::memory_order_acquire));
}

auto Scheduler::lifecycle_state() const noexcept -> LifecycleState {
    return lifecycle_.load(std::memory_order_acquire);
}

void Scheduler::commit_topology(TopologySnapshot snapshot) noexcept {
    levels_             = std::move(snapshot.levels);
    fixed_rate_affects_ = std::move(snapshot.fixed_rate_affects);
    compute_affects_    = std::move(snapshot.compute_affects);
}

// ============================================================================
// Pause/Resume Helpers (for hot-add)
// ============================================================================

void Scheduler::request_compute_loop_pause() noexcept {
    // Request pause
    {
        std::lock_guard lock(pause_mutex_);
        pause_requested_.store(true, std::memory_order_release);
    }
    // Wait for compute loop to pause
    {
        std::unique_lock lock(pause_mutex_);
        pause_cv_.wait(lock, [this] { return paused_.load(std::memory_order_acquire); });
    }
}

void Scheduler::resume_compute_loop() noexcept {
    {
        std::lock_guard lock(pause_mutex_);
        pause_requested_.store(false, std::memory_order_release);
    }
    pause_cv_.notify_all();
}

std::uint64_t Scheduler::sum_notify_counts() const noexcept {
    std::uint64_t total = 0;
    for (const auto& s : fixed_rate_stats_) {
        total += s.notify_count.load(std::memory_order_relaxed);
    }
    return total;
}

void Scheduler::bind_external_compute_sources() noexcept {
    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        if (auto* external = compute_systems_[i].system->as_external_compute();
            external != nullptr) {
            external->bind_external_ready_slot(&ready_systems_, i);
        }
    }
}

// ============================================================================
// Notification
// ============================================================================

void Scheduler::notify(const std::size_t system_index) noexcept {
    // Selective wake: only mark affected compute systems as ready
    if (system_index < fixed_rate_affects_.size()) {
        ready_systems_.fetch_or(fixed_rate_affects_[system_index], std::memory_order_release);
    }
    if (system_index < fixed_rate_stats_.size()) {
        fixed_rate_stats_[system_index].notify_count.fetch_add(1, std::memory_order_relaxed);
    }
}

// ============================================================================
// Stats
// ============================================================================

auto Scheduler::stats() const noexcept -> Stats {
    return Stats{
        .notify_count        = sum_notify_counts(),
        .compute_cycle_count = compute_cycles_.load(std::memory_order_relaxed),
    };
}

void Scheduler::print_systems() const {
    // Unified printer for systems with metadata (eliminates duplicate loop structure)
    auto print_systems = [](const auto& systems, const char* type_name, auto print_fn) {
        printf("%s systems (%zu):\n", type_name, systems.size());
        for (const auto& entry : systems) {
            const auto& m = entry.system->meta();
            print_fn(entry, m);
        }
    };

    print_systems(fixed_rate_systems_, "FixedRate", [](const auto& entry, const auto& m) {
        printf(
            "  [%s] freq:%uHz affinity:%d priority:%d notify:%s spsc:%zu spmc:%zu\n",
            m.name.c_str(), entry.policy.frequency_hz, entry.policy.cpu_affinity,
            entry.policy.thread_priority, entry.policy.notifies ? "yes" : "no",
            m.spsc_channels.size(), m.spmc_channels.size());
    });

    print_systems(
        compute_systems_, "Compute", [](const auto& entry [[maybe_unused]], const auto& m) {
            printf(
                "  [%s] spsc:%zu spmc:%zu res:%zu res_mut:%zu\n", m.name.c_str(),
                m.spsc_channels.size(), m.spmc_channels.size(), m.reads.size(), m.writes.size());
        });

    printf("Execution levels: %zu\n", levels_.size());
    for (std::size_t i = 0; i < levels_.size(); ++i) {
        printf("  Level %zu: ", i);
        for (const auto idx : levels_[i]) {
            printf("[%s] ", compute_systems_[idx].system->meta().name.c_str());
        }
        printf("\n");
    }

    std::vector<std::uint64_t> notifying_fixed_rate_affects(fixed_rate_systems_.size(), 0);
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        if (fixed_rate_systems_[i].policy.notifies) {
            notifying_fixed_rate_affects[i] =
                i < fixed_rate_affects_.size() ? fixed_rate_affects_[i] : 0U;
        }
    }

    // Print wake chains using unified helper
    print_wake_chains(
        "Wake chains (notifying fixed_rate -> compute):", fixed_rate_systems_,
        notifying_fixed_rate_affects, compute_systems_);
    print_wake_chains(
        "Wake chains (compute -> compute):", compute_systems_, compute_affects_, compute_systems_);
}

void Scheduler::print_mermaid_wake_chains() const {
    if (lifecycle_state() == LifecycleState::Configuring) {
        print_mermaid_rebuild_hint(
            "Scheduler graph not built. Call build() to render wake chains.");
        return;
    }

    printf("```mermaid\n");
    printf("flowchart LR\n\n");

    const auto is_external_compute_source = [&](const std::size_t idx) {
        return compute_systems_[idx].system->as_external_compute() != nullptr;
    };
    const auto channel_labels = build_channel_label_map(fixed_rate_systems_, compute_systems_);

    // Build in_edges and out_edges for each compute system
    std::vector<std::size_t> in_edges(compute_systems_.size(), 0);
    std::vector<std::size_t> out_edges(compute_systems_.size(), 0);

    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        out_edges[i] = (i < compute_affects_.size()) ? std::popcount(compute_affects_[i]) : 0;
    }

    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if (i != j && (compute_affects_[i] & (1UL << j))) {
                in_edges[j]++;
            }
        }
    }

    // Also count edges from fixed_rate systems
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        if (!fixed_rate_systems_[i].policy.notifies) {
            continue;
        }
        auto affects = i < fixed_rate_affects_.size() ? fixed_rate_affects_[i] : 0U;
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if (affects & (1UL << j)) {
                in_edges[j]++;
            }
        }
    }

    // Helper: classify compute systems by wake semantics, not raw channel shape.
    auto classify_system = [&](std::size_t idx) -> const char* {
        if (is_external_compute_source(idx)) {
            return "external";
        }
        const bool has_out = out_edges[idx] > 0;
        const bool has_in  = in_edges[idx] > 0;

        if (has_out && has_in)
            return "pipeline"; // (1, 1) 有进有出
        if (!has_out)
            return "sink";
        return "internal";
    };

    bool has_notifying_fixed_rate = false;
    for (const auto& entry : fixed_rate_systems_) {
        has_notifying_fixed_rate |= entry.policy.notifies;
    }
    if (has_notifying_fixed_rate) {
        printf("    subgraph ExternalWake[Notifying FixedRate Sources]\n");
        printf("        direction TB\n");
        for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
            if (!fixed_rate_systems_[i].policy.notifies) {
                continue;
            }
            const auto& name = fixed_rate_systems_[i].system->meta().name;
            const auto freq  = fixed_rate_systems_[i].policy.frequency_hz;
            printf("        E%d[\"%s<br/>%dHz\"]:::fixedStyle\n", (int)i, name.c_str(), freq);
        }
        printf("    end\n\n");
    }

    bool has_silent_fixed_rate = false;
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        if (!fixed_rate_systems_[i].policy.notifies) {
            has_silent_fixed_rate = true;
            break;
        }
    }
    if (has_silent_fixed_rate) {
        printf("    subgraph SilentInputs[Silent FixedRate Side Inputs]\n");
        printf("        direction TB\n");
        for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
            if (fixed_rate_systems_[i].policy.notifies) {
                continue;
            }
            const auto& name = fixed_rate_systems_[i].system->meta().name;
            const auto freq  = fixed_rate_systems_[i].policy.frequency_hz;
            printf("        E%d[\"%s<br/>%dHz\"]:::fixedStyle\n", (int)i, name.c_str(), freq);
        }
        printf("    end\n\n");
    }

    // Group compute systems by wake role
    std::vector<std::size_t> external_compute_systems, pipeline_systems, sink_systems,
        internal_systems;
    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        const auto* type = classify_system(i);
        if (strcmp(type, "external") == 0)
            external_compute_systems.push_back(i);
        else if (strcmp(type, "sink") == 0)
            sink_systems.push_back(i);
        else if (strcmp(type, "internal") == 0)
            internal_systems.push_back(i);
        else
            pipeline_systems.push_back(i);
    }

    if (!external_compute_systems.empty()) {
        printf("    subgraph ExternalCompute[External Compute Sources]\n");
        printf("        direction TB\n");
        for (auto i : external_compute_systems) {
            const auto& name = compute_systems_[i].system->meta().name;
            printf("        C%d[\"%s\"]:::inputStyle\n", (int)i, name.c_str());
        }
        printf("    end\n\n");
    }

    if (!pipeline_systems.empty()) {
        printf("    subgraph Pipeline[Pipeline Systems]\n");
        printf("        direction TB\n");
        for (auto i : pipeline_systems) {
            const auto& name = compute_systems_[i].system->meta().name;
            printf("        C%d[\"%s\"]:::pipelineStyle\n", (int)i, name.c_str());
        }
        printf("    end\n\n");
    }

    if (!sink_systems.empty()) {
        printf("    subgraph Sink[Sink Systems]\n");
        printf("        direction TB\n");
        for (auto i : sink_systems) {
            const auto& name = compute_systems_[i].system->meta().name;
            printf("        C%d[\"%s\"]:::outputStyle\n", (int)i, name.c_str());
        }
        printf("    end\n\n");
    }

    if (!internal_systems.empty()) {
        printf("    subgraph Internal[Internal Compute Roots]\n");
        printf("        direction TB\n");
        for (auto i : internal_systems) {
            const auto& name = compute_systems_[i].system->meta().name;
            printf("        C%d[\"%s\"]:::orphanStyle\n", (int)i, name.c_str());
        }
        printf("    end\n\n");
    }

    // Helper: collect channel topics that connect writer to reader
    auto get_channel_topics = [&](const SystemBase* writer,
                                  const SystemBase* reader) -> std::string {
        std::vector<std::string> topics;
        const auto& writer_meta = writer->meta();
        const auto& reader_meta = reader->meta();

        // Helper to check channel connections
        auto check_channels = [&](const auto& writer_channels, const auto& reader_channels) {
            for (const auto& wc : writer_channels) {
                if (wc.kind != channel_kind::spsc_writer && wc.kind != channel_kind::spmc_writer) {
                    continue;
                }
                for (const auto& rc : reader_channels) {
                    if (rc.kind != channel_kind::spsc_reader
                        && rc.kind != channel_kind::spmc_reader) {
                        continue;
                    }
                    if (wc.type == rc.type && wc.topic == rc.topic) {
                        const ChannelKey key{wc.type, wc.topic};
                        if (auto it = channel_labels.find(key); it != channel_labels.end()) {
                            topics.push_back(it->second);
                        }
                    }
                }
            }
        };

        check_channels(writer_meta.spsc_channels, reader_meta.spsc_channels);
        check_channels(writer_meta.spsc_channels, reader_meta.spmc_channels);
        check_channels(writer_meta.spmc_channels, reader_meta.spmc_channels);

        if (topics.empty()) {
            return "";
        }
        std::sort(topics.begin(), topics.end());
        topics.erase(std::unique(topics.begin(), topics.end()), topics.end());
        if (topics.size() == 1) {
            return topics[0];
        }
        // Multiple channels: join with commas
        std::string result;
        for (const auto& t : topics) {
            if (!result.empty())
                result += ", ";
            result += t;
        }
        return result;
    };

    // FixedRate → Compute
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        if (!fixed_rate_systems_[i].policy.notifies) {
            continue;
        }
        auto affects = i < fixed_rate_affects_.size() ? fixed_rate_affects_[i] : 0U;
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if (affects & (1UL << j)) {
                const auto* writer = fixed_rate_systems_[i].system.get();
                const auto* reader = compute_systems_[j].system.get();
                const auto topic   = get_channel_topics(writer, reader);
                if (topic.empty()) {
                    printf("    E%d --> C%d\n", (int)i, (int)j);
                } else {
                    printf("    E%d -->|\"%s\"| C%d\n", (int)i, topic.c_str(), (int)j);
                }
            }
        }
    }

    // Compute → Compute (channels only)
    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        auto affects = i < compute_affects_.size() ? compute_affects_[i] : 0U;
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if ((affects & (1UL << j)) && i != j) {
                const auto* writer       = compute_systems_[i].system.get();
                const auto* reader       = compute_systems_[j].system.get();
                const auto channel_topic = get_channel_topics(writer, reader);
                if (channel_topic.empty()) {
                    printf("    C%d --> C%d\n", (int)i, (int)j);
                } else {
                    printf("    C%d -->|\"%s\"| C%d\n", (int)i, channel_topic.c_str(), (int)j);
                }
            }
        }
    }

    printf("```\n");
}

void Scheduler::print_mermaid_data_flow() const {
    printf("```mermaid\n");
    printf("flowchart LR\n\n");

    const auto is_external_compute_source = [&](const std::size_t idx) {
        return compute_systems_[idx].system->as_external_compute() != nullptr;
    };
    const auto channel_labels = build_channel_label_map(fixed_rate_systems_, compute_systems_);

    // Define distinct path colors for different data flows
    const char* path_colors[] = {
        "#e53935", // Red
        "#1e88e5", // Blue
        "#43a047", // Green
        "#fb8c00", // Orange
        "#8e24aa", // Purple
        "#00acc1", // Cyan
        "#fdd835", // Yellow
        "#6d4c41", // Brown
        "#546e7a", // Blue Grey
        "#d81b60", // Pink
    };
    constexpr std::size_t num_colors = sizeof(path_colors) / sizeof(path_colors[0]);

    // ========================================================================
    // Build data flow graph from channels and shared resources
    // ========================================================================

    struct ChannelEndpoint {
        std::size_t system_idx;
        bool is_fixed_rate;
    };

    // Channel connection: (type, topic) -> (writer, readers)
    struct ChannelConnection {
        std::vector<ChannelEndpoint> writers;
        std::vector<ChannelEndpoint> readers;
        std::type_index type;
        std::type_index topic;
        channel_kind kind;
    };

    std::unordered_map<ChannelKey, ChannelConnection, ChannelKeyHash> channel_graph;
    struct ResourceAccess {
        std::vector<ChannelEndpoint> readers;
        std::vector<ChannelEndpoint> mutators;
    };
    std::map<std::type_index, ResourceAccess> resource_graph;

    const std::size_t fixed_count      = fixed_rate_systems_.size();
    const std::size_t compute_count    = compute_systems_.size();
    const auto fixed_resource_labels   = build_resource_label_map(fixed_rate_systems_);
    const auto compute_resource_labels = build_resource_label_map(compute_systems_);
    std::map<std::type_index, std::string> resource_labels = fixed_resource_labels;
    resource_labels.insert(compute_resource_labels.begin(), compute_resource_labels.end());

    // Helper: process a system's channels
    auto process_system_channels = [&](std::size_t idx, bool is_fixed, const SystemMeta& meta) {
        ChannelEndpoint endpoint{idx, is_fixed};

        // Process SPSC channels
        for (const auto& ch : meta.spsc_channels) {
            ChannelKey key{ch.type, ch.topic};
            auto it = channel_graph.find(key);
            if (it == channel_graph.end()) {
                it = channel_graph
                         .emplace(
                             key,
                             ChannelConnection{
                                 .writers = {},
                                 .readers = {},
                                 .type    = ch.type,
                                 .topic   = ch.topic,
                                 .kind    = ch.kind})
                         .first;
            }

            if (ch.kind == channel_kind::spsc_writer) {
                it->second.writers.push_back(endpoint);
            } else if (ch.kind == channel_kind::spsc_reader) {
                it->second.readers.push_back(endpoint);
            }
        }

        // Process SPMC channels
        for (const auto& ch : meta.spmc_channels) {
            ChannelKey key{ch.type, ch.topic};
            auto it = channel_graph.find(key);
            if (it == channel_graph.end()) {
                it = channel_graph
                         .emplace(
                             key,
                             ChannelConnection{
                                 .writers = {},
                                 .readers = {},
                                 .type    = ch.type,
                                 .topic   = ch.topic,
                                 .kind    = ch.kind})
                         .first;
            }

            if (ch.kind == channel_kind::spmc_writer) {
                it->second.writers.push_back(endpoint);
            } else if (ch.kind == channel_kind::spmc_reader) {
                it->second.readers.push_back(endpoint);
            }
        }

        for (const auto& type : meta.reads) {
            resource_graph[type].readers.push_back(endpoint);
        }
        for (const auto& type : meta.writes) {
            resource_graph[type].mutators.push_back(endpoint);
        }
    };

    // Build graphs from all systems
    for (std::size_t i = 0; i < fixed_count; ++i) {
        const auto& meta = fixed_rate_systems_[i].system->meta();
        process_system_channels(i, true, meta);
    }
    for (std::size_t i = 0; i < compute_count; ++i) {
        const auto& meta = compute_systems_[i].system->meta();
        process_system_channels(i, false, meta);
    }

    auto classify_compute_by_data_flow = [&](const std::size_t idx) -> const char* {
        if (is_external_compute_source(idx)) {
            return "external";
        }

        const auto& meta   = compute_systems_[idx].system->meta();
        const bool has_pub = std::any_of(
                                 meta.spsc_channels.begin(), meta.spsc_channels.end(),
                                 [](const auto& c) { return c.kind == channel_kind::spsc_writer; })
                          || std::any_of(
                                 meta.spmc_channels.begin(), meta.spmc_channels.end(),
                                 [](const auto& c) { return c.kind == channel_kind::spmc_writer; })
                          || !meta.writes.empty();
        const bool has_sub = std::any_of(
                                 meta.spsc_channels.begin(), meta.spsc_channels.end(),
                                 [](const auto& c) { return c.kind == channel_kind::spsc_reader; })
                          || std::any_of(
                                 meta.spmc_channels.begin(), meta.spmc_channels.end(),
                                 [](const auto& c) { return c.kind == channel_kind::spmc_reader; })
                          || !meta.reads.empty() || !meta.writes.empty();

        if (has_pub && has_sub)
            return "pipeline";
        if (!has_pub && has_sub)
            return "sink";
        if (has_pub && !has_sub)
            return "internal";
        return "orphan";
    };

    std::vector<std::size_t> notifying_fixed_systems, silent_fixed_systems,
        external_compute_systems, pipeline_systems, sink_systems, internal_systems, orphan_systems;

    for (std::size_t i = 0; i < fixed_count; ++i) {
        if (fixed_rate_systems_[i].policy.notifies) {
            notifying_fixed_systems.push_back(i);
        } else {
            silent_fixed_systems.push_back(i);
        }
    }

    for (std::size_t i = 0; i < compute_count; ++i) {
        const auto* type = classify_compute_by_data_flow(i);
        if (strcmp(type, "external") == 0)
            external_compute_systems.push_back(i);
        else if (strcmp(type, "pipeline") == 0)
            pipeline_systems.push_back(i);
        else if (strcmp(type, "sink") == 0)
            sink_systems.push_back(i);
        else if (strcmp(type, "internal") == 0)
            internal_systems.push_back(i);
        else
            orphan_systems.push_back(i);
    }

    // Helper: print system node
    auto print_system_node = [&](std::size_t idx, bool is_fixed, const char* style) {
        const auto& meta   = is_fixed ? fixed_rate_systems_[idx].system->meta()
                                      : compute_systems_[idx].system->meta();
        const char* prefix = is_fixed ? "E" : "C";
        if (is_fixed) {
            const auto freq = fixed_rate_systems_[idx].policy.frequency_hz;
            printf(
                "        %s%d[\"%s<br/>%dHz\"]:::%s\n", prefix, (int)idx, meta.name.c_str(), freq,
                style);
        } else {
            printf("        %s%d[\"%s\"]:::%s\n", prefix, (int)idx, meta.name.c_str(), style);
        }
    };

    if (!notifying_fixed_systems.empty()) {
        printf("    subgraph NotifyingFixed[Notifying FixedRate Sources]\n");
        for (auto idx : notifying_fixed_systems) {
            print_system_node(idx, true, "fixedStyle");
        }
        printf("    end\n\n");
    }

    if (!silent_fixed_systems.empty()) {
        printf("    subgraph SilentFixed[Silent FixedRate Side Inputs]\n");
        for (auto idx : silent_fixed_systems) {
            print_system_node(idx, true, "fixedStyle");
        }
        printf("    end\n\n");
    }

    if (!external_compute_systems.empty()) {
        printf("    subgraph ComputeSources[External Compute Sources]\n");
        for (auto idx : external_compute_systems) {
            print_system_node(idx, false, "inputStyle");
        }
        printf("    end\n\n");
    }

    if (!pipeline_systems.empty()) {
        printf("    subgraph Pipeline[Pipeline Systems]\n");
        for (auto idx : pipeline_systems) {
            print_system_node(idx, false, "pipelineStyle");
        }
        printf("    end\n\n");
    }

    if (!sink_systems.empty()) {
        printf("    subgraph Sink[Sink Systems]\n");
        for (auto idx : sink_systems) {
            print_system_node(idx, false, "outputStyle");
        }
        printf("    end\n\n");
    }

    if (!internal_systems.empty()) {
        printf("    subgraph Internal[Internal Compute Roots]\n");
        for (auto idx : internal_systems) {
            print_system_node(idx, false, "orphanStyle");
        }
        printf("    end\n\n");
    }

    if (!orphan_systems.empty()) {
        printf("    subgraph Orphan[Orphan Systems]\n");
        for (auto idx : orphan_systems) {
            print_system_node(idx, false, "orphanStyle");
        }
        printf("    end\n\n");
    }

    std::map<std::type_index, std::size_t> resource_indices;
    std::size_t next_resource_idx = 0;
    for (const auto& [type, access] : resource_graph) {
        if (access.readers.empty() && access.mutators.empty()) {
            continue;
        }
        resource_indices.emplace(type, next_resource_idx++);
    }
    if (!resource_indices.empty()) {
        printf("    subgraph Resources[Shared Resources]\n");
        for (const auto& [type, idx] : resource_indices) {
            const auto label_it = resource_labels.find(type);
            if (label_it == resource_labels.end()) {
                continue;
            }
            printf(
                "        R%d[\"%s<br/>resource\"]:::resourceStyle\n", (int)idx,
                label_it->second.c_str());
        }
        printf("    end\n\n");
        printf("    %% Dashed grey edges below indicate shared resource access only\n\n");
    }

    // ========================================================================
    // Draw channel edges
    // ========================================================================

    // Map to store color index for each channel (same channel = same color)
    std::map<ChannelKey, std::size_t> channel_color_map;
    std::size_t next_color_idx = 0;

    // Draw channel edges (pub/sub) - collect edges first
    std::vector<std::pair<std::string, std::string>> styled_edges;
    std::vector<std::string> channel_comments;
    for (const auto& [key, conn] : channel_graph) {
        const auto label_it = channel_labels.find(key);
        if (label_it == channel_labels.end()) {
            continue;
        }
        const auto& channel_label = label_it->second;

        if (conn.writers.size() > 1) {
            channel_comments.push_back(
                "    %% Skipped invalid channel \"" + channel_label + "\": multiple writers");
            continue;
        }
        if (conn.writers.empty()) {
            if (!conn.readers.empty()) {
                channel_comments.push_back(
                    "    %% Skipped invalid channel \"" + channel_label + "\": no writer");
            }
            continue;
        }
        if (conn.readers.empty()) {
            continue;
        }

        const auto& writer        = conn.writers.front();
        const char* writer_prefix = writer.is_fixed_rate ? "E" : "C";

        // Assign color to this channel identity (same key = same color)
        if (!channel_color_map.contains(key)) {
            channel_color_map[key] = next_color_idx % num_colors;
            next_color_idx++;
        }
        const std::size_t color_idx = channel_color_map[key];

        for (const auto& reader : conn.readers) {
            const char* reader_prefix = reader.is_fixed_rate ? "E" : "C";
            char edge_buf[256];
            snprintf(
                edge_buf, sizeof(edge_buf), "    %s%d -->|\"%s\"| %s%d", writer_prefix,
                (int)writer.system_idx, channel_label.c_str(), reader_prefix,
                (int)reader.system_idx);
            styled_edges.emplace_back(
                edge_buf, "stroke:" + std::string(path_colors[color_idx]) + ",stroke-width:2px");
        }
    }

    std::vector<std::string> seen_resource_edge_defs;
    auto has_seen_resource_edge = [&](const std::string& edge_def) {
        return std::find(seen_resource_edge_defs.begin(), seen_resource_edge_defs.end(), edge_def)
            != seen_resource_edge_defs.end();
    };
    auto remember_resource_edge = [&](std::string edge_def) {
        if (has_seen_resource_edge(edge_def)) {
            return;
        }
        seen_resource_edge_defs.push_back(edge_def);
        styled_edges.emplace_back(
            std::move(edge_def), "stroke:#7f8c8d,stroke-width:2px,stroke-dasharray: 5 5");
    };

    for (const auto& [type, access] : resource_graph) {
        const auto resource_it = resource_indices.find(type);
        const auto label_it    = resource_labels.find(type);
        if (resource_it == resource_indices.end() || label_it == resource_labels.end()) {
            continue;
        }

        const auto resource_idx = resource_it->second;
        for (const auto& reader : access.readers) {
            const char* reader_prefix = reader.is_fixed_rate ? "E" : "C";
            char edge_buf[256];
            snprintf(
                edge_buf, sizeof(edge_buf), "    R%d -->|\"res\"| %s%d", (int)resource_idx,
                reader_prefix, (int)reader.system_idx);
            remember_resource_edge(edge_buf);
        }
        for (const auto& mutator : access.mutators) {
            const char* mutator_prefix = mutator.is_fixed_rate ? "E" : "C";
            char read_edge_buf[256];
            snprintf(
                read_edge_buf, sizeof(read_edge_buf), "    R%d -->|\"res_mut\"| %s%d",
                (int)resource_idx, mutator_prefix, (int)mutator.system_idx);
            remember_resource_edge(read_edge_buf);

            char write_edge_buf[256];
            snprintf(
                write_edge_buf, sizeof(write_edge_buf), "    %s%d -->|\"res_mut\"| R%d",
                mutator_prefix, (int)mutator.system_idx, (int)resource_idx);
            remember_resource_edge(write_edge_buf);
        }
    }

    for (const auto& comment : channel_comments) {
        printf("%s\n", comment.c_str());
    }
    if ((!channel_comments.empty() || !resource_indices.empty()) && !styled_edges.empty()) {
        printf("\n");
    }

    for (const auto& [edge_def, _] : styled_edges) {
        printf("%s\n", edge_def.c_str());
    }

    for (std::size_t i = 0; i < styled_edges.size(); ++i) {
        const auto& style = styled_edges[i].second;
        printf("    linkStyle %zu %s\n", i, style.c_str());
    }

    printf("```\n");
}

void Scheduler::print_mermaid_execution_levels() const {
    if (lifecycle_state() == LifecycleState::Configuring) {
        print_mermaid_rebuild_hint(
            "Scheduler graph not built. Call build() to render execution levels.");
        return;
    }

    printf("```mermaid\n");
    printf("flowchart LR\n\n");           // Changed from TB to LR for horizontal layout

    const auto is_external_compute_source = [&](const std::size_t idx) {
        return compute_systems_[idx].system->as_external_compute() != nullptr;
    };

    bool has_notifying_fixed_rate = false;
    bool has_silent_fixed_rate    = false;
    for (const auto& entry : fixed_rate_systems_) {
        has_notifying_fixed_rate |= entry.policy.notifies;
        has_silent_fixed_rate |= !entry.policy.notifies;
    }

    if (has_notifying_fixed_rate) {
        printf("    subgraph Wakers[Notifying FixedRate Sources]\n");
        printf("        direction TB\n"); // Vertical within subgraph for horizontal main flow
        for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
            if (!fixed_rate_systems_[i].policy.notifies) {
                continue;
            }
            const auto& name = fixed_rate_systems_[i].system->meta().name;
            const auto freq  = fixed_rate_systems_[i].policy.frequency_hz;
            printf(
                "        E%d[\"E%d: %s<br/>%dHz\"]:::fixedStyle\n", (int)i, (int)i, name.c_str(),
                freq);
        }
        printf("    end\n\n");
    }

    if (has_silent_fixed_rate) {
        printf("    subgraph SideInputs[Silent FixedRate Side Inputs]\n");
        printf("        direction TB\n");
        for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
            if (fixed_rate_systems_[i].policy.notifies) {
                continue;
            }
            const auto& name = fixed_rate_systems_[i].system->meta().name;
            const auto freq  = fixed_rate_systems_[i].policy.frequency_hz;
            printf(
                "        E%d[\"E%d: %s<br/>%dHz\"]:::fixedStyle\n", (int)i, (int)i, name.c_str(),
                freq);
        }
        printf("    end\n\n");
    }

    if (levels_.empty()) {
        printf("    NoLevels[\"No execution levels computed\"]\n");
        printf("```\n");
        return;
    }

    // Build in_edges and out_edges for each compute system
    std::vector<std::size_t> in_edges(compute_systems_.size(), 0);
    std::vector<std::size_t> out_edges(compute_systems_.size(), 0);

    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        out_edges[i] = (i < compute_affects_.size()) ? std::popcount(compute_affects_[i]) : 0;
    }

    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if (i != j && (compute_affects_[i] & (1UL << j))) {
                in_edges[j]++;
            }
        }
    }

    // Also count edges from fixed_rate systems
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        if (!fixed_rate_systems_[i].policy.notifies) {
            continue;
        }
        auto affects = i < fixed_rate_affects_.size() ? fixed_rate_affects_[i] : 0U;
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if (affects & (1UL << j)) {
                in_edges[j]++;
            }
        }
    }

    // Helper: classify system type based on graph edges
    auto classify_by_edges = [&](std::size_t idx) -> const char* {
        if (is_external_compute_source(idx)) {
            return "external";
        }
        const bool has_out = out_edges[idx] > 0;
        const bool has_in  = in_edges[idx] > 0;

        if (has_out && has_in)
            return "pipeline"; // (1, 1) 有进有出
        if (!has_out)
            return "sink";
        return "internal";
    };

    // Print each level as a subgraph
    for (std::size_t level_idx = 0; level_idx < levels_.size(); ++level_idx) {
        const auto& level = levels_[level_idx];
        printf(
            "    subgraph L%d[\"Level %d: %zu systems\"]\n", (int)level_idx, (int)level_idx,
            level.size());
        printf("        direction TB\n"); // Vertical within subgraph for horizontal main flow

        for (std::size_t i = 0; i < level.size(); ++i) {
            const auto sys_idx   = level[i];
            const auto& name     = compute_systems_[sys_idx].system->meta().name;
            const auto& meta     = compute_systems_[sys_idx].system->meta();
            const auto io_counts = count_channel_io(meta);

            const auto* sys_type    = classify_by_edges(sys_idx);
            const char* style_class = (strcmp(sys_type, "external") == 0) ? "inputStyle"
                                    : (strcmp(sys_type, "sink") == 0)     ? "outputStyle"
                                                                          : "pipelineStyle";
            const char* extra       = (strcmp(sys_type, "external") == 0) ? " [external]" : "";

            printf(
                "        C%d[\"C%d: %s%s\\n(%d in, %d out)\"]:::%s\n", (int)sys_idx, (int)sys_idx,
                name.c_str(), extra, (int)io_counts.inputs, (int)io_counts.outputs, style_class);
        }

        printf("    end\n\n");
    }

    // ========================================================================
    // Draw edges: FixedRate -> compute systems, then between consecutive levels
    // ========================================================================

    // FixedRate wakers -> compute systems (direct wake targets)
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        if (!fixed_rate_systems_[i].policy.notifies) {
            continue;
        }
        auto affects = i < fixed_rate_affects_.size() ? fixed_rate_affects_[i] : 0U;
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if (affects & (1UL << j)) {
                printf("    E%d --> C%d\n", (int)i, (int)j);
            }
        }
    }

    // Draw all direct compute dependencies, including edges that skip intermediate levels.
    for (std::size_t src_idx = 0; src_idx < compute_systems_.size(); ++src_idx) {
        auto src_affects = src_idx < compute_affects_.size() ? compute_affects_[src_idx] : 0U;
        while (src_affects != 0) {
            const auto dst_idx = static_cast<std::size_t>(std::countr_zero(src_affects));
            src_affects &= src_affects - 1;
            if (src_idx != dst_idx) {
                printf("    C%d --> C%d\n", (int)src_idx, (int)dst_idx);
            }
        }
    }

    printf("```\n");
    printf("\n%% Pipeline Analysis:\n");

    // Count system types
    std::size_t external_count = 0, pipeline_count = 0, sink_count = 0, internal_count = 0;
    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        const auto* sys_type = classify_by_edges(i);
        if (strcmp(sys_type, "external") == 0)
            external_count++;
        else if (strcmp(sys_type, "sink") == 0)
            sink_count++;
        else if (strcmp(sys_type, "internal") == 0)
            internal_count++;
        else
            pipeline_count++;
    }

    printf(
        "%% - Notifying fixed_rate sources: %zu\n",
        std::count_if(
            fixed_rate_systems_.begin(), fixed_rate_systems_.end(),
            [](const auto& entry) { return entry.policy.notifies; }));
    printf(
        "%% - Silent fixed_rate side inputs: %zu\n",
        std::count_if(
            fixed_rate_systems_.begin(), fixed_rate_systems_.end(),
            [](const auto& entry) { return !entry.policy.notifies; }));
    printf("%% - Total depth: %zu levels\n", (std::size_t)levels_.size());
    printf(
        "%% - Compute system types: %zu external, %zu pipeline, %zu sink, %zu internal\n",
        external_count, pipeline_count, sink_count, internal_count);
    printf("%% - Max parallelism: ");
    std::size_t max_parallel = 0;
    std::size_t max_level    = 0;
    for (std::size_t i = 0; i < levels_.size(); ++i) {
        if (levels_[i].size() > max_parallel) {
            max_parallel = levels_[i].size();
            max_level    = i;
        }
    }
    printf("%zu systems at Level %zu\n", max_parallel, max_level);
}

void Scheduler::print_stats() const {
    const auto now            = Clock::now();
    const auto elapsed_sec    = std::chrono::duration<double>(now - stats_start_time_).count();
    const auto compute_cycles = compute_cycles_.load(std::memory_order_relaxed);
    const auto compute_total  = compute_total_time_ns_.load(std::memory_order_relaxed);
    const auto compute_last   = compute_last_time_ns_.load(std::memory_order_relaxed);
    const auto notify_total   = sum_notify_counts();

    printf("\n");
    printf("=== Scheduler Performance (%.1fs) ===\n", elapsed_sec);

    // Compute loop stats - more compact
    printf("Loop: %-4" PRIu64 " cycles (%.1f Hz)", compute_cycles, compute_cycles / elapsed_sec);
    if (compute_cycles > 0) {
        printf(
            " | avg: %.2f ms | last: %.2f ms", compute_total / 1e6 / compute_cycles,
            compute_last / 1e6);
    }
    printf(" | notifies: %-4" PRIu64 " (%.1f Hz)\n", notify_total, notify_total / elapsed_sec);

    // Helper lambda: print latency statistics
    auto print_latency_stats = [](const auto& latency) {
        if (latency.sample_count > 0) {
            printf(
                "   min:%.2f p50:%.2f p95:%.2f max:%.2f ms", latency.min_ns / 1e6,
                latency.p50_ns / 1e6, latency.p95_ns / 1e6, latency.max_ns / 1e6);
            printf("\n");
        }
    };

    // Unified stats printer (eliminates duplicate loop structure)
    auto print_system_stats = [&](const auto& systems, const auto& stats, auto print_fn) {
        for (std::size_t i = 0; i < systems.size(); ++i) {
            const auto& entry     = systems[i];
            const auto& sys_stats = stats[i];
            print_fn(entry, sys_stats, elapsed_sec);
        }
    };

    // FixedRate systems - system name on one line, stats on the next
    print_system_stats(
        fixed_rate_systems_, fixed_rate_stats_,
        [&](const auto& entry, const auto& sys_stats, double) {
            const auto notify_cnt = sys_stats.notify_count.load(std::memory_order_relaxed);
            const auto exec_count = sys_stats.execution_count.load(std::memory_order_relaxed);
            const auto record_cnt = sys_stats.record_count.load(std::memory_order_relaxed);
            const auto latency    = sys_stats.latency_hist.compute();
            const auto actual_hz  = record_cnt / elapsed_sec;
            const bool write_mode = counts_written_calls(entry.system->meta());

            printf(
                "  %-30s@%.1fHz(%uHz) %s:%" PRIu64, entry.system->meta().name.c_str(), actual_hz,
                entry.policy.frequency_hz, write_mode ? "writes" : "runs", record_cnt);
            if (write_mode) {
                printf(" exec:%" PRIu64, exec_count);
            }
            if (notify_cnt > 0) {
                printf(" notif:%" PRIu64, notify_cnt);
            }
            printf("\n");

            print_latency_stats(latency);
        });

    // Compute systems - system name on one line, stats on the next
    print_system_stats(
        compute_systems_, compute_stats_, [&](const auto& entry, const auto& sys_stats, double) {
            const auto run_count = sys_stats.run_count.load(std::memory_order_relaxed);
            const auto latency   = sys_stats.latency_hist.compute();
            const auto actual_hz = run_count / elapsed_sec;

            printf(
                "  %-30s@%.1fHz runs:%" PRIu64, entry.system->meta().name.c_str(), actual_hz,
                run_count);
            printf("\n");

            print_latency_stats(latency);
        });
    printf("======================================\n");
    printf("\n");
}

auto Scheduler::get_stats_json() const -> std::string {
    using json = nlohmann::json;

    const auto elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(Clock::now() - stats_start_time_);
    const double elapsed_sec = elapsed.count();

    // Compute loop stats
    const auto cycles       = compute_cycles_.load(std::memory_order_relaxed);
    const auto total_time   = compute_total_time_ns_.load(std::memory_order_relaxed);
    const auto last_time    = compute_last_time_ns_.load(std::memory_order_relaxed);
    const auto notify_total = sum_notify_counts();

    json root;
    root["runtime_seconds"] = elapsed_sec;

    // Compute loop
    json& compute_json             = root["compute_loop"];
    compute_json["cycles"]         = cycles;
    compute_json["frequency_hz"]   = cycles / elapsed_sec;
    compute_json["avg_time_ms"]    = cycles > 0 ? (total_time / 1e6 / cycles) : 0.0;
    compute_json["last_time_ms"]   = last_time / 1e6;
    compute_json["total_notifies"] = notify_total;

    // Helper lambda: add latency statistics to JSON object
    auto add_latency_stats = [](json& sys, const auto& latency, double) {
        sys["sampled_runs"] = latency.sample_count;
        sys["min_ms"]       = latency.min_ns / 1e6;
        sys["p50_ms"]       = latency.p50_ns / 1e6;
        sys["p95_ms"]       = latency.p95_ns / 1e6;
        sys["p99_ms"]       = latency.p99_ns / 1e6;
        sys["p999_ms"]      = latency.p999_ns / 1e6;
        sys["mean_ms"]      = latency.mean_ns / 1e6;
        sys["max_ms"]       = latency.max_ns / 1e6;
        sys["stddev_ms"]    = latency.stddev_ns / 1e6;
    };

    // Unified JSON stats generator (eliminates duplicate loop structure)
    auto generate_system_stats = [&](const auto& systems, const auto& stats, json& arr,
                                     auto add_extra_fn) {
        for (std::size_t i = 0; i < systems.size(); ++i) {
            const auto& entry     = systems[i];
            const auto& sys_stats = stats[i];
            const auto latency    = sys_stats.latency_hist.compute();

            json sys;
            add_latency_stats(sys, latency, elapsed_sec);
            add_extra_fn(sys, entry, sys_stats);
            arr[entry.system->meta().name] = sys;
        }
    };

    // FixedRate systems
    json& fixed_rate_arr = root["fixed_rate_systems"] = json::object();
    generate_system_stats(
        fixed_rate_systems_, fixed_rate_stats_, fixed_rate_arr,
        [elapsed_sec](json& sys, const auto& entry, const auto& stats) {
            const auto notify_cnt = stats.notify_count.load(std::memory_order_relaxed);
            const auto exec_count = stats.execution_count.load(std::memory_order_relaxed);
            const auto record_cnt = stats.record_count.load(std::memory_order_relaxed);
            const bool write_mode = counts_written_calls(entry.system->meta());
            sys["target_hz"]      = entry.policy.frequency_hz;
            sys["notifies"]       = notify_cnt;
            sys["executions"]     = exec_count;
            sys["runs"]           = record_cnt;
            sys["actual_hz"]      = record_cnt / elapsed_sec;
            sys["count_mode"]     = write_mode ? "written_calls" : "run_calls";
        });

    // Compute systems
    json& compute_arr = root["compute_systems"] = json::object();
    generate_system_stats(
        compute_systems_, compute_stats_, compute_arr,
        [elapsed_sec](json& sys, const auto&, const auto& stats) {
            const auto run_count = stats.run_count.load(std::memory_order_relaxed);
            sys["runs"]          = run_count;
            sys["actual_hz"]     = run_count / elapsed_sec;
        });

    return root.dump();
}

// ============================================================================
// FixedRate Thread
// ============================================================================

void Scheduler::run_fixed_rate_thread(FixedRateContext ctx) {
    const auto keep_running = [&ctx] {
        return is_running_state(ctx.lifecycle->load(std::memory_order_acquire));
    };

    // Set CPU affinity
    if (ctx.policy.cpu_affinity >= 0) {
        auto result = primitive::ThreadAffinity::pin_to_core(
            static_cast<std::uint32_t>(ctx.policy.cpu_affinity));
        if (!result) {
            // Log warning but continue
            SPDLOG_WARN("{}: {}", ctx.system->meta().name, result.error());
        }
    }

    // Set thread priority
    if (ctx.policy.thread_priority > 0) {
        primitive::ThreadAffinity::RealtimeConfig rt_config{};
        rt_config.priority = static_cast<std::uint8_t>(ctx.policy.thread_priority);
        if (auto result = primitive::ThreadAffinity::set_realtime_priority(rt_config); !result) {
            SPDLOG_WARN("{}: {}", ctx.system->meta().name, result.error());
        }
    }

    const auto frequency        = ctx.policy.frequency_hz;
    auto& stats                 = ctx.scheduler->fixed_rate_stats_[ctx.system_index];
    const bool write_mode       = counts_written_calls(ctx.system->meta());
    const auto record_execution = [&](const bool written, const std::uint64_t elapsed) {
        stats.execution_count.fetch_add(1, std::memory_order_relaxed);
        if (!write_mode || written) {
            stats.record_count.fetch_add(1, std::memory_order_relaxed);
            stats.latency_hist.record(elapsed);
        }
        if (written && ctx.policy.notifies) [[likely]] {
            ctx.scheduler->notify(ctx.system_index);
        }
    };

    if (frequency == 0) {
        // Unlimited frequency mode: run as fast as possible
        while (keep_running()) {
            primitive::LatencyProbe probe;
            const bool written = ctx.system->run(*ctx.world);
            const auto elapsed = probe.elapsed_ns();

            record_execution(written, elapsed);
        }
        return;
    }

    // Fixed frequency mode
    // Assume frequency > 0 to help optimizer eliminate division-by-zero checks
    [[assume(frequency > 0)]];
    const auto period =
        std::chrono::nanoseconds(1'000'000'000ULL / static_cast<std::uint64_t>(frequency));
    auto next_tick = Clock::now();

    while (keep_running()) [[likely]] {
        next_tick += period;

        // Execute system
        primitive::LatencyProbe probe;
        const bool written = ctx.system->run(*ctx.world);
        const auto elapsed = probe.elapsed_ns();

        record_execution(written, elapsed);

        // Busy wait until next tick
        std::this_thread::sleep_until(next_tick);
    }
}

// ============================================================================
// Compute Loop
// ============================================================================

void Scheduler::run_compute_loop() {
    // Adaptive backoff: spin -> yield -> short sleep
    // This reduces CPU usage during idle while maintaining low latency
    constexpr std::size_t SPIN_LIMIT  = 100;  // Spin iterations before yield
    constexpr std::size_t YIELD_LIMIT = 1000; // Yield iterations before sleep
    constexpr auto SLEEP_DURATION     = std::chrono::microseconds(10); // Short sleep
    std::size_t idle_count            = 0;

    const auto should_print = config_.print_stats;
    auto print_interval     = std::chrono::seconds(5);
    auto last_print         = Clock::now();

    while (is_running()) [[likely]] {
        // Check for pause request (hot-add in progress)
        if (pause_requested_.load(std::memory_order_acquire)) [[unlikely]] {
            // Reset idle counter on resume
            idle_count = 0;

            // Signal that we've paused
            {
                std::lock_guard lock(pause_mutex_);
                paused_.store(true, std::memory_order_release);
            }
            pause_cv_.notify_all();

            // Wait for resume signal
            {
                std::unique_lock lock(pause_mutex_);
                pause_cv_.wait(lock, [this] {
                    return !pause_requested_.load(std::memory_order_acquire) || !is_running();
                });
                paused_.store(false, std::memory_order_release);
            }

            // Check if we should exit after resume
            if (!is_running()) {
                break;
            }
            continue;
        }

        // Atomically exchange ready_systems with 0, getting the set of systems to run
        const auto ready = ready_systems_.exchange(0, std::memory_order_acq_rel);

        if (ready != 0) [[likely]] {
            // Reset idle counter when work is available
            idle_count = 0;

            primitive::LatencyProbe probe;
            const auto result  = run_compute_selective(ready);
            const auto elapsed = probe.elapsed_ns();

            compute_cycles_.fetch_add(1, std::memory_order_relaxed);
            compute_total_time_ns_.fetch_add(elapsed, std::memory_order_relaxed);
            compute_last_time_ns_.store(elapsed, std::memory_order_relaxed);
            if (result.deferred_mask != 0) {
                ready_systems_.fetch_or(result.deferred_mask, std::memory_order_release);
            }

            // Check stats after compute (amortize cost)
            if (should_print) [[unlikely]] {
                const auto now = Clock::now();
                if (now - last_print >= print_interval) [[unlikely]] {
                    print_stats();
                    last_print = now;
                }
            }
        } else {
            // Adaptive backoff strategy:
            // - Spin first (lowest latency, ~10 cycles per SPIN_HINT)
            // - Then yield (medium latency, ~1-10μs)
            // - Finally short sleep (higher latency but saves CPU)
            ++idle_count;

            if (idle_count <= SPIN_LIMIT) {
                // Phase 1: Spin (lowest latency)
                SPIN_HINT();
            } else if (idle_count <= YIELD_LIMIT) {
                // Phase 2: Yield (let other threads run)
                std::this_thread::yield();
            } else {
                // Phase 3: Short sleep (save CPU during extended idle)
                std::this_thread::sleep_for(SLEEP_DURATION);
            }
        }
    }
    // Execute shutdown hooks before lifecycle transition, so systems
    // can perform cleanup (e.g. stop camera streaming) before the
    // scheduler stops executing.
    for (auto& hook : shutdown_hooks_) {
        hook();
    }
}

auto Scheduler::run_compute_selective(const std::uint64_t ready_mask) -> ComputeRoundResult {
    if (compute_systems_.empty()) [[unlikely]] {
        return {};
    }

    std::uint64_t pending_ready = ready_mask;
    std::uint64_t executed_mask = 0;
    std::uint64_t written_mask  = 0;
    std::uint64_t deferred_mask = 0;

    auto enqueue_ready = [&](const std::uint64_t new_bits) {
        deferred_mask |= new_bits & executed_mask;
        pending_ready |= new_bits & ~executed_mask;
    };

    enqueue_ready(ready_systems_.exchange(0, std::memory_order_acq_rel));

    while (pending_ready != 0) {
        bool ran_any = false;

        compute_arena_->execute([&] {
            std::size_t to_run_buf[64];

            for (const auto& level : levels_) {
                enqueue_ready(ready_systems_.exchange(0, std::memory_order_acq_rel));

                std::size_t to_run_count = 0;
                std::uint64_t level_mask = 0;
                for (const auto idx : level) {
                    const std::uint64_t bit = 1ULL << idx;
                    if ((pending_ready & bit) != 0) {
                        to_run_buf[to_run_count++] = idx;
                        level_mask |= bit;
                    }
                }

                if (to_run_count == 0) [[unlikely]] {
                    continue;
                }

                ran_any = true;
                pending_ready &= ~level_mask;
                executed_mask |= level_mask;

                std::atomic<std::uint64_t> level_written{0};

                if (to_run_count == 1) {
                    const auto idx = to_run_buf[0];
                    primitive::LatencyProbe probe;
                    const bool written = compute_systems_[idx].system->run(world_);
                    const auto elapsed = probe.elapsed_ns();

                    compute_stats_[idx].run_count.fetch_add(1, std::memory_order_relaxed);
                    compute_stats_[idx].latency_hist.record(elapsed);
                    if (written) [[likely]] {
                        level_written.store(1ULL << idx, std::memory_order_relaxed);
                    }
                } else {
                    tbb::task_group tg;
                    for (std::size_t i = 0; i < to_run_count; ++i) {
                        const auto idx = to_run_buf[i];
                        tg.run([this, idx, &level_written] {
                            primitive::LatencyProbe probe;
                            const bool written = compute_systems_[idx].system->run(world_);
                            const auto elapsed = probe.elapsed_ns();

                            compute_stats_[idx].run_count.fetch_add(1, std::memory_order_relaxed);
                            compute_stats_[idx].latency_hist.record(elapsed);
                            if (written) [[likely]] {
                                level_written.fetch_or(1ULL << idx, std::memory_order_relaxed);
                            }
                        });
                    }
                    tg.wait();
                }

                const auto level_written_mask = level_written.load(std::memory_order_relaxed);
                written_mask |= level_written_mask;

                std::uint64_t cascade = 0;
                auto writers          = level_written_mask;
                while (writers != 0) {
                    const auto writer_idx = static_cast<std::size_t>(std::countr_zero(writers));
                    writers &= writers - 1;
                    cascade |= compute_affects_[writer_idx];
                }

                enqueue_ready(cascade);
            }
        });

        enqueue_ready(ready_systems_.exchange(0, std::memory_order_acq_rel));
        if (!ran_any) [[unlikely]] {
            break;
        }
    }

    return ComputeRoundResult{
        .written_mask  = written_mask,
        .deferred_mask = deferred_mask,
    };
}

// ============================================================================
// Dependency Graph
// ============================================================================

auto Scheduler::build_topology_snapshot() const -> std::expected<TopologySnapshot, BuildError> {
    const auto n = compute_systems_.size();
    TopologySnapshot snapshot;

    // Enforce max 64 compute systems for bitmask scheduling
    if (n > TooManyComputeSystemsError::max_count) {
        return std::unexpected(TooManyComputeSystemsError{.count = n});
    }

    // ========================================================================
    // Phase 1: Collect channel usage information
    // ========================================================================

    // Helper: check if channel kind is a writer
    const auto is_writer = [](channel_kind k) {
        return k == channel_kind::spsc_writer || k == channel_kind::spmc_writer;
    };

    // Helper: check if channel kind is SPSC
    const auto is_spsc = [](channel_kind k) {
        return k == channel_kind::spsc_reader || k == channel_kind::spsc_writer;
    };

    struct ChannelEndpoint {
        std::size_t index;
        bool is_fixed_rate;
    };

    struct ChannelUsage {
        channel_kind kind = channel_kind::local; ///< Will be set on first use
        std::vector<ChannelEndpoint> writers;
        std::vector<ChannelEndpoint> readers;
        std::string first_system;                ///< For kind conflict reporting
    };

    std::map<ChannelKey, ChannelUsage> channels;

    auto endpoint_name = [&](const ChannelEndpoint endpoint) -> std::string {
        const auto& name = endpoint.is_fixed_rate
                             ? fixed_rate_systems_[endpoint.index].system->meta().name
                             : compute_systems_[endpoint.index].system->meta().name;
        if (endpoint.is_fixed_rate) {
            return fmt::format("{} (fixed_rate)", name);
        }
        return name;
    };

    auto process_system_channels = [&](const auto& systems,
                                       const bool is_fixed_rate) -> BuildResult {
        for (std::size_t i = 0; i < systems.size(); ++i) {
            const auto& meta = systems[i].system->meta();

            auto process_channel = [&](const ChannelMeta& ch,
                                       const bool expect_spsc) -> BuildResult {
                ChannelKey key{ch.type, ch.topic};
                auto& usage              = channels[key];
                const bool has_endpoints = !usage.writers.empty() || !usage.readers.empty();

                if (has_endpoints) {
                    const auto is_expected_kind =
                        expect_spsc ? is_spsc(usage.kind) : !is_spsc(usage.kind);
                    if (!is_expected_kind) {
                        return std::unexpected(
                            ChannelKindConflict{
                                .key           = ChannelKeyInfo(ch.type, ch.topic, expect_spsc),
                                .first_system  = usage.first_system,
                                .second_system = meta.name,
                            });
                    }
                } else {
                    usage.kind         = ch.kind;
                    usage.first_system = meta.name;
                }

                const ChannelEndpoint endpoint{
                    .index         = i,
                    .is_fixed_rate = is_fixed_rate,
                };
                if (is_writer(ch.kind)) {
                    usage.writers.push_back(endpoint);
                } else {
                    usage.readers.push_back(endpoint);
                }
                return {};
            };

            for (const auto& ch : meta.spsc_channels) {
                if (auto result = process_channel(ch, true); !result) {
                    return result;
                }
            }
            for (const auto& ch : meta.spmc_channels) {
                if (auto result = process_channel(ch, false); !result) {
                    return result;
                }
            }
        }
        return {};
    };

    if (auto result = process_system_channels(fixed_rate_systems_, true); !result) {
        return std::unexpected(result.error());
    }
    if (auto result = process_system_channels(compute_systems_, false); !result) {
        return std::unexpected(result.error());
    }

    // ========================================================================
    // Phase 2: Validate channel constraints
    // ========================================================================

    // Helper: collect reader names from indices
    auto collect_endpoint_names = [&](const std::vector<ChannelEndpoint>& endpoints) {
        std::vector<std::string> names;
        names.reserve(endpoints.size());
        for (const auto endpoint : endpoints) {
            names.push_back(endpoint_name(endpoint));
        }
        return names;
    };

    for (const auto& [key, usage] : channels) {
        // Multiple writers is always an error
        if (usage.writers.size() > 1) {
            return std::unexpected(
                MultipleWritersError{
                    .key     = ChannelKeyInfo(key.type, key.topic, is_spsc(usage.kind)),
                    .writers = collect_endpoint_names(usage.writers),
                });
        }

        // SPSC with multiple readers is an error
        if (is_spsc(usage.kind) && usage.readers.size() > 1) {
            return std::unexpected(
                MultipleReadersError{
                    .key     = ChannelKeyInfo(key.type, key.topic, true),
                    .readers = collect_endpoint_names(usage.readers),
                });
        }

        // Reader without writer is an error (neither fixed_rate nor compute writer)
        if (usage.writers.empty() && !usage.readers.empty()) {
            return std::unexpected(
                OrphanedReaderError{
                    .key     = ChannelKeyInfo(key.type, key.topic, is_spsc(usage.kind)),
                    .readers = collect_endpoint_names(usage.readers),
                });
        }

        // Orphaned compute writer is a warning (printed but not an error). Fixed-rate writers with
        // no readers are fine because they may still act as periodic sources.
        if (!usage.writers.empty() && usage.readers.empty() && !usage.writers[0].is_fixed_rate) {
            SPDLOG_WARN(
                "[WARN] Channel has writer '{}' but no readers\n",
                compute_systems_[usage.writers[0].index].system->meta().name.c_str());
        }
    }

    // ========================================================================
    // Phase 3: Build adjacency list and in-degree (channels only)
    // ========================================================================

    // Use bitmask for adjacency instead of std::set (O(1) vs O(log n))
    // adj_mask[i] = bitmask of nodes that node i points to
    std::vector<std::uint64_t> adj_mask(n, 0);
    std::vector<std::size_t> in_degree(n, 0);

    for (const auto& [key, usage] : channels) {
        if (usage.writers.empty() || usage.readers.empty()) {
            continue;
        }

        const auto writer = usage.writers[0];
        if (writer.is_fixed_rate) {
            continue;
        }

        for (const auto reader : usage.readers) {
            if (!reader.is_fixed_rate && writer.index != reader.index) {
                const std::uint64_t bit = 1UL << reader.index;
                if ((adj_mask[writer.index] & bit) == 0) {
                    // New edge
                    adj_mask[writer.index] |= bit;
                    ++in_degree[reader.index];
                }
            }
        }
    }

    // Compute systems must be reachable from an external source:
    // - a notifying fixed_rate writer feeding compute readers
    // - a compute system that can be woken externally (for example, RclPubSystem)
    std::uint64_t external_source_mask = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (compute_systems_[i].system->as_external_compute() != nullptr) {
            external_source_mask |= (1ULL << i);
        }
    }
    for (const auto& [key, usage] : channels) {
        if (usage.writers.empty() || usage.readers.empty()) {
            continue;
        }

        const auto writer = usage.writers[0];
        if (!writer.is_fixed_rate || !fixed_rate_systems_[writer.index].policy.notifies) {
            continue;
        }

        for (const auto reader : usage.readers) {
            if (!reader.is_fixed_rate) {
                external_source_mask |= (1ULL << reader.index);
            }
        }
    }

    // ========================================================================
    // Phase 4: Kahn's algorithm for topological sort (by levels)
    // ========================================================================

    snapshot.levels.clear();
    std::vector<std::size_t> current_level;

    // Start with nodes that have no dependencies
    for (std::size_t i = 0; i < n; ++i) {
        if (in_degree[i] == 0) {
            current_level.push_back(i);
        }
    }

    std::size_t processed = 0;
    while (!current_level.empty()) {
        snapshot.levels.push_back(current_level);
        processed += current_level.size();

        std::vector<std::size_t> next_level;
        next_level.reserve(compute_systems_.size()); // Pre-allocate max possible
        for (std::size_t node : current_level) {
            std::uint64_t neighbors = adj_mask[node];
            while (neighbors) {
                const std::size_t neighbor = std::countr_zero(neighbors);
                neighbors &= neighbors - 1;          // Clear lowest set bit
                if (--in_degree[neighbor] == 0) {
                    next_level.push_back(neighbor);
                }
            }
        }
        current_level = std::move(next_level);
    }

    // ========================================================================
    // Phase 5: Cycle detection
    // ========================================================================

    if (processed < n) {
        // There's a cycle - find it using DFS
        std::vector<std::string> cycle;

        // Simple cycle detection: follow edges until we revisit a node
        std::vector visited(n, false);
        std::vector in_stack(n, false);
        std::vector<std::size_t> path;

        std::function<bool(std::size_t)> find_cycle = [&](const std::size_t node) -> bool {
            visited[node]  = true;
            in_stack[node] = true;
            path.push_back(node);

            std::uint64_t neighbors = adj_mask[node];
            while (neighbors) {
                const std::size_t neighbor = std::countr_zero(neighbors);
                neighbors &= neighbors - 1; // Clear lowest set bit

                if (!visited[neighbor]) {
                    if (find_cycle(neighbor)) {
                        return true;
                    }
                } else if (in_stack[neighbor]) {
                    // Found cycle - extract it
                    auto it = std::find(path.begin(), path.end(), neighbor);
                    for (; it != path.end(); ++it) {
                        cycle.push_back(compute_systems_[*it].system->meta().name);
                    }
                    cycle.push_back(compute_systems_[neighbor].system->meta().name);
                    return true;
                }
            }

            path.pop_back();
            in_stack[node] = false;
            return false;
        };

        // Start DFS from nodes with remaining in-degree
        for (std::size_t i = 0; i < n && cycle.empty(); ++i) {
            if (in_degree[i] > 0 && !visited[i]) {
                find_cycle(i);
            }
        }

        return std::unexpected(DependencyCycleError{.cycle = std::move(cycle)});
    }

    // ========================================================================
    // Phase 6: Liveness validation
    // ========================================================================

    const auto all_compute_mask =
        n == 64 ? std::numeric_limits<std::uint64_t>::max() : ((1ULL << n) - 1ULL);
    std::uint64_t reachable = external_source_mask;
    std::uint64_t frontier  = external_source_mask;

    while (frontier != 0) {
        std::uint64_t next = 0;
        auto sources       = frontier;
        while (sources != 0) {
            const auto idx = static_cast<std::size_t>(std::countr_zero(sources));
            sources &= sources - 1;
            next |= adj_mask[idx];
        }
        frontier = next & ~reachable;
        reachable |= frontier;
    }

    if ((reachable & all_compute_mask) != all_compute_mask) {
        std::vector<std::string> unreachable;
        for (std::size_t i = 0; i < n; ++i) {
            const auto bit = 1ULL << i;
            if ((reachable & bit) == 0) {
                unreachable.push_back(compute_systems_[i].system->meta().name);
            }
        }
        return std::unexpected(UnreachableComputeSystemsError{.systems = std::move(unreachable)});
    }

    // ========================================================================
    // Phase 7: Build wake chains (waker -> direct wakee only)
    // ========================================================================
    // Wake chains connect systems that directly communicate via channels.
    // Transitive propagation is handled by compute-round closure.

    // Wake chains for fixed_rate systems:
    // fixed_rate_affects_[i] = bitmask of compute systems that directly read this fixed_rate's
    // channels
    snapshot.fixed_rate_affects.assign(fixed_rate_systems_.size(), 0);

    // Helper lambda: process writer channels and collect direct readers only
    auto collect_direct_readers = [&](const auto& meta_channels, std::uint64_t& affects) {
        for (const auto& ch : meta_channels) {
            if (is_writer(ch.kind)) {
                ChannelKey key{ch.type, ch.topic};
                if (auto it = channels.find(key); it != channels.end()) {
                    for (const auto reader : it->second.readers) {
                        if (!reader.is_fixed_rate) {
                            affects |= (1UL << reader.index); // Direct compute reader only
                        }
                    }
                }
            }
        }
    };

    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        std::uint64_t affects = 0;
        const auto& meta      = fixed_rate_systems_[i].system->meta();

        // Collect direct readers of this fixed_rate's channels
        collect_direct_readers(meta.spsc_channels, affects);
        collect_direct_readers(meta.spmc_channels, affects);

        snapshot.fixed_rate_affects[i] = affects;
    }

    // Wake chains for compute systems:
    // compute_affects_[i] = bitmask of compute systems that directly depend on system i
    // via channels
    snapshot.compute_affects.assign(n, 0);

    // Channel dependencies
    for (const auto& [key, usage] : channels) {
        if (usage.writers.empty() || usage.readers.empty()) {
            continue;
        }

        const auto writer = usage.writers[0];
        if (writer.is_fixed_rate) {
            continue;
        }

        for (const auto reader : usage.readers) {
            if (!reader.is_fixed_rate && writer.index != reader.index) {
                snapshot.compute_affects[writer.index] |= (1UL << reader.index);
            }
        }
    }

    return snapshot;
}

} // namespace talos::scheduler
