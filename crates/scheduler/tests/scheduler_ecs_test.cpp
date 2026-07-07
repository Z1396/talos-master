#include "scheduler/error_formatter.hpp"
#include "scheduler/scheduler.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fmt/core.h>
#include <fmt/std.h>
#include <thread>

// ============================================================================
// Test Types
// ============================================================================

struct ImuData {
    int seq       = 0;
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 9.8f;
};

struct CameraFrame {
    int frame_id = 0;
    int width    = 1920;
    int height   = 1080;
};

struct FusionResult {
    int imu_seq   = 0;
    int frame_id  = 0;
    float state_x = 0.0f;
};

// Tags for channels
struct ImuChannel {};
struct CameraChannel {};
struct FusionChannel {};
struct CustomFusionOut {};

// ============================================================================
// Test: Scheduler with Fixed rate and Pool Systems
// ============================================================================
void test_scheduler_fixed_rate_and_pool() {
    printf("=== Test: Fixed rate + Pool Systems ===\n");

    talos::Scheduler scheduler({});

    std::atomic imu_count{0};
    std::atomic camera_count{0};
    std::atomic fusion_count{0};
    // IMU: 100Hz, silent update (no notify)
    scheduler.add_system<talos::fixed_rate_silent<100>>(
        "imu_reader", [&imu_count](talos::spmc_mut<ImuData, ImuChannel> imu_out) {
            const int seq = imu_count.fetch_add(1, std::memory_order_relaxed);
            imu_out.write(ImuData{.seq = seq, .accel_x = 0.1f * seq});
        });

    // Camera: 30Hz, notify (triggers compute)
    scheduler.add_system<talos::fixed_rate<30>>(
        "camera_reader", [&camera_count](talos::spmc_mut<CameraFrame, CameraChannel> cam_out) {
            const int frame = camera_count.fetch_add(1, std::memory_order_relaxed);
            cam_out.write(CameraFrame{.frame_id = frame});
        });

    // Fusion: pool_compute, triggered by camera notify
    scheduler.add_system<talos::pool_compute>(
        "fusion",
        [&fusion_count](
            talos::spmc<ImuData, ImuChannel> imu_in, talos::spmc<CameraFrame, CameraChannel> cam_in,
            talos::spsc_mut<FusionResult, FusionChannel> fusion_out) {
            // Read latest values
            if (const auto imu = imu_in.read()) {
                if (const auto cam = cam_in.read()) {
                    fusion_count.fetch_add(1, std::memory_order_relaxed);
                    fusion_out.write(
                        FusionResult{
                            .imu_seq  = imu->seq,
                            .frame_id = cam->frame_id,
                            .state_x  = imu->accel_x,
                        });
                }
            }
        });

    scheduler.print_systems();

    // Build
    auto build_result = scheduler.build();
    if (!build_result) {
        printf("  ERROR: Build failed: %s\n", fmt::format("{}", build_result.error()).c_str());
        return;
    }

    // Run in background thread
    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });

    // Let it run for 200ms
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Stop
    scheduler.stop();
    scheduler_thread.join();

    // Check results
    auto stats       = scheduler.stats();
    int final_imu    = imu_count.load();
    int final_camera = camera_count.load();
    int final_fusion = fusion_count.load();

    printf("  Results after 200ms:\n");
    printf("    IMU reads:      %d (expected ~20 at 100Hz)\n", final_imu);
    printf("    Camera reads:   %d (expected ~6 at 30Hz)\n", final_camera);
    printf("    Fusion runs:    %d\n", final_fusion);
    printf("    Notify count:   %lu\n", stats.notify_count);
    printf("    Compute cycles: %lu\n", stats.compute_cycle_count);

    // Verify
    bool imu_ok    = final_imu >= 10;                              // At least 10 IMU reads
    bool camera_ok = final_camera >= 3;                            // At least 3 camera reads
    bool fusion_ok = final_fusion >= 1;                            // At least 1 fusion
    bool notify_ok =
        stats.notify_count == static_cast<uint64_t>(final_camera); // Only camera notifies

    if (imu_ok && camera_ok && fusion_ok && notify_ok) {
        printf("  Fixed rate + Pool test passed!\n\n");
    } else {
        printf("  ERROR: Test failed\n");
        printf(
            "    imu_ok=%d camera_ok=%d fusion_ok=%d notify_ok=%d\n", imu_ok, camera_ok, fusion_ok,
            notify_ok);
    }
}

// ============================================================================
// Test: Silent vs Notify
// ============================================================================

void test_silent_vs_notify() {
    printf("=== Test: Silent vs Notify ===\n");

    struct NotifyTag {};

    talos::Scheduler scheduler({});

    std::atomic silent_count{0};
    std::atomic notify_count{0};
    std::atomic sink_count{0};

    // Silent system at 200Hz - should NOT trigger compute
    scheduler.add_system<talos::fixed_rate_silent<200>>("silent_source", [&silent_count]() {
        silent_count.fetch_add(1, std::memory_order_relaxed);
    });

    // Notify system at 50Hz - SHOULD trigger compute when it writes
    scheduler.add_system<talos::fixed_rate<50>>(
        "notify_source", [&notify_count](talos::spmc_mut<int, NotifyTag> out) {
            const auto count = notify_count.fetch_add(1, std::memory_order_relaxed) + 1;
            out.write(count);
        });

    scheduler.add_system<talos::pool_compute>(
        "notify_sink", [&sink_count](talos::spmc<int, NotifyTag> in) {
            if (in.read()) {
                sink_count.fetch_add(1, std::memory_order_relaxed);
            }
        });

    (void)scheduler.build();

    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    scheduler.stop();
    scheduler_thread.join();

    const auto stats = scheduler.stats();

    printf("  Silent calls:   %d (200Hz, ~30 expected)\n", silent_count.load());
    printf("  Notify calls:   %d (50Hz, ~7 expected)\n", notify_count.load());
    printf("  Actual notifies: %lu (should match notify calls)\n", stats.notify_count);



    // Only the notify_source should have triggered notifies
    const bool notify_match = stats.notify_count == static_cast<uint64_t>(notify_count.load());

    if (notify_match && silent_count.load() > notify_count.load() && sink_count.load() > 0) {
        printf("  Silent vs Notify test passed!\n\n");
    } else {
        printf("  ERROR: Test failed\n");
    }
}

// ============================================================================
// Test: Multiple Writers Error
// ============================================================================

void test_multiple_writers_error() {
    printf("=== Test: Multiple Writers Error ===\n");

    struct SharedChannel {};

    talos::Scheduler scheduler({});

    // Two systems writing to the same channel - should fail
    scheduler.add_system<talos::pool_compute>(
        "writer1", [](talos::spsc_mut<int, SharedChannel> out) { out.write(1); });

    scheduler.add_system<talos::pool_compute>(
        "writer2", [](talos::spsc_mut<int, SharedChannel> out) { out.write(2); });

    auto result = scheduler.build();
    if (result) {
        printf("  ERROR: Build should have failed with MultipleWritersError\n");
        return;
    }

    // Check error type
    if (!std::holds_alternative<talos::MultipleWritersError>(result.error())) {
        printf("  ERROR: Wrong error type: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    auto& err = std::get<talos::MultipleWritersError>(result.error());
    printf("  Got expected error: %s\n", fmt::format("{}", result.error()).c_str());
    printf("  Writers: ");
    for (const auto& w : err.writers) {
        printf("'%s' ", w.c_str());
    }
    printf("\n  Multiple Writers Error test passed!\n\n");
}

// ============================================================================
// Test: Multiple Readers Error (SPSC only)
// ============================================================================

void test_multiple_readers_error() {
    printf("=== Test: Multiple Readers Error (SPSC) ===\n");

    struct SpscChannel {};

    talos::Scheduler scheduler({});

    // One writer
    scheduler.add_system<talos::pool_compute>(
        "writer", [](talos::spsc_mut<int, SpscChannel> out) { out.write(42); });

    // Two readers on SPSC - should fail
    scheduler.add_system<talos::pool_compute>(
        "reader1", [](talos::spsc<int, SpscChannel> in) { (void)in.read(); });

    scheduler.add_system<talos::pool_compute>(
        "reader2", [](talos::spsc<int, SpscChannel> in) { (void)in.read(); });

    auto result = scheduler.build();
    if (result) {
        printf("  ERROR: Build should have failed with MultipleReadersError\n");
        return;
    }

    if (!std::holds_alternative<talos::MultipleReadersError>(result.error())) {
        printf("  ERROR: Wrong error type: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    auto& err = std::get<talos::MultipleReadersError>(result.error());
    printf("  Got expected error: %s\n", fmt::format("{}", result.error()).c_str());
    printf("  Readers: ");
    for (const auto& r : err.readers) {
        printf("'%s' ", r.c_str());
    }
    printf("\n  Multiple Readers Error test passed!\n\n");
}

// ============================================================================
// Test: SPMC Multiple Readers (should be OK)
// ============================================================================

void test_spmc_multiple_readers_ok() {
    printf("=== Test: SPMC Multiple Readers (OK) ===\n");

    struct BroadcastChannel {};

    talos::Scheduler scheduler({});

    // One external source feeding multiple compute readers
    scheduler.add_system<talos::fixed_rate<30>>(
        "broadcaster", [](talos::spmc_mut<int, BroadcastChannel> out) { out.write(42); });

    // Multiple readers on SPMC - should be OK
    scheduler.add_system<talos::pool_compute>(
        "subscriber1", [](talos::spmc<int, BroadcastChannel> in) { (void)in.read(); });

    scheduler.add_system<talos::pool_compute>(
        "subscriber2", [](talos::spmc<int, BroadcastChannel> in) { (void)in.read(); });

    scheduler.add_system<talos::pool_compute>(
        "subscriber3", [](talos::spmc<int, BroadcastChannel> in) { (void)in.read(); });

    auto result = scheduler.build();
    if (!result) {
        printf("  ERROR: Build failed: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    printf("  SPMC with 3 readers built successfully!\n");
    printf("  SPMC Multiple Readers test passed!\n\n");
}

// ============================================================================
// Test: Orphaned Reader Error
// ============================================================================

void test_orphaned_reader_error() {
    printf("=== Test: Orphaned Reader Error ===\n");

    struct OrphanChannel {};

    talos::Scheduler scheduler({});

    // Reader without writer - should fail
    scheduler.add_system<talos::pool_compute>(
        "orphan_reader", [](talos::spsc<int, OrphanChannel> in) { (void)in.read(); });

    auto result = scheduler.build();
    if (result) {
        printf("  ERROR: Build should have failed with OrphanedReaderError\n");
        return;
    }

    if (!std::holds_alternative<talos::OrphanedReaderError>(result.error())) {
        printf("  ERROR: Wrong error type: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    printf("  Got expected error: %s\n", fmt::format("{}", result.error()).c_str());
    printf("  Orphaned Reader Error test passed!\n\n");
}

// ============================================================================
// Test: Channel Kind Conflict (SPSC vs SPMC)
// ============================================================================

void test_channel_kind_conflict() {
    printf("=== Test: Channel Kind Conflict ===\n");

    struct ConflictChannel {};

    talos::Scheduler scheduler({});

    // Write as SPSC
    scheduler.add_system<talos::pool_compute>(
        "spsc_writer", [](talos::spsc_mut<int, ConflictChannel> out) { out.write(1); });

    // Read as SPMC - conflict!
    scheduler.add_system<talos::pool_compute>(
        "spmc_reader", [](talos::spmc<int, ConflictChannel> in) { (void)in.read(); });

    auto result = scheduler.build();
    if (result) {
        printf("  ERROR: Build should have failed with ChannelKindConflict\n");
        return;
    }

    if (!std::holds_alternative<talos::ChannelKindConflict>(result.error())) {
        printf("  ERROR: Wrong error type: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    printf("  Got expected error: %s\n", fmt::format("{}", result.error()).c_str());
    printf("  Channel Kind Conflict test passed!\n\n");
}

// ============================================================================
// Test: Dependency Cycle Error
// ============================================================================

void test_dependency_cycle_error() {
    printf("=== Test: Dependency Cycle Error ===\n");

    struct ChannelAB {};
    struct ChannelBC {};
    struct ChannelCA {};

    talos::Scheduler scheduler({});

    // A -> B -> C -> A (cycle)
    scheduler.add_system<talos::pool_compute>(
        "system_a", [](talos::spsc<int, ChannelCA> in, talos::spsc_mut<int, ChannelAB> out) {
            if (const auto v = in.read())
                out.write(*v + 1);
        });

    scheduler.add_system<talos::pool_compute>(
        "system_b", [](talos::spsc<int, ChannelAB> in, talos::spsc_mut<int, ChannelBC> out) {
            if (const auto v = in.read())
                out.write(*v + 1);
        });

    scheduler.add_system<talos::pool_compute>(
        "system_c", [](talos::spsc<int, ChannelBC> in, talos::spsc_mut<int, ChannelCA> out) {
            if (const auto v = in.read())
                out.write(*v + 1);
        });

    auto result = scheduler.build();
    if (result) {
        printf("  ERROR: Build should have failed with DependencyCycleError\n");
        return;
    }

    if (!std::holds_alternative<talos::DependencyCycleError>(result.error())) {
        printf("  ERROR: Wrong error type: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    auto& err = std::get<talos::DependencyCycleError>(result.error());
    printf("  Got expected error: %s\n", fmt::format("{}", result.error()).c_str());
    printf("  Cycle involves %zu systems\n", err.cycle.size());
    printf("  Dependency Cycle Error test passed!\n\n");
}

// ============================================================================
// Test: Unreachable Compute Error
// ============================================================================

void test_unreachable_compute_error() {
    printf("=== Test: Unreachable Compute Error ===\n");

    talos::Scheduler scheduler({});

    scheduler.add_system<talos::pool_compute>("isolated_compute", []() {});

    auto result = scheduler.build();
    if (result) {
        printf("  ERROR: Build should have failed with UnreachableComputeSystemsError\n");
        return;
    }

    if (!std::holds_alternative<talos::UnreachableComputeSystemsError>(result.error())) {
        printf("  ERROR: Wrong error type: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    printf("  Got expected error: %s\n", fmt::format("{}", result.error()).c_str());
    printf("  Unreachable Compute Error test passed!\n\n");
}

// ============================================================================
// Test: Too Many Compute Systems Error
// ============================================================================

void test_too_many_compute_systems_error() {
    printf("=== Test: Too Many Compute Systems Error ===\n");

    talos::Scheduler scheduler({});

    for (std::size_t i = 0; i <= talos::TooManyComputeSystemsError::max_count; ++i) {
        scheduler.add_system<talos::pool_compute>(fmt::format("compute_{}", i), []() {});
    }

    auto result = scheduler.build();
    if (result) {
        printf("  ERROR: Build should have failed with TooManyComputeSystemsError\n");
        return;
    }

    if (!std::holds_alternative<talos::TooManyComputeSystemsError>(result.error())) {
        printf("  ERROR: Wrong error type: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    auto& err = std::get<talos::TooManyComputeSystemsError>(result.error());
    printf("  Got expected error: %s\n", fmt::format("{}", result.error()).c_str());
    printf("  Count: %zu\n", err.count);
    printf("  Too Many Compute Systems Error test passed!\n\n");
}

// ============================================================================
// Test: Valid Dependency Chain (no cycle)
// ============================================================================

void test_valid_dependency_chain() {
    printf("=== Test: Valid Dependency Chain ===\n");

    struct SourceOut {};
    struct Stopice1Out {};
    struct Stopice2Out {};
    struct Stopice3Out {};

    talos::Scheduler scheduler({});

    // Linear reactive pipeline: source -> stopice1 -> stopice2 -> stopice3
    scheduler.add_system<talos::fixed_rate<30>>(
        "source", [](talos::spsc_mut<int, SourceOut> out) { out.write(1); });

    scheduler.add_system<talos::pool_compute>(
        "stopice1", [](talos::spsc<int, SourceOut> in, talos::spsc_mut<int, Stopice1Out> out) {
            if (const auto v = in.read())
                out.write(*v);
        });

    scheduler.add_system<talos::pool_compute>(
        "stopice2", [](talos::spsc<int, Stopice1Out> in, talos::spsc_mut<int, Stopice2Out> out) {
            if (const auto v = in.read())
                out.write(*v + 1);
        });

    scheduler.add_system<talos::pool_compute>(
        "stopice3", [](talos::spsc<int, Stopice2Out> in, talos::spsc_mut<int, Stopice3Out> out) {
            if (const auto v = in.read())
                out.write(*v + 1);
        });

    auto result = scheduler.build();
    if (!result) {
        printf("  ERROR: Build failed: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    scheduler.print_systems();
    printf("  Valid Dependency Chain test passed!\n\n");
}

// ============================================================================
// Test: Failed Safe Hot-Add Leaves Scheduler Configuring
// ============================================================================

void test_failed_safe_hot_add_leaves_scheduler_configuring() {
    printf("=== Test: Failed Safe Hot-Add Leaves Scheduler Configuring ===\n");

    struct SafeHotAddChannel {};

    talos::Scheduler scheduler({});

    scheduler.add_system<talos::fixed_rate<30>>(
        "source", [](talos::spmc_mut<int, SafeHotAddChannel> out) { out.write(1); });
    scheduler.add_system<talos::pool_compute>(
        "sink", [](talos::spmc<int, SafeHotAddChannel> in) { (void)in.read(); });

    auto initial_build = scheduler.build();
    if (!initial_build) {
        printf(
            "  ERROR: Initial build failed: %s\n",
            fmt::format("{}", initial_build.error()).c_str());
        return;
    }

    auto duplicate_writer = [](talos::spmc_mut<int, SafeHotAddChannel> out) { out.write(2); };
    auto result           = scheduler.hot_add_system(
        talos::make_system<decltype(duplicate_writer), talos::pool_compute>(
            "duplicate_writer", std::move(duplicate_writer)));

    if (result) {
        printf("  ERROR: hot_add_system should have failed with MultipleWritersError\n");
        return;
    }

    if (!std::holds_alternative<talos::MultipleWritersError>(result.error())) {
        printf("  ERROR: Wrong error type: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    if (scheduler.lifecycle_state() != talos::Scheduler::LifecycleState::Configuring) {
        printf(
            "  ERROR: Scheduler should have remained in Configuring state after failed hot-add\n");
        return;
    }

    auto run_result = scheduler.run();
    if (run_result || run_result.error() != talos::SchedulerError::NotBuilt) {
        printf("  ERROR: run() should have failed with NotBuilt after failed hot-add\n");
        return;
    }

    printf("  Failed Safe Hot-Add Leaves Scheduler Configuring test passed!\n\n");
}

// ============================================================================
// Test: Repeated Build Binds Custom System Only Once
// ============================================================================

class BindCountingFixedRateSystem final : public talos::SystemBase {
    talos::SystemMeta meta_;
    std::atomic<int>* bind_count_;

public:
    explicit BindCountingFixedRateSystem(std::atomic<int>& bind_count) noexcept
        : bind_count_(&bind_count) {
        meta_.name   = "bind_counting_fixed_rate";
        meta_.policy = talos::make_policy_info<talos::fixed_rate_silent<1>>();
    }

    void bind(talos::World&) noexcept override {
        bind_count_->fetch_add(1, std::memory_order_relaxed);
    }

    bool run(talos::World&) noexcept override { return false; }

    const talos::SystemMeta& meta() const noexcept override { return meta_; }
};

void test_repeated_build_binds_custom_system_once() {
    printf("=== Test: Repeated Build Binds Custom System Once ===\n");

    talos::Scheduler scheduler({});

    std::atomic<int> bind_count{0};

    auto idx_result =
        scheduler.add_system(std::make_unique<BindCountingFixedRateSystem>(bind_count));
    if (!idx_result) {
        printf("  ERROR: add_system failed\n");
        return;
    }

    auto first_build = scheduler.build();
    if (!first_build) {
        printf("  ERROR: First build failed: %s\n", fmt::format("{}", first_build.error()).c_str());
        return;
    }

    auto second_build = scheduler.build();
    if (!second_build) {
        printf(
            "  ERROR: Second build failed: %s\n", fmt::format("{}", second_build.error()).c_str());
        return;
    }

    const auto count = bind_count.load(std::memory_order_relaxed);
    printf("  Bind count: %d\n", count);
    if (count != 1) {
        printf("  ERROR: Expected bind() to run exactly once\n\n");
        return;
    }

    printf("  Repeated Build Binds Custom System Once test passed!\n\n");
}

// ============================================================================
// Test: Inject Custom SystemBase into Scheduler
// ============================================================================
class CustomFusionSystem final : public talos::SystemBase {
    talos::SystemMeta meta_;
    std::atomic<int>* runs_;
    talos::spmc<CameraFrame, CameraChannel> cam_in_{};
    talos::spsc_mut<FusionResult, CustomFusionOut> out_{};
    bool written_ = false;

public:
    explicit CustomFusionSystem(std::atomic<int>& runs) noexcept
        : runs_(&runs) {
        meta_.name   = "custom_fusion";
        meta_.policy = talos::make_policy_info<talos::pool_compute>();

        meta_.spmc_channels.push_back(
            talos::ChannelMeta{
                .type  = typeid(CameraFrame),
                .topic = typeid(CameraChannel),
                .kind  = talos::channel_kind::spmc_reader,
            });
        meta_.spsc_channels.push_back(
            talos::ChannelMeta{
                .type  = typeid(FusionResult),
                .topic = typeid(CustomFusionOut),
                .kind  = talos::channel_kind::spsc_writer,
            });
    }

    void bind(talos::World& world) noexcept override {
        cam_in_            = world.get_spmc_reader<CameraFrame, CameraChannel>();
        out_               = world.get_spsc_writer<FusionResult, CustomFusionOut>();
        out_.written_flag_ = &written_;
    }

    bool run(talos::World& /*world*/) noexcept override {
        written_ = false;
        if (const auto cam = cam_in_.read()) {
            runs_->fetch_add(1, std::memory_order_relaxed);
            out_.write(
                FusionResult{
                    .imu_seq  = 0,
                    .frame_id = cam->frame_id,
                    .state_x  = 0.0f,
                });
        }
        return written_;
    }

    const talos::SystemMeta& meta() const noexcept override { return meta_; }
};

void test_custom_system_injection() {
    printf("=== Test: Custom System Injection ===\n");

    talos::Scheduler scheduler({});

    std::atomic<int> camera_count{0};
    std::atomic<int> custom_runs{0};

    scheduler.add_system<talos::fixed_rate<60>>(
        "camera_reader", [&camera_count](talos::spmc_mut<CameraFrame, CameraChannel> cam_out) {
            const int frame = camera_count.fetch_add(1, std::memory_order_relaxed);
            cam_out.write(CameraFrame{.frame_id = frame});
        });

    auto idx_result = scheduler.add_system(std::make_unique<CustomFusionSystem>(custom_runs));
    if (!idx_result) {
        printf("  ERROR: add_system failed\n");
        return;
    }

    auto build_result = scheduler.build();
    if (!build_result) {
        printf("  ERROR: Build failed: %s\n", fmt::format("{}", build_result.error()).c_str());
        return;
    }

    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    scheduler.stop();
    scheduler_thread.join();

    const auto runs = custom_runs.load(std::memory_order_relaxed);
    printf("  Custom system runs: %d\n", runs);
    if (runs <= 0) {
        printf("  ERROR: Expected custom system to run at least once.\n\n");
        return;
    }
    printf("  Custom System Injection test passed!\n\n");
}

// ============================================================================
// Main
// ============================================================================

int main() {
    printf("\n");
    printf("==========================================\n");
    printf("  Scheduler ECS Test Suite\n");
    printf("==========================================\n\n");

    // Lifecycle tests
    test_scheduler_fixed_rate_and_pool();
    test_silent_vs_notify();

    // Conflict detection tests
    test_multiple_writers_error();
    test_multiple_readers_error();
    test_spmc_multiple_readers_ok();
    test_orphaned_reader_error();
    test_channel_kind_conflict();
    test_dependency_cycle_error();
    test_unreachable_compute_error();
    test_too_many_compute_systems_error();
    test_valid_dependency_chain();
    test_failed_safe_hot_add_leaves_scheduler_configuring();
    test_repeated_build_binds_custom_system_once();
    test_custom_system_injection();

    printf("==========================================\n");
    printf("  All scheduler tests completed!\n");
    printf("==========================================\n\n");

    return 0;
}
