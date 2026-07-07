#pragma once
// 调度器错误格式化器（fmt自定义错误打印）
#include "scheduler/error_formatter.hpp"
// 调度器核心类定义、SystemBase、通道类型、构建接口
#include "scheduler/scheduler.hpp"

// 原子多线程同步
#include <atomic>
// 标准时间库ms休眠
#include <chrono>
// C标准printf/硬退出
#include <cstdio>
// fmt格式化库核心
#include <fmt/core.h>
// fmt标准类型格式化（std::string等）
#include <fmt/std.h>
// C++20 jthread 自动回收线程
#include <thread>

// ============================================================================
// 全局测试数据类型定义（仿真消息、话题标签）
// ============================================================================

/**
 * @brief IMU惯性测量单元消息结构体
 * 仿真IMU传感器输出，携带序列号、三轴加速度
 */
struct ImuData {
    // 帧序列号，自增区分每条数据
    int seq       = 0;
    // X轴加速度
    float accel_x = 0.0f;
    // Y轴加速度
    float accel_y = 0.0f;
    // Z轴加速度，默认重力9.8
    float accel_z = 9.8f;
};

/**
 * @brief 相机图像帧消息结构体
 */
struct CameraFrame {
    // 图像帧序号
    int frame_id = 0;
    // 图像宽度
    int width    = 1920;
    // 图像高度
    int height   = 1080;
};

/**
 * @brief 融合输出结果消息
 * IMU+相机融合后状态输出
 */
struct FusionResult {
    // 对应IMU序列号
    int imu_seq   = 0;
    // 对应相机帧号
    int frame_id  = 0;
    // 融合输出X轴状态
    float state_x = 0.0f;
};

// 话题空标签类型，用于区分不同业务通道（编译期唯一标识）
struct ImuChannel {};
struct CameraChannel {};
struct FusionChannel {};
struct CustomFusionOut {};

// ============================================================================
// 测试1：固定频率静默系统 + 触发式池计算系统 整体调度
// ============================================================================
/**
 * @brief 测试固定频率系统(silent/notify) + 池计算系统完整调度链路
 * 业务拓扑：
 * 1. 100Hz 静默IMU发布器（silent，写完不唤醒计算线程）
 * 2. 30Hz 相机发布器（notify，写入后置就绪位唤醒融合计算）
 * 3. 池计算融合系统：读取IMU+相机，输出融合结果
 * 校验指标：运行200ms后各系统执行次数、唤醒计数匹配相机发布次数
 */
void test_scheduler_fixed_rate_and_pool() {
    printf("=== Test: Fixed rate + Pool Systems ===\n");

    // 初始化空调度器实例
    talos::Scheduler scheduler({});

    // 原子计数器，统计各系统执行次数，多线程无锁累加
    std::atomic imu_count{0};
    std::atomic camera_count{0};
    std::atomic fusion_count{0};

    // 100Hz 静默IMU发布系统：fixed_rate_silent 写入通道不触发计算唤醒
    scheduler.add_system<talos::fixed_rate_silent<100>>(
        "imu_reader", [&imu_count](talos::spmc_mut<ImuData, ImuChannel> imu_out) {
            // 原子自增获取当前序列号，relaxed内存序仅计数
            const int seq = imu_count.fetch_add(1, std::memory_order_relaxed);
            // 写入SPMC多生产者通道
            imu_out.write(ImuData{.seq = seq, .accel_x = 0.1f * seq});
        });

    // 30Hz 相机发布系统：普通fixed_rate，write后标记就绪位唤醒下游计算
    scheduler.add_system<talos::fixed_rate<30>>(
        "camera_reader", [&camera_count](talos::spmc_mut<CameraFrame, CameraChannel> cam_out) {
            const int frame = camera_count.fetch_add(1, std::memory_order_relaxed);
            cam_out.write(CameraFrame{.frame_id = frame});
        });

    // 池计算融合系统：依赖IMU读通道、相机读通道、融合写通道
    scheduler.add_system<talos::pool_compute>(
        "fusion",
        [&fusion_count](
            talos::spmc<ImuData, ImuChannel> imu_in, talos::spmc<CameraFrame, CameraChannel> cam_in,
            talos::spsc_mut<FusionResult, FusionChannel> fusion_out) {
            // 读取最新IMU快照
            if (const auto imu = imu_in.read()) {
                // 读取最新相机快照
                if (const auto cam = cam_in.read()) {
                    fusion_count.fetch_add(1, std::memory_order_relaxed);
                    // 写入融合输出
                    fusion_out.write(
                        FusionResult{
                            .imu_seq  = imu->seq,
                            .frame_id = cam->frame_id,
                            .state_x  = imu->accel_x,
                        });
                }
            }
        });

    // 打印调度器内部所有系统元数据、通道拓扑
    scheduler.print_systems();

    // 构建调度拓扑，校验依赖、通道冲突、环路等
    auto build_result = scheduler.build();
    // 构建失败打印错误并直接返回
    if (!build_result) {
        printf("  ERROR: Build failed: %s\n", fmt::format("{}", build_result.error()).c_str());
        return;
    }

    // 后台线程启动调度循环
    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });

    // 并发运行200毫秒
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 发送停止信号，调度循环退出
    scheduler.stop();
    // 等待调度线程完全回收
    scheduler_thread.join();

    // 读取运行统计、各系统执行总数
    auto stats       = scheduler.stats();
    int final_imu    = imu_count.load();
    int final_camera = camera_count.load();
    int final_fusion = fusion_count.load();

    // 打印运行结果
    printf("  Results after 200ms:\n");
    printf("    IMU reads:      %d (expected ~20 at 100Hz)\n", final_imu);
    printf("    Camera reads:   %d (expected ~6 at 30Hz)\n", final_camera);
    printf("    Fusion runs:    %d\n", final_fusion);
    printf("    Notify count:   %lu\n", stats.notify_count);
    printf("    Compute cycles: %lu\n", stats.compute_cycle_count);

    // 校验预期指标
    bool imu_ok    = final_imu >= 10;                              // IMU至少执行10次
    bool camera_ok = final_camera >= 3;                            // 相机至少3帧
    bool fusion_ok = final_fusion >= 1;                            // 融合至少执行一次
    bool notify_ok =
        stats.notify_count == static_cast<uint64_t>(final_camera); // 唤醒次数严格等于相机发布次数

    // 全部校验通过打印成功，否则打印失败详情
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
// 测试2：静默fixed_rate系统 与 notify触发系统行为区分
// ============================================================================
/**
 * @brief 区分silent静默周期系统与普通notify周期系统行为
 * 1. silent 200Hz系统：无输出通道，不会产生notify唤醒任何计算任务
 * 2. notify 50Hz系统：写入SPMC通道，每次write置就绪位唤醒池计算
 * 校验：notify总数严格等于发布次数，silent高频执行不产生唤醒标记
 */
void test_silent_vs_notify() {
    printf("=== Test: Silent vs Notify ===\n");

    struct NotifyTag {};

    talos::Scheduler scheduler({});

    // 各系统执行计数原子变量
    std::atomic silent_count{0};
    std::atomic notify_count{0};
    std::atomic sink_count{0};

    // 200Hz 静默周期系统：无输出通道，不会触发notify
    scheduler.add_system<talos::fixed_rate_silent<200>>("silent_source", [&silent_count]() {
        silent_count.fetch_add(1, std::memory_order_relaxed);
    });

    // 50Hz 普通周期发布系统，带SPMC输出通道，写入触发notify
    scheduler.add_system<talos::fixed_rate<50>>(
        "notify_source", [&notify_count](talos::spmc_mut<int, NotifyTag> out) {
            const auto count = notify_count.fetch_add(1, std::memory_order_relaxed) + 1;
            out.write(count);
        });

    // 池计算消费系统，读取notify通道数据
    scheduler.add_system<talos::pool_compute>(
        "notify_sink", [&sink_count](talos::spmc<int, NotifyTag> in) {
            if (in.read()) {
                sink_count.fetch_add(1, std::memory_order_relaxed);
            }
        });

    // 构建拓扑
    (void)scheduler.build();

    // 后台运行调度器
    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });

    // 运行150ms
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    // 停止调度并回收线程
    scheduler.stop();
    scheduler_thread.join();

    const auto stats = scheduler.stats();

    // 打印统计
    printf("  Silent calls:   %d (200Hz, ~30 expected)\n", silent_count.load());
    printf("  Notify calls:   %d (50Hz, ~7 expected)\n", notify_count.load());
    printf("  Actual notifies: %lu (should match notify calls)\n", stats.notify_count);

    // 唤醒计数必须等于发布次数，silent执行次数大于notify代表静默不产生唤醒
    const bool notify_match = stats.notify_count == static_cast<uint64_t>(notify_count.load());

    if (notify_match && silent_count.load() > notify_count.load() && sink_count.load() > 0) {
        printf("  Silent vs Notify test passed!\n\n");
    } else {
        printf("  ERROR: Test failed\n");
    }
}

// ============================================================================
// 测试3：SPSC通道多写入器冲突检测（构建失败）
// ============================================================================
/**
 * @brief 校验SPSC单生产者通道禁止多个写入系统，构建触发MultipleWritersError
 * 同一SPSC通道注册两个writer，build返回错误变体
 */
void test_multiple_writers_error() {
    printf("=== Test: Multiple Writers Error ===\n");

    struct SharedChannel {};

    talos::Scheduler scheduler({});

    // 第一个写入系统
    scheduler.add_system<talos::pool_compute>(
        "writer1", [](talos::spsc_mut<int, SharedChannel> out) { out.write(1); });

    // 第二个写入系统，同一SPSC通道，触发冲突
    scheduler.add_system<talos::pool_compute>(
        "writer2", [](talos::spsc_mut<int, SharedChannel> out) { out.write(2); });

    // 执行构建
    auto result = scheduler.build();
    // 预期构建失败，成功则报错
    if (result) {
        printf("  ERROR: Build should have failed with MultipleWritersError\n");
        return;
    }

    // 校验错误变体类型为多写入器错误
    if (!std::holds_alternative<talos::MultipleWritersError>(result.error())) {
        printf("  ERROR: Wrong error type: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    // 提取错误结构体打印详情
    auto& err = std::get<talos::MultipleWritersError>(result.error());
    printf("  Got expected error: %s\n", fmt::format("{}", result.error()).c_str());
    printf("  Writers: ");
    for (const auto& w : err.writers) {
        printf("'%s' ", w.c_str());
    }
    printf("\n  Multiple Writers Error test passed!\n\n");
}

// ============================================================================
// 测试4：SPSC通道多读取器冲突（构建失败）
// ============================================================================
/**
 * @brief SPSC单消费者通道不允许多个读取系统，触发MultipleReadersError
 */
void test_multiple_readers_error() {
    printf("=== Test: Multiple Readers Error (SPSC) ===\n");

    struct SpscChannel {};

    talos::Scheduler scheduler({});

    // 单个写入器
    scheduler.add_system<talos::pool_compute>(
        "writer", [](talos::spsc_mut<int, SpscChannel> out) { out.write(42); });

    // 两个读取器，SPSC不支持多读
    scheduler.add_system<talos::pool_compute>(
        "reader1", [](talos::spsc<int, SpscChannel> in) { (void)in.read(); });

    scheduler.add_system<talos::pool_compute>(
        "reader2", [](talos::spsc<int, SpscChannel> in) { (void)in.read(); });

    auto result = scheduler.build();
    if (result) {
        printf("  ERROR: Build should have failed with MultipleReadersError\n");
        return;
    }

    // 校验错误类型
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
// 测试5：SPMC多读取器合法（无冲突）
// ============================================================================
/**
 * @brief SPMC多生产者多消费者通道支持任意数量读取器，构建正常通过
 */
void test_spmc_multiple_readers_ok() {
    printf("=== Test: SPMC Multiple Readers (OK) ===\n");

    struct BroadcastChannel {};

    talos::Scheduler scheduler({});

    // 单一发布器
    scheduler.add_system<talos::fixed_rate<30>>(
        "broadcaster", [](talos::spmc_mut<int, BroadcastChannel> out) { out.write(42); });

    // 三个订阅读取器，SPMC允许
    scheduler.add_system<talos::pool_compute>(
        "subscriber1", [](talos::spmc<int, BroadcastChannel> in) { (void)in.read(); });

    scheduler.add_system<talos::pool_compute>(
        "subscriber2", [](talos::spmc<int, BroadcastChannel> in) { (void)in.read(); });

    scheduler.add_system<talos::pool_compute>(
        "subscriber3", [](talos::spmc<int, BroadcastChannel> in) { (void)in.read(); });

    // 构建预期成功
    auto result = scheduler.build();
    if (!result) {
        printf("  ERROR: Build failed: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    printf("  SPMC with 3 readers built successfully!\n");
    printf("  SPMC Multiple Readers test passed!\n\n");
}

// ============================================================================
// 测试6：孤儿读取器（无对应写入器）错误
// ============================================================================
/**
 * @brief 存在读取系统，但无任何写入系统，触发OrphanedReaderError
 */
void test_orphaned_reader_error() {
    printf("=== Test: Orphaned Reader Error ===\n");

    struct OrphanChannel {};

    talos::Scheduler scheduler({});

    // 仅有读取器，无写入发布器
    scheduler.add_system<talos::pool_compute>(
        "orphan_reader", [](talos::spsc<int, OrphanChannel> in) { (void)in.read(); });

    auto result = scheduler.build();
    if (result) {
        printf("  ERROR: Build should have failed with OrphanedReaderError\n");
        return;
    }

    // 校验错误类型
    if (!std::holds_alternative<talos::OrphanedReaderError>(result.error())) {
        printf("  ERROR: Wrong error type: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    printf("  Got expected error: %s\n", fmt::format("{}", result.error()).c_str());
    printf("  Orphaned Reader Error test passed!\n\n");
}

// ============================================================================
// 测试7：通道类型混合冲突（同一通道同时SPSC+SPMC）
// ============================================================================
/**
 * @brief 同一频道同时使用SPSC写入、SPMC读取，通道类型冲突ChannelKindConflict
 */
void test_channel_kind_conflict() {
    printf("=== Test: Channel Kind Conflict ===\n");

    struct ConflictChannel {};

    talos::Scheduler scheduler({});

    // SPSC写入
    scheduler.add_system<talos::pool_compute>(
        "spsc_writer", [](talos::spsc_mut<int, ConflictChannel> out) { out.write(1); });

    // SPMC读取，类型冲突
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
// 测试8：依赖环形环路错误
// ============================================================================
/**
 * @brief 系统读写形成环形依赖 A→B→C→A，拓扑无法排序，触发DependencyCycleError
 */
void test_dependency_cycle_error() {
    printf("=== Test: Dependency Cycle Error ===\n");

    struct ChannelAB {};
    struct ChannelBC {};
    struct ChannelCA {};

    talos::Scheduler scheduler({});

    // A 读CA，写AB
    scheduler.add_system<talos::pool_compute>(
        "system_a", [](talos::spsc<int, ChannelCA> in, talos::spsc_mut<int, ChannelAB> out) {
            if (const auto v = in.read())
                out.write(*v + 1);
        });

    // B 读AB，写BC
    scheduler.add_system<talos::pool_compute>(
        "system_b", [](talos::spsc<int, ChannelAB> in, talos::spsc_mut<int, ChannelBC> out) {
            if (const auto v = in.read())
                out.write(*v + 1);
        });

    // C 读BC，写CA，闭环形成环路
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
// 测试9：孤立不可达计算系统错误
// ============================================================================
/**
 * @brief 池计算系统无任何前置触发源、无外部唤醒，永远不会执行，UnreachableComputeSystemsError
 */
void test_unreachable_compute_error() {
    printf("=== Test: Unreachable Compute Error ===\n");

    talos::Scheduler scheduler({});

    // 无输入通道、无外部触发的孤立计算任务
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
// 测试10：计算系统数量超限错误
// ============================================================================
/**
 * @brief 池计算系统数量超过调度器最大配置上限，触发TooManyComputeSystemsError
 */
void test_too_many_compute_systems_error() {
    printf("=== Test: Too Many Compute Systems Error ===\n");

    talos::Scheduler scheduler({});

    // 创建上限+1个计算系统，超出阈值
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
// 测试11：合法线性依赖链路（无环路，构建正常）
// ============================================================================
/**
 * @brief 线性流水线依赖：源→stage1→stage2→stage3，无循环，拓扑可排序，构建通过
 */
void test_valid_dependency_chain() {
    printf("=== Test: Valid Dependency Chain ===\n");

    struct SourceOut {};
    struct Stopice1Out {};
    struct Stopice2Out {};
    struct Stopice3Out {};

    talos::Scheduler scheduler({});

    // 30Hz外部源
    scheduler.add_system<talos::fixed_rate<30>>(
        "source", [](talos::spsc_mut<int, SourceOut> out) { out.write(1); });

    // 三级流水线池计算，线性依赖无环
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
// 测试12：运行时热添加系统失败后，调度器维持Configuring状态
// ============================================================================
/**
 * @brief 已完成初始build后，hot_add_system添加冲突系统失败
 * 校验：添加失败后调度器仍处于Configuring配置状态，禁止run()执行
 */
void test_failed_safe_hot_add_leaves_scheduler_configuring() {
    printf("=== Test: Failed Safe Hot-Add Leaves Scheduler Configuring ===\n");

    struct SafeHotAddChannel {};

    talos::Scheduler scheduler({});

    // 初始正常拓扑：单发布+单订阅
    scheduler.add_system<talos::fixed_rate<30>>(
        "source", [](talos::spmc_mut<int, SafeHotAddChannel> out) { out.write(1); });
    scheduler.add_system<talos::pool_compute>(
        "sink", [](talos::spmc<int, SafeHotAddChannel> in) { (void)in.read(); });

    // 第一次构建成功
    auto initial_build = scheduler.build();
    if (!initial_build) {
        printf(
            "  ERROR: Initial build failed: %s\n",
            fmt::format("{}", initial_build.error()).c_str());
        return;
    }

    // 热添加第二个写入器，同一SPMC通道触发多写入错误
    auto duplicate_writer = [](talos::spmc_mut<int, SafeHotAddChannel> out) { out.write(2); };
    auto result           = scheduler.hot_add_system(
        talos::make_system<decltype(duplicate_writer), talos::pool_compute>(
            "duplicate_writer", std::move(duplicate_writer)));

    // 预期添加失败
    if (result) {
        printf("  ERROR: hot_add_system should have failed with MultipleWritersError\n");
        return;
    }

    // 校验错误类型
    if (!std::holds_alternative<talos::MultipleWritersError>(result.error())) {
        printf("  ERROR: Wrong error type: %s\n", fmt::format("{}", result.error()).c_str());
        return;
    }

    // 校验调度器生命周期状态仍为配置中，不允许运行
    if (scheduler.lifecycle_state() != talos::Scheduler::LifecycleState::Configuring) {
        printf(
            "  ERROR: Scheduler should have remained in Configuring state after failed hot-add\n");
        return;
    }

    // 调用run()预期返回NotBuilt错误，禁止启动调度循环
    auto run_result = scheduler.run();
    if (run_result || run_result.error() != talos::SchedulerError::NotBuilt) {
        printf("  ERROR: run() should have failed with NotBuilt after failed hot-add\n");
        return;
    }

    printf("  Failed Safe Hot-Add Leaves Scheduler Configuring test passed!\n\n");
}

// ============================================================================
// 测试13：重复build仅执行一次自定义系统bind()
// ============================================================================
/**
 * @brief 自定义SystemBase子类bind()仅在第一次build时调用，重复build不重复绑定
 * 自定义系统：BindCountingFixedRateSystem，bind()原子计数，校验仅执行1次
 */
class BindCountingFixedRateSystem final : public talos::SystemBase {
    talos::SystemMeta meta_;
    std::atomic<int>* bind_count_;

public:
    explicit BindCountingFixedRateSystem(std::atomic<int>& bind_count) noexcept
        : bind_count_(&bind_count) {
        // 构造元数据：1Hz静默周期
        meta_.name   = "bind_counting_fixed_rate";
        meta_.policy = talos::make_policy_info<talos::fixed_rate_silent<1>>();
    }

    // 绑定World资源，原子计数+1
    void bind(talos::World&) noexcept override {
        bind_count_->fetch_add(1, std::memory_order_relaxed);
    }

    // 周期执行返回false，无持续任务
    bool run(talos::World&) noexcept override { return false; }

    // 返回系统元数据
    const talos::SystemMeta& meta() const noexcept override { return meta_; }
};

void test_repeated_build_binds_custom_system_once() {
    printf("=== Test: Repeated Build Binds Custom System Once ===\n");

    talos::Scheduler scheduler({});

    std::atomic<int> bind_count{0};

    // 添加自定义系统
    auto idx_result =
        scheduler.add_system(std::make_unique<BindCountingFixedRateSystem>(bind_count));
    if (!idx_result) {
        printf("  ERROR: add_system failed\n");
        return;
    }

    // 第一次构建，触发bind()
    auto first_build = scheduler.build();
    if (!first_build) {
        printf("  ERROR: First build failed: %s\n", fmt::format("{}", first_build.error()).c_str());
        return;
    }

    // 第二次重复build，不重复执行bind()
    auto second_build = scheduler.build();
    if (!second_build) {
        printf(
            "  ERROR: Second build failed: %s\n", fmt::format("{}", second_build.error()).c_str());
        return;
    }

    // 读取绑定计数，必须等于1
    const auto count = bind_count.load(std::memory_order_relaxed);
    printf("  Bind count: %d\n", count);
    if (count != 1) {
        printf("  ERROR: Expected bind() to run exactly once\n\n");
        return;
    }

    printf("  Repeated Build Binds Custom System Once test passed!\n\n");
}

// ============================================================================
// 测试14：注入完全自定义SystemBase子类到调度器
// ============================================================================
/**
 * @brief 完全自定义SystemBase实现融合系统，手动构造SystemMeta通道信息
 * 不依赖模板自动推导元数据，手动声明SPMC读、SPSC写通道，正常参与调度运行
 */
class CustomFusionSystem final : public talos::SystemBase {
    talos::SystemMeta meta_;
    std::atomic<int>* runs_;
    // SPMC相机读取通道句柄
    talos::spmc<CameraFrame, CameraChannel> cam_in_{};
    // SPSC融合输出写入通道
    talos::spsc_mut<FusionResult, CustomFusionOut> out_{};
    // 标记本轮run是否产生写入
    bool written_ = false;

public:
    explicit CustomFusionSystem(std::atomic<int>& runs) noexcept
        : runs_(&runs) {
        meta_.name   = "custom_fusion";
        meta_.policy = talos::make_policy_info<talos::pool_compute>();

        // 手动添加SPMC读通道元数据
        meta_.spmc_channels.push_back(
            talos::ChannelMeta{
                .type  = typeid(CameraFrame),
                .topic = typeid(CameraChannel),
                .kind  = talos::channel_kind::spmc_reader,
            });
        // 手动添加SPSC写通道元数据
        meta_.spsc_channels.push_back(
            talos::ChannelMeta{
                .type  = typeid(FusionResult),
                .topic = typeid(CustomFusionOut),
                .kind  = talos::spsc_writer,
            });
    }

    // 绑定World获取通道句柄
    void bind(talos::World& world) noexcept override {
        cam_in_            = world.get_spmc_reader<CameraFrame, CameraChannel>();
        out_               = world.get_spsc_writer<FusionResult, CustomFusionOut>();
        out_.written_flag_ = &written_;
    }

    // 调度周期执行逻辑
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

    // 返回手动构造的元数据
    const talos::SystemMeta& meta() const noexcept override { return meta_; }
};

void test_custom_system_injection() {
    printf("=== Test: Custom System Injection ===\n");

    talos::Scheduler scheduler({});

    std::atomic<int> camera_count{0};
    std::atomic<int> custom_runs{0};

    // 60Hz相机发布系统
    scheduler.add_system<talos::fixed_rate<60>>(
        "camera_reader", [&camera_count](talos::spmc_mut<CameraFrame, CameraChannel> cam_out) {
            const int frame = camera_count.fetch_add(1, std::memory_order_relaxed);
            cam_out.write(CameraFrame{.frame_id = frame});
        });

    // 注入完全自定义融合系统
    auto idx_result = scheduler.add_system(std::make_unique<CustomFusionSystem>(custom_runs));
    if (!idx_result) {
        printf("  ERROR: add_system failed\n");
        return;
    }

    // 构建拓扑
    auto build_result = scheduler.build();
    if (!build_result) {
        printf("  ERROR: Build failed: %s\n", fmt::format("{}", build_result.error()).c_str());
        return;
    }

    // 后台运行调度200ms
    std::jthread scheduler_thread([&scheduler]() { (void)scheduler.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    scheduler.stop();
    scheduler_thread.join();

    // 校验自定义系统至少执行一次
    const auto runs = custom_runs.load(std::memory_order_relaxed);
    printf("  Custom system runs: %d\n", runs);
    if (runs <= 0) {
        printf("  ERROR: Expected custom system to run at least once.\n\n");
        return;
    }
    printf("  Custom System Injection test passed!\n\n");
}

// ============================================================================
// 程序入口main：批量执行全部测试用例
// ============================================================================
int main() {
    printf("\n");
    printf("==========================================\n");
    printf("  Scheduler ECS Test Suite\n");
    printf("==========================================\n\n");

    // 生命周期、调度基础测试
    test_scheduler_fixed_rate_and_pool();
    test_silent_vs_notify();

    // 拓扑冲突检测类测试
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