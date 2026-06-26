// Google Test 单元测试框架
#include <gtest/gtest.h>

// C++标准容器、时间、文件系统、IO、数值、字符串、线程基础库
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

// JSON序列化库，录制输出快照使用JSONL格式存储数据流状态
#include <nlohmann/json.hpp>
// OpenCV核心，用于仿真生成测试图像帧
#include <opencv2/core.hpp>

// 业务分层类型头文件
// L2 能量机关视觉观测结构体
#include "L2_perception/rune/types.hpp"
// L3 能量计状态
#include "L3_estimation/energy_meter/types.hpp"
// L3 目标跟踪输出
#include "L3_estimation/tracker/types.hpp"
// L4 云台控制意图
#include "L4_planning/control_intent.hpp"
// L4 选中目标快照
#include "L4_planning/selected_target_snapshot.hpp"
// L4 目标选择追踪日志
#include "L4_planning/target_selection_trace.hpp"
// L5 武器发射指令
#include "L5_weapon/fire_control.hpp"

// 全局配置、消息通道主题、运行时、高精度时钟、弹道资源、基础业务类型
#include "config.hpp"
#include "core/channel_topics.hpp"
#include "core/runtime.hpp"
#include "core/time.hpp"
#include "core/trajectory/resource.hpp"
#include "core/types.hpp"

// 录制器运行时封装、任务调度器
#include "runtime/capturer.hpp"
#include "scheduler/scheduler.hpp"

// 匿名命名空间：隔离所有测试工具函数与测试用例，防止全局符号冲突
namespace {

// 文件系统简写别名
namespace fs = std::filesystem;
// JSON库简写
using json   = nlohmann::json;
// 时间字面量支持（1000ms、1s 写法）
using namespace std::chrono_literals;

/**
 * @brief 仿真数据生产者工作模式枚举，用于模拟不同机器人运行工况
 * Continuous：持续正常全量输出所有数据流
 * StaleDetector：前1s正常输出检测帧，1.5s后检测流不再更新（模拟检测卡死、数据陈旧）
 * ControlBurstThenIdle：控制数据前1.2s持续输出，之后停止更新
 * FollowingActive：云台持续跟踪目标模式，相机录制高帧率
 * FireActive：开火模式，相机录制高帧率
 */
enum class ProducerMode {
    Continuous,
    StaleDetector,
    ControlBurstThenIdle,
    FollowingActive,
    FireActive,
};

/**
 * @brief 创建带唯一时间戳的临时测试目录，避免多测试文件目录冲突
 * @param name 测试场景标识字符串
 * @return fs::path 临时目录完整路径
 */
auto make_temp_dir(std::string_view name) -> fs::path {
    // 使用steady_clock高精度时间戳保证目录全局唯一
    const auto unique = std::chrono::steady_clock::now().time_since_epoch().count();
    // 拼接临时目录根路径+测试标识+唯一序列号
    const auto path   = fs::temp_directory_path()
                    / ("talos-capturer-test-" + std::string(name) + "-" + std::to_string(unique));
    // 递归创建目录
    fs::create_directories(path);
    return path;
}

/**
 * @brief 读取JSON Lines文件（一行一个json对象），返回json对象数组
 * @param path snapshots.jsonl 文件路径
 * @return std::vector<json> 每行解析后的json集合
 */
auto read_jsonl(const fs::path& path) -> std::vector<json> {
    std::ifstream in(path);
    std::vector<json> rows;
    std::string line;
    // 逐行读取
    while (std::getline(in, line)) {
        // 跳过空行，防止解析异常
        if (!line.empty()) {
            rows.push_back(json::parse(line));
        }
    }
    return rows;
}

/**
 * @brief 读取完整单个JSON文件并解析
 */
auto read_json_file(const fs::path& path) -> json {
    std::ifstream in(path);
    return json::parse(in);
}

/**
 * @brief 读取普通文本文件全部内容为字符串（用于比对配置文件快照）
 */
auto read_text_file(const fs::path& path) -> std::string {
    std::ifstream in(path);
    // 使用流迭代器一次性读取全部字符
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

/**
 * @brief 在录制输出根目录下查找第一个运行录制子目录
 * Capturer启动会自动生成一个时间戳命名的run子目录存放图片、日志、配置快照
 * @param output_dir 录制输出根目录
 * @return 首个子目录路径，无目录返回空path
 */
auto find_run_dir(const fs::path& output_dir) -> fs::path {
    for (const auto& entry : fs::directory_iterator(output_dir)) {
        if (entry.is_directory()) {
            return entry.path();
        }
    }
    return {};
}

/**
 * @brief 统计指定目录下jpg图片数量，支持文件名前缀过滤（camera_）
 * @param dir 图片目录
 * @param prefix 文件名前缀筛选，空则统计全部jpg
 * @return 匹配文件个数
 */
auto count_jpegs(const fs::path& dir, std::string_view prefix = "") -> size_t {
    size_t count = 0;
    for (const auto& entry : fs::directory_iterator(dir)) {
        // 跳过非文件、非jpg后缀
        if (!entry.is_regular_file() || entry.path().extension() != ".jpg") {
            continue;
        }
        // 存在前缀且文件名不以前缀开头则跳过
        if (!prefix.empty() && entry.path().filename().string().rfind(prefix, 0) != 0) {
            continue;
        }
        ++count;
    }
    return count;
}

/**
 * @brief 获取目录下所有jpg图片文件名列表，用于校验只有camera开头图片
 */
auto image_filenames(const fs::path& dir) -> std::vector<std::string> {
    std::vector<std::string> names;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (!entry.is_regular_file() || entry.path().extension() != ".jpg") {
            continue;
        }
        names.push_back(entry.path().filename().string());
    }
    return names;
}

/**
 * @brief 获取项目源码根目录路径，由编译宏 TALOS_SOURCE_DIR 注入
 */
auto source_root() -> fs::path { return fs::path(TALOS_SOURCE_DIR); }

/**
 * @brief 作用域自动切换工作目录RAII类
 * 构造：切换到指定目录；析构：自动恢复原始工作目录
 * 避免测试修改全局cwd污染其他用例
 */
class ScopedCurrentPath {
public:
    explicit ScopedCurrentPath(const fs::path& path)
        : original_(fs::current_path()) {
        fs::current_path(path);
    }

    ~ScopedCurrentPath() { fs::current_path(original_); }

private:
    fs::path original_;
};

/**
 * @brief 构造录制器启动上下文：后端直写、英雄机器人、标准视觉配置
 */
auto launch_context() -> fcs::runtime::CapturerLaunchContext {
    return {.backend = fcs::Direct, .robot = "hero", .vision = "std"};
}

/**
 * @brief 向调度器注册仿真数据流生产者系统
 * 固定20Hz定时输出全链路仿真数据：图像、检测、测量、跟踪、能量机关、控制指令、IMU等
 * @param scheduler Talos全局调度器
 * @param mode 仿真工况模式，控制各数据流启停/新旧状态
 */
void register_test_producer(talos::Scheduler& scheduler, ProducerMode mode) {
    // 添加20Hz定时任务系统
    scheduler.add_system<talos::fixed_rate<20>>(
        "capturer_test_producer",
        // 闭包捕获工况、启动时间戳；入参为各通道SPMC多生产者单消费者写句柄
        [mode, start_ns = fcs::clock::now_ns()](
            // 图像帧输出通道
            talos::spmc_mut<fcs::ImageFrame, fcs::ImageChannelTopic> image_out,
            // 装甲检测批量结果通道
            talos::spmc_mut<fcs::ArmorDetectionBatch, fcs::DetectionChannelTopic> detection_out,
            // PnP测量结果通道
            talos::spmc_mut<fcs::ArmorMeasurementBatch, fcs::MeasurementChannelTopic>
                measurement_out,
            // L3跟踪器输出通道
            talos::spmc_mut<fcs::L3::TrackerOutputs, fcs::TrackerOutputChannelTopic> tracker_out,
            // 能量机关观测通道
            talos::spmc_mut<fcs::rune::RuneObservation, fcs::RuneObservationChannelTopic>
                rune_observation_out,
            // 能量机关调试帧通道
            talos::spmc_mut<fcs::rune::RuneDebugFrame, fcs::RuneDebugFrameChannelTopic>
                rune_debug_out,
            // 能量计状态通道
            talos::spmc_mut<fcs::energy_meter::EnergyMeterState, fcs::EnergyMeterStateChannelTopic>
                energy_meter_out,
            // L4选中目标快照通道
            talos::spmc_mut<
                fcs::L4::SelectedTargetSnapshot, fcs::SelectedTargetSnapshotChannelTopic>
                selected_target_out,
            // 目标选择日志通道
            talos::spmc_mut<fcs::L4::TargetSelectionTrace, fcs::TargetSelectionTraceChannelTopic>
                target_selection_trace_out,
            // 云台控制意图通道
            talos::spmc_mut<fcs::L4::ControlIntent, fcs::ControlIntentChannelTopic>
                control_intent_out,
            // L5武器发射指令通道
            talos::spmc_mut<fcs::L5::WeaponCommand, fcs::WeaponCommandChannelTopic> weapon_out,
            // 全局跟踪状态共享资源
            fcs::core::following_mut following,
            // IMU/控制资源快照通道
            talos::spmc_mut<
                fcs::core::ControlResourceSnapshot, fcs::RuntimeControlStateChannelTopic>
                control_state_out) mutable {
            // 当前高精度纳秒时间戳
            const uint64_t now_ns       = fcs::clock::now_ns();
            // 运行超过900ms进入检测器陈旧阶段
            const bool stale_phase      = (now_ns - start_ns) >= 900'000'000ULL;
            // 运行超过1200ms进入控制数据空闲阶段
            const bool burst_idle_phase = (now_ns - start_ns) >= 1200'000'000ULL;
            // 陈旧阶段使用帧ID=2，正常阶段ID=1，用于区分新旧帧标记
            const uint64_t camera_frame = stale_phase ? 2 : 1;
            // 正常帧蓝色画布，陈旧帧红色画布，便于日志区分
            const cv::Scalar color = stale_phase ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
            // 生成4x4纯色测试图像
            cv::Mat image(4, 4, CV_8UC3, color);

            // 图像帧每帧必写，无论何种工况
            image_out.write(fcs::ImageFrame{image.clone(), now_ns, camera_frame});

            // lambda判断当前工况是否需要输出检测/测量/跟踪数据
            const bool publish_detector = [&]() {
                switch (mode) {
                case ProducerMode::Continuous:
                case ProducerMode::FollowingActive:
                case ProducerMode::FireActive:
                case ProducerMode::ControlBurstThenIdle: return true;
                // StaleDetector模式：仅非陈旧阶段输出检测，900ms后不再写入检测通道
                case ProducerMode::StaleDetector: return !stale_phase;
                }
                return false;
            }();

            std::optional<fcs::L3::TrackerOutput> selected_tracker;
            if (publish_detector) {
                // 写入空检测批量包，携带当前图像与时间戳
                detection_out.write(
                    fcs::ArmorDetectionBatch{{}, image.clone(), now_ns, camera_frame});
                // 写入空PnP测量包
                measurement_out.write(fcs::ArmorMeasurementBatch{{}, now_ns, camera_frame});

                // 构造跟踪器输出，区分Detecting/Tracking状态、不同装甲ID
                fcs::L3::TrackerOutputs trackers;
                fcs::L3::TrackerOutput tracker;
                tracker.timestamp_ns = now_ns;
                tracker.status       = stale_phase ? fcs::L3::TrackerStatus::Tracking
                                                   : fcs::L3::TrackerStatus::Detecting;
                tracker.target_name  = stale_phase ? fcs::ArmorName::Two : fcs::ArmorName::One;
                selected_tracker     = tracker;
                trackers.push_back(tracker);
                tracker_out.write(trackers);
            }

            // 能量机关观测帧持续输出，不受工况影响
            fcs::rune::RuneObservation rune_observation;
            rune_observation.timestamp_ns = now_ns;
            rune_observation.frame_id     = camera_frame;
            rune_observation_out.write(rune_observation);

            // 能量机关调试帧持续输出
            fcs::rune::RuneDebugFrame rune_debug;
            rune_debug.timestamp_ns = now_ns;
            rune_debug.frame_id     = camera_frame;
            rune_debug_out.write(rune_debug);

            // 能量计状态持续输出
            fcs::energy_meter::EnergyMeterState energy_meter;
            energy_meter.timestamp_ns = now_ns;
            energy_meter_out.write(energy_meter);

            // 选中目标快照，有跟踪结果则填充距离、预测时间
            fcs::L4::SelectedTargetSnapshot selected_target;
            selected_target.timestamp_ns = now_ns;
            if (selected_tracker) {
                selected_target.distance            = 1.0;
                selected_target.predicted_future_ns = now_ns + 10'000'000ULL;
                selected_target.source              = fcs::L4::GimbalPlanSource::Armor;
                selected_target.tracker             = *selected_tracker;
            }
            selected_target_out.write(selected_target);

            // 目标选择日志持续输出
            fcs::L4::TargetSelectionTrace target_selection_trace;
            target_selection_trace.timestamp_ns = now_ns;
            target_selection_trace_out.write(target_selection_trace);

            // 云台保持控制指令持续下发
            fcs::L4::ControlIntent control_intent = fcs::L4::HoldCommand{.timestamp_ns = now_ns};
            control_intent_out.write(control_intent);

            // 武器指令：仅FireActive模式置开火标记
            fcs::L5::WeaponCommand weapon_command;
            weapon_command.timestamp_ns = now_ns;
            weapon_command.fire         = mode == ProducerMode::FireActive;
            weapon_out.write(weapon_command);

            // 全局跟踪状态资源写入：仅FollowingActive置true
            following->store(mode == ProducerMode::FollowingActive);

            // 构造IMU惯性测量单元仿真数据，新旧阶段俯仰偏航数值区分
            fcs::core::ImuState imu_state;
            imu_state.timestamp_ns = now_ns;
            imu_state.yaw          = stale_phase ? 0.2 : 0.1;
            imu_state.pitch        = stale_phase ? 0.05 : -0.05;
            imu_state.roll         = 0.0;
            imu_state.yaw_vel      = 0.01;
            imu_state.pitch_vel    = -0.02;
            imu_state.roll_vel     = 0.0;

            // 判断当前工况是否输出IMU/控制资源快照
            const bool publish_control_state = [&]() {
                switch (mode) {
                case ProducerMode::Continuous:
                case ProducerMode::StaleDetector:
                case ProducerMode::FollowingActive:
                case ProducerMode::FireActive: return true;
                // ControlBurstThenIdle：超过1.2s不再输出控制数据
                case ProducerMode::ControlBurstThenIdle: return !burst_idle_phase;
                }
                return false;
            }();

            if (publish_control_state) {
                control_state_out.write(
                    fcs::core::ControlResourceSnapshot{
                        .sample_timestamp_ns = now_ns,
                        .imu                 = imu_state,
                        .detecting_color     = fcs::ArmorColor::Blue,
                        .bullet_speed        = 20.0,
                    });
            }
        });
}

/**
 * @brief 启动调度器运行指定时长，运行结束自动停止并等待线程回收
 * @param scheduler 调度器实例
 * @param duration 运行毫秒时长
 */
void run_scheduler_for(talos::Scheduler& scheduler, std::chrono::milliseconds duration) {
    // 构建调度器系统拓扑，失败直接终止测试
    const auto build_result = scheduler.build();
    ASSERT_TRUE(build_result.has_value()) << "scheduler build failed";

    std::optional<talos::SchedulerError> run_error;
    // 新建后台线程执行调度器主循环
    std::thread scheduler_thread([&]() {
        if (const auto result = scheduler.run(); !result) {
            run_error = result.error();
        }
    });

    // 主线程休眠指定时长
    std::this_thread::sleep_for(duration);
    // 发送停止信号
    scheduler.stop();
    // 阻塞等待调度线程退出
    scheduler_thread.join();
    // 运行过程不能存在错误
    ASSERT_FALSE(run_error.has_value());
}

/**
 * @brief 初始化运行时全局共享资源，供录制器、生产者读取
 * @param world Talos ECS资源世界
 */
void seed_runtime_resources(talos::World& world) {
    // 坐标变换系统资源
    (void)world.emplace_resource<fast_tf::CoordinateSystem>();
    // IMU状态资源
    world.insert_resource(fcs::core::ImuState{});
    // 默认识别装甲颜色蓝色
    world.insert_resource(fcs::ArmorColor::Blue);
    // 弹道初速资源20m/s
    world.insert_resource(fcs::core::trajectory::bullet_speed_data{.bullet_speed = 20.0});
    // 云台跟踪状态资源
    static_cast<void>(world.emplace_resource<fcs::core::FollowingState>());
}

// ====================== GTest 测试用例开始 ======================

/**
 * @brief 测试：磁盘剩余空间极低时，录制器仍会正常写入快照JSON日志
 * 原理：将预留最小磁盘字节设为uint64最大值，强制触发low_disk状态
 * 校验：snapshots.jsonl每行capture_status标记low_disk，且包含selected_target数据流
 */
TEST(Capturer, LowDiskStillWritesSnapshotRows) {
    const fs::path output_dir = make_temp_dir("low-disk");

    talos::Scheduler scheduler;
    seed_runtime_resources(scheduler.world());

    // 注册持续输出仿真数据流
    register_test_producer(scheduler, ProducerMode::Continuous);

    fcs::CapturerConfig config;
    config.enabled             = true;
    config.output_dir          = output_dir.string();
    // 预留磁盘空间设为最大值，模拟磁盘即将占满
    config.reserved_free_bytes = std::numeric_limits<uint64_t>::max();

    // 注册录制器系统到调度器
    fcs::runtime::register_runtime_capturer_system(scheduler, config, launch_context());

    // 运行2.2秒采集数据
    run_scheduler_for(scheduler, 2200ms);

    // 找到录制run子目录
    const fs::path run_dir = find_run_dir(output_dir);
    ASSERT_FALSE(run_dir.empty());

    // 读取快照日志
    const auto rows = read_jsonl(run_dir / "snapshots.jsonl");
    ASSERT_GE(rows.size(), 2u);

    // 逐行校验磁盘低水位标记
    for (const auto& row : rows) {
        EXPECT_EQ(row.at("capture_status"), "low_disk");
        ASSERT_TRUE(row.at("streams").contains("selected_target"));
    }

    // 清理临时目录
    fs::remove_all(output_dir);
}

/**
 * @brief 测试：录制启动时自动拷贝全部配置文件快照存入run目录run.json记录配置信息
 * 校验点：
 * 1. run.json存在config_snapshot字段，记录入口配置、拷贝文件数量
 * 2. 录制目录下配置快照与源码原始配置文件内容完全一致
 * 3. 相机录制策略空闲/活跃帧率参数正确持久化
 */
TEST(Capturer, RunMetadataStoresConfigSnapshotCopies) {
    const fs::path output_dir = make_temp_dir("config-snapshot");

    talos::Scheduler scheduler;
    seed_runtime_resources(scheduler.world());

    fcs::CapturerConfig config;
    config.enabled             = true;
    config.output_dir          = output_dir.string();
    config.reserved_free_bytes = 0;

    fcs::runtime::register_runtime_capturer_system(scheduler, config, launch_context());

    const fs::path run_dir = find_run_dir(output_dir);
    ASSERT_FALSE(run_dir.empty());

    // 读取录制元数据
    const auto run_json = read_json_file(run_dir / "run.json");
    ASSERT_TRUE(run_json.contains("config_snapshot"));

    const auto& config_snapshot = run_json.at("config_snapshot");
    // 入口配置文件路径校验
    EXPECT_EQ(config_snapshot.at("entry_source_path").get<std::string>(), "at_vision.toml");
    EXPECT_EQ(config_snapshot.at("entry_snapshot_path").get<std::string>(), "at_vision.toml");
    EXPECT_EQ(config_snapshot.at("config_source_dir").get<std::string>(), "config");
    EXPECT_EQ(config_snapshot.at("config_snapshot_dir").get<std::string>(), "config");
    // 至少拷贝1个配置文件
    EXPECT_GT(config_snapshot.at("copied_files").get<size_t>(), 0u);

    // 校验入口配置快照文件与源码一致
    const fs::path entry_snapshot =
        run_dir / config_snapshot.at("entry_snapshot_path").get<std::string>();
    ASSERT_TRUE(fs::exists(entry_snapshot));
    EXPECT_EQ(read_text_file(entry_snapshot), read_text_file(source_root() / "at_vision.toml"));

    // 预设需要拷贝的三份核心配置
    const std::array<fs::path, 3> expected_files{
        fs::path("config") / "vision_base.toml",
        fs::path("config") / "vision" / "std.toml",
        fs::path("config") / "robot" / "hero.toml",
    };

    // 逐一比对快照与源文件内容
    for (const auto& relative_path : expected_files) {
        const fs::path snapshot_path = run_dir / relative_path;
        ASSERT_TRUE(fs::exists(snapshot_path));
        EXPECT_EQ(read_text_file(snapshot_path), read_text_file(source_root() / relative_path));
    }

    // 校验相机录制帧率策略持久化
    ASSERT_TRUE(run_json.contains("camera_capture_policy"));
    EXPECT_EQ(run_json.at("camera_capture_policy").at("idle_hz"), 4);
    EXPECT_EQ(run_json.at("camera_capture_policy").at("active_hz"), 20);

    fs::remove_all(output_dir);
}

/**
 * @brief 测试：禁止录制输出目录嵌套在配置快照源目录内（防止递归拷贝配置文件死循环）
 * 场景：输出目录设置在config文件夹内部，录制器应直接拒绝创建目录
 */
TEST(Capturer, RejectsOutputDirectoryInsideConfigSnapshotSource) {
    const fs::path working_dir = make_temp_dir("nested-output");
    // 模拟项目config目录结构
    fs::create_directories(working_dir / "config");
    std::ofstream(working_dir / "at_vision.toml") << "";
    std::ofstream(working_dir / "config" / "vision_base.toml") << "";

    // 输出目录嵌套在config下
    const fs::path nested_output = working_dir / "config" / "record";
    {
        // RAII临时切换工作目录
        const ScopedCurrentPath cwd(working_dir);

        talos::Scheduler scheduler;
        seed_runtime_resources(scheduler.world());

        fcs::CapturerConfig config;
        config.enabled             = true;
        config.output_dir          = nested_output.string();
        config.reserved_free_bytes = 0;

        fcs::runtime::register_runtime_capturer_system(scheduler, config, launch_context());
        // 目录不会被创建
        EXPECT_FALSE(fs::exists(nested_output));
    }

    fs::remove_all(working_dir);
}

/**
 * @brief 测试：空闲模式下相机图像低帧率录制（4Hz），1.4s内图片数量3~6张
 */
TEST(Capturer, IdleModeWritesCameraAtLowRate) {
    const fs::path output_dir = make_temp_dir("idle-rate");

    talos::Scheduler scheduler;
    seed_runtime_resources(scheduler.world());

    register_test_producer(scheduler, ProducerMode::Continuous);

    fcs::CapturerConfig config;
    config.enabled             = true;
    config.output_dir          = output_dir.string();
    config.reserved_free_bytes = 0;

    fcs::runtime::register_runtime_capturer_system(scheduler, config, launch_context());

    run_scheduler_for(scheduler, 1400ms);

    const fs::path run_dir = find_run_dir(output_dir);
    ASSERT_FALSE(run_dir.empty());

    // 统计camera_开头图片数量
    const size_t idle_jpegs = count_jpegs(run_dir / "images", "camera_");
    EXPECT_GE(idle_jpegs, 3u);
    EXPECT_LE(idle_jpegs, 6u);

    fs::remove_all(output_dir);
}

/**
 * @brief 测试：云台跟踪模式激活，相机录制升频至20Hz，图片数量显著增多
 */
TEST(Capturer, FollowingModeRaisesCameraWriteRate) {
    const fs::path output_dir = make_temp_dir("following-rate");

    talos::Scheduler scheduler;
    seed_runtime_resources(scheduler.world());

    register_test_producer(scheduler, ProducerMode::FollowingActive);

    fcs::CapturerConfig config;
    config.enabled             = true;
    config.output_dir          = output_dir.string();
    config.reserved_free_bytes = 0;

    fcs::runtime::register_runtime_capturer_system(scheduler, config, launch_context());

    run_scheduler_for(scheduler, 1400ms);

    const fs::path run_dir = find_run_dir(output_dir);
    ASSERT_FALSE(run_dir.empty());

    const size_t follow_jpegs = count_jpegs(run_dir / "images", "camera_");
    // 高帧率区间8~24张
    EXPECT_GE(follow_jpegs, 8u);
    EXPECT_LE(follow_jpegs, 24u);

    fs::remove_all(output_dir);
}

/**
 * @brief 测试：开火模式下相机同样高帧率录制
 */
TEST(Capturer, FireModeRaisesCameraWriteRate) {
    const fs::path output_dir = make_temp_dir("fire-rate");

    talos::Scheduler scheduler;
    seed_runtime_resources(scheduler.world());

    register_test_producer(scheduler, ProducerMode::FireActive);

    fcs::CapturerConfig config;
    config.enabled             = true;
    config.output_dir          = output_dir.string();
    config.reserved_free_bytes = 0;

    fcs::runtime::register_runtime_capturer_system(scheduler, config, launch_context());

    run_scheduler_for(scheduler, 1400ms);

    const fs::path run_dir = find_run_dir(output_dir);
    ASSERT_FALSE(run_dir.empty());

    const size_t fire_jpegs = count_jpegs(run_dir / "images", "camera_");
    EXPECT_GE(fire_jpegs, 8u);
    EXPECT_LE(fire_jpegs, 24u);

    fs::remove_all(output_dir);
}

/**
 * @brief 测试：图片目录仅存储camera_前缀原始相机图，无其他无关图片
 */
TEST(Capturer, StoresOnlyOriginalCameraImages) {
    const fs::path output_dir = make_temp_dir("camera-only-images");

    talos::Scheduler scheduler;
    seed_runtime_resources(scheduler.world());

    register_test_producer(scheduler, ProducerMode::Continuous);

    fcs::CapturerConfig config;
    config.enabled             = true;
    config.output_dir          = output_dir.string();
    config.reserved_free_bytes = 0;

    fcs::runtime::register_runtime_capturer_system(scheduler, config, launch_context());

    run_scheduler_for(scheduler, 2200ms);

    const fs::path run_dir = find_run_dir(output_dir);
    ASSERT_FALSE(run_dir.empty());

    // 遍历所有图片文件名，必须全部以camera_开头
    for (const auto& filename : image_filenames(run_dir / "images")) {
        EXPECT_EQ(filename.rfind("camera_", 0), 0U);
    }

    fs::remove_all(output_dir);
}

/**
 * @brief 测试：检测流数据陈旧时，快照日志正确标记stale、updated_this_tick字段
 * 校验新旧帧ID不一致、图像文件路径不同、陈旧标记置true
 */
TEST(Capturer, StaleDetectorMetadataIsPreserved) {
    const fs::path output_dir = make_temp_dir("stale-detector");

    talos::Scheduler scheduler;
    seed_runtime_resources(scheduler.world());

    // 注册检测器陈旧工况生产者
    register_test_producer(scheduler, ProducerMode::StaleDetector);

    fcs::CapturerConfig config;
    config.enabled             = true;
    config.output_dir          = output_dir.string();
    config.reserved_free_bytes = 0;

    fcs::runtime::register_runtime_capturer_system(scheduler, config, launch_context());

    run_scheduler_for(scheduler, 3500ms);

    const fs::path run_dir = find_run_dir(output_dir);
    ASSERT_FALSE(run_dir.empty());

    const auto rows = read_jsonl(run_dir / "snapshots.jsonl");
    ASSERT_FALSE(rows.empty());

    bool found_stale_row = false;
    // 遍历快照寻找检测器陈旧条目
    for (const auto& row : rows) {
        const auto& camera   = row.at("streams").at("camera");
        const auto& detector = row.at("streams").at("detector");

        if (!camera.value("present", false) || !detector.value("present", false)) {
            continue;
        }
        // 相机帧ID=2、检测帧ID=1代表检测数据未更新
        if (camera.at("frame_id") != 2 || detector.at("frame_id") != 1) {
            continue;
        }
        if (!detector.at("stale").get<bool>()) {
            continue;
        }
        found_stale_row = true;
        // 本tick未更新检测流
        EXPECT_FALSE(detector.at("updated_this_tick").get<bool>());
        EXPECT_TRUE(detector.at("stale").get<bool>());
        // 相机图与检测缓存图文件不同
        if (camera.at("image_path").is_string() && detector.at("image_path").is_string()) {
            EXPECT_NE(
                camera.at("image_path").get<std::string>(),
                detector.at("image_path").get<std::string>());
        }
        break;
    }

    EXPECT_TRUE(found_stale_row);
    fs::remove_all(output_dir);
}

/**
 * @brief 测试：快照JSON中control_window二进制控制块元数据完整可校验
 * 校验：二进制文件存在、编码格式、采样数量、文件大小等于头+样本总字节
 */
TEST(Capturer, SnapshotReferencesRecordedControlWindow) {
    const fs::path output_dir = make_temp_dir("control-window");

    talos::Scheduler scheduler;
    seed_runtime_resources(scheduler.world());

    register_test_producer(scheduler, ProducerMode::Continuous);

    fcs::CapturerConfig config;
    config.enabled             = true;
    config.output_dir          = output_dir.string();
    config.reserved_free_bytes = 0;

    fcs::runtime::register_runtime_capturer_system(scheduler, config, launch_context());

    run_scheduler_for(scheduler, 2200ms);

    const fs::path run_dir = find_run_dir(output_dir);
    ASSERT_FALSE(run_dir.empty());

    const auto rows = read_jsonl(run_dir / "snapshots.jsonl");
    ASSERT_FALSE(rows.empty());
    const auto run_json = read_json_file(run_dir / "run.json");

    bool found_written_window = false;
    for (const auto& row : rows) {
        const auto& control_window = row.at("control_window");
        if (!control_window.at("written").get<bool>()) {
            continue;
        }

        found_written_window = true;
        EXPECT_EQ(control_window.at("frequency_hz"), 250);
        EXPECT_GT(control_window.at("sample_count").get<size_t>(), 0u);
        ASSERT_TRUE(control_window.at("path").is_string());
        ASSERT_TRUE(control_window.at("chunk_header_size").is_number_unsigned());
        ASSERT_TRUE(control_window.at("sample_size").is_number_unsigned());

        // 二进制控制文件路径
        const fs::path control_path = run_dir / control_window.at("path").get<std::string>();
        ASSERT_TRUE(fs::exists(control_path));
        EXPECT_GT(fs::file_size(control_path), 0u);
        // 小端显式编码
        EXPECT_EQ(control_window.at("encoding"), "little_endian_explicit");
        EXPECT_EQ(run_json.at("control").at("encoding"), "little_endian_explicit");
        // 文件字节数 = 块头大小 + 样本数*单样本字节
        EXPECT_EQ(
            fs::file_size(control_path),
            control_window.at("chunk_header_size").get<uint64_t>()
                + (control_window.at("sample_count").get<uint64_t>()
                   * control_window.at("sample_size").get<uint64_t>()));
        break;
    }

    EXPECT_TRUE(found_written_window);
    fs::remove_all(output_dir);
}

/**
 * @brief 测试：控制数据流停止发布后，快照标记resources.stale=true，复用最后一份有效样本时间戳
 */
TEST(Capturer, ControlResourcesGoStaleWhenPublisherIdles) {
    const fs::path output_dir = make_temp_dir("control-stale");

    talos::Scheduler scheduler;
    seed_runtime_resources(scheduler.world());

    // 注册前1.2s输出控制数据、之后停止的工况
    register_test_producer(scheduler, ProducerMode::ControlBurstThenIdle);

    fcs::CapturerConfig config;
    config.enabled             = true;
    config.output_dir          = output_dir.string();
    config.reserved_free_bytes = 0;

    fcs::runtime::register_runtime_capturer_system(scheduler, config, launch_context());

    run_scheduler_for(scheduler, 3500ms);

    const fs::path run_dir = find_run_dir(output_dir);
    ASSERT_FALSE(run_dir.empty());

    const auto rows = read_jsonl(run_dir / "snapshots.jsonl");
    ASSERT_FALSE(rows.empty());

    std::optional<uint64_t> last_fresh_sample_timestamp_ns;
    bool found_stale_row = false;
    for (const auto& row : rows) {
        const auto& resources = row.at("resources");
        if (!resources.value("present", false)) {
            continue;
        }

        const uint64_t sample_timestamp_ns = resources.at("sample_timestamp_ns").get<uint64_t>();
        // 本tick更新则刷新最新有效时间戳
        if (resources.at("updated_this_tick").get<bool>()) {
            last_fresh_sample_timestamp_ns = sample_timestamp_ns;
            continue;
        }
        // 陈旧且存在历史有效样本
        if (!resources.at("stale").get<bool>() || !last_fresh_sample_timestamp_ns.has_value()) {
            continue;
        }

        found_stale_row = true;
        // 陈旧样本复用上次新鲜时间戳
        EXPECT_EQ(sample_timestamp_ns, *last_fresh_sample_timestamp_ns);
        // 样本时间早于当前tick时间
        EXPECT_LT(sample_timestamp_ns, row.at("tick_timestamp_ns").get<uint64_t>());
        break;
    }

    EXPECT_TRUE(found_stale_row);
    fs::remove_all(output_dir);
}

/**
 * @brief 测试：控制资源快照正确携带IMU存在标记、时间戳、惯性数据字段
 */
TEST(Capturer, ControlResourcesExposeImuPresenceBitWhenPresent) {
    const fs::path output_dir = make_temp_dir("control-imu-present");

    talos::Scheduler scheduler;
    seed_runtime_resources(scheduler.world());

    register_test_producer(scheduler, ProducerMode::Continuous);

    fcs::CapturerConfig config;
    config.enabled             = true;
    config.output_dir          = output_dir.string();
    config.reserved_free_bytes = 0;

    fcs::runtime::register_runtime_capturer_system(scheduler, config, launch_context());

    run_scheduler_for(scheduler, 2200ms);

    const fs::path run_dir = find_run_dir(output_dir);
    ASSERT_FALSE(run_dir.empty());

    const auto rows = read_jsonl(run_dir / "snapshots.jsonl");
    ASSERT_FALSE(rows.empty());

    bool found_resources_row = false;
    for (const auto& row : rows) {
        const auto& resources = row.at("resources");
        if (!resources.value("present", false)) {
            continue;
        }

        found_resources_row = true;
        ASSERT_TRUE(resources.contains("imu_state"));
        const auto& imu_state = resources.at("imu_state");
        // IMU数据存在标记为true
        ASSERT_TRUE(imu_state.contains("present"));
        EXPECT_TRUE(imu_state.at("present").get<bool>());
        // IMU时间戳为无符号数字
        ASSERT_TRUE(imu_state.at("timestamp_ns").is_number_unsigned());
        break;
    }

    EXPECT_TRUE(found_resources_row);
    fs::remove_all(output_dir);
}

/**
 * @brief 底层SPMC多生产者单消费者通道单元测试
 * 验证通道generation计数可区分当前tick是否更新数据，用于上层判定stale/updated
 * 逻辑：
 * 1. 初始无数据，读取无值、未更新
 * 2. 写入42，读取有值、updated=true
 * 3. 不写入再次读取，仍有值、updated=false（数据陈旧）
 * 4. 写入43，再次updated=true
 */
TEST(Capturer, ChannelGenerationSamplingTracksUpdatedAndStale) {
    // 自定义空Topic标记用于测试通道
    struct TestTopic {};

    // 创建SPMC通道，拆分读写句柄
    auto channel = talos::primitive::make_spmc_channel<int>();
    auto split   = channel.split();
    auto& writer = split.writer;
    auto& reader = split.reader;

    // 包装读取器为带Topic的SPMC读取封装
    talos::spmc<int, TestTopic> wrapped_reader{.ptr_ = &reader};

    // 单次采样函数：返回(当前值, 是否本tick更新)
    const auto sample = [&]() {
        const auto previous_generation = wrapped_reader.last_generation();
        auto value                     = wrapped_reader.read_current();
        // 新世代号大于上次读取 = 本tick有新数据写入
        const bool updated_this_tick =
            value.has_value() && wrapped_reader.last_generation() > previous_generation;
        return std::pair{value, updated_this_tick};
    };

    // 首次采样无数据
    auto [initial_value, initial_updated] = sample();
    EXPECT_FALSE(initial_value.has_value());
    EXPECT_FALSE(initial_updated);

    // 写入42，采样判定更新
    writer.write(42);
    auto [fresh_value, fresh_updated] = sample();
    ASSERT_TRUE(fresh_value.has_value());
    EXPECT_EQ(*fresh_value, 42);
    EXPECT_TRUE(fresh_updated);

    // 不写入再次采样，数据存在但未更新（陈旧）
    auto [stale_value, stale_updated] = sample();
    ASSERT_TRUE(stale_value.has_value());
    EXPECT_EQ(*stale_value, 42);
    EXPECT_FALSE(stale_updated);

    // 写入新值43，再次判定更新
    writer.write(43);
    auto [next_value, next_updated] = sample();
    ASSERT_TRUE(next_value.has_value());
    EXPECT_EQ(*next_value, 43);
    EXPECT_TRUE(next_updated);
}

} // namespace fcs