// Google测试框架核心头文件，提供TEST/ASSERT_TRUE/EXPECT_EQ等断言宏
#include <gtest/gtest.h>

// C++标准通用库
#include <algorithm>
#include <chrono>        // 高精度时钟、超时等待
#include <cmath>         // 基础数学函数atan2/hypot等
#include <expected>      // C++23 错误处理预期类型，无异常数据流
#include <iostream>      // 标准控制台IO
#include <memory>        // 智能指针unique_ptr/shared_ptr
#include <mutex>         // 多线程互斥锁，测试夹具状态同步
#include <numbers>       // 标准数学常量 π
#include <optional>      // 可选空值容器，等待接口返回结果
#include <string_view>   // 只读字符串视图，无拷贝
#include <thread>        // 线程创建、休眠
#include <utility>       // std::move 移动语义
#include <vector>        // 动态数组容器

// ===================== 业务模块头文件：机器人五层架构 =====================
// L2感知：装甲ROI读取、图像预处理输出
#include "L2_perception/armor/readback_roi.hpp"
// L3估计：能量机关大符、简易大符、多目标跟踪器数据结构
#include "L3_estimation/energy_meter/types.hpp"
#include "L3_estimation/ldm_naive/types.hpp"
#include "L3_estimation/tracker/new_tracker.hpp"
#include "L3_estimation/tracker/types.hpp"
// L4规划：瞄准求解器、瞄准系统注册、有限状态机、控制输出、云台规划、选中目标快照、选择追踪日志
#include "L4_planning/aimer/aimer.hpp"
#include "L4_planning/aimer/aimer_systems.hpp"
#include "L4_planning/aimer/fsm.hpp"
#include "L4_planning/control_intent.hpp"
#include "L4_planning/gimbal_planner/types.hpp"
#include "L4_planning/selected_target_snapshot.hpp"
#include "L4_planning/target_selection_trace.hpp"
// L5武器：发射决策逻辑
#include "L5_weapon/fire_decision.hpp"

// 全局配置、运行时、时间工具、弹道求解、TF坐标系、调度器错误格式化、调度器核心
#include "camera_config.hpp"
#include "core/runtime.hpp"
#include "core/time.hpp"
#include "core/trajectory/resource.hpp"
#include "core/trajectory/solver/solver_interfaces.hpp"
#include "frame.hpp"
#include "scheduler/error_formatter.hpp"
#include "scheduler/scheduler.hpp"

/**
 * @namespace 全局匿名命名空间
 * 隔离测试工具类、工厂构造函数、测试夹具，仅本测试文件内部可见
 * 内容分为三大块：
 * 1. 模拟弹道求解器（正常Mock/报错Invalid）
 * 2. 批量工厂函数：快速构造TF变换、目标状态、装甲观测、相机配置、跟踪输出等测试数据
 * 3. AimerSystemHarness 完整ECS调度测试夹具：拉起真实调度器、模拟各路SPMC输入、捕获规划输出用于断言
 * 4. 下方TEST宏：单个单元测试，针对Aimer瞄准器分阶段逻辑校验
 */
namespace {

// 时间字面量简写 500ms/10ms
using namespace std::chrono_literals;

// =====================================================================================
// 1. Mock弹道求解器：纯几何直线无空气阻力，用于单元测试解耦复杂弹道迭代
// =====================================================================================
/**
 * @brief 模拟弹道求解器（无迭代、无空气阻力直线模型）
 * 真实业务弹道会迭代重力补偿，单元测试只关心角度输出逻辑，屏蔽复杂弹道计算干扰
 */
class MockTrajectorySolver final : public fcs::core::trajectory::solver::TrajectorySolver {
public:
    /**
     * @brief 直线弹道解算：仅计算水平/俯仰几何角，飞行距离/弹速
     * @param target_pos 目标odom三维坐标
     * @param v0 弹丸初速度
     * @return AimSolution 瞄准解：yaw/pitch/飞行时间/迭代次数
     */
    [[nodiscard]] std::expected<fcs::core::trajectory::solver::AimSolution, std::string>
        solve(const Eigen::Vector3d& target_pos, double v0) const noexcept override {
        const double distance = target_pos.norm();
        const double xy       = std::hypot(target_pos.x(), target_pos.y());
        // 水平偏航角 atan2(y,x)
        return fcs::core::trajectory::solver::AimSolution{
            .yaw            = std::atan2(target_pos.y(), target_pos.x()),
            .pitch          = std::atan2(target_pos.z(), xy),
            .time_of_flight = distance / std::max(v0, 1e-6), // 防止除0
            .iterations     = 1, // Mock仅单次计算，无迭代
        };
    }

    // 生成弹道轨迹曲线，测试不需要返回空
    [[nodiscard]] std::vector<std::pair<double, double>>
        generate_trajectory(double, double, double) const noexcept override {
        return {};
    }

    // 求解器名称，日志区分
    [[nodiscard]] std::string_view solver_name() const noexcept override { return "mock"; }

    // 无真实弹道模型
    [[nodiscard]] const fcs::core::trajectory::model::BallisticModel*
        get_model() const noexcept override {
        return nullptr;
    }
};

/**
 * @brief 报错弹道求解器：solve永远返回失败，用于测试瞄准器异常容错逻辑
 */
class InvalidTrajectorySolver final : public fcs::core::trajectory::solver::TrajectorySolver {
public:
    [[nodiscard]] std::expected<fcs::core::trajectory::solver::AimSolution, std::string>
        solve(const Eigen::Vector3d&, double) const noexcept override {
        return std::unexpected("InvalidTrajectorySolver: always fails");
    }

    [[nodiscard]] std::vector<std::pair<double, double>>
        generate_trajectory(double, double, double) const noexcept override {
        return {};
    }

    [[nodiscard]] std::string_view solver_name() const noexcept override { return "invalid"; }

    [[nodiscard]] const fcs::core::trajectory::model::BallisticModel*
        get_model() const noexcept override {
        return nullptr;
    }
};

// =====================================================================================
// 2. 批量工厂构造函数：快速生成各类测试用空/标准结构体，减少测试样板代码
// =====================================================================================
// 生成云台单位变换（无平移无旋转）
fcs::L4::Aimer::GimbalTransform make_gimbal() {
    return fcs::L4::Aimer::GimbalTransform::from_rpy(0.0, 0.0, 0.0);
}

// 生成枪口零偏移变换
fcs::L4::Aimer::MuzzleTransform make_muzzle() {
    return fcs::L4::Aimer::MuzzleTransform::from_translation(0.0, 0.0, 0.0);
}

// 生成带自定义xyz偏移的枪口变换
fcs::L4::Aimer::MuzzleTransform make_muzzle(double x, double y, double z) {
    return fcs::L4::Aimer::MuzzleTransform::from_translation(x, y, z);
}

/**
 * @brief 构造机器人整车目标状态（4装甲标准小车）
 * @param yaw 车身偏航角
 * @param v_yaw 车身旋转角速度，默认0
 * @return RobotTargetState 固定坐标(1,0,0)，前后装甲半径、高度预设
 */
fcs::L3::RobotTargetState make_robot_target(double yaw, double v_yaw = 0.0) {
    fcs::L3::RobotTargetState target;
    target.position   = Eigen::Vector3d(1.0, 0.0, 0.0);
    target.velocity   = Eigen::Vector3d::Zero();
    target.yaw        = yaw;
    target.v_yaw      = v_yaw;
    target.radius0    = 0.2;
    target.radius1    = 0.3;
    target.z1         = 0.4;
    target.armors_num = 4;
    return target;
}

/**
 * @brief 构造前哨站2D平面目标
 * @param yaw 前哨旋转角
 * @param v_yaw 角速度默认0
 */
fcs::L3::OutpostTargetState make_outpost_target(double yaw, double v_yaw = 0.0) {
    fcs::L3::OutpostTargetState target;
    target.position = Eigen::Vector2d(1.0, 0.0);
    target.velocity = Eigen::Vector3d::Zero();
    target.yaw      = yaw;
    target.v_yaw    = v_yaw;
    target.z        = {0.0, 0.0, 0.0};
    return target;
}

/**
 * @brief 生成单帧装甲观测测量值
 * @param armor_pose 装甲位姿 [x,y,z,yaw]
 * @param name 装甲类型编号
 * @param image_center_distance 装甲离图像中心距离，用于选择权重
 */
fcs::ArmorMeasurement make_measurement(
    const Eigen::Vector4d& armor_pose, fcs::ArmorName name, float image_center_distance = 0.0f) {
    fcs::ArmorMeasurement measurement;
    measurement.transform = fcs::ArmorMeasurement::Transform::from_rpy(
        0.0, 0.0, armor_pose[3], armor_pose[0], armor_pose[1], armor_pose[2]);
    measurement.name                     = name;
    measurement.color                    = fcs::ArmorColor::Blue;
    measurement.type                     = fcs::ArmorType::Small;
    measurement.confidence               = 1.0f;
    measurement.distance_to_image_center = image_center_distance;
    measurement.timestamp_ns             = 0;
    return measurement;
}

/**
 * @brief 生成完整静态TF坐标系树（所有变换单位矩阵，无机械外参偏移）
 * 测试统一基准，消除标定外参干扰
 */
fast_tf::CoordinateSystem make_test_coordinate_system() {
    auto system = fast_tf::CoordinateSystem();
    fast_tf::update<fast_tf::odom>(system, {}, 0);
    fast_tf::update<fast_tf::gimbal_yaw_fuxk_frame>(system, {}, 0);
    fast_tf::update(system, fast_tf::EdgeTransform<fast_tf::gimbal_pitch_fuxk_frame>::from_translation(0, 0, 0),0);
    fast_tf::update(system, fast_tf::EdgeTransform<fast_tf::camera_link_fuxk_frame>::from_translation(0, 0, 0),0);
    fast_tf::update(system, fast_tf::EdgeTransform<fast_tf::camera_optical_fuxk_frame>::from_translation(0, 0, 0), 0);
    fast_tf::update(system, fast_tf::EdgeTransform<fast_tf::muzzle_link_fuxk_frame>::from_translation(0, 0, 0),0);
    return system;
}

/**
 * @brief 生成简易相机内参矩阵 fx=100 fy=100 cx=50 cy=60，无畸变
 */
fcs::CameraConfig make_test_camera_config() {
    fcs::CameraConfig config;
    config.camera_matrix << 100.0, 0.0, 50.0,
                            0.0, 100.0, 60.0,
                            0.0, 0.0, 1.0;
    return config;
}

/**
 * @brief 构造单目标跟踪器输出结果
 * @param name 装甲ID
 * @param status 跟踪状态Idle/Detecting/Tracking/TempLost
 * @param position 目标odom坐标
 * @param image_center_distance_px 图像中心像素距离
 * @param color 队伍颜色默认蓝方
 * @param last_observation_timestamp_ns 最后观测时间戳
 */
fcs::L3::TrackerOutput make_robot_tracker_output(
    fcs::ArmorName name, fcs::L3::TrackerStatus status, const Eigen::Vector3d& position,
    double image_center_distance_px, fcs::ArmorColor color = fcs::ArmorColor::Blue,
    uint64_t last_observation_timestamp_ns = 0) {
    fcs::L3::TrackerOutput tracker;
    tracker.status                        = status;
    tracker.target_name                   = name;
    tracker.target_color                  = color;
    tracker.target_jumped                 = false;
    tracker.last_armor_id                 = 0;
    tracker.last_image_center_distance_px = image_center_distance_px;
    tracker.last_observation_timestamp_ns = last_observation_timestamp_ns;

    fcs::L3::RobotTargetState state;
    state.position   = position;
    state.velocity   = Eigen::Vector3d::Zero();
    state.yaw        = 0.0;
    state.v_yaw      = 0.0;
    state.radius0    = 0.2;
    state.radius1    = 0.2;
    state.z1         = position.z();
    state.armors_num = 4;
    tracker.state    = state;
    return tracker;
}

// =====================================================================================
// 3. AimerSystemHarness 完整ECS调度测试夹具
// 用途：拉起真实talos调度器，注册全套瞄准相关系统，模拟各路输入SPMC，提供阻塞等待接口捕获输出
// 用于集成测试整套L4瞄准流水线，区别于下方纯Aimer类单元测试
// =====================================================================================
class AimerSystemHarness {
public:
    /**
     * @brief 夹具构造函数：初始化调度器、插入全局资源、注册所有瞄准相关生产者/消费者系统
     * @param config L4瞄准器配置
     * @param readback_roi_config ROI读取配置
     */
    explicit AimerSystemHarness(
        fcs::L4::L4Config config = {}, fcs::L2::ArmorReadbackRoiConfig readback_roi_config = {}) {
        // 插入全局静态TF、相机配置、ROI配置
        scheduler_.world().insert_resource(make_test_coordinate_system());
        scheduler_.world().insert_resource(make_test_camera_config());
        scheduler_.world().insert_resource(readback_roi_config);
        // 注入Mock弹道求解器
        scheduler_.world().insert_resource(
            std::unique_ptr<fcs::core::trajectory::solver::TrajectorySolver>(
                std::make_unique<MockTrajectorySolver>()));
        // 弹丸初速全局资源 30m/s
        scheduler_.world().insert_resource(
            fcs::core::trajectory::bullet_speed_data{.bullet_speed = 30.0});
        // 跟随模式开关全局资源（优先大符/装甲）
        static_cast<void>(scheduler_.world().emplace_resource<fcs::core::FollowingState>());

        // 注册全套瞄准规划业务系统
        fcs::L4::register_aimer_systems(scheduler_, std::move(config));

        // ---------------------- 模拟输入生产者系统（200Hz固定周期） ----------------------
        // 1. 跟踪器输出模拟生产者：外部设置trackers，每200Hz写入SPMC通道给瞄准器读取
        scheduler_.add_system<talos::fixed_rate<200>>(
            "aimer_system_test_tracker_producer",
            [tracker_state = tracker_state_](
                talos::spmc_mut<fcs::L3::TrackerOutputs, fcs::TrackerOutputChannelTopic>
                    tracker_out) {
                fcs::L3::TrackerOutputs trackers;
                {
                    std::scoped_lock lock(tracker_state->mutex);
                    trackers = tracker_state->trackers;
                }

                const uint64_t now_ns = fcs::clock::now_ns();
                // 统一填充当前时间戳
                for (auto& tracker : trackers) {
                    tracker.timestamp_ns = now_ns;
                    if (tracker.status == fcs::L3::TrackerStatus::Tracking) {
                        tracker.last_observation_timestamp_ns = now_ns;
                    } else if (tracker.last_observation_timestamp_ns == 0) {
                        tracker.last_observation_timestamp_ns = now_ns;
                    }
                }
                tracker_out.write(std::move(trackers));
            });

        // 2. 能量机关大符模拟生产者：默认空跟踪状态
        scheduler_.add_system<talos::fixed_rate<200>>(
            "aimer_system_test_rune_producer",
            [](talos::spmc_mut<
                fcs::energy_meter::EnergyMeterState, fcs::EnergyMeterStateChannelTopic>
                   rune_out) {
                fcs::energy_meter::EnergyMeterState state;
                state.timestamp_ns   = fcs::clock::now_ns();
                state.tracking_valid = false;
                rune_out.write(std::move(state));
            });

        // 3. 简易大符LDM生产者：空闲无目标
        scheduler_.add_system<talos::fixed_rate<200>>(
            "aimer_system_test_ldm_producer", [](talos::spmc_mut<fcs::L3::ldm::LdmState> ldm_out) {
                fcs::L3::ldm::LdmState state;
                state.timestamp_ns = fcs::clock::now_ns();
                state.status       = fcs::L3::TrackerStatus::Idle;
                ldm_out.write(std::move(state));
            });

        // ---------------------- 输出捕获消费者系统（200Hz） ----------------------
        // 1. 捕获控制规划输出ControlIntent，存入线程安全plan_state
        scheduler_.add_system<talos::fixed_rate<200>>(
            "aimer_system_test_plan_consumer",
            [plan_state = plan_state_](
                talos::spmc<fcs::L4::ControlIntent, fcs::ControlIntentChannelTopic> plan_in) {
                const auto plan = plan_in.read_current();
                if (!plan) {
                    return;
                }
                std::scoped_lock lock(plan_state->mutex);
                plan_state->plan = *plan;
            });

        // 2. 捕获选中目标快照SelectedTargetSnapshot
        scheduler_.add_system<talos::fixed_rate<200>>(
            "aimer_system_test_selected_target_consumer",
            [selected_target_state = selected_target_state_](
                talos::spmc<
                    fcs::L4::SelectedTargetSnapshot, fcs::SelectedTargetSnapshotChannelTopic>
                    selected_target_in) {
                const auto selected_target = selected_target_in.read_current();
                if (!selected_target) {
                    return;
                }
                std::scoped_lock lock(selected_target_state->mutex);
                selected_target_state->snapshot = *selected_target;
            });

        // 3. 捕获目标选择日志Trace，用于调试选择逻辑
        scheduler_.add_system<talos::fixed_rate<200>>(
            "aimer_system_test_trace_consumer",
            [trace_state = trace_state_](
                talos::spmc<fcs::L4::TargetSelectionTrace, fcs::TargetSelectionTraceChannelTopic>
                    trace_in) {
                const auto trace = trace_in.read_current();
                if (!trace) {
                    return;
                }
                std::scoped_lock lock(trace_state->mutex);
                trace_state->trace = *trace;
            });
    }

    /**
     * @brief 构建调度拓扑，启动后台调度线程
     */
    void start() {
        const auto build_result = scheduler_.build();
        ASSERT_TRUE(build_result.has_value())
            << "scheduler build failed: " << fmt::format("{}", build_result.error());

        // 后台线程运行调度循环
        scheduler_thread_ = std::thread([this]() {
            if (const auto result = scheduler_.run(); !result) {
                run_error_ = result.error();
            }
        });
    }

    /**
     * @brief 停止调度器，阻塞等待线程退出，校验无运行错误
     */
    void stop() {
        scheduler_.stop();
        if (scheduler_thread_.joinable()) {
            scheduler_thread_.join();
        }
        ASSERT_FALSE(run_error_.has_value());
    }

    /**
     * @brief 析构自动停止调度器，防止资源泄漏
     */
    ~AimerSystemHarness() {
        scheduler_.stop();
        if (scheduler_thread_.joinable()) {
            scheduler_thread_.join();
        }
    }

    /**
     * @brief 外部测试用例设置跟踪目标列表，清空历史输出缓存
     * @param trackers 批量跟踪器输出数组
     */
    void set_trackers(fcs::L3::TrackerOutputs trackers) {
        {
            std::scoped_lock lock(tracker_state_->mutex);
            tracker_state_->trackers = std::move(trackers);
        }
        // 清空上次规划、选中目标、选择日志缓存
        {
            std::scoped_lock lock(plan_state_->mutex);
            plan_state_->plan.reset();
        }
        {
            std::scoped_lock lock(selected_target_state_->mutex);
            selected_target_state_->snapshot.reset();
        }
        {
            std::scoped_lock lock(trace_state_->mutex);
            trace_state_->trace.reset();
        }
    }

    /**
     * @brief 设置全局跟随模式：true优先大符，false优先装甲
     */
    void set_following(bool active) {
        scheduler_.world().get_res_mut<fcs::core::FollowingState>()->store(active);
    }

    /**
     * @brief 阻塞等待规划输出ControlIntent，带超时防止死锁
     * @param timeout 超时毫秒，默认500ms
     * @return 规划指令可选空值
     */
    std::optional<fcs::L4::ControlIntent>
        wait_for_plan(std::chrono::milliseconds timeout = 500ms) const {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::scoped_lock lock(plan_state_->mutex);
                if (plan_state_->plan.has_value()) {
                    return plan_state_->plan;
                }
            }
            std::this_thread::sleep_for(10ms);
        }
        return std::nullopt;
    }

    /**
     * @brief 等待选中指定装甲ID的目标快照
     * @param target_name 装甲编号
     * @param timeout 超时750ms
     * @return 目标快照，超时返回最后一帧或空
     */
    std::optional<fcs::L4::SelectedTargetSnapshot> wait_for_target(
        fcs::ArmorName target_name, std::chrono::milliseconds timeout = 750ms) const {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        std::optional<fcs::L4::SelectedTargetSnapshot> last_snapshot;
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::scoped_lock lock(selected_target_state_->mutex);
                if (selected_target_state_->snapshot.has_value()) {
                    last_snapshot = selected_target_state_->snapshot;
                    if (selected_target_state_->snapshot->has_target()
                        && selected_target_state_->snapshot->tracker.target_name == target_name) {
                        return selected_target_state_->snapshot;
                    }
                }
            }
            std::this_thread::sleep_for(10ms);
        }
        return last_snapshot;
    }

    /**
     * @brief 等待时间戳大于指定值的新选中目标快照，用于时序隔离测试
     */
    std::optional<fcs::L4::SelectedTargetSnapshot> wait_for_selected_target_after(
        uint64_t min_timestamp_ns, std::chrono::milliseconds timeout = 750ms) const {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        std::optional<fcs::L4::SelectedTargetSnapshot> last_snapshot;
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::scoped_lock lock(selected_target_state_->mutex);
                if (selected_target_state_->snapshot.has_value()
                    && selected_target_state_->snapshot->timestamp_ns > min_timestamp_ns) {
                    last_snapshot = selected_target_state_->snapshot;
                    return selected_target_state_->snapshot;
                }
            }
            std::this_thread::sleep_for(10ms);
        }
        return last_snapshot;
    }

    /**
     * @brief 等待装甲跟踪/射击类规划指令，排除Hold保持指令，要求时间戳大于给定值
     */
    std::optional<fcs::L4::ControlIntent> wait_for_armor_plan_after(
        uint64_t min_timestamp_ns, std::chrono::milliseconds timeout = 750ms) const {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        std::optional<fcs::L4::ControlIntent> last_plan;
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::scoped_lock lock(plan_state_->mutex);
                if (plan_state_->plan.has_value()) {
                    // std::visit 多态匹配三种指令Track/Shot/Hold
                    const bool is_armor = std::visit(
                        [min_timestamp_ns](const auto& cmd) {
                            using T = std::decay_t<decltype(cmd)>;
                            if constexpr (std::is_same_v<T, fcs::L4::TrackCommand>)
                                return cmd.timestamp_ns > min_timestamp_ns;
                            if constexpr (std::is_same_v<T, fcs::L4::ShotCommand>)
                                return cmd.timestamp_ns > min_timestamp_ns;
                            if constexpr (std::is_same_v<T, fcs::L4::HoldCommand>)
                                return false;
                        },
                        *plan_state_->plan);
                    if (is_armor) {
                        last_plan = plan_state_->plan;
                    }
                }
            }
            std::this_thread::sleep_for(10ms);
        }
        return last_plan;
    }

    /**
     * @brief 等待目标选择日志Trace，要求包含候选目标列表、时间戳大于阈值
     */
    std::optional<fcs::L4::TargetSelectionTrace> wait_for_selection_trace_after(
        uint64_t min_timestamp_ns, std::chrono::milliseconds timeout = 750ms) const {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        std::optional<fcs::L4::TargetSelectionTrace> last_trace;
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::scoped_lock lock(trace_state_->mutex);
                if (trace_state_->trace.has_value()
                    && trace_state_->trace->timestamp_ns > min_timestamp_ns) {
                    last_trace = trace_state_->trace;
                    if (!trace_state_->trace->candidates.empty()) {
                        return trace_state_->trace;
                    }
                }
            }
            std::this_thread::sleep_for(10ms);
        }
        return last_trace;
    }

    /**
     * @brief 等待L2 ROI回读缓存快照有效
     */
    std::optional<fcs::L2::TrackerReadbackSnapshot>
        wait_for_readback_tracker(std::chrono::milliseconds timeout = 750ms) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto snapshot =
                scheduler_.world().get_res<fcs::L2::TrackerReadbackCache>()->load();
            if (snapshot.valid) {
                return snapshot;
            }
            std::this_thread::sleep_for(10ms);
        }
        return std::nullopt;
    }

private:
    // 多线程共享状态结构体，带互斥锁保护
    struct TrackerState {
        mutable std::mutex mutex;
        fcs::L3::TrackerOutputs trackers;
    };
    struct PlanState {
        mutable std::mutex mutex;
        std::optional<fcs::L4::ControlIntent> plan;
    };
    struct SelectedTargetState {
        mutable std::mutex mutex;
        std::optional<fcs::L4::SelectedTargetSnapshot> snapshot;
    };
    struct TraceState {
        mutable std::mutex mutex;
        std::optional<fcs::L4::TargetSelectionTrace> trace;
    };

    talos::Scheduler scheduler_{};
    std::shared_ptr<TrackerState> tracker_state_{std::make_shared<TrackerState>()};
    std::shared_ptr<PlanState> plan_state_{std::make_shared<PlanState>()};
    std::shared_ptr<SelectedTargetState> selected_target_state_{std::make_shared<SelectedTargetState>()};
    std::shared_ptr<TraceState> trace_state_{std::make_shared<TraceState>()};
    std::optional<talos::SchedulerError> run_error_;
    std::thread scheduler_thread_;
};

} // 匿名命名空间结束

// =====================================================================================
// 4. 单元测试 TEST(AimerPhase, xxx)：纯 Aimer::aim() 函数单元测试
// 不拉起完整调度器，直接实例化Aimer类，输入目标/上下文，断言输出选中装甲ID，分阶段校验瞄准选择逻辑
// 测试覆盖Aimer有限状态机所有瞄准阶段：SingleArmor / WholeCarArmor / WholeCarPair / WholeCarCenter
// =====================================================================================

/**
 * @test SingleArmor阶段：锁定指定对称装甲，上下文preferred_armor_id决定输出ID
 * 业务逻辑：单装甲锁定模式，强制使用传入的优先装甲编号
 */
TEST(AimerPhase, SingleArmorLocksSymmetricCandidates) {
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    MockTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    const auto muzzle = make_muzzle();
    const auto target = make_robot_target(-std::numbers::pi / 4.0);

    // 锁定0号装甲
    fcs::L4::ArmorAimContext lock_left;
    lock_left.target_jumped      = true;
    lock_left.phase              = fcs::L4::ArmorAimPhase::SingleArmor;
    lock_left.preferred_armor_id = 0;
    const auto left = aimer.aim(target, lock_left, gimbal, muzzle, 0, 0, 0.0, 30.0, solver);
    ASSERT_TRUE(left.has_value()) << left.error();
    EXPECT_EQ(left->selected_armor_id, 0);

    // 锁定1号装甲
    fcs::L4::ArmorAimContext lock_right = lock_left;
    lock_right.preferred_armor_id       = 1;
    const auto right = aimer.aim(target, lock_right, gimbal, muzzle, 0, 0, 0.0, 30.0, solver);
    ASSERT_TRUE(right.has_value()) << right.error();
    EXPECT_EQ(right->selected_armor_id, 1);
}

/**
 * @test WholeCarArmor整车装甲模式：根据车身角速度选择最小预测误差装甲
 * 业务逻辑：车身正向旋转选0号，反向选1号，忽略旧优先装甲
 */
TEST(AimerPhase, WholeCarArmorUsesPredictedMinDeltaSelection) {
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    MockTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    const auto muzzle = make_muzzle();

    fcs::L4::ArmorAimContext context;
    context.target_jumped = true;
    context.phase         = fcs::L4::ArmorAimPhase::WholeCarArmor;

    // 车身正旋 选0
    const auto positive = aimer.aim(
        make_robot_target(-std::numbers::pi / 4.0, 8.0), context, gimbal, muzzle, 0, 0, 0.0, 30.0,
        solver);
    ASSERT_TRUE(positive.has_value()) << positive.error();
    EXPECT_EQ(positive->selected_armor_id, 0);

    // 车身负旋 选1
    const auto negative = aimer.aim(
        make_robot_target(-std::numbers::pi / 4.0, -8.0), context, gimbal, muzzle, 0, 0, 0.0, 30.0,
        solver);
    ASSERT_TRUE(negative.has_value()) << negative.error();
    EXPECT_EQ(negative->selected_armor_id, 1);
}

/**
 * @test WholeCarArmor忽略过期preferred_armor，完全依靠角速度预测
 */
TEST(AimerPhase, WholeCarArmorIgnoresStalePreferredArmor) {
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    MockTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    const auto muzzle = make_muzzle();

    fcs::L4::ArmorAimContext context;
    context.target_jumped      = true;
    context.phase              = fcs::L4::ArmorAimPhase::WholeCarArmor;
    context.preferred_armor_id = 2; // 强行设置优先2，预期被忽略

    const auto prediction = aimer.aim(
        make_robot_target(-25.0 * std::numbers::pi / 180.0, -8.0), context, gimbal, muzzle, 0, 0,
        0.0, 30.0, solver);
    ASSERT_TRUE(prediction.has_value()) << prediction.error();
    EXPECT_NE(prediction->selected_armor_id, 2);
    EXPECT_EQ(prediction->selected_armor_id, 0);
}

/**
 * @test WholeCarPair上下装甲对：车身有高度差优先上层装甲ID=1
 */
TEST(AimerPhase, WholeCarPairUsesHighLayerWhenTargetHasHeight) {
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    MockTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    const auto muzzle = make_muzzle();

    fcs::L4::ArmorAimContext context;
    context.target_jumped = true;
    context.phase         = fcs::L4::ArmorAimPhase::WholeCarPair;

    const auto prediction = aimer.aim(
        make_robot_target(-std::numbers::pi / 4.0, 8.0), context, gimbal, muzzle, 0, 0, 0.0, 30.0,
        solver);
    ASSERT_TRUE(prediction.has_value()) << prediction.error();
    EXPECT_EQ(prediction->selected_armor_id, 1);
}

/**
 * @test WholeCarPair车身扁平无高度，优先下层装甲ID=0
 */
TEST(AimerPhase, WholeCarPairUsesLowLayerWhenTargetIsFlat) {
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    MockTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    const auto muzzle = make_muzzle();

    auto target    = make_robot_target(-std::numbers::pi / 4.0, 8.0);
    target.z1      = target.position.z();
    target.radius1 = target.radius0; // 上下装甲同高，扁平车身

    fcs::L4::ArmorAimContext context;
    context.target_jumped = true;
    context.phase         = fcs::L4::ArmorAimPhase::WholeCarPair;

    const auto prediction = aimer.aim(target, context, gimbal, muzzle, 0, 0, 0.0, 30.0, solver);
    ASSERT_TRUE(prediction.has_value()) << prediction.error();
    EXPECT_EQ(prediction->selected_armor_id, 0);
}

/**
 * @test WholeCarCenter整车中心模式：背对云台选底层装甲ID=2
 */
TEST(AimerPhase, WholeCarCenterLowLayerKeepsWholeCarSelection) {
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    MockTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    const auto muzzle = make_muzzle();

    fcs::L4::ArmorAimContext context;
    context.target_jumped = true;
    context.phase         = fcs::L4::ArmorAimPhase::WholeCarCenter;

    const auto target = make_robot_target(std::numbers::pi, 12.0);

    const auto prediction = aimer.aim(target, context, gimbal, muzzle, 0, 0, 0.0, 30.0, solver);
    ASSERT_TRUE(prediction.has_value()) << prediction.error();
    EXPECT_EQ(prediction->selected_armor_id, 2);
}

/**
 * @test WholeCarCenter面朝云台选上层装甲ID=1
 */
TEST(AimerPhase, WholeCarCenterHighLayerKeepsWholeCarSelection) {
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    MockTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    const auto muzzle = make_muzzle();

    fcs::L4::ArmorAimContext context;
    context.target_jumped = true;
    context.phase         = fcs::L4::ArmorAimPhase::WholeCarCenter;

    const auto prediction = aimer.aim(
        make_robot_target(-std::numbers::pi / 2.0, 12.0), context, gimbal, muzzle, 0, 0, 0.0, 30.0,
        solver);
    ASSERT_TRUE(prediction.has_value()) << prediction.error();
    EXPECT_EQ(prediction->selected_armor_id, 1);
}

/**
 * @test WholeCarCenter无视过期preferred_armor，依据车身朝向选择装甲
 */
TEST(AimerPhase, WholeCarCenterUsesCurrentActiveArmorInsteadOfStalePreferredLayer) {
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    MockTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    const auto muzzle = make_muzzle();

    fcs::L4::ArmorAimContext context;
    context.target_jumped      = true;
    context.phase              = fcs::L4::ArmorAimPhase::WholeCarCenter;
    context.preferred_armor_id = 1;

    const auto prediction = aimer.aim(
        make_robot_target(std::numbers::pi, 12.0), context, gimbal, muzzle, 0, 0, 0.0, 30.0,
        solver);
    ASSERT_TRUE(prediction.has_value()) << prediction.error();
    EXPECT_EQ(prediction->selected_armor_id, 2);
}

/**
 * @test WholeCarCenter前哨站目标，选择ID=1装甲
 */
TEST(AimerPhase, WholeCarCenterOutpostUsesAllArmors) {
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    MockTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    const auto muzzle = make_muzzle();

    fcs::L4::ArmorAimContext context;
    context.target_jumped      = true;
    context.phase              = fcs::L4::ArmorAimPhase::WholeCarCenter;
    context.preferred_armor_id = 0;

    const auto prediction = aimer.aim(
        make_outpost_target(-2.0 * std::numbers::pi / 3.0, 12.0), context, gimbal, muzzle, 0, 0,
        0.0, 30.0, solver);
    ASSERT_TRUE(prediction.has_value()) << prediction.error();
    EXPECT_EQ(prediction->selected_armor_id, 1);
}

/**
 * @test WholeCarCenter带枪口偏移投影校验：装甲坐标沿枪口射线投影，距离、坐标数值断言
 * 校验坐标投影、飞行距离计算精度
 */
TEST(AimerPhase, WholeCarCenterProjectsAlongMuzzleRay) {
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    MockTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    // 枪口x=1向前偏移
    const auto muzzle = make_muzzle(1.0, 0.0, 0.0);

    auto target     = make_robot_target(std::atan2(5.0, 1.0));
    target.position = Eigen::Vector3d(1.0, 5.0, 0.0);
    target.velocity = Eigen::Vector3d::Zero();
    target.radius0  = 0.2;
    target.radius1  = 0.3;
    target.z1       = 0.4;

    fcs::L4::ArmorAimContext context;
    context.target_jumped      = true;
    context.phase              = fcs::L4::ArmorAimPhase::WholeCarCenter;
    context.preferred_armor_id = 1;

    const auto prediction = aimer.aim(target, context, gimbal, muzzle, 0, 0, 0.0, 30.0, solver);
    ASSERT_TRUE(prediction.has_value()) << prediction.error();
    EXPECT_EQ(prediction->selected_armor_id, 0);
    // 浮点精度断言 1e-6容差
    EXPECT_NEAR(prediction->predicted_position.x(), 1.0, 1e-6);
    EXPECT_NEAR(prediction->predicted_position.y(), 4.8, 1e-6);
    EXPECT_NEAR(prediction->predicted_position.z(), target.position.z(), 1e-6);
    EXPECT_NEAR(prediction->distance, 4.8, 1e-6);
}

/**
 * @test 飞行时间精细化修正：跨装甲切换时使用最终选中装甲计算飞行时间，误差在容差内
 */
TEST(AimerPhase, FlightTimeTracksFinalArmorAcrossHandoff) {
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    MockTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    const auto muzzle = make_muzzle();

    fcs::L4::ArmorAimContext context;
    context.target_jumped = true;
    context.phase         = fcs::L4::ArmorAimPhase::SingleArmor;

    // 弹速0.6m/s慢速，放大飞行时间误差便于校验
    const auto prediction =
        aimer.aim(make_robot_target(-3.0, -2.0), context, gimbal, muzzle, 0, 0, 0.0, 0.6, solver);
    ASSERT_TRUE(prediction.has_value()) << prediction.error();
    EXPECT_NE(prediction->rough_selected_armor_id, prediction->selected_armor_id);
    // 飞行时间 = 目标距离 / 弹速，在允许容差内
    EXPECT_NEAR(
        prediction->flying_time, prediction->predicted_position.norm() / 0.6,
        fcs::L4::kFlyingTimeRefineTolerance);
}

TEST(AimerPhase, TrackerJumpedLatchesWhenAnyVisibleArmorIsNonZero) {
    fcs::L3::TrackerConfig config;
    config.robot.lost_threshold     = 1.0;
    config.robot.tracking_threshold = 0;
    config.robot.matcher_gate       = 100.0;

    fcs::L3::TrackerNew tracker(config);

    fcs::ArmorMeasurementBatch first_batch;
    first_batch.measurements.push_back(
        make_measurement(Eigen::Vector4d{0.8, 0.0, 0.0, 0.0}, fcs::ArmorName::Three, 0.0f));
    ASSERT_TRUE(tracker.first_meet(first_batch).has_value());

    const auto first_output = tracker.get_output();
    ASSERT_TRUE(first_output.robot_state() != nullptr);
    EXPECT_FALSE(first_output.target_jumped);

    const auto armor_poses = first_output.robot_state()->armor_poses();

    fcs::ArmorMeasurementBatch update_batch;
    update_batch.measurements.push_back(make_measurement(armor_poses[0], fcs::ArmorName::Three));
    update_batch.measurements.push_back(make_measurement(armor_poses[1], fcs::ArmorName::Three));

    ASSERT_TRUE(tracker.update(update_batch));

    const auto output = tracker.get_output();
    ASSERT_TRUE(output.robot_state() != nullptr);
    EXPECT_TRUE(output.target_jumped);
}

TEST(AimerPhase, BallisticFailureFailsClosed) {
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    InvalidTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    const auto muzzle = make_muzzle();

    fcs::L4::ArmorAimContext context;
    context.target_jumped = true;
    context.phase         = fcs::L4::ArmorAimPhase::SingleArmor;

    const auto prediction = aimer.aim(
        make_robot_target(-std::numbers::pi / 4.0), context, gimbal, muzzle, 0, 0, 0.0, 30.0,
        solver);
    EXPECT_FALSE(prediction.has_value());
}

TEST(AimerPhase, TrackerArmorVyawTracksSinusoidalOmegaInput) {
    constexpr double kPi       = std::numbers::pi;
    constexpr double kDt       = 0.01;
    constexpr double kDuration = 10.0;
    constexpr double kWarmup   = 2.0;

    auto omega = [](double t) noexcept { return 2.5 + 0.5 * std::sin(2.0 * kPi * t); };

    fcs::L3::TrackerConfig config;
    config.robot.lost_threshold                 = 1.0;
    config.robot.tracking_threshold             = 0;
    config.robot.matcher_gate                   = 100.0;
    config.robot.model.sigma_a_xy               = 10.0;
    config.robot.model.sigma_a_z                = 2.0;
    config.robot.model.sigma_a_yaw              = 20.0;
    config.robot.model.sigma_r0                 = 1.0;
    config.robot.model.sigma_r1                 = 1.0;
    config.robot.model.sigma_h                  = 0.8;
    config.robot.model.meas_yaw_var_floor       = 4e-3;
    config.robot.model.meas_pitch_var_floor     = 4e-3;
    config.robot.model.meas_dist_var_floor      = 0.3;
    config.robot.model.meas_dist_delta_angle_k  = 1.0;
    config.robot.model.meas_armor_yaw_var_floor = 9e-2;
    config.robot.model.meas_armor_yaw_range_k   = 0.005;
    config.robot_inekf.radius0                  = 0.23;
    config.robot_inekf.radius1                  = 0.23;
    config.robot_inekf.height                   = 0.1;

    fcs::L3::TrackerNew tracker(config);

    fcs::L3::RobotTargetState truth;
    truth.position   = Eigen::Vector3d(3.0, 0.4, 0.2);
    truth.velocity   = Eigen::Vector3d::Zero();
    truth.yaw        = 0.3;
    truth.v_yaw      = omega(0.0);
    truth.radius0    = config.robot_inekf.radius0;
    truth.radius1    = config.robot_inekf.radius1;
    truth.z1         = truth.position.z() + config.robot_inekf.height;
    truth.armors_num = 4;

    fcs::ArmorMeasurementBatch first_batch;
    first_batch.measurements.push_back(
        make_measurement(truth.armor_poses()[0], fcs::ArmorName::Three));
    ASSERT_TRUE(tracker.first_meet(first_batch).has_value());

    double time                 = 0.0;
    double abs_err_sum          = 0.0;
    double sq_err_sum           = 0.0;
    double max_abs_err          = 0.0;
    double est_min_after_settle = std::numeric_limits<double>::infinity();
    double est_max_after_settle = -std::numeric_limits<double>::infinity();
    size_t sample_count         = 0;

    const int steps = static_cast<int>(kDuration / kDt);
    for (int step = 0; step < steps; ++step) {
        tracker.predict(kDt);

        time += kDt;
        truth.v_yaw = omega(time);
        truth.yaw += truth.v_yaw * kDt;

        fcs::ArmorMeasurementBatch batch;
        const auto poses = truth.armor_poses();
        for (const auto& pose : poses) {
            batch.measurements.push_back(make_measurement(pose, fcs::ArmorName::Three));
        }

        ASSERT_TRUE(tracker.update(batch));

        const auto output = tracker.get_output();
        ASSERT_TRUE(output.robot_state() != nullptr);

        const double est_v_yaw = output.robot_state()->v_yaw;
        if (time >= kWarmup) {
            const double err = est_v_yaw - truth.v_yaw;
            abs_err_sum += std::abs(err);
            sq_err_sum += err * err;
            max_abs_err          = std::max(max_abs_err, std::abs(err));
            est_min_after_settle = std::min(est_min_after_settle, est_v_yaw);
            est_max_after_settle = std::max(est_max_after_settle, est_v_yaw);
            ++sample_count;
        }
    }

    ASSERT_GT(sample_count, 0u);

    const double mae  = abs_err_sum / static_cast<double>(sample_count);
    const double rmse = std::sqrt(sq_err_sum / static_cast<double>(sample_count));
    const double amp  = 0.5 * (est_max_after_settle - est_min_after_settle);

    std::cout << "sinusoidal omega tracking: "
              << "mae=" << mae << ", rmse=" << rmse << ", max_abs_err=" << max_abs_err
              << ", est_min=" << est_min_after_settle << ", est_max=" << est_max_after_settle
              << ", est_amp=" << amp << '\n';

    EXPECT_LT(mae, 0.25);
    EXPECT_LT(rmse, 0.30);
    EXPECT_LT(max_abs_err, 0.70);
    EXPECT_GT(amp, 0.15);
    EXPECT_LT(amp, 0.60);
}

TEST(AimerPhase, TrackerArmorIdealFireHitRateOnSinusoidalOmegaInput) {
    constexpr double kPi          = std::numbers::pi;
    constexpr double kDt          = 0.01;
    constexpr double kDuration    = 10.0;
    constexpr double kWarmup      = 2.0;
    constexpr double kBulletSpeed = 30.0;
    constexpr double kArmorWidth  = 0.135;
    constexpr double kArmorHeight = 0.135;

    auto omega  = [](double t) noexcept { return 2.5 + 0.5 * std::sin(2.0 * kPi * t); };
    auto yaw_at = [](double t) noexcept {
        return 0.3 + 2.5 * t + (1.0 - std::cos(2.0 * kPi * t)) / (4.0 * kPi);
    };
    auto wrap_angle = [](double angle) noexcept {
        return std::remainder(angle, 2.0 * std::numbers::pi);
    };

    fcs::L3::TrackerConfig tracker_config;
    tracker_config.robot.lost_threshold                 = 1.0;
    tracker_config.robot.tracking_threshold             = 0;
    tracker_config.robot.matcher_gate                   = 100.0;
    tracker_config.robot.model.sigma_a_xy               = 10.0;
    tracker_config.robot.model.sigma_a_z                = 2.0;
    tracker_config.robot.model.sigma_a_yaw              = 20.0;
    tracker_config.robot.model.sigma_r0                 = 1.0;
    tracker_config.robot.model.sigma_r1                 = 1.0;
    tracker_config.robot.model.sigma_h                  = 0.8;
    tracker_config.robot.model.meas_yaw_var_floor       = 4e-3;
    tracker_config.robot.model.meas_pitch_var_floor     = 4e-3;
    tracker_config.robot.model.meas_dist_var_floor      = 0.3;
    tracker_config.robot.model.meas_dist_delta_angle_k  = 1.0;
    tracker_config.robot.model.meas_armor_yaw_var_floor = 9e-2;
    tracker_config.robot.model.meas_armor_yaw_range_k   = 0.005;
    tracker_config.robot_inekf.radius0                  = 0.23;
    tracker_config.robot_inekf.radius1                  = 0.23;
    tracker_config.robot_inekf.height                   = 0.1;

    fcs::L3::TrackerNew tracker(tracker_config);
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    MockTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    const auto muzzle = make_muzzle();

    auto truth_state_at = [&](double t) noexcept {
        fcs::L3::RobotTargetState truth;
        truth.position   = Eigen::Vector3d(4.0, 0.0, 0.2);
        truth.velocity   = Eigen::Vector3d::Zero();
        truth.yaw        = yaw_at(t);
        truth.v_yaw      = omega(t);
        truth.radius0    = tracker_config.robot_inekf.radius0;
        truth.radius1    = tracker_config.robot_inekf.radius1;
        truth.z1         = truth.position.z() + tracker_config.robot_inekf.height;
        truth.armors_num = 4;
        return truth;
    };

    const auto truth0 = truth_state_at(0.0);
    fcs::ArmorMeasurementBatch first_batch;
    first_batch.measurements.push_back(
        make_measurement(truth0.armor_poses()[0], fcs::ArmorName::Three));
    ASSERT_TRUE(tracker.first_meet(first_batch).has_value());

    size_t total_shots                   = 0;
    size_t total_hits                    = 0;
    size_t settled_shots                 = 0;
    size_t settled_hits                  = 0;
    double yaw_abs_err_sum               = 0.0;
    double pitch_abs_err_sum             = 0.0;
    double max_abs_yaw_err               = 0.0;
    double max_abs_pitch_err             = 0.0;
    double min_yaw_margin_after_warmup   = std::numeric_limits<double>::infinity();
    double min_pitch_margin_after_warmup = std::numeric_limits<double>::infinity();

    const int steps = static_cast<int>(kDuration / kDt);
    for (int step = 1; step <= steps; ++step) {
        const double time = static_cast<double>(step) * kDt;
        tracker.predict(kDt);

        const auto truth_now = truth_state_at(time);
        fcs::ArmorMeasurementBatch batch;
        for (const auto& pose : truth_now.armor_poses()) {
            batch.measurements.push_back(make_measurement(pose, fcs::ArmorName::Three));
        }
        ASSERT_TRUE(tracker.update(batch));

        const auto output = tracker.get_output();
        ASSERT_TRUE(output.robot_state() != nullptr);

        fcs::L4::ArmorAimContext context;
        context.target_jumped      = output.target_jumped;
        context.phase              = fcs::L4::ArmorAimPhase::SingleArmor;
        context.preferred_armor_id = output.last_armor_id;

        const uint64_t measurement_ns = static_cast<uint64_t>(std::llround(time * 1.0e9));
        const auto prediction         = aimer.aim(
            *output.robot_state(), context, gimbal, muzzle, measurement_ns, measurement_ns, 0.0,
            kBulletSpeed, solver);
        ASSERT_TRUE(prediction.has_value()) << prediction.error();

        const double future_time =
            static_cast<double>(prediction->predicted_future_ns - measurement_ns) * 1.0e-9 + time;
        const auto truth_future = truth_state_at(future_time);
        const auto actual_pose =
            truth_future.armor_poses()[static_cast<size_t>(prediction->selected_armor_id)];
        const Eigen::Vector3d actual_pos = actual_pose.head<3>() - muzzle.translation();

        const double actual_yaw = std::atan2(actual_pos.y(), actual_pos.x());
        const double actual_pitch =
            std::atan2(actual_pos.z(), std::hypot(actual_pos.x(), actual_pos.y()));
        const double yaw_err   = wrap_angle(prediction->aim_yaw - actual_yaw);
        const double pitch_err = prediction->aim_pitch - actual_pitch;
        const double distance  = actual_pos.norm();
        const double yaw_tol   = std::atan2(kArmorWidth / 2.0, distance);
        const double pitch_tol = std::atan2(kArmorHeight / 2.0, distance);
        const bool hit         = std::abs(yaw_err) < yaw_tol && std::abs(pitch_err) < pitch_tol;

        ++total_shots;
        if (hit) {
            ++total_hits;
        }

        if (time >= kWarmup) {
            ++settled_shots;
            if (hit) {
                ++settled_hits;
            }
            yaw_abs_err_sum += std::abs(yaw_err);
            pitch_abs_err_sum += std::abs(pitch_err);
            max_abs_yaw_err   = std::max(max_abs_yaw_err, std::abs(yaw_err));
            max_abs_pitch_err = std::max(max_abs_pitch_err, std::abs(pitch_err));
            min_yaw_margin_after_warmup =
                std::min(min_yaw_margin_after_warmup, yaw_tol - std::abs(yaw_err));
            min_pitch_margin_after_warmup =
                std::min(min_pitch_margin_after_warmup, pitch_tol - std::abs(pitch_err));
        }
    }

    ASSERT_GT(total_shots, 0u);
    ASSERT_GT(settled_shots, 0u);

    const double total_hit_rate =
        static_cast<double>(total_hits) / static_cast<double>(total_shots);
    const double settled_hit_rate =
        static_cast<double>(settled_hits) / static_cast<double>(settled_shots);
    const double mean_abs_yaw_err   = yaw_abs_err_sum / static_cast<double>(settled_shots);
    const double mean_abs_pitch_err = pitch_abs_err_sum / static_cast<double>(settled_shots);

    std::cout << "ideal fire on sinusoidal omega: "
              << "total_hit_rate=" << total_hit_rate << ", settled_hit_rate=" << settled_hit_rate
              << ", mean_abs_yaw_err=" << mean_abs_yaw_err
              << ", max_abs_yaw_err=" << max_abs_yaw_err
              << ", mean_abs_pitch_err=" << mean_abs_pitch_err
              << ", max_abs_pitch_err=" << max_abs_pitch_err
              << ", min_yaw_margin=" << min_yaw_margin_after_warmup
              << ", min_pitch_margin=" << min_pitch_margin_after_warmup << '\n';

    EXPECT_GT(total_hit_rate, 0.95);
    EXPECT_GT(settled_hit_rate, 0.99);
    EXPECT_GT(min_yaw_margin_after_warmup, 0.0);
    EXPECT_GT(min_pitch_margin_after_warmup, 0.0);
}

TEST(AimerPhase, TrackerArmorIdealFireHitRateOnTwoSecondAbsSinBurst) {
    constexpr double kPi          = std::numbers::pi;
    constexpr double kDt          = 0.01;
    constexpr double kDuration    = 8.0;
    constexpr double kWarmup      = 1.0;
    constexpr double kBulletSpeed = 30.0;
    constexpr double kArmorWidth  = 0.135;
    constexpr double kArmorHeight = 0.135;
    constexpr double kBaseOmega   = 2.0;
    constexpr double kPeakOmega   = 3.0;
    constexpr double kBurstStart  = 2.0;
    constexpr double kBurstEnd    = 4.0;

    auto omega = [](double t) noexcept {
        if (t < kBurstStart || t > kBurstEnd) {
            return kBaseOmega;
        }
        const double phase = kPi * (t - kBurstStart);
        return kBaseOmega + (kPeakOmega - kBaseOmega) * std::abs(std::sin(phase));
    };

    auto yaw_at = [&](double t) noexcept {
        const double pre_burst  = std::min(t, kBurstStart);
        const double burst_t    = std::clamp(t, kBurstStart, kBurstEnd) - kBurstStart;
        const double post_burst = std::max(0.0, t - kBurstEnd);

        const double yaw_pre   = kBaseOmega * pre_burst;
        const double yaw_burst = 2.0 * burst_t + (1.0 - std::cos(kPi * burst_t)) / kPi;
        const double yaw_post  = kBaseOmega * post_burst;
        return 0.3 + yaw_pre + yaw_burst + yaw_post;
    };

    fcs::L3::TrackerConfig tracker_config;
    tracker_config.robot.lost_threshold                 = 1.0;
    tracker_config.robot.tracking_threshold             = 0;
    tracker_config.robot.matcher_gate                   = 100.0;
    tracker_config.robot.model.sigma_a_xy               = 10.0;
    tracker_config.robot.model.sigma_a_z                = 2.0;
    tracker_config.robot.model.sigma_a_yaw              = 20.0;
    tracker_config.robot.model.sigma_r0                 = 1.0;
    tracker_config.robot.model.sigma_r1                 = 1.0;
    tracker_config.robot.model.sigma_h                  = 0.8;
    tracker_config.robot.model.meas_yaw_var_floor       = 4e-3;
    tracker_config.robot.model.meas_pitch_var_floor     = 4e-3;
    tracker_config.robot.model.meas_dist_var_floor      = 0.3;
    tracker_config.robot.model.meas_dist_delta_angle_k  = 1.0;
    tracker_config.robot.model.meas_armor_yaw_var_floor = 9e-2;
    tracker_config.robot.model.meas_armor_yaw_range_k   = 0.005;
    tracker_config.robot_inekf.radius0                  = 0.23;
    tracker_config.robot_inekf.radius1                  = 0.23;
    tracker_config.robot_inekf.height                   = 0.1;

    fcs::L3::TrackerNew tracker(tracker_config);
    fcs::L4::Aimer aimer(fcs::L4::AimerConfig{});
    MockTrajectorySolver solver;
    const auto gimbal = make_gimbal();
    const auto muzzle = make_muzzle();

    auto wrap_angle = [](double angle) noexcept {
        return std::remainder(angle, 2.0 * std::numbers::pi);
    };

    auto truth_state_at = [&](double t) noexcept {
        fcs::L3::RobotTargetState truth;
        truth.position   = Eigen::Vector3d(4.0, 0.0, 0.2);
        truth.velocity   = Eigen::Vector3d::Zero();
        truth.yaw        = yaw_at(t);
        truth.v_yaw      = omega(t);
        truth.radius0    = tracker_config.robot_inekf.radius0;
        truth.radius1    = tracker_config.robot_inekf.radius1;
        truth.z1         = truth.position.z() + tracker_config.robot_inekf.height;
        truth.armors_num = 4;
        return truth;
    };

    const auto truth0 = truth_state_at(0.0);
    fcs::ArmorMeasurementBatch first_batch;
    first_batch.measurements.push_back(
        make_measurement(truth0.armor_poses()[0], fcs::ArmorName::Three));
    ASSERT_TRUE(tracker.first_meet(first_batch).has_value());

    size_t total_shots     = 0;
    size_t total_hits      = 0;
    size_t burst_shots     = 0;
    size_t burst_hits      = 0;
    size_t settled_shots   = 0;
    size_t settled_hits    = 0;
    double yaw_abs_err_sum = 0.0;
    double max_abs_yaw_err = 0.0;

    const int steps = static_cast<int>(kDuration / kDt);
    for (int step = 1; step <= steps; ++step) {
        const double time = static_cast<double>(step) * kDt;
        tracker.predict(kDt);

        const auto truth_now = truth_state_at(time);
        fcs::ArmorMeasurementBatch batch;
        for (const auto& pose : truth_now.armor_poses()) {
            batch.measurements.push_back(make_measurement(pose, fcs::ArmorName::Three));
        }
        ASSERT_TRUE(tracker.update(batch));

        const auto output = tracker.get_output();
        ASSERT_TRUE(output.robot_state() != nullptr);

        fcs::L4::ArmorAimContext context;
        context.target_jumped      = output.target_jumped;
        context.phase              = fcs::L4::ArmorAimPhase::SingleArmor;
        context.preferred_armor_id = output.last_armor_id;

        const uint64_t measurement_ns = static_cast<uint64_t>(std::llround(time * 1.0e9));
        const auto prediction         = aimer.aim(
            *output.robot_state(), context, gimbal, muzzle, measurement_ns, measurement_ns, 0.0,
            kBulletSpeed, solver);
        ASSERT_TRUE(prediction.has_value()) << prediction.error();

        const double future_time =
            static_cast<double>(prediction->predicted_future_ns - measurement_ns) * 1.0e-9 + time;
        const auto truth_future = truth_state_at(future_time);
        const auto actual_pose =
            truth_future.armor_poses()[static_cast<size_t>(prediction->selected_armor_id)];
        const Eigen::Vector3d actual_pos = actual_pose.head<3>() - muzzle.translation();

        const double actual_yaw = std::atan2(actual_pos.y(), actual_pos.x());
        const double actual_pitch =
            std::atan2(actual_pos.z(), std::hypot(actual_pos.x(), actual_pos.y()));
        const double yaw_err   = wrap_angle(prediction->aim_yaw - actual_yaw);
        const double pitch_err = prediction->aim_pitch - actual_pitch;
        const double distance  = actual_pos.norm();
        const double yaw_tol   = std::atan2(kArmorWidth / 2.0, distance);
        const double pitch_tol = std::atan2(kArmorHeight / 2.0, distance);
        const bool hit         = std::abs(yaw_err) < yaw_tol && std::abs(pitch_err) < pitch_tol;

        ++total_shots;
        if (hit) {
            ++total_hits;
        }
        if (time >= kBurstStart && time <= kBurstEnd) {
            ++burst_shots;
            if (hit) {
                ++burst_hits;
            }
        }
        if (time >= kWarmup) {
            ++settled_shots;
            if (hit) {
                ++settled_hits;
            }
            yaw_abs_err_sum += std::abs(yaw_err);
            max_abs_yaw_err = std::max(max_abs_yaw_err, std::abs(yaw_err));
        }
    }

    ASSERT_GT(total_shots, 0u);
    ASSERT_GT(burst_shots, 0u);
    ASSERT_GT(settled_shots, 0u);

    const double total_hit_rate =
        static_cast<double>(total_hits) / static_cast<double>(total_shots);
    const double burst_hit_rate =
        static_cast<double>(burst_hits) / static_cast<double>(burst_shots);
    const double settled_hit_rate =
        static_cast<double>(settled_hits) / static_cast<double>(settled_shots);
    const double mean_abs_yaw_err = yaw_abs_err_sum / static_cast<double>(settled_shots);

    std::cout << "ideal fire on abs-sin burst omega: "
              << "total_hit_rate=" << total_hit_rate << ", burst_hit_rate=" << burst_hit_rate
              << ", settled_hit_rate=" << settled_hit_rate
              << ", mean_abs_yaw_err=" << mean_abs_yaw_err
              << ", max_abs_yaw_err=" << max_abs_yaw_err << '\n';

    EXPECT_GT(total_hit_rate, 0.95);
    EXPECT_GT(burst_hit_rate, 0.95);
    EXPECT_GT(settled_hit_rate, 0.98);
}

TEST(AimerPhaseFsm, JumpedFastTargetWalksAuthorityFourStateFsm) {
    fcs::L4::L4Config config;
    config.aimer.single_whole_up   = 0.1;
    config.aimer.single_whole_down = 0.05;
    config.aimer.whole_pair_up     = 0.2;
    config.aimer.whole_pair_down   = 0.15;
    config.aimer.pair_center_up    = 0.3;
    config.aimer.pair_center_down  = 0.25;
    config.aimer.transfer_thresh   = 0;

    fcs::L4::ArmorAimPhase phase = fcs::L4::ArmorAimPhase::SingleArmor;
    int overflow_count           = 0;

    fcs::L4::advance_armor_aim_phase(config.aimer, 10.0, true, phase, overflow_count);
    EXPECT_EQ(phase, fcs::L4::ArmorAimPhase::WholeCarArmor);

    fcs::L4::advance_armor_aim_phase(config.aimer, 10.0, true, phase, overflow_count);
    EXPECT_EQ(phase, fcs::L4::ArmorAimPhase::WholeCarPair);

    fcs::L4::advance_armor_aim_phase(config.aimer, 10.0, true, phase, overflow_count);
    EXPECT_EQ(phase, fcs::L4::ArmorAimPhase::WholeCarCenter);
}

TEST(AimerSystemsTargetSelection, PrefersSmallerOpticalOffsetOverCloserTarget) {
    AimerSystemHarness harness;
    harness.start();

    harness.set_trackers({
        make_robot_tracker_output(
            fcs::ArmorName::One, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(2.5, 0.8, 0.0),
            280.0),
        make_robot_tracker_output(
            fcs::ArmorName::Two, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(5.0, 0.0, 0.0),
            10.0),
    });

    const auto selected_target = harness.wait_for_target(fcs::ArmorName::Two);
    const auto trace           = harness.wait_for_selection_trace_after(0);
    ASSERT_TRUE(selected_target.has_value());
    ASSERT_TRUE(trace.has_value());
    ASSERT_FALSE(trace->candidates.empty());
    EXPECT_EQ(selected_target->tracker.target_name, fcs::ArmorName::Two);
    EXPECT_TRUE(trace->candidates.front().selected);
    EXPECT_EQ(trace->candidates.front().target_name, fcs::ArmorName::Two);

    harness.stop();
}

TEST(AimerSystemsTargetSelection, TrackingTargetBeatsTempLostFallback) {
    AimerSystemHarness harness;
    harness.start();

    harness.set_trackers({
        make_robot_tracker_output(
            fcs::ArmorName::One, fcs::L3::TrackerStatus::TempLost, Eigen::Vector3d(2.5, 0.0, 0.0),
            10.0, fcs::ArmorColor::Blue, 1),
        make_robot_tracker_output(
            fcs::ArmorName::Two, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.15, 0.0),
            120.0),
    });

    const auto selected_target = harness.wait_for_target(fcs::ArmorName::Two);
    const auto trace           = harness.wait_for_selection_trace_after(0);
    ASSERT_TRUE(selected_target.has_value());
    ASSERT_TRUE(trace.has_value());
    ASSERT_FALSE(trace->candidates.empty());
    EXPECT_EQ(selected_target->tracker.target_name, fcs::ArmorName::Two);
    EXPECT_EQ(trace->candidates.front().track_status, fcs::L3::TrackerStatus::Tracking);

    harness.stop();
}

TEST(AimerSystemsTargetSelection, KeepsCurrentTargetWhenScoreGainIsWithinSwitchMargin) {
    fcs::L4::L4Config config;
    config.aimer.target_selection.decider       = fcs::L4::ArmorTargetDeciderKind::Unmanned;
    config.aimer.target_selection.switch_margin = 1.0;
    AimerSystemHarness harness(config);
    harness.start();

    harness.set_trackers({
        make_robot_tracker_output(
            fcs::ArmorName::One, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            80.0),
    });
    const auto initial_target = harness.wait_for_target(fcs::ArmorName::One);
    ASSERT_TRUE(initial_target.has_value());

    harness.set_trackers({
        make_robot_tracker_output(
            fcs::ArmorName::One, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            80.0),
        make_robot_tracker_output(
            fcs::ArmorName::Two, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            70.0),
    });

    const auto selected_target =
        harness.wait_for_selected_target_after(initial_target->timestamp_ns);
    const auto plan  = harness.wait_for_armor_plan_after(initial_target->timestamp_ns);
    const auto trace = harness.wait_for_selection_trace_after(initial_target->timestamp_ns);
    ASSERT_TRUE(selected_target.has_value());
    ASSERT_TRUE(plan.has_value());
    ASSERT_TRUE(trace.has_value());
    ASSERT_GE(trace->candidates.size(), 2u);
    const auto selected_it =
        std::find_if(trace->candidates.begin(), trace->candidates.end(), [](const auto& candidate) {
            return candidate.selected;
        });
    const auto runner_up_it =
        std::find_if(trace->candidates.begin(), trace->candidates.end(), [](const auto& candidate) {
            return candidate.runner_up;
        });
    ASSERT_NE(selected_it, trace->candidates.end());
    ASSERT_NE(runner_up_it, trace->candidates.end());
    EXPECT_EQ(selected_target->tracker.target_name, fcs::ArmorName::One);
    EXPECT_TRUE(trace->kept_current_target);
    EXPECT_EQ(selected_it->target_name, fcs::ArmorName::One);
    EXPECT_EQ(runner_up_it->target_name, fcs::ArmorName::Two);

    harness.stop();
}

TEST(AimerSystemsTargetSelection, SwitchesWhenNewTargetClearlyWins) {
    fcs::L4::L4Config config;
    config.aimer.target_selection.decider = fcs::L4::ArmorTargetDeciderKind::Unmanned;
    AimerSystemHarness harness(config);
    harness.start();

    harness.set_trackers({
        make_robot_tracker_output(
            fcs::ArmorName::One, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            120.0),
    });
    const auto initial_target = harness.wait_for_target(fcs::ArmorName::One);
    ASSERT_TRUE(initial_target.has_value());

    harness.set_trackers({
        make_robot_tracker_output(
            fcs::ArmorName::One, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            220.0),
        make_robot_tracker_output(
            fcs::ArmorName::Two, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            10.0),
    });

    const auto selected_target =
        harness.wait_for_selected_target_after(initial_target->timestamp_ns);
    const auto plan  = harness.wait_for_armor_plan_after(initial_target->timestamp_ns);
    const auto trace = harness.wait_for_selection_trace_after(initial_target->timestamp_ns);
    ASSERT_TRUE(selected_target.has_value());
    ASSERT_TRUE(plan.has_value());
    ASSERT_TRUE(trace.has_value());
    ASSERT_FALSE(trace->candidates.empty());
    EXPECT_EQ(selected_target->tracker.target_name, fcs::ArmorName::Two);
    EXPECT_FALSE(trace->kept_current_target);
    EXPECT_TRUE(trace->candidates.front().selected);
    EXPECT_EQ(trace->candidates.front().target_name, fcs::ArmorName::Two);

    harness.stop();
}

TEST(AimerSystemsTargetSelection, MannedKeepsCurrentTargetUntilTrackerIsLost) {
    fcs::L4::L4Config config;
    config.aimer.target_selection.decider = fcs::L4::ArmorTargetDeciderKind::Manned;
    AimerSystemHarness harness(config);
    harness.start();

    harness.set_trackers({
        make_robot_tracker_output(
            fcs::ArmorName::One, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            20.0),
        make_robot_tracker_output(
            fcs::ArmorName::Two, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            80.0),
    });
    const auto initial_target = harness.wait_for_target(fcs::ArmorName::One);
    ASSERT_TRUE(initial_target.has_value());

    harness.set_trackers({
        make_robot_tracker_output(
            fcs::ArmorName::One, fcs::L3::TrackerStatus::TempLost, Eigen::Vector3d(3.0, 0.0, 0.0),
            220.0, fcs::ArmorColor::Blue, initial_target->tracker.last_observation_timestamp_ns),
        make_robot_tracker_output(
            fcs::ArmorName::Two, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            5.0),
    });

    const auto locked_target = harness.wait_for_selected_target_after(initial_target->timestamp_ns);
    const auto locked_trace  = harness.wait_for_selection_trace_after(initial_target->timestamp_ns);
    ASSERT_TRUE(locked_target.has_value());
    ASSERT_TRUE(locked_trace.has_value());
    EXPECT_EQ(locked_target->tracker.target_name, fcs::ArmorName::One);
    EXPECT_TRUE(locked_trace->kept_current_target);

    harness.set_trackers({
        make_robot_tracker_output(
            fcs::ArmorName::Two, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            5.0),
    });

    const uint64_t switched_after_timestamp =
        std::max(locked_target->timestamp_ns, locked_trace->timestamp_ns);
    const auto switched_target = harness.wait_for_selected_target_after(switched_after_timestamp);
    const auto switched_trace  = harness.wait_for_selection_trace_after(switched_after_timestamp);
    ASSERT_TRUE(switched_target.has_value());
    ASSERT_TRUE(switched_trace.has_value());
    EXPECT_EQ(switched_target->tracker.target_name, fcs::ArmorName::Two);
    EXPECT_FALSE(switched_trace->kept_current_target);

    harness.stop();
}

TEST(AimerSystemsTargetSelection, EnteringFollowingRelocksToUvCenterNearestTarget) {
    fcs::L4::L4Config config;
    config.aimer.target_selection.decider       = fcs::L4::ArmorTargetDeciderKind::Unmanned;
    config.aimer.target_selection.switch_margin = 1.0;
    AimerSystemHarness harness(config);
    harness.start();

    harness.set_following(false);
    harness.set_trackers({
        make_robot_tracker_output(
            fcs::ArmorName::One, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            80.0),
        make_robot_tracker_output(
            fcs::ArmorName::Two, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            70.0),
    });

    const auto unmanned_target = harness.wait_for_target(fcs::ArmorName::One);
    ASSERT_TRUE(unmanned_target.has_value());

    harness.set_following(true);

    const auto manned_target =
        harness.wait_for_selected_target_after(unmanned_target->timestamp_ns);
    const auto manned_trace = harness.wait_for_selection_trace_after(unmanned_target->timestamp_ns);
    ASSERT_TRUE(manned_target.has_value());
    ASSERT_TRUE(manned_trace.has_value());
    EXPECT_EQ(manned_target->tracker.target_name, fcs::ArmorName::Two);
    EXPECT_FALSE(manned_trace->kept_current_target);

    harness.stop();
}

TEST(AimerSystemsTargetSelection, EmitsRankedTraceForAllCandidates) {
    AimerSystemHarness harness;
    harness.start();

    harness.set_trackers({
        make_robot_tracker_output(
            fcs::ArmorName::One, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            220.0),
        make_robot_tracker_output(
            fcs::ArmorName::Two, fcs::L3::TrackerStatus::Tracking, Eigen::Vector3d(3.0, 0.0, 0.0),
            10.0),
    });

    const auto trace = harness.wait_for_selection_trace_after(0);
    ASSERT_TRUE(trace.has_value());
    ASSERT_EQ(trace->candidates.size(), 2u);

    EXPECT_EQ(trace->candidates[0].rank, 1);
    EXPECT_TRUE(trace->candidates[0].selected);
    EXPECT_EQ(trace->candidates[0].target_name, fcs::ArmorName::Two);
    EXPECT_TRUE(trace->candidates[0].aim_valid);

    EXPECT_EQ(trace->candidates[1].rank, 2);
    EXPECT_TRUE(trace->candidates[1].runner_up);
    EXPECT_EQ(trace->candidates[1].target_name, fcs::ArmorName::One);
    EXPECT_TRUE(trace->candidates[1].aim_valid);
    EXPECT_GT(trace->candidates[0].total_score, trace->candidates[1].total_score);

    harness.stop();
}

TEST(AimerSystemsTargetSelection, EmitsInvalidSnapshotWhenNoArmorTargetAvailable) {
    AimerSystemHarness harness;
    harness.start();

    harness.set_trackers({});

    const auto selected_target = harness.wait_for_selected_target_after(0);
    ASSERT_TRUE(selected_target.has_value());
    EXPECT_FALSE(selected_target->has_target());
    EXPECT_EQ(selected_target->tracker.target_name, fcs::ArmorName::Invalid);
    EXPECT_EQ(selected_target->source, fcs::L4::GimbalPlanSource::Armor);

    harness.stop();
}

TEST(AimerSystemsTargetSelection, TempLostRetainsLastOpticalMeasurement) {
    fcs::L3::TrackerConfig config;
    config.robot.model              = fcs::L3::RobotEkfMotionModel::Params{};
    config.robot.lost_threshold     = 1.0;
    config.robot.tracking_threshold = 0;
    config.robot.matcher_gate       = 10.0;
    config.robot_inekf.radius0      = 0.2;
    config.robot_inekf.radius1      = 0.2;
    config.robot_inekf.height       = 0.0;

    fcs::L3::TrackerNew tracker(config);

    fcs::ArmorMeasurementBatch batch;
    batch.timestamp_ns = 100;
    batch.measurements.push_back(
        make_measurement(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0), fcs::ArmorName::Three, 42.0f));
    batch.measurements.front().timestamp_ns = 100;

    ASSERT_TRUE(tracker.first_meet(batch).has_value());
    tracker.predict(0.01);
    ASSERT_TRUE(tracker.update(batch));

    const auto tracking_output = tracker.get_output();
    EXPECT_DOUBLE_EQ(tracking_output.last_image_center_distance_px, 42.0);
    EXPECT_EQ(tracking_output.last_observation_timestamp_ns, 100u);

    fcs::ArmorMeasurementBatch empty_batch;
    empty_batch.timestamp_ns = 200;
    tracker.predict(0.01);
    ASSERT_FALSE(tracker.update(empty_batch));

    const auto temp_lost_output = tracker.get_output();
    EXPECT_EQ(temp_lost_output.status, fcs::L3::TrackerStatus::TempLost);
    EXPECT_DOUBLE_EQ(temp_lost_output.last_image_center_distance_px, 42.0);
    EXPECT_EQ(temp_lost_output.last_observation_timestamp_ns, 100u);
}

TEST(AimerSystemsTargetSelection, ReadbackRoiFreshnessUsesLastObservationTimestampDuringTempLost) {
    fcs::L2::ArmorReadbackRoiConfig readback_roi_config;
    readback_roi_config.enabled         = true;
    readback_roi_config.stale_timeout_s = 0.20;

    AimerSystemHarness harness({}, readback_roi_config);
    harness.start();

    constexpr uint64_t kLastObservationTimestampNs = 123456u;
    harness.set_trackers({make_robot_tracker_output(
        fcs::ArmorName::One, fcs::L3::TrackerStatus::TempLost, Eigen::Vector3d(0.0, 0.0, 10.0),
        12.0, fcs::ArmorColor::Blue, kLastObservationTimestampNs)});

    const auto selected_target = harness.wait_for_selected_target_after(0);
    ASSERT_TRUE(selected_target.has_value());
    EXPECT_GT(selected_target->tracker.timestamp_ns, kLastObservationTimestampNs);
    EXPECT_EQ(selected_target->tracker.last_observation_timestamp_ns, kLastObservationTimestampNs);

    const auto tracker_snapshot = harness.wait_for_readback_tracker();
    ASSERT_TRUE(tracker_snapshot.has_value());
    EXPECT_TRUE(tracker_snapshot->valid);
    EXPECT_EQ(tracker_snapshot->timestamp_ns, kLastObservationTimestampNs);
    EXPECT_EQ(tracker_snapshot->tracker.target_name, fcs::ArmorName::One);

    harness.stop();
}
