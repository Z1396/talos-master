/**
 * @file trajectory_optimizer.cpp
 * @brief L5武器层轨迹优化器实现
 *
 * 本文件实现了针对ARM Cortex-A55优化的MPC轨迹优化器。
 * 核心思想是使用DualSmallMpcSolver替代原本的TinyMPC库,性能提升:
 * 1. 消除软件模拟的双精度浮点(ARM A55上double比float慢约20倍)
 * 2. 消除动态矩阵分配开销(每轮ADMM迭代)
 * 3. 消除TinyMPC内存泄漏所需的5次分配RAII封装
 *
 * 算法完全相同——同样的Riccati反向/正向扫描,
 * 同样的松弛变量/对偶更新,同样的终止检查。只是没有额外开销。
 */

#include "L5_weapon/enhanced/trajectory_optimizer.hpp"

#include "L5_weapon/fire_decision.hpp"
#include "core/math/normalize.hpp"

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace fcs::L5 {
namespace {

using fcs::core::math::normalize_angle;

using ReferenceTrajectory = core::trajectory::ReferenceTrajectory;

/**
 * @brief 将时间延迟量化为离散步数
 *
 * 根据参考轨迹的时间步长,将实际时间延迟转换为轨迹索引偏移量。
 *
 * @param reference_age_s 参考轨迹年龄(秒)
 * @param dt_s 时间步长(秒)
 *
 * @return 离散步数(非负整数)
 */
[[nodiscard]] int quantize_reference_age_steps(double reference_age_s, double dt_s) noexcept {
    if (!(reference_age_s > 0.0) || !(dt_s > 0.0)) {
        return 0;
    }
    return std::max(0, static_cast<int>(std::lround(reference_age_s / dt_s)));
}

/**
 * @brief 单轴求解器视图(泛型辅助类)
 *
 * 为yaw或pitch轴提供统一的接口访问DualSmallMpcSolver或DualMpcOsqpSolver。
 * 自动处理索引边界检查。
 */
template <typename Solver>
struct AxisSolverView {
    const Solver* solver{nullptr};
    int axis{0};
    int horizon{0};

    /**
     * @brief 获取状态值(位置或速度)
     *
     * @param dim 维度索引(0=位置,1=速度)
     * @param k 时间步索引
     *
     * @return 状态值(自动clamp到有效范围)
     */
    [[nodiscard]] float state(int dim, int k) const noexcept {
        const int clamped_k = std::clamp(k, 0, horizon - 1);
        return solver->state(axis, dim, clamped_k);
    }

    /**
     * @brief 获取控制输入(加速度)
     *
     * @param k 时间步索引[0, horizon-2]
     *
     * @return 加速度值(自动clamp到有效范围)
     */
    [[nodiscard]] float input(int k) const noexcept {
        const int clamped_k = std::clamp(k, 0, horizon - 2);
        return solver->input(axis, clamped_k);
    }
};

/**
 * @brief 球坐标转笛卡尔坐标
 *
 * 用于可视化调试,将(yaw, pitch, distance)转换为(x, y, z)。
 *
 * @param distance 距离(米)
 * @param yaw yaw角度(弧度)
 * @param pitch pitch角度(弧度)
 *
 * @return 笛卡尔坐标向量(x, y, z)
 */
[[nodiscard]] Eigen::Vector3d
    spherical_to_cartesian(double distance, double yaw, double pitch) noexcept {
    return Eigen::Vector3d{
        distance * std::cos(pitch) * std::cos(yaw),
        distance * std::cos(pitch) * std::sin(yaw),
        distance * std::sin(pitch),
    };
}

/**
 * @brief 构建可视化调试数据
 *
 * 提取参考轨迹和优化后的轨迹,用于Foxglove可视化。
 *
 * @param center_index 中心时刻索引
 * @param trajectory L4参考轨迹
 * @param reference_start_index 参考轨迹起始索引
 * @param reference_horizon 参考轨迹长度
 * @param yaw_solver yaw轴求解器视图
 * @param pitch_solver pitch轴求解器视图
 *
 * @return 可视化调试数据
 */
template <typename YawSolver, typename PitchSolver>
[[nodiscard]] WeaponVisualizationDebugData build_debug_data(
    int center_index, const ReferenceTrajectory& trajectory, int reference_start_index,
    int reference_horizon, const YawSolver& yaw_solver, const PitchSolver& pitch_solver) {
    WeaponVisualizationDebugData debug;
    debug.center_index    = center_index;
    debug.lookahead_index = debug.center_index;
    debug.reference_plan.reserve(reference_horizon);
    debug.optimized_plan.reserve(reference_horizon);

    // 遍历所有时间步,提取轨迹数据
    for (int k = 0; k < reference_horizon; ++k) {
        const int ref_k = reference_start_index + k;

        // 提取参考轨迹点
        debug.reference_plan.push_back(
            TrajectoryPlanSample{
                .yaw      = normalize_angle(trajectory.state(0, ref_k) + trajectory.yaw_origin),
                .pitch    = trajectory.state(2, ref_k),
                .distance = trajectory.distances[ref_k],
                .tof      = trajectory.time_of_flights[ref_k],
            });

        // 提取优化后的轨迹点
        const double optimized_yaw =
            normalize_angle(static_cast<double>(yaw_solver.state(0, k)) + trajectory.yaw_origin);
        const double optimized_pitch =
            normalize_angle(static_cast<double>(pitch_solver.state(0, k)));
        debug.optimized_plan.push_back(
            TrajectoryPlanSample{
                .yaw      = optimized_yaw,
                .pitch    = optimized_pitch,
                .distance = trajectory.distances[ref_k],
                .tof      = trajectory.time_of_flights[ref_k],
            });
    }

    return debug;
}

} // namespace

/**
 * @brief 构造函数:初始化MPC求解器
 *
 * 如果配置启用,则立即初始化DualSmallMpcSolver实例。
 */
TinyMpcTrajectoryOptimizer::TinyMpcTrajectoryOptimizer(
    const WeaponControllerConfig& config,
    const L4::ReferenceTrajectoryConfig& trajectory_cfg) noexcept
    : config_(config)
    , trajectory_cfg_(trajectory_cfg) {
    if (config_.enabled) {
        initialize_solvers();
    }
}

/**
 * @brief 透传模式:构建Shot指令(无MPC优化)
 *
 * 直接将L4的Shot意图转换为武器指令,不进行轨迹优化。
 * 用于单次瞄准场景,如能量机关打击。
 *
 * 特点:
 * - 位置直接透传,速度和加速度设为0
 * - 不需要参考轨迹
 * - 响应快但可能不如Track模式平滑
 */
WeaponCommand TinyMpcTrajectoryOptimizer::passthrough(
    const L4::ShotCommand& shot, uint64_t command_timestamp_ns) const noexcept {
    WeaponCommand cmd;
    cmd.timestamp_ns      = command_timestamp_ns;
    cmd.plan_timestamp_ns = shot.timestamp_ns;
    cmd.plan_yaw          = shot.yaw;
    cmd.plan_pitch        = shot.pitch;
    cmd.plan_distance     = shot.distance;
    cmd.yaw               = shot.yaw;
    cmd.pitch             = shot.pitch;
    cmd.distance          = shot.distance;

    // Shot模式:不提供速度/加速度前馈
    cmd.yaw_vel     = 0.0;
    cmd.pitch_vel   = 0.0;
    cmd.yaw_accel   = 0.0;
    cmd.pitch_accel = 0.0;
    return cmd;
}

/**
 * @brief 优化模式:对Track意图执行MPC轨迹优化
 *
 * 这是整个武器层最核心的算法流程,包含以下关键步骤:
 *
 * 1. 参考轨迹新鲜度检查
 *    - 计算参考轨迹年龄(当前时间 - 轨迹生成时间)
 *    - 拒绝过时的参考轨迹(超过阈值则报错)
 *    - 目的:避免在过时数据上浪费计算资源
 *
 * 2. 参考轨迹窗口对齐
 *    - 根据时间延迟将参考轨迹窗口向前移动
 *    - 确保MPC求解的"当前"时刻与实际时间对齐
 *    - 例如:如果参考轨迹已延迟10ms,则从第10ms开始采样
 *
 * 3. 设置MPC求解器参数
 *    - 初始状态:从参考轨迹提取当前位置和速度
 *    - 参考轨迹:将整个预测窗口的参考状态传给求解器
 *    - 边界处理:如果参考轨迹短于求解器步数,用最后一个点填充
 *
 * 4. 执行MPC求解
 *    - DualSmallMpcSolver内部执行ADMM迭代
 *    - 典型收敛时间:<5ms(ARM Cortex-A55)
 *
 * 5. 提取控制指令
 *    - 提取中心时刻的位置和速度(期望轨迹)
 *    - 提取控制索引的加速度(前馈控制)
 *    - 中心索引通常是horizon_back,代表当前最佳瞄准点
 *
 * 6. 构建武器指令
 *    - 组装位置、速度、加速度
 *    - 应用pitch角度限制(机械约束)
 *    - 可选:构建可视化调试数据
 *
 * @param track L4层的Track指令(包含参考轨迹)
 * @param command_timestamp_ns 当前时间戳(纳秒)
 *
 * @return 成功返回优化后的武器指令,失败返回错误信息
 *
 * @note 性能关键路径,避免内存分配和系统调用
 */
std::expected<WeaponCommand, std::string> TinyMpcTrajectoryOptimizer::optimize(
    const L4::TrackCommand& track, uint64_t command_timestamp_ns) noexcept {
    // 步骤1:检查求解器是否就绪
    if (!config_.enabled) {
        return std::unexpected("mpc_weapon disabled");
    }
    if (!ready_) {
        return std::unexpected("mpc solvers are not ready");
    }

    // 步骤2:计算参考轨迹年龄并检查新鲜度
    // 参考轨迹年龄 = 当前时间 - 参考轨迹生成时间
    const double reference_age_s =
        static_cast<double>(
            static_cast<int64_t>(command_timestamp_ns) - static_cast<int64_t>(track.timestamp_ns))
        * 1e-9;

    // 如果参考轨迹过时(超过阈值),拒绝求解
    if (reference_age_s > config_.reference_age_threshold_s) {
        return std::unexpected(
            "reference trajectory is stale (" + std::to_string(reference_age_s) + "s > "
            + std::to_string(config_.reference_age_threshold_s) + "s)");
    }

    const ReferenceTrajectory& reference = track.control_trajectory;

    // 步骤3:计算参考轨迹窗口对齐参数
    // 将时间延迟转换为离散的轨迹索引偏移
    const int solver_horizon    = batched_solver_.horizon();
    const int reference_horizon = reference.horizon();
    const int reference_shift_steps =
        quantize_reference_age_steps(std::max(reference_age_s, 0.0), trajectory_cfg_.dt);

    // 参考轨迹起始索引(考虑时间延迟)
    const int reference_start_index =
        std::clamp(reference_shift_steps, 0, std::max(reference_horizon - 1, 0));

    // 可用参考轨迹长度和有效预测长度
    const int available_reference_horizon = reference_horizon - reference_start_index;
    const int effective_horizon           = std::min(available_reference_horizon, solver_horizon);

    // 检查有效预测长度是否足够
    if (effective_horizon < 2) {
        return std::unexpected("reference trajectory horizon is too short for MPC");
    }

    // 步骤4:设置求解器初始状态(从参考轨迹提取)
    batched_solver_.set_x0(
        reference.state(0, reference_start_index), reference.state(1, reference_start_index),
        reference.state(2, reference_start_index), reference.state(3, reference_start_index));

    // 步骤5:设置参考轨迹(逐时间步)
    for (int i = 0; i < effective_horizon; ++i) {
        const int ref_i = reference_start_index + i;
        batched_solver_.set_ref_col(
            i, static_cast<float>(reference.state(0, ref_i)),
            static_cast<float>(reference.state(1, ref_i)),
            static_cast<float>(reference.state(2, ref_i)),
            static_cast<float>(reference.state(3, ref_i)));
    }

    // 边界处理:如果参考轨迹短于求解器步数,用最后一个点填充
    if (effective_horizon < solver_horizon) {
        const int last_ref = reference_start_index + effective_horizon - 1;
        for (int i = effective_horizon; i < solver_horizon; ++i) {
            batched_solver_.set_ref_col(
                i, static_cast<float>(reference.state(0, last_ref)),
                static_cast<float>(reference.state(1, last_ref)),
                static_cast<float>(reference.state(2, last_ref)),
                static_cast<float>(reference.state(3, last_ref)));
        }
    }

    // 步骤6:执行MPC求解
    batched_solver_.solve();

    // 步骤7:创建求解器视图(方便提取数据)
    const AxisSolverView yaw_solver{&batched_solver_, DualSmallMpcSolver::kYawAxis, solver_horizon};
    const AxisSolverView pitch_solver{
        &batched_solver_, DualSmallMpcSolver::kPitchAxis, solver_horizon};

    // 步骤8:计算中心索引和控制索引
    // 中心索引:期望瞄准点(通常是horizon_back)
    // 控制索引:提取加速度的位置(比中心索引少1)
    const int center_index  = std::clamp(trajectory_cfg_.horizon_back, 0, effective_horizon - 1);
    const int control_index = std::clamp(center_index, 0, effective_horizon - 2);
    const int reference_center_index = reference_start_index + center_index;

    // 步骤9:从参考轨迹提取瞄准点(L4的原始瞄准目标)
    const double plan_yaw      = normalize_angle(reference.yaw_origin);
    const double plan_pitch    = reference.state(2, reference_start_index + center_index);
    const double plan_distance = reference.distances[reference_center_index];

    // 步骤10:构建武器指令
    WeaponCommand cmd;
    cmd.timestamp_ns      = command_timestamp_ns;
    cmd.plan_timestamp_ns = track.timestamp_ns;
    cmd.plan_yaw          = plan_yaw;
    cmd.plan_pitch        = plan_pitch;
    cmd.plan_distance     = plan_distance;

    // 步骤11:提取MPC优化后的位置(应用yaw原点偏移)
    cmd.yaw = normalize_angle(
        static_cast<double>(yaw_solver.state(0, center_index)) + reference.yaw_origin);

    // 应用pitch角度限制(机械约束)
    cmd.pitch = std::clamp(
        static_cast<double>(pitch_solver.state(0, center_index)), config_.pitch_min,
        config_.pitch_max);

    // 步骤12:提取速度和加速度(前馈控制)
    cmd.yaw_vel     = static_cast<double>(yaw_solver.state(1, center_index));
    cmd.pitch_vel   = static_cast<double>(pitch_solver.state(1, center_index));
    cmd.yaw_accel   = static_cast<double>(yaw_solver.input(control_index));
    cmd.pitch_accel = static_cast<double>(pitch_solver.input(control_index));

    // 附加信息
    cmd.distance = reference.distances[reference_center_index];
    cmd.tof      = reference.time_of_flights[reference_center_index];

    // 可选:构建可视化调试数据
    if (config_.enable_debug) {
        cmd.viz_debug = build_debug_data(
            center_index, reference, reference_start_index, effective_horizon, yaw_solver,
            pitch_solver);
    }

    // 步骤13:数值有效性检查(防止NaN/Inf)
    const auto finite = [](double value) noexcept { return std::isfinite(value); };
    if (!finite(cmd.yaw) || !finite(cmd.pitch) || !finite(cmd.yaw_vel) || !finite(cmd.pitch_vel)
        || !finite(cmd.yaw_accel) || !finite(cmd.pitch_accel) || !finite(cmd.distance)
        || !finite(cmd.tof)) {
        return std::unexpected("MPC output contains non-finite values");
    }

    return cmd;
}

/**
 * @brief 初始化MPC求解器
 *
 * 根据配置参数构造DualSmallMpcSolver实例。
 * 仅在构造函数中调用一次。
 *
 * 关键步骤:
 * 1. 计算预测步数(horizon_ahead + horizon_back + 1)
 * 2. 检查是否超过求解器容量上限
 * 3. 配置yaw轴参数(Q、R权重,最大加速度)
 * 4. 配置pitch轴参数(增加状态约束)
 * 5. 构造求解器并设置收敛参数
 */
void TinyMpcTrajectoryOptimizer::initialize_solvers() noexcept {
    // 步骤1:计算预测步数
    // horizon = 前向预测步数 + 后向步数 + 当前点(1)
    const int requested_horizon = trajectory_cfg_.horizon_ahead + trajectory_cfg_.horizon_back + 1;

    // 步骤2:检查是否超过求解器容量上限
    const int horizon = std::min(requested_horizon, DualSmallMpcSolver::kMaxHorizon);
    const float dt    = static_cast<float>(trajectory_cfg_.dt);
    const float rho   = static_cast<float>(config_.mpc.rho);

    // 如果请求步数超过上限,发出警告并自动裁剪
    if (requested_horizon > horizon) {
        SPDLOG_WARN(
            "L5 MPC horizon {} exceeds fixed-capacity limit {}; clipping to {}. "
            "Increase the solver limit or reduce L4 horizon if this is unintended.",
            requested_horizon, DualSmallMpcSolver::kMaxHorizon, horizon);
    }

    // 步骤3:配置yaw轴参数
    DualSmallMpcSolver::AxisConfig yaw_cfg;
    yaw_cfg.q_pos   = static_cast<float>(config_.mpc.yaw.q_pos);
    yaw_cfg.q_vel   = static_cast<float>(config_.mpc.yaw.q_vel);
    yaw_cfg.r       = static_cast<float>(config_.mpc.yaw.r);
    yaw_cfg.max_acc = static_cast<float>(config_.mpc.yaw.max_acc);

    // 步骤4:配置pitch轴参数(增加状态约束)
    DualSmallMpcSolver::AxisConfig pitch_cfg;
    pitch_cfg.q_pos   = static_cast<float>(config_.mpc.pitch.q_pos);
    pitch_cfg.q_vel   = static_cast<float>(config_.mpc.pitch.q_vel);
    pitch_cfg.r       = static_cast<float>(config_.mpc.pitch.r);
    pitch_cfg.max_acc = static_cast<float>(config_.mpc.pitch.max_acc);

    // pitch轴启用状态约束(防止机械碰撞)
    pitch_cfg.enable_state_bound = true;
    pitch_cfg.state_min          = static_cast<float>(config_.pitch_min);
    pitch_cfg.state_max          = static_cast<float>(config_.pitch_max);

    // 步骤5:构造求解器
    auto solver = DualSmallMpcSolver::create(dt, horizon, rho, yaw_cfg, pitch_cfg);
    if (!solver) {
        // 构造失败(参数无效)
        ready_ = false;
        return;
    }

    // 设置收敛参数
    solver->set_settings(static_cast<float>(config_.mpc.abs_tol), config_.mpc.max_iterations);

    // 移动赋值(避免拷贝)
    batched_solver_ = std::move(*solver);
    ready_          = true;
}

} // namespace fcs::L5
