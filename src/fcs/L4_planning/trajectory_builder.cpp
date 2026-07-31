/**
 * @file trajectory_builder.cpp
 * @brief L4规划层参考轨迹构建器实现
 *
 * 本文件实现了轨迹构建的核心算法，包括：
 * - 有限差分法计算角度速率
 * - 参考轨迹采样点构建
 * - 状态矩阵组装
 *
 * 算法细节：
 * - 使用中心差分法（精度O(h²)）计算导数，比前向/后向差分更精确
 * - 对yaw角度使用归一化差分，避免±π跳变
 * - 边界条件：两端使用前向/后向差分
 *
 * 性能优化：
 * - 预分配vector容量，避免动态扩容
 * - 使用lambda表达式内联小函数
 * - 减少临时对象创建
 *
 * 潜在风险提示：
 * - **数值稳定性**：当时间步长dt过小时，有限差分可能引入数值误差
 * - **角度跳变**：yaw角度接近±π时需特殊处理，否则差分结果异常
 * - **弹道求解失败**：当目标超出有效射程时，aimer.aim()可能失败
 * - **状态一致性**：build_sample lambda在循环内解析variant，需保证种子状态有效
 *
 * 优化建议：
 * - **SIMD优化**：使用Eigen的SIMD指令加速状态矩阵计算
 * - **并行化**：使用TBB并行构建采样点（需解决依赖问题）
 * - **缓存优化**：缓存弹道求解结果，避免重复计算
 * - **自适应步长**：根据目标运动状态动态调整采样步长
 */

#include "L4_planning/trajectory_builder.hpp"
#include "core/math/normalize.hpp"

#include <fmt/core.h>
#include <vector>

namespace fcs::L4 {
namespace {

using fcs::core::math::normalize_angle;

using core::trajectory::ReferenceTrajectory;
using StateMatrix = ReferenceTrajectory::StateMatrix;

/**
 * @brief 参考轨迹采样点
 *
 * 存储单个时间步的轨迹数据，用于后续差分计算。
 */
struct ReferenceSample {
    double yaw{0.0};            ///< yaw角度（弧度）
    double pitch{0.0};          ///< pitch角度（弧度）
    double distance{0.0};       ///< 目标距离（米）
    double time_of_flight{0.0}; ///< 弹道飞行时间（秒）
    int selected_armor_id{0};   ///< 选中的装甲板ID
};

/**
 * @brief 有限差分法计算导数（普通值）
 *
 * 使用中心差分法计算一阶导数：
 * f'(x) ≈ [f(x+h) - f(x-h)] / (2h)
 *
 * 边界条件：
 * - 左边界（index=0）：前向差分 f'(x) ≈ [f(x+h) - f(x)] / h
 * - 右边界（index=size-1）：后向差分 f'(x) ≈ [f(x) - f(x-h)] / h
 *
 * @param samples 采样点序列
 * @param index 当前索引
 * @param dt 时间步长（秒）
 * @return 导数值
 *
 * @note 时间复杂度O(1)，空间复杂度O(1)
 *
 * 潜在风险：
 * - **数值误差**：当dt过小（<1e-6）时，可能引入舍入误差
 * - **精度下降**：边界条件使用前向/后向差分，精度O(h)而非O(h²)
 * - **噪声敏感**：对高频噪声敏感，建议先平滑采样点
 */
[[nodiscard]] double
    finite_difference(const std::vector<double>& samples, int index, double dt) noexcept {
    const int size = static_cast<int>(samples.size());
    if (size <= 1) {
        return 0.0; // 单点或空序列，导数为0
    }
    if (index <= 0) {
        // 左边界：前向差分
        return (samples[1] - samples[0]) / dt;
    }
    if (index >= size - 1) {
        // 右边界：后向差分
        return (samples[size - 1] - samples[size - 2]) / dt;
    }
    // 中心差分（精度最高）
    return (samples[index + 1] - samples[index - 1]) / (2.0 * dt);
}

/**
 * @brief 有限差分法计算导数（角度值）
 *
 * 针对角度的特殊处理：
 * - 使用normalize_angle归一化角度差，避免±π跳变
 * - 例如：从179°到-179°，差分应为2°而非-358°
 *
 * @param samples 角度采样点序列（弧度）
 * @param index 当前索引
 * @param dt 时间步长（秒）
 * @return 角度速率（弧度/秒）
 *
 * 潜在风险：
 * - **周期边界**：当角度快速穿越±π时，归一化可能引入错误
 * - **角速度过大**：当角速度超过π/dt时，归一化可能导致方向反转
 * - **建议**：确保角速度估计准确，避免过大的采样步长
 */
[[nodiscard]] double
    finite_difference_angle(const std::vector<double>& samples, int index, double dt) noexcept {
    const int size = static_cast<int>(samples.size());
    if (size <= 1) {
        return 0.0;
    }
    if (index <= 0) {
        return normalize_angle(samples[1] - samples[0]) / dt;
    }
    if (index >= size - 1) {
        return normalize_angle(samples[size - 1] - samples[size - 2]) / dt;
    }
    return normalize_angle(samples[index + 1] - samples[index - 1]) / (2.0 * dt);
}

/**
 * @brief 构建单个参考轨迹采样点（模板函数）
 *
 * 核心功能：
 * - 调用Aimer::aim()计算指定时间偏移的瞄准角度
 * - 提取瞄准结果中的关键数据（yaw, pitch, distance等）
 * - 返回结构化的采样点数据
 *
 * @tparam TargetState 目标状态类型（RobotTargetState或OutpostTargetState）
 * @param aimer 瞄准器实例
 * @param state 目标状态
 * @param context 装甲板瞄准上下文（包含阶段、偏好等信息）
 * @param state_timestamp_ns 状态时间戳（纳秒）
 * @param current_ns 当前时间戳（纳秒）
 * @param sample_offset_s 采样时间偏移（秒）
 * @param gimbal 云台变换矩阵
 * @param muzzle 枪口变换矩阵
 * @param solver 弹道求解器
 * @param bullet_speed 弹速
 * @return 成功时返回ReferenceSample，失败时返回错误信息
 *
 * @note 这是性能热点函数，每个轨迹构建调用多次
 *
 * 潜在风险：
 * - **弹道求解失败**：当目标超出射程或弹道无法收敛时，返回错误
 * - **装甲板选择冲突**：不同时间偏移可能选择不同装甲板，导致轨迹不连续
 * - **状态预测误差**：长时间偏移（>200ms）预测误差可能累积
 * - **建议**：限制预测时域，增加鲁棒性检查
 */
template <typename TargetState>
[[nodiscard]] std::expected<ReferenceSample, std::string> build_reference_sample(
    const Aimer& aimer, const TargetState& state, ArmorAimContext context,
    uint64_t state_timestamp_ns, uint64_t current_ns, double sample_offset_s,
    const Aimer::GimbalTransform& gimbal, const Aimer::MuzzleTransform& muzzle,
    const core::trajectory::solver::TrajectorySolver& solver, double bullet_speed) noexcept {
    const auto prediction = aimer.aim(
        state, context, gimbal, muzzle, state_timestamp_ns, current_ns, sample_offset_s,
        bullet_speed, solver);
    if (!prediction) {
        return std::unexpected(prediction.error());
    }

    return ReferenceSample{
        .yaw               = prediction->aim_yaw,
        .pitch             = prediction->aim_pitch,
        .distance          = prediction->distance,
        .time_of_flight    = prediction->flying_time,
        .selected_armor_id = prediction->selected_armor_id,
    };
}

/**
 * @brief 转换瞄准阶段为射击阶段
 *
 * 关键逻辑：
 * - WholeCarCenter → WholeCarArmor：射击轨迹必须瞄准真实装甲板
 * - 其他阶段保持不变
 *
 * @param phase 原始瞄准阶段
 * @return 射击使用的瞄准阶段
 */
[[nodiscard]] ArmorAimPhase fire_aim_phase(ArmorAimPhase phase) noexcept {
    return phase == ArmorAimPhase::WholeCarCenter ? ArmorAimPhase::WholeCarArmor : phase;
}

/**
 * @brief 构建参考轨迹（内部实现，支持指定瞄准阶段）
 *
 * 核心算法流程：
 * 1. 预分配内存（状态矩阵、距离向量、飞行时间向量）
 * 2. 遍历预测时域，计算每个采样点
 * 3. 使用有限差分法计算角度速率
 * 4. 组装最终轨迹
 *
 * 关键优化：
 * - 在循环外解析variant，避免循环内重复分支判断
 * - 使用lambda捕获常用变量，减少参数传递
 * - 预先检查错误条件，提前返回
 *
 * @param seed 规划器种子
 * @param aimer 瞄准器
 * @param horizon_cfg 时域配置
 * @param ctx 构建上下文
 * @param phase 瞄准阶段（控制轨迹用原始phase，射击轨迹用转换后的phase）
 * @return 成功时返回轨迹，失败时返回错误
 *
 * 潜在风险：
 * - **内存分配**：horizon过大可能导致栈溢出或内存不足
 * - **计算延迟**：长时域（horizon>20）可能引入不可接受的延迟
 * - **状态失效**：种子状态失效时立即返回错误，需在上层处理
 * - **依赖检查**：build_sample lambda在每次调用时检查状态有效性，需保证效率
 *
 * 优化建议：
 * - **预分配池**：使用对象池复用轨迹内存，避免频繁分配
 * - **并行计算**：使用TBB并行构建采样点（需处理依赖关系）
 * - **早停机制**：当连续多个采样点失败时，提前终止并返回错误
 * - **增量更新**：对于连续帧，考虑增量更新轨迹而非全量重建
 */
[[nodiscard]] std::expected<ReferenceTrajectory, std::string> build_reference_trajectory_with_phase(
    const PlannerSeed& seed, const Aimer& aimer, const ReferenceTrajectoryConfig& horizon_cfg,
    const TrajectoryBuildContext& ctx, ArmorAimPhase phase) noexcept {
    const int horizon      = horizon_cfg.horizon_ahead + horizon_cfg.horizon_back + 1;
    const int horizon_back = horizon_cfg.horizon_back;
    const double dt        = horizon_cfg.dt;

    // 预分配轨迹内存
    ReferenceTrajectory trajectory{
        .state           = StateMatrix(4, horizon),
        .distances       = std::vector<double>(horizon, 0.0),
        .time_of_flights = std::vector<double>(horizon, 0.0),
    };

    // 采样点缓冲区
    std::vector<double> yaw_samples(horizon, 0.0);
    std::vector<double> pitch_samples(horizon, 0.0);

    // 构建瞄准上下文
    ArmorAimContext context;
    context.target_jumped      = seed.target_jumped;
    context.phase              = phase;
    context.preferred_armor_id = seed.selected_armor_id;

    // 关键优化：在循环外解析variant一次，避免循环内重复分支
    // 如果seed没有有效状态，立即失败，避免在热循环中分配"空规划器种子"字符串
    const auto build_sample = [&](int k) -> std::expected<ReferenceSample, std::string> {
        const double sample_offset = (static_cast<double>(k) - horizon_back) * dt;
        if (const auto* robot = seed.robot_state()) {
            return build_reference_sample(
                aimer, *robot, context, seed.state_timestamp_ns, ctx.current_ns, sample_offset,
                ctx.gimbal, ctx.muzzle, *ctx.trajectory_solver, ctx.bullet_speed);
        }
        if (const auto* outpost = seed.outpost_state()) {
            return build_reference_sample(
                aimer, *outpost, context, seed.state_timestamp_ns, ctx.current_ns, sample_offset,
                ctx.gimbal, ctx.muzzle, *ctx.trajectory_solver, ctx.bullet_speed);
        }
        return std::unexpected("empty planner seed");
    };

    // 遍历时域，构建采样点
    for (int k = 0; k < horizon; ++k) {
        auto sample = build_sample(k);
        if (!sample) {
            return std::unexpected(sample.error());
        }

        // 更新偏好装甲板ID（用于下一轮瞄准）
        context.preferred_armor_id = sample->selected_armor_id;

        yaw_samples[k]                = sample->yaw;
        pitch_samples[k]              = sample->pitch;
        trajectory.distances[k]       = sample->distance;
        trajectory.time_of_flights[k] = sample->time_of_flight;
    }

    // 设置yaw原点（用于相对坐标）
    trajectory.yaw_origin = yaw_samples[horizon_back];

    // 计算状态矩阵（使用有限差分）
    for (int k = 0; k < horizon; ++k) {
        trajectory.state(0, k) = normalize_angle(yaw_samples[k] - trajectory.yaw_origin);
        trajectory.state(1, k) = finite_difference_angle(yaw_samples, k, dt);
        trajectory.state(2, k) = pitch_samples[k];
        trajectory.state(3, k) = finite_difference(pitch_samples, k, dt);
    }

    return trajectory;
}

} // namespace

/**
 * @brief 构建控制参考轨迹（公开API）
 *
 * 使用原始瞄准阶段构建轨迹，用于MPC控制。
 *
 * @see build_reference_trajectory_with_phase()
 */
[[nodiscard]] std::expected<ReferenceTrajectory, std::string> build_reference_trajectory(
    const PlannerSeed& seed, const Aimer& aimer, const ReferenceTrajectoryConfig& horizon_cfg,
    const TrajectoryBuildContext& ctx) noexcept {
    return build_reference_trajectory_with_phase(seed, aimer, horizon_cfg, ctx, seed.aim_phase);
}

/**
 * @brief 构建射击参考轨迹（公开API）
 *
 * 使用转换后的瞄准阶段构建轨迹，用于火力门控判断。
 * WholeCarCenter阶段会转换为WholeCarArmor阶段。
 *
 * @see build_reference_trajectory_with_phase()
 */
[[nodiscard]] std::expected<ReferenceTrajectory, std::string> build_fire_reference_trajectory(
    const PlannerSeed& seed, const Aimer& aimer, const ReferenceTrajectoryConfig& horizon_cfg,
    const TrajectoryBuildContext& ctx) noexcept {
    return build_reference_trajectory_with_phase(
        seed, aimer, horizon_cfg, ctx, fire_aim_phase(seed.aim_phase));
}

/**
 * @brief 构建LDM目标参考轨迹
 *
 * LDM轨迹构建的特殊性：
 * - 使用常速度模型预测目标位置
 * - 没有装甲板选择逻辑（因为是点目标）
 * - 直接调用Aimer::aim(LdmState)接口
 *
 * 算法流程：
 * 1. 检查LDM状态是否处于跟踪模式
 * 2. 遍历预测时域，计算每个采样点
 * 3. 使用有限差分法计算角度速率
 * 4. 组装最终轨迹
 *
 * @param state LDM目标状态
 * @param aimer 瞄准器
 * @param horizon_cfg 时域配置
 * @param ctx 构建上下文
 * @return 成功时返回轨迹，失败时返回错误
 *
 * 潜在风险：
 * - **LDM状态失效**：非跟踪状态下立即返回错误
 * - **弹道求解失败**：LDM目标可能距离较远，弹道求解可能失败
 * - **状态预测误差**：常速度模型在长时间偏移时误差较大
 * - **建议**：结合LDM运动模型改进预测精度
 */
[[nodiscard]] std::expected<ReferenceTrajectory, std::string> build_ldm_reference_trajectory(
    const L3::ldm::LdmState& state, const Aimer& aimer,
    const ReferenceTrajectoryConfig& horizon_cfg, const TrajectoryBuildContext& ctx) noexcept {

    if (!state.is_tracking()) {
        return std::unexpected("LDM target is not tracking");
    }

    const int horizon      = horizon_cfg.horizon_ahead + horizon_cfg.horizon_back + 1;
    const int horizon_back = horizon_cfg.horizon_back;
    const double dt        = horizon_cfg.dt;

    // 预分配轨迹内存
    ReferenceTrajectory trajectory{
        .state           = StateMatrix(4, horizon),
        .distances       = std::vector<double>(horizon, 0.0),
        .time_of_flights = std::vector<double>(horizon, 0.0),
    };

    // 采样点缓冲区
    std::vector<double> yaw_samples(horizon, 0.0);
    std::vector<double> pitch_samples(horizon, 0.0);

    // 遍历时域，构建采样点
    for (int k = 0; k < horizon; ++k) {
        const double sample_offset = (static_cast<double>(k) - horizon_back) * dt;

        // 调用LDM专用瞄准接口
        const auto prediction = aimer.aim(
            state, ctx.gimbal, ctx.muzzle, ctx.current_ns, sample_offset, ctx.bullet_speed,
            *ctx.trajectory_solver);
        if (!prediction) {
            return std::unexpected(
                fmt::format("LDM reference trajectory step {} failed: {}", k, prediction.error()));
        }

        yaw_samples[k]                = prediction->aim_yaw;
        pitch_samples[k]              = prediction->aim_pitch;
        trajectory.distances[k]       = prediction->distance;
        trajectory.time_of_flights[k] = prediction->flying_time;
    }

    // 设置yaw原点
    trajectory.yaw_origin = yaw_samples[horizon_back];

    // 计算状态矩阵
    for (int k = 0; k < horizon; ++k) {
        trajectory.state(0, k) = normalize_angle(yaw_samples[k] - trajectory.yaw_origin);
        trajectory.state(1, k) = finite_difference_angle(yaw_samples, k, dt);
        trajectory.state(2, k) = pitch_samples[k];
        trajectory.state(3, k) = finite_difference(pitch_samples, k, dt);
    }

    return trajectory;
}

} // namespace fcs::L4
