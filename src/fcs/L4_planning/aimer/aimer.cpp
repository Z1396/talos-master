/**
 * @file aimer.cpp
 * @brief L4规划层瞄准器实现
 *
 * 本文件实现了Aimer类的核心瞄准算法，负责：
 * - 目标位置预测
 * - 装甲板选择
 * - 弹道求解
 * - 瞄准角度计算
 *
 * 核心算法：
 * 1. 位置预测：使用目标运动模型（速度、角速度）预测未来位置
 * 2. 装甲板选择：基于前向窗口和角度差选择最优装甲板
 * 3. 飞行时间精化：迭代求解弹道飞行时间
 * 4. 瞄准角度计算：考虑重力和空气阻力的弹道补偿
 *
 * 支持的目标类型：
 * - RobotTargetState：机器人目标（带装甲板旋转）
 * - OutpostTargetState：前哨站目标（固定3块装甲板）
 * - EnergyMeterState：能量机关目标（绕轴旋转）
 * - LdmState：LDM目标（匀速直线运动）
 *
 * 性能优化：
 * - 使用模板函数避免虚函数调用
 * - 装甲板选择使用前向窗口过滤，减少计算量
 * - 飞行时间迭代使用early exit优化
 */

#include "L4_planning/aimer/aimer.hpp"
#include "L3_estimation/ldm_naive/types.hpp"
#include "core/math/normalize.hpp"

#include <cstdlib>
#include <groups/SEn3.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

namespace fcs::L4 {
namespace {

using fcs::core::math::normalize_angle;

/// 能量机关固定转速（rad/s）
/// 小能量机关固定转速 π/3
inline constexpr double kFixedRuneSpeed = std::numbers::pi / 3.0;

/**
 * @brief 时间戳偏移计算（纳秒级）
 *
 * 安全地将时间戳加上偏移量，处理溢出和边界情况。
 *
 * @param base_ns 基准时间戳（纳秒）
 * @param offset_s 时间偏移（秒）
 * @return 偏移后的时间戳（纳秒）
 *
 * @note 使用long double避免整数溢出
 * @note 处理负数偏移和溢出情况
 */
[[nodiscard]] uint64_t offset_timestamp_ns(uint64_t base_ns, double offset_s) noexcept {
    if (!std::isfinite(offset_s)) {
        return base_ns;
    }

    const long double shifted_ns =
        static_cast<long double>(base_ns) + static_cast<long double>(offset_s) * 1.0e9L;
    if (shifted_ns <= 0.0L) {
        return 0;
    }

    const long double max_ns = static_cast<long double>(std::numeric_limits<uint64_t>::max());
    if (shifted_ns >= max_ns) {
        return std::numeric_limits<uint64_t>::max();
    }

    return static_cast<uint64_t>(std::llround(shifted_ns));
}

/**
 * @brief 计算车身中心yaw角
 *
 * 根据目标位置计算从相机到目标的yaw角。
 *
 * @param x 目标x坐标（米）
 * @param y 目标y坐标（米）
 * @return yaw角（弧度）
 */
[[nodiscard]] double center_yaw_from_xy(double x, double y) noexcept { return std::atan2(y, x); }

/**
 * @brief 清洗偏好装甲板ID
 *
 * 检查偏好装甲板ID是否有效（在[0, armors_num)范围内）。
 *
 * @param preferred 偏好装甲板ID
 * @param armors_num 装甲板数量
 * @return 有效时返回装甲板ID，否则返回nullopt
 */
[[nodiscard]] std::optional<int>
    sanitize_preferred(const std::optional<int>& preferred, int armors_num) noexcept {
    if (!preferred.has_value()) {
        return std::nullopt;
    }
    if (*preferred < 0 || *preferred >= armors_num) {
        return std::nullopt;
    }
    return preferred;
}

/**
 * @brief 选择最小角度差的装甲板
 *
 * 从候选列表中选择角度差最小的装甲板。
 *
 * @param candidates 候选装甲板ID列表
 * @param delta_angles 角度差数组
 * @return 最优装甲板ID
 */
[[nodiscard]] int pick_best_by_min_delta(
    const std::vector<int>& candidates, const std::vector<double>& delta_angles) noexcept {
    if (candidates.empty()) {
        return 0;
    }

    int best        = candidates.front();
    double best_abs = std::abs(delta_angles[static_cast<size_t>(best)]);
    for (const int id : candidates) {
        const double value = std::abs(delta_angles[static_cast<size_t>(id)]);
        if (value < best_abs) {
            best     = id;
            best_abs = value;
        }
    }
    return best;
}

/**
 * @brief 获取前向窗口半径（弧度）
 *
 * 将配置中的前向窗口角度转换为弧度。
 *
 * @param config 瞄准器配置
 * @return 前向窗口半径（弧度）
 */
[[nodiscard]] double front_window_rad(const AimerConfig& config) noexcept {
    return config.front_window_deg * std::numbers::pi / 180.0;
}

/**
 * @brief 过滤前向窗口内的装甲板
 *
 * 选择角度差在指定窗口内的装甲板。
 *
 * @param candidates 候选装甲板ID列表
 * @param delta_angles 角度差数组
 * @param front_window 前向窗口半径（弧度）
 * @return 过滤后的装甲板ID列表
 */
[[nodiscard]] std::vector<int> filter_front_window(
    const std::vector<int>& candidates, const std::vector<double>& delta_angles,
    double front_window) {
    std::vector<int> filtered;
    filtered.reserve(candidates.size());
    for (const int id : candidates) {
        if (std::abs(delta_angles[static_cast<size_t>(id)]) <= front_window) {
            filtered.push_back(id);
        }
    }
    return filtered;
}

/**
 * @brief 构建装甲板角度差数组
 *
 * 计算每块装甲板相对于车身中心的角度差。
 *
 * @tparam N 装甲板数量
 * @param armor_poses 装甲板位姿数组
 * @param center_yaw 车身中心yaw角
 * @return 角度差数组
 */
template <size_t N>
[[nodiscard]] std::vector<double>
    build_delta_angles(const std::array<Eigen::Vector4d, N>& armor_poses, double center_yaw) {
    std::vector<double> deltas;
    deltas.reserve(N);
    for (const auto& pose : armor_poses) {
        deltas.push_back(normalize_angle(pose[3] - center_yaw));
    }
    return deltas;
}

/**
 * @brief 选择瞄准装甲板ID
 *
 * 核心算法：基于前向窗口和偏好选择装甲板。
 *
 * 选择策略：
 * 1. 如果有偏好装甲板且在前向窗口内，选择偏好装甲板
 * 2. 否则，从前向窗口内选择角度差最小的装甲板
 * 3. 如果前向窗口内无装甲板，从所有装甲板中选择角度差最小的
 *
 * @tparam N 装甲板数量
 * @param armor_poses 装甲板位姿数组
 * @param center_yaw 车身中心yaw角
 * @param context 瞄准上下文（包含偏好和跳跃标记）
 * @param config 瞄准器配置
 * @param is_outpost 是否为前哨站目标
 * @return 选中的装甲板ID
 */
template <size_t N>
[[nodiscard]] int select_armor_id(
    const std::array<Eigen::Vector4d, N>& armor_poses, double center_yaw,
    const ArmorAimContext& context, const AimerConfig& config, bool is_outpost) noexcept {
    const int armors_num = static_cast<int>(N);

    if (!context.target_jumped) {
        return 0;
    }

    const std::vector<double> delta_angles = build_delta_angles(armor_poses, center_yaw);

    std::vector<int> all_ids;
    all_ids.reserve(armors_num);
    for (int i = 0; i < armors_num; ++i) {
        all_ids.push_back(i);
    }

    const auto candidates = filter_front_window(all_ids, delta_angles, front_window_rad(config));
    const auto preferred  = sanitize_preferred(context.preferred_armor_id, armors_num);
    if (!candidates.empty()) {
        if (preferred.has_value()
            && std::find(candidates.begin(), candidates.end(), *preferred) != candidates.end()) {
            return *preferred;
        }
        return pick_best_by_min_delta(candidates, delta_angles);
    }

    (void)is_outpost;
    return pick_best_by_min_delta(all_ids, delta_angles);
}

/**
 * @brief 计算中心瞄准位置
 *
 * 在WholeCarCenter模式下，计算车身中心代理点位置。
 *
 * @param center 目标中心位置
 * @param muzzle_pos 枪口位置
 * @param radius 目标半径
 * @param armor_z 装甲板z坐标
 * @return 代理瞄准位置
 */
[[nodiscard]] Eigen::Vector3d project_center_aim(
    const Eigen::Vector3d& center, const Eigen::Vector3d& muzzle_pos, double radius,
    double armor_z) noexcept {
    Eigen::Vector2d ray   = center.head<2>() - muzzle_pos.head<2>();
    const double ray_norm = ray.norm();
    if (ray_norm > 1e-6) {
        ray /= ray_norm;
    } else {
        const double yaw = center_yaw_from_xy(center.x(), center.y());
        ray              = Eigen::Vector2d{std::cos(yaw), std::sin(yaw)};
    }

    const Eigen::Vector2d projected_xy = center.head<2>() - radius * ray;
    return Eigen::Vector3d{projected_xy.x(), projected_xy.y(), armor_z};
}

/**
 * @brief 获取目标装甲板数量（机器人）
 *
 * @param target 机器人目标状态
 * @return 装甲板数量（限制在[1,4]范围内）
 */
[[nodiscard]] int target_armors_num(const L3::RobotTargetState& target) noexcept {
    return std::clamp(target.armors_num, 1, 4);
}

/**
 * @brief 获取目标装甲板数量（前哨站）
 *
 * @param target 前哨站目标状态（未使用）
 * @return 固定3块装甲板
 */
[[nodiscard]] int target_armors_num(const L3::OutpostTargetState& /*target*/) noexcept {
    return L3::OutpostTargetState::armors_num;
}

} // namespace

// ============================================================================
// predict_position - compatibility helpers
// ============================================================================

Eigen::Vector3d Aimer::predict_position(
    const energy_meter::EnergyMeterState& target, double predict_time) const noexcept {
    const Eigen::Vector3d r_center = target.r_center_odom;

    // 直接使用tracker计算的radius，避免从position推导引入数值误差
    // 这对上下位置（roll≈±90°）的精度尤其重要
    const double radius = target.radius;

    // 当前 roll 来自 UKF
    const double current_roll = target.roll + target.blade_id * 2 * M_PI / 5;
    double delta_theta        = 0.0;
    const int dir             = target.direction;

    // 预测 roll 变化
    if (target.model_valid) {
        if (target.is_big_rune) {
            // Δθ = dir · [(a/ω)·(cos(ω·t) - cos(ω·(t+dt))) + b·dt]
            const double a     = target.a;
            const double omega = target.omega;
            const double b     = target.b;
            const double t0    = target.t;

            delta_theta =
                dir
                * ((a / omega) * (std::cos(omega * t0) - std::cos(omega * (t0 + predict_time)))
                   + b * predict_time);
        } else {
            delta_theta = dir * kFixedRuneSpeed * predict_time;
        }
    }

    // 预测 roll
    const double predicted_roll = current_roll + delta_theta;

    // 基于 UKF 的 roll、pitch、yaw 计算预测位置
    // 叶片在竖直平面 (Y-Z) 内绕水平轴 (X) 旋转 roll 角度
    const double sr = std::sin(predicted_roll);
    const double cr = std::cos(predicted_roll);
    Eigen::Vector3d blade_local(0.0, -radius * sr, radius * cr);

    // 应用 pitch 旋转 (绕 Y 轴)
    const double sp = std::sin(target.pitch);
    const double cp = std::cos(target.pitch);
    blade_local     = Eigen::Vector3d(
        blade_local.x() * cp + blade_local.z() * sp, blade_local.y(),
        -blade_local.x() * sp + blade_local.z() * cp);

    // 应用 yaw 旋转 (绕 Z 轴)，再平移到圆心
    const double cy = std::cos(target.yaw);
    const double sy = std::sin(target.yaw);
    const Eigen::Vector3d blade_offset(
        blade_local.x() * cy - blade_local.y() * sy, blade_local.x() * sy + blade_local.y() * cy,
        blade_local.z());

    Eigen::Vector3d predicted_position = r_center + blade_offset;

    return predicted_position;
}

Eigen::Vector3d
    Aimer::predict_position(const L3::RobotTargetState& target, double dt) const noexcept {
    return predict_aim_point(target, dt, {}, {}).position;
}

Eigen::Vector3d
    Aimer::predict_position(const L3::OutpostTargetState& target, double dt) const noexcept {
    return predict_aim_point(target, dt, {}, {}).position;
}

Aimer::PredictedAimPoint Aimer::predict_aim_point(
    const L3::RobotTargetState& target, double dt, const ArmorAimContext& context,
    const MuzzleTransform& muzzle, std::optional<int> forced_armor_id) const noexcept {
    L3::RobotTargetState predicted = target;
    predicted.position += predicted.velocity * dt;
    predicted.yaw += predicted.v_yaw * dt;

    const auto armor_poses  = predicted.armor_poses();
    const double center_yaw = center_yaw_from_xy(predicted.position.x(), predicted.position.y());

    const int armor_id =
        sanitize_preferred(forced_armor_id, predicted.armors_num)
            .value_or(select_armor_id(armor_poses, center_yaw, context, config_, false));
    const Eigen::Vector4d& armor_pose = armor_poses[static_cast<size_t>(armor_id)];
    const double radius               = (armor_id % 2 == 0) ? predicted.radius0 : predicted.radius1;

    PredictedAimPoint result;
    result.armor_id    = armor_id;
    result.delta_angle = normalize_angle(armor_pose[3] - center_yaw);
    result.position    = armor_pose.head<3>();

    if (context.phase == ArmorAimPhase::WholeCarCenter) {
        result.position =
            project_center_aim(predicted.position, muzzle.translation(), radius, armor_pose[2]);
    }
    return result;
}

Aimer::PredictedAimPoint Aimer::predict_aim_point(
    const L3::OutpostTargetState& target, double dt, const ArmorAimContext& context,
    const MuzzleTransform& muzzle, std::optional<int> forced_armor_id) const noexcept {
    L3::OutpostTargetState predicted = target;
    predicted.position += predicted.velocity.head<2>() * dt;
    predicted.yaw += predicted.v_yaw * dt;

    const auto armor_poses  = predicted.armor_poses();
    const double center_yaw = center_yaw_from_xy(predicted.position.x(), predicted.position.y());
    const int armor_id =
        sanitize_preferred(forced_armor_id, L3::OutpostTargetState::armors_num)
            .value_or(select_armor_id(armor_poses, center_yaw, context, config_, true));
    const Eigen::Vector4d& armor_pose = armor_poses[static_cast<size_t>(armor_id)];

    PredictedAimPoint result;
    result.armor_id    = armor_id;
    result.delta_angle = normalize_angle(armor_pose[3] - center_yaw);
    result.position    = armor_pose.head<3>();

    if (context.phase == ArmorAimPhase::WholeCarCenter) {
        result.position = project_center_aim(
            Eigen::Vector3d{predicted.position.x(), predicted.position.y(), 0.0},
            muzzle.translation(), L3::OutpostTargetState::radius, armor_pose[2]);
    }

    return result;
}

// ============================================================================
// aim_generic - Template method for targets with prediction
// ============================================================================

template <class TargetType>
std::expected<TargetPrediction, std::string> Aimer::aim_generic(
    const TargetType& target, const ArmorAimContext& context, const GimbalTransform& gimbal,
    const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
    double prediction_delay, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {

    TargetPrediction output;
    output.target_type     = get_target_type<TargetType>();
    output.timestamp_ns    = current_ns;
    output.target_velocity = target.velocity;
    output.target_v_yaw    = target.v_yaw;
    output.aim_phase       = context.phase;

    const double age =
        static_cast<double>(static_cast<int64_t>(current_ns) - static_cast<int64_t>(measurement_ns))
        * 1e-9;
    const double total_delay = age + prediction_delay;

    const Eigen::Vector3d muzzle_pos        = muzzle.translation();
    const auto refine_total_prediction_time = [&](int armor_id) noexcept {
        return refine_flying_time(
            solver,
            [&](double dt) -> Eigen::Vector3d {
                return predict_aim_point(target, dt, context, muzzle, armor_id).position
                     - muzzle_pos;
            },
            total_delay, bullet_speed);
    };

    const int armors_num             = target_armors_num(target);
    const auto preferred             = sanitize_preferred(context.preferred_armor_id, armors_num);
    const double front_window        = front_window_rad(config_);
    bool has_preferred               = false;
    bool has_front_best              = false;
    bool has_any_best                = false;
    double preferred_time            = total_delay;
    double front_best_time           = total_delay;
    double any_best_time             = total_delay;
    PredictedAimPoint preferred_pred = {};
    PredictedAimPoint front_best     = {};
    PredictedAimPoint any_best       = {};

    for (int armor_id = 0; armor_id < armors_num; ++armor_id) {
        const double candidate_time = refine_total_prediction_time(armor_id);
        const PredictedAimPoint candidate =
            predict_aim_point(target, candidate_time, context, muzzle, armor_id);
        const double abs_delta = std::abs(candidate.delta_angle);
        if (!has_any_best || abs_delta < std::abs(any_best.delta_angle)) {
            has_any_best  = true;
            any_best      = candidate;
            any_best_time = candidate_time;
        }
        if (abs_delta > front_window) {
            continue;
        }
        if (preferred.has_value() && armor_id == *preferred) {
            has_preferred  = true;
            preferred_pred = candidate;
            preferred_time = candidate_time;
        }
        if (!has_front_best || abs_delta < std::abs(front_best.delta_angle)) {
            has_front_best  = true;
            front_best      = candidate;
            front_best_time = candidate_time;
        }
    }

    const double total_prediction_time =
        has_preferred ? preferred_time : (has_front_best ? front_best_time : any_best_time);
    const PredictedAimPoint predicted =
        has_preferred ? preferred_pred : (has_front_best ? front_best : any_best);

    output.predicted_position      = predicted.position;
    output.predicted_future_ns     = offset_timestamp_ns(measurement_ns, total_prediction_time);
    output.flying_time             = total_prediction_time - total_delay;
    output.selected_armor_id       = predicted.armor_id;
    output.rough_selected_armor_id = predicted.armor_id;

    auto aim_angles =
        compute_gimbal_aim_angles(output.predicted_position, gimbal, muzzle, solver, bullet_speed);
    if (!aim_angles) {
        return std::unexpected(aim_angles.error());
    }

    output.aim_yaw   = aim_angles->first;
    output.aim_pitch = aim_angles->second;
    output.distance  = (output.predicted_position - muzzle.translation()).norm();

    return output;
}

// ============================================================================
// aim_generic_rune - Template method for targets rune
// ============================================================================

template <class TargetType>
std::expected<TargetPrediction, std::string> Aimer::aim_generic_rune(
    const TargetType& target, const GimbalTransform& gimbal, const MuzzleTransform& muzzle,
    uint64_t current_ns, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {

    TargetPrediction output;
    output.target_type     = get_target_type<TargetType>();
    output.timestamp_ns    = current_ns;
    output.target_velocity = Eigen::Vector3d::Zero();
    output.target_v_yaw    = 0.0;

    const Eigen::Vector3d origin_odom = muzzle.translation();

    double image_delay       = static_cast<double>(current_ns - target.timestamp_ns) * 1e-9;
    const double total_delay = image_delay + config_.delay; //

    const auto refine_total_prediction_time = [&]() noexcept {
        return refine_flying_time(
            solver,
            [&](double dt) -> Eigen::Vector3d {
                return predict_position(target, dt) - origin_odom;
            },
            total_delay, bullet_speed);
    };

    double total_time          = refine_total_prediction_time();
    output.predicted_position  = predict_position(target, total_time);
    output.predicted_future_ns = offset_timestamp_ns(target.timestamp_ns, total_time);
    const auto solution =
        solver.solve(output.predicted_position - muzzle.translation(), bullet_speed);
    if (!solution.has_value()) {
        return std::unexpected(solution.error());
    }
    output.flying_time = solution->time_of_flight;

    const auto [yaw, pitch] = compose_gimbal_aim_angles(*solution, gimbal, muzzle);

    output.aim_yaw   = yaw;
    output.aim_pitch = pitch;
    output.distance  = (output.predicted_position - muzzle.translation()).norm();

    // 计算 predicted_roll 用于 Rune
    if constexpr (std::is_same_v<TargetType, energy_meter::EnergyMeterState>) {
        const double current_roll = target.roll + target.blade_id * 2 * M_PI / 5;
        double delta_theta        = 0.0;
        if (target.model_valid) {
            const int dir = target.direction;
            if (target.is_big_rune) {
                const double a  = target.a;
                const double w  = target.omega;
                const double b  = target.b;
                const double t0 = target.t;
                delta_theta     = dir
                            * ((a / w) * (std::cos(w * t0) - std::cos(w * (t0 + total_time)))
                               + b * total_time);
            } else {
                delta_theta = dir * kFixedRuneSpeed * total_time;
            }
        } else {
            delta_theta = target.direction * kFixedRuneSpeed * total_time;
        }
        output.predicted_roll = current_roll + delta_theta;
    }

    return output;
}

// ============================================================================
// aim - Robot
// ============================================================================

std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::RobotTargetState& target, const GimbalTransform& gimbal,
    const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
    double bullet_speed, const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    return aim(
        target, ArmorAimContext{}, gimbal, muzzle, measurement_ns, current_ns, 0.0, bullet_speed,
        solver);
}

std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::RobotTargetState& target, const GimbalTransform& gimbal,
    const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
    double extra_prediction_delay, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    return aim(
        target, ArmorAimContext{}, gimbal, muzzle, measurement_ns, current_ns,
        extra_prediction_delay, bullet_speed, solver);
}

std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::RobotTargetState& target, const ArmorAimContext& context,
    const GimbalTransform& gimbal, const MuzzleTransform& muzzle, uint64_t measurement_ns,
    uint64_t current_ns, double extra_prediction_delay, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    return aim_generic(
        target, context, gimbal, muzzle, measurement_ns, current_ns,
        config_.delay + extra_prediction_delay, bullet_speed, solver);
}

// ============================================================================
// aim - Outpost
// ============================================================================

std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::OutpostTargetState& target, const GimbalTransform& gimbal,
    const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
    double bullet_speed, const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    return aim(
        target, ArmorAimContext{}, gimbal, muzzle, measurement_ns, current_ns, 0.0, bullet_speed,
        solver);
}

std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::OutpostTargetState& target, const GimbalTransform& gimbal,
    const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
    double extra_prediction_delay, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    return aim(
        target, ArmorAimContext{}, gimbal, muzzle, measurement_ns, current_ns,
        extra_prediction_delay, bullet_speed, solver);
}

std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::OutpostTargetState& target, const ArmorAimContext& context,
    const GimbalTransform& gimbal, const MuzzleTransform& muzzle, uint64_t measurement_ns,
    uint64_t current_ns, double extra_prediction_delay, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    return aim_generic(
        target, context, gimbal, muzzle, measurement_ns, current_ns,
        config_.delay + extra_prediction_delay, bullet_speed, solver);
}

// ============================================================================
// aim - Rune (EnergyMeterState)
// ============================================================================

std::expected<TargetPrediction, std::string> Aimer::aim(
    const energy_meter::EnergyMeterState& state, const GimbalTransform& gimbal,
    const MuzzleTransform& muzzle, uint64_t current_ns, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    return aim_generic_rune(state, gimbal, muzzle, current_ns, bullet_speed, solver);
}

std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::ldm::LdmState& target, const GimbalTransform& gimbal, const MuzzleTransform& muzzle,
    uint64_t current_ns, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    return aim(target, gimbal, muzzle, current_ns, 0.0, bullet_speed, solver);
}

std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::ldm::LdmState& target, const GimbalTransform& gimbal, const MuzzleTransform& muzzle,
    uint64_t current_ns, double extra_prediction_delay, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {

    TargetPrediction output;
    output.target_type     = get_target_type<L3::ldm::LdmState>();
    output.timestamp_ns    = current_ns;
    output.target_velocity = target.velocity_world();
    output.target_v_yaw    = 0.0;

    // Use pre-computed predicted position from tracker, re-predicting with extra delay.
    const double age_s =
        static_cast<double>(
            static_cast<int64_t>(current_ns) - static_cast<int64_t>(target.timestamp_ns))
        * 1e-9;
    const double total_delay   = age_s + config_.delay + extra_prediction_delay;
    const auto X_pred          = predict_state(target, std::max(0.0, total_delay));
    output.predicted_position  = X_pred.p();
    output.target_velocity     = X_pred.R() * X_pred.v();
    output.predicted_future_ns = target.timestamp_ns + static_cast<uint64_t>(total_delay * 1.0e9);

    const auto solution =
        solver.solve(output.predicted_position - muzzle.translation(), bullet_speed);
    if (!solution.has_value()) {
        return std::unexpected(solution.error());
    }
    output.flying_time = solution->time_of_flight;

    const auto [yaw, pitch] = compose_gimbal_aim_angles(*solution, gimbal, muzzle);

    output.aim_yaw   = yaw;
    output.aim_pitch = pitch;
    output.distance  = (output.predicted_position - muzzle.translation()).norm();

    return output;
}

group::SEn3<double, 2>
    Aimer::predict_state(const L3::ldm::LdmState& target, double dt) const noexcept {
    // SE2(3) right-perturbation: X̂₊ = X̂ * exp(xi),  xi = (dθ=0, dv=0, dp=v_body*dt)
    using Nominal = group::SEn3<double, 2>;
    using Xi      = Nominal::VectorType;

    if (dt <= 0.0) {
        return target.X;
    }

    Xi xi                     = Xi::Zero();
    xi.template segment<3>(6) = target.X.v() * dt;

    return target.X * Nominal::exp(xi);
}

} // namespace fcs::L4
