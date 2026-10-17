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

// 引入瞄准器头文件
#include "L4_planning/aimer/aimer.hpp"
// 引入LDM吊射目标类型定义
#include "L3_estimation/ldm_naive/types.hpp"
// 引入角度归一化工具
#include "core/math/normalize.hpp"

// 引入C标准库，用于std::abs等
#include <cstdlib>
// 引入SE(3)李群头文件
#include <groups/SEn3.hpp>

// 引入算法库，用于std::find、std::clamp等
#include <algorithm>
// 引入数学库，用于std::sin、std::cos等
#include <cmath>
// 引入数值极限定义
#include <limits>
// 引入数学常数
#include <numbers>
// 引入可选值容器
#include <optional>
// 引入动态数组容器
#include <vector>

// 进入fcs::L4命名空间
namespace fcs::L4 {
// 匿名命名空间，仅本编译单元可见
namespace {

// 引入角度归一化函数到当前作用域
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
    // 如果偏移量不是有限数（NaN/无穷），直接返回基准时间戳
    if (!std::isfinite(offset_s)) {
        return base_ns;
    }

    // 使用long double计算偏移后的时间戳，避免整数溢出
    const long double shifted_ns =
        static_cast<long double>(base_ns) + static_cast<long double>(offset_s) * 1.0e9L;
    // 如果偏移后时间戳小于等于0，返回0
    if (shifted_ns <= 0.0L) {
        return 0;
    }

    // 获取uint64_t的最大值
    const long double max_ns = static_cast<long double>(std::numeric_limits<uint64_t>::max());
    // 如果偏移后时间戳超过最大值，返回最大值（饱和处理）
    if (shifted_ns >= max_ns) {
        return std::numeric_limits<uint64_t>::max();
    }

    // 四舍五入转换为uint64_t并返回
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
    // 如果偏好ID无值，返回空
    if (!preferred.has_value()) {
        return std::nullopt;
    }
    // 如果偏好ID小于0或大于等于装甲板数量，返回空
    if (*preferred < 0 || *preferred >= armors_num) {
        return std::nullopt;
    }
    // 返回有效的偏好ID
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
    // 如果候选列表为空，返回0
    if (candidates.empty()) {
        return 0;
    }

    // 初始最优为第一个候选
    int best        = candidates.front();
    // 初始最优角度差绝对值
    double best_abs = std::abs(delta_angles[static_cast<size_t>(best)]);
    // 遍历所有候选
    for (const int id : candidates) {
        // 当前候选的角度差绝对值
        const double value = std::abs(delta_angles[static_cast<size_t>(id)]);
        // 如果更小，更新最优
        if (value < best_abs) {
            best     = id;
            best_abs = value;
        }
    }
    // 返回最优装甲板ID
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
    // 将度转换为弧度
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
    // 创建过滤结果列表
    std::vector<int> filtered;
    // 预分配空间
    filtered.reserve(candidates.size());
    // 遍历所有候选
    for (const int id : candidates) {
        // 如果角度差在窗口内，加入结果
        if (std::abs(delta_angles[static_cast<size_t>(id)]) <= front_window) {
            filtered.push_back(id);
        }
    }
    // 返回过滤结果
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
    // 创建角度差数组
    std::vector<double> deltas;
    // 预分配空间
    deltas.reserve(N);
    // 遍历所有装甲板位姿
    for (const auto& pose : armor_poses) {
        // 计算归一化角度差（装甲板yaw - 车身中心yaw）
        deltas.push_back(normalize_angle(pose[3] - center_yaw));
    }
    // 返回角度差数组
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
    // 获取装甲板数量
    const int armors_num = static_cast<int>(N);

    // 如果目标没有跳变，默认选择0号装甲板
    if (!context.target_jumped) {
        return 0;
    }

    // 构建角度差数组
    const std::vector<double> delta_angles = build_delta_angles(armor_poses, center_yaw);

    // 创建所有装甲板ID列表
    std::vector<int> all_ids;
    // 预分配空间
    all_ids.reserve(armors_num);
    // 填充ID列表
    for (int i = 0; i < armors_num; ++i) {
        all_ids.push_back(i);
    }

    // 过滤前向窗口内的候选装甲板
    const auto candidates = filter_front_window(all_ids, delta_angles, front_window_rad(config));
    // 清洗偏好装甲板ID
    const auto preferred  = sanitize_preferred(context.preferred_armor_id, armors_num);
    // 如果候选列表非空
    if (!candidates.empty()) {
        // 如果偏好ID存在且在候选中，返回偏好ID
        if (preferred.has_value()
            && std::find(candidates.begin(), candidates.end(), *preferred) != candidates.end()) {
            return *preferred;
        }
        // 否则返回候选中的最小角度差装甲板
        return pick_best_by_min_delta(candidates, delta_angles);
    }

    // 忽略前哨站标记（预留参数）
    (void)is_outpost;
    // 前向窗口内无候选，从所有装甲板中选择最小角度差
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
    // 计算从枪口到目标中心的2D射线
    Eigen::Vector2d ray   = center.head<2>() - muzzle_pos.head<2>();
    // 计算射线长度
    const double ray_norm = ray.norm();
    // 如果射线长度有效，归一化
    if (ray_norm > 1e-6) {
        ray /= ray_norm;
    } else {
        // 否则使用目标中心yaw作为射线方向
        const double yaw = center_yaw_from_xy(center.x(), center.y());
        ray              = Eigen::Vector2d{std::cos(yaw), std::sin(yaw)};
    }

    // 计算投影后的2D位置（沿射线后退半径距离）
    const Eigen::Vector2d projected_xy = center.head<2>() - radius * ray;
    // 返回3D投影位置，z使用装甲板z坐标
    return Eigen::Vector3d{projected_xy.x(), projected_xy.y(), armor_z};
}

/**
 * @brief 获取目标装甲板数量（机器人）
 *
 * @param target 机器人目标状态
 * @return 装甲板数量（限制在[1,4]范围内）
 */
[[nodiscard]] int target_armors_num(const L3::RobotTargetState& target) noexcept {
    // 钳位装甲板数量到[1,4]
    return std::clamp(target.armors_num, 1, 4);
}

/**
 * @brief 获取目标装甲板数量（前哨站）
 *
 * @param target 前哨站目标状态（未使用）
 * @return 固定3块装甲板
 */
[[nodiscard]] int target_armors_num(const L3::OutpostTargetState& /*target*/) noexcept {
    // 返回前哨站固定装甲板数量
    return L3::OutpostTargetState::armors_num;
}

} // namespace

// ============================================================================
// predict_position - compatibility helpers
// ============================================================================

// 能量机关位置预测（兼容接口）
Eigen::Vector3d Aimer::predict_position(
    const energy_meter::EnergyMeterState& target, double predict_time) const noexcept {
    // 获取能量机关旋转中心
    const Eigen::Vector3d r_center = target.r_center_odom;

    // 直接使用tracker计算的radius，避免从position推导引入数值误差
    // 这对上下位置（roll≈±90°）的精度尤其重要
    const double radius = target.radius;

    // 当前 roll 来自 UKF
    const double current_roll = target.roll + target.blade_id * 2 * M_PI / 5;
    // 初始化角度增量
    double delta_theta        = 0.0;
    // 获取旋转方向
    const int dir             = target.direction;

    // 预测 roll 变化
    if (target.model_valid) {
        // 如果是大能量机关
        if (target.is_big_rune) {
            // Δθ = dir · [(a/ω)·(cos(ω·t) - cos(ω·(t+dt))) + b·dt]
            const double a     = target.a;
            const double omega = target.omega;
            const double b     = target.b;
            const double t0    = target.t;

            // 计算大能量机关的角度增量
            delta_theta =
                dir
                * ((a / omega) * (std::cos(omega * t0) - std::cos(omega * (t0 + predict_time)))
                   + b * predict_time);
        } else {
            // 小能量机关固定转速
            delta_theta = dir * kFixedRuneSpeed * predict_time;
        }
    }

    // 预测 roll
    const double predicted_roll = current_roll + delta_theta;

    // 基于 UKF 的 roll、pitch、yaw 计算预测位置
    // 叶片在竖直平面 (Y-Z) 内绕水平轴 (X) 旋转 roll 角度
    const double sr = std::sin(predicted_roll);
    const double cr = std::cos(predicted_roll);
    // 计算叶片局部坐标
    Eigen::Vector3d blade_local(0.0, -radius * sr, radius * cr);

    // 应用 pitch 旋转 (绕 Y 轴)
    const double sp = std::sin(target.pitch);
    const double cp = std::cos(target.pitch);
    // 绕Y轴旋转叶片局部坐标
    blade_local     = Eigen::Vector3d(
        blade_local.x() * cp + blade_local.z() * sp, blade_local.y(),
        -blade_local.x() * sp + blade_local.z() * cp);

    // 应用 yaw 旋转 (绕 Z 轴)，再平移到圆心
    const double cy = std::cos(target.yaw);
    const double sy = std::sin(target.yaw);
    // 绕Z轴旋转得到世界坐标偏移
    const Eigen::Vector3d blade_offset(
        blade_local.x() * cy - blade_local.y() * sy, blade_local.x() * sy + blade_local.y() * cy,
        blade_local.z());

    // 预测位置 = 旋转中心 + 叶片偏移
    Eigen::Vector3d predicted_position = r_center + blade_offset;

    // 返回预测位置
    return predicted_position;
}

// 机器人位置预测（兼容接口）
Eigen::Vector3d
    Aimer::predict_position(const L3::RobotTargetState& target, double dt) const noexcept {
    // 调用predict_aim_point获取预测位置
    return predict_aim_point(target, dt, {}, {}).position;
}

// 前哨站位置预测（兼容接口）
Eigen::Vector3d
    Aimer::predict_position(const L3::OutpostTargetState& target, double dt) const noexcept {
    // 调用predict_aim_point获取预测位置
    return predict_aim_point(target, dt, {}, {}).position;
}

// 机器人瞄准点预测
Aimer::PredictedAimPoint Aimer::predict_aim_point(
    const L3::RobotTargetState& target, double dt, const ArmorAimContext& context,
    const MuzzleTransform& muzzle, std::optional<int> forced_armor_id) const noexcept {
    // 拷贝目标状态
    L3::RobotTargetState predicted = target;
    // 预测位置：当前位置 + 速度*dt
    predicted.position += predicted.velocity * dt;
    // 预测yaw：当前yaw + 角速度*dt
    predicted.yaw += predicted.v_yaw * dt;

    // 获取预测后的装甲板位姿
    const auto armor_poses  = predicted.armor_poses();
    // 计算车身中心yaw
    const double center_yaw = center_yaw_from_xy(predicted.position.x(), predicted.position.y());

    // 选择装甲板ID：优先使用强制指定的，否则调用选择算法
    const int armor_id =
        sanitize_preferred(forced_armor_id, predicted.armors_num)
            .value_or(select_armor_id(armor_poses, center_yaw, context, config_, false));
    // 获取选中装甲板的位姿
    const Eigen::Vector4d& armor_pose = armor_poses[static_cast<size_t>(armor_id)];
    // 根据装甲板奇偶选择半径
    const double radius               = (armor_id % 2 == 0) ? predicted.radius0 : predicted.radius1;

    // 创建预测瞄准点结果
    PredictedAimPoint result;
    // 记录装甲板ID
    result.armor_id    = armor_id;
    // 记录角度差
    result.delta_angle = normalize_angle(armor_pose[3] - center_yaw);
    // 记录装甲板位置
    result.position    = armor_pose.head<3>();

    // 如果是整车中心瞄准阶段
    if (context.phase == ArmorAimPhase::WholeCarCenter) {
        // 计算中心代理瞄准位置
        result.position =
            project_center_aim(predicted.position, muzzle.translation(), radius, armor_pose[2]);
    }
    // 返回结果
    return result;
}

// 前哨站瞄准点预测
Aimer::PredictedAimPoint Aimer::predict_aim_point(
    const L3::OutpostTargetState& target, double dt, const ArmorAimContext& context,
    const MuzzleTransform& muzzle, std::optional<int> forced_armor_id) const noexcept {
    // 拷贝目标状态
    L3::OutpostTargetState predicted = target;
    // 预测位置：当前位置 + 2D速度*dt
    predicted.position += predicted.velocity.head<2>() * dt;
    // 预测yaw：当前yaw + 角速度*dt
    predicted.yaw += predicted.v_yaw * dt;

    // 获取预测后的装甲板位姿
    const auto armor_poses  = predicted.armor_poses();
    // 计算车身中心yaw
    const double center_yaw = center_yaw_from_xy(predicted.position.x(), predicted.position.y());
    // 选择装甲板ID
    const int armor_id =
        sanitize_preferred(forced_armor_id, L3::OutpostTargetState::armors_num)
            .value_or(select_armor_id(armor_poses, center_yaw, context, config_, true));
    // 获取选中装甲板的位姿
    const Eigen::Vector4d& armor_pose = armor_poses[static_cast<size_t>(armor_id)];

    // 创建预测瞄准点结果
    PredictedAimPoint result;
    // 记录装甲板ID
    result.armor_id    = armor_id;
    // 记录角度差
    result.delta_angle = normalize_angle(armor_pose[3] - center_yaw);
    // 记录装甲板位置
    result.position    = armor_pose.head<3>();

    // 如果是整车中心瞄准阶段
    if (context.phase == ArmorAimPhase::WholeCarCenter) {
        // 计算中心代理瞄准位置（z=0平面）
        result.position = project_center_aim(
            Eigen::Vector3d{predicted.position.x(), predicted.position.y(), 0.0},
            muzzle.translation(), L3::OutpostTargetState::radius, armor_pose[2]);
    }

    // 返回结果
    return result;
}

// ============================================================================
// aim_generic - Template method for targets with prediction
// ============================================================================

// 通用瞄准模板方法（适用于机器人/前哨站）
template <class TargetType>
std::expected<TargetPrediction, std::string> Aimer::aim_generic(
    const TargetType& target, const ArmorAimContext& context, const GimbalTransform& gimbal,
    const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
    double prediction_delay, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {

    // 创建瞄准预测输出
    TargetPrediction output;
    // 设置目标类型
    output.target_type     = get_target_type<TargetType>();
    // 设置时间戳
    output.timestamp_ns    = current_ns;
    // 设置目标速度
    output.target_velocity = target.velocity;
    // 设置目标yaw角速度
    output.target_v_yaw    = target.v_yaw;
    // 设置瞄准阶段
    output.aim_phase       = context.phase;

    // 计算目标年龄（从测量到当前的时间差，秒）从现在到取图那一刻的时间差
    const double age =
        static_cast<double>(static_cast<int64_t>(current_ns) - static_cast<int64_t>(measurement_ns))
        * 1e-9;
    // 总延迟 = 目标年龄 + 预测延迟
    const double total_delay = age + prediction_delay;

    // 获取枪口位置
    const Eigen::Vector3d muzzle_pos        = muzzle.translation();
    // 定义飞行时间精化lambda（针对指定装甲板）
    const auto refine_total_prediction_time = [&](int armor_id) noexcept {
        // 调用refine_flying_time精化飞行时间
        return refine_flying_time(
            solver,
            // 预测函数：返回预测瞄准点与枪口的位置差
            [&](double dt) -> Eigen::Vector3d {
                return predict_aim_point(target, dt, context, muzzle, armor_id).position - muzzle_pos;
            },
            total_delay, bullet_speed);
    };

    // 获取装甲板数量
    const int armors_num             = target_armors_num(target);
    // 清洗偏好装甲板ID
    const auto preferred= sanitize_preferred(context.preferred_armor_id, armors_num);
    // 获取前向窗口半径
    const double front_window        = front_window_rad(config_);
    // 是否找到偏好装甲板
    bool has_preferred               = false;
    // 是否找到前向窗口内最优
    bool has_front_best              = false;
    // 是否找到任意最优
    bool has_any_best                = false;
    // 偏好装甲板的预测时间
    double preferred_time            = total_delay;
    // 前向窗口最优的预测时间
    double front_best_time           = total_delay;
    // 任意最优的预测时间
    double any_best_time             = total_delay;
    // 偏好装甲板的预测结果
    PredictedAimPoint preferred_pred = {};
    // 前向窗口最优的预测结果
    PredictedAimPoint front_best     = {};
    // 任意最优的预测结果
    PredictedAimPoint any_best       = {};

    // 遍历所有装甲板
    for (int armor_id = 0; armor_id < armors_num; ++armor_id) {
        // 精化该装甲板的预测时间
        const double candidate_time = refine_total_prediction_time(armor_id);
        // 预测该装甲板的瞄准点
        const PredictedAimPoint candidate =
            predict_aim_point(target, candidate_time, context, muzzle, armor_id);
        // 计算角度差绝对值
        const double abs_delta = std::abs(candidate.delta_angle);
        // 更新任意最优
        if (!has_any_best || abs_delta < std::abs(any_best.delta_angle)) {
            has_any_best  = true;
            any_best      = candidate;
            any_best_time = candidate_time;
        }
        // 如果角度差超出前向窗口，跳过
        if (abs_delta > front_window) {
            continue;
        }
        // 如果是偏好装甲板
        if (preferred.has_value() && armor_id == *preferred) {
            has_preferred  = true;
            preferred_pred = candidate;
            preferred_time = candidate_time;
        }
        // 更新前向窗口最优
        if (!has_front_best || abs_delta < std::abs(front_best.delta_angle)) {
            has_front_best  = true;
            front_best      = candidate;
            front_best_time = candidate_time;
        }
    }

    // 选择最终预测时间：优先偏好 > 前向窗口最优 > 任意最优
    const double total_prediction_time =
        has_preferred ? preferred_time : (has_front_best ? front_best_time : any_best_time);
    // 选择最终预测结果：优先偏好 > 前向窗口最优 > 任意最优
    const PredictedAimPoint predicted =
        has_preferred ? preferred_pred : (has_front_best ? front_best : any_best);

    // 记录预测位置
    output.predicted_position      = predicted.position;
    // 记录预测未来时间戳
    output.predicted_future_ns     = offset_timestamp_ns(measurement_ns, total_prediction_time);
    // 记录飞行时间（总预测时间 - 总延迟）
    output.flying_time             = total_prediction_time - total_delay;
    // 记录选中装甲板ID
    output.selected_armor_id       = predicted.armor_id;
    // 记录粗略选中装甲板ID
    output.rough_selected_armor_id = predicted.armor_id;

    // 计算云台瞄准角度
    auto aim_angles =
        compute_gimbal_aim_angles(output.predicted_position, gimbal, muzzle, solver, bullet_speed);
    // 如果计算失败
    if (!aim_angles) {
        // 返回错误
        return std::unexpected(aim_angles.error());
    }

    // 记录瞄准yaw角
    output.aim_yaw   = aim_angles->first;
    // 记录瞄准pitch角
    output.aim_pitch = aim_angles->second;
    // 记录距离
    output.distance  = (output.predicted_position - muzzle.translation()).norm();

    // 返回瞄准预测结果
    return output;
}

// ============================================================================
// aim_generic_rune - Template method for targets rune
// ============================================================================

// 能量机关通用瞄准模板方法
template <class TargetType>
std::expected<TargetPrediction, std::string> Aimer::aim_generic_rune(
    const TargetType& target, const GimbalTransform& gimbal, const MuzzleTransform& muzzle,
    uint64_t current_ns, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {

    // 创建瞄准预测输出
    TargetPrediction output;
    // 设置目标类型
    output.target_type     = get_target_type<TargetType>();
    // 设置时间戳
    output.timestamp_ns    = current_ns;
    // 能量机关无速度，设为0
    output.target_velocity = Eigen::Vector3d::Zero();
    // 能量机关无yaw角速度，设为0
    output.target_v_yaw    = 0.0;

    // 获取枪口位置（世界坐标）
    const Eigen::Vector3d origin_odom = muzzle.translation();

    // 计算图像延迟（当前时间 - 目标时间戳）
    double image_delay       = static_cast<double>(current_ns - target.timestamp_ns) * 1e-9;
    // 总延迟 = 图像延迟 + 配置延迟
    const double total_delay = image_delay + config_.delay; //

    // 定义飞行时间精化lambda
    const auto refine_total_prediction_time = [&]() noexcept {
        // 调用refine_flying_time精化飞行时间
        return refine_flying_time(
            solver,
            // 预测函数：返回预测位置与枪口的位置差
            [&](double dt) -> Eigen::Vector3d {
                return predict_position(target, dt) - origin_odom;
            },
            total_delay, bullet_speed);
    };

    // 精化总预测时间
    double total_time          = refine_total_prediction_time();
    // 预测位置
    output.predicted_position  = predict_position(target, total_time);
    // 预测未来时间戳
    output.predicted_future_ns = offset_timestamp_ns(target.timestamp_ns, total_time);
    // 求解弹道
    const auto solution =
        solver.solve(output.predicted_position - muzzle.translation(), bullet_speed);
    // 如果求解失败
    if (!solution.has_value()) {
        // 返回错误
        return std::unexpected(solution.error());
    }
    // 记录飞行时间
    output.flying_time = solution->time_of_flight;

    // 组合云台瞄准角度
    const auto [yaw, pitch] = compose_gimbal_aim_angles(*solution, gimbal, muzzle);

    // 记录瞄准yaw角
    output.aim_yaw   = yaw;
    // 记录瞄准pitch角
    output.aim_pitch = pitch;
    // 记录距离
    output.distance  = (output.predicted_position - muzzle.translation()).norm();

    // 计算 predicted_roll 用于 Rune
    if constexpr (std::is_same_v<TargetType, energy_meter::EnergyMeterState>) {
        // 当前roll（含叶片偏移）
        const double current_roll = target.roll + target.blade_id * 2 * M_PI / 5;
        // 初始化角度增量
        double delta_theta        = 0.0;
        // 如果模型有效
        if (target.model_valid) {
            // 获取旋转方向
            const int dir = target.direction;
            // 如果是大能量机关
            if (target.is_big_rune) {
                const double a  = target.a;
                const double w  = target.omega;
                const double b  = target.b;
                const double t0 = target.t;
                // 计算大能量机关角度增量
                delta_theta     = dir
                            * ((a / w) * (std::cos(w * t0) - std::cos(w * (t0 + total_time)))
                               + b * total_time);
            } else {
                // 小能量机关固定转速
                delta_theta = dir * kFixedRuneSpeed * total_time;
            }
        } else {
            // 模型无效时使用固定转速
            delta_theta = target.direction * kFixedRuneSpeed * total_time;
        }
        // 记录预测roll
        output.predicted_roll = current_roll + delta_theta;
    }

    // 返回瞄准预测结果
    return output;
}

// ============================================================================
// aim - Robot
// ============================================================================

// 机器人瞄准（无上下文、无额外延迟）
std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::RobotTargetState& target, const GimbalTransform& gimbal,
    const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
    double bullet_speed, const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    // 转发到带上下文的重载
    return aim(
        target, ArmorAimContext{}, gimbal, muzzle, measurement_ns, current_ns, 0.0, bullet_speed,
        solver);
}

// 机器人瞄准（无上下文、有额外延迟）
std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::RobotTargetState& target, const GimbalTransform& gimbal,
    const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
    double extra_prediction_delay, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    // 转发到带上下文的重载
    return aim(
        target, ArmorAimContext{}, gimbal, muzzle, measurement_ns, current_ns,
        extra_prediction_delay, bullet_speed, solver);
}

// 机器人瞄准（带上下文、有额外延迟）
std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::RobotTargetState& target, const ArmorAimContext& context,
    const GimbalTransform& gimbal, const MuzzleTransform& muzzle, uint64_t measurement_ns,
    uint64_t current_ns, double extra_prediction_delay, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    // 调用通用瞄准模板，配置延迟+额外延迟
    return aim_generic(
        target, context, gimbal, muzzle, measurement_ns, current_ns,
        config_.delay + extra_prediction_delay, bullet_speed, solver);
}

// ============================================================================
// aim - Outpost
// ============================================================================

// 前哨站瞄准（无上下文、无额外延迟）
std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::OutpostTargetState& target, const GimbalTransform& gimbal,
    const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
    double bullet_speed, const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    // 转发到带上下文的重载
    return aim(
        target, ArmorAimContext{}, gimbal, muzzle, measurement_ns, current_ns, 0.0, bullet_speed,
        solver);
}

// 前哨站瞄准（无上下文、有额外延迟）
std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::OutpostTargetState& target, const GimbalTransform& gimbal,
    const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
    double extra_prediction_delay, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    // 转发到带上下文的重载
    return aim(
        target, ArmorAimContext{}, gimbal, muzzle, measurement_ns, current_ns,
        extra_prediction_delay, bullet_speed, solver);
}

// 前哨站瞄准（带上下文、有额外延迟）
std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::OutpostTargetState& target, const ArmorAimContext& context,
    const GimbalTransform& gimbal, const MuzzleTransform& muzzle, uint64_t measurement_ns,
    uint64_t current_ns, double extra_prediction_delay, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    // 调用通用瞄准模板，配置延迟+额外延迟
    return aim_generic(
        target, context, gimbal, muzzle, measurement_ns, current_ns,
        config_.delay + extra_prediction_delay, bullet_speed, solver);
}

// ============================================================================
// aim - Rune (EnergyMeterState)
// ============================================================================

// 能量机关瞄准
std::expected<TargetPrediction, std::string> Aimer::aim(
    const energy_meter::EnergyMeterState& state, const GimbalTransform& gimbal,
    const MuzzleTransform& muzzle, uint64_t current_ns, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    // 调用能量机关通用瞄准模板
    return aim_generic_rune(state, gimbal, muzzle, current_ns, bullet_speed, solver);
}

// LDM瞄准（无额外延迟）
std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::ldm::LdmState& target, const GimbalTransform& gimbal, const MuzzleTransform& muzzle,
    uint64_t current_ns, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {
    // 转发到带额外延迟的重载
    return aim(target, gimbal, muzzle, current_ns, 0.0, bullet_speed, solver);
}

// LDM瞄准（带额外延迟）
std::expected<TargetPrediction, std::string> Aimer::aim(
    const L3::ldm::LdmState& target, const GimbalTransform& gimbal, const MuzzleTransform& muzzle,
    uint64_t current_ns, double extra_prediction_delay, double bullet_speed,
    const core::trajectory::solver::TrajectorySolver& solver) const noexcept {

    // 创建瞄准预测输出
    TargetPrediction output;
    // 设置目标类型
    output.target_type     = get_target_type<L3::ldm::LdmState>();
    // 设置时间戳
    output.timestamp_ns    = current_ns;
    // 设置目标速度（世界坐标系）
    output.target_velocity = target.velocity_world();
    // LDM无yaw角速度
    output.target_v_yaw    = 0.0;

    // Use pre-computed predicted position from tracker, re-predicting with extra delay.
    // 计算目标年龄（秒）
    const double age_s =
        static_cast<double>(
            static_cast<int64_t>(current_ns) - static_cast<int64_t>(target.timestamp_ns))
        * 1e-9;
    // 总延迟 = 年龄 + 配置延迟 + 额外延迟
    const double total_delay   = age_s + config_.delay + extra_prediction_delay;
    // 预测状态（SE2(3)李群）
    const auto X_pred          = predict_state(target, std::max(0.0, total_delay));
    // 记录预测位置
    output.predicted_position  = X_pred.p();
    // 记录预测速度（旋转到世界坐标系）
    output.target_velocity     = X_pred.R() * X_pred.v();
    // 记录预测未来时间戳
    output.predicted_future_ns = target.timestamp_ns + static_cast<uint64_t>(total_delay * 1.0e9);

    // 求解弹道
    const auto solution =
        solver.solve(output.predicted_position - muzzle.translation(), bullet_speed);
    // 如果求解失败
    if (!solution.has_value()) {
        // 返回错误
        return std::unexpected(solution.error());
    }
    // 记录飞行时间
    output.flying_time = solution->time_of_flight;

    // 组合云台瞄准角度
    const auto [yaw, pitch] = compose_gimbal_aim_angles(*solution, gimbal, muzzle);

    // 记录瞄准yaw角
    output.aim_yaw   = yaw;
    // 记录瞄准pitch角
    output.aim_pitch = pitch;
    // 记录距离
    output.distance  = (output.predicted_position - muzzle.translation()).norm();

    // 返回瞄准预测结果
    return output;
}

// LDM状态预测（SE2(3)李群右扰动）
group::SEn3<double, 2>
    Aimer::predict_state(const L3::ldm::LdmState& target, double dt) const noexcept {
    // SE2(3) right-perturbation: X̂₊ = X̂ * exp(xi),  xi = (dθ=0, dv=0, dp=v_body*dt)
    // 定义标称李群类型
    using Nominal = group::SEn3<double, 2>;
    // 定义李代数向量类型
    using Xi      = Nominal::VectorType;

    // 如果dt小于等于0，直接返回原始状态
    if (dt <= 0.0) {
        return target.X;
    }

    // 初始化李代数向量为0
    Xi xi                     = Xi::Zero();
    // 设置平移部分为体坐标系速度*dt
    xi.template segment<3>(6) = target.X.v() * dt;

    // 返回右扰动后的李群状态
    return target.X * Nominal::exp(xi);
}

// 关闭fcs::L4命名空间
} // namespace fcs::L4