/**
 * @file aimer.hpp
 * @brief 瞄准点预测核心模块
 *
 * 本文件定义了Aimer类，负责为所有目标类型（Robot/Outpost/Rune/LDM）计算预测瞄准点。
 * Aimer是L4规划层的关键组件，接收L3估计层的目标状态，输出带有弹道补偿的瞄准角度。
 *
 * 核心功能：
 * - 目标位置预测：根据目标运动模型预测未来时刻的位置
 * - 装甲板选择：根据目标姿态和瞄准策略选择最佳装甲板
 * - 弹道补偿：考虑重力、空气阻力的弹道轨迹求解
 * - 云台角度计算：将瞄准点转换为云台控制角度
 *
 * 设计模式：
 * - 使用重载aim()方法处理不同目标类型（Robot/Outpost/Rune/LDM）
 * - 使用模板方法模式实现通用预测逻辑
 * - 使用std::expected处理计算失败，避免异常传播
 */
#pragma once

#include <Eigen/Core>
#include <expected>
#include <string>

#include <groups/SEn3.hpp>

#include "L3_estimation/energy_meter/types.hpp"
#include "L3_estimation/ldm_naive/types.hpp"
#include "L3_estimation/tracker/types.hpp"
#include "L4_planning/aimer/aimer_utils.hpp"
#include "L4_planning/aimer/types.hpp"
#include "L4_planning/config.hpp"
#include "core/trajectory/solver/solver_interfaces.hpp"
#include "frame.hpp"

namespace fcs::L4 {

/**
 * @class Aimer
 * @brief 瞄准点预测核心类，为所有目标类型提供统一的预测接口
 *
 * Aimer类是L4规划层的核心组件，负责将L3估计层输出的目标状态转换为
 * 带有弹道补偿的云台控制角度。该类使用重载的aim()方法支持多种目标类型。
 *
 * 核心算法流程：
 * 1. 根据目标类型选择合适的预测方法
 * 2. 计算预测延迟（系统延迟 + 弹道飞行时间）
 * 3. 预测目标在未来时刻的位置和姿态
 * 4. 选择最佳装甲板作为打击目标
 * 5. 计算弹道轨迹并求解瞄准角度
 *
 * 使用示例：
 * @code
 * Aimer aimer(config);
 * auto result = aimer.aim(robot_target, gimbal_tf, muzzle_tf, ...);
 * if (result.has_value()) {
 *     // 使用预测结果控制云台
 * }
 * @endcode
 */
class Aimer {
public:
    /// 云台坐标系变换类型（从odom到gimbal_pitch）
    using GimbalTransform = fast_tf::FrameTransform<fast_tf::odom, fast_tf::gimbal_pitch>;
    /// 枪口坐标系变换类型（从odom到muzzle）
    using MuzzleTransform = fast_tf::FrameTransform<fast_tf::odom, fast_tf::muzzle>;

    /**
     * @brief 构造函数
     * @param config 瞄准器配置参数（包含预测延迟、瞄准策略等）
     */
    explicit Aimer(const AimerConfig& config) noexcept
        : config_(config) {}

    /**
     * @brief 获取配置参数
     * @return 配置参数的const引用
     */
    [[nodiscard]] const AimerConfig& config() const noexcept { return config_; }

    /**
     * @brief 为机器人目标计算瞄准点（步兵/英雄/哨兵）
     *
     * 核心算法：
     * 1. 预测目标在未来时刻的位置（考虑系统延迟和弹道飞行时间）
     * 2. 根据目标姿态选择最佳装甲板
     * 3. 计算弹道轨迹并求解瞄准角度
     *
     * @param target 机器人目标状态（包含位置、速度、姿态等）
     * @param gimbal 云台坐标系变换
     * @param muzzle 枪口坐标系变换
     * @param measurement_ns 测量时间戳（纳秒）
     * @param current_ns 当前时间戳（纳秒）
     * @param bullet_speed 弹丸速度（米/秒）
     * @param solver 弹道求解器
     * @return 预测结果或错误信息
     */
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::RobotTargetState& target, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
            double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /**
     * @brief 为机器人目标计算瞄准点（带额外预测延迟）
     * @param target 机器人目标状态
     * @param gimbal 云台坐标系变换
     * @param muzzle 枪口坐标系变换
     * @param measurement_ns 测量时间戳（纳秒）
     * @param current_ns 当前时间戳（纳秒）
     * @param extra_prediction_delay 额外预测延迟（秒），叠加在配置延迟之上
     * @param bullet_speed 弹丸速度（米/秒）
     * @param solver 弹道求解器
     * @return 预测结果或错误信息
     */
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::RobotTargetState& target, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
            double extra_prediction_delay, double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /**
     * @brief 为机器人目标计算瞄准点（带瞄准上下文）
     * @param target 机器人目标状态
     * @param context 瞄准上下文（包含瞄准阶段、偏好装甲板等）
     * @param gimbal 云台坐标系变换
     * @param muzzle 枪口坐标系变换
     * @param measurement_ns 测量时间戳（纳秒）
     * @param current_ns 当前时间戳（纳秒）
     * @param extra_prediction_delay 额外预测延迟（秒）
     * @param bullet_speed 弹丸速度（米/秒）
     * @param solver 弹道求解器
     * @return 预测结果或错误信息
     */
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::RobotTargetState& target, const ArmorAimContext& context,
            const GimbalTransform& gimbal, const MuzzleTransform& muzzle, uint64_t measurement_ns,
            uint64_t current_ns, double extra_prediction_delay, double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /**
     * @brief 为前哨站目标计算瞄准点
     *
     * 前哨站目标特点：
     * - 固定位置旋转，具有周期性运动
     * - 需要预测其旋转角度和装甲板位置
     *
     * @param target 前哨站目标状态
     * @param gimbal 云台坐标系变换
     * @param muzzle 枪口坐标系变换
     * @param measurement_ns 测量时间戳（纳秒）
     * @param current_ns 当前时间戳（纳秒）
     * @param bullet_speed 弹丸速度（米/秒）
     * @param solver 弹道求解器
     * @return 预测结果或错误信息
     */
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::OutpostTargetState& target, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
            double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /**
     * @brief 为前哨站目标计算瞄准点（带额外预测延迟）
     * @param target 前哨站目标状态
     * @param gimbal 云台坐标系变换
     * @param muzzle 枪口坐标系变换
     * @param measurement_ns 测量时间戳（纳秒）
     * @param current_ns 当前时间戳（纳秒）
     * @param extra_prediction_delay 额外预测延迟（秒）
     * @param bullet_speed 弹丸速度（米/秒）
     * @param solver 弹道求解器
     * @return 预测结果或错误信息
     */
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::OutpostTargetState& target, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
            double extra_prediction_delay, double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /**
     * @brief 为前哨站目标计算瞄准点（带瞄准上下文）
     * @param target 前哨站目标状态
     * @param context 瞄准上下文
     * @param gimbal 云台坐标系变换
     * @param muzzle 枪口坐标系变换
     * @param measurement_ns 测量时间戳（纳秒）
     * @param current_ns 当前时间戳（纳秒）
     * @param extra_prediction_delay 额外预测延迟（秒）
     * @param bullet_speed 弹丸速度（米/秒）
     * @param solver 弹道求解器
     * @return 预测结果或错误信息
     */
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::OutpostTargetState& target, const ArmorAimContext& context,
            const GimbalTransform& gimbal, const MuzzleTransform& muzzle, uint64_t measurement_ns,
            uint64_t current_ns, double extra_prediction_delay, double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /**
     * @brief 为能量机关目标计算瞄准点
     *
     * 能量机关特点：
     * - 大幅度旋转运动，需要预测其旋转角度
     * - 不需要考虑系统延迟补偿（相对位置固定）
     *
     * @param state 能量机关状态
     * @param gimbal 云台坐标系变换
     * @param muzzle 枪口坐标系变换
     * @param current_ns 当前时间戳（纳秒）
     * @param bullet_speed 弹丸速度（米/秒）
     * @param solver 弹道求解器
     * @return 预测结果或错误信息
     */
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const energy_meter::EnergyMeterState& state, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t current_ns, double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /**
     * @brief 为LDM目标计算瞄准点
     *
     * LDM（Light Decision Module）目标特点：
     * - 使用SEn3(2)群表示位姿（位置+速度）
     * - 支持更复杂的运动模型
     *
     * @param state LDM目标状态
     * @param gimbal 云台坐标系变换
     * @param muzzle 枪口坐标系变换
     * @param current_ns 当前时间戳（纳秒）
     * @param bullet_speed 弹丸速度（米/秒）
     * @param solver 弹道求解器
     * @return 预测结果或错误信息
     */
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::ldm::LdmState& state, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t current_ns, double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /**
     * @brief 为LDM目标计算瞄准点（带额外预测延迟）
     * @param state LDM目标状态
     * @param gimbal 云台坐标系变换
     * @param muzzle 枪口坐标系变换
     * @param current_ns 当前时间戳（纳秒）
     * @param extra_prediction_delay 额外预测延迟（秒）
     * @param bullet_speed 弹丸速度（米/秒）
     * @param solver 弹道求解器
     * @return 预测结果或错误信息
     */
    [[nodiscard]] std::expected<TargetPrediction, std::string>
        aim(const L3::ldm::LdmState& state, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, uint64_t current_ns, double extra_prediction_delay,
            double bullet_speed,
            const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

private:
    // ========================================================================
    // 目标类型工具
    // ========================================================================

    /**
     * @brief 获取目标类型的枚举值（编译期）
     * @tparam TargetType 目标状态类型
     * @return 对应的AimerTargetType枚举值
     */
    template <class TargetType>
    [[nodiscard]] static constexpr AimerTargetType get_target_type() noexcept;

    // ========================================================================
    // 装甲板选择辅助
    // ========================================================================

    /**
     * @struct PredictedAimPoint
     * @brief 预测的瞄准点信息
     *
     * 用于存储预测的装甲板位置、角度偏差和装甲板ID
     */
    struct PredictedAimPoint {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()}; ///< 预测位置（odom坐标系）
        double delta_angle{0.0};                           ///< 角度偏差（弧度）
        int armor_id{0};                                   ///< 装甲板ID
    };

    // ========================================================================
    // 通用云台瞄准角度计算（所有目标类型共享）
    // ========================================================================

    /**
     * @brief 组合云台瞄准角度
     *
     * 将弹道解算得到的枪口姿态转换为云台控制角度。
     * 核心算法：
     * 1. 计算期望的枪口姿态（RPY）
     * 2. 计算枪口到云台的姿态增量
     * 3. 计算云台的期望姿态并提取yaw/pitch角度
     *
     * @param solution 弹道解算结果
     * @param gimbal 云台坐标系变换
     * @param muzzle 枪口坐标系变换
     * @return <yaw, pitch> 云台控制角度（弧度）
     */
    [[nodiscard]] static std::pair<double, double> compose_gimbal_aim_angles(
        const core::trajectory::solver::AimSolution& solution, const GimbalTransform& gimbal,
        const MuzzleTransform& muzzle) noexcept {
        // 计算期望的枪口姿态
        const auto desired_muzzle = math_fuxk::rpy(0.0, solution.pitch, solution.yaw).so3();
        // 计算枪口到云台的姿态增量
        const auto muzzle_delta = muzzle.so3().inv() * desired_muzzle;
        // 计算云台的期望姿态
        const auto desired_gimbal = gimbal.so3() * muzzle_delta;
        // 提取yaw和pitch角度
        const auto [r, p, y] = math_fuxk::rpy(desired_gimbal).rpy();
        return {y, p};
    }

    /**
     * @brief 计算云台瞄准角度（通用方法）
     *
     * 为所有目标类型提供统一的云台角度计算接口。
     *
     * @param predicted_pos 预测的目标位置（odom坐标系）
     * @param gimbal 云台坐标系变换
     * @param muzzle 枪口坐标系变换
     * @param solver 弹道求解器
     * @param bullet_speed 弹丸速度（米/秒）
     * @return <yaw, pitch> 云台控制角度或错误信息
     */
    [[nodiscard]] static std::expected<std::pair<double, double>, std::string>
        compute_gimbal_aim_angles(
            const Eigen::Vector3d& predicted_pos, const GimbalTransform& gimbal,
            const MuzzleTransform& muzzle, const core::trajectory::solver::TrajectorySolver& solver,
            double bullet_speed) noexcept {
        // 求解弹道轨迹
        const auto solution = solver.solve(predicted_pos - muzzle.translation(), bullet_speed);
        if (!solution.has_value()) {
            return std::unexpected(solution.error());
        }
        // 组合云台角度
        return compose_gimbal_aim_angles(*solution, gimbal, muzzle);
    }

    // ========================================================================
    // 通用aim() - 模板方法实现通用瞄准逻辑
    // ========================================================================

    /**
     * @brief 通用瞄准实现（带预测的目标：Robot/Outpost）
     *
     * 模板方法，为需要预测的目标提供统一的处理流程：
     * 1. 计算预测延迟
     * 2. 预测目标位置
     * 3. 选择装甲板
     * 4. 计算弹道和云台角度
     *
     * @tparam TargetType 目标状态类型
     * @param target 目标状态
     * @param context 瞄准上下文
     * @param gimbal 云台坐标系变换
     * @param muzzle 枪口坐标系变换
     * @param measurement_ns 测量时间戳（纳秒）
     * @param current_ns 当前时间戳（纳秒）
     * @param prediction_delay 预测延迟（秒）
     * @param bullet_speed 弹丸速度（米/秒）
     * @param solver 弹道求解器
     * @return 预测结果或错误信息
     */
    template <class TargetType>
    [[nodiscard]] std::expected<TargetPrediction, std::string> aim_generic(
        const TargetType& target, const ArmorAimContext& context, const GimbalTransform& gimbal,
        const MuzzleTransform& muzzle, uint64_t measurement_ns, uint64_t current_ns,
        double prediction_delay, double bullet_speed,
        const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /**
     * @brief 通用瞄准实现（不带预测的目标：Rune）
     *
     * 为不需要预测的目标提供简化的处理流程。
     *
     * @tparam TargetType 目标状态类型
     * @param target 目标状态
     * @param gimbal 云台坐标系变换
     * @param muzzle 枪口坐标系变换
     * @param current_ns 当前时间戳（纳秒）
     * @param bullet_speed 弹丸速度（米/秒）
     * @param solver 弹道求解器
     * @return 预测结果或错误信息
     */
    template <class TargetType>
    [[nodiscard]] std::expected<TargetPrediction, std::string> aim_generic_rune(
        const TargetType& target, const GimbalTransform& gimbal, const MuzzleTransform& muzzle,
        uint64_t current_ns, double bullet_speed,
        const core::trajectory::solver::TrajectorySolver& solver) const noexcept;

    /**
     * @brief 预测机器人目标在未来时刻的位置
     *
     * 使用目标运动模型（位置+速度）进行线性预测。
     *
     * @param target 机器人目标状态
     * @param dt 预测时间间隔（秒）
     * @return 预测的位置向量（odom坐标系）
     */
    [[nodiscard]] Eigen::Vector3d
        predict_position(const L3::RobotTargetState& target, double dt) const noexcept;

    /**
     * @brief 预测前哨站目标在未来时刻的位置
     *
     * 使用前哨站旋转模型预测装甲板位置。
     *
     * @param target 前哨站目标状态
     * @param dt 预测时间间隔（秒）
     * @return 预测的位置向量（odom坐标系）
     */
    [[nodiscard]] Eigen::Vector3d
        predict_position(const L3::OutpostTargetState& target, double dt) const noexcept;

    /**
     * @brief 预测能量机关目标在未来时刻的位置
     *
     * 使用能量机关旋转模型预测打击位置。
     *
     * @param target 能量机关状态
     * @param dt 预测时间间隔（秒）
     * @return 预测的位置向量（odom坐标系）
     */
    [[nodiscard]] Eigen::Vector3d
        predict_position(const energy_meter::EnergyMeterState& target, double dt) const noexcept;

    /**
     * @brief 预测LDM目标在未来时刻的SE2(3)状态
     *
     * 使用右扰动模型：X̂₊ = X̂ * exp(xi)，其中xi为速度扰动。
     * SEn3(2)群同时表示位置和速度，适合复杂运动模型。
     *
     * @param target LDM目标状态
     * @param dt 预测时间间隔（秒）
     * @return 预测的SEn3(2)状态
     */
    [[nodiscard]] group::SEn3<double, 2>
        predict_state(const L3::ldm::LdmState& target, double dt) const noexcept;

    /**
     * @brief 预测机器人目标的瞄准点
     *
     * 核心算法：
     * 1. 预测目标位置和姿态
     * 2. 根据瞄准阶段选择装甲板
     * 3. 计算装甲板位置和角度偏差
     *
     * @param target 机器人目标状态
     * @param dt 预测时间间隔（秒）
     * @param context 瞄准上下文
     * @param muzzle 枪口坐标系变换
     * @param forced_armor_id 强制选择的装甲板ID（可选）
     * @return 预测的瞄准点信息
     */
    [[nodiscard]] PredictedAimPoint predict_aim_point(
        const L3::RobotTargetState& target, double dt, const ArmorAimContext& context,
        const MuzzleTransform& muzzle,
        std::optional<int> forced_armor_id = std::nullopt) const noexcept;

    /**
     * @brief 预测前哨站目标的瞄准点
     *
     * @param target 前哨站目标状态
     * @param dt 预测时间间隔（秒）
     * @param context 瞄准上下文
     * @param muzzle 枪口坐标系变换
     * @param forced_armor_id 强制选择的装甲板ID（可选）
     * @return 预测的瞄准点信息
     */
    [[nodiscard]] PredictedAimPoint predict_aim_point(
        const L3::OutpostTargetState& target, double dt, const ArmorAimContext& context,
        const MuzzleTransform& muzzle,
        std::optional<int> forced_armor_id = std::nullopt) const noexcept;

    AimerConfig config_; ///< 瞄准器配置参数
};

// ========================================================================
// get_target_type 模板特化 - 编译期目标类型映射
// ========================================================================

/**
 * @brief 机器人目标类型特化
 * @return AimerTargetType::Robot
 */
template <>
inline constexpr AimerTargetType Aimer::get_target_type<L3::RobotTargetState>() noexcept {
    return AimerTargetType::Robot;
}

/**
 * @brief 前哨站目标类型特化
 * @return AimerTargetType::Outpost
 */
template <>
inline constexpr AimerTargetType Aimer::get_target_type<L3::OutpostTargetState>() noexcept {
    return AimerTargetType::Outpost;
}

/**
 * @brief 能量机关目标类型特化
 * @return AimerTargetType::Rune
 */
template <>
inline constexpr AimerTargetType Aimer::get_target_type<energy_meter::EnergyMeterState>() noexcept {
    return AimerTargetType::Rune;
}

/**
 * @brief LDM目标类型特化
 * @return AimerTargetType::Ldm
 */
template <>
inline constexpr AimerTargetType Aimer::get_target_type<L3::ldm::LdmState>() noexcept {
    return AimerTargetType::Ldm;
}

} // namespace fcs::L4
