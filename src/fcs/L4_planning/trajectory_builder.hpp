/**
 * @file trajectory_builder.hpp
 * @brief L4规划层参考轨迹构建器
 *
 * 本文件定义了L4规划层的核心轨迹构建功能，负责从规划器种子状态生成参考轨迹。
 * 参考轨迹用于L5武器控制层的MPC优化，是实现精确射击的关键组件。
 *
 * 核心功能：
 * - 构建控制轨迹：用于MPC轨迹跟踪优化
 * - 构建射击轨迹：用于火力门控判断（使用真实装甲板瞄准）
 * - 构建LDM轨迹：针对移动目标的预测轨迹
 *
 * 算法原理：
 * 1. 遍历预测时域，每个时间步调用Aimer::aim()计算瞄准角度
 * 2. 使用有限差分法计算角度速率（yaw_rate, pitch_rate）
 * 3. 组装状态矩阵 [yaw, yaw_rate, pitch, pitch_rate] × horizon
 * 4. 计算弹道飞行时间和距离等元数据
 *
 * 性能考虑：
 * - 轨迹构建是计算密集型操作，每个控制周期执行一次
 * - 有限差分法可能引入高频噪声，需要平滑处理
 * - 弹道求解器可能失败，需要优雅降级处理
 *
 * 线程安全：非线程安全，不要并发调用
 */

#pragma once

#include "L3_estimation/ldm_naive/types.hpp"
#include "L4_planning/aimer/aimer.hpp"
#include "L4_planning/config.hpp"
#include "L4_planning/gimbal_planner/types.hpp"
#include "core/trajectory/reference_trajectory.hpp"
#include "core/trajectory/solver/solver_interfaces.hpp"

#include <expected>
#include <string>

namespace fcs::L4 {

/**
 * @brief 轨迹构建运行时上下文
 *
 * 包含每个控制周期构建轨迹所需的所有运行时信息。
 * 这些信息在轨迹构建过程中不可变，作为函数参数传递。
 *
 * 设计考虑：
 * - 将频繁变化的数据（时间戳、弹速）与稳定的组件（求解器、变换矩阵）分离
 * - 使用指针传递求解器，避免拷贝大型对象
 * - GimbalTransform和MuzzleTransform是预计算的变换矩阵，提高性能
 */
struct TrajectoryBuildContext {
    uint64_t current_ns{0};        ///< 当前时间戳（纳秒）
    Aimer::GimbalTransform gimbal; ///< 云台坐标系变换矩阵（odom→gimbal_pitch）
    Aimer::MuzzleTransform muzzle; ///< 枪口坐标系变换矩阵（odom→muzzle）
    const core::trajectory::solver::TrajectorySolver* trajectory_solver{
        nullptr};                  ///< 弹道求解器指针（非空）
    double bullet_speed{0.0};      ///< 弹丸速度（米/秒）
};

/**
 * @brief 从规划器种子构建参考轨迹
 *
 * 核心算法流程：
 * 1. 根据时域配置计算采样点数量（horizon_ahead + horizon_back + 1）
 * 2. 遍历每个时间步，计算预测延迟偏移量
 * 3. 调用Aimer::aim()计算该时间步的瞄准角度
 * 4. 使用有限差分法计算角度速率（yaw_rate, pitch_rate）
 * 5. 组装状态矩阵和元数据
 *
 * 关键设计点：
 * - 使用中心差分而非前向/后向差分，提高精度
 * - yaw角度归一化处理，避免角度跳变
 * - pitch角度不做限幅，由L5负责（因为L5需要根据物理限制动态调整）
 *
 * @param seed 规划器种子，包含L3跟踪器状态（机器人或前哨站）
 * @param aimer 瞄准器实例，提供预测和弹道求解功能
 * @param horizon_cfg 参考轨迹时域配置（前向步数、后向步数、时间步长）
 * @param ctx 轨迹构建上下文（时间戳、坐标系变换、弹速、求解器）
 * @return 成功时返回ReferenceTrajectory，失败时返回错误信息
 *
 * @pre seed必须包含有效的机器人或前哨站状态
 * @pre ctx.trajectory_solver必须非空
 *
 * @warning 非线程安全，不要在共享数据上并发调用
 *
 * @note 性能关键路径：每个控制周期执行一次，约250Hz
 */
[[nodiscard]] std::expected<core::trajectory::ReferenceTrajectory, std::string>
    build_reference_trajectory(
        const PlannerSeed& seed, const Aimer& aimer, const ReferenceTrajectoryConfig& horizon_cfg,
        const TrajectoryBuildContext& ctx) noexcept;

/**
 * @brief 构建射击参考轨迹（用于L5火力门控）
 *
 * 与控制轨迹的区别：
 * - WholeCarCenter模式下，控制轨迹瞄准车身中心代理点（用于平滑跟踪）
 * - 射击轨迹始终瞄准真实装甲板（用于判断是否可击中）
 * - 这样可以避免"瞄准中心但射击判断用的是装甲板"的不一致
 *
 * 关键场景：
 * - 当目标处于WholeCarCenter阶段时，控制轨迹使用中心代理点
 * - 但火力门控需要判断是否能击中真实装甲板
 * - 因此需要单独构建射击轨迹，使用WholeCarArmor阶段
 *
 * @param seed 规划器种子，包含L3跟踪器状态
 * @param aimer 瞄准器实例
 * @param horizon_cfg 时域配置
 * @param ctx 轨迹构建上下文
 * @return 成功时返回ReferenceTrajectory，失败时返回错误信息
 *
 * @see build_reference_trajectory() 控制轨迹构建函数
 */
[[nodiscard]] std::expected<core::trajectory::ReferenceTrajectory, std::string>
    build_fire_reference_trajectory(
        const PlannerSeed& seed, const Aimer& aimer, const ReferenceTrajectoryConfig& horizon_cfg,
        const TrajectoryBuildContext& ctx) noexcept;

/**
 * @brief 从LDM目标状态构建参考轨迹
 *
 * LDM（Linear Dynamic Model）是一种简化的目标运动模型，假设：
 * - 目标做匀速直线运动（在SE2(3)群上）
 * - 使用常速度预测模型，不考虑复杂的机动
 *
 * 算法流程：
 * 1. 检查LDM状态是否处于跟踪模式
 * 2. 遍历预测时域，每个时间步计算目标预测位置
 * 3. 调用Aimer::aim(LdmState)计算瞄准角度
 * 4. 使用有限差分法计算角度速率
 * 5. 组装状态矩阵和元数据
 *
 * @param state LDM目标状态（必须处于跟踪模式）
 * @param aimer 瞄准器实例
 * @param horizon_cfg 时域配置
 * @param ctx 轨迹构建上下文
 * @return 成功时返回ReferenceTrajectory，失败时返回错误信息
 *
 * @pre state.is_tracking()必须为true
 * @pre ctx.trajectory_solver必须非空
 *
 * @warning 非线程安全，不要在共享数据上并发调用
 *
 * @note 适用于移动目标（如哨兵、基地）的跟踪场景
 */
[[nodiscard]] std::expected<core::trajectory::ReferenceTrajectory, std::string>
    build_ldm_reference_trajectory(
        const L3::ldm::LdmState& state, const Aimer& aimer,
        const ReferenceTrajectoryConfig& horizon_cfg, const TrajectoryBuildContext& ctx) noexcept;

} // namespace fcs::L4
