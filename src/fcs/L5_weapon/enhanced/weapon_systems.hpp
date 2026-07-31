/**
 * @file weapon_systems.hpp
 * @brief L5武器层增强版系统注册接口
 *
 * 本文件提供了增强版武器系统的注册接口,用于将武器控制系统集成到调度器中。
 * 核心功能:
 * 1. 采样开火轨迹:从Track指令的参考轨迹中提取开火瞄准点
 * 2. 应用开火门:根据当前云台姿态判断是否可以开火
 * 3. 注册武器系统:将武器控制逻辑注册为调度器系统
 *
 * 系统架构:
 * - L4生成控制意图(Track/Shot/Hold)
 * - L5轨迹优化器执行MPC优化
 * - L5开火门判断是否开火
 * - 输出武器指令到硬件层
 *
 * @note 这是增强版武器系统,使用MPC轨迹优化器
 */

#pragma once

#include "L4_planning/config.hpp"
#include "L4_planning/control_intent.hpp"
#include "L5_weapon/config.hpp"
#include "L5_weapon/fire_control.hpp"
#include "scheduler/thin.hpp"

#include <optional>

namespace fcs::L5 {

struct WeaponControllerConfig;

/**
 * @brief 从Track指令采样开火轨迹瞄准点
 *
 * 从Track指令的fire_trajectory中提取当前时刻的最佳瞄准点。
 * 这个瞄准点用于开火门判断,而非轨迹跟踪。
 *
 * 算法流程:
 * 1. 计算参考轨迹年龄(当前时间 - 生成时间)
 * 2. 将年龄转换为轨迹索引偏移
 * 3. 提取中心索引的瞄准点
 *
 * @param track L4的Track指令(包含fire_trajectory)
 * @param trajectory_cfg 轨迹配置
 * @param command_timestamp_ns 当前时间戳
 *
 * @return 成功返回瞄准点,失败返回nullopt
 */
[[nodiscard]] std::optional<core::trajectory::ReferenceTrajectory::AimPoint> sample_fire_trajectory(
    const L4::TrackCommand& track, const L4::ReferenceTrajectoryConfig& trajectory_cfg,
    uint64_t command_timestamp_ns) noexcept;

/**
 * @brief 对Track指令应用开火门判断
 *
 * 根据当前云台姿态和开火轨迹判断是否可以开火。
 * 这是L5层最核心的开火决策逻辑。
 *
 * 判断逻辑:
 * 1. 从fire_trajectory采样瞄准点
 * 2. 调用is_on_target计算误差和射击窗口
 * 3. 将结果填充到WeaponCommand中
 *
 * @param cmd 待处理的武器指令
 * @param track L4的Track指令
 * @param trajectory_cfg 轨迹配置
 * @param fire_cfg 开火决策配置
 * @param current_yaw 当前云台yaw角度
 * @param current_pitch 当前云台pitch角度
 *
 * @return 应用开火门后的武器指令
 */
[[nodiscard]] WeaponCommand apply_track_fire_gate(
    WeaponCommand cmd, const L4::TrackCommand& track,
    const L4::ReferenceTrajectoryConfig& trajectory_cfg, const FireDecisionConfig& fire_cfg,
    double current_yaw, double current_pitch) noexcept;

/**
 * @brief 注册增强版武器系统到调度器
 *
 * 将武器控制逻辑注册为一个fixed_rate系统(250Hz)。
 * 系统负责:
 * - 接收L4控制意图
 * - 执行MPC轨迹优化
 * - 应用开火门判断
 * - 输出武器指令
 *
 * 工作流程(每4ms):
 * 1. 读取控制意图通道
 * 2. 根据意图类型分支处理:
 *    - Track:MPC优化 → 开火门判断
 *    - Shot:透传 → 开火门判断
 *    - Hold:不发送指令
 * 3. 写入武器指令通道
 *
 * @param scheduler 调度器实例
 * @param config L5配置(移动语义,避免拷贝)
 */
void register_enhanced_weapon_system(talos::Scheduler& scheduler, L5Config&& config);

} // namespace fcs::L5
