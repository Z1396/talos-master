/**
 * @file imu_only_demo.hpp
 * @brief ImuOnly 调试模式：直接生成正弦云台指令下发给 MCU
 *
 * 不使用相机、识别、跟踪、预瞄，按配置的幅度/频率生成 yaw/pitch 正弦指令，
 * 通过 WeaponCommand 通道交给 L1 的 weapon_output 系统下发，用于调试伺服响应。
 * 该模式下 fire 恒为 false，不会发射弹丸。
 */

#pragma once

#include "runtime/config_loader.hpp"
#include "scheduler/scheduler.hpp"

namespace fcs::runtime {

/**
 * @brief 注册 ImuOnly 正弦云台指令系统
 * @param scheduler 调度器实例
 * @param cfg 正弦指令参数（幅度、频率、距离）
 */
void register_imu_only_demo_system(
    talos::Scheduler& scheduler, const hardware::ImuOnlyDemoConfig& cfg);

} // namespace fcs::runtime
