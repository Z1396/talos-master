/**
 * @file l1_l2_setup.hpp
 * @brief L1/L2层系统注册与配置接口
 *
 * 本文件提供了初始化和注册L1（采集层）和L2（感知层）系统的统一接口。
 *
 * L1层包括：
 * - IMU数据读取系统（订阅共享内存中的云台姿态）
 * - 相机采集系统（发布图像帧到调度器）
 *
 * L2层包括：
 * - 装甲板检测器后端（神经网络推理）
 * - PnP求解器资源（位姿估计）
 *
 * 该接口根据硬件配置（真实硬件 vs 回放模式）自动选择适当的数据源，
 * 并在调度器中注册所有必需的系统、资源和通道。
 *
 * @author Talos Team
 * @date 2024
 */

#pragma once

#include <expected>
#include <string>

#include "camera_config.hpp"
#include "runtime/config_loader.hpp"
#include "scheduler/thin.hpp"

namespace fcs {
struct HardwareConfig;
} // namespace fcs

namespace fcs::runtime {

/**
 * @brief L1/L2设置结果
 *
 * 包含初始化后需要暴露给其他模块的配置信息
 */
struct L1L2SetupResult {
    CameraConfig camera_config; ///< 相机配置（内参、分辨率等）
};

/**
 * @brief 创建并注册L1采集层和L2感知层系统
 * @param world 调度器世界对象（用于资源和通道管理）
 * @param scheduler 调度器实例（用于系统注册）
 * @param cfg 硬件后端配置（指定数据源类型）
 * @return 设置结果或错误信息
 *
 * 根据硬件配置创建并注册以下系统：
 * - IMU读取系统：订阅云台姿态数据
 * - 相机采集系统：发布图像帧（ImageFrame）
 * - 武器输出系统：消费L5层的武器命令（WeaponCommand）
 * - L2检测器后端：装甲板检测神经网络
 * - PnP求解器资源：3D位姿估计
 * - CameraConfig资源：相机内参配置
 *
 * 硬件配置说明：
 * - 真实硬件模式（daedalus = false）：必须提供hardware配置，从共享内存读取数据
 * - 回放模式（daedalus = true）：不使用hardware配置，从回放数据源读取
 *
 * @note 该函数必须在调度器启动前调用
 * @note 系统依赖关系由调度器自动分析，无需手动指定
 */
[[nodiscard]] std::expected<L1L2SetupResult, std::string>
    setup_l1(talos::World& world, talos::Scheduler& scheduler, hardware::HardwareBackendConfig cfg);

} // namespace fcs::runtime
