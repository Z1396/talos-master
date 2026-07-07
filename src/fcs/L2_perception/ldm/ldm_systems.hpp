#pragma once
// LDM大符检测器全套阈值、几何配置结构体 LdmDetectorConfig
#include "L2_perception/ldm/ldm_config.hpp"
// 相机标定参数结构体：内参、畸变系数、分辨率、坐标系定义
#include "camera_config.hpp"
// Talos轻量调度器头文件，包含Scheduler任务注册、系统管理API
#include "scheduler/thin.hpp"

/**
 * @brief 顶层命名空间 fcs::L2::ldm
 * fcs：项目总工程命名空间
 * L2：第二层感知模块（图像视觉检测层）
 * ldm：Laser Module 激光大符识别子模块
 */
namespace fcs::L2::ldm {

/**
 * @brief 注册整套LDM大符感知流水线系统到Talos实时调度器
 * 流水线包含两大核心子系统：2D图像检测器 LdmDetector + 三维PnP位姿求解器 LdmSolver
 * @param scheduler 调度器实例，用于挂载固定频率视觉任务、注入全局共享资源
 * @param config LdmDetectorConfig 检测器完整配置，右值引用，内部移动存储避免拷贝开销
 * @param camera_config 相机标定静态参数（只读，生命周期由外部保证）
 * noexcept 标记函数全程不会抛出C++异常，满足机器人硬实时任务稳定性要求
 */
/// @brief Register LDM detection systems (detector + solver)
/// @param scheduler The scheduler to register systems with
/// @param config LDM detector configuration
void register_ldm_systems(
    talos::Scheduler& scheduler, LdmDetectorConfig&& config,
    const CameraConfig& camera_config) noexcept;

} // namespace fcs::L2::ldm