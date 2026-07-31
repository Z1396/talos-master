/**
 * @file aimer_systems.hpp
 * @brief L4规划层瞄准系统注册接口
 *
 * 本文件声明了瞄准系统的注册函数，负责将瞄准相关系统注册到调度器。
 *
 * 系统架构：
 * - l4_aimer系统：250Hz固定频率运行，处理目标选择和轨迹构建
 * - l4_optimal_target_readback_roi系统：处理回读ROI缓存更新
 *
 * 数据流：
 * L3跟踪器输出 → l4_aimer → ControlIntent → L5武器控制层
 *                     ↓
 *                SelectedTargetSnapshot → Foxglove可视化
 *                     ↓
 *                TargetSelectionTrace → 调试分析
 */

#pragma once

#include "L4_planning/config.hpp"
#include "scheduler/thin.hpp"

namespace fcs::L4 {

/**
 * @brief 注册瞄准系统到调度器
 *
 * 注册以下独立系统：
 * 1. RobotAimerSystem：处理机器人目标，发布AimerOutput
 * 2. OutpostAimerSystem：处理前哨站目标，发布AimerOutput
 * 3. RuneAimerSystem：处理能量机关目标，发布AimerOutput
 *
 * 所有瞄准器消费跟踪器/能量计量器输出，产生纯预测结果（AimerOutput），
 * 供规划层消费。
 *
 * @param scheduler 调度器实例
 * @param config L4规划配置（使用移动语义）
 *
 * @note 配置对象会被移动，调用后不应再使用
 */
void register_aimer_systems(talos::Scheduler& scheduler, L4Config&& config);

} // namespace fcs::L4
