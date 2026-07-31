/**
 * @file ldm_naive_systems.hpp
 * @brief LDM Naive跟踪系统注册接口
 *
 * ## 文件功能概述
 * 声明LDM（Landing Device Marker）朴素跟踪系统的调度器注册函数。
 * 该函数将LdmTracker封装为固定频率系统，集成到Talos调度框架。
 *
 * ## 系统架构位置
 * - 层级：L3估计层（Estimation Layer）
 * - 模块：ldm_naive（朴素跟踪器，基于Invariant EKF）
 * - 依赖：L2感知层（LdmMeasurement）、调度器框架
 *
 * ## 使用方式
 * ```cpp
 * // 在runtime初始化中调用
 * NaiveLdmConfig config = load_config();
 * register_naive_ldm_systems(scheduler, config);
 * ```
 *
 * ## 设计理念
 * - 分离声明与实现：头文件只声明接口，cpp文件实现细节
 * - 配置注入：通过NaiveLdmConfig参数传递模型参数、跟踪阈值等
 * - 调度器集成：遵循Talos系统的注册模式
 *
 * @see ldm_naive_systems.cpp 实现文件
 * @see LdmTracker 跟踪器核心类
 * @see NaiveLdmConfig 配置结构体
 *
 * @author Talos Team
 * @date 2024
 */

#pragma once

#include "ldm_naive_config.hpp"
#include "scheduler/thin.hpp"

namespace fcs::L3::ldm {

/**
 * @brief 注册LDM朴素跟踪系统到调度器
 *
 * 创建并注册固定频率（250Hz）的LDM跟踪系统：
 * - 从L2感知层读取位姿测量
 * - 运行Invariant EKF进行状态估计
 * - 向下游发布LdmState
 *
 * @param scheduler 调度器引用（用于注册系统）
 * @param config 配置参数（模型参数、跟踪阈值、初始方差等）
 *
 * @note 必须在调度器启动前调用
 * @warning 多次调用会注册多个系统（通常不需要）
 */
void register_naive_ldm_systems(talos::Scheduler& scheduler, const NaiveLdmConfig& config);

} // namespace fcs::L3::ldm
