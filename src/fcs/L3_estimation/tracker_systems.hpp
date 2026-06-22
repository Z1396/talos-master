#pragma once
// 注释：本头文件仅做轻量化对外API声明，所有函数实现不写在这里
// 实际业务逻辑、系统注册完整代码放在 cpp 实现文件：src/fcs/L3_estimation/tracker_systems.cpp
// 目的：减少头文件依赖、缩短编译时间、隔离实现细节，符合大型工程分层规范

// 引入Talos调度器轻量化头文件，仅包含Scheduler基础注册接口，不引入冗余系统定义
#include "scheduler/thin.hpp"

namespace fcs::L3 {

// 前置类型声明（仅告知编译器存在这两个类，无需包含完整头文件）
// 1. Tracker：目标跟踪器核心算法类（卡尔曼滤波/匹配逻辑）
class Tracker;
// 2. TrackerConfig：跟踪器算法参数结构体（阈值、最大存活帧数、滤波Q/R噪声矩阵等）
struct TrackerConfig;

/**
 * @brief 向Talos调度器注册整套L3目标跟踪流水线计算系统
 *
 * 完整数据流流水线拓扑：
 *  ArmorMeasurementBatch 三维装甲测量结果【SPMC单生产者多消费者通道】
 *                ↓（通道有新数据自动触发）
 *        TrackerSystem 跟踪计算系统（执行策略 pool_compute 事件驱动）
 *                ↓ 跟踪完成输出
 *        TrackerOutput 跟踪结果数据包【SPMC通道】
 *
 * 流水线逻辑说明：
 * 1. L2感知PnP解算模块持续往 ArmorMeasurementBatch 写入3D装甲观测；
 * 2. 通道产生新数据，自动唤醒跟踪系统执行卡尔曼滤波、目标匹配、生命周期管理；
 * 3. 跟踪完成后生成 TrackerOutput，供上层L4决策、Foxglove可视化订阅使用。
 *
 * @param scheduler Talos调度器实例，所有跟踪系统挂载到该调度器管理
 * @param config 跟踪器算法配置，采用右值引用，函数内部会移动存入全局World共享资源，无拷贝开销
 */
void register_tracker_systems(talos::Scheduler& scheduler, TrackerConfig&& config);

} // namespace fcs::L3