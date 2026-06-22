#pragma once
// 轻量化调度器最小头文件，仅提供Scheduler注册接口，无多余模板依赖，加快编译
#include "scheduler/thin.hpp"

namespace fcs::L4 {

// 前置声明：L4规划层完整配置结构体，包含各自瞄器、规划器全部算法参数
struct L4Config;

/**
 * @brief 向Talos调度器注册整套L4自瞄+运动规划流水线系统
 *
 * 架构说明：替代旧版单一庞大的单体规划器，分层解耦设计
 * 两层分层结构：
 * 1. Aimer 自瞄预测层：纯运动预测模块，分三类专用预测器
 *    - RobotAimer：敌方机器人装甲弹道预测
 *    - OutpostAimer：前哨站预测
 *    - RuneAimer：能量机关风车扇叶运动预测
 *    职责：接收L3跟踪输出，做目标运动滤波、弹丸飞行时间补偿预测
 *
 * 2. Plan adapter 规划适配层：
 *    将Aimer输出的预测坐标，封装为统一 ControlIntent 互斥指令
 *    ControlIntent 包含三类互斥指令：TrackCommand 持续跟随 / ShotCommand 射击 / HoldCommand 云台保持
 *    输出下发至L1底层硬件输出接口、可视化模块
 *
 * @param scheduler Talos全局调度器实例，所有L4计算系统挂载至此调度器管理
 * @param config L4完整配置参数，右值引用接收，函数内部移动存入全局共享资源，无深拷贝开销
 */
void register_l4_planning_systems(::talos::Scheduler& scheduler, L4Config&& config);

} // namespace fcs::L4