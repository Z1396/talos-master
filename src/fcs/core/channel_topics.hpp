// 头文件保护，防止重复引入引发重定义问题
#pragma once

/**
 * @namespace fcs
 * @brief 火控系统（Fire Control System）顶层命名空间
 *        本文件全部为**空标签结构体**，用于定义数据流通道/话题标识
 *        配合调度器/IPC/消息队列实现模块间解耦通信，属于「用类型系统建模业务」的典型实践
 */
namespace fcs {

// ============================================================================
// 装甲检测流水线 传统通道/话题定义
// 数据流链路：相机图像 → 检测器 → PnP解算器 → 目标跟踪器
// 通信模式统一：SPMC = 单生产者、多消费者（一个模块发数据，多个模块订阅使用）
// ============================================================================

/**
 * @struct ImageChannelTopic
 * @brief 图像数据通道标识
 * @流向 相机模块 → 目标检测器
 * @承载数据 上一文件定义的 ImageFrame
 * @模式 SPMC 单生产者、多消费者
 */
struct ImageChannelTopic {};            // L1→L2 图像流

/**
 * @struct DetectionChannelTopic
 * @brief 2D检测结果通道标识
 * @流向 目标检测器 → PnP位姿解算器
 * @承载数据 上一文件定义的 ArmorDetectionBatch
 * @模式 SPMC 单生产者、多消费者
 */
struct DetectionChannelTopic {};// L2→L3 检测结果流

/**
 * @struct MeasurementChannelTopic
 * @brief 3D位姿测量结果通道标识
 * @流向 PnP位姿解算器 → 目标跟踪器
 * @承载数据 上一文件定义的 ArmorMeasurementBatch
 * @模式 SPMC 单生产者、多消费者
 * @detail 记录PnP解算器输出的3D位姿测量结果，包括目标ID、位姿（位置、旋转）、置信度等
 */
struct MeasurementChannelTopic {};// L3→L4 测量流   

/**
 * @struct LdmDetectionChannelTopic
 * @brief 符类装甲2D检测结果专属通道
 * @desc Ldm 为特定装甲/符类业务分支，独立通道隔离数据流
 * @流向 符类装甲检测器 → PnP位姿解算器
 * @承载数据 上一文件定义的 LdmArmorDetectionBatch
 * @模式 SPMC 单生产者、多消费者
 */
struct LdmDetectionChannelTopic {};// LDM 光斑检测流

/**
 * @struct LdmMeasurementChannelTopic
 * @brief 符类装甲3D位姿测量结果专属通道
 * @desc Ldm 为特定装甲/符类业务分支，独立通道隔离数据流
 * @流向 PnP位姿解算器 → 目标跟踪器
 * @承载数据 上一文件定义的 LdmArmorMeasurementBatch
 * @模式 SPMC 单生产者、多消费者
 * @detail 记录PnP解算器输出的符类装甲3D位姿测量结果，包括目标ID、位姿（位置、旋转）、置信度等
 */
struct LdmMeasurementChannelTopic {};// LDM 测量流

// ============================================================================
// L3 状态估计流水线 通道/话题定义
// 层级说明：L3 = 状态估计层，负责目标跟踪、观测数据融合
// ============================================================================
/**
 * @struct TrackerOutputChannelTopic
 * @brief 跟踪器输出通道标识
 * @流向 目标跟踪器 → 解算决策模块
 * @detail 记录目标跟踪器输出的目标状态，包括ID、位姿（位置、旋转）、速度、健康状态等
 * @detail 记录目标跟踪器输出的目标状态，包括ID、位姿（位置、旋转）、速度、健康状态等
 * @承载数据 fcs::L3::TrackerOutputs 跟踪结果结构体
 * @detail 记录目标跟踪器输出的跟踪结果，包括目标ID、位姿（位置、旋转）、速度、健康状态等       
 * @模式 SPMC 单生产者、多消费者
 */
struct TrackerOutputChannelTopic {};// L3→L4 追踪输出流

/**
 * @struct RuneObservationChannelTopic
 * @brief 能量机关观测数据通道
 * @desc 专门承载符盘/能量机关的观测、检测数据
 * @流向 能量机关检测模块 → 解算决策模块
 * @detail 记录符盘/能量机关的观测数据，包括检测到的符类装甲、符盘中心、旋转角度等
 * @detail 承载数据 fcs::L3::RuneObservation 符类装甲观测数据
 * @模式 SPMC 单生产者、多消费者
 */
struct RuneObservationChannelTopic {};// L3→L4 能量机关观测流

/**
 * @struct RuneDebugFrameChannelTopic
 * @brief 能量机关调试帧通道
 * @desc 专供可视化、调试、日志录制使用的原始帧数据
 * @模式 SPMC 单生产者、多消费者
 */
struct RuneDebugFrameChannelTopic {};// L3→数据录制 能量机关调试帧流

/**
 * @struct EnergyMeterStateChannelTopic
 * @brief 能量值状态通道
 * @desc 承载机器人自身能量、弹丸数量、状态等信息
 * @模式 SPMC 单生产者、多消费者    
 */
struct EnergyMeterStateChannelTopic {};// L3→L4 能量值状态流

// ============================================================================
// L4 规划决策流水线 通道/话题定义
// 层级说明：L4 = 规划决策层，负责目标选择、射击/跟随指令生成
// ============================================================================

/**
 * @struct ControlIntentChannelTopic
 * @brief 控制意图指令通道
 * @流向 L4规划器 → L5武器执行层
 * @承载数据 fcs::L4::ControlIntent
 * @detail 内部通过 std::variant 封装三类互斥指令：
 *         TrackCommand(跟随指令) / ShotCommand(射击指令) / HoldCommand(保持待命)
 *         是「用 variant 表达互斥选择」的工程实践
 * @模式 SPMC 单生产者、多消费者
 */
struct ControlIntentChannelTopic {};// L4→L5 控制意图流

/**
 * @struct SelectedTargetSnapshotChannelTopic
 * @brief 已选中目标快照通道
 * @流向 目标选择器 → 可视化/数据录制/标定模块
 * @承载数据 fcs::L4::SelectedTargetSnapshot 目标瞬时状态快照
 * @模式 SPMC 单生产者、多消费者
 * @detail 记录当前选中目标的瞬时状态，包括ID、位姿、速度、健康状态等
 */
struct SelectedTargetSnapshotChannelTopic {};// L4→数据录制 已选中目标快照流    

/**
 * @struct TargetSelectionTraceChannelTopic
 * @brief 目标选择过程诊断日志通道
 * @流向 目标选择器 → 可视化/数据录制模块
 * @承载数据 fcs::L4::TargetSelectionTrace 选目标过程轨迹、决策日志
 * @模式 SPMC 单生产者、多消费者
 * @detail 记录目标选择器的运行过程，包括每次决策的目标ID、置信度、决策路径等
 */
struct TargetSelectionTraceChannelTopic {};// L4→数据录制 目标选择轨迹流

// ============================================================================
// L5 武器执行系统 通道/话题定义
// 层级说明：L5 = 底层执行层，接收上层指令，控制云台、发射机构
// ============================================================================

/**
 * @struct WeaponCommandChannelTopic
 * @brief 武器最终执行指令通道
 * @流向 火控核心 → 硬件输出接口
 * @承载数据 fcs::L5::WeaponCommand 武器执行指令
 * @模式 SPMC 单生产者、多消费者
 */
struct WeaponCommandChannelTopic {};// L5→执行器 武器命令流

/**
 * @struct RuntimeControlStateChannelTopic
 * @brief 实时控制资源状态快照通道
 * @流向 IMU/硬件资源管理模块 → 数据录制模块
 * @承载数据 fcs::core::ControlResourceSnapshot 整机控制资源、传感器状态
 * @模式 SPMC 单生产者、多消费者
 */
struct RuntimeControlStateChannelTopic {};// L5→数据录制 实时控制资源状态流

/**
// ============================================================================
// 真值数据通道（调试/标定/算法评估专用）
// 数据源：外部IPC真值读取模块，用途：算法对比、可视化、精度评测
// ============================================================================

/**
 * @struct GroundTruthBatchChannelTopic
 * @brief 真值数据包通道
 * @流向 真值IPC读取模块 → 可视化模块
 * @承载数据 ipc::GroundTruthBatch 全局真值标注数据
 * @模式 SPMC 单生产者、多消费者
 */
struct GroundTruthBatchChannelTopic {};// 真值数据包流

} // namespace fcs