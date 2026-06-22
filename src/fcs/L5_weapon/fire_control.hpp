#pragma once
// 头文件保护指令：保证该头文件在整个编译过程中只会被包含一次，避免重复定义、编译报错
#include <Eigen/Core>
// Eigen 线性代数库，提供矩阵/向量基础浮点类型，本文件用于角度、距离、位姿浮点存储计算
#include <cstdint>
// C++标准固定宽度整数头文件，提供 uint64_t 无符号64位整数，用于高精度纳秒时间戳
#include <optional>
// C++17 标准库：std::optional，用于「可选字段」，区分“无数据”和“默认0值”，专门存放可选调试/故障信息
#include <string>
// 标准字符串，存储故障降级原因文本
#include <vector>
// 动态数组容器，存放弹道规划离散采样点序列

// 顶层工程命名空间 fcs，子模块 L5 层：武器控制/弹道预测/云台输出层（感知L2→预测L4→武器输出L5分层架构）
namespace fcs::L5 {

// ============================================================================
// WeaponCommand - 武器控制命令
// 模块说明：L5层最终输出给云台电控、发射机构的完整控制指令结构体
// 包含弹道规划结果、云台目标角度、角速度/角加速度前馈、开火信号、误差判定、调试可视化、优化降级故障标记
// ============================================================================

/// 弹道规划离散采样点
/// 存储一条弹道轨迹上单个时间点对应的云台偏航、俯仰、距离、子弹飞行时间
struct TrajectoryPlanSample {
    double yaw{0.0};      // 当前采样点对应的云台yaw目标角度，单位：弧度
    double pitch{0.0};    // 当前采样点对应的云台pitch目标角度，单位：弧度
    double distance{0.0}; // 当前采样点到目标的直线距离，单位：米
    double tof{0.0};      // 子弹从枪口飞到该点的飞行时间 Time of Flight，单位：秒
};

/// 武器弹道可视化调试数据
/// 仅调试/上位机绘图使用，正式运行可无数据，用std::optional承载
struct WeaponVisualizationDebugData {
    // 原始未优化弹道规划序列（L4层输出原始轨迹）
    std::vector<TrajectoryPlanSample> reference_plan;
    // 经过MPC模型预测控制优化后的弹道轨迹序列（L5优化输出）
    std::vector<TrajectoryPlanSample> optimized_plan;
    int center_index{0};       // 优化轨迹中心点下标（瞄准核心点）
    // 历史兼容别名：早期代码叫前瞻下标，当前版本逻辑与center_index完全等价，仅做兼容保留
    int lookahead_index{0};        // Legacy alias, currently mirrors center_index in L5.
};

/// L5层输出武器总控制指令结构体
/// 每一帧弹道计算完成后生成一份，下发给云台控制器、发射机构
struct WeaponCommand {
    uint64_t timestamp_ns{0};      // L5层生成本条指令的系统时间戳，单位：纳秒，用于多模块时序对齐
    uint64_t plan_timestamp_ns{0}; // 本条指令依赖的上游L4弹道预测帧时间戳，用于时序校验、丢包判断

    // L4原始未优化弹道数据（仅调试打印/对比，实际云台不使用该组值）
    double plan_yaw{0.0};
    double plan_pitch{0.0};
    double plan_distance{0.0}; // L4输出原始目标距离，单位：米

    // ====================== 云台最终目标角度（执行机构使用） ======================
    double yaw{0.0};   // 下发云台的最终偏航目标角度，单位：弧度
    double pitch{0.0}; // 下发云台的最终俯仰目标角度，单位：弧度

    // ====================== 一阶前馈补偿：角速度（用于云台电机提前预判加速） ======================
    double yaw_vel{0.0};   // yaw轴目标角速度，rad/s
    double pitch_vel{0.0}; // pitch轴目标角速度，rad/s

    // ====================== 二阶前馈补偿：角加速度（高速追踪强预测补偿） ======================
    double yaw_accel{0.0};
    double pitch_accel{0.0};

    // ====================== 发射控制逻辑 ======================
    bool fire{false}; // 开火指令标志位：true允许发射子弹，false禁止开火
    double yaw_error{-1.0};       // 当前yaw轴实际角度与目标角度的偏差，-1代表无效未计算
    double pitch_error{-1.0};     // 当前pitch轴角度偏差，-1代表无效
    double shooting_range_yaw{-1.0}; // 允许开火的yaw角度误差阈值，偏差小于该值才可开火，-1代表无限制
    double shooting_range_pitch{-1.0}; // 允许开火的pitch误差阈值
    double ref_yaw{-1.0}; // 参考基准yaw角度（弹道优化基准值，调试用）
    double ref_pitch{-1.0}; // 参考基准pitch角度

    // ====================== 弹道飞行参数 ======================
    double tof{0.0};      // 子弹飞行总时间 Time of Flight，单位 s
    double distance{0.0}; // 当前瞄准目标直线距离，单位 m

    // ====================== 可视化调试数据（可选） ======================
    // std::optional：无调试数据时为空，不占用内存；仅上位机绘图、仿真时填充
    std::optional<WeaponVisualizationDebugData> viz_debug;

    /// 故障降级原因标记
    /// 触发场景：L5层MPC弹道优化求解失败，无法完成完整最优解，降级使用兜底方案（直通原始轨迹/中心点直瞄）
    /// 错误来源：TinyMpcTrajectoryOptimizer优化函数内部报错，或上游L4 ShotCommand传递下来的故障信息
    /// 存储说明：nullopt = 正常最优解计算成功；有字符串则存储故障描述文本（如模型不收敛、目标过远、数值奇异）
    std::optional<std::string> degradation_reason{};
};
} // namespace fcs::L5