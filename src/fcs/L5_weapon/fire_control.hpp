#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace fcs::L5 {

// ============================================================================
// WeaponCommand - 武器控制命令
// ============================================================================

struct TrajectoryPlanSample {
    double yaw{0.0};
    double pitch{0.0};
    double distance{0.0};
    double tof{0.0};
};

struct WeaponVisualizationDebugData {
    std::vector<TrajectoryPlanSample> reference_plan;
    std::vector<TrajectoryPlanSample> optimized_plan;
    int center_index{0};
    int lookahead_index{0};        // Legacy alias, currently mirrors center_index in L5.
};

struct WeaponCommand {
    uint64_t timestamp_ns{0};      // L5 命令生成时间戳 (纳秒)
    uint64_t plan_timestamp_ns{0}; // 对应的 L4 plan 时间戳 (纳秒)
    // L4 原始 plan（优化前，仅调试）
    double plan_yaw{0.0};
    double plan_pitch{0.0};
    double plan_distance{0.0}; // 目标距离 (米)

    // 云台角度
    double yaw{0.0};   // 目标 yaw (弧度)
    double pitch{0.0}; // 目标 pitch (弧度)

    // 前馈 (用于预测补偿)
    double yaw_vel{0.0};   // yaw 角速度 (rad/s)
    double pitch_vel{0.0}; // pitch 角速度 (rad/s)

    // 前馈 (用于预测补偿)
    double yaw_accel{0.0};
    double pitch_accel{0.0};

    // 发射控制
    bool fire{false}; // 发射指令
    double yaw_error{-1.0};
    double pitch_error{-1.0};
    double shooting_range_yaw{-1.0};
    double shooting_range_pitch{-1.0};
    double ref_yaw{-1.0};
    double ref_pitch{-1.0};

    // 弹道信息
    double tof{0.0};      // 飞行时间 (秒)
    double distance{0.0}; // 目标距离 (米)

    // 可视化调试数据
    std::optional<WeaponVisualizationDebugData> viz_debug;

    /// Set when L5 could not run full MPC optimization and fell back to a
    /// degraded output (passthrough / center-aim-point fallback).
    /// Carries the error from TinyMpcTrajectoryOptimizer::optimize() or
    /// propagated from L4's ShotCommand::degradation_reason.
    /// nullopt = nominal output (MPC succeeded or intentional passthrough).
    std::optional<std::string> degradation_reason{};
};
} // namespace fcs::L5
