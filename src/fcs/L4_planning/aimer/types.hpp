/**
 * @file types.hpp
 * @brief aimer模块类型定义
 *
 * 本文件定义了瞄准模块的核心数据类型，包括：
 * - 瞄准阶段枚举（ArmorAimPhase）
 * - 瞄准上下文结构（ArmorAimContext）
 * - 目标类型枚举（AimerTargetType）
 * - 目标预测结果结构（TargetPrediction）
 *
 * 这些类型是Aimer模块与上层Planner模块之间的数据接口。
 */
#pragma once

#include "core/armor_types.hpp"

#include <Eigen/Core>
#include <cstdint>
#include <optional>

namespace fcs::L4 {

/**
 * @enum ArmorAimPhase
 * @brief 装甲板瞄准阶段
 *
 * 定义了4个瞄准阶段，根据目标旋转速度动态切换。
 * 状态转换由fsm.hpp中的状态机控制。
 */
enum class ArmorAimPhase : uint8_t {
    SingleArmor    = 0, ///< 单装甲板模式：目标转速低，锁定单块装甲板
    WholeCarArmor  = 1, ///< 整车装甲板模式：目标转速中等，考虑所有装甲板
    WholeCarPair   = 2, ///< 整车双板模式：目标转速较高，瞄准对称装甲板对
    WholeCarCenter = 3, ///< 整车中心模式：目标转速极高，瞄准车辆中心
};

/**
 * @struct ArmorAimContext
 * @brief 装甲板瞄准上下文
 *
 * 用于传递瞄准策略的运行时信息，控制装甲板选择逻辑。
 */
struct ArmorAimContext {
    bool target_jumped{false};                  ///< 目标是否跳跃（是否已锁定）
    ArmorAimPhase phase{ArmorAimPhase::SingleArmor}; ///< 当前瞄准阶段
    std::optional<int> preferred_armor_id{};    ///< 偏好的装甲板ID（可选）
};

/**
 * @enum AimerTargetType
 * @brief 瞄准器目标类型枚举
 *
 * 区分不同类型的目标，用于选择相应的预测算法。
 */
enum class AimerTargetType : uint8_t {
    Robot,   ///< 机器人目标（步兵/英雄/哨兵）
    Outpost, ///< 前哨站目标
    Rune,    ///< 能量机关目标
    Ldm,     ///< LDM目标（Light Decision Module）
};

// ============================================================================
// 目标预测 - Aimer层的纯预测结果
// ============================================================================

/**
 * @struct TargetPrediction
 * @brief 目标预测结果
 *
 * Aimer层的输出数据，包含目标在弹丸到达时刻的预测状态。
 * 该结果由Planner层消费，用于生成优化的轨迹和前馈控制项。
 *
 * 数据包含：
 * - 时间信息：输出时间戳、预测到达时间
 * - 预测位置：目标在odom坐标系下的预测位置、距离、飞行时间
 * - 瞄准角度：带弹道补偿的云台控制角度（yaw/pitch）
 * - 目标运动：目标速度、角速度、预测roll角（能量机关专用）
 * - 辅助信息：瞄准阶段、选中的装甲板ID、装甲板类型
 */
struct TargetPrediction {
    // ========================================================================
    // 时间信息
    // ========================================================================
    uint64_t timestamp_ns{0};        ///< 输出时间戳（纳秒）
    uint64_t predicted_future_ns{0};  ///< 预测的弹丸到达时间（纳秒）

    // ========================================================================
    // 预测的目标状态（odom坐标系）
    // ========================================================================
    Eigen::Vector3d predicted_position; ///< 预测的打击位置（odom坐标系）
    double distance{0.0};               ///< 到目标的距离（米）
    double flying_time{0.0};            ///< 弹丸飞行时间（秒）

    // ========================================================================
    // 瞄准角度（odom坐标系，已做弹道补偿）
    // ========================================================================
    double aim_yaw{0.0};   ///< 目标yaw角度（弧度）
    double aim_pitch{0.0}; ///< 目标pitch角度（弧度）

    // ========================================================================
    // 目标运动（供Planner层生成参考轨迹）
    // ========================================================================
    Eigen::Vector3d target_velocity; ///< 目标速度 [vx, vy, vz]（米/秒）
    double target_v_yaw{0.0};        ///< 目标角速度（弧度/秒）
    double predicted_roll{0.0};      ///< 预测的roll角度（弧度，能量机关专用）

    ArmorAimPhase aim_phase{ArmorAimPhase::SingleArmor}; ///< 瞄准阶段
    int selected_armor_id{0};         ///< 选中的装甲板ID
    int rough_selected_armor_id{0};   ///< 粗略选中的装甲板ID（用于调试）

    // ========================================================================
    // 简单开火建议（目标是否在射击窗口内）
    // ========================================================================
    ArmorType armor_type{ArmorType::Invalid}; ///< 装甲板类型

    // ========================================================================
    // 元数据
    // ========================================================================
    AimerTargetType target_type; ///< 目标类型
};

} // namespace fcs::L4
