/**
 * @file gimbal_planner/types.hpp
 * @brief L4规划层云台规划器类型定义
 *
 * 本文件定义了云台规划器的核心数据结构，包括：
 * - PlannerSeed：轨迹构建器的原始输入
 * - 通道Topics：L4层的数据流标识
 *
 * 设计理念：
 * - 使用variant表达多种目标状态，避免继承和虚函数
 * - 提供便捷的类型检查和访问方法
 * - 与L3跟踪器输出解耦，规划层有自己的数据视图
 *
 * 数据流：
 * L3::TrackerOutput → PlannerSeed → trajectory_builder → ReferenceTrajectory
 */

#pragma once

#include "L3_estimation/tracker/types.hpp"
#include "L4_planning/aimer/types.hpp"
#include "core/armor_types.hpp"
#include "core/channel_topics.hpp"

#include <cstdint>
#include <optional>
#include <variant>

namespace fcs::L4 {

// ============================================================================
// 规划器种子（L4内部数据结构）
// ============================================================================

/**
 * @brief 规划器种子
 *
 * 包含轨迹构建器所需的原始L3目标状态。
 * 这是L4规划层的内部数据结构，用于在轨迹构建过程中传递目标信息。
 *
 * 设计考虑：
 * - 使用variant存储机器人或前哨站状态，避免继承
 * - 包含时间戳、跳跃标记、瞄准阶段等完整上下文
 * - 提供便捷的类型检查方法，简化客户端代码
 *
 * 生命周期：
 * - 由try_armor()创建
 * - 传递给trajectory_builder
 * - 使用后立即销毁
 */
struct PlannerSeed {
    using State = std::variant<std::monostate, L3::RobotTargetState, L3::OutpostTargetState>;

    uint64_t state_timestamp_ns{0};                 ///< 状态时间戳（纳秒）
    State state{};                                  ///< 目标状态（机器人或前哨站）
    bool target_jumped{false};                      ///< 目标是否跳跃（新装甲板出现）
    std::optional<int> tracker_last_armor_id{};     ///< 跟踪器最后看到的装甲板ID
    ArmorAimPhase aim_phase{ArmorAimPhase::SingleArmor}; ///< 瞄准阶段
    int selected_armor_id{0};                       ///< 选中的装甲板ID
    ArmorType armor_type{ArmorType::Invalid};       ///< 装甲板类型

    /**
     * @brief 检查是否为机器人目标
     * @return true表示机器人，false表示其他
     */
    [[nodiscard]] bool is_robot() const noexcept {
        return std::holds_alternative<L3::RobotTargetState>(state);
    }

    /**
     * @brief 检查是否为前哨站目标
     * @return true表示前哨站，false表示其他
     */
    [[nodiscard]] bool is_outpost() const noexcept {
        return std::holds_alternative<L3::OutpostTargetState>(state);
    }

    /**
     * @brief 获取机器人状态指针
     * @return 机器人状态指针，如果不是机器人则返回nullptr
     */
    [[nodiscard]] const L3::RobotTargetState* robot_state() const noexcept {
        return std::get_if<L3::RobotTargetState>(&state);
    }

    /**
     * @brief 获取前哨站状态指针
     * @return 前哨站状态指针，如果不是前哨站则返回nullptr
     */
    [[nodiscard]] const L3::OutpostTargetState* outpost_state() const noexcept {
        return std::get_if<L3::OutpostTargetState>(&state);
    }
};

// ============================================================================
// L4通道Topics
// ============================================================================

using fcs::L3::TrackerOutputChannelTopic;          ///< 跟踪器输出通道Topic（复用L3定义）

} // namespace fcs::L4
