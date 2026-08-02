#pragma once

#include "L3_estimation/tracker/types.hpp"
#include "core/armor_types.hpp"
#include "scene_builder.hpp"
#include "tactical_palette.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <optional>

namespace fcs::L3::vis {

// Shorthand
namespace viz = fcs::visualization;
namespace tac = visualization::tactical;

// ============================================================================
// Tracker Output Helper Functions - Tracker 输出辅助函数
// ============================================================================
//
// 这些函数从 TrackerOutput 中提取位置、速度等信息用于可视化。
// 原本位于 foxglove_systems_impl.hpp，现已移动到独立的 L3 模块。
// ============================================================================

/// @brief Get center position from tracker output as optional Vector3
/// @return 有效目标返回位置，无效状态返回 std::nullopt
[[nodiscard]] inline std::optional<::foxglove::schemas::Vector3>
    get_tracker_position(const TrackerOutput& output) noexcept {
    if (output.is_robot()) {
        const auto& state = *output.robot_state();
        return viz::make_vector3(state.position.x(), state.position.y(), state.position.z());
    }
    if (output.is_outpost()) {
        const auto& state  = *output.outpost_state();
        const double avg_z = (state.z[0] + state.z[1] + state.z[2]) / 3.0;
        return viz::make_vector3(state.position.x(), state.position.y(), avg_z);
    }
    return std::nullopt;
}

/// @brief Get velocity from tracker output
[[nodiscard]] inline Eigen::Vector3d get_tracker_velocity(const TrackerOutput& output) noexcept {
    if (output.is_robot()) {
        return output.robot_state()->velocity;
    }
    if (output.is_outpost()) {
        return output.outpost_state()->velocity;
    }
    return Eigen::Vector3d::Zero();
}

/// @brief Get angular velocity from tracker output
[[nodiscard]] inline double get_tracker_v_yaw(const TrackerOutput& output) noexcept {
    if (output.is_robot()) {
        return output.robot_state()->v_yaw;
    }
    if (output.is_outpost()) {
        return output.outpost_state()->v_yaw;
    }
    return 0.0;
}

// ============================================================================
// Armor Visualization Helpers - 装甲板可视化辅助函数
// ============================================================================

/// @brief 创建单个装甲板立方体
/// @param armor_pose 装甲板位姿 [x, y, z, yaw]
/// @param tilt_angle 装甲板倾斜角（前哨站为负，机器人为正）
/// @param height 装甲板高度（大/小装甲板）
[[nodiscard]] inline ::foxglove::schemas::CubePrimitive
    make_armor_cube(const Eigen::Vector4d& armor_pose, double tilt_angle, double height) noexcept {
    const Eigen::Quaterniond q = Eigen::AngleAxisd(armor_pose[3], Eigen::Vector3d::UnitZ())
                               * Eigen::AngleAxisd(tilt_angle, Eigen::Vector3d::UnitY());

    ::foxglove::schemas::CubePrimitive cube;
    cube.pose = viz::make_pose(
        viz::make_vector3(armor_pose[0], armor_pose[1], armor_pose[2]), viz::make_quaternion(q));
    cube.size  = {tac::Geometry::ARMOR_THICKNESS, height, tac::Geometry::ARMOR_WIDTH};
    cube.color = viz::make_color(0.75, 0.75, 0.75, 0.25);
    return cube;
}

/// @brief Add armor cubes to scene entity for outpost target
inline void
    add_outpost_armor_cubes(viz::EntityBuilder& builder, const OutpostTargetState& state) noexcept {
    const auto armor_poses = state.armor_poses();
    static_assert(armor_poses.size() == 3);

    for (const auto& armor_pose : armor_poses) {
        // 前哨站装甲板向下倾斜，使用小装甲板高度
        builder.add_cube(make_armor_cube(
            armor_pose, -tac::Geometry::ARMOR_TILT_ANGLE, tac::Geometry::ARMOR_HEIGHT_SMALL));
    }
}

/// @brief Add armor cubes to scene entity for robot target
inline void add_robot_armor_cubes(
    viz::EntityBuilder& builder, const RobotTargetState& state, ArmorName target_name) noexcept {
    // 复用项目统一的装甲尺寸判断逻辑：英雄(One)、基地大装甲(BaseLarge) 为大装甲
    const bool is_big_armor = cls_to_armor_type(target_name) == ArmorType::Large;
    const auto armor_poses  = state.armor_poses();
    static_assert(armor_poses.size() == 4);

    for (int i = 0; i < 4; i++) {
        // 机器人装甲板向上倾斜
        auto cube = make_armor_cube(
            armor_poses[i], tac::Geometry::ARMOR_TILT_ANGLE,
            is_big_armor ? tac::Geometry::ARMOR_HEIGHT_BIG : tac::Geometry::ARMOR_HEIGHT_SMALL);

        ::foxglove::schemas::TextPrimitive text;
        text.text = fmt::format("{}", i);
        text.pose = cube.pose;
        if (text.pose->position.has_value()) {
            text.pose->position->z += tac::L3::LABEL_OFFSET_Z;
        }
        text.font_size = tac::Text::SIZE_SMALL;
        text.color     = tac::Text::PRIMARY;
        text.billboard = tac::Text::BILLBOARD_ENABLED;

        builder.add_cube(cube);
        builder.add_text(std::move(text));
    }
}

} // namespace fcs::L3::vis
