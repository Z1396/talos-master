#pragma once

#include "L3_estimation/tracker/types.hpp"
#include "scene_builder.hpp"
#include "tactical_palette.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/format.h>

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

/// @brief Get center position from tracker output as Vector3
[[nodiscard]] inline ::foxglove::schemas::Vector3
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
    return viz::make_vector3(0, 0, 0);
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

/// @brief Add armor cubes to scene entity for outpost target
inline void
    add_outpost_armor_cubes(viz::EntityBuilder& builder, const OutpostTargetState& state) noexcept {
    const auto armor_poses = state.armor_poses();
    static_assert(armor_poses.size() == 3);

    for (const auto& armor_pose : armor_poses) {
        const Eigen::Quaterniond q =
            Eigen::AngleAxisd(armor_pose[3], Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(-tac::Geometry::ARMOR_TILT_ANGLE, Eigen::Vector3d::UnitY());

        ::foxglove::schemas::CubePrimitive cube;
        cube.pose = viz::make_pose(
            viz::make_vector3(armor_pose[0], armor_pose[1], armor_pose[2]),
            viz::make_quaternion(q));
        cube.size = {
            tac::Geometry::ARMOR_THICKNESS, tac::Geometry::ARMOR_HEIGHT_SMALL,
            tac::Geometry::ARMOR_WIDTH};
        cube.color = viz::make_color(0.75, 0.75, 0.75, 0.25);
        builder.add_cube(std::move(cube));
    }
}

/// @brief Add armor cubes to scene entity for robot target
inline void add_robot_armor_cubes(
    viz::EntityBuilder& builder, const RobotTargetState& state, ArmorName target_name) noexcept {
    const bool is_big_armor = target_name == ArmorName::One || target_name == ArmorName::Base;
    const auto armor_poses  = state.armor_poses();
    static_assert(armor_poses.size() == 4);

    for (int i = 0; i < 4; i++) {
        const Eigen::Quaterniond q =
            Eigen::AngleAxisd(armor_poses[i][3], Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(tac::Geometry::ARMOR_TILT_ANGLE, Eigen::Vector3d::UnitY());

        ::foxglove::schemas::CubePrimitive cube;
        cube.pose = viz::make_pose(
            viz::make_vector3(armor_poses[i][0], armor_poses[i][1], armor_poses[i][2]),
            viz::make_quaternion(q));
        cube.size = {
            tac::Geometry::ARMOR_THICKNESS,
            is_big_armor ? tac::Geometry::ARMOR_HEIGHT_BIG : tac::Geometry::ARMOR_HEIGHT_SMALL,
            tac::Geometry::ARMOR_WIDTH};
        cube.color = viz::make_color(0.75, 0.75, 0.75, 0.25);
        builder.add_cube(std::move(cube));

        ::foxglove::schemas::TextPrimitive text;
        text.text = fmt::format("{}", i);
        text.pose = cube.pose;
        if (text.pose->position.has_value()) {
            text.pose->position->z += tac::L3::LABEL_OFFSET_Z;
        }
        text.font_size = tac::Text::SIZE_SMALL;
        text.color     = tac::Text::PRIMARY;
        text.billboard = tac::Text::BILLBOARD_ENABLED;
        builder.add_text(std::move(text));
    }
}

} // namespace fcs::L3::vis
