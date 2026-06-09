#pragma once

#include "core/armor_types.hpp"

#include <Eigen/Core>
#include <cstdint>
#include <optional>

namespace fcs::L4 {

enum class ArmorAimPhase : uint8_t {
    SingleArmor    = 0,
    WholeCarArmor  = 1,
    WholeCarPair   = 2,
    WholeCarCenter = 3,
};

struct ArmorAimContext {
    bool target_jumped{false};
    ArmorAimPhase phase{ArmorAimPhase::SingleArmor};
    std::optional<int> preferred_armor_id{};
};

/// Aimer target type enumeration
enum class AimerTargetType : uint8_t {
    Robot,   ///< Robot target (Infantry/Hero/Sentry)
    Outpost, ///< Outpost target
    Rune,    ///< Rune (Energy Depot) target
    Ldm,     ///< Rune (Energy Depot) target
};

// ============================================================================
// Target Prediction - Pure Prediction Result from Aimer
// ============================================================================

/// Target prediction result from Aimer
///
/// This is the output of the Aimer layer, which performs pure prediction
/// with ballistic compensation. The Planner layer consumes this output
/// to generate optimized trajectories with feedforward terms.
///
/// Contains predicted target state at bullet arrival time, computed aim
/// angles (with ballistic compensation), and simple fire advice.
struct TargetPrediction {
    // ========================================================================
    // Time Information
    // ========================================================================
    uint64_t timestamp_ns{0};        ///< Output timestamp (ns)
    uint64_t predicted_future_ns{0}; ///< Predicted bullet arrival time (ns)

    // ========================================================================
    // Predicted Target State (odom frame)
    // ========================================================================
    Eigen::Vector3d predicted_position; ///< Predicted hit position (odom frame)
    double distance{0.0};               ///< Distance to target (meters)
    double flying_time{0.0};            ///< Bullet flight time (seconds)

    // ========================================================================
    // Aim Angles (odom frame, with ballistic compensation)
    // ========================================================================
    double aim_yaw{0.0};   ///< Target yaw angle (radians)
    double aim_pitch{0.0}; ///< Target pitch angle (radians)

    // ========================================================================
    // Target Motion (for Planner reference trajectory generation)
    // ========================================================================
    Eigen::Vector3d target_velocity; ///< Target velocity [vx, vy, vz] (m/s)
    double target_v_yaw{0.0};        ///< Target angular velocity (rad/s)
    double predicted_roll{0.0};      ///< Predicted roll angle (radians, for Rune)
    ArmorAimPhase aim_phase{ArmorAimPhase::SingleArmor};
    int selected_armor_id{0};
    int rough_selected_armor_id{0};

    // ========================================================================
    // Simple Fire Advice (is target in shooting window?)
    // ========================================================================
    ArmorType armor_type{ArmorType::Invalid};

    // ========================================================================
    // Metadata
    // ========================================================================
    AimerTargetType target_type; ///< Target type
};

} // namespace fcs::L4
