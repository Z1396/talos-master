#pragma once

#include "core/math/normalize.hpp"

#include <algorithm>
#include <cmath>

namespace fcs::L5 {

/// Fire decision configuration
struct FireDecisionConfig {
    /// Minimum shooting threshold (radians), used for long distances
    /// Default aligned with sp_vision_25 planner
    double fire_thresh{0.003};
    /// Physical window dimensions (meters)
    double shooting_range_h{0.12};
    double shooting_range_w_small{0.12};
    double shooting_range_w_large{0.24};
};

struct FireDecision {
    bool fire;
    double yaw_error;
    double pitch_error;
    double shooting_range_yaw;
    double shooting_range_pitch;
};

/// Check if target is within shootable range
///
/// Converts physical window (meters) to angular tolerance and checks if current gimbal
/// attitude is within range. This is the unified fire decision logic used by L4 aimers
/// and the L5 fire gate.
///
/// @param cfg Fire decision configuration
/// @param cur_yaw Current gimbal yaw (radians)
/// @param cur_pitch Current gimbal pitch (radians)
/// @param target_yaw Target yaw (radians)
/// @param target_pitch Target pitch (radians)
/// @param distance Distance to target (meters)
///
/// @return true if target is within shootable range
[[nodiscard]] inline FireDecision is_on_target(
    const FireDecisionConfig& cfg, double cur_yaw, double cur_pitch, double target_yaw,
    double target_pitch, double distance) noexcept {

    // Convert physical window to angular tolerance
    auto shooting_range_yaw   = std::abs(std::atan2(cfg.shooting_range_w_small / 2.0, distance));
    auto shooting_range_pitch = std::abs(std::atan2(cfg.shooting_range_h / 2.0, distance));

    // Set minimum angle threshold
    const auto max_error = cfg.fire_thresh;
    shooting_range_yaw   = std::max(shooting_range_yaw, max_error);
    shooting_range_pitch = std::max(shooting_range_pitch, max_error);
    auto yaw_error       = std::abs(core::math::normalize_angle(target_yaw - cur_yaw));
    auto pitch_error     = std::abs(core::math::normalize_angle(target_pitch - cur_pitch));
    return FireDecision{
        .fire = std::abs(core::math::normalize_angle(target_yaw - cur_yaw)) < shooting_range_yaw
             && std::abs(core::math::normalize_angle(target_pitch - cur_pitch))
                    < shooting_range_pitch,
        .yaw_error            = yaw_error,
        .pitch_error          = pitch_error,
        .shooting_range_yaw   = shooting_range_yaw,
        .shooting_range_pitch = shooting_range_pitch};
}
} // namespace fcs::L5
