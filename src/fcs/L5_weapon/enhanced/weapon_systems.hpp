#pragma once

#include "L4_planning/config.hpp"
#include "L4_planning/control_intent.hpp"
#include "L5_weapon/config.hpp"
#include "L5_weapon/fire_control.hpp"
#include "scheduler/thin.hpp"

#include <optional>

namespace fcs::L5 {

struct WeaponControllerConfig;

[[nodiscard]] std::optional<core::trajectory::ReferenceTrajectory::AimPoint> sample_fire_trajectory(
    const L4::TrackCommand& track, const L4::ReferenceTrajectoryConfig& trajectory_cfg,
    uint64_t command_timestamp_ns) noexcept;

[[nodiscard]] WeaponCommand apply_track_fire_gate(
    WeaponCommand cmd, const L4::TrackCommand& track,
    const L4::ReferenceTrajectoryConfig& trajectory_cfg, const FireDecisionConfig& fire_cfg,
    double current_yaw, double current_pitch) noexcept;

void register_enhanced_weapon_system(talos::Scheduler& scheduler, L5Config&& config);

} // namespace fcs::L5
