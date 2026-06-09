#pragma once

#include "core/trajectory/reference_trajectory.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

namespace fcs::L4 {

// ============================================================================
// L4 → L5 Control Intent
// ============================================================================
//
// L4 translates tracker state into a pure control intention for L5.
// The variant constructors precisely encode the three operational modes:
//
//   TrackCommand  = "I predicted a trajectory. Follow it with MPC."
//   ShotCommand   = "Target is here. Aim and fire."
//   HoldCommand   = "No target. Stay put."
//
// degradation_reason on ShotCommand is set only when trajectory construction
// failed and L4 is falling back to direct aim.  Normal Shots (Rune, pre-tracking)
// leave it nullopt so downstream can distinguish intentional shots from degraded ones.
//
// No enum tags, no runtime checks needed.  The variant IS the specification.

/// Track: full reference trajectory for MPC optimization.
struct TrackCommand {
    uint64_t timestamp_ns{0};
    core::trajectory::ReferenceTrajectory control_trajectory;
    core::trajectory::ReferenceTrajectory fire_trajectory;
};

/// Shot: direct aim without trajectory optimization.
struct ShotCommand {
    uint64_t timestamp_ns{0};
    double yaw{0.0};
    double pitch{0.0};
    double distance{0.0};

    /// Set when this ShotCommand is a degradation fallback (trajectory build
    /// failed and L4 fell through to direct aim) rather than an intentional
    /// direct-shot (Rune, pre-tracking, etc.).
    /// nullopt = intentional shot.
    std::optional<std::string> degradation_reason{};
};

/// Hold: no target, maintain current position.
struct HoldCommand {
    uint64_t timestamp_ns{0};
};

/// L4 → L5 control intent.
using ControlIntent = std::variant<TrackCommand, ShotCommand, HoldCommand>;

} // namespace fcs::L4
