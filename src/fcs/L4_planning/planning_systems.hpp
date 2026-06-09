#pragma once

#include "scheduler/thin.hpp"

namespace fcs::L4 {

struct L4Config;

/// Register L4 Aimer + Planner pipeline systems
///
/// Unified architecture replacing the legacy monolithic planner:
/// - Aimer layer: RobotAimer, OutpostAimer, RuneAimer (pure prediction)
/// - Plan adapter layer: wraps aimer prediction into ControlIntent (Track/Shot/Hold)
///
/// @param scheduler The scheduler
/// @param config Complete L4 configuration (aimers + planner)
void register_l4_planning_systems(::talos::Scheduler& scheduler, L4Config&& config);

} // namespace fcs::L4
