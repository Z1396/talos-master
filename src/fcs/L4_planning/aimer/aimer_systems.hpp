#pragma once

#include "L4_planning/config.hpp"
#include "scheduler/thin.hpp"

namespace fcs::L4 {

/// Register aimer systems with the scheduler
///
/// This function registers three independent aimer systems:
/// - RobotAimerSystem: processes RobotTargetState and publishes AimerOutput
/// - OutpostAimerSystem: processes OutpostTargetState and publishes AimerOutput
/// - RuneAimerSystem: processes EnergyMeterState and publishes AimerOutput
///
/// All aimers consume tracker/energy meter outputs and produce pure prediction
/// results (AimerOutput) that are consumed by the planner layer.
///
/// @param scheduler The scheduler
/// @param config L4 planning configuration
void register_aimer_systems(talos::Scheduler& scheduler, L4Config&& config);

} // namespace fcs::L4
