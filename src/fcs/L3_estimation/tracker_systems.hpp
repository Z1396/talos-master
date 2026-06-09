#pragma once

// Keep this header as a thin API surface.
// The implementation lives in `src/fcs/L3_estimation/tracker_systems.cpp`.

#include "scheduler/thin.hpp"

namespace fcs::L3 {

class Tracker;
struct TrackerConfig;

/// Register L3 tracker pipeline systems
///
/// Pipeline:
///   ArmorMeasurementBatch [SPMC] -> TrackerSystem [pool_compute]
///                                               ↓
///                                        TrackerOutput [SPMC]
///
/// ## Parameters
///
/// - `scheduler`: the scheduler to register systems with
/// - `config`: tracker configuration (will be moved into resource)
void register_tracker_systems(talos::Scheduler& scheduler, TrackerConfig&& config);

} // namespace fcs::L3
