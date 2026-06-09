#pragma once

#include "energy_meter_config.hpp"
#include "scheduler/thin.hpp"

namespace fcs::energy_meter {

/// Register energy meter L3 estimation systems with the talos scheduler.
void register_energy_meter_systems(talos::Scheduler& scheduler, EnergyMeterL3Config&& config);

} // namespace fcs::energy_meter
