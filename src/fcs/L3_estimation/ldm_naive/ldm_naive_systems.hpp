#pragma once

#include "ldm_naive_config.hpp"
#include "scheduler/thin.hpp"

namespace fcs::L3::ldm {

/// Register naive LDM estimation system with the scheduler.
void register_naive_ldm_systems(talos::Scheduler& scheduler, const NaiveLdmConfig& config);

} // namespace fcs::L3::ldm
