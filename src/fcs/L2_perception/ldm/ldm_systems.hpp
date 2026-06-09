#pragma once

#include "L2_perception/ldm/ldm_config.hpp"
#include "camera_config.hpp"
#include "scheduler/thin.hpp"

namespace fcs::L2::ldm {

/// @brief Register LDM detection systems (detector + solver)
/// @param scheduler The scheduler to register systems with
/// @param config LDM detector configuration
void register_ldm_systems(
    talos::Scheduler& scheduler, LdmDetectorConfig&& config,
    const CameraConfig& camera_config) noexcept;

} // namespace fcs::L2::ldm
