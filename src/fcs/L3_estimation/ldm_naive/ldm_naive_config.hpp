#pragma once

#include "ldm_kinematic_params.hpp"

#include <cstdint>

namespace fcs::L3::ldm {

struct NaiveLdmConfig {
    uint32_t tracking_threshold{5};
    double lost_threshold{1.0};

    double initial_sigma_rot{0.05};
    double initial_sigma_velocity_body{5.0};
    double initial_sigma_position{0.20};

    LdmKinematicParams model{};
};

} // namespace fcs::L3::ldm
