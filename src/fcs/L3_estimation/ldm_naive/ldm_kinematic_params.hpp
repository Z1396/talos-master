#pragma once

#include <cstddef>

namespace fcs::L3::ldm {

/// Noise parameters for the LDM kinematic model.
///
/// Extracted from LdmKinematic so that headers only needing config
/// (e.g. NaiveLdmConfig) do not transitively pull in <groups/SEn3.hpp>.
struct LdmKinematicParams {
    using Scalar = double;

    Scalar sigma_inert_omega = Scalar(50.0); // rad/s
    Scalar sigma_inert_accel = Scalar(50.0); // m/s^2

    // SO(3) log residual noise, rad.
    Scalar sigma_rot_x = Scalar(0.5);
    Scalar sigma_rot_y = Scalar(0.5);
    Scalar sigma_rot_z = Scalar(1.0);

    // Bearing measurement noise, rad.
    Scalar sigma_r_bearing_yaw   = Scalar(5e-1);
    Scalar sigma_r_bearing_pitch = Scalar(2e-1);

    // Distance measurement noise:
    // sigma_d = sigma_distance_min
    //         + k_distance_depth * abs(depth)
    //         + k_distance_planar * planar_offset
    Scalar sigma_distance_min = Scalar(0.01); // m
    Scalar k_distance_depth   = Scalar(0.5);  // m / m
    Scalar k_distance_planar  = Scalar(0.5);  // m / m
};

} // namespace fcs::L3::ldm
