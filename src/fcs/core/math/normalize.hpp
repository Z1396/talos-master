#pragma once

#include <cmath>
#include <numbers>

namespace fcs::core::math {

/// Normalize angle to [-pi, pi].
[[nodiscard]] inline double normalize_angle(double angle) noexcept {
    double r = std::remainder(angle, 2.0 * std::numbers::pi);

    if (r <= -std::numbers::pi)
        r += 2.0 * std::numbers::pi;

    return r;
}
[[nodiscard]] inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz) noexcept {
    const double x = xyz.x();
    const double y = xyz.y();
    const double z = xyz.z();

    const double distance   = xyz.norm();
    const double yaw        = std::atan2(y, x);
    const double horizontal = std::hypot(x, y); // More accurate and faster than sqrt(x*x + y*y)
    const double pitch      = std::atan2(-z, horizontal);

    return {yaw, pitch, distance};
}

} // namespace fcs::core::math
