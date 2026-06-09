#pragma once

#include <Eigen/Core>

#include <vector>

namespace fcs::core::trajectory {

/// Reference trajectory for weapon control.
///
/// Contains the state matrix [yaw, yaw_rate, pitch, pitch_rate] × horizon,
/// distances, time-of-flight values, and the yaw origin.
///
/// The center step (where relative yaw ≈ 0) is the current aim point
/// that L4 originally computed.
///
/// Built by L4 planning layer, consumed by L5 weapon layer.
struct ReferenceTrajectory {
    using StateMatrix = Eigen::Matrix<double, 4, Eigen::Dynamic>;

    /// State matrix: 4 rows × horizon columns.
    /// Row 0: yaw (relative to yaw_origin)
    /// Row 1: yaw_rate (rad/s)
    /// Row 2: pitch (rad)
    /// Row 3: pitch_rate (rad/s)
    StateMatrix state;

    /// Distance to target at each horizon step (meters).
    std::vector<double> distances;

    /// Projectile time-of-flight at each step (seconds).
    std::vector<double> time_of_flights;

    /// Yaw angle origin for normalization (yaw at center horizon step).
    double yaw_origin{0.0};

    /// Total horizon length.
    [[nodiscard]] int horizon() const noexcept { return static_cast<int>(state.cols()); }

    /// The aim point at the trajectory center (current L4 target).
    /// Returns yaw, pitch, distance for the step closest to the origin.
    struct AimPoint {
        double yaw{0.0};
        double pitch{0.0};
        double distance{0.0};
    };

    [[nodiscard]] AimPoint center_aim_point() const noexcept {
        const int n = horizon();
        if (n <= 0)
            return {};
        int center = 0;
        for (int k = 1; k < n; ++k)
            if (std::abs(state(0, k)) < std::abs(state(0, center)))
                center = k;
        return AimPoint{
            .yaw      = yaw_origin,
            .pitch    = state(2, center),
            .distance = distances[center],
        };
    }
};

} // namespace fcs::core::trajectory
