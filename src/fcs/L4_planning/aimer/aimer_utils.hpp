#pragma once

#include <Eigen/Core>

#include "core/trajectory/solver/solver_interfaces.hpp"

#include <algorithm>
#include <cmath>
#include <expected>

namespace fcs::L4 {

/// Maximum iterations for ballistic flying time refinement
/// Ballistic problems typically converge in 3-5 iterations
/// Reduced from 20 for ~65% computation reduction
inline constexpr int kMaxFlyingTimeRefineIter = 7;

/// Tolerance for flying time convergence (seconds)
/// 1ms tolerance corresponds to ~1mm height error for 1m/s vertical target motion
inline constexpr double kFlyingTimeRefineTolerance = 1e-3;

/// Compute aim point (yaw, pitch) for given target position
///
/// @param solver Ballistic trajectory solver
/// @param target_pos_muzzle Target position in muzzle frame
/// @param bullet_speed Bullet speed (m/s)
/// @return {yaw, pitch} in radians, or error string if solver failed
[[nodiscard]] inline std::expected<Eigen::Vector2d, std::string> compute_aim_point(
    const core::trajectory::solver::TrajectorySolver& solver,
    const Eigen::Vector3d& target_pos_muzzle, double bullet_speed) noexcept {
    const auto solution = solver.solve(target_pos_muzzle, bullet_speed);
    if (!solution.has_value()) {
        return std::unexpected(solution.error());
    }
    return Eigen::Vector2d{solution->yaw, solution->pitch};
}

/// Perform iterative flying time refinement
///
/// @param solver Ballistic trajectory solver
/// @param predict_fn Function to predict position at given time (seconds)
/// @param initial_delay Initial delay to start from (seconds)
/// @param bullet_speed Bullet velocity (m/s)
/// @return Refined total delay (initial_delay + flying_time)
template <class PredictFn>
[[nodiscard]] double refine_flying_time(
    const core::trajectory::solver::TrajectorySolver& solver, PredictFn&& predict_fn,
    double initial_delay, double bullet_speed) noexcept {
    const Eigen::Vector3d current_pos = predict_fn(initial_delay);
    const double distance             = current_pos.norm();
    const double safe_bullet_speed    = std::max(1e-3, bullet_speed);
    double flying_time                = distance / safe_bullet_speed;
    double last_valid_flying_time     = flying_time;

    for (int i = 0; i < kMaxFlyingTimeRefineIter; ++i) {
        const Eigen::Vector3d predicted_pos = predict_fn(initial_delay + flying_time);
        const auto solution                 = solver.solve(predicted_pos, bullet_speed);

        if (!solution.has_value() || !std::isfinite(solution->time_of_flight)
            || solution->time_of_flight <= 0.0) {
            break;
        }

        if (std::abs(solution->time_of_flight - flying_time) < kFlyingTimeRefineTolerance) {
            last_valid_flying_time = solution->time_of_flight;
            break;
        }
        flying_time            = solution->time_of_flight;
        last_valid_flying_time = flying_time;
    }
    return initial_delay + last_valid_flying_time;
}

} // namespace fcs::L4
