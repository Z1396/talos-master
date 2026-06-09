#pragma once

#include "core/trajectory/model/ballistic_model.hpp"
#include <Eigen/Core>

#include <expected>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace fcs::core::trajectory::solver {

/// Complete trajectory solution result
///
/// Value object representing the complete result of solving for a ballistic
/// trajectory to hit a target. When returned via std::expected, a present value
/// guarantees a converged, valid solution.
struct AimSolution {
    double yaw{0.0};            ///< Firing azimuth angle (radians)
    double pitch{0.0};          ///< Firing elevation angle (radians)
    double time_of_flight{0.0}; ///< Time of flight (seconds)
    int iterations{0};          ///< Number of iterations performed
};

/// Trajectory solver interface (stateless)
///
/// This interface provides the core solving functionality for ballistic
/// trajectory compensation. It is deliberately stateless - the bullet velocity
/// is passed as a parameter rather than stored, eliminating the need for
/// const_cast tricks.
///
/// All solvers support visualization through generate_trajectory().
class TrajectorySolver {
public:
    virtual ~TrajectorySolver() = default;

    /// Solve for firing angles to hit target position
    ///
    /// @param target_pos Target position in world coordinates (meters)
    /// @param v0 Muzzle velocity (m/s)
    /// @return AimSolution on success, or error string describing the failure reason
    [[nodiscard]] virtual std::expected<AimSolution, std::string>
        solve(const Eigen::Vector3d& target_pos, double v0) const noexcept = 0;

    /// Generate trajectory points for visualization
    ///
    /// @param pitch Firing pitch angle (radians)
    /// @param v0 Muzzle velocity (m/s)
    /// @param max_distance Maximum horizontal distance to sample (meters)
    /// @return Vector of (x, z) trajectory points
    [[nodiscard]] virtual std::vector<std::pair<double, double>>
        generate_trajectory(double pitch, double v0, double max_distance) const noexcept = 0;

    /// Get human-readable solver name for debugging
    [[nodiscard]] virtual std::string_view solver_name() const noexcept = 0;

    /// Get the underlying ballistic model (for TableLookupSolver table building)
    /// This replaces dynamic_cast usage with compile-time type-safe dispatch
    [[nodiscard]] virtual const model::BallisticModel* get_model() const noexcept = 0;
};

} // namespace fcs::core::trajectory::solver
