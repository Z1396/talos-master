#pragma once

#include <Eigen/Core>
#include <optional>

namespace fcs::core::trajectory::model {

/// Result of computing bullet impact at a given horizontal distance.
struct ImpactResult {
    double z{0.0};   ///< Vertical position at impact (meters, positive = up)
    double tof{0.0}; ///< Time of flight to impact (seconds)
    double x{0.0};   ///< Actual horizontal distance reached (meters)
};

/// Abstract ballistic model for computing bullet trajectories.
///
/// This interface separates the physical model from the solving algorithm,
/// allowing different models (ideal, linear drag, quadratic drag) to be
/// combined with different solvers (iterative, table lookup).
class BallisticModel {
public:
    virtual ~BallisticModel() = default;

    /// Compute bullet impact at a given horizontal distance.
    ///
    /// @param range Target horizontal distance (meters)
    /// @param pitch Firing angle relative to horizontal (radians, positive = up)
    /// @param v0 Muzzle velocity (m/s)
    /// @return Impact position and time of flight, or nullopt if computation fails
    [[nodiscard]] virtual std::optional<ImpactResult>
        compute_impact(double range, double pitch, double v0) const noexcept = 0;

    /// Get human-readable model name for debugging.
    [[nodiscard]] virtual std::string_view model_name() const noexcept = 0;
};

} // namespace fcs::core::trajectory::model
