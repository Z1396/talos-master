#pragma once

#include "ballistic_model.hpp"

namespace fcs::core::trajectory::model {

// ============================================================================
// Ideal Ballistic Model (No Air Resistance)
// ============================================================================

/// Ideal ballistic model with no air resistance (analytic solution).
///
/// Uses the standard vacuum trajectory equations:
/// - x(t) = v0 * cos(theta) * t
/// - z(t) = v0 * sin(theta) * t - 0.5 * g * t^2
///
/// This model provides a closed-form solution with O(1) computation.
class IdealModel final : public BallisticModel {
public:
    explicit IdealModel(double gravity = 9.8) noexcept
        : gravity_(gravity) {}

    [[nodiscard]] std::optional<ImpactResult>
        compute_impact(double range, double pitch, double v0) const noexcept override {
        if (v0 < 1e-6 || range < 0.0) {
            return ImpactResult{};
        }

        const double cos_pitch = std::cos(pitch);
        if (std::abs(cos_pitch) < 1e-6) {
            return std::nullopt; // Vertical shot, cannot reach horizontal range
        }

        const double tof = range / (v0 * cos_pitch);
        if (tof < 0.0 || !std::isfinite(tof)) {
            return std::nullopt;
        }

        const double z = v0 * std::sin(pitch) * tof - 0.5 * gravity_ * tof * tof;

        return ImpactResult{.z = z, .tof = tof, .x = range};
    }

    [[nodiscard]] std::string_view model_name() const noexcept override { return "Ideal"; }

    [[nodiscard]] double gravity() const noexcept { return gravity_; }

private:
    double gravity_;
};

} // namespace fcs::core::trajectory::model
