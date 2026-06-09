#pragma once

#include "ballistic_model.hpp"

namespace fcs::core::trajectory::model {

// ============================================================================
// Linear Resistance Model
// ============================================================================

/// Ballistic model with linear air resistance (analytic solution).
///
/// Uses a simplified linear drag model where acceleration is proportional
/// to velocity. This provides an analytic solution with exp() terms.

class LinearDragModel final : public BallisticModel {
public:
    struct ResistanceParams {
        double gravity{9.8};      ///< Gravity acceleration (m/s^2)
        double resistance{0.001}; ///< Linear resistance coefficient (1/m)
    };

    // Note: no default params to avoid pedantic errors with default member initializers
    explicit LinearDragModel(const ResistanceParams& params) noexcept
        : gravity_(params.gravity)
        , resistance_(params.resistance) {}

    // Factory function for default construction
    [[nodiscard]] static LinearDragModel create_default() noexcept {
        return LinearDragModel{ResistanceParams{}};
    }

    [[nodiscard]] std::optional<ImpactResult>
        compute_impact(double range, double pitch, double v0) const noexcept override {
        if (v0 < 1e-6 || range < 0.0) {
            return ImpactResult{};
        }

        const double r         = resistance_ < 1e-4 ? 1e-4 : resistance_;
        const double cos_pitch = std::cos(pitch);
        if (std::abs(cos_pitch) < 1e-6) {
            return std::nullopt;
        }

        // Time to reach range: t = (exp(r*x) - 1) / (r * v0 * cos(theta))
        const double tof = (std::exp(r * range) - 1.0) / (r * v0 * cos_pitch);
        if (tof < 0.0 || !std::isfinite(tof)) {
            return std::nullopt;
        }

        // Vertical position: z = v0 * sin(theta) * t - 0.5 * g * t^2
        const double z = v0 * std::sin(pitch) * tof - 0.5 * gravity_ * tof * tof;

        return ImpactResult{.z = z, .tof = tof, .x = range};
    }

    [[nodiscard]] std::string_view model_name() const noexcept override { return "Resistance"; }

    [[nodiscard]] double gravity() const noexcept { return gravity_; }
    [[nodiscard]] double resistance() const noexcept { return resistance_; }

private:
    double gravity_;
    double resistance_;
};

} // namespace fcs::core::trajectory::model
