#pragma once

#include "L3_estimation/ldm_naive/ldm_kinematic_model.hpp"
#include "L3_estimation/tracker/types.hpp"

#include <Eigen/Core>

#include <cstdint>

namespace fcs::L3::ldm {

/// LDM tracker state — SE2(3) group element + metadata.
///
/// The core state is stored as a full SE2(3) (SEn3<double, 2>) nominal, NOT
/// decomposed into flat Vector3d fields.  Flat position / rotation / velocity
/// accessors exist only at the **visualization boundary** — they are NOT the
/// internal representation.
///
/// ## SE2(3) structure (Nominal)
///
///   X = (R, v_body, p)
///     • R       — SO(3) rotation           (world → body)
///     • v_body  — body-frame velocity      (R³)
///     • p       — world-frame position     (R³)
///
/// Prediction is done via Lie algebra right-perturbation:
///
///   xi  = (dθ=0, dv=0, dp=v_body·dt)
///   X̂₊ = X̂ * exp(xi)
///
struct LdmState {
    using Nominal    = group::SEn3<double, 2>;
    using SO3Type    = Nominal::SO3Type;
    using VectorType = Eigen::Vector3d;

    // ========================================================================
    // Timestamps and status
    // ========================================================================

    uint64_t timestamp_ns{0};
    uint64_t last_observation_timestamp_ns{0};
    TrackerStatus status{TrackerStatus::Idle};
    bool accurate{false};

    // ========================================================================
    // SE2(3) group element — THE internal state.  NOT decomposed.
    // ========================================================================

    Nominal X{}; // default = identity (R=I3, v=0, p=0)

    // ========================================================================
    // Pre-computed prediction (for L4 aimer convenience)
    // ========================================================================

    Eigen::Vector3d predicted_position_odom{Eigen::Vector3d::Zero()};
    uint64_t predicted_future_ns{0};

    // ========================================================================
    // Visualization-boundary accessors
    //
    // These exist ONLY so that Foxglove/visualisation code can read flat
    // Vector3d/Matrix3d without including the full SE2(3) machinery.
    // They are NOT the system-internal representation.
    // ========================================================================

    /// World-frame position (visualization boundary).
    [[nodiscard]] const Eigen::Vector3d& position() const noexcept { return X.p(); }

    /// World→body rotation matrix (visualization boundary).
    [[nodiscard]] const Eigen::Matrix3d& rotation() const noexcept { return X.R(); }

    /// Body-frame velocity (visualization boundary).
    [[nodiscard]] const Eigen::Vector3d& velocity_body() const noexcept { return X.v(); }

    /// World-frame velocity (computed on access — not stored).
    [[nodiscard]] Eigen::Vector3d velocity_world() const noexcept { return X.R() * X.v(); }

    // ========================================================================
    // Query helpers
    // ========================================================================

    [[nodiscard]] bool is_tracking() const noexcept {
        return status == TrackerStatus::Tracking || status == TrackerStatus::TempLost;
    }
};

} // namespace fcs::L3::ldm
