#pragma once

#include "L3_estimation/tracker/util.hpp"
#include "ldm_kinematic_params.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <groups/SEn3.hpp>
#include <limits>
#include <numbers>

namespace fcs::L3::ldm {

enum LdmPoseUpdate : uint8_t {
    ROT_X,
    ROT_Y,
    ROT_Z,
    BEARING_YAW,
    BEARING_PITCH,
    BEARING_DISTANCE,
    POSE_UPDATE_MAX
};

struct LdmKinematic {
    using Scalar            = double;
    static constexpr int NZ = POSE_UPDATE_MAX;

    using Nominal           = group::SEn3<Scalar, 2>;
    using Xi                = Nominal::VectorType;
    using CovXi             = Nominal::TMatrixType;
    static constexpr int NX = Xi::RowsAtCompileTime;

    using Innovation         = Eigen::Matrix<Scalar, NZ, 1>;
    using CovInnovation      = Eigen::Matrix<Scalar, NZ, NZ>;
    using InnovationJacobian = Eigen::Matrix<Scalar, NZ, NX>;

    /// Re-exported from ldm_kinematic_params.hpp for backward compatibility.
    using Params = LdmKinematicParams;

    /// The symmetry group of the LDM octagonal prism.
    ///
    /// The LDM is a regular octagonal prism. Its pose observation has a C₈
    /// face-index ambiguity: rotating the body frame by n·45° about body Y
    /// gives an equivalent face assignment.
    ///
    /// Therefore the visual measurement is not a single R in SO(3), but an
    /// equivalence class:
    ///
    ///     [R_obs] = { R_obs * Ry(n·π/4) | n = 0..7 }  ⊂  SO(3)/C₈
    ///
    /// The full prism cross-section has D₈ symmetry (8 rotations + 8
    /// reflections), but reflections are improper (det=-1), not in SO(3).
    /// Only the rotation subgroup C₈ ⊂ SO(3) is relevant for pose filtering.
    ///
    /// ## Why not D₈ or the octahedral group O?
    ///
    ///   • D₈ reflections are improper (det=-1) — not representable in SO(3).
    ///   • The prism has one distinguished central axis (body Y).  The other
    ///     two axes are defined by the face normal and the face-tangent
    ///     direction, which are C₈-degenerate.  This is different from an
    ///     octahedron (no distinguished axis, 24 chiral symmetries).
    ///
    /// ## Physical interpretation
    ///
    /// The camera can see which way the prism's central axis points, but
    /// "which face is face 0" is ambiguous by n·45°.  Equivalently:
    ///
    ///     R  ≈  (central_axis_direction, roll_around_axis)
    ///     roll_around_axis ～  roll_around_axis + n·π/4
    static constexpr int kSymmetryCount = 8;

    static const std::array<Eigen::Matrix3d, kSymmetryCount>& symmetry_rotations() noexcept {
        constexpr double kPi4  = std::numbers::pi_v<double> / 4.0;
        static const auto kSym = [] {
            std::array<Eigen::Matrix3d, kSymmetryCount> sym{};
            for (int n = 0; n < kSymmetryCount; ++n) {
                sym[static_cast<size_t>(n)] =
                    Eigen::AngleAxisd(static_cast<double>(n) * kPi4, Eigen::Vector3d::UnitY())
                        .toRotationMatrix();
            }
            return sym;
        }();
        return kSym;
    }

    /// Nearest-lift: lift [R_obs] ∈ SO(3)/C₈ back to a continuous
    /// trajectory in SO(3) by picking the C₈ representative closest to the
    /// reference rotation `R_ref` (the filter's predicted rotation).
    ///
    /// ## Motivation
    ///
    /// The 8 faces of the octagonal prism are identical → PnP face assignment
    /// is arbitrary and R_obs can jump by n·45° between frames.  Instead of
    /// maintaining 8 EKF branches, we *lift* R_obs to the one representative
    /// closest to R_ref.  This is valid when the angular displacement per
    /// frame is much smaller than 45°/2 = 22.5° (the C₈ branch safety
    /// radius), which holds for typical non-tumbling drone motion.
    ///
    /// Mathematically:
    ///
    ///     R_canon = R_obs · S*
    ///     S* = argmin_{S ∈ C₈}  ||log(R_refᵀ · R_obs · S)||
    ///
    /// This is NOT recovering an absolute face index.  It is a local lift
    /// from the quotient SO(3)/C₈ back to SO(3), using temporal continuity
    /// to pick the representative whose face labels extend from the previous
    /// frame.
    ///
    /// ## Returns
    ///
    ///   {R_canon, branch_confidence}
    ///
    ///   branch_confidence = geodesic distance to the *second-best*
    ///                       symmetry representative minus the best.
    ///   Large (> a few degrees) → unambiguous lift.
    ///   Small → R_obs sits near the C₈ branch boundary; the filter should
    ///           inflate rotation noise (see R()).
    struct LiftResult {
        Eigen::Matrix3d R_canon;
        double branch_confidence;
    };

    [[nodiscard]] static LiftResult
        nearest_lift(const Eigen::Matrix3d& R_obs, const Eigen::Matrix3d& R_ref) noexcept {
        using SO3 = typename Nominal::SO3Type;

        const auto& sym = symmetry_rotations();

        Eigen::Matrix3d R_best = R_obs;
        double best_d          = std::numeric_limits<double>::infinity();
        double second_d        = std::numeric_limits<double>::infinity();

        for (const auto& S : sym) {
            const Eigen::Matrix3d R_candidate = R_obs * S;
            const Eigen::Matrix3d R_err       = R_ref.transpose() * R_candidate;
            const Eigen::Vector3d err         = SO3::log(R_err);
            const double d                    = err.norm();

            if (d < best_d) {
                second_d = best_d;
                best_d   = d;
                R_best   = R_candidate;
            } else if (d < second_d) {
                second_d = d;
            }
        }

        const double confidence = (second_d < std::numeric_limits<double>::infinity())
                                    ? (second_d - best_d)
                                    : std::numeric_limits<double>::infinity();

        return {R_best, confidence};
    }

    struct PoseMeasurement {
        Eigen::Matrix3d R_world_body{Eigen::Matrix3d::Identity()};
        Eigen::Vector3d p_world_body{Eigen::Vector3d::Zero()};

        /// Branch ambiguity from the last nearest-lift, populated by the
        /// tracker before calling pose_innovation.  Large → unambiguous.
        double branch_confidence{std::numeric_limits<double>::infinity()};
    };

    Params params{};

    static Nominal predict_state(const Nominal& X, Scalar dt) noexcept {
        if (dt <= Scalar(0)) {
            return X;
        }

        const auto v = X.v();

        Xi xi = Xi::Zero();

        // SEn3<*,2> stores [rotation, body-frame velocity, world position].
        // Right-multiplying dp integrates p_dot = R * v_body.
        xi.template segment<3>(6).noalias() = v * dt;

        return X * Nominal::exp(xi);
    }

    [[nodiscard]] Nominal f(const Nominal& x, double dt) const noexcept {
        return predict_state(x, dt);
    }

    [[nodiscard]] static Innovation
        pose_innovation(const Nominal& predicted, const PoseMeasurement& observed) noexcept {
        using SO3 = typename Nominal::SO3Type;

        Innovation nu = Innovation::Zero();

        // SO(3) residual in body frame.
        const Eigen::Matrix3d R_pred  = predicted.R();
        const Eigen::Matrix3d dR_mat  = R_pred.transpose() * observed.R_world_body;
        const Eigen::Vector3d rot_err = SO3::log(SO3(dR_mat));

        // ROT_X and ROT_Z (pitch/roll, observable directions) — raw SO(3) residual.
        // The prism's central axis is body Y; pitch/roll around X/Z are determined
        // by the camera view of the axis direction and have no C₈ ambiguity.
        // This component-wise split of the SO(3) log into ROT_X, ROT_Y, ROT_Z
        // is a local approximation near the predicted attitude.  It is valid
        // when axis alignment is close (|Δφ| << 22.5°), which holds under the
        // same safety condition as nearest_lift.
        nu[ROT_X] = rot_err.x();
        nu[ROT_Z] = rot_err.z();

        // ROT_Y (roll around the prism's central axis) — C₈ latent residual.
        //
        // The 8 faces are identical → the raw yaw error Δφ has an n·45° ambiguity.
        // Instead of picking a face label (discrete branch), we project through
        // the 8-fold phase, which is C₈-invariant and continuous:
        //
        //     δφ = (1/8) · atan2(sin(8·Δφ), cos(8·Δφ))
        //
        // This naturally lands in [-22.5°, 22.5°] with no branch boundary.
        // The R_pred vs R_obs axis alignment must already be close (|Δφ| << 22.5°)
        // for the EKF linearization to hold — this is the same safety condition as
        // the nearest-lift approach, but without the explicit face label selection.
        nu[ROT_Y] = std::atan2(std::sin(8.0 * rot_err.y()), std::cos(8.0 * rot_err.y())) / 8.0;

        // ── Bearing innovation (unchanged) ──
        const Eigen::Vector3d predicted_ypd = fcs::L3::xyz2ypd(predicted.p());
        const Eigen::Vector3d observed_ypd  = fcs::L3::xyz2ypd(observed.p_world_body);
        nu[BEARING_YAW]      = fcs::L3::shortest_rad(predicted_ypd.x(), observed_ypd.x());
        nu[BEARING_PITCH]    = fcs::L3::shortest_rad(predicted_ypd.y(), observed_ypd.y());
        nu[BEARING_DISTANCE] = observed_ypd.z() - predicted_ypd.z();
        return nu;
    }

    [[nodiscard]] static InnovationJacobian pose_update_H(const Nominal& predicted) noexcept {
        InnovationJacobian H = InnovationJacobian::Zero();
        H.template block<3, 3>(ROT_X, 0).setIdentity();
        H.template block<3, 3>(BEARING_YAW, 6).noalias() =
            fcs::L3::xyz2ypd_jacobian(predicted.p()) * predicted.R();
        return H;
    }

    [[nodiscard]] CovXi Q(double dt) const noexcept {
        CovXi Q = CovXi::Zero();
        if (dt <= Scalar(0)) {
            return Q;
        }

        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;

        const Mat3 I3 = Mat3::Identity();

        const Scalar dt2 = dt * dt;
        const Scalar dt3 = dt2 * dt;
        const Scalar dt4 = dt2 * dt2;

        const Scalar omega_var = params.sigma_inert_omega * params.sigma_inert_omega; // (rad/s)^2
        const Scalar accel_var = params.sigma_inert_accel * params.sigma_inert_accel; // (m/s^2)^2

        // Xi = [dtheta, dv, dp]

        // dtheta_noise = omega_noise * dt
        Q.template block<3, 3>(0, 0) = omega_var * dt2 * I3; // rad^2

        // dv_noise = accel_noise * dt
        Q.template block<3, 3>(3, 3) = accel_var * dt2 * I3; // (m/s)^2

        // dp_noise = 0.5 * accel_noise * dt^2
        Q.template block<3, 3>(6, 6) = Scalar(0.25) * accel_var * dt4 * I3; // m^2

        // Cov(dv, dp)
        Q.template block<3, 3>(3, 6) = Scalar(0.5) * accel_var * dt3 * I3; // m^2/s

        Q.template block<3, 3>(6, 3) = Q.template block<3, 3>(3, 6).transpose();

        return Q;
    }

    [[nodiscard]] CovInnovation R(const PoseMeasurement& z) const noexcept {
        using std::abs;
        using std::cos;
        using std::sin;
        using std::sqrt;

        Eigen::Matrix<Scalar, NZ, 1> diag;
        diag.setZero();

        const Eigen::Vector3d ypd = fcs::L3::xyz2ypd(z.p_world_body);

        const Scalar yaw      = ypd.x();
        const Scalar pitch    = ypd.y();
        const Scalar distance = abs(ypd.z());

        // Bearing convention:
        //
        // x = d cos(pitch) cos(yaw)
        // y = d cos(pitch) sin(yaw)
        // z = -d sin(pitch)
        //
        // depth: forward component
        // planar_offset: off-axis component
        const Scalar cp = cos(pitch);
        const Scalar sp = sin(pitch);
        const Scalar cy = cos(yaw);
        const Scalar sy = sin(yaw);

        const Scalar depth = abs(distance * cp * cy);

        const Scalar lateral       = distance * cp * sy;
        const Scalar vertical      = -distance * sp;
        const Scalar planar_offset = sqrt(lateral * lateral + vertical * vertical);

        const Scalar sigma_distance = params.sigma_distance_min + params.k_distance_depth * depth
                                    + params.k_distance_planar * planar_offset;

        diag[ROT_X] = params.sigma_rot_x * params.sigma_rot_x;

        diag[ROT_Y] = params.sigma_rot_y * params.sigma_rot_y;

        diag[ROT_Z] = params.sigma_rot_z * params.sigma_rot_z;

        diag[BEARING_YAW] = params.sigma_r_bearing_yaw * params.sigma_r_bearing_yaw;

        diag[BEARING_PITCH] = params.sigma_r_bearing_pitch * params.sigma_r_bearing_pitch;

        diag[BEARING_DISTANCE] = sigma_distance * sigma_distance;

        // Inflate rotation noise when near the C₈ branch boundary.
        // (branch_confidence small → symmetry boundary is near).
        if (z.branch_confidence < std::numbers::pi_v<double> / 12.0) { // < 15°
            // Linearly scale noise from baseline (confidence=15°) → 10× (confidence=0)
            const double factor =
                1.0
                + 9.0
                      * std::clamp(
                          1.0 - z.branch_confidence / (std::numbers::pi_v<double> / 12.0), 0.0,
                          1.0);
            diag[ROT_X] *= factor;
            diag[ROT_Y] *= factor;
            diag[ROT_Z] *= factor;
        }

        CovInnovation R = CovInnovation::Zero();
        R.diagonal()    = diag;
        return R;
    }
};

} // namespace fcs::L3::ldm
