#pragma once

#include <Eigen/Core>
#include <cmath>

namespace fcs::L3 {

/// State space indices for the cascade acceleration EKF (8-dim)
enum AccelState : uint8_t {
    VX_A,      // Linear velocity X
    VY_A,      // Linear velocity Y
    VZ_A,      // Linear velocity Z
    AX,        // Linear acceleration X
    AY,        // Linear acceleration Y
    AZ,        // Linear acceleration Z
    VYAW_A,    // Angular velocity
    ALPHA_YAW, // Angular acceleration
    ACCEL_STATE_MAX,
};

/// Measurement indices for the cascade acceleration EKF (4-dim)
enum AccelMeasure : uint8_t {
    AM_VX,
    AM_VY,
    AM_VZ,
    AM_VYAW,
    ACCEL_MEASURE_MAX,
};

/// Cascade acceleration EKF motion model.
///
/// Takes velocity estimates from the main robot EKF as measurements and
/// estimates smoothed linear/angular acceleration states.
///
/// State: [vx, vy, vz, ax, ay, az, v_yaw, alpha_yaw]
/// Measurement: [vx, vy, vz, v_yaw]
/// Model: Constant-acceleration (CA) — velocity integrates acceleration,
///        acceleration is random walk.
struct AccelEkfMotionModel {
    using Scalar                    = double;
    static constexpr int NX         = ACCEL_STATE_MAX;
    static constexpr int NZ         = ACCEL_MEASURE_MAX;
    static constexpr int ARMORS_NUM = 1; // unused, required by EkfTargetInfo

    using VecX  = Eigen::Matrix<Scalar, NX, 1>;
    using VecZ  = Eigen::Matrix<Scalar, NZ, 1>;
    using MatXX = Eigen::Matrix<Scalar, NX, NX>;
    using MatZZ = Eigen::Matrix<Scalar, NZ, NZ>;

    static constexpr int kVyawIndex    = VYAW_A;
    static constexpr bool kHasLogRadii = false;

    struct Params {
        // Process noise: how much acceleration can change per second²
        Scalar sigma_jerk_xy  = 14.5; // XY jerk noise (acceleration random walk)
        Scalar sigma_jerk_z   = 0.5;  // Z jerk noise
        Scalar sigma_jerk_yaw = 10.0; // Angular jerk noise

        // Measurement noise: variance of velocity estimates from main EKF
        Scalar meas_v_xy_var = 0.05; // XY velocity variance
        Scalar meas_v_z_var  = 0.1;  // Z velocity variance
        Scalar meas_vyaw_var = 0.1;  // Angular velocity variance
    };
    Params params{};

    [[nodiscard]] static int clamp_armor_id(int id) noexcept { return 0; }

    template <typename T>
    static void predict_state(const T* x, const T& dt, T* xp) noexcept {
        // Constant-acceleration model: v += a*dt, a unchanged
        xp[VX_A]      = x[VX_A] + x[AX] * dt;
        xp[VY_A]      = x[VY_A] + x[AY] * dt;
        xp[VZ_A]      = x[VZ_A] + x[AZ] * dt;
        xp[AX]        = x[AX];
        xp[AY]        = x[AY];
        xp[AZ]        = x[AZ];
        xp[VYAW_A]    = x[VYAW_A] + x[ALPHA_YAW] * dt;
        xp[ALPHA_YAW] = x[ALPHA_YAW];
    }

    template <typename T>
    static void measure_state(const T* x, int /*armor_id*/, T* z) noexcept {
        // Direct observation of velocity states
        z[AM_VX]   = x[VX_A];
        z[AM_VY]   = x[VY_A];
        z[AM_VZ]   = x[VZ_A];
        z[AM_VYAW] = x[VYAW_A];
    }

    [[nodiscard]] VecX f(const VecX& x, double dt) const noexcept {
        VecX xp = VecX::Zero();
        predict_state<double>(x.data(), dt, xp.data());
        return xp;
    }

    [[nodiscard]] VecZ h(const VecX& x, int armor_id = 0) const noexcept {
        VecZ z = VecZ::Zero();
        measure_state<double>(x.data(), armor_id, z.data());
        return z;
    }

    [[nodiscard]] MatXX Q(double dt) const noexcept {
        MatXX Q = MatXX::Zero();
        if (dt <= 0.0) {
            return Q;
        }

        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;

        const double sigma_xy2  = params.sigma_jerk_xy * params.sigma_jerk_xy;
        const double sigma_z2   = params.sigma_jerk_z * params.sigma_jerk_z;
        const double sigma_yaw2 = params.sigma_jerk_yaw * params.sigma_jerk_yaw;

        // fill_cv: for a [position, velocity] pair driven by white noise acceleration,
        // Q block is: [[dt4/4, dt3/2], [dt3/2, dt2]] * sigma^2
        // Here "position" = velocity, "velocity" = acceleration, driven by jerk noise.
        auto fill_cv = [&](int v_idx, int a_idx, double q) {
            Q(v_idx, v_idx) = q * dt4 / 4.0;
            Q(v_idx, a_idx) = q * dt3 / 2.0;
            Q(a_idx, v_idx) = q * dt3 / 2.0;
            Q(a_idx, a_idx) = q * dt2;
        };

        fill_cv(VX_A, AX, sigma_xy2);
        fill_cv(VY_A, AY, sigma_xy2);
        fill_cv(VZ_A, AZ, sigma_z2);
        fill_cv(VYAW_A, ALPHA_YAW, sigma_yaw2);

        return Q;
    }

    [[nodiscard]] MatZZ R(const VecZ& /*z*/) const noexcept {
        (void)params; // suppress unused member warning in case R becomes static
        MatZZ R             = MatZZ::Zero();
        R(AM_VX, AM_VX)     = params.meas_v_xy_var;
        R(AM_VY, AM_VY)     = params.meas_v_xy_var;
        R(AM_VZ, AM_VZ)     = params.meas_v_z_var;
        R(AM_VYAW, AM_VYAW) = params.meas_vyaw_var;
        return R;
    }

    [[nodiscard]] Eigen::Matrix<Scalar, NZ, 1> R_diag(const VecZ& z) const noexcept {
        Eigen::Matrix<Scalar, NZ, 1> R_diag;
        R_diag << params.meas_v_xy_var, params.meas_v_xy_var, params.meas_v_z_var,
            params.meas_vyaw_var;
        return R_diag;
    }
};

} // namespace fcs::L3
