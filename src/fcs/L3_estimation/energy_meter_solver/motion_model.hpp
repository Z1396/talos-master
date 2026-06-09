#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace energy_meter {

inline constexpr double FIXED_RUNE_SPEED = std::numbers::pi / 3.0;
inline constexpr int ARMORS_NUM          = 5;

inline int clamp_armor_id(int id) noexcept { return std::clamp(id, 0, ARMORS_NUM - 1); }

// ── Shared measurement indices (identical for both models) ──

enum Measure : uint8_t {
    CENTER_X,   // 0
    CENTER_Y,   // 1
    CENTER_Z,   // 2
    RUNE_YAW,   // 3
    ARMOR_ROLL, // 4
    MEASURE_MAX,
};

// Shared distance-dependent measurement noise
struct MeasurementNoiseParams {
    double R_scale{0.1};
    double meas_dist_k{0.28};
};

[[nodiscard]] inline Eigen::Matrix<double, MEASURE_MAX, MEASURE_MAX> measurement_R(
    const Eigen::Matrix<double, MEASURE_MAX, 1>& z, const MeasurementNoiseParams& p) noexcept {
    const double x     = z[CENTER_X];
    const double y     = z[CENTER_Y];
    const double z_val = z[CENTER_Z];

    const double distance = std::sqrt(x * x + y * y + z_val * z_val);
    const double yaw      = std::atan2(y, x);

    const double delta_yaw = [&]() {
        double a = std::fmod(yaw - z[RUNE_YAW] + M_PI, 2.0 * M_PI);
        if (a <= 0.0)
            a += 2.0 * M_PI;
        return a - M_PI;
    }();

    const double K      = p.meas_dist_k;
    const double weight = std::log(K * distance + 1.0);

    Eigen::Matrix<double, MEASURE_MAX, MEASURE_MAX> Rs =
        Eigen::Matrix<double, MEASURE_MAX, MEASURE_MAX>::Zero();
    Rs.diagonal() << p.R_scale * (weight + std::abs(delta_yaw / 5.0 + 0.5)),
        p.R_scale * (weight + std::abs(delta_yaw / 5.0 + 0.5)), p.R_scale * weight,
        (std::log(std::abs(distance) + 1) / 150.0 + 5e-2), 5e-2;
    return Rs;
}

// ============================================================================
// BigRuneModel — sinusoidal rotation: θ' = dir · [(a/ω)(cos(ωτ)-cos(ω(τ+dt))) + b·dt]
// State: [XC, YC, ZC, YAW, THETA, TAU, A, W] (8D)
// ============================================================================

struct BigRuneModel {
    using Scalar            = double;
    static constexpr int NX = 8;
    static constexpr int NZ = MEASURE_MAX;

    enum State : uint8_t {
        XC,    // 0
        YC,    // 1
        ZC,    // 2
        YAW,   // 3
        THETA, // 4
        TAU,   // 5 — accumulated time since activation
        A,     // 6 — amplitude a ∈ [0.780, 1.045]
        W,     // 7 — angular frequency ω ∈ [1.884, 2.000]
    };

    using VecX  = Eigen::Matrix<Scalar, NX, 1>;
    using VecZ  = Eigen::Matrix<Scalar, NZ, 1>;
    using MatXX = Eigen::Matrix<Scalar, NX, NX>;
    using MatZZ = Eigen::Matrix<Scalar, NZ, NZ>;

    struct Params {
        Scalar sigma_xy{2.0};
        Scalar sigma_z{1.5};
        Scalar sigma_yaw{1e-4};
        Scalar sigma_theta{0.2};
        Scalar sigma_a{0.10};
        Scalar sigma_w{0.15};

        Scalar radius{0.7};
        MeasurementNoiseParams meas{};
    };

    Params params{};

    static constexpr double AMPLITUDE_SUM = 2.090;
    static constexpr double A_LOWER       = 0.780;
    static constexpr double A_UPPER       = 1.045;
    static constexpr double W_LOWER       = 1.884;
    static constexpr double W_UPPER       = 2.000;

    template <typename S>
    static void predict_state(const S* x, const S& dt, int dir, S* xp) noexcept {
        using std::cos;

        for (int i = 0; i < NX; ++i) {
            xp[i] = x[i];
        }

        const S a = x[A];
        const S w = x[W];
        const S b = S(AMPLITUDE_SUM) - a;

        const S t_new = x[TAU] + dt;
        xp[TAU]       = t_new;

        const S delta_theta = (a / w) * (cos(w * x[TAU]) - cos(w * t_new)) + b * dt;
        xp[THETA]           = x[THETA] + S(static_cast<double>(dir)) * delta_theta;
    }

    template <typename S>
    static void measure_state(const S* x, int armor_id, S* z) noexcept {
        const int id         = clamp_armor_id(armor_id);
        const S angle_offset = S(2.0 * M_PI / 5.0 * static_cast<double>(id));

        z[CENTER_X]   = x[XC];
        z[CENTER_Y]   = x[YC];
        z[CENTER_Z]   = x[ZC];
        z[RUNE_YAW]   = x[YAW];
        z[ARMOR_ROLL] = x[THETA] + angle_offset;
    }

    [[nodiscard]] VecZ h(const VecX& x, int armor_id) const noexcept {
        VecZ z = VecZ::Zero();
        measure_state<double>(x.data(), armor_id, z.data());
        return z;
    }

    [[nodiscard]] MatXX Q(double dt) const noexcept {
        MatXX q = MatXX::Zero();
        if (dt <= 0.0) {
            return q;
        }

        q(XC, XC)       = params.sigma_xy * params.sigma_xy * dt;
        q(YC, YC)       = params.sigma_xy * params.sigma_xy * dt;
        q(ZC, ZC)       = params.sigma_z * params.sigma_z * dt;
        q(YAW, YAW)     = params.sigma_yaw * params.sigma_yaw * dt;
        q(THETA, THETA) = params.sigma_theta * params.sigma_theta * dt;
        q(TAU, TAU)     = 0.0;
        q(A, A)         = params.sigma_a * params.sigma_a * dt;
        q(W, W)         = params.sigma_w * params.sigma_w * dt;

        return q;
    }

    [[nodiscard]] MatZZ R(const VecZ& z) const noexcept { return measurement_R(z, params.meas); }

    [[nodiscard]] Eigen::Matrix<Scalar, NZ, 1> R_diag(const VecZ& z) const {
        return R(z).diagonal();
    }
};

// ============================================================================
// SmallRuneModel — constant speed rotation: θ' = dir · ω_fixed · dt
// State: [XC, YC, ZC, YAW, THETA] (5D)
// ============================================================================

struct SmallRuneModel {
    using Scalar            = double;
    static constexpr int NX = 5;
    static constexpr int NZ = MEASURE_MAX;

    enum State : uint8_t {
        XC,    // 0
        YC,    // 1
        ZC,    // 2
        YAW,   // 3
        THETA, // 4
    };

    using VecX  = Eigen::Matrix<Scalar, NX, 1>;
    using VecZ  = Eigen::Matrix<Scalar, NZ, 1>;
    using MatXX = Eigen::Matrix<Scalar, NX, NX>;
    using MatZZ = Eigen::Matrix<Scalar, NZ, NZ>;

    struct Params {
        // Process noise — very low for constant-speed rune (auto_buff v1=0.001)
        Scalar sigma_xy{0.01};
        Scalar sigma_z{0.01};
        Scalar sigma_yaw{1e-4};
        Scalar sigma_theta{0.001};

        Scalar radius{0.7};
        MeasurementNoiseParams meas{};
    };

    Params params{};

    template <typename S>
    static void predict_state(const S* x, const S& dt, int dir, S* xp) noexcept {
        for (int i = 0; i < NX; ++i) {
            xp[i] = x[i];
        }
        xp[THETA] = x[THETA] + S(static_cast<double>(dir)) * S(FIXED_RUNE_SPEED) * dt;
    }

    template <typename S>
    static void measure_state(const S* x, int armor_id, S* z) noexcept {
        const int id         = clamp_armor_id(armor_id);
        const S angle_offset = S(2.0 * M_PI / 5.0 * static_cast<double>(id));

        z[CENTER_X]   = x[XC];
        z[CENTER_Y]   = x[YC];
        z[CENTER_Z]   = x[ZC];
        z[RUNE_YAW]   = x[YAW];
        z[ARMOR_ROLL] = x[THETA] + angle_offset;
    }

    [[nodiscard]] VecZ h(const VecX& x, int armor_id) const noexcept {
        VecZ z = VecZ::Zero();
        measure_state<double>(x.data(), armor_id, z.data());
        return z;
    }

    [[nodiscard]] MatXX Q(double dt) const noexcept {
        MatXX q = MatXX::Zero();
        if (dt <= 0.0) {
            return q;
        }

        q(XC, XC)       = params.sigma_xy * params.sigma_xy * dt;
        q(YC, YC)       = params.sigma_xy * params.sigma_xy * dt;
        q(ZC, ZC)       = params.sigma_z * params.sigma_z * dt;
        q(YAW, YAW)     = params.sigma_yaw * params.sigma_yaw * dt;
        q(THETA, THETA) = params.sigma_theta * params.sigma_theta * dt;

        return q;
    }

    [[nodiscard]] MatZZ R(const VecZ& z) const noexcept { return measurement_R(z, params.meas); }

    [[nodiscard]] Eigen::Matrix<Scalar, NZ, 1> R_diag(const VecZ& z) const {
        return R(z).diagonal();
    }
};

// b = 2.090 - a (constant sum constraint from rules)
inline double to_b(double a) { return BigRuneModel::AMPLITUDE_SUM - a; }

} // namespace energy_meter
