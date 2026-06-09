#pragma once

#include "L5_weapon/fire_decision.hpp"
namespace fcs::L5 {

struct MpcAxisConfig {
    double q_pos{9e6};
    double q_vel{0.0};
    double r{1.0};
    double max_acc{50.0};
};

struct MpcSolverConfig {
    int max_iterations{10};
    double abs_tol{1e-3};
    double rho{1.0};
    MpcAxisConfig yaw{9e6, 0.0, 1.0, 50.0};
    MpcAxisConfig pitch{9e6, 0.0, 1.0, 100.0};
};

// ============================================================================
// Enhanced Weapon Controller Configuration (L5 Layer)
// ============================================================================

struct WeaponControllerConfig {
    bool enabled{true};
    bool enable_debug{true};

    double pitch_min{-0.69813169};
    double pitch_max{0.69813169};

    /// Maximum age of L4 reference trajectory before L5 rejects it (seconds).
    /// If the reference trajectory is older than this threshold, L5 falls back
    /// to passthrough instead of solving on stale data.
    double reference_age_threshold_s{0.02};

    MpcSolverConfig mpc{};
};

// ============================================================================
// L5 Weapon Full Config
// ============================================================================

struct L5Config {
    // Fire decision configuration shared by L4 aiming and L5 fire gating.
    FireDecisionConfig fire_decision{};
    WeaponControllerConfig mpc_weapon{};
};

} // namespace fcs::L5
