#pragma once

#include "L4_planning/config.hpp"
#include "L4_planning/control_intent.hpp"
#include "L5_weapon/config.hpp"
#include "L5_weapon/fire_control.hpp"
#include "dual_small_mpc_solver.hpp"
#include <cstdint>
#include <expected>
#include <string>

namespace fcs::L5 {

/// TinyMPC-based trajectory optimizer for the L5 weapon layer.
///
/// Consumes a pre-built reference trajectory from L4, solves independent
/// yaw/pitch MPC problems via DualSmallMpcSolver, and emits the center-horizon
/// command with an L5 fire gate.
class TinyMpcTrajectoryOptimizer {
public:
    explicit TinyMpcTrajectoryOptimizer(
        const WeaponControllerConfig& config,
        const L4::ReferenceTrajectoryConfig& trajectory_cfg) noexcept;
    ~TinyMpcTrajectoryOptimizer() noexcept = default;

    TinyMpcTrajectoryOptimizer(const TinyMpcTrajectoryOptimizer&)            = delete;
    TinyMpcTrajectoryOptimizer& operator=(const TinyMpcTrajectoryOptimizer&) = delete;
    TinyMpcTrajectoryOptimizer(TinyMpcTrajectoryOptimizer&&)                 = delete;
    TinyMpcTrajectoryOptimizer& operator=(TinyMpcTrajectoryOptimizer&&)      = delete;

    [[nodiscard]] bool enabled() const noexcept { return config_.enabled; }

    /// Build a pass-through command from a Shot intent (no MPC).
    [[nodiscard]] WeaponCommand
        passthrough(const L4::ShotCommand& shot, uint64_t command_timestamp_ns) const noexcept;

    /// Run MPC optimization on a Track intent's reference trajectory.
    [[nodiscard]] std::expected<WeaponCommand, std::string>
        optimize(const L4::TrackCommand& track, uint64_t command_timestamp_ns) noexcept;

private:
    WeaponControllerConfig config_;
    L4::ReferenceTrajectoryConfig trajectory_cfg_;
    DualSmallMpcSolver batched_solver_;
    bool ready_{false};

    void initialize_solvers() noexcept;
};

} // namespace fcs::L5
