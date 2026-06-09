#pragma once

#include "L3_estimation/ldm_naive/types.hpp"
#include "L4_planning/aimer/aimer.hpp"
#include "L4_planning/config.hpp"
#include "L4_planning/gimbal_planner/types.hpp"
#include "core/trajectory/reference_trajectory.hpp"
#include "core/trajectory/solver/solver_interfaces.hpp"

#include <expected>
#include <string>

namespace fcs::L4 {

/// Runtime context needed for trajectory building at each control cycle.
struct TrajectoryBuildContext {
    uint64_t current_ns{0};
    Aimer::GimbalTransform gimbal;
    Aimer::MuzzleTransform muzzle;
    const core::trajectory::solver::TrajectorySolver* trajectory_solver{nullptr};
    double bullet_speed{0.0};
};

/// Build a reference trajectory from a planner seed.
///
/// Iterates over the prediction horizon, calling `Aimer::aim()` at each step with
/// appropriate prediction delay offsets. Produces a `ReferenceTrajectory`
/// with the state matrix [yaw, yaw_rate, pitch, pitch_rate] × horizon plus
/// metadata for fire gate computation.
///
/// Pitch clamping is NOT applied here — that is L5's job.
///
/// ## Preconditions
/// - `seed` must contain valid robot or outpost state
/// - `ctx.trajectory_solver` must be non-null
///
/// ## Thread safety
/// NOT thread-safe. Do not call concurrently on shared data.
[[nodiscard]] std::expected<core::trajectory::ReferenceTrajectory, std::string>
    build_reference_trajectory(
        const PlannerSeed& seed, const Aimer& aimer, const ReferenceTrajectoryConfig& horizon_cfg,
        const TrajectoryBuildContext& ctx) noexcept;

/// Build the real armor trajectory used by the L5 fire gate.
///
/// Unlike the control trajectory, this never uses WholeCarCenter's center proxy;
/// in that phase it falls back to real armor aiming so fire is judged against
/// an actual hittable target.
[[nodiscard]] std::expected<core::trajectory::ReferenceTrajectory, std::string>
    build_fire_reference_trajectory(
        const PlannerSeed& seed, const Aimer& aimer, const ReferenceTrajectoryConfig& horizon_cfg,
        const TrajectoryBuildContext& ctx) noexcept;

/// Build a reference trajectory from an LDM target state.
///
/// Uses the LDM constant-velocity prediction model to iterate over the
/// horizon, calling `Aimer::aim(const LdmState&, ...)` at each step.
/// Produces a `ReferenceTrajectory` compatible with L5 weapon control.
///
/// ## Preconditions
/// - `state.is_tracking()` must be true
/// - `ctx.trajectory_solver` must be non-null
///
/// ## Thread safety
/// NOT thread-safe. Do not call concurrently on shared data.
[[nodiscard]] std::expected<core::trajectory::ReferenceTrajectory, std::string>
    build_ldm_reference_trajectory(
        const L3::ldm::LdmState& state, const Aimer& aimer,
        const ReferenceTrajectoryConfig& horizon_cfg, const TrajectoryBuildContext& ctx) noexcept;

} // namespace fcs::L4
