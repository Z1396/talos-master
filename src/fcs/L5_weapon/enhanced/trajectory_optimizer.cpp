#include "L5_weapon/enhanced/trajectory_optimizer.hpp"

#include "L5_weapon/fire_decision.hpp"
#include "core/math/normalize.hpp"

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <utility>

// =============================================================================
// Optimized MPC trajectory optimizer for ARM Cortex-A55.
//
// Uses DualSmallMpcSolver (float, fixed-size Eigen) instead of vendored TinyMPC
// (double, dynamic Eigen). This eliminates:
//   1. Software-emulated double-precision FP (~20x overhead on A55)
//   2. Dynamic matrix allocation overhead per ADMM iteration
//   3. The 5-allocation RAII wrapper needed to paper over TinyMPC's memory leaks
//
// The ADMM algorithm is identical — same Riccati backward/forward pass,
// same slack/dual update, same termination check. Just without the overhead.
// =============================================================================

namespace fcs::L5 {
namespace {

using fcs::core::math::normalize_angle;

using ReferenceTrajectory = core::trajectory::ReferenceTrajectory;

[[nodiscard]] int quantize_reference_age_steps(double reference_age_s, double dt_s) noexcept {
    if (!(reference_age_s > 0.0) || !(dt_s > 0.0)) {
        return 0;
    }
    return std::max(0, static_cast<int>(std::lround(reference_age_s / dt_s)));
}

template <typename Solver>
struct AxisSolverView {
    const Solver* solver{nullptr};
    int axis{0};
    int horizon{0};

    [[nodiscard]] float state(int dim, int k) const noexcept {
        const int clamped_k = std::clamp(k, 0, horizon - 1);
        return solver->state(axis, dim, clamped_k);
    }

    [[nodiscard]] float input(int k) const noexcept {
        const int clamped_k = std::clamp(k, 0, horizon - 2);
        return solver->input(axis, clamped_k);
    }
};

[[nodiscard]] Eigen::Vector3d
    spherical_to_cartesian(double distance, double yaw, double pitch) noexcept {
    return Eigen::Vector3d{
        distance * std::cos(pitch) * std::cos(yaw),
        distance * std::cos(pitch) * std::sin(yaw),
        distance * std::sin(pitch),
    };
}

template <typename YawSolver, typename PitchSolver>
[[nodiscard]] WeaponVisualizationDebugData build_debug_data(
    int center_index, const ReferenceTrajectory& trajectory, int reference_start_index,
    int reference_horizon, const YawSolver& yaw_solver, const PitchSolver& pitch_solver) {
    WeaponVisualizationDebugData debug;
    debug.center_index    = center_index;
    debug.lookahead_index = debug.center_index;
    debug.reference_plan.reserve(reference_horizon);
    debug.optimized_plan.reserve(reference_horizon);

    for (int k = 0; k < reference_horizon; ++k) {
        const int ref_k = reference_start_index + k;
        debug.reference_plan.push_back(
            TrajectoryPlanSample{
                .yaw      = normalize_angle(trajectory.state(0, ref_k) + trajectory.yaw_origin),
                .pitch    = trajectory.state(2, ref_k),
                .distance = trajectory.distances[ref_k],
                .tof      = trajectory.time_of_flights[ref_k],
            });

        const double optimized_yaw =
            normalize_angle(static_cast<double>(yaw_solver.state(0, k)) + trajectory.yaw_origin);
        const double optimized_pitch =
            normalize_angle(static_cast<double>(pitch_solver.state(0, k)));
        debug.optimized_plan.push_back(
            TrajectoryPlanSample{
                .yaw      = optimized_yaw,
                .pitch    = optimized_pitch,
                .distance = trajectory.distances[ref_k],
                .tof      = trajectory.time_of_flights[ref_k],
            });
    }

    return debug;
}

} // namespace

TinyMpcTrajectoryOptimizer::TinyMpcTrajectoryOptimizer(
    const WeaponControllerConfig& config,
    const L4::ReferenceTrajectoryConfig& trajectory_cfg) noexcept
    : config_(config)
    , trajectory_cfg_(trajectory_cfg) {
    if (config_.enabled) {
        initialize_solvers();
    }
}

WeaponCommand TinyMpcTrajectoryOptimizer::passthrough(
    const L4::ShotCommand& shot, uint64_t command_timestamp_ns) const noexcept {
    WeaponCommand cmd;
    cmd.timestamp_ns      = command_timestamp_ns;
    cmd.plan_timestamp_ns = shot.timestamp_ns;
    cmd.plan_yaw          = shot.yaw;
    cmd.plan_pitch        = shot.pitch;
    cmd.plan_distance     = shot.distance;
    cmd.yaw               = shot.yaw;
    cmd.pitch             = shot.pitch;
    cmd.distance          = shot.distance;
    // Shot mode: no velocity/acceleration feedforward.
    cmd.yaw_vel     = 0.0;
    cmd.pitch_vel   = 0.0;
    cmd.yaw_accel   = 0.0;
    cmd.pitch_accel = 0.0;
    return cmd;
}

std::expected<WeaponCommand, std::string> TinyMpcTrajectoryOptimizer::optimize(
    const L4::TrackCommand& track, uint64_t command_timestamp_ns) noexcept {
    if (!config_.enabled) {
        return std::unexpected("mpc_weapon disabled");
    }
    if (!ready_) {
        return std::unexpected("mpc solvers are not ready");
    }

    // Reject stale reference trajectories to avoid solving on outdated data.
    const double reference_age_s =
        static_cast<double>(
            static_cast<int64_t>(command_timestamp_ns) - static_cast<int64_t>(track.timestamp_ns))
        * 1e-9;
    if (reference_age_s > config_.reference_age_threshold_s) {
        return std::unexpected(
            "reference trajectory is stale (" + std::to_string(reference_age_s) + "s > "
            + std::to_string(config_.reference_age_threshold_s) + "s)");
    }

    const ReferenceTrajectory& reference = track.control_trajectory;

    // Age the reference window forward so the sampled "current" command stays aligned with now.
    const int solver_horizon    = batched_solver_.horizon();
    const int reference_horizon = reference.horizon();
    const int reference_shift_steps =
        quantize_reference_age_steps(std::max(reference_age_s, 0.0), trajectory_cfg_.dt);
    const int reference_start_index =
        std::clamp(reference_shift_steps, 0, std::max(reference_horizon - 1, 0));
    const int available_reference_horizon = reference_horizon - reference_start_index;
    const int effective_horizon           = std::min(available_reference_horizon, solver_horizon);
    if (effective_horizon < 2) {
        return std::unexpected("reference trajectory horizon is too short for MPC");
    }

    batched_solver_.set_x0(
        reference.state(0, reference_start_index), reference.state(1, reference_start_index),
        reference.state(2, reference_start_index), reference.state(3, reference_start_index));
    for (int i = 0; i < effective_horizon; ++i) {
        const int ref_i = reference_start_index + i;
        batched_solver_.set_ref_col(
            i, static_cast<float>(reference.state(0, ref_i)),
            static_cast<float>(reference.state(1, ref_i)),
            static_cast<float>(reference.state(2, ref_i)),
            static_cast<float>(reference.state(3, ref_i)));
    }
    if (effective_horizon < solver_horizon) {
        const int last_ref = reference_start_index + effective_horizon - 1;
        for (int i = effective_horizon; i < solver_horizon; ++i) {
            batched_solver_.set_ref_col(
                i, static_cast<float>(reference.state(0, last_ref)),
                static_cast<float>(reference.state(1, last_ref)),
                static_cast<float>(reference.state(2, last_ref)),
                static_cast<float>(reference.state(3, last_ref)));
        }
    }
    batched_solver_.solve();

    const AxisSolverView yaw_solver{&batched_solver_, DualSmallMpcSolver::kYawAxis, solver_horizon};
    const AxisSolverView pitch_solver{
        &batched_solver_, DualSmallMpcSolver::kPitchAxis, solver_horizon};

    const int center_index  = std::clamp(trajectory_cfg_.horizon_back, 0, effective_horizon - 1);
    const int control_index = std::clamp(center_index, 0, effective_horizon - 2);
    const int reference_center_index = reference_start_index + center_index;

    // Extract the aim point from the trajectory's center step (originally L4's aim).
    const double plan_yaw      = normalize_angle(reference.yaw_origin);
    const double plan_pitch    = reference.state(2, reference_start_index + center_index);
    const double plan_distance = reference.distances[reference_center_index];

    WeaponCommand cmd;
    cmd.timestamp_ns      = command_timestamp_ns;
    cmd.plan_timestamp_ns = track.timestamp_ns;
    cmd.plan_yaw          = plan_yaw;
    cmd.plan_pitch        = plan_pitch;
    cmd.plan_distance     = plan_distance;
    cmd.yaw               = normalize_angle(
        static_cast<double>(yaw_solver.state(0, center_index)) + reference.yaw_origin);
    cmd.pitch = std::clamp(
        static_cast<double>(pitch_solver.state(0, center_index)), config_.pitch_min,
        config_.pitch_max);
    cmd.yaw_vel     = static_cast<double>(yaw_solver.state(1, center_index));
    cmd.pitch_vel   = static_cast<double>(pitch_solver.state(1, center_index));
    cmd.yaw_accel   = static_cast<double>(yaw_solver.input(control_index));
    cmd.pitch_accel = static_cast<double>(pitch_solver.input(control_index));
    cmd.distance    = reference.distances[reference_center_index];
    cmd.tof         = reference.time_of_flights[reference_center_index];
    if (config_.enable_debug) {
        cmd.viz_debug = build_debug_data(
            center_index, reference, reference_start_index, effective_horizon, yaw_solver,
            pitch_solver);
    }

    const auto finite = [](double value) noexcept { return std::isfinite(value); };
    if (!finite(cmd.yaw) || !finite(cmd.pitch) || !finite(cmd.yaw_vel) || !finite(cmd.pitch_vel)
        || !finite(cmd.yaw_accel) || !finite(cmd.pitch_accel) || !finite(cmd.distance)
        || !finite(cmd.tof)) {
        return std::unexpected("MPC output contains non-finite values");
    }

    return cmd;
}

void TinyMpcTrajectoryOptimizer::initialize_solvers() noexcept {
    const int requested_horizon = trajectory_cfg_.horizon_ahead + trajectory_cfg_.horizon_back + 1;
    const int horizon           = std::min(requested_horizon, DualSmallMpcSolver::kMaxHorizon);
    const float dt              = static_cast<float>(trajectory_cfg_.dt);
    const float rho             = static_cast<float>(config_.mpc.rho);

    if (requested_horizon > horizon) {
        SPDLOG_WARN(
            "L5 MPC horizon {} exceeds fixed-capacity limit {}; clipping to {}. "
            "Increase the solver limit or reduce L4 horizon if this is unintended.",
            requested_horizon, DualSmallMpcSolver::kMaxHorizon, horizon);
    }

    DualSmallMpcSolver::AxisConfig yaw_cfg;
    yaw_cfg.q_pos   = static_cast<float>(config_.mpc.yaw.q_pos);
    yaw_cfg.q_vel   = static_cast<float>(config_.mpc.yaw.q_vel);
    yaw_cfg.r       = static_cast<float>(config_.mpc.yaw.r);
    yaw_cfg.max_acc = static_cast<float>(config_.mpc.yaw.max_acc);

    DualSmallMpcSolver::AxisConfig pitch_cfg;
    pitch_cfg.q_pos              = static_cast<float>(config_.mpc.pitch.q_pos);
    pitch_cfg.q_vel              = static_cast<float>(config_.mpc.pitch.q_vel);
    pitch_cfg.r                  = static_cast<float>(config_.mpc.pitch.r);
    pitch_cfg.max_acc            = static_cast<float>(config_.mpc.pitch.max_acc);
    pitch_cfg.enable_state_bound = true;
    pitch_cfg.state_min          = static_cast<float>(config_.pitch_min);
    pitch_cfg.state_max          = static_cast<float>(config_.pitch_max);

    auto solver = DualSmallMpcSolver::create(dt, horizon, rho, yaw_cfg, pitch_cfg);
    if (!solver) {
        ready_ = false;
        return;
    }
    solver->set_settings(static_cast<float>(config_.mpc.abs_tol), config_.mpc.max_iterations);
    batched_solver_ = std::move(*solver);
    ready_          = true;
}

} // namespace fcs::L5
