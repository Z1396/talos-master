#include "L4_planning/trajectory_builder.hpp"
#include "core/math/normalize.hpp"

#include <fmt/core.h>
#include <vector>

namespace fcs::L4 {
namespace {

using fcs::core::math::normalize_angle;

using core::trajectory::ReferenceTrajectory;
using StateMatrix = ReferenceTrajectory::StateMatrix;

struct ReferenceSample {
    double yaw{0.0};
    double pitch{0.0};
    double distance{0.0};
    double time_of_flight{0.0};
    int selected_armor_id{0};
};

[[nodiscard]] double
    finite_difference(const std::vector<double>& samples, int index, double dt) noexcept {
    const int size = static_cast<int>(samples.size());
    if (size <= 1) {
        return 0.0;
    }
    if (index <= 0) {
        return (samples[1] - samples[0]) / dt;
    }
    if (index >= size - 1) {
        return (samples[size - 1] - samples[size - 2]) / dt;
    }
    return (samples[index + 1] - samples[index - 1]) / (2.0 * dt);
}

[[nodiscard]] double
    finite_difference_angle(const std::vector<double>& samples, int index, double dt) noexcept {
    const int size = static_cast<int>(samples.size());
    if (size <= 1) {
        return 0.0;
    }
    if (index <= 0) {
        return normalize_angle(samples[1] - samples[0]) / dt;
    }
    if (index >= size - 1) {
        return normalize_angle(samples[size - 1] - samples[size - 2]) / dt;
    }
    return normalize_angle(samples[index + 1] - samples[index - 1]) / (2.0 * dt);
}

template <typename TargetState>
[[nodiscard]] std::expected<ReferenceSample, std::string> build_reference_sample(
    const Aimer& aimer, const TargetState& state, ArmorAimContext context,
    uint64_t state_timestamp_ns, uint64_t current_ns, double sample_offset_s,
    const Aimer::GimbalTransform& gimbal, const Aimer::MuzzleTransform& muzzle,
    const core::trajectory::solver::TrajectorySolver& solver, double bullet_speed) noexcept {
    const auto prediction = aimer.aim(
        state, context, gimbal, muzzle, state_timestamp_ns, current_ns, sample_offset_s,
        bullet_speed, solver);
    if (!prediction) {
        return std::unexpected(prediction.error());
    }

    return ReferenceSample{
        .yaw               = prediction->aim_yaw,
        .pitch             = prediction->aim_pitch,
        .distance          = prediction->distance,
        .time_of_flight    = prediction->flying_time,
        .selected_armor_id = prediction->selected_armor_id,
    };
}

[[nodiscard]] ArmorAimPhase fire_aim_phase(ArmorAimPhase phase) noexcept {
    return phase == ArmorAimPhase::WholeCarCenter ? ArmorAimPhase::WholeCarArmor : phase;
}

[[nodiscard]] std::expected<ReferenceTrajectory, std::string> build_reference_trajectory_with_phase(
    const PlannerSeed& seed, const Aimer& aimer, const ReferenceTrajectoryConfig& horizon_cfg,
    const TrajectoryBuildContext& ctx, ArmorAimPhase phase) noexcept {
    const int horizon      = horizon_cfg.horizon_ahead + horizon_cfg.horizon_back + 1;
    const int horizon_back = horizon_cfg.horizon_back;
    const double dt        = horizon_cfg.dt;

    ReferenceTrajectory trajectory{
        .state           = StateMatrix(4, horizon),
        .distances       = std::vector<double>(horizon, 0.0),
        .time_of_flights = std::vector<double>(horizon, 0.0),
    };

    std::vector<double> yaw_samples(horizon, 0.0);
    std::vector<double> pitch_samples(horizon, 0.0);

    ArmorAimContext context;
    context.target_jumped      = seed.target_jumped;
    context.phase              = phase;
    context.preferred_armor_id = seed.selected_armor_id;

    // Resolve the target variant once before the loop.
    // If the seed has no valid state, fail immediately rather than
    // allocating "empty planner seed" strings inside the hot loop.
    const auto build_sample = [&](int k) -> std::expected<ReferenceSample, std::string> {
        const double sample_offset = (static_cast<double>(k) - horizon_back) * dt;
        if (const auto* robot = seed.robot_state()) {
            return build_reference_sample(
                aimer, *robot, context, seed.state_timestamp_ns, ctx.current_ns, sample_offset,
                ctx.gimbal, ctx.muzzle, *ctx.trajectory_solver, ctx.bullet_speed);
        }
        if (const auto* outpost = seed.outpost_state()) {
            return build_reference_sample(
                aimer, *outpost, context, seed.state_timestamp_ns, ctx.current_ns, sample_offset,
                ctx.gimbal, ctx.muzzle, *ctx.trajectory_solver, ctx.bullet_speed);
        }
        return std::unexpected("empty planner seed");
    };

    for (int k = 0; k < horizon; ++k) {
        auto sample = build_sample(k);
        if (!sample) {
            return std::unexpected(sample.error());
        }

        context.preferred_armor_id = sample->selected_armor_id;

        yaw_samples[k]                = sample->yaw;
        pitch_samples[k]              = sample->pitch;
        trajectory.distances[k]       = sample->distance;
        trajectory.time_of_flights[k] = sample->time_of_flight;
    }

    trajectory.yaw_origin = yaw_samples[horizon_back];

    for (int k = 0; k < horizon; ++k) {
        trajectory.state(0, k) = normalize_angle(yaw_samples[k] - trajectory.yaw_origin);
        trajectory.state(1, k) = finite_difference_angle(yaw_samples, k, dt);
        trajectory.state(2, k) = pitch_samples[k];
        trajectory.state(3, k) = finite_difference(pitch_samples, k, dt);
    }

    return trajectory;
}

} // namespace

[[nodiscard]] std::expected<ReferenceTrajectory, std::string> build_reference_trajectory(
    const PlannerSeed& seed, const Aimer& aimer, const ReferenceTrajectoryConfig& horizon_cfg,
    const TrajectoryBuildContext& ctx) noexcept {
    return build_reference_trajectory_with_phase(seed, aimer, horizon_cfg, ctx, seed.aim_phase);
}

[[nodiscard]] std::expected<ReferenceTrajectory, std::string> build_fire_reference_trajectory(
    const PlannerSeed& seed, const Aimer& aimer, const ReferenceTrajectoryConfig& horizon_cfg,
    const TrajectoryBuildContext& ctx) noexcept {
    return build_reference_trajectory_with_phase(
        seed, aimer, horizon_cfg, ctx, fire_aim_phase(seed.aim_phase));
}

[[nodiscard]] std::expected<ReferenceTrajectory, std::string> build_ldm_reference_trajectory(
    const L3::ldm::LdmState& state, const Aimer& aimer,
    const ReferenceTrajectoryConfig& horizon_cfg, const TrajectoryBuildContext& ctx) noexcept {

    if (!state.is_tracking()) {
        return std::unexpected("LDM target is not tracking");
    }

    const int horizon      = horizon_cfg.horizon_ahead + horizon_cfg.horizon_back + 1;
    const int horizon_back = horizon_cfg.horizon_back;
    const double dt        = horizon_cfg.dt;

    ReferenceTrajectory trajectory{
        .state           = StateMatrix(4, horizon),
        .distances       = std::vector<double>(horizon, 0.0),
        .time_of_flights = std::vector<double>(horizon, 0.0),
    };

    std::vector<double> yaw_samples(horizon, 0.0);
    std::vector<double> pitch_samples(horizon, 0.0);

    for (int k = 0; k < horizon; ++k) {
        const double sample_offset = (static_cast<double>(k) - horizon_back) * dt;

        const auto prediction = aimer.aim(
            state, ctx.gimbal, ctx.muzzle, ctx.current_ns, sample_offset, ctx.bullet_speed,
            *ctx.trajectory_solver);
        if (!prediction) {
            return std::unexpected(
                fmt::format("LDM reference trajectory step {} failed: {}", k, prediction.error()));
        }

        yaw_samples[k]                = prediction->aim_yaw;
        pitch_samples[k]              = prediction->aim_pitch;
        trajectory.distances[k]       = prediction->distance;
        trajectory.time_of_flights[k] = prediction->flying_time;
    }

    trajectory.yaw_origin = yaw_samples[horizon_back];

    for (int k = 0; k < horizon; ++k) {
        trajectory.state(0, k) = normalize_angle(yaw_samples[k] - trajectory.yaw_origin);
        trajectory.state(1, k) = finite_difference_angle(yaw_samples, k, dt);
        trajectory.state(2, k) = pitch_samples[k];
        trajectory.state(3, k) = finite_difference(pitch_samples, k, dt);
    }

    return trajectory;
}

} // namespace fcs::L4
