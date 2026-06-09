#include <gtest/gtest.h>

#include "L5_weapon/enhanced/trajectory_optimizer.hpp"
#include "L5_weapon/enhanced/weapon_systems.hpp"

#include <Eigen/Core>

namespace fcs::L5 {
namespace {

using ReferenceTrajectory = core::trajectory::ReferenceTrajectory;

ReferenceTrajectory make_reference_trajectory(int horizon) {
    ReferenceTrajectory trajectory;
    trajectory.state = ReferenceTrajectory::StateMatrix::Zero(4, horizon);
    trajectory.distances.reserve(horizon);
    trajectory.time_of_flights.reserve(horizon);

    for (int i = 0; i < horizon; ++i) {
        trajectory.distances.push_back(static_cast<double>(i));
        trajectory.time_of_flights.push_back(static_cast<double>(i) * 0.01);
    }

    return trajectory;
}

L4::TrackCommand make_track(const ReferenceTrajectory& reference, uint64_t timestamp_ns) {
    return L4::TrackCommand{
        .timestamp_ns       = timestamp_ns,
        .control_trajectory = reference,
        .fire_trajectory    = reference,
    };
}

ReferenceTrajectory make_constant_reference_trajectory(
    int horizon, double yaw, double pitch, double distance, double tof) {
    ReferenceTrajectory trajectory;
    trajectory.state = ReferenceTrajectory::StateMatrix::Zero(4, horizon);
    trajectory.distances.reserve(horizon);
    trajectory.time_of_flights.reserve(horizon);
    trajectory.yaw_origin = yaw;

    for (int i = 0; i < horizon; ++i) {
        trajectory.state(2, i) = pitch;
        trajectory.distances.push_back(distance);
        trajectory.time_of_flights.push_back(tof);
    }

    return trajectory;
}

TEST(TrajectoryOptimizerTest, FreshReferenceKeepsOriginalCenterSample) {
    WeaponControllerConfig config;
    config.reference_age_threshold_s = 1.0;
    config.enable_debug              = true;

    L4::ReferenceTrajectoryConfig trajectory_cfg;
    trajectory_cfg.horizon_back  = 2;
    trajectory_cfg.horizon_ahead = 3;
    trajectory_cfg.dt            = 0.1;

    TinyMpcTrajectoryOptimizer optimizer(config, trajectory_cfg);
    const auto reference =
        make_reference_trajectory(trajectory_cfg.horizon_back + trajectory_cfg.horizon_ahead + 1);
    const auto track = make_track(reference, 1'000'000'000ULL);

    auto result = optimizer.optimize(track, track.timestamp_ns);
    ASSERT_TRUE(result.has_value()) << result.error();

    const WeaponCommand& cmd = *result;
    EXPECT_DOUBLE_EQ(cmd.distance, 2.0);
    ASSERT_TRUE(cmd.viz_debug.has_value());
    EXPECT_EQ(cmd.viz_debug->center_index, 2);
    EXPECT_EQ(cmd.viz_debug->reference_plan.size(), 6U);
    EXPECT_DOUBLE_EQ(cmd.viz_debug->reference_plan.front().distance, 0.0);
}

TEST(TrajectoryOptimizerTest, ReferenceAgeShiftsWindowForwardAndShrinksDebugHorizon) {
    WeaponControllerConfig config;
    config.reference_age_threshold_s = 1.0;
    config.enable_debug              = true;

    L4::ReferenceTrajectoryConfig trajectory_cfg;
    trajectory_cfg.horizon_back  = 2;
    trajectory_cfg.horizon_ahead = 3;
    trajectory_cfg.dt            = 0.1;

    TinyMpcTrajectoryOptimizer optimizer(config, trajectory_cfg);
    const auto reference =
        make_reference_trajectory(trajectory_cfg.horizon_back + trajectory_cfg.horizon_ahead + 1);
    const auto track = make_track(reference, 1'000'000'000ULL);

    auto result = optimizer.optimize(track, 1'200'000'000ULL);
    ASSERT_TRUE(result.has_value()) << result.error();

    const WeaponCommand& cmd = *result;
    EXPECT_DOUBLE_EQ(cmd.distance, 4.0);
    EXPECT_DOUBLE_EQ(cmd.tof, 0.04);
    ASSERT_TRUE(cmd.viz_debug.has_value());
    EXPECT_EQ(cmd.viz_debug->center_index, 2);
    EXPECT_EQ(cmd.viz_debug->reference_plan.size(), 4U);
    EXPECT_DOUBLE_EQ(cmd.viz_debug->reference_plan.front().distance, 2.0);
    EXPECT_DOUBLE_EQ(cmd.viz_debug->reference_plan[2].distance, 4.0);
}

TEST(TrajectoryOptimizerTest, HighTofTrackFireGateUsesFireTrajectoryInsteadOfControlTrajectory) {
    L4::ReferenceTrajectoryConfig trajectory_cfg;
    trajectory_cfg.horizon_back  = 2;
    trajectory_cfg.horizon_ahead = 2;
    trajectory_cfg.dt            = 0.1;
    const int horizon            = trajectory_cfg.horizon_back + trajectory_cfg.horizon_ahead + 1;

    const auto control = make_constant_reference_trajectory(horizon, 0.0, 0.0, 12.0, 1.5);
    const auto fire    = make_constant_reference_trajectory(horizon, 0.08, 0.0, 12.0, 1.5);
    const L4::TrackCommand track{
        .timestamp_ns       = 1'000'000'000ULL,
        .control_trajectory = control,
        .fire_trajectory    = fire,
    };

    FireDecisionConfig fire_cfg;
    fire_cfg.fire_thresh = 0.001;

    WeaponCommand cmd;
    cmd.timestamp_ns  = track.timestamp_ns;
    cmd.plan_yaw      = control.yaw_origin;
    cmd.plan_pitch    = 0.0;
    cmd.plan_distance = 12.0;

    auto center_aligned = apply_track_fire_gate(cmd, track, trajectory_cfg, fire_cfg, 0.0, 0.0);
    EXPECT_FALSE(center_aligned.fire);
    EXPECT_NEAR(center_aligned.yaw_error, 0.08, 1e-9);

    auto fire_aligned = apply_track_fire_gate(cmd, track, trajectory_cfg, fire_cfg, 0.08, 0.0);
    EXPECT_TRUE(fire_aligned.fire);
    EXPECT_NEAR(fire_aligned.yaw_error, 0.0, 1e-9);
}

TEST(TrajectoryOptimizerTest, HighTofTrackFireGateRejectsAmplifiedYawBias) {
    L4::ReferenceTrajectoryConfig trajectory_cfg;
    trajectory_cfg.horizon_back  = 1;
    trajectory_cfg.horizon_ahead = 1;
    trajectory_cfg.dt            = 0.1;
    const int horizon            = trajectory_cfg.horizon_back + trajectory_cfg.horizon_ahead + 1;

    const auto fire = make_constant_reference_trajectory(horizon, 0.0, 0.0, 12.0, 1.5);
    const L4::TrackCommand track{
        .timestamp_ns       = 1'000'000'000ULL,
        .control_trajectory = fire,
        .fire_trajectory    = fire,
    };

    FireDecisionConfig fire_cfg;
    fire_cfg.fire_thresh            = 0.001;
    fire_cfg.shooting_range_w_small = 0.12;
    fire_cfg.shooting_range_h       = 0.12;

    WeaponCommand cmd;
    cmd.timestamp_ns = track.timestamp_ns;

    auto biased = apply_track_fire_gate(cmd, track, trajectory_cfg, fire_cfg, 0.012, 0.0);
    EXPECT_FALSE(biased.fire);
    EXPECT_NEAR(biased.yaw_error, 0.012, 1e-9);
    EXPECT_LT(biased.shooting_range_yaw, biased.yaw_error);
}

} // namespace
} // namespace fcs::L5
