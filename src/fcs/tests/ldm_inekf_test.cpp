#include <gtest/gtest.h>

#include "L3_estimation/ldm_naive/ldm_tracker.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <numbers>
#include <optional>

namespace {

using Model   = fcs::L3::ldm::LdmKinematic;
using Nominal = Model::Nominal;
using CovXi   = Model::CovXi;

Nominal make_nominal(
    const Eigen::Matrix3d& R, const Eigen::Vector3d& velocity, const Eigen::Vector3d& position) {
    Nominal::IsometriesType t{};
    t[0] = velocity;
    t[1] = position;
    return Nominal(R, t);
}

void expect_symmetric(const CovXi& P, double tolerance = 1e-10) {
    EXPECT_LE((P - P.transpose()).norm(), tolerance);
}

double right_error_norm(const Nominal& estimate, const Nominal& truth) {
    return Nominal::log(estimate.inv() * truth).norm();
}

Model::PoseMeasurement pose_measurement_from_nominal(const Nominal& X) {
    return Model::PoseMeasurement{.R_world_body = X.R(), .p_world_body = X.p()};
}

Model::PoseMeasurement
    make_pose_measurement(const Eigen::Matrix3d& R, const Eigen::Vector3d& position) {
    return Model::PoseMeasurement{.R_world_body = R, .p_world_body = position};
}

} // namespace

TEST(LdmKinematic, PredictMovesPositionByVelocityAtIdentityRotation) {
    const Eigen::Vector3d velocity{1.0, -2.0, 0.5};
    const Eigen::Vector3d position{3.0, 4.0, 5.0};
    const Nominal x0 = make_nominal(Eigen::Matrix3d::Identity(), velocity, position);

    const Nominal predicted = Model::predict_state(x0, 0.2);

    EXPECT_NEAR((predicted.v() - velocity).norm(), 0.0, 1e-12);
    EXPECT_NEAR((predicted.p() - (position + 0.2 * velocity)).norm(), 0.0, 1e-12);
    EXPECT_NEAR((predicted.R() - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12);
}

TEST(LdmKinematic, PredictTreatsVelocityAsBodyFrame) {
    const Eigen::Matrix3d R =
        Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d velocity_body{2.0, 0.0, 0.0};
    const Eigen::Vector3d position{1.0, 2.0, 3.0};
    const Nominal x0 = make_nominal(R, velocity_body, position);

    const Nominal predicted = Model::predict_state(x0, 0.25);

    EXPECT_NEAR((predicted.v() - velocity_body).norm(), 0.0, 1e-12);
    EXPECT_NEAR((predicted.p() - (position + R * velocity_body * 0.25)).norm(), 0.0, 1e-12);
}

TEST(LdmKinematic, NoiseMatricesHaveExpectedDiagonalShape) {
    Model model{};
    const Model::PoseMeasurement z{
        .p_world_body = Eigen::Vector3d{5.0, 0.0, 0.0}
    };

    const auto R    = model.R(z);
    const auto diag = R.diagonal();

    EXPECT_EQ(R.rows(), Model::NZ);
    EXPECT_EQ(R.cols(), Model::NZ);
    EXPECT_NEAR((R.diagonal() - diag).norm(), 0.0, 1e-12);
    EXPECT_GT(diag[fcs::L3::ldm::BEARING_YAW], 0.0);
    EXPECT_GT(diag[fcs::L3::ldm::BEARING_DISTANCE], 0.0);
    EXPECT_GT(diag[fcs::L3::ldm::ROT_X], 0.0);
}

TEST(LdmKinematic, PoseInnovationUsesSo3LogAndWrapsBearing) {
    const Nominal predicted = make_nominal(
        Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
        fcs::L3::ypd2xyz(Eigen::Vector3d{std::numbers::pi - 1e-3, 0.0, 3.0}));

    const Eigen::Matrix3d R_observed =
        Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Model::PoseMeasurement observed{
        .R_world_body = R_observed,
        .p_world_body = fcs::L3::ypd2xyz(Eigen::Vector3d{-std::numbers::pi + 2e-3, 0.0, 4.0})};

    const auto innovation = Model::pose_innovation(predicted, observed);

    EXPECT_NEAR(innovation[fcs::L3::ldm::ROT_X], 0.01, 1e-12);
    EXPECT_NEAR(innovation[fcs::L3::ldm::ROT_Y], 0.0, 1e-12);
    EXPECT_NEAR(innovation[fcs::L3::ldm::ROT_Z], 0.0, 1e-12);
    EXPECT_NEAR(innovation[fcs::L3::ldm::BEARING_YAW], 3e-3, 1e-12);
    EXPECT_DOUBLE_EQ(innovation[fcs::L3::ldm::BEARING_DISTANCE], 1.0);
}

TEST(LdmKinematic, PoseUpdateJacobianMapsRightPositionPerturbationToYpd) {
    const Eigen::Matrix3d R = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d p{3.0, 1.0, -0.5};
    const Nominal predicted = make_nominal(R, Eigen::Vector3d::Zero(), p);

    const auto H = Model::pose_update_H(predicted);

    EXPECT_NEAR(
        (H.template block<3, 3>(fcs::L3::ldm::ROT_X, 0) - Eigen::Matrix3d::Identity()).norm(), 0.0,
        1e-12);
    EXPECT_NEAR(
        (H.template block<3, 3>(fcs::L3::ldm::BEARING_YAW, 6) - fcs::L3::xyz2ypd_jacobian(p) * R)
            .norm(),
        0.0, 1e-12);
}

TEST(LdmInEkf, PredictKeepsCovarianceSymmetricAndAddsProcessNoise) {
    fcs::L3::ldm::LdmInEkfTracker target;
    const Nominal x0 = make_nominal(
        Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d{5.0, 0.0, 0.0});
    const CovXi P0 = CovXi::Identity() * 0.01;

    target.initialize(Model::Params{}, x0, P0);
    const double trace_before = target.P().trace();
    target.predict(0.1);

    expect_symmetric(target.P());
    EXPECT_GT(target.P().trace(), trace_before);
}

TEST(LdmInEkf, UpdateReducesRightInvariantErrorForSyntheticMeasurement) {
    fcs::L3::ldm::LdmInEkfTracker target;
    const Nominal x0 = make_nominal(
        Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d{5.0, 0.0, 0.0});

    const Eigen::Matrix3d R_truth = (Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitZ())
                                     * Eigen::AngleAxisd(-0.02, Eigen::Vector3d::UnitY())
                                     * Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX()))
                                        .toRotationMatrix();
    const Nominal truth =
        make_nominal(R_truth, Eigen::Vector3d::Zero(), Eigen::Vector3d{5.1, 0.1, -0.05});

    const CovXi P0 = CovXi::Identity() * 1.0;
    target.initialize(Model::Params{}, x0, P0);
    target.predict(0.01);

    const double before = right_error_norm(target.nominal(), truth);
    target.update(pose_measurement_from_nominal(truth));
    const double after = right_error_norm(target.nominal(), truth);

    expect_symmetric(target.P());
    EXPECT_LT(after, before);
}

TEST(LdmTracker, ConfirmedMeasurementsPromoteToTracking) {
    fcs::L3::ldm::NaiveLdmConfig config;
    config.tracking_threshold = 1;
    fcs::L3::ldm::LdmTracker tracker(config);

    const auto z0 =
        make_pose_measurement(Eigen::Matrix3d::Identity(), Eigen::Vector3d{5.0, 0.0, 0.0});
    const auto z1 =
        make_pose_measurement(Eigen::Matrix3d::Identity(), Eigen::Vector3d{5.1, 0.0, 0.0});

    tracker.update(1'000'000, z0);
    EXPECT_EQ(tracker.status(), fcs::L3::TrackerStatus::Detecting);
    ASSERT_TRUE(tracker.get_output().has_value());
    EXPECT_FALSE(tracker.get_output()->is_tracking());

    tracker.update(2'000'000, z1);
    EXPECT_EQ(tracker.status(), fcs::L3::TrackerStatus::Tracking);

    const auto output = tracker.get_output();
    ASSERT_TRUE(output.has_value());
    EXPECT_TRUE(output->is_tracking());
    EXPECT_TRUE(output->accurate);
    EXPECT_EQ(output->last_observation_timestamp_ns, 2'000'000);
}

TEST(LdmTracker, MissingMeasurementEntersTempLostAndTimesOut) {
    fcs::L3::ldm::NaiveLdmConfig config;
    config.tracking_threshold = 1;
    config.lost_threshold     = 0.05;
    fcs::L3::ldm::LdmTracker tracker(config);

    const auto z =
        make_pose_measurement(Eigen::Matrix3d::Identity(), Eigen::Vector3d{5.0, 0.0, 0.0});

    tracker.update(1'000'000, z);
    tracker.update(2'000'000, z);
    ASSERT_EQ(tracker.status(), fcs::L3::TrackerStatus::Tracking);

    tracker.update(3'000'000, std::nullopt);
    EXPECT_EQ(tracker.status(), fcs::L3::TrackerStatus::TempLost);
    ASSERT_TRUE(tracker.get_output().has_value());
    EXPECT_TRUE(tracker.get_output()->is_tracking());
    EXPECT_FALSE(tracker.get_output()->accurate);

    tracker.update(100'000'000, std::nullopt);
    EXPECT_EQ(tracker.status(), fcs::L3::TrackerStatus::Idle);
    EXPECT_FALSE(tracker.get_output().has_value());
}

TEST(LdmTracker, DetectingDropsToIdleOnMissingMeasurement) {
    fcs::L3::ldm::NaiveLdmConfig config;
    fcs::L3::ldm::LdmTracker tracker(config);

    const auto z =
        make_pose_measurement(Eigen::Matrix3d::Identity(), Eigen::Vector3d{5.0, 0.0, 0.0});

    tracker.update(1'000'000, z);
    ASSERT_EQ(tracker.status(), fcs::L3::TrackerStatus::Detecting);

    tracker.update(2'000'000, std::nullopt);
    EXPECT_EQ(tracker.status(), fcs::L3::TrackerStatus::Idle);
    EXPECT_FALSE(tracker.get_output().has_value());
}

TEST(LdmTracker, OutputVelocityWorldMatchesBodyVelocityThroughRotation) {
    fcs::L3::ldm::NaiveLdmConfig config;
    config.tracking_threshold = 1;
    fcs::L3::ldm::LdmTracker tracker(config);

    const Eigen::Matrix3d R = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto z0           = make_pose_measurement(R, Eigen::Vector3d{5.0, 0.0, 0.0});
    const auto z1           = make_pose_measurement(R, Eigen::Vector3d{5.2, 0.1, 0.0});
    const auto z2           = make_pose_measurement(R, Eigen::Vector3d{5.4, 0.2, 0.0});

    tracker.update(1'000'000, z0);
    tracker.update(11'000'000, z1);
    tracker.update(21'000'000, z2);

    const auto output = tracker.get_output();
    ASSERT_TRUE(output.has_value());
    EXPECT_EQ(output->status, fcs::L3::TrackerStatus::Tracking);
    EXPECT_NEAR(
        (output->velocity_world - output->R_world_body * output->velocity_body).norm(), 0.0, 1e-12);
}
