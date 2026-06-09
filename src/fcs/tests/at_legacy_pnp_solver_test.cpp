#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <numbers>
#include <vector>

#include "core/types.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "L2_perception/armor/solver.hpp"
#include "L3_estimation/tracker/new_motion_model.hpp"
#include "camera_config.hpp"

namespace {

fcs::CameraConfig make_test_camera_config() {
    fcs::CameraConfig config;
    config.camera_matrix << 820.0, 0.0, 720.0, 0.0, 815.0, 540.0, 0.0, 0.0, 1.0;
    config.distort_coefficient << -0.085, 0.012, 0.0015, -0.0008, 0.0;
    config.width  = 1440;
    config.height = 1080;
    return config;
}

std::vector<cv::Point3f> armor_model_points(fcs::ArmorType type) {
    constexpr double kSmallWidth  = 135.0 / 1000.0;
    constexpr double kSmallHeight = 55.0 / 1000.0;
    constexpr double kLargeWidth  = 230.0 / 1000.0;
    constexpr double kLargeHeight = 55.0 / 1000.0;

    const double width  = (type == fcs::ArmorType::Large) ? kLargeWidth : kSmallWidth;
    const double height = (type == fcs::ArmorType::Large) ? kLargeHeight : kSmallHeight;

    const float hw = static_cast<float>(width * 0.5);
    const float hh = static_cast<float>(height * 0.5);
    return {
        cv::Point3f(0.0f, hw, hh),
        cv::Point3f(0.0f, -hw, hh),
        cv::Point3f(0.0f, -hw, -hh),
        cv::Point3f(0.0f, hw, -hh),
    };
}

Eigen::Matrix3d armor_rotation(double yaw, fcs::ArmorName name) {
    const double tilt = fcs::L2::armor_pitch_rad_for(name);
    const Eigen::AngleAxisd yaw_rot(yaw, Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd pitch_rot(tilt, Eigen::Vector3d::UnitY());
    return (yaw_rot * pitch_rot).toRotationMatrix();
}

cv::Mat eigen_rotation_to_rvec(const Eigen::Matrix3d& rotation) {
    cv::Mat rvec;
    cv::Mat rotation_cv(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            rotation_cv.at<double>(r, c) = rotation(r, c);
        }
    }
    cv::Rodrigues(rotation_cv, rvec);
    return rvec;
}

fcs::ArmorDetection make_detection(
    fcs::ArmorName name, fcs::ArmorColor color, const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation, const fcs::CameraConfig& camera_config) {
    const auto type = fcs::cls_to_armor_type(name);
    const auto obj  = armor_model_points(type);
    const auto rvec = eigen_rotation_to_rvec(rotation);
    const cv::Mat tvec =
        (cv::Mat_<double>(3, 1) << translation.x(), translation.y(), translation.z());

    cv::Mat camera_matrix(3, 3, CV_64F);
    cv::Mat dist_coeffs(1, 5, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            camera_matrix.at<double>(r, c) = camera_config.camera_matrix(r, c);
        }
    }
    for (int i = 0; i < 5; ++i) {
        dist_coeffs.at<double>(0, i) = camera_config.distort_coefficient(i);
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs, projected);
    EXPECT_EQ(projected.size(), 4u);

    std::array<cv::Point2f, 4> corners{};
    std::copy(projected.begin(), projected.end(), corners.begin());

    const auto center = (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;
    constexpr float kSyntheticDetectorScaleBiasPx = 0.65f;
    for (auto& corner : corners) {
        const cv::Point2f delta = center - corner;
        const float norm        = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        if (norm > 1e-6f) {
            corner += (kSyntheticDetectorScaleBiasPx / norm) * delta;
        }
    }
    return fcs::ArmorDetection(corners, name, color, 1.0f);
}

fcs::ArmorDetection
    bias_detection_towards_center(const fcs::ArmorDetection& detection, float bias_px) {
    auto corners      = detection.corners;
    const auto center = detection.center();

    for (auto& corner : corners) {
        const cv::Point2f delta = center - corner;
        const float norm        = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        if (norm > 1e-6f) {
            corner += (bias_px / norm) * delta;
        }
    }

    return fcs::ArmorDetection(corners, detection.name, detection.color, detection.confidence);
}

fcs::ArmorDetection debiased_detection_for_projection(const fcs::ArmorDetection& detection) {
    const auto debiased = fcs::L2::debias_correlated_corner_scale(detection.image_points());
    std::array<cv::Point2f, 4> corners{};
    std::copy(debiased.begin(), debiased.end(), corners.begin());
    return fcs::ArmorDetection(corners, detection.name, detection.color, detection.confidence);
}

double reprojection_rmse_px(
    const fcs::CameraArmorMeasurement& measurement, const fcs::ArmorDetection& detection,
    const fcs::CameraConfig& camera_config) {
    const auto obj_points = armor_model_points(detection.type);
    const auto rotation   = measurement.transform.rotation();
    const auto t          = measurement.transform.translation();
    const auto rvec       = eigen_rotation_to_rvec(rotation);
    const cv::Mat tvec    = (cv::Mat_<double>(3, 1) << t.x(), t.y(), t.z());

    cv::Mat camera_matrix(3, 3, CV_64F);
    cv::Mat dist_coeffs(1, 5, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            camera_matrix.at<double>(r, c) = camera_config.camera_matrix(r, c);
        }
    }
    for (int i = 0; i < 5; ++i) {
        dist_coeffs.at<double>(0, i) = camera_config.distort_coefficient(i);
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj_points, rvec, tvec, camera_matrix, dist_coeffs, projected);

    double sum_sq = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
        const cv::Point2f d = projected[i] - detection.corners[i];
        sum_sq += static_cast<double>(d.x) * static_cast<double>(d.x)
                + static_cast<double>(d.y) * static_cast<double>(d.y);
    }
    return std::sqrt(sum_sq / static_cast<double>(projected.size()));
}

double rotation_error_rad(
    const fcs::CameraArmorMeasurement& measurement, const Eigen::Matrix3d& rotation_gt) {
    const Eigen::Matrix3d delta = measurement.transform.rotation() * rotation_gt.transpose();
    return Eigen::AngleAxisd(delta).angle();
}

TEST(PnPSolver, RecoversSmallArmorPoseWithoutPriorUnderDistortion) {
    const auto camera_config = make_test_camera_config();

    fcs::L2::PnPSolver solver(camera_config);

    const double yaw_gt               = 0.42;
    const Eigen::Matrix3d rotation_gt = armor_rotation(yaw_gt, fcs::ArmorName::Three);
    const Eigen::Vector3d translation_gt(0.14, -0.08, 4.2);
    const auto detection = make_detection(
        fcs::ArmorName::Three, fcs::ArmorColor::Blue, rotation_gt, translation_gt, camera_config);

    const auto result = solver.solve_with_ba(detection, Eigen::Matrix3d::Identity(), 1234);
    ASSERT_TRUE(result.has_value());

    const auto& measurement = *result;
    EXPECT_LT((measurement.transform.translation() - translation_gt).norm(), 2e-3);
    EXPECT_LT(rotation_error_rad(measurement, rotation_gt), 3e-3);
    EXPECT_LT(
        reprojection_rmse_px(
            measurement, debiased_detection_for_projection(detection), camera_config),
        0.1);
}

TEST(PnPSolver, RecoversLargeArmorPoseWithPosePriorUnderDistortion) {
    const auto camera_config = make_test_camera_config();

    fcs::L2::PnPSolver solver(camera_config);

    const double yaw_gt               = -0.31;
    const Eigen::Matrix3d rotation_gt = armor_rotation(yaw_gt, fcs::ArmorName::One);
    const Eigen::Vector3d translation_gt(-0.18, 0.06, 5.4);
    const auto detection = make_detection(
        fcs::ArmorName::One, fcs::ArmorColor::Red, rotation_gt, translation_gt, camera_config);

    const Eigen::Matrix3d rotation_prior    = armor_rotation(yaw_gt + 0.06, fcs::ArmorName::One);
    const Eigen::Vector3d translation_prior = translation_gt + Eigen::Vector3d(0.03, -0.02, 0.10);
    const auto prior_rvec                   = eigen_rotation_to_rvec(rotation_prior);

    std::vector<fcs::L2::PnPSolver::PosePrior> priors;
    priors.push_back({
        .rvec =
            cv::Vec3d(prior_rvec.at<double>(0), prior_rvec.at<double>(1), prior_rvec.at<double>(2)),
        .tvec      = cv::Vec3d(translation_prior.x(), translation_prior.y(), translation_prior.z()),
        .hint_cost = 1e-4,
        .armor_id  = 0,
    });

    const auto result = solver.solve_with_ba(detection, Eigen::Matrix3d::Identity(), 5678, priors);
    ASSERT_TRUE(result.has_value());

    const auto& measurement = *result;
    EXPECT_LT((measurement.transform.translation() - translation_gt).norm(), 2e-3);
    EXPECT_LT(rotation_error_rad(measurement, rotation_gt), 3e-3);
    EXPECT_LT(
        reprojection_rmse_px(
            measurement, debiased_detection_for_projection(detection), camera_config),
        0.1);
}

TEST(PnPSolver, KeepsConstrainedPoseManifoldWithBiasedCornersAndPrior) {
    const auto camera_config = make_test_camera_config();

    fcs::L2::PnPSolver solver(camera_config);

    const double yaw_gt               = 0.57;
    const Eigen::Matrix3d rotation_gt = armor_rotation(yaw_gt, fcs::ArmorName::Five);
    const Eigen::Vector3d translation_gt(0.09, -0.03, 4.8);
    const auto detection_gt = make_detection(
        fcs::ArmorName::Five, fcs::ArmorColor::Blue, rotation_gt, translation_gt, camera_config);
    const auto detection = bias_detection_towards_center(detection_gt, 1.2f);

    const Eigen::Matrix3d rotation_prior    = armor_rotation(yaw_gt - 0.08, fcs::ArmorName::Five);
    const Eigen::Vector3d translation_prior = translation_gt + Eigen::Vector3d(-0.02, 0.02, 0.12);
    const auto prior_rvec                   = eigen_rotation_to_rvec(rotation_prior);

    std::vector<fcs::L2::PnPSolver::PosePrior> priors;
    priors.push_back({
        .rvec =
            cv::Vec3d(prior_rvec.at<double>(0), prior_rvec.at<double>(1), prior_rvec.at<double>(2)),
        .tvec      = cv::Vec3d(translation_prior.x(), translation_prior.y(), translation_prior.z()),
        .hint_cost = 5e-4,
        .armor_id  = 0,
    });

    const auto result = solver.solve_with_ba(detection, Eigen::Matrix3d::Identity(), 9012, priors);
    ASSERT_TRUE(result.has_value());

    const auto& measurement = *result;
    const double yaw_est    = fcs::L2::extract_yaw_from_rotation(
        measurement.transform.rotation(), Eigen::Matrix3d::Identity());
    const Eigen::Matrix3d constrained_rotation = armor_rotation(yaw_est, detection.name);
    const Eigen::Matrix3d delta =
        measurement.transform.rotation() * constrained_rotation.transpose();

    EXPECT_LT(Eigen::AngleAxisd(delta).angle(), 1e-6);
    EXPECT_LT(rotation_error_rad(measurement, rotation_gt), 3e-2);
    EXPECT_GT(measurement.transform.translation().z(), 1.0);
}

TEST(PnPSolver, CorrelatedCornerScaleBiasInflatesLogDistanceCovariance) {
    const auto camera_config = make_test_camera_config();

    fcs::L2::PnPSolver solver(camera_config);

    const double yaw_gt               = 0.18;
    const Eigen::Matrix3d rotation_gt = armor_rotation(yaw_gt, fcs::ArmorName::Three);
    const Eigen::Vector3d translation_gt(0.0, -0.04, 6.0);
    const auto detection_gt = make_detection(
        fcs::ArmorName::Three, fcs::ArmorColor::Blue, rotation_gt, translation_gt, camera_config);
    const auto biased_detection = bias_detection_towards_center(detection_gt, 1.0f);

    const auto clean_result = solver.solve_with_ba(detection_gt, Eigen::Matrix3d::Identity(), 3456);
    const auto biased_result =
        solver.solve_with_ba(biased_detection, Eigen::Matrix3d::Identity(), 3457);

    ASSERT_TRUE(clean_result.has_value());
    ASSERT_TRUE(biased_result.has_value());

    const double clean_log_var   = clean_result->pnp_cov_ypdr(2, 2);
    const double biased_log_var  = biased_result->pnp_cov_ypdr(2, 2);
    const double biased_distance = biased_result->transform.translation().norm();
    const double gt_distance     = translation_gt.norm();

    EXPECT_GT(biased_distance, gt_distance * 1.05);
    EXPECT_GT(biased_log_var, clean_log_var);
    EXPECT_GT(std::sqrt(std::max(0.0, biased_log_var)), 0.08);
}

} // namespace
