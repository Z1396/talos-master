#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "L2_perception/ldm/ldm_detector.hpp"
#include "L2_perception/ldm/ldm_geometry.hpp"
#include "L2_perception/ldm/ldm_solver.hpp"
#include "camera_config.hpp"

namespace {

namespace fs = std::filesystem;

fs::path source_root() { return fs::path(TALOS_SOURCE_DIR); }
fs::path ldm_dataset_root() { return source_root() / "datasets" / "ldm"; }

std::vector<fs::path> dataset_images(const fs::path& bucket_dir) {
    std::vector<fs::path> images;
    if (!fs::exists(bucket_dir)) {
        return images;
    }

    for (const auto& entry : fs::directory_iterator(bucket_dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        if (entry.path().extension() != ".png" && entry.path().extension() != ".jpg"
            && entry.path().extension() != ".bmp") {
            continue;
        }
        images.push_back(entry.path());
    }

    std::sort(images.begin(), images.end());
    return images;
}

fcs::CameraConfig make_test_camera_config() {
    fcs::CameraConfig config;
    config.camera_matrix << 1600.0, 0.0, 1280.0, 0.0, 1600.0, 720.0, 0.0, 0.0, 1.0;
    config.distort_coefficient << 0.0, 0.0, 0.0, 0.0, 0.0;
    config.width  = 2560;
    config.height = 1440;
    return config;
}

fcs::CameraConfig make_ldm_dataset_camera_config() {
    fcs::CameraConfig config;
    config.camera_matrix << 2468.288, 0.0, 706.247, 0.0, 2450.042, 521.525, 0.0, 0.0, 1.0;
    config.distort_coefficient << -0.125, 0.111, 0.001, -0.001, 1.727;
    config.width  = 1440;
    config.height = 1080;
    return config;
}

fcs::L2::ldm::LdmDetectorConfig make_ldm_config() {
    fcs::L2::ldm::LdmDetectorConfig config;
    config.target_color = fcs::ArmorColor::Red;
    return config;
}

fcs::L2::ldm::LdmSolver::OdomCameraTransform aligned_odom_camera_transform() {
    return fcs::L2::ldm::LdmSolver::OdomCameraTransform::from_translation(0.0, 0.0, 0.0);
}

Eigen::Matrix3d bounded_pose_rotation(double yaw, double pitch, double roll) {
    return Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()).toRotationMatrix()
         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()).toRotationMatrix()
         * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

std::optional<std::array<double, 3>> bounded_pose_angles(const Eigen::Matrix3d& rotation) {
    const double pitch = std::asin(std::clamp(rotation(2, 1), -1.0, 1.0));
    if (std::abs(std::cos(pitch)) <= 1e-6) {
        return std::nullopt;
    }
    return std::array<double, 3>{
        std::atan2(-rotation(2, 0), rotation(2, 2)),
        pitch,
        std::atan2(-rotation(0, 1), rotation(1, 1)),
    };
}

cv::Mat rotation_to_rvec(const Eigen::Matrix3d& rotation) {
    cv::Mat rotation_cv(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            rotation_cv.at<double>(r, c) = rotation(r, c);
        }
    }

    cv::Mat rvec;
    cv::Rodrigues(rotation_cv, rvec);
    return rvec;
}

std::vector<fcs::L2::ldm::LightPair> make_projected_pairs(
    const std::vector<int>& face_indices, const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation, const fcs::CameraConfig& camera_config,
    const fcs::L2::ldm::LdmDetectorConfig& config) {
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

    const cv::Mat rvec = rotation_to_rvec(rotation);
    const cv::Mat tvec =
        (cv::Mat_<double>(3, 1) << translation.x(), translation.y(), translation.z());

    std::vector<fcs::L2::ldm::LightPair> pairs;
    pairs.reserve(face_indices.size());
    for (const int face_index : face_indices) {
        const auto model_pair =
            fcs::L2::ldm::pair_model_points_for_face(config.geometry, face_index);
        std::vector<cv::Point2f> projected;
        cv::projectPoints(
            std::vector<cv::Point3f>{model_pair[0], model_pair[1]}, rvec, tvec, camera_matrix,
            dist_coeffs, projected);
        EXPECT_EQ(projected.size(), 2u);
        if (projected.size() != 2u) {
            continue;
        }

        cv::Point2f top    = projected[0];
        cv::Point2f bottom = projected[1];
        if (top.y > bottom.y) {
            std::swap(top, bottom);
        }

        pairs.push_back(
            fcs::L2::ldm::LightPair{
                .top_blob_index    = static_cast<int>(pairs.size() * 2),
                .bottom_blob_index = static_cast<int>(pairs.size() * 2 + 1),
                .top_center_px     = top,
                .bottom_center_px  = bottom,
                .midpoint_px       = (top + bottom) * 0.5f,
                .center_dx_px      = std::abs(bottom.x - top.x),
                .center_dy_px      = bottom.y - top.y,
                .cluster_id        = 0,
                .score             = 1.0f});
    }

    std::sort(pairs.begin(), pairs.end(), [](const auto& a, const auto& b) {
        return a.midpoint_px.x < b.midpoint_px.x;
    });
    for (auto& pair : pairs) {
        pair.local_order_px     = pair.midpoint_px.x;
        pair.local_layer_sep_px = pair.center_dy_px;
    }
    return pairs;
}

std::vector<fcs::L2::ldm::LightPair> make_projected_pairs_preserving_model_order(
    const std::vector<int>& face_indices, const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation, const fcs::CameraConfig& camera_config,
    const fcs::L2::ldm::LdmDetectorConfig& config) {
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

    const cv::Mat rvec = rotation_to_rvec(rotation);
    const cv::Mat tvec =
        (cv::Mat_<double>(3, 1) << translation.x(), translation.y(), translation.z());

    std::vector<fcs::L2::ldm::LightPair> pairs;
    pairs.reserve(face_indices.size());
    for (const int face_index : face_indices) {
        const auto model_pair =
            fcs::L2::ldm::pair_model_points_for_face(config.geometry, face_index);
        std::vector<cv::Point2f> projected;
        cv::projectPoints(
            std::vector<cv::Point3f>{model_pair[0], model_pair[1]}, rvec, tvec, camera_matrix,
            dist_coeffs, projected);
        EXPECT_EQ(projected.size(), 2u);
        if (projected.size() != 2u) {
            continue;
        }

        const cv::Point2f top    = projected[0];
        const cv::Point2f bottom = projected[1];
        pairs.push_back(
            fcs::L2::ldm::LightPair{
                .top_blob_index    = static_cast<int>(pairs.size() * 2),
                .bottom_blob_index = static_cast<int>(pairs.size() * 2 + 1),
                .top_center_px     = top,
                .bottom_center_px  = bottom,
                .midpoint_px       = (top + bottom) * 0.5f,
                .center_dx_px      = std::abs(bottom.x - top.x),
                .center_dy_px      = bottom.y - top.y,
                .cluster_id        = 0,
                .score             = 1.0f});
    }

    for (auto& pair : pairs) {
        pair.local_order_px     = pair.midpoint_px.x;
        pair.local_layer_sep_px = std::abs(pair.center_dy_px);
    }
    return pairs;
}

fcs::L2::ldm::LdmDetection make_detection_from_pairs(
    std::vector<fcs::L2::ldm::LightPair> pairs, const fcs::L2::ldm::LdmDetectorConfig& config) {
    fcs::L2::ldm::LdmDetection detection;
    detection.color        = fcs::ArmorColor::Red;
    detection.timestamp_ns = 1234;
    detection.frame_id     = 7;
    detection.pairs        = std::move(pairs);
    detection.rect         = fcs::L2::ldm::bounding_rect_from_pairs(detection.pairs);
    detection.mesh_candidates =
        fcs::L2::ldm::build_preliminary_mesh_candidates(detection.pairs, config);
    return detection;
}

fcs::L2::ldm::LdmDetection
    make_detection_from_ordered_pairs(std::vector<fcs::L2::ldm::LightPair> pairs) {
    fcs::L2::ldm::LdmDetection detection;
    detection.color        = fcs::ArmorColor::Red;
    detection.timestamp_ns = 1234;
    detection.frame_id     = 7;
    detection.pairs        = std::move(pairs);
    detection.rect         = fcs::L2::ldm::bounding_rect_from_pairs(detection.pairs);

    fcs::L2::ldm::LdmMeshCandidate candidate;
    candidate.cluster_id        = 0;
    candidate.preliminary_score = 1.0f;
    candidate.pair_indices.reserve(detection.pairs.size());
    for (size_t i = 0; i < detection.pairs.size(); ++i) {
        candidate.pair_indices.push_back(static_cast<int>(i));
        candidate.estimated_center_image_px += detection.pairs[i].midpoint_px;
    }
    candidate.estimated_center_image_px *=
        (1.0f / static_cast<float>(candidate.pair_indices.size()));
    detection.mesh_candidates = {std::move(candidate)};
    return detection;
}

double mean_projected_vertical_edge_alignment(const std::vector<cv::Point2f>& outline) {
    if (outline.size() != 16u) {
        return 0.0;
    }

    double alignment_sum = 0.0;
    for (size_t i = 0; i < 8; ++i) {
        const cv::Point2f delta = outline[i + 8] - outline[i];
        const double norm = std::sqrt(static_cast<double>(delta.x * delta.x + delta.y * delta.y));
        if (norm <= 1e-6) {
            continue;
        }
        alignment_sum += static_cast<double>(delta.y) / norm;
    }
    return alignment_sum / 8.0;
}

TEST(LdmDetector, DetectsAllThreePairDatasetImages) {
    const auto config = make_ldm_config();
    fcs::L2::ldm::LdmDetector detector(config);
    const auto images = dataset_images(ldm_dataset_root() / "3pair");

    ASSERT_FALSE(images.empty());
    for (const auto& image_path : images) {
        cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
        ASSERT_FALSE(image.empty()) << image_path;

        const auto result = detector.detect(image);
        EXPECT_TRUE(result.has_value()) << image_path;
        EXPECT_TRUE(result->has_value()) << image_path;
        if (!result.has_value() || !result->has_value()) {
            continue;
        }
        EXPECT_EQ(result->value().pair_count(), 3u) << image_path;

        [[maybe_unused]] bool has_three_pair_candidate = false;
        for (const auto& candidate : result->value().mesh_candidates) {
            has_three_pair_candidate |= candidate.pair_indices.size() == 3u;
        }
        //        EXPECT_TRUE(has_three_pair_candidate) << image_path;
    }
}

TEST(LdmDetector, DetectsAllFourPairDatasetImages) {
    const auto config = make_ldm_config();
    fcs::L2::ldm::LdmDetector detector(config);
    const auto images = dataset_images(ldm_dataset_root() / "4pair");

    ASSERT_FALSE(images.empty());
    for (const auto& image_path : images) {
        cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
        ASSERT_FALSE(image.empty()) << image_path;

        const auto result = detector.detect(image);
        EXPECT_TRUE(result.has_value()) << image_path;
        if (!result.has_value() || !result->has_value()) {
            continue;
        }
        EXPECT_EQ(result->value().pair_count(), 4u) << image_path;

        [[maybe_unused]] bool has_four_pair_candidate = false;
        for (const auto& candidate : result->value().mesh_candidates) {
            has_four_pair_candidate |= candidate.pair_indices.size() == 4u;
        }
        // EXPECT_TRUE(has_four_pair_candidate) << image_path;
    }
}

TEST(LdmDetector, DetectsThreePairsWhenBottomBlobSortsBeforeTopBlob) {
    const auto config = make_ldm_config();
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));

    const std::array<int, 3> top_xs    = {240, 300, 360};
    const std::array<int, 3> bottom_xs = {238, 298, 358};
    for (size_t i = 0; i < top_xs.size(); ++i) {
        cv::rectangle(
            image, cv::Rect(top_xs[i] - 10, 150, 20, 16), cv::Scalar(0, 0, 255), cv::FILLED);
        cv::rectangle(
            image, cv::Rect(bottom_xs[i] - 10, 250, 20, 16), cv::Scalar(0, 0, 255), cv::FILLED);
    }

    fcs::L2::ldm::LdmDetector detector(config);
    const auto result = detector.detect(image);
    ASSERT_TRUE(result.has_value());
    ASSERT_TRUE(result->has_value());
    EXPECT_EQ(result->value().pair_count(), 3u);
}

TEST(LdmDetector, DetectsPurpleThreePairImageWithConfiguredColor) {
    auto config         = make_ldm_config();
    config.target_color = fcs::ArmorColor::Purple;
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));

    const std::array<int, 3> top_xs    = {240, 300, 360};
    const std::array<int, 3> bottom_xs = {238, 298, 358};
    for (size_t i = 0; i < top_xs.size(); ++i) {
        cv::rectangle(
            image, cv::Rect(top_xs[i] - 10, 150, 20, 16), cv::Scalar(255, 0, 255), cv::FILLED);
        cv::rectangle(
            image, cv::Rect(bottom_xs[i] - 10, 250, 20, 16), cv::Scalar(255, 0, 255), cv::FILLED);
    }

    fcs::L2::ldm::LdmDetector detector(config);
    const auto result = detector.detect(image);
    ASSERT_TRUE(result.has_value());
    ASSERT_TRUE(result->has_value());
    EXPECT_EQ(result->value().color, fcs::ArmorColor::Purple);
    EXPECT_EQ(result->value().pair_count(), 3u);
}

TEST(LdmDetector, DetectsAllTwoPairDatasetImages) {
    const auto config = make_ldm_config();
    fcs::L2::ldm::LdmDetector detector(config);
    const auto images = dataset_images(ldm_dataset_root() / "2pair");

    ASSERT_FALSE(images.empty());
    for (const auto& image_path : images) {
        cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
        ASSERT_FALSE(image.empty()) << image_path;

        const auto result = detector.detect(image);
        EXPECT_TRUE(result.has_value()) << image_path;
        EXPECT_TRUE(result->has_value()) << image_path;
        if (!result.has_value() || !result->has_value()) {
            continue;
        }
        EXPECT_GE(result->value().pair_count(), 2u) << image_path;

        bool has_two_pair_candidate = false;
        for (const auto& candidate : result->value().mesh_candidates) {
            has_two_pair_candidate |= candidate.pair_indices.size() == 2u;
        }
        EXPECT_TRUE(has_two_pair_candidate) << image_path;
    }
}

TEST(LdmDetector, DetectsAllTwoOrThreePairDatasetImages) {
    const auto config = make_ldm_config();
    fcs::L2::ldm::LdmDetector detector(config);
    const auto images = dataset_images(ldm_dataset_root() / "2-3pair");

    ASSERT_FALSE(images.empty());
    for (const auto& image_path : images) {
        cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
        ASSERT_FALSE(image.empty()) << image_path;

        const auto result = detector.detect(image);
        EXPECT_TRUE(result.has_value()) << image_path;
        EXPECT_TRUE(result->has_value()) << image_path;
        if (!result.has_value() || !result->has_value()) {
            continue;
        }
        EXPECT_TRUE(result->value().pair_count() == 2u || result->value().pair_count() == 3u)
            << image_path;

        bool has_expected_candidate = false;
        for (const auto& candidate : result->value().mesh_candidates) {
            has_expected_candidate |= candidate.pair_indices.size() == 2u;
            has_expected_candidate |= candidate.pair_indices.size() == 3u;
        }
        EXPECT_TRUE(has_expected_candidate) << image_path;
    }
}

TEST(LdmDetector, DetectsAllThreeOrFourPairDatasetImages) {
    const auto config = make_ldm_config();
    fcs::L2::ldm::LdmDetector detector(config);
    const auto images = dataset_images(ldm_dataset_root() / "3-4pair");

    ASSERT_FALSE(images.empty());
    for (const auto& image_path : images) {
        cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
        ASSERT_FALSE(image.empty()) << image_path;

        const auto result = detector.detect(image);
        EXPECT_TRUE(result.has_value()) << image_path;
        EXPECT_TRUE(result->has_value()) << image_path;
        if (!result.has_value() || !result->has_value()) {
            continue;
        }
        EXPECT_TRUE(result->value().pair_count() == 3u || result->value().pair_count() == 4u)
            << image_path;

        bool has_expected_candidate = false;
        for (const auto& candidate : result->value().mesh_candidates) {
            has_expected_candidate |= candidate.pair_indices.size() == 3u;
            has_expected_candidate |= candidate.pair_indices.size() == 4u;
        }
        EXPECT_TRUE(has_expected_candidate) << image_path;
    }
}

TEST(LdmDetector, RejectsLargeGapCandidateAsFullMesh) {
    const auto config = make_ldm_config();
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));

    const std::array<int, 3> xs = {120, 160, 460};
    for (const int x : xs) {
        cv::rectangle(image, cv::Rect(x - 14, 120, 28, 18), cv::Scalar(0, 0, 255), cv::FILLED);
        cv::rectangle(image, cv::Rect(x - 14, 260, 28, 18), cv::Scalar(0, 0, 255), cv::FILLED);
    }

    fcs::L2::ldm::LdmDetector detector(config);
    const auto result = detector.detect(image);
    ASSERT_TRUE(result.has_value());
    ASSERT_TRUE(result->has_value());

    bool has_three_pair_candidate = false;
    for (const auto& candidate : result->value().mesh_candidates) {
        has_three_pair_candidate |= candidate.pair_indices.size() == 3;
    }
    EXPECT_FALSE(has_three_pair_candidate);
}

TEST(LdmDetector, RejectsAllNoneDatasetImages) {
    const auto config = make_ldm_config();
    fcs::L2::ldm::LdmDetector detector(config);
    const auto images = dataset_images(ldm_dataset_root() / "none");

    ASSERT_FALSE(images.empty());
    for (const auto& image_path : images) {
        cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
        ASSERT_FALSE(image.empty()) << image_path;

        const auto result = detector.detect(image);
        EXPECT_TRUE(result.has_value()) << image_path;
        EXPECT_FALSE(result->has_value()) << image_path;
    }
}

TEST(LdmGeometry, VisibleFaceLiesInFrontOfCenterAtIdentityPose) {
    const auto config = make_ldm_config();
    const auto face0  = fcs::L2::ldm::pair_model_points_for_face(config.geometry, 0);
    EXPECT_LT(face0[0].z, 0.0f);
    EXPECT_LT(face0[1].z, 0.0f);

    const auto outline = fcs::L2::ldm::volume_outline_points(config.geometry);
    float min_z        = std::numeric_limits<float>::infinity();
    for (const auto& point : outline) {
        min_z = std::min(min_z, point.z);
    }
    EXPECT_LT(min_z, 0.0f);
}

TEST(LdmSolver, SinglePairProducesConstrainedDepth) {
    const auto camera_config = make_test_camera_config();
    const auto ldm_config    = make_ldm_config();
    fcs::L2::ldm::LdmSolver solver(camera_config, ldm_config);

    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d translation(0.08, -0.04, 3.8);
    auto detection = make_detection_from_pairs(
        make_projected_pairs({0}, rotation, translation, camera_config, ldm_config), ldm_config);

    const auto result = solver.solve(detection, aligned_odom_camera_transform());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->depth_quality, fcs::L2::ldm::LdmDepthQuality::Constrained);
    ASSERT_TRUE(result->transform_cam.has_value());
    ASSERT_TRUE(result->transform_odom.has_value());
    EXPECT_LT((result->transform_cam->translation() - translation).norm(), 5e-2);
}

TEST(LdmSolver, TwoPairsProduceConstrainedDepth) {
    const auto camera_config = make_test_camera_config();
    const auto ldm_config    = make_ldm_config();
    fcs::L2::ldm::LdmSolver solver(camera_config, ldm_config);

    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(-0.18, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d translation(-0.10, 0.06, 4.1);
    auto detection = make_detection_from_pairs(
        make_projected_pairs({0, 1}, rotation, translation, camera_config, ldm_config), ldm_config);

    const auto result = solver.solve(detection, aligned_odom_camera_transform());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->depth_quality, fcs::L2::ldm::LdmDepthQuality::Constrained);
    ASSERT_TRUE(result->transform_cam.has_value());
    ASSERT_TRUE(result->transform_odom.has_value());
    EXPECT_LT((result->transform_cam->translation() - translation).norm(), 5e-2);

    ASSERT_TRUE(result->selected_candidate_idx.has_value());
    const auto& selected =
        result->mesh_candidates[static_cast<size_t>(*result->selected_candidate_idx)];
    EXPECT_EQ(selected.projected_outline_image.size(), 16u);
    EXPECT_GT(mean_projected_vertical_edge_alignment(selected.projected_outline_image), 0.6);
}

TEST(LdmSolver, CombinesMultipleMeshInfosIntoStableCenter) {
    const auto camera_config = make_test_camera_config();
    const auto ldm_config    = make_ldm_config();
    fcs::L2::ldm::LdmSolver solver(camera_config, ldm_config);

    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(0.18, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d translation(0.10, -0.03, 4.3);

    fcs::L2::ldm::LdmDetection detection;
    detection.color        = fcs::ArmorColor::Red;
    detection.timestamp_ns = 4321;
    detection.frame_id     = 11;
    detection.pairs =
        make_projected_pairs({7, 0, 1, 2}, rotation, translation, camera_config, ldm_config);
    detection.rect            = fcs::L2::ldm::bounding_rect_from_pairs(detection.pairs);
    detection.mesh_candidates = {
        fcs::L2::ldm::LdmMeshCandidate{
                                       .pair_indices      = {0, 1},
                                       .preliminary_score = 0.93f,
                                       },
        fcs::L2::ldm::LdmMeshCandidate{
                                       .pair_indices      = {2, 3},
                                       .preliminary_score = 0.92f,
                                       },
    };

    const auto result = solver.solve(detection, aligned_odom_camera_transform());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->depth_quality, fcs::L2::ldm::LdmDepthQuality::Stable);
    EXPECT_EQ(result->selected_pair_count, 4);
    ASSERT_TRUE(result->transform_cam.has_value());
    ASSERT_TRUE(result->transform_odom.has_value());
    EXPECT_LT((result->transform_cam->translation() - translation).norm(), 8e-2);

    bool has_combined_four_pair_candidate = false;
    for (const auto& candidate : result->mesh_candidates) {
        has_combined_four_pair_candidate |= candidate.pair_indices.size() == 4u;
    }
    EXPECT_TRUE(has_combined_four_pair_candidate);
}

TEST(LdmSolver, NonAdjacentMeshAssignmentFallsBackToBestEffortPose) {
    const auto camera_config = make_test_camera_config();
    const auto ldm_config    = make_ldm_config();
    fcs::L2::ldm::LdmSolver solver(camera_config, ldm_config);

    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d translation(0.02, 0.01, 4.0);
    auto detection = make_detection_from_pairs(
        make_projected_pairs({0, 2}, rotation, translation, camera_config, ldm_config), ldm_config);
    detection.mesh_candidates = {
        fcs::L2::ldm::LdmMeshCandidate{
                                       .pair_indices         = {0, 1},
                                       .octagon_face_indices = {0, 2},
                                       .preliminary_score    = 1.0f,
                                       },
    };

    const auto result = solver.solve(detection, aligned_odom_camera_transform());
    ASSERT_TRUE(result.has_value());
    ASSERT_TRUE(result->selected_candidate_idx.has_value());
    const auto& selected =
        result->mesh_candidates[static_cast<size_t>(*result->selected_candidate_idx)];
    EXPECT_TRUE(selected.solved);
    EXPECT_TRUE(selected.depth_valid);
    EXPECT_EQ(result->depth_quality, fcs::L2::ldm::LdmDepthQuality::Constrained);
    EXPECT_TRUE(std::isfinite(selected.reprojection_rmse_px));
}

TEST(LdmSolver, ThreeAxisPoseNearLimitStaysInsideBoundedStateSpace) {
    const auto camera_config                 = make_test_camera_config();
    auto ldm_config                          = make_ldm_config();
    ldm_config.rmse_stable_threshold_px      = 0.5;
    ldm_config.rmse_constrained_threshold_px = 0.5;
    fcs::L2::ldm::LdmSolver solver(camera_config, ldm_config);

    const double yaw   = 0.78;
    const double pitch = -0.68;
    const double roll  = 0.62;
    ASSERT_LT(std::abs(yaw), ldm_config.max_pose_angle_rad);
    ASSERT_LT(std::abs(pitch), ldm_config.max_pose_angle_rad);
    ASSERT_LT(std::abs(roll), ldm_config.max_pose_angle_rad);

    const Eigen::Matrix3d rotation = bounded_pose_rotation(yaw, pitch, roll);
    const Eigen::Vector3d translation(0.12, -0.05, 4.6);
    auto detection = make_detection_from_ordered_pairs(make_projected_pairs_preserving_model_order(
        {7, 0, 1, 2}, rotation, translation, camera_config, ldm_config));

    const auto result = solver.solve(detection, aligned_odom_camera_transform());
    ASSERT_TRUE(result.has_value());
    ASSERT_TRUE(result->transform_cam.has_value());
    EXPECT_LT((result->transform_cam->translation() - translation).norm(), 8e-2);

    ASSERT_TRUE(result->selected_candidate_idx.has_value());
    const auto& selected =
        result->mesh_candidates[static_cast<size_t>(*result->selected_candidate_idx)];
    EXPECT_TRUE(selected.depth_valid);
    EXPECT_TRUE(std::isfinite(selected.reprojection_rmse_px));
    EXPECT_LT(selected.reprojection_rmse_px, 0.5f);

    const auto angles = bounded_pose_angles(result->transform_cam->rotation());
    ASSERT_TRUE(angles.has_value());
    EXPECT_LE(std::abs((*angles)[0]), ldm_config.max_pose_angle_rad + 1e-9);
    EXPECT_LE(std::abs((*angles)[1]), ldm_config.max_pose_angle_rad + 1e-9);
    EXPECT_LE(std::abs((*angles)[2]), ldm_config.max_pose_angle_rad + 1e-9);
}

TEST(LdmSolver, PoseOutsideBoundedStateSpaceProducesBestEffortOnly) {
    const auto camera_config                 = make_test_camera_config();
    auto ldm_config                          = make_ldm_config();
    ldm_config.rmse_stable_threshold_px      = 0.25;
    ldm_config.rmse_constrained_threshold_px = 0.25;
    fcs::L2::ldm::LdmSolver solver(camera_config, ldm_config);

    const Eigen::Matrix3d rotation = bounded_pose_rotation(0.15, 1.35, 0.10);
    const Eigen::Vector3d translation(0.02, -0.03, 4.3);
    auto detection = make_detection_from_ordered_pairs(make_projected_pairs_preserving_model_order(
        {7, 0, 1, 2}, rotation, translation, camera_config, ldm_config));

    const auto result = solver.solve(detection, aligned_odom_camera_transform());
    ASSERT_TRUE(result.has_value());
    ASSERT_TRUE(result->selected_candidate_idx.has_value());
    const auto& selected =
        result->mesh_candidates[static_cast<size_t>(*result->selected_candidate_idx)];
    EXPECT_TRUE(selected.solved);
    EXPECT_FALSE(selected.depth_valid);
    EXPECT_EQ(result->depth_quality, fcs::L2::ldm::LdmDepthQuality::None);
    EXPECT_TRUE(std::isfinite(selected.reprojection_rmse_px));
}

TEST(LdmSolver, DetectedDatasetTargetsHaveFiniteRmse) {
    const auto camera_config = make_ldm_dataset_camera_config();
    const auto ldm_config    = make_ldm_config();
    fcs::L2::ldm::LdmDetector detector(ldm_config);
    fcs::L2::ldm::LdmSolver solver(camera_config, ldm_config);

    const std::array<const char*, 5> dataset_buckets = {
        "2pair", "2-3pair", "3pair", "3-4pair", "4pair",
    };

    size_t solved_measurement_count = 0;
    for (const char* bucket : dataset_buckets) {
        const auto images = dataset_images(ldm_dataset_root() / bucket);
        ASSERT_FALSE(images.empty()) << bucket;

        for (const auto& image_path : images) {
            cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
            ASSERT_FALSE(image.empty()) << image_path;

            const auto detection_result = detector.detect(image);
            ASSERT_TRUE(detection_result.has_value()) << image_path;
            if (!detection_result->has_value()) {
                continue;
            }

            const auto measurement =
                solver.solve(detection_result->value(), aligned_odom_camera_transform());
            if (!measurement.has_value()) {
                EXPECT_TRUE(false) << image_path << " error=" << measurement.error();
                continue;
            }
            ++solved_measurement_count;

            size_t solved_candidate_count = 0;
            for (size_t candidate_idx = 0; candidate_idx < measurement->mesh_candidates.size();
                 ++candidate_idx) {
                const auto& candidate = measurement->mesh_candidates[candidate_idx];
                if (!candidate.solved) {
                    continue;
                }

                ++solved_candidate_count;
                EXPECT_TRUE(std::isfinite(candidate.reprojection_rmse_px))
                    << image_path << " candidate_idx=" << candidate_idx
                    << " rmse=" << candidate.reprojection_rmse_px;
            }
            EXPECT_GT(solved_candidate_count, 0u) << image_path;
        }
    }
    EXPECT_GT(solved_measurement_count, 0u);
}

TEST(LdmSolver, ThreePairsRecoverStableCenterAndOdomProjection) {
    const auto camera_config = make_test_camera_config();
    const auto ldm_config    = make_ldm_config();
    fcs::L2::ldm::LdmSolver solver(camera_config, ldm_config);

    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(0.22, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d translation(0.14, -0.05, 4.6);
    auto detection = make_detection_from_pairs(
        make_projected_pairs({7, 0, 1}, rotation, translation, camera_config, ldm_config),
        ldm_config);

    const auto T_odom_camera =
        fcs::L2::ldm::LdmSolver::OdomCameraTransform::from_translation(1.0, 2.0, 3.0);
    const auto result = solver.solve(detection, T_odom_camera);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->depth_quality, fcs::L2::ldm::LdmDepthQuality::Stable);
    ASSERT_TRUE(result->transform_cam.has_value());
    ASSERT_TRUE(result->transform_odom.has_value());
    EXPECT_LT((result->transform_cam->translation() - translation).norm(), 5e-2);
    EXPECT_LT(
        (result->transform_odom->translation() - (translation + Eigen::Vector3d(1.0, 2.0, 3.0)))
            .norm(),
        5e-2);

    ASSERT_TRUE(result->selected_candidate_idx.has_value());
    const auto& selected =
        result->mesh_candidates[static_cast<size_t>(*result->selected_candidate_idx)];
    EXPECT_EQ(selected.projected_outline_image.size(), 16u);
    EXPECT_GT(mean_projected_vertical_edge_alignment(selected.projected_outline_image), 0.6);
}

} // namespace
