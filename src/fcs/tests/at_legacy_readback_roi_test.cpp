#include <gtest/gtest.h>

#include "L2_perception/armor/readback_roi.hpp"

namespace {

TEST(ReadbackRoi, ResolveMaintainsAspectAndMinimumSize) {
    const cv::Size frame_size(1440, 1080);
    const cv::Rect2f raw_roi(100.0f, 120.0f, 80.0f, 40.0f);
    fcs::L2::ArmorReadbackRoiConfig config;
    config.margin_ratio_x = 0.10;
    config.margin_ratio_y = 0.10;

    const auto resolved =
        fcs::L2::resolve_readback_roi(frame_size, raw_roi, config, {.width = 640, .height = 640});
    ASSERT_TRUE(resolved.has_value());

    EXPECT_GE(resolved->width, 640);
    EXPECT_GE(resolved->height, 640);
    EXPECT_EQ(resolved->width, resolved->height);
    EXPECT_GE(resolved->x, 0);
    EXPECT_GE(resolved->y, 0);
    EXPECT_LE(resolved->x + resolved->width, frame_size.width);
    EXPECT_LE(resolved->y + resolved->height, frame_size.height);
}

TEST(ReadbackRoi, ResolveShiftsOutOfBoundsRoiBackIntoFrame) {
    const cv::Size frame_size(1440, 1080);
    const cv::Rect2f raw_roi(1300.0f, 900.0f, 120.0f, 120.0f);
    fcs::L2::ArmorReadbackRoiConfig config;
    config.margin_ratio_x = 0.0;
    config.margin_ratio_y = 0.0;

    const auto resolved =
        fcs::L2::resolve_readback_roi(frame_size, raw_roi, config, {.width = 416, .height = 416});
    ASSERT_TRUE(resolved.has_value());

    EXPECT_EQ(resolved->x + resolved->width, frame_size.width);
    EXPECT_EQ(resolved->y + resolved->height, frame_size.height);
}

TEST(ReadbackRoi, ResolveReturnsNulloptWhenMinInputCannotFitFrame) {
    const cv::Size frame_size(640, 480);
    const cv::Rect2f raw_roi(10.0f, 10.0f, 50.0f, 50.0f);
    const auto resolved = fcs::L2::resolve_readback_roi(
        frame_size, raw_roi, fcs::L2::ArmorReadbackRoiConfig{}, {.width = 896, .height = 672});
    EXPECT_FALSE(resolved.has_value());
}

TEST(ReadbackRoi, ExpandUsesIndependentMarginsForXAndY) {
    const cv::Rect2f raw_roi(100.0f, 200.0f, 50.0f, 80.0f);
    fcs::L2::ArmorReadbackRoiConfig config;
    config.margin_ratio_x = 0.20;
    config.margin_ratio_y = 0.50;

    const auto expanded = fcs::L2::expand_raw_roi(raw_roi, config);
    EXPECT_FLOAT_EQ(expanded.x, 90.0f);
    EXPECT_FLOAT_EQ(expanded.y, 160.0f);
    EXPECT_FLOAT_EQ(expanded.width, 70.0f);
    EXPECT_FLOAT_EQ(expanded.height, 160.0f);
}

TEST(ReadbackRoi, ProjectBoxToImageProducesFiniteRoi) {
    fcs::CameraConfig camera_config;
    camera_config.camera_matrix << 100.0, 0.0, 50.0, 0.0, 100.0, 60.0, 0.0, 0.0, 1.0;

    const auto T_camera_odom =
        fast_tf::TransformMatrixd<fast_tf::camera_optical, fast_tf::odom>::from_translation(
            0.0, 0.0, 0.0);
    const auto roi = fcs::L2::project_box_to_image(
        Eigen::Vector3d(0.0, 0.0, 10.0), 0.0, T_camera_odom, camera_config, {0.8, 0.8, 0.6});

    ASSERT_TRUE(roi.has_value());
    EXPECT_GT(roi->width, 0.0f);
    EXPECT_GT(roi->height, 0.0f);
    EXPECT_LT(roi->x, 50.0f);
    EXPECT_GT(roi->x + roi->width, 50.0f);
    EXPECT_LT(roi->y, 60.0f);
    EXPECT_GT(roi->y + roi->height, 60.0f);
}

} // namespace
