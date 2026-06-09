#pragma once

#include "calibration_board.hpp"
#include "calibration_config.hpp"
#include "calibration_types.hpp"
#include "charuco_detector.hpp"
#include "chessboard_detector.hpp"
#include "handeye_calibrator.hpp"
#include "intrinsic_calibrator.hpp"

#include "L1_sensor/parcel.hpp"
#include "core/channel_topics.hpp"
#include "foxglove_server.hpp"
#include "foxglove_types.hpp"
#include "frame.hpp"
#include "scheduler/scheduler.hpp"

#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include <thread>
#include <utility.hpp>

namespace fcs::calibration {

/// @brief Register calibration systems with the scheduler
/// @tparam App Scheduler application type
/// @param app Scheduler application
/// @param config Calibration configuration
void register_calibration_systems(
    talos::scheduler::Scheduler& app, std::shared_ptr<CalibrationConfig> config,
    std::shared_ptr<CalibrationBoard> board,
    std::shared_ptr<IntrinsicCalibrator> intrinsic_calibrator,
    std::shared_ptr<HandEyeCalibrator> handeye_calibrator,
    std::shared_ptr<IntrinsicResult> intrinsic_result) {

    // 1. Corner detection system (pool_compute)
    // Detects calibration board corners in incoming images
    app.add_system<talos::pool_compute>(
        "calibration_corner_detector",
        [board](
            talos::spmc<ImageFrame, ImageChannelTopic> img_in,
            talos::spmc_mut<CornerDetection, CalibrationCornerChannelTopic> corner_out,
            talos::res<std::shared_ptr<CalibrationStatus>> status) {
            if (!img_in.has_new()) {
                return;
            }

            auto frame = img_in.read();
            if (!frame || frame->image.empty()) {
                return;
            }

            // Only detect if in capturing state
            if ((*status)->state != CalibrationState::Capturing) {
                return;
            }

            auto detection = board->detect(frame->image, frame->timestamp_ns);
            if (detection) {
                corner_out.write(std::move(*detection));
                SPDLOG_DEBUG("Corner detection successful");
            }
        });

    // 2. Intrinsic calibration sample collector (pool_compute)
    app.add_system<talos::pool_compute>(
        "calibration_intrinsic_collector",
        [intrinsic_calibrator, config](
            talos::spmc<CornerDetection, CalibrationCornerChannelTopic> corner_in,
            talos::res_mut<std::shared_ptr<CalibrationStatus>> status) {
            if (!corner_in.has_new()) {
                return;
            }

            // Only collect if in intrinsic mode and capturing
            if (config->mode != CalibrationMode::Intrinsic
                && config->mode != CalibrationMode::Full) {
                return;
            }

            if ((*status)->state != CalibrationState::Capturing) {
                return;
            }

            auto detection = corner_in.read();
            if (!detection || !detection->success) {
                return;
            }

            using namespace std::chrono_literals;
            std::this_thread::sleep_for(3000ms);
            // Check diversity
            if (!intrinsic_calibrator->is_diverse_enough(*detection)) {
                SPDLOG_DEBUG("Sample rejected: not diverse enough");
                return;
            }

            // Add sample
            if (auto result = intrinsic_calibrator->add_sample(*detection); !result) {
                SPDLOG_WARN("Failed to add sample: {}", result.error());
                return;
            }

            (*status)->sample_count = intrinsic_calibrator->sample_count();
            SPDLOG_INFO(
                "Intrinsic sample collected: {}/{}", (*status)->sample_count,
                (*status)->target_samples);
        });

    // 3. Hand-eye calibration sample collector (pool_compute)
    app.add_system<talos::pool_compute>(
        "calibration_handeye_collector",
        [handeye_calibrator, intrinsic_result, config](
            talos::spmc<CornerDetection, CalibrationCornerChannelTopic> corner_in,
            talos::res<fast_tf::CoordinateSystem> tf_buffer,
            talos::res_mut<std::shared_ptr<CalibrationStatus>> status) {
            if (!corner_in.has_new()) {
                return;
            }

            // Only collect if in handeye mode
            if (config->mode != CalibrationMode::Handeye && config->mode != CalibrationMode::Full) {
                return;
            }

            if ((*status)->state != CalibrationState::Capturing) {
                return;
            }

            auto detection = corner_in.read();
            if (!detection || !detection->success) {
                return;
            }

            // Get gimbal pose from TF buffer
            auto tf_result = fast_tf::lookup<fast_tf::odom, fast_tf::gimbal_pitch>(
                *tf_buffer, detection->timestamp_ns);

            if (!tf_result) {
                SPDLOG_WARN("TF lookup failed for handeye sample");
                return;
            }

            // Check diversity
            if (!handeye_calibrator->is_diverse_enough(*tf_result)) {
                SPDLOG_DEBUG("Handeye sample rejected: not diverse enough");
                return;
            }

            // Solve PnP to get board → camera pose
            cv::Mat camera_matrix, dist_coeffs;
            cv::eigen2cv(intrinsic_result->camera_matrix, camera_matrix);

            dist_coeffs = cv::Mat(1, 5, CV_64F);
            for (int i = 0; i < 5; ++i) {
                dist_coeffs.at<double>(0, i) = intrinsic_result->distort_coefficient(0, i);
            }

            cv::Vec3d rvec, tvec;
            bool pnp_success = cv::solvePnP(
                detection->object_points, detection->image_points, camera_matrix, dist_coeffs, rvec,
                tvec);

            if (!pnp_success) {
                SPDLOG_WARN("PnP solve failed");
                return;
            }

            const auto board_pose = HandEyeCalibrator::opencv_to_ros(rvec, tvec);

            if (auto result =
                    handeye_calibrator->add_sample(*tf_result, board_pose, detection->timestamp_ns);
                !result) {
                SPDLOG_WARN("Failed to add handeye sample: {}", result.error());
                return;
            }

            (*status)->sample_count = handeye_calibrator->sample_count();
            SPDLOG_INFO(
                "Handeye sample collected: {}/{}", (*status)->sample_count,
                (*status)->target_samples);
        });

    // 4. Foxglove visualization system (pool_compute)
    app.add_system<talos::fixed_rate<30>>(
        "calibration_visualization",
        [board, config](
            talos::spmc<CornerDetection, CalibrationCornerChannelTopic> corner_in,
            talos::spmc<ImageFrame, ImageChannelTopic> img_in,
            talos::res<std::shared_ptr<visualization::FoxgloveServer>> server,
            talos::res<std::shared_ptr<CalibrationStatus>> status) {
            if (!(*server)->is_initialized()) {
                return;
            }

            // Prefer corner detection image if available
            cv::Mat vis_img;
            timestamp_ns_t ts = 0;

            if (corner_in.has_new()) {
                auto detection = corner_in.read();
                if (detection && detection->success) {
                    vis_img = board->draw_corners(detection->image, *detection);
                    ts      = detection->timestamp_ns;
                }
            } else if (img_in.has_new()) {
                auto frame = img_in.read();
                if (frame && !frame->image.empty()) {
                    vis_img = frame->image.clone();
                    ts      = frame->timestamp_ns;
                }
            }

            if (vis_img.empty()) {
                return;
            }

            // Add status text overlay
            cv::putText(
                vis_img,
                fmt::format("Samples: {}/{}", (*status)->sample_count, (*status)->target_samples),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            // Compress and send
            std::vector<uint8_t> compressed;
            std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 85};
            if (!cv::imencode(".jpg", vis_img, compressed, params)) {
                return;
            }

            visualization::CalibrationImageMessage msg;
            msg.payload.timestamp = visualization::timestamp_from_ns(ts);
            msg.payload.frame_id  = "calibration_view";
            msg.payload.format    = "jpeg";
            msg.payload.data      = std::vector<std::byte>(
                reinterpret_cast<const std::byte*>(compressed.data()),
                reinterpret_cast<const std::byte*>(compressed.data() + compressed.size()));

            (*server)->enqueue_message(std::move(msg));
        });
}

} // namespace fcs::calibration
