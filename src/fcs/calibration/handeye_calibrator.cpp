#include "calibration/handeye_calibrator.hpp"
#include "euler.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <spdlog/spdlog.h>

namespace fcs::calibration {

HandEyeCalibrator::HandEyeCalibrator(
    const CaptureConfig& capture, const HandeyeConfig& handeye) noexcept
    : capture_config_(capture)
    , handeye_config_(handeye)
    , samples_() {
    samples_.reserve(capture.max_samples);
}

fast_tf::TransformMatrixd<fast_tf::camera, calibration_board_frame>
    HandEyeCalibrator::opencv_to_ros(const cv::Vec3d& rvec, const cv::Vec3d& tvec) noexcept {
    // Convert rvec/tvec to 4x4 matrix
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);

    Eigen::Matrix3d R_eigen;
    cv::cv2eigen(R_cv, R_eigen);

    Eigen::Vector3d t_eigen(tvec[0], tvec[1], tvec[2]);

    // Create OpenCV frame transform
    Eigen::Matrix4d T_opencv   = Eigen::Matrix4d::Identity();
    T_opencv.block<3, 3>(0, 0) = R_eigen;
    T_opencv.block<3, 1>(0, 3) = t_eigen;

    // camera_optical → camera_link transform: rpy[-π/2, 0, -π/2]
    // This converts from OpenCV convention to ROS convention
    constexpr double pi = std::numbers::pi;
    auto T_optical_to_link =
        fast_tf::TransformMatrixd<fast_tf::camera, fast_tf::camera_optical>::from_rpy(
            -pi / 2.0, 0.0, -pi / 2.0, 0.0, 0.0, 0.0);

    // OpenCV solvePnP gives board -> camera_optical. Convert to board -> camera_link.
    fast_tf::TransformMatrixd<fast_tf::camera_optical, calibration_board_frame> T_opencv_tf(
        T_opencv);
    return T_optical_to_link * T_opencv_tf;
}

bool HandEyeCalibrator::is_diverse_enough(
    const fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch>& gimbal_pose)
    const noexcept {
    if (samples_.empty()) {
        return true;
    }

    const Eigen::Matrix3d R_new = gimbal_pose.rotation();
    const Eigen::Vector3d t_new = gimbal_pose.translation();
    auto euler_new              = math_fuxk::rpy(R_new);

    constexpr double deg_to_rad = std::numbers::pi / 180.0;
    const double min_angle_rad  = capture_config_.min_angle_diff * deg_to_rad;
    const double min_trans      = capture_config_.min_translation_diff;

    for (const auto& sample : samples_) {
        const Eigen::Matrix3d R_old = sample.gimbal_pose.rotation();
        const Eigen::Vector3d t_old = sample.gimbal_pose.translation();
        auto euler_old              = math_fuxk::rpy(R_old);

        // Angle difference (sum of absolute differences in euler angles)
        double roll_diff  = std::abs(euler_new.roll - euler_old.roll);
        double pitch_diff = std::abs(euler_new.pitch - euler_old.pitch);
        double yaw_diff   = std::abs(euler_new.yaw - euler_old.yaw);
        double angle_diff = roll_diff + pitch_diff + yaw_diff;

        // Translation difference
        double trans_diff = (t_new - t_old).norm();

        if (angle_diff < min_angle_rad && trans_diff < min_trans) {
            return false;
        }
    }

    return true;
}

std::expected<void, std::string> HandEyeCalibrator::add_sample(
    const fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch>& gimbal_pose,
    const fast_tf::TransformMatrixd<fast_tf::camera, calibration_board_frame>& board_pose,
    timestamp_ns_t ts) noexcept {
    if (samples_.size() >= capture_config_.max_samples) {
        return std::unexpected("Maximum sample count reached");
    }

    PoseSample sample;
    sample.gimbal_pose  = gimbal_pose;
    sample.board_pose   = board_pose;
    sample.timestamp_ns = ts;

    samples_.push_back(sample);
    return {};
}

std::expected<HandEyeResult, std::string> HandEyeCalibrator::calibrate() noexcept {
    if (samples_.size() < capture_config_.min_samples) {
        return std::unexpected(
            fmt::format(
                "Not enough samples: {} < {}", samples_.size(), capture_config_.min_samples));
    }

    // Prepare data for cv::calibrateHandEye
    // A: gripper → base (gimbal_link → odom)
    // B: target → camera (board → camera, in ROS frame)
    std::vector<cv::Mat> R_gripper2base, t_gripper2base;
    std::vector<cv::Mat> R_target2cam, t_target2cam;

    R_gripper2base.reserve(samples_.size());
    t_gripper2base.reserve(samples_.size());
    R_target2cam.reserve(samples_.size());
    t_target2cam.reserve(samples_.size());

    for (const auto& sample : samples_) {
        // A: gimbal → odom (directly use)
        Eigen::Matrix3d R_A = sample.gimbal_pose.rotation();
        Eigen::Vector3d t_A = sample.gimbal_pose.translation();

        cv::Mat R_A_cv, t_A_cv;
        cv::eigen2cv(R_A, R_A_cv);
        cv::eigen2cv(t_A, t_A_cv);
        R_gripper2base.push_back(R_A_cv);
        t_gripper2base.push_back(t_A_cv);

        Eigen::Matrix3d R_B = sample.board_pose.rotation();
        Eigen::Vector3d t_B = sample.board_pose.translation();

        cv::Mat R_B_cv, t_B_cv;
        cv::eigen2cv(R_B, R_B_cv);
        cv::eigen2cv(t_B, t_B_cv);
        R_target2cam.push_back(R_B_cv);
        t_target2cam.push_back(t_B_cv);
    }

    // Run hand-eye calibration
    cv::Mat R_cam2gripper, t_cam2gripper;
    cv::HandEyeCalibrationMethod method = to_opencv_method(handeye_config_.method);

    cv::calibrateHandEye(
        R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper,
        method);

    // Convert result to Eigen
    Eigen::Matrix3d R_result;
    Eigen::Vector3d t_result;
    cv::cv2eigen(R_cam2gripper, R_result);
    cv::cv2eigen(t_cam2gripper, t_result);

    // Extract euler angles
    auto euler = math_fuxk::rpy(R_result);

    // Calculate RMS error (AX vs XB consistency)
    double total_error  = 0.0;
    Eigen::Matrix4d X   = Eigen::Matrix4d::Identity();
    X.block<3, 3>(0, 0) = R_result;
    X.block<3, 1>(0, 3) = t_result;

    for (size_t i = 0; i < samples_.size(); ++i) {
        const auto& sample = samples_[i];
        Eigen::Matrix4d A  = sample.gimbal_pose.matrix();
        Eigen::Matrix4d B  = sample.board_pose.matrix();

        Eigen::Matrix4d AX = A * X;
        Eigen::Matrix4d XB = X * B;

        double diff = (AX - XB).norm();
        total_error += diff * diff;
    }
    double rms = std::sqrt(total_error / static_cast<double>(samples_.size()));

    HandEyeResult result;
    result.translation = t_result;
    result.rpy         = Eigen::Vector3d(euler.roll, euler.pitch, euler.yaw);
    result.rms_error   = rms;
    result.num_samples = static_cast<uint32_t>(samples_.size());

    return result;
}

void HandEyeCalibrator::clear() noexcept { samples_.clear(); }

static auto current_datetime_string() noexcept -> std::string {
    auto now      = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm    = *std::localtime(&t);
    char buffer[100];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm);
    return buffer;
}

std::expected<void, std::string> save_handeye_result(
    const HandEyeResult& result, const std::string& path, HandEyeMethod method) noexcept {
    std::ofstream file(path);
    if (!file) {
        return std::unexpected(fmt::format("Failed to open file for writing: {}", path));
    }

    file << std::fixed << std::setprecision(6);

    file << "# Camera Extrinsic Calibration Result (Hand-Eye)\n";
    file << "# RMS Error: " << result.rms_error << " mm\n";
    file << "# Samples: " << result.num_samples << "\n\n";
    file << "# Date: " << current_datetime_string() << "\n";

    file << "[transform]\n";
    file << "# camera_link → gimbal_link\n";
    file << "# Translation: [x, y, z] in mm\n";
    file << "# Rotation: [roll, pitch, yaw] in radians\n";
    file << "translation = [" << result.translation.x() << ", " << result.translation.y() << ", "
         << result.translation.z() << "]\n";
    file << "rotation = [" << result.rpy.x() << ", " << result.rpy.y() << ", " << result.rpy.z()
         << "]\n\n";

    file << "[calibration_info]\n";
    file << "rms_error = " << result.rms_error << "\n";
    file << "num_samples = " << result.num_samples << "\n";
    file << "method = \"" << magic_enum::enum_name(method) << "\"\n";

    return {};
}

void print_handeye_result(const HandEyeResult& result) noexcept {
    constexpr double rad_to_deg = 180.0 / std::numbers::pi;

    SPDLOG_INFO("=== Camera Extrinsic Calibration Result ===");
    SPDLOG_INFO("Transform: gimbal_link → camera_link");
    SPDLOG_INFO("");
    SPDLOG_INFO("Translation:");
    SPDLOG_INFO("  X (forward): {:7.2f} mm", result.translation.x() * 1000.0);
    SPDLOG_INFO("  Y (left):    {:7.2f} mm", result.translation.y() * 1000.0);
    SPDLOG_INFO("  Z (up):      {:7.2f} mm", result.translation.z() * 1000.0);
    SPDLOG_INFO("");
    SPDLOG_INFO("Rotation:");
    SPDLOG_INFO("  Roll:  {:7.2f}°", result.rpy.x() * rad_to_deg);
    SPDLOG_INFO("  Pitch: {:7.2f}°", result.rpy.y() * rad_to_deg);
    SPDLOG_INFO("  Yaw:   {:7.2f}°", result.rpy.z() * rad_to_deg);
    SPDLOG_INFO("");
    SPDLOG_INFO("RMS Error: {:.2f} mm", result.rms_error * 1000.0);
    SPDLOG_INFO("Samples: {}", result.num_samples);
}

} // namespace fcs::calibration
