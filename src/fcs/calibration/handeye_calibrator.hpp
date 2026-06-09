#pragma once

#include "calibration_config.hpp"
#include "calibration_types.hpp"
#include "matrix.hpp"

#include <expected>
#include <opencv2/core.hpp>
#include <vector>

namespace fcs::calibration {

/// @brief Hand-eye calibration for Eye-in-Hand configuration
///
/// Solves AX = XB problem:
/// - A: gimbal_link → odom (from IMU)
/// - B: board → camera_optical (from PnP, converted to ROS frame)
/// - X: camera_link → gimbal_link (result)
class HandEyeCalibrator {
public:
    /// @brief Construct hand-eye calibrator
    /// @param capture Capture configuration
    /// @param handeye Hand-eye method configuration
    explicit HandEyeCalibrator(const CaptureConfig& capture, const HandeyeConfig& handeye) noexcept;

    /// @brief Add pose sample pair
    /// @param gimbal_pose gimbal_link → odom transform (from IMU)
    /// @param board_pose board → camera_link transform (from PnP, converted to ROS frame)
    /// @param ts Timestamp
    /// @return Success or error message
    [[nodiscard]] std::expected<void, std::string> add_sample(
        const fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch>& gimbal_pose,
        const fast_tf::TransformMatrixd<fast_tf::camera, calibration_board_frame>& board_pose,
        timestamp_ns_t ts) noexcept;

    /// @brief Check if pose is diverse enough from existing samples
    /// @param gimbal_pose New gimbal pose to check
    /// @return true if sample adds diversity
    [[nodiscard]] bool is_diverse_enough(
        const fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch>& gimbal_pose)
        const noexcept;

    /// @brief Run hand-eye calibration with collected samples
    /// @return Calibration result or error message
    [[nodiscard]] std::expected<HandEyeResult, std::string> calibrate() noexcept;

    /// @brief Get current sample count
    [[nodiscard]] uint32_t sample_count() const noexcept {
        return static_cast<uint32_t>(samples_.size());
    }

    /// @brief Clear all collected samples
    void clear() noexcept;

    /// @brief Convert OpenCV pose to ROS coordinate frame
    /// OpenCV: X-right, Y-down, Z-forward
    /// ROS: X-forward, Y-left, Z-up
    /// The conversion applies camera_optical → camera_link transform: rpy[-π/2, 0, -π/2]
    [[nodiscard]] static fast_tf::TransformMatrixd<fast_tf::camera, calibration_board_frame>
        opencv_to_ros(const cv::Vec3d& rvec, const cv::Vec3d& tvec) noexcept;

private:
    CaptureConfig capture_config_;
    HandeyeConfig handeye_config_;
    std::vector<PoseSample> samples_;
};

/// @brief Save hand-eye calibration result to TOML file
/// @param result Calibration result
/// @param path Output file path
/// @param method Method used for calibration
/// @return Success or error message
[[nodiscard]] std::expected<void, std::string> save_handeye_result(
    const HandEyeResult& result, const std::string& path, HandEyeMethod method) noexcept;

/// @brief Print human-readable calibration result to console
void print_handeye_result(const HandEyeResult& result) noexcept;

} // namespace fcs::calibration
