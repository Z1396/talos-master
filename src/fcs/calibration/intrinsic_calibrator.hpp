#pragma once

#include "calibration_config.hpp"
#include "calibration_types.hpp"

#include <expected>
#include <opencv2/core.hpp>
#include <vector>

namespace fcs::calibration {

/// @brief Camera intrinsic calibration
class IntrinsicCalibrator {
public:
    /// @brief Construct intrinsic calibrator
    /// @param config Capture and intrinsic configuration
    explicit IntrinsicCalibrator(
        const CaptureConfig& capture, const IntrinsicConfig& intrinsic) noexcept;

    /// @brief Add calibration sample
    /// @param detection Corner detection result
    /// @return Success or error message
    [[nodiscard]] std::expected<void, std::string>
        add_sample(const CornerDetection& detection) noexcept;

    /// @brief Check if sample is diverse enough from existing samples
    /// @param detection New detection to check
    /// @return true if sample adds diversity
    [[nodiscard]] bool is_diverse_enough(const CornerDetection& detection) const noexcept;

    /// @brief Run calibration with collected samples
    /// @param image_size Image dimensions
    /// @return Calibration result or error message
    [[nodiscard]] std::expected<IntrinsicResult, std::string>
        calibrate(cv::Size image_size) noexcept;

    /// @brief Compute reprojection of corners using calibration result
    /// @param detection Corner detection
    /// @param result Calibration result
    /// @return Reprojected image points
    [[nodiscard]] std::vector<cv::Point2f>
        reproject(const CornerDetection& detection, const IntrinsicResult& result) const noexcept;

    /// @brief Get current sample count
    [[nodiscard]] uint32_t sample_count() const noexcept {
        return static_cast<uint32_t>(samples_.size());
    }

    /// @brief Clear all collected samples
    void clear() noexcept;

private:
    CaptureConfig capture_config_;
    IntrinsicConfig intrinsic_config_;
    std::vector<CornerDetection> samples_;
    std::vector<cv::Vec3d> sample_rvecs_; // For diversity checking
    std::vector<cv::Vec3d> sample_tvecs_;
};

/// @brief Save intrinsic calibration result to TOML file
/// @param result Calibration result
/// @param path Output file path
/// @return Success or error message
[[nodiscard]] std::expected<void, std::string>
    save_intrinsic_result(const IntrinsicResult& result, const std::string& path) noexcept;

} // namespace fcs::calibration
