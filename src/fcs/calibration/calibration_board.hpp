#pragma once

#include "calibration_config.hpp"
#include "calibration_types.hpp"

#include <expected>
#include <memory>
#include <opencv2/core.hpp>

namespace fcs::calibration {

/// @brief Abstract interface for calibration board detection
class CalibrationBoard {
public:
    virtual ~CalibrationBoard() = default;

    /// @brief Detect calibration pattern in image
    /// @param image Input image (grayscale or BGR)
    /// @param ts Capture timestamp
    /// @return Detection result or error message
    [[nodiscard]] virtual std::expected<CornerDetection, std::string>
        detect(const cv::Mat& image, timestamp_ns_t ts) noexcept = 0;

    /// @brief Get object points in board coordinate frame
    [[nodiscard]] virtual const std::vector<cv::Point3f>& object_points() const noexcept = 0;

    /// @brief Draw detected corners on image (for visualization)
    /// @param image Input image
    /// @param detection Detection result
    /// @return Image with corners drawn
    [[nodiscard]] virtual cv::Mat
        draw_corners(const cv::Mat& image, const CornerDetection& detection) const noexcept = 0;

    /// @brief Get board type
    [[nodiscard]] virtual BoardType type() const noexcept = 0;
};

/// @brief Factory function to create calibration board detector
/// @param config Board configuration
/// @return Board detector or error message
[[nodiscard]] std::expected<std::unique_ptr<CalibrationBoard>, std::string>
    create_board(const BoardConfig& board_config, const CharucoConfig& charuco_config) noexcept;

} // namespace fcs::calibration
