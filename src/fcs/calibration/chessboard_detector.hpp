#pragma once

#include "calibration_board.hpp"

namespace fcs::calibration {

/// @brief Chessboard pattern detector
class ChessboardDetector : public CalibrationBoard {
public:
    /// @brief Construct chessboard detector
    /// @param width Number of inner corners (width)
    /// @param height Number of inner corners (height)
    /// @param square_size Square size in meters
    ChessboardDetector(uint32_t width, uint32_t height, double square_size) noexcept;

    [[nodiscard]] std::expected<CornerDetection, std::string>
        detect(const cv::Mat& image, timestamp_ns_t ts) noexcept override;

    [[nodiscard]] const std::vector<cv::Point3f>& object_points() const noexcept override;

    [[nodiscard]] cv::Mat draw_corners(
        const cv::Mat& image, const CornerDetection& detection) const noexcept override;

    [[nodiscard]] BoardType type() const noexcept override { return BoardType::Chessboard; }

private:
    cv::Size board_size_;
    double square_size_;
    std::vector<cv::Point3f> object_points_;
};

} // namespace fcs::calibration
