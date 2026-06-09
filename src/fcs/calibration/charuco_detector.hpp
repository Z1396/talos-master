#pragma once

#include "calibration_board.hpp"

#ifdef HAVE_OPENCV_ARUCO
// Include OpenCV core headers first to ensure types are defined
# include <opencv2/aruco.hpp>
# include <opencv2/aruco/charuco.hpp>
# include <opencv2/core.hpp>
#endif

namespace fcs::calibration {

#ifdef HAVE_OPENCV_ARUCO

/// @brief ChArUco board detector (requires OpenCV aruco module)
class CharucoDetector : public CalibrationBoard {
public:
    /// @brief Construct ChArUco detector
    /// @param width Number of squares (width)
    /// @param height Number of squares (height)
    /// @param square_size Square size in meters
    /// @param marker_size ArUco marker size in meters
    /// @param dictionary ArUco dictionary type
    CharucoDetector(
        uint32_t width, uint32_t height, double square_size, double marker_size,
        ArucoDictionary dictionary) noexcept;

    [[nodiscard]] std::expected<CornerDetection, std::string>
        detect(const cv::Mat& image, timestamp_ns_t ts) noexcept override;

    [[nodiscard]] const std::vector<cv::Point3f>& object_points() const noexcept override;

    [[nodiscard]] cv::Mat draw_corners(
        const cv::Mat& image, const CornerDetection& detection) const noexcept override;

    [[nodiscard]] BoardType type() const noexcept override { return BoardType::ChArUco; }

private:
    cv::Size board_size_;
    double square_size_;
    double marker_size_;
    cv::aruco::Dictionary dictionary_;
    cv::aruco::CharucoBoard charuco_board_;
    cv::aruco::DetectorParameters detector_params_;
    std::vector<cv::Point3f> object_points_;
};

#else

/// @brief Stub CharucoDetector when aruco module is not available
class CharucoDetector : public CalibrationBoard {
public:
    CharucoDetector(uint32_t, uint32_t, double, double, ArucoDictionary) noexcept {}

    [[nodiscard]] std::expected<CornerDetection, std::string>
        detect(const cv::Mat&, timestamp_ns_t) noexcept override {
        return std::unexpected("ChArUco support not available (OpenCV aruco module not found)");
    }

    [[nodiscard]] const std::vector<cv::Point3f>& object_points() const noexcept override {
        static std::vector<cv::Point3f> empty;
        return empty;
    }

    [[nodiscard]] cv::Mat
        draw_corners(const cv::Mat& image, const CornerDetection&) const noexcept override {
        return image.clone();
    }

    [[nodiscard]] BoardType type() const noexcept override { return BoardType::ChArUco; }
};

#endif // HAVE_OPENCV_ARUCO

} // namespace fcs::calibration
