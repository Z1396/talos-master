#include "calibration/chessboard_detector.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace fcs::calibration {

ChessboardDetector::ChessboardDetector(uint32_t width, uint32_t height, double square_size) noexcept
    : board_size_(static_cast<int>(width), static_cast<int>(height))
    , square_size_(square_size)
    , object_points_() {
    // Generate object points in board coordinate frame
    // Origin at top-left corner, Z=0 plane
    object_points_.reserve(static_cast<size_t>(width) * height);
    for (uint32_t i = 0; i < height; ++i) {
        for (uint32_t j = 0; j < width; ++j) {
            object_points_.emplace_back(
                static_cast<float>(j * square_size), static_cast<float>(i * square_size), 0.0f);
        }
    }
}

std::expected<CornerDetection, std::string>
    ChessboardDetector::detect(const cv::Mat& image, timestamp_ns_t ts) noexcept {
    CornerDetection result;
    result.timestamp_ns  = ts;
    result.object_points = object_points_;
    result.image         = image.clone();

    // Convert to grayscale if needed
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // Find chessboard corners
    int flags = cv::CALIB_CB_FAST_CHECK;

    result.success = cv::findChessboardCorners(gray, board_size_, result.image_points, flags);

    if (!result.success) {
        return std::unexpected("Chessboard pattern not found");
    }

    // Refine corner positions to subpixel accuracy
    cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
    cv::cornerSubPix(gray, result.image_points, cv::Size(11, 11), cv::Size(-1, -1), criteria);

    return result;
}

const std::vector<cv::Point3f>& ChessboardDetector::object_points() const noexcept {
    return object_points_;
}

cv::Mat ChessboardDetector::draw_corners(
    const cv::Mat& image, const CornerDetection& detection) const noexcept {
    cv::Mat output = image.clone();
    if (output.channels() == 1) {
        cv::cvtColor(output, output, cv::COLOR_GRAY2BGR);
    }

    cv::drawChessboardCorners(output, board_size_, detection.image_points, detection.success);

    return output;
}

} // namespace fcs::calibration
