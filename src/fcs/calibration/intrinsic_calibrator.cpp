#include "calibration/intrinsic_calibrator.hpp"

#include <cmath>
#include <fstream>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace fcs::calibration {

IntrinsicCalibrator::IntrinsicCalibrator(
    const CaptureConfig& capture, const IntrinsicConfig& intrinsic) noexcept
    : capture_config_(capture)
    , intrinsic_config_(intrinsic)
    , samples_()
    , sample_rvecs_()
    , sample_tvecs_() {
    samples_.reserve(capture.max_samples);
    sample_rvecs_.reserve(capture.max_samples);
    sample_tvecs_.reserve(capture.max_samples);
}

bool IntrinsicCalibrator::is_diverse_enough(const CornerDetection& detection) const noexcept {
    if (samples_.empty()) {
        return true;
    }

    // Estimate pose for diversity check (rough estimate using first sample's camera matrix guess)
    cv::Mat camera_matrix          = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0, 0) = 1000.0; // fx guess
    camera_matrix.at<double>(1, 1) = 1000.0; // fy guess
    camera_matrix.at<double>(0, 2) = 640.0;  // cx guess
    camera_matrix.at<double>(1, 2) = 360.0;  // cy guess

    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    cv::Vec3d rvec, tvec;

    bool success = cv::solvePnP(
        detection.object_points, detection.image_points, camera_matrix, dist_coeffs, rvec, tvec);

    if (!success) {
        return false;
    }

    // Check angle difference from all existing samples
    constexpr double deg_to_rad = std::numbers::pi / 180.0;
    const double min_angle_rad  = capture_config_.min_angle_diff * deg_to_rad;
    const double min_trans      = capture_config_.min_translation_diff;

    for (size_t i = 0; i < sample_rvecs_.size(); ++i) {
        // Angle difference (using Rodrigues rotation vector norm)
        cv::Vec3d rvec_diff = rvec - sample_rvecs_[i];
        double angle_diff   = cv::norm(rvec_diff);

        // Translation difference
        cv::Vec3d tvec_diff = tvec - sample_tvecs_[i];
        double trans_diff   = cv::norm(tvec_diff);

        // If too similar to any existing sample, reject
        if (angle_diff < min_angle_rad && trans_diff < min_trans) {
            return false;
        }
    }

    return true;
}

std::expected<void, std::string>
    IntrinsicCalibrator::add_sample(const CornerDetection& detection) noexcept {
    if (!detection.success) {
        return std::unexpected("Detection was not successful");
    }

    if (samples_.size() >= capture_config_.max_samples) {
        return std::unexpected("Maximum sample count reached");
    }

    // Estimate pose for diversity tracking
    cv::Mat camera_matrix          = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0, 0) = 1000.0;
    camera_matrix.at<double>(1, 1) = 1000.0;
    camera_matrix.at<double>(0, 2) = 640.0;
    camera_matrix.at<double>(1, 2) = 360.0;

    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    cv::Vec3d rvec, tvec;

    bool success = cv::solvePnP(
        detection.object_points, detection.image_points, camera_matrix, dist_coeffs, rvec, tvec);

    if (!success) {
        return std::unexpected("Failed to estimate pose for diversity check");
    }

    samples_.push_back(detection);
    sample_rvecs_.push_back(rvec);
    sample_tvecs_.push_back(tvec);

    return {};
}

std::expected<IntrinsicResult, std::string>
    IntrinsicCalibrator::calibrate(cv::Size image_size) noexcept {
    if (samples_.size() < capture_config_.min_samples) {
        return std::unexpected(
            fmt::format(
                "Not enough samples: {} < {}", samples_.size(), capture_config_.min_samples));
    }

    // Prepare calibration data
    std::vector<std::vector<cv::Point3f>> object_points_list;
    std::vector<std::vector<cv::Point2f>> image_points_list;

    object_points_list.reserve(samples_.size());
    image_points_list.reserve(samples_.size());

    for (const auto& sample : samples_) {
        object_points_list.push_back(sample.object_points);
        image_points_list.push_back(sample.image_points);
    }

    // Initialize camera matrix
    cv::Mat camera_matrix          = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0, 2) = image_size.width / 2.0;
    camera_matrix.at<double>(1, 2) = image_size.height / 2.0;

    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;

    // Run calibration
    int flags  = intrinsic_config_.to_opencv_flags();
    double rms = cv::calibrateCamera(
        object_points_list, image_points_list, image_size, camera_matrix, dist_coeffs, rvecs, tvecs,
        flags);

    // Convert to result
    IntrinsicResult result;
    cv::cv2eigen(camera_matrix, result.camera_matrix);

    // dist_coeffs is 5x1, convert to 1x5
    for (int i = 0; i < 5; ++i) {
        result.distort_coefficient(0, i) = dist_coeffs.at<double>(i, 0);
    }

    result.rms_error   = rms;
    result.num_samples = static_cast<uint32_t>(samples_.size());
    result.width       = static_cast<uint32_t>(image_size.width);
    result.height      = static_cast<uint32_t>(image_size.height);

    return result;
}

std::vector<cv::Point2f> IntrinsicCalibrator::reproject(
    const CornerDetection& detection, const IntrinsicResult& result) const noexcept {
    std::vector<cv::Point2f> reprojected;

    cv::Mat camera_matrix;
    cv::eigen2cv(result.camera_matrix, camera_matrix);

    cv::Mat dist_coeffs(1, 5, CV_64F);
    for (int i = 0; i < 5; ++i) {
        dist_coeffs.at<double>(0, i) = result.distort_coefficient(0, i);
    }

    cv::Vec3d rvec, tvec;
    bool success = cv::solvePnP(
        detection.object_points, detection.image_points, camera_matrix, dist_coeffs, rvec, tvec);

    if (!success) {
        return reprojected;
    }

    cv::projectPoints(detection.object_points, rvec, tvec, camera_matrix, dist_coeffs, reprojected);

    return reprojected;
}

void IntrinsicCalibrator::clear() noexcept {
    samples_.clear();
    sample_rvecs_.clear();
    sample_tvecs_.clear();
}

static auto current_datetime_string() noexcept -> std::string {
    auto now      = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm    = *std::localtime(&t);
    char buffer[100];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm);
    return buffer;
}

std::expected<void, std::string>
    save_intrinsic_result(const IntrinsicResult& result, const std::string& path) noexcept {
    std::ofstream file(path);
    if (!file) {
        return std::unexpected(fmt::format("Failed to open file for writing: {}", path));
    }

    // Format as TOML
    file << "# Camera Intrinsic Calibration Result\n";
    file << "# RMS Error: " << result.rms_error << " pixels\n";
    file << "# Samples: " << result.num_samples << "\n\n";
    file << "# Date: " << current_datetime_string() << "\n";

    file << "[camera]\n";

    // Write camera matrix as flat array (row-major)
    file << "camera_matrix = [";
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            file << result.camera_matrix(i, j);
            if (i < 2 || j < 2)
                file << ", ";
        }
    }
    file << "]\n";

    // Write distortion coefficients
    file << "distort_coefficient = [";
    for (int i = 0; i < 5; ++i) {
        file << result.distort_coefficient(0, i);
        if (i < 4)
            file << ", ";
    }
    file << "]\n";

    file << "width = " << result.width << "\n";
    file << "height = " << result.height << "\n\n";

    file << "[calibration_info]\n";
    file << "rms_error = " << result.rms_error << "\n";
    file << "num_samples = " << result.num_samples << "\n";

    return {};
}

} // namespace fcs::calibration
