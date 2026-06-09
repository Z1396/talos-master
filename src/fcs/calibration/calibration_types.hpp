#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#undef HAVE_OPENCV_ARUCO
#ifdef HAVE_OPENCV_ARUCO
# include <opencv2/aruco.hpp>
#endif
#include <vector>

#include "core/types.hpp"

namespace fcs::calibration {

struct calibration_board_frame {};

// ============================================================================
// Channel Topics
// ============================================================================

/// @brief Channel tag for corner detection results
struct CalibrationCornerChannelTopic {};

/// @brief Channel tag for calibration status updates
struct CalibrationStatusChannelTopic {};

// ============================================================================
// Enum Classes (magic_enum auto-parses from TOML strings)
// ============================================================================

/// @brief Calibration board type
enum class BoardType : uint8_t {
    Chessboard,  ///< Standard chessboard pattern (TOML: "Chessboard")
    ChArUco,     ///< ChArUco board with ArUco markers (TOML: "ChArUco")
    CirclesGrid, ///< Circles grid pattern (TOML: "CirclesGrid")
};

/// @brief Calibration operation mode
enum class CalibrationMode : uint8_t {
    Intrinsic, ///< Camera intrinsic calibration only
    Handeye,   ///< Hand-eye calibration only (requires intrinsics)
    Full,      ///< Full calibration: intrinsic + hand-eye
};

/// @brief Hand-eye calibration method (maps to cv::HandEyeCalibrationMethod)
enum class HandEyeMethod : uint8_t {
    Tsai,       ///< Tsai-Lenz method (cv::CALIB_HAND_EYE_TSAI)
    Park,       ///< Park method (cv::CALIB_HAND_EYE_PARK)
    Horaud,     ///< Horaud method (cv::CALIB_HAND_EYE_HORAUD)
    Andreff,    ///< Andreff method (cv::CALIB_HAND_EYE_ANDREFF)
    Daniilidis, ///< Daniilidis method (cv::CALIB_HAND_EYE_DANIILIDIS)
};

/// @brief ArUco dictionary type for ChArUco boards
enum class ArucoDictionary : uint8_t {
    DICT_4X4_50,
    DICT_4X4_100,
    DICT_4X4_250,
    DICT_5X5_50,
    DICT_5X5_100,
    DICT_5X5_250,
    DICT_6X6_50,
    DICT_6X6_100,
    DICT_6X6_250,
};

/// @brief Calibration state machine
enum class CalibrationState : uint8_t {
    Idle,        ///< Waiting to start
    Capturing,   ///< Collecting samples
    Calibrating, ///< Running calibration algorithm
    Completed,   ///< Calibration finished successfully
    Failed,      ///< Calibration failed
};

// ============================================================================
// Enum → OpenCV conversion functions
// ============================================================================

/// @brief Convert HandEyeMethod to OpenCV constant
[[nodiscard]] constexpr cv::HandEyeCalibrationMethod to_opencv_method(HandEyeMethod m) noexcept {
    switch (m) {
    case HandEyeMethod::Tsai: return cv::CALIB_HAND_EYE_TSAI;
    case HandEyeMethod::Park: return cv::CALIB_HAND_EYE_PARK;
    case HandEyeMethod::Horaud: return cv::CALIB_HAND_EYE_HORAUD;
    case HandEyeMethod::Andreff: return cv::CALIB_HAND_EYE_ANDREFF;
    case HandEyeMethod::Daniilidis: return cv::CALIB_HAND_EYE_DANIILIDIS;
    }
    return cv::CALIB_HAND_EYE_TSAI; // unreachable with -Wswitch-enum
}

#ifdef HAVE_OPENCV_ARUCO

/// @brief Convert ArucoDictionary to OpenCV constant
[[nodiscard]] constexpr cv::aruco::PredefinedDictionaryType
    to_opencv_dict(ArucoDictionary d) noexcept {
    switch (d) {
    case ArucoDictionary::DICT_4X4_50: return cv::aruco::DICT_4X4_50;
    case ArucoDictionary::DICT_4X4_100: return cv::aruco::DICT_4X4_100;
    case ArucoDictionary::DICT_4X4_250: return cv::aruco::DICT_4X4_250;
    case ArucoDictionary::DICT_5X5_50: return cv::aruco::DICT_5X5_50;
    case ArucoDictionary::DICT_5X5_100: return cv::aruco::DICT_5X5_100;
    case ArucoDictionary::DICT_5X5_250: return cv::aruco::DICT_5X5_250;
    case ArucoDictionary::DICT_6X6_50: return cv::aruco::DICT_6X6_50;
    case ArucoDictionary::DICT_6X6_100: return cv::aruco::DICT_6X6_100;
    case ArucoDictionary::DICT_6X6_250: return cv::aruco::DICT_6X6_250;
    }
    return cv::aruco::DICT_6X6_250; // unreachable
}
#endif

// ============================================================================
// Data Structures
// ============================================================================

/// @brief Corner detection result from calibration board
struct CornerDetection {
    std::vector<cv::Point2f> image_points;  ///< Detected corners in image coordinates
    std::vector<cv::Point3f> object_points; ///< Corresponding 3D points in board frame
    cv::Mat image;                          ///< Original image (for visualization)
    timestamp_ns_t timestamp_ns{0};         ///< Capture timestamp
    bool success{false};                    ///< Detection success flag
};

/// @brief Camera intrinsic calibration result
struct IntrinsicResult {
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix{
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Identity()};
    Eigen::Matrix<double, 1, 5> distort_coefficient{Eigen::Matrix<double, 1, 5>::Zero()};
    double rms_error{0.0};   ///< RMS reprojection error in pixels
    uint32_t num_samples{0}; ///< Number of calibration samples used
    uint32_t width{0};       ///< Image width
    uint32_t height{0};      ///< Image height
};

/// @brief Hand-eye calibration result (gimbal_link → camera_link)
struct HandEyeResult {
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()}; ///< [x, y, z] in meters
    Eigen::Vector3d rpy{Eigen::Vector3d::Zero()};         ///< [roll, pitch, yaw] in radians
    double rms_error{0.0};                                ///< RMS error in meters
    uint32_t num_samples{0};                              ///< Number of pose pairs used
};

/// @brief Calibration status for UI/monitoring
struct CalibrationStatus {
    CalibrationState state{CalibrationState::Idle};
    uint32_t sample_count{0};
    uint32_t target_samples{30};
    double current_error{0.0};
    std::string message;
};

/// @brief Pose sample for hand-eye calibration
struct PoseSample {
    fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::gimbal_pitch> gimbal_pose;
    fast_tf::TransformMatrixd<fast_tf::camera, calibration_board_frame> board_pose;
    timestamp_ns_t timestamp_ns;
};

} // namespace fcs::calibration
