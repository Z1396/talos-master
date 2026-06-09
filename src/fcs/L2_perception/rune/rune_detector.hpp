
#ifndef RUNE_DETECTOR_RUNE_DETECTOR_HPP_
#define RUNE_DETECTOR_RUNE_DETECTOR_HPP_

// std
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "L2_perception/rune/rune_config.hpp"
#include "core/armor_types.hpp"
#include "frame.hpp"
#include "matrix.hpp"
#include <expected>
#include <string>

#include "L2_perception/rune/rune_detector_types.hpp"

namespace fyt::rune {

// ============================================================================
// Constrained Reprojection Error for Rune (5DOF: roll + yaw + translation)
// ============================================================================
// R_cam_rune = R_cam_imu * R_z(yaw) * R_x(roll)
// Pitch is constrained to 0 — rune plane has no tilt.

struct RuneConstrainedReprojError {
    RuneConstrainedReprojError(
        const Eigen::Vector3d& Pw, const Eigen::Vector2d& uv, const Eigen::Matrix3d& R_cam_imu)
        : Pw_(Pw)
        , uv_(uv)
        , R_cam_imu_(R_cam_imu) {}

    template <typename T>
    bool operator()(
        const T* const roll, const T* const yaw, const T* const translation, T* residuals) const {
        // R_x(roll)
        const T cr = ceres::cos(roll[0]);
        const T sr = ceres::sin(roll[0]);
        Eigen::Matrix<T, 3, 3> R_roll;
        R_roll << T(1), T(0), T(0), T(0), cr, -sr, T(0), sr, cr;

        // R_z(yaw)
        const T cy = ceres::cos(yaw[0]);
        const T sy = ceres::sin(yaw[0]);
        Eigen::Matrix<T, 3, 3> R_yaw;
        R_yaw << cy, -sy, T(0), sy, cy, T(0), T(0), T(0), T(1);

        // R = R_cam_imu * R_z(yaw) * R_x(roll)
        Eigen::Matrix<T, 3, 3> R_cam_imu = R_cam_imu_.template cast<T>();
        Eigen::Matrix<T, 3, 3> R         = R_cam_imu * R_yaw * R_roll;

        // 3D model point (rune frame)
        Eigen::Matrix<T, 3, 1> Pw;
        Pw << T(Pw_.x()), T(Pw_.y()), T(Pw_.z());

        // Translation in camera frame
        Eigen::Matrix<T, 3, 1> t;
        t << translation[0], translation[1], translation[2];

        // Transform to camera frame
        Eigen::Matrix<T, 3, 1> Pc = R * Pw + t;
        const T& Xc               = Pc(0);
        const T& Yc               = Pc(1);
        const T& Zc               = Pc(2);

        constexpr double kMinDepth = 1e-3;
        if (Zc < T(kMinDepth)) {
            return false;
        }

        // Normalized image plane coordinates
        residuals[0] = Xc / Zc - T(uv_.x());
        residuals[1] = Yc / Zc - T(uv_.y());
        return true;
    }

    // Ceres takes ownership via AddResidualBlock
    static ceres::CostFunction* Create(
        const Eigen::Vector3d& Pw, const Eigen::Vector2d& uv, const Eigen::Matrix3d& R_cam_imu) {
        return new ceres::AutoDiffCostFunction<RuneConstrainedReprojError, 2, 1, 1, 3>(
            new RuneConstrainedReprojError(Pw, uv, R_cam_imu));
    }

private:
    Eigen::Vector3d Pw_;
    Eigen::Vector2d uv_;
    Eigen::Matrix3d R_cam_imu_;
};

[[nodiscard]] inline Eigen::Matrix3d rune_roll_rotation(double roll) noexcept {
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    Eigen::Matrix3d R;
    R << 1.0, 0.0, 0.0, 0.0, cr, -sr, 0.0, sr, cr;
    return R;
}

[[nodiscard]] inline Eigen::Matrix3d rune_yaw_rotation(double yaw) noexcept {
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    Eigen::Matrix3d R;
    R << cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0;
    return R;
}

class RuneDetector {
public:
    struct PosePrior {
        cv::Mat rvec;
        cv::Mat tvec;
        double timestamp{0.0};
    };

    explicit RuneDetector(const fcs::rune::RuneDetectorConfig& params = {});

    bool detect(const cv::Mat& frame, fcs::ArmorColor color);

    std::expected<fast_tf::TransformMatrixd<rune, fast_tf::odom>, std::string> solve(
        const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, double timestamp_sec,
        const fast_tf::TransformMatrixd<fast_tf::odom, fast_tf::camera_optical>& odom_T_cam);

    void setKeyPoints();
    void setLocalRoi();

    cv::Mat localMask;
    std::vector<cv::Rect2f> targetROIs;
    cv::Rect2f centerRoi;
    cv::Mat arrowImg;
    cv::Mat targetImg;
    cv::Mat rCenterImg;
    CenterR rcenter;
    std::vector<Target> targets;
    std::vector<Arrow> arrows;
    Status status;
    double timestamp_sec{0.0};

private:
    void processPictures(const cv::Mat& src, fcs::ArmorColor color);
    bool isArrowRoiMatched(const Arrow& arrow, const cv::Rect2f& roi);

    bool detectCenterR();
    bool detectAllArrows();
    bool detectAllTargets();

    int image_width_;
    int image_height_;
    fcs::rune::RuneDetectorConfig params_;

    // 上一帧追踪的靶信息（用于目标连续性）
    cv::Point2f last_tracked_target_center_{0, 0};
    double last_tracked_target_angle_{0.0};
    double last_timestamp_ = 0;
    bool has_last_target_{false};

    // PnP prior from previous frame
    PosePrior pose_prior_;

    std::vector<cv::Point> r_template_norm_;
    cv::Mat close_kernel_;

public:
    // 重投影调试信息
    std::vector<cv::Point2f> debug_detected_points_;
    std::vector<cv::Point2f> debug_reprojected_points_;
    double debug_reproj_error_{0.0};
};
} // namespace fyt::rune
#endif // RUNE_DETECTOR_RUNE_DETECTOR_HPP_
