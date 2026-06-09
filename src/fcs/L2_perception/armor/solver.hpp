#pragma once

#include "core/types.hpp"
#include "core/types_pnp.hpp"

#include <algorithm>
#include <cmath>
#include <expected>
#include <limits>
#include <numbers>
#include <opencv2/core/matx.hpp>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "camera_config.hpp"
#include "euler.hpp"

namespace fcs::L2 {

// ============================================================================
// PnP Solver Error Types
// ============================================================================

enum class PnPError {
    InvalidDetection,
    SolveFailed,
};

[[nodiscard]] constexpr std::string_view pnp_error_str(PnPError e) noexcept {
    switch (e) {
    case PnPError::InvalidDetection: return "Invalid detection";
    case PnPError::SolveFailed: return "Solve failed";
    }
    return "Unknown error";
}

// ============================================================================
// Constrained Reprojection Error (4DOF Optimization)
// ============================================================================

/// Ceres cost function for constrained reprojection refinement.
/// Optimizes armor yaw in the odom/imu frame plus camera-frame translation
/// parameterized as optical-frame yaw/pitch/distance. Armor pitch stays fixed.
struct ConstrainedReprojError {
    ConstrainedReprojError(
        const Eigen::Vector3d& Pw, const Eigen::Vector2d& uv, const Eigen::Matrix3d& R_cam_imu,
        const Eigen::Matrix3d& R_pitch)
        : Pw_(Pw)
        , uv_(uv)
        , R_cam_imu_(R_cam_imu)
        , R_pitch_(R_pitch) {}

    template <typename T>
    bool operator()(const T* const yaw, const T* const translation_ypd, T* residuals) const {
        // R_yaw: rotation around Z axis in IMU frame
        const T cy = ceres::cos(yaw[0]);
        const T sy = ceres::sin(yaw[0]);
        Eigen::Matrix<T, 3, 3> R_yaw;
        R_yaw << cy, -sy, T(0), sy, cy, T(0), T(0), T(0), T(1);

        // R = R_cam_imu * R_yaw * R_pitch
        Eigen::Matrix<T, 3, 3> R_cam_imu = R_cam_imu_.template cast<T>();
        Eigen::Matrix<T, 3, 3> R_pitch   = R_pitch_.template cast<T>();
        Eigen::Matrix<T, 3, 3> R         = R_cam_imu * R_yaw * R_pitch;

        // 3D model point (armor frame)
        Eigen::Matrix<T, 3, 1> Pw;
        Pw << T(Pw_.x()), T(Pw_.y()), T(Pw_.z());

        // Translation in camera optical frame, parameterized as:
        // yaw = atan2(x, z), pitch = atan2(y, hypot(x, z)), distance = ||t||.
        const T t_yaw      = translation_ypd[0];
        const T t_pitch    = translation_ypd[1];
        const T t_distance = translation_ypd[2];
        const T cp         = ceres::cos(t_pitch);
        Eigen::Matrix<T, 3, 1> t;
        t << t_distance * cp * ceres::sin(t_yaw), t_distance * ceres::sin(t_pitch),
            t_distance * cp * ceres::cos(t_yaw);

        // Transform to camera frame
        Eigen::Matrix<T, 3, 1> Pc = R * Pw + t;
        const T& Xc               = Pc(0);
        const T& Yc               = Pc(1);
        const T& Zc               = Pc(2);

        // Reject points behind camera or too close (avoids division by ~0 → NaN)
        constexpr double kMinDepth = 1e-3; // 1mm minimum depth
        if (Zc < T(kMinDepth)) {
            return false;
        }

        // Normalized image plane coordinates
        residuals[0] = Xc / Zc - T(uv_.x());
        residuals[1] = Yc / Zc - T(uv_.y());
        return true;
    }

    // ====================================================================
    // CERES SOLVER OWNERSHIP TRANSFER (RAII-compliant)
    // ====================================================================
    //
    // Ceres Problem takes ownership of CostFunction and LossFunction pointers
    // via Problem::AddResidualBlock(). The Problem destructor deletes them.
    //
    // Source: ceres-solver/include/ceres/problem.h
    //   "If set to TAKE_OWNERSHIP (default), the problem object will delete
    //    the corresponding object on destruction."
    //
    // Therefore:
    //   - We allocate with new (required by Ceres API)
    //   - We transfer ownership via AddResidualBlock()
    //   - We DO NOT delete (Ceres Problem destructor handles it)
    //   - NO LEAKS, NO DOUBLE-FREE
    //
    // This is NOT like tinympc - Ceres is RAII-compliant~ ♪
    // ====================================================================
    static ceres::CostFunction* Create(
        const Eigen::Vector3d& Pw, const Eigen::Vector2d& uv, const Eigen::Matrix3d& R_cam_imu,
        const Eigen::Matrix3d& R_pitch) {
        return new ceres::AutoDiffCostFunction<ConstrainedReprojError, 2, 1, 3>(
            new ConstrainedReprojError(Pw, uv, R_cam_imu, R_pitch));
    }

private:
    Eigen::Vector3d Pw_;
    Eigen::Vector2d uv_;
    Eigen::Matrix3d R_cam_imu_;
    Eigen::Matrix3d R_pitch_;
};

[[nodiscard]] inline Eigen::Vector3d
    camera_translation_to_ypd(const Eigen::Vector3d& translation) noexcept {
    const double horizontal = std::hypot(translation.x(), translation.z());
    return {
        std::atan2(translation.x(), translation.z()),
        std::atan2(translation.y(), horizontal),
        translation.norm(),
    };
}

[[nodiscard]] inline Eigen::Vector3d
    camera_ypd_to_translation(const Eigen::Vector3d& translation_ypd) noexcept {
    const double yaw      = translation_ypd.x();
    const double pitch    = translation_ypd.y();
    const double distance = translation_ypd.z();
    const double cp       = std::cos(pitch);

    return {
        distance * cp * std::sin(yaw),
        distance * std::sin(pitch),
        distance * cp * std::cos(yaw),
    };
}

[[nodiscard]] inline double armor_pitch_rad_for(ArmorName name) noexcept {
    constexpr double kTiltRad = 15.0 * std::numbers::pi / 180.0;
    return (name == ArmorName::Outpost) ? -kTiltRad : kTiltRad;
}

[[nodiscard]] inline Eigen::Matrix3d armor_pitch_rotation(double armor_pitch_rad) noexcept {
    const double cp = std::cos(armor_pitch_rad);
    const double sp = std::sin(armor_pitch_rad);
    Eigen::Matrix3d R_pitch;
    R_pitch << cp, 0.0, sp, 0.0, 1.0, 0.0, -sp, 0.0, cp;
    return R_pitch;
}

[[nodiscard]] inline Eigen::Matrix3d yaw_rotation(double yaw) noexcept {
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    Eigen::Matrix3d R_yaw;
    R_yaw << cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0;
    return R_yaw;
}

[[nodiscard]] inline std::vector<cv::Point2f>
    debias_correlated_corner_scale(std::vector<cv::Point2f> img_points) noexcept {
    if (img_points.size() != 4) {
        return img_points;
    }

    cv::Point2f center(0.0f, 0.0f);
    for (const auto& p : img_points) {
        center += p;
    }
    center *= 0.25f;

    // Detector keypoints tend to sit slightly inside bright light-bar corners.
    // A fixed pixel-domain outward correction removes the non-zero mean scale
    // bias; the remaining uncertainty is still modeled in covariance below.
    constexpr float kCorrelatedScaleDebiasPx = 0.65f;
    for (auto& p : img_points) {
        const cv::Point2f radial = p - center;
        const float norm         = std::sqrt(radial.x * radial.x + radial.y * radial.y);
        if (std::isfinite(norm) && norm > 1e-6f) {
            p += (kCorrelatedScaleDebiasPx / norm) * radial;
        }
    }
    return img_points;
}

/// Reprojection Jacobian with respect to x = [armor_yaw, bearing_yaw, bearing_pitch, log_distance].
[[nodiscard]] static Eigen::Matrix<double, 8, 4> compute_reprojection_jacobian_x(
    const std::vector<Eigen::Vector3d>& obj_pts, const Eigen::Matrix3d& R_cam_imu,
    const Eigen::Matrix3d& R_pitch, double armor_yaw, const Eigen::Vector3d& ypd) noexcept {
    Eigen::Matrix<double, 8, 4> J;
    J.setZero();
    if (obj_pts.size() != 4) {
        J.setConstant(std::numeric_limits<double>::quiet_NaN());
        return J;
    }

    const double r = armor_yaw;
    const double a = ypd.x();
    const double b = ypd.y();
    const double d = ypd.z();
    if (!(d > 1e-9) || !std::isfinite(d)) {
        J.setConstant(std::numeric_limits<double>::quiet_NaN());
        return J;
    }
    const double cr = std::cos(r);
    const double sr = std::sin(r);

    Eigen::Matrix3d R_yaw;
    R_yaw << cr, -sr, 0.0, sr, cr, 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix3d dR_yaw_dr;
    dR_yaw_dr << -sr, -cr, 0.0, cr, -sr, 0.0, 0.0, 0.0, 0.0;

    const Eigen::Matrix3d R        = R_cam_imu * R_yaw * R_pitch;
    const Eigen::Matrix3d dR_dr    = R_cam_imu * dR_yaw_dr * R_pitch;
    const double sa                = std::sin(a);
    const double ca                = std::cos(a);
    const double sb                = std::sin(b);
    const double cb                = std::cos(b);
    const Eigen::Vector3d dt_da    = {d * cb * ca, 0.0, -d * cb * sa};
    const Eigen::Vector3d dt_db    = {-d * sb * sa, d * cb, -d * sb * ca};
    const Eigen::Vector3d dt_dlogd = {d * cb * sa, d * sb, d * cb * ca};

    for (int i = 0; i < 4; ++i) {
        const Eigen::Vector3d& Pw = obj_pts[static_cast<size_t>(i)];
        const Eigen::Vector3d Pc  = R * Pw + Eigen::Vector3d{d * cb * sa, d * sb, d * cb * ca};
        const double X            = Pc.x();
        const double Y            = Pc.y();
        const double Z            = Pc.z();
        if (!(Z > 1e-9) || !Pc.allFinite()) {
            J.setConstant(std::numeric_limits<double>::quiet_NaN());
            return J;
        }

        Eigen::Matrix<double, 2, 3> J_proj;
        J_proj << 1.0 / Z, 0.0, -X / (Z * Z), 0.0, 1.0 / Z, -Y / (Z * Z);

        J.block<2, 1>(2 * i, 0) = J_proj * (dR_dr * Pw);
        J.block<2, 1>(2 * i, 1) = J_proj * dt_da;
        J.block<2, 1>(2 * i, 2) = J_proj * dt_db;
        J.block<2, 1>(2 * i, 3) = J_proj * dt_dlogd;
    }
    return J;
}

struct PnpGeometryInfo {
    Eigen::Matrix4d cov_ypdr{Eigen::Matrix4d::Identity() * 1e6};
    double condition_number{1e6};
};

template <int N>
[[nodiscard]] static Eigen::Matrix<double, N, N> pseudo_inverse_spd(
    const Eigen::Matrix<double, N, N>& H, double relative_threshold = 1e-9) noexcept {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N>> es(H);
    if (es.info() != Eigen::Success || !H.allFinite()) {
        return Eigen::Matrix<double, N, N>::Identity() * 1e6;
    }

    const auto evals        = es.eigenvalues();
    const auto evecs        = es.eigenvectors();
    const double lambda_max = std::max(evals.maxCoeff(), 1e-18);
    const double threshold  = relative_threshold * lambda_max;

    Eigen::Matrix<double, N, N> D_inv = Eigen::Matrix<double, N, N>::Zero();
    for (int i = 0; i < N; ++i) {
        D_inv(i, i) = (evals(i) > threshold) ? 1.0 / evals(i) : 1e6;
    }

    return evecs * D_inv * evecs.transpose();
}

enum class RangeParam {
    Distance,
    LogDistance,
};

[[nodiscard]] static PnpGeometryInfo compute_pnp_geometry_info_ypdr(
    const std::vector<Eigen::Vector3d>& obj_pts, const Eigen::Matrix3d& R_cam_imu,
    const Eigen::Matrix3d& R_pitch, double armor_yaw, const Eigen::Vector3d& refined_ypd,
    const std::vector<Eigen::Vector2d>& img_pts, double residual_variance,
    double correlated_scale_variance,
    RangeParam output_range_param = RangeParam::Distance) noexcept {
    PnpGeometryInfo info;
    const Eigen::Matrix<double, 8, 4> J =
        compute_reprojection_jacobian_x(obj_pts, R_cam_imu, R_pitch, armor_yaw, refined_ypd);
    if (!J.allFinite()) {
        return info;
    }

    const Eigen::Matrix4d H     = J.transpose() * J;
    const Eigen::Matrix4d H_inv = pseudo_inverse_spd<4>(H);

    Eigen::Matrix4d R_ayplogd = H_inv * std::max(residual_variance, 1e-12);
    R_ayplogd                 = 0.5 * (R_ayplogd + R_ayplogd.transpose());

    if (img_pts.size() == 4 && std::isfinite(correlated_scale_variance)
        && correlated_scale_variance > 0.0) {

        Eigen::Vector2d center = Eigen::Vector2d::Zero();
        for (const auto& p : img_pts) {
            center += p;
        }
        center *= 0.25;

        Eigen::Matrix<double, 8, 1> scale_mode;
        scale_mode.setZero();

        bool valid = true;
        for (int i = 0; i < 4; ++i) {
            const Eigen::Vector2d radial = img_pts[static_cast<size_t>(i)] - center;
            const double norm            = radial.norm();

            if (!std::isfinite(norm) || norm < 1e-12) {
                valid = false;
                break;
            }

            scale_mode.template segment<2>(2 * i) = radial / norm;
        }

        if (valid && scale_mode.allFinite()) {
            const Eigen::Vector4d dx = H_inv * J.transpose() * scale_mode;
            if (dx.allFinite()) {
                R_ayplogd += correlated_scale_variance * (dx * dx.transpose());
                R_ayplogd = 0.5 * (R_ayplogd + R_ayplogd.transpose());
            }
        }
    }

    Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
    P(0, 1)           = 1.0;
    P(1, 2)           = 1.0;
    P(2, 3)           = 1.0;
    P(3, 0)           = 1.0;

    Eigen::Matrix4d R = P * R_ayplogd * P.transpose();

    if (output_range_param == RangeParam::Distance) {
        const double d = refined_ypd.z();

        if (std::isfinite(d) && d > 1e-9) {
            Eigen::Matrix4d S = Eigen::Matrix4d::Identity();
            S(2, 2)           = d; // log_distance -> distance
            R                 = S * R * S.transpose();
        } else {
            return info;
        }
    }

    R = 0.5 * (R + R.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> psd(R);
    if (psd.info() == Eigen::Success && R.allFinite()) {
        Eigen::Vector4d evals = psd.eigenvalues();
        Eigen::Matrix4d evecs = psd.eigenvectors();

        for (int i = 0; i < 4; ++i) {
            evals(i) = std::max(evals(i), 1e-15);
        }

        R = evecs * evals.asDiagonal() * evecs.transpose();
        R = 0.5 * (R + R.transpose());
    }

    info.cov_ypdr = R;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(H);
    if (es.info() == Eigen::Success && H.allFinite()) {
        const auto evals        = es.eigenvalues();
        const double lambda_min = std::max(evals.minCoeff(), 1e-18);
        const double lambda_max = std::max(evals.maxCoeff(), lambda_min);
        info.condition_number   = std::clamp(lambda_max / lambda_min, 1.0, 1e6);
    }

    return info;
}

[[nodiscard]] inline double extract_yaw_from_rotation(
    const Eigen::Matrix3d& R_cam_armor, const Eigen::Matrix3d& R_imu_cam) noexcept {
    const Eigen::Matrix3d R_imu_armor = R_imu_cam * R_cam_armor;

    // Clamp to [-1, 1] — noisy PnP can produce slightly non-orthogonal R.
    const double r01   = std::clamp(-R_imu_armor(0, 1), -1.0, 1.0);
    const double r11   = std::clamp(R_imu_armor(1, 1), -1.0, 1.0);
    const double yaw_s = std::asin(r01);
    const double yaw_c = std::acos(r11);

    if (std::abs(yaw_s) > 1e-5) {
        return (yaw_s > 0.0) ? yaw_c : -yaw_c;
    }
    return (r11 > 0.0) ? 0.0 : std::numbers::pi;
}

// ============================================================================
// AT Legacy PnP Solver
// ============================================================================

/// PnP solver for armor pose estimation
class PnPSolver {
public:
    using PoseResult = std::expected<CameraArmorMeasurement, PnPError>;

    struct PosePrior {
        cv::Vec3d rvec{};
        cv::Vec3d tvec{};
        double hint_cost{0.0};
        int armor_id{-1};
    };

    /// Construct with camera parameters — fully initialized, no separate init() needed.
    explicit PnPSolver(const CameraConfig& config) {
        cv::eigen2cv(config.camera_matrix, camera_matrix_);
        cv::eigen2cv(config.distort_coefficient, dist_coeffs_);

        // Pre-compute image center for distance calculation
        image_center_ = cv::Point2f(
            config.width / 2.0, // cx
            config.height / 2.0 // cy
        );

        build_model_points();
    }

    /// Solve armor pose with prior-seeded constrained reprojection refinement.
    /// OpenCV PnP is used only to enter a plausible local basin; the final solve
    /// happens on the physical 4DoF manifold (yaw + camera-frame translation).
    /// @param detection Armor detection with 4 corners
    /// @param R_imu_cam Rotation from camera frame into the yaw frame (imu/odom)
    /// @param timestamp_ns Timestamp for the measurement
    /// @return Measurement with refined 3D pose, or error
    [[nodiscard]] PoseResult solve_with_ba(
        const ArmorDetection& detection, const Eigen::Matrix3d& R_imu_cam, uint64_t timestamp_ns,
        const std::vector<PosePrior>& pose_priors = {}) const {
        auto input = make_pnp_input(detection);
        if (!input) {
            return std::unexpected(input.error());
        }

        // Step 1: Initial 6DoF pose estimate in the undistorted normalized plane
        const auto initial_pose = solve_initial_pose(*input, pose_priors);
        if (!initial_pose.has_value()) {
            return std::unexpected(PnPError::SolveFailed);
        }
        cv::Mat rvec = initial_pose->rvec.clone();
        cv::Mat tvec = initial_pose->tvec.clone();

        // Step 2: Convert rvec/tvec to Eigen.
        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);

        Eigen::Matrix3d R_cam_armor;
        cv::cv2eigen(R_cv, R_cam_armor);

        Eigen::Vector3d t_cam_armor(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

        // Bail out on degenerate PnP (NaN/Inf from IPPE with bad geometry)
        {
            const double* rd = rvec.ptr<double>();
            const double* td = tvec.ptr<double>();
            for (int i = 0; i < 3; ++i) {
                if (!std::isfinite(rd[i]) || !std::isfinite(td[i])) {
                    return std::unexpected(PnPError::SolveFailed);
                }
            }
        }

        // Step 3: Prepare the constrained pose model.
        const Eigen::Matrix3d R_cam_imu = R_imu_cam.transpose();
        const double armor_pitch_rad    = armor_pitch_rad_for(detection.name);
        const Eigen::Matrix3d R_pitch   = armor_pitch_rotation(armor_pitch_rad);
        double yaw                      = extract_yaw_from_rotation(R_cam_armor, R_imu_cam);
        const Eigen::Vector3d initial_translation_ypd = camera_translation_to_ypd(t_cam_armor);
        double translation_ypd[3]                     = {
            initial_translation_ypd.x(),
            initial_translation_ypd.y(),
            initial_translation_ypd.z(),
        };

        // Step 4: Prepare 3D model points and normalized 2D observations.
        std::vector<Eigen::Vector3d> obj_pts_eigen;
        obj_pts_eigen.reserve(input->obj_points.size());
        for (const auto& p : input->obj_points) {
            obj_pts_eigen.emplace_back(p.x, p.y, p.z);
        }

        std::vector<Eigen::Vector2d> img_pts_eigen;
        img_pts_eigen.reserve(input->norm_points.size());
        for (const auto& p : input->norm_points) {
            img_pts_eigen.emplace_back(p.x, p.y);
        }

        // Step 5: Build Ceres problem - optimize constrained pose on the normalized plane.

        // ====================================================================
        // CERES SOLVER OWNERSHIP TRANSFER (RAII-compliant)
        // ====================================================================
        //
        // Ceres Problem takes ownership of CostFunction and LossFunction pointers.
        // See ConstrainedReprojError::Create() above for full provenance.
        //
        // We allocate with new (required by Ceres API), transfer ownership
        // via AddResidualBlock(), and let Ceres Problem destructor handle cleanup.
        //
        // NO manual delete needed - Ceres is RAII-compliant~ ♪
        // ====================================================================
        ceres::Problem problem;
        // Huber loss for robustness (ownership transferred to problem)
        // auto* loss_function = new ceres::HuberLoss(0.5);

        for (size_t i = 0; i < obj_pts_eigen.size(); ++i) {
            ceres::CostFunction* cost = ConstrainedReprojError::Create(
                obj_pts_eigen[i], img_pts_eigen[i], R_cam_imu, R_pitch);
            problem.AddResidualBlock(cost, nullptr, &yaw, translation_ypd);
        }

        // Constrain armor yaw to [-pi, pi] and optical bearing to the front camera hemisphere.
        problem.SetParameterLowerBound(&yaw, 0, -std::numbers::pi);
        problem.SetParameterUpperBound(&yaw, 0, std::numbers::pi);
        constexpr double kBearingLimit = std::numbers::pi / 2.0 - 1e-6;
        problem.SetParameterLowerBound(translation_ypd, 0, -kBearingLimit);
        problem.SetParameterUpperBound(translation_ypd, 0, kBearingLimit);
        problem.SetParameterLowerBound(translation_ypd, 1, -kBearingLimit);
        problem.SetParameterUpperBound(translation_ypd, 1, kBearingLimit);
        // if (detection.name == ArmorName::Outpost) {
        //     auto original_norm = initial_pose.value().prior_distance_m
        //                            ? initial_pose.value().prior_distance_m.value()
        //                            : cv::norm(tvec);
        //     problem.SetParameterLowerBound(translation_ypd, 2, std::max(0.0, original_norm -
        //     0.5)); problem.SetParameterUpperBound(translation_ypd, 2, original_norm + 0.5);
        // }

        ceres::Solver::Options options;
        options.linear_solver_type           = ceres::DENSE_QR;
        options.max_num_iterations           = 20;
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // Step 6: Reconstruct refined constrained pose if optimization succeeded.
        const Eigen::Vector3d refined_translation = camera_ypd_to_translation(
            Eigen::Vector3d{translation_ypd[0], translation_ypd[1], translation_ypd[2]});
        const bool translation_finite =
            std::isfinite(translation_ypd[0]) && std::isfinite(translation_ypd[1])
            && std::isfinite(translation_ypd[2]) && refined_translation.allFinite()
            && refined_translation.z() > 1e-3;
        if (summary.IsSolutionUsable() && std::isfinite(yaw) && translation_finite) {
            const Eigen::Matrix3d R_cam_armor_refined = R_cam_imu * yaw_rotation(yaw) * R_pitch;

            // Write back the constrained pose.
            cv::Mat R_refined_cv;
            cv::eigen2cv(R_cam_armor_refined, R_refined_cv);
            cv::Rodrigues(R_refined_cv, rvec);
            tvec =
                (cv::Mat_<double>(3, 1) << refined_translation.x(), refined_translation.y(),
                 refined_translation.z());
        }
        // If optimization failed, keep the initial basin pose.

        auto measurement =
            make_camera_measurement(detection, rvec, tvec, timestamp_ns, input->dist_to_center);
        if (summary.IsSolutionUsable() && std::isfinite(yaw) && translation_finite) {
            const Eigen::Vector3d refined_ypd{
                translation_ypd[0],
                translation_ypd[1],
                translation_ypd[2],
            };
            const double final_rmse    = normalized_reprojection_rmse(*input, rvec, tvec);
            const double point_count   = static_cast<double>(obj_pts_eigen.size());
            const double residual_dim  = 2.0 * point_count;
            constexpr double kParamDim = 4.0;
            const double dof           = std::max(1.0, residual_dim - kParamDim);
            const double sigma2_from_residual =
                std::isfinite(final_rmse) ? final_rmse * final_rmse * point_count / dof : 0.0;
            const double fx    = camera_matrix_.at<double>(0, 0);
            const double fy    = camera_matrix_.at<double>(1, 1);
            const double focal = std::sqrt(std::max(1e-9, fx * fy));
            // Independent keypoint jitter. The correlated scale mode below carries the
            // range-critical corner shrink/expand uncertainty.
            // constexpr double kCorrelatedScaleSigmaPx        = 10.0;
            auto distance_factor = 0.25 / 5.0;
            if (detection.name == ArmorName::Outpost) {
                distance_factor = 4.0 / 5.0;
            }
            const double kIndependentCornerSigmaPxFloor =
                distance_factor * translation_ypd[2] + 0.5;
            const auto kCorrelatedScaleSigmaPx = distance_factor * translation_ypd[2] + 1.0;
            const double sigma_norm_floor      = kIndependentCornerSigmaPxFloor / focal;
            const double sigma_norm_scale      = kCorrelatedScaleSigmaPx / focal;
            const double residual_variance =
                std::max(sigma2_from_residual, sigma_norm_floor * sigma_norm_floor);
            const PnpGeometryInfo pnp_geometry = compute_pnp_geometry_info_ypdr(
                obj_pts_eigen, R_cam_imu, R_pitch, yaw, refined_ypd, img_pts_eigen,
                residual_variance, sigma_norm_scale * sigma_norm_scale);
            measurement.pnp_cov_ypdr         = pnp_geometry.cov_ypdr;
            measurement.pnp_condition_number = pnp_geometry.condition_number;
        }
        return measurement;
    }

    /// Solve PnP with BA for multiple detections
    [[nodiscard]] std::vector<CameraArmorMeasurement> solve_batch_with_ba(
        const std::vector<ArmorDetection>& detections, const Eigen::Matrix3d& R_imu_cam,
        uint64_t timestamp_ns) const {
        std::vector<CameraArmorMeasurement> measurements;
        measurements.reserve(detections.size());

        for (const auto& det : detections) {
            auto result = solve_with_ba(det, R_imu_cam, timestamp_ns);
            if (result) {
                measurements.push_back(std::move(*result));
            }
        }

        return measurements;
    }

private:
    struct SolvePose {
        cv::Mat rvec;
        cv::Mat tvec;
        double reprojection_rmse{std::numeric_limits<double>::infinity()};
        std::optional<double> prior_armor_yaw{};
        std::optional<double> prior_distance_m{};
    };

    struct PnPInput {
        const std::vector<cv::Point3f>& obj_points;
        std::vector<cv::Point2f> img_points;
        std::vector<cv::Point2f> norm_points;
        float dist_to_center;
    };

    [[nodiscard]] static cv::Mat identity_camera_matrix() { return cv::Mat::eye(3, 3, CV_64F); }

    [[nodiscard]] static bool pose_is_finite(const cv::Mat& rvec, const cv::Mat& tvec) {
        const double* rd = rvec.ptr<double>();
        const double* td = tvec.ptr<double>();
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(rd[i]) || !std::isfinite(td[i])) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] static double normalized_reprojection_rmse(
        const PnPInput& input, const cv::Mat& rvec, const cv::Mat& tvec) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(
            input.obj_points, rvec, tvec, identity_camera_matrix(), cv::Mat{}, projected);
        if (projected.size() != input.norm_points.size()) {
            return std::numeric_limits<double>::infinity();
        }

        double sum_sq = 0.0;
        for (size_t i = 0; i < projected.size(); ++i) {
            const cv::Point2f d = projected[i] - input.norm_points[i];
            sum_sq += static_cast<double>(d.x) * static_cast<double>(d.x)
                    + static_cast<double>(d.y) * static_cast<double>(d.y);
        }
        return std::sqrt(sum_sq / static_cast<double>(projected.size()));
    }

    [[nodiscard]] std::optional<SolvePose> solve_with_ippe(const PnPInput& input) const {
        cv::Mat rvec, tvec;
        const bool success = cv::solvePnP(
            input.obj_points, input.norm_points, identity_camera_matrix(), cv::Mat{}, rvec, tvec,
            false, cv::SOLVEPNP_IPPE);
        if (!success || !pose_is_finite(rvec, tvec)) {
            return std::nullopt;
        }

        const double reprojection_rmse = normalized_reprojection_rmse(input, rvec, tvec);

        return SolvePose{
            .rvec              = std::move(rvec),
            .tvec              = std::move(tvec),
            .reprojection_rmse = reprojection_rmse,
        };
    }

    [[nodiscard]] std::optional<SolvePose>
        solve_with_priors(const PnPInput& input, const std::vector<PosePrior>& pose_priors) const {
        if (pose_priors.empty()) {
            return std::nullopt;
        }
        auto ippe = solve_with_ippe(input);
        if (!ippe) {
            return std::nullopt;
        }
        auto norm = cv::norm(ippe->tvec);

        std::vector<PosePrior> sorted_priors = pose_priors;
        std::sort(
            sorted_priors.begin(), sorted_priors.end(),
            [norm](const PosePrior& a, const PosePrior& b) {
                return std::abs(norm - cv::norm(a.tvec)) < std::abs(norm - cv::norm(b.tvec));
            });

        std::optional<SolvePose> best;
        for (const auto& prior : sorted_priors) {
            const double prior_distance = std::sqrt(
                prior.tvec[0] * prior.tvec[0] + prior.tvec[1] * prior.tvec[1]
                + prior.tvec[2] * prior.tvec[2]);
            if (!std::isfinite(prior_distance) || prior_distance <= 1e-3) {
                continue;
            }

            cv::Mat rvec = (cv::Mat_<double>(3, 1) << prior.rvec[0], prior.rvec[1], prior.rvec[2]);
            cv::Mat tvec = (cv::Mat_<double>(3, 1) << prior.tvec[0], prior.tvec[1], prior.tvec[2]);
            const bool success = cv::solvePnP(
                input.obj_points, input.norm_points, identity_camera_matrix(), cv::Mat{}, rvec,
                tvec, true, cv::SOLVEPNP_ITERATIVE);
            if (!success || !pose_is_finite(rvec, tvec)) {
                continue;
            }

            const double reprojection_rmse = normalized_reprojection_rmse(input, rvec, tvec);
            if (!std::isfinite(reprojection_rmse)) {
                continue;
            }
            Eigen::Matrix3d x;
            cv::Mat xx;
            cv::Rodrigues(prior.rvec, xx);
            cv::cv2eigen(xx, x);
            if (!best.has_value() || reprojection_rmse < best->reprojection_rmse) {
                best = SolvePose{
                    .rvec              = std::move(rvec),
                    .tvec              = std::move(tvec),
                    .reprojection_rmse = reprojection_rmse,
                    .prior_armor_yaw   = math_fuxk::rpy(x).yaw,
                    .prior_distance_m  = prior_distance,
                };
            }
        }

        return best;
    }

    [[nodiscard]] std::optional<SolvePose>
        solve_initial_pose(const PnPInput& input, const std::vector<PosePrior>& pose_priors) const {
        const auto prior_pose = solve_with_priors(input, pose_priors);
        if (prior_pose.has_value()) {
            return prior_pose;
        }
        return solve_with_ippe(input);
    }

    [[nodiscard]] std::expected<PnPInput, PnPError>
        make_pnp_input(const ArmorDetection& detection) const {
        auto img_points = debias_correlated_corner_scale(detection.image_points());
        if (img_points.size() != 4) {
            return std::unexpected(PnPError::InvalidDetection);
        }

        std::vector<cv::Point2f> norm_points;
        cv::undistortPoints(img_points, norm_points, camera_matrix_, dist_coeffs_);
        if (norm_points.size() != img_points.size()) {
            return std::unexpected(PnPError::InvalidDetection);
        }

        const auto& obj_points =
            (detection.type == ArmorType::Large) ? large_armor_points_ : small_armor_points_;

        return PnPInput{
            .obj_points     = obj_points,
            .img_points     = std::move(img_points),
            .norm_points    = std::move(norm_points),
            .dist_to_center = static_cast<float>(cv::norm(detection.center() - image_center_)),
        };
    }

    void build_model_points() {
        constexpr double SMALL_ARMOR_WIDTH  = 135.0 / 1000.0;
        constexpr double SMALL_ARMOR_HEIGHT = 55.0 / 1000.0;
        constexpr double LARGE_ARMOR_WIDTH  = 230.0 / 1000.0;
        constexpr double LARGE_ARMOR_HEIGHT = 55.0 / 1000.0;

        // Small armor: 135mm x 55mm
        const float sw      = static_cast<float>(SMALL_ARMOR_WIDTH / 2.0);
        const float sh      = static_cast<float>(SMALL_ARMOR_HEIGHT / 2.0);
        small_armor_points_ = {
            cv::Point3f(0, sw, sh),   // Top-left
            cv::Point3f(0, -sw, sh),  // Top-right
            cv::Point3f(0, -sw, -sh), // Bottom-right
            cv::Point3f(0, sw, -sh)   // Bottom-left
        };

        // Large armor: 230mm x 55mm
        const float lw      = static_cast<float>(LARGE_ARMOR_WIDTH / 2.0);
        const float lh      = static_cast<float>(LARGE_ARMOR_HEIGHT / 2.0);
        large_armor_points_ = {
            cv::Point3f(0, lw, lh),   // Top-left
            cv::Point3f(0, -lw, lh),  // Top-right
            cv::Point3f(0, -lw, -lh), // Bottom-right
            cv::Point3f(0, lw, -lh)   // Bottom-left
        };
    }

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Point2f image_center_;

    std::vector<cv::Point3f> small_armor_points_;
    std::vector<cv::Point3f> large_armor_points_;
};

} // namespace fcs::L2
