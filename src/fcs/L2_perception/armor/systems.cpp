#include "L2_perception/armor/systems.hpp"

#include "L2_perception/armor/backend.hpp"
#include "L2_perception/armor/config.hpp"
#include "L2_perception/armor/readback_roi.hpp"
#include "L2_perception/armor/solver.hpp"
#include "core/math/normalize.hpp"
#include "core/runtime.hpp"
#include "core/types.hpp"
#include "frame.hpp"
#include "scheduler/scheduler.hpp"
#include <opencv2/imgproc.hpp>
#include <optional>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <magic_enum.hpp>

namespace fcs::L2 {

namespace {

std::array<cv::Point2f, 4> sort_corners(const std::array<cv::Point2f, 4>& points) {
    cv::Point2f center(0.0f, 0.0f);

    for (const auto& p : points) {
        center += p;
    }
    center *= 0.25f;

    struct Item {
        cv::Point2f point;
        float angle;
    };

    std::vector<Item> sorted;
    sorted.reserve(4);

    for (const auto& p : points) {
        cv::Point2f dir = p - center;

        float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        if (len > 1e-6f) {
            dir.x /= len;
            dir.y /= len;
        }

        // 等价于 Rust glam::Vec2::angle_to(Vec2::X)
        // angle_to(X) = atan2(cross(dir, X), dot(dir, X))
        // cross(dir, X) = dir.x * 0 - dir.y * 1 = -dir.y
        // dot(dir, X) = dir.x
        float angle = std::atan2(-dir.y, dir.x) * 180.0f / static_cast<float>(CV_PI);

        sorted.push_back({p, angle});
    }

    // 角度 descending 排序
    std::sort(sorted.begin(), sorted.end(), [](const Item& a, const Item& b) {
        return a.angle > b.angle;
    });

    return {sorted[0].point, sorted[1].point, sorted[2].point, sorted[3].point};
}

[[nodiscard]] std::optional<cv::Point2f> normalize_image_point(
    const cv::Point2f& image_point, const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs) noexcept {
    std::vector<cv::Point2f> norm_points;
    cv::undistortPoints(
        std::vector<cv::Point2f>{image_point}, norm_points, camera_matrix, dist_coeffs);
    if (norm_points.size() != 1) {
        return std::nullopt;
    }

    const auto& point = norm_points.front();
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return std::nullopt;
    }
    return point;
}

[[nodiscard]] bool tracker_snapshot_matches_detection(
    const TrackerReadbackSnapshot& snapshot, const ArmorDetection& detection) noexcept {
    return snapshot.valid && snapshot.tracker.target_name == detection.name
        && snapshot.tracker.target_color == detection.color;
}

[[nodiscard]] Eigen::Vector3d
    camera_translation_to_ypd_local(const Eigen::Vector3d& translation) noexcept {
    const double horizontal = std::hypot(translation.x(), translation.z());
    return {
        std::atan2(translation.x(), translation.z()),
        std::atan2(translation.y(), horizontal),
        translation.norm(),
    };
}

[[nodiscard]] Eigen::Matrix3d camera_ypld_to_xyz_jacobian(const Eigen::Vector3d& ypd) noexcept {
    const double yaw      = ypd.x();
    const double pitch    = ypd.y();
    const double distance = ypd.z();

    const double sy = std::sin(yaw);
    const double cy = std::cos(yaw);
    const double sp = std::sin(pitch);
    const double cp = std::cos(pitch);

    Eigen::Matrix3d J;
    J << distance * cp * cy, -distance * sp * sy, distance * cp * sy, 0.0, distance * cp,
        distance * sp, -distance * cp * sy, -distance * sp * cy, distance * cp * cy;
    return J;
}

[[nodiscard]] Eigen::Matrix3d odom_xyz_to_ypld_jacobian(const Eigen::Vector3d& xyz) noexcept {
    const double x     = xyz.x();
    const double y     = xyz.y();
    const double z     = xyz.z();
    const double r2_xy = x * x + y * y;
    const double r2    = r2_xy + z * z;
    const double r_xy  = std::sqrt(r2_xy);
    const double r     = std::sqrt(r2);

    if (r_xy < 1e-10 || r < 1e-10) {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix3d J;
    J(0, 0) = -y / r2_xy;
    J(0, 1) = x / r2_xy;
    J(0, 2) = 0.0;

    const double denom = r2 * r_xy;
    J(1, 0)            = x * z / denom;
    J(1, 1)            = y * z / denom;
    J(1, 2)            = -r_xy / r2;

    J(2, 0) = x / r2;
    J(2, 1) = y / r2;
    J(2, 2) = z / r2;
    return J;
}

[[nodiscard]] Eigen::Matrix4d reframe_camera_pnp_cov_ypdr_to_odom(
    const CameraArmorMeasurement& camera_measurement, const ArmorMeasurement& odom_measurement,
    const fast_tf::FrameTransform<fast_tf::odom, fast_tf::camera_optical>& T_odom_camera) noexcept {
    if (!camera_measurement.pnp_cov_ypdr.allFinite()) {
        return Eigen::Matrix4d::Identity() * 1e6;
    }

    const Eigen::Vector3d camera_ypd =
        camera_translation_to_ypd_local(camera_measurement.transform.translation());
    const Eigen::Matrix3d J_camera_xyz = camera_ypld_to_xyz_jacobian(camera_ypd);
    const Eigen::Matrix3d J_odom_ypd =
        odom_xyz_to_ypld_jacobian(odom_measurement.transform.translation());

    Eigen::Matrix4d J   = Eigen::Matrix4d::Zero();
    J.block<3, 3>(0, 0) = J_odom_ypd * T_odom_camera.rotation() * J_camera_xyz;
    J(3, 3)             = 1.0;

    Eigen::Matrix4d cov = J * camera_measurement.pnp_cov_ypdr * J.transpose();
    cov                 = 0.5 * (cov + cov.transpose());
    if (!cov.allFinite()) {
        return Eigen::Matrix4d::Identity() * 1e6;
    }
    return cov;
}

[[nodiscard]] std::vector<PnPSolver::PosePrior> make_pose_priors(
    const ArmorDetection& detection, const TrackerReadbackSnapshot& snapshot,
    const fast_tf::FrameTransform<fast_tf::camera_optical, fast_tf::odom>& T_odom_camera,
    const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) noexcept {
    if (!tracker_snapshot_matches_detection(snapshot, detection)) {
        return {};
    }

    const auto det_center_norm =
        normalize_image_point(detection.center(), camera_matrix, dist_coeffs);
    if (!det_center_norm.has_value()) {
        return {};
    }

    const auto armor_poses = tracker_armor_poses(snapshot.tracker);
    if (armor_poses.empty()) {
        return {};
    }

    const double armor_pitch_rad = armor_pitch_rad_for(detection.name);
    const double cp              = std::cos(armor_pitch_rad);
    const double sp              = std::sin(armor_pitch_rad);
    Eigen::Matrix3d R_pitch;
    R_pitch << cp, 0.0, sp, 0.0, 1.0, 0.0, -sp, 0.0, cp;

    const Eigen::Matrix3d R_odom_camera = T_odom_camera.rotation();
    const Eigen::Vector3d t_odom_camera = T_odom_camera.translation();

    std::vector<PnPSolver::PosePrior> priors;
    priors.reserve(armor_poses.size());

    for (size_t i = 0; i < armor_poses.size(); ++i) {
        const auto& armor_pose = armor_poses[i];
        const double armor_yaw = armor_pose[3];
        const double cy        = std::cos(armor_yaw);
        const double sy        = std::sin(armor_yaw);

        Eigen::Matrix3d R_yaw;
        R_yaw << cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0;

        const Eigen::Matrix3d R_odom_armor   = R_yaw * R_pitch;
        const Eigen::Matrix3d R_camera_armor = R_odom_camera * R_odom_armor;
        const Eigen::Vector3d t_camera_armor =
            R_odom_camera * Eigen::Vector3d(armor_pose[0], armor_pose[1], armor_pose[2])
            + t_odom_camera;

        if (!std::isfinite(t_camera_armor.x()) || !std::isfinite(t_camera_armor.y())
            || !std::isfinite(t_camera_armor.z()) || t_camera_armor.z() <= 1e-3) {
            continue;
        }

        const cv::Point2f prior_center_norm(
            static_cast<float>(t_camera_armor.x() / t_camera_armor.z()),
            static_cast<float>(t_camera_armor.y() / t_camera_armor.z()));
        if (!std::isfinite(prior_center_norm.x) || !std::isfinite(prior_center_norm.y)) {
            continue;
        }

        cv::Mat R_cv;
        cv::eigen2cv(R_camera_armor, R_cv);
        cv::Mat rvec;
        cv::Rodrigues(R_cv, rvec);

        const double dx  = static_cast<double>(prior_center_norm.x - det_center_norm->x);
        const double dy  = static_cast<double>(prior_center_norm.y - det_center_norm->y);
        double hint_cost = dx * dx + dy * dy;
        if (static_cast<int>(i) == snapshot.selected_armor_id) {
            hint_cost *= 0.85;
        } else if (static_cast<int>(i) == snapshot.rough_selected_armor_id) {
            hint_cost *= 0.92;
        }

        priors.push_back(
            PnPSolver::PosePrior{
                .rvec      = cv::Vec3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)),
                .tvec      = cv::Vec3d(t_camera_armor.x(), t_camera_armor.y(), t_camera_armor.z()),
                .hint_cost = hint_cost,
                .armor_id  = static_cast<int>(i),
            });
    }

    return priors;
}

void offset_detections(std::vector<ArmorDetection>& detections, const cv::Point& offset) noexcept {
    if (offset.x == 0 && offset.y == 0) {
        return;
    }

    const cv::Point2f offset_f(static_cast<float>(offset.x), static_cast<float>(offset.y));
    for (auto& detection : detections) {
        for (auto& corner : detection.corners) {
            corner += offset_f;
        }
        detection.rect.x += static_cast<float>(offset.x);
        detection.rect.y += static_cast<float>(offset.y);
    }
}

[[nodiscard]] inline bool
    is_sub_frame_roi(const cv::Rect& roi, const cv::Size& frame_size) noexcept {
    return roi.x > 0 || roi.y > 0 || roi.width < frame_size.width || roi.height < frame_size.height;
}

struct AspectRatioBounds {
    double min;
    double max;
};

[[nodiscard]] constexpr double armor_expected_rect_aspect_ratio(ArmorType type) noexcept {
    switch (type) {
    case ArmorType::Small: return 135.0 / 55.0;
    case ArmorType::Large: return 230.0 / 55.0;
    case ArmorType::Invalid: return 0.0;
    }
    std::abort();
}

[[nodiscard]] constexpr AspectRatioBounds armor_rect_aspect_ratio_bounds(ArmorType type) noexcept {
    constexpr double min_scale = 0.45;
    constexpr double max_scale = 1.30;
    if (type == ArmorType::Invalid) {
        return {
            armor_expected_rect_aspect_ratio(ArmorType::Small) * min_scale,
            armor_expected_rect_aspect_ratio(ArmorType::Large) * max_scale,
        };
    }

    const double expected_ratio = armor_expected_rect_aspect_ratio(type);
    return {expected_ratio * min_scale, expected_ratio * max_scale};
}

[[nodiscard]] bool detection_rect_aspect_ratio_is_valid(const ArmorDetection& detection) noexcept {
    float min_x = detection.corners[0].x;
    float max_x = detection.corners[0].x;
    float min_y = detection.corners[0].y;
    float max_y = detection.corners[0].y;

    for (const auto& corner : detection.corners) {
        if (!std::isfinite(corner.x) || !std::isfinite(corner.y)) {
            return false;
        }
        min_x = std::min(min_x, corner.x);
        max_x = std::max(max_x, corner.x);
        min_y = std::min(min_y, corner.y);
        max_y = std::max(max_y, corner.y);
    }

    const float width  = max_x - min_x;
    const float height = max_y - min_y;
    if (width <= 1e-3f || height <= 1e-3f) {
        return false;
    }

    if (width <= height) {
        return false;
    }

    const double aspect_ratio = static_cast<double>(width) / static_cast<double>(height);
    const auto bounds         = armor_rect_aspect_ratio_bounds(detection.type);
    return bounds.min <= aspect_ratio && aspect_ratio <= bounds.max;
}

} // namespace

void register_detection_systems(talos::Scheduler& scheduler) noexcept {
    auto& world = scheduler.world();
    if (!world.has_resource<ArmorReadbackRoiConfig>()) {
        world.insert_resource(ArmorReadbackRoiConfig{});
    }
    if (!world.has_resource<TrackerReadbackCache>()) {
        world.insert_resource(TrackerReadbackCache{});
    }

    scheduler.add_system<talos::fixed_rate<200>>(
        "armor_detector",
        [](talos::spmc<ImageFrame, ImageChannelTopic> image_in,
           talos::spmc_mut<ArmorDetectionBatch, DetectionChannelTopic> detection_out,
           talos::res_mut<std::shared_ptr<DetectorBackend>> backend,
           talos::res<fast_tf::CoordinateSystem> tf_system, talos::res<CameraConfig> camera_config,
           talos::res<ArmorReadbackRoiConfig> readback_roi_config,
           talos::res<TrackerReadbackCache> readback_cache, core::capabilities cap,
           core::detecting_color detecting_color_) mutable {
            auto frame = image_in.read();
            if (!frame) {
                return;
            }
            if (!core::capable(*cap, core::Capability::Armor)) {
                detection_out.write(
                    ArmorDetectionBatch({}, frame->image, frame->timestamp_ns, frame->frame_id));
                return;
            }

            ArmorColor detect_color = *detecting_color_;

            cv::Rect detector_roi(0, 0, frame->image.cols, frame->image.rows);
            cv::Mat detector_input = frame->image;

            auto tf_lookup = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(
                *tf_system, frame->timestamp_ns);

            if (!tf_lookup) {
                SPDLOG_ERROR(
                    "TF lookup failed for timestamp {}: {}", frame->timestamp_ns,
                    tf_lookup.error());
                return;
            }

            auto T_odom_camera = tf_lookup.value();
            if (readback_roi_config->enabled) {
                const auto snapshot = readback_cache->load();
                if (tracker_snapshot_is_fresh(
                        snapshot, *readback_roi_config, frame->timestamp_ns)) {
                    const auto maybe_roi = resolve_readback_roi(
                        frame->image.size(),
                        project_tracker_box_to_image(
                            snapshot.tracker, T_odom_camera.inverse(), *camera_config,
                            readback_roi_config->box_size_m)
                            .value_or(cv::Rect2f{}),
                        *readback_roi_config, backend->get()->input_resolution());
                    if (maybe_roi) {
                        detector_roi   = *maybe_roi;
                        detector_input = frame->image(detector_roi);
                    }
                }
            }
            const bool has_detector_roi = is_sub_frame_roi(detector_roi, frame->image.size());

            auto result = backend->get()->detect(detector_input, detect_color);
            if (!result) {
                detection_out.write(
                    ArmorDetectionBatch{
                        {},
                        frame->image,
                        frame->timestamp_ns,
                        frame->frame_id,
                        has_detector_roi,
                        detector_roi});
                return;
            }

            auto detections = std::move(*result);
            for (auto& detection : detections) {
                detection.corners = sort_corners(detection.corners);
            }
            offset_detections(detections, detector_roi.tl());
            std::vector<ArmorDetection> detections_for_pnp;
            detections_for_pnp.reserve(detections.size());
            std::copy_if(
                detections.begin(), detections.end(), std::back_inserter(detections_for_pnp),
                [detect_color](const ArmorDetection& det) {
                    return det.color == detect_color && detection_rect_aspect_ratio_is_valid(det);
                });

            detection_out.write(
                ArmorDetectionBatch{
                    std::move(detections_for_pnp), frame->image, frame->timestamp_ns,
                    frame->frame_id, has_detector_roi, detector_roi});
        });
    scheduler.add_system<talos::fixed_rate<200>>(
        "armor_solver",
        [](talos::spmc<ArmorDetectionBatch, DetectionChannelTopic> detection_in,
           talos::spmc_mut<ArmorMeasurementBatch, MeasurementChannelTopic> measurement_out,
           talos::res<std::shared_ptr<PnPSolver>> solver_ptr,
           talos::res<fast_tf::CoordinateSystem> tf_system, talos::res<CameraConfig> camera_config,
           talos::res<ArmorReadbackRoiConfig> readback_roi_config, core::capabilities cap,
           talos::res<TrackerReadbackCache> readback_cache) mutable {
            if (!core::capable(*cap, core::Capability::Armor)) {
                return;
            }
            auto detections = detection_in.read();
            if (!detections) {
                return;
            }

            auto tf_lookup = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(
                *tf_system, detections->timestamp_ns);

            if (!tf_lookup) {
                SPDLOG_ERROR(
                    "TF lookup failed for timestamp {}: {}", detections->timestamp_ns,
                    tf_lookup.error());
                return;
            }

            const auto T_odom_camera     = tf_lookup.value();
            const auto T_camera_odom     = T_odom_camera.inverse();
            const auto R_odom_camera     = T_odom_camera.rotation();
            const auto tracker_snapshot  = readback_cache->load();
            const bool snapshot_is_fresh = tracker_snapshot_is_fresh(
                tracker_snapshot, *readback_roi_config, detections->timestamp_ns);

            // Pre-convert camera intrinsics once (avoid per-frame cv::eigen2cv allocations).
            cv::Mat camera_matrix_cv;
            cv::Mat dist_coeffs_cv;
            cv::eigen2cv(camera_config->camera_matrix, camera_matrix_cv);
            cv::eigen2cv(camera_config->distort_coefficient, dist_coeffs_cv);

            std::vector<CameraArmorMeasurement> measurements;
            measurements.reserve(detections->detections.size());
            for (const auto& detection : detections->detections) {
                const auto priors = snapshot_is_fresh
                                      ? make_pose_priors(
                                            detection, tracker_snapshot, T_camera_odom,
                                            camera_matrix_cv, dist_coeffs_cv)
                                      : std::vector<PnPSolver::PosePrior>{};
                auto result =
                    (*solver_ptr)
                        ->solve_with_ba(detection, R_odom_camera, detections->timestamp_ns, priors);
                if (result) {
                    if (result->name == ArmorName::Outpost) {
                        auto target_in_ref = T_odom_camera * result->transform;

                        auto target_pos_yaw = core::math::xyz2ypd(target_in_ref.translation())[0];
                        auto [roll, pitch, target_yaw] = target_in_ref.euler_rot().rpy();

                        auto delta_angle = core::math::normalize_angle(target_yaw - target_pos_yaw)
                                         * 180.0 / std::numbers::pi;
                        if (std::abs(delta_angle) > 25) {
                            continue;
                        }
                    }
                    measurements.push_back(std::move(*result));
                }
            }

            std::vector<ArmorMeasurement> odom_measurements;
            odom_measurements.reserve(measurements.size());

            for (const auto& meas : measurements) {
                auto odom_measurement = meas.reframe(T_odom_camera);
                odom_measurement.pnp_cov_ypdr =
                    reframe_camera_pnp_cov_ypdr_to_odom(meas, odom_measurement, T_odom_camera);
                odom_measurements.push_back(std::move(odom_measurement));
            }

            measurement_out.write(
                ArmorMeasurementBatch{
                    std::move(odom_measurements), detections->timestamp_ns, detections->frame_id});
        });
}

std::expected<DetectorBackendHandle, std::string>
    create_detector_backend_handle(const ArmorDetectorConfig& config) noexcept {
    auto backend_result = create_detector_backend(config);
    if (!backend_result) {
        return std::unexpected(std::move(backend_result.error()));
    }

    auto backend_ptr  = std::make_shared<DetectorBackend>(std::move(*backend_result));
    auto backend_name = std::string(magic_enum::enum_name(config.backend_type));
    return DetectorBackendHandle{std::move(backend_ptr), std::move(backend_name)};
}

std::shared_ptr<PnPSolver> create_pnp_solver(const CameraConfig& config) noexcept {
    return std::make_shared<PnPSolver>(config);
}

} // namespace fcs::L2
