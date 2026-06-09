#pragma once

#include "L3_estimation/tracker/types.hpp"
#include "camera_config.hpp"
#include "frame.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace fcs::L2 {

struct ArmorReadbackRoiConfig {
    bool enabled{false};
    double stale_timeout_s{0.20};
    double margin_ratio_x{0.10};
    double margin_ratio_y{0.10};
    std::array<double, 3> box_size_m{0.8, 0.8, 0.6};
};

struct TrackerReadbackSnapshot {
    bool valid{false};
    uint64_t timestamp_ns{0};
    uint64_t projection_timestamp_ns{0};
    int selected_armor_id{0};
    int rough_selected_armor_id{0};
    L3::TrackerOutput tracker{};
};

class TrackerReadbackCache {
public:
    TrackerReadbackCache() noexcept = default;

    TrackerReadbackCache(const TrackerReadbackCache& other) noexcept {
        std::scoped_lock lock(other.mutex_);
        snapshot_ = other.snapshot_;
    }

    TrackerReadbackCache& operator=(const TrackerReadbackCache& other) noexcept {
        if (this == &other) {
            return *this;
        }

        std::scoped_lock lock(mutex_, other.mutex_);
        snapshot_ = other.snapshot_;
        return *this;
    }

    TrackerReadbackCache(TrackerReadbackCache&& other) noexcept {
        std::scoped_lock lock(other.mutex_);
        snapshot_ = other.snapshot_;
    }

    TrackerReadbackCache& operator=(TrackerReadbackCache&& other) noexcept {
        if (this == &other) {
            return *this;
        }

        std::scoped_lock lock(mutex_, other.mutex_);
        snapshot_ = other.snapshot_;
        return *this;
    }

    [[nodiscard]] TrackerReadbackSnapshot load() const noexcept {
        std::scoped_lock lock(mutex_);
        return snapshot_;
    }

    void store(const TrackerReadbackSnapshot& snapshot) noexcept {
        std::scoped_lock lock(mutex_);
        snapshot_ = snapshot;
    }

    void invalidate(uint64_t timestamp_ns = 0) noexcept {
        std::scoped_lock lock(mutex_);
        snapshot_ = TrackerReadbackSnapshot{.valid = false, .timestamp_ns = timestamp_ns};
    }

private:
    mutable std::mutex mutex_;
    TrackerReadbackSnapshot snapshot_{};
};

struct BackendInputResolution {
    int width{0};
    int height{0};

    [[nodiscard]] bool valid() const noexcept { return width > 0 && height > 0; }

    [[nodiscard]] double aspect_ratio() const noexcept {
        return valid() ? static_cast<double>(width) / static_cast<double>(height) : 1.0;
    }
};

[[nodiscard]] inline bool tracker_snapshot_is_fresh(
    const TrackerReadbackSnapshot& snapshot, const ArmorReadbackRoiConfig& config,
    uint64_t image_timestamp_ns) noexcept {
    if (!snapshot.valid || snapshot.timestamp_ns == 0
        || image_timestamp_ns < snapshot.timestamp_ns) {
        return false;
    }

    const auto max_age_ns = static_cast<uint64_t>(std::max(0.0, config.stale_timeout_s) * 1e9);
    return image_timestamp_ns - snapshot.timestamp_ns <= max_age_ns;
}

[[nodiscard]] inline bool is_valid_raw_roi(const cv::Rect2f& roi) noexcept {
    return std::isfinite(roi.x) && std::isfinite(roi.y) && std::isfinite(roi.width)
        && std::isfinite(roi.height) && roi.width > 0.0f && roi.height > 0.0f;
}

[[nodiscard]] inline cv::Rect2f
    expand_raw_roi(const cv::Rect2f& roi, const ArmorReadbackRoiConfig& config) noexcept {
    if (!is_valid_raw_roi(roi)) {
        return {};
    }

    const float margin_ratio_x =
        static_cast<float>(std::max(0.0, std::min(config.margin_ratio_x, 10.0)));
    const float margin_ratio_y =
        static_cast<float>(std::max(0.0, std::min(config.margin_ratio_y, 10.0)));
    const float margin_x = roi.width * margin_ratio_x;
    const float margin_y = roi.height * margin_ratio_y;

    return cv::Rect2f(
        roi.x - margin_x, roi.y - margin_y, roi.width + 2.0f * margin_x,
        roi.height + 2.0f * margin_y);
}

[[nodiscard]] inline std::optional<cv::Rect> resolve_readback_roi(
    const cv::Size& frame_size, const cv::Rect2f& raw_roi, const ArmorReadbackRoiConfig& config,
    const BackendInputResolution& input_resolution) noexcept {
    if (frame_size.width <= 0 || frame_size.height <= 0 || !input_resolution.valid()
        || !is_valid_raw_roi(raw_roi)) {
        return std::nullopt;
    }

    const cv::Rect2f expanded = expand_raw_roi(raw_roi, config);
    if (!is_valid_raw_roi(expanded)) {
        return std::nullopt;
    }

    double final_w      = std::max<double>(expanded.width, input_resolution.width);
    double final_h      = std::max<double>(expanded.height, input_resolution.height);
    const double aspect = input_resolution.aspect_ratio();

    if (final_w / final_h < aspect) {
        final_w = final_h * aspect;
    } else {
        final_h = final_w / aspect;
    }

    if (final_w > frame_size.width || final_h > frame_size.height) {
        return std::nullopt;
    }

    const double cx = expanded.x + expanded.width * 0.5;
    const double cy = expanded.y + expanded.height * 0.5;
    double x        = cx - final_w * 0.5;
    double y        = cy - final_h * 0.5;

    x = std::clamp(x, 0.0, static_cast<double>(frame_size.width) - final_w);
    y = std::clamp(y, 0.0, static_cast<double>(frame_size.height) - final_h);

    const int left   = std::max(0, static_cast<int>(std::floor(x)));
    const int top    = std::max(0, static_cast<int>(std::floor(y)));
    const int right  = std::min(frame_size.width, static_cast<int>(std::ceil(x + final_w)));
    const int bottom = std::min(frame_size.height, static_cast<int>(std::ceil(y + final_h)));
    if (right <= left || bottom <= top) {
        return std::nullopt;
    }

    return cv::Rect(left, top, right - left, bottom - top);
}

[[nodiscard]] inline std::optional<std::pair<Eigen::Vector3d, double>>
    tracker_box_center_and_yaw(const L3::TrackerOutput& tracker) noexcept {
    if (const auto* robot = tracker.robot_state()) {
        return std::pair{
            Eigen::Vector3d(
                robot->position.x(), robot->position.y(), 0.5 * (robot->position.z() + robot->z1)),
            robot->yaw};
    }

    if (const auto* outpost = tracker.outpost_state()) {
        const double mean_z = (outpost->z[0] + outpost->z[1] + outpost->z[2]) / 3.0;
        return std::pair{
            Eigen::Vector3d(outpost->position.x(), outpost->position.y(), mean_z), outpost->yaw};
    }

    return std::nullopt;
}

[[nodiscard]] inline std::vector<Eigen::Vector4d>
    tracker_armor_poses(const L3::TrackerOutput& tracker) noexcept {
    if (const auto* robot = tracker.robot_state()) {
        const auto poses = robot->armor_poses();
        return {poses.begin(), poses.end()};
    }

    if (const auto* outpost = tracker.outpost_state()) {
        const auto poses = outpost->armor_poses();
        return {poses.begin(), poses.end()};
    }

    return {};
}

[[nodiscard]] inline std::optional<cv::Rect2f> project_box_to_image(
    const Eigen::Vector3d& center_odom, double yaw,
    const fast_tf::FrameTransform<fast_tf::camera_optical, fast_tf::odom>& T_odom_camera,
    const CameraConfig& camera_config, const std::array<double, 3>& box_size_m) noexcept {
    const double fx = camera_config.camera_matrix(0, 0);
    const double fy = camera_config.camera_matrix(1, 1);
    const double cx = camera_config.camera_matrix(0, 2);
    const double cy = camera_config.camera_matrix(1, 2);
    if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) || !std::isfinite(cy)
        || fx <= 0.0 || fy <= 0.0) {
        return std::nullopt;
    }

    const double hx = std::max(0.0, box_size_m[0]) * 0.5;
    const double hy = std::max(0.0, box_size_m[1]) * 0.5;
    const double hz = std::max(0.0, box_size_m[2]) * 0.5;
    if (hx <= 0.0 || hy <= 0.0 || hz <= 0.0) {
        return std::nullopt;
    }

    const Eigen::AngleAxisd yaw_rot(yaw, Eigen::Vector3d::UnitZ());
    const Eigen::Matrix3d R_odom_camera = T_odom_camera.rotation();
    const Eigen::Vector3d t_odom_camera = T_odom_camera.translation();

    auto project_point = [&](const Eigen::Vector3d& point_odom) -> std::optional<cv::Point2f> {
        const Eigen::Vector3d point_camera = R_odom_camera * point_odom + t_odom_camera;
        if (!std::isfinite(point_camera.x()) || !std::isfinite(point_camera.y())
            || !std::isfinite(point_camera.z()) || point_camera.z() <= 1e-3) {
            return std::nullopt;
        }

        const double u = fx * point_camera.x() / point_camera.z() + cx;
        const double v = fy * point_camera.y() / point_camera.z() + cy;
        if (!std::isfinite(u) || !std::isfinite(v)) {
            return std::nullopt;
        }

        return cv::Point2f(static_cast<float>(u), static_cast<float>(v));
    };

    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();

    for (const double sx : {-hx, hx}) {
        for (const double sy : {-hy, hy}) {
            for (const double sz : {-hz, hz}) {
                const Eigen::Vector3d point_odom =
                    center_odom + yaw_rot * Eigen::Vector3d(sx, sy, sz);
                const auto pixel = project_point(point_odom);
                if (!pixel) {
                    return std::nullopt;
                }

                min_x = std::min(min_x, pixel->x);
                min_y = std::min(min_y, pixel->y);
                max_x = std::max(max_x, pixel->x);
                max_y = std::max(max_y, pixel->y);
            }
        }
    }

    cv::Rect2f roi(min_x, min_y, max_x - min_x, max_y - min_y);
    if (!is_valid_raw_roi(roi)) {
        return std::nullopt;
    }
    return roi;
}

[[nodiscard]] inline std::optional<cv::Rect2f> project_tracker_box_to_image(
    const L3::TrackerOutput& tracker,
    const fast_tf::FrameTransform<fast_tf::camera_optical, fast_tf::odom>& T_odom_camera,
    const CameraConfig& camera_config, const std::array<double, 3>& box_size_m) noexcept {
    const auto center_and_yaw = tracker_box_center_and_yaw(tracker);
    if (!center_and_yaw) {
        return std::nullopt;
    }
    return project_box_to_image(
        center_and_yaw->first, center_and_yaw->second, T_odom_camera, camera_config, box_size_m);
}

} // namespace fcs::L2
