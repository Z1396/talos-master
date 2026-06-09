#pragma once

#include "core/armor_types.hpp"
#include "frame.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string_view>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/core.h>
#include <magic_enum.hpp>
#include <opencv2/imgproc.hpp>

namespace fcs::L2::ldm {
enum class DetectorError { InvalidImage };

struct ldm_frame {
    static constexpr std::string_view frame_id = "ldm";
    using ancestor                             = void;
};

using CameraLdmTransform = fast_tf::FrameTransform<fast_tf::camera_optical, ldm_frame>;
using OdomLdmTransform   = fast_tf::FrameTransform<fast_tf::odom, ldm_frame>;

struct LdmCandidatePose {
    CameraLdmTransform camera{};
    OdomLdmTransform odom{};
};

enum class LdmDepthQuality : uint8_t {
    None = 0,
    BearingOnly,
    Constrained,
    Stable,
};

struct LightBlob {
    cv::Rect2f rect{};
    cv::Point2f center_px{};
    float area_px{0.0f};
    float aspect_ratio{0.0f};
    float fill_ratio{0.0f};
    int cluster_id{-1};
    float local_order_px{0.0f};
    float local_layer_px{0.0f};
};

struct LightPair {
    int top_blob_index{-1};
    int bottom_blob_index{-1};
    cv::Point2f top_center_px{};
    cv::Point2f bottom_center_px{};
    cv::Point2f midpoint_px{};
    float center_dx_px{0.0f};
    float center_dy_px{0.0f};
    int cluster_id{-1};
    float local_order_px{0.0f};
    float local_layer_sep_px{0.0f};
    float score{0.0f};
};

struct LdmMeshCandidate {
    std::vector<int> pair_indices{};
    std::vector<int> octagon_face_indices{};
    int cluster_id{-1};
    bool solved{false};
    bool depth_valid{false};
    float preliminary_score{0.0f};
    float reprojection_rmse_px{std::numeric_limits<float>::quiet_NaN()};
    float score{0.0f};
    cv::Point2f estimated_center_image_px{};
    LdmCandidatePose pose{};
    std::vector<cv::Point2f> projected_outline_image{};
};

struct LdmDetection {
    uint64_t timestamp_ns{0};
    uint64_t frame_id{0};
    cv::Rect2f rect{};
    ArmorColor color = ArmorColor::Neutral;
    bool accurate{false};
    std::vector<LightBlob> blobs{};
    std::vector<LightPair> pairs{};
    std::vector<LdmMeshCandidate> mesh_candidates{};
    std::optional<int> selected_candidate_idx{};
    std::optional<cv::Point2f> center_image_px{};

    [[nodiscard]] size_t pair_count() const noexcept { return pairs.size(); }
};

struct LdmMeasurement {
    uint64_t timestamp_ns{0};
    uint64_t frame_id{0};
    ArmorColor color{ArmorColor::Neutral};
    bool accurate{false};
    int pair_count_total{0};
    int selected_pair_count{0};
    cv::Point2f center_image_px{};
    Eigen::Vector3d bearing_cam{0.0, 0.0, 1.0};
    std::optional<CameraLdmTransform> transform_cam{};
    std::optional<OdomLdmTransform> transform_odom{};
    LdmDepthQuality depth_quality{LdmDepthQuality::None};
    float confidence{0.0f};
    std::vector<LdmMeshCandidate> mesh_candidates{};
    std::optional<int> selected_candidate_idx{};
};

} // namespace fcs::L2::ldm

namespace fmt {

template <>
struct formatter<fcs::L2::ldm::LdmDepthQuality> : formatter<std::string_view> {
    auto format(const fcs::L2::ldm::LdmDepthQuality quality, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(quality), ctx);
    }
};

} // namespace fmt
