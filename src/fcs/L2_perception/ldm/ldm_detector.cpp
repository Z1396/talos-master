#include "L2_perception/ldm/ldm_detector.hpp"

#include "L2_perception/ldm/ldm_geometry.hpp"
#include "core/armor_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace fcs::L2::ldm {

namespace {

[[nodiscard]] float
    min_candidate_score(const LdmMeshCandidate& candidate, const LdmDetectorConfig& config) {
    return static_cast<float>(
        (candidate.pair_indices.size() <= 2) ? config.min_preliminary_candidate_score_two_pair
                                             : config.min_preliminary_candidate_score);
}

[[nodiscard]] size_t candidate_cluster_pair_count(
    const LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs) {
    if (candidate.cluster_id < 0) {
        return candidate.pair_indices.size();
    }

    return static_cast<size_t>(
        std::count_if(pairs.begin(), pairs.end(), [&](const LightPair& pair) {
            return pair.cluster_id == candidate.cluster_id;
        }));
}

[[nodiscard]] bool has_min_isolated_two_pair_order_span(
    const LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs,
    const LdmDetectorConfig& config) {
    if (candidate.pair_indices.size() != 2u) {
        return true;
    }
    if (candidate_cluster_pair_count(candidate, pairs) != 2u) {
        return true;
    }
    if (!std::isfinite(config.min_isolated_two_pair_order_span_ratio)
        || config.min_isolated_two_pair_order_span_ratio < 0.0) {
        return false;
    }

    float mean_pair_layer = 0.0f;
    for (const int pair_idx : candidate.pair_indices) {
        if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
            return false;
        }
        mean_pair_layer += pair_layer_separation_px(pairs[static_cast<size_t>(pair_idx)]);
    }
    mean_pair_layer /= static_cast<float>(candidate.pair_indices.size());
    if (mean_pair_layer <= 1e-3f) {
        return false;
    }

    const auto first_pair_idx = static_cast<size_t>(candidate.pair_indices.front());
    const auto last_pair_idx  = static_cast<size_t>(candidate.pair_indices.back());
    const float order_span =
        std::abs(pairs[last_pair_idx].local_order_px - pairs[first_pair_idx].local_order_px);
    return order_span / mean_pair_layer
        >= static_cast<float>(config.min_isolated_two_pair_order_span_ratio);
}

[[nodiscard]] bool candidate_passes_detection_gate(
    const LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs,
    const LdmDetectorConfig& config) {
    if (static_cast<int>(candidate.pair_indices.size()) < config.min_pairs_for_detection) {
        return false;
    }
    if (candidate.preliminary_score < min_candidate_score(candidate, config)) {
        return false;
    }
    if (!has_min_isolated_two_pair_order_span(candidate, pairs, config)) {
        return false;
    }

    if (candidate.pair_indices.size() <= 2) {
        double mean_center_dy_px = 0.0;
        int valid_pair_count     = 0;
        for (const int pair_idx : candidate.pair_indices) {
            if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
                continue;
            }
            mean_center_dy_px += pairs[static_cast<size_t>(pair_idx)].center_dy_px;
            ++valid_pair_count;
        }
        if (valid_pair_count == 0) {
            return false;
        }
        mean_center_dy_px /= static_cast<double>(valid_pair_count);
        if (mean_center_dy_px < config.min_two_pair_mean_center_dy_px) {
            constexpr float kFlatTwoPairScoreMargin = 0.015f;
            if (candidate.preliminary_score
                < min_candidate_score(candidate, config) + kFlatTwoPairScoreMargin) {
                return false;
            }
        }
    }

    return true;
}

void retain_selected_candidate_support(std::vector<LdmMeshCandidate>& candidates) {
    if (candidates.empty()) {
        return;
    }

    auto selected_pair_indices = candidates.front().pair_indices;
    std::sort(selected_pair_indices.begin(), selected_pair_indices.end());
    candidates.erase(
        std::remove_if(
            candidates.begin(), candidates.end(),
            [&](const LdmMeshCandidate& candidate) {
                return !std::all_of(
                    candidate.pair_indices.begin(), candidate.pair_indices.end(),
                    [&](const int pair_idx) {
                        return std::binary_search(
                            selected_pair_indices.begin(), selected_pair_indices.end(), pair_idx);
                    });
            }),
        candidates.end());
}

[[nodiscard]] float min_projected_pair_vertical_ratio(const LdmDetectorConfig& config) {
    const double max_pose_angle =
        (std::isfinite(config.max_pose_angle_rad) && config.max_pose_angle_rad > 0.0)
            ? std::min(config.max_pose_angle_rad, std::numbers::pi_v<double> * 0.5 - 1e-3)
            : 0.872664626;
    return std::max(0.35f, 0.75f * static_cast<float>(std::cos(max_pose_angle)));
}

[[nodiscard]] cv::Point2f rect_center(const cv::Rect& rect) {
    return cv::Point2f(
        rect.x + static_cast<float>(rect.width) * 0.5f,
        rect.y + static_cast<float>(rect.height) * 0.5f);
}

[[nodiscard]] cv::Rect2f rect2f_from_rect(const cv::Rect& rect) {
    return cv::Rect2f(
        static_cast<float>(rect.x), static_cast<float>(rect.y), static_cast<float>(rect.width),
        static_cast<float>(rect.height));
}

[[nodiscard]] bool hue_matches_target_color(double hue, ArmorColor color) {
    switch (color) {
    case ArmorColor::Blue: return hue >= 90.0 && hue <= 140.0;
    case ArmorColor::Purple: return hue >= 125.0 && hue <= 165.0;
    case ArmorColor::Red: return (hue >= 0.0 && hue <= 30.0) || (hue >= 160.0 && hue <= 180.0);
    default:
        return hue_matches_target_color(hue, ArmorColor::Red)
            || hue_matches_target_color(hue, ArmorColor::Blue)
            || hue_matches_target_color(hue, ArmorColor::Purple);
    }
}

[[nodiscard]] cv::Mat
    threshold_target_color(const cv::Mat& image_bgr, ArmorColor color, int min_value = 80) {
    cv::Mat hsv;
    cv::cvtColor(image_bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    switch (color) {
    case ArmorColor::Blue:
        cv::inRange(hsv, cv::Scalar(90, 70, min_value), cv::Scalar(140, 255, 255), mask);
        break;
    case ArmorColor::Purple:
        cv::inRange(hsv, cv::Scalar(125, 70, min_value), cv::Scalar(165, 255, 255), mask);
        break;
    case ArmorColor::Red: {
        cv::Mat mask1, mask2;
        cv::inRange(hsv, cv::Scalar(0, 80, min_value), cv::Scalar(30, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(160, 80, min_value), cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;
        break;
    }
    default: {
        cv::Mat red    = threshold_target_color(image_bgr, ArmorColor::Red, min_value);
        cv::Mat blue   = threshold_target_color(image_bgr, ArmorColor::Blue, min_value);
        cv::Mat purple = threshold_target_color(image_bgr, ArmorColor::Purple, min_value);
        mask           = red | blue | purple;
        break;
    }
    }

    // Remove thin red borders / HUD overlays so they cannot connect unrelated pixels
    // into a single frame-sized contour.
    constexpr int border_px = 2;
    if (mask.rows > border_px * 2 && mask.cols > border_px * 2) {
        mask.rowRange(0, border_px).setTo(0);
        mask.rowRange(mask.rows - border_px, mask.rows).setTo(0);
        mask.colRange(0, border_px).setTo(0);
        mask.colRange(mask.cols - border_px, mask.cols).setTo(0);
    }
    return mask;
}

[[nodiscard]] std::optional<LightBlob> make_light_blob_from_contour(
    const std::vector<cv::Point>& contour, const cv::Mat& mask, const cv::Point& offset,
    const LdmDetectorConfig& config) {
    cv::Rect rect = cv::boundingRect(contour);
    if (offset.x != 0 || offset.y != 0) {
        rect.x += offset.x;
        rect.y += offset.y;
    }
    if (rect.width <= 0 || rect.height <= 0) {
        return std::nullopt;
    }

    const float aspect_ratio =
        static_cast<float>(rect.width) / static_cast<float>(std::max(rect.height, 1));
    if (aspect_ratio < static_cast<float>(config.min_blob_aspect_ratio)
        || aspect_ratio > static_cast<float>(config.max_blob_aspect_ratio)) {
        return std::nullopt;
    }

    const double contour_area   = cv::contourArea(contour);
    const int pixel_count       = cv::countNonZero(mask(rect));
    const double effective_area = (contour_area >= static_cast<double>(config.min_blob_area_px))
                                    ? contour_area
                                    : static_cast<double>(pixel_count);
    if (effective_area < static_cast<double>(config.min_blob_area_px)) {
        return std::nullopt;
    }

    const float rect_area      = static_cast<float>(std::max(1, rect.width * rect.height));
    const float fill_ratio_geo = static_cast<float>(contour_area) / rect_area;
    const float fill_ratio_px  = static_cast<float>(pixel_count) / rect_area;
    const bool passes_fill     = fill_ratio_geo >= static_cast<float>(config.min_blob_fill_ratio);
    const bool passes_sparse   = pixel_count >= config.min_sparse_blob_pixel_count;
    if (!passes_fill && !passes_sparse) {
        return std::nullopt;
    }

    const bool used_sparse_gate = !passes_fill;
    const float fill_ratio      = used_sparse_gate ? fill_ratio_px : fill_ratio_geo;
    return LightBlob{
        .rect         = rect2f_from_rect(rect),
        .center_px    = rect_center(rect),
        .area_px      = static_cast<float>(contour_area),
        .aspect_ratio = aspect_ratio,
        .fill_ratio   = fill_ratio};
}

[[nodiscard]] std::optional<LightBlob> make_narrow_yaw_blob_from_contour(
    const std::vector<cv::Point>& contour, const cv::Mat& hsv, const cv::Mat& mask,
    const cv::Point& offset, const LdmDetectorConfig& config, ArmorColor color) {
    cv::Rect rect = cv::boundingRect(contour);
    if (offset.x != 0 || offset.y != 0) {
        rect.x += offset.x;
        rect.y += offset.y;
    }
    if (rect.width <= 0 || rect.height <= 0) {
        return std::nullopt;
    }

    constexpr float kMinNarrowAspect = 0.15f;
    constexpr int kMaxNarrowWidth    = 5;
    constexpr int kMinNarrowHeight   = 10;
    constexpr int kMaxNarrowHeight   = 24;
    constexpr int kMinNarrowPixels   = 36;
    constexpr double kMinNarrowArea  = 20.0;

    const float aspect_ratio =
        static_cast<float>(rect.width) / static_cast<float>(std::max(rect.height, 1));
    if (aspect_ratio >= static_cast<float>(config.min_blob_aspect_ratio)
        || aspect_ratio < kMinNarrowAspect || rect.width > kMaxNarrowWidth
        || rect.height < kMinNarrowHeight || rect.height > kMaxNarrowHeight) {
        return std::nullopt;
    }
    if (rect.x < 0 || rect.y < 0 || rect.x + rect.width > hsv.cols
        || rect.y + rect.height > hsv.rows) {
        return std::nullopt;
    }

    const double contour_area = cv::contourArea(contour);
    const int pixel_count     = cv::countNonZero(mask(rect));
    if (contour_area < kMinNarrowArea || pixel_count < kMinNarrowPixels) {
        return std::nullopt;
    }

    const double mean_hue = cv::mean(hsv(rect), mask(rect))[0];
    if (!hue_matches_target_color(mean_hue, color)) {
        return std::nullopt;
    }

    const float rect_area     = static_cast<float>(std::max(1, rect.width * rect.height));
    const float fill_ratio_px = static_cast<float>(pixel_count) / rect_area;
    return LightBlob{
        .rect         = rect2f_from_rect(rect),
        .center_px    = rect_center(rect),
        .area_px      = static_cast<float>(contour_area),
        .aspect_ratio = aspect_ratio,
        .fill_ratio   = fill_ratio_px};
}

[[nodiscard]] std::vector<LightBlob> resolve_merged_light_blobs(
    const std::vector<LightBlob>& blobs, const std::vector<LightPair>& pairs,
    const std::vector<LdmMeshCandidate>& mesh_candidates, const LdmDetectorConfig& config) {
    const double pair_separation_m = config.geometry.pair_center_separation_m;
    const double window_length_m   = config.geometry.window_length_m;
    if (!std::isfinite(pair_separation_m) || pair_separation_m <= 0.0
        || !std::isfinite(window_length_m) || window_length_m <= 0.0
        || !std::isfinite(config.max_resolved_window_length_fraction)
        || config.max_resolved_window_length_fraction <= 0.0
        || !std::isfinite(config.max_merged_window_pair_separation_px)
        || config.max_merged_window_pair_separation_px <= 0.0) {
        return blobs;
    }

    const float max_single_window_ratio = static_cast<float>(
        config.max_resolved_window_length_fraction * window_length_m / pair_separation_m);
    std::vector<float> candidate_pair_separations(blobs.size(), 0.0f);
    for (const auto& candidate : mesh_candidates) {
        for (const int pair_idx : candidate.pair_indices) {
            if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
                continue;
            }
            const auto& pair = pairs[static_cast<size_t>(pair_idx)];
            for (const int blob_idx : {pair.top_blob_index, pair.bottom_blob_index}) {
                if (blob_idx < 0 || static_cast<size_t>(blob_idx) >= blobs.size()) {
                    continue;
                }
                candidate_pair_separations[static_cast<size_t>(blob_idx)] = std::max(
                    candidate_pair_separations[static_cast<size_t>(blob_idx)], pair.center_dy_px);
            }
        }
    }

    std::vector<LightBlob> resolved;
    resolved.reserve(blobs.size() * 2u);
    for (size_t blob_idx = 0; blob_idx < blobs.size(); ++blob_idx) {
        const auto& blob            = blobs[blob_idx];
        const float pair_separation = candidate_pair_separations[blob_idx];
        if (pair_separation <= 1e-3f
            || pair_separation > static_cast<float>(config.max_merged_window_pair_separation_px)
            || blob.rect.width / pair_separation <= max_single_window_ratio) {
            resolved.push_back(blob);
            continue;
        }

        const float left_width  = std::floor(blob.rect.width * 0.5f);
        const float right_width = blob.rect.width - left_width;
        if (left_width <= 1.0f || right_width <= 1.0f) {
            resolved.push_back(blob);
            continue;
        }

        for (int part = 0; part < 2; ++part) {
            LightBlob half  = blob;
            half.rect.x     = blob.rect.x + ((part == 0) ? 0.0f : left_width);
            half.rect.width = (part == 0) ? left_width : right_width;
            half.center_px  = cv::Point2f(
                half.rect.x + half.rect.width * 0.5f, half.rect.y + half.rect.height * 0.5f);
            half.area_px *= 0.5f;
            half.aspect_ratio = half.rect.width / std::max(half.rect.height, 1.0f);
            resolved.push_back(half);
        }
    }
    return resolved;
}

[[nodiscard]] bool
    has_narrow_yaw_pair_mate(const LightBlob& blob, const std::vector<LightBlob>& narrow_blobs) {
    constexpr float kMaxMateDx = 10.0f;
    constexpr float kMinMateDy = 20.0f;
    constexpr float kMaxMateDy = 60.0f;

    return std::any_of(narrow_blobs.begin(), narrow_blobs.end(), [&](const LightBlob& other) {
        if (&other == &blob) {
            return false;
        }
        const float dx = std::abs(other.center_px.x - blob.center_px.x);
        const float dy = std::abs(other.center_px.y - blob.center_px.y);
        return dx <= kMaxMateDx && dy >= kMinMateDy && dy <= kMaxMateDy;
    });
}

[[nodiscard]] bool has_full_mesh_regular_context(
    const LightBlob& blob, const std::vector<LightBlob>& regular_blobs) {
    constexpr float kContextDx            = 80.0f;
    constexpr float kContextDy            = 80.0f;
    constexpr int kMinRegularContextBlobs = 6;

    int nearby_count = 0;
    for (const auto& regular_blob : regular_blobs) {
        if (std::abs(regular_blob.center_px.x - blob.center_px.x) <= kContextDx
            && std::abs(regular_blob.center_px.y - blob.center_px.y) <= kContextDy) {
            ++nearby_count;
        }
    }
    return nearby_count >= kMinRegularContextBlobs;
}

[[nodiscard]] std::vector<LightBlob> detect_light_blobs(
    const cv::Mat& image_bgr, const LdmDetectorConfig& config, ArmorColor color) {
    if (image_bgr.empty()) {
        return {};
    }

    cv::Mat hsv;
    cv::cvtColor(image_bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask      = threshold_target_color(image_bgr, color);
    cv::Mat core_mask = threshold_target_color(image_bgr, color, 140);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBlob> blobs;
    std::vector<LightBlob> narrow_yaw_blobs;
    blobs.reserve(contours.size());
    narrow_yaw_blobs.reserve(contours.size());
    for (const auto& contour : contours) {
        const cv::Rect rect = cv::boundingRect(contour);
        if (rect.width <= 0 || rect.height <= 0) {
            continue;
        }

        std::vector<std::vector<cv::Point>> core_contours;
        cv::Mat core_roi = core_mask(rect);
        cv::findContours(core_roi, core_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<LightBlob> core_blobs;
        core_blobs.reserve(core_contours.size());
        for (const auto& core_contour : core_contours) {
            auto blob = make_light_blob_from_contour(core_contour, core_mask, rect.tl(), config);
            if (blob.has_value()) {
                core_blobs.push_back(*blob);
                continue;
            }
            auto narrow_blob = make_narrow_yaw_blob_from_contour(
                core_contour, hsv, core_mask, rect.tl(), config, color);
            if (narrow_blob.has_value()) {
                narrow_yaw_blobs.push_back(*narrow_blob);
            }
        }
        if (core_blobs.size() >= 2u) {
            blobs.insert(blobs.end(), core_blobs.begin(), core_blobs.end());
            continue;
        }

        auto blob = make_light_blob_from_contour(contour, mask, {}, config);
        if (blob.has_value()) {
            blobs.push_back(*blob);
        }
    }

    for (const auto& narrow_blob : narrow_yaw_blobs) {
        if (has_narrow_yaw_pair_mate(narrow_blob, narrow_yaw_blobs)
            && has_full_mesh_regular_context(narrow_blob, blobs)) {
            blobs.push_back(narrow_blob);
        }
    }

    std::sort(blobs.begin(), blobs.end(), [](const LightBlob& a, const LightBlob& b) {
        if (a.center_px.x != b.center_px.x) {
            return a.center_px.x < b.center_px.x;
        }
        return a.center_px.y < b.center_px.y;
    });
    return blobs;
}

constexpr float kClusterExpandXRatio      = 1.0f;
constexpr float kClusterExpandYRatio      = 3.5f;
constexpr size_t kPcaSeedBlobLimit        = 6;
constexpr size_t kMaxClusterBlobCount     = 20;
constexpr float kMaxPairOrderDeltaRatio   = 1.5f;
constexpr float kPairOrderScoreDeltaRatio = 2.5f;
constexpr float kMinAxisBalance           = 0.4f;

[[nodiscard]] float dot(const cv::Point2f& lhs, const cv::Point2f& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] cv::Point2f orient_axis(cv::Point2f axis, bool prefer_y_positive) {
    if (prefer_y_positive) {
        if (axis.y < 0.0f) {
            axis *= -1.0f;
        }
        return axis;
    }

    if (std::abs(axis.x) >= std::abs(axis.y)) {
        if (axis.x < 0.0f) {
            axis *= -1.0f;
        }
    } else if (axis.y < 0.0f) {
        axis *= -1.0f;
    }
    return axis;
}

[[nodiscard]] cv::Rect2f expanded_rect(const LightBlob& blob) {
    const float margin_x = blob.rect.width * kClusterExpandXRatio;
    const float margin_y = std::max(blob.rect.width, blob.rect.height) * kClusterExpandYRatio;
    return cv::Rect2f(
        blob.rect.x - margin_x, blob.rect.y - margin_y, blob.rect.width + 2.0f * margin_x,
        blob.rect.height + 2.0f * margin_y);
}

[[nodiscard]] bool rects_overlap(const cv::Rect2f& lhs, const cv::Rect2f& rhs) {
    const float left   = std::max(lhs.x, rhs.x);
    const float top    = std::max(lhs.y, rhs.y);
    const float right  = std::min(lhs.x + lhs.width, rhs.x + rhs.width);
    const float bottom = std::min(lhs.y + lhs.height, rhs.y + rhs.height);
    return right >= left && bottom >= top;
}

[[nodiscard]] std::vector<std::vector<size_t>> cluster_blob_indices(std::vector<LightBlob>& blobs) {
    std::vector<std::vector<size_t>> clusters;
    if (blobs.empty()) {
        return clusters;
    }

    std::vector<cv::Rect2f> grown_rects;
    grown_rects.reserve(blobs.size());
    for (const auto& blob : blobs) {
        grown_rects.push_back(expanded_rect(blob));
    }

    std::vector<bool> visited(blobs.size(), false);
    for (size_t seed_idx = 0; seed_idx < blobs.size(); ++seed_idx) {
        if (visited[seed_idx]) {
            continue;
        }

        std::vector<size_t> cluster;
        std::vector<size_t> stack{seed_idx};
        visited[seed_idx] = true;
        while (!stack.empty()) {
            const size_t idx = stack.back();
            stack.pop_back();
            cluster.push_back(idx);

            for (size_t other_idx = 0; other_idx < blobs.size(); ++other_idx) {
                if (visited[other_idx]) {
                    continue;
                }
                if (!rects_overlap(grown_rects[idx], grown_rects[other_idx])) {
                    continue;
                }
                visited[other_idx] = true;
                stack.push_back(other_idx);
            }
        }

        clusters.push_back(std::move(cluster));
    }

    std::sort(clusters.begin(), clusters.end(), [&](const auto& lhs, const auto& rhs) {
        const auto cluster_center_x = [&](const std::vector<size_t>& cluster) {
            float sum = 0.0f;
            for (const size_t idx : cluster) {
                sum += blobs[idx].center_px.x;
            }
            return sum / static_cast<float>(cluster.size());
        };
        return cluster_center_x(lhs) < cluster_center_x(rhs);
    });

    for (size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id) {
        for (const size_t blob_idx : clusters[cluster_id]) {
            blobs[blob_idx].cluster_id     = static_cast<int>(cluster_id);
            blobs[blob_idx].local_order_px = 0.0f;
            blobs[blob_idx].local_layer_px = 0.0f;
        }
    }
    return clusters;
}

struct AxisSplit {
    cv::Point2f layer_axis{};
    cv::Point2f order_axis{};
    float split_value{0.0f};
    float score{-1.0f};
    int matched_pair_count{0};
    float matched_pair_score{0.0f};
    int best_candidate_pair_count{0};
    float best_candidate_preliminary{0.0f};
    float best_candidate_score{0.0f};
};

struct MatchResult {
    int pair_count{0};
    float score{0.0f};
};

struct ProjectedPairSelection {
    size_t first_local_idx{0};
    size_t second_local_idx{0};
    float score{0.0f};
};

struct ProjectedMatching {
    MatchResult result{};
    std::vector<ProjectedPairSelection> selected_pairs{};
};

[[nodiscard]] bool better_match(const MatchResult& lhs, const MatchResult& rhs) {
    return lhs.pair_count > rhs.pair_count
        || (lhs.pair_count == rhs.pair_count && lhs.score > rhs.score);
}

[[nodiscard]] float pair_count_priority(int pair_count) {
    switch (pair_count) {
    case 1: return 0.25f;
    case 2: return 0.55f;
    case 3: return 0.82f;
    default: return 0.96f;
    }
}

[[nodiscard]] std::optional<float> score_projected_pair(
    const LightBlob& first, float first_order, float first_layer, const LightBlob& second,
    float second_order, float second_layer, const LdmDetectorConfig& config) {
    if ((first_layer <= 0.0f) == (second_layer <= 0.0f)) {
        return std::nullopt;
    }

    const float avg_w = (first.rect.width + second.rect.width) * 0.5f;
    const float avg_h = (first.rect.height + second.rect.height) * 0.5f;
    if (avg_w <= 1e-3f || avg_h <= 1e-3f) {
        return std::nullopt;
    }

    const float order_delta = std::abs(first_order - second_order);
    const float size_scale  = std::max({avg_w, avg_h, 1.0f});
    const float order_limit = kMaxPairOrderDeltaRatio * size_scale;
    if (order_delta > order_limit) {
        return std::nullopt;
    }

    const float layer_separation = std::abs(first_layer - second_layer);
    const float min_layer_sep =
        static_cast<float>(config.min_pair_center_dy_ratio) * std::max(avg_h, 1.0f);
    const float max_layer_sep =
        static_cast<float>(config.max_pair_center_dy_ratio) * std::max(avg_h, 1.0f);
    if (layer_separation < min_layer_sep || layer_separation > max_layer_sep) {
        return std::nullopt;
    }

    const cv::Point2f image_delta = second.center_px - first.center_px;
    const float image_distance =
        std::sqrt(image_delta.x * image_delta.x + image_delta.y * image_delta.y);
    if (image_distance <= 1e-3f) {
        return std::nullopt;
    }
    const float projected_vertical_ratio = std::abs(image_delta.y) / image_distance;
    if (projected_vertical_ratio < min_projected_pair_vertical_ratio(config)) {
        return std::nullopt;
    }

    const float width_delta =
        std::abs(first.rect.width - second.rect.width) / std::max(avg_w, 1.0f);
    const float height_delta =
        std::abs(first.rect.height - second.rect.height) / std::max(avg_h, 1.0f);
    if (width_delta > static_cast<float>(config.max_pair_size_delta_ratio)
        || height_delta > static_cast<float>(config.max_pair_size_delta_ratio)) {
        return std::nullopt;
    }

    const float order_score_limit = kPairOrderScoreDeltaRatio * size_scale;
    const float order_score =
        std::max(0.0f, 1.0f - order_delta / std::max(order_score_limit, 1.0f));
    const float width_score  = std::max(0.0f, 1.0f - width_delta);
    const float height_score = std::max(0.0f, 1.0f - height_delta);
    const float fill_score   = 0.5f * (first.fill_ratio + second.fill_ratio);
    const float layer_balance =
        std::abs(std::abs(first_layer) - std::abs(second_layer)) / std::max(layer_separation, 1.0f);
    const float balance_score = std::max(0.0f, 1.0f - layer_balance);

    return std::clamp(
        0.35f * order_score + 0.15f * balance_score + 0.15f * width_score + 0.15f * height_score
            + 0.20f * fill_score,
        0.0f, 1.0f);
}

[[nodiscard]] ProjectedMatching best_projected_matching(
    const std::vector<LightBlob>& blobs, const std::vector<size_t>& cluster_indices,
    const std::vector<float>& order_values, const std::vector<float>& layer_values,
    const LdmDetectorConfig& config) {
    if (cluster_indices.empty() || cluster_indices.size() > kMaxClusterBlobCount) {
        return {};
    }

    struct Candidate {
        size_t first_local_idx{0};
        size_t second_local_idx{0};
        float score{0.0f};
    };

    std::vector<Candidate> pair_candidates;
    pair_candidates.reserve(cluster_indices.size() * 3);
    std::vector<std::vector<int>> adjacency(cluster_indices.size());
    for (size_t first_local_idx = 0; first_local_idx < cluster_indices.size(); ++first_local_idx) {
        for (size_t second_local_idx = first_local_idx + 1;
             second_local_idx < cluster_indices.size(); ++second_local_idx) {
            const size_t first_blob_idx  = cluster_indices[first_local_idx];
            const size_t second_blob_idx = cluster_indices[second_local_idx];
            auto score                   = score_projected_pair(
                blobs[first_blob_idx], order_values[first_local_idx], layer_values[first_local_idx],
                blobs[second_blob_idx], order_values[second_local_idx],
                layer_values[second_local_idx], config);
            if (!score.has_value()) {
                continue;
            }

            const int candidate_idx = static_cast<int>(pair_candidates.size());
            pair_candidates.push_back(
                Candidate{
                    .first_local_idx  = first_local_idx,
                    .second_local_idx = second_local_idx,
                    .score            = *score,
                });
            adjacency[first_local_idx].push_back(candidate_idx);
            adjacency[second_local_idx].push_back(candidate_idx);
        }
    }
    if (pair_candidates.empty()) {
        return {};
    }

    const uint64_t full_mask = (uint64_t{1} << cluster_indices.size()) - 1u;
    std::vector<bool> cached(static_cast<size_t>(full_mask + 1u), false);
    std::vector<MatchResult> best(static_cast<size_t>(full_mask + 1u));
    std::vector<int> best_choice(static_cast<size_t>(full_mask + 1u), -2);
    const auto solve = [&](auto&& self, uint64_t used_mask) -> MatchResult {
        if (used_mask == full_mask) {
            return {};
        }
        if (cached[static_cast<size_t>(used_mask)]) {
            return best[static_cast<size_t>(used_mask)];
        }

        size_t first_unused = 0;
        while ((used_mask & (uint64_t{1} << first_unused)) != 0u) {
            ++first_unused;
        }

        MatchResult best_result = self(self, used_mask | (uint64_t{1} << first_unused));
        int choice              = -1;
        for (const int candidate_idx : adjacency[first_unused]) {
            const auto& candidate    = pair_candidates[static_cast<size_t>(candidate_idx)];
            const size_t other_idx   = (candidate.first_local_idx == first_unused)
                                         ? candidate.second_local_idx
                                         : candidate.first_local_idx;
            const uint64_t other_bit = (uint64_t{1} << other_idx);
            if ((used_mask & other_bit) != 0u) {
                continue;
            }

            auto candidate_result =
                self(self, used_mask | (uint64_t{1} << first_unused) | other_bit);
            candidate_result.pair_count += 1;
            candidate_result.score += candidate.score;
            if (better_match(candidate_result, best_result)) {
                best_result = candidate_result;
                choice      = candidate_idx;
            }
        }

        cached[static_cast<size_t>(used_mask)]      = true;
        best[static_cast<size_t>(used_mask)]        = best_result;
        best_choice[static_cast<size_t>(used_mask)] = choice;
        return best_result;
    };

    ProjectedMatching matching;
    matching.result = solve(solve, 0u);

    uint64_t used_mask = 0u;
    while (used_mask != full_mask) {
        size_t first_unused = 0;
        while ((used_mask & (uint64_t{1} << first_unused)) != 0u) {
            ++first_unused;
        }

        const int candidate_idx = best_choice[static_cast<size_t>(used_mask)];
        if (candidate_idx < 0) {
            used_mask |= (uint64_t{1} << first_unused);
            continue;
        }

        const auto& candidate = pair_candidates[static_cast<size_t>(candidate_idx)];
        matching.selected_pairs.push_back(
            ProjectedPairSelection{
                .first_local_idx  = candidate.first_local_idx,
                .second_local_idx = candidate.second_local_idx,
                .score            = candidate.score,
            });
        const size_t other_idx = (candidate.first_local_idx == first_unused)
                                   ? candidate.second_local_idx
                                   : candidate.first_local_idx;
        used_mask |= (uint64_t{1} << first_unused) | (uint64_t{1} << other_idx);
    }
    return matching;
}

[[nodiscard]] float values_stddev(const std::vector<float>& values) {
    if (values.size() <= 1) {
        return 0.0f;
    }

    const float mean =
        std::accumulate(values.begin(), values.end(), 0.0f) / static_cast<float>(values.size());
    float sq_sum = 0.0f;
    for (const float value : values) {
        const float delta = value - mean;
        sq_sum += delta * delta;
    }
    return std::sqrt(sq_sum / static_cast<float>(values.size()));
}

[[nodiscard]] std::optional<AxisSplit> evaluate_axis_split(
    const std::vector<LightBlob>& blobs, const std::vector<size_t>& cluster_indices,
    const cv::Point2f& mean, cv::Point2f layer_axis, cv::Point2f order_axis,
    const LdmDetectorConfig& config) {
    if (cluster_indices.size() < 2) {
        return std::nullopt;
    }

    layer_axis = orient_axis(layer_axis, true);
    order_axis = orient_axis(order_axis, false);

    std::vector<float> projected_values;
    projected_values.reserve(cluster_indices.size());
    for (const size_t blob_idx : cluster_indices) {
        const cv::Point2f centered = blobs[blob_idx].center_px - mean;
        projected_values.push_back(dot(centered, layer_axis));
    }

    std::sort(projected_values.begin(), projected_values.end());
    AxisSplit best_split{
        .layer_axis  = layer_axis,
        .order_axis  = order_axis,
        .split_value = 0.0f,
        .score       = -1.0f,
    };

    for (size_t split = 1; split < projected_values.size(); ++split) {
        const size_t lower_count = split;
        const size_t upper_count = projected_values.size() - split;
        const float balance      = static_cast<float>(std::min(lower_count, upper_count))
                            / static_cast<float>(std::max(lower_count, upper_count));
        if (balance < kMinAxisBalance) {
            continue;
        }

        const float gap = projected_values[split] - projected_values[split - 1];
        if (gap <= 1e-3f) {
            continue;
        }

        std::vector<float> lower(projected_values.begin(), projected_values.begin() + split);
        std::vector<float> upper(
            projected_values.begin() + static_cast<std::ptrdiff_t>(split), projected_values.end());
        const float score =
            gap * (0.55f + 0.45f * balance) / (4.0f + values_stddev(lower) + values_stddev(upper));
        if (score <= best_split.score) {
            continue;
        }

        best_split.split_value = 0.5f * (projected_values[split - 1] + projected_values[split]);
        best_split.score       = score;
    }

    if (best_split.score <= 0.0f) {
        return std::nullopt;
    }

    std::vector<float> order_values;
    std::vector<float> layer_values;
    order_values.reserve(cluster_indices.size());
    layer_values.reserve(cluster_indices.size());
    for (const size_t blob_idx : cluster_indices) {
        const cv::Point2f centered = blobs[blob_idx].center_px - mean;
        order_values.push_back(dot(centered, best_split.order_axis));
        layer_values.push_back(dot(centered, best_split.layer_axis) - best_split.split_value);
    }
    const auto matching =
        best_projected_matching(blobs, cluster_indices, order_values, layer_values, config);
    best_split.matched_pair_count = matching.result.pair_count;
    best_split.matched_pair_score = matching.result.score;

    std::vector<LightPair> projected_pairs;
    projected_pairs.reserve(matching.selected_pairs.size());
    for (const auto& selection : matching.selected_pairs) {
        const size_t first_blob_idx  = cluster_indices[selection.first_local_idx];
        const size_t second_blob_idx = cluster_indices[selection.second_local_idx];
        const auto& first            = blobs[first_blob_idx];
        const auto& second           = blobs[second_blob_idx];
        const size_t top_blob_idx =
            (first.center_px.y <= second.center_px.y) ? first_blob_idx : second_blob_idx;
        const size_t bottom_blob_idx =
            (top_blob_idx == first_blob_idx) ? second_blob_idx : first_blob_idx;
        const auto& top    = blobs[top_blob_idx];
        const auto& bottom = blobs[bottom_blob_idx];
        projected_pairs.push_back(
            LightPair{
                .top_blob_index    = static_cast<int>(top_blob_idx),
                .bottom_blob_index = static_cast<int>(bottom_blob_idx),
                .top_center_px     = top.center_px,
                .bottom_center_px  = bottom.center_px,
                .midpoint_px       = (top.center_px + bottom.center_px) * 0.5f,
                .center_dx_px      = std::abs(bottom.center_px.x - top.center_px.x),
                .center_dy_px      = std::abs(bottom.center_px.y - top.center_px.y),
                .cluster_id        = 0,
                .local_order_px    = 0.5f
                                * (order_values[selection.first_local_idx]
                                   + order_values[selection.second_local_idx]),
                .local_layer_sep_px = std::abs(
                    layer_values[selection.first_local_idx]
                    - layer_values[selection.second_local_idx]),
                .score = selection.score,
            });
    }

    auto preliminary_candidates = build_detection_mesh_candidates(projected_pairs, config);
    for (const auto& candidate : preliminary_candidates) {
        if (!candidate_passes_detection_gate(candidate, projected_pairs, config)) {
            continue;
        }

        const int pair_count = static_cast<int>(candidate.pair_indices.size());
        const float detection_score =
            0.55f * pair_count_priority(pair_count) + 0.45f * candidate.preliminary_score;
        if (pair_count > best_split.best_candidate_pair_count
            || (pair_count == best_split.best_candidate_pair_count
                && detection_score > best_split.best_candidate_score)) {
            best_split.best_candidate_pair_count  = pair_count;
            best_split.best_candidate_preliminary = candidate.preliminary_score;
            best_split.best_candidate_score       = detection_score;
        }
    }
    return best_split;
}

[[nodiscard]] std::optional<AxisSplit> estimate_cluster_axes(
    const std::vector<LightBlob>& blobs, const std::vector<size_t>& cluster_indices,
    const LdmDetectorConfig& config) {
    if (cluster_indices.size() < 2) {
        return std::nullopt;
    }

    std::vector<size_t> seed_indices = cluster_indices;
    std::sort(seed_indices.begin(), seed_indices.end(), [&](size_t lhs, size_t rhs) {
        return blobs[lhs].area_px > blobs[rhs].area_px;
    });
    if (seed_indices.size() > kPcaSeedBlobLimit) {
        seed_indices.resize(kPcaSeedBlobLimit);
    }
    if (seed_indices.size() < 2) {
        return std::nullopt;
    }

    cv::Mat samples(static_cast<int>(seed_indices.size()), 2, CV_32F);
    for (size_t row = 0; row < seed_indices.size(); ++row) {
        samples.at<float>(static_cast<int>(row), 0) = blobs[seed_indices[row]].center_px.x;
        samples.at<float>(static_cast<int>(row), 1) = blobs[seed_indices[row]].center_px.y;
    }

    cv::PCA pca(samples, cv::Mat(), cv::PCA::DATA_AS_ROW);
    const cv::Point2f mean(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1));
    const cv::Point2f axis0(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
    const cv::Point2f axis1(pca.eigenvectors.at<float>(1, 0), pca.eigenvectors.at<float>(1, 1));

    std::vector<std::optional<AxisSplit>> candidates;
    candidates.push_back(evaluate_axis_split(blobs, cluster_indices, mean, axis0, axis1, config));
    candidates.push_back(evaluate_axis_split(blobs, cluster_indices, mean, axis1, axis0, config));
    candidates.push_back(evaluate_axis_split(
        blobs, cluster_indices, mean, cv::Point2f(0.0f, 1.0f), cv::Point2f(1.0f, 0.0f), config));
    candidates.push_back(evaluate_axis_split(
        blobs, cluster_indices, mean, cv::Point2f(1.0f, 0.0f), cv::Point2f(0.0f, 1.0f), config));

    std::optional<AxisSplit> best_candidate;
    for (const auto& candidate : candidates) {
        if (!candidate.has_value()) {
            continue;
        }
        if (!best_candidate.has_value()
            || candidate->best_candidate_score > best_candidate->best_candidate_score
            || (std::abs(candidate->best_candidate_score - best_candidate->best_candidate_score)
                    <= 1e-3f
                && candidate->best_candidate_pair_count > best_candidate->best_candidate_pair_count)
            || (std::abs(candidate->best_candidate_score - best_candidate->best_candidate_score)
                    <= 1e-3f
                && candidate->best_candidate_pair_count == best_candidate->best_candidate_pair_count
                && candidate->matched_pair_count > best_candidate->matched_pair_count)
            || (std::abs(candidate->best_candidate_score - best_candidate->best_candidate_score)
                    <= 1e-3f
                && candidate->best_candidate_pair_count == best_candidate->best_candidate_pair_count
                && candidate->matched_pair_count == best_candidate->matched_pair_count
                && candidate->score > best_candidate->score)) {
            best_candidate = candidate;
        }
    }
    return best_candidate;
}

void annotate_blob_clusters(std::vector<LightBlob>& blobs, const LdmDetectorConfig& config) {
    const auto clusters = cluster_blob_indices(blobs);
    for (const auto& cluster_indices : clusters) {
        auto axes = estimate_cluster_axes(blobs, cluster_indices, config);
        if (!axes.has_value()) {
            continue;
        }

        cv::Point2f mean(0.0f, 0.0f);
        for (const size_t seed_idx : cluster_indices) {
            mean += blobs[seed_idx].center_px;
        }
        mean *= (1.0f / static_cast<float>(cluster_indices.size()));

        // Re-estimate the mean from the seed PCA inputs so local coordinates match the split.
        std::vector<size_t> seed_indices = cluster_indices;
        std::sort(seed_indices.begin(), seed_indices.end(), [&](size_t lhs, size_t rhs) {
            return blobs[lhs].area_px > blobs[rhs].area_px;
        });
        if (seed_indices.size() > kPcaSeedBlobLimit) {
            seed_indices.resize(kPcaSeedBlobLimit);
        }
        if (!seed_indices.empty()) {
            mean = cv::Point2f(0.0f, 0.0f);
            for (const size_t seed_idx : seed_indices) {
                mean += blobs[seed_idx].center_px;
            }
            mean *= (1.0f / static_cast<float>(seed_indices.size()));
        }

        for (const size_t blob_idx : cluster_indices) {
            const cv::Point2f centered     = blobs[blob_idx].center_px - mean;
            blobs[blob_idx].local_order_px = dot(centered, axes->order_axis);
            blobs[blob_idx].local_layer_px = dot(centered, axes->layer_axis) - axes->split_value;
        }
    }
}

struct PairMetrics {
    size_t top_blob_idx{0};
    size_t bottom_blob_idx{0};
    float order_delta{0.0f};
    float layer_separation{0.0f};
    float score{0.0f};
};

[[nodiscard]] std::optional<PairMetrics> score_pair(
    const LightBlob& first, size_t first_idx, const LightBlob& second, size_t second_idx,
    const LdmDetectorConfig& config) {
    if (first.cluster_id < 0 || first.cluster_id != second.cluster_id) {
        return std::nullopt;
    }
    auto score = score_projected_pair(
        first, first.local_order_px, first.local_layer_px, second, second.local_order_px,
        second.local_layer_px, config);
    if (!score.has_value()) {
        return std::nullopt;
    }

    const float order_delta      = std::abs(first.local_order_px - second.local_order_px);
    const float layer_separation = std::abs(first.local_layer_px - second.local_layer_px);

    PairMetrics metrics;
    metrics.top_blob_idx     = (first.center_px.y <= second.center_px.y) ? first_idx : second_idx;
    metrics.bottom_blob_idx  = (metrics.top_blob_idx == first_idx) ? second_idx : first_idx;
    metrics.order_delta      = order_delta;
    metrics.layer_separation = layer_separation;
    metrics.score            = *score;
    return metrics;
}

struct PairCandidate {
    size_t first_local_idx{0};
    size_t second_local_idx{0};
    PairMetrics metrics{};
    float score{0.0f};
};

[[nodiscard]] std::vector<LightPair>
    build_light_pairs(const std::vector<LightBlob>& blobs, const LdmDetectorConfig& config) {
    std::vector<LightPair> pairs;
    if (blobs.size() < 2) {
        return pairs;
    }

    std::vector<int> cluster_ids;
    cluster_ids.reserve(blobs.size());
    for (const auto& blob : blobs) {
        if (blob.cluster_id < 0) {
            continue;
        }
        if (std::find(cluster_ids.begin(), cluster_ids.end(), blob.cluster_id)
            == cluster_ids.end()) {
            cluster_ids.push_back(blob.cluster_id);
        }
    }

    for (const int cluster_id : cluster_ids) {
        std::vector<size_t> cluster_blob_indices;
        cluster_blob_indices.reserve(blobs.size());
        for (size_t blob_idx = 0; blob_idx < blobs.size(); ++blob_idx) {
            if (blobs[blob_idx].cluster_id == cluster_id) {
                cluster_blob_indices.push_back(blob_idx);
            }
        }
        if (cluster_blob_indices.size() < 2 || cluster_blob_indices.size() > kMaxClusterBlobCount) {
            continue;
        }

        std::vector<PairCandidate> pair_candidates;
        pair_candidates.reserve(cluster_blob_indices.size() * 3);
        std::vector<std::vector<int>> adjacency(cluster_blob_indices.size());
        for (size_t first_local_idx = 0; first_local_idx < cluster_blob_indices.size();
             ++first_local_idx) {
            for (size_t second_local_idx = first_local_idx + 1;
                 second_local_idx < cluster_blob_indices.size(); ++second_local_idx) {
                const size_t first_blob_idx  = cluster_blob_indices[first_local_idx];
                const size_t second_blob_idx = cluster_blob_indices[second_local_idx];
                auto metrics                 = score_pair(
                    blobs[first_blob_idx], first_blob_idx, blobs[second_blob_idx], second_blob_idx,
                    config);
                if (!metrics.has_value()) {
                    continue;
                }

                const int candidate_idx = static_cast<int>(pair_candidates.size());
                pair_candidates.push_back(
                    PairCandidate{
                        .first_local_idx  = first_local_idx,
                        .second_local_idx = second_local_idx,
                        .metrics          = *metrics,
                        .score            = metrics->score,
                    });
                adjacency[first_local_idx].push_back(candidate_idx);
                adjacency[second_local_idx].push_back(candidate_idx);
            }
        }
        if (pair_candidates.empty()) {
            continue;
        }

        const uint64_t full_mask = (uint64_t{1} << cluster_blob_indices.size()) - 1u;
        std::vector<bool> cached(static_cast<size_t>(full_mask + 1u), false);
        std::vector<MatchResult> best(static_cast<size_t>(full_mask + 1u));
        std::vector<int> best_choice(static_cast<size_t>(full_mask + 1u), -2);

        const auto solve = [&](auto&& self, uint64_t used_mask) -> MatchResult {
            if (used_mask == full_mask) {
                return {};
            }
            if (cached[static_cast<size_t>(used_mask)]) {
                return best[static_cast<size_t>(used_mask)];
            }

            size_t first_unused = 0;
            while ((used_mask & (uint64_t{1} << first_unused)) != 0u) {
                ++first_unused;
            }

            MatchResult best_result = self(self, used_mask | (uint64_t{1} << first_unused));
            int choice              = -1;
            for (const int candidate_idx : adjacency[first_unused]) {
                const auto& candidate    = pair_candidates[static_cast<size_t>(candidate_idx)];
                const size_t other_idx   = (candidate.first_local_idx == first_unused)
                                             ? candidate.second_local_idx
                                             : candidate.first_local_idx;
                const uint64_t other_bit = (uint64_t{1} << other_idx);
                if ((used_mask & other_bit) != 0u) {
                    continue;
                }

                auto candidate_result =
                    self(self, used_mask | (uint64_t{1} << first_unused) | other_bit);
                candidate_result.pair_count += 1;
                candidate_result.score += candidate.score;
                if (better_match(candidate_result, best_result)) {
                    best_result = candidate_result;
                    choice      = candidate_idx;
                }
            }

            cached[static_cast<size_t>(used_mask)]      = true;
            best[static_cast<size_t>(used_mask)]        = best_result;
            best_choice[static_cast<size_t>(used_mask)] = choice;
            return best_result;
        };

        solve(solve, 0u);

        uint64_t used_mask = 0u;
        while (used_mask != full_mask) {
            size_t first_unused = 0;
            while ((used_mask & (uint64_t{1} << first_unused)) != 0u) {
                ++first_unused;
            }

            const int candidate_idx = best_choice[static_cast<size_t>(used_mask)];
            if (candidate_idx < 0) {
                used_mask |= (uint64_t{1} << first_unused);
                continue;
            }

            const auto& candidate      = pair_candidates[static_cast<size_t>(candidate_idx)];
            const auto& top            = blobs[candidate.metrics.top_blob_idx];
            const auto& bottom         = blobs[candidate.metrics.bottom_blob_idx];
            const cv::Point2f midpoint = (top.center_px + bottom.center_px) * 0.5f;
            pairs.push_back(
                LightPair{
                    .top_blob_index     = static_cast<int>(candidate.metrics.top_blob_idx),
                    .bottom_blob_index  = static_cast<int>(candidate.metrics.bottom_blob_idx),
                    .top_center_px      = top.center_px,
                    .bottom_center_px   = bottom.center_px,
                    .midpoint_px        = midpoint,
                    .center_dx_px       = std::abs(bottom.center_px.x - top.center_px.x),
                    .center_dy_px       = std::abs(bottom.center_px.y - top.center_px.y),
                    .cluster_id         = cluster_id,
                    .local_order_px     = 0.5f * (top.local_order_px + bottom.local_order_px),
                    .local_layer_sep_px = candidate.metrics.layer_separation,
                    .score              = candidate.score,
                });

            const size_t other_idx = (candidate.first_local_idx == first_unused)
                                       ? candidate.second_local_idx
                                       : candidate.first_local_idx;
            used_mask |= (uint64_t{1} << first_unused) | (uint64_t{1} << other_idx);
        }
    }

    std::sort(pairs.begin(), pairs.end(), [](const LightPair& a, const LightPair& b) {
        if (a.cluster_id != b.cluster_id) {
            return a.cluster_id < b.cluster_id;
        }
        return a.local_order_px < b.local_order_px;
    });
    return pairs;
}

/// Use the PCA-derived ordering and vertical-separation profile to assign faces
/// relative to the visible face looking most directly at the camera.
[[nodiscard]] bool assign_face_indices_for_candidate(
    LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs) {
    if (candidate.pair_indices.size() < 2) {
        return false;
    }
    if (candidate.pair_indices.size() != candidate.octagon_face_indices.size()) {
        candidate.octagon_face_indices.resize(candidate.pair_indices.size());
    }

    struct Ordered {
        int pair_idx;
        float order;
        float sep;
    };
    std::vector<Ordered> ordered;
    ordered.reserve(pairs.size());

    const int cluster_id = [&]() {
        for (const int pidx : candidate.pair_indices) {
            if (pidx < 0 || static_cast<size_t>(pidx) >= pairs.size()) {
                continue;
            }
            return pairs[static_cast<size_t>(pidx)].cluster_id;
        }
        return -1;
    }();

    for (size_t pair_idx = 0; pair_idx < pairs.size(); ++pair_idx) {
        if (cluster_id >= 0 && pairs[pair_idx].cluster_id != cluster_id) {
            continue;
        }
        ordered.push_back(
            {static_cast<int>(pair_idx), pairs[pair_idx].local_order_px,
             pair_layer_separation_px(pairs[pair_idx])});
    }
    if (ordered.size() < 2) {
        return false;
    }
    std::sort(ordered.begin(), ordered.end(), [](const Ordered& a, const Ordered& b) {
        return a.order < b.order;
    });

    size_t front_idx = 0;
    for (size_t i = 1; i < ordered.size(); ++i) {
        if (ordered[i].sep > ordered[front_idx].sep) {
            front_idx = i;
        }
    }

    struct AssignedPair {
        int pair_idx;
        size_t rank;
        int face;
    };
    std::vector<AssignedPair> assigned;
    assigned.reserve(candidate.pair_indices.size());

    for (size_t i = 0; i < candidate.pair_indices.size(); ++i) {
        const auto rank_it = std::find_if(ordered.begin(), ordered.end(), [&](const Ordered& item) {
            return item.pair_idx == candidate.pair_indices[i];
        });
        if (rank_it == ordered.end()) {
            return false;
        }
        const size_t rank = static_cast<size_t>(std::distance(ordered.begin(), rank_it));
        int face          = static_cast<int>(rank) - static_cast<int>(front_idx);
        face %= 8;
        if (face < 0) {
            face += 8;
        }
        assigned.push_back(
            AssignedPair{.pair_idx = candidate.pair_indices[i], .rank = rank, .face = face});
    }

    std::sort(assigned.begin(), assigned.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.rank < rhs.rank;
    });

    candidate.pair_indices.clear();
    candidate.octagon_face_indices.clear();
    candidate.pair_indices.reserve(assigned.size());
    candidate.octagon_face_indices.reserve(assigned.size());
    for (const auto& item : assigned) {
        candidate.pair_indices.push_back(item.pair_idx);
        candidate.octagon_face_indices.push_back(item.face);
    }
    return true;
}

[[nodiscard]] std::optional<LdmDetection>
    detect_laser_module(const cv::Mat& image, const LdmDetectorConfig& config, ArmorColor color) {
    auto blobs = detect_light_blobs(image, config, color);
    annotate_blob_clusters(blobs, config);
    auto pairs = build_light_pairs(blobs, config);
    if (pairs.empty()) {
        return std::nullopt;
    }

    auto mesh_candidates = build_detection_mesh_candidates(pairs, config);
    mesh_candidates.erase(
        std::remove_if(
            mesh_candidates.begin(), mesh_candidates.end(),
            [&](const LdmMeshCandidate& candidate) {
                return !candidate_passes_detection_gate(candidate, pairs, config);
            }),
        mesh_candidates.end());
    if (mesh_candidates.empty()) {
        return std::nullopt;
    }
    retain_selected_candidate_support(mesh_candidates);

    auto resolved_blobs = resolve_merged_light_blobs(blobs, pairs, mesh_candidates, config);
    if (resolved_blobs.size() != blobs.size()) {
        annotate_blob_clusters(resolved_blobs, config);
        auto resolved_pairs      = build_light_pairs(resolved_blobs, config);
        auto resolved_candidates = build_detection_mesh_candidates(resolved_pairs, config);
        resolved_candidates.erase(
            std::remove_if(
                resolved_candidates.begin(), resolved_candidates.end(),
                [&](const LdmMeshCandidate& candidate) {
                    return !candidate_passes_detection_gate(candidate, resolved_pairs, config);
                }),
            resolved_candidates.end());
        if (!resolved_candidates.empty()
            && resolved_candidates.front().pair_indices.size()
                   > mesh_candidates.front().pair_indices.size()) {
            retain_selected_candidate_support(resolved_candidates);
            blobs           = std::move(resolved_blobs);
            pairs           = std::move(resolved_pairs);
            mesh_candidates = std::move(resolved_candidates);
        }
    }

    std::vector<int> selected_pair_indices = mesh_candidates.front().pair_indices;
    std::sort(selected_pair_indices.begin(), selected_pair_indices.end());
    selected_pair_indices.erase(
        std::unique(selected_pair_indices.begin(), selected_pair_indices.end()),
        selected_pair_indices.end());
    if (selected_pair_indices.empty()) {
        return std::nullopt;
    }

    std::vector<LightPair> selected_pairs;
    selected_pairs.reserve(selected_pair_indices.size());
    std::vector<int> selected_blob_indices;
    selected_blob_indices.reserve(selected_pair_indices.size() * 2);
    std::vector<int> pair_remap(pairs.size(), -1);
    for (const int pair_idx : selected_pair_indices) {
        if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
            continue;
        }
        const auto& pair                          = pairs[static_cast<size_t>(pair_idx)];
        pair_remap[static_cast<size_t>(pair_idx)] = static_cast<int>(selected_pairs.size());
        selected_pairs.push_back(pair);
        selected_blob_indices.push_back(pair.top_blob_index);
        selected_blob_indices.push_back(pair.bottom_blob_index);
    }
    if (selected_pairs.empty()) {
        return std::nullopt;
    }

    std::sort(selected_blob_indices.begin(), selected_blob_indices.end());
    selected_blob_indices.erase(
        std::unique(selected_blob_indices.begin(), selected_blob_indices.end()),
        selected_blob_indices.end());

    std::vector<LightBlob> selected_blobs;
    selected_blobs.reserve(selected_blob_indices.size());
    std::vector<int> blob_remap(blobs.size(), -1);
    for (const int old_blob_idx : selected_blob_indices) {
        if (old_blob_idx < 0 || static_cast<size_t>(old_blob_idx) >= blobs.size()) {
            continue;
        }
        blob_remap[static_cast<size_t>(old_blob_idx)] = static_cast<int>(selected_blobs.size());
        selected_blobs.push_back(blobs[static_cast<size_t>(old_blob_idx)]);
    }

    for (auto& pair : selected_pairs) {
        if (pair.top_blob_index >= 0
            && static_cast<size_t>(pair.top_blob_index) < blob_remap.size()) {
            pair.top_blob_index = blob_remap[static_cast<size_t>(pair.top_blob_index)];
        }
        if (pair.bottom_blob_index >= 0
            && static_cast<size_t>(pair.bottom_blob_index) < blob_remap.size()) {
            pair.bottom_blob_index = blob_remap[static_cast<size_t>(pair.bottom_blob_index)];
        }
    }

    for (auto& candidate : mesh_candidates) {
        for (auto& pair_idx : candidate.pair_indices) {
            if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pair_remap.size()) {
                pair_idx = -1;
                continue;
            }
            pair_idx = pair_remap[static_cast<size_t>(pair_idx)];
        }
        candidate.pair_indices.erase(
            std::remove(candidate.pair_indices.begin(), candidate.pair_indices.end(), -1),
            candidate.pair_indices.end());
    }
    mesh_candidates.erase(
        std::remove_if(
            mesh_candidates.begin(), mesh_candidates.end(),
            [](const LdmMeshCandidate& candidate) { return candidate.pair_indices.empty(); }),
        mesh_candidates.end());
    if (mesh_candidates.empty()) {
        return std::nullopt;
    }

    // Assign face indices relative to the visible face with the largest apparent height.
    for (auto& candidate : mesh_candidates) {
        (void)assign_face_indices_for_candidate(candidate, selected_pairs);
    }

    LdmDetection detection;
    detection.color                  = color;
    detection.blobs                  = std::move(selected_blobs);
    detection.pairs                  = std::move(selected_pairs);
    detection.mesh_candidates        = std::move(mesh_candidates);
    detection.rect                   = bounding_rect_from_pairs(detection.pairs);
    detection.selected_candidate_idx = 0;
    detection.center_image_px        = detection.mesh_candidates.front().estimated_center_image_px;
    return detection;
}

[[nodiscard]] std::pair<size_t, float> best_candidate_signature(const LdmDetection& detection) {
    size_t max_candidate_pairs = 0;
    float max_candidate_score  = 0.0f;
    for (const auto& candidate : detection.mesh_candidates) {
        max_candidate_pairs = std::max(max_candidate_pairs, candidate.pair_indices.size());
        max_candidate_score = std::max(max_candidate_score, candidate.preliminary_score);
    }
    return {max_candidate_pairs, max_candidate_score};
}

[[nodiscard]] bool
    detection_better_than(const LdmDetection& lhs, const std::optional<LdmDetection>& rhs) {
    if (!rhs.has_value()) {
        return true;
    }

    if (lhs.pair_count() != rhs->pair_count()) {
        return lhs.pair_count() > rhs->pair_count();
    }

    const auto [lhs_candidate_pairs, lhs_candidate_score] = best_candidate_signature(lhs);
    const auto [rhs_candidate_pairs, rhs_candidate_score] = best_candidate_signature(*rhs);
    if (lhs_candidate_pairs != rhs_candidate_pairs) {
        return lhs_candidate_pairs > rhs_candidate_pairs;
    }
    return lhs_candidate_score > rhs_candidate_score;
}

} // namespace

LdmDetector::LdmDetector(LdmDetectorConfig config) noexcept
    : config_(config) {}

std::expected<std::optional<LdmDetection>, DetectorError>
    LdmDetector::detect(const cv::Mat& image) const noexcept {
    if (image.empty()) {
        return std::unexpected(DetectorError::InvalidImage);
    }

    std::array<ArmorColor, 4> colors = {
        config_.target_color,
        ArmorColor::Red,
        ArmorColor::Blue,
        ArmorColor::Purple,
    };

    std::optional<LdmDetection> best_detection;
    std::array<bool, 4> tried_colors = {false, false, false, false};
    for (const ArmorColor color : colors) {
        const auto color_index = static_cast<size_t>(color);
        if (color_index >= tried_colors.size() || tried_colors[color_index]) {
            continue;
        }
        tried_colors[color_index] = true;

        auto detection_result = detect_laser_module(image, config_, color);
        if (!detection_result.has_value()) {
            continue;
        }
        if (detection_better_than(*detection_result, best_detection)) {
            best_detection = std::move(*detection_result);
        }
    }
    return best_detection;
}

std::expected<std::optional<LdmDetection>, DetectorError>
    LdmDetector::detect(const cv::Mat& image, ArmorColor color) const noexcept {
    if (image.empty()) {
        return std::unexpected(DetectorError::InvalidImage);
    }
    return detect_laser_module(image, config_, color);
}
} // namespace fcs::L2::ldm
