#pragma once

#include "ldm_config.hpp"
#include "types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <numeric>
#include <vector>

namespace fcs::L2::ldm {

[[nodiscard]] inline double octagon_face_center_radius_m(const LdmGeometryConfig& geometry) {
    return geometry.octagon_circumradius_m * std::cos(std::numbers::pi_v<double> / 8.0);
}

[[nodiscard]] inline std::array<cv::Point3f, 2>
    pair_model_points_for_face(const LdmGeometryConfig& geometry, int face_index) {
    const double angle      = static_cast<double>(face_index) * (std::numbers::pi_v<double> / 4.0);
    const double radius     = octagon_face_center_radius_m(geometry);
    const double half_y_sep = geometry.pair_center_separation_m * 0.5;
    const float x           = static_cast<float>(radius * std::sin(angle));
    const float z           = static_cast<float>(-radius * std::cos(angle));
    const float top_y       = static_cast<float>(-half_y_sep);
    const float bottom_y    = static_cast<float>(half_y_sep);
    return {cv::Point3f(x, top_y, z), cv::Point3f(x, bottom_y, z)};
}

[[nodiscard]] inline std::array<cv::Point3f, 16>
    volume_outline_points(const LdmGeometryConfig& geometry) {
    std::array<cv::Point3f, 16> points{};
    const double radius       = geometry.octagon_circumradius_m;
    const float top_y         = static_cast<float>(-geometry.volume_height_m * 0.5);
    const float bottom_y      = static_cast<float>(geometry.volume_height_m * 0.5);
    const double angle_offset = std::numbers::pi_v<double> / 8.0;

    for (int i = 0; i < 8; ++i) {
        const double angle =
            angle_offset + static_cast<double>(i) * (std::numbers::pi_v<double> / 4.0);
        const float x                       = static_cast<float>(radius * std::sin(angle));
        const float z                       = static_cast<float>(-radius * std::cos(angle));
        points[static_cast<size_t>(i)]      = cv::Point3f(x, top_y, z);
        points[static_cast<size_t>(i) + 8u] = cv::Point3f(x, bottom_y, z);
    }
    return points;
}

[[nodiscard]] inline cv::Rect2f bounding_rect_from_pairs(const std::vector<LightPair>& pairs) {
    if (pairs.empty()) {
        return {};
    }

    std::vector<cv::Point2f> pts;
    pts.reserve(pairs.size() * 2);
    for (const auto& pair : pairs) {
        pts.push_back(pair.top_center_px);
        pts.push_back(pair.bottom_center_px);
    }
    return cv::boundingRect(pts);
}

[[nodiscard]] inline float pair_layer_separation_px(const LightPair& pair) {
    return (pair.local_layer_sep_px > 0.0f) ? pair.local_layer_sep_px : pair.center_dy_px;
}

[[nodiscard]] inline float pair_group_score(
    const std::vector<LightPair>& pairs, const std::vector<int>& ordered_pair_indices, size_t start,
    size_t length, const LdmDetectorConfig& config) {
    if (length == 0) {
        return 0.0f;
    }

    float mean_pair_score = 0.0f;
    float mean_pair_layer = 0.0f;
    for (size_t i = 0; i < length; ++i) {
        const auto pair_idx = static_cast<size_t>(ordered_pair_indices[start + i]);
        const auto& pair    = pairs[pair_idx];
        mean_pair_score += pair.score;
        mean_pair_layer += pair_layer_separation_px(pair);
    }
    mean_pair_score /= static_cast<float>(length);
    mean_pair_layer /= static_cast<float>(length);

    float gap_consistency = 1.0f;
    float layer_alignment = 1.0f;

    if (length >= 2) {
        std::vector<float> gaps;
        gaps.reserve(length - 1);
        for (size_t i = start + 1; i < start + length; ++i) {
            const auto lhs_idx = static_cast<size_t>(ordered_pair_indices[i - 1]);
            const auto rhs_idx = static_cast<size_t>(ordered_pair_indices[i]);
            const float gap    = pairs[rhs_idx].local_order_px - pairs[lhs_idx].local_order_px;
            if (gap <= 0.0f) {
                return 0.0f;
            }
            gaps.push_back(gap);
        }

        const float mean_gap =
            std::accumulate(gaps.begin(), gaps.end(), 0.0f) / static_cast<float>(gaps.size());
        if (mean_gap <= 1e-3f) {
            return 0.0f;
        }

        float sq_sum = 0.0f;
        for (const float gap : gaps) {
            const float delta = gap - mean_gap;
            sq_sum += delta * delta;
        }
        const float std_gap = std::sqrt(sq_sum / static_cast<float>(gaps.size()));
        const float gap_cv  = std_gap / mean_gap;
        if (gap_cv > static_cast<float>(config.max_gap_cv)) {
            if (length < 3) {
                return 0.0f;
            }
            // 3+ pairs: non-uniform gaps are expected for slanted-face projections.
            // Let the PnP solver judge instead of filtering here.
            gap_consistency = 0.1f;
        } else {
            gap_consistency =
                std::max(0.0f, 1.0f - 0.5f * gap_cv / static_cast<float>(config.max_gap_cv));
        }
    }

    if (mean_pair_layer > 1e-3f) {
        float max_delta = 0.0f;
        for (size_t i = 0; i < length; ++i) {
            const auto pair_idx = static_cast<size_t>(ordered_pair_indices[start + i]);
            max_delta           = std::max(
                max_delta, std::abs(pair_layer_separation_px(pairs[pair_idx]) - mean_pair_layer));
        }
        layer_alignment =
            std::max(0.0f, 1.0f - max_delta / std::max(1.0f, mean_pair_layer * 0.35f));
    }

    const float coverage_bonus = 0.05f * static_cast<float>((length > 2) ? (length - 2) : 0u);
    return std::clamp(
        0.55f * mean_pair_score + 0.30f * gap_consistency + 0.15f * layer_alignment
            + coverage_bonus,
        0.0f, 1.0f);
}

inline void append_mesh_candidate(
    std::vector<LdmMeshCandidate>* candidates, const std::vector<LightPair>& pairs,
    const std::vector<int>& ordered_pair_indices, size_t start, size_t length, int cluster_id,
    const LdmDetectorConfig& config) {
    LdmMeshCandidate candidate;
    candidate.cluster_id = cluster_id;
    candidate.pair_indices.assign(
        ordered_pair_indices.begin() + static_cast<std::ptrdiff_t>(start),
        ordered_pair_indices.begin() + static_cast<std::ptrdiff_t>(start + length));
    candidate.preliminary_score =
        pair_group_score(pairs, ordered_pair_indices, start, length, config);
    if (candidate.preliminary_score <= 0.0f) {
        return;
    }

    cv::Point2f center(0.0f, 0.0f);
    for (const int pidx : candidate.pair_indices) {
        center += pairs[static_cast<size_t>(pidx)].midpoint_px;
    }
    candidate.estimated_center_image_px =
        center * (1.0f / static_cast<float>(candidate.pair_indices.size()));
    candidates->push_back(std::move(candidate));
}

inline void sort_mesh_candidates(std::vector<LdmMeshCandidate>* candidates) {
    std::sort(
        candidates->begin(), candidates->end(),
        [](const LdmMeshCandidate& a, const LdmMeshCandidate& b) {
            if (a.pair_indices.size() != b.pair_indices.size()) {
                return a.pair_indices.size() > b.pair_indices.size();
            }
            return a.preliminary_score > b.preliminary_score;
        });
}

[[nodiscard]] inline std::vector<LdmMeshCandidate> build_preliminary_mesh_candidates(
    const std::vector<LightPair>& pairs, const LdmDetectorConfig& config) {
    std::vector<LdmMeshCandidate> candidates;
    if (pairs.empty()) {
        return candidates;
    }

    std::vector<int> cluster_ids;
    cluster_ids.reserve(pairs.size());
    for (const auto& pair : pairs) {
        if (pair.cluster_id < 0) {
            continue;
        }
        if (std::find(cluster_ids.begin(), cluster_ids.end(), pair.cluster_id)
            == cluster_ids.end()) {
            cluster_ids.push_back(pair.cluster_id);
        }
    }

    for (const int cluster_id : cluster_ids) {
        std::vector<int> ordered_pair_indices;
        ordered_pair_indices.reserve(pairs.size());
        for (size_t pair_idx = 0; pair_idx < pairs.size(); ++pair_idx) {
            if (pairs[pair_idx].cluster_id == cluster_id) {
                ordered_pair_indices.push_back(static_cast<int>(pair_idx));
            }
        }
        if (ordered_pair_indices.empty()) {
            continue;
        }

        std::sort(ordered_pair_indices.begin(), ordered_pair_indices.end(), [&](int lhs, int rhs) {
            return pairs[static_cast<size_t>(lhs)].local_order_px
                 < pairs[static_cast<size_t>(rhs)].local_order_px;
        });

        for (size_t length = ordered_pair_indices.size(); length >= 1u; --length) {
            for (size_t start = 0; start + length <= ordered_pair_indices.size(); ++start) {
                append_mesh_candidate(
                    &candidates, pairs, ordered_pair_indices, start, length, cluster_id, config);
            }
            if (length == 1u) {
                break;
            }
        }
    }

    sort_mesh_candidates(&candidates);
    return candidates;
}

[[nodiscard]] inline std::vector<LdmMeshCandidate> build_detection_mesh_candidates(
    const std::vector<LightPair>& pairs, const LdmDetectorConfig& config) {
    std::vector<LdmMeshCandidate> candidates;
    if (pairs.empty()) {
        return candidates;
    }

    std::vector<int> cluster_ids;
    cluster_ids.reserve(pairs.size());
    for (const auto& pair : pairs) {
        if (pair.cluster_id < 0) {
            continue;
        }
        if (std::find(cluster_ids.begin(), cluster_ids.end(), pair.cluster_id)
            == cluster_ids.end()) {
            cluster_ids.push_back(pair.cluster_id);
        }
    }

    const size_t min_candidate_length =
        std::max<size_t>(2u, static_cast<size_t>(std::max(config.min_pairs_for_detection, 0)));
    for (const int cluster_id : cluster_ids) {
        std::vector<int> ordered_pair_indices;
        ordered_pair_indices.reserve(pairs.size());
        for (size_t pair_idx = 0; pair_idx < pairs.size(); ++pair_idx) {
            if (pairs[pair_idx].cluster_id == cluster_id) {
                ordered_pair_indices.push_back(static_cast<int>(pair_idx));
            }
        }
        if (ordered_pair_indices.size() < min_candidate_length) {
            continue;
        }

        std::sort(ordered_pair_indices.begin(), ordered_pair_indices.end(), [&](int lhs, int rhs) {
            return pairs[static_cast<size_t>(lhs)].local_order_px
                 < pairs[static_cast<size_t>(rhs)].local_order_px;
        });

        append_mesh_candidate(
            &candidates, pairs, ordered_pair_indices, 0, ordered_pair_indices.size(), cluster_id,
            config);

        float mean_pair_layer = 0.0f;
        for (const int pair_idx : ordered_pair_indices) {
            mean_pair_layer += pair_layer_separation_px(pairs[static_cast<size_t>(pair_idx)]);
        }
        mean_pair_layer /= static_cast<float>(ordered_pair_indices.size());
        if (!std::isfinite(mean_pair_layer) || mean_pair_layer <= 1e-3f
            || !std::isfinite(config.max_adjacent_face_order_gap_ratio)
            || config.max_adjacent_face_order_gap_ratio <= 0.0) {
            continue;
        }

        std::vector<size_t> split_points;
        for (size_t i = 1; i < ordered_pair_indices.size(); ++i) {
            const auto lhs_idx = static_cast<size_t>(ordered_pair_indices[i - 1]);
            const auto rhs_idx = static_cast<size_t>(ordered_pair_indices[i]);
            const float gap    = pairs[rhs_idx].local_order_px - pairs[lhs_idx].local_order_px;
            if (gap / mean_pair_layer
                > static_cast<float>(config.max_adjacent_face_order_gap_ratio)) {
                split_points.push_back(i);
            }
        }
        if (split_points.empty()) {
            continue;
        }

        size_t segment_start = 0;
        for (const size_t split_point : split_points) {
            const size_t length = split_point - segment_start;
            if (length >= min_candidate_length) {
                append_mesh_candidate(
                    &candidates, pairs, ordered_pair_indices, segment_start, length, cluster_id,
                    config);
            }
            segment_start = split_point;
        }
        const size_t tail_length = ordered_pair_indices.size() - segment_start;
        if (tail_length >= min_candidate_length) {
            append_mesh_candidate(
                &candidates, pairs, ordered_pair_indices, segment_start, tail_length, cluster_id,
                config);
        }
    }

    sort_mesh_candidates(&candidates);
    return candidates;
}

} // namespace fcs::L2::ldm
