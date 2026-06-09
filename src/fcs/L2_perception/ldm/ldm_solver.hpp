#pragma once

#include "L2_perception/ldm/ldm_geometry.hpp"
#include "L2_perception/ldm/types.hpp"
#include "camera_config.hpp"
#include "frame.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <expected>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace fcs::L2::ldm {

// ============================================================================
// YPD spherical coordinate helpers (yaw-pitch-distance)
//
// Camera-frame translation is reparameterized into a physically meaningful
// spherical triplet:
//   yaw     = atan2(x, z)          – bearing in the XZ plane
//   pitch   = atan2(y, hypot(x,z)) – elevation
//   distance = ||t||               – range
//
// This decouples direction from depth, making Ceres bounds physically
// interpretable and avoiding the coordinate coupling that Cartesian (x,y,z)
// induces in the Jacobian.
// ============================================================================
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

class LdmSolver {
public:
    using OdomCameraTransform = fast_tf::FrameTransform<fast_tf::odom, fast_tf::camera_optical>;

    /// Prior pose hint from a tracker or previous frame.  Used to constrain
    /// the Ceres distance parameter band during refinement, preventing the
    /// solver from drifting into depth basins that are physically implausible
    /// given the inter-frame motion budget.
    struct PosePrior {
        cv::Vec3d rvec{};
        cv::Vec3d tvec{};
    };

    explicit LdmSolver(
        const CameraConfig& config, const LdmDetectorConfig& detector_config) noexcept
        : config_(detector_config) {
        cv::eigen2cv(config.camera_matrix, camera_matrix_);
        cv::eigen2cv(config.distort_coefficient, dist_coeffs_);

        const double fx   = std::abs(config.camera_matrix(0, 0));
        const double fy   = std::abs(config.camera_matrix(1, 1));
        residual_scale_x_ = (std::isfinite(fx) && fx > 1e-6) ? fx : 1.0;
        residual_scale_y_ = (std::isfinite(fy) && fy > 1e-6) ? fy : 1.0;

        const double mean_focal_scale = 0.5 * (residual_scale_x_ + residual_scale_y_);
        huber_delta_px_               = std::max(1.0, 0.005 * mean_focal_scale);
    }

    [[nodiscard]] std::expected<LdmMeasurement, std::string> solve(
        const LdmDetection& detection, const OdomCameraTransform& T_odom_camera,
        const std::optional<PosePrior>& prior = std::nullopt) const {
        if (detection.pairs.empty()) {
            // return std::unexpected(std::string("solve: detection has no pairs (NoPairs)"));
            return LdmMeasurement{
                .timestamp_ns = detection.timestamp_ns,
                .frame_id     = detection.frame_id,
                .color        = detection.color,
                .accurate     = detection.accurate,
            };
        }

        std::vector<LdmMeshCandidate> evaluated_candidates;
        const auto evaluate_candidates = [&](const std::vector<LdmMeshCandidate>& base_candidates) {
            for (const auto& base_candidate : base_candidates) {
                if (base_candidate.pair_indices.empty()) {
                    continue;
                }

                auto candidate = base_candidate;
                assign_visible_face_indices(candidate, detection.pairs);
                if (!has_face_assignment(candidate)) {
                    continue;
                }
                evaluate_candidate(candidate, detection.pairs, T_odom_camera, prior);
                evaluated_candidates.push_back(std::move(candidate));
            }
        };

        // ── Phase 1: Evaluate detector-provided candidates first ──
        if (!detection.mesh_candidates.empty()) {
            evaluated_candidates.reserve(detection.mesh_candidates.size() * 16);
            evaluate_candidates(detection.mesh_candidates);
        }

        const auto has_solved_candidate = [&]() {
            return std::any_of(
                evaluated_candidates.begin(), evaluated_candidates.end(),
                [](const LdmMeshCandidate& candidate) { return candidate.solved; });
        };

        // If detector-side hypotheses cannot produce a pose, fall back to the
        // exhaustive contiguous-arc hypotheses used by the solver.
        if (evaluated_candidates.empty() || !has_solved_candidate()
            || detection.mesh_candidates.size() > 1u) {
            auto preliminary_candidates =
                build_preliminary_mesh_candidates(detection.pairs, config_);
            if (preliminary_candidates.empty() && evaluated_candidates.empty()) {
                return std::unexpected(
                    std::string("solve: no preliminary mesh candidates (NoCandidate)"));
            }
            evaluated_candidates.reserve(
                evaluated_candidates.size() + preliminary_candidates.size() * 16);
            evaluate_candidates(preliminary_candidates);
        }

        if (evaluated_candidates.empty()) {
            return std::unexpected(
                std::string("solve: no evaluated candidates after face assignment (NoCandidate)"));
        }

        // ── Phase 2: Build combined candidates ──
        auto combined_candidates =
            build_combined_candidates(evaluated_candidates, detection.pairs, T_odom_camera, prior);
        evaluated_candidates.insert(
            evaluated_candidates.end(), combined_candidates.begin(), combined_candidates.end());

        // ── Phase 3: Select best candidate ──
        std::optional<size_t> best_idx;
        for (size_t i = 0; i < evaluated_candidates.size(); ++i) {
            const auto& candidate = evaluated_candidates[i];
            if (!best_idx.has_value()
                || candidate_selection_better(candidate, evaluated_candidates[*best_idx])) {
                best_idx = i;
            }
        }
        if (!best_idx.has_value()) {
            return std::unexpected(std::string("solve: no best candidate selected (NoCandidate)"));
        }

        // ── Phase 4: Build output measurement ──
        const size_t selected_idx = *best_idx;
        const auto selected       = evaluated_candidates[selected_idx];

        LdmMeasurement measurement;
        measurement.timestamp_ns        = detection.timestamp_ns;
        measurement.frame_id            = detection.frame_id;
        measurement.color               = detection.color;
        measurement.accurate            = detection.accurate;
        measurement.pair_count_total    = static_cast<int>(detection.pairs.size());
        measurement.selected_pair_count = static_cast<int>(selected.pair_indices.size());
        measurement.center_image_px     = selected.estimated_center_image_px;
        measurement.bearing_cam     = bearing_for_image_point(selected.estimated_center_image_px);
        measurement.mesh_candidates = std::move(evaluated_candidates);
        measurement.selected_candidate_idx = static_cast<int>(selected_idx);

        if (selected.solved) {
            measurement.transform_cam  = selected.pose.camera;
            measurement.transform_odom = selected.pose.odom;
        }
        if (selected.solved && selected.depth_valid) {
            measurement.depth_quality = (selected.pair_indices.size() >= 3)
                                          ? LdmDepthQuality::Stable
                                          : LdmDepthQuality::Constrained;
        } else if (selected.solved) {
            measurement.depth_quality = LdmDepthQuality::None;
        } else {
            measurement.depth_quality = LdmDepthQuality::BearingOnly;
        }

        measurement.confidence = selected.solved
                                   ? confidence_for(selected)
                                   : std::clamp(0.15f * selected.preliminary_score, 0.0f, 0.25f);
        return measurement;
    }

private:
    struct BoundedPoseSeed {
        double yaw{0.0};
        double pitch{0.0};
        double roll{0.0};
        Eigen::Vector3d translation{0.0, 0.0, 1.0};
        double normalized_rmse{std::numeric_limits<double>::infinity()};
    };

    struct CandidateCorrespondences {
        std::vector<cv::Point3f> obj_points_cv{};
        std::vector<cv::Point2f> img_points_cv{};
        std::vector<Eigen::Vector3d> obj_points{};
        std::vector<Eigen::Vector2d> normalized_points{};
    };

    struct ConstrainedPoseSolution {
        Eigen::Matrix3d rotation_cam{Eigen::Matrix3d::Identity()};
        Eigen::Vector3d center_cam{0.0, 0.0, 1.0};
        cv::Mat rvec{};
        cv::Mat tvec{};
        float reprojection_rmse_px{std::numeric_limits<float>::quiet_NaN()};
        float vertical_axis_term{0.0f};
        float face_visibility_term{0.0f};
        cv::Point2f projected_center_image{};
        std::vector<cv::Point2f> projected_outline_image{};
    };

    struct BoundedPoseReprojectionError {
        BoundedPoseReprojectionError(
            const Eigen::Vector3d& model_point, const Eigen::Vector2d& normalized_point,
            double residual_scale_x, double residual_scale_y)
            : model_x(model_point.x())
            , model_y(model_point.y())
            , model_z(model_point.z())
            , observed_x(normalized_point.x())
            , observed_y(normalized_point.y())
            , residual_scale_x(residual_scale_x)
            , residual_scale_y(residual_scale_y) {}

        /// @param angles           [yaw, pitch, roll] – Euler angles (Y-X-Z intrinsic)
        /// @param translation_ypd  [t_yaw, t_pitch, t_distance] – spherical translation
        /// @param residuals        [u_px, v_px] focal-weighted reprojection error
        template <typename T>
        bool operator()(const T* const angles, const T* const translation_ypd, T* residuals) const {
            const T yaw   = angles[0];
            const T pitch = angles[1];
            const T roll  = angles[2];
            const T cy    = ceres::cos(yaw);
            const T sy    = ceres::sin(yaw);
            const T cp    = ceres::cos(pitch);
            const T sp    = ceres::sin(pitch);
            const T cr    = ceres::cos(roll);
            const T sr    = ceres::sin(roll);

            // Rotate model point: R_yaw → R_pitch → R_roll
            const T yaw_x = cy * T(model_x) + sy * T(model_z);
            const T yaw_y = T(model_y);
            const T yaw_z = -sy * T(model_x) + cy * T(model_z);

            const T pitch_x = yaw_x;
            const T pitch_y = cp * yaw_y - sp * yaw_z;
            const T pitch_z = sp * yaw_y + cp * yaw_z;

            // Reconstruct Cartesian translation from YPD spherical coordinates.
            // YPD decouples bearing/distance → physically meaningful Ceres bounds.
            const T t_yaw      = translation_ypd[0];
            const T t_pitch    = translation_ypd[1];
            const T t_distance = translation_ypd[2];
            const T ctp        = ceres::cos(t_pitch);
            const T tx         = t_distance * ctp * ceres::sin(t_yaw);
            const T ty         = t_distance * ceres::sin(t_pitch);
            const T tz         = t_distance * ctp * ceres::cos(t_yaw);

            const T x = cr * pitch_x - sr * pitch_y + tx;
            const T y = sr * pitch_x + cr * pitch_y + ty;
            const T z = pitch_z + tz;

            // Reject points behind camera or too close (avoids division by ~0 → NaN)
            constexpr double kMinDepth = 1e-3;
            if (z < T(kMinDepth)) {
                return false;
            }

            residuals[0] = (x / z - T(observed_x)) * T(residual_scale_x);
            residuals[1] = (y / z - T(observed_y)) * T(residual_scale_y);
            return true;
        }

        [[nodiscard]] static ceres::CostFunction* Create(
            const Eigen::Vector3d& model_point, const Eigen::Vector2d& normalized_point,
            double residual_scale_x, double residual_scale_y) {
            return new ceres::AutoDiffCostFunction<BoundedPoseReprojectionError, 2, 3, 3>(
                new BoundedPoseReprojectionError(
                    model_point, normalized_point, residual_scale_x, residual_scale_y));
        }

        double model_x{0.0};
        double model_y{0.0};
        double model_z{0.0};
        double observed_x{0.0};
        double observed_y{0.0};
        double residual_scale_x{1.0};
        double residual_scale_y{1.0};
    };

    [[nodiscard]] bool has_face_assignment(const LdmMeshCandidate& candidate) const noexcept {
        return !candidate.pair_indices.empty()
            && candidate.pair_indices.size() == candidate.octagon_face_indices.size();
    }

    [[nodiscard]] static Eigen::Vector3d face_outward_normal_model(int face_idx) {
        const double angle = static_cast<double>(face_idx) * (std::numbers::pi_v<double> / 4.0);
        return Eigen::Vector3d(std::sin(angle), 0.0, -std::cos(angle));
    }

    [[nodiscard]] Eigen::Vector3d face_center_model(int face_idx) const {
        return octagon_face_center_radius_m(config_.geometry) * face_outward_normal_model(face_idx);
    }

    void assign_visible_face_indices(
        LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs) const {
        if (candidate.pair_indices.empty()) {
            return;
        }

        struct OrderedClusterPair {
            int pair_idx;
            float order_px;
            float separation_px;
        };

        const int cluster_id = [&]() {
            for (const int pair_idx : candidate.pair_indices) {
                if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
                    continue;
                }
                return pairs[static_cast<size_t>(pair_idx)].cluster_id;
            }
            return -1;
        }();

        std::vector<OrderedClusterPair> ordered_cluster;
        ordered_cluster.reserve(pairs.size());
        for (size_t pair_idx = 0; pair_idx < pairs.size(); ++pair_idx) {
            if (cluster_id >= 0 && pairs[pair_idx].cluster_id != cluster_id) {
                continue;
            }
            ordered_cluster.push_back(
                OrderedClusterPair{
                    .pair_idx      = static_cast<int>(pair_idx),
                    .order_px      = pairs[pair_idx].local_order_px,
                    .separation_px = pair_layer_separation_px(pairs[pair_idx]),
                });
        }
        if (ordered_cluster.empty()) {
            for (const int pair_idx : candidate.pair_indices) {
                if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
                    continue;
                }
                const auto& pair = pairs[static_cast<size_t>(pair_idx)];
                ordered_cluster.push_back(
                    OrderedClusterPair{
                        .pair_idx      = pair_idx,
                        .order_px      = pair.local_order_px,
                        .separation_px = pair_layer_separation_px(pair),
                    });
            }
        }

        if (ordered_cluster.empty()) {
            return;
        }
        std::sort(
            ordered_cluster.begin(), ordered_cluster.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.order_px < rhs.order_px; });

        size_t front_rank = 0;
        for (size_t rank = 1; rank < ordered_cluster.size(); ++rank) {
            if (ordered_cluster[rank].separation_px > ordered_cluster[front_rank].separation_px) {
                front_rank = rank;
            }
        }

        struct AssignedPair {
            int pair_idx;
            size_t rank;
            int face;
        };
        std::vector<AssignedPair> assigned;
        assigned.reserve(candidate.pair_indices.size());

        for (size_t candidate_idx = 0; candidate_idx < candidate.pair_indices.size();
             ++candidate_idx) {
            const int pair_idx = candidate.pair_indices[candidate_idx];
            if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
                candidate.octagon_face_indices.clear();
                return;
            }
            const auto rank_it = std::find_if(
                ordered_cluster.begin(), ordered_cluster.end(),
                [&](const OrderedClusterPair& ordered) { return ordered.pair_idx == pair_idx; });
            if (rank_it == ordered_cluster.end()) {
                candidate.octagon_face_indices.clear();
                return;
            }

            const size_t rank =
                static_cast<size_t>(std::distance(ordered_cluster.begin(), rank_it));
            int face = static_cast<int>(rank) - static_cast<int>(front_rank);
            face %= 8;
            if (face < 0) {
                face += 8;
            }
            assigned.push_back(AssignedPair{.pair_idx = pair_idx, .rank = rank, .face = face});
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
    }

    [[nodiscard]] bool
        face_sequence_is_consistent(const std::vector<int>& face_indices) const noexcept {
        if (face_indices.size() <= 1) {
            return true;
        }

        for (size_t i = 1; i < face_indices.size(); ++i) {
            int step = (face_indices[i] - face_indices[i - 1]) % 8;
            if (step < 0) {
                step += 8;
            }
            if (step != 1) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] static bool candidate_selection_better(
        const LdmMeshCandidate& lhs, const LdmMeshCandidate& rhs) noexcept {
        const auto tier = [](const LdmMeshCandidate& candidate) noexcept {
            if (candidate.depth_valid) {
                return 3;
            }
            if (candidate.solved && std::isfinite(candidate.reprojection_rmse_px)) {
                return 2;
            }
            if (std::isfinite(candidate.reprojection_rmse_px)) {
                return 1;
            }
            return 0;
        };

        const int lhs_tier = tier(lhs);
        const int rhs_tier = tier(rhs);
        if (lhs_tier != rhs_tier) {
            return lhs_tier > rhs_tier;
        }
        if (lhs_tier >= 2 && lhs.reprojection_rmse_px != rhs.reprojection_rmse_px) {
            return lhs.reprojection_rmse_px < rhs.reprojection_rmse_px;
        }
        if (lhs.pair_indices.size() != rhs.pair_indices.size()) {
            return lhs.pair_indices.size() > rhs.pair_indices.size();
        }
        return lhs.score > rhs.score;
    }

    [[nodiscard]] bool center_estimates_are_compatible(
        const LdmMeshCandidate& lhs, const LdmMeshCandidate& rhs) const noexcept {
        if (!lhs.solved || !rhs.solved) {
            return true;
        }

        const Eigen::Vector3d lhs_center = lhs.pose.camera.translation();
        const Eigen::Vector3d rhs_center = rhs.pose.camera.translation();
        const double min_depth           = std::max(1.0, std::min(lhs_center.z(), rhs_center.z()));
        const double max_center_delta_m  = std::max(0.08, 0.03 * min_depth);
        return (lhs_center - rhs_center).norm() <= max_center_delta_m;
    }

    [[nodiscard]] bool
        same_assignment(const LdmMeshCandidate& lhs, const LdmMeshCandidate& rhs) const noexcept {
        return lhs.cluster_id == rhs.cluster_id && lhs.pair_indices == rhs.pair_indices
            && lhs.octagon_face_indices == rhs.octagon_face_indices;
    }

    [[nodiscard]] std::optional<LdmMeshCandidate> try_merge_candidates(
        const LdmMeshCandidate& base, const LdmMeshCandidate& extra,
        const std::vector<LightPair>& pairs, const OdomCameraTransform& T_odom_camera,
        const std::optional<PosePrior>& prior = std::nullopt) const {
        if (!has_face_assignment(base) || !has_face_assignment(extra)) {
            return std::nullopt;
        }
        if (base.cluster_id >= 0 && extra.cluster_id >= 0 && base.cluster_id != extra.cluster_id) {
            return std::nullopt;
        }
        if (!center_estimates_are_compatible(base, extra)) {
            return std::nullopt;
        }

        std::vector<int> pair_to_face(pairs.size(), -1);
        std::array<int, 8> face_to_pair{};
        face_to_pair.fill(-1);

        const auto insert_assignment = [&](const LdmMeshCandidate& candidate) -> bool {
            for (size_t i = 0; i < candidate.pair_indices.size(); ++i) {
                const int pair_idx = candidate.pair_indices[i];
                const int face_idx = candidate.octagon_face_indices[i];
                if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
                    return false;
                }
                if (face_idx < 0 || face_idx >= 8) {
                    return false;
                }

                if (pair_to_face[static_cast<size_t>(pair_idx)] != -1) {
                    if (pair_to_face[static_cast<size_t>(pair_idx)] != face_idx) {
                        return false;
                    }
                    continue;
                }
                if (face_to_pair[static_cast<size_t>(face_idx)] != -1) {
                    return false;
                }

                pair_to_face[static_cast<size_t>(pair_idx)] = face_idx;
                face_to_pair[static_cast<size_t>(face_idx)] = pair_idx;
            }
            return true;
        };

        if (!insert_assignment(base) || !insert_assignment(extra)) {
            return std::nullopt;
        }

        LdmMeshCandidate merged;
        merged.cluster_id = base.cluster_id;
        merged.pair_indices.reserve(pairs.size());
        merged.octagon_face_indices.reserve(pairs.size());
        for (size_t pair_idx = 0; pair_idx < pair_to_face.size(); ++pair_idx) {
            if (pair_to_face[pair_idx] < 0) {
                continue;
            }
            merged.pair_indices.push_back(static_cast<int>(pair_idx));
            merged.octagon_face_indices.push_back(pair_to_face[pair_idx]);
        }
        if (merged.pair_indices.size()
            <= std::max(base.pair_indices.size(), extra.pair_indices.size())) {
            return std::nullopt;
        }
        if (!face_sequence_is_consistent(merged.octagon_face_indices)) {
            return std::nullopt;
        }

        const float lhs_weight = static_cast<float>(base.pair_indices.size());
        const float rhs_weight = static_cast<float>(extra.pair_indices.size());
        merged.preliminary_score =
            ((lhs_weight * base.preliminary_score) + (rhs_weight * extra.preliminary_score))
            / std::max(1.0f, lhs_weight + rhs_weight);
        evaluate_candidate(merged, pairs, T_odom_camera, prior);
        if (!merged.solved) {
            return std::nullopt;
        }
        return merged;
    }

    [[nodiscard]] std::vector<LdmMeshCandidate> build_combined_candidates(
        const std::vector<LdmMeshCandidate>& evaluated_candidates,
        const std::vector<LightPair>& pairs, const OdomCameraTransform& T_odom_camera,
        const std::optional<PosePrior>& prior = std::nullopt) const {
        std::vector<size_t> source_indices;
        source_indices.reserve(evaluated_candidates.size());
        for (size_t i = 0; i < evaluated_candidates.size(); ++i) {
            const auto& candidate = evaluated_candidates[i];
            if (!candidate.solved || candidate.pair_indices.size() < 2
                || !has_face_assignment(candidate)) {
                continue;
            }
            source_indices.push_back(i);
        }
        if (source_indices.size() < 2) {
            return {};
        }

        std::sort(source_indices.begin(), source_indices.end(), [&](size_t lhs, size_t rhs) {
            const auto& a = evaluated_candidates[lhs];
            const auto& b = evaluated_candidates[rhs];
            if (a.pair_indices.size() != b.pair_indices.size()) {
                return a.pair_indices.size() > b.pair_indices.size();
            }
            return a.score > b.score;
        });
        if (source_indices.size() > 32) {
            source_indices.resize(32);
        }

        std::vector<LdmMeshCandidate> combined_candidates;
        combined_candidates.reserve(source_indices.size());
        for (const size_t seed_idx : source_indices) {
            LdmMeshCandidate current = evaluated_candidates[seed_idx];
            bool grew                = false;

            while (true) {
                std::optional<LdmMeshCandidate> best_merge;
                for (const size_t other_idx : source_indices) {
                    if (other_idx == seed_idx) {
                        continue;
                    }

                    auto merged = try_merge_candidates(
                        current, evaluated_candidates[other_idx], pairs, T_odom_camera, prior);
                    if (!merged.has_value()) {
                        continue;
                    }
                    if (!best_merge.has_value()
                        || merged->pair_indices.size() > best_merge->pair_indices.size()
                        || (merged->pair_indices.size() == best_merge->pair_indices.size()
                            && merged->score > best_merge->score)) {
                        best_merge = std::move(merged);
                    }
                }

                if (!best_merge.has_value()) {
                    break;
                }
                current = std::move(*best_merge);
                grew    = true;
            }

            if (!grew) {
                continue;
            }

            bool replaced_existing = false;
            for (auto& existing : combined_candidates) {
                if (!same_assignment(existing, current)) {
                    continue;
                }
                if (current.score > existing.score) {
                    existing = current;
                }
                replaced_existing = true;
                break;
            }
            if (!replaced_existing) {
                combined_candidates.push_back(std::move(current));
            }
        }

        std::sort(
            combined_candidates.begin(), combined_candidates.end(),
            [](const LdmMeshCandidate& a, const LdmMeshCandidate& b) {
                if (a.pair_indices.size() != b.pair_indices.size()) {
                    return a.pair_indices.size() > b.pair_indices.size();
                }
                return a.score > b.score;
            });
        return combined_candidates;
    }

    [[nodiscard]] cv::Point2f estimate_candidate_center_image(
        const std::vector<LightPair>& pairs, const std::vector<int>& pair_indices) const {
        cv::Point2f center(0.0f, 0.0f);
        if (pair_indices.empty()) {
            return center;
        }
        for (const int pair_idx : pair_indices) {
            center += pairs[static_cast<size_t>(pair_idx)].midpoint_px;
        }
        center *= (1.0f / static_cast<float>(pair_indices.size()));
        return center;
    }

    [[nodiscard]] std::expected<Eigen::Vector2d, std::string>
        normalized_image_point(const cv::Point2f& point_px) const {
        std::vector<cv::Point2f> undistorted;
        cv::undistortPoints(
            std::vector<cv::Point2f>{point_px}, undistorted, camera_matrix_, dist_coeffs_);
        if (undistorted.size() != 1u) {
            return std::unexpected(
                std::string("normalized_image_point: cv::undistortPoints returned no result"));
        }
        return Eigen::Vector2d(
            static_cast<double>(undistorted.front().x), static_cast<double>(undistorted.front().y));
    }

    [[nodiscard]] Eigen::Vector3d bearing_for_image_point(const cv::Point2f& point_px) const {
        const auto normalized_point = normalized_image_point(point_px);
        if (!normalized_point.has_value()) {
            return Eigen::Vector3d(0.0, 0.0, 1.0);
        }

        Eigen::Vector3d bearing(normalized_point->x(), normalized_point->y(), 1.0);
        if (bearing.norm() <= 1e-9) {
            return Eigen::Vector3d(0.0, 0.0, 1.0);
        }
        return bearing.normalized();
    }

    [[nodiscard]] static Eigen::Matrix3d yaw_rotation_matrix(double yaw) {
        return bounded_pose_rotation(yaw, 0.0, 0.0);
    }

    [[nodiscard]] static Eigen::Matrix3d
        bounded_pose_rotation(double yaw, double pitch, double roll) {
        return Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()).toRotationMatrix()
             * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()).toRotationMatrix()
             * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()).toRotationMatrix();
    }

    [[nodiscard]] static double pose_angle_limit(double raw_limit) {
        if (!std::isfinite(raw_limit) || raw_limit <= 0.0) {
            return 0.872664626;
        }
        return std::min(raw_limit, std::numbers::pi_v<double> * 0.5 - 1e-3);
    }

    [[nodiscard]] static bool angle_within_limit(double angle, double max_angle) {
        return std::isfinite(angle) && std::abs(angle) <= max_angle + 1e-9;
    }

    [[nodiscard]] static std::expected<std::array<double, 3>, std::string>
        decompose_rotation(const Eigen::Matrix3d& rotation) noexcept {
        const double pitch = std::asin(std::clamp(rotation(2, 1), -1.0, 1.0));
        if (std::abs(std::cos(pitch)) <= 1e-6) {
            return std::unexpected(
                std::string("decompose_rotation: cos(pitch) too close to zero (gimbal lock)"));
        }

        const double yaw  = std::atan2(-rotation(2, 0), rotation(2, 2));
        const double roll = std::atan2(-rotation(0, 1), rotation(1, 1));
        return std::array<double, 3>{yaw, pitch, roll};
    }

    [[nodiscard]] std::expected<CandidateCorrespondences, std::string>
        build_candidate_correspondences(
            const LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs) const {
        if (!has_face_assignment(candidate)) {
            return std::unexpected(
                std::string("build_candidate_correspondences: candidate has no face assignment"));
        }

        CandidateCorrespondences correspondences;
        correspondences.obj_points_cv.reserve(candidate.pair_indices.size() * 2u);
        correspondences.img_points_cv.reserve(candidate.pair_indices.size() * 2u);
        correspondences.obj_points.reserve(candidate.pair_indices.size() * 2u);
        correspondences.normalized_points.reserve(candidate.pair_indices.size() * 2u);

        for (size_t i = 0; i < candidate.pair_indices.size(); ++i) {
            const int pair_idx = candidate.pair_indices[i];
            const int face_idx = candidate.octagon_face_indices[i];
            if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
                return std::unexpected(
                    std::string("build_candidate_correspondences: pair_idx out of range"));
            }
            if (face_idx < 0 || face_idx >= 8) {
                return std::unexpected(
                    std::string("build_candidate_correspondences: face_idx out of range"));
            }

            const auto model_pair_points = pair_model_points_for_face(config_.geometry, face_idx);
            const std::array<cv::Point2f, 2> image_points{
                pairs[static_cast<size_t>(pair_idx)].top_center_px,
                pairs[static_cast<size_t>(pair_idx)].bottom_center_px,
            };

            for (size_t point_idx = 0; point_idx < 2u; ++point_idx) {
                const auto normalized_point = normalized_image_point(image_points[point_idx]);
                if (!normalized_point.has_value()) {
                    return std::unexpected(
                        std::string(
                            "build_candidate_correspondences: normalized_image_point failed"));
                }

                const auto& model_point = model_pair_points[point_idx];
                correspondences.obj_points_cv.push_back(model_point);
                correspondences.img_points_cv.push_back(image_points[point_idx]);
                correspondences.obj_points.emplace_back(
                    static_cast<double>(model_point.x), static_cast<double>(model_point.y),
                    static_cast<double>(model_point.z));
                correspondences.normalized_points.push_back(*normalized_point);
            }
        }

        if (correspondences.obj_points.size() < 2u) {
            return std::unexpected(
                std::string("build_candidate_correspondences: fewer than 2 correspondences built"));
        }
        return correspondences;
    }

    [[nodiscard]] static std::expected<Eigen::Vector3d, std::string> solve_translation_for_rotation(
        const CandidateCorrespondences& correspondences, const Eigen::Matrix3d& rotation) {
        Eigen::MatrixXd lhs(static_cast<Eigen::Index>(correspondences.obj_points.size() * 2u), 3);
        Eigen::VectorXd rhs(static_cast<Eigen::Index>(correspondences.obj_points.size() * 2u));

        Eigen::Index row = 0;
        for (size_t i = 0; i < correspondences.obj_points.size(); ++i) {
            const Eigen::Vector3d rotated = rotation * correspondences.obj_points[i];
            const double u                = correspondences.normalized_points[i].x();
            const double v                = correspondences.normalized_points[i].y();

            lhs(row, 0) = 1.0;
            lhs(row, 1) = 0.0;
            lhs(row, 2) = -u;
            rhs(row)    = u * rotated.z() - rotated.x();
            ++row;

            lhs(row, 0) = 0.0;
            lhs(row, 1) = 1.0;
            lhs(row, 2) = -v;
            rhs(row)    = v * rotated.z() - rotated.y();
            ++row;
        }

        const Eigen::Vector3d translation = lhs.colPivHouseholderQr().solve(rhs);
        if (!std::isfinite(translation.x()) || !std::isfinite(translation.y())
            || !std::isfinite(translation.z()) || translation.z() <= 1e-3) {
            return std::unexpected(
                std::string(
                    "solve_translation_for_rotation: invalid/non-finite translation or depth <= "
                    "1e-3"));
        }
        return translation;
    }

    [[nodiscard]] static double normalized_reprojection_rmse(
        const CandidateCorrespondences& correspondences, const Eigen::Matrix3d& rotation,
        const Eigen::Vector3d& translation) {
        double sq_error_sum = 0.0;
        size_t count        = 0;

        for (size_t i = 0; i < correspondences.obj_points.size(); ++i) {
            const Eigen::Vector3d cam_point =
                rotation * correspondences.obj_points[i] + translation;
            if (!std::isfinite(cam_point.x()) || !std::isfinite(cam_point.y())
                || !std::isfinite(cam_point.z()) || cam_point.z() <= 1e-6) {
                return std::numeric_limits<double>::infinity();
            }
            const Eigen::Vector2d projected(
                cam_point.x() / cam_point.z(), cam_point.y() / cam_point.z());
            const Eigen::Vector2d delta = projected - correspondences.normalized_points[i];
            sq_error_sum += delta.squaredNorm();
            ++count;
        }

        if (count == 0u) {
            return std::numeric_limits<double>::infinity();
        }
        return std::sqrt(sq_error_sum / static_cast<double>(count));
    }

    [[nodiscard]] static double normalized_reprojection_rmse(
        const CandidateCorrespondences& correspondences, const BoundedPoseSeed& seed) {
        return normalized_reprojection_rmse(
            correspondences, bounded_pose_rotation(seed.yaw, seed.pitch, seed.roll),
            seed.translation);
    }

    [[nodiscard]] std::vector<BoundedPoseSeed>
        make_pnp_pose_seeds(const CandidateCorrespondences& correspondences) const {
        if (correspondences.obj_points_cv.size() < 4u) {
            return {};
        }

        std::vector<BoundedPoseSeed> seeds;

        // ITERATIVE with identity initial guess has no automatic fallback in
        // OpenCV. If ITERATIVE doesn't converge (the initial translation at
        // origin is 3-5m from the true depth), fall back to a bare EPNP
        // solve.  EPNP is closed-form, needs no initial guess, and produces
        // a seed that Ceres can refine.
        {
            constexpr int flag = cv::SOLVEPNP_ITERATIVE;
            cv::Mat rvec;
            cv::Mat tvec;
            bool solved = false;
            try {
                solved = cv::solvePnP(
                    correspondences.obj_points_cv, correspondences.img_points_cv, camera_matrix_,
                    dist_coeffs_, rvec, tvec, false, flag);
            } catch (const cv::Exception&) {
                // ITERATIVE threw; try EPNP as fallback.
                try {
                    solved = cv::solvePnP(
                        correspondences.obj_points_cv, correspondences.img_points_cv,
                        camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_EPNP);
                } catch (const cv::Exception&) {
                    return seeds;
                }
            }
            if (!solved) {
                // ITERATIVE didn't converge from identity; try EPNP.
                try {
                    solved = cv::solvePnP(
                        correspondences.obj_points_cv, correspondences.img_points_cv,
                        camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_EPNP);
                } catch (const cv::Exception&) {
                    return seeds;
                }
                if (!solved) {
                    return seeds;
                }
            }

            cv::Mat rotation_cv;
            try {
                cv::Rodrigues(rvec, rotation_cv);
            } catch (const cv::Exception&) {
                return seeds;
            }

            cv::Mat rotation_cv64;
            cv::Mat tvec64;
            rotation_cv.convertTo(rotation_cv64, CV_64F);
            tvec.convertTo(tvec64, CV_64F);
            if (rotation_cv64.rows != 3 || rotation_cv64.cols != 3 || tvec64.total() != 3u) {
                return seeds;
            }

            Eigen::Matrix3d rotation_cam;
            cv::cv2eigen(rotation_cv64, rotation_cam);
            const auto angles = decompose_rotation(rotation_cam);
            if (!angles.has_value()) {
                return seeds;
            }

            const cv::Mat tvec_col = tvec64.reshape(1, 3);
            BoundedPoseSeed seed;
            seed.yaw         = (*angles)[0];
            seed.pitch       = (*angles)[1];
            seed.roll        = (*angles)[2];
            seed.translation = Eigen::Vector3d(
                tvec_col.at<double>(0, 0), tvec_col.at<double>(1, 0), tvec_col.at<double>(2, 0));
            if (!std::isfinite(seed.translation.x()) || !std::isfinite(seed.translation.y())
                || !std::isfinite(seed.translation.z()) || seed.translation.z() <= 1e-3) {
                return seeds;
            }

            seed.normalized_rmse = normalized_reprojection_rmse(correspondences, seed);
            if (std::isfinite(seed.normalized_rmse)) {
                seeds.push_back(seed);
            }
        }
        return seeds;
    }

    [[nodiscard]] std::vector<BoundedPoseSeed>
        make_pose_seeds(const CandidateCorrespondences& correspondences) const {
        std::vector<BoundedPoseSeed> seeds = make_pnp_pose_seeds(correspondences);
        constexpr int kYawSeedCount        = 33;
        constexpr std::array<double, 5> kPitchRollRatios{-1.0, -0.5, 0.0, 0.5, 1.0};
        const double max_angle = pose_angle_limit(config_.max_pose_angle_rad);
        seeds.reserve(
            seeds.size() + kYawSeedCount * kPitchRollRatios.size() * kPitchRollRatios.size());

        for (int yaw_idx = 0; yaw_idx < kYawSeedCount; ++yaw_idx) {
            const double yaw = -std::numbers::pi_v<double>
                             + (2.0 * std::numbers::pi_v<double> * static_cast<double>(yaw_idx)
                                / static_cast<double>(kYawSeedCount - 1));
            for (const double pitch_ratio : kPitchRollRatios) {
                const double pitch = pitch_ratio * max_angle;
                for (const double roll_ratio : kPitchRollRatios) {
                    const double roll              = roll_ratio * max_angle;
                    const Eigen::Matrix3d rotation = bounded_pose_rotation(yaw, pitch, roll);
                    std::expected<Eigen::Vector3d, std::string> translation =
                        solve_translation_for_rotation(correspondences, rotation);
                    if (!translation.has_value()) {
                        continue;
                    }

                    BoundedPoseSeed seed;
                    seed.yaw             = yaw;
                    seed.pitch           = pitch;
                    seed.roll            = roll;
                    seed.translation     = *translation;
                    seed.normalized_rmse = normalized_reprojection_rmse(correspondences, seed);
                    if (std::isfinite(seed.normalized_rmse)) {
                        seeds.push_back(seed);
                    }
                }
            }
        }

        std::sort(seeds.begin(), seeds.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.normalized_rmse < rhs.normalized_rmse;
        });
        if (seeds.size() > 8u) {
            seeds.resize(8u);
        }
        return seeds;
    }

    [[nodiscard]] std::expected<BoundedPoseSeed, std::string> refine_pose_seed(
        const CandidateCorrespondences& correspondences, const BoundedPoseSeed& seed,
        bool lock_yaw = false, const std::optional<PosePrior>& prior = std::nullopt) const {
        const double max_angle = pose_angle_limit(config_.max_pose_angle_rad);
        double angles[3]       = {
            std::remainder(seed.yaw, 2.0 * std::numbers::pi_v<double>),
            std::clamp(seed.pitch, -max_angle, max_angle),
            std::clamp(seed.roll, -max_angle, max_angle),
        };

        // Convert seed Cartesian translation to YPD spherical coordinates for
        // Ceres.  YPD decouples bearing from depth → physically meaningful
        // bounds that prevent the solver from sliding along degenerate valleys
        // in the Cartesian Jacobian.
        const Eigen::Vector3d seed_ypd = camera_translation_to_ypd(seed.translation);
        double translation_ypd[3]      = {seed_ypd.x(), seed_ypd.y(), seed_ypd.z()};

        ceres::Problem problem;
        for (size_t i = 0; i < correspondences.obj_points.size(); ++i) {
            problem.AddResidualBlock(
                BoundedPoseReprojectionError::Create(
                    correspondences.obj_points[i], correspondences.normalized_points[i],
                    residual_scale_x_, residual_scale_y_),
                new ceres::HuberLoss(huber_delta_px_), angles, translation_ypd);
        }
        // Decompose prior rvec into yaw/pitch/roll for angle bounds.
        // Uses the same Euler convention as bounded_pose_rotation:
        //   R = R(roll, Z) * R(pitch, X) * R(yaw, Y)
        std::optional<std::array<double, 3>> prior_angles;
        if (prior.has_value()) {
            cv::Mat R_cv;
            cv::Rodrigues(cv::Mat(prior->rvec), R_cv);
            Eigen::Matrix3d R_prior;
            cv::cv2eigen(R_cv, R_prior);
            auto decomposed = decompose_rotation(R_prior);
            if (decomposed.has_value()) {
                prior_angles = *decomposed;
            }
        }

        if (lock_yaw) {
            // Yaw is geometrically determined by face assignment — narrow band
            // for noise smoothing without allowing visibility-breaking drift.
            constexpr double kYawBand = 0.1;
            problem.SetParameterLowerBound(angles, 0, seed.yaw - kYawBand);
            problem.SetParameterUpperBound(angles, 0, seed.yaw + kYawBand);
        } else if (prior_angles.has_value()) {
            // Prior yaw with a band — constrains the full yaw search to a
            // physically plausible neighbourhood around the tracker prediction.
            constexpr double kPriorYawBand = 2.8;
            const double prior_yaw =
                std::remainder((*prior_angles)[0], 2.0 * std::numbers::pi_v<double>);
            problem.SetParameterLowerBound(angles, 0, prior_yaw - kPriorYawBand);
            problem.SetParameterUpperBound(angles, 0, prior_yaw + kPriorYawBand);
        } else {
            problem.SetParameterLowerBound(angles, 0, -std::numbers::pi_v<double>);
            problem.SetParameterUpperBound(angles, 0, std::numbers::pi_v<double>);
        }

        if (prior_angles.has_value()) {
            constexpr double kPriorAngleBand = 0.5;
            problem.SetParameterLowerBound(
                angles, 1, std::max(-max_angle, (*prior_angles)[1] - kPriorAngleBand));
            problem.SetParameterUpperBound(
                angles, 1, std::min(max_angle, (*prior_angles)[1] + kPriorAngleBand));
            problem.SetParameterLowerBound(
                angles, 2, std::max(-max_angle, (*prior_angles)[2] - kPriorAngleBand));
            problem.SetParameterUpperBound(
                angles, 2, std::min(max_angle, (*prior_angles)[2] + kPriorAngleBand));
        } else {
            problem.SetParameterLowerBound(angles, 1, -max_angle);
            problem.SetParameterUpperBound(angles, 1, max_angle);
            problem.SetParameterLowerBound(angles, 2, -max_angle);
            problem.SetParameterUpperBound(angles, 2, max_angle);
        }

        // YPD bearing bounds: constrain to front camera hemisphere.
        constexpr double kBearingLimit = std::numbers::pi_v<double> / 2.0 - 1e-6;
        problem.SetParameterLowerBound(translation_ypd, 0, -kBearingLimit);
        problem.SetParameterUpperBound(translation_ypd, 0, kBearingLimit);
        problem.SetParameterLowerBound(translation_ypd, 1, -kBearingLimit);
        problem.SetParameterUpperBound(translation_ypd, 1, kBearingLimit);

        // Distance bound: use prior to set a physically plausible band, otherwise
        // only enforce the minimum-depth (1mm) lower bound.
        if (prior.has_value()) {
            const double prior_distance = cv::norm(prior->tvec);
            if (std::isfinite(prior_distance) && prior_distance > 1e-3) {
                constexpr double kDistanceBand = 1.0;
                problem.SetParameterLowerBound(
                    translation_ypd, 2, std::max(1e-3, prior_distance - kDistanceBand));
                problem.SetParameterUpperBound(translation_ypd, 2, prior_distance + kDistanceBand);
            } else {
                problem.SetParameterLowerBound(translation_ypd, 2, 1e-3);
            }
        } else {
            problem.SetParameterLowerBound(translation_ypd, 2, 1e-3);
        }

        ceres::Solver::Options options;
        options.linear_solver_type           = ceres::DENSE_QR;
        options.max_num_iterations           = 50;
        options.minimizer_progress_to_stdout = false;
        options.function_tolerance           = 0.0;
        options.gradient_tolerance           = 0.0;
        options.parameter_tolerance          = 0.0;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        BoundedPoseSeed refined;
        refined.yaw         = std::remainder(angles[0], 2.0 * std::numbers::pi_v<double>);
        refined.pitch       = angles[1];
        refined.roll        = angles[2];
        refined.translation = camera_ypd_to_translation(
            Eigen::Vector3d{translation_ypd[0], translation_ypd[1], translation_ypd[2]});
        if (!summary.IsSolutionUsable() || !angle_within_limit(refined.pitch, max_angle)
            || !angle_within_limit(refined.roll, max_angle)
            || !std::isfinite(refined.translation.x()) || !std::isfinite(refined.translation.y())
            || !std::isfinite(refined.translation.z()) || refined.translation.z() <= 1e-3) {
            return std::unexpected(
                std::string("refine_pose_seed: solution not usable or angles/translation invalid"));
        }

        refined.normalized_rmse = normalized_reprojection_rmse(correspondences, refined);
        if (!std::isfinite(refined.normalized_rmse)) {
            return std::unexpected(std::string("refine_pose_seed: normalized RMSE is not finite"));
        }
        return refined;
    }

    [[nodiscard]] std::expected<BoundedPoseSeed, std::string> solve_constrained_pose_seed(
        const LdmMeshCandidate& candidate, const CandidateCorrespondences& correspondences,
        const std::optional<PosePrior>& prior = std::nullopt) const {
        const auto seeds = make_pose_seeds(correspondences);
        if (seeds.empty()) {
            return std::unexpected(
                std::string("solve_constrained_pose_seed: no pose seeds generated"));
        }

        std::optional<BoundedPoseSeed> best_seed;
        for (const auto& seed : seeds) {
            const auto refined = refine_pose_seed(correspondences, seed, false, prior);
            if (!refined.has_value()) {
                continue;
            }
            if (!visible_face_score(
                     candidate, bounded_pose_rotation(refined->yaw, refined->pitch, refined->roll),
                     refined->translation)
                     .has_value()) {
                continue;
            }
            if (!best_seed.has_value() || refined->normalized_rmse < best_seed->normalized_rmse) {
                best_seed = *refined;
            }
        }
        if (!best_seed.has_value()) {
            return std::unexpected(
                std::string(
                    "solve_constrained_pose_seed: no valid seed after refinement and visibility "
                    "check"));
        }
        return *best_seed;
    }

    [[nodiscard]] std::expected<BoundedPoseSeed, std::string> solve_visible_pose_seed(
        const LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs,
        const CandidateCorrespondences& correspondences,
        const std::optional<PosePrior>& prior = std::nullopt) const {
        if (!has_face_assignment(candidate)) {
            return std::unexpected(
                std::string("solve_visible_pose_seed: candidate has no face assignment"));
        }

        size_t front_candidate_idx = 0;
        float max_separation_px    = -1.0f;
        for (size_t i = 0; i < candidate.pair_indices.size(); ++i) {
            const int pair_idx = candidate.pair_indices[i];
            if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
                return std::unexpected(
                    std::string("solve_visible_pose_seed: pair_idx out of range in loop"));
            }
            const float separation_px =
                pair_layer_separation_px(pairs[static_cast<size_t>(pair_idx)]);
            if (separation_px > max_separation_px) {
                max_separation_px   = separation_px;
                front_candidate_idx = i;
            }
        }

        const int front_face = candidate.octagon_face_indices[front_candidate_idx];
        if (front_face < 0 || front_face >= 8) {
            return std::unexpected(
                std::string("solve_visible_pose_seed: front face index out of range"));
        }

        const Eigen::Vector3d bearing =
            bearing_for_image_point(candidate.estimated_center_image_px);
        const double bearing_xz_norm = std::hypot(bearing.x(), bearing.z());
        if (!std::isfinite(bearing_xz_norm) || bearing_xz_norm <= 1e-9) {
            return std::unexpected(
                std::string("solve_visible_pose_seed: bearing norm is not finite or too small"));
        }

        const double front_face_angle =
            static_cast<double>(front_face) * (std::numbers::pi_v<double> / 4.0);
        const double bearing_yaw = std::atan2(bearing.x(), bearing.z());

        BoundedPoseSeed seed;
        seed.yaw = std::remainder(front_face_angle + bearing_yaw, 2.0 * std::numbers::pi_v<double>);
        seed.pitch = 0.0;
        seed.roll  = 0.0;

        const auto translation = solve_translation_for_rotation(
            correspondences, bounded_pose_rotation(seed.yaw, 0.0, 0.0));
        if (!translation.has_value()) {
            return std::unexpected(
                std::string("solve_visible_pose_seed: solve_translation_for_rotation failed"));
        }
        seed.translation     = *translation;
        seed.normalized_rmse = normalized_reprojection_rmse(correspondences, seed);
        if (!std::isfinite(seed.normalized_rmse)) {
            return std::unexpected(
                std::string("solve_visible_pose_seed: seed normalized RMSE is not finite"));
        }

        // Yaw is locked — only pitch/roll/translation are refined.  The yaw
        // from face assignment (adjacent faces at 45° is ground truth) must
        // not drift, otherwise observed faces become invisible.
        auto refined = refine_pose_seed(correspondences, seed, true, prior);
        if (!refined.has_value()) {
            return std::unexpected(std::string("solve_visible_pose_seed: refine_pose_seed failed"));
        }
        return std::move(*refined);
    }

    [[nodiscard]] float pixel_reprojection_rmse(
        const CandidateCorrespondences& correspondences, const cv::Mat& rvec,
        const cv::Mat& tvec) const {
        std::vector<cv::Point2f> reproj_points;
        cv::projectPoints(
            correspondences.obj_points_cv, rvec, tvec, camera_matrix_, dist_coeffs_, reproj_points);
        if (reproj_points.size() != correspondences.img_points_cv.size()) {
            return std::numeric_limits<float>::infinity();
        }

        double sq_error_sum = 0.0;
        for (size_t i = 0; i < reproj_points.size(); ++i) {
            const cv::Point2f delta = reproj_points[i] - correspondences.img_points_cv[i];
            sq_error_sum += static_cast<double>(delta.x) * static_cast<double>(delta.x)
                          + static_cast<double>(delta.y) * static_cast<double>(delta.y);
        }
        return static_cast<float>(std::sqrt(
            sq_error_sum / static_cast<double>(std::max<size_t>(1, reproj_points.size()))));
    }

    [[nodiscard]] std::expected<ConstrainedPoseSolution, std::string> build_pose_solution(
        const LdmMeshCandidate& candidate, const CandidateCorrespondences& correspondences,
        const Eigen::Matrix3d& rotation_cam, const Eigen::Vector3d& center_cam) const {
        if (!std::isfinite(center_cam.x()) || !std::isfinite(center_cam.y())
            || !std::isfinite(center_cam.z()) || center_cam.z() <= 1e-3) {
            return std::unexpected(
                std::string("build_pose_solution: center_cam is not finite or depth <= 1e-3"));
        }

        ConstrainedPoseSolution solution;
        solution.rotation_cam = rotation_cam;
        solution.center_cam   = center_cam;

        cv::Mat rotation_cv;
        cv::eigen2cv(solution.rotation_cam, rotation_cv);
        cv::Rodrigues(rotation_cv, solution.rvec);
        solution.tvec =
            (cv::Mat_<double>(3, 1) << solution.center_cam.x(), solution.center_cam.y(),
             solution.center_cam.z());

        solution.reprojection_rmse_px =
            pixel_reprojection_rmse(correspondences, solution.rvec, solution.tvec);
        if (!std::isfinite(solution.reprojection_rmse_px)) {
            return std::unexpected(
                std::string("build_pose_solution: reprojection RMSE is not finite"));
        }

        std::vector<cv::Point2f> projected_center;
        cv::projectPoints(
            std::vector<cv::Point3f>{cv::Point3f(0.0f, 0.0f, 0.0f)}, solution.rvec, solution.tvec,
            camera_matrix_, dist_coeffs_, projected_center);
        if (!projected_center.empty()) {
            solution.projected_center_image = projected_center.front();
        }

        const auto outline_model_points = volume_outline_points(config_.geometry);
        cv::projectPoints(
            std::vector<cv::Point3f>(outline_model_points.begin(), outline_model_points.end()),
            solution.rvec, solution.tvec, camera_matrix_, dist_coeffs_,
            solution.projected_outline_image);

        const auto face_visibility_score = visible_face_score(candidate, rotation_cam, center_cam);
        if (!face_visibility_score.has_value()) {
            return std::unexpected(
                std::string("build_pose_solution: visible_face_score returned no value"));
        }
        solution.face_visibility_term = *face_visibility_score;

        solution.vertical_axis_term = projected_vertical_axis_score(solution.rvec, solution.tvec);
        return solution;
    }

    [[nodiscard]] std::expected<ConstrainedPoseSolution, std::string>
        solve_single_face_candidate_pose(
            const LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs) const {
        if (!has_face_assignment(candidate) || candidate.pair_indices.size() != 1u) {
            return std::unexpected(
                std::string(
                    "solve_single_face_candidate_pose: candidate has no face assignment or has "
                    "!= 1 pair"));
        }

        const auto correspondences = build_candidate_correspondences(candidate, pairs);
        if (!correspondences.has_value() || correspondences->normalized_points.size() != 2u) {
            return std::unexpected(
                std::string(
                    "solve_single_face_candidate_pose: correspondences invalid or != 2 normalized "
                    "points"));
        }

        const int face_idx = candidate.octagon_face_indices.front();
        if (face_idx < 0 || face_idx >= 8) {
            return std::unexpected(
                std::string("solve_single_face_candidate_pose: face index out of range"));
        }

        const Eigen::Vector2d top_norm    = correspondences->normalized_points[0];
        const Eigen::Vector2d bottom_norm = correspondences->normalized_points[1];
        const double normalized_height    = bottom_norm.y() - top_norm.y();
        const double model_height         = config_.geometry.pair_center_separation_m;
        if (!std::isfinite(normalized_height) || normalized_height <= 1e-6
            || model_height <= 1e-6) {
            return std::unexpected(
                std::string(
                    "solve_single_face_candidate_pose: normalized_height or model_height "
                    "invalid"));
        }

        const double face_center_z = model_height / normalized_height;
        const double face_center_x = 0.5 * (top_norm.x() + bottom_norm.x()) * face_center_z;
        const double face_center_y = 0.5 * (top_norm.y() + bottom_norm.y()) * face_center_z;
        const Eigen::Vector3d face_center_cam(face_center_x, face_center_y, face_center_z);

        const int pair_idx = candidate.pair_indices.front();
        if (pair_idx < 0 || static_cast<size_t>(pair_idx) >= pairs.size()) {
            return std::unexpected(
                std::string("solve_single_face_candidate_pose: pair_idx out of range"));
        }
        const Eigen::Vector3d bearing =
            bearing_for_image_point(pairs[static_cast<size_t>(pair_idx)].midpoint_px);
        const double yaw = std::remainder(
            static_cast<double>(face_idx) * (std::numbers::pi_v<double> / 4.0)
                + std::atan2(bearing.x(), bearing.z()),
            2.0 * std::numbers::pi_v<double>);
        const Eigen::Matrix3d rotation_cam = yaw_rotation_matrix(yaw);

        const auto model_pair = pair_model_points_for_face(config_.geometry, face_idx);
        const Eigen::Vector3d face_center_model(
            0.5 * static_cast<double>(model_pair[0].x + model_pair[1].x),
            0.5 * static_cast<double>(model_pair[0].y + model_pair[1].y),
            0.5 * static_cast<double>(model_pair[0].z + model_pair[1].z));
        const Eigen::Vector3d center_cam = face_center_cam - rotation_cam * face_center_model;

        return build_pose_solution(candidate, *correspondences, rotation_cam, center_cam);
    }

    [[nodiscard]] std::expected<ConstrainedPoseSolution, std::string>
        solve_constrained_candidate_pose(
            const LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs,
            const std::optional<PosePrior>& prior = std::nullopt) const {
        if (candidate.pair_indices.size() == 1u) {
            return solve_single_face_candidate_pose(candidate, pairs);
        }

        const auto correspondences = build_candidate_correspondences(candidate, pairs);
        if (!correspondences.has_value()) {
            return std::unexpected(
                std::string(
                    "solve_constrained_candidate_pose: build_candidate_correspondences failed"));
        }
        if (correspondences->obj_points.size() < 4u) {
            return std::unexpected(
                std::string("solve_constrained_candidate_pose: fewer than 4 object points"));
        }

        // 3+ pairs: use PnP with full yaw search (more stable across distance).
        // < 3 pairs: use face-assignment yaw (locked to avoid visibility drift).
        auto seed = candidate.pair_indices.size() >= 3u
                      ? solve_constrained_pose_seed(candidate, *correspondences, prior)
                      : solve_visible_pose_seed(candidate, pairs, *correspondences, prior);
        if (!seed.has_value() && candidate.pair_indices.size() < 3u) {
            seed = solve_constrained_pose_seed(candidate, *correspondences, prior);
        }
        if (!seed.has_value()) {
            return std::unexpected(
                std::string("solve_constrained_candidate_pose: no valid seed after pose solving"));
        }

        return build_pose_solution(
            candidate, *correspondences, bounded_pose_rotation(seed->yaw, seed->pitch, seed->roll),
            seed->translation);
    }

    [[nodiscard]] float
        projected_vertical_axis_score(const cv::Mat& rvec, const cv::Mat& tvec) const {
        const float half_axis =
            static_cast<float>(std::max(config_.geometry.pair_center_separation_m, 1e-3) * 0.5);
        std::vector<cv::Point2f> projected_axis;
        cv::projectPoints(
            std::vector<cv::Point3f>{
                cv::Point3f(0.0f, -half_axis, 0.0f), cv::Point3f(0.0f, half_axis, 0.0f)},
            rvec, tvec, camera_matrix_, dist_coeffs_, projected_axis);
        if (projected_axis.size() != 2u) {
            return 0.0f;
        }

        const cv::Point2f delta = projected_axis[1] - projected_axis[0];
        const float norm        = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        if (norm <= 1e-3f) {
            return 0.0f;
        }
        return std::clamp(delta.y / norm, 0.0f, 1.0f);
    }

    [[nodiscard]] std::expected<float, std::string> visible_face_score(
        const LdmMeshCandidate& candidate, const Eigen::Matrix3d& rotation_cam,
        const Eigen::Vector3d& center_cam) const {
        if (!has_face_assignment(candidate)) {
            return std::unexpected(
                std::string("visible_face_score: candidate has no face assignment"));
        }

        constexpr double kMinVisibleFaceAlignment = 1e-6;
        double score_sum                          = 0.0;
        int score_count                           = 0;
        for (const int face_idx : candidate.octagon_face_indices) {
            if (face_idx < 0 || face_idx >= 8) {
                return std::unexpected(std::string("visible_face_score: face index out of range"));
            }

            const Eigen::Vector3d normal_cam = rotation_cam * face_outward_normal_model(face_idx);
            const Eigen::Vector3d face_cam =
                rotation_cam * face_center_model(face_idx) + center_cam;
            const double face_range = face_cam.norm();
            if (!std::isfinite(face_range) || face_range <= 1e-9 || face_cam.z() <= 1e-6) {
                return std::unexpected(
                    std::string("visible_face_score: face range or z not finite/valid"));
            }

            const Eigen::Vector3d face_to_camera = -face_cam / face_range;
            const double alignment               = normal_cam.dot(face_to_camera);
            if (!std::isfinite(alignment) || alignment <= kMinVisibleFaceAlignment) {
                return std::unexpected(
                    std::string(
                        "visible_face_score: face-to-camera alignment not finite or too small"));
            }
            score_sum += std::clamp(alignment, 0.0, 1.0);
            ++score_count;
        }

        if (score_count <= 0) {
            return std::unexpected(std::string("visible_face_score: no valid scored faces"));
        }
        return static_cast<float>(score_sum / static_cast<double>(score_count));
    }

    void evaluate_candidate(
        LdmMeshCandidate& candidate, const std::vector<LightPair>& pairs,
        const OdomCameraTransform& T_odom_camera,
        const std::optional<PosePrior>& prior = std::nullopt) const {
        candidate.estimated_center_image_px =
            estimate_candidate_center_image(pairs, candidate.pair_indices);
        if (!face_sequence_is_consistent(candidate.octagon_face_indices)) {
            candidate.solved               = false;
            candidate.depth_valid          = false;
            candidate.reprojection_rmse_px = std::numeric_limits<float>::quiet_NaN();
            candidate.score                = 0.35f * candidate.preliminary_score;
            return;
        }

        const auto solution = solve_constrained_candidate_pose(candidate, pairs, prior);
        if (!solution.has_value()) {
            candidate.solved               = false;
            candidate.depth_valid          = false;
            candidate.reprojection_rmse_px = std::numeric_limits<float>::quiet_NaN();
            candidate.score                = 0.35f * candidate.preliminary_score;
            return;
        }

        candidate.solved                    = true;
        candidate.reprojection_rmse_px      = solution->reprojection_rmse_px;
        candidate.estimated_center_image_px = solution->projected_center_image;

        const auto transform_cam =
            CameraLdmTransform::from_rt(solution->rotation_cam, solution->center_cam);
        candidate.pose = LdmCandidatePose{
            .camera = transform_cam,
            .odom   = T_odom_camera * transform_cam,
        };
        candidate.projected_outline_image = solution->projected_outline_image;

        const float vertical_axis_term   = solution->vertical_axis_term;
        const float face_visibility_term = solution->face_visibility_term;

        const double rmse_limit = (candidate.pair_indices.size() >= 3)
                                    ? config_.rmse_stable_threshold_px
                                    : config_.rmse_constrained_threshold_px;
        candidate.depth_valid =
            std::isfinite(solution->center_cam.x()) && std::isfinite(solution->center_cam.y())
            && std::isfinite(solution->center_cam.z()) && solution->center_cam.z() > 1e-3
            && candidate.reprojection_rmse_px <= static_cast<float>(rmse_limit);

        const float pair_term = pair_count_score(static_cast<int>(candidate.pair_indices.size()));
        const float rmse_term = std::exp(
            -candidate.reprojection_rmse_px
            / static_cast<float>(std::max(1.0, config_.rmse_constrained_threshold_px)));
        const float depth_bonus = candidate.depth_valid ? 0.22f : -0.10f;
        candidate.score         = std::clamp(
            0.32f * candidate.preliminary_score + 0.20f * pair_term + 0.18f * rmse_term
                + 0.15f * vertical_axis_term + 0.15f * face_visibility_term + depth_bonus,
            0.0f, 1.2f);
    }

    [[nodiscard]] float pair_count_score(int pair_count) const {
        switch (pair_count) {
        case 1: return 0.25f;
        case 2: return 0.55f;
        case 3: return 0.82f;
        default: return 0.96f;
        }
    }

    [[nodiscard]] float confidence_for(const LdmMeshCandidate& candidate) const {
        const float base_pair = pair_count_score(static_cast<int>(candidate.pair_indices.size()));
        const float prelim    = candidate.preliminary_score;
        const float quality_term = (candidate.pair_indices.size() >= 3u) ? 0.95f : 0.70f;

        const float rmse_term =
            std::isfinite(candidate.reprojection_rmse_px)
                ? std::exp(
                      -candidate.reprojection_rmse_px
                      / static_cast<float>(std::max(1.0, config_.rmse_constrained_threshold_px)))
                : 0.0f;
        return std::clamp(
            0.35f * base_pair + 0.25f * prelim + 0.20f * rmse_term + 0.20f * quality_term, 0.0f,
            1.0f);
    }

private:
    LdmDetectorConfig config_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    double residual_scale_x_{1.0};
    double residual_scale_y_{1.0};
    double huber_delta_px_{1.0};
};

} // namespace fcs::L2::ldm
