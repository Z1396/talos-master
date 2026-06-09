#include "L3_estimation/tracker/util.hpp"
#include "base.hpp"
#include "foxglove_types.hpp"
#include "scene_builder.hpp"

#include "L2_perception/ldm/ldm_config.hpp"
#include "L2_perception/ldm/ldm_geometry.hpp"
#include "L2_perception/ldm/types.hpp"
#include "core/channel_topics.hpp"
#include "core/types.hpp"
#include "frame.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <system_helpers.hpp>

namespace fcs::visualization::foxglove::systems {

namespace {

constexpr uint64_t kLdmSceneLifetimeNs = 5'000'000;

[[nodiscard]] nlohmann::json matrix4_to_json(const Eigen::Matrix4d& mat) {
    nlohmann::json rows = nlohmann::json::array();
    for (int r = 0; r < 4; ++r) {
        nlohmann::json row = nlohmann::json::array();
        for (int c = 0; c < 4; ++c) {
            row.push_back(mat(r, c));
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

[[nodiscard]] nlohmann::json matrix4_diag_to_json(const Eigen::Matrix4d& mat) {
    nlohmann::json diag = nlohmann::json::array();
    for (int i = 0; i < 4; ++i) {
        diag.push_back(mat(i, i));
    }
    return diag;
}

[[nodiscard]] std::string matrix4_row_string(const Eigen::Matrix4d& mat, int row) {
    return fmt::format(
        "[{:.3e}, {:.3e}, {:.3e}, {:.3e}]", mat(row, 0), mat(row, 1), mat(row, 2), mat(row, 3));
}

[[nodiscard]] ::foxglove::schemas::Color
    ldm_scene_color(const fcs::L2::ldm::LdmMeasurement& measurement) noexcept {
    switch (measurement.depth_quality) {
    case fcs::L2::ldm::LdmDepthQuality::Stable: return tac::L2::LDM_STABLE;
    case fcs::L2::ldm::LdmDepthQuality::Constrained: return tac::L2::LDM_CONSTRAINED;
    case fcs::L2::ldm::LdmDepthQuality::BearingOnly: return tac::L2::LDM_BEARING_ONLY;
    case fcs::L2::ldm::LdmDepthQuality::None: return tac::L2::LDM_NONE;
    }
    return tac::L2::LDM_NONE;
}

[[nodiscard]] std::string ldm_label(const fcs::L2::ldm::LdmMeasurement& measurement) {
    return fmt::format(
        "LDM {}/{} {}", measurement.selected_pair_count, measurement.pair_count_total,
        measurement.depth_quality);
}

// ============================================================================
// Face visibility from mesh candidate assignment
// ============================================================================

/// @brief Determine which octagon faces are visible based on the selected
///        mesh candidate's face assignment. Faces that the detector observed
///        (listed in `octagon_face_indices`) are always visible. Unobserved
///        faces are invisible (dashed in the 3D scene).
///
/// This is the correct "state space" for the 3D scene: the mesh candidate
/// itself tells us which faces were actually seen — there is no need for a
/// separate geometric visibility check from the camera.
[[nodiscard]] std::array<bool, 8>
    ldm_volume_assigned_faces(const fcs::L2::ldm::LdmMeasurement& measurement) noexcept {
    std::array<bool, 8> visible{};
    if (measurement.selected_candidate_idx.has_value() && *measurement.selected_candidate_idx >= 0
        && static_cast<size_t>(*measurement.selected_candidate_idx)
               < measurement.mesh_candidates.size()) {
        const auto& selected =
            measurement.mesh_candidates[static_cast<size_t>(*measurement.selected_candidate_idx)];
        for (const int face_idx : selected.octagon_face_indices) {
            if (face_idx >= 0 && face_idx < 8) {
                visible[static_cast<size_t>(face_idx)] = true;
            }
        }
    }
    return visible;
}

// ============================================================================
// Dashed 3D line simulation (Foxglove has no native dashed line support)
// ============================================================================

/// @brief Add a dashed edge from p1 to p2 as individual line_strip primitives.
inline void add_ldm_dashed_edge(
    viz::EntityBuilder& builder, const ::foxglove::schemas::Point3& p1,
    const ::foxglove::schemas::Point3& p2, const ::foxglove::schemas::Color& color,
    double thickness, double dash_len, double gap_len) {
    const double dx  = p2.x - p1.x;
    const double dy  = p2.y - p1.y;
    const double dz  = p2.z - p1.z;
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!std::isfinite(len) || len <= 1e-6) {
        return;
    }

    const double step  = dash_len + gap_len;
    const int num_segs = std::max(1, static_cast<int>(len / step));

    for (int s = 0; s < num_segs; ++s) {
        const double t0 = static_cast<double>(s) * step / len;
        const double t1 = std::min(1.0, (static_cast<double>(s) * step + dash_len) / len);
        if (t0 >= 1.0) {
            break;
        }
        builder.line_strip(
            {viz::make_point3(p1.x + dx * t0, p1.y + dy * t0, p1.z + dz * t0),
             viz::make_point3(p1.x + dx * t1, p1.y + dy * t1, p1.z + dz * t1)},
            color, thickness);
    }
}

// ============================================================================
// Add LDM volume outline edges to a builder with visibility-based styling
// ============================================================================

/// @brief Add all LDM volume edges (top ring, bottom ring, vertical) to the
///        given entity builder. Visible edges are drawn as solid line_strip
///        primitives; invisible (back‑facing) edges are drawn as dashed
///        line_strip primitives so the user can distinguish them at a glance.
inline void add_ldm_volume_edges(
    viz::EntityBuilder& builder, const fcs::L2::ldm::LdmDetectorConfig& config,
    const std::array<bool, 8>& visible_faces, const ::foxglove::schemas::Color& solid_color,
    const ::foxglove::schemas::Color& dashed_color, double thickness) {
    const auto outline = fcs::L2::ldm::volume_outline_points(config.geometry);

    // Convert cv::Point3f → foxglove::Point3
    std::array<::foxglove::schemas::Point3, 16> pts;
    for (size_t i = 0; i < 16; ++i) {
        pts[i] = viz::make_point3(
            static_cast<double>(outline[i].x), static_cast<double>(outline[i].y),
            static_cast<double>(outline[i].z));
    }

    constexpr double kDashLen = 0.020;
    constexpr double kGapLen  = 0.012;

    for (size_t i = 0; i < 8; ++i) {
        const size_t next = (i + 1) % 8;

        // --- Top ring edge: i → next (visible if face `next` is visible) ---
        if (visible_faces[next]) {
            builder.line_strip({pts[i], pts[next]}, solid_color, thickness);
        } else {
            add_ldm_dashed_edge(
                builder, pts[i], pts[next], dashed_color, thickness, kDashLen, kGapLen);
        }

        // --- Bottom ring edge: i+8 → next+8 (same visibility) ---
        if (visible_faces[next]) {
            builder.line_strip({pts[i + 8], pts[next + 8]}, solid_color, thickness);
        } else {
            add_ldm_dashed_edge(
                builder, pts[i + 8], pts[next + 8], dashed_color, thickness, kDashLen, kGapLen);
        }

        // --- Vertical edge: i ↔ i+8 (visible if either adjacent face is visible) ---
        const bool vert_visible = visible_faces[i] || visible_faces[next];
        if (vert_visible) {
            builder.line_strip({pts[i], pts[i + 8]}, solid_color, thickness);
        } else {
            add_ldm_dashed_edge(
                builder, pts[i], pts[i + 8], dashed_color, thickness, kDashLen, kGapLen);
        }
    }
}

// ============================================================================
// Entity builders
// ============================================================================

[[nodiscard]] ::foxglove::schemas::SceneEntity make_ldm_volume_entity_odom(
    const fcs::L2::ldm::LdmMeasurement& measurement,
    const fcs::L2::ldm::LdmDetectorConfig& config) {
    const auto color = ldm_scene_color(measurement);
    const auto& pose = *measurement.transform_odom;

    // Face visibility is determined by the selected mesh candidate's
    // observed face indices — no separate geometric check needed.
    const auto visible_faces = ldm_volume_assigned_faces(measurement);

    auto builder = viz::EntityBuilder::create<fast_tf::odom>("l2", "ldm");
    builder.timestamp(measurement.timestamp_ns)
        .lifetime(kLdmSceneLifetimeNs)
        .position(pose.translation())
        .orientation(pose.quaternion())
        .color(color);

    const auto dimmed_color = tac::scaled_rgb(color, 0.45);

    // Draw edges: solid for visible, dashed for invisible
    add_ldm_volume_edges(builder, config, visible_faces, color, dimmed_color, 0.0025);

    builder.size(0.025)
        .sphere()
        .text_with_offset(
            ldm_label(measurement), 0.0, 0.0, config.geometry.volume_height_m * 0.8,
            tac::L2::LABEL_FONT_SIZE)
        .metadata("confidence", fmt::format("{:.3f}", measurement.confidence))
        .metadata("depth_quality", std::string(magic_enum::enum_name(measurement.depth_quality)))
        .metadata("pair_count_total", std::to_string(measurement.pair_count_total))
        .metadata("selected_pair_count", std::to_string(measurement.selected_pair_count));
    return builder.build();
}

[[nodiscard]] ::foxglove::schemas::SceneEntity make_ldm_volume_entity_camera(
    const fcs::L2::ldm::LdmMeasurement& measurement,
    const fcs::L2::ldm::LdmDetectorConfig& config) {
    const auto color = ldm_scene_color(measurement);
    const auto& pose = *measurement.transform_cam;

    const auto visible_faces = ldm_volume_assigned_faces(measurement);
    auto builder             = viz::EntityBuilder::create<fast_tf::camera_optical>("l2", "ldm_cam");
    builder.timestamp(measurement.timestamp_ns)
        .lifetime(kLdmSceneLifetimeNs)
        .position(pose.translation())
        .orientation(pose.quaternion())
        .color(color);

    const auto dimmed_color = tac::scaled_rgb(color, 0.45);

    // Draw edges: solid for visible, dashed for invisible
    add_ldm_volume_edges(builder, config, visible_faces, color, dimmed_color, 0.0025);

    builder.size(0.0025)
        .sphere()
        .text_with_offset(
            ldm_label(measurement), 0.0, 0.0, config.geometry.volume_height_m * 0.8,
            tac::L2::LABEL_FONT_SIZE)
        .metadata("confidence", fmt::format("{:.3f}", measurement.confidence))
        .metadata("depth_quality", std::string(magic_enum::enum_name(measurement.depth_quality)))
        .metadata("pair_count_total", std::to_string(measurement.pair_count_total))
        .metadata("selected_pair_count", std::to_string(measurement.selected_pair_count));
    return builder.build();
}

[[nodiscard]] ::foxglove::schemas::SceneEntity
    make_ldm_bearing_entity(const fcs::L2::ldm::LdmMeasurement& measurement) {
    const auto color          = ldm_scene_color(measurement);
    const Eigen::Vector3d end = measurement.bearing_cam.normalized() * 2.0;

    return viz::EntityBuilder::create<fast_tf::camera_optical>("l2", "ldm_bearing")
        .timestamp(measurement.timestamp_ns)
        .lifetime(kLdmSceneLifetimeNs)
        .line_strip({viz::make_point3(0.0, 0.0, 0.0), viz::make_point3(end)}, color, 0.0025)
        .position(end)
        .color(color)
        .size(0.025)
        .sphere()
        .text_with_offset(ldm_label(measurement), 0.0, 0.0, 0.08, tac::L2::LABEL_FONT_SIZE)
        .metadata("confidence", fmt::format("{:.3f}", measurement.confidence))
        .metadata("depth_quality", std::string(magic_enum::enum_name(measurement.depth_quality)))
        .metadata("pair_count_total", std::to_string(measurement.pair_count_total))
        .metadata("selected_pair_count", std::to_string(measurement.selected_pair_count))
        .build();
}

} // namespace

/// @brief Register L2 perception layer systems (detection, measurement)
///
/// This includes:
/// - foxglove_l2_detection_pub: Publishes armor detection JSON data
/// - foxglove_l2_measurement_scene: Publishes measurement scene visualization
/// - foxglove_l2_measurement_pub: Publishes measurement JSON data
void register_l2_perception_systems(talos::scheduler::Scheduler& app) {

    // =========================================================================
    // DETECTION PUBLISHER
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_l2_detection_pub",
        [](talos::spmc<ArmorDetectionBatch, DetectionChannelTopic> det_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!foxglove_ready(*server, det_in)) {
                return;
            }

            auto batch = det_in.read();
            if (!batch) {
                return;
            }

            nlohmann::json armors_json;
            armors_json["timestamp_ns"]      = batch->timestamp_ns;
            armors_json["timestamp"]         = nlohmann::json::object();
            armors_json["timestamp"]["sec"]  = batch->timestamp_ns / 1000000000L;
            armors_json["timestamp"]["nsec"] = batch->timestamp_ns % 1000000000L;
            armors_json["armors"]            = nlohmann::json::array();

            for (size_t i = 0; i < batch->detections.size(); ++i) {
                const auto& det = batch->detections[i];
                nlohmann::json obj;
                obj["center"]["x"] = det.center().x;
                obj["center"]["y"] = det.center().y;

                obj["corners"] = nlohmann::json::array();
                for (const auto& pt : det.corners) {
                    obj["corners"].push_back({pt.x, pt.y});
                }
                obj["confidence"] = det.confidence;
                obj["color"]      = magic_enum::enum_name(det.color);
                obj["type"]       = magic_enum::enum_name(det.type);
                obj["name"]       = magic_enum::enum_name(det.name);
                armors_json["armors"].push_back(obj);
            }

            detail::publish_json_message<DebugArmorsMessage>(*server, armors_json);
        });

    // =========================================================================
    // MEASUREMENT VISUALIZATION (L2)
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_l2_measurement_scene",
        [](talos::spmc<ArmorMeasurementBatch, MeasurementChannelTopic> meas_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!foxglove_ready(*server, meas_in))
                return;

            auto batch = meas_in.read();
            if (!batch) {
                return;
            }

            std::vector<::foxglove::schemas::SceneEntity> entities;

            for (size_t i = 0; i < batch->measurements.size(); ++i) {
                const auto& m = batch->measurements[i];
                const auto t  = m.transform.translation();
                const auto q  = m.transform.quaternion();

                auto builder =
                    viz::EntityBuilder::create<fast_tf::odom>("l2", fmt::format("armor_{}", i));
                builder.position(t.x(), t.y(), t.z())
                    .orientation(q.x(), q.y(), q.z(), q.w())
                    .color(tac::L2::MEASUREMENT_GHOST)
                    .cube(
                        tac::Geometry::ARMOR_THICKNESS, tac::Geometry::ARMOR_HEIGHT_SMALL,
                        tac::Geometry::ARMOR_WIDTH)
                    .text_with_offset(
                        fmt::format(
                            "{} {}", magic_enum::enum_name(m.name), magic_enum::enum_name(m.type)),
                        0, -tac::Text::SIZE_MEDIUM, 0, tac::L2::LABEL_FONT_SIZE)
                    .metadata("confidence", fmt::format("{:.3f}", m.confidence))
                    .metadata("color", std::string(magic_enum::enum_name(m.color)))
                    .metadata("pnp_cov_order", "bearing_yaw,bearing_pitch,log_distance,armor_yaw")
                    .metadata("pnp_condition", fmt::format("{:.3e}", m.pnp_condition_number))
                    .metadata("pnp_cov[0]", matrix4_row_string(m.pnp_cov_ypdr, 0))
                    .metadata("pnp_cov[1]", matrix4_row_string(m.pnp_cov_ypdr, 1))
                    .metadata("pnp_cov[2]", matrix4_row_string(m.pnp_cov_ypdr, 2))
                    .metadata("pnp_cov[3]", matrix4_row_string(m.pnp_cov_ypdr, 3))
                    .timestamp(m.timestamp_ns);

                auto entity = builder.build();

                entities.push_back(std::move(entity));
            }

            // Publish to semantic channel
            publish_scene_if_nonempty<SceneMessage>(*server, std::move(entities));
        });

    app.add_system<talos::pool_compute>(
        "foxglove_l2_ldm_scene",
        [](talos::spmc<fcs::L2::ldm::LdmMeasurement, LdmMeasurementChannelTopic> ldm_meas_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server,
           talos::res<fcs::L2::ldm::LdmDetectorConfig> ldm_config) {
            if (!foxglove_ready(*server, ldm_meas_in)) {
                return;
            }

            auto measurement = ldm_meas_in.read();
            if (!measurement) {
                return;
            }

            std::vector<::foxglove::schemas::SceneEntity> entities;
            if (measurement->transform_odom.has_value()) {
                entities.push_back(make_ldm_volume_entity_odom(*measurement, *ldm_config));
            } else if (measurement->transform_cam.has_value()) {
                entities.push_back(make_ldm_volume_entity_camera(*measurement, *ldm_config));
            } else {
                entities.push_back(make_ldm_bearing_entity(*measurement));
            }

            publish_scene_if_nonempty<SceneMessage>(*server, std::move(entities));
        });

    // =========================================================================
    // LDM DETECTION JSON PUBLISHER
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_l2_ldm_detection_json",
        [](talos::spmc<fcs::L2::ldm::LdmDetection, LdmDetectionChannelTopic> det_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, det_in)) {
                return;
            }

            auto det = det_in.read();
            if (!det) {
                return;
            }

            nlohmann::json j;
            j["timestamp_ns"]      = det->timestamp_ns;
            j["frame_id"]          = det->frame_id;
            j["timestamp"]         = nlohmann::json::object();
            j["timestamp"]["sec"]  = det->timestamp_ns / 1000000000L;
            j["timestamp"]["nsec"] = det->timestamp_ns % 1000000000L;
            j["color"]             = std::string(magic_enum::enum_name(det->color));
            j["accurate"]          = det->accurate;
            j["rect"]              = nlohmann::json::object();
            j["rect"]["x"]         = det->rect.x;
            j["rect"]["y"]         = det->rect.y;
            j["rect"]["w"]         = det->rect.width;
            j["rect"]["h"]         = det->rect.height;

            if (det->center_image_px.has_value()) {
                j["center_image_px"]      = nlohmann::json::object();
                j["center_image_px"]["x"] = det->center_image_px->x;
                j["center_image_px"]["y"] = det->center_image_px->y;
            }

            j["blob_count"]      = det->blobs.size();
            j["pair_count"]      = det->pairs.size();
            j["candidate_count"] = det->mesh_candidates.size();

            j["candidates"] = nlohmann::json::array();
            for (const auto& cand : det->mesh_candidates) {
                nlohmann::json cj;
                cj["cluster_id"]           = cand.cluster_id;
                cj["solved"]               = cand.solved;
                cj["depth_valid"]          = cand.depth_valid;
                cj["preliminary_score"]    = cand.preliminary_score;
                cj["reprojection_rmse_px"] = cand.reprojection_rmse_px;
                cj["score"]                = cand.score;
                cj["pair_count"]           = cand.pair_indices.size();
                cj["face_indices"]         = nlohmann::json::array();
                for (const int fi : cand.octagon_face_indices) {
                    cj["face_indices"].push_back(fi);
                }
                j["candidates"].push_back(cj);
            }

            if (det->selected_candidate_idx.has_value()) {
                j["selected_candidate_idx"] = *det->selected_candidate_idx;
            }

            detail::publish_json_message<LdmDetectionMessage>(*server, j);
        });

    // =========================================================================
    // LDM MEASUREMENT JSON PUBLISHER
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_l2_ldm_measurement_json",
        [](talos::spmc<fcs::L2::ldm::LdmMeasurement, LdmMeasurementChannelTopic> meas_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, meas_in)) {
                return;
            }

            auto meas = meas_in.read();
            if (!meas) {
                return;
            }

            nlohmann::json j;
            j["timestamp_ns"]         = meas->timestamp_ns;
            j["frame_id"]             = meas->frame_id;
            j["timestamp"]            = nlohmann::json::object();
            j["timestamp"]["sec"]     = meas->timestamp_ns / 1000000000L;
            j["timestamp"]["nsec"]    = meas->timestamp_ns % 1000000000L;
            j["color"]                = std::string(magic_enum::enum_name(meas->color));
            j["accurate"]             = meas->accurate;
            j["pair_count_total"]     = meas->pair_count_total;
            j["selected_pair_count"]  = meas->selected_pair_count;
            j["center_image_px"]      = nlohmann::json::object();
            j["center_image_px"]["x"] = meas->center_image_px.x;
            j["center_image_px"]["y"] = meas->center_image_px.y;
            j["bearing_cam"]          = {
                meas->bearing_cam.x(), meas->bearing_cam.y(), meas->bearing_cam.z()};
            j["depth_quality"]   = std::string(magic_enum::enum_name(meas->depth_quality));
            j["confidence"]      = meas->confidence;
            j["candidate_count"] = meas->mesh_candidates.size();

            if (meas->selected_candidate_idx.has_value()) {
                j["selected_candidate_idx"] = *meas->selected_candidate_idx;
            }

            if (meas->transform_cam.has_value()) {
                const auto& t                     = meas->transform_cam->translation();
                const auto& q                     = meas->transform_cam->quaternion();
                j["transform_cam"]                = nlohmann::json::object();
                j["transform_cam"]["dist"]        = t.norm();
                j["transform_cam"]["position"]    = {t.x(), t.y(), t.z()};
                j["transform_cam"]["orientation"] = {q.x(), q.y(), q.z(), q.w()};
            }

            if (meas->transform_odom.has_value()) {
                const auto& t                      = meas->transform_odom->translation();
                const auto& q                      = meas->transform_odom->quaternion();
                j["transform_odom"]                = nlohmann::json::object();
                j["transform_odom"]["dist"]        = t.norm();
                j["transform_odom"]["position"]    = {t.x(), t.y(), t.z()};
                j["transform_odom"]["orientation"] = {q.x(), q.y(), q.z(), q.w()};
            }

            detail::publish_json_message<LdmMeasurementMessage>(*server, j);
        });

    // =========================================================================
    // MEASUREMENT JSON PUBLISHER
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_l2_measurement_pub",
        [](talos::spmc<ArmorMeasurementBatch, MeasurementChannelTopic> meas_in,
           talos::res<fast_tf::CoordinateSystem> tf_system,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, meas_in)) {
                return;
            }

            auto batch = meas_in.read();
            if (!batch) {
                return;
            }

            nlohmann::json j;
            j["timestamp_ns"]      = batch->timestamp_ns;
            j["timestamp"]         = nlohmann::json::object();
            j["timestamp"]["sec"]  = batch->timestamp_ns / 1000000000L;
            j["timestamp"]["nsec"] = batch->timestamp_ns % 1000000000L;
            j["measurements"]      = nlohmann::json::array();

            auto tf_lookup = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(
                *tf_system, batch->timestamp_ns);

            for (const auto& m : batch->measurements) {
                const auto t   = m.transform.translation();
                const auto q   = m.transform.quaternion();
                auto [r, p, y] = m.transform.euler_rot().rpy();

                nlohmann::json mj;
                mj["timestamp_ns"]          = m.timestamp_ns;
                mj["timestamp"]             = nlohmann::json::object();
                mj["timestamp"]["sec"]      = m.timestamp_ns / 1000000000L;
                mj["timestamp"]["nsec"]     = m.timestamp_ns % 1000000000L;
                mj["name"]                  = std::string(magic_enum::enum_name(m.name));
                mj["type"]                  = std::string(magic_enum::enum_name(m.type));
                mj["color"]                 = std::string(magic_enum::enum_name(m.color));
                mj["confidence"]            = m.confidence;
                mj["position"]              = {t.x(), t.y(), t.z()};
                mj["orientation"]           = {q.x(), q.y(), q.z(), q.w()};
                mj["euler"]["roll"]         = r;
                mj["euler"]["pitch"]        = p;
                mj["euler"]["yaw"]          = y;
                mj["distance"]              = t.norm();
                mj["pnp_geometry"]["order"] = {
                    "bearing_yaw", "bearing_pitch", "log_distance", "armor_yaw"};
                mj["pnp_geometry"]["cov_ypdr"]  = matrix4_to_json(m.pnp_cov_ypdr);
                mj["pnp_geometry"]["diag"]      = matrix4_diag_to_json(m.pnp_cov_ypdr);
                mj["pnp_geometry"]["condition"] = m.pnp_condition_number;
                mj["pnp_geometry"]["note"] =
                    "normalized-plane PnP covariance propagated to measurement coordinates";

                if (tf_lookup) {
                    auto target_in_ref = (m.transform);

                    auto target_pos_yaw = fcs::L3::xyz2ypd(target_in_ref.translation())[0];
                    auto [roll, pitch, target_yaw] = target_in_ref.euler_rot().rpy();

                    mj["delta_angle"] =
                        L3::shortest_rad(target_pos_yaw, target_yaw) * 180.0 / std::numbers::pi;
                }

                j["measurements"].push_back(mj);
            }

            detail::publish_json_message<MeasurementMessage>(*server, j);
        });
}

} // namespace fcs::visualization::foxglove::systems
