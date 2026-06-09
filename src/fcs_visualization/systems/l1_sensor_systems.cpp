#include "L2_perception/ldm/ldm_config.hpp"
#include "L2_perception/ldm/types.hpp"
#include "base.hpp"
#include "core/runtime.hpp"
#include "foxglove_config.hpp"
#include "foxglove_types.hpp"
#include "quanta/stream_encoder.hpp"

#include "camera_config.hpp"
#include "core/channel_topics.hpp"
#include "core/types.hpp"
#include "frame.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <numbers>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <utility>

namespace fcs::visualization::foxglove::systems {

namespace detail {

struct QuantaFrameStamp {
    int64_t pts{0};
    uint64_t timestamp_ns{0};
};

struct LdmOverlayState {
    std::optional<L2::ldm::LdmDetection> latest_detection{};
    std::optional<L2::ldm::LdmMeasurement> latest_measurement{};
};

struct QuantaPublisherState {
    std::optional<quanta::StreamEncoder> encoder{};
    std::deque<QuantaFrameStamp> pending_frames{};
    std::string last_error{};
    int src_width{0};
    int src_height{0};
    int framerate{0};
    int64_t next_pts{0};
};

[[nodiscard]] inline cv::Rect2f
    projected_outline_bounds(const std::vector<cv::Point2f>& projected_outline) noexcept {
    if (projected_outline.empty()) {
        return {};
    }
    return cv::boundingRect(projected_outline);
}

[[nodiscard]] inline Eigen::Vector3d ldm_face_outward_normal_model(size_t face_idx) {
    const double angle = static_cast<double>(face_idx) * (std::numbers::pi_v<double> / 4.0);
    return Eigen::Vector3d(std::sin(angle), 0.0, -std::cos(angle));
}

[[nodiscard]] inline bool ldm_face_visible_from_camera(
    const L2::ldm::LdmMeshCandidate& candidate, const L2::ldm::LdmGeometryConfig& geometry,
    size_t face_idx) {
    if (!candidate.solved) {
        return true;
    }

    const auto& pose               = candidate.pose.camera;
    const Eigen::Vector3d normal   = ldm_face_outward_normal_model(face_idx);
    const Eigen::Vector3d normal_c = pose.rotation() * normal;
    const Eigen::Vector3d face_c   = pose.rotation()
                                     * (geometry.octagon_circumradius_m
                                        * std::cos(std::numbers::pi_v<double> / 8.0) * normal)
                                 + pose.translation();
    const double face_range = face_c.norm();
    if (!std::isfinite(face_range) || face_range <= 1e-9 || face_c.z() <= 1e-6) {
        return false;
    }

    return normal_c.dot(-face_c / face_range) > 1e-6;
}

[[nodiscard]] inline std::array<bool, 8> ldm_visible_faces_from_camera(
    const L2::ldm::LdmMeshCandidate& candidate, const L2::ldm::LdmGeometryConfig& geometry) {
    std::array<bool, 8> visible{};
    for (size_t face_idx = 0; face_idx < visible.size(); ++face_idx) {
        visible[face_idx] = ldm_face_visible_from_camera(candidate, geometry, face_idx);
    }
    return visible;
}

inline void draw_dashed_line(
    cv::Mat& image, cv::Point2f start, cv::Point2f end, const cv::Scalar& color, int thickness) {
    const double length = cv::norm(end - start);
    if (!std::isfinite(length) || length <= 1.0) {
        return;
    }

    constexpr double kDashPx    = 7.0;
    constexpr double kGapPx     = 5.0;
    constexpr double kStep      = kDashPx + kGapPx;
    const cv::Point2f direction = (end - start) * static_cast<float>(1.0 / std::max(length, 1.0));
    const int num_segments      = static_cast<int>(length / kStep) + 1;
    for (int seg = 0; seg < num_segments; ++seg) {
        const double offset   = static_cast<double>(seg) * kStep;
        const double dash_end = std::min(length, offset + kDashPx);
        cv::line(
            image, start + direction * static_cast<float>(offset),
            start + direction * static_cast<float>(dash_end), color, thickness, cv::LINE_AA);
    }
}

inline void draw_visibility_line(
    cv::Mat& image, cv::Point2f start, cv::Point2f end, const cv::Scalar& color, bool visible) {
    if (visible) {
        cv::line(image, start, end, color, tac::Image::LINE_MEDIUM, cv::LINE_AA);
        return;
    }

    const cv::Scalar dashed_color = tac::to_cv_bgr(tac::Image::LDM_SECONDARY);
    draw_dashed_line(image, start, end, dashed_color, tac::Image::LINE_THIN);
}

inline void draw_ldm_projected_outline(
    cv::Mat& image, const L2::ldm::LdmMeshCandidate& candidate,
    const L2::ldm::LdmGeometryConfig& geometry, const cv::Scalar& color) {
    const auto& projected_outline = candidate.projected_outline_image;
    if (projected_outline.size() != 16) {
        return;
    }

    const auto visible_faces = ldm_visible_faces_from_camera(candidate, geometry);
    for (size_t i = 0; i < 8; ++i) {
        const size_t next       = (i + 1) % 8;
        const bool side_visible = visible_faces[next];
        draw_visibility_line(
            image, projected_outline[i], projected_outline[next], color, side_visible);
        draw_visibility_line(
            image, projected_outline[i + 8], projected_outline[next + 8], color, side_visible);
        draw_visibility_line(
            image, projected_outline[i], projected_outline[i + 8], color,
            visible_faces[i] || visible_faces[next]);
    }
}

inline void log_quanta_error_once(QuantaPublisherState& state, std::string message) noexcept {
    if (state.last_error == message)
        return;
    state.last_error = std::move(message);
    SPDLOG_WARN("Foxglove quanta encoder: {}", state.last_error);
}

[[nodiscard]] inline bool ensure_quanta_encoder(
    QuantaPublisherState& state, const quanta::EncodeParams& cfg, int src_width,
    int src_height) noexcept {
    if (state.encoder && state.src_width == src_width && state.src_height == src_height
        && state.framerate == cfg.framerate) {
        return true;
    }

    auto enc = quanta::StreamEncoder::create(cfg, src_width, src_height, cfg.framerate);
    if (!enc) {
        log_quanta_error_once(state, std::move(enc.error()));
        state.encoder.reset();
        return false;
    }
    state.encoder.emplace(std::move(*enc));
    state.pending_frames.clear();
    state.next_pts   = 0;
    state.src_width  = src_width;
    state.src_height = src_height;
    state.framerate  = cfg.framerate;

    state.last_error.clear();
    SPDLOG_INFO(
        "Foxglove quanta encoder initialized: {}x{} -> <= {}x{}, {} bps @ {} fps", src_width,
        src_height, cfg.max_width, cfg.max_height, cfg.target_bitrate, cfg.framerate);
    return true;
}

[[nodiscard]] inline uint64_t take_timestamp_for_pts(
    QuantaPublisherState& state, int64_t pts, uint64_t fallback_timestamp_ns) noexcept {
    for (auto it = state.pending_frames.begin(); it != state.pending_frames.end(); ++it) {
        if (it->pts != pts)
            continue;

        const uint64_t timestamp_ns = it->timestamp_ns;
        state.pending_frames.erase(it);
        return timestamp_ns;
    }
    return fallback_timestamp_ns;
}

inline void publish_quanta_video(
    FoxgloveServer& server, QuantaPublisherState& state, const quanta::EncodeParams& cfg,
    const cv::Mat& image_bgr, uint64_t timestamp_ns) noexcept {
    if (!ensure_quanta_encoder(state, cfg, image_bgr.cols, image_bgr.rows))
        return;

    const int64_t pts = state.next_pts++;
    auto push_result =
        state.encoder->push_frame(image_bgr.data, static_cast<int>(image_bgr.step[0]), pts);
    if (!push_result) {
        log_quanta_error_once(state, std::move(push_result.error()));
        return;
    }

    state.pending_frames.push_back(QuantaFrameStamp{.pts = pts, .timestamp_ns = timestamp_ns});

    while (auto packet = state.encoder->poll_packet()) {
        const uint64_t packet_timestamp_ns =
            take_timestamp_for_pts(state, packet->pts, timestamp_ns);

        VideoMessage msg;
        msg.payload.timestamp = timestamp_from_ns(packet_timestamp_ns);
        msg.payload.frame_id  = "camera_optical_frame";
        msg.payload.format    = "h265";
        msg.payload.data      = std::vector<std::byte>(
            reinterpret_cast<const std::byte*>(packet->data.get()),
            reinterpret_cast<const std::byte*>(packet->data.get() + packet->size));

        server.enqueue_message(std::move(msg));
    }
}

inline void publish_jpeg_image(
    FoxgloveServer& server, const cv::Mat& image_bgr, uint64_t timestamp_ns) noexcept {
    std::vector<uint8_t> compressed;
    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 45};
    if (!cv::imencode(".jpg", image_bgr, compressed, params)) {
        return;
    }

    ImageMessage msg;
    msg.payload.timestamp = timestamp_from_ns(timestamp_ns);
    msg.payload.frame_id  = "camera_optical_frame";
    msg.payload.format    = "jpeg";
    msg.payload.data      = std::vector<std::byte>(
        reinterpret_cast<const std::byte*>(compressed.data()),
        reinterpret_cast<const std::byte*>(compressed.data() + compressed.size()));

    server.enqueue_message(std::move(msg));
}

[[nodiscard]] inline bool frame_sync_ok(
    uint64_t producer_timestamp_ns, uint64_t consumer_timestamp_ns, uint64_t max_skew_ns) noexcept {
    const uint64_t skew = (producer_timestamp_ns > consumer_timestamp_ns)
                            ? (producer_timestamp_ns - consumer_timestamp_ns)
                            : (consumer_timestamp_ns - producer_timestamp_ns);
    return skew <= max_skew_ns;
}

} // namespace detail

/// @brief Register L1 sensor layer systems (image publishing)
///
/// This includes:
/// - foxglove_image_pub: Publishes camera images with detection boxes overlaid
/// - Also publishes TF and camera calibration
void register_l1_sensor_systems(talos::scheduler::Scheduler& app) {

    // =========================================================================
    // IMAGE PUBLISHER (Camera calibration + TF)
    //
    // Synchronized rendering: image is bundled with detection result
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_l1_image_pub",
        [video_state = detail::QuantaPublisherState{}, ldm_state = detail::LdmOverlayState{}](
            talos::spmc<ArmorDetectionBatch, DetectionChannelTopic> det_in,
            talos::spmc<L2::ldm::LdmDetection, LdmDetectionChannelTopic> ldm_in,
            talos::spmc<L2::ldm::LdmMeasurement, LdmMeasurementChannelTopic> ldm_meas_in,
            talos::res<std::shared_ptr<FoxgloveServer>> server, talos::res<CameraConfig> cam,
            talos::res<FoxgloveConfig> foxglove_cfg,
            [[maybe_unused]] core::detecting_color detecting_color_,
            talos::res<fast_tf::CoordinateSystem> tf_buffer,
            talos::res<L2::ldm::LdmDetectorConfig> ldm_config) mutable {
            if (!foxglove_ready(*server, det_in)) {
                return;
            }

            auto batch = det_in.read();
            if (!batch || batch->image.empty()) {
                return;
            }

            // Publish TF
            (*server)->publish_tf(*tf_buffer, batch->timestamp_ns);

            // Clone image and draw detection boxes
            cv::Mat img_bgr = batch->image.clone();

            const cv::Scalar roi_color = tac::to_cv_bgr(
                batch->has_detector_roi ? tac::Image::ROI_VALID : tac::Image::ROI_MISSING);
            cv::rectangle(
                img_bgr, batch->detector_roi, roi_color, tac::Image::LINE_MEDIUM, cv::LINE_AA);

            // Draw optical center (principal point). cx, cy are at camera_matrix(0,2)/(1,2).
            const double cx = cam->camera_matrix(0, 2);
            const double cy = cam->camera_matrix(1, 2);
            cv::drawMarker(
                img_bgr, cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
                tac::to_cv_bgr(tac::Image::OPTICAL_CENTER), cv::MARKER_CROSS,
                tac::Image::MARKER_SIZE, tac::Image::LINE_MEDIUM);

            const cv::Scalar box_color       = tac::to_cv_bgr(tac::Image::DETECTION_BOX);
            const cv::Scalar text_color      = tac::to_cv_bgr(tac::Image::DETECTION_TEXT);
            std::array<cv::Scalar, 4> colors = {
                cv::Scalar(255, 0, 0),  // 红色 (RT)
                cv::Scalar(0, 255, 0),  // 绿色 (LT)
                cv::Scalar(0, 0, 255),  // 蓝色 (LB)
                cv::Scalar(0, 255, 255) // 黄色 (RB)
            };
            for (const auto& det : batch->detections) {
                // Draw detection box
                for (size_t j = 0; j < det.corners.size(); j++) {
                    auto pp1 = det.corners[j];
                    auto pp2 = det.corners[(j + 1) % 4];
                    cv::circle(img_bgr, pp1, 2, colors[(j + 1) % 4], tac::Image::LINE_MEDIUM);
                    cv::arrowedLine(
                        img_bgr, pp1, pp2, colors[(j + 1) % 4], tac::Image::LINE_MEDIUM,
                        cv::LINE_AA, 0, 10.0 / cv::norm(pp1 - pp2));
                }

                // Draw debug info below the box
                // Text position: bottom-left corner of the detection box
                cv::Point2f text_pos(det.rect.x, det.rect.y + det.rect.height + 20);

                // Line 1: name and type (e.g., "Sentry Small")
                std::string name_type = fmt::format(
                    "{} {} {}", magic_enum::enum_name(det.name), magic_enum::enum_name(det.type),
                    magic_enum::enum_name(det.color));
                cv::putText(
                    img_bgr, name_type, text_pos, cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_SMALL,
                    text_color, tac::Image::TEXT_THIN, cv::LINE_AA);

                // Line 2: confidence (e.g., "0.95")
                cv::Point2f conf_pos(det.rect.x, det.rect.y + det.rect.height + 40);
                std::string conf_str = fmt::format("conf: {:.2f}", det.confidence);
                cv::putText(
                    img_bgr, conf_str, conf_pos, cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_SMALL,
                    text_color, tac::Image::TEXT_THIN, cv::LINE_AA);
            }
            constexpr uint64_t kMaxLdmOverlaySkewNs = 200'000'000;
            const auto maybe_ldm_det                = ldm_in.read_current();
            const auto maybe_ldm_meas               = ldm_meas_in.read_current();
            if (maybe_ldm_det) {
                ldm_state.latest_detection = *maybe_ldm_det;
            }
            if (maybe_ldm_meas) {
                ldm_state.latest_measurement = *maybe_ldm_meas;
            }

            const auto use_ldm_det = ldm_state.latest_detection.has_value()
                                  && (ldm_state.latest_detection->frame_id == batch->frame_id
                                      || detail::frame_sync_ok(
                                          ldm_state.latest_detection->timestamp_ns,
                                          batch->timestamp_ns, kMaxLdmOverlaySkewNs));

            if (use_ldm_det) {
                const auto& ldm_det           = *ldm_state.latest_detection;
                const cv::Scalar blob_color   = tac::to_cv_bgr(tac::Image::LDM_SECONDARY);
                const cv::Scalar pair_color   = tac::to_cv_bgr(tac::Image::LDM_PRIMARY);
                const cv::Scalar center_color = tac::to_cv_bgr(tac::Image::LDM_CENTER);
                cv::Rect2f ldm_rect           = ldm_det.rect;

                // Draw blobs
                for (const auto& blob : ldm_det.blobs) {
                    cv::rectangle(
                        img_bgr, blob.rect, blob_color, tac::Image::LINE_THIN, cv::LINE_AA);
                }

                // Draw pairs
                for (size_t i = 0; i < ldm_det.pairs.size(); ++i) {
                    const auto& pair = ldm_det.pairs[i];
                    cv::line(
                        img_bgr, pair.top_center_px, pair.bottom_center_px, pair_color,
                        tac::Image::LINE_MEDIUM, cv::LINE_AA);
                    cv::circle(img_bgr, pair.top_center_px, 4, colors[0], tac::Image::LINE_MEDIUM);
                    cv::circle(
                        img_bgr, pair.bottom_center_px, 4, colors[2], tac::Image::LINE_MEDIUM);
                    cv::circle(img_bgr, pair.midpoint_px, 3, center_color, -1);
                    cv::putText(
                        img_bgr, fmt::format("p{}", i), pair.midpoint_px + cv::Point2f(4.0f, -4.0f),
                        cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_SMALL, center_color,
                        tac::Image::TEXT_THIN, cv::LINE_AA);
                }

                // Compute rect from pairs
                {
                    std::vector<cv::Point2f> pair_pts;
                    for (const auto& pair : ldm_det.pairs) {
                        pair_pts.push_back(pair.top_center_px);
                        pair_pts.push_back(pair.bottom_center_px);
                    }
                    if (!pair_pts.empty()) {
                        ldm_rect = cv::boundingRect(pair_pts);
                    }
                }

                if (ldm_rect.width > 0.0f && ldm_rect.height > 0.0f) {
                    cv::rectangle(
                        img_bgr, ldm_rect, pair_color, tac::Image::LINE_MEDIUM, cv::LINE_AA);
                    cv::putText(
                        img_bgr, fmt::format("LDM {}", ldm_det.pairs.size()),
                        cv::Point(
                            static_cast<int>(ldm_rect.x),
                            std::max(20, static_cast<int>(ldm_rect.y) - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_MEDIUM, pair_color,
                        tac::Image::TEXT_MEDIUM_PX, cv::LINE_AA);
                }
            }

            const auto use_ldm_meas = ldm_state.latest_measurement.has_value()
                                   && (ldm_state.latest_measurement->frame_id == batch->frame_id
                                       || detail::frame_sync_ok(
                                           ldm_state.latest_measurement->timestamp_ns,
                                           batch->timestamp_ns, kMaxLdmOverlaySkewNs));
            if (use_ldm_meas) {
                const auto& ldm_meas = *ldm_state.latest_measurement;
                cv::putText(
                    img_bgr,
                    fmt::format(
                        "LDM pairs={}/{} {} conf={:.2f}", ldm_meas.selected_pair_count,
                        ldm_meas.pair_count_total, ldm_meas.depth_quality, ldm_meas.confidence),
                    cv::Point(30, 32), cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_MEDIUM,
                    tac::to_cv_bgr(tac::Image::LDM_PRIMARY), tac::Image::TEXT_MEDIUM_PX,
                    cv::LINE_AA);

                if (ldm_meas.transform_cam.has_value()) {
                    const auto center = ldm_meas.transform_cam->translation();
                    cv::putText(
                        img_bgr,
                        fmt::format(
                            "cam=[{:.2f},{:.2f},{:.2f}]m", center.x(), center.y(), center.z()),
                        cv::Point(30, 56), cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_SMALL,
                        tac::to_cv_bgr(tac::Image::LDM_SECONDARY), tac::Image::TEXT_THIN,
                        cv::LINE_AA);
                }

                // Draw mesh projected outline and info from measurement
                const auto& selected_idx = ldm_meas.selected_candidate_idx;
                if (selected_idx.has_value() && *selected_idx >= 0
                    && static_cast<size_t>(*selected_idx) < ldm_meas.mesh_candidates.size()) {
                    const auto& selected =
                        ldm_meas.mesh_candidates[static_cast<size_t>(*selected_idx)];
                    if (selected.projected_outline_image.size() == 16) {
                        detail::draw_ldm_projected_outline(
                            img_bgr, selected, ldm_config->geometry, box_color);
                    }

                    // Draw center marker
                    cv::drawMarker(
                        img_bgr, ldm_meas.center_image_px, tac::to_cv_bgr(tac::Image::LDM_CENTER),
                        cv::MARKER_CROSS, 18, tac::Image::LINE_MEDIUM);

                    // Faces and RMSE text
                    std::string faces_text;
                    for (size_t i = 0; i < selected.octagon_face_indices.size(); ++i) {
                        if (!faces_text.empty()) {
                            faces_text += ",";
                        }
                        faces_text += std::to_string(selected.octagon_face_indices[i]);
                    }
                    const std::string rmse_text =
                        std::isfinite(selected.reprojection_rmse_px)
                            ? fmt::format("{:.2f}", selected.reprojection_rmse_px)
                            : "n/a";

                    const cv::Rect2f meas_rect =
                        detail::projected_outline_bounds(selected.projected_outline_image);
                    cv::putText(
                        img_bgr, fmt::format("ldm faces={} rmse={}", faces_text, rmse_text),
                        cv::Point(
                            static_cast<int>(meas_rect.x),
                            std::max(20, static_cast<int>(meas_rect.y) - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, tac::Image::TEXT_SMALL,
                        tac::to_cv_bgr(tac::Image::LDM_PRIMARY), tac::Image::TEXT_THIN,
                        cv::LINE_AA);
                }
            }

            if (foxglove_cfg->transport == FoxgloveTransport::Mcap) {
                detail::publish_quanta_video(
                    *(*server), video_state, foxglove_cfg->quanta, img_bgr, batch->timestamp_ns);
            } else {
                detail::publish_jpeg_image(*(*server), img_bgr, batch->timestamp_ns);
            }

            // Publish camera calibration
            std::array<double, 9> camera_matrix_arr;
            std::copy_n(cam->camera_matrix.data(), 9, camera_matrix_arr.begin());
            std::vector<double> dist_coeffs(
                cam->distort_coefficient.data(),
                cam->distort_coefficient.data() + cam->distort_coefficient.size());

            (*server)->publish_camera_calibration(
                cam->width, cam->height, camera_matrix_arr, dist_coeffs, batch->timestamp_ns);
        });
}

} // namespace fcs::visualization::foxglove::systems
