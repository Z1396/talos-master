#include "base.hpp"
#include "foxglove_types.hpp"
#include "scene_builder.hpp"
#include "scheduler/scheduler.hpp"

#include "L2_perception/rune/rune_config.hpp"
#include "L2_perception/rune/types.hpp"
#include "L3_estimation/energy_meter/types.hpp"
#include "core/channel_topics.hpp"
#include "frame.hpp"

#include <fmt/format.h>

namespace fcs::visualization::foxglove::systems {

/// @brief Register rune-specific visualization systems
///
/// This includes:
/// - foxglove_rune_debug_images: Publishes rune debug images (arrow, target, center)
/// - foxglove_rune_debug_json: Publishes rune debug JSON data
/// - foxglove_rune_scene: Publishes rune scene visualization
void register_rune_systems(talos::scheduler::Scheduler& app) {

    // =========================================================================
    // RUNE DEBUG IMAGES PUBLISHER
    // =========================================================================

    app.add_system<talos::fixed_rate<30>>(
        "foxglove_rune_debug_images",
        [](talos::spmc<::fcs::rune::RuneDebugFrame, RuneDebugFrameChannelTopic> dbg_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, dbg_in)) {
                return;
            }

            auto dbg = dbg_in.read();
            if (!dbg) {
                return;
            }

            auto publish = [&](const std::vector<uint8_t>& bytes, auto builder) {
                if (bytes.empty()) {
                    return;
                }
                auto msg              = builder();
                msg.payload.timestamp = timestamp_from_ns(dbg->timestamp_ns);
                msg.payload.frame_id  = "camera_optical_frame";
                msg.payload.format    = "jpeg";
                msg.payload.data      = std::vector<std::byte>(
                    reinterpret_cast<const std::byte*>(bytes.data()),
                    reinterpret_cast<const std::byte*>(bytes.data() + bytes.size()));
                (*server)->enqueue_message(std::move(msg));
            };

            publish(dbg->arrow_jpeg, [] { return RuneArrowImageMessage{}; });
            publish(dbg->target_jpeg, [] { return RuneTargetImageMessage{}; });
            publish(dbg->rcenter_jpeg, [] { return RuneCenterImageMessage{}; });
        });

    // =========================================================================
    // RUNE DEBUG JSON PUBLISHER
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_rune_debug_json",
        [](talos::spmc<::fcs::rune::RuneDebugFrame, RuneDebugFrameChannelTopic> dbg_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server,
           talos::res<::fcs::rune::RuneDetectorConfig> cfg) {
            if (!detail::foxglove_ready(*server, dbg_in)) {
                return;
            }

            auto dbg = dbg_in.read();
            if (!dbg) {
                return;
            }

            fmt::memory_buffer buf;
            fmt::format_to(
                std::back_inserter(buf),
                "{{\"timestamp_ns\":{},\"frame_id\":{},\"detect_ok\":{},\"tf_ok\":{}"
                ",\"solve_ok\":{},\"observation_valid\":{},\"status_code\":{},\"arrows\":{},"
                "\"targets\":{},\"global_roi\":{{\"x\":{:.3f},\"y\":{:.3f},\"w\":{:.3f},\"h\":{:."
                "3f}}},\"center_roi\":{{\"x\":{:.3f},\"y\":{:.3f},\"w\":{:.3f},\"h\":{:.3f}}},"
                "\"target_rois\":[",
                dbg->timestamp_ns, dbg->frame_id, dbg->detect_reversed ? "true" : "false",
                dbg->tf_ok ? "true" : "false", dbg->solve_ok ? "true" : "false",
                dbg->observation_valid ? "true" : "false", dbg->status_code, dbg->arrows_count,
                dbg->targets_count, dbg->global_roi.x, dbg->global_roi.y, dbg->global_roi.w,
                dbg->global_roi.h, dbg->center_roi.x, dbg->center_roi.y, dbg->center_roi.w,
                dbg->center_roi.h);

            for (size_t i = 0; i < dbg->target_rois.size(); ++i) {
                const auto& r = dbg->target_rois[i];
                if (i != 0) {
                    fmt::format_to(std::back_inserter(buf), ",");
                }
                fmt::format_to(
                    std::back_inserter(buf),
                    "{{\"x\":{:.3f},\"y\":{:.3f},\"w\":{:.3f},\"h\":{:.3f}}}", r.x, r.y, r.w, r.h);
            }

            fmt::format_to(
                std::back_inserter(buf),
                "],\"thresholds\":{{\"arrow\":{},\"target\":{},\"rcenter\":{}}}}}",
                cfg->arrow_threshold, cfg->target_threshold, cfg->rcenter_threshold);

            RuneDebugMessage msg;
            msg.payload = json_to_bytes(fmt::to_string(buf));
            (*server)->enqueue_message(std::move(msg));
        });

    // =========================================================================
    // RUNE SCENE PUBLISHER
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_rune_scene",
        [](talos::spmc<::fcs::rune::RuneObservation, RuneObservationChannelTopic> obs_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, obs_in)) {
                return;
            }

            auto obs = obs_in.read();
            if (!obs || !obs->valid) {
                return;
            }

            std::vector<::foxglove::schemas::SceneEntity> entities;

            entities.push_back(
                viz::EntityBuilder::create<fast_tf::odom>("rune", "r_center")
                    .position(obs->r_center_odom.translation())
                    .size(0.08)
                    .color(tac::L4::MPC_REFERENCE)
                    .sphere()
                    .timestamp(obs->timestamp_ns)
                    .build());

            for (size_t i = 0; i < obs->target_positions_odom.size(); ++i) {
                const auto& p = obs->target_positions_odom[i];
                entities.push_back(
                    viz::EntityBuilder::create<fast_tf::odom>("rune", fmt::format("target_{}", i))
                        .position(p)
                        .size(0.06)
                        .color(tac::L4::MPC_PRESENT)
                        .sphere()
                        .timestamp(obs->timestamp_ns)
                        .build());
            }

            publish_scene_if_nonempty<RuneSceneMessage>(*server, std::move(entities));
        });

    // =========================================================================
    // RUNE EKF SCENE — All 5 blade positions from EKF state
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_rune_ekf_scene",
        [](talos::spmc<::fcs::energy_meter::EnergyMeterState, EnergyMeterStateChannelTopic>
               state_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, state_in)) {
                return;
            }

            auto state = state_in.read();
            if (!state || !state->tracking_valid) {
                return;
            }

            std::vector<::foxglove::schemas::SceneEntity> entities;

            // R center
            entities.push_back(
                viz::EntityBuilder::create<fast_tf::odom>("rune_ekf", "r_center")
                    .position(state->r_center_odom)
                    .size(0.10)
                    .color(tac::L4::MPC_REFERENCE)
                    .sphere()
                    .timestamp(state->timestamp_ns)
                    .build());

            const double base_roll = state->roll;
            const double yaw       = state->yaw;
            const double pitch     = state->pitch;
            const double radius    = state->radius;
            const int tracked_id   = state->blade_id;

            // Rotation helpers: yaw (Z), pitch (Y)
            const double cy = std::cos(yaw);
            const double sy = std::sin(yaw);
            const double sp = std::sin(pitch);
            const double cp = std::cos(pitch);

            for (int i = 0; i < 5; ++i) {
                const double roll_i = base_roll + static_cast<double>(i) * 2.0 * M_PI / 5.0;
                const double sr     = std::sin(roll_i);
                const double cr     = std::cos(roll_i);

                // Blade in local frame (before yaw/pitch rotation)
                Eigen::Vector3d blade_local(0.0, -radius * sr, radius * cr);

                // Pitch rotation (Y axis)
                blade_local = Eigen::Vector3d(
                    blade_local.x() * cp + blade_local.z() * sp, blade_local.y(),
                    -blade_local.x() * sp + blade_local.z() * cp);

                // Yaw rotation (Z axis) + translate to center
                const Eigen::Vector3d pos =
                    state->r_center_odom
                    + Eigen::Vector3d(
                        blade_local.x() * cy - blade_local.y() * sy,
                        blade_local.x() * sy + blade_local.y() * cy, blade_local.z());

                const bool is_tracked = (i == tracked_id);
                const auto color =
                    is_tracked ? tac::L4::MPC_PRESENT : tac::with_alpha(tac::Semantic::INERT, 0.30);
                const double size = is_tracked ? 0.08 : 0.04;

                entities.push_back(
                    viz::EntityBuilder::create<fast_tf::odom>(
                        "rune_ekf", fmt::format("blade_{}", i))
                        .position(pos)
                        .size(size)
                        .color(color)
                        .sphere()
                        .text_with_offset(fmt::format("b{}", i), 0, 0, 0.06, tac::Text::SIZE_SMALL)
                        .timestamp(state->timestamp_ns)
                        .build());
            }

            // Rotation arc: line loop showing the blade circle
            {
                constexpr int arc_points = 24;
                std::vector<::foxglove::schemas::Point3> circle_points;
                circle_points.reserve(arc_points);
                for (int k = 0; k < arc_points; ++k) {
                    const double angle =
                        static_cast<double>(k) / static_cast<double>(arc_points) * 2.0 * M_PI;
                    const double sr = std::sin(angle);
                    const double cr = std::cos(angle);
                    Eigen::Vector3d local(0.0, -radius * sr, radius * cr);
                    local = Eigen::Vector3d(
                        local.x() * cp + local.z() * sp, local.y(),
                        -local.x() * sp + local.z() * cp);
                    Eigen::Vector3d world(
                        state->r_center_odom.x() + local.x() * cy - local.y() * sy,
                        state->r_center_odom.y() + local.x() * sy + local.y() * cy,
                        state->r_center_odom.z() + local.z());
                    circle_points.push_back(viz::make_point3(world));
                }
                auto arc_builder =
                    viz::EntityBuilder::create<fast_tf::odom>("rune_ekf", "arc")
                        .line_loop(
                            circle_points, tac::with_alpha(tac::Semantic::INERT, 0.24), 0.005)
                        .timestamp(state->timestamp_ns);
                entities.push_back(std::move(arc_builder).build());
            }

            // Model info text
            {
                std::string info;
                if (state->model_valid) {
                    info = fmt::format(
                        "a={:.3f} w={:.3f} b={:.3f} dir={}", state->a, state->omega, state->b,
                        state->direction);
                } else {
                    info = "model: converging";
                }
                entities.push_back(
                    viz::EntityBuilder::create<fast_tf::odom>("rune_ekf", "info")
                        .position(state->r_center_odom + Eigen::Vector3d(0, 0, radius + 0.15))
                        .text(info, tac::Text::SIZE_SMALL)
                        .timestamp(state->timestamp_ns)
                        .build());
            }

            publish_scene_if_nonempty<RuneEkfSceneMessage>(*server, std::move(entities));
        });
}

} // namespace fcs::visualization::foxglove::systems
