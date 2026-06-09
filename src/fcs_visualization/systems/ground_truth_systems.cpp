#include "base.hpp"
#include "foxglove_types.hpp"
#include "scene_builder.hpp"

#include "core/channel_topics.hpp"
#include "frame.hpp"
#include "shm_layout.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace fcs::visualization::foxglove::systems {

namespace {

constexpr auto kRuneColor  = tac::with_alpha(tac::Semantic::STATUS_ACTIVE, 0.62);
constexpr auto kLabelColor = tac::Text::PRIMARY;

constexpr double kRuneSphereRadius = 0.15;
constexpr double kLabelOffsetZ     = 0.18;
constexpr double kLabelFontSize    = 0.08;

[[nodiscard]] const char* armor_label_name(uint8_t label) noexcept {
    switch (label) {
    case 1: return "Hero";
    case 2: return "Engineer";
    case 3: return "Infantry3";
    case 4: return "Infantry4";
    case 5: return "Infantry5";
    case 6: return "Sentry";
    case 7: return "Outpost";
    default: return "Unknown";
    }
}

[[nodiscard]] const char* team_name(uint8_t team) noexcept { return team == 0 ? "Red" : "Blue"; }

[[nodiscard]] const char* mechanism_state_name(uint8_t state) noexcept {
    switch (state) {
    case 0: return "Inactive";
    case 1: return "Activating";
    case 2: return "Activated";
    case 3: return "Failed";
    default: return "Unknown";
    }
}

[[nodiscard]] const char* rune_mode_name(uint8_t mode) noexcept {
    return mode == 0 ? "Small" : "Large";
}

} // namespace

/// @brief Register ground truth visualization systems (daedalus mode only)
///
/// Publishes:
/// - SceneUpdate (/ground_truth/scene): 3D spheres at each robot/rune position
/// - JSON (/ground_truth): raw ground truth data dump
void register_ground_truth_systems(talos::scheduler::Scheduler& app) {

    // =========================================================================
    // Ground Truth Scene + JSON publisher
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_ground_truth_pub",
        [](talos::spmc<ipc::GroundTruthBatch, GroundTruthBatchChannelTopic> gt_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, gt_in)) {
                return;
            }

            auto batch = gt_in.read();
            if (!batch) {
                return;
            }

            const uint64_t timestamp_ns = batch->timestamp_ns;
            std::vector<::foxglove::schemas::SceneEntity> entities;

            // --- Rune targets ---
            for (uint32_t i = 0; i < batch->rune_count; ++i) {
                const auto& rune = batch->runes[i];
                const Eigen::Vector3d center(
                    rune.r_center_odom[0], rune.r_center_odom[1], rune.r_center_odom[2]);

                // Rune center sphere
                auto entity =
                    viz::EntityBuilder::create<fast_tf::world>("gt", fmt::format("rune_{}", i))
                        .position(center)
                        .size(kRuneSphereRadius)
                        .color(kRuneColor)
                        .sphere()
                        .timestamp(timestamp_ns)
                        .build();

                // Label: "Red/Rune Small"
                const std::string label = fmt::format(
                    "{}/Rune {} [{}]", team_name(rune.team), rune_mode_name(rune.rune_mode),
                    mechanism_state_name(rune.mechanism_state));

                auto label_entity =
                    viz::EntityBuilder::create<fast_tf::world>(
                        "gt", fmt::format("rune_label_{}", i))
                        .position(center)
                        .color(kLabelColor)
                        .text_with_offset(label, 0, 0, kLabelOffsetZ, kLabelFontSize)
                        .timestamp(timestamp_ns)
                        .build();

                entities.push_back(std::move(entity));
                entities.push_back(std::move(label_entity));
            }

            // Publish scene
            if (!entities.empty()) {
                GroundTruthSceneMessage msg;
                msg.payload.entities = std::move(entities);
                (*server)->enqueue_message(std::move(msg));
            }

            // Publish JSON dump
            {
                nlohmann::json gt_json;
                gt_json["frame_seq"]    = batch->frame_seq;
                gt_json["timestamp_ns"] = batch->timestamp_ns;

                gt_json["targets"] = nlohmann::json::array();
                for (uint32_t i = 0; i < batch->target_count; ++i) {
                    const auto& tgt = batch->targets[i];
                    gt_json["targets"].push_back({
                        {       "team",                                 team_name(tgt.team)},
                        {"armor_label",                   armor_label_name(tgt.armor_label)},
                        { "is_outpost",                                      tgt.is_outpost},
                        {   "position", {tgt.position[0], tgt.position[1], tgt.position[2]}},
                        {       "vyaw",                                            tgt.vyaw},
                        {        "yaw",                                             tgt.yaw},
                    });
                }

                auto runes_obj = nlohmann::json::object();
                for (uint32_t i = 0; i < batch->rune_count; ++i) {
                    const auto& r           = batch->runes[i];
                    const char* mode        = rune_mode_name(r.rune_mode);
                    nlohmann::json rune_obj = {
                        {           "team",team_name(r.team)                                           },
                        {"mechanism_state",   mechanism_state_name(r.mechanism_state)},
                        {  "r_center_odom",
                         {r.r_center_odom[0], r.r_center_odom[1], r.r_center_odom[2]}},
                        {         "radius",                                  r.radius},
                        {  "current_angle",                           r.current_angle},
                        {         "v_roll",                                  r.v_roll},
                        {      "direction",                               r.direction},
                        {  "sin_amplitude",                           r.sin_amplitude},
                        {      "sin_omega",                               r.sin_omega},
                        {      "sin_phase",                               r.sin_phase},
                        {     "sin_offset",                              r.sin_offset},
                        {  "relative_time",                           r.relative_time},
                        {       "blade_id",                                r.blade_id},
                    };
                    std::vector<uint8_t> activations(
                        r.target_activations, r.target_activations + 5);
                    rune_obj["target_activations"] = activations;
                    runes_obj[mode]                = std::move(rune_obj);
                }
                gt_json["runes"] = std::move(runes_obj);

                detail::publish_json_message<GroundTruthMessage>(*server, gt_json);
            }
        });
}

} // namespace fcs::visualization::foxglove::systems
