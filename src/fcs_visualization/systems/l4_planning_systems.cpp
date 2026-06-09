#include "L4_planning/selected_target_snapshot.hpp"
#include "L4_planning/target_selection_trace.hpp"
#include "L5_weapon/fire_control.hpp"
#include "base.hpp"
#include "core/time.hpp"
#include "core/trajectory/resource.hpp"
#include "foxglove_types.hpp"
#include "scene_builder.hpp"

#include "L4_planning/control_intent.hpp"
#include "core/channel_topics.hpp"
#include "frame.hpp"

#include <fmt/format.h>
#include <magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <system_helpers.hpp>
#include <tactical_palette.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace fcs::visualization::foxglove::systems {

namespace l4_detail {

struct FutureArmorSnapshot {
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
    double angular_velocity{0.0};
    std::vector<Eigen::Vector4d> armors;
};

[[nodiscard]] inline std::string yes_no(bool value) { return value ? "YES" : "NO"; }

[[nodiscard]] inline std::string tracker_identity(const ::fcs::L3::TrackerOutput& target) {
    return fmt::format(
        "{}/{}", magic_enum::enum_name(target.target_color),
        magic_enum::enum_name(target.target_name));
}

[[nodiscard]] inline std::string
    selection_identity(const ::fcs::L4::TargetSelectionTraceEntry& entry) {
    return fmt::format(
        "{}/{}", magic_enum::enum_name(entry.target_color),
        magic_enum::enum_name(entry.target_name));
}

[[nodiscard]] inline std::string format_score(double value) { return fmt::format("{:.3f}", value); }

[[nodiscard]] inline std::string format_image_center_px(double value) {
    return std::isfinite(value) ? fmt::format("{:.1f}", value) : "n/a";
}

[[nodiscard]] inline std::string
    selection_trace_entity_id(const ::fcs::L4::TargetSelectionTraceEntry& entry) {
    return fmt::format(
        "selection_candidate_{}_{}", magic_enum::enum_name(entry.target_color),
        magic_enum::enum_name(entry.target_name));
}

inline void append_selection_trace_metadata(
    viz::EntityBuilder& builder, const ::fcs::L4::TargetSelectionTrace& trace,
    const ::fcs::L4::TargetSelectionTraceEntry& entry) {
    builder.metadata("target_selection.identity", selection_identity(entry))
        .metadata("target_selection.rank", std::to_string(entry.rank))
        .metadata("target_selection.selected", yes_no(entry.selected))
        .metadata("target_selection.runner_up", yes_no(entry.runner_up))
        .metadata("target_selection.was_previously_selected", yes_no(entry.was_previously_selected))
        .metadata("target_selection.aim_valid", yes_no(entry.aim_valid))
        .metadata(
            "target_selection.track_status", std::string(magic_enum::enum_name(entry.track_status)))
        .metadata("target_selection.total_score", format_score(entry.total_score))
        .metadata("target_selection.image_center_score", format_score(entry.image_center_score))
        .metadata("target_selection.track_state_score", format_score(entry.track_state_score))
        .metadata("target_selection.tof_score", format_score(entry.tof_score))
        .metadata("target_selection.gimbal_effort_score", format_score(entry.gimbal_effort_score))
        .metadata("target_selection.armor_name_score", format_score(entry.armor_name_score))
        .metadata(
            "target_selection.image_center_weighted", format_score(entry.image_center_weighted))
        .metadata("target_selection.track_state_weighted", format_score(entry.track_state_weighted))
        .metadata("target_selection.tof_weighted", format_score(entry.tof_weighted))
        .metadata(
            "target_selection.gimbal_effort_weighted", format_score(entry.gimbal_effort_weighted))
        .metadata("target_selection.armor_name_weighted", format_score(entry.armor_name_weighted))
        .metadata("target_selection.weighted_sum", format_score(entry.weighted_sum))
        .metadata("target_selection.total_weight", format_score(entry.total_weight))
        .metadata(
            "target_selection.image_center_distance_px",
            format_image_center_px(entry.image_center_distance_px))
        .metadata("target_selection.optical_age_s", format_score(entry.optical_age_s))
        .metadata(
            "target_selection.tof_s",
            std::isfinite(entry.tof_s) ? format_score(entry.tof_s) : "n/a")
        .metadata(
            "target_selection.distance_m",
            std::isfinite(entry.distance_m) ? format_score(entry.distance_m) : "n/a")
        .metadata(
            "target_selection.yaw_effort_deg",
            std::isfinite(entry.yaw_effort_deg) ? format_score(entry.yaw_effort_deg) : "n/a")
        .metadata(
            "target_selection.pitch_effort_deg",
            std::isfinite(entry.pitch_effort_deg) ? format_score(entry.pitch_effort_deg) : "n/a")
        .metadata("target_selection.kept_current_target", yes_no(trace.kept_current_target))
        .metadata("target_selection.switch_margin", format_score(trace.switch_margin))
        .metadata(
            "target_selection.previous_target",
            trace.had_previous_target
                ? fmt::format(
                      "{}/{}", magic_enum::enum_name(trace.previous_target_color),
                      magic_enum::enum_name(trace.previous_target_name))
                : "none")
        .metadata("target_selection.aim_error", entry.aim_error.empty() ? "none" : entry.aim_error);

    if (entry.target_center) {
        builder.metadata(
            "target_selection.target_center",
            fmt::format(
                "{:.3f}, {:.3f}, {:.3f}", entry.target_center->x(), entry.target_center->y(),
                entry.target_center->z()));
    }
}

[[nodiscard]] inline std::string armor_id_label(int armor_id) {
    return fmt::format("Armor {}", armor_id);
}

[[nodiscard]] inline Eigen::Quaterniond armor_orientation(double yaw) {
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(tac::Geometry::ARMOR_TILT_ANGLE, Eigen::Vector3d::UnitY());
}

[[nodiscard]] inline std::string
    key_point_label(const std::string& name, const ::fcs::L5::TrajectoryPlanSample& sample) {
    return fmt::format("{} {:.3f}", name, sample.distance);
}

[[nodiscard]] inline nlohmann::json json_vec3(const Eigen::Vector3d& value) {
    return {value.x(), value.y(), value.z()};
}

[[nodiscard]] inline nlohmann::json
    json_filter_convergence(const ::fcs::L3::FilterConvergenceState& state) {
    return {
        {                       "status",                                    magic_enum::enum_name(state.status)},
        {"normalized_innovation_squared", std::isfinite(state.normalized_innovation_squared)
 ? nlohmann::json(state.normalized_innovation_squared)
 : nlohmann::json(nullptr)                                             },
        {          "max_covariance_diag",           std::isfinite(state.max_covariance_diag)
           ? nlohmann::json(state.max_covariance_diag)
           : nlohmann::json(nullptr)                                   },
        {"consecutive_converged_updates",                                    state.consecutive_converged_updates},
        { "consecutive_diverged_updates",                                     state.consecutive_diverged_updates},
    };
}

[[nodiscard]] inline nlohmann::json json_robot_state(const ::fcs::L3::RobotTargetState& state) {
    nlohmann::json state_json = {
        {       "type",                                    "robot"},
        {   "position",                  json_vec3(state.position)},
        {   "distance",                      state.position.norm()},
        {   "velocity",                  json_vec3(state.velocity)},
        {        "yaw",                                  state.yaw},
        {      "v_yaw",                                state.v_yaw},
        {     "radius",             {state.radius0, state.radius1}},
        {         "z1",                                   state.z1},
        { "armors_num",                           state.armors_num},
        {"convergence", json_filter_convergence(state.convergence)},
        {"armor_poses",                    nlohmann::json::array()},
    };

    const auto armors = state.armor_poses();
    for (size_t i = 0; i < armors.size(); ++i) {
        state_json["armor_poses"].push_back({
            {      "id",                        static_cast<int>(i)},
            {"position", {armors[i][0], armors[i][1], armors[i][2]}},
            {     "yaw",                               armors[i][3]},
        });
    }

    return state_json;
}

[[nodiscard]] inline nlohmann::json json_outpost_state(const ::fcs::L3::OutpostTargetState& state) {
    nlohmann::json state_json = {
        {       "type",                                  "outpost"},
        {   "position",   {state.position.x(), state.position.y()}},
        {   "distance",                      state.position.norm()},
        {   "velocity",                  json_vec3(state.velocity)},
        {        "yaw",                                  state.yaw},
        {      "v_yaw",                                state.v_yaw},
        {          "z",       {state.z[0], state.z[1], state.z[2]}},
        {     "radius",      ::fcs::L3::OutpostTargetState::radius},
        { "armors_num",  ::fcs::L3::OutpostTargetState::armors_num},
        {"convergence", json_filter_convergence(state.convergence)},
        {"armor_poses",                    nlohmann::json::array()},
    };

    const auto armors = state.armor_poses();
    for (size_t i = 0; i < armors.size(); ++i) {
        state_json["armor_poses"].push_back({
            {      "id",                        static_cast<int>(i)},
            {"position", {armors[i][0], armors[i][1], armors[i][2]}},
            {     "yaw",                               armors[i][3]},
        });
    }

    return state_json;
}

[[nodiscard]] inline nlohmann::json json_tracker_output(const ::fcs::L3::TrackerOutput& output) {
    nlohmann::json target_json = {
        {                 "timestamp_ns",           output.timestamp_ns                                         },
        {                       "status",                                   magic_enum::enum_name(output.status)},
        {                  "target_name",                              magic_enum::enum_name(output.target_name)},
        {                 "target_color",                             magic_enum::enum_name(output.target_color)},
        {                "target_jumped",                                                   output.target_jumped},
        {                "last_armor_id",
         output.last_armor_id ? nlohmann::json(*output.last_armor_id) : nlohmann::json(nullptr)                 },
        {"last_image_center_distance_px", std::isfinite(output.last_image_center_distance_px)
 ? nlohmann::json(output.last_image_center_distance_px)
 : nlohmann::json(nullptr)                                             },
        {"last_observation_timestamp_ns",                                   output.last_observation_timestamp_ns},
        {                   "state_kind",                                                                "empty"},
    };

    if (const auto* robot = output.robot_state()) {
        target_json["state_kind"] = "robot";
        target_json["state"]      = json_robot_state(*robot);
    } else if (const auto* outpost = output.outpost_state()) {
        target_json["state_kind"] = "outpost";
        target_json["state"]      = json_outpost_state(*outpost);
    } else {
        target_json["state"] = nlohmann::json::object();
    }

    return target_json;
}

[[nodiscard]] inline nlohmann::json
    json_target_selection_trace_entry(const ::fcs::L4::TargetSelectionTraceEntry& entry) {
    return nlohmann::json{
        {                    "rank",                                  entry.rank                                    },
        {             "target_name",                           std::string(magic_enum::enum_name(entry.target_name))},
        {            "target_color",                          std::string(magic_enum::enum_name(entry.target_color))},
        {            "track_status",                          std::string(magic_enum::enum_name(entry.track_status))},
        {               "aim_valid",                                                                 entry.aim_valid},
        { "was_previously_selected",                                                   entry.was_previously_selected},
        {                "selected",                                                                  entry.selected},
        {               "runner_up",                                                                 entry.runner_up},
        {               "aim_error",                                                                 entry.aim_error},
        {           "target_center",          entry.target_center ? nlohmann::json(json_vec3(*entry.target_center))
          : nlohmann::json(nullptr)                                             },
        {"image_center_distance_px",               std::isfinite(entry.image_center_distance_px)
               ? nlohmann::json(entry.image_center_distance_px)
               : nlohmann::json(nullptr)                                        },
        {           "optical_age_s",                                                             entry.optical_age_s},
        {                   "tof_s",
         std::isfinite(entry.tof_s) ? nlohmann::json(entry.tof_s) : nlohmann::json(nullptr)                         },
        {              "distance_m", std::isfinite(entry.distance_m) ? nlohmann::json(entry.distance_m)
 : nlohmann::json(nullptr)                                                      },
        {          "yaw_effort_deg",                         std::isfinite(entry.yaw_effort_deg)
                         ? nlohmann::json(entry.yaw_effort_deg)
                         : nlohmann::json(nullptr)                              },
        {        "pitch_effort_deg",                       std::isfinite(entry.pitch_effort_deg)
                       ? nlohmann::json(entry.pitch_effort_deg)
                       : nlohmann::json(nullptr)                                },
        {      "image_center_score",                                                        entry.image_center_score},
        {       "track_state_score",                                                         entry.track_state_score},
        {               "tof_score",                                                                 entry.tof_score},
        {     "gimbal_effort_score",                                                       entry.gimbal_effort_score},
        {        "armor_name_score",                                                          entry.armor_name_score},
        {   "image_center_weighted",                                                     entry.image_center_weighted},
        {    "track_state_weighted",                                                      entry.track_state_weighted},
        {            "tof_weighted",                                                              entry.tof_weighted},
        {  "gimbal_effort_weighted",                                                    entry.gimbal_effort_weighted},
        {     "armor_name_weighted",                                                       entry.armor_name_weighted},
        {            "weighted_sum",                                                              entry.weighted_sum},
        {            "total_weight",                                                              entry.total_weight},
        {             "total_score",                                                               entry.total_score},
    };
}

[[nodiscard]] inline nlohmann::json
    json_target_selection_trace(const ::fcs::L4::TargetSelectionTrace& trace) {
    nlohmann::json candidates = nlohmann::json::array();
    for (const auto& candidate : trace.candidates) {
        candidates.push_back(json_target_selection_trace_entry(candidate));
    }

    return nlohmann::json{
        {         "timestamp_ns",                                              trace.timestamp_ns},
        {  "had_previous_target",                                       trace.had_previous_target},
        { "previous_target_name",  std::string(magic_enum::enum_name(trace.previous_target_name))},
        {"previous_target_color", std::string(magic_enum::enum_name(trace.previous_target_color))},
        {  "kept_current_target",                                       trace.kept_current_target},
        {        "switch_margin",                                             trace.switch_margin},
        {           "candidates",                                           std::move(candidates)},
    };
}

[[nodiscard]] inline nlohmann::json json_control_intent(const ::fcs::L4::ControlIntent& intent) {
    nlohmann::json result;
    std::visit(
        [&](const auto& cmd) {
            using T                = std::decay_t<decltype(cmd)>;
            result["timestamp_ns"] = cmd.timestamp_ns;
            if constexpr (std::is_same_v<T, ::fcs::L4::TrackCommand>) {
                result["mode"]            = "Track";
                result["control_horizon"] = cmd.control_trajectory.horizon();
                result["fire_horizon"]    = cmd.fire_trajectory.horizon();
            } else if constexpr (std::is_same_v<T, ::fcs::L4::ShotCommand>) {
                result["mode"]     = "Shot";
                result["yaw"]      = cmd.yaw;
                result["pitch"]    = cmd.pitch;
                result["distance"] = cmd.distance;
                if (cmd.degradation_reason) {
                    result["degradation_reason"] = *cmd.degradation_reason;
                }
            } else if constexpr (std::is_same_v<T, ::fcs::L4::HoldCommand>) {
                result["mode"] = "Hold";
            }
        },
        intent);
    return result;
}

[[nodiscard]] inline nlohmann::json
    json_solver_target(const ::fcs::L4::SelectedTargetSnapshot& snapshot) {
    nlohmann::json target_json             = json_tracker_output(snapshot.tracker);
    target_json["timestamp_ns"]            = snapshot.timestamp_ns;
    target_json["timestamp"]               = nlohmann::json::object();
    target_json["timestamp"]["sec"]        = snapshot.timestamp_ns / 1000000000L;
    target_json["timestamp"]["nsec"]       = snapshot.timestamp_ns % 1000000000L;
    target_json["tracking"]                = snapshot.tracker.is_tracking();
    target_json["valid"]                   = snapshot.has_target();
    target_json["optimal_target"]          = target_json["valid"];
    target_json["source"]                  = std::string(magic_enum::enum_name(snapshot.source));
    target_json["plan_distance"]           = snapshot.distance;
    target_json["predicted_future_ns"]     = snapshot.predicted_future_ns;
    target_json["aim_phase"]               = std::string(magic_enum::enum_name(snapshot.aim_phase));
    target_json["selected_armor_id"]       = snapshot.selected_armor_id;
    target_json["rough_selected_armor_id"] = snapshot.rough_selected_armor_id;

    return target_json;
}

inline void append_selected_target_metadata(
    viz::EntityBuilder& builder, const ::fcs::L4::SelectedTargetSnapshot& snapshot) {
    builder.metadata("target.identity", tracker_identity(snapshot.tracker))
        .metadata("target.status", std::string(magic_enum::enum_name(snapshot.tracker.status)))
        .metadata(
            "target.last_image_center_distance_px",
            format_image_center_px(snapshot.tracker.last_image_center_distance_px))
        .metadata(
            "target.last_observation_timestamp_ns",
            std::to_string(snapshot.tracker.last_observation_timestamp_ns));
}

inline void append_plan_metadata(
    viz::EntityBuilder& builder, const ::fcs::L4::ControlIntent& intent,
    const ::fcs::L4::SelectedTargetSnapshot* selected_target = nullptr) {
    builder.metadata(
        "mode", std::visit(
                    [](const auto& cmd) -> std::string {
                        using T = std::decay_t<decltype(cmd)>;
                        if constexpr (std::is_same_v<T, ::fcs::L4::TrackCommand>)
                            return "Track";
                        if constexpr (std::is_same_v<T, ::fcs::L4::ShotCommand>)
                            return "Shot";
                        if constexpr (std::is_same_v<T, ::fcs::L4::HoldCommand>)
                            return "Hold";
                    },
                    intent));

    // Surface degradation in scene metadata if present.
    std::visit(
        [&](const auto& cmd) {
            using T = std::decay_t<decltype(cmd)>;
            if constexpr (std::is_same_v<T, ::fcs::L4::ShotCommand>) {
                if (cmd.degradation_reason) {
                    builder.metadata("degradation_reason", *cmd.degradation_reason);
                }
            }
        },
        intent);

    if (selected_target) {
        append_selected_target_metadata(builder, *selected_target);
    }
}

} // namespace l4_detail

/// @brief Register L4 planning layer systems (gimbal, MPC)
///
/// This includes:
/// - foxglove_solver_target_pub: Publishes the current optimal target JSON
/// - foxglove_target_selection_pub: Publishes full choose-optimal trace JSON
/// - foxglove_l4_gimbal_cmd_pub: Publishes gimbal command JSON
/// - foxglove_l4_gimbal_scene: Publishes gimbal scene (selection, prediction, trajectory)
/// - foxglove_l4_mpc_trajectory_pub: Publishes MPC trajectory JSON
/// - foxglove_l4_mpc_prediction_scene: Publishes full MPC horizon visualization
void register_l4_planning_systems(talos::scheduler::Scheduler& app) {
    app.add_system<talos::pool_compute>(
        "foxglove_solver_target_pub",
        [](talos::spmc<::fcs::L4::SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic>
               selected_target_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, selected_target_in)) {
                return;
            }

            auto selected_target = selected_target_in.read();
            if (!selected_target) {
                return;
            }

            detail::publish_json_message<TargetMessage>(
                *server, l4_detail::json_solver_target(*selected_target));
        });

    app.add_system<talos::pool_compute>(
        "foxglove_target_selection_pub",
        [](talos::spmc<::fcs::L4::TargetSelectionTrace, TargetSelectionTraceChannelTopic> trace_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, trace_in)) {
                return;
            }

            auto trace = trace_in.read();
            if (!trace) {
                return;
            }

            detail::publish_json_message<TargetSelectionTraceMessage>(
                *server, l4_detail::json_target_selection_trace(*trace));
        });

    // =========================================================================
    // GIMBAL COMMAND PUBLISHER
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_l4_gimbal_cmd_pub",
        [](talos::spmc<::fcs::L5::WeaponCommand, WeaponCommandChannelTopic> cmd_in,
           talos::spmc<::fcs::L4::ControlIntent, ControlIntentChannelTopic> plan_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, cmd_in)) {
                return;
            }

            auto cmd = cmd_in.read();
            if (!cmd || cmd->distance < 0) {
                return;
            }
            const auto plan_snapshot = plan_in.read_current();
            const auto* plan =
                (plan_snapshot
                 && std::visit([](const auto& c) { return c.timestamp_ns; }, *plan_snapshot)
                        == cmd->plan_timestamp_ns)
                    ? std::addressof(*plan_snapshot)
                    : nullptr;

            nlohmann::json cmd_json;
            cmd_json["timestamp_ns"]      = cmd->timestamp_ns;
            cmd_json["plan_timestamp_ns"] = cmd->plan_timestamp_ns;
            cmd_json["timestamp"]         = nlohmann::json::object();
            cmd_json["timestamp"]["sec"]  = cmd->timestamp_ns / 1000000000L;
            cmd_json["timestamp"]["nsec"] = cmd->timestamp_ns % 1000000000L;
            cmd_json["pre_plan"]          = {
                {     "yaw",      cmd->plan_yaw},
                {   "pitch",    cmd->plan_pitch},
                {"distance", cmd->plan_distance},
            };
            cmd_json["post_plan"] = {
                {        "yaw",   cmd->yaw},
                {      "pitch", cmd->pitch},
                {"fire_advice",  cmd->fire},
            };
            cmd_json["yaw"]                  = cmd->yaw;
            cmd_json["pitch"]                = cmd->pitch;
            cmd_json["v_yaw"]                = cmd->yaw_vel;
            cmd_json["v_pitch"]              = cmd->pitch_vel;
            cmd_json["a_yaw"]                = cmd->yaw_accel;
            cmd_json["a_pitch"]              = cmd->pitch_accel;
            cmd_json["distance"]             = cmd->distance;
            cmd_json["tof"]                  = cmd->tof;
            cmd_json["fire_advice"]          = cmd->fire;
            cmd_json["yaw_error"]            = cmd->yaw_error;
            cmd_json["pitch_error"]          = cmd->pitch_error;
            cmd_json["shooting_range_yaw"]   = cmd->shooting_range_yaw;
            cmd_json["shooting_range_pitch"] = cmd->shooting_range_pitch;
            cmd_json["ref_yaw"]              = cmd->ref_yaw;
            cmd_json["ref_pitch"]            = cmd->ref_pitch;
            if (cmd->degradation_reason) {
                cmd_json["degradation_reason"] = *cmd->degradation_reason;
            }
            if (cmd->viz_debug) {
                cmd_json["mpc"] = {
                    {   "center_index",          cmd->viz_debug->center_index},
                    {"lookahead_index",       cmd->viz_debug->lookahead_index},
                    { "reference_size", cmd->viz_debug->reference_plan.size()},
                    { "optimized_size", cmd->viz_debug->optimized_plan.size()},
                };
            }
            if (plan) {
                cmd_json["l4_plan"] = l4_detail::json_control_intent(*plan);
            }

            detail::publish_json_message<GimbalCmdMessage>(*server, cmd_json);
        });

    app.add_system<talos::pool_compute>(
        "foxglove_l4_mpc_trajectory_pub",
        [](talos::spmc<::fcs::L5::WeaponCommand, WeaponCommandChannelTopic> cmd_in,
           talos::spmc<::fcs::L4::ControlIntent, ControlIntentChannelTopic> plan_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, cmd_in)) {
                return;
            }

            auto cmd = cmd_in.read();
            if (!cmd || !cmd->viz_debug) {
                return;
            }
            const auto plan_snapshot = plan_in.read_current();
            const auto* plan =
                (plan_snapshot
                 && std::visit([](const auto& c) { return c.timestamp_ns; }, *plan_snapshot)
                        == cmd->plan_timestamp_ns)
                    ? std::addressof(*plan_snapshot)
                    : nullptr;

            nlohmann::json traj_json;
            traj_json["timestamp_ns"]      = cmd->timestamp_ns;
            traj_json["plan_timestamp_ns"] = cmd->plan_timestamp_ns;
            traj_json["center_index"]      = cmd->viz_debug->center_index;
            traj_json["lookahead_index"]   = cmd->viz_debug->lookahead_index;
            traj_json["reference"]         = nlohmann::json::array();
            traj_json["optimized"]         = nlohmann::json::array();

            for (int i = 0; i < static_cast<int>(cmd->viz_debug->reference_plan.size()); ++i) {
                const auto& reference = cmd->viz_debug->reference_plan[i];
                traj_json["reference"].push_back({
                    {          "index",                                i},
                    {"temporal_offset", i - cmd->viz_debug->center_index},
                    {            "yaw",                    reference.yaw},
                    {          "pitch",                  reference.pitch},
                    {       "distance",               reference.distance},
                    {            "tof",                    reference.tof},
                });
            }

            for (int i = 0; i < static_cast<int>(cmd->viz_debug->optimized_plan.size()); ++i) {
                const auto& optimized = cmd->viz_debug->optimized_plan[i];
                traj_json["optimized"].push_back({
                    {          "index",                                i},
                    {"temporal_offset", i - cmd->viz_debug->center_index},
                    {            "yaw",                    optimized.yaw},
                    {          "pitch",                  optimized.pitch},
                    {       "distance",               optimized.distance},
                    {            "tof",                    optimized.tof},
                });
            }

            if (plan) {
                traj_json["l4_plan"] = l4_detail::json_control_intent(*plan);
            }

            detail::publish_json_message<MpcTrajectoryMessage>(*server, traj_json);
        });

    // =========================================================================
    // GIMBAL SCENE VISUALIZATION (L4) - Selection + Prediction + Trajectory
    // =========================================================================

    app.add_system<talos::pool_compute>(
        "foxglove_l4_gimbal_scene",
        [](talos::spmc<::fcs::L4::ControlIntent, ControlIntentChannelTopic> plan_in,
           talos::spmc<::fcs::L4::SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic>
               selected_target_in,
           talos::spmc<::fcs::L4::TargetSelectionTrace, TargetSelectionTraceChannelTopic> trace_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, plan_in))
                return;
            auto intent = plan_in.read();
            if (!intent || std::holds_alternative<::fcs::L4::HoldCommand>(*intent))
                return;
            // Extract timestamp from variant for matching with other channels.
            const uint64_t plan_ts =
                std::visit([](const auto& cmd) -> uint64_t { return cmd.timestamp_ns; }, *intent);
            const auto selected_target_snapshot = selected_target_in.read_current();
            [[maybe_unused]] const auto* selected_target =
                (selected_target_snapshot && selected_target_snapshot->timestamp_ns == plan_ts)
                    ? std::addressof(*selected_target_snapshot)
                    : nullptr;
            const auto trace_snapshot = trace_in.read_current();
            const auto* trace         = (trace_snapshot && trace_snapshot->timestamp_ns == plan_ts)
                                          ? std::addressof(*trace_snapshot)
                                          : nullptr;

            std::vector<::foxglove::schemas::SceneEntity> entities;

            if (trace) {
                for (const auto& candidate : trace->candidates) {
                    if (!candidate.target_center) {
                        continue;
                    }

                    const auto tier  = candidate.selected  ? tac::SelectionTier::Selected
                                     : candidate.runner_up ? tac::SelectionTier::RunnerUp
                                                           : tac::SelectionTier::Eliminated;
                    const auto style = tac::selection_style(tier);

                    auto builder = viz::EntityBuilder::create<fast_tf::odom>(
                                       "l4", l4_detail::selection_trace_entity_id(candidate))
                                       .timestamp(trace->timestamp_ns)
                                       .position(*candidate.target_center)
                                       .size(tac::L4::SELECTION_SIZE * style.size_scale)
                                       .color(style.color)
                                       .alpha(style.alpha)
                                       .sphere();

                    if (style.show_label) {
                        builder.text_with_offset(
                            fmt::format(
                                "#{}{} {}", candidate.rank, candidate.selected ? "*" : "",
                                l4_detail::selection_identity(candidate)),
                            tac::L4::SELECTION_SIZE * 1.5, 0.0, tac::L3::LABEL_OFFSET_Z,
                            tac::Text::SIZE_SMALL);
                    }

                    l4_detail::append_selection_trace_metadata(builder, *trace, candidate);
                    entities.push_back(builder.build());
                }
            }
            publish_scene_if_nonempty<GimbalSceneMessage>(*server, std::move(entities));
        });

    app.add_system<talos::fixed_rate<250>>(
        "foxglove_l4_mpc_prediction_scene",
        [](talos::spmc<::fcs::L5::WeaponCommand, WeaponCommandChannelTopic> cmd_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server,
           core::trajectory::trajectory_solver solver, talos::res<fast_tf::CoordinateSystem> coord,
           core::trajectory::bullet_speed bullet_speed) {
            if (!detail::foxglove_ready(*server, cmd_in)) {
                return;
            }

            std::vector<::foxglove::schemas::SceneEntity> entities;
            auto cmd = cmd_in.read();
            if (cmd && cmd->viz_debug) {
                entities.reserve(2);

                // ── Reference trajectory connecting line ─────────────────────
                if (cmd->viz_debug->reference_plan.size() >= 2) {
                    std::vector<::foxglove::schemas::Point3> ref_line_points;
                    ref_line_points.reserve(cmd->viz_debug->reference_plan.size());
                    for (const auto& reference : cmd->viz_debug->reference_plan) {
                        ref_line_points.push_back(
                            detail::spherical_to_cartesian_point3(
                                reference.distance, reference.yaw, reference.pitch));
                    }
                    entities.push_back(
                        viz::EntityBuilder::create<fast_tf::odom>("l4", "mpc_ref_line")
                            .timestamp(cmd->timestamp_ns)
                            .line_strip(
                                std::move(ref_line_points), tac::L4::MPC_REFERENCE,
                                tac::L4::TRAJECTORY_LINE_THICKNESS)
                            .build());
                }

                // ── Optimized trajectory connecting line ─────────────────────
                if (cmd->viz_debug->optimized_plan.size() >= 2) {
                    std::vector<::foxglove::schemas::Point3> opt_line_points;
                    opt_line_points.reserve(cmd->viz_debug->optimized_plan.size());
                    for (const auto& optimized : cmd->viz_debug->optimized_plan) {
                        opt_line_points.push_back(
                            detail::spherical_to_cartesian_point3(
                                optimized.distance, optimized.yaw, optimized.pitch));
                    }
                    entities.push_back(
                        viz::EntityBuilder::create<fast_tf::odom>("l4", "mpc_opt_line")
                            .timestamp(cmd->timestamp_ns)
                            .line_strip(
                                std::move(opt_line_points), tac::L4::MPC_PRESENT,
                                tac::L4::TRAJECTORY_LINE_THICKNESS)
                            .build());
                }

                const int center_index = std::clamp(
                    cmd->viz_debug->center_index, 0,
                    static_cast<int>(cmd->viz_debug->optimized_plan.size()) - 1);

                const auto add_key_marker =
                    [&](const std::string& entity_id, const ::fcs::L5::TrajectoryPlanSample& sample,
                        const std::string& label, const ::foxglove::schemas::Color& color,
                        double size) {
                        entities.push_back(
                            viz::EntityBuilder::create<fast_tf::odom>("l4", entity_id)
                                .timestamp(cmd->timestamp_ns)
                                .position(
                                    detail::spherical_to_cartesian(
                                        sample.distance, sample.yaw, sample.pitch))
                                .size(size)
                                .color(color)
                                .sphere()
                                .text_with_offset(
                                    label, 0.0, 0.0, tac::L3::LABEL_OFFSET_Z, tac::Text::SIZE_SMALL)
                                .build());
                    };

                add_key_marker(
                    "mpc_ref_center",
                    cmd->viz_debug->reference_plan[static_cast<size_t>(center_index)],
                    l4_detail::key_point_label(
                        "Ref", cmd->viz_debug->reference_plan[static_cast<size_t>(center_index)]),
                    tac::L4::MPC_REFERENCE, tac::L4::SELECTION_SIZE);

                add_key_marker(
                    "mpc_opt_center",
                    cmd->viz_debug->optimized_plan[static_cast<size_t>(center_index)],
                    l4_detail::key_point_label(
                        "Opt", cmd->viz_debug->optimized_plan[static_cast<size_t>(center_index)]),
                    tac::L4::MPC_PRESENT, tac::L4::SELECTION_SIZE);
                auto trajectory = solver->get()->generate_trajectory(
                    cmd->pitch, bullet_speed->bullet_speed, cmd->distance);
                std::vector<Eigen::Vector3d> traj_points;
                traj_points.reserve(trajectory.size());
                for (const auto& [x, z] : trajectory) {
                    traj_points.emplace_back(
                        x * std::cos(cmd->pitch) + z * std::sin(cmd->pitch), 0,
                        -x * std::sin(cmd->pitch) + z * std::cos(cmd->pitch));
                }
                auto traj_entities =
                    viz::patterns::bullet_trajectory(traj_points, cmd->fire, cmd->timestamp_ns);
                entities.insert(
                    entities.end(), std::make_move_iterator(traj_entities.begin()),
                    std::make_move_iterator(traj_entities.end()));
            } else {

                auto f = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::gimbal>(
                    *coord, clock::now_ns());
                if (f) {
                    auto [r, p, y] = f->euler_rot().rpy();
                    auto trajectory =
                        solver->get()->generate_trajectory(p, bullet_speed->bullet_speed, 15.0);
                    std::vector<Eigen::Vector3d> traj_points;
                    traj_points.reserve(trajectory.size());
                    for (const auto& [x, z] : trajectory) {
                        traj_points.emplace_back(
                            x * std::cos(p) + z * std::sin(p), 0,
                            -x * std::sin(p) + z * std::cos(p));
                    }
                    auto traj_entities =
                        viz::patterns::bullet_trajectory(traj_points, false, clock::now_ns());
                    entities.insert(
                        entities.end(), std::make_move_iterator(traj_entities.begin()),
                        std::make_move_iterator(traj_entities.end()));
                }
            }
            publish_scene_if_nonempty<MpcPredictionSceneMessage>(*server, std::move(entities));
        });
}
} // namespace fcs::visualization::foxglove::systems
