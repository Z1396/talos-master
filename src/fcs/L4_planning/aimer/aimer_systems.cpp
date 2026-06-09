#include "L4_planning/aimer/aimer_systems.hpp"
#include "L3_estimation/ldm_naive/types.hpp"
#include "L4_planning/aimer/aimer.hpp"
#include "L4_planning/aimer/armor_target_decider.hpp"
#include "L4_planning/aimer/fsm.hpp"

#include "L2_perception/armor/readback_roi.hpp"
#include "L3_estimation/energy_meter/types.hpp"
#include "L3_estimation/tracker/types.hpp"
#include "L4_planning/aimer/types.hpp"
#include "L4_planning/common/transform_utils.hpp"
#include "L4_planning/config.hpp"
#include "L4_planning/control_intent.hpp"
#include "L4_planning/plan_source.hpp"
#include "L4_planning/selected_target_snapshot.hpp"
#include "L4_planning/target_selection_trace.hpp"
#include "L4_planning/trajectory_builder.hpp"
#include "camera_config.hpp"
#include "core/channel_topics.hpp"
#include "core/runtime.hpp"
#include "core/target_key.hpp"
#include "core/time.hpp"
#include "core/trajectory/resource.hpp"
#include "euler.hpp"
#include "scheduler/scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <expected>
#include <frame.hpp>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <spdlog/spdlog.h>
#include <unordered_map>
#include <vector>

namespace fcs::L4 {

namespace {

struct TargetPhaseState {
    ArmorAimPhase phase{ArmorAimPhase::SingleArmor};
    int overflow_count{0};
    std::optional<int> jumped_selected_armor_id{};
};

struct CandidateEvaluation {
    core::TargetKey key{};
    L3::TrackerOutput tracker{};
    TargetPrediction prediction{};
    TargetSelectionScores scores{};
    TargetSelectionTraceEntry trace_entry{};
};

struct GimbalEffortMetrics {
    double yaw_delta_rad{std::numeric_limits<double>::infinity()};
    double pitch_delta_rad{std::numeric_limits<double>::infinity()};
    double score{0.0};
};

struct ReadbackRoiTimestamps {
    uint64_t projection_timestamp_ns{0};
    uint64_t freshness_timestamp_ns{0};
};

struct ArmorCandidateCollection {
    std::vector<core::TargetKey> active_keys{};
    std::vector<CandidateEvaluation> candidates{};
    std::vector<TargetSelectionTraceEntry> rejected{};
};

[[nodiscard]] double clamp01(double value) noexcept { return std::clamp(value, 0.0, 1.0); }

[[nodiscard]] double elapsed_seconds(uint64_t start_ns, uint64_t end_ns) noexcept {
    if (end_ns <= start_ns) {
        return 0.0;
    }
    return static_cast<double>(end_ns - start_ns) * 1e-9;
}

[[nodiscard]] double
    optical_age_seconds(const L3::TrackerOutput& tracker, uint64_t current_ns) noexcept {
    if (tracker.last_observation_timestamp_ns == 0) {
        return 0.0;
    }
    return elapsed_seconds(tracker.last_observation_timestamp_ns, current_ns);
}

[[nodiscard]] core::TargetKey make_target_key(const L3::TrackerOutput& tracker) noexcept {
    return core::TargetKey{tracker.target_name, tracker.target_color};
}

[[nodiscard]] bool tracker_supports_readback_roi(const L3::TrackerOutput& tracker) noexcept {
    return tracker.is_tracking() && tracker.target_name != ArmorName::Invalid
        && (tracker.is_robot() || tracker.is_outpost());
}

[[nodiscard]] std::optional<L3::TrackerOutput> pick_readback_tracker_fallback(
    const std::optional<L3::TrackerOutputs>& trackers,
    const std::optional<core::TargetKey>& preferred_key) noexcept {
    if (!trackers) {
        return std::nullopt;
    }

    if (preferred_key.has_value()) {
        for (const auto& tracker : *trackers) {
            if (tracker_supports_readback_roi(tracker)
                && make_target_key(tracker) == *preferred_key) {
                return tracker;
            }
        }
    }

    const L3::TrackerOutput* best = nullptr;
    for (const auto& tracker : *trackers) {
        if (!tracker_supports_readback_roi(tracker)) {
            continue;
        }

        if (best == nullptr) {
            best = &tracker;
            continue;
        }

        const bool tracker_has_finite = std::isfinite(tracker.last_image_center_distance_px);
        const bool best_has_finite    = std::isfinite(best->last_image_center_distance_px);
        if (tracker_has_finite != best_has_finite) {
            if (tracker_has_finite) {
                best = &tracker;
            }
            continue;
        }

        if (tracker_has_finite
            && tracker.last_image_center_distance_px < best->last_image_center_distance_px) {
            best = &tracker;
            continue;
        }

        if (tracker.last_observation_timestamp_ns > best->last_observation_timestamp_ns) {
            best = &tracker;
        }
    }

    if (best == nullptr) {
        return std::nullopt;
    }
    return *best;
}

[[nodiscard]] ReadbackRoiTimestamps
    resolve_readback_roi_timestamps(const SelectedTargetSnapshot& snapshot) noexcept {
    const uint64_t projection_timestamp_ns =
        snapshot.tracker.timestamp_ns != 0 ? snapshot.tracker.timestamp_ns : snapshot.timestamp_ns;
    const uint64_t freshness_timestamp_ns = snapshot.tracker.last_observation_timestamp_ns != 0
                                              ? snapshot.tracker.last_observation_timestamp_ns
                                              : projection_timestamp_ns;
    return {
        .projection_timestamp_ns = projection_timestamp_ns,
        .freshness_timestamp_ns  = freshness_timestamp_ns,
    };
}

[[nodiscard]] std::optional<Eigen::Vector3d>
    tracker_center_world_position(const L3::TrackerOutput& tracker) noexcept {
    if (const auto* robot_state = tracker.robot_state()) {
        return robot_state->position;
    }
    if (const auto* outpost_state = tracker.outpost_state()) {
        return Eigen::Vector3d{
            outpost_state->position.x(),
            outpost_state->position.y(),
            (outpost_state->z[0] + outpost_state->z[1] + outpost_state->z[2]) / 3.0,
        };
    }
    return std::nullopt;
}

[[nodiscard]] bool contains_target_key(
    const std::vector<core::TargetKey>& keys, const core::TargetKey& key) noexcept {
    return std::find(keys.begin(), keys.end(), key) != keys.end();
}

[[nodiscard]] double target_abs_v_yaw(const L3::TrackerOutput& tracker) noexcept {
    if (const auto* robot_state = tracker.robot_state()) {
        return std::abs(robot_state->v_yaw);
    }
    if (const auto* outpost_state = tracker.outpost_state()) {
        return std::abs(outpost_state->v_yaw);
    }
    return 0.0;
}

[[nodiscard]] std::expected<TargetPrediction, std::string> aim_tracker_target(
    const Aimer& aimer, const L3::TrackerOutput& tracker, const ArmorAimContext& context,
    const Aimer::GimbalTransform& gimbal, const Aimer::MuzzleTransform& muzzle, uint64_t current_ns,
    double bullet_speed, const core::trajectory::solver::TrajectorySolver& solver) noexcept {
    if (const auto* robot_state = tracker.robot_state()) {
        return aimer.aim(
            *robot_state, context, gimbal, muzzle, tracker.timestamp_ns, current_ns, 0.0,
            bullet_speed, solver);
    }
    if (const auto* outpost_state = tracker.outpost_state()) {
        return aimer.aim(
            *outpost_state, context, gimbal, muzzle, tracker.timestamp_ns, current_ns, 0.0,
            bullet_speed, solver);
    }
    return std::unexpected("tracker output does not contain a supported target state");
}

[[nodiscard]] double score_image_center(
    const TargetSelectionConfig& cfg, const L3::TrackerOutput& tracker,
    uint64_t current_ns) noexcept {
    if (!std::isfinite(tracker.last_image_center_distance_px)
        || tracker.last_observation_timestamp_ns == 0 || cfg.refs.image_center_ref_px <= 1e-6) {
        return 0.0;
    }

    double score = 1.0 - tracker.last_image_center_distance_px / cfg.refs.image_center_ref_px;
    score        = clamp01(score);

    if (tracker.status == L3::TrackerStatus::TempLost) {
        if (cfg.optical_stale_timeout_s <= 1e-6) {
            return 0.0;
        }
        const double age = elapsed_seconds(tracker.last_observation_timestamp_ns, current_ns);
        score *= clamp01(1.0 - age / cfg.optical_stale_timeout_s);
    }

    return score;
}

[[nodiscard]] double
    score_track_state(const TargetSelectionConfig& cfg, const L3::TrackerOutput& tracker) noexcept {
    return tracker.status == L3::TrackerStatus::Tracking ? 1.0 : cfg.temp_lost_state_score;
}

[[nodiscard]] double
    score_tof(const TargetSelectionConfig& cfg, const TargetPrediction& prediction) noexcept {
    if (!std::isfinite(prediction.flying_time) || cfg.refs.tof_ref_s <= 1e-6) {
        return 0.0;
    }
    return clamp01(1.0 - prediction.flying_time / cfg.refs.tof_ref_s);
}

[[nodiscard]] GimbalEffortMetrics compute_gimbal_effort_metrics(
    const TargetSelectionConfig& cfg, double current_yaw, double current_pitch,
    const TargetPrediction& prediction) noexcept {
    GimbalEffortMetrics metrics;
    const double yaw_ref   = cfg.refs.yaw_effort_ref_deg * std::numbers::pi / 180.0;
    const double pitch_ref = cfg.refs.pitch_effort_ref_deg * std::numbers::pi / 180.0;
    if (yaw_ref <= 1e-6 || pitch_ref <= 1e-6) {
        return metrics;
    }

    metrics.yaw_delta_rad =
        std::abs(std::remainder(prediction.aim_yaw - current_yaw, 2.0 * std::numbers::pi));
    const double command_pitch_in_tf = -prediction.aim_pitch;
    metrics.pitch_delta_rad          = std::abs(command_pitch_in_tf - current_pitch);
    const double effort =
        0.5 * (metrics.yaw_delta_rad / yaw_ref + metrics.pitch_delta_rad / pitch_ref);
    metrics.score = clamp01(1.0 - effort);
    return metrics;
}

[[nodiscard]] TargetSelectionScores score_candidate(
    const TargetSelectionConfig& cfg, const L3::TrackerOutput& tracker,
    const TargetPrediction& prediction, double current_yaw, double current_pitch,
    uint64_t current_ns) noexcept {
    TargetSelectionScores scores;
    scores.image_center = score_image_center(cfg, tracker, current_ns);
    scores.track_state  = score_track_state(cfg, tracker);
    scores.tof          = score_tof(cfg, prediction);
    scores.gimbal_effort =
        compute_gimbal_effort_metrics(cfg, current_yaw, current_pitch, prediction).score;
    scores.armor_name = clamp01(cfg.armor_name_score.score(tracker.target_name));

    const double weighted_sum = cfg.weights.image_center * scores.image_center
                              + cfg.weights.track_state * scores.track_state
                              + cfg.weights.tof * scores.tof
                              + cfg.weights.gimbal_effort * scores.gimbal_effort
                              + cfg.weights.armor_name * scores.armor_name;
    const double total_weight = cfg.weights.image_center + cfg.weights.track_state + cfg.weights.tof
                              + cfg.weights.gimbal_effort + cfg.weights.armor_name;

    scores.total = total_weight > 1e-6 ? weighted_sum / total_weight : 0.0;
    return scores;
}

[[nodiscard]] TargetSelectionTraceEntry build_scored_trace_entry(
    const TargetSelectionConfig& cfg, const L3::TrackerOutput& tracker,
    const TargetPrediction& prediction, const TargetSelectionScores& scores, double current_yaw,
    double current_pitch, uint64_t current_ns, bool was_previously_selected) noexcept {
    const auto effort_metrics =
        compute_gimbal_effort_metrics(cfg, current_yaw, current_pitch, prediction);

    TargetSelectionTraceEntry entry;
    entry.target_name              = tracker.target_name;
    entry.target_color             = tracker.target_color;
    entry.track_status             = tracker.status;
    entry.aim_valid                = true;
    entry.was_previously_selected  = was_previously_selected;
    entry.target_center            = tracker_center_world_position(tracker);
    entry.image_center_distance_px = tracker.last_image_center_distance_px;
    entry.optical_age_s            = optical_age_seconds(tracker, current_ns);
    entry.tof_s                    = prediction.flying_time;
    entry.distance_m               = prediction.distance;
    entry.yaw_effort_deg           = effort_metrics.yaw_delta_rad * 180.0 / std::numbers::pi;
    entry.pitch_effort_deg         = effort_metrics.pitch_delta_rad * 180.0 / std::numbers::pi;

    entry.image_center_score  = scores.image_center;
    entry.track_state_score   = scores.track_state;
    entry.tof_score           = scores.tof;
    entry.gimbal_effort_score = scores.gimbal_effort;
    entry.armor_name_score    = scores.armor_name;

    entry.image_center_weighted  = cfg.weights.image_center * scores.image_center;
    entry.track_state_weighted   = cfg.weights.track_state * scores.track_state;
    entry.tof_weighted           = cfg.weights.tof * scores.tof;
    entry.gimbal_effort_weighted = cfg.weights.gimbal_effort * scores.gimbal_effort;
    entry.armor_name_weighted    = cfg.weights.armor_name * scores.armor_name;
    entry.weighted_sum           = entry.image_center_weighted + entry.track_state_weighted
                       + entry.tof_weighted + entry.gimbal_effort_weighted
                       + entry.armor_name_weighted;
    entry.total_weight = cfg.weights.image_center + cfg.weights.track_state + cfg.weights.tof
                       + cfg.weights.gimbal_effort + cfg.weights.armor_name;
    entry.total_score = scores.total;
    return entry;
}

[[nodiscard]] TargetSelectionTraceEntry build_rejected_trace_entry(
    const L3::TrackerOutput& tracker, uint64_t current_ns, std::string reason,
    bool was_previously_selected) noexcept {
    TargetSelectionTraceEntry entry;
    entry.target_name              = tracker.target_name;
    entry.target_color             = tracker.target_color;
    entry.track_status             = tracker.status;
    entry.was_previously_selected  = was_previously_selected;
    entry.target_center            = tracker_center_world_position(tracker);
    entry.image_center_distance_px = tracker.last_image_center_distance_px;
    entry.optical_age_s            = optical_age_seconds(tracker, current_ns);
    entry.aim_valid                = false;
    entry.aim_error                = std::move(reason);
    return entry;
}

[[nodiscard]] TargetSelectionTrace build_target_selection_trace(
    uint64_t current_ns, const std::optional<core::TargetKey>& previous_key,
    const ArmorTargetDecider& decider, double switch_margin,
    const ArmorTargetDeciderResult& decider_result,
    const std::vector<CandidateEvaluation>& candidates,
    std::vector<TargetSelectionTraceEntry> rejected) {
    TargetSelectionTrace trace{
        .timestamp_ns        = current_ns,
        .kept_current_target = decider_result.kept_current_target,
        .switch_margin =
            std::holds_alternative<UnmannedArmorTargetDecider>(decider) ? switch_margin : 0.0,
    };

    if (previous_key.has_value()) {
        trace.had_previous_target   = true;
        trace.previous_target_name  = previous_key->name;
        trace.previous_target_color = previous_key->color;
    }

    trace.candidates.reserve(candidates.size() + rejected.size());
    for (size_t rank = 0; rank < decider_result.ranked_indices.size(); ++rank) {
        const size_t candidate_index = decider_result.ranked_indices[rank];
        auto candidate_trace         = candidates[candidate_index].trace_entry;
        candidate_trace.rank         = static_cast<int>(rank + 1);
        candidate_trace.selected     = candidate_index == decider_result.selected_index;
        candidate_trace.runner_up =
            decider_result.runner_up.has_value() && candidate_index == *decider_result.runner_up;
        trace.candidates.push_back(std::move(candidate_trace));
    }

    for (auto& rejected_entry : rejected) {
        rejected_entry.rank = static_cast<int>(trace.candidates.size() + 1);
        trace.candidates.push_back(std::move(rejected_entry));
    }

    return trace;
}

// Decision — 三种目标源输出的统一类型
struct AimDecision {
    ControlIntent intent;
    SelectedTargetSnapshot snapshot;
    TargetSelectionTrace trace;
};

// AimContext — 三个决策函数共享的只读上下文
struct AimContext {
    const Aimer::GimbalTransform& gimbal;
    const Aimer::MuzzleTransform& muzzle;
    uint64_t current_ns;
    double bullet_speed;
    const core::trajectory::solver::TrajectorySolver& solver;
    const Aimer& aimer;
};

// 从 TargetPrediction 构造 SelectedTargetSnapshot（三个 try_* 共享）。
[[nodiscard]] SelectedTargetSnapshot make_snapshot(
    const TargetPrediction& pred, GimbalPlanSource source,
    L3::TrackerOutput tracker = {}) noexcept {
    return SelectedTargetSnapshot{
        .timestamp_ns            = pred.timestamp_ns,
        .source                  = source,
        .distance                = pred.distance,
        .predicted_future_ns     = pred.predicted_future_ns,
        .aim_phase               = pred.aim_phase,
        .selected_armor_id       = pred.selected_armor_id,
        .rough_selected_armor_id = pred.rough_selected_armor_id,
        .tracker                 = std::move(tracker),
    };
}

// 从 TrackerOutput + TargetPrediction 构造 PlannerSeed（仅 try_armor 使用）。
[[nodiscard]] std::optional<PlannerSeed>
    make_planner_seed(const L3::TrackerOutput& tracker, const TargetPrediction& pred) noexcept {
    PlannerSeed s;
    s.state_timestamp_ns    = tracker.timestamp_ns;
    s.target_jumped         = tracker.target_jumped;
    s.tracker_last_armor_id = tracker.last_armor_id;
    s.aim_phase             = pred.aim_phase;
    s.selected_armor_id     = pred.selected_armor_id;
    s.armor_type            = pred.armor_type;
    if (const auto* rs = tracker.robot_state()) {
        s.state = *rs;
        return s;
    }
    if (const auto* os = tracker.outpost_state()) {
        s.state = *os;
        return s;
    }
    return std::nullopt;
}

[[nodiscard]] ArmorAimContext build_armor_aim_context(
    const L3::TrackerOutput& tracker, const TargetPhaseState& phase_state) noexcept {
    ArmorAimContext aim_ctx;
    aim_ctx.target_jumped      = tracker.target_jumped;
    aim_ctx.phase              = phase_state.phase;
    aim_ctx.preferred_armor_id = tracker.last_armor_id;
    if (tracker.target_jumped && phase_state.jumped_selected_armor_id.has_value()) {
        aim_ctx.preferred_armor_id = phase_state.jumped_selected_armor_id;
    }
    return aim_ctx;
}

[[nodiscard]] ArmorCandidateCollection collect_armor_candidates(
    const std::optional<L3::TrackerOutputs>& trackers, const AimContext& ctx,
    const AimerConfig& aimer_cfg, const TargetSelectionConfig& sel_cfg,
    std::unordered_map<core::TargetKey, TargetPhaseState, core::TargetKeyHash>& phase_states,
    const std::optional<core::TargetKey>& previous_key) {
    ArmorCandidateCollection result;

    const size_t reserve_n = trackers ? trackers->size() : 0U;
    result.active_keys.reserve(reserve_n);
    result.candidates.reserve(reserve_n);
    result.rejected.reserve(reserve_n);

    if (!trackers.has_value()) {
        return result;
    }

    const auto cur_euler   = ctx.gimbal.euler_rot();
    const double cur_yaw   = cur_euler.yaw;
    const double cur_pitch = cur_euler.pitch;

    for (const auto& tracker : *trackers) {
        if (!tracker.is_tracking()) {
            continue;
        }

        const auto key = make_target_key(tracker);
        result.active_keys.push_back(key);
        const bool was_prev = previous_key.has_value() && key == *previous_key;

        auto& phase_state = phase_states[key];
        advance_armor_aim_phase(
            aimer_cfg, target_abs_v_yaw(tracker), tracker.target_jumped, phase_state.phase,
            phase_state.overflow_count);

        auto prediction = aim_tracker_target(
            ctx.aimer, tracker, build_armor_aim_context(tracker, phase_state), ctx.gimbal,
            ctx.muzzle, ctx.current_ns, ctx.bullet_speed, ctx.solver);
        if (!prediction) {
            result.rejected.push_back(
                build_rejected_trace_entry(tracker, ctx.current_ns, prediction.error(), was_prev));
            continue;
        }

        CandidateEvaluation candidate;
        candidate.key        = key;
        candidate.tracker    = tracker;
        candidate.prediction = *prediction;
        candidate.scores =
            score_candidate(sel_cfg, tracker, *prediction, cur_yaw, cur_pitch, ctx.current_ns);
        candidate.trace_entry = build_scored_trace_entry(
            sel_cfg, tracker, *prediction, candidate.scores, cur_yaw, cur_pitch, ctx.current_ns,
            was_prev);
        result.candidates.push_back(std::move(candidate));
    }

    return result;
}

auto cleanup_stale_phase_states(
    const std::vector<core::TargetKey>& active_keys,
    std::unordered_map<core::TargetKey, TargetPhaseState, core::TargetKeyHash>& phase_states,
    ArmorTargetDeciderState& decider_state) -> void {
    if (active_keys.empty()) {
        phase_states.clear();
        decider_state.selected_key.reset();
        return;
    }

    std::erase_if(
        phase_states, [&](const auto& kv) { return !contains_target_key(active_keys, kv.first); });
}

auto update_selected_phase_state(
    CandidateEvaluation& selected,
    std::unordered_map<core::TargetKey, TargetPhaseState, core::TargetKeyHash>& phase_states)
    -> void {
    selected.prediction.armor_type = cls_to_armor_type(selected.tracker.target_name);

    auto& phase_state      = phase_states[selected.key];
    const bool jump_active = selected.tracker.is_tracking() && selected.tracker.target_jumped;
    const bool keep_jump_memory =
        selected.prediction.aim_phase != ArmorAimPhase::SingleArmor || jump_active;
    if (!keep_jump_memory) {
        phase_state.jumped_selected_armor_id.reset();
        return;
    }

    phase_state.jumped_selected_armor_id = selected.prediction.selected_armor_id;
}

auto reset_selected_target_key_for_non_armor_source(ArmorTargetDeciderState& decider_state)
    -> void {
    decider_state.selected_key.reset();
}

auto write_hold_outputs(
    talos::spmc_mut<SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic>& out_snap,
    talos::spmc_mut<TargetSelectionTrace, TargetSelectionTraceChannelTopic>& out_trace,
    talos::spmc_mut<ControlIntent, ControlIntentChannelTopic>& out_ctrl,
    ArmorTargetDeciderState& decider_state, const std::optional<L3::TrackerOutput>& fallback)
    -> void {
    const uint64_t now_ns = fcs::clock::now_ns();
    SelectedTargetSnapshot snap{.timestamp_ns = now_ns};
    if (fallback.has_value()) {
        snap.source                = GimbalPlanSource::Armor;
        snap.tracker               = *fallback;
        decider_state.selected_key = make_target_key(*fallback);
    } else {
        decider_state.selected_key.reset();
    }

    out_snap.write(std::move(snap));
    out_trace.write(TargetSelectionTrace{.timestamp_ns = now_ns});
    out_ctrl.write(HoldCommand{.timestamp_ns = now_ns});
}

// 从 TargetPrediction + 轨迹构建原料构建 ControlIntent。
// 轨迹构建成功 → TrackCommand；失败 → ShotCommand（带 degradation 标记）。
[[nodiscard]] ControlIntent build_control_intent(
    const TargetPrediction& pred, const AimContext& ctx,
    const ReferenceTrajectoryConfig& ref_traj_cfg, const std::optional<PlannerSeed>& seed,
    const std::optional<L3::ldm::LdmState>& ldm_for_track) noexcept {
    const TrajectoryBuildContext build_ctx{
        .current_ns        = ctx.current_ns,
        .gimbal            = ctx.gimbal,
        .muzzle            = ctx.muzzle,
        .trajectory_solver = &ctx.solver,
        .bullet_speed      = ctx.bullet_speed,
    };

    std::optional<std::string> degradation;

    if (seed.has_value()) {
        auto control_traj = build_reference_trajectory(*seed, ctx.aimer, ref_traj_cfg, build_ctx);
        if (control_traj) {
            auto fire_traj =
                build_fire_reference_trajectory(*seed, ctx.aimer, ref_traj_cfg, build_ctx);
            if (!fire_traj) {
                SPDLOG_WARN("Failed to build fire reference trajectory: {}", fire_traj.error());
            }
            return TrackCommand{
                .timestamp_ns       = pred.timestamp_ns,
                .control_trajectory = std::move(*control_traj),
                .fire_trajectory =
                    fire_traj ? std::move(*fire_traj) : core::trajectory::ReferenceTrajectory{},
            };
        }
        degradation = control_traj.error();
        SPDLOG_WARN("Failed to build reference trajectory: {}", *degradation);
    } else if (ldm_for_track.has_value() && ldm_for_track->is_tracking()) {
        auto control_traj =
            build_ldm_reference_trajectory(*ldm_for_track, ctx.aimer, ref_traj_cfg, build_ctx);
        if (control_traj) {
            auto fire_traj = *control_traj;
            return TrackCommand{
                .timestamp_ns       = pred.timestamp_ns,
                .control_trajectory = std::move(*control_traj),
                .fire_trajectory    = std::move(fire_traj),
            };
        }
        degradation = control_traj.error();
        SPDLOG_WARN("Failed to build LDM reference trajectory: {}", *degradation);
    }

    return ShotCommand{
        .timestamp_ns       = pred.timestamp_ns,
        .yaw                = pred.aim_yaw,
        .pitch              = pred.aim_pitch,
        .distance           = pred.distance,
        .degradation_reason = std::move(degradation),
    };
}

// try_armor — 多目标 tracker → 评分 → 选优 → 弹道
[[nodiscard]] std::optional<AimDecision> try_armor(
    const std::optional<L3::TrackerOutputs>& trackers, const AimContext& ctx,
    const ArmorTargetDecider& decider, ArmorTargetDeciderState& decider_state,
    const AimerConfig& aimer_cfg, const TargetSelectionConfig& sel_cfg,
    const ReferenceTrajectoryConfig& ref_traj_cfg,
    std::unordered_map<core::TargetKey, TargetPhaseState, core::TargetKeyHash>& phase_states) {
    const auto previous_key = decider_state.selected_key;
    auto collection =
        collect_armor_candidates(trackers, ctx, aimer_cfg, sel_cfg, phase_states, previous_key);

    cleanup_stale_phase_states(collection.active_keys, phase_states, decider_state);

    if (collection.candidates.empty()) {
        return std::nullopt;
    }

    std::vector<ArmorTargetDeciderCandidate> decider_candidates;
    decider_candidates.reserve(collection.candidates.size());
    for (const auto& candidate : collection.candidates) {
        decider_candidates.push_back(
            ArmorTargetDeciderCandidate{
                .key     = candidate.key,
                .tracker = candidate.tracker,
                .scores  = candidate.scores,
            });
    }

    const auto decider_result =
        decide_armor_target(decider, decider_state, std::span{decider_candidates});
    if (!decider_result.has_value()) {
        return std::nullopt;
    }

    auto& selected = collection.candidates[decider_result->selected_index];
    auto trace     = build_target_selection_trace(
        ctx.current_ns, previous_key, decider, sel_cfg.switch_margin, *decider_result,
        collection.candidates, std::move(collection.rejected));

    update_selected_phase_state(selected, phase_states);

    auto seed = make_planner_seed(selected.tracker, selected.prediction);

    auto snap = make_snapshot(selected.prediction, GimbalPlanSource::Armor, selected.tracker);

    return AimDecision{
        .intent = build_control_intent(selected.prediction, ctx, ref_traj_cfg, seed, std::nullopt),
        .snapshot = std::move(snap),
        .trace    = std::move(trace),
    };
}

// try_rune — EnergyMeterState → 直接射击（不构建轨迹）
[[nodiscard]] std::optional<AimDecision>
    try_rune(const std::optional<energy_meter::EnergyMeterState>& rune_in, const AimContext& ctx) {
    if (!rune_in || !rune_in->model_valid) {
        return std::nullopt;
    }

    auto pred = ctx.aimer.aim(
        rune_in.value(), ctx.gimbal, ctx.muzzle, ctx.current_ns, ctx.bullet_speed, ctx.solver);
    if (!pred)
        return std::nullopt;

    auto snap = make_snapshot(*pred, GimbalPlanSource::Rune);

    return AimDecision{
        .intent =
            ShotCommand{
                        .timestamp_ns = pred->timestamp_ns,
                        .yaw          = pred->aim_yaw,
                        .pitch        = pred->aim_pitch,
                        .distance     = pred->distance,
                        },
        .snapshot = std::move(snap),
        .trace    = TargetSelectionTrace{.timestamp_ns = ctx.current_ns},
    };
}

// try_ldm — LdmState → 弹道或直接射击
[[nodiscard]] std::optional<AimDecision> try_ldm(
    const std::optional<L3::ldm::LdmState>& ldm_in, const AimContext& ctx,
    const ReferenceTrajectoryConfig& ref_traj_cfg) {
    if (!ldm_in) {
        // Router handles cache lifecycle — don't clear here
        return std::nullopt;
    }

    auto pred = ctx.aimer.aim(
        ldm_in.value(), ctx.gimbal, ctx.muzzle, ctx.current_ns, ctx.bullet_speed, ctx.solver);
    if (!pred)
        return std::nullopt;

    auto snap = make_snapshot(*pred, GimbalPlanSource::Armor);

    return AimDecision{
        .intent   = build_control_intent(*pred, ctx, ref_traj_cfg, std::nullopt, ldm_in),
        .snapshot = std::move(snap),
        .trace    = TargetSelectionTrace{.timestamp_ns = ctx.current_ns},
    };
}

} // namespace

void register_aimer_systems(talos::Scheduler& scheduler, L4Config&& config) {
    auto& world = scheduler.world();
    if (!world.has_resource<L2::ArmorReadbackRoiConfig>()) {
        world.insert_resource(L2::ArmorReadbackRoiConfig{});
    }
    if (!world.has_resource<L2::TrackerReadbackCache>()) {
        world.insert_resource(L2::TrackerReadbackCache{});
    }

    auto unmanned_armor_target_decider = make_armor_target_decider(
        config.aimer.target_selection.decider, config.aimer.target_selection.switch_margin);
    auto manned_armor_target_decider = make_armor_target_decider(
        ArmorTargetDeciderKind::Manned, config.aimer.target_selection.switch_margin);

    world.insert_resource(Aimer(config.aimer));
    world.insert_resource(std::move(config));

    scheduler.add_system<talos::fixed_rate<250>>(
        "l4_aimer",
        [phase_states =
             std::unordered_map<core::TargetKey, TargetPhaseState, core::TargetKeyHash>{},
         unmanned_armor_target_decider = std::move(unmanned_armor_target_decider),
         manned_armor_target_decider   = std::move(manned_armor_target_decider),
         was_following = false, decider_state = ArmorTargetDeciderState{}](
            talos::res<L4Config> cfg, talos::res<Aimer> aimer,
            core::trajectory::trajectory_solver solver,
            core::trajectory::bullet_speed bullet_speed_, core::following following,
            talos::spmc<L3::TrackerOutputs, TrackerOutputChannelTopic> tracker_in,
            talos::spmc<energy_meter::EnergyMeterState, EnergyMeterStateChannelTopic> rune_in,
            talos::spmc<L3::ldm::LdmState> ldm_in, talos::res<fast_tf::CoordinateSystem> tf_buffer,
            talos::spmc_mut<SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic> out_snap,
            talos::spmc_mut<TargetSelectionTrace, TargetSelectionTraceChannelTopic> out_trace,
            talos::spmc_mut<ControlIntent, ControlIntentChannelTopic> out_ctrl) mutable {
            const uint64_t now_ns = fcs::clock::now_ns();
            const auto xf         = lookup_gimbal_muzzle_transforms(*tf_buffer, now_ns);
            if (!xf) [[unlikely]] {
                SPDLOG_WARN("tf lookup failed: {}", xf.error());
                return;
            }
            const auto& [gimbal, muzzle, ts_ns] = *xf;

            const AimContext ctx{
                .gimbal       = gimbal,
                .muzzle       = muzzle,
                .current_ns   = now_ns,
                .bullet_speed = bullet_speed_->bullet_speed,
                .solver       = **solver,
                .aimer        = *aimer,
            };

            const bool is_following = following->load();
            if (is_following && !was_following) {
                // Entering manned mode must relock to the current UV-center-nearest target.
                decider_state.selected_key.reset();
            }
            const ArmorTargetDecider& active_armor_target_decider =
                is_following ? manned_armor_target_decider : unmanned_armor_target_decider;
            was_following = is_following;

            const auto tracker_vec = tracker_in.read_current();
            const auto fallback =
                pick_readback_tracker_fallback(tracker_vec, decider_state.selected_key);

            auto decision = try_armor(
                tracker_vec, ctx, active_armor_target_decider, decider_state, cfg->aimer,
                cfg->aimer.target_selection, cfg->reference_trajectory, phase_states);

            if (!decision) {
                decision = try_rune(rune_in.read_current(), ctx);
                if (decision) {
                    // Rune doesn't use armor-tracker-key tracking — reset to avoid
                    // stale key influencing the next armor-selection cycle.
                    reset_selected_target_key_for_non_armor_source(decider_state);
                }
            }
            if (!decision) {
                auto i = ldm_in.read();
                if (i->status == L3::TrackerStatus::Detecting) {
                    return;
                }
                decision = try_ldm(ldm_in.read(), ctx, cfg->reference_trajectory);
                if (decision) {
                    // LDM doesn't use armor-tracker-key tracking — reset to avoid
                    // stale key influencing the next armor-selection cycle.
                    reset_selected_target_key_for_non_armor_source(decider_state);
                }
            }

            if (decision) {
                out_snap.write(std::move(decision->snapshot));
                out_trace.write(std::move(decision->trace));
                out_ctrl.write(std::move(decision->intent));
            } else {
                write_hold_outputs(out_snap, out_trace, out_ctrl, decider_state, fallback);
            }
        });

    scheduler.add_system<talos::pool_compute>(
        "l4_optimal_target_readback_roi",
        [](talos::spmc<SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic>
               selected_target_in,
           talos::res<L2::ArmorReadbackRoiConfig> readback_roi_config,
           talos::res_mut<L2::TrackerReadbackCache> readback_cache) {
            auto has_readback_tracker = [](const SelectedTargetSnapshot& snapshot) noexcept {
                return snapshot.source == GimbalPlanSource::Armor
                    && tracker_supports_readback_roi(snapshot.tracker);
            };
            if (!selected_target_in.has_new()) {
                return;
            }

            const auto selected_target = selected_target_in.read();
            if (!selected_target) {
                return;
            }

            if (!readback_roi_config->enabled || !has_readback_tracker(*selected_target)) {
                readback_cache->invalidate(selected_target->timestamp_ns);
                return;
            }

            const auto roi_timestamps = resolve_readback_roi_timestamps(*selected_target);
            readback_cache->store(
                L2::TrackerReadbackSnapshot{
                    .valid                   = true,
                    .timestamp_ns            = roi_timestamps.freshness_timestamp_ns,
                    .projection_timestamp_ns = roi_timestamps.projection_timestamp_ns,
                    .selected_armor_id       = selected_target->selected_armor_id,
                    .rough_selected_armor_id = selected_target->rough_selected_armor_id,
                    .tracker                 = selected_target->tracker,
                });
        });
}

} // namespace fcs::L4
