/**
 * @file aimer_systems.cpp
 * @brief L4规划层瞄准系统实现
 *
 * 本文件实现了L4规划层的核心瞄准系统，负责：
 * - 多目标选择和评分
 * - 轨迹构建和控制意图生成
 * - 三种目标源的处理（装甲板、能量机关、LDM）
 *
 * 核心算法：
 * 1. 目标评分：基于图像中心距离、跟踪状态、飞行时间、云台调整量、装甲板名称
 * 2. 目标选择：无人驾驶模式（最优评分）vs 有人驾驶模式（保持当前目标+切换余量）
 * 3. 轨迹构建：尝试构建参考轨迹，失败则降级为直接射击
 * 4. ROI管理：为L2感知层提供回读ROI信息
 *
 * 设计理念：
 * - 使用函数式风格，分离数据收集、评分、决策
 * - 使用variant表达决策结果，避免运行时类型检查
 * - 诊断数据与控制数据分离，支持可视化而不影响执行
 *
 * 性能考虑：
 * - 系统运行频率250Hz，每帧约4ms
 * - 目标评分是性能热点，需要优化
 * - 弹道求解是计算密集型，使用缓存加速
 *
 * 线程安全：系统内部状态通过lambda捕获，保证线程安全
 */

#include "L4_planning/aimer/aimer_systems.hpp"
#include "L3_estimation/ldm_naive/types.hpp"          // LDM吊射滤波结果结构体
#include "L4_planning/aimer/aimer.hpp"                 // 瞄准核心类Aimer，负责弹道解算
#include "L4_planning/aimer/armor_target_decider.hpp"  // 装甲目标决策器(有人/无人模式)
#include "L4_planning/aimer/fsm.hpp"                   // 瞄准阶段状态机

#include "L2_perception/armor/readback_roi.hpp"        // L2感知ROI回读配置、缓存
#include "L3_estimation/energy_meter/types.hpp"         // 能量机关L3输出结构体
#include "L3_estimation/tracker/types.hpp"              // 装甲跟踪器输出
#include "L4_planning/aimer/types.hpp"
#include "L4_planning/common/transform_utils.hpp"       // TF坐标变换工具
#include "L4_planning/config.hpp"                      // L4全局配置
#include "L4_planning/control_intent.hpp"              // 控制指令：TrackCommand/ShotCommand/HoldCommand
#include "L4_planning/plan_source.hpp"                  // 目标来源枚举(装甲/符/LDM)
#include "L4_planning/selected_target_snapshot.hpp"    // 选中目标快照
#include "L4_planning/target_selection_trace.hpp"       // 目标选择诊断日志
#include "L4_planning/trajectory_builder.hpp"           // 轨迹生成器
#include "camera_config.hpp"
#include "core/channel_topics.hpp"                     // 总线Topic定义
#include "core/runtime.hpp"
#include "core/target_key.hpp"                         // 目标唯一标识(颜色+装甲ID)
#include "core/time.hpp"                               // 高精度时钟
#include "core/trajectory/resource.hpp"                // 弹道求解器全局资源
#include "euler.hpp"
#include "scheduler/scheduler.hpp"                     // Talos调度器

#include <algorithm>
#include <cmath>
#include <expected>           // C++23 错误处理，替代异常
#include <frame.hpp>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <spdlog/spdlog.h>
#include <unordered_map>
#include <vector>

namespace fcs::L4 {

namespace { // 匿名命名空间，仅本编译单元可见

/**
 * @brief 目标相位状态
 *
 * 跟踪每个目标的瞄准阶段状态，用于实现WholeCarCenter到WholeCarArmor的平滑过渡。
 * 瞄准阶段切换逻辑：先瞄准整车中心防止丢失，稳定后切换到单独装甲板击打
 */
struct TargetPhaseState {
    ArmorAimPhase phase{ArmorAimPhase::SingleArmor};  ///< 当前瞄准阶段：整车中心/单装甲
    int overflow_count{0};                            ///< 阶段切换计数，平滑过渡防抖
    std::optional<int> jumped_selected_armor_id{};    ///< 目标跳变后记忆选中的装甲ID，防跳变乱切装甲
};

/**
 * @brief 候选目标评估结果
 * 单个候选装甲目标全套评估数据，用于打分择优
 */
struct CandidateEvaluation {
    core::TargetKey key{};                            ///< 目标唯一键(敌方颜色+装甲编号)
    L3::TrackerOutput tracker{};                      ///< L3卡尔曼跟踪输出
    TargetPrediction prediction{};                    ///< 弹道解算后的瞄准预测结果
    TargetSelectionScores scores{};                   ///< 五项加权打分分数
    TargetSelectionTraceEntry trace_entry{};          ///< 诊断追踪记录，上位机可视化查看打分详情
};

/**
 * @brief 云台调整量度量
 * 计算云台从当前姿态转到瞄准姿态需要转动多少角度，用于打分(转动幅度越小分数越高)
 */
struct GimbalEffortMetrics {
    double yaw_delta_rad{std::numeric_limits<double>::infinity()};   ///< yaw需要转动弧度
    double pitch_delta_rad{std::numeric_limits<double>::infinity()}; ///< pitch需要转动弧度
    double score{0.0};                                               ///< 云台代价得分[0,1]，转动越小得分越高
};

/**
 * @brief 回读ROI时间戳信息
 * 下发ROI给L2感知，限制相机只在目标区域做识别，缩小检测范围提速、抗噪
 */
struct ReadbackRoiTimestamps {
    uint64_t projection_timestamp_ns{0};              ///< 轨迹预测对应的时间戳
    uint64_t freshness_timestamp_ns{0};               ///< 目标最后观测时间戳，判断目标新鲜度
};

/**
 * @brief 装甲板候选集合
 * 一轮目标筛选的全部装甲候选、被淘汰目标，用于诊断
 */
struct ArmorCandidateCollection {
    std::vector<core::TargetKey> active_keys{};       ///< 当前存活的所有目标Key
    std::vector<CandidateEvaluation> candidates{};    ///< 有效可击打候选装甲
    std::vector<TargetSelectionTraceEntry> rejected{}; ///< 被剔除的目标(无跟踪/弹道求解失败)，用于日志排查
};

// 工具函数：数值钳位至 [0,1]
[[nodiscard]] double clamp01(double value) noexcept { return std::clamp(value, 0.0, 1.0); }

// 计算两个时间戳间隔，转为秒
[[nodiscard]] double elapsed_seconds(uint64_t start_ns, uint64_t end_ns) noexcept {
    if (end_ns <= start_ns) {
        return 0.0;
    }
    return static_cast<double>(end_ns - start_ns) * 1e-9;
}

// 目标视觉滞留时长：多久没有收到图像观测，越大代表目标越久没被看见
[[nodiscard]] double
optical_age_seconds(const L3::TrackerOutput& tracker, uint64_t current_ns) noexcept {
    if (tracker.last_observation_timestamp_ns == 0) {
        return 0.0;
    }
    return elapsed_seconds(tracker.last_observation_timestamp_ns, current_ns);
}

// 根据跟踪器构造目标唯一标识Key
[[nodiscard]] core::TargetKey make_target_key(const L3::TrackerOutput& tracker) noexcept {
    return core::TargetKey{tracker.target_name, tracker.target_color};
}

// 判断该目标是否支持下发ROI给感知层
[[nodiscard]] bool tracker_supports_readback_roi(const L3::TrackerOutput& tracker) noexcept {
    return tracker.is_tracking() && tracker.target_name != ArmorName::Invalid
        && (tracker.is_robot() || tracker.is_outpost());
}

/**
 * @brief 挑选最优目标用于ROI下发兜底
 * 优先选中当前锁定目标，若无则挑选画面最中心、最新观测的敌方车辆
 */
[[nodiscard]] std::optional<L3::TrackerOutput> pick_readback_tracker_fallback(
    const std::optional<L3::TrackerOutputs>& trackers,
    const std::optional<core::TargetKey>& preferred_key) noexcept {
    if (!trackers) {
        return std::nullopt;
    }

    // 优先使用当前锁定目标
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

        // 优先有有效画面距离的目标
        const bool tracker_has_finite = std::isfinite(tracker.last_image_center_distance_px);
        const bool best_has_finite    = std::isfinite(best->last_image_center_distance_px);
        if (tracker_has_finite != best_has_finite) {
            if (tracker_has_finite) {
                best = &tracker;
            }
            continue;
        }

        // 画面越靠近图像中心优先级越高
        if (tracker_has_finite
            && tracker.last_image_center_distance_px < best->last_image_center_distance_px) {
            best = &tracker;
            continue;
        }

        // 其次选择观测时间最新的目标
        if (tracker.last_observation_timestamp_ns > best->last_observation_timestamp_ns) {
            best = &tracker;
        }
    }

    if (best == nullptr) {
        return std::nullopt;
    }
    return *best;
}

// 解析选中目标快照的两个时间戳，用于ROI缓存
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

// 获取敌方车辆/基地中心世界坐标
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

// 判断当前候选是否是上一轮选中的目标
[[nodiscard]] bool contains_target_key(
    const std::vector<core::TargetKey>& keys, const core::TargetKey& key) noexcept {
    return std::find(keys.begin(), keys.end(), key) != keys.end();
}

// 获取敌方车辆自转yaw角速度绝对值，用于阶段切换
[[nodiscard]] double target_abs_v_yaw(const L3::TrackerOutput& tracker) noexcept {
    if (const auto* robot_state = tracker.robot_state()) {
        return std::abs(robot_state->v_yaw);
    }
    if (const auto* outpost_state = tracker.outpost_state()) {
        return std::abs(outpost_state->v_yaw);
    }
    return 0.0;
}

/**
 * @brief 对单个跟踪目标执行弹道瞄准解算
 * 输入车辆状态，输出带提前量的瞄准yaw/pitch
 */
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

// 打分项1：图像中心分数，目标越靠近画面中心得分越高；临时丢失时随时间衰减分数
[[nodiscard]] double score_image_center(
    const TargetSelectionConfig& cfg, const L3::TrackerOutput& tracker,
    uint64_t current_ns) noexcept {
    if (!std::isfinite(tracker.last_image_center_distance_px)
        || tracker.last_observation_timestamp_ns == 0 || cfg.refs.image_center_ref_px <= 1e-6) {
        return 0.0;
    }

    double score = 1.0 - tracker.last_image_center_distance_px / cfg.refs.image_center_ref_px;
    score        = clamp01(score);

    // 临时丢失目标，分数随观测间隔衰减，丢太久直接失去候选资格
    if (tracker.status == L3::TrackerStatus::TempLost) {
        if (cfg.optical_stale_timeout_s <= 1e-6) {
            return 0.0;
        }
        const double age = elapsed_seconds(tracker.last_observation_timestamp_ns, current_ns);
        score *= clamp01(1.0 - age / cfg.optical_stale_timeout_s);
    }

    return score;
}

// 打分项2：跟踪状态分数，正常跟踪=1分，临时丢失使用配置低分
[[nodiscard]] double
score_track_state(const TargetSelectionConfig& cfg, const L3::TrackerOutput& tracker) noexcept {
    return tracker.status == L3::TrackerStatus::Tracking ? 1.0 : cfg.temp_lost_state_score;
}

// 打分项3：子弹飞行时间分数，距离越近飞行时间越短，得分越高
[[nodiscard]] double
score_tof(const TargetSelectionConfig& cfg, const TargetPrediction& prediction) noexcept {
    if (!std::isfinite(prediction.flying_time) || cfg.refs.tof_ref_s <= 1e-6) {
        return 0.0;
    }
    return clamp01(1.0 - prediction.flying_time / cfg.refs.tof_ref_s);
}

// 计算云台转动代价并换算得分
[[nodiscard]] GimbalEffortMetrics compute_gimbal_effort_metrics(
    const TargetSelectionConfig& cfg, double current_yaw, double current_pitch,
    const TargetPrediction& prediction) noexcept {
    GimbalEffortMetrics metrics;
    const double yaw_ref   = cfg.refs.yaw_effort_ref_deg * std::numbers::pi / 180.0;
    const double pitch_ref = cfg.refs.pitch_effort_ref_deg * std::numbers::pi / 180.0;
    if (yaw_ref <= 1e-6 || pitch_ref <= 1e-6) {
        return metrics;
    }

    // 计算yaw最短转向角度(处理±180°环绕)
    metrics.yaw_delta_rad =
        std::abs(std::remainder(prediction.aim_yaw - current_yaw, 2.0 * std::numbers::pi));
    const double command_pitch_in_tf = -prediction.aim_pitch;
    metrics.pitch_delta_rad          = std::abs(command_pitch_in_tf - current_pitch);
    const double effort =
        0.5 * (metrics.yaw_delta_rad / yaw_ref + metrics.pitch_delta_rad / pitch_ref);
    metrics.score = clamp01(1.0 - effort);
    return metrics;
}

/**
 * @brief 完整加权打分函数
 * 5个维度加权求和得到目标总分，总分越高越优先击打
 * 维度：画面居中程度、跟踪状态、子弹飞行时长、云台转动代价、装甲类型优先级
 */
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

    // 加权求和
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

// 生成选中目标诊断日志条目，用于上位机回放目标选择全过程
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

    // 各分项分数
    entry.image_center_score  = scores.image_center;
    entry.track_state_score   = scores.track_state;
    entry.tof_score           = scores.tof;
    entry.gimbal_effort_score = scores.gimbal_effort;
    entry.armor_name_score    = scores.armor_name;

    // 加权分项
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

// 生成被淘汰目标的诊断日志，记录淘汰原因
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

// 组装完整一轮目标选择诊断轨迹数据包
[[nodiscard]] TargetSelectionTrace build_target_selection_trace(
    uint64_t current_ns, const std::optional<core::TargetKey>& previous_key,
    const ArmorTargetDecider& decider, double switch_margin,
    const ArmorTargetDeciderResult& decider_result,
    const std::vector<CandidateEvaluation>& candidates,
    std::vector<TargetSelectionTraceEntry> rejected) {
    TargetSelectionTrace trace{
        .timestamp_ns        = current_ns,
        .kept_current_target = decider_result.kept_current_target,
        // 只有自动驾驶模式才有切换阈值余量，手控模式切换余量为0
        .switch_margin =
            std::holds_alternative<UnmannedArmorTargetDecider>(decider) ? switch_margin : 0.0,
    };

    if (previous_key.has_value()) {
        trace.had_previous_target   = true;
        trace.previous_target_name  = previous_key->name;
        trace.previous_target_color = previous_key->color;
    }

    trace.candidates.reserve(candidates.size() + rejected.size());
    // 写入排名靠前的候选目标
    for (size_t rank = 0; rank < decider_result.ranked_indices.size(); ++rank) {
        const size_t candidate_index = decider_result.ranked_indices[rank];
        auto candidate_trace         = candidates[candidate_index].trace_entry;
        candidate_trace.rank         = static_cast<int>(rank + 1);
        candidate_trace.selected     = candidate_index == decider_result.selected_index;
        candidate_trace.runner_up =
            decider_result.runner_up.has_value() && candidate_index == *decider_result.runner_up;
        trace.candidates.push_back(std::move(candidate_trace));
    }

    // 写入被淘汰目标
    for (auto& rejected_entry : rejected) {
        rejected_entry.rank = static_cast<int>(trace.candidates.size() + 1);
        trace.candidates.push_back(std::move(rejected_entry));
    }

    return trace;
}

// 统一决策输出结构：最终控制指令+选中目标快照+诊断日志
struct AimDecision {
    ControlIntent intent;
    SelectedTargetSnapshot snapshot;
    TargetSelectionTrace trace;
};

// 瞄准全局只读上下文，避免重复传参
struct AimContext {
    const Aimer::GimbalTransform& gimbal;
    const Aimer::MuzzleTransform& muzzle;
    uint64_t current_ns;
    double bullet_speed;
    const core::trajectory::solver::TrajectorySolver& solver;
    const Aimer& aimer;
};

// 根据预测结果生成选中目标快照，下发给其他模块
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

// 构造轨迹规划种子，用于生成车辆运动参考轨迹
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

// 构造装甲瞄准阶段上下文，用于整车/单装甲平滑切换
[[nodiscard]] ArmorAimContext build_armor_aim_context(
    const L3::TrackerOutput& tracker, const TargetPhaseState& phase_state) noexcept {
    ArmorAimContext aim_ctx;
    aim_ctx.target_jumped      = tracker.target_jumped;
    aim_ctx.phase              = phase_state.phase;
    aim_ctx.preferred_armor_id = tracker.last_armor_id;
    // 目标跳变时沿用记忆的装甲ID，防止瞬间乱切装甲
    if (tracker.target_jumped && phase_state.jumped_selected_armor_id.has_value()) {
        aim_ctx.preferred_armor_id = phase_state.jumped_selected_armor_id;
    }
    return aim_ctx;
}

/**
 * @brief 收集所有装甲候选目标
 * 遍历所有L3跟踪结果，过滤有效目标、逐个打分、记录淘汰目标
 */
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

    // 获取当前云台yaw/pitch
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
        // 更新瞄准阶段(整车中心/单装甲)
        advance_armor_aim_phase(
            aimer_cfg, target_abs_v_yaw(tracker), tracker.target_jumped, phase_state.phase,
            phase_state.overflow_count);

        // 弹道解算
        auto prediction = aim_tracker_target(
            ctx.aimer, tracker, build_armor_aim_context(tracker, phase_state), ctx.gimbal,
            ctx.muzzle, ctx.current_ns, ctx.bullet_speed, ctx.solver);
        if (!prediction) {
            // 弹道求解失败，加入淘汰列表
            result.rejected.push_back(
                build_rejected_trace_entry(tracker, ctx.current_ns, prediction.error(), was_prev));
            continue;
        }

        // 打分封装
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

// 清理已经消失的目标的阶段状态，防止哈希表膨胀
auto cleanup_stale_phase_states(
    const std::vector<core::TargetKey>& active_keys,
    std::unordered_map<core::TargetKey, TargetPhaseState, core::TargetKeyHash>& phase_states,
    ArmorTargetDeciderState& decider_state) -> void {
    if (active_keys.empty()) {
        phase_states.clear();
        decider_state.selected_key.reset();
        return;
    }

    // 删除不在活跃目标列表内的状态
    std::erase_if(
        phase_states, [&](const auto& kv) { return !contains_target_key(active_keys, kv.first); });
}

// 更新选中目标的阶段状态，记忆跳变时锁定的装甲ID
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

// 击打能量机关/LDM时，清空装甲选中标记，避免切回装甲时状态错乱
auto reset_selected_target_key_for_non_armor_source(ArmorTargetDeciderState& decider_state)
    -> void {
    decider_state.selected_key.reset();
}

// 无任何目标可击打时，输出HoldCommand保持云台不动，并兜底下发ROI
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

/**
 * @brief 根据预测结果生成最终控制指令
 * 优先构建车辆运动参考轨迹(TrackCommand，云台平滑跟随移动目标)
 * 轨迹构建失败降级为直接单发瞄准ShotCommand
 */
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

    // 装甲目标：构建车辆运动参考轨迹实现平滑跟随
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
        // LDM吊射目标构建轨迹
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

    // 轨迹构建失败，降级为直接射击指令
    return ShotCommand{
        .timestamp_ns       = pred.timestamp_ns,
        .yaw                = pred.aim_yaw,
        .pitch              = pred.aim_pitch,
        .distance           = pred.distance,
        .degradation_reason = std::move(degradation),
    };
}

/**
 * @brief 装甲目标处理主逻辑
 * 收集候选 → 决策器择优 → 生成控制指令
 */
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

    // 组装候选送入决策器
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

/**
 * @brief 能量机关目标分支
 * 能量机关无运动轨迹规划，直接输出ShotCommand瞬时瞄准
 */
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

/**
 * @brief LDM吊射目标分支
 * 可构建吊射轨迹平滑击打，失败则直接吊射瞄准
 */
[[nodiscard]] std::optional<AimDecision> try_ldm(
    const std::optional<L3::ldm::LdmState>& ldm_in, const AimContext& ctx,
    const ReferenceTrajectoryConfig& ref_traj_cfg) {
    if (!ldm_in) {
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

/**
 * @brief 向Talos调度器注册L4瞄准系统
 * 1. 注册全局配置资源
 * 2. 创建自动驾驶/手控两套目标决策器
 * 3. 注册250Hz主瞄准系统
 * 4. 注册异步ROI下发系统
 */
void register_aimer_systems(talos::Scheduler& scheduler, L4Config&& config) {
    auto& world = scheduler.world();
    // 注入ROI配置与缓存全局资源
    if (!world.has_resource<L2::ArmorReadbackRoiConfig>()) {
        world.insert_resource(L2::ArmorReadbackRoiConfig{});
    }
    if (!world.has_resource<L2::TrackerReadbackCache>()) {
        world.insert_resource(L2::TrackerReadbackCache{});
    }

    // 构造两种模式决策器
    auto unmanned_armor_target_decider = make_armor_target_decider(
        config.aimer.target_selection.decider, config.aimer.target_selection.switch_margin);
    auto manned_armor_target_decider = make_armor_target_decider(
        ArmorTargetDeciderKind::Manned, config.aimer.target_selection.switch_margin);

    world.insert_resource(Aimer(config.aimer));
    world.insert_resource(std::move(config));

    // 注册250Hz定频瞄准主系统
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
            // 查询当前云台、枪口坐标系变换
            const auto xf         = lookup_gimbal_muzzle_transforms(*tf_buffer, now_ns);
            if (!xf) [[unlikely]] {
                SPDLOG_WARN("tf lookup failed: {}", xf.error());
                return;
            }
            const auto& [gimbal, muzzle, ts_ns] = *xf;

            // 封装全局上下文
            const AimContext ctx{
                .gimbal       = gimbal,
                .muzzle       = muzzle,
                .current_ns   = now_ns,
                .bullet_speed = bullet_speed_->bullet_speed,
                .solver       = **solver,
                .aimer        = *aimer,
            };

            // 判断是否进入手控跟随模式
            const bool is_following = following->load();
            // 刚切入手控模式，清空选中目标，强制重新锁定画面中心目标
            if (is_following && !was_following) {
                decider_state.selected_key.reset();
            }
            // 切换当前生效的决策器
            const ArmorTargetDecider& active_armor_target_decider =
                is_following ? manned_armor_target_decider : unmanned_armor_target_decider;
            was_following = is_following;

            const auto tracker_vec = tracker_in.read_current();
            // 获取兜底目标用于ROI下发
            const auto fallback =
                pick_readback_tracker_fallback(tracker_vec, decider_state.selected_key);

            // 优先级1：尝试击打装甲板
            auto decision = try_armor(
                tracker_vec, ctx, active_armor_target_decider, decider_state, cfg->aimer,
                cfg->aimer.target_selection, cfg->reference_trajectory, phase_states);

            // 优先级2：无装甲可打，尝试击打能量机关
            if (!decision) {
                decision = try_rune(rune_in.read_current(), ctx);
                if (decision) {
                    reset_selected_target_key_for_non_armor_source(decider_state);
                }
            }
            // 优先级3：无装甲无能量机关，尝试LDM吊射
            if (!decision) {
                auto i = ldm_in.read();
                if (i->status == L3::TrackerStatus::Detecting) {
                    return;
                }
                decision = try_ldm(ldm_in.read(), ctx, cfg->reference_trajectory);
                if (decision) {
                    reset_selected_target_key_for_non_armor_source(decider_state);
                }
            }

            // 输出控制指令
            if (decision) {
                out_snap.write(std::move(decision->snapshot));
                out_trace.write(std::move(decision->trace));
                out_ctrl.write(std::move(decision->intent));
            } else {
                // 无目标，云台保持不动
                write_hold_outputs(out_snap, out_trace, out_ctrl, decider_state, fallback);
            }
        });

    // 额外异步线程池系统：下发ROI给L2感知，缩小识别区域提升性能
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

            // ROI功能关闭/目标不合法，清空缓存
            if (!readback_roi_config->enabled || !has_readback_tracker(*selected_target)) {
                readback_cache->invalidate(selected_target->timestamp_ns);
                return;
            }

            const auto roi_timestamps = resolve_readback_roi_timestamps(*selected_target);
            // 将目标ROI信息写入全局缓存，L2感知读取后裁剪图像
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

} // namespace fcs::L4_planning::aimer