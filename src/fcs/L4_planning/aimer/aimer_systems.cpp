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
 * @brief ROI回退选择器：从多个跟踪器结果里挑选一个适合做ROI裁剪的跟踪器输出
 * [[nodiscard]] 调用者必须接收返回值，禁止忽略返回结果，防止逻辑bug
 * @param trackers 所有跟踪器输出集合，std::optional，可能为空
 * @param preferred_key 优先锁定的目标key（遥控器锁定/上一轮选中的目标），std::optional可以无值
 * @return std::optional<L3::TrackerOutput> 选出来的跟踪器；std::nullopt代表没有可用跟踪器
 * @noexcept 不会抛出异常
 *
 * 业务场景：装甲检测开启ROI，不需要整张图推理；从跟踪器挑一个目标，取它的区域作为ROI输入检测器。
 * 逻辑优先级：①优先用户锁定目标 → ②有有效像素距离 > ③离图像中心更近 > ④观测时间最新
 */
[[nodiscard]] std::optional<L3::TrackerOutput> pick_readback_tracker_fallback(
    const std::optional<L3::TrackerOutputs>& trackers,
    const std::optional<core::TargetKey>& preferred_key) noexcept {

    // 如果跟踪器集合本身无值，直接返回空，没有任何跟踪结果
    if (!trackers) {
        return std::nullopt;
    }

    // --------------------------第一优先级：优先使用遥控器/上一轮锁定的目标--------------------------
    if (preferred_key.has_value()) // 存在优先锁定的目标key
    {
        // 遍历全部跟踪器
        for (const auto& tracker : *trackers)
        {
            // tracker_supports_readback_roi(tracker)：判断该跟踪器是否支持ROI回读（有些跟踪器不能做ROI）
            // make_target_key(tracker)：从跟踪器生成唯一目标ID，和preferred_key比对
            if (tracker_supports_readback_roi(tracker)
                && make_target_key(tracker) == *preferred_key)
            {
                // 找到优先锁定目标，直接返回这个跟踪器，不再往下遍历
                return tracker;
            }
        }
    }
    // 走到这里：没有优先目标，或者优先目标已经丢失/不支持ROI，进入兜底自动挑选逻辑

    const L3::TrackerOutput* best = nullptr; // 记录当前最优跟踪器的指针，初始为空

    // 遍历全部跟踪结果，筛选可用ROI跟踪器，打分选出best
    for (const auto& tracker : *trackers)
    {
        // 当前跟踪器不支持ROI裁剪，直接跳过
        if (!tracker_supports_readback_roi(tracker)) {
            continue;
        }

        // best是空，说明第一个可用跟踪器，直接赋值为best
        if (best == nullptr) {
            best = &tracker;
            continue;
        }

        // --------------------------第二优先级：优先拥有有效的图像中心距离（非NaN/无穷）--------------------------
        // std::isfinite：判断浮点数不是NaN、不是正负无穷，代表距离数值有效
        const bool tracker_has_finite = std::isfinite(tracker.last_image_center_distance_px);
        const bool best_has_finite    = std::isfinite(best->last_image_center_distance_px);

        // 两个状态不一致：一个有效、一个无效
        if (tracker_has_finite != best_has_finite)
        {
            if (tracker_has_finite)
            {
                // 当前tracker数值有效，旧best无效，替换best
                best = &tracker;
            }
            continue; // 结束本轮循环，进入下一个tracker
        }

        // --------------------------第三优先级：画面上离图像中心越近优先级越高--------------------------
        // last_image_center_distance_px：目标框距离画面中心点像素距离，越小越靠近画面中间
        if (tracker_has_finite
            && tracker.last_image_center_distance_px < best->last_image_center_distance_px)
        {
            best = &tracker;
            continue;
        }

        // --------------------------第四优先级：观测时间戳更新的优先--------------------------
        // last_observation_timestamp_ns：这个跟踪器最后一次观测到目标的纳秒时间戳，越大越新
        if (tracker.last_observation_timestamp_ns > best->last_observation_timestamp_ns)
        {
            best = &tracker;
        }
    }

    // 循环结束，best仍然为空：没有任何可用跟踪器
    if (best == nullptr) {
        return std::nullopt;
    }

    // 解引用指针，拷贝构造返回跟踪器输出
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
 * [[nodiscard]]：强制调用方处理返回值，禁止忽略结果，防止漏判空值引发UB
 *
 * @param trackers L3层所有跟踪器输出集合，std::optional，可能为空
 * @param ctx 瞄准上下文：时间戳、云台状态、机器人状态等运行时信息
 * @param decider 目标选择决策器（std::variant，自动/手控二选一）
 * @param decider_state 决策器状态：保存上一轮选中的target key、切换状态，会被函数修改（入参输出）
 * @param aimer_cfg 瞄准模块总配置yaml
 * @param sel_cfg 目标选择相关配置，切换裕度、过滤阈值等
 * @param ref_traj_cfg 参考弹道配置：子弹初速、重力、弹道补偿参数
 * @param phase_states 每个目标的相位状态map；key=目标唯一ID，value=目标时序相位；会被本函数修改
 * @return std::optional<AimDecision> 瞄准决策结果；nullopt代表没有可用装甲目标，不输出瞄准指令
 */
[[nodiscard]] std::optional<AimDecision> try_armor(
    const std::optional<L3::TrackerOutputs>& trackers, const AimContext& ctx,
    const ArmorTargetDecider& decider, ArmorTargetDeciderState& decider_state,
    const AimerConfig& aimer_cfg, const TargetSelectionConfig& sel_cfg,
    const ReferenceTrajectoryConfig& ref_traj_cfg,
    std::unordered_map<core::TargetKey, TargetPhaseState, core::TargetKeyHash>& phase_states) {

    // 拿到上一轮已经选中的目标ID，用于候选收集、防频繁切换
    const auto previous_key = decider_state.selected_key;

    // ==========步骤1：收集装甲候选目标==========
    // 输入跟踪器结果，过滤、打分，输出一批合法候选装甲；同时更新部分相位状态
    auto collection =
        collect_armor_candidates(trackers, ctx, aimer_cfg, sel_cfg, phase_states, previous_key);

    // ==========步骤2：清理过期目标相位状态==========
    // active_keys：当前帧有效的目标集合；把不在active_keys里面的旧目标从phase_states删掉，防止map无限膨胀内存泄漏
    cleanup_stale_phase_states(collection.active_keys, phase_states, decider_state);

    // 没有任何候选装甲，直接返回空，本帧不做瞄准
    if (collection.candidates.empty()) {
        return std::nullopt;
    }

    // ==========步骤3：转换数据格式，适配决策器输入==========
    // 把收集出来的候选，包装成决策器需要的 ArmorTargetDeciderCandidate 结构体
    std::vector<ArmorTargetDeciderCandidate> decider_candidates;
    // 预分配内存，避免vector多次realloc扩容
    decider_candidates.reserve(collection.candidates.size());

    for (const auto& candidate : collection.candidates) {
        decider_candidates.push_back(
            ArmorTargetDeciderCandidate{
                .key     = candidate.key,        // 目标唯一ID
                .tracker = candidate.tracker,    // 跟踪器输出位姿、观测信息
                .scores  = candidate.scores,     // 各项打分（距离、角度、置信度等）
            });
    }

    // ==========步骤4：调用目标选择决策器选出最优目标==========
    // std::span：视图，不拷贝vector数据，只读传给决策器；decider_state会被修改，记录切换状态
    const auto decider_result =
        decide_armor_target(decider, decider_state, std::span{decider_candidates});

    // 决策器选不出目标，返回空，本帧放弃瞄准
    if (!decider_result.has_value()) {
        return std::nullopt;
    }

    // 根据决策返回的下标，取出被选中的候选目标
    auto& selected = collection.candidates[decider_result->selected_index];

    // ==========步骤5：构建目标选择追踪trace，用于Foxglove调试可视化、日志回放==========
    // trace记录：时间戳、上一轮目标、决策器类型、切换阈值、选中结果、全部候选、被拒绝的候选列表
    auto trace     = build_target_selection_trace(
        ctx.current_ns, previous_key, decider, sel_cfg.switch_margin, *decider_result,
        collection.candidates, std::move(collection.rejected));

    // ==========步骤6：更新被选中目标的相位状态（反弹/旋转时序）==========
    // phase_states保存目标旋转相位，用于装甲预测；修改map中对应key的状态
    update_selected_phase_state(selected, phase_states);

    // ==========步骤7：生成规划器seed初始值==========
    // seed：给弹道/云台规划器的初始猜测值，用上一帧预测结果加速求解MPC
    auto seed = make_planner_seed(selected.tracker, selected.prediction);

    // ==========步骤8：生成目标快照Snapshot==========
    // 保存当前目标全部状态：位姿、预测、来源标记（GimbalPlanSource::Armor标记来源是装甲模块），供外部录制、可视化
    auto snap = make_snapshot(selected.prediction, GimbalPlanSource::Armor, selected.tracker);

    // ==========步骤9：组装最终瞄准决策返回==========
    // intent：控制意图，送给武器/云台系统；snap状态快照；trace调试追踪信息
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
 * @brief 能量机关（LDM）瞄准分支主逻辑
 * [[nodiscard]] 必须接收返回值，禁止忽略，防止漏判空
 *
 * @param ldm_in L3层能量机关检测跟踪状态，std::optional，无值代表没有检测到能量机关
 * @param ctx 瞄准上下文：云台状态、炮口、时间戳、弹速、求解器等
 * @param ref_traj_cfg 弹道参考配置，重力、弹道补偿参数
 * @return std::optional<AimDecision> 瞄准决策；nullopt：无能量机关，不输出瞄准指令
 */
[[nodiscard]] std::optional<AimDecision> try_ldm(
    const std::optional<L3::ldm::LdmState>& ldm_in, const AimContext& ctx,
    const ReferenceTrajectoryConfig& ref_traj_cfg) {

    // 没有能量机关输入，直接返回空，本帧不处理能量机关瞄准
    if (!ldm_in) {
        return std::nullopt;
    }

    // 调用aimer.aim做能量机关预测瞄准
    // 输入：能量机关状态、云台当前姿态、炮口位姿、当前时间、子弹速度、弹道求解器
    // 返回std::optional，失败代表预测求解失败
    auto pred = ctx.aimer.aim(
        ldm_in.value(), ctx.gimbal, ctx.muzzle, ctx.current_ns, ctx.bullet_speed, ctx.solver);

    // 预测求解失败，返回空，放弃本帧能量机关瞄准
    if (!pred)
        return std::nullopt;

    // 生成目标状态快照，标记来源为Armor（能量机关复用这套快照结构），用于录制、Foxglove可视化
    auto snap = make_snapshot(*pred, GimbalPlanSource::Armor);

    // 组装瞄准决策返回
    return AimDecision{
        // 生成云台控制意图；seed传std::nullopt：能量机关不使用MPC热启动种子
        // 最后传入ldm_in，记录原始能量机关输入给intent
        .intent   = build_control_intent(*pred, ctx, ref_traj_cfg, std::nullopt, ldm_in),
        .snapshot = std::move(snap),
        // trace只填时间戳；能量机关没有多候选目标选择逻辑，没有选目标trace信息
        .trace    = TargetSelectionTrace{.timestamp_ns = ctx.current_ns},
    };
}


} // namespace

/**
 * @brief 注册L4瞄准系统整套系统到talos调度器
 * @param scheduler talos调度器实例，管理所有系统、资源、通道
 * @param config L4层瞄准配置，配置所有权会move进入world资源
 *
 * 整套瞄准系统拆成两套system：
 * 1. l4_aimer：**250Hz定频主瞄准逻辑**，做目标选择、弹道求解、输出云台控制指令
 * 2. l4_optimal_target_readback_roi：线程池异步任务，计算ROI区域下发给L2感知，缩小检测范围加速推理
 *
 * 数据流：
 * L3 TrackerOutputs(SPMC通道) → L4瞄准系统 → ControlIntent控制意图(SPMC) → L1层输出给云台MCU
 *         └── 输出SelectedTargetSnapshot快照、Trace调试数据给Foxglove可视化
 */
void register_aimer_systems(talos::Scheduler& scheduler, L4Config&& config) {
    auto& world = scheduler.world();

    // ========== 注入全局资源：ROI配置、跟踪器缓存 ==========
    // 如果全局资源还不存在，则插入资源；防止多次调用register重复插入报错
    if (!world.has_resource<L2::ArmorReadbackRoiConfig>()) {
        world.insert_resource(L2::ArmorReadbackRoiConfig{});
    }
    if (!world.has_resource<L2::TrackerReadbackCache>()) {
        world.insert_resource(L2::TrackerReadbackCache{});
    }

    // ========== 构造两套目标决策器 ==========
    // 自动模式决策器：自动选装甲、切换目标
    auto unmanned_armor_target_decider = make_armor_target_decider(
        config.aimer.target_selection.decider, config.aimer.target_selection.switch_margin);
    // 手控模式决策器：手控跟随模式，优先画面中心目标
    auto manned_armor_target_decider = make_armor_target_decider(
        ArmorTargetDeciderKind::Manned, config.aimer.target_selection.switch_margin);

    // 将瞄准配置、L4总配置移动到ECS全局资源，供system参数res<>读取
    world.insert_resource(Aimer(config.aimer));
    world.insert_resource(std::move(config));

    // =================================================================================
    // 【核心主系统 l4_aimer】250Hz定频运行，瞄准主循环
    // talos::fixed_rate<250>：调度器约束，强制250Hz执行，周期4ms
    // lambda捕获列表：[]里面的变量是**系统本地状态**，每次调用不会销毁，持久保存
    //      std::unordered_map<core::TargetKey, TargetPhaseState, core::TargetKeyHash> phase_states
    //          → 每个目标对应的弹道相位状态，多目标状态持久保存
    //      unmanned_armor_target_decider / manned_armor_target_decider：两种决策器实例
    //      was_following：上一帧是否手控跟随，做状态跳变检测
    //      decider_state：目标选择器内部状态，保存当前选中目标key
    //
    // 系统参数（talos注入）：
    // talos::res<T>        → 只读全局资源
    // talos::res_mut<T>    → 可写全局资源
    // talos::spmc<In,T>    → SPMC只读输入通道（多消费者）
    // talos::spmc_mut<Out,T> → SPMC可写输出通道，写消息给下游/可视化
    // =================================================================================
    scheduler.add_system<talos::fixed_rate<250>>(
        "l4_aimer",
        [phase_states =
             std::unordered_map<core::TargetKey, TargetPhaseState, core::TargetKeyHash>{},
         unmanned_armor_target_decider = std::move(unmanned_armor_target_decider),
         manned_armor_target_decider   = std::move(manned_armor_target_decider),
         was_following = false, decider_state = ArmorTargetDeciderState{}](
            talos::res<L4Config> cfg,                          // 只读L4配置资源
            talos::res<Aimer> aimer,                           // 瞄准模块参数
            core::trajectory::trajectory_solver solver,         // 弹道求解器句柄（共享）
            core::trajectory::bullet_speed bullet_speed_,      // 弹丸初速资源
            core::following following,                         // 手控跟随开关原子标记
            talos::spmc<L3::TrackerOutputs, TrackerOutputChannelTopic> tracker_in, // L3跟踪输出输入通道
            talos::spmc<energy_meter::EnergyMeterState, EnergyMeterStateChannelTopic> rune_in, // 能量机关状态输入
            talos::spmc<L3::ldm::LdmState> ldm_in,             // LDM吊射目标输入通道
            talos::res<fast_tf::CoordinateSystem> tf_buffer,   // fast‑tf坐标变换缓存资源
            talos::spmc_mut<SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic> out_snap, // 输出选中目标快照（可视化+ROI使用）
            talos::spmc_mut<TargetSelectionTrace, TargetSelectionTraceChannelTopic> out_trace,   // 输出目标选择调试trace，给Foxglove
            talos::spmc_mut<ControlIntent, ControlIntentChannelTopic> out_ctrl) mutable {        // 输出云台控制意图，下发L1硬件层

            const uint64_t now_ns = fcs::clock::now_ns();

            // 查询当前时刻：云台坐标系、枪口坐标系变换；从fast_tf缓存插值得到
            const auto xf         = lookup_gimbal_muzzle_transforms(*tf_buffer, now_ns);
            // [[unlikely]]编译器提示：这个分支属于极少发生的冷路径，编译器做优化
            if (!xf) [[unlikely]] {
                SPDLOG_WARN("tf lookup failed: {}", xf.error());
                return; // TF查询失败，本帧直接放弃输出控制
            }
            // 结构化绑定：gimbal云台位姿，muzzle枪口位姿，ts_ns变换对应的时间戳
            const auto& [gimbal, muzzle, ts_ns] = *xf;

            // 组装瞄准上下文，把本帧所有不变参数打包，传递给各个决策函数
            const AimContext ctx{
                .gimbal       = gimbal,
                .muzzle       = muzzle,
                .current_ns   = now_ns,
                .bullet_speed = bullet_speed_->bullet_speed,
                .solver       = **solver,
                .aimer        = *aimer,
            };

            // ========== 手控跟随模式状态机 ==========
            // following是原子变量：外部UI/遥控器设置，true=手控跟随模式
            const bool is_following = following->load();
            // 检测上升沿：上一帧不是手控，本帧切到手控
            if (is_following && !was_following) {
                decider_state.selected_key.reset(); // 清空选中目标，手控强制重新选中心目标
            }

            // 根据模式切换使用哪一套目标决策器
            const ArmorTargetDecider& active_armor_target_decider =
                is_following ? manned_armor_target_decider : unmanned_armor_target_decider;
            // 更新状态，保存本帧跟随标记，给下一帧做边沿检测
            was_following = is_following;

            // 读取L3跟踪器最新一帧全部跟踪目标输出
            const auto tracker_vec = tracker_in.read_current();

            // 获取兜底目标：当完全没有新决策输出时，ROI模块可以继续使用上一个有效目标
            const auto fallback = pick_readback_tracker_fallback(tracker_vec, decider_state.selected_key);

            // ========== 目标优先级逻辑：1.装甲板优先 ==========
            // try_armor：装甲目标选择、弹道计算，输出瞄准决策std::optional<Decision>
            auto decision = try_armor(
                tracker_vec, ctx, active_armor_target_decider, decider_state, cfg->aimer,
                cfg->aimer.target_selection, cfg->reference_trajectory, phase_states);

            // ========== 优先级2：没有装甲目标，尝试打能量机关 ==========
            if (!decision) {
                decision = try_rune(rune_in.read_current(), ctx);
                if (decision) {
                    // 选中非装甲目标，清空装甲目标key，防止状态错乱
                    reset_selected_target_key_for_non_armor_source(decider_state);
                }
            }

            // ========== 优先级3：没有装甲、没有能量机关，尝试LDM吊射 ==========
            if (!decision) {
                auto i = ldm_in.read();
                // 如果LDM还处于检测状态，不输出吊射决策，直接返回
                if (i->status == L3::TrackerStatus::Detecting) {
                    return;
                }
                decision = try_ldm(ldm_in.read(), ctx, cfg->reference_trajectory);
                if (decision) {
                    reset_selected_target_key_for_non_armor_source(decider_state);
                }
            }

            // ========== 输出结果到各个SPMC通道 ==========
            if (decision) {
                // 有有效瞄准决策：把快照、trace调试信息、控制意图写入SPMC通道
                out_snap.write(std::move(decision->snapshot));
                out_trace.write(std::move(decision->trace));
                out_ctrl.write(std::move(decision->intent));
            } else {
                // 完全没有任何可打击目标：输出hold保持指令，云台不动
                write_hold_outputs(out_snap, out_trace, out_ctrl, decider_state, fallback);
            }
        });

    // =================================================================================
    // 【异步线程池系统 l4_optimal_target_readback_roi】 pool_compute 线程池任务
    // 不在250Hz主循环，丢进后台线程池执行，做ROI计算，不阻塞瞄准主逻辑
    // 作用：告诉L2感知，只需要检测目标附近一小块图像区域，裁剪ROI，降低推理算力开销
    // 输入：SelectedTargetSnapshot（l4_aimer输出的选中目标快照）
    // 输出：写入TrackerReadbackCache全局资源，L2检测器读取这个cache做图像裁剪
    // =================================================================================
    scheduler.add_system<talos::pool_compute>(
        "l4_optimal_target_readback_roi",
        [](talos::spmc<SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic> selected_target_in,
           talos::res<L2::ArmorReadbackRoiConfig> readback_roi_config,
           talos::res_mut<L2::TrackerReadbackCache> readback_cache) {
            // lambda内部辅助函数：判断当前快照是否支持ROI回传，只有装甲目标才支持
            auto has_readback_tracker = [](const SelectedTargetSnapshot& snapshot) noexcept {
                return snapshot.source == GimbalPlanSource::Armor
                    && tracker_supports_readback_roi(snapshot.tracker);
            };

            // spmc通道没有新数据，直接退出，不做计算
            if (!selected_target_in.has_new()) {
                return;
            }

            // 读取最新选中目标快照
            const auto selected_target = selected_target_in.read();
            if (!selected_target) {
                return;
            }

            // ROI总开关关闭 / 当前目标类型不支持ROI → 使缓存失效，L2不再做ROI裁剪
            if (!readback_roi_config->enabled || !has_readback_tracker(*selected_target)) {
                readback_cache->invalidate(selected_target->timestamp_ns);
                return;
            }

            // 根据目标快照，计算ROI对应的两组时间戳：投影时间、新鲜度时间
            const auto roi_timestamps = resolve_readback_roi_timestamps(*selected_target);

            // 将ROI信息存入全局缓存；L2装甲检测系统读取此缓存，对原图做ROI裁剪
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