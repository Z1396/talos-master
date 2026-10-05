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

// 引入瞄准系统头文件，声明本文件实现的接口
#include "L4_planning/aimer/aimer_systems.hpp"
// 引入LDM吊射滤波结果结构体定义
#include "L3_estimation/ldm_naive/types.hpp"          // LDM吊射滤波结果结构体
// 引入瞄准核心类Aimer，负责弹道解算
#include "L4_planning/aimer/aimer.hpp"                // 瞄准核心类Aimer，负责弹道解算
// 引入装甲目标决策器，区分有人/无人模式
#include "L4_planning/aimer/armor_target_decider.hpp" // 装甲目标决策器(有人/无人模式)
// 引入瞄准阶段状态机
#include "L4_planning/aimer/fsm.hpp"                  // 瞄准阶段状态机

// 引入L2感知ROI回读配置与缓存
#include "L2_perception/armor/readback_roi.hpp"       // L2感知ROI回读配置、缓存
// 引入能量机关L3输出结构体
#include "L3_estimation/energy_meter/types.hpp"       // 能量机关L3输出结构体
// 引入装甲跟踪器输出结构体
#include "L3_estimation/tracker/types.hpp"            // 装甲跟踪器输出
// 引入L4瞄准系统类型定义
#include "L4_planning/aimer/types.hpp"
// 引入TF坐标变换工具
#include "L4_planning/common/transform_utils.hpp"     // TF坐标变换工具
// 引入L4全局配置
#include "L4_planning/config.hpp"                     // L4全局配置
// 引入控制指令定义：TrackCommand/ShotCommand/HoldCommand
#include "L4_planning/control_intent.hpp" // 控制指令：TrackCommand/ShotCommand/HoldCommand
// 引入目标来源枚举(装甲/符/LDM)
#include "L4_planning/plan_source.hpp"    // 目标来源枚举(装甲/符/LDM)
// 引入选中目标快照定义
#include "L4_planning/selected_target_snapshot.hpp" // 选中目标快照
// 引入目标选择诊断日志定义
#include "L4_planning/target_selection_trace.hpp"   // 目标选择诊断日志
// 引入轨迹生成器
#include "L4_planning/trajectory_builder.hpp"       // 轨迹生成器
// 引入相机配置
#include "camera_config.hpp"
// 引入总线Topic定义
#include "core/channel_topics.hpp"                  // 总线Topic定义
// 引入运行时核心
#include "core/runtime.hpp"
// 引入目标唯一标识(颜色+装甲ID)
#include "core/target_key.hpp"                      // 目标唯一标识(颜色+装甲ID)
// 引入高精度时钟
#include "core/time.hpp"                            // 高精度时钟
// 引入弹道求解器全局资源
#include "core/trajectory/resource.hpp"             // 弹道求解器全局资源
// 引入欧拉角工具
#include "euler.hpp"
// 引入Talos调度器
#include "scheduler/scheduler.hpp"                  // Talos调度器

// 引入算法库，用于std::clamp、std::find等
#include <algorithm>
// 引入数学库，用于std::abs、std::remainder等
#include <cmath>
// 引入C++23错误处理，替代异常
#include <expected>                                 // C++23 错误处理，替代异常
// 引入帧定义
#include <frame.hpp>
// 引入函数式编程工具
#include <functional>
// 引入数值极限定义
#include <limits>
// 引入数学常数
#include <numbers>
// 引入可选值容器
#include <optional>
// 引入spdlog日志库
#include <spdlog/spdlog.h>
// 引入无序映射容器
#include <unordered_map>
// 引入动态数组容器
#include <vector>

// 进入fcs::L4命名空间
namespace fcs::L4 {

// 匿名命名空间，仅本编译单元可见
namespace {                                         // 匿名命名空间，仅本编译单元可见

/**
 * @brief 目标相位状态
 *
 * 跟踪每个目标的瞄准阶段状态，用于实现WholeCarCenter到WholeCarArmor的平滑过渡。
 * 瞄准阶段切换逻辑：先瞄准整车中心防止丢失，稳定后切换到单独装甲板击打
 */
struct TargetPhaseState {
    // 当前瞄准阶段：整车中心/单装甲
    ArmorAimPhase phase{ArmorAimPhase::SingleArmor}; ///< 当前瞄准阶段：整车中心/单装甲
    // 阶段切换计数，平滑过渡防抖
    int overflow_count{0};                           ///< 阶段切换计数，平滑过渡防抖
    // 目标跳变后记忆选中的装甲ID，防跳变乱切装甲
    std::optional<int> jumped_selected_armor_id{};   ///< 目标跳变后记忆选中的装甲ID，防跳变乱切装甲
};

/**
 * @brief 候选目标评估结果
 * 单个候选装甲目标全套评估数据，用于打分择优
 */
struct CandidateEvaluation {
    // 目标唯一键(敌方颜色+装甲编号)
    core::TargetKey key{};                   ///< 目标唯一键(敌方颜色+装甲编号)
    // L3卡尔曼跟踪输出
    L3::TrackerOutput tracker{};             ///< L3卡尔曼跟踪输出
    // 弹道解算后的瞄准预测结果
    TargetPrediction prediction{};           ///< 弹道解算后的瞄准预测结果
    // 五项加权打分分数
    TargetSelectionScores scores{};          ///< 五项加权打分分数
    // 诊断追踪记录，上位机可视化查看打分详情
    TargetSelectionTraceEntry trace_entry{}; ///< 诊断追踪记录，上位机可视化查看打分详情
};

/**
 * @brief 云台调整量度量
 * 计算云台从当前姿态转到瞄准姿态需要转动多少角度，用于打分(转动幅度越小分数越高)
 */
struct GimbalEffortMetrics {
    // yaw需要转动弧度
    double yaw_delta_rad{std::numeric_limits<double>::infinity()};   ///< yaw需要转动弧度
    // pitch需要转动弧度
    double pitch_delta_rad{std::numeric_limits<double>::infinity()}; ///< pitch需要转动弧度
    // 云台代价得分[0,1]，转动越小得分越高
    double score{0.0}; ///< 云台代价得分[0,1]，转动越小得分越高
};

/**
 * @brief 回读ROI时间戳信息
 * 下发ROI给L2感知，限制相机只在目标区域做识别，缩小检测范围提速、抗噪
 */
struct ReadbackRoiTimestamps {
    // 轨迹预测对应的时间戳
    uint64_t projection_timestamp_ns{0}; ///< 轨迹预测对应的时间戳
    // 目标最后观测时间戳，判断目标新鲜度
    uint64_t freshness_timestamp_ns{0};  ///< 目标最后观测时间戳，判断目标新鲜度
};

/**
 * @brief 装甲板候选集合
 * 一轮目标筛选的全部装甲候选、被淘汰目标，用于诊断
 */
struct ArmorCandidateCollection {
    // 当前存活的所有目标Key
    std::vector<core::TargetKey> active_keys{};    ///< 当前存活的所有目标Key
    // 有效可击打候选装甲
    std::vector<CandidateEvaluation> candidates{}; ///< 有效可击打候选装甲
    // 被剔除的目标(无跟踪/弹道求解失败)，用于日志排查
    std::vector<TargetSelectionTraceEntry>
        rejected{}; ///< 被剔除的目标(无跟踪/弹道求解失败)，用于日志排查
};

// 工具函数：数值钳位至 [0,1]
[[nodiscard]] double clamp01(double value) noexcept { return std::clamp(value, 0.0, 1.0); }

// 计算两个时间戳间隔，转为秒
[[nodiscard]] double elapsed_seconds(uint64_t start_ns, uint64_t end_ns) noexcept {
    // 如果结束时间不大于开始时间，返回0
    if (end_ns <= start_ns) {
        return 0.0;
    }
    // 计算时间差并转换为秒
    return static_cast<double>(end_ns - start_ns) * 1e-9;
}

// 目标视觉滞留时长：多久没有收到图像观测，越大代表目标越久没被看见
[[nodiscard]] double
    optical_age_seconds(const L3::TrackerOutput& tracker, uint64_t current_ns) noexcept {
    // 如果最后观测时间戳为0，说明从未观测过，返回0
    if (tracker.last_observation_timestamp_ns == 0) {
        return 0.0;
    }
    // 计算从最后观测到当前时间的间隔秒数
    return elapsed_seconds(tracker.last_observation_timestamp_ns, current_ns);
}

// 根据跟踪器构造目标唯一标识Key
[[nodiscard]] core::TargetKey make_target_key(const L3::TrackerOutput& tracker) noexcept {
    // 使用目标名称和颜色构造唯一键
    return core::TargetKey{tracker.target_name, tracker.target_color};
}

// 判断该目标是否支持下发ROI给感知层
[[nodiscard]] bool tracker_supports_readback_roi(const L3::TrackerOutput& tracker) noexcept {
    // 条件：正在跟踪、目标名称有效、是机器人或前哨站
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
        for (const auto& tracker : *trackers) {
            // tracker_supports_readback_roi(tracker)：判断该跟踪器是否支持ROI回读（有些跟踪器不能做ROI）
            // make_target_key(tracker)：从跟踪器生成唯一目标ID，和preferred_key比对
            if (tracker_supports_readback_roi(tracker) && make_target_key(tracker) == *preferred_key) {
                // 找到优先锁定目标，直接返回这个跟踪器，不再往下遍历
                return tracker;
            }
        }
    }
    // 走到这里：没有优先目标，或者优先目标已经丢失/不支持ROI，进入兜底自动挑选逻辑

    const L3::TrackerOutput* best = nullptr; // 记录当前最优跟踪器的指针，初始为空

    // 遍历全部跟踪结果，筛选可用ROI跟踪器，打分选出best
    for (const auto& tracker : *trackers) {
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
        if (tracker_has_finite != best_has_finite) {
            if (tracker_has_finite) {
                // 当前tracker数值有效，旧best无效，替换best
                best = &tracker;
            }
            continue; // 结束本轮循环，进入下一个tracker
        }

        // --------------------------第三优先级：画面上离图像中心越近优先级越高--------------------------
        // last_image_center_distance_px：目标框距离画面中心点像素距离，越小越靠近画面中间
        if (tracker_has_finite
            && tracker.last_image_center_distance_px < best->last_image_center_distance_px) {
            best = &tracker;
            continue;
        }

        // --------------------------第四优先级：观测时间戳更新的优先--------------------------
        // last_observation_timestamp_ns：这个跟踪器最后一次观测到目标的纳秒时间戳，越大越新
        if (tracker.last_observation_timestamp_ns > best->last_observation_timestamp_ns) {
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
    // 投影时间戳：优先使用跟踪器的时间戳，若为0则使用快照时间戳
    const uint64_t projection_timestamp_ns =
        snapshot.tracker.timestamp_ns != 0 ? snapshot.tracker.timestamp_ns : snapshot.timestamp_ns;
    // 新鲜度时间戳：优先使用跟踪器的最后观测时间戳，若为0则使用投影时间戳
    const uint64_t freshness_timestamp_ns = snapshot.tracker.last_observation_timestamp_ns != 0
                                              ? snapshot.tracker.last_observation_timestamp_ns
                                              : projection_timestamp_ns;
    // 返回组装好的时间戳结构体
    return {
        .projection_timestamp_ns = projection_timestamp_ns,
        .freshness_timestamp_ns  = freshness_timestamp_ns,
    };
}

// 获取敌方车辆/基地中心世界坐标
[[nodiscard]] std::optional<Eigen::Vector3d>
    tracker_center_world_position(const L3::TrackerOutput& tracker) noexcept {
    // 如果是机器人状态，直接返回其位置
    if (const auto* robot_state = tracker.robot_state()) {
        return robot_state->position;
    }
    // 如果是前哨站状态，返回其中心位置（三个z坐标的平均值）
    if (const auto* outpost_state = tracker.outpost_state()) {
        return Eigen::Vector3d{
            outpost_state->position.x(),
            outpost_state->position.y(),
            (outpost_state->z[0] + outpost_state->z[1] + outpost_state->z[2]) / 3.0,
        };
    }
    // 都不匹配，返回空
    return std::nullopt;
}

// 判断当前候选是否是上一轮选中的目标
[[nodiscard]] bool contains_target_key(
    const std::vector<core::TargetKey>& keys, const core::TargetKey& key) noexcept {
    // 在keys中查找key，找到则返回true
    return std::find(keys.begin(), keys.end(), key) != keys.end();
}

// 获取敌方车辆自转yaw角速度绝对值，用于阶段切换
[[nodiscard]] double target_abs_v_yaw(const L3::TrackerOutput& tracker) noexcept {
    // 如果是机器人状态，返回其yaw角速度绝对值
    if (const auto* robot_state = tracker.robot_state()) {
        return std::abs(robot_state->v_yaw);
    }
    // 如果是前哨站状态，返回其yaw角速度绝对值
    if (const auto* outpost_state = tracker.outpost_state()) {
        return std::abs(outpost_state->v_yaw);
    }
    // 都不匹配，返回0
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
    // 如果是机器人状态，调用aimer的aim函数进行弹道解算
    if (const auto* robot_state = tracker.robot_state()) {
        return aimer.aim(
            *robot_state, context, gimbal, muzzle, tracker.timestamp_ns, 
            current_ns, 0.0,
            bullet_speed, solver);
    }
    // 如果是前哨站状态，调用aimer的aim函数进行弹道解算
    if (const auto* outpost_state = tracker.outpost_state()) {
        return aimer.aim(
            *outpost_state, context, gimbal, muzzle, tracker.timestamp_ns, 
            current_ns, 0.0,
            bullet_speed, solver);
    }
    // 都不匹配，返回错误信息
    return std::unexpected("tracker output does not contain a supported target state");
}

// 打分项1：图像中心分数，目标越靠近画面中心得分越高；临时丢失时随时间衰减分数
[[nodiscard]] double score_image_center(
    const TargetSelectionConfig& cfg, const L3::TrackerOutput& tracker,
    uint64_t current_ns) noexcept {
    // 如果图像中心距离无效、无观测时间戳、参考距离无效，返回0分
    if (!std::isfinite(tracker.last_image_center_distance_px)
        || tracker.last_observation_timestamp_ns == 0 || cfg.refs.image_center_ref_px <= 1e-6) {
        return 0.0;
    }

    // 计算基础分数：1减去归一化距离
    double score = 1.0 - tracker.last_image_center_distance_px / cfg.refs.image_center_ref_px;
    // 钳位到[0,1]
    score        = clamp01(score);

    // 临时丢失目标，分数随观测间隔衰减，丢太久直接失去候选资格
    if (tracker.status == L3::TrackerStatus::TempLost) {
        // 如果超时时间配置无效，直接返回0
        if (cfg.optical_stale_timeout_s <= 1e-6) {
            return 0.0;
        }
        // 计算目标丢失时长
        const double age = elapsed_seconds(tracker.last_observation_timestamp_ns, current_ns);
        // 按衰减系数降低分数
        score *= clamp01(1.0 - age / cfg.optical_stale_timeout_s);
    }

    // 返回最终分数
    return score;
}

// 打分项2：跟踪状态分数，正常跟踪=1分，临时丢失使用配置低分
[[nodiscard]] double
    score_track_state(const TargetSelectionConfig& cfg, const L3::TrackerOutput& tracker) noexcept {
    // 正常跟踪返回1.0，否则返回配置的临时丢失分数
    return tracker.status == L3::TrackerStatus::Tracking ? 1.0 : cfg.temp_lost_state_score;
}

// 打分项3：子弹飞行时间分数，距离越近飞行时间越短，得分越高
[[nodiscard]] double
    score_tof(const TargetSelectionConfig& cfg, const TargetPrediction& prediction) noexcept {
    // 如果飞行时间无效或参考时间无效，返回0分
    if (!std::isfinite(prediction.flying_time) || cfg.refs.tof_ref_s <= 1e-6) {
        return 0.0;
    }
    // 计算分数：1减去归一化飞行时间，钳位到[0,1]
    return clamp01(1.0 - prediction.flying_time / cfg.refs.tof_ref_s);
}

// 计算云台转动代价并换算得分
[[nodiscard]] GimbalEffortMetrics compute_gimbal_effort_metrics(
    const TargetSelectionConfig& cfg, double current_yaw, double current_pitch,
    const TargetPrediction& prediction) noexcept {
    // 创建度量结果结构体
    GimbalEffortMetrics metrics;
    // 将yaw参考角度从度转换为弧度
    const double yaw_ref   = cfg.refs.yaw_effort_ref_deg * std::numbers::pi / 180.0;
    // 将pitch参考角度从度转换为弧度
    const double pitch_ref = cfg.refs.pitch_effort_ref_deg * std::numbers::pi / 180.0;
    // 如果参考角度无效，返回默认metrics（无穷大）
    if (yaw_ref <= 1e-6 || pitch_ref <= 1e-6) {
        return metrics;
    }

    // 计算yaw最短转向角度(处理±180°环绕)
    metrics.yaw_delta_rad =
        std::abs(std::remainder(prediction.aim_yaw - current_yaw, 2.0 * std::numbers::pi));
    // 将预测pitch取反（TF坐标系转换）
    const double command_pitch_in_tf = -prediction.aim_pitch;
    // 计算pitch转向角度绝对值
    metrics.pitch_delta_rad          = std::abs(command_pitch_in_tf - current_pitch);
    // 计算综合代价：yaw和pitch归一化后的平均值
    const double effort =
        0.5 * (metrics.yaw_delta_rad / yaw_ref + metrics.pitch_delta_rad / pitch_ref);
    // 分数为1减去代价，钳位到[0,1]
    metrics.score = clamp01(1.0 - effort);
    // 返回度量结果
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
    // 创建打分结果结构体
    TargetSelectionScores scores;
    // 计算图像中心分数
    scores.image_center = score_image_center(cfg, tracker, current_ns);
    // 计算跟踪状态分数
    scores.track_state  = score_track_state(cfg, tracker);
    // 计算飞行时间分数
    scores.tof          = score_tof(cfg, prediction);
    // 计算云台调整量分数
    scores.gimbal_effort =
        compute_gimbal_effort_metrics(cfg, current_yaw, current_pitch, prediction).score;
    // 计算装甲名称分数并钳位
    scores.armor_name = clamp01(cfg.armor_name_score.score(tracker.target_name));

    // 加权求和
    const double weighted_sum = cfg.weights.image_center * scores.image_center
                              + cfg.weights.track_state * scores.track_state
                              + cfg.weights.tof * scores.tof
                              + cfg.weights.gimbal_effort * scores.gimbal_effort
                              + cfg.weights.armor_name * scores.armor_name;
    // 计算总权重
    const double total_weight = cfg.weights.image_center + cfg.weights.track_state + cfg.weights.tof
                              + cfg.weights.gimbal_effort + cfg.weights.armor_name;

    // 归一化总分，避免除零
    scores.total = total_weight > 1e-6 ? weighted_sum / total_weight : 0.0;
    // 返回打分结果
    return scores;
}

// 生成选中目标诊断日志条目，用于上位机回放目标选择全过程
[[nodiscard]] TargetSelectionTraceEntry build_scored_trace_entry(
    const TargetSelectionConfig& cfg, const L3::TrackerOutput& tracker,
    const TargetPrediction& prediction, const TargetSelectionScores& scores, double current_yaw,
    double current_pitch, uint64_t current_ns, bool was_previously_selected) noexcept {
    // 计算云台调整量度量
    const auto effort_metrics =
        compute_gimbal_effort_metrics(cfg, current_yaw, current_pitch, prediction);

    // 创建诊断条目
    TargetSelectionTraceEntry entry;
    // 记录目标名称
    entry.target_name              = tracker.target_name;
    // 记录目标颜色
    entry.target_color             = tracker.target_color;
    // 记录跟踪状态
    entry.track_status             = tracker.status;
    // 标记瞄准有效
    entry.aim_valid                = true;
    // 记录是否上一轮选中
    entry.was_previously_selected  = was_previously_selected;
    // 记录目标中心世界坐标
    entry.target_center            = tracker_center_world_position(tracker);
    // 记录图像中心距离
    entry.image_center_distance_px = tracker.last_image_center_distance_px;
    // 记录视觉滞留时长
    entry.optical_age_s            = optical_age_seconds(tracker, current_ns);
    // 记录飞行时间
    entry.tof_s                    = prediction.flying_time;
    // 记录距离
    entry.distance_m               = prediction.distance;
    // 记录yaw调整角度（度）
    entry.yaw_effort_deg           = effort_metrics.yaw_delta_rad * 180.0 / std::numbers::pi;
    // 记录pitch调整角度（度）
    entry.pitch_effort_deg         = effort_metrics.pitch_delta_rad * 180.0 / std::numbers::pi;

    // 各分项分数
    // 图像中心分数
    entry.image_center_score  = scores.image_center;
    // 跟踪状态分数
    entry.track_state_score   = scores.track_state;
    // 飞行时间分数
    entry.tof_score           = scores.tof;
    // 云台调整量分数
    entry.gimbal_effort_score = scores.gimbal_effort;
    // 装甲名称分数
    entry.armor_name_score    = scores.armor_name;

    // 加权分项
    // 图像中心加权分
    entry.image_center_weighted  = cfg.weights.image_center * scores.image_center;
    // 跟踪状态加权分
    entry.track_state_weighted   = cfg.weights.track_state * scores.track_state;
    // 飞行时间加权分
    entry.tof_weighted           = cfg.weights.tof * scores.tof;
    // 云台调整量加权分
    entry.gimbal_effort_weighted = cfg.weights.gimbal_effort * scores.gimbal_effort;
    // 装甲名称加权分
    entry.armor_name_weighted    = cfg.weights.armor_name * scores.armor_name;
    // 加权总分
    entry.weighted_sum           = entry.image_center_weighted + entry.track_state_weighted
                       + entry.tof_weighted + entry.gimbal_effort_weighted
                       + entry.armor_name_weighted;
    // 总权重
    entry.total_weight = cfg.weights.image_center + cfg.weights.track_state + cfg.weights.tof
                       + cfg.weights.gimbal_effort + cfg.weights.armor_name;
    // 归一化总分
    entry.total_score = scores.total;
    // 返回诊断条目
    return entry;
}

// 生成被淘汰目标的诊断日志，记录淘汰原因
[[nodiscard]] TargetSelectionTraceEntry build_rejected_trace_entry(
    const L3::TrackerOutput& tracker, uint64_t current_ns, std::string reason,
    bool was_previously_selected) noexcept {
    // 创建诊断条目
    TargetSelectionTraceEntry entry;
    // 记录目标名称
    entry.target_name              = tracker.target_name;
    // 记录目标颜色
    entry.target_color             = tracker.target_color;
    // 记录跟踪状态
    entry.track_status             = tracker.status;
    // 记录是否上一轮选中
    entry.was_previously_selected  = was_previously_selected;
    // 记录目标中心世界坐标
    entry.target_center            = tracker_center_world_position(tracker);
    // 记录图像中心距离
    entry.image_center_distance_px = tracker.last_image_center_distance_px;
    // 记录视觉滞留时长
    entry.optical_age_s            = optical_age_seconds(tracker, current_ns);
    // 标记瞄准无效
    entry.aim_valid                = false;
    // 记录淘汰原因
    entry.aim_error                = std::move(reason);
    // 返回诊断条目
    return entry;
}

// 组装完整一轮目标选择诊断轨迹数据包
[[nodiscard]] TargetSelectionTrace build_target_selection_trace(
    uint64_t current_ns, const std::optional<core::TargetKey>& previous_key,
    const ArmorTargetDecider& decider, double switch_margin,
    const ArmorTargetDeciderResult& decider_result,
    const std::vector<CandidateEvaluation>& candidates,
    std::vector<TargetSelectionTraceEntry> rejected) {
    // 创建诊断轨迹结构体
    TargetSelectionTrace trace{
        // 当前时间戳
        .timestamp_ns        = current_ns,
        // 是否保持当前目标
        .kept_current_target = decider_result.kept_current_target,
        // 只有自动驾驶模式才有切换阈值余量，手控模式切换余量为0
        .switch_margin =
            std::holds_alternative<UnmannedArmorTargetDecider>(decider) ? switch_margin : 0.0,
    };

    // 如果存在上一轮目标
    if (previous_key.has_value()) {
        // 标记有上一轮目标
        trace.had_previous_target   = true;
        // 记录上一轮目标名称
        trace.previous_target_name  = previous_key->name;
        // 记录上一轮目标颜色
        trace.previous_target_color = previous_key->color;
    }

    // 预留候选+淘汰目标的空间
    trace.candidates.reserve(candidates.size() + rejected.size());
    // 写入排名靠前的候选目标
    for (size_t rank = 0; rank < decider_result.ranked_indices.size(); ++rank) {
        // 获取候选索引
        const size_t candidate_index = decider_result.ranked_indices[rank];
        // 拷贝候选诊断条目
        auto candidate_trace         = candidates[candidate_index].trace_entry;
        // 设置排名（从1开始）
        candidate_trace.rank         = static_cast<int>(rank + 1);
        // 标记是否被选中
        candidate_trace.selected     = candidate_index == decider_result.selected_index;
        // 标记是否是第二名
        candidate_trace.runner_up =
            decider_result.runner_up.has_value() && candidate_index == *decider_result.runner_up;
        // 加入诊断轨迹
        trace.candidates.push_back(std::move(candidate_trace));
    }

    // 写入被淘汰目标
    for (auto& rejected_entry : rejected) {
        // 设置排名（排在候选之后）
        rejected_entry.rank = static_cast<int>(trace.candidates.size() + 1);
        // 加入诊断轨迹
        trace.candidates.push_back(std::move(rejected_entry));
    }

    // 返回诊断轨迹
    return trace;
}

// 统一决策输出结构：最终控制指令+选中目标快照+诊断日志
struct AimDecision {
    // 控制指令
    ControlIntent intent;
    // 选中目标快照
    SelectedTargetSnapshot snapshot;
    // 诊断日志
    TargetSelectionTrace trace;
};

// 瞄准全局只读上下文，避免重复传参
struct AimContext {
    // 云台变换
    const Aimer::GimbalTransform& gimbal;
    // 枪口变换
    const Aimer::MuzzleTransform& muzzle;
    // 当前时间戳
    uint64_t current_ns;
    // 弹速
    double bullet_speed;
    // 弹道求解器
    const core::trajectory::solver::TrajectorySolver& solver;
    // 瞄准器
    const Aimer& aimer;
};

// 根据预测结果生成选中目标快照，下发给其他模块
[[nodiscard]] SelectedTargetSnapshot make_snapshot(
    const TargetPrediction& pred, GimbalPlanSource source,
    L3::TrackerOutput tracker = {}) noexcept {
    // 返回组装好的快照
    return SelectedTargetSnapshot{
        // 时间戳
        .timestamp_ns            = pred.timestamp_ns,
        // 来源
        .source                  = source,
        // 距离
        .distance                = pred.distance,
        // 预测未来时间戳
        .predicted_future_ns     = pred.predicted_future_ns,
        // 瞄准阶段
        .aim_phase               = pred.aim_phase,
        // 选中装甲ID
        .selected_armor_id       = pred.selected_armor_id,
        // 粗略选中装甲ID
        .rough_selected_armor_id = pred.rough_selected_armor_id,
        // 跟踪器输出
        .tracker                 = std::move(tracker),
    };
}

// 构造轨迹规划种子，用于生成车辆运动参考轨迹
[[nodiscard]] std::optional<PlannerSeed>
    make_planner_seed(const L3::TrackerOutput& tracker, const TargetPrediction& pred) noexcept {
    // 创建规划种子
    PlannerSeed s;
    // 设置状态时间戳
    s.state_timestamp_ns    = tracker.timestamp_ns;
    // 设置目标是否跳变
    s.target_jumped         = tracker.target_jumped;
    // 设置跟踪器最后装甲ID
    s.tracker_last_armor_id = tracker.last_armor_id;
    // 设置瞄准阶段
    s.aim_phase             = pred.aim_phase;
    // 设置选中装甲ID
    s.selected_armor_id     = pred.selected_armor_id;
    // 设置装甲类型
    s.armor_type            = pred.armor_type;
    // 如果是机器人状态，设置状态并返回
    if (const auto* rs = tracker.robot_state()) {
        s.state = *rs;
        return s;
    }
    // 如果是前哨站状态，设置状态并返回
    if (const auto* os = tracker.outpost_state()) {
        s.state = *os;
        return s;
    }
    // 都不匹配，返回空
    return std::nullopt;
}

// 构造装甲瞄准阶段上下文，用于整车/单装甲平滑切换
[[nodiscard]] ArmorAimContext build_armor_aim_context(
    const L3::TrackerOutput& tracker, const TargetPhaseState& phase_state) noexcept {
    // 创建瞄准上下文
    ArmorAimContext aim_ctx;
    // 设置目标是否跳变
    aim_ctx.target_jumped      = tracker.target_jumped;
    // 设置当前瞄准阶段
    aim_ctx.phase              = phase_state.phase;
    // 设置优先装甲ID为跟踪器最后装甲ID
    aim_ctx.preferred_armor_id = tracker.last_armor_id;
    // 目标跳变时沿用记忆的装甲ID，防止瞬间乱切装甲
    if (tracker.target_jumped && phase_state.jumped_selected_armor_id.has_value()) {
        aim_ctx.preferred_armor_id = phase_state.jumped_selected_armor_id;
    }
    // 返回瞄准上下文
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
    // 创建候选集合结果对象
    ArmorCandidateCollection result;

    // 计算预留空间大小：如果trackers有值则用其大小，否则为0
    const size_t reserve_n = trackers ? trackers->size() : 0U;
    // 预留活跃目标Key的空间
    result.active_keys.reserve(reserve_n);
    // 预留候选评估结果的空间
    result.candidates.reserve(reserve_n);
    // 预留被淘汰目标的空间
    result.rejected.reserve(reserve_n);

    // 如果跟踪器集合无值，直接返回空结果
    if (!trackers.has_value()) {
        return result;
    }

    // 获取当前云台yaw/pitch
    const auto cur_euler   = ctx.gimbal.euler_rot();
    // 提取当前yaw角
    const double cur_yaw   = cur_euler.yaw;
    // 提取当前pitch角
    const double cur_pitch = cur_euler.pitch;

    // 遍历所有跟踪器输出
    for (const auto& tracker : *trackers) {
        // 如果该跟踪器不在跟踪状态，跳过
        if (!tracker.is_tracking()) {
            continue;
        }

        // 根据跟踪器构造目标唯一键
        const auto key = make_target_key(tracker);
        // 将目标键加入活跃目标列表
        result.active_keys.push_back(key);
        // 判断该目标是否是上一轮选中的目标
        const bool was_prev = previous_key.has_value() && key == *previous_key;

        // 获取或创建该目标的相位状态引用
        auto& phase_state = phase_states[key];

        // 判断到底是哪一级的旋转速度，更新瞄准阶段（整车中心/单装甲）
        advance_armor_aim_phase(
            aimer_cfg, target_abs_v_yaw(tracker), tracker.target_jumped, phase_state.phase,
            phase_state.overflow_count);

        // 弹道解算
        auto prediction = aim_tracker_target(
            ctx.aimer, tracker, build_armor_aim_context(tracker, phase_state), ctx.gimbal,
            ctx.muzzle, ctx.current_ns, ctx.bullet_speed, ctx.solver);
        // 如果弹道解算失败
        if (!prediction) {
            // 弹道求解失败，加入淘汰列表
            result.rejected.push_back(
                build_rejected_trace_entry(tracker, ctx.current_ns, prediction.error(), was_prev));
            // 继续处理下一个跟踪器
            continue;
        }

        // 打分封装
        CandidateEvaluation candidate;
        // 设置候选目标键
        candidate.key        = key;
        // 设置跟踪器输出
        candidate.tracker    = tracker;
        // 设置弹道预测结果
        candidate.prediction = *prediction;
        // 计算五项加权打分
        candidate.scores =
            score_candidate(sel_cfg, tracker, *prediction, cur_yaw, cur_pitch, ctx.current_ns);
        // 构建诊断追踪条目
        candidate.trace_entry = build_scored_trace_entry(
            sel_cfg, tracker, *prediction, candidate.scores, cur_yaw, cur_pitch, ctx.current_ns,
            was_prev);
        // 将候选加入候选列表
        result.candidates.push_back(std::move(candidate));
    }

    // 返回候选集合
    return result;
}

// 清理已经消失的目标的阶段状态，防止哈希表膨胀
auto cleanup_stale_phase_states(
    const std::vector<core::TargetKey>& active_keys,
    std::unordered_map<core::TargetKey, TargetPhaseState, core::TargetKeyHash>& phase_states,
    ArmorTargetDeciderState& decider_state) -> void {
    // 如果活跃目标列表为空
    if (active_keys.empty()) {
        // 清空所有相位状态
        phase_states.clear();
        // 重置决策器选中的目标键
        decider_state.selected_key.reset();
        // 直接返回
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
    // 根据目标名称转换为装甲类型
    selected.prediction.armor_type = cls_to_armor_type(selected.tracker.target_name);

    // 获取或创建该目标的相位状态引用
    auto& phase_state      = phase_states[selected.key];
    // 判断目标跳变是否活跃：正在跟踪且发生跳变
    const bool jump_active = selected.tracker.is_tracking() && selected.tracker.target_jumped;
    // 判断是否需要保留跳变记忆：非单装甲阶段或跳变活跃
    const bool keep_jump_memory =
        selected.prediction.aim_phase != ArmorAimPhase::SingleArmor || jump_active;
    // 如果不需要保留跳变记忆
    if (!keep_jump_memory) {
        // 重置跳变记忆的装甲ID
        phase_state.jumped_selected_armor_id.reset();
        // 返回
        return;
    }

    // 更新跳变记忆的装甲ID为当前选中的装甲ID
    phase_state.jumped_selected_armor_id = selected.prediction.selected_armor_id;
}

// 击打能量机关/LDM时，清空装甲选中标记，避免切回装甲时状态错乱
auto reset_selected_target_key_for_non_armor_source(ArmorTargetDeciderState& decider_state)
    -> void {
    // 重置决策器选中的目标键
    decider_state.selected_key.reset();
}

// 无任何目标可击打时，输出HoldCommand保持云台不动，并兜底下发ROI
auto write_hold_outputs(
    talos::spmc_mut<SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic>& out_snap,
    talos::spmc_mut<TargetSelectionTrace, TargetSelectionTraceChannelTopic>& out_trace,
    talos::spmc_mut<ControlIntent, ControlIntentChannelTopic>& out_ctrl,
    ArmorTargetDeciderState& decider_state, const std::optional<L3::TrackerOutput>& fallback)
    -> void {
    // 获取当前纳秒时间戳
    const uint64_t now_ns = fcs::clock::now_ns();
    // 创建快照，只设置时间戳
    SelectedTargetSnapshot snap{.timestamp_ns = now_ns};
    // 如果有兜底跟踪器
    if (fallback.has_value()) {
        // 设置来源为装甲
        snap.source                = GimbalPlanSource::Armor;
        // 设置跟踪器为兜底跟踪器
        snap.tracker               = *fallback;
        // 更新决策器选中目标键为兜底目标
        decider_state.selected_key = make_target_key(*fallback);
    } else {
        // 没有兜底目标，重置选中目标键
        decider_state.selected_key.reset();
    }

    // 写入快照到输出通道
    out_snap.write(std::move(snap));
    // 写入空的诊断轨迹到输出通道
    out_trace.write(TargetSelectionTrace{.timestamp_ns = now_ns});
    // 写入HoldCommand保持指令到控制通道
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
    // 构造轨迹构建上下文
    const TrajectoryBuildContext build_ctx{
        // 当前时间戳
        .current_ns        = ctx.current_ns,
        // 云台变换
        .gimbal            = ctx.gimbal,
        // 枪口变换
        .muzzle            = ctx.muzzle,
        // 弹道求解器指针
        .trajectory_solver = &ctx.solver,
        // 弹速
        .bullet_speed      = ctx.bullet_speed,
    };

    // 降级原因字符串（可选）
    std::optional<std::string> degradation;

    // 装甲目标：构建车辆运动参考轨迹实现平滑跟随
    if (seed.has_value()) {
        // 构建控制参考轨迹
        auto control_traj = build_reference_trajectory(*seed, ctx.aimer, ref_traj_cfg, build_ctx);
        // 如果控制轨迹构建成功
        if (control_traj) {
            // 构建开火参考轨迹
            auto fire_traj =
                build_fire_reference_trajectory(*seed, ctx.aimer, ref_traj_cfg, build_ctx);
            // 如果开火轨迹构建失败
            if (!fire_traj) {
                // 记录警告日志
                SPDLOG_WARN("Failed to build fire reference trajectory: {}", fire_traj.error());
            }
            // 返回TrackCommand跟踪指令
            return TrackCommand{
                // 时间戳
                .timestamp_ns       = pred.timestamp_ns,
                // 控制轨迹
                .control_trajectory = std::move(*control_traj),
                // 开火轨迹：成功则使用，失败则使用空轨迹
                .fire_trajectory =
                    fire_traj ? std::move(*fire_traj) : core::trajectory::ReferenceTrajectory{},
            };
        }
        // 控制轨迹构建失败，记录降级原因
        degradation = control_traj.error();
        // 记录警告日志
        SPDLOG_WARN("Failed to build reference trajectory: {}", *degradation);
    } else if (ldm_for_track.has_value() && ldm_for_track->is_tracking()) {
        // LDM吊射目标构建轨迹
        auto control_traj =
            build_ldm_reference_trajectory(*ldm_for_track, ctx.aimer, ref_traj_cfg, build_ctx);
        // 如果LDM轨迹构建成功
        if (control_traj) {
            // 拷贝一份控制轨迹作为开火轨迹
            auto fire_traj = *control_traj;
            // 返回TrackCommand跟踪指令
            return TrackCommand{
                // 时间戳
                .timestamp_ns       = pred.timestamp_ns,
                // 控制轨迹
                .control_trajectory = std::move(*control_traj),
                // 开火轨迹
                .fire_trajectory    = std::move(fire_traj),
            };
        }
        // LDM轨迹构建失败，记录降级原因
        degradation = control_traj.error();
        // 记录警告日志
        SPDLOG_WARN("Failed to build LDM reference trajectory: {}", *degradation);
    }

    // 轨迹构建失败，降级为直接射击指令
    return ShotCommand{
        // 时间戳
        .timestamp_ns       = pred.timestamp_ns,
        // 瞄准yaw角
        .yaw                = pred.aim_yaw,
        // 瞄准pitch角
        .pitch              = pred.aim_pitch,
        // 距离
        .distance           = pred.distance,
        // 降级原因
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

    // 遍历所有候选
    for (const auto& candidate : collection.candidates) {
        // 将候选转换为决策器输入格式并加入列表
        decider_candidates.push_back(
            ArmorTargetDeciderCandidate{
                // 目标唯一ID
                .key     = candidate.key,     // 目标唯一ID
                // 跟踪器输出位姿、观测信息
                .tracker = candidate.tracker, // 跟踪器输出位姿、观测信息
                // 各项打分（距离、角度、置信度等）
                .scores  = candidate.scores,  // 各项打分（距离、角度、置信度等）
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
    auto trace = build_target_selection_trace(
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
        // 构建控制意图，LDM参数传nullopt
        .intent = build_control_intent(selected.prediction, ctx, ref_traj_cfg, seed, std::nullopt),
        // 移动快照
        .snapshot = std::move(snap),
        // 移动诊断轨迹
        .trace    = std::move(trace),
    };
}

/**
 * @brief 能量机关目标分支
 * 能量机关无运动轨迹规划，直接输出ShotCommand瞬时瞄准
 */
[[nodiscard]] std::optional<AimDecision>
    try_rune(const std::optional<energy_meter::EnergyMeterState>& rune_in, const AimContext& ctx) {
    // 如果能量机关输入无值或模型无效，返回空
    if (!rune_in || !rune_in->model_valid) {
        return std::nullopt;
    }

    // 调用aimer进行能量机关弹道解算
    auto pred = ctx.aimer.aim(
        rune_in.value(), ctx.gimbal, ctx.muzzle, ctx.current_ns, ctx.bullet_speed, ctx.solver);
    // 如果解算失败，返回空
    if (!pred)
        return std::nullopt;

    // 生成目标快照，来源标记为Rune
    auto snap = make_snapshot(*pred, GimbalPlanSource::Rune);

    // 组装瞄准决策返回
    return AimDecision{
        // 控制意图：直接射击指令
        .intent =
            ShotCommand{
                        // 时间戳
                        .timestamp_ns = pred->timestamp_ns,
                        // 瞄准yaw角
                        .yaw          = pred->aim_yaw,
                        // 瞄准pitch角
                        .pitch        = pred->aim_pitch,
                        // 距离
                        .distance     = pred->distance,
                        },
        // 移动快照
        .snapshot = std::move(snap),
        // 诊断轨迹只填时间戳
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
        // 移动快照
        .snapshot = std::move(snap),
        // trace只填时间戳；能量机关没有多候选目标选择逻辑，没有选目标trace信息
        .trace = TargetSelectionTrace{.timestamp_ns = ctx.current_ns},
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
    // 获取调度器的world资源容器引用
    auto& world = scheduler.world();

    // ========== 注入全局资源：ROI配置、跟踪器缓存 ==========
    // 如果全局资源还不存在，则插入资源；防止多次调用register重复插入报错
    if (!world.has_resource<L2::ArmorReadbackRoiConfig>()) {
        // 插入ROI配置资源
        world.insert_resource(L2::ArmorReadbackRoiConfig{});
    }
    // 如果跟踪器缓存资源不存在
    if (!world.has_resource<L2::TrackerReadbackCache>()) {
        // 插入跟踪器缓存资源
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
    // 移动L4总配置到资源
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
    // =================================================================================
    scheduler.add_system<talos::fixed_rate<250>>(
        // 系统名称
        "l4_aimer",
        // lambda捕获列表：捕获本地持久状态
        [phase_states = std::unordered_map<core::TargetKey, TargetPhaseState, core::TargetKeyHash>{},
         // 移动自动决策器
         unmanned_armor_target_decider = std::move(unmanned_armor_target_decider),
         // 移动手控决策器
         manned_armor_target_decider   = std::move(manned_armor_target_decider),
         // 上一帧是否跟随标记
         was_following = false, decider_state = ArmorTargetDeciderState{}](
            // 只读L4配置资源
            talos::res<L4Config> cfg,                        // 只读L4配置资源
            // 瞄准模块参数
            talos::res<Aimer> aimer,                         // 瞄准模块参数
            // 弹道求解器句柄（共享）
            core::trajectory::trajectory_solver solver,      // 弹道求解器句柄（共享）
            // 弹丸初速资源
            core::trajectory::bullet_speed bullet_speed_,    // 弹丸初速资源
            // 手控跟随开关原子标记
            core::following following,                       // 手控跟随开关原子标记
            // L3跟踪输出输入通道
            talos::spmc<L3::TrackerOutputs, TrackerOutputChannelTopic> tracker_in,                        // L3跟踪输出输入通道
            // 能量机关状态输入
            talos::spmc<energy_meter::EnergyMeterState, EnergyMeterStateChannelTopic> rune_in,              // 能量机关状态输入
            // LDM吊射目标输入通道
            talos::spmc<L3::ldm::LdmState> ldm_in,           // LDM吊射目标输入通道
            // fast‑tf坐标变换缓存资源
            talos::res<fast_tf::CoordinateSystem> tf_buffer, // fast‑tf坐标变换缓存资源
            // 输出选中目标快照（可视化+ROI使用）
            talos::spmc_mut<SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic> out_snap,         // 输出选中目标快照（可视化+ROI使用）
            // 输出目标选择调试trace，给Foxglove
            talos::spmc_mut<TargetSelectionTrace, TargetSelectionTraceChannelTopic> out_trace,             // 输出目标选择调试trace，给Foxglove
            // 输出云台控制意图，下发L1硬件层
            talos::spmc_mut<ControlIntent, ControlIntentChannelTopic> out_ctrl) mutable {          // 输出云台控制意图，下发L1硬件层
            // 获取当前纳秒时间戳
            const uint64_t now_ns = fcs::clock::now_ns();

            // 查询当前时刻：云台坐标系、枪口坐标系变换；从fast_tf缓存插值得到
            const auto xf = lookup_gimbal_muzzle_transforms(*tf_buffer, now_ns);
            // [[unlikely]]编译器提示：这个分支属于极少发生的冷路径，编译器做优化
            if (!xf) [[unlikely]] {
                // 记录TF查询失败警告
                SPDLOG_WARN("tf lookup failed: {}", xf.error());
                // TF查询失败，本帧直接放弃输出控制
                return; // TF查询失败，本帧直接放弃输出控制
            }
            // 结构化绑定：gimbal云台位姿，muzzle枪口位姿，ts_ns变换对应的时间戳
            const auto& [gimbal, muzzle, ts_ns] = *xf;
 
            // 组装瞄准上下文，把本帧所有不变参数打包，传递给各个决策函数
            const AimContext ctx{
                // 云台变换
                .gimbal       = gimbal,
                // 枪口变换
                .muzzle       = muzzle,
                // 当前时间戳
                .current_ns   = now_ns,
                // 子弹速度
                .bullet_speed = bullet_speed_->bullet_speed,
                // 弹道求解器
                .solver       = **solver,
                // 瞄准器
                .aimer        = *aimer,
            };

            // ========== 手控跟随模式状态机 ==========
            // following是原子变量：外部UI/遥控器设置，true=手控跟随模式
            const bool is_following = following->load();
            // 检测上升沿：上一帧不是手控，本帧切到手控
            if (is_following && !was_following) {
                // 清空选中目标，手控强制重新选中心目标
                decider_state.selected_key.reset(); // 清空选中目标，手控强制重新选中心目标
            }

            // 根据模式切换使用哪一套目标决策器（有人/无人）
            const ArmorTargetDecider& active_armor_target_decider =
                is_following ? manned_armor_target_decider : unmanned_armor_target_decider;
            // 更新状态，保存本帧跟随标记，给下一帧做边沿检测
            was_following = is_following;

            // 读取L3跟踪器最新一帧全部跟踪目标输出
            const auto tracker_vec = tracker_in.read_current();

            // 获取兜底目标：当完全没有新决策输出时，ROI模块可以继续使用上一个有效目标
            const auto fallback =
                pick_readback_tracker_fallback(tracker_vec, decider_state.selected_key);

            // ========== 目标优先级逻辑：1.装甲板优先 ========== 
            // try_armor：装甲目标选择、弹道计算，输出瞄准决策std::optional<Decision>
            auto decision = try_armor(
                tracker_vec, ctx, active_armor_target_decider, decider_state, cfg->aimer,
                cfg->aimer.target_selection, cfg->reference_trajectory, phase_states);

            // ========== 优先级2：没有装甲目标，尝试打能量机关 ==========
            if (!decision) {
                // 尝试能量机关瞄准
                decision = try_rune(rune_in.read_current(), ctx);
                // 如果能量机关瞄准成功
                if (decision) {
                    // 选中非装甲目标，清空装甲目标key，防止状态错乱
                    reset_selected_target_key_for_non_armor_source(decider_state);
                }
            }

            // ========== 优先级3：没有装甲、没有能量机关，尝试LDM吊射 ==========
            if (!decision) {
                // 读取LDM状态
                auto i = ldm_in.read();
                // 如果LDM还处于检测状态，不输出吊射决策，直接返回
                if (i->status == L3::TrackerStatus::Detecting) {
                    // 直接返回
                    return;
                }
                // 尝试LDM吊射瞄准
                decision = try_ldm(ldm_in.read(), ctx, cfg->reference_trajectory);
                // 如果LDM瞄准成功
                if (decision) {
                    // 清空装甲选中标记
                    reset_selected_target_key_for_non_armor_source(decider_state);
                }
            }

            // ========== 输出结果到各个SPMC通道 ==========
            if (decision) {
                // 有有效瞄准决策：把快照、trace调试信息、控制意图写入SPMC通道
                out_snap.write(std::move(decision->snapshot));
                // 写入诊断轨迹
                out_trace.write(std::move(decision->trace));
                // 写入控制意图
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
        // 系统名称
        "l4_optimal_target_readback_roi",
        // lambda：无捕获，纯计算
        [](talos::spmc<SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic> selected_target_in,
           talos::res<L2::ArmorReadbackRoiConfig> readback_roi_config,
           talos::res_mut<L2::TrackerReadbackCache> readback_cache) {
            // lambda内部辅助函数：判断当前快照是否支持ROI回传，只有装甲目标才支持
            auto has_readback_tracker = [](const SelectedTargetSnapshot& snapshot) noexcept {
                // 来源为装甲且跟踪器支持ROI
                return snapshot.source == GimbalPlanSource::Armor
                    && tracker_supports_readback_roi(snapshot.tracker);
            };

            // spmc通道没有新数据，直接退出，不做计算
            if (!selected_target_in.has_new()) {
                return;
            }

            // 读取最新选中目标快照
            const auto selected_target = selected_target_in.read();
            // 如果快照无值，直接返回
            if (!selected_target) {
                return;
            }

            // ROI总开关关闭 / 当前目标类型不支持ROI → 使缓存失效，L2不再做ROI裁剪
            if (!readback_roi_config->enabled || !has_readback_tracker(*selected_target)) {
                // 使缓存失效
                readback_cache->invalidate(selected_target->timestamp_ns);
                // 返回
                return;
            }

            // 根据目标快照，计算ROI对应的两组时间戳：投影时间、新鲜度            
            const auto roi_timestamps = resolve_readback_roi_timestamps(*selected_target);

            // 将ROI信息存入全局缓存；L2装甲检测系统读取此缓存，对原图做ROI裁剪
            readback_cache->store(
                L2::TrackerReadbackSnapshot{
                    // 标记有效
                    .valid                   = true,
                    // 时间戳使用新鲜度时间戳
                    .timestamp_ns            = roi_timestamps.freshness_timestamp_ns,
                    // 投影时间戳
                    .projection_timestamp_ns = roi_timestamps.projection_timestamp_ns,
                    // 选中装甲ID
                    .selected_armor_id       = selected_target->selected_armor_id,
                    // 粗略选中装甲ID
                    .rough_selected_armor_id = selected_target->rough_selected_armor_id,
                    // 跟踪器输出
                    .tracker                 = selected_target->tracker,
                });
        });
}

// 关闭fcs::L4命名空间
} // namespace fcs::L4