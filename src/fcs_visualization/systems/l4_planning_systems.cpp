// 业务层头文件：L4目标选中快照、目标选择追踪日志
#include "L4_planning/selected_target_snapshot.hpp"
#include "L4_planning/target_selection_trace.hpp"
// L5火控层：发射控制指令
#include "L5_weapon/fire_control.hpp"
// 基础工具、时间戳、弹道求解资源、Foxglove消息类型、场景构建器
#include "base.hpp"
#include "core/time.hpp"
#include "core/trajectory/resource.hpp"
#include "foxglove_types.hpp"
#include "scene_builder.hpp"
#include "scheduler/scheduler.hpp"
// L4控制意图、通道话题定义、帧结构
#include "L4_planning/control_intent.hpp"
#include "core/channel_topics.hpp"
#include "frame.hpp"

// 第三方工具库
#include <fmt/format.h>         // 高性能字符串格式化，替代sprintf
#include <magic_enum.hpp>       // C++17无宏枚举反射，枚举<->字符串互转
#include <nlohmann/json.hpp>    // JSON序列化/反序列化
#include <system_helpers.hpp>   // 系统底层工具
#include <tactical_palette.hpp> // 战术可视化配色、尺寸常量（目标球体大小、线宽、颜色）

// STL标准库
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

// 顶层命名空间：fcs = FireControlSystem 火控系统
namespace fcs::visualization::foxglove::systems {

// 内部实现细节命名空间，隔离对外接口，仅本文件使用
namespace l4_detail {

/// 结构体：未来装甲预测快照（预留，用于目标装甲未来位姿可视化）
struct FutureArmorSnapshot {
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};          // 装甲中心世界坐标
    Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()}; // 装甲线速度
    double angular_velocity{0.0};                             // 装甲角速度
    std::vector<Eigen::Vector4d> armors;                      // 多装甲位姿数组 (x,y,z,yaw)
};

/// 工具函数：布尔值转 YES/NO 字符串，用于可视化元数据展示
/// [[nodiscard]] 强制接收返回值，防止丢弃格式化结果
[[nodiscard]] inline std::string yes_no(bool value) { return value ? "YES" : "NO"; }

/// 工具函数：拼接追踪目标唯一标识 颜色/装甲名（如 RED/ARMOR_1）
/// 入参：L3追踪输出结构体
[[nodiscard]] inline std::string tracker_identity(const ::fcs::L3::TrackerOutput& target) {
    return fmt::format(
        "{}/{}",
        magic_enum::enum_name(target.target_color), // 枚举转字符串：RED/BLUE
        magic_enum::enum_name(target.target_name)); // 枚举转字符串：ARMOR1/BASE
}

/// 工具函数：拼接候选目标标识（目标选择日志专用）
[[nodiscard]] inline std::string
    selection_identity(const ::fcs::L4::TargetSelectionTraceEntry& entry) {
    return fmt::format(
        "{}/{}", magic_enum::enum_name(entry.target_color),
        magic_enum::enum_name(entry.target_name));
}

/// 工具函数：分数保留3位小数格式化
[[nodiscard]] inline std::string format_score(double value) { return fmt::format("{:.3f}", value); }

/// 工具函数：图像像素坐标格式化；非有限值(NaN/Inf)输出n/a
[[nodiscard]] inline std::string format_image_center_px(double value) {
    return std::isfinite(value) ? fmt::format("{:.1f}", value) : "n/a";
}

/// 工具函数：生成候选目标实体唯一ID，用于Foxglove场景区分不同球体
[[nodiscard]] inline std::string
    selection_trace_entity_id(const ::fcs::L4::TargetSelectionTraceEntry& entry) {
    return fmt::format(
        "selection_candidate_{}_{}", magic_enum::enum_name(entry.target_color),
        magic_enum::enum_name(entry.target_name));
}

/**
 * @brief 给场景实体追加目标选择全维度打分元数据
 * @param builder Foxglove场景实体构建器，用于球体/标签附加信息
 * @param trace 完整目标选择日志帧
 * @param entry 当前单个候选目标打分条目
 * @note 所有打分项、权重、距离、跟踪状态全部存入可视化面板
 */
inline void append_selection_trace_metadata(
    viz::EntityBuilder& builder, const ::fcs::L4::TargetSelectionTrace& trace,
    const ::fcs::L4::TargetSelectionTraceEntry& entry) {
    builder
        .metadata("target_selection.identity", selection_identity(entry)) // 目标身份 RED/ARMOR
        .metadata("target_selection.rank", std::to_string(entry.rank))    // 候选排名 1/2/3...
        .metadata("target_selection.selected", yes_no(entry.selected))    // 是否被选中最优目标
        .metadata("target_selection.runner_up", yes_no(entry.runner_up))  // 是否次优备选
        .metadata(
            "target_selection.was_previously_selected",
            yes_no(entry.was_previously_selected))                        // 上一帧是否选中
        .metadata("target_selection.aim_valid", yes_no(entry.aim_valid))  // 瞄准解是否有效
        .metadata(
            "target_selection.track_status",
            std::string(magic_enum::enum_name(entry.track_status)))       // 跟踪状态枚举
        // 原始单项得分
        .metadata("target_selection.total_score", format_score(entry.total_score))
        .metadata("target_selection.image_center_score", format_score(entry.image_center_score))
        .metadata("target_selection.track_state_score", format_score(entry.track_state_score))
        .metadata("target_selection.tof_score", format_score(entry.tof_score))
        .metadata("target_selection.gimbal_effort_score", format_score(entry.gimbal_effort_score))
        .metadata("target_selection.armor_name_score", format_score(entry.armor_name_score))
        // 加权后单项得分
        .metadata(
            "target_selection.image_center_weighted", format_score(entry.image_center_weighted))
        .metadata("target_selection.track_state_weighted", format_score(entry.track_state_weighted))
        .metadata("target_selection.tof_weighted", format_score(entry.tof_weighted))
        .metadata(
            "target_selection.gimbal_effort_weighted", format_score(entry.gimbal_effort_weighted))
        .metadata("target_selection.armor_name_weighted", format_score(entry.armor_name_weighted))
        .metadata("target_selection.weighted_sum", format_score(entry.weighted_sum)) // 加权总分
        .metadata("target_selection.total_weight", format_score(entry.total_weight)) // 总权重系数
        // 图像像素距离
        .metadata(
            "target_selection.image_center_distance_px",
            format_image_center_px(entry.image_center_distance_px))
        .metadata(
            "target_selection.optical_age_s", format_score(entry.optical_age_s)) // 视觉观测时延
        // 飞行时间、距离（空值输出n/a）
        .metadata(
            "target_selection.tof_s",
            std::isfinite(entry.tof_s) ? format_score(entry.tof_s) : "n/a")
        .metadata(
            "target_selection.distance_m",
            std::isfinite(entry.distance_m) ? format_score(entry.distance_m) : "n/a")
        // 云台转动代价角度
        .metadata(
            "target_selection.yaw_effort_deg",
            std::isfinite(entry.yaw_effort_deg) ? format_score(entry.yaw_effort_deg) : "n/a")
        .metadata(
            "target_selection.pitch_effort_deg",
            std::isfinite(entry.pitch_effort_deg) ? format_score(entry.pitch_effort_deg) : "n/a")
        // 全局选择日志帧信息
        .metadata(
            "target_selection.kept_current_target",
            yes_no(trace.kept_current_target)) // 是否保持上帧目标不切换
        .metadata(
            "target_selection.switch_margin", format_score(trace.switch_margin)) // 切换目标阈值余量
        .metadata(
            "target_selection.previous_target",
            trace.had_previous_target
                ? fmt::format(
                      "{}/{}", magic_enum::enum_name(trace.previous_target_color),
                      magic_enum::enum_name(trace.previous_target_name))
                : "none")                                        // 上一帧选中目标，无则填none
        .metadata(
            "target_selection.aim_error",
            entry.aim_error.empty() ? "none" : entry.aim_error); // 瞄准失败原因

    // 存在目标世界坐标时追加三维坐标元数据
    if (entry.target_center) {
        builder.metadata(
            "target_selection.target_center",
            fmt::format(
                "{:.3f}, {:.3f}, {:.3f}", entry.target_center->x(), entry.target_center->y(),
                entry.target_center->z()));
    }
}

/// 工具函数：装甲ID文本标签 Armor 0 / Armor 1
[[nodiscard]] inline std::string armor_id_label(int armor_id) {
    return fmt::format("Armor {}", armor_id);
}

/// 工具函数：生成装甲朝向四元数
/// @param yaw 装甲水平偏航角
/// @return 绕Z轴yaw + 绕Y轴固定装甲倾斜角 的旋转四元数
[[nodiscard]] inline Eigen::Quaterniond armor_orientation(double yaw) {
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(tac::Geometry::ARMOR_TILT_ANGLE, Eigen::Vector3d::UnitY());
}

/// 工具函数：弹道关键点标签 名称+距离(m)
[[nodiscard]] inline std::string
    key_point_label(const std::string& name, const ::fcs::L5::TrajectoryPlanSample& sample) {
    return fmt::format("{} {:.3f}", name, sample.distance);
}

/// 工具函数：Eigen三维向量转为JSON数组 [x,y,z]
[[nodiscard]] inline nlohmann::json json_vec3(const Eigen::Vector3d& value) {
    return {value.x(), value.y(), value.z()};
}

/**
 * @brief 滤波器收敛状态序列化JSON
 * @param state 卡尔曼滤波收敛指标结构体
 * @return JSON对象，异常数值填null方便前端判断
 */
[[nodiscard]] inline nlohmann::json
    json_filter_convergence(const ::fcs::L3::FilterConvergenceState& state) {
    return {
        {                       "status",                       std::string(magic_enum::enum_name(state.status))}, // 收敛/发散/未初始化状态
        {"normalized_innovation_squared", std::isfinite(state.normalized_innovation_squared)
 ? nlohmann::json(state.normalized_innovation_squared)
 : nlohmann::json(nullptr)                                             }, // 归一化新息平方
        {          "max_covariance_diag",           std::isfinite(state.max_covariance_diag)
           ? nlohmann::json(state.max_covariance_diag)
           : nlohmann::json(nullptr)                                   }, // 协方差矩阵最大对角线（不确定性）
        {"consecutive_converged_updates",                                    state.consecutive_converged_updates}, // 连续收敛帧数
        { "consecutive_diverged_updates",                                     state.consecutive_diverged_updates}, // 连续发散帧数
    };
}

/**
 * @brief 敌方机器人目标状态序列化为JSON
 * @param state 机器人跟踪状态（车底盘+多装甲）
 * @return 包含位姿、速度、装甲点位、收敛滤波信息的JSON
 */
[[nodiscard]] inline nlohmann::json json_robot_state(const ::fcs::L3::RobotTargetState& state) {
    nlohmann::json state_json = {
        {       "type",                                    "robot"},
        {   "position",                  json_vec3(state.position)}, // 底盘中心坐标
        {   "distance",                      state.position.norm()}, // 相对我方距离
        {   "velocity",                  json_vec3(state.velocity)}, // 底盘三维速度
        {        "yaw",                                  state.yaw}, // 底盘偏航角
        {      "v_yaw",                                state.v_yaw}, // 底盘旋转角速度
        {     "radius",             {state.radius0, state.radius1}}, // 机器人长宽半径
        {         "z1",                                   state.z1}, // 底盘高度偏移
        { "armors_num",                           state.armors_num}, // 装甲总数量
        {"convergence", json_filter_convergence(state.convergence)}, // 滤波收敛状态
        {"armor_poses",                    nlohmann::json::array()}, // 装甲点位数组
    };

    // 遍历所有装甲，写入装甲ID、三维坐标、偏航角
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

/**
 * @brief 前哨站/基地目标状态序列化JSON
 * @param state 前哨静态目标跟踪结构体
 * @return JSON，结构同机器人，固定装甲数量与半径
 */
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

/**
 * @brief L3追踪输出完整序列化
 * @param output 一帧跟踪结果，支持机器人/前哨两种目标类型
 * @return JSON：时间戳、目标颜色名称、跟踪状态、目标实体状态
 */
[[nodiscard]] inline nlohmann::json json_tracker_output(const ::fcs::L3::TrackerOutput& output) {
    nlohmann::json target_json = {
        {                 "timestamp_ns",           output.timestamp_ns                                         },
        {                       "status",                      std::string(magic_enum::enum_name(output.status))},
        {                  "target_name",                 std::string(magic_enum::enum_name(output.target_name))},
        {                 "target_color",                std::string(magic_enum::enum_name(output.target_color))},
        {                "target_jumped",                                                   output.target_jumped}, // 目标跳变（跟踪丢失重捕获）
        {                "last_armor_id",
         output.last_armor_id ? nlohmann::json(*output.last_armor_id) : nlohmann::json(nullptr)                 },
        {"last_image_center_distance_px", std::isfinite(output.last_image_center_distance_px)
 ? nlohmann::json(output.last_image_center_distance_px)
 : nlohmann::json(nullptr)                                             },
        {"last_observation_timestamp_ns",                                   output.last_observation_timestamp_ns},
        {                   "state_kind",                                                                "empty"}, // 标记目标类型 robot/outpost/空
    };

    // 多态分支：判断当前跟踪的是机器人还是前哨站，注入对应状态JSON
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

/**
 * @brief 单个候选目标打分条目序列化JSON
 * @param entry 目标选择器单条候选打分记录
 * @return 全量打分维度JSON，前端可用于表格展示排名
 */
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

/**
 * @brief 完整一帧目标选择日志序列化
 * @param trace 整帧目标选择器输出（包含全部候选目标数组）
 * @return JSON：帧时间戳、上帧目标、是否保持目标、候选数组
 */
[[nodiscard]] inline nlohmann::json
    json_target_selection_trace(const ::fcs::L4::TargetSelectionTrace& trace) {
    nlohmann::json candidates = nlohmann::json::array();
    // 遍历所有候选目标，序列化为子对象存入数组
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

/**
 * @brief L4控制意图variant序列化
 * @param intent std::variant<TrackCommand, ShotCommand, HoldCommand> 多态控制指令
 * @return JSON，通过std::visit编译期分支匹配不同指令类型
 * @note std::visit + if constexpr 编译期多态分发，无运行时开销
 */
[[nodiscard]] inline nlohmann::json json_control_intent(const ::fcs::L4::ControlIntent& intent) {
    nlohmann::json result;
    // 访问variant，编译期匹配内部存储的指令类型
    std::visit(
        [&](const auto& cmd) {
            using T                = std::decay_t<decltype(cmd)>; // 获取variant内部真实类型
            result["timestamp_ns"] = cmd.timestamp_ns;
            if constexpr (std::is_same_v<T, ::fcs::L4::TrackCommand>) {
                // 跟踪模式：输出MPC控制时域、发射时域长度
                result["mode"]            = "Track";
                result["control_horizon"] = cmd.control_trajectory.horizon();
                result["fire_horizon"]    = cmd.fire_trajectory.horizon();
            } else if constexpr (std::is_same_v<T, ::fcs::L4::ShotCommand>) {
                // 射击模式：输出瞄准角度、距离、退化原因
                result["mode"]     = "Shot";
                result["yaw"]      = cmd.yaw;
                result["pitch"]    = cmd.pitch;
                result["distance"] = cmd.distance;
                if (cmd.degradation_reason) {
                    result["degradation_reason"] = *cmd.degradation_reason;
                }
            } else if constexpr (std::is_same_v<T, ::fcs::L4::HoldCommand>) {
                // 保持云台不动模式
                result["mode"] = "Hold";
            }
        },
        intent);
    return result;
}

/**
 * @brief 选中目标快照序列化（Foxglove最优目标主消息）
 * @param snapshot L4输出的当前最优选中目标完整快照
 * @return JSON 融合L3跟踪输出 + L4规划层附加预测、瞄准相位信息
 */
[[nodiscard]] inline nlohmann::json
    json_solver_target(const ::fcs::L4::SelectedTargetSnapshot& snapshot) {
    // 复用L3跟踪序列化基础字段
    nlohmann::json target_json = json_tracker_output(snapshot.tracker);
    // 追加L4规划层专属字段
    target_json["timestamp_ns"]      = snapshot.timestamp_ns;
    target_json["timestamp"]         = nlohmann::json::object();
    target_json["timestamp"]["sec"]  = snapshot.timestamp_ns / 1000000000L;      // 纳秒转秒
    target_json["timestamp"]["nsec"] = snapshot.timestamp_ns % 1000000000L;      // 剩余纳秒
    target_json["tracking"]          = snapshot.tracker.is_tracking();           // 是否稳定跟踪
    target_json["valid"]             = snapshot.has_target();                    // 是否存在有效目标
    target_json["optimal_target"]    = target_json["valid"];
    target_json["source"] = std::string(magic_enum::enum_name(snapshot.source)); // 目标来源枚举
    target_json["plan_distance"]       = snapshot.distance;                      // 规划瞄准距离
    target_json["predicted_future_ns"] = snapshot.predicted_future_ns;           // 向前预测时长
    target_json["aim_phase"] = std::string(magic_enum::enum_name(snapshot.aim_phase)); // 瞄准阶段
    target_json["selected_armor_id"]       = snapshot.selected_armor_id;       // 当前最优装甲ID
    target_json["rough_selected_armor_id"] = snapshot.rough_selected_armor_id; // 粗选装甲ID

    return target_json;
}

/**
 * @brief 给选中目标3D实体附加基础元数据
 * @param builder Foxglove场景构建器
 * @param snapshot L4选中目标快照
 */
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

/**
 * @brief 给MPC/云台场景实体附加规划层元数据（控制模式、退化原因、选中目标信息）
 * @param builder 场景构建器
 * @param intent L4控制意图variant
 * @param selected_target 可选：当前选中目标快照指针，空则不追加目标信息
 */
inline void append_plan_metadata(
    viz::EntityBuilder& builder, const ::fcs::L4::ControlIntent& intent,
    const ::fcs::L4::SelectedTargetSnapshot* selected_target = nullptr) {
    // std::visit编译期匹配控制模式 Track/Shot/Hold
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

    // 仅射击模式存在瞄准退化原因，写入元数据
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

    // 传入有效目标快照时追加目标身份元数据
    if (selected_target) {
        append_selected_target_metadata(builder, *selected_target);
    }
}

} // namespace l4_detail

/**
 * @brief 注册L4规划层所有Foxglove可视化系统任务
 * @param app Talos全局调度器实例
 * @detail 注册6个计算任务：
 * 1. foxglove_solver_target_pub：发布当前最优选中目标JSON消息
 * 2. foxglove_target_selection_pub：发布全候选目标打分日志JSON
 * 3. foxglove_l4_gimbal_cmd_pub：发布L5云台执行指令JSON
 * 4. foxglove_l4_gimbal_scene：渲染候选目标球体3D场景
 * 5. foxglove_l4_mpc_trajectory_pub：发布MPC参考/优化轨迹数组JSON
 * 6. foxglove_l4_mpc_prediction_scene：250Hz定时渲染MPC预测轨迹+子弹弹道
 */
void register_l4_planning_systems(talos::scheduler::Scheduler& app) {
    // ========== 任务1：发布当前最优选中目标JSON ==========
    app.add_system<talos::pool_compute>(
        "foxglove_solver_target_pub",
        [](talos::spmc<::fcs::L4::SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic>
               selected_target_in, // SPMC多生产者单消费者通道：L4输出选中目标
           talos::res<std::shared_ptr<FoxgloveServer>> server) { // Foxglove服务器资源句柄
            // Foxglove服务未就绪直接退出
            if (!detail::foxglove_ready(*server, selected_target_in)) {
                return;
            }
            // 读取通道最新帧快照
            auto selected_target = selected_target_in.read();
            // 无有效目标快照跳过发布
            if (!selected_target) {
                return;
            }
            // 序列化并发布TargetMessage类型JSON消息
            detail::publish_json_message<TargetMessage>(
                *server, l4_detail::json_solver_target(*selected_target));
        });

    // ========== 任务2：发布目标选择全候选打分日志 ==========
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
    // 任务3：云台执行指令JSON发布（L5 WeaponCommand）
    // =========================================================================
    app.add_system<talos::pool_compute>(
        "foxglove_l4_gimbal_cmd_pub",
        [](talos::spmc<::fcs::L5::WeaponCommand, WeaponCommandChannelTopic> cmd_in,
           talos::spmc<::fcs::L4::ControlIntent, ControlIntentChannelTopic> plan_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, cmd_in)) {
                return;
            }
            // 读取最新云台执行指令
            auto cmd = cmd_in.read();
            // 无效指令（距离<0）跳过
            if (!cmd || cmd->distance < 0) {
                return;
            }
            // 读取当前L4规划控制意图快照
            const auto plan_snapshot = plan_in.read_current();
            // 匹配时间戳：仅保留和云台指令同一规划帧的控制意图
            const auto* plan =
                (plan_snapshot
                 && std::visit([](const auto& c) { return c.timestamp_ns; }, *plan_snapshot)
                        == cmd->plan_timestamp_ns)
                    ? std::addressof(*plan_snapshot)
                    : nullptr;

            // 构建云台指令JSON
            nlohmann::json cmd_json;
            cmd_json["timestamp_ns"]      = cmd->timestamp_ns;
            cmd_json["plan_timestamp_ns"] = cmd->plan_timestamp_ns;
            cmd_json["timestamp"]         = nlohmann::json::object();
            cmd_json["timestamp"]["sec"]  = cmd->timestamp_ns / 1000000000L;
            cmd_json["timestamp"]["nsec"] = cmd->timestamp_ns % 1000000000L;
            // 规划前原始目标角度
            cmd_json["pre_plan"] = {
                {     "yaw",      cmd->plan_yaw},
                {   "pitch",    cmd->plan_pitch},
                {"distance", cmd->plan_distance},
            };
            // MPC优化后输出云台指令（带开火建议）
            cmd_json["post_plan"] = {
                {        "yaw",   cmd->yaw},
                {      "pitch", cmd->pitch},
                {"fire_advice",  cmd->fire},
            };
            // 云台运动状态：角度、角速度、角加速度
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
            // 瞄准退化原因（如遮挡、滤波发散）
            if (cmd->degradation_reason) {
                cmd_json["degradation_reason"] = *cmd->degradation_reason;
            }
            // MPC调试信息：参考/优化轨迹长度、中心索引
            if (cmd->viz_debug) {
                cmd_json["mpc"] = {
                    {   "center_index",          cmd->viz_debug->center_index},
                    {"lookahead_index",       cmd->viz_debug->lookahead_index},
                    { "reference_size", cmd->viz_debug->reference_plan.size()},
                    { "optimized_size", cmd->viz_debug->optimized_plan.size()},
                };
            }
            // 匹配到对应L4规划意图时注入控制模式JSON
            if (plan) {
                cmd_json["l4_plan"] = l4_detail::json_control_intent(*plan);
            }
            // 发布云台指令JSON消息
            detail::publish_json_message<GimbalCmdMessage>(*server, cmd_json);
        });

    // ========== 任务4：MPC参考/优化轨迹数组JSON发布 ==========
    app.add_system<talos::pool_compute>(
        "foxglove_l4_mpc_trajectory_pub",
        [](talos::spmc<::fcs::L5::WeaponCommand, WeaponCommandChannelTopic> cmd_in,
           talos::spmc<::fcs::L4::ControlIntent, ControlIntentChannelTopic> plan_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, cmd_in)) {
                return;
            }
            auto cmd = cmd_in.read();
            // 无MPC调试数据直接跳过
            if (!cmd || !cmd->viz_debug) {
                return;
            }
            // 匹配同时间戳L4规划意图
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
            traj_json["reference"]         = nlohmann::json::array(); // MPC参考轨迹（原始目标）
            traj_json["optimized"]         = nlohmann::json::array(); // MPC优化平滑轨迹

            // 填充参考轨迹每一个时域点
            for (int i = 0; i < static_cast<int>(cmd->viz_debug->reference_plan.size()); ++i) {
                const auto& reference = cmd->viz_debug->reference_plan[i];
                traj_json["reference"].push_back({
                    {          "index",                                i},
                    {"temporal_offset", i - cmd->viz_debug->center_index}, // 相对当前帧时域偏移
                    {            "yaw",                    reference.yaw},
                    {          "pitch",                  reference.pitch},
                    {       "distance",               reference.distance},
                    {            "tof",                    reference.tof},
                });
            }
            // 填充MPC优化后平滑轨迹
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
            // 附加L4控制意图信息
            if (plan) {
                traj_json["l4_plan"] = l4_detail::json_control_intent(*plan);
            }
            detail::publish_json_message<MpcTrajectoryMessage>(*server, traj_json);
        });

    // =========================================================================
    // 任务5：L4目标选择候选球体3D场景渲染（事件驱动，仅非Hold模式执行）
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
            // 保持云台模式不渲染目标场景
            if (!intent || std::holds_alternative<::fcs::L4::HoldCommand>(*intent))
                return;
            // 提取当前规划帧时间戳，用于多通道数据时间对齐
            const uint64_t plan_ts =
                std::visit([](const auto& cmd) -> uint64_t { return cmd.timestamp_ns; }, *intent);
            // 读取同时间戳选中目标快照、目标选择日志
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
            // 存在目标选择日志时遍历所有候选目标生成球体实体
            if (trace) {
                for (const auto& candidate : trace->candidates) {
                    // 无三维坐标跳过渲染
                    if (!candidate.target_center) {
                        continue;
                    }
                    // 根据排名分级：最优选中 / 次优备选 / 淘汰候选
                    const auto tier = candidate.selected  ? tac::SelectionTier::Selected
                                    : candidate.runner_up ? tac::SelectionTier::RunnerUp
                                                          : tac::SelectionTier::Eliminated;
                    // 获取对应层级配色、球体缩放、透明度、标签显示开关
                    const auto style = tac::selection_style(tier);

                    // 构建Foxglove球体实体：odom坐标系、唯一ID、坐标、大小、颜色
                    auto builder = viz::EntityBuilder::create<fast_tf::odom>(
                                       "l4", l4_detail::selection_trace_entity_id(candidate))
                                       .timestamp(trace->timestamp_ns)
                                       .position(*candidate.target_center)
                                       .size(tac::L4::SELECTION_SIZE * style.size_scale)
                                       .color(style.color)
                                       .alpha(style.alpha)
                                       .sphere();
                    // 开启标签时追加文字标注 #1 RED/ARMOR*
                    if (style.show_label) {
                        builder.text_with_offset(
                            fmt::format(
                                "#{}{} {}", candidate.rank, candidate.selected ? "*" : "",
                                l4_detail::selection_identity(candidate)),
                            tac::L4::SELECTION_SIZE * 1.5, 0.0, tac::L3::LABEL_OFFSET_Z,
                            tac::Text::SIZE_SMALL);
                    }
                    // 附加全打分元数据到实体，鼠标悬浮Foxglove可查看
                    l4_detail::append_selection_trace_metadata(builder, *trace, candidate);
                    entities.push_back(builder.build());
                }
            }
            // 实体数组非空则发布3D场景消息
            publish_scene_if_nonempty<GimbalSceneMessage>(*server, std::move(entities));
        });

    // =========================================================================
    // 任务6：250Hz定时MPC预测轨迹+子弹弹道渲染
    // =========================================================================
    app.add_system<talos::fixed_rate<250>>(
        "foxglove_l4_mpc_prediction_scene",
        [](talos::spmc<::fcs::L5::WeaponCommand, WeaponCommandChannelTopic> cmd_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server,
           core::trajectory::trajectory_solver solver,    // 弹道求解器资源
           talos::res<fast_tf::CoordinateSystem> coord,   // TF坐标变换资源
           core::trajectory::bullet_speed bullet_speed) { // 子弹初速资源
            if (!detail::foxglove_ready(*server, cmd_in)) {
                return;
            }
            std::vector<::foxglove::schemas::SceneEntity> entities;
            auto cmd = cmd_in.read();
            // 存在有效MPC调试数据：渲染MPC参考/优化轨迹线、关键点
            if (cmd && cmd->viz_debug) {
                entities.reserve(2);
                // 1. 绘制MPC参考轨迹连线（原始目标时域序列）
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
                // 2. 绘制MPC优化平滑轨迹连线
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
                // 约束中心索引在合法区间
                const int center_index = std::clamp(
                    cmd->viz_debug->center_index, 0,
                    static_cast<int>(cmd->viz_debug->optimized_plan.size()) - 1);
                // 内部lambda：复用代码生成轨迹关键点球体+文字标签
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
                // 绘制参考轨迹当前时域中心点
                add_key_marker(
                    "mpc_ref_center",
                    cmd->viz_debug->reference_plan[static_cast<size_t>(center_index)],
                    l4_detail::key_point_label(
                        "Ref", cmd->viz_debug->reference_plan[static_cast<size_t>(center_index)]),
                    tac::L4::MPC_REFERENCE, tac::L4::SELECTION_SIZE);
                // 绘制优化轨迹当前时域中心点
                add_key_marker(
                    "mpc_opt_center",
                    cmd->viz_debug->optimized_plan[static_cast<size_t>(center_index)],
                    l4_detail::key_point_label(
                        "Opt", cmd->viz_debug->optimized_plan[static_cast<size_t>(center_index)]),
                    tac::L4::MPC_PRESENT, tac::L4::SELECTION_SIZE);
                // 根据当前云台俯仰、距离求解真实子弹抛物线弹道
                auto trajectory = solver->get()->generate_trajectory(
                    cmd->pitch, bullet_speed->bullet_speed, cmd->distance);
                std::vector<Eigen::Vector3d> traj_points;
                traj_points.reserve(trajectory.size());
                // 弹道点坐标系转换
                for (const auto& [x, z] : trajectory) {
                    traj_points.emplace_back(
                        x * std::cos(cmd->pitch) + z * std::sin(cmd->pitch), 0,
                        -x * std::sin(cmd->pitch) + z * std::cos(cmd->pitch));
                }
                // 生成弹道线实体并入场景数组
                auto traj_entities =
                    viz::patterns::bullet_trajectory(traj_points, cmd->fire, cmd->timestamp_ns);
                entities.insert(
                    entities.end(), std::make_move_iterator(traj_entities.begin()),
                    std::make_move_iterator(traj_entities.end()));
            } else {
                // 无有效MPC调试数据：绘制当前云台俯仰的默认弹道（15m测试距离）
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
            // 发布MPC预测3D场景
            publish_scene_if_nonempty<MpcPredictionSceneMessage>(*server, std::move(entities));
        });
}
} // namespace fcs::visualization::foxglove::systems