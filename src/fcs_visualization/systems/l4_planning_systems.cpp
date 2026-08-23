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
        // 当前目标选择决策的时间戳(纳秒)，和图像帧/跟踪帧时间对齐，Foxglove时序回放用
        {         "timestamp_ns",                                              trace.timestamp_ns},
        // 是否存在上一帧有效锁定目标 true=有旧目标；false=之前无目标，首次进入跟踪
        {  "had_previous_target",                                       trace.had_previous_target},
        // 上一帧锁定的目标装甲编号枚举，magic_enum转字符串，例如 "ARMOR_0"
        { "previous_target_name",  std::string(magic_enum::enum_name(trace.previous_target_name))},
        // 上一帧目标颜色："Red" / "Blue"
        {"previous_target_color", std::string(magic_enum::enum_name(trace.previous_target_color))},
        // 是否保留继续沿用当前目标，true：不切换目标；false：发生目标切换
        {  "kept_current_target",                                       trace.kept_current_target},
        // 目标切换裕度阈值，无人车决策器参数：新目标得分需要比旧目标高出该值才允许切目标，防止频繁抖动跳目标
        {        "switch_margin",                                             trace.switch_margin},
        // 候选目标数组，std::move转移，避免拷贝；数组每一项包含：候选id、得分、距离、代价、是否允许被选中等调试信息
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
    target_json["optimal_target"]    = target_json["valid"];                        // 是否存在有效目标
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
    // =========================================================================
// 任务1：发布【L4选中目标快照】JSON消息到Foxglove
// 作用：可视化当前火控最终选中的目标（id、装甲、距离、状态）
// =========================================================================
app.add_system<talos::pool_compute>(
        "foxglove_solver_target_pub",
        [](// spmc输入通道：L4输出的选中目标快照，其他模块写，本system只读
           talos::spmc<::fcs::L4::SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic> selected_target_in,
           // res只读资源：Foxglove服务实例，websocket服务端
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            // 前置检查：服务指针有效、通道可读；不满足直接return，不执行可视化
            if (!detail::foxglove_ready(*server, selected_target_in)) {
                return;
            }
            // read()：读取SPMC通道最新一帧快照；无新数据返回std::nullopt
            auto selected_target = selected_target_in.read();
            // 没有选中目标，跳过本次发布
            if (!selected_target) {
                return;
            }
            // 序列化：把SelectedTargetSnapshot结构体转nlohmann::json，通过websocket发给Foxglove
            detail::publish_json_message<TargetMessage>(
                *server, l4_detail::json_solver_target(*selected_target));
        });

// ========== 任务2：发布【目标选择全候选打分日志】JSON ==========
// 用途：调试目标选择决策器，看每帧所有候选目标的代价、打分、排名，是谁被淘汰、谁是备选
app.add_system<talos::pool_compute>(
        "foxglove_target_selection_pub",
        [](// SPMC通道：TargetSelectionTrace，每帧目标选择完整trace日志
           talos::spmc<::fcs::L4::TargetSelectionTrace, TargetSelectionTraceChannelTopic> trace_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, trace_in)) {
                return;
            }
            // 读取最新一帧选择trace
            auto trace = trace_in.read();
            // 当前帧无trace数据直接跳过
            if (!trace) {
                return;
            }
            // trace序列化为json，Foxglove表格面板查看全部候选打分
            detail::publish_json_message<TargetSelectionTraceMessage>(
                *server, l4_detail::json_target_selection_trace(*trace));
        });

// =========================================================================
// 任务3：发布【L5云台执行指令WeaponCommand】JSON
// 核心：MPC输出最终云台指令、速度加速度、开火建议、MPC调试信息、退化原因
// 输入两个SPMC：WeaponCommand(MPC输出执行指令) + ControlIntent(L4上层规划意图)
// 做时间戳对齐，把同一帧的L4规划意图附加到消息中方便比对
// =========================================================================
app.add_system<talos::pool_compute>(
        "foxglove_l4_gimbal_cmd_pub",
        [](// 输入1：L5 MPC输出最终云台执行指令
           talos::spmc<::fcs::L5::WeaponCommand, WeaponCommandChannelTopic> cmd_in,
           // 输入2：L4输出控制意图（目标选择、期望瞄准角度）
           talos::spmc<::fcs::L4::ControlIntent, ControlIntentChannelTopic> plan_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, cmd_in)) {
                return;
            }
            // 读取MPC输出最新云台指令
            auto cmd = cmd_in.read();
            // 无效指令：空快照 或者距离小于0，直接丢弃
            if (!cmd || cmd->distance < 0) {
                return;
            }
            // read_current()：读取通道当前快照，后续做时间戳匹配
            const auto plan_snapshot = plan_in.read_current();
            /*
             * std::visit 变体访问：ControlIntent是std::variant多态类型
             * 提取variant内部结构体的timestamp_ns
             * 判断：L4规划帧时间戳 和 MPC指令的plan_timestamp_ns是否完全匹配
             * 匹配成功则拿到plan指针，否则plan=nullptr，不附加L4规划数据
             * 目的：保证json里面的l4_plan和mpc指令属于同一计算帧，避免跨帧错位
             */
            const auto* plan =
                (plan_snapshot
                 && std::visit([](const auto& c) { return c.timestamp_ns; }, *plan_snapshot)
                        == cmd->plan_timestamp_ns)
                    ? std::addressof(*plan_snapshot)
                    : nullptr;

            // 手动组装json，导出MPC全部调试字段
            nlohmann::json cmd_json;
            cmd_json["timestamp_ns"]      = cmd->timestamp_ns;        // 本指令生成时间戳(ns)
            cmd_json["plan_timestamp_ns"] = cmd->plan_timestamp_ns;    // 对应的上层L4规划帧时间戳
            cmd_json["timestamp"]         = nlohmann::json::object();
            cmd_json["timestamp"]["sec"]  = cmd->timestamp_ns / 1000000000L; // 秒部分
            cmd_json["timestamp"]["nsec"] = cmd->timestamp_ns % 1000000000L;// 纳秒余数

            // pre_plan：MPC优化之前，L4给出原始期望瞄准角度
            cmd_json["pre_plan"] = {
                {     "yaw",      cmd->plan_yaw},      // 原始期望偏航
                {   "pitch",    cmd->plan_pitch},      // 原始期望俯仰
                {"distance", cmd->plan_distance},      // 原始目标距离
            };
            // post_plan：MPC优化平滑之后输出云台目标角度，附带开火建议
            cmd_json["post_plan"] = {
                {        "yaw",   cmd->yaw},
                {      "pitch", cmd->pitch},
                {"fire_advice",  cmd->fire},          // true=建议开火
            };

            // MPC输出完整云台状态
            cmd_json["yaw"]                  = cmd->yaw;               // 输出目标偏航角(rad)
            cmd_json["pitch"]                = cmd->pitch;             // 输出目标俯仰角(rad)
            cmd_json["v_yaw"]                = cmd->yaw_vel;           // 目标偏航角速度 rad/s
            cmd_json["v_pitch"]              = cmd->pitch_vel;         // 目标俯仰角速度 rad/s
            cmd_json["a_yaw"]                = cmd->yaw_accel;         // 目标偏航角加速度 rad/s²
            cmd_json["a_pitch"]              = cmd->pitch_accel;      // 目标俯仰角加速度 rad/s²
            cmd_json["distance"]             = cmd->distance;          // 预测目标距离
            cmd_json["tof"]                  = cmd->tof;               // 子弹飞行时间time‑of‑flight
            cmd_json["fire_advice"]          = cmd->fire;               // 是否建议开火
            cmd_json["yaw_error"]            = cmd->yaw_error;         // 偏航跟踪误差
            cmd_json["pitch_error"]          = cmd->pitch_error;       // 俯仰跟踪误差
            cmd_json["shooting_range_yaw"]   = cmd->shooting_range_yaw;// 允许开火偏航误差阈值
            cmd_json["shooting_range_pitch"] = cmd->shooting_range_pitch;//允许开火俯仰误差阈值
            cmd_json["ref_yaw"]              = cmd->ref_yaw;          // MPC参考轨迹yaw
            cmd_json["ref_pitch"]            = cmd->ref_pitch;         // MPC参考轨迹pitch

            // degradation_reason std::optional<std::string>；瞄准退化原因（遮挡、滤波发散、目标丢失）
            if (cmd->degradation_reason) {
                cmd_json["degradation_reason"] = *cmd->degradation_reason;
            }

            // viz_debug 可选MPC调试结构体，存参考轨迹、优化轨迹数组
            if (cmd->viz_debug) {
                cmd_json["mpc"] = {
                    {   "center_index",          cmd->viz_debug->center_index},   // 当前时刻在时域窗口的下标
                    {"lookahead_index",       cmd->viz_debug->lookahead_index},  // 前向预测终点下标
                    { "reference_size", cmd->viz_debug->reference_plan.size()}, // MPC参考轨迹点数量
                    { "optimized_size", cmd->viz_debug->optimized_plan.size()}, // MPC优化轨迹点数量
                };
            }

            // 如果时间戳匹配到L4规划意图，把ControlIntent序列化塞进json
            if (plan) {
                cmd_json["l4_plan"] = l4_detail::json_control_intent(*plan);
            }
            // 通过websocket发送云台指令json消息
            detail::publish_json_message<GimbalCmdMessage>(*server, cmd_json);
        });

// ========== 任务4：MPC参考/优化轨迹数组JSON发布 ==========
// 把MPC时域窗口每一个时间采样点（yaw/pitch/distance/tof）输出json，Foxglove表格查看整条预测时域序列
app.add_system<talos::pool_compute>(
        "foxglove_l4_mpc_trajectory_pub",
        [](talos::spmc<::fcs::L5::WeaponCommand, WeaponCommandChannelTopic> cmd_in,
           talos::spmc<::fcs::L4::ControlIntent, ControlIntentChannelTopic> plan_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, cmd_in)) {
                return;
            }
            auto cmd = cmd_in.read();
            // 没有MPC调试数据直接跳过
            if (!cmd || !cmd->viz_debug) {
                return;
            }
            // 和任务3完全一样：时间戳对齐L4上层规划意图
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
            traj_json["reference"]         = nlohmann::json::array(); // MPC参考轨迹数组（原始目标序列）
            traj_json["optimized"]         = nlohmann::json::array(); // MPC平滑优化后轨迹数组

            // 遍历参考轨迹所有时域采样点
            for (int i = 0; i < static_cast<int>(cmd->viz_debug->reference_plan.size()); ++i) {
                const auto& reference = cmd->viz_debug->reference_plan[i];
                traj_json["reference"].push_back({
                    {          "index",                                i},
                    {"temporal_offset", i - cmd->viz_debug->center_index}, // 相对当前时刻时域偏移，0=当前，正数未来，负数过去
                    {            "yaw",                    reference.yaw},
                    {          "pitch",                  reference.pitch},
                    {       "distance",               reference.distance},
                    {            "tof",                    reference.tof},
                });
            }
            // 遍历MPC优化之后的轨迹点
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
            // 附加对齐后的L4规划意图
            if (plan) {
                traj_json["l4_plan"] = l4_detail::json_control_intent(*plan);
            }
            detail::publish_json_message<MpcTrajectoryMessage>(*server, traj_json);
        });

// =========================================================================
// 任务5：L4目标选择候选球体3D场景渲染
// 功能：Foxglove 3D视图绘制所有候选目标球体
// 颜色区分：选中目标(主选) / runner_up次优备选 / eliminated淘汰候选
// 球体附带悬浮元数据：代价、rank，鼠标hover看打分；附带文本标签#1 RED_ARMOR
// 约束：云台Hold保持模式不渲染场景
// =========================================================================
app.add_system<talos::pool_compute>(
        "foxglove_l4_gimbal_scene",
        [](// L4控制意图通道
           talos::spmc<::fcs::L4::ControlIntent, ControlIntentChannelTopic> plan_in,
           // L4选中目标快照通道
           talos::spmc<::fcs::L4::SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic>
               selected_target_in,
           // 目标选择trace打分日志通道
           talos::spmc<::fcs::L4::TargetSelectionTrace, TargetSelectionTraceChannelTopic> trace_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server) {
            if (!detail::foxglove_ready(*server, plan_in))
                return;
            auto intent = plan_in.read();
            // std::holds_alternative 判断variant当前类型：如果是HoldCommand（云台保持模式）直接返回，不渲染
            if (!intent || std::holds_alternative<::fcs::L4::HoldCommand>(*intent))
                return;

            // 提取本帧L4规划的时间戳，用来多通道时间对齐
            const uint64_t plan_ts =
                std::visit([](const auto& cmd) -> uint64_t { return cmd.timestamp_ns; }, *intent);

            // 读取同时间戳的选中目标快照
            const auto selected_target_snapshot = selected_target_in.read_current();
            [[maybe_unused]] const auto* selected_target =
                (selected_target_snapshot && selected_target_snapshot->timestamp_ns == plan_ts)
                    ? std::addressof(*selected_target_snapshot)
                    : nullptr;

            // 读取同时间戳目标选择trace日志
            const auto trace_snapshot = trace_in.read_current();
            const auto* trace         = (trace_snapshot && trace_snapshot->timestamp_ns == plan_ts)
                                          ? std::addressof(*trace_snapshot)
                                          : nullptr;

            std::vector<::foxglove::schemas::SceneEntity> entities;
            // 如果存在trace候选日志，逐个生成球体实体
            if (trace) {
                for (const auto& candidate : trace->candidates) {
                    // 候选目标没有三维世界坐标，跳过渲染
                    if (!candidate.target_center) {
                        continue;
                    }
                    /*
                     * tier 候选等级枚举
                     * Selected：本帧最终选中目标
                     * RunnerUp：次优备选目标
                     * Eliminated：被淘汰候选
                     */
                    const auto tier = candidate.selected  ? tac::SelectionTier::Selected
                                    : candidate.runner_up ? tac::SelectionTier::RunnerUp
                                                          : tac::SelectionTier::Eliminated;
                    // 根据等级获取渲染样式：颜色、缩放系数、透明度、是否显示标签
                    const auto style = tac::selection_style(tier);

                    // EntityBuilder构建Foxglove球体实体，坐标系odom
                    auto builder = viz::EntityBuilder::create<fast_tf::odom>(
                                       "l4", l4_detail::selection_trace_entity_id(candidate))
                                       .timestamp(trace->timestamp_ns)
                                       .position(*candidate.target_center)  // 球体世界坐标
                                       .size(tac::L4::SELECTION_SIZE * style.size_scale) //球体半径
                                       .color(style.color)
                                       .alpha(style.alpha)
                                       .sphere(); // 实体类型球体

                    // 如果样式配置开启标签，追加文本悬浮标签
                    if (style.show_label) {
                        builder.text_with_offset(
                            fmt::format(
                                "#{}{} {}", candidate.rank, candidate.selected ? "*" : "",
                                l4_detail::selection_identity(candidate)),
                            tac::L4::SELECTION_SIZE * 1.5, 0.0, tac::L3::LABEL_OFFSET_Z,
                            tac::Text::SIZE_SMALL);
                    }
                    // 把候选全部打分元数据附加到SceneEntity，Foxglove鼠标悬浮可以看到所有cost字段
                    l4_detail::append_selection_trace_metadata(builder, *trace, candidate);
                    entities.push_back(builder.build());
                }
            }
            // 实体数组不为空，发送3D场景websocket消息；空就跳过避免无效消息
            publish_scene_if_nonempty<GimbalSceneMessage>(*server, std::move(entities));
        });

// =========================================================================
// 任务6：250Hz定时MPC预测轨迹+子弹弹道渲染
// talos::fixed_rate<250>：固定250Hz周期执行，**不等待消息触发**
// 两件事：
// 1. MPC有效时：绘制MPC参考轨迹线、优化平滑轨迹线、时域中心点标记球体
// 2. 根据当前云台姿态求解子弹抛物线弹道，绘制弹道线
// 兜底逻辑：没有MPC调试数据，拿当前真实云台姿态，模拟15m距离弹道用于调试
// 依赖资源：弹道求解solver、TF坐标系、子弹初速bullet_speed
// =========================================================================
app.add_system<talos::fixed_rate<250>>(
        "foxglove_l4_mpc_prediction_scene",
        [](talos::spmc<::fcs::L5::WeaponCommand, WeaponCommandChannelTopic> cmd_in,
           talos::res<std::shared_ptr<FoxgloveServer>> server,
           core::trajectory::trajectory_solver solver,    // 弹道求解器资源，计算子弹重力弹道
           talos::res<fast_tf::CoordinateSystem> coord,   // FastTF坐标系资源，查询gimbal→odom变换
           core::trajectory::bullet_speed bullet_speed) { // 子弹初速参数资源
            if (!detail::foxglove_ready(*server, cmd_in)) {
                return;
            }
            std::vector<::foxglove::schemas::SceneEntity> entities;
            auto cmd = cmd_in.read();

            // MPC调试数据有效：渲染MPC预测轨迹
            if (cmd && cmd->viz_debug) {
                entities.reserve(2);
                // -------- 绘制MPC参考轨迹连线（原始目标时域序列） --------
                if (cmd->viz_debug->reference_plan.size() >= 2) {
                    std::vector<::foxglove::schemas::Point3> ref_line_points;
                    ref_line_points.reserve(cmd->viz_debug->reference_plan.size());
                    // spherical_to_cartesian：球坐标(distance/yaw/pitch)转odom系笛卡尔3D点
                    for (const auto& reference : cmd->viz_debug->reference_plan) {
                        ref_line_points.push_back(
                            detail::spherical_to_cartesian_point3(
                                reference.distance, reference.yaw, reference.pitch));
                    }
                    // line_strip：连续折线实体
                    entities.push_back(
                        viz::EntityBuilder::create<fast_tf::odom>("l4", "mpc_ref_line")
                            .timestamp(cmd->timestamp_ns)
                            .line_strip(
                                std::move(ref_line_points), tac::L4::MPC_REFERENCE,
                                tac::L4::TRAJECTORY_LINE_THICKNESS)
                            .build());
                }
                // -------- 绘制MPC优化平滑轨迹连线 --------
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

                // std::clamp 把center_index限制合法下标，防止数组越界崩溃
                const int center_index = std::clamp(
                    cmd->viz_debug->center_index, 0,
                    static_cast<int>(cmd->viz_debug->optimized_plan.size()) - 1);

                // 局部lambda封装：生成轨迹关键点球体+文本标签，复用代码
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
                // 绘制参考轨迹当前时域中心点球体标签 Ref
                add_key_marker(
                    "mpc_ref_center",
                    cmd->viz_debug->reference_plan[static_cast<size_t>(center_index)],
                    l4_detail::key_point_label(
                        "Ref", cmd->viz_debug->reference_plan[static_cast<size_t>(center_index)]),
                    tac::L4::MPC_REFERENCE, tac::L4::SELECTION_SIZE);
                // 绘制优化轨迹当前时域中心点球体标签 Opt
                add_key_marker(
                    "mpc_opt_center",
                    cmd->viz_debug->optimized_plan[static_cast<size_t>(center_index)],
                    l4_detail::key_point_label(
                        "Opt", cmd->viz_debug->optimized_plan[static_cast<size_t>(center_index)]),
                    tac::L4::MPC_PRESENT, tac::L4::SELECTION_SIZE);

                // 弹道求解器：根据MPC输出pitch、子弹初速、预测距离，生成子弹抛物线轨迹点
                auto trajectory = solver->get()->generate_trajectory(
                    cmd->pitch, bullet_speed->bullet_speed, cmd->distance);
                std::vector<Eigen::Vector3d> traj_points;
                traj_points.reserve(trajectory.size());
                // 弹道坐标转换到odom坐标系
                for (const auto& [x, z] : trajectory) {
                    traj_points.emplace_back(
                        x * std::cos(cmd->pitch) + z * std::sin(cmd->pitch), 0,
                        -x * std::sin(cmd->pitch) + z * std::cos(cmd->pitch));
                }
                // 生成子弹弹道线SceneEntity
                auto traj_entities =
                    viz::patterns::bullet_trajectory(traj_points, cmd->fire, cmd->timestamp_ns);
                entities.insert(
                    entities.end(), std::make_move_iterator(traj_entities.begin()),
                    std::make_move_iterator(traj_entities.end()));
            } else {
                // 兜底分支：没有MPC调试数据，拿真实云台姿态模拟15m弹道，方便调试
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