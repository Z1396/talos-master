/**
 * @file target_selection_trace.hpp
 * @brief L4规划层目标选择追踪诊断数据结构
 *
 * 本文件定义了目标选择过程的完整诊断数据，用于：
 * - 可视化目标选择决策过程
 * - 录制和回放选择历史
 * - 调试评分算法
 * - 性能分析和优化
 *
 * 设计理念：
 * - 诊断数据与控制意图分离，避免影响执行路径
 * - 完整记录所有候选目标的评分过程
 * - 支持离线分析和可视化
 *
 * 数据流：
 * L4::try_armor() → TargetSelectionTrace → Foxglove可视化 → 录制
 */

#pragma once

#include "L3_estimation/tracker/types.hpp"

#include <Eigen/Core>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace fcs::L4 {

/**
 * @brief 单个目标候选的完整诊断信息
 *
 * 记录单个目标从进入候选列表到最终决策的完整过程。
 * 包括原始数据、评分过程、加权计算和最终决策。
 *
 * 设计考虑：
 * - 使用infinity作为默认值，便于识别无效数据
 * - 分离原始数据、评分、加权三个层次
 * - 记录完整上下文，便于离线分析
 */
struct TargetSelectionTraceEntry {
    // ========== 目标基本信息 ==========
    ArmorName target_name{ArmorName::Invalid};      ///< 目标装甲板名称（如HERO、INFANTRY等）
    ArmorColor target_color{ArmorColor::Neutral};   ///< 目标颜色（RED、BLUE）
    L3::TrackerStatus track_status{L3::TrackerStatus::Idle}; ///< 跟踪器状态

    // ========== 决策结果 ==========
    int rank{0};                                    ///< 排名（1=最佳，2=次佳，...）
    bool aim_valid{false};                          ///< 瞄准是否成功
    bool was_previously_selected{false};            ///< 是否是上一帧选中的目标
    bool selected{false};                           ///< 是否被选中
    bool runner_up{false};                          ///< 是否是备选目标
    std::string aim_error{};                        ///< 瞄准失败时的错误信息
    std::optional<Eigen::Vector3d> target_center{}; ///< 目标中心位置（世界坐标系）

    // ========== 原始指标 ==========
    double image_center_distance_px{std::numeric_limits<double>::infinity()}; ///< 图像中心距离（像素）
    double optical_age_s{0.0};                      ///< 最后观测时间距今（秒）
    double tof_s{std::numeric_limits<double>::infinity()}; ///< 弹道飞行时间（秒）
    double distance_m{std::numeric_limits<double>::infinity()}; ///< 目标距离（米）
    double yaw_effort_deg{std::numeric_limits<double>::infinity()}; ///< yaw调整量（度）
    double pitch_effort_deg{std::numeric_limits<double>::infinity()}; ///< pitch调整量（度）

    // ========== 单项评分（归一化到[0,1]）==========
    double image_center_score{0.0};                 ///< 图像中心评分
    double track_state_score{0.0};                  ///< 跟踪状态评分
    double tof_score{0.0};                          ///< 飞行时间评分
    double gimbal_effort_score{0.0};                ///< 云台调整评分
    double armor_name_score{0.0};                   ///< 装甲板名称评分

    // ========== 加权评分 ==========
    double image_center_weighted{0.0};              ///< 图像中心加权评分
    double track_state_weighted{0.0};               ///< 跟踪状态加权评分
    double tof_weighted{0.0};                       ///< 飞行时间加权评分
    double gimbal_effort_weighted{0.0};             ///< 云台调整加权评分
    double armor_name_weighted{0.0};                ///< 装甲板名称加权评分

    // ========== 总分 ==========
    double weighted_sum{0.0};                       ///< 加权总分
    double total_weight{0.0};                       ///< 总权重
    double total_score{0.0};                        ///< 最终评分（weighted_sum / total_weight）
};

/**
 * @brief 单次L4更新的完整目标选择追踪
 *
 * 记录一次目标选择决策的完整过程，包括：
 * - 上一次选择的目标
 * - 所有候选目标的详细评分
 * - 最终决策结果
 *
 * 用途：
 * - Foxglove实时可视化
 * - 录制和回放分析
 * - 调试评分算法
 * - 性能监控
 */
struct TargetSelectionTrace {
    uint64_t timestamp_ns{0};                       ///< 时间戳（纳秒）
    bool had_previous_target{false};                ///< 是否有上一帧目标
    ArmorName previous_target_name{ArmorName::Invalid}; ///< 上一帧目标名称
    ArmorColor previous_target_color{ArmorColor::Neutral}; ///< 上一帧目标颜色
    bool kept_current_target{false};                ///< 是否保持当前目标
    double switch_margin{0.0};                      ///< 切换余量（仅有人驾驶模式）
    std::vector<TargetSelectionTraceEntry> candidates{}; ///< 所有候选目标的追踪记录
};

} // namespace fcs::L4
