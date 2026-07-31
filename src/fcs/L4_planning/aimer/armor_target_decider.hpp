/**
 * @file armor_target_decider.hpp
 * @brief 装甲板目标选择决策器
 *
 * 本文件定义了目标选择决策器的接口和数据结构，用于从多个候选目标中选择最佳的打击目标。
 * 决策器支持两种模式：无人模式（基于评分）和有人模式（基于图像中心距离）。
 *
 * 核心功能：
 * - 目标评分计算：综合考虑距离、姿态、跟踪状态等因素
 * - 目标选择策略：根据决策模式选择最优目标
 * - 目标切换保护：避免频繁切换目标导致云台抖动
 *
 * 设计模式：
 * - 使用std::variant表达两种决策策略（Unmanned/Manned）
 * - 使用状态机记录当前选中的目标，实现平滑切换
 */
#pragma once

#include "L3_estimation/tracker/types.hpp"
#include "core/target_key.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>
#include <vector>

namespace fcs::L4 {

/**
 * @enum ArmorTargetDeciderKind
 * @brief 目标选择决策器类型
 */
enum class ArmorTargetDeciderKind : uint8_t {
    Unmanned = 0, ///< 无人模式：基于评分选择目标
    Manned   = 1, ///< 有人模式：基于图像中心距离选择目标
};

/**
 * @struct TargetSelectionScores
 * @brief 目标选择评分结构
 *
 * 用于量化评估目标优先级的多个维度。
 * 各项评分已加权融合为total总分。
 */
struct TargetSelectionScores {
    double total{0.0};          ///< 总分（加权融合）
    double image_center{0.0};   ///< 图像中心距离评分（越小越好）
    double track_state{0.0};    ///< 跟踪状态评分（稳定性）
    double tof{0.0};            ///< 飞行时间评分（距离）
    double gimbal_effort{0.0};  ///< 云台运动代价评分（转动角度）
    double armor_name{0.0};     ///< 装甲板类型评分（优先级）
};

/**
 * @struct ArmorTargetDeciderCandidate
 * @brief 目标选择候选者
 *
 * 表示一个待选择的候选目标，包含目标标识、跟踪输出和评分信息。
 */
struct ArmorTargetDeciderCandidate {
    core::TargetKey key{};            ///< 目标唯一标识
    L3::TrackerOutput tracker{};      ///< 跟踪器输出（位置、姿态等）
    TargetSelectionScores scores{};   ///< 综合评分
};

/**
 * @struct ArmorTargetDeciderState
 * @brief 目标选择决策器状态
 *
 * 记录当前选中的目标，用于实现目标切换的平滑过渡。
 */
struct ArmorTargetDeciderState {
    std::optional<core::TargetKey> selected_key{}; ///< 当前选中的目标标识
};

/**
 * @struct UnmannedArmorTargetDecider
 * @brief 无人模式目标选择决策器
 *
 * 特点：
 * - 基于综合评分选择目标（total分数）
 * - 具有切换阈值保护，避免频繁切换
 * - 适合自动瞄准场景
 */
struct UnmannedArmorTargetDecider {
    double switch_margin{0.08}; ///< 目标切换的评分阈值（新目标需比旧目标高出此值才切换）
};

/**
 * @struct MannedArmorTargetDecider
 * @brief 有人模式目标选择决策器
 *
 * 特点：
 * - 基于图像中心距离选择目标（跟随操作员意图）
 * - 保持当前目标，除非完全丢失
 * - 适合手动辅助瞄准场景
 */
struct MannedArmorTargetDecider {};

/// 目标选择决策器类型（使用std::variant表达两种策略）
using ArmorTargetDecider = std::variant<UnmannedArmorTargetDecider, MannedArmorTargetDecider>;

/**
 * @struct ArmorTargetDeciderResult
 * @brief 目标选择决策结果
 *
 * 包含选择的目标索引、排名信息等。
 */
struct ArmorTargetDeciderResult {
    size_t selected_index{0};                 ///< 选中的目标索引
    bool kept_current_target{false};          ///< 是否保持了当前目标（未切换）
    std::vector<size_t> ranked_indices{};     ///< 所有目标的排名索引
    std::optional<size_t> runner_up{};        ///< 第二优选择（备选目标）
};

/**
 * @brief 创建目标选择决策器
 *
 * 根据决策器类型和参数创建对应的决策器实例。
 *
 * @param kind 决策器类型（Unmanned/Manned）
 * @param switch_margin 目标切换阈值（仅Unmanned模式有效）
 * @return 决策器实例
 */
[[nodiscard]] auto
    make_armor_target_decider(ArmorTargetDeciderKind kind, double switch_margin) noexcept
    -> ArmorTargetDecider;

/**
 * @brief 执行目标选择决策
 *
 * 核心算法：
 * 1. 根据决策器类型选择排序策略
 * 2. 对所有候选目标进行排序
 * 3. 应用切换保护策略（Unmanned模式）
 * 4. 返回选择结果和排名信息
 *
 * @param decider 决策器实例
 * @param state 决策器状态（可修改，记录当前选中目标）
 * @param candidates 候选目标列表
 * @return 决策结果，如果没有候选目标则返回nullopt
 */
[[nodiscard]] auto decide_armor_target(
    const ArmorTargetDecider& decider, ArmorTargetDeciderState& state,
    std::span<const ArmorTargetDeciderCandidate> candidates) noexcept
    -> std::optional<ArmorTargetDeciderResult>;

} // namespace fcs::L4
