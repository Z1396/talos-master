/**
 * @file armor_target_decider.cpp
 * @brief 装甲板目标选择决策器实现
 *
 * 本文件实现了目标选择决策器的核心算法，包括：
 * - 基于评分的目标排序（无人模式）
 * - 基于图像中心距离的目标排序（有人模式）
 * - 目标切换保护机制
 * - 目标保持策略
 *
 * 核心算法原理：
 * 1. 无人模式（Unmanned）：
 *    - 使用综合评分排序，优先选择总分最高的目标
 *    - 应用切换阈值保护：新目标必须显著优于当前目标才切换
 *    - 避免因评分微小波动导致的频繁切换
 *
 * 2. 有人模式（Manned）：
 *    - 使用图像中心距离排序，跟随操作员意图
 *    - 保持当前目标直到丢失，不主动切换
 *    - 适合手动辅助瞄准场景
 */
#include "L4_planning/aimer/armor_target_decider.hpp"

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace fcs::L4 {

namespace {

/**
 * @brief 查找当前选中目标在候选列表中的索引
 *
 * @param candidates 候选目标列表
 * @param selected_key 当前选中的目标标识
 * @return 目标索引，如果未找到则返回nullopt
 */
[[nodiscard]] auto find_selected_candidate_index(
    std::span<const ArmorTargetDeciderCandidate> candidates,
    const std::optional<core::TargetKey>& selected_key) noexcept -> std::optional<size_t> {
    if (!selected_key.has_value()) {
        return std::nullopt;
    }

    // 遍历候选列表，查找匹配的目标标识
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].key == *selected_key) {
            return i;
        }
    }

    return std::nullopt;
}

/**
 * @brief 比较两个候选目标的优先级（基于总分）
 *
 * 排序优先级：
 * 1. 总分越高优先级越高
 * 2. 总分相同时，图像中心距离越小优先级越高
 * 3. 以上都相同，最新观测时间越晚优先级越高
 *
 * @param lhs 左侧候选目标
 * @param rhs 右侧候选目标
 * @return lhs优先级是否高于rhs
 */
[[nodiscard]] auto compare_total_score(
    const ArmorTargetDeciderCandidate& lhs, const ArmorTargetDeciderCandidate& rhs) noexcept
    -> bool {
    // 首先比较总分（越高越好）
    if (lhs.scores.total != rhs.scores.total) {
        return lhs.scores.total > rhs.scores.total;
    }

    // 检查图像中心距离是否有效
    const bool lhs_has_finite = std::isfinite(lhs.tracker.last_image_center_distance_px);
    const bool rhs_has_finite = std::isfinite(rhs.tracker.last_image_center_distance_px);

    // 有效值优先于无效值
    if (lhs_has_finite != rhs_has_finite) {
        return lhs_has_finite;
    }

    // 如果都有有效值，距离越小越好
    if (lhs_has_finite
        && lhs.tracker.last_image_center_distance_px != rhs.tracker.last_image_center_distance_px) {
        return lhs.tracker.last_image_center_distance_px
             < rhs.tracker.last_image_center_distance_px;
    }

    // 最后比较观测时间（越晚越好）
    return lhs.tracker.last_observation_timestamp_ns > rhs.tracker.last_observation_timestamp_ns;
}

/**
 * @brief 比较两个候选目标的优先级（基于图像中心距离）
 *
 * 排序优先级：
 * 1. 图像中心距离越小优先级越高（有人模式跟随操作员意图）
 * 2. 距离相同时，总分越高优先级越高
 * 3. 以上都相同，最新观测时间越晚优先级越高
 *
 * @param lhs 左侧候选目标
 * @param rhs 右侧候选目标
 * @return lhs优先级是否高于rhs
 */
[[nodiscard]] auto compare_image_center_distance(
    const ArmorTargetDeciderCandidate& lhs, const ArmorTargetDeciderCandidate& rhs) noexcept
    -> bool {
    // 检查图像中心距离是否有效
    const bool lhs_has_finite = std::isfinite(lhs.tracker.last_image_center_distance_px);
    const bool rhs_has_finite = std::isfinite(rhs.tracker.last_image_center_distance_px);

    // 有效值优先于无效值
    if (lhs_has_finite != rhs_has_finite) {
        return lhs_has_finite;
    }

    // 如果都有有效值，距离越小越好
    if (lhs_has_finite
        && lhs.tracker.last_image_center_distance_px != rhs.tracker.last_image_center_distance_px) {
        return lhs.tracker.last_image_center_distance_px
             < rhs.tracker.last_image_center_distance_px;
    }

    // 然后比较总分（越高越好）
    if (lhs.scores.total != rhs.scores.total) {
        return lhs.scores.total > rhs.scores.total;
    }

    // 最后比较观测时间（越晚越好）
    return lhs.tracker.last_observation_timestamp_ns > rhs.tracker.last_observation_timestamp_ns;
}

/**
 * @brief 对候选目标进行排序并返回排名索引
 *
 * @tparam Compare 比较函数类型
 * @param candidates 候选目标列表
 * @param compare 比较函数
 * @return 排序后的索引数组（最优在前）
 */
template <typename Compare>
[[nodiscard]] auto
    ranked_indices_by(std::span<const ArmorTargetDeciderCandidate> candidates, Compare compare)
        -> std::vector<size_t> {
    // 创建索引数组
    std::vector<size_t> indices;
    indices.reserve(candidates.size());
    for (size_t i = 0; i < candidates.size(); ++i) {
        indices.push_back(i);
    }

    // 使用比较函数对索引排序
    std::ranges::sort(
        indices, [&](size_t lhs, size_t rhs) { return compare(candidates[lhs], candidates[rhs]); });
    return indices;
}

/**
 * @brief 在已排序的索引中找到第一个非当前选中目标的索引
 *
 * 用于确定备选目标（runner_up）。
 *
 * @param ranked_indices 已排序的索引数组
 * @param selected_index 当前选中目标的索引
 * @return 第一个非选中目标的索引，如果没有其他目标则返回nullopt
 */
[[nodiscard]] auto
    first_non_selected_index(std::span<const size_t> ranked_indices, size_t selected_index) noexcept
    -> std::optional<size_t> {
    for (const size_t index : ranked_indices) {
        if (index != selected_index) {
            return index;
        }
    }

    return std::nullopt;
}

/**
 * @brief 无人模式目标选择决策
 *
 * 核心算法：
 * 1. 使用总分对目标排序
 * 2. 检查是否需要切换目标（应用切换阈值保护）
 * 3. 更新选中目标状态
 * 4. 返回决策结果和备选目标
 *
 * 切换保护机制：
 * - 新目标的评分必须超过当前目标评分 + switch_margin
 * - 否则保持当前目标，避免频繁切换
 *
 * @param decider 无人模式决策器参数
 * @param state 决策器状态（可修改）
 * @param candidates 候选目标列表
 * @return 决策结果，如果没有候选目标则返回nullopt
 */
[[nodiscard]] auto decide_unmanned_target(
    const UnmannedArmorTargetDecider& decider, ArmorTargetDeciderState& state,
    std::span<const ArmorTargetDeciderCandidate> candidates) noexcept
    -> std::optional<ArmorTargetDeciderResult> {
    // 如果没有候选目标，清空选中状态
    if (candidates.empty()) {
        state.selected_key.reset();
        return std::nullopt;
    }

    // 根据总分排序
    auto ranked = ranked_indices_by(candidates, compare_total_score);

    // 默认选择评分最高的目标
    size_t selected_index    = ranked.front();
    bool kept_current_target = false;

    // 检查当前目标是否仍在候选列表中
    const auto current_index = find_selected_candidate_index(candidates, state.selected_key);
    if (current_index.has_value() && *current_index != ranked.front()) {
        // 应用切换阈值保护
        const double best_score    = candidates[ranked.front()].scores.total;
        const double current_score = candidates[*current_index].scores.total;

        // 如果新目标没有显著优于当前目标，保持当前目标
        if (best_score <= current_score + decider.switch_margin) {
            selected_index      = *current_index;
            kept_current_target = true;
        }
    }

    // 更新选中状态
    state.selected_key   = candidates[selected_index].key;

    // 确定备选目标
    const auto runner_up = kept_current_target ? std::optional<size_t>{ranked.front()}
                                               : first_non_selected_index(ranked, selected_index);

    return ArmorTargetDeciderResult{
        .selected_index      = selected_index,
        .kept_current_target = kept_current_target,
        .ranked_indices      = std::move(ranked),
        .runner_up           = runner_up,
    };
}

/**
 * @brief 有人模式目标选择决策
 *
 * 核心算法：
 * 1. 使用图像中心距离对目标排序
 * 2. 保持当前目标（如果仍在候选列表中）
 * 3. 更新选中目标状态
 * 4. 返回决策结果和备选目标
 *
 * 保持策略：
 * - 始终保持当前目标（跟随操作员意图）
 * - 将当前目标调整到排名列表首位，便于上层使用
 *
 * @param state 决策器状态（可修改）
 * @param candidates 候选目标列表
 * @return 决策结果，如果没有候选目标则返回nullopt
 */
[[nodiscard]] auto decide_manned_target(
    ArmorTargetDeciderState& state,
    std::span<const ArmorTargetDeciderCandidate> candidates) noexcept
    -> std::optional<ArmorTargetDeciderResult> {
    // 如果没有候选目标，清空选中状态
    if (candidates.empty()) {
        state.selected_key.reset();
        return std::nullopt;
    }

    // 根据图像中心距离排序
    auto ranked = ranked_indices_by(candidates, compare_image_center_distance);

    // 默认选择距离最小的目标
    size_t selected_index    = ranked.front();
    bool kept_current_target = false;

    // 检查当前目标是否仍在候选列表中
    if (const auto current_index = find_selected_candidate_index(candidates, state.selected_key);
        current_index.has_value()) {
        // 保持当前目标
        selected_index      = *current_index;
        kept_current_target = true;

        // 将当前目标调整到排名列表首位（便于上层使用）
        ranked.erase(std::remove(ranked.begin(), ranked.end(), selected_index), ranked.end());
        ranked.insert(ranked.begin(), selected_index);
    }

    // 更新选中状态
    state.selected_key   = candidates[selected_index].key;

    // 确定备选目标
    const auto runner_up = first_non_selected_index(ranked, selected_index);

    return ArmorTargetDeciderResult{
        .selected_index      = selected_index,
        .kept_current_target = kept_current_target,
        .ranked_indices      = std::move(ranked),
        .runner_up           = runner_up,
    };
}

} // namespace

/**
 * @brief 工厂函数：根据枚举类型构造不同的目标选择决策器
 * @param kind 决策器类型枚举：Unmanned自动选目标 / Manned手控优先
 * @param switch_margin 自动模式下切换目标的代价裕度阈值
 * @return ArmorTargetDecider 返回std::variant变体，可以存放两种决策器其中一种
 * noexcept 不抛异常
 *
 * 返回类型 ArmorTargetDecider 本质是 std::variant<UnmannedArmorTargetDecider, MannedArmorTargetDecider>
 */
auto make_armor_target_decider(ArmorTargetDeciderKind kind, double switch_margin) noexcept
    -> ArmorTargetDecider {
    switch (kind) {
    // 自动模式：机器人自主根据代价选装甲目标，带切换裕度防止频繁跳目标
    case ArmorTargetDeciderKind::Unmanned:
        return UnmannedArmorTargetDecider{.switch_margin = switch_margin};
    // 手控模式：允许遥控器手动强制指定目标，算法服从遥控器指令
    case ArmorTargetDeciderKind::Manned:
        return MannedArmorTargetDecider{};
    }

    // switch枚举没有覆盖全部分支时的兜底返回，消除编译器warning
    // 理论不会走到这里，防御性编程
    return UnmannedArmorTargetDecider{.switch_margin = switch_margin};
}


/**
 * @brief 执行目标选择决策
 *
 * 使用std::visit实现策略模式，根据决策器类型调用对应的决策算法。
 *
 * @param decider 决策器实例
 * @param state 决策器状态
 * @param candidates 候选目标列表
 * @return 决策结果
 */
auto decide_armor_target(
    const ArmorTargetDecider& decider, ArmorTargetDeciderState& state,
    std::span<const ArmorTargetDeciderCandidate> candidates) noexcept
    -> std::optional<ArmorTargetDeciderResult> {
    return std::visit(
        [&](const auto& concrete_decider) -> std::optional<ArmorTargetDeciderResult> {
            using T = std::decay_t<decltype(concrete_decider)>;
            // 编译期分支选择（使用if constexpr避免运行时开销）
            if constexpr (std::is_same_v<T, UnmannedArmorTargetDecider>) {
                return decide_unmanned_target(concrete_decider, state, candidates);
            } else {
                return decide_manned_target(state, candidates);
            }
        },
        decider);
}

} // namespace fcs::L4
