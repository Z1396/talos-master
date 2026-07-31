/**
 * @file selected_target_snapshot.hpp
 * @brief L4规划层选中目标快照
 *
 * 本文件定义了L4规划层的选中目标快照数据结构。
 * 这是一个可观察性数据结构，用于向外部消费者（如可视化、录制系统）
 * 提供当前选中目标的摘要信息。
 *
 * 设计理念：
 * - 仅包含决策层需要的数据，不包含L5执行层的敏感数据
 * - 轻量级结构，便于高频传输和存储
 * - 自包含设计，可以独立判断是否有有效目标
 *
 * 数据流：
 * L4::try_armor() → SelectedTargetSnapshot → Foxglove可视化 / 录制
 */

#pragma once

#include "L3_estimation/tracker/types.hpp"
#include "L4_planning/aimer/types.hpp"
#include "L4_planning/plan_source.hpp"

namespace fcs::L4 {

/**
 * @brief L4选中目标快照
 *
 * 记录当前选中目标的决策层视图，用于可观察性消费者。
 * 这个快照是L4规划层的输出摘要，不包含L5武器控制层的执行细节。
 *
 * 设计考虑：
 * - 分离决策数据和执行数据，避免暴露内部实现细节
 * - 提供足够的信息用于可视化和调试
 * - 通过has_target()方法提供便捷的有效性检查
 *
 * 使用场景：
 * - Foxglove实时可视化当前目标
 * - 录制系统记录目标选择历史
 * - 上位机显示当前目标信息
 */
struct SelectedTargetSnapshot {
    uint64_t timestamp_ns{0};                       ///< 时间戳（纳秒）
    GimbalPlanSource source{GimbalPlanSource::Armor}; ///< 目标源（Armor或Rune）
    double distance{-1.0};                          ///< 目标距离（米），负值表示无效
    uint64_t predicted_future_ns{0};                ///< 预测未来时间戳（纳秒）
    ArmorAimPhase aim_phase{ArmorAimPhase::SingleArmor}; ///< 瞄准阶段
    int selected_armor_id{0};                       ///< 选中的装甲板ID（精确）
    int rough_selected_armor_id{0};                 ///< 粗略选中的装甲板ID（用于ROI）
    L3::TrackerOutput tracker{};                    ///< 跟踪器输出（包含完整目标信息）

    /**
     * @brief 检查是否有有效目标
     *
     * 判断标准：
     * - source必须为Armor（Rune目标不进入此通道）
     * - distance必须大于0（负值表示无效）
     * - tracker.target_name必须有效
     *
     * @return true表示有有效目标，false表示无目标
     */
    [[nodiscard]] bool has_target() const noexcept {
        return source == GimbalPlanSource::Armor && distance > 0.0
            && tracker.target_name != ArmorName::Invalid;
    }
};

} // namespace fcs::L4
