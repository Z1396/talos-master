/**
 * @file fsm.hpp
 * @brief 装甲板瞄准状态机
 *
 * 本文件实现了装甲板瞄准策略的状态机，根据目标角速度动态调整瞄准模式。
 * 状态机通过检测目标旋转速度，在单装甲板模式和多装甲板模式之间切换，
 * 以适应不同转速的目标。
 *
 * 状态机设计：
 * - 4个状态：单装甲板 -> 整车装甲板 -> 整车双板 -> 整车中心
 * - 使用计数器实现状态转换的滞后性，避免频繁切换
 * - 通过阈值控制状态转换条件
 *
 * 状态转换逻辑：
 * 1. SingleArmor（单装甲板）：目标转速低，锁定单块装甲板
 * 2. WholeCarArmor（整车装甲板）：目标转速中等，考虑所有装甲板
 * 3. WholeCarPair（整车双板）：目标转速较高，瞄准对称装甲板对
 * 4. WholeCarCenter（整车中心）：目标转速极高，瞄准车辆中心
 *
 * 应用场景：
 * - 低速旋转目标：精确打击单块装甲板
 * - 中速旋转目标：选择最佳装甲板进行打击
 * - 高速旋转目标：打击对称装甲板或车辆中心
 */
#pragma once

#include "L4_planning/aimer/types.hpp"
#include "L4_planning/config.hpp"

#include <cmath>

namespace fcs::L4 {

/**
 * @brief 推进装甲板瞄准阶段状态机
 *
 * 根据目标角速度和状态转换规则更新瞄准阶段。
 * 状态机使用计数器实现滞后机制，避免因噪声导致的频繁切换。
 *
 * 核心算法：
 * 1. 检测目标是否跳跃（target_jumped）
 * 2. 根据当前阶段和角速度计算计数器增量
 * 3. 计数器超过阈值时触发状态转换
 * 4. 状态转换后计数器清零
 *
 * @param cfg 瞄准器配置参数（包含各阈值）
 * @param abs_v_yaw 目标角速度绝对值（弧度/秒）
 * @param target_jumped 目标是否跳跃（是否已锁定）
 * @param phase 当前瞄准阶段（输入/输出）
 * @param overflow_count 溢出计数器（输入/输出）
 */
inline void advance_armor_aim_phase(
    const AimerConfig& cfg, double abs_v_yaw, bool target_jumped, ArmorAimPhase& phase,
    int& overflow_count) noexcept {
    // 如果目标未跳跃（未锁定），强制回到单装甲板模式
    if (!target_jumped) {
        phase          = ArmorAimPhase::SingleArmor;
        overflow_count = 0;
        return;
    }

    // 根据当前阶段执行状态转换逻辑
    switch (phase) {
    case ArmorAimPhase::SingleArmor:
        // 单装甲板 -> 整车装甲板：当角速度超过上限时计数
        overflow_count = (abs_v_yaw > cfg.single_whole_up) ? overflow_count + 1 : 0;
        if (overflow_count > cfg.transfer_thresh) {
            phase          = ArmorAimPhase::WholeCarArmor;
            overflow_count = 0;
        }
        break;

    case ArmorAimPhase::WholeCarArmor:
        // 整车装甲板 -> 整车双板 或 -> 单装甲板
        if (abs_v_yaw > cfg.whole_pair_up) {
            ++overflow_count;  // 角速度高，向双板转换
        } else if (abs_v_yaw < cfg.single_whole_down) {
            --overflow_count;  // 角速度低，向单装甲板转换
        } else {
            overflow_count = 0; // 角速度在中间范围，重置计数器
        }
        // 计数器绝对值超过阈值时触发转换
        if (std::abs(overflow_count) > cfg.transfer_thresh) {
            phase = (overflow_count > 0) ? ArmorAimPhase::WholeCarPair : ArmorAimPhase::SingleArmor;
            overflow_count = 0;
        }
        break;

    case ArmorAimPhase::WholeCarPair:
        // 整车双板 -> 整车中心 或 -> 整车装甲板
        if (abs_v_yaw > cfg.pair_center_up) {
            ++overflow_count;  // 角速度高，向中心转换
        } else if (abs_v_yaw < cfg.whole_pair_down) {
            --overflow_count;  // 角速度低，向整车装甲板转换
        } else {
            overflow_count = 0;
        }
        if (std::abs(overflow_count) > cfg.transfer_thresh) {
            phase =
                (overflow_count > 0) ? ArmorAimPhase::WholeCarCenter : ArmorAimPhase::WholeCarArmor;
            overflow_count = 0;
        }
        break;

    case ArmorAimPhase::WholeCarCenter:
        // 整车中心 -> 整车双板：当角速度低于下限时计数
        overflow_count = (abs_v_yaw < cfg.pair_center_down) ? overflow_count + 1 : 0;
        if (overflow_count > cfg.transfer_thresh) {
            phase          = ArmorAimPhase::WholeCarPair;
            overflow_count = 0;
        }
        break;
    }
}

} // namespace fcs::L4
