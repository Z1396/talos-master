/**
 * @file fsm.hpp
 * @brief 装甲板瞄准阶段状态机（根据目标小陀螺转速，切换四种瞄准策略）
 *
 * 状态机共4个状态，按目标角速度 |v_yaw| 从低到高排列：
 *   1. SingleArmor     单板锁定：转速低，目标姿态稳定，锁定正对的一块装甲板精确打击
 *   2. WholeCarArmor   整车建模：转速中等，对整车四块装甲统一建模选择最佳打击板
 *   3. WholeCarPair    双板瞄准：转速较高（小陀螺），瞄准对称的两块装甲板对
 *   4. WholeCarCenter  中心瞄准：转速极高，装甲板切换过快无法逐板打击，直接瞄车体中心
 *
 * 转换机制（两层防抖）：
 *   1. 滞环阈值：相邻两状态共用一对 up/down 阈值（up < down，中间留缓冲带），
 *      角速度在缓冲带内来回波动不会触发转换
 *   2. 计数器：条件需连续满足 transfer_thresh 帧才真正切换，单帧噪声无效
 *
 * 阈值与状态对的关系（数值见 AimerConfig，单板=1.0/1.5，双板=6.5/7.5，中心=15.0/16.5）：
 *   - single_whole_up/down   管 SingleArmor ↔ WholeCarArmor 这对转换
 *   - whole_pair_up/down     管 WholeCarArmor ↔ WholeCarPair 这对转换
 *   - pair_center_up/down    管 WholeCarPair ↔ WholeCarCenter 这对转换
 *
 * 前置条件：仅当 tracker 报告 target_jumped（目标在旋转）时状态机才向上推进；
 * 未跳跃目标一律强制回落 SingleArmor（低转速下单板锁定最精确）。
 */
#pragma once

#include "L4_planning/aimer/types.hpp"
#include "L4_planning/config.hpp"

#include <cmath>

namespace fcs::L4 {

/**
 * @brief 推进装甲板瞄准阶段状态机（每帧调用一次）
 *
 * @param cfg            瞄准器配置（六个滞环阈值 + transfer_thresh 计数阈值）
 * @param abs_v_yaw      目标角速度绝对值（rad/s，来自 L3 EKF 的 vyaw）
 * @param target_jumped  目标是否在旋转（L3 的 target_jumped_：观测到非锚点装甲即 true）
 * @param phase          当前瞄准阶段（输入/输出，跨帧保持）
 * @param overflow_count 溢出计数器（输入/输出，跨帧保持；正=向高档累积，负=向低档累积）
 */
inline void advance_armor_aim_phase(
    const AimerConfig& cfg, double abs_v_yaw, bool target_jumped, ArmorAimPhase& phase,
    int& overflow_count) noexcept {
    // 目标未旋转（没有观测到侧面装甲板）：无小陀螺风险，强制单板锁定模式
    if (!target_jumped) {
        phase          = ArmorAimPhase::SingleArmor;
        overflow_count = 0;
        return;
    }

    // 根据当前阶段执行状态转换逻辑；三档滞环的 up 阈值管升档、down 阈值管降档
    switch (phase) {
    case ArmorAimPhase::SingleArmor:
        // 最低档只升不降：角速度超过 single_whole_up(1.5) 时正向累积计数，否则清零
        overflow_count = (abs_v_yaw > cfg.single_whole_up) ? overflow_count + 1 : 0;
        if (overflow_count > cfg.transfer_thresh) {
            phase          = ArmorAimPhase::WholeCarArmor; // 连续超阈值 → 升入整车建模
            overflow_count = 0;
        }
        break;

    case ArmorAimPhase::WholeCarArmor:
        // 中间档双向判断：升门 whole_pair_up(6.5)，降门 single_whole_down(1.0)
        if (abs_v_yaw > cfg.whole_pair_up) {
            ++overflow_count;   // 转速够快，向高档（双板）累积
        } else if (abs_v_yaw < cfg.single_whole_down) {
            --overflow_count;   // 转速够慢，向低档（单板）累积
        } else {
            overflow_count = 0; // 角速度落在两门之间的滞环缓冲带：清零重来
        }
        // 计数器带符号累积，任一方向连续超过阈值才切换
        if (std::abs(overflow_count) > cfg.transfer_thresh) {
            phase = (overflow_count > 0) ? ArmorAimPhase::WholeCarPair : ArmorAimPhase::SingleArmor;
            overflow_count = 0;
        }
        break;

    case ArmorAimPhase::WholeCarPair:
        // 双板档双向判断：升门 pair_center_up(16.5)，降门 whole_pair_down(7.5)
        if (abs_v_yaw > cfg.pair_center_up) {
            ++overflow_count;   // 转速极高，向高档（中心）累积
        } else if (abs_v_yaw < cfg.whole_pair_down) {
            --overflow_count;   // 转速回落，向低档（整车建模）累积
        } else {
            overflow_count = 0; // 滞环缓冲带：清零
        }
        if (std::abs(overflow_count) > cfg.transfer_thresh) {
            phase =
                (overflow_count > 0) ? ArmorAimPhase::WholeCarCenter : ArmorAimPhase::WholeCarArmor;
            overflow_count = 0;
        }
        break;

    case ArmorAimPhase::WholeCarCenter:
        // 最高档只降不升：角速度跌破 pair_center_down(15.0) 时负向累积，否则清零
        overflow_count = (abs_v_yaw < cfg.pair_center_down) ? overflow_count + 1 : 0;
        if (overflow_count > cfg.transfer_thresh) {
            phase          = ArmorAimPhase::WholeCarPair; // 连续低于阈值 → 退回双板
            overflow_count = 0;
        }
        break;
    }
}

} // namespace fcs::L4
