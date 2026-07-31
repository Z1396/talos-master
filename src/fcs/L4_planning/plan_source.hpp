/**
 * @file plan_source.hpp
 * @brief L4规划层目标源枚举定义
 *
 * 本文件定义了云台规划的目标源类型。
 * 这是一个简单的枚举类型，用于标识当前正在跟踪的目标类型。
 *
 * 设计理念：
 * - 使用枚举类而非整数，提供类型安全
 * - 使用uint8_t作为底层类型，节省内存
 * - 预留值1和2，便于未来扩展
 *
 * 使用场景：
 * - SelectedTargetSnapshot中使用，标识目标类型
 * - 控制流程中用于分支判断（不同目标源的处理逻辑不同）
 */

#pragma once

#include <cstdint>

namespace fcs::L4 {

/**
 * @brief 云台规划目标源
 *
 * 标识当前云台正在跟踪的目标类型。
 * 不同的目标源对应不同的处理流程和优化策略。
 */
enum class GimbalPlanSource : uint8_t {
    Armor = 1,  ///< 装甲板目标（机器人、前哨站）：需要轨迹跟踪和MPC优化
    Rune  = 2,  ///< 能量机关目标：直接瞄准，不构建轨迹
};

} // namespace fcs::L4
