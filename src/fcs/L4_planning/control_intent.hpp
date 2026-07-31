/**
 * @file control_intent.hpp
 * @brief L4→L5控制意图数据结构
 *
 * 本文件定义了L4规划层向L5武器控制层传递的控制意图。
 * 使用std::variant表达三种互斥的控制模式，实现类型安全的命令传递。
 *
 * 核心设计理念：
 * - 用variant表达互斥状态，避免enum + nullable payload的反模式
 * - 编译期类型检查，消除运行时错误
 * - variant本身就是规范，无需额外文档说明
 *
 * 三种控制模式：
 * 1. TrackCommand：完整轨迹跟踪，用于MPC优化（正常跟踪模式）
 * 2. ShotCommand：直接瞄准，不构建轨迹（能量机关、预跟踪、降级模式）
 * 3. HoldCommand：无目标，保持当前位置（待机模式）
 *
 * 降级机制：
 * - TrackCommand构建失败时，自动降级为ShotCommand
 * - ShotCommand.degradation_reason记录降级原因
 * - nullopt表示有意为之的直接射击（如能量机关）
 *
 * 数据流：
 * L4::try_armor() → ControlIntent → L5武器控制层
 */

#pragma once

#include "core/trajectory/reference_trajectory.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

namespace fcs::L4 {

// ============================================================================
// L4 → L5控制意图
// ============================================================================
//
// L4将跟踪器状态转换为纯控制意图，传递给L5武器控制层。
// variant构造函数精确编码三种操作模式：
//
//   TrackCommand  = "我预测了一条轨迹，用MPC跟踪它"
//   ShotCommand   = "目标在这里，直接瞄准射击"
//   HoldCommand   = "无目标，保持当前位置"
//
// ShotCommand的degradation_reason仅在轨迹构建失败且L4降级为直接瞄准时设置。
// 正常的直接射击（能量机关、预跟踪等）保持nullopt，
// 以便下游区分有意为之的射击和降级射击。
//
// 无需enum标签，无需运行时检查。variant本身就是规范。

/**
 * @brief 轨迹跟踪命令
 *
 * 包含完整的参考轨迹，用于L5的MPC轨迹优化。
 * 这是正常跟踪模式下的控制命令，提供最优的控制性能。
 *
 * 轨迹类型：
 * - control_trajectory：控制轨迹（可能使用中心代理点）
 * - fire_trajectory：射击轨迹（始终瞄准真实装甲板）
 *
 * 分离原因：
 * - WholeCarCenter模式下，控制轨迹瞄准车身中心（用于平滑跟踪）
 * - 但火力门控需要判断是否能击中真实装甲板
 * - 因此需要两条轨迹分别用于控制和射击判断
 */
struct TrackCommand {
    uint64_t timestamp_ns{0};                       ///< 时间戳（纳秒）
    core::trajectory::ReferenceTrajectory control_trajectory; ///< 控制轨迹（用于MPC）
    core::trajectory::ReferenceTrajectory fire_trajectory;     ///< 射击轨迹（用于火力门控）
};

/**
 * @brief 直接射击命令
 *
 * 不构建轨迹，直接指定瞄准角度。
 * 用于：
 * - 能量机关目标（不需要轨迹预测）
 * - 预跟踪阶段（目标刚出现，轨迹尚未稳定）
 * - 降级模式（轨迹构建失败时的后备方案）
 *
 * 降级标记：
 * - degradation_reason有值：表示轨迹构建失败，降级为直接射击
 * - degradation_reason为空：表示有意为之的直接射击（如能量机关）
 */
struct ShotCommand {
    uint64_t timestamp_ns{0};                       ///< 时间戳（纳秒）
    double yaw{0.0};                                ///< yaw角度（弧度）
    double pitch{0.0};                              ///< pitch角度（弧度）
    double distance{0.0};                           ///< 目标距离（米）

    /**
     * @brief 降级原因
     *
     * 当轨迹构建失败且L4降级为直接瞄准时设置。
     * nullopt表示有意为之的直接射击（能量机关、预跟踪等）。
     */
    std::optional<std::string> degradation_reason{};
};

/**
 * @brief 保持命令
 *
 * 无目标时的控制命令，指示云台保持当前位置。
 * 这是待机模式的控制命令，避免云台乱动。
 */
struct HoldCommand {
    uint64_t timestamp_ns{0};                       ///< 时间戳（纳秒）
};

/**
 * @brief L4→L5控制意图
 *
 * 使用variant表达三种互斥的控制模式，提供类型安全的命令传递。
 * 这是L4规划层的核心输出，驱动L5武器控制层的行为。
 */
using ControlIntent = std::variant<TrackCommand, ShotCommand, HoldCommand>;

} // namespace fcs::L4
