/**
 * @file fire_decision.hpp
 * @brief L5武器层开火决策模块
 *
 * 本文件实现了开火决策逻辑,用于判断云台是否已瞄准目标。
 * 核心思想是将物理射击窗口(米)转换为角度容差,并与当前云台姿态进行比较。
 * 这是L4瞄准系统和L5开火门的统一决策接口。
 */

#pragma once

#include "core/math/normalize.hpp"

#include <algorithm>
#include <cmath>

namespace fcs::L5 {

/**
 * @brief 开火决策配置参数
 *
 * 定义了射击窗口的物理尺寸和角度阈值。
 * 这些参数直接影响开火判断的灵敏度和容错范围。
 */
struct FireDecisionConfig {
    /// 最小射击角度阈值(弧度),用于远距离目标
    /// 当距离很远时,角度容差会很小,该阈值提供下限保护
    /// 默认值0.003弧度(~0.17度)与sp_vision_25规划器对齐
    double fire_thresh{0.003};

    /// 射击窗口高度(米),装甲板物理尺寸
    double shooting_range_h{0.12};

    /// 小装甲板射击窗口宽度(米)
    double shooting_range_w_small{0.12};

    /// 大装甲板射击窗口宽度(米)
    double shooting_range_w_large{0.24};
};

/**
 * @brief 开火决策结果
 *
 * 包含开火判断结果和详细的误差分析数据。
 * 用于调试和可视化决策过程。
 */
struct FireDecision {
    /// 是否可以开火(云台已瞄准目标)
    bool fire;

    /// yaw方向误差(弧度,绝对值)
    double yaw_error;

    /// pitch方向误差(弧度,绝对值)
    double pitch_error;

    /// yaw方向允许的射击范围(弧度)
    double shooting_range_yaw;

    /// pitch方向允许的射击范围(弧度)
    double shooting_range_pitch;
};

/**
 * @brief 判断目标是否在可射击范围内
 *
 * 核心算法原理:
 * 1. 将物理射击窗口(米)通过几何关系转换为角度容差
 * 2. 考虑距离因素:近距离时角度容差大,远距离时角度容差小
 * 3. 设置角度容差下限,防止远距离时容差过小
 * 4. 归一化角度误差,处理周期性边界问题
 *
 * 应用场景:
 * - L4瞄准系统:用于判断是否需要调整云台
 * - L5开火门:用于最终决定是否开火
 *
 * @param cfg 开火决策配置参数
 * @param cur_yaw 当前云台yaw角度(弧度)
 * @param cur_pitch 当前云台pitch角度(弧度)
 * @param target_yaw 目标yaw角度(弧度)
 * @param target_pitch 目标pitch角度(弧度)
 * @param distance 目标距离(米)
 *
 * @return FireDecision结构体,包含开火判断结果和详细误差数据
 *
 * @note 性能优化:使用inline和noexcept,适合高频调用场景
 * @warning pitch符号约定:当前pitch取负值是因为坐标系定义不同
 */
[[nodiscard]] inline FireDecision is_on_target(
    const FireDecisionConfig& cfg, double cur_yaw, double cur_pitch, double target_yaw,
    double target_pitch, double distance) noexcept {

    // 步骤1: 将物理窗口尺寸转换为角度容差
    // 使用反正切计算角度: tan(angle) = (window_size/2) / distance
    // 注意: atan2比atan更安全,能处理distance=0的情况
    auto shooting_range_yaw   = std::abs(std::atan2(cfg.shooting_range_w_small / 2.0, distance));
    auto shooting_range_pitch = std::abs(std::atan2(cfg.shooting_range_h / 2.0, distance));

    // 步骤2: 设置最小角度阈值
    // 防止远距离时角度容差过小,导致无法命中
    // 这是一个经验值,根据实际弹道特性调整
    const auto max_error = cfg.fire_thresh;
    shooting_range_yaw   = std::max(shooting_range_yaw, max_error);
    shooting_range_pitch = std::max(shooting_range_pitch, max_error);

    // 步骤3: 计算角度误差(归一化到[-π, π])
    // 归一化处理周期性边界,例如 -179度和+179度实际只差2度
    auto yaw_error   = std::abs(core::math::normalize_angle(target_yaw - cur_yaw));
    auto pitch_error = std::abs(core::math::normalize_angle(target_pitch - cur_pitch));

    // 步骤4: 综合判断yaw和pitch是否都在允许范围内
    // 只有yaw和pitch同时满足条件才开火
    return FireDecision{
        .fire = std::abs(core::math::normalize_angle(target_yaw - cur_yaw)) < shooting_range_yaw
             && std::abs(core::math::normalize_angle(target_pitch - cur_pitch))
                    < shooting_range_pitch,
        .yaw_error            = yaw_error,
        .pitch_error          = pitch_error,
        .shooting_range_yaw   = shooting_range_yaw,
        .shooting_range_pitch = shooting_range_pitch};
}
} // namespace fcs::L5
