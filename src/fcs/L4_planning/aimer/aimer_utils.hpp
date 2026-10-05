/**
 * @file aimer_utils.hpp
 * @brief L4规划层瞄准工具函数
 *
 * 本文件提供瞄准系统的核心算法工具，包括：
 * - 弹道飞行时间迭代精化
 * - 瞄准点计算
 *
 * 核心算法：
 * 弹道飞行时间迭代精化（refine_flying_time）：
 * - 问题：弹道求解需要飞行时间，但飞行时间取决于目标位置，而目标位置又在运动
 * - 解决：迭代求解，初始估计→求解弹道→更新飞行时间→重新预测位置→收敛
 * - 收敛判据：飞行时间变化小于1ms
 * - 最大迭代次数：7次（经验值，通常3-5次收敛）
 *
 * 性能优化：
 * - 减少最大迭代次数从20到7，约65%计算量减少
 * - 使用模板函数避免虚函数调用
 * - 内联小函数
 */
#pragma once
#include <Eigen/Core>
#include "core/trajectory/solver/solver_interfaces.hpp"
#include <algorithm>
#include <cmath>
#include <expected>
namespace fcs::L4 {
/// 最大飞行时间精化迭代次数
/// 弹道问题通常在3-5次迭代内收敛
/// 从20减少到7，约65%计算量减少
inline constexpr int kMaxFlyingTimeRefineIter = 7;
/// 飞行时间收敛容差（秒）
/// 1ms容差对应约1mm高度误差（对于1m/s垂直目标运动）
inline constexpr double kFlyingTimeRefineTolerance = 1e-3;
/**
 * @brief 计算瞄准点（yaw, pitch）
 *
 * 给定目标位置（枪口坐标系）和弹速，计算瞄准角度。
 *
 * @param solver 弹道求解器
 * @param target_pos_muzzle 枪口坐标系下的目标位置
 * @param bullet_speed 弹速（米/秒）
 * @return 成功时返回{yaw, pitch}（弧度），失败时返回错误信息
 *
 * @note 这是简单包装函数，直接调用solver.solve()
 */
[[nodiscard]] inline std::expected<Eigen::Vector2d, std::string> compute_aim_point(
    const core::trajectory::solver::TrajectorySolver& solver,
    const Eigen::Vector3d& target_pos_muzzle, double bullet_speed) noexcept {
    // 调用弹道求解器，输入枪口下目标三维坐标、子弹初速，求解弹道
    const auto solution = solver.solve(target_pos_muzzle, bullet_speed);
    // 判断弹道求解是否失败（无解/异常）
    if (!solution.has_value()) {
        // 失败，把求解器返回的错误信息向上传递
        return std::unexpected(solution.error());
    }
    // 求解成功，把弹道解算出来的偏航、俯仰角打包成2维Eigen向量返回（单位弧度）
    return Eigen::Vector2d{solution->yaw, solution->pitch};
}
/**
 * @brief 执行迭代飞行时间精化
 *
 * 核心算法：迭代求解弹道飞行时间
 *
 * 迭代过程：
 * 1. 初始估计：飞行时间 = 距离 / 弹速
 * 2. 预测位置：根据初始延迟+飞行时间预测目标位置
 * 3. 弹道求解：求解该位置需要的飞行时间
 * 4. 收敛检查：如果飞行时间变化小于容差，收敛
 * 5. 更新飞行时间，重复步骤2-4
 *
 * 收敛保证：
 * - 对于匀速运动的目标，通常3-5次迭代收敛
 * - 对于机动目标，可能需要更多迭代
 * - 设置最大迭代次数避免无限循环
 *
 * @tparam PredictFn 预测函数类型（签名：Eigen::Vector3d(double dt)）
 * @param solver 弹道求解器
 * @param predict_fn 预测函数（输入延迟时间，输出目标位置）
 * @param initial_delay 初始延迟（秒）
 * @param bullet_speed 弹速（米/秒）
 * @return 精化后的总延迟（initial_delay + flying_time）
 *
 * @note 使用模板函数避免虚函数调用，提高性能
 *
 * 使用示例：
 * @code
 * auto refine_total_prediction_time = [&](int armor_id) noexcept {
 *     return refine_flying_time(
 *         solver,
 *         [&](double dt) -> Eigen::Vector3d {
 *             return predict_aim_point(target, dt, context, muzzle, armor_id).position - muzzle_pos;
 *         },
 *         total_delay, bullet_speed);
 * };
 * @endcode
 */
template <class PredictFn>
[[nodiscard]] double refine_flying_time(
    const core::trajectory::solver::TrajectorySolver& solver, PredictFn&& predict_fn,
    double initial_delay, double bullet_speed) noexcept {
    // 用【初始延迟】预测目标当前位置（还未叠加子弹飞行时间）
    const Eigen::Vector3d current_pos = predict_fn(initial_delay);
    // 计算枪口到目标当前位置的直线欧氏距离
    const double distance             = current_pos.norm();
    // 保护：弹速下限兜底，防止除0崩溃，最小限制0.001m/s
    const double safe_bullet_speed    = std::max(1e-3, bullet_speed);
    // 飞行时间初始猜测：直线距离 / 子弹初速（不考虑重力、目标运动的粗估计）
    double flying_time                = distance / safe_bullet_speed;
    // 保存上一轮有效的飞行时间，迭代失败时可以回退到上一个合法值，防止返回NaN
    double last_valid_flying_time     = flying_time;

    // 迭代循环，最多执行kMaxFlyingTimeRefineIter=7次
    for (int i = 0; i < kMaxFlyingTimeRefineIter; ++i) {
        // 总预测时长 = 图像/传输初始延迟 + 子弹飞行时间；预测该总时长后的目标位置
        const Eigen::Vector3d predicted_pos = predict_fn(initial_delay + flying_time);
        // 代入弹道求解器，基于预测出来的目标位置，重新计算真实弹道飞行时间
        const auto solution                 = solver.solve(predicted_pos, bullet_speed);

        // 合法性校验：弹道求解失败 或者 飞行时间不是有限数字 或者飞行时间<=0（物理非法）
        if (!solution.has_value() || !std::isfinite(solution->time_of_flight)
            || solution->time_of_flight <= 0.0) {
            // 求解异常，直接跳出迭代，使用上一轮保存的合法飞行时间
            break;
        }

        // 收敛判断：本次新算出飞行时间 和 上一轮飞行时间差值小于阈值1ms
        if (std::abs(solution->time_of_flight - flying_time) < kFlyingTimeRefineTolerance) {
            // 满足收敛条件，更新有效飞行时间
            last_valid_flying_time = solution->time_of_flight;
            // 跳出迭代，不用继续循环
            break;
        }

        // 不满足收敛条件：更新飞行时间，进入下一轮迭代
        flying_time            = solution->time_of_flight;
        // 记录本轮的合法飞行时间，作为兜底备份
        last_valid_flying_time = flying_time;
    }
    // 返回【总预测时间】= 系统初始延迟 + 迭代精化后的子弹飞行时间
    return initial_delay + last_valid_flying_time;
}
} // namespace fcs::L4
