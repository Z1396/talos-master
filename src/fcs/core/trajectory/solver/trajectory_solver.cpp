#include "core/trajectory/solver/trajectory_solver.hpp"

// 数学库：atan2、hypot、pi、fabs等
#include <cmath>
// 错误返回容器 std::expected
#include <expected>
// 错误信息字符串
#include <string>
// 格式化日志、错误文本
#include <fmt/format.h>

namespace fcs::core::trajectory::solver {

// ============================================================================
// DirectSolver 求解器实现（解析模型专用迭代修正求解器）
// ============================================================================

/**
 * @brief 反向求解瞄准角：给定目标世界坐标，迭代修正俯仰角直至落点高度匹配目标高度
 * 适配IdealModel/LinearDragModel这类拥有快速compute_impact解析解的弹道模型
 * @param target_pos 目标三维世界坐标 (x,y,z)
 * @param v0 枪口初速度 m/s
 * @return 收敛后的瞄准解 / 迭代超限、角度越界、模型计算失败错误
 */
std::expected<AimSolution, std::string>
    DirectSolver::solve(const Eigen::Vector3d& target_pos, double v0) const noexcept {
    // 目标高度（Z轴）
    const double target_height = target_pos.z();
    // 水平直线距离 sqrt(x²+y²)
    const double distance      = std::hypot(target_pos.x(), target_pos.y());

    // 目标距离极近，无弹道意义，直接报错
    if (distance < 1e-6) {
        return std::unexpected(
            fmt::format("DirectSolver::solve: target too close, distance={:.6f}m", distance));
    }

    // 迭代超参数固定常量
    constexpr int kMaxIterations      = 20;       // 最大迭代次数上限
    constexpr double kHeightTolerance = 0.01;      // 高度收敛阈值 0.01米
    constexpr double kMaxPitch        = std::numbers::pi / 2.5; // 最大允许俯仰角，约72度

    double iterative_height = target_height; // 迭代修正虚拟高度
    double angle            = std::atan2(target_height, distance); // 初始俯仰角猜测
    double impact_height    = 0.0; // 模型计算出的落点高度
    double dh               = 0.0; // 目标高度与落点高度差值（修正量）
    double tof              = 0.0; // 当前迭代对应的飞行时间
    int iterations          = 0; // 当前迭代步数

    // 迭代循环修正俯仰角
    for (int i = 0; i < kMaxIterations; ++i) {
        iterations = i + 1;
        // 使用修正后的虚拟高度重新计算俯仰角
        angle      = std::atan2(iterative_height, distance);

        // 俯仰角超出安全上限，判定无解
        if (std::abs(angle) > kMaxPitch) {
            return std::unexpected(
                fmt::format(
                    "DirectSolver::solve: pitch angle {:.4f}rad exceeds maximum {:.4f}rad at "
                    "iteration {}",
                    angle, kMaxPitch, iterations));
        }

        // 调用底层弹道模型正向计算该角度下的落点
        const auto impact = model_->compute_impact(distance, angle, v0);
        // 模型计算失败（垂直发射、数值溢出等），返回错误
        if (!impact) {
            return std::unexpected(
                fmt::format(
                    "DirectSolver::solve: model compute_impact failed at distance={:.3f}m, "
                    "angle={:.4f}rad, v0={:.1f}m/s, iteration={}",
                    distance, angle, v0, iterations));
        }

        impact_height = impact->z;
        tof           = impact->tof;
        // 计算高度误差：目标高度 - 计算落点高度
        dh            = target_height - impact_height;

        // 误差小于收敛阈值，求解成功，组装返回结果
        if (std::abs(dh) < kHeightTolerance) {
            return AimSolution{
                // 水平方位角 yaw = atan2(y, x)
                .yaw            = std::atan2(target_pos.y(), target_pos.x()),
                .pitch          = angle,
                .time_of_flight = tof,
                .iterations     = iterations};
        }

        // 修正虚拟高度，下一轮迭代缩小误差
        iterative_height += dh;
    }

    // 达到最大迭代次数仍未收敛，返回失败
    return std::unexpected(
        fmt::format(
            "DirectSolver::solve: failed to converge after {} iterations, final dh={:.4f}m, "
            "tolerance={:.4f}m",
            kMaxIterations, std::abs(dh), kHeightTolerance));
}

/**
 * @brief 正向生成弹道离散采样点，用于Foxglove可视化曲线绘制
 * 按固定步长0.03米采样每个水平距离对应的高度z
 * @param pitch 发射俯仰角
 * @param v0 初速度 m/s
 * @param max_distance 最大采样水平距离
 * @return 向量，每一组pair(水平距离x, 高度z)
 */
std::vector<std::pair<double, double>>
    DirectSolver::generate_trajectory(double pitch, double v0, double max_distance) const noexcept {
    // 采样步长 3cm
    constexpr double kTrajectoryStep = 0.03;

    // 最大距离为负，返回空轨迹
    if (max_distance < 0.0) {
        return {};
    }

    // 预计算采样点数量，预分配内存避免扩容
    const size_t num_points = static_cast<size_t>(max_distance / kTrajectoryStep) + 1;
    std::vector<std::pair<double, double>> trajectory;
    trajectory.reserve(num_points);

    // 逐段采样
    for (double x = 0; x < max_distance; x += kTrajectoryStep) {
        const auto impact = model_->compute_impact(x, pitch, v0);
        // 计算失败则高度置0
        const double z    = impact ? impact->z : 0.0;
        trajectory.emplace_back(x, z);
    }

    return trajectory;
}

// ============================================================================
// IterativeSolver 求解器实现（可配置迭代参数通用迭代求解器）
// ============================================================================

/**
 * @brief 可配置迭代求解器反向瞄准求解，直接复用detail通用迭代函数
 * @param target_pos 目标三维坐标
 * @param v0 初速度
 * @return 收敛瞄准解 / 错误信息
 */
std::expected<AimSolution, std::string>
    IterativeSolver::solve(const Eigen::Vector3d& target_pos, double v0) const noexcept {
    // 转发至底层通用迭代工具函数，传入求解器配置里的迭代上限、高度阈值、最大俯仰角
    return detail::iterative_solve_pitch(
        *model_, target_pos, v0, static_cast<int>(config_.max_iterations), config_.height_tolerance,
        config_.max_pitch);
}

/**
 * @brief 生成可视化弹道采样点，逻辑与DirectSolver完全一致
 */
std::vector<std::pair<double, double>> IterativeSolver::generate_trajectory(
    double pitch, double v0, double max_distance) const noexcept {
    constexpr double kTrajectoryStep = 0.03;

    if (max_distance < 0.0) {
        return {};
    }

    const size_t num_points = static_cast<size_t>(max_distance / kTrajectoryStep) + 1;
    std::vector<std::pair<double, double>> trajectory;
    trajectory.reserve(num_points);

    for (double x = 0; x < max_distance; x += kTrajectoryStep) {
        const auto impact = model_->compute_impact(x, pitch, v0);
        const double z    = impact ? impact->z : 0.0;
        trajectory.emplace_back(x, z);
    }

    return trajectory;
}

// ============================================================================
// 底层通用工具函数 detail 命名空间
// ============================================================================

namespace detail {

/**
 * @brief 通用俯仰角迭代求解核心逻辑（DirectSolver与IterativeSolver复用）
 * 抽离公共迭代逻辑，消除代码重复
 * @param model 底层弹道模型只读引用
 * @param target_pos 目标三维坐标
 * @param v0 初速度
 * @param max_iterations 最大迭代次数上限
 * @param height_tolerance 高度收敛阈值
 * @param max_pitch 允许最大俯仰角
 * @return 收敛瞄准解 / 各类失败错误字符串
 */
std::expected<AimSolution, std::string> iterative_solve_pitch(
    const model::BallisticModel& model, const Eigen::Vector3d& target_pos, double v0,
    int max_iterations, double height_tolerance, double max_pitch) noexcept {

    const double target_height = target_pos.z();
    const double distance      = std::hypot(target_pos.x(), target_pos.y());
    // 水平方位角提前计算，求解成功直接填入结果
    const double yaw           = std::atan2(target_pos.y(), target_pos.x());

    // 目标距离过近，无弹道求解意义
    if (distance < 1e-6) {
        return std::unexpected(
            fmt::format("iterative_solve_pitch: target too close, distance={:.6f}m", distance));
    }

    double iterative_height = target_height;
    double angle            = std::atan2(target_height, distance);
    double impact_height    = 0.0;
    double dh               = 0.0;
    double tof              = 0.0;
    int iterations          = 0;

    // 迭代修正俯仰角循环
    for (int i = 0; i < max_iterations; ++i) {
        iterations = i + 1;
        angle      = std::atan2(iterative_height, distance);

        // 俯仰角超出安全上限，求解失败
        if (std::abs(angle) > max_pitch) {
            return std::unexpected(
                fmt::format(
                    "iterative_solve_pitch: pitch angle {:.4f}rad exceeds maximum {:.4f}rad at "
                    "iteration {}",
                    angle, max_pitch, iterations));
        }

        // 调用模型正向计算落点
        const auto impact = model.compute_impact(distance, angle, v0);
        if (!impact) {
            return std::unexpected(
                fmt::format(
                    "iterative_solve_pitch: model compute_impact failed at distance={:.3f}m, "
                    "angle={:.4f}rad, v0={:.1f}m/s, iteration={}",
                    distance, angle, v0, iterations));
        }

        impact_height = impact->z;
        tof           = impact->tof;
        dh            = target_height - impact_height;

        // 高度误差满足阈值，求解完成
        if (std::abs(dh) < height_tolerance) {
            return AimSolution{
                .yaw = yaw, .pitch = angle, .time_of_flight = tof, .iterations = iterations};
        }

        // 修正虚拟高度，缩小下一轮误差
        iterative_height += dh;
    }

    // 迭代耗尽未收敛
    return std::unexpected(
        fmt::format(
            "iterative_solve_pitch: failed to converge after {} iterations, final dh={:.4f}m, "
            "tolerance={:.4f}m",
            max_iterations, std::abs(dh), height_tolerance));
}

} // namespace detail

} // namespace fcs::core::trajectory::solver