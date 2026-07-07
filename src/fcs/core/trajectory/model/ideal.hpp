#pragma once
// 弹道抽象基类 BallisticModel、ImpactResult 定义
#include "ballistic_model.hpp"

// 数学函数 std::cos / std::sin / std::isfinite
#include <cmath>
// std::string_view
#include <string>

namespace fcs::core::trajectory::model {

// ============================================================================
// 理想弹道模型（无空气阻力，真空抛物线解析解）
// ============================================================================

/**
 * @brief 理想无空气阻力弹道模型，继承 BallisticModel 抽象接口
 * 纯解析闭式解，O(1) 常数时间计算，无迭代，速度极快
 * 物理方程（真空抛体运动）：
 * 水平位移：x(t) = v0 * cosθ * t
 * 竖直高度：z(t) = v0 * sinθ * t - 0.5 * g * t²
 * θ = pitch 发射俯仰角，g 重力加速度
 */
class IdealModel final : public BallisticModel {
public:
    /**
     * @brief 构造理想弹道模型
     * @param gravity 重力加速度，默认 9.8 m/s²
     * @ noexcept 无抛异常
     */
    explicit IdealModel(double gravity = 9.8) noexcept
        : gravity_(gravity) {}

    /**
     * @brief 重写基类接口：计算指定水平射程对应的落点、飞行时间
     * @param range 目标水平距离（米）
     * @param pitch 发射俯仰角（弧度，向上为正）
     * @param v0 枪口初速度 m/s
     * @return std::optional<ImpactResult>
     *         有效：弹道计算结果；nullopt：无解（垂直发射、非法输入）
     */
    [[nodiscard]] std::optional<ImpactResult>
        compute_impact(double range, double pitch, double v0) const noexcept override {
        // 初速度趋近0 或 射程为负数，返回默认空结果（无实际弹道）
        if (v0 < 1e-6 || range < 0.0) {
            return ImpactResult{};
        }

        const double cos_pitch = std::cos(pitch);
        // cos(pitch) 接近0，代表垂直向上/向下发射，不存在水平射程，无解
        if (std::abs(cos_pitch) < 1e-6) {
            return std::nullopt;
        }

        // 解析解直接计算飞行时间 t = 水平距离 / 水平分速度
        const double tof = range / (v0 * cos_pitch);
        // 飞行时间为负 / 非有限浮点数，判定计算失效
        if (tof < 0.0 || !std::isfinite(tof)) {
            return std::nullopt;
        }

        // 代入竖直方向运动方程，计算落点高度 z
        const double z = v0 * std::sin(pitch) * tof - 0.5 * gravity_ * tof * tof;

        // x=range：无空气阻力，水平速度恒定，实际抵达距离等于目标射程
        return ImpactResult{.z = z, .tof = tof, .x = range};
    }

    /**
     * @brief 重写基类接口：返回模型名称字符串，用于日志/调试区分
     */
    [[nodiscard]] std::string_view model_name() const noexcept override { return "Ideal"; }

    /**
     * @brief 获取当前配置的重力加速度
     */
    [[nodiscard]] double gravity() const noexcept { return gravity_; }

private:
    double gravity_; ///< 重力加速度 g，单位 m/s²
};

} // namespace fcs::core::trajectory::model