#pragma once
// 弹道抽象基类 BallisticModel、落点结果 ImpactResult 头文件
#include "ballistic_model.hpp"

// 数学函数 exp、cos、sin、isfinite
#include <cmath>
// std::string_view
#include <string>

namespace fcs::core::trajectory::model {

// ============================================================================
// 线性空气阻力弹道模型（带解析闭式解）
// ============================================================================

/**
 * @brief 线性空气阻力弹道模型，final禁止继承，实现BallisticModel抽象接口
 * 物理假设：空气阻力加速度与弹丸瞬时速度成正比，存在指数解析解，无需数值迭代
 * 适用低速弹丸，阻力简化为线性项，计算开销远小于二次阻力模型
 */
class LinearDragModel final : public BallisticModel {
public:
    /**
     * @brief 模型配置参数结构体
     * @param gravity 重力加速度 m/s²，默认9.8
     * @param resistance 线性阻力系数，单位 1/m，代表速度衰减比例
     */
    struct ResistanceParams {
        double gravity{9.8};      ///< Gravity acceleration (m/s²)
        double resistance{0.001}; ///< Linear resistance coefficient (1/m)
    };

    /**
     * @brief 构造函数，接收完整参数结构体初始化
     * 不提供无参构造，规避多默认成员初始化编译警告
     * @param params 重力、阻力系数配置包
     */
    explicit LinearDragModel(const ResistanceParams& params) noexcept
        : gravity_(params.gravity)
        , resistance_(params.resistance) {}

    /**
     * @brief 静态工厂方法，快速生成使用默认参数的模型实例
     * @return 填充默认gravity=9.8、resistance=0.001的LinearDragModel对象
     */
    [[nodiscard]] static LinearDragModel create_default() noexcept {
        return LinearDragModel{ResistanceParams{}};
    }

    /**
     * @brief 重写基类虚接口：给定目标水平射程，求解落点高度、飞行时间
     * @param range 目标水平距离（米）
     * @param pitch 发射俯仰角（弧度，向上为正）
     * @param v0 枪口初速度 m/s
     * @return std::optional<ImpactResult>
     *         有效：弹道计算结果；nullopt：无解（垂直发射、数值溢出、非法输入）
     */
    [[nodiscard]] std::optional<ImpactResult>
        compute_impact(double range, double pitch, double v0) const noexcept override {
        // 初速度趋近0 / 负射程，无有效弹道，返回空结果结构体
        if (v0 < 1e-6 || range < 0.0) {
            return ImpactResult{};
        }

        // 限制阻力系数下限，防止r趋近0退化为无穷大运算，最小1e-4
        const double r         = resistance_ < 1e-4 ? 1e-4 : resistance_;
        const double cos_pitch = std::cos(pitch);
        // cos(pitch)接近0，垂直发射无水平分量，无法抵达水平射程，无解
        if (std::abs(cos_pitch) < 1e-6) {
            return std::nullopt;
        }

        // 线性阻力解析解飞行时间公式：t = (exp(r*x) - 1) / (r * v0 * cosθ)
        const double tof = (std::exp(r * range) - 1.0) / (r * v0 * cos_pitch);
        // 飞行时间为负 / 浮点数溢出无穷，判定计算失效
        if (tof < 0.0 || !std::isfinite(tof)) {
            return std::nullopt;
        }

        // 竖直方向运动方程：无阻力匀加速（仅线性阻力作用于水平方向，竖直忽略阻力）
        const double z = v0 * std::sin(pitch) * tof - 0.5 * gravity_ * tof * tof;

        // x=range：解析解以目标射程为基准计算，输出水平距离等于输入range
        return ImpactResult{.z = z, .tof = tof, .x = range};
    }

    /**
     * @brief 重写基类接口，返回模型标识名称，日志调试区分模型
     */
    [[nodiscard]] std::string_view model_name() const noexcept override { return "Resistance"; }

    /**
     * @brief 读取当前配置重力加速度
     */
    [[nodiscard]] double gravity() const noexcept { return gravity_; }
    /**
     * @brief 读取当前线性阻力系数
     */
    [[nodiscard]] double resistance() const noexcept { return resistance_; }

private:
    double gravity_;    ///< 重力加速度 g m/s²
    double resistance_;///< 线性阻力系数 r (1/m)
};

} // namespace fcs::core::trajectory::model