#pragma once
// 弹道物理模型抽象基类 BallisticModel
#include "core/trajectory/model/ballistic_model.hpp"
// 求解器顶层虚接口 TrajectorySolver、AimSolution 定义
#include "core/trajectory/solver/solver_interfaces.hpp"

// Eigen三维向量，目标世界坐标存储
#include <Eigen/Core>

// 独占智能指针 std::unique_ptr、std::make_unique
#include <memory>
// 圆周率数学常量
#include <numbers>
// std::pair 轨迹点存储
#include <utility>
// 动态数组，存储可视化轨迹采样点
#include <vector>

namespace fcs::core::trajectory::solver {

// ============================================================================
// DirectSolver 解析解求解器（适配IdealModel / LinearDragModel 闭式模型）
// ============================================================================

/**
 * @brief 解析解专用求解器，final禁止继承，实现 TrajectorySolver 虚接口
 * 仅适配拥有快速闭式compute_impact的弹道模型：理想无阻力、线性空气阻力模型
 * 优势：迭代收敛速度极快，计算精度高，算力开销远低于通用迭代求解器
 */
class DirectSolver final : public TrajectorySolver {
public:
    /**
     * @brief 模板构造函数，接收任意BallisticModel派生模型右值独占指针
     * C++20 requires约束：强制入参必须是BallisticModel子类，编译期校验类型合法
     * @param model 弹道模型独占指针，所有权转移至求解器
     */
    template <typename Model>
    requires(std::derived_from<Model, model::BallisticModel>)
    explicit DirectSolver(std::unique_ptr<Model>&& model) noexcept
        : model_(std::move(model)) {}

    /**
     * @brief 静态工厂方法，快速生成DirectSolver独占指针，统一创建入口
     * @param model 任意弹道模型基类独占指针
     * @return 封装好求解器的unique_ptr<TrajectorySolver>多态指针
     */
    [[nodiscard]] static std::unique_ptr<DirectSolver>
        create(std::unique_ptr<model::BallisticModel> model) noexcept {
        return std::make_unique<DirectSolver>(std::move(model));
    }

    /**
     * @brief 重写虚接口：反向迭代求解命中目标所需发射俯仰/方位角
     * @param target_pos 世界坐标系目标三维坐标
     * @param v0 枪口初速度 m/s
     * @return 收敛瞄准解 / 迭代超限、角度越界、模型计算失败错误字符串
     */
    [[nodiscard]] std::expected<AimSolution, std::string>
        solve(const Eigen::Vector3d& target_pos, double v0) const noexcept override;

    /**
     * @brief 重写虚接口：正向批量生成弹道离散采样点，用于Foxglove可视化曲线
     * @param pitch 发射俯仰角（弧度）
     * @param v0 初速度 m/s
     * @param max_distance 最大水平采样距离（米）
     * @return vector<(x,z)> 水平距离+高度采样序列
     */
    [[nodiscard]] std::vector<std::pair<double, double>>
        generate_trajectory(double pitch, double v0, double max_distance) const noexcept override;

    /**
     * @brief 重写虚接口：返回求解器标识名称，日志/调试区分求解器类型
     */
    [[nodiscard]] std::string_view solver_name() const noexcept override { return "Direct"; }

    /**
     * @brief 只读获取底层弹道模型独占指针，供上层业务读取模型参数
     */
    [[nodiscard]] const std::unique_ptr<model::BallisticModel>& model() const noexcept {
        return model_;
    }

    /**
     * @brief 重写虚接口：返回底层弹道模型裸指针，替代不安全dynamic_cast，无运行时开销
     */
    [[nodiscard]] const model::BallisticModel* get_model() const noexcept override {
        return model_.get();
    }

private:
    std::unique_ptr<model::BallisticModel> model_; ///< 底层弹道物理模型独占资源
};

// ============================================================================
// IterativeSolver 通用迭代求解器（适配任意弹道模型，含二次阻力模型）
// ============================================================================

/**
 * @brief 通用迭代逼近求解器，final禁止继承，实现TrajectorySolver虚接口
 * 兼容任意BallisticModel实现（包括无解析解的二次空气阻力模型）
 * 迭代逻辑：视线角初始猜测 → 正向计算落点高度 → 修正俯仰角直至收敛
 * 完全无状态，所有迭代约束通过内部Config配置结构体存储
 */
class IterativeSolver final : public TrajectorySolver {
public:
    /**
     * @brief 求解器迭代参数配置结构体
     * @param max_iterations 迭代次数上限
     * @param height_tolerance 高度收敛阈值（米），误差小于此判定收敛
     * @param max_pitch 允许最大俯仰角（弧度），超出直接判定无解
     */
    struct Config {
        uint32_t max_iterations{20};              ///< Maximum solving iterations
        double height_tolerance{0.01};            ///< Convergence threshold (meters)
        double max_pitch{std::numbers::pi / 2.5}; ///< Maximum valid pitch (radians)
    };

    /**
     * @brief 模板构造函数，接收弹道模型+迭代配置
     * requires编译期约束入参模型必须继承BallisticModel
     * @param model 弹道模型独占指针，所有权转移
     * @param config 迭代收敛约束配置
     */
    template <typename Model>
    requires(std::derived_from<Model, model::BallisticModel>)
    explicit IterativeSolver(std::unique_ptr<Model>&& model, Config config) noexcept
        : model_(std::move(model))
        , config_(config) {}

    /**
     * @brief 静态工厂方法，使用默认Config参数创建求解器实例
     * @param model 弹道模型基类独占指针
     * @return 封装迭代求解器的unique_ptr多态指针
     */
    [[nodiscard]] static std::unique_ptr<IterativeSolver>
        create(std::unique_ptr<model::BallisticModel> model) noexcept {
        return std::make_unique<IterativeSolver>(std::move(model), Config{});
    }

    /**
     * @brief 重写虚接口：反向迭代求解瞄准角，复用detail通用迭代函数
     * @param target_pos 目标三维世界坐标
     * @param v0 枪口初速度 m/s
     * @return 收敛瞄准解 / 各类失败错误字符串
     */
    [[nodiscard]] std::expected<AimSolution, std::string>
        solve(const Eigen::Vector3d& target_pos, double v0) const noexcept override;

    /**
     * @brief 重写虚接口：正向生成可视化弹道采样点，逻辑同DirectSolver
     */
    [[nodiscard]] std::vector<std::pair<double, double>>
        generate_trajectory(double pitch, double v0, double max_distance) const noexcept override;

    /**
     * @brief 重写虚接口：返回求解器名称标识
     */
    [[nodiscard]] std::string_view solver_name() const noexcept override { return "Iterative"; }

    /**
     * @brief 只读获取底层弹道模型独占指针
     */
    [[nodiscard]] const std::unique_ptr<model::BallisticModel>& model() const noexcept {
        return model_;
    }

    /**
     * @brief 只读获取迭代求解器配置参数
     */
    [[nodiscard]] const Config& config() const noexcept { return config_; }

    /**
     * @brief 重写虚接口：返回底层弹道模型裸指针，类型安全无dynamic_cast
     */
    [[nodiscard]] const model::BallisticModel* get_model() const noexcept override {
        return model_.get();
    }

private:
    std::unique_ptr<model::BallisticModel> model_; ///< 底层弹道物理模型
    Config config_; ///< 迭代收敛约束配置（最大迭代、误差阈值、最大俯仰角）
};

// ============================================================================
// 全局工具函数命名空间 detail（公共迭代逻辑抽离，消除代码重复）
// ============================================================================

namespace detail {

/**
 * @brief 通用无状态俯仰角迭代求解核心纯函数
 * DirectSolver、IterativeSolver共用同一套迭代逻辑，仅入参传递不同约束参数
 * @param model 只读弹道模型引用
 * @param target_pos 目标三维坐标
 * @param v0 枪口初速度
 * @param max_iterations 迭代上限
 * @param height_tolerance 高度收敛阈值
 * @param max_pitch 允许最大俯仰角
 * @return 收敛瞄准解 / 迭代超限、角度越界、模型计算失败错误
 */
[[nodiscard]] std::expected<AimSolution, std::string> iterative_solve_pitch(
    const model::BallisticModel& model, const Eigen::Vector3d& target_pos, double v0,
    int max_iterations, double height_tolerance, double max_pitch) noexcept;

} // namespace detail

} // namespace fcs::core::trajectory::solver