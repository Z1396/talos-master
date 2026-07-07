#pragma once
// 全局轨迹顶层配置结构体 TrajectoryConfig
#include "../config.hpp"
// 模型通用配置结构体（重力、阻力系数、模型类型枚举 ModelType）
#include "../model/config.hpp"
// 理想无阻力弹道模型 IdealModel 头文件
#include "../model/ideal.hpp"
// 线性空气阻力模型 LinearDragModel 头文件
#include "../model/linear_drag.hpp"
// 求解器抽象接口定义
#include "solver_interfaces.hpp"
// 直接解析解求解器 DirectSolver、TrajectorySolver 基类
#include "trajectory_solver.hpp"

// 智能独占指针 std::unique_ptr、std::make_unique
#include <memory>

namespace fcs::core::trajectory::solver {

/**
 * @brief 轨迹求解器工厂内联函数，根据配置自动生成对应弹道求解器实例
 * @param config 轨迹全局配置，包含模型类型、重力、阻力等参数
 * @return std::unique_ptr<TrajectorySolver> 多态求解器独占智能指针
 * @ noexcept 无抛异常，配置非法无匹配分支时无返回值（编译器强制枚举全分支覆盖）
 * @ [[nodiscard]] 强制处理返回指针，不可忽略求解器实例
 */
[[nodiscard]] inline std::unique_ptr<TrajectorySolver>
    create_solver(const TrajectoryConfig& config) noexcept {
    // 根据配置指定的弹道模型类型分支匹配
    switch (config.model->type) {
    // 分支1：理想无空气阻力模型
    case model::ModelType::Ideal:
        // 1. 构造 IdealModel 物理模型实例，传入配置重力值
        // 2. 外层包裹 DirectSolver 解析求解器，实现 TrajectorySolver 顶层接口
        return std::make_unique<DirectSolver>(
            std::make_unique<model::IdealModel>(config.model->gravity));
    // 分支2：线性空气阻力模型
    case model::ModelType::LinearDrag:
        // 构造 LinearDragModel 所需参数结构体，填充配置里的重力、线性阻力系数
        // 传入模型构造函数，再封装进 DirectSolver 求解器
        return std::make_unique<DirectSolver>(
            std::make_unique<model::LinearDragModel>(model::LinearDragModel::ResistanceParams{
                .gravity = config.model->gravity, .resistance = config.model->resistance}));
    }
}

} // namespace fcs::core::trajectory::solver