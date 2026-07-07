// 头文件保护，防止重复包含
#pragma once

// 引入全局基础配置头文件
#include "config.hpp"
// 引入Talos调度器（ECS世界、资源管理核心）
#include "scheduler/scheduler.hpp"
// 弹道求解器配置结构体定义
#include "solver/config.hpp"
// 弹道求解器抽象接口
#include "solver/solver_interfaces.hpp"

// 项目命名空间：fcs框架 -> core核心模块 -> trajectory弹道子模块
namespace fcs::core::trajectory {

/// 弹道资源：存储弹丸初速单一数据
/// 作为全局资源存入ECS World，供所有自瞄、弹道求解系统读取
struct bullet_speed_data {
    // 弹丸初速，单位 m/s
    double bullet_speed;
};

// ==================== ECS资源类型别名封装 ====================
/// trajectory_solver：只读资源，存储弹道求解器智能指针
/// talos::res<T> = ECS只读资源包装器，多线程/多系统共享读取安全
using trajectory_solver = talos::res<std::unique_ptr<solver::TrajectorySolver>>;

/// trajectory_config：弹道模型配置只读资源
/// 封装model::ModelConfig（重力、阻力、迭代次数等物理参数）
using trajectory_config = talos::res<model::ModelConfig>;

/// bullet_speed：弹速 只读资源（读取弹道计算用）
using bullet_speed     = talos::res<bullet_speed_data>;
/// bullet_speed_mut：弹速 可变资源（修改弹速，硬件/配置更新时写入）
using bullet_speed_mut = talos::res_mut<bullet_speed_data>;

/**
 * @brief 弹道模块资源注册函数
 * 将弹道模型配置、弹道求解器实例注册进ECS全局World资源容器
 * @param scheduler Talos全局调度器，持有整个ECS世界
 * @param config 外部传入的完整弹道初始化配置（包含模型参数、求解器参数）
 * @noexcept 函数保证不会抛出异常
 */
constexpr void register_resource(talos::Scheduler& scheduler, TrajectoryConfig&& config) noexcept {
    // 1. 把弹道物理模型参数（重力、阻力、迭代次数等）插入全局资源
    // config.model 内部持有ModelConfig，get()取出内部对象存入World
    scheduler.world().insert_resource(config.model.get());

    // 2. 根据配置创建对应类型弹道求解器实例（Ideal/LinearDrag/RK4）
    auto solver = solver::create_solver(config);

    // 3. 将创建好的求解器智能指针移入ECS全局资源，供系统统一调用求解弹道
    scheduler.world().insert_resource(std::move(solver));
}

} // namespace fcs::core::trajectory