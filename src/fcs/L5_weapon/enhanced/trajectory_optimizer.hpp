/**
 * @file trajectory_optimizer.hpp
 * @brief L5武器层基于MPC的轨迹优化器
 *
 * 本文件实现了轨迹优化器,用于将L4层生成的参考轨迹转换为平滑的控制指令。
 * 核心思想:
 * 1. 接收L4的参考轨迹(可能包含运动目标预测)
 * 2. 使用DualSmallMpcSolver求解最优控制序列
 * 3. 提取中心时刻的控制指令(速度+加速度前馈)
 * 4. 在L5层进行开火门判断
 *
 * 优化目标:
 * - 跟踪L4参考轨迹(位置+速度)
 * - 平滑控制输入(限制加速度)
 * - 满足机械约束(云台角度限制)
 *
 * 工作流程:
 * 1. Track模式:MPC优化 → 提取中心指令 → 开火门判断
 * 2. Shot模式:直接透传目标 → 开火门判断
 * 3. Hold模式:不发送指令(保持当前姿态)
 *
 * @note 性能关键路径,使用DualSmallMpcSolver而非DualMpcOsqpSolver
 */

#pragma once

#include "L4_planning/config.hpp"
#include "L4_planning/control_intent.hpp"
#include "L5_weapon/config.hpp"
#include "L5_weapon/fire_control.hpp"
#include "dual_small_mpc_solver.hpp"
#include <cstdint>
#include <expected>
#include <string>

namespace fcs::L5 {

/**
 * @brief 基于TinyMPC的轨迹优化器(L5武器层)
 *
 * 消费L4层预构建的参考轨迹,通过DualSmallMpcSolver独立求解yaw/pitch MPC问题,
 * 并输出中心时刻的武器指令(带L5开火门)。
 *
 * 设计模式:
 * - 使用静态工厂方法初始化(create)
 * - 不可拷贝(包含求解器状态)
 * - 可移动(资源转移成本低)
 *
 * 性能优化:
 * - 使用float精度(ARM优化)
 * - 固定容量数组(零分配)
 * - 提前初始化求解器(避免运行时构造)
 */
class TinyMpcTrajectoryOptimizer {
public:
    /**
     * @brief 构造函数(初始化求解器)
     *
     * @param config 武器控制器配置(MPC参数、阈值等)
     * @param trajectory_cfg L4轨迹配置(预测步数、时间步长等)
     */
    explicit TinyMpcTrajectoryOptimizer(
        const WeaponControllerConfig& config,
        const L4::ReferenceTrajectoryConfig& trajectory_cfg) noexcept;

    ~TinyMpcTrajectoryOptimizer() noexcept = default;

    /// 禁止拷贝(包含求解器状态)
    TinyMpcTrajectoryOptimizer(const TinyMpcTrajectoryOptimizer&)            = delete;
    TinyMpcTrajectoryOptimizer& operator=(const TinyMpcTrajectoryOptimizer&) = delete;

    /// 禁止移动(单例模式,不应移动)
    TinyMpcTrajectoryOptimizer(TinyMpcTrajectoryOptimizer&&)                 = delete;
    TinyMpcTrajectoryOptimizer& operator=(TinyMpcTrajectoryOptimizer&&)      = delete;

    /// 检查MPC优化是否启用
    [[nodiscard]] bool enabled() const noexcept { return config_.enabled; }

    /**
     * @brief 透传模式:构建Shot指令(无MPC优化)
     *
     * 用于Shot意图,直接将目标位置传递给云台。
     * 不进行轨迹优化,速度和加速度设为0。
     *
     * @param shot L4层的Shot指令
     * @param command_timestamp_ns 当前时间戳(纳秒)
     *
     * @return 透传后的武器指令
     */
    [[nodiscard]] WeaponCommand
        passthrough(const L4::ShotCommand& shot, uint64_t command_timestamp_ns) const noexcept;

    /**
     * @brief 优化模式:对Track意图的参考轨迹执行MPC优化
     *
     * 核心算法流程:
     * 1. 检查参考轨迹新鲜度(拒绝过时数据)
     * 2. 设置求解器初始状态和参考轨迹
     * 3. 执行MPC求解
     * 4. 提取中心时刻的状态和输入
     * 5. 构建武器指令(位置+速度+加速度)
     *
     * @param track L4层的Track指令(包含参考轨迹)
     * @param command_timestamp_ns 当前时间戳
     *
     * @return 成功返回优化后的武器指令,失败返回错误信息
     */
    [[nodiscard]] std::expected<WeaponCommand, std::string>
        optimize(const L4::TrackCommand& track, uint64_t command_timestamp_ns) noexcept;

private:
    WeaponControllerConfig config_;           ///< 武器控制器配置
    L4::ReferenceTrajectoryConfig trajectory_cfg_; ///< L4轨迹配置
    DualSmallMpcSolver batched_solver_;       ///< 双轴MPC求解器(已初始化)
    bool ready_{false};                       ///< 求解器就绪标志

    /**
     * @brief 初始化MPC求解器
     *
     * 根据配置参数构造DualSmallMpcSolver实例。
     * 仅在构造函数中调用一次。
     */
    void initialize_solvers() noexcept;
};

} // namespace fcs::L5
