#pragma once
// 头文件保护宏，防止多次包含引发重复定义

#include "L5_weapon/fire_decision.hpp"
// 引入开火决策配置结构体 FireDecisionConfig

namespace fcs::L5 {
/**
 * L5：武器执行层
 * 上层：L4规划层下发参考轨迹；下层：云台电机控制、发射机构
 * 核心职责：MPC模型预测控制求解云台运动、物理限位约束、轨迹有效性校验、开火闸门控制
 */

/**
 * @brief MPC单轴代价与约束配置（yaw轴 / pitch轴通用）
 * MPC代价函数形式：
 * cost = q_pos * (pos_ref - pos)^2 + q_vel * vel^2 + r * control^2
 */
struct MpcAxisConfig {
    /// 位置跟踪权重：越大，越强制跟随L4下发的参考角度
    double q_pos{9e6};
    /// 速度惩罚权重：越大，抑制轴产生过高速度
    double q_vel{0.0};
    /// 控制输入惩罚权重：越大，输出控制量越平缓，抑制剧烈力矩输出
    double r{1.0};
    /// 轴最大角加速度限制 (rad/s²)，硬约束
    double max_acc{50.0};
};

/**
 * @brief MPC求解器全局参数 + Yaw/Pitch双轴独立配置
 */
struct MpcSolverConfig {
    /// 单次优化最大迭代次数，限制求解耗时，保证实时性
    int max_iterations{10};
    /// 收敛判定绝对误差阈值；优化误差小于该值即提前停止迭代
    double abs_tol{1e-3};
    /// ADMM/优化算法的步长参数rho，影响求解收敛速度
    double rho{1.0};

    /// 云台偏航轴（水平旋转）参数
    MpcAxisConfig yaw{9e6, 0.0, 1.0, 50.0};
    /// 云台俯仰轴（上下）参数；俯仰允许更大加速度 100rad/s²
    MpcAxisConfig pitch{9e6, 0.0, 1.0, 100.0};
};

/**
 * @brief 增强型武器控制器配置（L5核心）
 * 接收L4下发参考轨迹，运行MPC求解，输出受物理约束的云台指令
 */
struct WeaponControllerConfig {
    /// 武器控制器总开关
    bool enabled{true};
    /// 开启调试信息输出（日志/可视化）
    bool enable_debug{true};

    /// 俯仰角度下限(rad)，约 -40°
    double pitch_min{-0.69813169};
    /// 俯仰角度上限(rad)，约 +40°
    double pitch_max{0.69813169};

    /**
     * L4参考轨迹最大允许时效（秒）
     * 如果L4下发的参考轨迹时间戳超过该阈值，判定轨迹陈旧失效
     * L5不再执行MPC优化，切换为直通模式(passthrough)，避免使用过期预测轨迹导致打偏
     */
    double reference_age_threshold_s{0.02};

    /// MPC求解全套参数
    MpcSolverConfig mpc{};
};

/**
 * @brief L5武器层完整总配置
 */
struct L5Config {
    /// 开火决策配置：L4瞄准逻辑、L5开火闸门共用
    FireDecisionConfig fire_decision{};
    /// 基于MPC的云台武器控制器参数
    WeaponControllerConfig mpc_weapon{};
};

} // namespace fcs::L5