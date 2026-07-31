/**
 * @file types.hpp
 * @brief LDM状态类型定义 - SE2(3)群元素封装
 *
 * ## 文件功能概述
 * 定义LDM跟踪器的输出状态结构体LdmState，封装SE2(3)群元素及其元数据。
 * 核心状态以群元素形式存储，避免状态分解，仅在可视化边界提供访问器。
 *
 * ## 核心算法原理
 *
 * ### 1. SE2(3)群（扩展特殊欧氏群）
 * 状态表示为群元素X = (R, v_body, p)：
 * - R ∈ SO(3)：世界坐标系到物体坐标系的旋转矩阵
 * - v_body ∈ R³：物体坐标系下的速度矢量
 * - p ∈ R³：世界坐标系下的位置矢量
 *
 * ### 2. 群上状态传播
 * 通过李代数右扰动进行预测：
 *   ξ = (dθ=0, dv=0, dp=v_body·dt) ∈ se2(3)
 *   X̂₊ = X̂ * exp(ξ)
 *
 * 这种表示保持群结构性质，避免欧氏空间线性化带来的奇异性问题。
 *
 * ### 3. 设计理念：Parse, Don't Validate
 * - 核心状态始终以SE2(3)群元素形式存在（不分解为独立字段）
 * - 可视化访问器仅在边界提供（Foxglove等不需要了解群论）
 * - 强类型保证：无法构造非法的SE2(3)状态
 *
 * ## 关键数据结构
 * - LdmState：跟踪器输出结构体（时间戳、状态、SE2(3)元素、预测位置）
 *
 * ## 潜在风险提示
 * - 可视化访问器隐藏了群结构，用户需理解这只是边界接口
 * - 预测位置通过独立的李代数计算，需与滤波器预测保持一致
 *
 * ## 优化建议
 * - 可添加协方差矩阵输出（用于下游风险评估）
 * - 可提供SE2(3)的对数映射输出（用于调试和日志）
 *
 * @author Talos Team
 * @date 2024
 */

#pragma once

#include "L3_estimation/ldm_naive/ldm_kinematic_model.hpp"
#include "L3_estimation/tracker/types.hpp"

#include <Eigen/Core>

#include <cstdint>

namespace fcs::L3::ldm {

/**
 * @brief LDM跟踪器状态 - SE2(3)群元素 + 元数据
 *
 * 核心状态以完整SE2(3)群元素形式存储，**不分解**为独立的Vector3d字段。
 * 平铺的位置/旋转/速度访问器仅存在于**可视化边界**，不是内部表示。
 *
 * ## SE2(3)结构（Nominal）
 * X = (R, v_body, p)
 * - R：SO(3)旋转矩阵（world → body）
 * - v_body：物体系速度矢量（R³）
 * - p：世界系位置矢量（R³）
 *
 * 预测通过李代数右扰动：
 *   ξ = (dθ=0, dv=0, dp=v_body·dt)
 *   X̂₊ = X̂ * exp(ξ)
 *
 * ## 设计原则
 * - 核心域：状态始终为SE2(3)群元素（类型安全）
 * - 边界接口：提供平铺访问器供可视化使用（不暴露群论细节）
 * - 不可变访问：所有访问器为const方法
 *
 * ## 数据流
 * - 输入：LdmTracker::get_output()
 * - 输出：L4规划层（Aimer）、可视化系统（Foxglove）
 */
struct LdmState {
    using Nominal    = group::SEn3<double, 2>;  ///< SE2(3)群类型
    using SO3Type    = Nominal::SO3Type;        ///< SO(3)类型
    using VectorType = Eigen::Vector3d;         ///< 向量类型

    // ========================================================================
    // 时间戳和状态标记
    // ========================================================================

    uint64_t timestamp_ns{0};                  ///< 当前状态时间戳（纳秒）
    uint64_t last_observation_timestamp_ns{0};  ///< 最后观测时间戳（纳秒）
    TrackerStatus status{TrackerStatus::Idle};  ///< 跟踪状态（Idle/Detecting/Tracking/TempLost）
    bool accurate{false};                       ///< 精度标志（来自感知层）

    // ========================================================================
    // SE2(3)群元素 - **核心内部状态，不分解**
    // ========================================================================

    Nominal X{};  ///< SE2(3)状态（默认为单位元素：R=I3, v=0, p=0）

    // ========================================================================
    // 预计算预测（供L4规划层便利使用）
    // ========================================================================

    Eigen::Vector3d predicted_position_odom{Eigen::Vector3d::Zero()};  ///< 预测的世界系位置
    uint64_t predicted_future_ns{0};                                    ///< 预测时刻的时间戳

    // ========================================================================
    // 可视化边界访问器
    //
    // 这些访问器仅用于让Foxglove/可视化代码读取平铺的Vector3d/Matrix3d，
    // 无需引入完整的SE2(3)机制。它们**不是**系统内部表示。
    // ========================================================================

    /**
     * @brief 获取世界系位置（可视化边界）
     * @return 位置向量的常引用
     *
     * @note 这是SE2(3)元素的p分量，不是独立存储的字段
     */
    [[nodiscard]] const Eigen::Vector3d& position() const noexcept { return X.p(); }

    /**
     * @brief 获取世界→物体旋转矩阵（可视化边界）
     * @return 旋转矩阵的常引用
     *
     * @note 这是SE2(3)元素的R分量，不是独立存储的字段
     */
    [[nodiscard]] const Eigen::Matrix3d& rotation() const noexcept { return X.R(); }

    /**
     * @brief 获取物体系速度（可视化边界）
     * @return 速度向量的常引用
     *
     * @note 这是SE2(3)元素的v分量，不是独立存储的字段
     */
    [[nodiscard]] const Eigen::Vector3d& velocity_body() const noexcept { return X.v(); }

    /**
     * @brief 获取世界系速度（访问时计算 - 不存储）
     * @return 世界系速度向量
     *
     * @note v_world = R * v_body，通过群元素计算
     */
    [[nodiscard]] Eigen::Vector3d velocity_world() const noexcept { return X.R() * X.v(); }

    // ========================================================================
    // 查询辅助函数
    // ========================================================================

    /**
     * @brief 检查是否正在跟踪
     * @return true if 状态为Tracking或TempLost
     */
    [[nodiscard]] bool is_tracking() const noexcept {
        return status == TrackerStatus::Tracking || status == TrackerStatus::TempLost;
    }
};

} // namespace fcs::L3::ldm
