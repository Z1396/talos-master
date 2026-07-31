#pragma once

/**
 * @file config.hpp
 * @brief 弹道轨迹配置结构体定义
 *
 * 本文件定义了弹道轨迹求解器的配置参数，包括空气阻力模型、弹丸初速、
 * 重力加速度等物理参数，以及求解器的数值积分参数。
 *
 * ## 弹道模型概述
 *
 * Talos采用精确的空气阻力弹道模型，而非简化的抛物线模型。
 * 这是因为比赛场景中弹丸速度高(15-30 m/s)、飞行距离远(0.5-8m)，
 * 空气阻力会产生显著影响（可达10-20%的下坠偏差）。
 *
 * ## 核心物理参数
 *
 * 1. **空气阻力系数**：取决于弹丸形状、质量、直径
 * 2. **弹丸初速**：由裁判系统设置，范围15-30 m/s
 * 3. **重力加速度**：标准值9.8 m/s²，可微调适应不同场地
 * 4. **大气密度**：影响阻力大小，通常取1.225 kg/m³
 *
 * ## 数值求解参数
 *
 * 1. **积分步长**：RK4方法的步长，影响精度和计算量
 * 2. **最大迭代次数**：防止无限循环的安全限制
 * 3. **收敛阈值**：判断求解是否达到精度的标准
 *
 * ## 配置来源
 *
 * - **弹速**：从MCU/裁判系统实时获取，存储在HardwareConfig中
 * - **物理参数**：在模型配置文件中定义（trajectory_model.toml）
 * - **求解参数**：在求解器配置文件中定义（solver.toml）
 *
 * @see model/config.hpp 弹道模型配置
 * @see trajectory_builder.cpp 弹道求解器实现
 */

#include <expected>    // C++23 错误处理类型
#include <string>      // 错误信息字符串

// 弹道模型配置：包含空气阻力系数、重力参数等
#include "core/trajectory/model/config.hpp"

// TOML配置辅助工具：required、flatten等包装器
#include "toml/type_wrappers.hpp"

// 前向声明TOML表类型，避免引入重量级toml++头文件
namespace toml::inline v3 {
class table;
}

namespace fcs::core::trajectory {

/**
 * @brief 弹道轨迹配置结构体
 *
 * 封装弹道求解器所需的所有配置参数，采用分层设计：
 * - model: 弹道物理模型参数（空气阻力、重力等）
 * - 其他参数可在未来扩展（如求解器参数）
 *
 * ## 配置示例
 *
 * ```toml
 * [trajectory.model]
 * air_resistance_coefficient = 0.01  # 空气阻力系数 (kg/m)
 * gravity = 9.8                       # 重力加速度 (m/s²)
 * air_density = 1.225                 # 大气密度 (kg/m³)
 * mass = 0.003                        # 弹丸质量 (kg)
 * ```
 *
 * ## 使用流程
 *
 * 1. **加载配置**：从TOML文件解析到TrajectoryConfig
 * 2. **传递给求解器**：创建TrajectoryBuilder时传入配置
 * 3. **运行时更新**：弹速从MCU实时获取，通过Resource机制传递
 *
 * ## 性能考虑
 *
 * - **配置是不可变的**：运行期间不应修改，保证线程安全
 * - **使用flatten包装**：允许配置字段散布在父表中，提高可读性
 * - **模型参数固定**：物理参数在比赛前标定，运行时不变
 *
 * @note 所有参数都有合理的默认值，即使配置文件缺失也能工作
 * @note 弹速参数通过HardwareConfig传入，不在此结构体中
 */
struct TrajectoryConfig {
    /**
     * @brief 弹道模型配置
     *
     * 包含空气阻力模型所需的所有物理参数：
     * - air_resistance_coefficient: 空气阻力系数 (kg/m)
     * - gravity: 重力加速度 (m/s²)
     * - air_density: 大气密度 (kg/m³)
     * - mass: 弹丸质量 (kg)
     *
     * 使用flatten包装器，允许字段直接出现在父表（trajectory）中，
     * 而不必强制嵌套在model子表中，提高配置文件可读性。
     *
     * @see model::ModelConfig 详细参数说明
     */
    toml_helper::flatten<model::ModelConfig> model{};
};

} // namespace fcs::core::trajectory