// 头文件保护宏：防止该头文件被同一源文件多次重复包含，替代传统 #ifndef / #define 方案
#pragma once

// 引入标准固定宽度整数类型：uint32_t 无符号32位整型依赖此头文件
#include <cstdint>

// 前置声明 toml 库 v3 版本的 table 类，仅告诉编译器存在这个类，无需引入完整 toml 头文件，减少编译依赖
namespace toml::inline v3 {
class table;
}

// 命名空间层级：fcs 项目 -> core 核心模块 -> trajectory 弹道模块 -> model 弹道模型子模块
namespace fcs::core::trajectory::model {

/// 弹道物理模型枚举，区分两种不同弹道计算算法
enum class ModelType {
    Ideal,        // 理想弹道模型：忽略空气阻力，仅重力下落，计算速度快、精度低
    LinearDrag,   // 线性阻力弹道模型：考虑线性空气阻力，兼顾精度与计算开销（项目默认）
};

/// 弹道求解全局配置结构体
/// 存储各类弹道算法共用/专属物理参数，初始化时赋予合理工程默认值
struct ModelConfig {
    // 选择使用的弹道模型，默认：线性阻力模型
    ModelType type{ModelType::LinearDrag};

    /// 俯仰补偿迭代次数
    /// 弹道求解是迭代逼近方程，数值越大补偿精度越高，但CPU耗时线性上升；默认20次平衡性能与精度
    uint32_t iteration_times{20};

    /// 重力加速度，单位 m/s²，地球标准重力常量
    double gravity{9.8};

    /// 线性空气阻力系数
    /// 仅 LinearDrag 线性阻力模型 使用；表征弹丸受空气拖拽的线性损耗系数
    double resistance{0.001};

    /// 空气密度 ρ，单位 kg/m³
    /// 仅高精度RK4数值积分模型使用；标准常温常压空气密度默认值
    double rho{1.225};

    /// 弹丸直径，单位 米
    /// 仅RK4高精度模型使用，用于计算迎风截面积，参与空气阻力计算
    double diameter{0.017};

    /// 弹丸质量，单位 千克
    /// 仅RK4高精度模型使用，质量越大惯性越强，空气对弹道偏移影响越小
    double mass{0.0032};
};

} // namespace fcs::core::trajectory::model