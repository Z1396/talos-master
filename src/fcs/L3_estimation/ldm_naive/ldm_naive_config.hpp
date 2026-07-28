#pragma once
// 头文件保护，防止头文件被多次包含引发重复定义编译错误

#include "ldm_kinematic_params.hpp"
// 引入 LDM（能量机关/大符）运动学模型参数结构体 LdmKinematicParams

#include <cstdint>
// 标准固定宽度整数头文件，提供 uint32_t

namespace fcs::L3::ldm {
// 命名空间层级：
// fcs 框架顶层
// L3 状态估计层
// ldm 大能量机关(Large Damage Mechanism / Big Rune)朴素跟踪模块

/**
 * @brief NaiveLdm 简易大符卡尔曼跟踪器配置
 * 属于L3层状态估计配置，存放跟踪状态机、滤波器初始协方差、运动模型参数
 * 结构体支持FCS反射解析，可直接从TOML加载配置，成员自带默认值
 */
struct NaiveLdmConfig {
    /// 跟踪建立阈值（连续帧数）
    /// 连续成功匹配目标达到该帧数：DETECTING(检测中) → TRACKING(稳定跟踪)
    uint32_t tracking_threshold{5};

    /// 目标丢失超时阈值，单位：秒
    /// 持续丢失超过该时间，跟踪器清空目标，回到IDLE空闲状态
    double lost_threshold{1.0};

    /// 滤波器初始化：旋转量初始标准差
    /// 第一次创建跟踪目标时，角度/旋转状态初始协方差
    double initial_sigma_rot{0.05};

    /// 滤波器初始化：机体坐标系速度初始标准差
    /// 新建目标时，速度状态初始不确定度
    double initial_sigma_velocity_body{5.0};

    /// 滤波器初始化：位置初始标准差(m)
    /// 新建跟踪目标时，世界坐标位置初始不确定程度
    double initial_sigma_position{0.20};

    /// LDM运动学模型噪声参数（过程噪声、观测噪声等滤波参数）
    LdmKinematicParams model{};
};

} // namespace fcs::L3::ldm