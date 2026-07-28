#pragma once
// 头文件保护宏，防止多次#include导致重复定义编译错误

#include "L3_estimation/energy_meter_solver/motion_model.hpp"
// 引入能量机关运动模型：大符BigRuneModel、小符SmallRuneModel及其参数结构体Params

namespace fcs::energy_meter {
// 嵌套命名空间
// fcs：顶层框架命名空间
// energy_meter：能量机关（大符、小符）L3状态估计模块专属命名空间

/**
 * @brief 能量机关L3层跟踪滤波完整配置
 * 用于RM能量机关（大符Rune、小符S rune）卡尔曼跟踪、目标匹配、状态机控制
 * 结构体参数由FCS反射框架自动从TOML配置文件加载
 */
struct EnergyMeterL3Config {
    /// 大能量机关（大符）运动滤波模型噪声参数
    ::energy_meter::BigRuneModel::Params big_model{};
    /// 小能量机关（小符）运动滤波模型噪声参数
    ::energy_meter::SmallRuneModel::Params small_model{};

    /// 投票机制重置时间(秒)
    /// 长时间目标丢失后，旋转角度投票缓存清空，防止历史旧数据干扰角度解算
    double reset_vote_time{2.0};

    /// 目标丢失帧数阈值
    /// 连续丢失该帧数，跟踪器判定目标彻底消失，退出TRACKING稳定跟踪状态
    int lost_thres{10};

    /// 跟踪建立阈值
    /// 连续成功匹配目标达到该帧数：DETECTING(检测中) → TRACKING(稳定跟踪)
    int tracking_thres{3};

    /// 马氏距离匹配门限
    /// 数据关联：检测轮廓与现有跟踪目标计算马氏距离，小于阈值才判定为同一个目标
    /// 能量机关轮廓数量多、干扰大，门限25比机器人装甲跟踪(10)更大
    double matcher_gate{25.0};

    /// 叶片解锁持续帧数
    /// 满足帧数条件后，允许切换下一叶片、更新旋转角度，抑制抖动频繁跳叶片
    int blade_unlock_frames{10};
};

} // namespace fcs::energy_meter