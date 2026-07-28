#pragma once
// 头文件保护，防止多重包含引发重复定义编译错误

#include "new_motion_model.hpp"   // 运动模型参数结构体定义（RobotEkfMotionModel、OutpostEkfMotionModel）
#include <memory>                 // std::shared_ptr 智能指针头文件

namespace fcs::L3 {
// 嵌套命名空间
// fcs：框架顶层命名空间
// L3：L3层级估计模块（视觉目标滤波、跟踪、EKF状态估计模块）

// ============================================================================
// Tracker Configuration 跟踪器基础配置模板
// ============================================================================
/**
 * @brief 目标跟踪通用配置模板
 * @tparam T 运动模型参数类型（机器人EKF参数 / 前哨站EKF参数二选一）
 * 模板作用：复用同一套跟踪逻辑，区分【英雄/步兵机器人】和【前哨站】两套目标
 */
template <typename T>
struct TargetConfig {
    /// 临时丢失状态(TEMP_LOST)超时时间，单位：秒
    /// 超过该时长没有匹配目标，跟踪器切换回 IDLE 空闲状态
    double lost_threshold{};

    /// 连续成功匹配帧数阈值
    /// 连续检测到目标达到该帧数：状态 DETECTING（检测中）→ TRACKING（稳定跟踪）
    uint32_t tracking_threshold{};

    /// 马氏距离阈值门限
    /// 数据关联匹配使用：只有马氏距离小于该值，才认为检测框和现有跟踪目标是同一个物体
    double matcher_gate{10.0};

    /// 运动模型对应的滤波参数（卡尔曼/EKF噪声、初始协方差等）
    T model{};
};

/**
 * @brief 机器人InEKF几何参数
 * 描述装甲板相对于机器人中心的几何偏移模型
 */
struct RobotInEKFConfig {
    /// 0、2号装甲板距离机器人中心半径（m）
    double radius0{0.23};
    /// 1、3号装甲板距离机器人中心半径（m）
    double radius1{0.23};
    /// 1、3装甲板相对0、2装甲板的Z轴高度偏移（m）
    double height{0.0};
};

/**
 * @brief 跟踪器总配置
 * 区分两种目标：敌方机器人、前哨站
 */
struct TrackerConfig {
    /// 敌方步兵/英雄机器人跟踪配置
    TargetConfig<RobotEkfMotionModel::Params> robot;
    /// 前哨站跟踪配置
    TargetConfig<OutpostEkfMotionModel::Params> outpost;

    /// 机器人InEKF几何模型参数
    RobotInEKFConfig robot_inekf;
};

// ============================================================================
// L3 Estimation Full Config L3估计模块总配置
// ============================================================================
/**
 * @brief L3视觉状态估计模块顶层配置
 */
struct L3Config {
    /// 跟踪器全部参数
    TrackerConfig tracker{};

    /**
     * @brief 获取tracker配置的只读共享智能指针
     * [[nodiscard]] 警告：返回值不要忽略
     * @return std::shared_ptr<const TrackerConfig> 只读配置拷贝
     *
     * 工程用途：
     * 多线程场景分发配置快照。
     * 配置热更新时，生成全新一份TrackerConfig拷贝，
     * 各个视觉线程持有shared_ptr，旧配置自动在引用计数归零时释放。
     * const保证线程内不能修改配置，避免并发读写冲突。
     */
    [[nodiscard]] std::shared_ptr<const TrackerConfig> tracker_ptr() const {
        // 拷贝当前tracker对象，封装成只读shared_ptr返回
        return std::make_shared<const TrackerConfig>(tracker);
    }
};

} // namespace fcs::L3