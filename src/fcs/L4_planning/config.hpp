#pragma once
// 头文件保护，防止多次包含造成重复定义

#include "L4_planning/aimer/armor_target_decider.hpp"
#include "core/armor_types.hpp"          // 装甲枚举 ArmorName、ArmorTargetDeciderKind
#include "toml/ext/containers.hpp"       // TOML反射扩展，支持结构体自动解析

namespace fcs::L4 {
// L4：规划层（Planning），负责目标选择、自瞄有限状态机、弹道参考轨迹生成
// 上层：视觉L3状态估计；下层：L5武器控制器、弹道求解器

/**
 * @brief 目标选择各项代价权重
 * 多装甲候选目标打分模型：总分 = Σ(权重 × 归一化代价)
 * 分值越高越优先选中；权重越大，该维度对选择结果影响越强
 */
struct TargetSelectionWeights {
    double image_center{5.0};    // 图像中心权重：装甲越靠近画面中心越优先
    double track_state{3.0};     // 跟踪状态权重：稳定跟踪目标优先
    double tof{1.5};             // 飞行时间权重：子弹飞行时间越短优先（近距离目标）
    double gimbal_effort{1.0};   // 云台转动代价：云台需要转动幅度越小优先
    double armor_name{0.5};      // 装甲类型优先级权重（英雄/哨兵/基地等）
};

/**
 * @brief 代价归一化参考基准值
 * 所有代价需要除以参考值做归一化，把不同量纲的数据统一映射到相近区间
 */
struct TargetSelectionRefs {
    double image_center_ref_px{300.0};    // 图像中心距离参考像素
    double tof_ref_s{0.35};               // 子弹飞行时间参考值(s)
    double yaw_effort_ref_deg{20.0};      // 云台yaw转动角度参考(°)
    double pitch_effort_ref_deg{10.0};    // 云台pitch转动角度参考(°)
};

/**
 * @brief 不同装甲类型基础分值
 * 用于区分优先级：比如哨兵、基地装甲可以设置更高基础分，优先打击
 */
struct TargetSelectionArmorNameScore {
    double sentry{0.5};       // 哨兵装甲
    double one{0.5};          // 一号装甲
    double two{0.5};          // 二号装甲
    double three{0.5};        // 三号装甲
    double four{0.5};         // 四号装甲
    double five{0.5};         // 五号装甲
    double outpost{0.5};      // 前哨站装甲
    double base{0.5};         // 基地普通装甲
    double base_large{0.5};   // 基地大装甲
    double invalid{0.0};      // 无效装甲，最低分，绝不优先选择

    /**
     * @brief 根据装甲类型获取基础分数
     * @param name 装甲类型枚举
     * @return 对应分值
     */
    [[nodiscard]] double score(ArmorName name) const noexcept {
        switch (name) {
        case ArmorName::Sentry: return sentry;
        case ArmorName::One: return one;
        case ArmorName::Two: return two;
        case ArmorName::Three: return three;
        case ArmorName::Four: return four;
        case ArmorName::Five: return five;
        case ArmorName::Outpost: return outpost;
        case ArmorName::Base: return base;
        case ArmorName::BaseLarge: return base_large;
        case ArmorName::Invalid: return invalid;
        }
        return invalid;
    }
};

/**
 * @brief 目标选择器完整配置
 */
struct TargetSelectionConfig {
    // 目标决策器类型：无人/有人决策逻辑
    ArmorTargetDeciderKind decider{ArmorTargetDeciderKind::Unmanned};

    // 临时丢失目标的分数衰减系数
    double temp_lost_state_score{0.05};
    // 视觉观测陈旧超时：超过该时间观测失效，不再采信
    double optical_stale_timeout_s{0.25};

    /// 目标切换滞环阈值
    /// 新目标分数 - 当前目标分数 > switch_margin，才允许切换目标
    /// 防止分数小幅震荡导致云台频繁来回切换目标（防抖）
    double switch_margin{0.08};

    TargetSelectionWeights weights{};
    TargetSelectionRefs refs{};
    TargetSelectionArmorNameScore armor_name_score{};
};

/**
 * @brief 自瞄核心配置（Aimer，机器人/前哨/能量机关共用）
 */
struct AimerConfig {
    /// 系统总延迟补偿(秒)
    /// 包含相机曝光、图像处理、通信延迟、云台机械响应延迟
    double delay{0.01};

    // 四状态有限状态机阈值（单人、整车、成对装甲逻辑）
    // up：进入状态阈值；down：退出状态阈值（滞环，防止抖动）
    double single_whole_up{1.5};    // 进入【单人模式】阈值
    double single_whole_down{1.0};  // 退出【单人模式】阈值
    double whole_pair_up{6.5};      // 进入【整车模式】阈值
    double whole_pair_down{7.5};    // 退出【整车模式】阈值
    double pair_center_up{16.5};    // 进入【成对中心模式】阈值
    double pair_center_down{15.0};  // 退出【成对中心模式】阈值
    int transfer_thresh{50};        // 状态切换稳定帧数阈值

    /// 单装甲锁定前视窗口(角度)
    /// 只有装甲落在云台当前朝向±该角度内，才允许锁定
    double front_window_deg{60.0};

    // 来袭目标最小速度阈值（用于预判敌方冲锋）
    double min_coming_vel{1e-2};
    double min_coming_vel_horizon{0.5};

    // 加权多目标选择全套参数
    TargetSelectionConfig target_selection{};
};

/**
 * @brief 参考轨迹生成参数
 * L4规划层生成预测轨迹，提供给L5武器弹道求解器作为初值
 * 仅控制轨迹预测区间，物理限位（云台最大角度、发射频率）放在L5武器控制器
 */
struct ReferenceTrajectoryConfig {
    int horizon_ahead{50};  // 向前预测步数（未来轨迹）
    int horizon_back{50};   // 向后回溯步数（历史轨迹平滑）
    double dt{0.01};        // 轨迹离散时间步长 10ms
};

/**
 * @brief L4规划层总配置
 */
struct L4Config {
    // 自瞄公共配置：机器人、前哨、能量机关共用一套aimer逻辑
    AimerConfig aimer{};

    // 参考轨迹生成参数，L4生成，L5弹道求解器使用
    ReferenceTrajectoryConfig reference_trajectory{};
};

} // namespace fcs::L4_planning