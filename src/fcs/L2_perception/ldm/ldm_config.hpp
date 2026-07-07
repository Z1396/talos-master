#pragma once
// 装甲颜色枚举 ArmorColor 定义
#include "core/armor_types.hpp"

// std::array 存储二元Z轴范围
#include <array>

namespace fcs::L2::ldm {

/**
 * @brief LDM大符几何尺寸配置结构体
 * 存储大符八边形、灯条、灯条对、立体高度等物理三维尺寸（单位：米）
 * 全部参数为工厂标定固定物理尺寸，用于PnP三维重建
 */
struct LdmGeometryConfig {
    /// 八边形单条边长 (m)
    double octagon_side_length_m{0.020711};
    /// 八边形外接圆半径 (m)，八边形顶点到中心距离
    double octagon_circumradius_m{0.02706};
    /// 左右灯条中心水平间距 (m)
    double pair_center_separation_m{0.036514};
    /// 灯条长度 (m)
    double window_length_m{0.012};
    /// 灯条宽度 (m)
    double window_width_m{0.009618};
    /// 大符整体立体高度 (m)
    double volume_height_m{0.067};
    /// 可检测灯条对中心Z轴坐标范围 [最小值, 最大值] (m)
    std::array<double, 2> detectable_center_z_range_m{-0.01, 0.01};
};

/**
 * @brief LDM大符检测器完整阈值配置
 * 包含灯条色块筛选、灯条配对、多对校验、候选打分、PnP约束角度、几何尺寸子配置
 * 所有阈值用于图像像素级筛选与匹配过滤，控制误检/漏检平衡
 */
struct LdmDetectorConfig {
    /// 目标装甲颜色：红/蓝
    ArmorColor target_color{ArmorColor::Red};

    // -------------------------- 单灯条色块Blob筛选阈值 --------------------------
    /// 灯条最小像素面积，小于该值直接丢弃小噪点色块
    int min_blob_area_px{5};
    /// 稀疏灯条色块最少有效像素数量，过滤断裂残缺灯条
    int min_sparse_blob_pixel_count{8};
    /// 灯条填充率下限（轮廓面积/外接矩形面积），过滤空心零散噪点
    double min_blob_fill_ratio{0.35};
    /// 灯条最小长宽比，过滤过扁方块噪点
    double min_blob_aspect_ratio{0.35};
    /// 灯条最大长宽比，过滤超长条状杂线
    double max_blob_aspect_ratio{4.0};

    // -------------------------- 灯条对配对筛选阈值 --------------------------
    /// 配对两灯条中心竖直高度差比例下限
    double min_pair_center_dy_ratio{1.0};
    /// 配对两灯条中心竖直高度差比例上限，高度差过大会被判定非同一组灯条
    double max_pair_center_dy_ratio{12.0};
    /// 配对两灯条尺寸最大差异比例，尺寸差距过大剔除配对
    double max_pair_size_delta_ratio{0.75};
    /// 灯条之间灰度间隙最大变异系数，间隙明暗突变过大剔除配对
    double max_gap_cv{0.65};

    // -------------------------- 单组/两组灯条候选打分阈值 --------------------------
    /// 单组灯条候选最低综合匹配分数，低于此丢弃
    double min_preliminary_candidate_score{0.80};
    /// 两组灯条（完整大符）最低综合匹配分数，要求更高
    double min_preliminary_candidate_score_two_pair{0.87};
    /// 两组灯条中心竖直像素差最小值，高度过于贴近判定残缺
    double min_two_pair_mean_center_dy_px{12.0};
    /// 孤立双灯条前后跨度最小比例，过滤局部残缺双条
    double min_isolated_two_pair_order_span_ratio{0.25};
    /// 相邻灯条组序号间隙最大比例，剔除无序杂乱灯条组合
    double max_adjacent_face_order_gap_ratio{1.20};
    /// 解析后灯条长度允许最大缩放倍数，防止透视畸变过度拉伸
    double max_resolved_window_length_fraction{1.45};
    /// 合并后灯条对像素最大分离距离，超出则判定为两组独立灯条
    double max_merged_window_pair_separation_px{45.0};
    /// 构成有效大符最少灯条配对数量（至少2组灯条对）
    int min_pairs_for_detection{2};

    // -------------------------- PnP优化约束阈值 --------------------------
    /// 稳定姿态重投影RMSE像素阈值，低于判定位姿可靠
    double rmse_stable_threshold_px{8.0};
    /// 约束BA优化重投影误差上限，超过放弃该位姿解
    double rmse_constrained_threshold_px{8.0};
    /// 允许最大姿态俯仰/偏航角（弧度，约50°），超出判定透视畸变过大无效
    double max_pose_angle_rad{0.872664626};

    /// 大符物理几何尺寸子配置
    LdmGeometryConfig geometry{};
};

} // namespace fcs::L2::ldm