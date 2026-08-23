#pragma once
// 头文件保护，防止同一个头文件被多次 include，只编译一次

#include <algorithm>
#include <cmath>
#include <cstdint>

// Foxglove SDK 的消息结构体定义，Color 就在这里
#include <foxglove/schemas.hpp>
// OpenCV 基础类型，cv::Scalar 用于图像BGR颜色
#include <opencv2/core/types.hpp>

/**
 * @file tactical.hpp
 * @brief RoboMaster 战术可视化配色与尺寸常量头文件
 *
 * 整套 FCS 框架可视化配置：
 * 1. 定义 Foxglove 3D可视化用的 RGBA 颜色（0~1浮点数，不是0‑255）
 * 2. 定义图像叠加(OpenCV)、3D Marker绘制的线宽、字体大小、几何体尺寸
 * 3. 分层组织：L1图像层 → L2感知 → L3状态估计 → L4规划瞄准 → L5武器射击
 * 4. 提供工具函数：修改透明度、缩放RGB、foxglove颜色转OpenCV BGR
 *
 * Foxglove::schemas::Color 结构体：
 * struct Color { float r,g,b,a; }; 全部取值范围 [0.0 ~ 1.0]
 */
namespace fcs::visualization::tactical {

// -------------------------- 基础语义颜色库 --------------------------
namespace Semantic {

/**
 * @brief 基础语义配色，整套可视化的颜色源头
 * 设计思路：大部分场景使用低饱和度中性灰；高饱和颜色留给关键状态、决策、告警
 */
constexpr ::foxglove::schemas::Color STATUS_ACTIVE     = {0.00, 0.44, 0.90, 1.00}; // 激活状态：主蓝色
constexpr ::foxglove::schemas::Color DECISION_POSITIVE = {0.20, 0.70, 0.32, 1.00}; // 正向决策：绿色（允许开火、选中目标）
constexpr ::foxglove::schemas::Color DECISION_NEGATIVE = {0.92, 0.22, 0.20, 1.00}; // 负向决策：红色（丢失、拒绝、异常）
constexpr ::foxglove::schemas::Color ALERT             = {0.95, 0.58, 0.10, 1.00}; // 告警：橙黄色
constexpr ::foxglove::schemas::Color INERT             = {0.58, 0.62, 0.68, 1.00}; // 惰性/次要信息：浅灰，不抢视觉焦点

constexpr ::foxglove::schemas::Color CONTEXT = {0.36, 0.39, 0.44, 1.00}; // 上下文背景：深灰
constexpr ::foxglove::schemas::Color SURFACE = {0.12, 0.13, 0.15, 1.00}; // 表面底色：近乎黑色

} // namespace Semantic

// -------------------------- 机器人阵营颜色 --------------------------
namespace Team {
constexpr ::foxglove::schemas::Color RED  = {0.86, 0.20, 0.18, 0.70};  // 红方，带0.7透明度
constexpr ::foxglove::schemas::Color BLUE = {0.10, 0.38, 0.86, 0.70};  // 蓝方，带0.7透明度
} // namespace Team

// -------------------------- 坐标轴配色（XYZ轴） --------------------------
namespace Axis {
constexpr ::foxglove::schemas::Color X = {0.84, 0.24, 0.22, 0.72}; // X轴：红色
constexpr ::foxglove::schemas::Color Y = {0.22, 0.64, 0.32, 0.72}; // Y轴：绿色
constexpr ::foxglove::schemas::Color Z = {0.10, 0.38, 0.86, 0.72}; // Z轴：蓝色
} // namespace Axis

/**
 * @brief 修改颜色的alpha透明度，返回新颜色
 * @param color 输入foxglove颜色
 * @param alpha 新透明度 [0‑1]
 * @return 修改alpha后的color对象
 * @note [[nodiscard]] 警告：不要忽略返回值，此函数不会修改入参（值拷贝）
 * @note noexcept 保证函数不会抛出异常；constexpr 编译期可计算
 */
[[nodiscard]] constexpr ::foxglove::schemas::Color
    with_alpha(::foxglove::schemas::Color color, double alpha) noexcept {
    color.a = static_cast<float>(alpha);
    return color;
}

/**
 * @brief RGB整体亮度缩放，用于调暗/调亮颜色，alpha保持不变
 * @param scale 缩放系数 0~1变暗，>1变亮
 */
[[nodiscard]] constexpr ::foxglove::schemas::Color
    scaled_rgb(::foxglove::schemas::Color color, double scale) noexcept {
    color.r = static_cast<float>(color.r * scale);
    color.g = static_cast<float>(color.g * scale);
    color.b = static_cast<float>(color.b * scale);
    return color;
}

/**
 * @brief Foxglove RGBA(0‑1) → OpenCV cv::Scalar BGR(A)（0‑255）
 * @param color foxglove的Color结构体 r/g/b/a ∈ [0,1]
 * @return cv::Scalar(B,G,R,A)，供cv::rectangle、cv::putText绘图直接使用
 *
 * OpenCV图像通道顺序是B‑G‑R，和常规RGB相反，这里做了通道调换。
 * std::clamp 防止数值溢出（大于1或者负数），再乘以255转为0‑255像素值。
 */
[[nodiscard]] inline cv::Scalar to_cv_bgr(const ::foxglove::schemas::Color& color) noexcept {
    const auto channel = [](double value) { return std::clamp(value, 0.0, 1.0) * 255.0; };
    /*// 旧写法，IDE给划删除线
    return cv::Scalar(channel(color.b), channel(color.g), channel(color.r), channel(color.a));
    1. 横线不是`cv::Scalar`废弃，是 clang‑tidy 的 modernize 检查提示：**优先列表初始化**。
    2. 两个写法**运行效果完全一样，性能无差别**。
    3. `cv::Scalar(a,b,c,d)`老式；`return {a,b,c,d}`现代 C++ 列表初始化写法。
    4. `{}`好处：防止语法歧义、检测缩窄转换。*/
    return {channel(color.b), channel(color.g), channel(color.r), channel(color.a)};
}

// ============================================================================
// L1 - IMAGE OVERLAY 图像层：相机画面上画框、文字、关键点（OpenCV绘制）
// ============================================================================
namespace Image {

// ROI感兴趣区域
constexpr ::foxglove::schemas::Color ROI_VALID      = with_alpha(Semantic::ALERT, 0.78);    // ROI有效
constexpr ::foxglove::schemas::Color ROI_MISSING    = with_alpha(Semantic::DECISION_NEGATIVE, 0.78); // ROI缺失

// 检测框
constexpr ::foxglove::schemas::Color DETECTION_BOX  = with_alpha(Semantic::STATUS_ACTIVE, 0.92);    // 检测矩形框
constexpr ::foxglove::schemas::Color DETECTION_TEXT = with_alpha(Semantic::STATUS_ACTIVE, 0.90);   // 检测文字标签

// LDM能量机关图像绘制
// constexpr：编译期常量，编译阶段就计算出Color对象，运行时零开销，不占运行时计算
// ::foxglove::schemas::Color 全局命名空间，Foxglove可视化的颜色结构体(r,g,b,a)
// with_alpha(颜色枚举,透明度)：工具函数，基于语义颜色，覆写alpha透明度通道
constexpr ::foxglove::schemas::Color LDM_PRIMARY    = with_alpha(Semantic::STATUS_ACTIVE, 0.92);    // 能量机关主装甲颜色，透明度0.92
constexpr ::foxglove::schemas::Color LDM_SECONDARY  = with_alpha(Semantic::INERT, 0.72);           // 能量机关次要部件颜色，透明度0.72
constexpr ::foxglove::schemas::Color LDM_CENTER     = with_alpha(Semantic::ALERT, 0.90);            // 能量机关中心R点颜色，告警色调，透明度0.90

// 光心、角点可视化
constexpr ::foxglove::schemas::Color OPTICAL_CENTER = with_alpha(Semantic::INERT, 0.82);            // 相机光心可视化颜色，透明度0.82
constexpr ::foxglove::schemas::Color CORNER_PRIMARY = with_alpha(Semantic::STATUS_ACTIVE, 0.92);    // 图像角点（主）可视化颜色，透明度0.92
constexpr ::foxglove::schemas::Color CORNER_SECONDARY = with_alpha(Semantic::INERT, 0.72);          // 图像角点（次）可视化颜色，透明度0.72


// OpenCV绘图像素尺寸
constexpr int LINE_THIN      = 1;      // 细线
constexpr int LINE_MEDIUM    = 2;      // 中等线
constexpr int MARKER_SIZE    = 14;     // 角点标记大小(像素)
constexpr double TEXT_SMALL  = 0.45;   // OpenCV字体缩放
constexpr double TEXT_MEDIUM = 0.55;
constexpr int TEXT_THIN      = 1;      // 文字线条粗细
constexpr int TEXT_MEDIUM_PX = 2;

} // namespace Image

// ============================================================================
// L2 - PERCEPTION 感知层：原始测量、装甲检测、LDM检测结果（3D Marker）
// ============================================================================
namespace L2 { // L2感知层：PnP测量、原始检测结果可视化
constexpr ::foxglove::schemas::Color MEASUREMENT_GHOST      = {0.62, 0.66, 0.72, 0.22}; // 旧/过期测量，低透明度虚影
constexpr ::foxglove::schemas::Color MEASUREMENT_CONFIDENCE = {0.62, 0.66, 0.72, 0.48}; // 有效测量结果，透明度更高

// LDM能量机关不同状态颜色
constexpr ::foxglove::schemas::Color LDM_STABLE       = with_alpha(Semantic::STATUS_ACTIVE, 0.78); // 能量机关状态稳定
constexpr ::foxglove::schemas::Color LDM_CONSTRAINED  = with_alpha(Semantic::ALERT, 0.68);         // 能量机关受约束
constexpr ::foxglove::schemas::Color LDM_BEARING_ONLY = with_alpha(Semantic::INERT, 0.52);        // 仅获取角度，无距离
constexpr ::foxglove::schemas::Color LDM_NONE         = with_alpha(Semantic::CONTEXT, 0.30);      // 无能量机关检测结果

// 3D Marker物理尺寸，单位：米
constexpr double ARMOR_SIZE      = 0.055;  // 装甲标记球体尺寸
constexpr double LABEL_FONT_SIZE = 0.042;  // L2层文字标签字体大小

} // namespace L2

// ============================================================================
// L3 - ESTIMATION 状态估计层：卡尔曼跟踪、目标状态、不确定性椭圆
// ============================================================================
namespace L3 { // L3状态估计层，卡尔曼滤波跟踪相关可视化常量
constexpr ::foxglove::schemas::Color TRACKING_LOCKED    = with_alpha(Semantic::STATUS_ACTIVE, 0.92);   // 跟踪锁定，稳定跟踪目标
constexpr ::foxglove::schemas::Color TRACKING_ACQUIRING = with_alpha(Semantic::STATUS_ACTIVE, 0.58);   // 正在捕获目标，刚进入视野
constexpr ::foxglove::schemas::Color TRACKING_WARNING   = with_alpha(Semantic::ALERT, 0.62);          // 跟踪告警，噪声大，状态不稳定
constexpr ::foxglove::schemas::Color TRACKING_LOST = with_alpha(Semantic::DECISION_NEGATIVE, 0.48);   // 目标丢失，丢失跟踪

constexpr ::foxglove::schemas::Color PREDICTION_AMBER   = with_alpha(Semantic::ALERT, 0.62);          // 预测点警告色
constexpr ::foxglove::schemas::Color UNCERTAINTY_ORANGE = with_alpha(Semantic::ALERT, 0.18); // 协方差不确定椭圆，极低透明度
constexpr ::foxglove::schemas::Color PREDICTION_CONTEXT = with_alpha(Semantic::INERT, 0.42);          // 背景参考预测点颜色

// 3D Marker物理尺寸（米）
constexpr double TARGET_SIZE         = 0.095;   // 目标主体标记球体大小
constexpr double ARMOR_PLATE_SIZE    = 0.085;   // 装甲板标记尺寸
constexpr double UNCERTAINTY_SCALE   = 0.18;    // 协方差不确定椭圆缩放系数
constexpr double ROBOT_CENTER_SIZE   = 0.15;    // 机器人中心点标记大小
constexpr double OUTPOST_CENTER_SIZE = 0.17;    // 前哨站中心点标记大小
constexpr double LABEL_OFFSET_Z      = 0.115; // 标签向上偏移Z，避免贴模型表面被遮挡

} // namespace L3

// ============================================================================
// L4 - PLANNING 规划层：云台瞄准、弹道预测、候选目标选择、MPC
// ============================================================================
namespace L4 { // L4规划层，瞄准、弹道、目标选择、MPC可视化
constexpr ::foxglove::schemas::Color GIMBAL_AIM_FIRE =
    with_alpha(Semantic::DECISION_POSITIVE, 0.92); // 云台瞄准，允许开火
constexpr ::foxglove::schemas::Color GIMBAL_AIM_HOLD = with_alpha(Semantic::ALERT, 0.78); // 云台保持，暂不开火

constexpr ::foxglove::schemas::Color FUTURE_ARMOR   = with_alpha(Semantic::INERT, 0.34);  // 预测未来装甲位置颜色
constexpr ::foxglove::schemas::Color SPATIAL_LINK   = with_alpha(Semantic::CONTEXT, 0.32); // 空间连线辅助线
constexpr ::foxglove::schemas::Color SELECTION_LINK = with_alpha(Semantic::STATUS_ACTIVE, 0.42); // 目标选择连线

// 候选目标三档：选中、备选、淘汰
constexpr ::foxglove::schemas::Color CANDIDATE_SELECTED = with_alpha(Semantic::STATUS_ACTIVE, 0.94);    // 当前被选中目标
constexpr ::foxglove::schemas::Color CANDIDATE_RUNNER_UP  = with_alpha(Semantic::ALERT, 0.68);         // 备选候选目标
constexpr ::foxglove::schemas::Color CANDIDATE_ELIMINATED = with_alpha(Semantic::INERT, 0.22);         // 被淘汰候选目标

// 弹道轨迹
constexpr ::foxglove::schemas::Color TRAJECTORY_FIRE =
    with_alpha(Semantic::DECISION_POSITIVE, 0.42); // 可开火弹道
constexpr ::foxglove::schemas::Color TRAJECTORY_HOLD = with_alpha(Semantic::INERT, 0.18); // 不射击弹道

// MPC模型预测控制可视化
constexpr ::foxglove::schemas::Color MPC_PRESENT   = with_alpha(Semantic::STATUS_ACTIVE, 0.58); // MPC当前参考点
constexpr ::foxglove::schemas::Color MPC_REFERENCE = with_alpha(Semantic::ALERT, 0.42);         // MPC参考轨迹线

// 3D Marker几何尺寸（米）
constexpr double SPATIAL_LINK_THICKNESS    = 0.006; // 空间连线线条粗细
constexpr double TRAJECTORY_LINE_THICKNESS = 0.006; // 弹道轨迹线条粗细
constexpr double SELECTION_SIZE            = 0.058;// 选中目标标记尺寸
constexpr double PREDICTION_SIZE           = 0.036;// 预测点球体大小
constexpr double TRAJECTORY_DOT            = 0.014;// 弹道上离散点大小
constexpr double RING_HEIGHT               = 0.004;// 圆环标记高度
constexpr double RING_SCALE                = 1.55; // 圆环标记缩放倍数

} // namespace L4

// ============================================================================
// L5 - WEAPON 武器层：开火状态、冷却、中止射击
// ============================================================================
namespace L5 { // L5武器执行层，开火相关可视化颜色
constexpr ::foxglove::schemas::Color FIRE_EXECUTE  = with_alpha(Semantic::DECISION_POSITIVE, 0.98); // 正在开火
constexpr ::foxglove::schemas::Color FIRE_READY    = with_alpha(Semantic::DECISION_POSITIVE, 0.84); // 开火就绪，可以发射
constexpr ::foxglove::schemas::Color FIRE_COOLDOWN = with_alpha(Semantic::ALERT, 0.66);             // 武器冷却中
constexpr ::foxglove::schemas::Color FIRE_ABORTED  = with_alpha(Semantic::DECISION_NEGATIVE, 0.72); // 开火被中止
} // namespace L5

// 速度矢量箭头可视化
namespace Velocity { // 速度箭头可视化参数命名空间
constexpr ::foxglove::schemas::Color LINEAR  = with_alpha(Semantic::STATUS_ACTIVE, 0.78);  // 线速度箭头颜色
constexpr ::foxglove::schemas::Color ANGULAR = with_alpha(Semantic::ALERT, 0.72);         // 角速度箭头颜色

// 箭头几何体参数（米）
constexpr double ARROW_SHAFT_DIAMETER = 0.016; // 箭杆直径
constexpr double ARROW_HEAD_DIAMETER  = 0.032; // 箭头头部直径
constexpr double ARROW_MIN_LENGTH     = 0.0001;// 箭头最小长度，防止速度为0生成0长度报错
constexpr double ARROW_SHAFT_RATIO    = 0.8;   // 箭杆占箭头总长度比例
constexpr double ARROW_HEAD_RATIO     = 0.2;   // 箭头头部占总长度比例

} // namespace Velocity

// 3D文本标签样式
namespace Text { // 3D文本标签颜色、尺寸配置
constexpr ::foxglove::schemas::Color PRIMARY   = {0.92, 0.94, 0.96, 0.88};   // 主文本，亮白色
constexpr ::foxglove::schemas::Color SECONDARY = {0.72, 0.75, 0.78, 0.66};   // 次要文本，浅灰
constexpr ::foxglove::schemas::Color WARNING   = with_alpha(Semantic::ALERT, 0.78);  // 警告文本颜色
constexpr ::foxglove::schemas::Color ERROR     = with_alpha(Semantic::DECISION_NEGATIVE, 0.78); // 错误文本颜色

constexpr double SIZE_SMALL      = 0.035;  // 小号字体，米
constexpr double SIZE_MEDIUM     = 0.052;  // 中号字体
constexpr double SIZE_LARGE      = 0.070;  // 大号字体
constexpr double SIZE_DEFAULT    = SIZE_MEDIUM; // 默认字体使用中号
constexpr bool BILLBOARD_ENABLED = true; // 文本始终面向相机（Billboard），不会跟着模型旋转歪掉
} // namespace Text

// 装甲实体几何常量，物理尺寸，单位米
namespace Geometry { // 装甲真实物理几何参数，对应RM实物尺寸
constexpr double ARMOR_THICKNESS    = 0.03;    // 装甲板厚度
constexpr double ARMOR_HEIGHT_SMALL = 0.135;   // 小装甲板高度
constexpr double ARMOR_HEIGHT_BIG   = 0.23;    // 大装甲板高度
constexpr double ARMOR_WIDTH        = 0.125;   // 装甲板宽度
constexpr double ARMOR_TILT_ANGLE   = 0.2618; // 弧度，装甲板倾斜角
} // namespace Geometry

// 时间相关可视化参数
namespace Temporal { // Marker生命周期时间配置
constexpr uint64_t ENTITY_LIFETIME_NS  = 100'000'000ULL; // Marker存活时间，纳秒 100ms，过期Foxglove自动删掉
constexpr double MPC_EXTRAPOLATION_VEL = 5.0; // MPC外推最大速度限制
} // namespace Temporal

// 旧代码兼容层，别名映射，不改动老业务代码
namespace Legacy { // 遗留兼容别名，老代码不用改，直接复用上面L2/L3/L4常量
constexpr ::foxglove::schemas::Color COLOR_YELLOW = L4::GIMBAL_AIM_HOLD;   // 旧代码黄色别名，映射云台保持颜色
constexpr ::foxglove::schemas::Color COLOR_CYAN   = L3::TRACKING_LOCKED;    // 旧代码青色别名，映射跟踪锁定
constexpr ::foxglove::schemas::Color COLOR_GREEN  = L4::GIMBAL_AIM_FIRE;    // 旧代码绿色别名，映射允许开火
constexpr ::foxglove::schemas::Color COLOR_WHITE  = L2::MEASUREMENT_GHOST;  // 旧代码白色别名，映射测量虚影
constexpr ::foxglove::schemas::Color COLOR_RED    = L3::TRACKING_LOST;      // 旧代码红色别名，映射目标丢失
constexpr ::foxglove::schemas::Color COLOR_ORANGE = L3::PREDICTION_AMBER;    // 旧代码橙色别名，映射预测警告色

constexpr double SPHERE_SIZE_DEFAULT = L3::TARGET_SIZE;    // 默认球体大小，复用L3目标尺寸
constexpr double SPHERE_SIZE_SMALL   = L3::ARMOR_PLATE_SIZE;// 小球体，复用装甲板尺寸
constexpr double SPHERE_SIZE_TINY    = L4::TRAJECTORY_DOT;  // 微小球体，复用弹道点尺寸

} // namespace Legacy

/**
 * @brief 目标候选三档枚举
 * Selected：当前选中打击目标
 * RunnerUp：次选备选
 * Eliminated：被淘汰，不考虑
 */
enum class SelectionTier { Selected, RunnerUp, Eliminated };

/**
 * @brief 候选目标完整渲染样式结构体
 */
struct SelectionStyle {
    ::foxglove::schemas::Color color;
    double size_scale;    // 几何体缩放系数
    double alpha;         // 透明度
    bool show_label;      // 是否显示文字标签
};

/**
 * @brief 根据候选等级，返回一套对应的颜色、缩放、透明度、标签开关
 * @param tier 候选等级
 * @return SelectionStyle 样式结构体，编译期可返回constexpr
 */
[[nodiscard]] constexpr SelectionStyle selection_style(SelectionTier tier) noexcept {
    switch (tier) {
    case SelectionTier::Selected:
        return {
            .color = L4::CANDIDATE_SELECTED, .size_scale = 1.25, .alpha = 0.94, .show_label = true};
    case SelectionTier::RunnerUp:
        return {
            .color = L4::CANDIDATE_RUNNER_UP, .size_scale = 1.0, .alpha = 0.68, .show_label = true};
    case SelectionTier::Eliminated:
        return {
            .color      = L4::CANDIDATE_ELIMINATED,
            .size_scale = 0.68,
            .alpha      = 0.22,
            .show_label = false};
    }
    // 兜底返回淘汰样式
    return {
        .color = L4::CANDIDATE_ELIMINATED, .size_scale = 0.68, .alpha = 0.22, .show_label = false};
}

/**
 * @brief 根据跟踪器状态码，获取对应显示颜色
 * status_int: 2锁定，1捕获中，3告警，0/默认旧测量虚影
 */
[[nodiscard]] inline ::foxglove::schemas::Color tracker_status_color(int status_int) noexcept {
    switch (status_int) {
    case 2: return L3::TRACKING_LOCKED;
    case 1: return L3::TRACKING_ACQUIRING;
    case 3: return L3::TRACKING_WARNING;
    case 0:
    default: return L2::MEASUREMENT_GHOST;
    }
}

/**
 * @brief 根据是否允许开火，返回弹道颜色
 * @param can_fire true允许开火，false hold
 */
[[nodiscard]] inline ::foxglove::schemas::Color fire_decision_color(bool can_fire) noexcept {
    return can_fire ? L4::TRAJECTORY_FIRE : L4::TRAJECTORY_HOLD;
}

/**
 * @brief 时间衰减透明度：历史帧越远，透明度越低，做轨迹拖尾效果
 * @param base_color 基础颜色
 * @param temporal_distance 和当前帧的时间距离（帧差）
 * @return 衰减后的颜色
 */
[[nodiscard]] inline ::foxglove::schemas::Color
    temporal_fade(const ::foxglove::schemas::Color& base_color, int temporal_distance) noexcept {
    const int dist = std::abs(temporal_distance);
    double alpha   = 1.0;
    if (dist == 0) {
        alpha = 1.00;
    } else if (dist <= 2) {
        alpha = 0.76;
    } else if (dist <= 4) {
        alpha = 0.48;
    } else {
        alpha = 0.22;
    }
    // 在原有alpha基础上再乘衰减系数
    return with_alpha(base_color, alpha * base_color.a);
}

} // namespace fcs::visualization::tactical