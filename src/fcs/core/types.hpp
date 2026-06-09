// 头文件保护，防止重复包含
#pragma once

// 项目内部头文件：装甲相关枚举、类型定义
#include "core/armor_types.hpp"
// 项目内部头文件：坐标系帧标识定义
#include "frame.hpp"

// 标准库
#include <array>         // 固定大小数组
#include <cstdint>       // 固定宽度整型（uint64_t 等）
#include <string>        // 字符串
#include <string_view>   // 轻量只读字符串视图
#include <type_traits>   // 类型特征、编译期类型判断
#include <vector>        // 动态数组

// 线性代数/几何库：姿态、矩阵、向量运算
#include <Eigen/Core>
#include <Eigen/Geometry>
// OpenCV：图像、点、矩形等视觉基础结构
#include <opencv2/core.hpp>

/**
 * @namespace fcs
 * @brief 视觉感知模块命名空间，存放装甲检测、测量、帧数据相关结构体
 *        整体为机器人装甲视觉链路数据载体：原图 -> 2D检测 -> 3D位姿测量
 */
namespace fcs {

/**
 * @brief 空标签类型，用于标记「装甲坐标系」，编译期区分不同坐标系
 *        标签类：无成员、仅用作类型标识，配合模板实现编译期坐标系安全
 */
struct armor_frame_tag {};

/**
 * @brief 类型别名，简化装甲坐标系标签书写
 */
using armor_frame = armor_frame_tag;

// ============================================================================
// 注释：传统调度通道使用的帧数据结构（适配调度器数据流）
// ============================================================================

/**
 * @struct ImageFrame
 * @brief 带时间戳的原始图像帧
 *        视觉流水线最上游数据：相机原图 + 时间戳 + 帧编号
 */
struct ImageFrame {
    cv::Mat image;               // OpenCV 图像数据
    uint64_t timestamp_ns = 0;   // 时间戳，单位：纳秒
    uint64_t frame_id     = 0;   // 图像帧全局唯一编号

    // 默认构造
    ImageFrame() = default;

    /**
     * @brief 带参构造，使用移动语义避免图像拷贝
     * @param img 输入图像
     * @param ts 纳秒时间戳
     * @param fid 帧编号
     */
    ImageFrame(cv::Mat img, uint64_t ts, uint64_t fid)
        : image(std::move(img))  // 移动图像，提升性能
        , timestamp_ns(ts)
        , frame_id(fid) {}
};

/**
 * @struct ArmorDetection
 * @brief 单块装甲2D检测结果（PnP解算前，图像坐标系下）
 *        存储神经网络输出的单装甲关键点、类别、置信度等2D信息
 */
struct ArmorDetection {
    /// 四个角点：顺序 左上(TL)、右上(TR)、右下(BR)、左下(BL)
    std::array<cv::Point2f, 4> corners{};
    cv::Rect2f rect{};                // 装甲外接矩形
    ArmorName name   = ArmorName::Invalid;    // 装甲编号（枚举，默认无效）
    ArmorColor color = ArmorColor::Neutral;   // 装甲颜色（红/蓝/中立，默认中立）
    ArmorType type   = ArmorType::Invalid;    // 装甲类型（大装甲/小装甲，默认无效）
    float confidence = 0.0f;          // 检测置信度 [0, 1]
    int area         = 0;             // 装甲区域像素面积

    // 默认构造，成员自动零初始化
    ArmorDetection() = default;

    /**
     * @brief 从神经网络输出构造检测结果
     * @param pts 四个角点数组
     * @param armor_name 装甲编号
     * @param color 装甲颜色
     * @param conf 置信度
     */
    ArmorDetection(
        std::array<cv::Point2f, 4> pts, ArmorName armor_name, ArmorColor color, float conf);

    /**
     * @brief 计算装甲中心点（四个角点平均）
     * @return 图像坐标系下2D中心点
     * @noexcept 不抛出异常
     * @nodiscard 禁止忽略返回值
     */
    [[nodiscard]] cv::Point2f center() const noexcept {
        return (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;
    }

    /**
     * @brief 将固定数组角点转为std::vector，适配PnP接口
     * @return 四点构成的点向量
     * @nodiscard 禁止忽略返回值
     */
    [[nodiscard]] std::vector<cv::Point2f> image_points() const {
        return {corners[0], corners[1], corners[2], corners[3]};
    }
};

/**
 * @struct ArmorDetectionBatch
 * @brief 单张图像对应的批量装甲检测结果
 *        一张图可能检出多块装甲，该结构体打包所有检测结果+原图+附加信息
 */
struct ArmorDetectionBatch {
    std::vector<ArmorDetection> detections;  // 多张装甲检测结果列表
    cv::Mat image;                           // 原始图像（用于可视化、后处理）
    bool has_detector_roi = false;           // 是否启用检测感兴趣区域(ROI)
    cv::Rect detector_roi{};                 // 检测器ROI区域
    uint64_t timestamp_ns = 0;               // 帧纳秒时间戳
    uint64_t frame_id     = 0;               // 帧编号

    // 默认构造
    ArmorDetectionBatch() = default;

    /**
     * @brief 批量结果构造函数，使用移动语义减少拷贝
     * @param dets 检测结果列表
     * @param img 原始图像
     * @param ts 时间戳
     * @param fid 帧编号
     * @param has_roi 是否开启ROI
     * @param roi ROI矩形区域
     */
    ArmorDetectionBatch(
        std::vector<ArmorDetection> dets, cv::Mat img, uint64_t ts, uint64_t fid,
        bool has_roi = false, cv::Rect roi = {})
        : detections(std::move(dets))
        , image(std::move(img))
        , has_detector_roi(has_roi)
        , detector_roi(roi)
        , timestamp_ns(ts)
        , frame_id(fid) {}

    /**
     * @brief 判断当前帧是否未检测到任何装甲
     * @return true=无检测结果，false=有结果
     * @noexcept 不抛异常
     * @nodiscard 禁止忽略返回值
     */
    [[nodiscard]] bool empty() const noexcept { return detections.empty(); }

    /**
     * @brief 获取检测到的装甲数量
     * @return 装甲个数
     * @noexcept 不抛异常
     * @nodiscard 禁止忽略返回值
     */
    [[nodiscard]] size_t size() const noexcept { return detections.size(); }
};

/**
 * @template ArmorMeasurementT
 * @brief 模板结构体：单块装甲3D位姿测量结果
 * @tparam Frame 坐标系模板参数（编译期指定所属坐标系：相机系/世界系/里程计系等）
 *        数据来源：2D检测 + PnP解算 + TF坐标变换，输出3D位姿、协方差、评估指标
 */
template <fast_tf::frame Frame>
struct ArmorMeasurementT {
    /// 位姿变换矩阵：从当前Frame 到 装甲坐标系(armor_frame) 的变换矩阵
    using Transform = fast_tf::TransformMatrixd<Frame, armor_frame>;

    Transform transform{};                          // 坐标变换矩阵（位姿：旋转+平移）
    ArmorName name                 = ArmorName::Invalid;    // 装甲编号
    ArmorColor color               = ArmorColor::Neutral;   // 装甲颜色
    ArmorType type                 = ArmorType::Small;        // 装甲类型（默认小装甲）
    float confidence               = 0.0f;                  // 检测置信度
    /// 装甲中心到图像中心的像素距离（用于筛选、权重计算）
    float distance_to_image_center = 0.0f;

    /**
     * @brief PnP位姿协方差矩阵 4×4
     *        维度：[偏航角、俯仰角、对数距离、装甲自身偏航角]
     *        由重投影雅可比矩阵估计，用于EKF滤波
     */
    Eigen::Matrix4d pnp_cov_ypdr = Eigen::Matrix4d::Identity() * 1e6;

    /**
     * @brief PnP雅可比矩阵条件数
     *        数值稳定性指标：数值越大表示PnP解算越不稳定、结果不可信
     */
    double pnp_condition_number = 1e6;

    uint64_t timestamp_ns       = 0;  // 测量结果对应的纳秒时间戳

    // 默认构造，成员自动初始化
    ArmorMeasurementT() = default;

    /**
     * @brief 获取装甲相对于当前坐标系的直线距离（平移向量模长）
     * @return 空间距离，单位：米
     * @noexcept 不抛异常
     * @nodiscard 禁止忽略返回值
     */
    [[nodiscard]] double distance() const noexcept { return transform.translation().norm(); }

    /**
     * @brief 获取装甲姿态偏航角（绕Z轴）
     * @return 偏航角，单位：弧度
     * @noexcept 不抛异常
     * @nodiscard 禁止忽略返回值
     */
    [[nodiscard]] double yaw() const noexcept { return transform.euler_rot().yaw; }

    /**
     * @brief 获取装甲姿态俯仰角（绕Y轴）
     * @return 俯仰角，单位：弧度
     * @noexcept 不抛异常
     * @nodiscard 禁止忽略返回值
     */
    [[nodiscard]] double pitch() const noexcept { return transform.euler_rot().pitch; }

    /**
     * @brief 坐标重映射：将当前坐标系下的测量结果转换到目标坐标系
     * @tparam ToFrame 目标坐标系
     * @param frame_transform 变换矩阵：ToFrame -> 原Frame
     * @return 新坐标系下的装甲测量结果
     * @nodiscard 禁止忽略返回值
     * @note 模板函数实现写在类内，保证编译期模板实例化正常
     */
    template <fast_tf::frame ToFrame>
    [[nodiscard]] ArmorMeasurementT<ToFrame>
        reframe(const fast_tf::FrameTransform<ToFrame, Frame>& frame_transform) const {
        ArmorMeasurementT<ToFrame> reframed;
        // 矩阵左乘，完成坐标系转换
        reframed.transform                = frame_transform * transform;
        // 纯数据字段直接拷贝
        reframed.name                     = name;
        reframed.color                    = color;
        reframed.type                     = type;
        reframed.confidence               = confidence;
        reframed.distance_to_image_center = distance_to_image_center;
        reframed.pnp_cov_ypdr             = pnp_cov_ypdr;
        reframed.pnp_condition_number     = pnp_condition_number;
        reframed.timestamp_ns             = timestamp_ns;
        return reframed;
    }
};

// 类型别名：里程计坐标系下的装甲3D测量结果（业务主使用类型）
using ArmorMeasurement       = ArmorMeasurementT<fast_tf::odom>;
// 类型别名：相机光心坐标系下的装甲3D测量结果
using CameraArmorMeasurement = ArmorMeasurementT<fast_tf::camera_optical>;

/**
 * @template ArmorMeasurementBatchT
 * @brief 模板结构体：单帧图像对应的批量3D装甲测量结果
 * @tparam Frame 所属坐标系
 *        打包一帧内所有装甲的3D测量数据，作为数据流下游模块输入
 */
template <fast_tf::frame Frame>
struct ArmorMeasurementBatchT {
    std::vector<ArmorMeasurementT<Frame>> measurements;  // 多块装甲测量结果列表
    uint64_t timestamp_ns = 0;                          // 帧纳秒时间戳
    uint64_t frame_id     = 0;                          // 帧编号

    // 默认构造
    ArmorMeasurementBatchT() = default;

    /**
     * @brief 批量结果构造，移动语义减少数据拷贝
     * @param meas 测量结果列表
     * @param ts 时间戳
     * @param fid 帧编号
     */
    ArmorMeasurementBatchT(std::vector<ArmorMeasurementT<Frame>> meas, uint64_t ts, uint64_t fid)
        : measurements(std::move(meas))
        , timestamp_ns(ts)
        , frame_id(fid) {}

    /**
     * @brief 判断当前帧是否无有效3D测量结果
     * @return true=无结果，false=有结果
     * @noexcept 不抛异常
     * @nodiscard 禁止忽略返回值
     */
    [[nodiscard]] bool empty() const noexcept { return measurements.empty(); }

    /**
     * @brief 获取当前帧装甲测量数量
     * @return 装甲个数
     * @noexcept 不抛异常
     * @nodiscard 禁止忽略返回值
     */
    [[nodiscard]] size_t size() const noexcept { return measurements.size(); }
};

// 类型别名：里程计坐标系下的批量3D测量结果（全局业务默认使用）
using ArmorMeasurementBatch = ArmorMeasurementBatchT<fast_tf::odom>;

} // namespace fcs