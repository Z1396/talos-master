#pragma once
// 装甲颜色枚举 ArmorColor 定义
#include "core/armor_types.hpp"
// 图像帧基础结构体、时间戳类型
#include "frame.hpp"

// STL容器与基础数值类型
// 固定宽度整数类型 uint8_t / uint64_t
#include <cstdint>
// 数值极限、NaN常量
#include <limits>
// 可选值容器，区分有无有效数据
#include <optional>
// 编译期只读字符串视图，无内存拷贝
#include <string_view>
// 动态变长数组，存储灯条、灯对、候选网格
#include <vector>

// Eigen线性代数库，三维向量、旋转变换
#include <Eigen/Core>
#include <Eigen/Geometry>
// fmt格式化打印库
#include <fmt/core.h>
// 枚举反射库，快速枚举转字符串
#include <magic_enum.hpp>
// OpenCV图像处理基础浮点矩形、点
#include <opencv2/imgproc.hpp>

/**
 * @brief 顶层命名空间：fcs::L2::ldm
 * fcs：项目总工程命名空间
 * L2：第二层图像感知视觉模块
 * ldm：Laser Module 激光大符识别子模块
 * 全部大符识别业务数据结构体、枚举统一存放于此，隔离命名冲突
 */
namespace fcs::L2::ldm {

/**
 * @brief 检测器执行错误枚举
 * 标识图像检测流程运行时故障类型
 */
enum class DetectorError {
    /// 输入图像为空/尺寸非法，无法执行检测
    InvalidImage
};

/**
 * @brief 自定义坐标系标识结构体：ldm大符物体坐标系
 * 用于fast_tf坐标变换模板特化，标记变换目标坐标系为大符本体
 */
struct ldm_frame {
    /// 坐标系唯一字符串标识
    static constexpr std::string_view frame_id = "ldm";
    /// 无父级坐标系祖先，填void
    using ancestor = void;
};

/**
 * @brief 相机光学坐标系 → 大符本体坐标系变换
 * FrameTransform<源坐标系, 目标坐标系>
 * 存储旋转+平移，代表 T_cam_ldm：大符点转到相机坐标系
 */
using CameraLdmTransform = fast_tf::FrameTransform<fast_tf::camera_optical, ldm_frame>;

/**
 * @brief 里程计odom坐标系 → 大符本体坐标系变换
 * T_odom_ldm：大符三维点转换到里程计世界坐标系
 */
using OdomLdmTransform = fast_tf::FrameTransform<fast_tf::odom, ldm_frame>;

/**
 * @brief 单组大符候选完整位姿封装
 * 同时存储相机坐标系、里程计坐标系两套变换矩阵
 */
struct LdmCandidatePose {
    /// 相机→大符 位姿变换
    CameraLdmTransform camera{};
    /// 里程计→大符 位姿变换
    OdomLdmTransform odom{};
};

/**
 * @brief 深度/距离解算质量分级枚举 uint8_t存储节省内存
 * 用于下游跟踪器判断测量可信度
 */
enum class LdmDepthQuality : uint8_t {
    /// 无有效深度，仅能输出2D像素框
    None = 0,
    /// 仅方位角有效，无可靠距离（远距离、单面对识别）
    BearingOnly,
    /// 距离存在约束，精度一般（3~4个灯对）
    Constrained,
    /// 完整8面多灯对，深度稳定可靠，高精度测量
    Stable,
};

/**
 * @brief 单条灯条Blob结构体
 * 图像二值化后提取的灯条轮廓基础属性
 */
struct LightBlob {
    /// 灯条外接浮点矩形 x/y/宽高像素
    cv::Rect2f rect{};
    /// 灯条几何中心点像素坐标
    cv::Point2f center_px{};
    /// 轮廓面积（像素）
    float area_px{0.0f};
    /// 宽高比 width / height
    float aspect_ratio{0.0f};
    /// 填充率 = 轮廓面积 / 外接矩形面积，区分完整/残缺灯条
    float fill_ratio{0.0f};
    /// 所属聚类ID，-1代表无聚类
    int cluster_id{-1};
    /// PCA水平排序局部坐标，区分前后立面
    float local_order_px{0.0f};
    /// PCA竖直分层局部坐标，区分上下灯条
    float local_layer_px{0.0f};
};

/**
 * @brief 上下一对灯条配对结构体
 * 同一八边形立面上下两个灯条组成一组LightPair
 */
struct LightPair {
    /// 上方灯条在blobs数组内索引
    int top_blob_index{-1};
    /// 下方灯条在blobs数组内索引
    int bottom_blob_index{-1};
    /// 上灯条像素中心
    cv::Point2f top_center_px{};
    /// 下灯条像素中心
    cv::Point2f bottom_center_px{};
    /// 一对灯条中点
    cv::Point2f midpoint_px{};
    /// 上下灯条水平像素差值 dx
    float center_dx_px{0.0f};
    /// 上下灯条竖直像素差值 dy
    float center_dy_px{0.0f};
    /// 所属聚类ID
    int cluster_id{-1};
    /// 该配对水平局部排序坐标
    float local_order_px{0.0f};
    /// 配对竖直分层间距（PCA计算）
    float local_layer_sep_px{0.0f};
    /// 配对匹配打分 0~1，越高匹配大符模型越好
    float score{0.0f};
};

/**
 * @brief 大符网格候选结构体
 * 一段连续灯对组合，代表一个潜在完整大符，用于PnP求解
 */
struct LdmMeshCandidate {
    /// 该候选包含的所有LightPair索引列表
    std::vector<int> pair_indices{};
    /// 每一组灯对对应的八边形立面编号 0~7
    std::vector<int> octagon_face_indices{};
    /// 所属聚类ID
    int cluster_id{-1};
    /// 是否完成PnP位姿求解
    bool solved{false};
    /// 解算出的深度/距离是否有效
    bool depth_valid{false};
    /// 片段原始匹配打分（灯对均匀度、对齐度综合分）
    float preliminary_score{0.0f};
    /// PnP重投影误差，单位像素，NaN代表未求解
    float reprojection_rmse_px{std::numeric_limits<float>::quiet_NaN()};
    /// 综合最终打分（包含重投影误差、数量加分）
    float score{0.0f};
    /// 候选在图像上预估中心点
    cv::Point2f estimated_center_image_px{};
    /// 该候选解算出来的相机/里程计位姿
    LdmCandidatePose pose{};
    /// 大符八边形外轮廓投影到图像的像素点，用于可视化绘制
    std::vector<cv::Point2f> projected_outline_image{};
};

/**
 * @brief 2D图像层检测结果结构体
 * 存储原图提取全部灯条、灯对、候选网格，仅含像素信息，无三维位姿
 */
struct LdmDetection {
    /// 图像时间戳 纳秒
    uint64_t timestamp_ns{0};
    /// 图像帧序列号
    uint64_t frame_id{0};
    /// 全部灯条、灯对联合包围盒
    cv::Rect2f rect{};
    /// 本次识别目标颜色 红/蓝/紫/中性
    ArmorColor color = ArmorColor::Neutral;
    /// 是否高精度识别（紫色大符标记true）
    bool accurate{false};
    /// 原图提取所有合法灯条Blob
    std::vector<LightBlob> blobs{};
    /// 聚类配对生成的全部灯对
    std::vector<LightPair> pairs{};
    /// 所有筛选后的大符网格候选
    std::vector<LdmMeshCandidate> mesh_candidates{};
    /// 选中最优候选下标，无则std::nullopt
    std::optional<int> selected_candidate_idx{};
    /// 图像中大符中心点像素坐标
    std::optional<cv::Point2f> center_image_px{};

    /**
     * @brief 获取当前检测到的灯对总数量
     * const noexcept：只读、无异常
     * @return size_t 灯对数量
     */
    [[nodiscard]] size_t pair_count() const noexcept { return pairs.size(); }
};

/**
 * @brief 三维位姿测量结果结构体
 * PnP求解完成后输出，包含相机/里程计坐标系三维变换、距离可信度
 */
struct LdmMeasurement {
    /// 对应图像纳秒时间戳
    uint64_t timestamp_ns{0};
    /// 图像帧序号
    uint64_t frame_id{0};
    /// 识别装甲颜色
    ArmorColor color{ArmorColor::Neutral};
    /// 是否高精度识别（紫色大符）
    bool accurate{false};
    /// 图像内总灯对数量
    int pair_count_total{0};
    /// 参与PnP求解的有效灯对数量
    int selected_pair_count{0};
    /// 图像中大符中心点像素
    cv::Point2f center_image_px{};
    /// 相机坐标系下大符单位观测方向向量（仅方位，无距离）
    Eigen::Vector3d bearing_cam{0.0, 0.0, 1.0};
    /// 相机→大符完整变换矩阵（有距离才存在）
    std::optional<CameraLdmTransform> transform_cam{};
    /// 里程计世界→大符完整变换矩阵（有距离才存在）
    std::optional<OdomLdmTransform> transform_odom{};
    /// 深度/距离解算质量分级
    LdmDepthQuality depth_quality{LdmDepthQuality::None};
    /// 综合置信度 0~1
    float confidence{0.0f};
    /// 全部候选网格副本，用于下游可视化调试
    std::vector<LdmMeshCandidate> mesh_candidates{};
    /// 最优求解候选下标
    std::optional<int> selected_candidate_idx{};
};

} // namespace fcs::L2::ldm

/**
 * @brief fmt库自定义格式化特化：LdmDepthQuality枚举打印
 * 实现将枚举自动转为字符串文本输出日志
 */
namespace fmt {

template <>
struct formatter<fcs::L2::ldm::LdmDepthQuality> : formatter<std::string_view> {
    /**
     * @brief 格式化枚举值为字符串
     * @param quality 深度质量枚举
     * @param ctx fmt输出上下文
     * @return 格式化迭代器
     */
    auto format(const fcs::L2::ldm::LdmDepthQuality quality, format_context& ctx) const {
        // magic_enum获取枚举名称字符串，复用string_view格式化逻辑
        return formatter<std::string_view>::format(magic_enum::enum_name(quality), ctx);
    }
};

} // namespace fmt