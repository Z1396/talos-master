/**
 * @file traditional.hpp
 * @brief 传统装甲板检测后端实现（基于图像处理和几何特征）
 *
 * 本文件实现了 Talos 火控系统的传统装甲板检测算法，不依赖深度学习模型，
 * 而是通过图像预处理、灯条检测、几何匹配等经典计算机视觉技术实现。
 *
 * ## 算法核心流程
 *
 * 1. **图像预处理** (preprocess_image)
 *    - 颜色通道分离（根据装甲板颜色：红色/蓝色）
 *    - 阈值分割（OTSU 或固定阈值）
 *    - 形态学操作（开运算去噪、闭运算填充）
 *
 * 2. **灯条检测** (find_lights)
 *    - 轮廓提取
 *    - 几何特征过滤（长宽比、面积、倾斜角度）
 *    - 灯条拟合（最小外接矩形）
 *
 * 3. **灯条匹配** (match_lights)
 *    - 双灯条配对（基于几何约束）
 *    - 装甲板类型判断（大装甲板/小装甲板）
 *    - 干扰灯条排除（中间灯条检测）
 *
 * 4. **数字提取与分类** (extract_number + classifier_)
 *    - 透视变换提取数字区域
 *    - OTSU 二值化预处理
 *    - ONNX 分类器识别数字（1-5, 前哨站, 基地）
 *
 * ## 传统后端 vs 深度学习后端
 *
 * | 特性            | 传统后端                | 深度学习后端            |
 * |----------------|-----------------------|---------------------|
 * | 速度            | 快（纯 CPU，无推理开销）    | 较慢（需要 GPU 推理）     |
 * | 准确率          | 中等（依赖参数调优）        | 高（端到端学习）         |
 * | 鲁棒性          | 低（光照敏感）           | 高（数据驱动泛化）        |
 * | 部署要求         | 低（无需模型文件）         | 高（需要 ONNX/TensorRT）|
 * | 适用场景         | 算力受限平台、快速原型       | 高性能嵌入式设备        |
 *
 * ## 设计要点
 *
 * - **RAII 生命周期**：通过 `create()` 工厂函数构造，确保初始化失败可恢复
 * - **无异常保证**：所有公共接口返回 `std::expected<T, std::string>`
 * - **只移语义**：禁用拷贝，避免资源竞争和意外复制
 * - **延迟计算**：分类器只在检测到有效装甲板后才调用
 *
 * @author Talos Team
 * @date 2024
 */

#pragma once

#include "../backend.hpp"
#include "../config.hpp"
#include "traditional_classifier.hpp"
#include "traditional_types.hpp"

#include <memory>
#include <vector>

#include <opencv2/core.hpp>

namespace fcs::L2 {

// ============================================================================
// AT Legacy Traditional Backend Implementation
// ============================================================================

/**
 * @class TraditionalBackend
 * @brief 传统装甲板检测后端（基于图像处理 + 几何匹配）
 *
 * 该类实现了完整的传统装甲板检测流水线，包括：
 * - 图像预处理（颜色分离、阈值分割）
 * - 灯条检测与过滤
 * - 灯条配对匹配
 * - 数字区域提取
 * - ONNX 分类器识别
 *
 * 继承自 DetectorBackendBase，遵循 CRTP 静态多态模式，
 * 避免虚函数开销，同时保持接口统一。
 */
class TraditionalBackend : public DetectorBackendBase<TraditionalBackend> {
public:
    /// 检测结果类型：成功返回装甲板列表，失败返回错误信息
    using DetectionResult = std::expected<std::vector<ArmorDetection>, std::string>;

    /// 配置类型：传统检测参数（阈值、面积范围、长宽比等）
    using Config = ArmorTraditionalConfig;

    /**
     * @brief 工厂函数：构造完全初始化的检测后端
     *
     * 采用 "Construction IS Initialization" 模式，所有初始化工作在此完成：
     * - 加载 ONNX 分类器模型
     * - 验证配置参数
     * - 构建内部状态
     *
     * @param config 传统检测配置参数
     * @return 成功返回初始化后的 TraditionalBackend，失败返回错误信息
     *
     * @note 该函数保证 noexcept，失败通过 std::expected 返回
     * @note 无需调用额外的 init() 方法
     */
    [[nodiscard]] static std::expected<TraditionalBackend, std::string>
        create(Config config) noexcept;

    /// 析构函数：自动释放资源（RAII）
    ~TraditionalBackend() = default;

    // 移动构造与赋值（允许资源转移）
    TraditionalBackend(TraditionalBackend&&) noexcept            = default;
    TraditionalBackend& operator=(TraditionalBackend&&) noexcept = default;

    // 禁止拷贝（避免共享分类器模型导致的线程安全问题）
    TraditionalBackend(const TraditionalBackend&)                = delete;
    TraditionalBackend& operator=(const TraditionalBackend&)     = delete;

    /**
     * @brief 执行装甲板检测
     *
     * 完整的检测流水线：
     * 1. 预处理图像（颜色分离、二值化）
     * 2. 检测灯条（轮廓提取、几何过滤）
     * 3. 匹配灯条对（几何约束、中间灯条排除）
     * 4. 提取数字区域（透视变换）
     * 5. 分类数字（ONNX 推理）
     *
     * @param input 输入 BGR 图像（相机原始帧）
     * @param color 目标装甲板颜色（红色/蓝色），用于颜色分离
     * @return 成功返回检测到的装甲板列表，失败返回错误信息
     *
     * @note 该函数保证 noexcept，异常通过 std::expected 返回
     * @note 返回的 ArmorDetection 包含角点、颜色、类型、数字等信息
     */
    [[nodiscard]] DetectionResult detect_impl(const cv::Mat& input, ArmorColor color) noexcept;

    /**
     * @brief 获取当前配置参数
     * @return 配置参数的常量引用
     */
    [[nodiscard]] const Config& get_config() const noexcept { return config_; }

private:
    /**
     * @brief 私有构造函数（通过 create() 工厂函数调用）
     * @param config 检测配置参数
     */
    explicit TraditionalBackend(Config config) noexcept;

    /// 检测配置参数（阈值、面积范围、长宽比等）
    Config config_;

    /// ONNX 分类器（用于识别装甲板数字）
    /// 使用 unique_ptr 管理，支持延迟初始化和可选加载
    std::unique_ptr<TraditionalClassifier> classifier_;

    /**
     * @brief 图像预处理：BGR 图像转二值图像
     *
     * 根据目标装甲板颜色，分离对应颜色通道并二值化：
     * - 红色装甲板：提取红色通道（或 R - (G+B)/2）
     * - 蓝色装甲板：提取蓝色通道（或 B - (G+R)/2）
     *
     * 预处理流程：
     * 1. 颜色通道分离（增强目标颜色对比度）
     * 2. 灰度化（单通道）
     * 3. 二值化（OTSU 自适应阈值或固定阈值）
     * 4. 形态学操作（开运算去噪、闭运算填充孔洞）
     *
     * @param bgr_img 输入 BGR 图像
     * @param cfg 检测配置参数（包含阈值、形态学核大小等）
     * @return 二值化后的图像（0 或 255）
     */
    [[nodiscard]] cv::Mat
        preprocess_image(const cv::Mat& bgr_img, const ArmorTraditionalConfig& cfg) const noexcept;

    /**
     * @brief 检测图像中的灯条
     *
     * 灯条检测流程：
     * 1. 在二值图像上查找轮廓
     * 2. 对每个轮廓拟合最小外接矩形
     * 3. 根据几何特征过滤（长宽比、面积、倾斜角度）
     * 4. 构建 Light 对象（包含 top、bottom、axis 等几何信息）
     *
     * @param bgr_img 原始 BGR 图像（用于颜色识别）
     * @param binary_img 二值化图像（用于轮廓提取）
     * @param cfg 检测配置参数（包含面积范围、长宽比阈值等）
     * @return 检测到的灯条列表
     */
    [[nodiscard]] std::vector<Light> find_lights(
        const cv::Mat& bgr_img, const cv::Mat& binary_img,
        const ArmorTraditionalConfig& cfg) const noexcept;

    /**
     * @brief 判断轮廓是否为有效灯条
     *
     * 几何约束过滤：
     * - 长宽比：灯条细长（长宽比 > 阈值）
     * - 面积：排除过小噪声和过大干扰
     * - 倾斜角度：排除过于倾斜的灯条
     *
     * @param light 待判断的灯条对象
     * @param cfg 检测配置参数
     * @return true 表示是有效灯条，false 表示无效
     */
    [[nodiscard]] bool
        is_light(const Light& light, const ArmorTraditionalConfig& cfg) const noexcept;

    /**
     * @brief 匹配灯条对，形成装甲板检测结果
     *
     * 灯条匹配算法：
     * 1. 遍历所有灯条对组合
     * 2. 几何约束过滤：
     *    - 平行性：两灯条轴线接近平行
     *    - 长度比：两灯条长度接近
     *    - 间距：灯条间距在合理范围内
     * 3. 排除中间有其他灯条的配对（干扰排除）
     * 4. 判断装甲板类型（大装甲板/小装甲板）
     * 5. 提取数字区域并分类
     *
     * @param lights 灯条列表（可能被修改以优化匹配）
     * @param cfg 检测配置参数
     * @param gray_img 灰度图像（用于数字提取）
     * @return 匹配到的装甲板列表
     */
    [[nodiscard]] std::vector<ArmorDetection> match_lights(
        std::vector<Light>& lights, const ArmorTraditionalConfig& cfg,
        cv::Mat gray_img) const noexcept;

    /**
     * @brief 检查两个灯条之间是否存在其他灯条（干扰排除）
     *
     * 用于排除包含中间灯条的无效匹配：
     * - 英雄机器人可能有多灯条干扰
     * - 前哨站有多个灯条排列
     *
     * 算法原理：
     * 检查其他灯条的中心点是否落在两灯条构成的凸包内。
     *
     * @param i 第一个灯条索引
     * @param j 第二个灯条索引
     * @param lights 灯条列表
     * @return true 表示存在中间灯条（匹配无效），false 表示无干扰
     */
    [[nodiscard]] static bool
        contains_light(size_t i, size_t j, const std::vector<Light>& lights) noexcept;

    /**
     * @brief 判断两个灯条是否构成有效装甲板
     *
     * 几何约束判断：
     * - 平行性：两灯条轴线夹角 < 阈值
     * - 长度比：长短灯条比 < 阈值
     * - 间距：灯条间距与灯条长度的比在合理范围
     * - 宽高比：判断是大装甲板还是小装甲板
     *
     * @param light_1 第一个灯条
     * @param light_2 第二个灯条
     * @param cfg 检测配置参数
     * @return 装甲板类型（Small/Big/Invalid）
     */
    [[nodiscard]] ArmorType is_armor(
        const Light& light_1, const Light& light_2,
        const ArmorTraditionalConfig& cfg) const noexcept;

    /**
     * @brief 提取装甲板数字区域
     *
     * 数字提取流程：
     * 1. 根据两灯条的四个顶点，计算透视变换矩阵
     * 2. 透视变换矫正透视畸变，得到正视图
     * 3. OTSU 二值化去除背景干扰
     * 4. 归一化到分类器输入尺寸（28x28）
     *
     * @param src 原始 BGR 图像
     * @param lights_vertices 四个灯条顶点（TL, TR, BR, BL）
     * @param armor_type 装甲板类型（用于确定提取区域大小）
     * @return 提取的数字图像（灰度图）
     */
    [[nodiscard]] cv::Mat extract_number(
        const cv::Mat& src, const std::array<cv::Point2f, 4>& lights_vertices,
        ArmorType armor_type) const noexcept;
};

} // namespace fcs::L2