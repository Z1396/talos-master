#pragma once

/**
 * @file ort.hpp
 * @brief ONNXRuntime神经网络推理后端实现
 *
 * 本文件定义了OrtBackend类，它是Talos火控系统中装甲板检测的神经网络推理后端。
 * 该后端基于ONNXRuntime实现，支持跨平台硬件加速（CPU、OpenVINO、CoreML等）。
 *
 * 核心功能：
 * - 模型加载与初始化：支持多种执行提供器的自动选择与降级
 * - 图像预处理：实现letterbox缩放，保持目标长宽比
 * - 神经网络推理：执行装甲板检测模型推理
 * - 后处理：解析模型输出，提取装甲板检测结果
 * - NMS：非极大值抑制，去除重叠检测框
 *
 * 设计模式：
 * - PIMPL模式：隐藏ONNXRuntime的复杂头文件依赖，减少编译时间
 * - RAII：所有资源通过智能指针管理，自动释放
 * - 工厂模式：使用static create()方法进行两阶段初始化
 *
 * @author Talos Team
 * @date 2024
 */

#include "../backend.hpp"
#include "../config.hpp"
#include "L2_perception/armor/backends/base.hpp"
#include "core/armor_types.hpp"
#include "core/types.hpp"

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace fcs::L2 {

// ============================================================================
// AT Legacy NN Backend Implementation
// ============================================================================

/**
 * @class OrtBackend
 * @brief 基于ONNXRuntime的神经网络装甲板检测后端
 *
 * 该类实现了装甲板检测的完整推理流程，从模型加载到结果输出。
 * 继承自DetectorBackendBase<OrtBackend>，采用CRTP模式避免虚函数开销。
 *
 * 关键设计决策：
 * 1. **模型输入规格固定**：输入尺寸为640x640，适配YOLO系列模型
 * 2. **多类别支持**：支持64个类别的装甲板识别（4颜色×2尺寸×8编号）
 * 3. **关键点检测**：检测装甲板的4个角点，用于后续姿态估计
 * 4. **PIMPL封装**：隐藏ONNXRuntime实现细节，减少头文件污染
 *
 * 使用示例：
 * @code
 * ArmorOrtConfig config;
 * config.model_path = "/path/to/model.onnx";
 * config.confidence_threshold = 0.5;
 *
 * auto backend = OrtBackend::create(config);
 * if (backend) {
 *     auto result = backend->detect_impl(image, ArmorColor::Blue);
 *     // 处理检测结果
 * }
 * @endcode
 */
class OrtBackend : public DetectorBackendBase<OrtBackend> {
public:
    // ========================================================================
    // 模型输入规格常量
    // ========================================================================

    static constexpr int INPUT_W     = 640; ///< 模型输入宽度（像素），适配YOLO标准输入
    static constexpr int INPUT_H     = 640; ///< 模型输入高度（像素），适配YOLO标准输入
    static constexpr int NUM_COLORS  = 4;   ///< 支持的装甲板颜色数量（红、蓝、灰、紫）
    static constexpr int NUM_SIZES   = 2;   ///< 装甲板尺寸类型数量（小装甲板、大装甲板）
    static constexpr int NUM_CLASSES = 8;   ///< 装甲板编号类别数量（哨兵、1-5号、前哨站、基地）
    static constexpr int NUM_KPTS    = 4;   ///< 每个装甲板的关键点数量（4个角点）

    // ========================================================================
    // 类型别名定义
    // ========================================================================

    using DetectionResult = std::expected<
        std::vector<ArmorDetection>,
        std::string>;              ///< 检测结果类型：成功返回检测结果列表，失败返回错误信息
    using Config = ArmorOrtConfig; ///< 配置类型别名

    // ========================================================================
    // 构造与生命周期管理
    // ========================================================================

    /**
     * @brief 工厂方法：创建并初始化ONNXRuntime后端
     *
     * 这是创建OrtBackend实例的唯一正确方式。该方法实现了"构造即初始化"原则，
     * 确保返回的对象要么完全可用，要么返回详细的错误信息。
     *
     * **执行流程：**
     * 1. 创建ONNXRuntime环境和会话选项
     * 2. 根据平台选择最优执行提供器：
     *    - Linux x86_64: 优先OpenVINO（GPU_FP16），失败降级到GPU_FP32，最后CPU
     *    - macOS: 使用CoreML（CPUAndNeuralEngine）
     *    - 其他: 使用CPUExecutionProvider
     * 3. 加载ONNX模型文件
     * 4. 验证模型输入输出规格
     * 5. 分配推理所需的缓冲区
     *
     * @param config 配置参数，包含模型路径、阈值等
     * @return 成功返回初始化后的OrtBackend对象，失败返回错误信息
     *
     * @note 该方法noexcept，不会抛出异常，所有错误通过std::expected传播
     * @note 如果模型文件不存在或格式错误，会返回详细的错误信息
     *
     * @example
     * @code
     * ArmorOrtConfig config;
     * config.model_path = "/path/to/model.onnx";
     * auto backend = OrtBackend::create(config);
     * if (!backend) {
     *     SPDLOG_ERROR("Failed: {}", backend.error());
     * }
     * @endcode
     */
    [[nodiscard]] static std::expected<OrtBackend, std::string> create(Config config) noexcept;

    /**
     * @brief 析构函数
     *
     * 自动释放ONNXRuntime资源（会话、环境等）。
     * 由于使用PIMPL模式和unique_ptr，析构逻辑在实现文件中定义。
     */
    ~OrtBackend();

    /**
     * @brief 移动构造函数
     *
     * 支持移动语义，转移ONNXRuntime资源的所有权。
     * 移动后源对象处于有效但未指定状态。
     *
     * @param other 源对象
     */
    OrtBackend(OrtBackend&&) noexcept;

    /**
     * @brief 移动赋值运算符
     *
     * 支持移动语义，转移ONNXRuntime资源的所有权。
     *
     * @param other 源对象
     * @return 当前对象引用
     */
    OrtBackend& operator=(OrtBackend&&) noexcept;

    // 禁用拷贝操作：ONNXRuntime资源不可共享
    OrtBackend(const OrtBackend&)            = delete;
    OrtBackend& operator=(const OrtBackend&) = delete;

    // ========================================================================
    // 核心检测接口
    // ========================================================================

    /**
     * @brief 执行装甲板检测（同步接口）
     *
     * 该方法是检测流程的主入口，执行完整的推理管线：
     * 1. 预处理（letterbox缩放 + 归一化）
     * 2. 模型推理
     * 3. 后处理（解析输出 + 坐标变换）
     * 4. NMS（非极大值抑制）
     *
     * @param image 输入图像，支持任意尺寸，会自动缩放到模型输入尺寸
     * @param color 目标装甲板颜色，用于后续过滤（当前未使用，预留接口）
     * @return 成功返回检测结果列表，失败返回错误信息
     *
     * @note 该方法noexcept，所有错误通过std::expected传播
     * @note 检测结果的坐标已经变换回原始图像坐标系
     * @note 执行时间主要消耗在模型推理，通常为5-20ms（取决于硬件）
     */
    [[nodiscard]] DetectionResult detect_impl(const cv::Mat& image, ArmorColor color) noexcept;

    /**
     * @brief 获取可用的执行提供器列表
     *
     * 返回当前ONNXRuntime构建中可用的执行提供器，如：
     * - CPUExecutionProvider
     * - OpenVINOExecutionProvider
     * - CoreMLExecutionProvider
     *
     * @return 可用提供器名称列表
     *
     * @note 用于诊断和调试，确认硬件加速是否可用
     */
    [[nodiscard]] std::vector<std::string> execution_devices() const noexcept;

    /**
     * @brief 获取当前配置
     *
     * 返回创建时传入的配置对象，包含模型路径、阈值等参数。
     *
     * @return 配置对象的const引用
     */
    [[nodiscard]] const Config& get_config() const noexcept { return config_; }

private:
    // ========================================================================
    // 私有构造与内部实现
    // ========================================================================

    /**
     * @brief 私有构造函数
     *
     * 仅用于工厂方法内部。外部用户必须通过create()创建实例。
     *
     * @param config 配置参数
     */
    explicit OrtBackend(Config config) noexcept;

    // ========================================================================
    // 预处理上下文结构体
    // ========================================================================

    /**
     * @struct PreprocContext
     * @brief 预处理上下文，保存letterbox变换的参数
     *
     * 该结构体记录了从原始图像到模型输入的坐标变换关系，
     * 用于在推理后将检测坐标映射回原始图像。
     *
     * **Letterbox原理：**
     * 为了保持目标的长宽比，将原始图像缩放到模型输入尺寸内，
     * 并在剩余区域填充黑色像素。这样避免了图像拉伸变形，
     * 但需要在后处理时进行坐标反变换。
     *
     * 坐标变换公式：
     * - 模型坐标 -> 原始坐标：x_orig = (x_model - pad_x) * scale_x
     * - 原始坐标 -> 模型坐标：x_model = x_orig / scale_x + pad_x
     */
    struct PreprocContext {
        cv::Mat preprocessed; ///< 预处理后的图像，尺寸为INPUT_W × INPUT_H，已归一化到[0,1]
        float scale_x = 1.0f; ///< X方向缩放因子的倒数（用于坐标反变换）
        float scale_y = 1.0f; ///< Y方向缩放因子的倒数（用于坐标反变换）
        float pad_x   = 0.0f; ///< X方向填充偏移量（像素），letterbox左侧填充
        float pad_y   = 0.0f; ///< Y方向填充偏移量（像素），letterbox顶部填充
        int orig_w    = 0;    ///< 原始图像宽度（像素）
        int orig_h    = 0;    ///< 原始图像高度（像素）
    };

    // ========================================================================
    // 内部实现方法
    // ========================================================================

    /**
     * @brief 图像预处理：实现letterbox缩放与归一化
     *
     * **Letterbox算法流程：**
     * 1. 计算缩放比例r = min(INPUT_W/orig_w, INPUT_H/orig_h)
     * 2. 按比例r缩放原始图像，得到unpad_w × unpad_h图像
     * 3. 计算需要的填充量：pad_w = INPUT_W - unpad_w, pad_h = INPUT_H - unpad_h
     * 4. 在图像四周填充黑色像素（左、右、上、下）
     * 5. 像素值归一化到[0,1]区间，除以255.0
     *
     * @param image 原始输入图像，任意尺寸
     * @param cfg 配置参数（当前未使用）
     * @return 预处理上下文，包含变换后图像和坐标变换参数
     *
     * @note 输出图像通道顺序保持BGR，维度为[INPUT_H, INPUT_W, 3]
     */
    [[nodiscard]] PreprocContext
        preprocess(const cv::Mat& image, const ArmorOrtConfig& cfg) const noexcept;

    /**
     * @brief 执行ONNX模型推理
     *
     * **推理流程：**
     * 1. 将预处理后的图像从HWC格式转换为NCHW格式
     * 2. 创建ONNXRuntime输入张量
     * 3. 调用session->Run()执行推理
     * 4. 获取输出张量
     *
     * @param ctx 预处理上下文
     * @return 推理成功返回true，失败返回false
     *
     * @note 该方法会分配input_buffer_，如果之前有数据会被覆盖
     * @note 输出结果存储在impl_->output_tensors中
     */
    [[nodiscard]] bool infer(const PreprocContext& ctx) noexcept;

    /**
     * @brief 后处理：解析模型输出并提取装甲板检测结果
     *
     * **输出张量格式：**
     * - 形状：[1, max_det, 14]
     * - 每行14个值：[x, y, w, h, conf, cls_id, kpt0_x, kpt0_y, kpt1_x, kpt1_y, kpt2_x, kpt2_y,
     * kpt3_x, kpt3_y]
     *   - x, y, w, h: 边界框中心坐标和尺寸
     *   - conf: 置信度
     *   - cls_id: 类别ID（0-63）
     *   - kpt*: 4个角点坐标
     *
     * **处理流程：**
     * 1. 遍历所有检测结果
     * 2. 过滤低置信度检测（< confidence_threshold）
     * 3. 解析类别信息（颜色+编号）
     * 4. 提取4个角点坐标
     * 5. 将坐标从模型空间变换回原始图像空间
     * 6. 创建ArmorDetection对象
     *
     * @param ctx 预处理上下文，用于坐标变换
     * @param cfg 配置参数，包含置信度阈值
     * @return 装甲板检测结果列表
     */
    [[nodiscard]] std::vector<ArmorDetection>
        postprocess(const PreprocContext& ctx, const ArmorOrtConfig& cfg) const noexcept;

    /**
     * @brief 坐标变换：将检测坐标从模型空间映射回原始图像空间
     *
     * 该方法当前未使用，坐标变换已在postprocess中完成。
     * 保留该方法是为了保持接口兼容性和未来扩展。
     *
     * @param detections 检测结果列表（输入输出参数）
     * @param ctx 预处理上下文
     */
    void transform_coordinates(
        std::vector<ArmorDetection>& detections, const PreprocContext& ctx) const noexcept;

    /**
     * @brief 非极大值抑制（NMS）：去除重叠的检测框
     *
     * **NMS算法流程：**
     * 1. 按置信度降序排序所有检测框
     * 2. 从最高置信度开始，依次选择检测框
     * 3. 抑制所有与其IoU > nms_threshold的检测框
     * 4. 重复步骤2-3，直到所有检测框处理完毕
     * 5. 返回top_k个最高置信度的检测框
     *
     * @param detections 检测结果列表（会被修改）
     * @param cfg 配置参数，包含NMS阈值和top_k
     * @return NMS后的检测结果列表
     *
     * @note NMS阈值通常设为0.45，top_k通常设为100-200
     */
    [[nodiscard]] std::vector<ArmorDetection>
        nms(std::vector<ArmorDetection>& detections, const ArmorOrtConfig& cfg) const noexcept;

private:
    // ========================================================================
    // 成员变量
    // ========================================================================

    Config config_; ///< 配置参数，包含模型路径、阈值等

    /**
     * @brief PIMPL实现：隐藏ONNXRuntime的具体实现
     *
     * 使用PIMPL（Pointer to Implementation）模式的原因：
     * 1. **减少头文件依赖**：ONNXRuntime的头文件非常庞大，包含它会显著增加编译时间
     * 2. **稳定的ABI**：OrtBackend的头文件接口稳定，实现细节变化不影响调用方
     * 3. **减少编译时间**：修改impl实现不需要重新编译使用ort.hpp的代码
     *
     * Impl结构体包含：
     * - Ort::Env: ONNXRuntime环境对象
     * - Ort::SessionOptions: 会话配置选项
     * - Ort::Session: 推理会话对象
     * - 输入输出名称和缓冲区
     */
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace fcs::L2
