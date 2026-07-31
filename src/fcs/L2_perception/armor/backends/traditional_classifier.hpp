/**
 * @file traditional_classifier.hpp
 * @brief 传统装甲板数字分类器（基于 OpenCV DNN + ONNX 模型）
 *
 * 本文件实现了装甲板数字识别的 ONNX 分类器，用于识别装甲板上的数字：
 * - 数字编号：1, 2, 3, 4, 5
 * - 特殊目标：前哨站 (Outpost), 基地 (Base)
 *
 * ## 分类器架构
 *
 * 分类器基于 OpenCV DNN 模块，支持加载 ONNX 格式的模型：
 * - 典型架构：LeNet、MLP 等轻量级卷积神经网络
 * - 输入尺寸：28x28 灰度图像（与 extract_number 输出匹配）
 * - 输出：8 类别（One, Two, Three, Four, Five, Outpost, Sentry, Base）
 *
 * ## 推理流程
 *
 * 1. **预处理** (classify 方法内)
 *    - 输入：20x28 或 28x28 灰度图像（OTSU 二值化后）
 *    - 归一化：pixel_value / 255.0（将 [0, 255] 映射到 [0, 1]）
 *    - Blob 构造：cv::dnn::blobFromImage() 转换为网络输入格式
 *
 * 2. **网络推理**
 *    - OpenCV DNN 后端：DNN_BACKEND_DEFAULT
 *    - 目标设备：DNN_TARGET_CPU（纯 CPU 推理，无需 GPU）
 *    - 前向传播：net_.forward()
 *
 * 3. **后处理** (post_process_output 方法内)
 *    - 输出形状：[1, 8] 或 [1, 1, 8]
 *    - Softmax（可选）：将 logits 转换为概率分布
 *    - 最大值索引：找到置信度最高的类别
 *    - 索引映射：将索引转换为 ArmorName 枚举
 *
 * ## Softmax 处理说明
 *
 * 传统网络（如 LeNet）通常输出 logits（未归一化的分数），需要手动应用 softmax：
 * ```
 * softmax(x) = exp(x - max(x)) / Σ exp(x - max(x))
 * ```
 *
 * 减去 max(x) 是为了数值稳定性，防止 exp 溢出。
 *
 * 新网络可能已经包含 softmax 层，直接输出概率，此时 use_softmax = false。
 *
 * ## 与深度学习检测器的区别
 *
 * | 特性                | 传统分类器                | 深度学习检测器            |
 * |--------------------|-----------------------|---------------------|
 * | 输入                | 提取的数字区域（28x28）     | 原始图像（如 640x480）    |
 * | 任务                | 分类（8 类别）           | 检测 + 分类（端到端）      |
 * | 模型大小             | 小（几百 KB）           | 大（几十 MB）           |
 * | 推理速度             | 快（毫秒级）             | 慢（几十毫秒）           |
 * | 准确率              | 中等（依赖数字提取质量）      | 高（端到端优化）          |
 * | 适用场景             | 传统检测流程的最后一步       | 现代端到端检测           |
 *
 * ## 设计要点
 *
 * - **RAII 生命周期**：通过 `create()` 工厂函数构造，确保模型加载失败可恢复
 * - **无异常保证**：所有公共接口返回 `std::expected<T, std::string>`
 * - **只移语义**：禁用拷贝，避免共享网络模型导致的线程安全问题
 * - **延迟推理**：只在检测到有效装甲板后才调用分类器
 *
 * @author Talos Team
 * @date 2024
 */

#pragma once

#include "../backend.hpp"
#include "core/types.hpp"

#include <expected>
#include <string_view>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

namespace fcs::L2 {

// ============================================================================
// AT Legacy Traditional ONNX Classifier
// ============================================================================

/**
 * @class TraditionalClassifier
 * @brief 装甲板数字分类器（基于 OpenCV DNN + ONNX 模型）
 *
 * 该类封装了 ONNX 模型的加载和推理过程，用于识别装甲板上的数字。
 * 支持 LeNet、MLP 等轻量级网络，输入为 28x28 灰度图像。
 *
 * **使用示例：**
 * ```cpp
 * auto classifier = TraditionalClassifier::create("model.onnx");
 * if (classifier) {
 *     float confidence = 0;
 *     auto result = classifier->classify(number_img, confidence);
 *     if (result) {
 *         ArmorName name = *result;
 *         // 使用分类结果...
 *     }
 * }
 * ```
 *
 * **线程安全性：**
 * - OpenCV DNN 的推理操作本身是线程安全的
 * - 但同一个 cv::dnn::Net 对象不应在多个线程同时使用
 * - 建议每个线程拥有独立的 TraditionalClassifier 实例
 */
class TraditionalClassifier {
public:
    /// 分类结果类型：成功返回装甲板名称，失败返回错误信息
    using ClassifyResult = std::expected<ArmorName, std::string>;

    /**
     * @brief 工厂函数：加载 ONNX 模型并构造分类器
     *
     * 采用 "Construction IS Initialization" 模式，所有初始化工作在此完成：
     * - 加载 ONNX 模型文件
     * - 配置推理后端（DNN_BACKEND_DEFAULT）
     * - 配置目标设备（DNN_TARGET_CPU）
     *
     * @param model_path ONNX 模型文件路径（相对路径或绝对路径）
     * @param use_softmax 是否对输出应用 softmax（默认 true）
     *                    - true：网络输出 logits，需要手动 softmax
     *                    - false：网络已经包含 softmax，直接输出概率
     * @return 成功返回初始化后的 TraditionalClassifier，失败返回错误信息
     *
     * @note 该函数保证 noexcept，失败通过 std::expected 返回
     * @note 无需调用额外的 init() 方法
     * @note 模型加载失败会记录 SPDLOG_ERROR 日志
     */
    [[nodiscard]] static std::expected<TraditionalClassifier, std::string>
        create(std::string_view model_path, bool use_softmax = true) noexcept;

    // 移动构造与赋值（允许资源转移）
    // cv::dnn::Net 支持移动语义，可以安全转移
    TraditionalClassifier(TraditionalClassifier&&) noexcept            = default;
    TraditionalClassifier& operator=(TraditionalClassifier&&) noexcept = default;

    // 禁止拷贝（避免共享网络模型）
    // OpenCV DNN 的 Net 对象不应在多个实例间共享
    TraditionalClassifier(const TraditionalClassifier&)                = delete;
    TraditionalClassifier& operator=(const TraditionalClassifier&)     = delete;

    /**
     * @brief 对单张数字图像进行分类
     *
     * **预处理流程：**
     * 1. 归一化：pixel_value / 255.0（将 [0, 255] 映射到 [0, 1]）
     *    - 与传统网络的训练预处理保持一致
     *    - 匹配 atvision_ws 的实现
     * 2. Blob 构造：cv::dnn::blobFromImage()
     *    - 将 HWC 格式转换为 NCHW 格式（Network, Channels, Height, Width）
     *    - 无需额外缩放（输入已经是 28x28）
     *
     * **推理流程：**
     * 1. 设置网络输入：net_.setInput(blob)
     * 2. 前向传播：output = net_.forward()
     * 3. 后处理：调用 post_process_output() 提取类别和置信度
     *
     * @param number_img 输入图像（28x28 灰度图，OTSU 二值化后）
     * @param confidence_out 输出参数：预测置信度（0.0 ~ 1.0）
     * @return 成功返回装甲板名称（ArmorName），失败返回错误信息
     *
     * @note 输入图像应为预处理后的二值化图像，而非原始 BGR 图像
     * @note 该函数保证 noexcept，异常通过 std::expected 返回
     * @note 推理失败会记录 SPDLOG_ERROR 日志
     */
    [[nodiscard]] ClassifyResult
        classify(const cv::Mat& number_img, float& confidence_out) const noexcept;

private:
    /**
     * @brief 私有构造函数（通过 create() 工厂函数调用）
     *
     * @param model_path 模型文件路径（用于日志和调试）
     * @param use_softmax 是否应用 softmax
     * @param net OpenCV DNN 网络对象（已加载模型）
     */
    TraditionalClassifier(std::string model_path, bool use_softmax, cv::dnn::Net net) noexcept
        : model_path_(std::move(model_path))
        , net_(std::move(net))
        , use_softmax_(use_softmax) {}

    /**
     * @brief 后处理网络输出
     *
     * **处理流程：**
     * 1. 展平输出：将 [1, 8] 或 [1, 1, 8] 展平为 [1, 8]
     * 2. Softmax（可选）：将 logits 转换为概率分布
     *    ```
     *    softmax(x) = exp(x - max(x)) / Σ exp(x - max(x))
     *    ```
     *    - 减去 max(x) 防止数值溢出
     * 3. 找最大值：使用 cv::minMaxLoc() 找到最大概率及其索引
     * 4. 索引映射：将索引转换为 ArmorName 枚举
     *
     * @param output 网络原始输出（logits 或概率）
     * @param confidence_out 输出参数：预测置信度
     * @return 装甲板名称
     */
    [[nodiscard]] ArmorName
        post_process_output(const cv::Mat& output, float& confidence_out) const noexcept;

    /**
     * @brief 将网络输出索引映射到 ArmorName 枚举
     *
     * 索引与类别的对应关系（与训练数据标签一致）：
     * - 0: One（数字 1）
     * - 1: Two（数字 2）
     * - 2: Three（数字 3）
     * - 3: Four（数字 4）
     * - 4: Five（数字 5）
     * - 5: Outpost（前哨站）
     * - 6: Sentry（哨兵）
     * - 7: Base（基地）
     * - 其他: Invalid（无效）
     *
     * @param idx 网络输出索引（0-7）
     * @return 对应的 ArmorName 枚举值
     */
    [[nodiscard]] static ArmorName index_to_armor_name(int idx) noexcept;

    /// 模型文件路径（用于日志和调试）
    std::string model_path_;

    /// OpenCV DNN 网络对象
    /// mutable 因为 OpenCV DNN 的推理操作是非 const 的
    mutable cv::dnn::Net net_;

    /// 是否对输出应用 softmax
    /// - true：传统网络输出 logits，需要手动 softmax
    /// - false：新网络已经包含 softmax 层
    bool use_softmax_ = true;
};

} // namespace fcs::L2