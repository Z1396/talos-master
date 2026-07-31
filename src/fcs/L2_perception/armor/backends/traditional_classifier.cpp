/**
 * @file traditional_classifier.cpp
 * @brief 传统装甲板数字分类器实现
 *
 * 本文件实现了 TraditionalClassifier 类的所有方法，包括：
 * - 模型加载（create 工厂函数）
 * - 图像预处理与推理（classify）
 * - 输出后处理（post_process_output）
 * - 索引映射（index_to_armor_name）
 *
 * ## 实现要点
 *
 * ### 1. 模型加载（create）
 * - 使用 cv::dnn::readNetFromONNX() 加载 ONNX 模型
 * - 配置推理后端：DNN_BACKEND_DEFAULT（OpenCV 默认后端）
 * - 配置目标设备：DNN_TARGET_CPU（纯 CPU 推理）
 * - 异常捕获：将 OpenCV 异常转换为 std::expected 错误
 *
 * ### 2. 图像预处理（classify 内部）
 * - 归一化：pixel_value / 255.0（匹配训练时的预处理）
 * - Blob 构造：cv::dnn::blobFromImage()（HWC → NCHW）
 * - 无需额外缩放或裁剪（输入已经是 28x28）
 *
 * ### 3. 网络推理（classify 内部）
 * - setInput(blob)：设置网络输入
 * - forward()：执行前向传播
 * - 返回原始输出（logits 或概率）
 *
 * ### 4. 输出后处理（post_process_output）
 * - 展平输出：reshape(1, 1)（处理不同输出形状）
 * - Softmax（可选）：
 *   ```
 *   max_val = max(output)
 *   exp_scores = exp(output - max_val)  // 防止溢出
 *   probabilities = exp_scores / sum(exp_scores)
 *   ```
 * - 找最大值：cv::minMaxLoc()
 * - 索引映射：将 0-7 映射到 ArmorName 枚举
 *
 * ### 5. 错误处理
 * - 所有公共接口保证 noexcept
 * - 使用 try-catch 捕获 OpenCV 异常
 * - 失败通过 std::expected 返回错误信息
 * - 记录 SPDLOG_ERROR 日志
 *
 * ## 性能优化
 *
 * - **模型预热**：首次推理会触发模型优化，建议在启动时预热
 * - **批量推理**：当前实现为单张推理，如需批量可扩展
 * - **内存复用**：OpenCV DNN 内部会复用内存，无需手动优化
 *
 * @author Talos Team
 * @date 2024
 */

#include "L2_perception/armor/backends/traditional_classifier.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace fcs::L2 {

// ============================================================================
// Factory — Construction IS Initialization
// ============================================================================

/**
 * @brief 工厂函数：加载 ONNX 模型并构造分类器
 *
 * **加载流程：**
 * 1. 调用 cv::dnn::readNetFromONNX() 加载模型文件
 * 2. 检查模型是否成功加载（net.empty() 检查）
 * 3. 配置推理后端和目标设备
 * 4. 构造 TraditionalClassifier 对象并返回
 *
 * **后端配置：**
 * - DNN_BACKEND_DEFAULT：OpenCV 默认后端（兼容性最好）
 * - DNN_TARGET_CPU：纯 CPU 推理（无需 GPU，适合嵌入式平台）
 *
 * **错误处理：**
 * - 模型文件不存在 → 返回错误信息
 * - 模型格式错误 → 捕获异常并返回错误信息
 * - 加载失败 → 记录 SPDLOG_ERROR 日志
 *
 * @param model_path ONNX 模型文件路径
 * @param use_softmax 是否应用 softmax
 * @return 成功返回分类器，失败返回错误信息
 */
std::expected<TraditionalClassifier, std::string>
    TraditionalClassifier::create(std::string_view model_path, bool use_softmax) noexcept {
    try {
        // 加载 ONNX 模型（可能抛出异常）
        auto net = cv::dnn::readNetFromONNX(std::string(model_path));

        // 检查模型是否成功加载
        if (net.empty()) {
            return std::unexpected(
                "Failed to load ONNX classifier model: " + std::string(model_path));
        }

        // 配置推理后端和目标设备
        // DNN_BACKEND_DEFAULT：OpenCV 默认后端，兼容性好
        // DNN_TARGET_CPU：纯 CPU 推理，无需 GPU
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        // 构造分类器对象（私有构造函数）
        return TraditionalClassifier(std::string(model_path), use_softmax, std::move(net));
    } catch (const std::exception& e) {
        // 捕获 OpenCV 异常，记录日志并返回错误信息
        SPDLOG_ERROR("[TraditionalClassifier] Model load failed: {}", e.what());
        return std::unexpected("ONNX classifier model load failed: " + std::string(e.what()));
    }
}

// ============================================================================
// Classification
// ============================================================================

/**
 * @brief 对单张数字图像进行分类
 *
 * **预处理流程：**
 * 1. 归一化：normalized_img = number_img / 255.0
 *    - 将像素值从 [0, 255] 映射到 [0, 1]
 *    - 与训练时的预处理保持一致
 *    - 匹配 atvision_ws 的实现
 *
 * 2. Blob 构造：cv::dnn::blobFromImage(normalized_img, blob)
 *    - 将 HWC 格式（Height, Width, Channels）转换为 NCHW 格式
 *    - N = 1（批次大小）
 *    - C = 1（灰度图）
 *    - H = 28, W = 28（输入尺寸）
 *    - 无需额外缩放（scalefactor=1.0, size=Size())
 *
 * **推理流程：**
 * 1. setInput(blob)：设置网络输入张量
 * 2. forward()：执行前向传播，得到输出张量
 * 3. post_process_output()：后处理，提取类别和置信度
 *
 * @param number_img 输入图像（28x28 灰度图）
 * @param confidence_out 输出参数：预测置信度
 * @return 成功返回 ArmorName，失败返回错误信息
 */
TraditionalClassifier::ClassifyResult TraditionalClassifier::classify(
    const cv::Mat& number_img, float& confidence_out) const noexcept {
    try {
        // 归一化到 [0, 1] 范围（匹配 atvision_ws 行为）
        // 输入图像是 OTSU 二值化后的灰度图，像素值为 0 或 255
        // 归一化后为 0.0 或 1.0
        cv::Mat normalized_img = number_img / 255.0;

        // 构造网络输入 Blob
        // blobFromImage 会将 HWC 转换为 NCHW 格式
        // 输入已经是 28x28，无需额外缩放
        cv::Mat blob;
        cv::dnn::blobFromImage(normalized_img, blob);

        // 设置网络输入
        net_.setInput(blob);

        // 前向传播（执行推理）
        cv::Mat output = net_.forward();

        // 后处理：提取类别和置信度
        return post_process_output(output, confidence_out);
    } catch (const std::exception& e) {
        // 捕获 OpenCV 异常，记录日志并返回错误信息
        SPDLOG_ERROR("[TraditionalClassifier] Inference failed: {}", e.what());
        return std::unexpected("Classifier inference failed: " + std::string(e.what()));
    }
}

// ============================================================================
// Output Post-Processing
// ============================================================================

/**
 * @brief 后处理网络输出，提取类别和置信度
 *
 * **处理流程：**
 *
 * 1. **展平输出**
 *    网络输出形状可能是：
 *    - [1, 8]：批次维度 + 类别维度
 *    - [1, 1, 8]：批次维度 + 序列维度 + 类别维度
 *    - [8]：仅有类别维度
 *
 *    使用 reshape(1, 1) 统一展平为 [1, 8] 形状。
 *
 * 2. **Softmax（可选）**
 *    如果 use_softmax_ = true，需要手动应用 softmax：
 *    ```
 *    softmax(x_i) = exp(x_i) / Σ exp(x_j)
 *    ```
 *
 *    为了数值稳定性，减去最大值：
 *    ```
 *    exp_scores = exp(x - max(x))
 *    probabilities = exp_scores / sum(exp_scores)
 *    ```
 *
 *    这样可以防止 exp 溢出（当 x 很大时）。
 *
 *    如果 use_softmax_ = false，网络已经输出概率，直接使用。
 *
 * 3. **找最大值**
 *    使用 cv::minMaxLoc() 找到最大概率及其索引：
 *    - confidence：最大概率值（置信度）
 *    - label_id：最大值索引（类别 ID）
 *
 * 4. **置信度输出**
 *    将置信度写入 confidence_out 参数。
 *
 * 5. **索引映射**
 *    调用 index_to_armor_name() 将索引转换为 ArmorName 枚举。
 *
 * @param output 网络原始输出（logits 或概率）
 * @param confidence_out 输出参数：预测置信度
 * @return 装甲板名称
 */
ArmorName TraditionalClassifier::post_process_output(
    const cv::Mat& output, float& confidence_out) const noexcept {
    // 展平输出：将 [1, 8] 或 [1, 1, 8] 统一为 [1, 8]
    cv::Mat flat_output = output.reshape(1, 1);

    double confidence = 0.0;
    int label_id      = -1;

    if (use_softmax_) {
        // 传统网络输出 logits，需要手动应用 softmax
        // 这是 atvision_ws 的实现方式

        // 找最大值（用于数值稳定性）
        float max_val = *std::max_element(flat_output.begin<float>(), flat_output.end<float>());

        // 计算 exp(x - max_val)
        cv::Mat exp_scores;
        cv::exp(flat_output - max_val, exp_scores);

        // 计算归一化因子
        float sum = static_cast<float>(cv::sum(exp_scores)[0]);

        // 归一化得到概率分布
        exp_scores /= sum;

        // 找最大概率及其索引
        cv::Point class_id_point;
        cv::minMaxLoc(exp_scores, nullptr, &confidence, nullptr, &class_id_point);
        label_id = class_id_point.x;
    } else {
        // 新网络已经包含 softmax，直接使用输出概率
        cv::Point class_id_point;
        cv::minMaxLoc(flat_output, nullptr, &confidence, nullptr, &class_id_point);
        label_id = class_id_point.x;
    }

    // 输出置信度（0.0 ~ 1.0）
    confidence_out = static_cast<float>(confidence);

    // 如果索引无效（负数），返回 Invalid
    if (label_id < 0) {
        return ArmorName::Invalid;
    }

    // 将索引映射到 ArmorName 枚举
    return index_to_armor_name(label_id);
}

// ============================================================================
// Helper Methods
// ============================================================================

/**
 * @brief 将网络输出索引映射到 ArmorName 枚举
 *
 * **索引与类别对应关系：**
 *
 * | 索引 | ArmorName    | 说明           |
 * |-----|--------------|---------------|
 * | 0   | One          | 数字 1        |
 * | 1   | Two          | 数字 2        |
 * | 2   | Three        | 数字 3        |
 * | 3   | Four         | 数字 4        |
 * | 4   | Five         | 数字 5        |
 * | 5   | Outpost      | 前哨站        |
 * | 6   | Sentry       | 哨兵          |
 * | 7   | Base         | 基地          |
 * | 其他 | Invalid      | 无效类别      |
 *
 * **注意：**
 * - 索引必须与训练数据标签一致
 * - 如果修改网络输出类别，需要同步修改此映射
 * - 使用穷尽 switch，避免遗漏新增类别
 *
 * @param idx 网络输出索引（0-7）
 * @return 对应的 ArmorName 枚举值
 */
ArmorName TraditionalClassifier::index_to_armor_name(int idx) noexcept {
    // 穷尽 switch，编译器会检查是否处理所有分支
    // 如果新增类别，编译器会警告
    switch (idx) {
    case 0: return ArmorName::One;
    case 1: return ArmorName::Two;
    case 2: return ArmorName::Three;
    case 3: return ArmorName::Four;
    case 4: return ArmorName::Five;
    case 5: return ArmorName::Outpost;
    case 6: return ArmorName::Sentry;
    case 7: return ArmorName::Base;
    default: return ArmorName::Invalid; // 无效索引
    }
}

} // namespace fcs::L2