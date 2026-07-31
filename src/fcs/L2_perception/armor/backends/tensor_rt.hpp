#pragma once

/**
 * @file tensor_rt.hpp
 * @brief TensorRT装甲板检测后端头文件
 *
 * 本文件定义了基于NVIDIA TensorRT的高性能装甲板检测后端。相比ONNX Runtime后端，
 * TensorRT后端通过以下方式实现显著的性能提升：
 *
 * 1. **硬件加速**：充分利用NVIDIA GPU的Tensor Core、CUDA Core进行推理加速
 * 2. **层融合优化**：自动融合网络层，减少内存访问和kernel launch开销
 * 3. **精度优化**：支持FP16/INT8量化，在保持精度的同时大幅提升吞吐量
 * 4. **显存优化**：优化内存分配策略，减少显存占用和拷贝开销
 * 5. **CUDA预处理**：在GPU端完成letterbox预处理，避免CPU-GPU数据传输瓶颈
 *
 * ## 核心流程
 *
 * 1. **引擎加载/构建**：从.plan文件加载序列化的引擎，或从ONNX模型构建引擎
 * 2. **CUDA预处理**：使用CUDA kernel进行letterbox变换、归一化
 * 3. **异步推理**：使用CUDA stream实现异步推理，提高吞吐量
 * 4. **后处理**：在CPU端完成NMS、坐标映射、关键点提取
 *
 * ## 64类分类体系
 *
 * 本后端采用64类分类体系，编码空间为：4颜色 × 2尺寸 × 8数字
 *
 * - **颜色（4种）**：Red(红), Blue(蓝), Neutral(灰/无), Purple(紫)
 * - **尺寸（2种）**：Small（小装甲板）, Large（大装甲板）
 * - **数字（8种）**：Sentry(哨兵), 1-5(英雄/工程/步兵), Outpost(前哨站), Base(基地)
 *
 * 类别索引编码规则：
 * ```
 * class_id = color_id * 16 + size_id * 8 + number_id
 * 其中：
 *   color_id: 0=Red, 1=Blue, 2=Neutral, 3=Purple
 *   size_id:  0=Small, 1=Large
 *   number_id: 0=Sentry, 1-5=数字, 6=Outpost, 7=Base
 * ```
 *
 * ## 与ONNX Runtime的区别
 *
 * | 特性                | ONNX Runtime         | TensorRT                |
 * |---------------------|----------------------|-------------------------|
 * | 推理性能            | 较慢（CPU/GPU通用）  | 快（GPU专用优化）        |
 * | 精度支持            | FP32/FP16           | FP32/FP16/INT8         |
 * | 预处理方式          | CPU端OpenCV          | GPU端CUDA kernel        |
 * | 内存管理            | 通用内存分配         | 显存池优化              |
 * | 跨平台性            | 跨平台（CPU/GPU）    | 仅支持NVIDIA GPU        |
 * | 部署复杂度          | 简单（直接加载ONNX） | 复杂（需要引擎构建）    |
 *
 * @author Talos Team
 * @date 2024
 */

#include "../config.hpp"
#include "base.hpp"
#include "core/armor_types.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

namespace cv {
class Mat;
}

namespace fcs::L2 {

/**
 * @brief TensorRT装甲板检测后端
 *
 * 使用NVIDIA TensorRT推理引擎进行高性能装甲板检测。通过CRTP模式继承自
 * DetectorBackendBase，实现静态多态，避免虚函数开销。
 *
 * ## 设计模式
 *
 * - **PIMPL模式**：使用Impl结构体隐藏TensorRT实现细节，减少头文件依赖
 * - **RAII管理**：所有TensorRT/CUDA资源均通过RAII自动管理生命周期
 * - **工厂方法**：通过create()工厂方法构造，保证对象要么完全初始化，要么不存在
 *
 * ## 线程安全
 *
 * - 多个ExecutionContext支持并发推理
 * - 使用原子变量实现round-robin调度
 * - 每个stream独立，避免同步开销
 *
 * ## 性能优化策略
 *
 * 1. **多stream并发**：创建多个CUDA stream和ExecutionContext，充分利用GPU
 * 2. **异步拷贝**：使用cudaMemcpyAsync实现Host-Device异步数据传输
 * 3. **CUDA预处理**：在GPU端完成预处理，减少数据往返
 * 4. **Pin memory**：使用页锁定内存加速Host-Device传输
 */
class TrtBackend : public DetectorBackendBase<TrtBackend> {
public:
    // =========================================================================
    // 模型输入输出维度常量
    // =========================================================================

    /// 输入图像宽度（像素）
    static constexpr int INPUT_W     = 640;

    /// 输入图像高度（像素）
    static constexpr int INPUT_H     = 640;

    /// 颜色类别数量：Red, Blue, Neutral, Purple
    static constexpr int NUM_COLORS  = 4;

    /// 尺寸类别数量：Small, Large
    static constexpr int NUM_SIZES   = 2;

    /// 总类别数：4颜色 × 2尺寸 × 8数字 = 64类
    static constexpr int NUM_CLASSES = 64;

    /// 每个装甲板的关键点数量（4个角点）
    static constexpr int NUM_KPTS    = 4;

    /// 输出维度：4(bbox) + 1(conf) + 1(class) + 8(4个关键点×2坐标) = 14
    /// 输出格式：[x_center, y_center, w, h, confidence, class_id,
    ///           kpt0_x, kpt0_y, kpt1_x, kpt1_y, kpt2_x, kpt2_y, kpt3_x, kpt3_y]
    static constexpr int OUTPUT_DIM  = 14;

    // =========================================================================
    // 类型定义
    // =========================================================================

    /// 检测结果类型：成功返回检测结果列表，失败返回错误信息
    using DetectionResult = std::expected<std::vector<fcs::ArmorDetection>, std::string>;

    /// 配置类型
    using Config          = ArmorTensorRtConfig;

    /**
     * @brief 预处理上下文结构体
     *
     * 存储letterbox预处理所需的参数，用于将模型输出坐标映射回原图坐标。
     * 这些参数在预处理时计算，在后处理时用于坐标逆变换。
     *
     * ## Letterbox预处理原理
     *
     * 为了保持图像长宽比，将原始图像缩放并居中放置在目标尺寸画布上：
     *
     * 1. 计算缩放因子：scale = min(640/orig_w, 640/orig_h)
     * 2. 计算缩放后尺寸：new_w = orig_w * scale, new_h = orig_h * scale
     * 3. 计算padding：pad_x = (640 - new_w) / 2, pad_y = (640 - new_h) / 2
     * 4. 将缩放后的图像放置在画布中心，周围用(114,114,114)填充
     *
     * ## 坐标逆变换公式
     *
     * 原图坐标 = (模型输出坐标 - padding) / scale
     */
    struct PreprocContext {
        float scale_x;      ///< X方向缩放因子（通常等于scale_y）
        float scale_y;      ///< Y方向缩放因子
        int pad_x;          ///< X方向padding像素数
        int pad_y;          ///< Y方向padding像素数
        int orig_width;     ///< 原始图像宽度
        int orig_height;    ///< 原始图像高度
    };

    // =========================================================================
    // 构造与初始化
    // =========================================================================

    /**
     * @brief 工厂方法：构造完全初始化的后端实例
     *
     * **构造即初始化**：对象构造完成后立即可用，无需调用额外的init()方法。
     * 如果初始化失败，返回错误信息而不是抛出异常。
     *
     * ## 初始化流程
     *
     * 1. 设置CUDA设备（根据config.device_id）
     * 2. 创建TensorRT Runtime
     * 3. 加载或构建Engine：
     *    - 如果指定了engine_path且文件存在，直接加载序列化的引擎
     *    - 否则从ONNX模型构建引擎并缓存到engine_cache_dir
     * 4. 分配GPU内存（输入/输出缓冲区）
     * 5. 创建多个ExecutionContext和CUDA Stream（支持并发推理）
     *
     * ## 错误处理
     *
     * 所有可能失败的系统调用都返回std::expected，错误信息包含：
     * - 失败的具体操作（如"load engine", "allocate memory"）
     * - 系统错误信息（如cudaGetErrorString）
     * - 相关上下文（如文件路径、设备ID）
     *
     * @param config 配置参数，包含模型路径、设备ID、推理参数等
     * @return 成功返回初始化完成的TrtBackend实例，失败返回错误信息
     */
    [[nodiscard]] static std::expected<TrtBackend, std::string> create(Config config) noexcept;

    /// 析构函数：释放所有TensorRT和CUDA资源
    ~TrtBackend();

    /// 移动构造函数：转移资源所有权
    TrtBackend(TrtBackend&&) noexcept;

    /// 移动赋值运算符：转移资源所有权
    TrtBackend& operator=(TrtBackend&&) noexcept;

    /// 禁止拷贝构造（TensorRT资源不可拷贝）
    TrtBackend(const TrtBackend&)            = delete;

    /// 禁止拷贝赋值（TensorRT资源不可拷贝）
    TrtBackend& operator=(const TrtBackend&) = delete;

    // =========================================================================
    // 推理接口
    // =========================================================================

    /**
     * @brief 执行装甲板检测
     *
     * 这是CRTP基类调用的实际实现方法，完成完整的检测流程：
     *
     * 1. **预处理阶段**：
     *    - 计算letterbox参数（scale, padding）
     *    - 在GPU端执行CUDA kernel：resize + padding + BGR2RGB + 归一化
     *    - 将预处理结果写入GPU输入缓冲区
     *
     * 2. **推理阶段**：
     *    - 设置输入输出张量地址
     *    - 异步执行推理（enqueueV3）
     *    - 异步拷贝输出到Host内存
     *    - 同步等待推理完成
     *
     * 3. **后处理阶段**：
     *    - 解析输出张量：提取bbox、confidence、class、keypoints
     *    - 过滤低置信度检测
     *    - 将模型坐标映射回原图坐标
     *    - 应用NMS去除重复检测
     *    - 限制最大检测数量（top_k）
     *
     * @param image 输入图像（BGR格式，任意尺寸）
     * @param color 目标颜色（用于过滤，当前版本已禁用颜色过滤）
     * @return 成功返回检测结果列表，失败返回错误信息
     */
    DetectionResult detect_impl(const cv::Mat& image, ArmorColor color) noexcept;

private:
    /**
     * @brief 私有构造函数：仅初始化配置，不加载模型
     *
     * 真正的初始化在create()工厂方法中完成。私有构造函数防止直接构造未完全初始化的对象。
     *
     * @param config 配置参数
     */
    explicit TrtBackend(Config config) noexcept;

    /**
     * @brief PIMPL实现结构体
     *
     * 隐藏TensorRT和CUDA的实现细节，减少头文件依赖：
     * - nvinfer1::IRuntime
     * - nvinfer1::ICudaEngine
     * - nvinfer1::IExecutionContext
     * - cudaStream_t
     * - 设备内存指针
     *
     * 使用PIMPL的优势：
     * 1. 减少头文件编译依赖，加快编译速度
     * 2. 隐藏TensorRT API，避免API变更影响用户代码
     * 3. 减少头文件体积，改善编译缓存效率
     */
    struct Impl;
    std::unique_ptr<Impl> impl_;

    /// 配置参数副本
    Config config_;
};

} // namespace fcs::L2
