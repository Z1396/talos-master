// 头文件保护，防止重复包含引发编译错误
#pragma once

// 引入装甲板ROI读取配置、装甲板基础类型定义
#include "L2_perception/armor/readback_roi.hpp"
#include "core/armor_types.hpp"

// 标准库
#include <string>   // 存储模型路径、缓存目录等字符串
#include <vector>   // 存储多组步长等数组类型参数

// L2感知层 - 装甲板检测模块命名空间
namespace fcs::L2 {

/**
 * @brief 装甲板检测后端类型枚举
 * 根据编译宏按需启用不同推理后端，适配CPU/GPU/NPU多种硬件平台
 */
enum class ArmorBackendType : int {
    Traditional,    // 传统图像处理算法（纯CPU，无深度学习）
    OnnxRuntime,    // ONNX Runtime 推理后端（跨平台CPU/GPU通用推理）
#if TALOS_HAS_TENSORRT
    TensorRT,       // NVIDIA TensorRT 后端（GPU硬件加速，英伟达显卡专用）
#endif
#if TALOS_HAS_AXERA
    Axera,          // 翱捷AX650 NPU后端（端侧嵌入式NPU硬件加速）
#endif
};

// ============================================================================
// 传统图像处理算法 配置结构体
// 基于形态学、二值化、轮廓匹配的经典装甲板检测方案，无深度学习依赖
// ============================================================================
struct ArmorTraditionalConfig {
    bool advanced_binary = false;               // 是否启用高级二值化算法
    float dark_percentage{0.5};                 // 暗区域占比阈值
    int binary_threshold{};                     // 图像二值化阈值

    // -------------------- 灯条检测参数 --------------------
    double light_min_ratio{};                   // 灯条最小宽高比
    double light_max_ratio{};                   // 灯条最大宽高比
    double light_max_angle{};                   // 灯条最大倾斜角度
    int light_color_diff_thresh{};              // 灯条颜色差值阈值

    // -------------------- 装甲板配对参数 --------------------
    double armor_min_light_ratio{};             // 装甲板灯条最小比例
    double armor_min_small_center_distance{};    // 小装甲板灯条中心最小间距
    double armor_max_small_center_distance{};    // 小装甲板灯条中心最大间距
    double armor_min_large_center_distance{};    // 大装甲板灯条中心最小间距
    double armor_max_large_center_distance{};    // 大装甲板灯条中心最大间距
    double armor_max_angle{};                   // 装甲板整体最大偏角

    // -------------------- 分类器参数（传统算法配套简单分类） --------------------
    std::string classifier_model_path{};        // 分类模型文件路径
    double classifier_confidence_threshold = 0.5; // 分类置信度阈值
    bool classifier_enable_type_filtering  = true; // 是否开启类型过滤
    bool classifier_use_softmax            = true;
    // 说明：老旧模型直接输出原始分值(logits)，需启用Softmax转为概率分布
};

// ============================================================================
// ONNX Runtime 推理后端配置
// 通用深度学习推理框架，跨平台、部署简单，主流CPU/集显场景使用
// ============================================================================
struct ArmorOrtConfig {
    std::string model_path{};       // ONNX模型文件路径
    std::string cache_dir{};        // 推理缓存目录（优化启动速度）
    double confidence_threshold{};  // 检测置信度阈值，低于该值过滤目标
    double nms_threshold{};         // 非极大值抑制(NMS)重叠阈值
    int top_k{};                    // 单帧最多保留目标数量
};

// ============================================================================
// TensorRT 推理后端配置
// NVIDIA官方GPU加速框架，针对英伟达显卡做算子优化，推理速度最快
// ============================================================================
struct ArmorTensorRtConfig {
    std::string engine_path{};      // TensorRT 序列化引擎文件路径
    std::string model_path{};       // 原始ONNX模型路径
    std::string engine_cache_dir{}; // 引擎文件缓存目录

    // 硬件设备 & 并发配置
    int device_id                  = 0;    // 指定使用的GPU卡号，默认0号卡
    int num_streams                = 2;    // 推理流数量，控制并发数
    std::string compute_capability = "86"; // GPU算力版本，对应不同世代英伟达显卡

    // TensorRT 编译选项
    bool enable_fp16 = true;        // 启用FP16半精度推理，提速、降显存
    bool enable_dla  = false;       // 是否启用DLA硬件加速器（嵌入式NVIDIA平台）
    int dla_core     = 0;            // DLA核心编号

    // 检测后处理参数
    double confidence_threshold = 0.75; // 置信度阈值
    double nms_threshold        = 0.30; // NMS重叠阈值
    int top_k                   = 128;  // 单帧最大输出目标数
};

// ============================================================================
// Axera 翱捷NPU后端配置
// 面向嵌入式端AX650芯片，使用硬件NPU做深度学习推理，无独立GPU场景专用
// ============================================================================
struct ArmorAxeraConfig {
    std::string model_path{}; // 适配AX NPU的模型文件路径

    // 模型输入尺寸
    int input_width  = 768;    // 网络输入图像宽度
    int input_height = 576;    // 网络输入图像高度

    // 模型输出维度参数
    int num_colors           = 4;         // 支持的装甲板颜色类别数
    int num_kpts             = 4;          // 单装甲板关键点数量（四角）
    int num_pairs            = 12;         // 配对组合数量
    std::vector<int> strides = {8, 16, 32};// 多尺度检测步长

    // 检测后处理参数
    double confidence_threshold = 0.25; // 置信度阈值
    double nms_threshold        = 0.70; // NMS重叠阈值
    int top_k                   = 128;  // 单帧最大输出目标数
};

// ============================================================================
// 装甲板检测器总配置
// 统一管理所有后端 + ROI区域配置，运行时根据 backend_type 选择对应参数生效
// ============================================================================
struct ArmorDetectorConfig {
    ArmorBackendType backend_type;  // 当前启用的推理/算法后端类型
    ArmorTraditionalConfig traditional;  // 传统算法参数
    ArmorOrtConfig onnx_runtime;         // ONNX Runtime 参数
    ArmorTensorRtConfig tensor_rt;       // TensorRT GPU加速参数
    ArmorAxeraConfig axera;              // Axera NPU加速参数
    ArmorReadbackRoiConfig readback_roi; // 局部感兴趣区域(ROI)读取配置
};

} // namespace fcs::L2