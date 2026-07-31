/**
 * @file ort.cpp
 * @brief ONNXRuntime神经网络推理后端的实现文件
 *
 * 本文件实现了OrtBackend类的所有成员函数，包括：
 * - 模型加载与执行提供器选择
 * - 图像预处理（letterbox算法）
 * - ONNX模型推理
 * - 后处理与NMS算法
 *
 * **跨平台执行提供器选择策略：**
 * 1. Linux x86_64平台：
 *    - 首选OpenVINO GPU_FP16（Intel Xe架构iGPU）
 *    - 失败后尝试OpenVINO GPU_FP32
 *    - 最后降级到CPUExecutionProvider
 * 2. macOS平台：
 *    - 使用CoreMLExecutionProvider（CPUAndNeuralEngine）
 * 3. 其他平台：
 *    - 使用CPUExecutionProvider
 *
 * **关键算法实现：**
 * - Letterbox预处理：保持长宽比的图像缩放
 * - NMS：基于IoU的非极大值抑制
 * - 坐标变换：模型空间到原始图像空间的映射
 *
 * @author Talos Team
 * @date 2024
 */

#include "L2_perception/armor/backends/ort.hpp"

#include "L2_perception/armor/config.hpp"
#include "L2_perception/common/geometry.hpp"
#include "core/types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <numeric>
#include <onnxruntime_c_api.h>
#include <spdlog/common.h>
#include <spdlog/spdlog-inl.h>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

// macOS平台特定头文件：CoreML执行提供器
#ifdef __APPLE__
# include <Availability.h>
# include <coreml_provider_factory.h>
#endif

#include <onnxruntime_cxx_api.h>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace fcs::L2 {

// ============================================================================
// 类别映射表（64个类别）
// ============================================================================
/**
 * @brief 装甲板类别映射表
 *
 * ONNX模型输出64个类别，编码规则：
 * - 64类 = 4种颜色 × 2种尺寸 × 8个编号
 * - 颜色编码：0=红色(Red), 1=蓝色(Blue), 2=灰色(Neutral), 3=紫色(Purple)
 * - 尺寸编码：类别ID 0-7=小装甲板，8-15=大装甲板（每8个编号循环）
 * - 编号编码：mod 8得到编号（0=哨兵, 1-5=数字, 6=前哨站, 7=基地）
 *
 * 映射表结构：
 * - g_class_mappings[i] = {颜色, 编号}
 * - i的编码：颜色*16 + 尺寸*8 + 编号
 *
 * 示例：
 * - 类别0: 红色小装甲板哨兵
 * - 类别8: 红色大装甲板哨兵
 * - 类别16: 蓝色小装甲板哨兵
 */

namespace {

/**
 * @struct ClassMapping
 * @brief 类别映射结构体
 *
 * 将模型输出的类别ID映射到装甲板的颜色和编号。
 */
struct ClassMapping {
    ArmorColor color;  ///< 装甲板颜色（红、蓝、灰、紫）
    ArmorName name;    ///< 装甲板编号（哨兵、1-5号、前哨站、基地）
};

#if defined(__linux__) && defined(__x86_64__)
/**
 * @brief 检查指定的执行提供器是否可用
 *
 * 遍历ONNXRuntime提供的可用提供器列表，检查目标提供器是否存在。
 *
 * @param providers ONNXRuntime返回的可用提供器列表
 * @param provider_name 要查找的提供器名称
 * @return 提供器可用返回true，否则返回false
 *
 * @note 仅在Linux x86_64平台编译此函数
 */
[[nodiscard]] bool provider_available(
    const std::vector<std::string>& providers, std::string_view provider_name) noexcept {
    return std::any_of(providers.begin(), providers.end(), [&](const std::string& provider) {
        return provider == provider_name;
    });
}

/**
 * @brief 选择OpenVINO设备类型
 *
 * 选择最优的OpenVINO设备类型。当前策略：
 * - 优先选择GPU_FP16（Intel Xe架构iGPU的半精度推理）
 * - 如果会话创建失败，会降级到GPU_FP32
 *
 * @return 设备类型字符串，如"GPU_FP16"
 *
 * @note FP16推理速度更快，但某些旧设备可能不支持
 * @note 仅在Linux x86_64平台编译此函数
 */
[[nodiscard]] std::string pick_openvino_device_type() noexcept {
    // 在Intel Xe级iGPU上优先使用FP16。如果会话创建拒绝，则降级到FP32。
    return "GPU_FP16";
}
#endif

/**
 * @brief 类别映射表（64个元素的常量数组）
 *
 * 该映射表将模型输出的类别ID（0-63）映射到装甲板的颜色和编号。
 * 编码规则：
 * - 类别ID = 颜色索引 * 16 + 尺寸索引 * 8 + 编号索引
 * - 颜色索引：0=红, 1=蓝, 2=灰, 3=紫
 * - 尺寸索引：0=小, 1=大
 * - 编号索引：0=哨兵, 1-5=数字, 6=前哨站, 7=基地
 *
 * 映射表布局：
 * - 0-7:   红色小装甲板（哨兵、1-5、前哨站、基地）
 * - 8-15:  红色大装甲板
 * - 16-23: 蓝色小装甲板
 * - 24-31: 蓝色大装甲板
 * - 32-39: 灰色小装甲板
 * - 40-47: 灰色大装甲板
 * - 48-55: 紫色小装甲板
 * - 56-63: 紫色大装甲板
 */
constexpr std::array<ClassMapping, 64> g_class_mappings = {
    {
     // 红色小装甲板 (类别0-7)
        {ArmorColor::Red, ArmorName::Sentry},
     {ArmorColor::Red, ArmorName::One},
     {ArmorColor::Red, ArmorName::Two},
     {ArmorColor::Red, ArmorName::Three},
     {ArmorColor::Red, ArmorName::Four},
     {ArmorColor::Red, ArmorName::Five},
     {ArmorColor::Red, ArmorName::Outpost},
     {ArmorColor::Red, ArmorName::Base},
     // 红色大装甲板 (类别8-15)
        {ArmorColor::Red, ArmorName::Sentry},
     {ArmorColor::Red, ArmorName::One},
     {ArmorColor::Red, ArmorName::Two},
     {ArmorColor::Red, ArmorName::Three},
     {ArmorColor::Red, ArmorName::Four},
     {ArmorColor::Red, ArmorName::Five},
     {ArmorColor::Red, ArmorName::Outpost},
     {ArmorColor::Red, ArmorName::Base},
     // 蓝色小装甲板 (类别16-23)
        {ArmorColor::Blue, ArmorName::Sentry},
     {ArmorColor::Blue, ArmorName::One},
     {ArmorColor::Blue, ArmorName::Two},
     {ArmorColor::Blue, ArmorName::Three},
     {ArmorColor::Blue, ArmorName::Four},
     {ArmorColor::Blue, ArmorName::Five},
     {ArmorColor::Blue, ArmorName::Outpost},
     {ArmorColor::Blue, ArmorName::Base},
     // 蓝色大装甲板 (类别24-31)
        {ArmorColor::Blue, ArmorName::Sentry},
     {ArmorColor::Blue, ArmorName::One},
     {ArmorColor::Blue, ArmorName::Two},
     {ArmorColor::Blue, ArmorName::Three},
     {ArmorColor::Blue, ArmorName::Four},
     {ArmorColor::Blue, ArmorName::Five},
     {ArmorColor::Blue, ArmorName::Outpost},
     {ArmorColor::Blue, ArmorName::Base},
     // 灰色小装甲板 (类别32-39)
        {ArmorColor::Neutral, ArmorName::Sentry},
     {ArmorColor::Neutral, ArmorName::One},
     {ArmorColor::Neutral, ArmorName::Two},
     {ArmorColor::Neutral, ArmorName::Three},
     {ArmorColor::Neutral, ArmorName::Four},
     {ArmorColor::Neutral, ArmorName::Five},
     {ArmorColor::Neutral, ArmorName::Outpost},
     {ArmorColor::Neutral, ArmorName::Base},
     // 灰色大装甲板 (类别40-47)
        {ArmorColor::Neutral, ArmorName::Sentry},
     {ArmorColor::Neutral, ArmorName::One},
     {ArmorColor::Neutral, ArmorName::Two},
     {ArmorColor::Neutral, ArmorName::Three},
     {ArmorColor::Neutral, ArmorName::Four},
     {ArmorColor::Neutral, ArmorName::Five},
     {ArmorColor::Neutral, ArmorName::Outpost},
     {ArmorColor::Neutral, ArmorName::Base},
     // 紫色小装甲板 (类别48-55)
        {ArmorColor::Purple, ArmorName::Sentry},
     {ArmorColor::Purple, ArmorName::One},
     {ArmorColor::Purple, ArmorName::Two},
     {ArmorColor::Purple, ArmorName::Three},
     {ArmorColor::Purple, ArmorName::Four},
     {ArmorColor::Purple, ArmorName::Five},
     {ArmorColor::Purple, ArmorName::Outpost},
     {ArmorColor::Purple, ArmorName::Base},
     // 紫色大装甲板 (类别56-63)
        {ArmorColor::Purple, ArmorName::Sentry},
     {ArmorColor::Purple, ArmorName::One},
     {ArmorColor::Purple, ArmorName::Two},
     {ArmorColor::Purple, ArmorName::Three},
     {ArmorColor::Purple, ArmorName::Four},
     {ArmorColor::Purple, ArmorName::Five},
     {ArmorColor::Purple, ArmorName::Outpost},
     {ArmorColor::Purple, ArmorName::Base},
     }
};

} // anonymous namespace

/**
 * @brief ONNXRuntime日志适配器
 *
 * 该函数是ONNXRuntime的日志回调，将ONNXRuntime的日志消息转换为spdlog格式。
 * 这样可以统一整个项目的日志输出格式和管理策略。
 *
 * **日志级别映射：**
 * - ORT_LOGGING_LEVEL_VERBOSE -> spdlog::level::debug
 * - ORT_LOGGING_LEVEL_INFO    -> spdlog::level::info
 * - ORT_LOGGING_LEVEL_WARNING -> spdlog::level::warn
 * - ORT_LOGGING_LEVEL_ERROR   -> spdlog::level::err
 * - ORT_LOGGING_LEVEL_FATAL   -> spdlog::level::critical
 *
 * @param param 用户自定义参数（当前未使用）
 * @param severity ONNXRuntime日志级别
 * @param category 日志类别（如"OnnxRuntime"）
 * @param logid 日志ID
 * @param code_location 代码位置（格式："file:line function"）
 * @param message 日志消息内容
 *
 * @note 该函数会被ONNXRuntime内部调用，不应手动调用
 */
void spdlog_ort_logger(
    void* param, OrtLoggingLevel severity, const char* category, const char* logid,
    const char* code_location, const char* message) {
    spdlog::source_loc sloc{};

    // 解析ONNXRuntime提供的代码位置字符串（格式："file:line function"）
    std::istringstream iss(code_location);

    std::string file_line;
    std::string funcname;
    iss >> file_line;
    iss >> funcname;
    sloc.funcname = funcname.c_str();

    // 从"file:line"中分离文件名和行号
    auto colon_pos = file_line.rfind(':');
    auto filename  = file_line.substr(0, colon_pos);

    if (colon_pos != std::string::npos) {
        sloc.filename = filename.c_str();
        sloc.line     = std::stoi(file_line.substr(colon_pos + 1));
    } else {
        sloc.filename = file_line.c_str();
    }

    // 将ONNXRuntime日志级别映射到spdlog日志级别
    switch (severity) {
    case ORT_LOGGING_LEVEL_VERBOSE:
        spdlog::default_logger()->log(sloc, spdlog::level::debug, "[{}] {}", category, message);
        break;
    case ORT_LOGGING_LEVEL_INFO:
        spdlog::default_logger()->log(sloc, spdlog::level::info, "[{}] {}", category, message);
        break;
    case ORT_LOGGING_LEVEL_WARNING:
        spdlog::default_logger()->log(sloc, spdlog::level::warn, "[{}] {}", category, message);
        break;
    case ORT_LOGGING_LEVEL_ERROR:
        spdlog::default_logger()->log(sloc, spdlog::level::err, "[{}] {}", category, message);
        break;
    case ORT_LOGGING_LEVEL_FATAL:
        spdlog::default_logger()->log(sloc, spdlog::level::critical, "[{}] {}", category, message);
        break;
    }
}

/**
 * @struct OrtBackend::Impl
 * @brief ONNXRuntime资源封装（PIMPL实现）
 *
 * 该结构体封装了ONNXRuntime的所有运行时资源，包括：
 * - 环境、会话选项、会话对象
 * - 输入输出张量名称和缓冲区
 *
 * 通过PIMPL模式，这些实现细节被隐藏在cpp文件中，避免了
 * ONNXRuntime庞大头文件对编译时间的影响。
 */
struct OrtBackend::Impl {
    Ort::Env env;                       ///< ONNXRuntime环境对象，每个进程只需一个
    Ort::SessionOptions session_options; ///< 会话选项，配置线程数、优化级别等
    std::unique_ptr<Ort::Session> session; ///< 推理会话，封装模型和执行引擎

    Ort::AllocatorWithDefaultOptions allocator; ///< 默认内存分配器

    std::string input_name;   ///< 模型输入节点名称
    std::string output_name;  ///< 模型输出节点名称

    std::array<const char*, 1> input_names{};   ///< 输入节点名称数组（ONNXRuntime C API要求）
    std::array<const char*, 1> output_names{};  ///< 输出节点名称数组（ONNXRuntime C API要求）

    std::vector<int64_t> input_shape{1, 3, INPUT_H, INPUT_W}; ///< 输入张量形状 [batch, channels, height, width]

    std::vector<float> input_buffer;       ///< 输入数据缓冲区，存储预处理后的图像数据
    std::vector<Ort::Value> output_tensors; ///< 输出张量列表，存储推理结果

    /**
     * @brief 构造函数
     *
     * 初始化ONNXRuntime环境，配置日志级别和日志回调函数。
     * 使用spdlog_ort_logger将ONNXRuntime的日志重定向到spdlog。
     */
    Impl()
        : env(ORT_LOGGING_LEVEL_WARNING, "", spdlog_ort_logger, nullptr) {}
};

// ============================================================================
// 构造函数与析构函数
// ============================================================================

/**
 * @brief 私有构造函数
 *
 * 仅用于工厂方法内部。构造函数本身不执行任何可能失败的操作，
 * 所有初始化工作都在create()工厂方法中完成。
 *
 * @param config 配置参数
 */
OrtBackend::OrtBackend(Config config) noexcept
    : config_{std::move(config)}
    , impl_(std::make_unique<Impl>()) {}

/**
 * @brief 析构函数
 *
 * 自动释放ONNXRuntime资源。由于impl_是unique_ptr，
 * 析构时会自动调用Impl的析构函数，释放所有资源。
 */
OrtBackend::~OrtBackend() = default;

/**
 * @brief 移动构造函数
 *
 * 转移ONNXRuntime资源的所有权。移动操作是noexcept的，
 * 保证不会抛出异常。
 *
 * @param other 源对象
 */
OrtBackend::OrtBackend(OrtBackend&& other) noexcept
    : config_{std::move(other.config_)}
    , impl_{std::move(other.impl_)} {}

/**
 * @brief 移动赋值运算符
 *
 * 转移ONNXRuntime资源的所有权。自赋值检查防止意外释放。
 *
 * @param other 源对象
 * @return 当前对象引用
 */
OrtBackend& OrtBackend::operator=(OrtBackend&& other) noexcept {
    if (this != &other) {
        config_ = std::move(other.config_);
        impl_   = std::move(other.impl_);
    }
    return *this;
}

// ============================================================================
// 工厂方法 — 构造即初始化
// ============================================================================

/**
 * @brief 工厂方法：创建并初始化ONNXRuntime后端
 *
 * 该方法是创建OrtBackend实例的唯一正确方式。实现了"构造即初始化"原则，
 * 确保返回的对象要么完全可用，要么返回详细的错误信息。
 *
 * **跨平台执行提供器选择策略：**
 *
 * 1. **Linux x86_64平台**（优先使用OpenVINO硬件加速）：
 *    - 尝试OpenVINO GPU_FP16（半精度推理，速度最快）
 *    - 失败则降级到OpenVINO GPU_FP32（单精度推理）
 *    - 如果GPU不可用，使用CPUExecutionProvider
 *
 * 2. **macOS平台**（使用Apple Neural Engine）：
 *    - 配置CoreML使用CPUAndNeuralEngine（CPU + 神经引擎）
 *    - 使用MLProgram格式，启用FastPrediction优化
 *    - 允许GPU低精度累加，提升推理速度
 *
 * 3. **其他平台**：
 *    - 使用CPUExecutionProvider，配置2个线程
 *
 * **会话配置参数：**
 * - 线程数：2（平衡性能与资源占用）
 * - 图优化级别：ORT_ENABLE_ALL（启用所有优化）
 * - 执行模式：ORT_SEQUENTIAL（顺序执行，保证确定性）
 * - 确定性计算：true（每次推理结果相同）
 *
 * @param config 配置参数，包含模型路径、缓存目录等
 * @return 成功返回初始化后的OrtBackend对象，失败返回错误信息
 *
 * @note 该方法noexcept，不会抛出异常
 * @note 模型文件不存在或格式错误会返回详细错误信息
 */
std::expected<OrtBackend, std::string> OrtBackend::create(Config config) noexcept {
    try {
        // 步骤1: 创建后端对象（调用私有构造函数）
        OrtBackend backend(std::move(config));
        const auto& cfg = backend.get_config();

        auto& impl = *backend.impl_;
        std::string selected_provider{"CPUExecutionProvider"};
        const auto available_providers = Ort::GetAvailableProviders();

        // 步骤2: 配置会话选项
        // 设置线程数为2，平衡多线程性能和资源占用
        impl.session_options.SetIntraOpNumThreads(2);
        // 启用所有图优化（算子融合、常量折叠等）
        impl.session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        // 设置顺序执行模式，保证推理结果可复现
        impl.session_options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        // 启用确定性计算，避免浮点运算顺序差异
        impl.session_options.SetDeterministicCompute(true);

#if defined(__linux__) && defined(__x86_64__)
        // ====================================================================
        // Linux x86_64平台：OpenVINO执行提供器选择
        // ====================================================================
        if (provider_available(available_providers, "OpenVINOExecutionProvider")
            || provider_available(available_providers, "OpenVINO")) {
            const auto openvino_device_type = pick_openvino_device_type();

            try {
                // 尝试使用GPU_FP16（半精度推理）
                OrtOpenVINOProviderOptions openvino_options{};
                openvino_options.device_type = openvino_device_type.c_str();
                openvino_options.cache_dir   = cfg.cache_dir.empty() ? "" : cfg.cache_dir.c_str();

                impl.session_options.AppendExecutionProvider_OpenVINO(openvino_options);
                selected_provider = "OpenVINOExecutionProvider(" + openvino_device_type + ")";
            } catch (const std::exception& gpu_fp16_error) {
                // GPU_FP16失败，降级到GPU_FP32
                SPDLOG_WARN(
                    "[OrtBackend] Failed to enable OpenVINO {}: {}. Retrying with GPU_FP32.",
                    openvino_device_type, gpu_fp16_error.what());

                try {
                    OrtOpenVINOProviderOptions openvino_options{};
                    static constexpr char kFallbackDeviceType[] = "GPU_FP32";
                    openvino_options.device_type                = kFallbackDeviceType;
                    openvino_options.cache_dir = cfg.cache_dir.empty() ? "" : cfg.cache_dir.c_str();

                    impl.session_options.AppendExecutionProvider_OpenVINO(openvino_options);
                    selected_provider = "OpenVINOExecutionProvider(GPU_FP32)";
                } catch (const std::exception& gpu_fp32_error) {
                    // GPU完全不可用，降级到CPU
                    SPDLOG_WARN(
                        "[OrtBackend] Failed to enable OpenVINO GPU execution: {}. Falling back "
                        "to CPUExecutionProvider.",
                        gpu_fp32_error.what());
                }
            }
        } else {
            SPDLOG_INFO(
                "[OrtBackend] OpenVINOExecutionProvider is not available in this ONNX Runtime "
                "build. Falling back to CPUExecutionProvider.");
        }
#endif

#ifdef __APPLE__
        // ====================================================================
        // macOS平台：CoreML执行提供器配置
        // ====================================================================
        std::unordered_map<std::string, std::string> coreml_options;

        // 使用MLProgram格式（比MLModel更现代）
        coreml_options["ModelFormat"]                        = "MLProgram";
        // 使用CPU和神经引擎（最佳性能）
        coreml_options["MLComputeUnits"]                     = "CPUAndNeuralEngine";
        // 要求静态输入形状（优化性能）
        coreml_options["RequireStaticInputShapes"]           = "1";
        // 快速预测策略（减少延迟）
        coreml_options["SpecializationStrategy"]             = "FastPrediction";
        // 允许GPU低精度累加（提升速度）
        coreml_options["AllowLowPrecisionAccumulationOnGPU"] = "1";
        // 设置模型缓存目录
        coreml_options["ModelCacheDirectory"]                = cfg.cache_dir;

        impl.session_options.AppendExecutionProvider("CoreML", coreml_options);
        selected_provider = "CoreMLExecutionProvider";
#endif

        // 步骤3: 创建ONNXRuntime会话（加载模型）
        impl.session =
            std::make_unique<Ort::Session>(impl.env, cfg.model_path.c_str(), impl.session_options);

        // 步骤4: 获取输入输出节点名称
        auto input_name  = impl.session->GetInputNameAllocated(0, impl.allocator);
        auto output_name = impl.session->GetOutputNameAllocated(0, impl.allocator);

        impl.input_name  = input_name.get();
        impl.output_name = output_name.get();

        impl.input_names  = {impl.input_name.c_str()};
        impl.output_names = {impl.output_name.c_str()};

        // 步骤5: 验证模型输入形状
        auto input_type_info   = impl.session->GetInputTypeInfo(0);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        auto input_shape       = input_tensor_info.GetShape();

        if (input_shape.size() != 4) {
            return std::unexpected("[OrtBackend] Expected input shape [1,3,H,W]");
        }

        // 处理动态维度（模型导出时可能使用-1表示动态）
        if (input_shape[0] <= 0)
            input_shape[0] = 1;
        if (input_shape[1] <= 0)
            input_shape[1] = 3;
        if (input_shape[2] <= 0)
            input_shape[2] = INPUT_H;
        if (input_shape[3] <= 0)
            input_shape[3] = INPUT_W;

        if (input_shape[0] != 1 || input_shape[1] != 3) {
            return std::unexpected("[OrtBackend] Only NCHW input [1,3,H,W] is supported");
        }

        impl.input_shape = std::move(input_shape);

        // 步骤6: 记录初始化成功信息
        SPDLOG_INFO(
            "[OrtBackend] ONNXRuntime initialized. provider={}, input={}, output={}, shape=[{}, "
            "{}, {}, {}]",
            selected_provider, impl.input_name, impl.output_name, impl.input_shape[0],
            impl.input_shape[1], impl.input_shape[2], impl.input_shape[3]);

        return backend;
    } catch (const std::exception& e) {
        return std::unexpected(
            std::string("[OrtBackend] Failed to create ONNXRuntime backend: ") + e.what());
    }
}

// ============================================================================
// 预处理 — Letterbox算法实现
// ============================================================================

/**
 * @brief 图像预处理：实现letterbox缩放与归一化
 *
 * **Letterbox算法原理：**
 *
 * 为了保持目标的长宽比，避免图像拉伸变形，使用letterbox技术：
 * 1. 计算缩放比例r = min(INPUT_W/orig_w, INPUT_H/orig_h)
 * 2. 将原始图像按比例r缩放
 * 3. 在缩放后的图像四周填充黑色像素，使其达到INPUT_W × INPUT_H
 * 4. 像素值归一化到[0,1]区间
 *
 * **坐标变换关系：**
 * - 缩放因子：scale = 1.0f / r（用于坐标反变换）
 * - 填充偏移：pad_x, pad_y（模型坐标减去偏移后再缩放）
 *
 * **变换公式：**
 * - 模型坐标 -> 原始坐标：x_orig = (x_model - pad_x) / scale
 * - 原始坐标 -> 模型坐标：x_model = x_orig * scale + pad_x
 *
 * @param image 原始输入图像，支持任意尺寸
 * @param cfg 配置参数（当前未使用）
 * @return 预处理上下文，包含变换后图像和坐标变换参数
 *
 * @note 输出图像格式：CV_32FC3，范围[0,1]，BGR通道顺序
 * @note 该函数noexcept，不会抛出异常
 */
OrtBackend::PreprocContext
    OrtBackend::preprocess(const cv::Mat& image, const ArmorOrtConfig& /*cfg*/) const noexcept {
    PreprocContext ctx;
    ctx.orig_w = image.cols;
    ctx.orig_h = image.rows;

    if (!impl_ || !impl_->session) {
        SPDLOG_ERROR("[OrtBackend] Session is not initialized");
        return ctx;
    }

    const auto& shp = impl_->input_shape; // [1, 3, H, W]

    if (shp.size() != 4 || shp[0] != 1 || shp[1] != 3) {
        SPDLOG_ERROR(
            "[OrtBackend] Input shape mismatch, got [{}, {}, {}, {}]", shp.size() > 0 ? shp[0] : -1,
            shp.size() > 1 ? shp[1] : -1, shp.size() > 2 ? shp[2] : -1,
            shp.size() > 3 ? shp[3] : -1);
        return ctx;
    }

    const int input_h = static_cast<int>(shp[2]);
    const int input_w = static_cast<int>(shp[3]);

    // 步骤1: 计算缩放比例（保持长宽比）
    const float r = std::min(
        static_cast<float>(input_w) / static_cast<float>(image.cols),
        static_cast<float>(input_h) / static_cast<float>(image.rows));

    // 步骤2: 计算缩放后的尺寸
    const int unpad_w = static_cast<int>(std::round(r * static_cast<float>(image.cols)));
    const int unpad_h = static_cast<int>(std::round(r * static_cast<float>(image.rows)));

    // 步骤3: 计算填充量
    const int pad_w = input_w - unpad_w;
    const int pad_h = input_h - unpad_h;

    // 填充均匀分布（左右、上下对称）
    const int left   = pad_w / 2;
    const int right  = pad_w - left;
    const int top    = pad_h / 2;
    const int bottom = pad_h - top;

    // 步骤4: 保存坐标变换参数（用于后处理）
    ctx.scale_x = 1.0f / r;  // 缩放因子的倒数
    ctx.scale_y = 1.0f / r;
    ctx.pad_x   = static_cast<float>(left);
    ctx.pad_y   = static_cast<float>(top);

    // 步骤5: 执行缩放
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(unpad_w, unpad_h));

    // 步骤6: 填充边界（黑色填充）
    cv::copyMakeBorder(
        resized, resized, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // 步骤7: 像素值归一化到[0,1]
    resized.convertTo(resized, CV_32F, 1.0f / 255.0f);
    ctx.preprocessed = resized;

    return ctx;
}

/**
 * @brief 执行ONNX模型推理
 *
 * **推理流程：**
 * 1. 将预处理后的图像从HWC格式（高×宽×通道）转换为NCHW格式（批次×通道×高×宽）
 * 2. 创建ONNXRuntime输入张量
 * 3. 调用session->Run()执行推理
 * 4. 获取输出张量
 *
 * **内存布局转换：**
 * - OpenCV: HWC格式（height, width, channels）
 * - ONNXRuntime: NCHW格式（batch, channels, height, width）
 * - 需要将图像通道分离并重新排列
 *
 * @param ctx 预处理上下文
 * @return 推理成功返回true，失败返回false
 *
 * @note 该方法会分配input_buffer_，如果之前有数据会被覆盖
 * @note 输出结果存储在impl_->output_tensors中
 * @note 该函数noexcept，不会抛出异常
 */
bool OrtBackend::infer(const PreprocContext& ctx) noexcept {
    if (ctx.preprocessed.empty()) {
        return false;
    }

    if (!impl_ || !impl_->session) {
        SPDLOG_ERROR("[OrtBackend] Session is not initialized");
        return false;
    }

    try {
        const auto& shape = impl_->input_shape;

        const int64_t n = shape[0];  // batch size
        const int64_t c = shape[1];  // channels
        const int64_t h = shape[2];  // height
        const int64_t w = shape[3];  // width

        if (n != 1 || c != 3) {
            SPDLOG_ERROR("[OrtBackend] Only [1,3,H,W] input is supported");
            return false;
        }

        const size_t plane      = static_cast<size_t>(h * w);
        const size_t input_size = static_cast<size_t>(n * c * h * w);

        // 分配输入缓冲区
        impl_->input_buffer.resize(input_size);

        // 分离图像通道（HWC -> NCHW）
        cv::Mat ch[3];
        cv::split(ctx.preprocessed, ch);

        if (ch[0].rows != h || ch[0].cols != w) {
            SPDLOG_ERROR(
                "[OrtBackend] Preprocessed image shape mismatch, got {}x{}, expected {}x{}",
                ch[0].cols, ch[0].rows, w, h);
            return false;
        }

        // 将通道数据复制到输入缓冲区（连续存储）
        // 缓冲区布局：[R平面, G平面, B平面]
        std::memcpy(impl_->input_buffer.data() + plane * 0, ch[0].data, plane * sizeof(float));
        std::memcpy(impl_->input_buffer.data() + plane * 1, ch[1].data, plane * sizeof(float));
        std::memcpy(impl_->input_buffer.data() + plane * 2, ch[2].data, plane * sizeof(float));

        // 创建ONNXRuntime输入张量
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPUOutput);

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, impl_->input_buffer.data(), impl_->input_buffer.size(),
            impl_->input_shape.data(), impl_->input_shape.size());

        // 执行推理
        impl_->output_tensors = impl_->session->Run(
            Ort::RunOptions{nullptr}, impl_->input_names.data(), &input_tensor, 1,
            impl_->output_names.data(), 1);

        return !impl_->output_tensors.empty();
    } catch (const std::exception& e) {
        SPDLOG_ERROR("[OrtBackend] Inference failed: {}", e.what());
        return false;
    }
}

// ============================================================================
// 后处理 — 解析模型输出
// ============================================================================

/**
 * @brief 后处理：解析模型输出并提取装甲板检测结果
 *
 * **模型输出格式：**
 * - 形状：[1, max_det, 14]
 * - 每行14个值：[x, y, w, h, conf, cls_id, kpt0_x, kpt0_y, kpt1_x, kpt1_y, kpt2_x, kpt2_y, kpt3_x, kpt3_y]
 *   - x, y, w, h: 边界框中心坐标和尺寸（模型坐标系）
 *   - conf: 置信度（0-1）
 *   - cls_id: 类别ID（0-63）
 *   - kpt*: 4个角点坐标（模型坐标系）
 *
 * **关键点顺序映射：**
 * - 模型输出顺序：[左上, 右上, 右下, 左下]
 * - 目标顺序：[左上, 左下, 右下, 右上]（逆时针）
 * - 使用keypoint_map = {0, 3, 2, 1}进行重排
 *
 * **处理流程：**
 * 1. 遍历所有检测结果（最多max_det个）
 * 2. 过滤低置信度检测（< confidence_threshold）
 * 3. 解析类别信息（颜色+编号）
 * 4. 提取4个角点坐标并重排顺序
 * 5. 将坐标从模型空间变换回原始图像空间
 * 6. 创建ArmorDetection对象
 *
 * **坐标变换公式：**
 * - x_orig = (x_model - pad_x) * scale_x
 * - y_orig = (y_model - pad_y) * scale_y
 *
 * @param ctx 预处理上下文，用于坐标变换
 * @param cfg 配置参数，包含置信度阈值
 * @return 装甲板检测结果列表
 *
 * @note 该函数noexcept，不会抛出异常
 */
std::vector<ArmorDetection>
    OrtBackend::postprocess(const PreprocContext& ctx, const ArmorOrtConfig& cfg) const noexcept {
    std::vector<ArmorDetection> detections;

    if (!impl_ || impl_->output_tensors.empty()) {
        return detections;
    }

    const Ort::Value& out = impl_->output_tensors[0];

    if (!out.IsTensor()) {
        SPDLOG_WARN("[OrtBackend] Output is not a tensor");
        return detections;
    }

    const float* p = out.GetTensorData<float>();
    if (!p) {
        return detections;
    }

    auto tensor_info = out.GetTensorTypeAndShapeInfo();
    const auto shape = tensor_info.GetShape();

    // 验证输出形状：[1, max_det, 14]
    if (shape.size() != 3 || shape[0] != 1) {
        SPDLOG_WARN("[OrtBackend] Unexpected output shape");
        return detections;
    }

    const int max_det    = static_cast<int>(shape[1]);
    const int output_dim = static_cast<int>(shape[2]);

    if (output_dim < 14) {
        SPDLOG_WARN("[OrtBackend] Unexpected output dim: {}", output_dim);
        return detections;
    }

    // 关键点顺序映射：模型输出顺序 -> 逆时针顺序
    // 模型输出：[左上, 右上, 右下, 左下]
    // 目标顺序：[左上, 左下, 右下, 右上]
    static constexpr std::array<int, 4> keypoint_map = {0, 3, 2, 1};

    detections.reserve(128);

    const float conf_thresh = static_cast<float>(cfg.confidence_threshold);

    // 坐标变换参数
    const float scale_x = 1.0f / ctx.scale_x;
    const float scale_y = 1.0f / ctx.scale_y;

    // 遍历所有检测结果
    for (int i = 0; i < max_det; ++i) {
        const float* row = p + i * output_dim;

        // 步骤1: 检查置信度
        const float conf = row[4];
        if (!std::isfinite(conf) || conf < conf_thresh) {
            continue;  // 跳过低置信度检测
        }

        // 步骤2: 提取边界框
        const float x = row[0];
        const float y = row[1];
        const float w = row[2];
        const float h = row[3];

        // 验证边界框有效性
        if (!std::isfinite(x) || !std::isfinite(y) || w <= 0.0f || h <= 0.0f) {
            continue;
        }

        // 步骤3: 解析类别
        const int cls_id = static_cast<int>(row[5]);
        if (cls_id < 0 || cls_id >= static_cast<int>(g_class_mappings.size())) {
            continue;  // 无效类别ID
        }

        const auto class_info = g_class_mappings[cls_id];

        // 步骤4: 提取关键点并重排顺序
        std::array<cv::Point2f, 4> pts;
        for (int k = 0; k < 4; ++k) {
            const float kx       = row[6 + 2 * k];
            const float ky       = row[6 + 2 * k + 1];
            pts[keypoint_map[k]] = cv::Point2f(kx, ky);
        }

        // 步骤5: 坐标变换（模型空间 -> 原始图像空间）
        for (auto& pt : pts) {
            pt.x = (pt.x - ctx.pad_x) / scale_x;
            pt.y = (pt.y - ctx.pad_y) / scale_y;
        }

        // 步骤6: 创建装甲板检测对象
        ArmorDetection det(pts, class_info.name, class_info.color, conf);
        if (det.area > 0) {
            detections.push_back(det);
        }
    }

    return detections;
}

// ============================================================================
// 坐标变换
// ============================================================================

/**
 * @brief 坐标变换：将检测坐标从模型空间映射回原始图像空间
 *
 * 该方法当前未使用，坐标变换已在postprocess中完成。
 * 保留该方法是为了保持接口兼容性和未来扩展。
 *
 * @param detections 检测结果列表（输入输出参数）
 * @param ctx 预处理上下文
 */
void OrtBackend::transform_coordinates(
    std::vector<ArmorDetection>& detections, const PreprocContext& ctx) const noexcept {
    // 坐标变换已在postprocess中完成
    (void)detections;
    (void)ctx;
}

// ============================================================================
// NMS（非极大值抑制）
// ============================================================================

/**
 * @brief 非极大值抑制（NMS）：去除重叠的检测框
 *
 * **NMS算法原理：**
 *
 * 在目标检测中，同一个目标可能被检测多次，产生多个重叠的检测框。
 * NMS算法通过IoU（交并比）指标去除冗余检测，只保留最佳检测。
 *
 * **算法流程：**
 * 1. 按置信度降序排序所有检测框
 * 2. 从最高置信度开始，依次选择检测框作为"保留框"
 * 3. 计算保留框与所有其他检测框的IoU
 * 4. 抑制所有与保留框IoU > nms_threshold的检测框
 * 5. 重复步骤2-4，直到所有检测框处理完毕
 * 6. 返回top_k个最高置信度的检测框
 *
 * **IoU计算：**
 * - IoU = 交集面积 / 并集面积
 * - IoU ∈ [0, 1]，越大表示重叠度越高
 *
 * **参数选择：**
 * - nms_threshold: 通常设为0.45-0.7
 *   - 值越小，抑制越激进，可能丢失真实目标
 *   - 值越大，抑制越温和，可能保留冗余检测
 * - top_k: 保留的最大检测数量，通常设为100-200
 *
 * @param detections 检测结果列表（会被修改）
 * @param cfg 配置参数，包含NMS阈值和top_k
 * @return NMS后的检测结果列表
 *
 * @note 该函数noexcept，不会抛出异常
 * @note 输入列表会被修改（排序）
 */
std::vector<ArmorDetection> OrtBackend::nms(
    std::vector<ArmorDetection>& detections, const ArmorOrtConfig& cfg) const noexcept {
    if (detections.empty()) {
        return {};
    }

    // 步骤1: 按置信度降序排序
    std::sort(
        detections.begin(), detections.end(), [](const ArmorDetection& a, const ArmorDetection& b) {
            return a.confidence > b.confidence;
        });

    // 步骤2: NMS主循环
    const float nms_thresh = static_cast<float>(cfg.nms_threshold);
    std::vector<bool> suppressed(detections.size(), false);
    std::vector<ArmorDetection> result;
    result.reserve(detections.size());

    for (size_t i = 0; i < detections.size(); ++i) {
        if (suppressed[i])
            continue;  // 已被抑制，跳过

        // 保留当前检测框
        result.push_back(detections[i]);

        // 抑制所有与当前框重叠度高的检测框
        for (size_t j = i + 1; j < detections.size(); ++j) {
            if (suppressed[j])
                continue;  // 已被抑制，跳过

            // 计算IoU并判断是否抑制
            if (at_legacy::detail::iou(detections[i].rect, detections[j].rect) > nms_thresh) {
                suppressed[j] = true;
            }
        }
    }

    // 步骤3: 应用top_k限制（匹配tensor_rt的顺序）
    if (static_cast<int>(result.size()) > cfg.top_k) {
        result.resize(cfg.top_k);
    }

    return result;
}

// ============================================================================
// 检测主入口
// ============================================================================

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
OrtBackend::DetectionResult
    OrtBackend::detect_impl(const cv::Mat& image, ArmorColor color) noexcept {
    if (image.empty()) {
        return std::unexpected("Empty image passed to NN backend");
    }

    const auto& cfg = get_config();

    // 步骤1: 预处理
    auto ctx = preprocess(image, cfg);
    if (ctx.preprocessed.empty()) {
        return std::unexpected("NN backend preprocessing failed");
    }

    // 步骤2: 推理
    if (!infer(ctx)) {
        return std::unexpected("NN backend inference failed");
    }

    // 步骤3: 后处理
    auto detections = postprocess(ctx, cfg);

    // 步骤4: NMS
    auto result = nms(detections, cfg);

    // 调试输出（已注释）
    // SPDLOG_INFO("[OrtBackend] ===== Detection Results =====");
    // SPDLOG_INFO("[OrtBackend] Total detections: {}", result.size());
    // for (size_t i = 0; i < result.size(); ++i) {
    //     const auto& det = result[i];
    //     const auto center = det.center();
    //
    //     SPDLOG_INFO("[OrtBackend] [{}] {} {} | conf={:.3f} | area={}",
    //         i,
    //         to_string(det.name),
    //         to_string(det.color),
    //         det.confidence,
    //         det.area);
    //
    //     SPDLOG_INFO("[OrtBackend]     Center: ({:.1f}, {:.1f})", center.x, center.y);
    //
    //     SPDLOG_INFO("[OrtBackend]     Corners: TL=({:.1f},{:.1f}) TR=({:.1f},{:.1f})
    //     BR=({:.1f},{:.1f}) BL=({:.1f},{:.1f})",
    //         det.corners[0].x, det.corners[0].y,
    //         det.corners[1].x, det.corners[1].y,
    //         det.corners[2].x, det.corners[2].y,
    //         det.corners[3].x, det.corners[3].y);
    //
    //     SPDLOG_INFO("[OrtBackend]     BBox: x={:.1f} y={:.1f} w={:.1f} h={:.1f}",
    //         det.rect.x, det.rect.y, det.rect.width, det.rect.height);
    // }
    // SPDLOG_INFO("[OrtBackend] =================================");

    return result;
}

// ============================================================================
// 工具方法
// ============================================================================

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
std::vector<std::string> OrtBackend::execution_devices() const noexcept {
    try {
        return Ort::GetAvailableProviders();
    } catch (const std::exception& e) {
        SPDLOG_WARN("[OrtBackend] Failed to get ONNXRuntime providers: {}", e.what());
        return {};
    }
}

} // namespace fcs::L2