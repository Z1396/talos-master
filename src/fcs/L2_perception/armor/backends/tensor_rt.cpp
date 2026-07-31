/**
 * @file tensor_rt.cpp
 * @brief TensorRT装甲板检测后端实现文件
 *
 * 本文件实现了基于NVIDIA TensorRT的高性能装甲板检测后端，包含完整的推理流程：
 *
 * ## 核心模块
 *
 * 1. **TrtLogger**: TensorRT日志适配器，将TensorRT内部日志映射到spdlog
 * 2. **ClassMapping**: 64类分类映射表，将class_id解码为颜色和数字
 * 3. **Impl**: PIMPL实现结构，管理所有TensorRT和CUDA资源
 * 4. **引擎管理**: 从.plan文件加载引擎或从ONNX模型构建引擎
 * 5. **推理流程**: 预处理→推理→后处理的完整pipeline
 *
 * ## 性能优化要点
 *
 * ### CUDA预处理优化
 *
 * 传统CPU预处理流程：
 * ```
 * CPU: OpenCV resize/padding → BGR2RGB → 归一化
 *        ↓
 *     cudaMemcpy (Host → Device)
 *        ↓
 * GPU: 推理
 * ```
 *
 * CUDA预处理流程：
 * ```
 * CPU: 原图直接上传
 *        ↓
 *     cudaMemcpyAsync (Host → Device)
 *        ↓
 * GPU: CUDA kernel (letterbox + BGR2RGB + 归一化) → 推理
 * ```
 *
 * 性能优势：
 * - 减少CPU-GPU数据传输量（只传输原图，不传输预处理后的图）
 * - 利用GPU并行计算能力完成预处理
 * - 异步执行，与推理overlap
 *
 * ### 多Stream并发优化
 *
 * 创建多个CUDA stream和ExecutionContext，支持多线程并发推理：
 * - 每个stream独立的命令队列，避免串行等待
 * - 使用原子变量round-robin调度，无锁分配stream
 * - 充分利用GPU计算能力，提高吞吐量
 *
 * ## NMS算法实现
 *
 * 采用标准NMS（Non-Maximum Suppression）算法：
 *
 * 1. 按置信度降序排序所有检测框
 * 2. 从高到低遍历，保留当前框，抑制与其IoU > threshold的所有框
 * 3. 重复直到所有框处理完毕
 *
 * 时间复杂度：O(n²)，其中n为检测框数量
 * 空间复杂度：O(n)，需要suppressed标记数组
 *
 * ## 引擎缓存策略
 *
 * 为了避免每次启动都重新构建引擎，实现了引擎缓存机制：
 *
 * 1. 尝试从engine_path加载已有的.plan文件
 * 2. 如果不存在，从ONNX模型构建引擎
 * 3. 构建完成后，序列化引擎并保存到engine_cache_dir
 * 4. 缓存文件命名规则：{model_name}_sm{compute_capability}[_fp16].engine
 *
 * 这样既支持首次启动自动构建，也支持后续快速加载。
 *
 * @author Talos Team
 * @date 2024
 */

#include "L2_perception/armor/backends/tensor_rt.hpp"
#include "L2_perception/armor/backends/letterbox.cuh"
#include "L2_perception/armor/config.hpp"
#include "core/types.hpp"

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <thread>
#include <unordered_map>
#include <vector>

#include <cuda_runtime.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>

namespace fcs::L2 {

namespace fs = std::filesystem;

// ============================================================================
// TensorRT日志适配器
// ============================================================================

/**
 * @brief TensorRT日志适配器类
 *
 * 将NVIDIA TensorRT内部日志消息适配到Talos框架使用的spdlog日志系统。
 * TensorRT通过nvinfer1::ILogger接口输出日志，本类实现该接口并转发到spdlog。
 *
 * ## 日志级别映射
 *
 * | TensorRT Severity    | spdlog Level | 说明                         |
 * |----------------------|--------------|------------------------------|
 * | kINTERNAL_ERROR      | critical     | 内部错误，不可恢复           |
 * | kERROR               | err          | 错误，但程序可能继续运行     |
 * | kWARNING             | warn         | 警告，可能影响性能或正确性   |
 * | kINFO                | info         | 信息性消息                   |
 * | kVERBOSE             | debug        | 详细调试信息                 |
 *
 * ## 性能考虑
 *
 * 只记录kWARNING及以上级别的日志，避免过多日志影响性能：
 * - kINFO和kVERBOSE级别通常包含大量调试信息
 * - 生产环境下不需要这些详细信息
 * - 如果需要调试，可以修改阈值
 */
class TrtLogger : public nvinfer1::ILogger {
public:
    /**
     * @brief 日志回调函数
     *
     * 当TensorRT产生日志消息时调用，将消息转发到spdlog。
     *
     * @param severity TensorRT日志级别
     * @param msg 日志消息内容
     */
    void log(Severity severity, const char* msg) noexcept override {
        // 只记录WARNING及以上级别，避免过多日志
        if (severity <= Severity::kWARNING) {
            spdlog::default_logger()->log(
                spdlog::level::from_str(severity_to_str(severity)), "[TensorRT] {}", msg);
        }
    }

    /**
     * @brief 将TensorRT日志级别转换为spdlog日志级别字符串
     *
     * @param severity TensorRT日志级别
     * @return spdlog日志级别字符串
     */
    static const char* severity_to_str(Severity severity) noexcept {
        switch (severity) {
        case Severity::kINTERNAL_ERROR: return "critical"; // TensorRT内部错误
        case Severity::kERROR: return "err";               // 一般错误
        case Severity::kWARNING: return "warn";            // 警告
        case Severity::kINFO: return "info";               // 信息
        case Severity::kVERBOSE: return "debug";           // 详细信息
        default: return "trace";                           // 未知级别
        }
    }
};

/// 全局TensorRT日志适配器实例
/// TensorRT要求logger在runtime生命周期内存在，因此定义为static全局变量
static TrtLogger g_trt_logger;

// ============================================================================
// 64类分类映射表
// ============================================================================

/**
 * @brief 分类映射结构体
 *
 * 将模型的64类输出解码为具体的颜色和装甲板数字。
 */
struct ClassMapping {
    ArmorColor color; ///< 装甲板颜色
    ArmorName name;   ///< 装甲板数字/身份
};

/**
 * @brief 64类分类映射表
 *
 * ## 设计原理
 *
 * 本映射表实现了64类分类体系，编码空间为：4颜色 × 2尺寸 × 8数字
 *
 * ### 类别索引编码公式
 *
 * ```
 * class_id = color_id * 16 + size_id * 8 + number_id
 * ```
 *
 * 其中：
 * - color_id: 0=Red, 1=Blue, 2=Neutral, 3=Purple
 * - size_id:  0=Small, 1=Large
 * - number_id: 0=Sentry, 1-5=数字, 6=Outpost, 7=Base
 *
 * ### 数值编码示例
 *
 * | number_id | 含义      | 应用场景                     |
 * |-----------|-----------|------------------------------|
 * | 0         | Sentry    | 哨兵机器人（前后都有装甲板）|
 * | 1         | Hero      | 英雄机器人的"1"号装甲板     |
 * | 2         | Engineer  | 工程机器人的"2"号装甲板     |
 * | 3         | Infantry  | 步兵3号机器人的"3"号装甲板  |
 * | 4         | Infantry  | 步兵4号机器人的"4"号装甲板  |
 * | 5         | Infantry  | 步兵5号机器人的"5"号装甲板  |
 * | 6         | Outpost   | 前哨站装甲板                |
 * | 7         | Base      | 基地装甲板                  |
 *
 * ### 尺寸编码规则
 *
 * - **Small (小装甲板)**：步兵、英雄机器人的装甲板（窄长型）
 * - **Large (大装甲板)**：哨兵、前哨站、基地的装甲板（宽大型）
 *
 * 尺寸信息对后续的PNP解算和姿态估计很重要：
 * - 不同尺寸的装甲板有不同的物理尺寸
 * - PNP算法需要准确的装甲板物理尺寸作为输入
 *
 * ### 颜色编码规则
 *
 * | color_id | 颜色    | 含义                         |
 * |----------|---------|------------------------------|
 * | 0        | Red     | 红方装甲板                   |
 * | 1        | Blue    | 蓝方装甲板                   |
 * | 2        | Neutral | 灰色/熄灭装甲板（无法判断）  |
 * | 3        | Purple  | 紫色装甲板（特殊场景）       |
 *
 * ### 查表方法
 *
 * ```cpp
 * int class_id = model_output[5];  // 模型输出的类别索引（0-63）
 * ClassMapping mapping = g_class_mappings[class_id];
 * ArmorColor color = mapping.color;  // 解码颜色
 * ArmorName name = mapping.name;     // 解码数字/身份
 * ```
 *
 * ### 批评：为什么用硬编码而不是公式计算？
 *
 * 虽然可以用公式计算（class_id -> color/size/number），但硬编码映射表有以下优势：
 *
 * 1. **可读性**：一目了然每个class_id对应什么
 * 2. **可维护性**：修改映射关系只需改表，不需要改公式
 * 3. **灵活性**：支持非规则映射（虽然当前是规则的）
 * 4. **性能**：查表比计算更快（虽然差别微乎其微）
 *
 * 缺点：
 * - 占用少量静态存储空间（64 * 2 * 4 = 512字节）
 * - 需要保持与训练代码的一致性
 *
 * ### 内存布局
 *
 * 映射表按照class_id顺序排列：
 * - 0-7: Red, Small
 * - 8-15: Red, Large
 * - 16-23: Blue, Small
 * - 24-31: Blue, Large
 * - 32-39: Neutral, Small
 * - 40-47: Neutral, Large
 * - 48-55: Purple, Small
 * - 56-63: Purple, Large
 */
static constexpr std::array<ClassMapping, 64> g_class_mappings = {
    {
     // Red, Small (0-7) - 红方小装甲板
        {ArmorColor::Red, ArmorName::Sentry},  // 0: 哨兵
        {ArmorColor::Red, ArmorName::One},     // 1: 1号装甲板
        {ArmorColor::Red, ArmorName::Two},     // 2: 2号装甲板
        {ArmorColor::Red, ArmorName::Three},   // 3: 3号装甲板
        {ArmorColor::Red, ArmorName::Four},    // 4: 4号装甲板
        {ArmorColor::Red, ArmorName::Five},    // 5: 5号装甲板
        {ArmorColor::Red, ArmorName::Outpost}, // 6: 前哨站
        {ArmorColor::Red, ArmorName::Base},    // 7: 基地
                                               // Red, Large (8-15) - 红方大装甲板
        {ArmorColor::Red, ArmorName::Sentry},
     {ArmorColor::Red, ArmorName::One},
     {ArmorColor::Red, ArmorName::Two},
     {ArmorColor::Red, ArmorName::Three},
     {ArmorColor::Red, ArmorName::Four},
     {ArmorColor::Red, ArmorName::Five},
     {ArmorColor::Red, ArmorName::Outpost},
     {ArmorColor::Red, ArmorName::Base},
     // Blue, Small (16-23) - 蓝方小装甲板
        {ArmorColor::Blue, ArmorName::Sentry},
     {ArmorColor::Blue, ArmorName::One},
     {ArmorColor::Blue, ArmorName::Two},
     {ArmorColor::Blue, ArmorName::Three},
     {ArmorColor::Blue, ArmorName::Four},
     {ArmorColor::Blue, ArmorName::Five},
     {ArmorColor::Blue, ArmorName::Outpost},
     {ArmorColor::Blue, ArmorName::Base},
     // Blue, Large (24-31) - 蓝方大装甲板
        {ArmorColor::Blue, ArmorName::Sentry},
     {ArmorColor::Blue, ArmorName::One},
     {ArmorColor::Blue, ArmorName::Two},
     {ArmorColor::Blue, ArmorName::Three},
     {ArmorColor::Blue, ArmorName::Four},
     {ArmorColor::Blue, ArmorName::Five},
     {ArmorColor::Blue, ArmorName::Outpost},
     {ArmorColor::Blue, ArmorName::Base},
     // None (Gray), Small (32-39) - 灰色小装甲板
        {ArmorColor::Neutral, ArmorName::Sentry},
     {ArmorColor::Neutral, ArmorName::One},
     {ArmorColor::Neutral, ArmorName::Two},
     {ArmorColor::Neutral, ArmorName::Three},
     {ArmorColor::Neutral, ArmorName::Four},
     {ArmorColor::Neutral, ArmorName::Five},
     {ArmorColor::Neutral, ArmorName::Outpost},
     {ArmorColor::Neutral, ArmorName::Base},
     // None (Gray), Large (40-47) - 灰色大装甲板
        {ArmorColor::Neutral, ArmorName::Sentry},
     {ArmorColor::Neutral, ArmorName::One},
     {ArmorColor::Neutral, ArmorName::Two},
     {ArmorColor::Neutral, ArmorName::Three},
     {ArmorColor::Neutral, ArmorName::Four},
     {ArmorColor::Neutral, ArmorName::Five},
     {ArmorColor::Neutral, ArmorName::Outpost},
     {ArmorColor::Neutral, ArmorName::Base},
     // Purple, Small (48-55) - 紫色小装甲板
        {ArmorColor::Purple, ArmorName::Sentry},
     {ArmorColor::Purple, ArmorName::One},
     {ArmorColor::Purple, ArmorName::Two},
     {ArmorColor::Purple, ArmorName::Three},
     {ArmorColor::Purple, ArmorName::Four},
     {ArmorColor::Purple, ArmorName::Five},
     {ArmorColor::Purple, ArmorName::Outpost},
     {ArmorColor::Purple, ArmorName::Base},
     // Purple, Large (56-63) - 紫色大装甲板
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

// ============================================================================
// PIMPL实现结构体
// ============================================================================

/**
 * @brief TrtBackend的PIMPL实现结构体
 *
 * 隐藏TensorRT和CUDA的实现细节，减少头文件依赖。
 * 所有TensorRT和CUDA资源都由RAII管理，确保异常安全。
 *
 * ## 资源管理策略
 *
 * 1. **Runtime**: 全局TensorRT运行时，每个进程一个实例
 * 2. **Engine**: 序列化的计算图，包含优化后的网络结构
 * 3. **ExecutionContext**: 推理上下文，保存中间状态，支持并发推理
 * 4. **CUDA Stream**: 异步执行队列，支持多stream并发
 * 5. **Device Memory**: GPU显存缓冲区，用于输入输出数据
 *
 * ## 并发设计
 *
 * - 创建多个ExecutionContext和CUDA Stream
 * - 使用原子变量round-robin分配stream
 * - 每个stream独立执行，避免同步等待
 * - 适用于多线程推理场景
 *
 * ## 内存布局
 *
 * - device_input_: [N, 3, 640, 640] (NCHW格式，float32)
 * - device_output_: [N, max_det, 14] (检测结果)
 * - host_output_: 用于接收GPU输出的Host内存
 */
struct TrtBackend::Impl {
    // =========================================================================
    // TensorRT资源
    // =========================================================================

    /// TensorRT运行时实例
    /// 负责引擎的反序列化和执行上下文的创建
    std::unique_ptr<nvinfer1::IRuntime> runtime_;

    /// TensorRT引擎实例
    /// 包含优化后的网络计算图，可以创建多个执行上下文
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;

    /// 执行上下文池
    /// 每个上下文可以独立执行推理，支持多线程并发
    /// 数量由config.num_streams决定（通常为线程数）
    std::vector<std::unique_ptr<nvinfer1::IExecutionContext>> contexts_;

    // =========================================================================
    // CUDA资源
    // =========================================================================

    /// CUDA流池
    /// 每个stream独立执行命令队列，避免同步开销
    /// 数量与contexts_相同，一一对应
    std::vector<cudaStream_t> streams_;

    /// 设备端输入缓冲区指针
    /// 大小：640 * 640 * 3 * sizeof(float) = 4.9 MB
    void* device_input_{nullptr};

    /// 设备端输出缓冲区指针
    /// 大小：max_det * 14 * sizeof(float) (如max_det=100，约5.6 KB)
    void* device_output_{nullptr};

    /// Host端输出缓冲区
    /// 用于接收GPU异步拷贝的推理结果
    std::vector<float> host_output_;

    /// Host端输入缓冲区（用于页锁定内存）
    /// 当使用CUDA预处理时，暂存原始图像数据
    std::vector<unsigned char> host_input_;

    // =========================================================================
    // 并发控制
    // =========================================================================

    /// 上下文池互斥锁（保留，当前使用原子变量）
    std::mutex context_mutex_;

    /// 下一个分配的上下文索引（原子变量）
    /// 使用fetch_add实现无锁round-robin调度
    std::atomic<uint32_t> next_context_{0};

    // =========================================================================
    // 模型参数
    // =========================================================================

    /// 输入张量名称（从引擎元数据获取）
    std::string input_name_;

    /// 输出张量名称（从引擎元数据获取）
    std::string output_name_;

    /// 输入缓冲区大小（字节）
    size_t input_size_{0};

    /// 输出缓冲区大小（字节）
    size_t output_size_{0};

    /// 最大检测数量（从模型输出维度推断）
    int max_detections_{0};

    // =========================================================================
    // 关键点映射
    // =========================================================================

    /**
     * @brief 关键点顺序映射表
     *
     * YOLO模型输出的关键点顺序与Talos期望的顺序不同：
     *
     * **YOLO输出顺序**：[TL, BL, BR, TR] (Top-Left, Bottom-Left, Bottom-Right, Top-Right)
     * **Talos期望顺序**：[TL, TR, BR, BL] (顺时针方向，从左上开始)
     *
     * 这个映射表用于重新排列关键点顺序：
     * ```
     * output[i] = input[keypoint_map[i]]
     * ```
     *
     * 映射关系：
     * - output[0] (TL) = input[0] (TL) - 左上角不变
     * - output[1] (TR) = input[3] (TR) - 右上角
     * - output[2] (BR) = input[2] (BR) - 右下角不变
     * - output[3] (BL) = input[1] (BL) - 左下角
     *
     * 这样可以得到顺时针方向的关键点序列，便于后续PNP解算和坐标变换。
     */
    static constexpr std::array<int, 4> keypoint_map = {0, 3, 2, 1};

    // =========================================================================
    // 构造与析构
    // =========================================================================

    Impl() noexcept = default;

    /**
     * @brief 析构函数：释放所有资源
     *
     * 按照正确的顺序释放资源：
     * 1. 同步并销毁CUDA Stream
     * 2. 释放设备内存
     * 3. 销毁ExecutionContext
     * 4. 销毁Engine
     * 5. 销毁Runtime
     */
    ~Impl() noexcept { release(); }

    /**
     * @brief 释放所有资源
     *
     * ## 资源释放顺序的重要性
     *
     * CUDA和TensorRT资源有依赖关系，必须按正确顺序释放：
     *
     * 1. **Stream同步**：确保所有异步操作完成
     * 2. **Stream销毁**：释放CUDA流资源
     * 3. **显存释放**：释放cudaMalloc分配的内存
     * 4. **Context销毁**：ExecutionContext依赖Engine
     * 5. **Engine销毁**：Engine依赖Runtime
     * 6. **Runtime销毁**：最后释放TensorRT运行时
     *
     * 如果顺序错误，可能导致：
     * - 资源泄漏
     * - CUDA错误
     * - 程序崩溃
     */
    void release() noexcept {
        // 1. 同步并销毁CUDA Stream
        for (auto& stream : streams_) {
            if (stream) {
                // 等待stream上所有操作完成
                cudaStreamSynchronize(stream);
                // 销毁stream
                cudaStreamDestroy(stream);
            }
        }
        streams_.clear();

        // 2. 释放设备内存
        if (device_input_) {
            cudaFree(device_input_);
            device_input_ = nullptr;
        }
        if (device_output_) {
            cudaFree(device_output_);
            device_output_ = nullptr;
        }

        // 3-5. 销毁ExecutionContext、Engine、Runtime
        // unique_ptr自动调用析构函数，顺序正确
        contexts_.clear();
        engine_.reset();
        runtime_.reset();
    }
};

// ============================================================================
// 辅助函数
// ============================================================================

namespace {

/**
 * @brief 从.plan文件加载TensorRT引擎
 *
 * ## 引擎序列化格式
 *
 * TensorRT引擎以二进制格式序列化到.plan文件，包含：
 * - 优化后的网络计算图
 * - 针对特定GPU架构优化的kernel
 * - 权重数据和执行策略
 * - 内存分配和执行计划
 *
 * ## 加载流程
 *
 * 1. 打开.plan文件并获取文件大小
 * 2. 读取整个文件到内存
 * 3. 调用runtime->deserializeCudaEngine()反序列化
 * 4. 返回引擎实例
 *
 * ## 性能考虑
 *
 * - 直接加载预先构建的引擎比从ONNX构建快10-100倍
 * - 引擎文件是二进制格式，不可跨平台/跨架构
 * - 引擎文件通常较大（几十MB到几百MB）
 *
 * @param runtime TensorRT运行时实例
 * @param engine_path 引擎文件路径
 * @return 成功返回引擎实例，失败返回错误信息
 */
[[nodiscard]] std::expected<std::unique_ptr<nvinfer1::ICudaEngine>, std::string>
    load_engine(nvinfer1::IRuntime* runtime, const fs::path& engine_path) noexcept {
    try {
        // 打开文件，使用ate模式定位到文件末尾获取大小
        std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
            SPDLOG_ERROR("Failed to open engine file: {}", engine_path.string());
            return std::unexpected("TensorRT: failed to open engine file " + engine_path.string());
        }

        // 获取文件大小
        std::streamsize file_size = file.tellg();
        file.seekg(0, std::ios::beg);

        // 读取文件内容到内存
        std::vector<char> engine_data(file_size);
        if (!file.read(engine_data.data(), file_size)) {
            SPDLOG_ERROR("Failed to read engine file: {}", engine_path.string());
            return std::unexpected("TensorRT: failed to read engine file " + engine_path.string());
        }

        // 反序列化引擎（这是CPU密集操作）
        auto* engine = runtime->deserializeCudaEngine(engine_data.data(), file_size);
        if (!engine) {
            SPDLOG_ERROR("Failed to deserialize CUDA engine");
            return std::unexpected("TensorRT: failed to deserialize CUDA engine");
        }

        return std::unique_ptr<nvinfer1::ICudaEngine>(engine);
    } catch (const std::exception& e) {
        // 捕获所有异常并转换为expected错误
        SPDLOG_ERROR("[TrtBackend] Unexpected exception in load_engine: {}", e.what());
        return std::unexpected(
            "[TrtBackend::load_engine] Unhandled exception: " + std::string(e.what()));
    }
}

/**
 * @brief 从ONNX模型构建TensorRT引擎
 *
 * ## 构建流程
 *
 * 1. **创建Builder**: TensorRT构建器，负责优化和序列化网络
 * 2. **创建Network**: 网络定义，使用显式batch模式
 * 3. **创建Config**: 构建配置，设置优化选项
 * 4. **解析ONNX**: 使用ONNX Parser解析模型结构
 * 5. **构建引擎**: 优化网络并生成执行计划
 * 6. **序列化缓存**: 将引擎保存到文件，避免重复构建
 *
 * ## 优化选项
 *
 * ### FP16精度
 *
 * - 利用Tensor Core加速（2-4倍性能提升）
 * - 精度损失通常可忽略（装甲板检测对精度不敏感）
 * - 需要GPU支持FP16（Volta及以后架构）
 *
 * ### DLA (Deep Learning Accelerator)
 *
 * - NVIDIA Jetson系列的专用推理加速器
 * - 低功耗、高效率，适合边缘计算
 * - Xavier有2个DLA核心，Orin有2个DLA核心
 *
 * ### 内存限制
 *
 * - 设置workspace大小为可用显存的90%
 * - 避免构建时内存不足
 * - 大workspace允许更激进的优化
 *
 * ## 引擎缓存策略
 *
 * 构建引擎是耗时操作（几秒到几分钟），因此将构建好的引擎缓存到文件：
 *
 * - 缓存路径：{cache_dir}/{model_name}_sm{compute_capability}[_fp16].engine
 * - 命名包含GPU架构信息，避免跨架构混用
 * - 后续启动直接加载缓存，秒级启动
 *
 * ## 构建时间估计
 *
 * | 模型大小 | GPU型号       | FP16 | 构建时间 |
 * |---------|---------------|------|----------|
 * | YOLOv5s | RTX 3080      | 是   | 5-10秒   |
 * | YOLOv5m | RTX 3080      | 是   | 10-20秒  |
 * | YOLOv5s | Jetson Xavier | 是   | 30-60秒  |
 *
 * @param runtime TensorRT运行时实例
 * @param onnx_path ONNX模型路径
 * @param cache_dir 引擎缓存目录
 * @param compute_capability GPU计算能力（如"75", "86"）
 * @param enable_fp16 是否启用FP16精度
 * @param enable_dla 是否启用DLA核心
 * @param dla_core DLA核心编号（0或1）
 * @return 成功返回引擎实例，失败返回错误信息
 */
[[nodiscard]] std::expected<std::unique_ptr<nvinfer1::ICudaEngine>, std::string>
    build_engine_from_onnx(
        nvinfer1::IRuntime* runtime, const fs::path& onnx_path, const fs::path& cache_dir,
        const std::string& compute_capability, bool enable_fp16, bool enable_dla,
        int dla_core) noexcept {

    // 1. 创建Builder（TensorRT构建器）
    auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(g_trt_logger));
    if (!builder) {
        SPDLOG_ERROR("Failed to create TensorRT builder");
        return std::unexpected("TensorRT: failed to create builder");
    }

    // 2. 创建Network（显式batch模式，TensorRT 8.5+要求）
    const auto explicit_batch =
        1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network =
        std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicit_batch));
    if (!network) {
        SPDLOG_ERROR("Failed to create network definition");
        return std::unexpected("TensorRT: failed to create network definition");
    }

    // 3. 创建BuilderConfig
    auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
    if (!config) {
        SPDLOG_ERROR("Failed to create builder config");
        return std::unexpected("TensorRT: failed to create builder config");
    }

    // 4. 设置FP16模式（如果支持）
    if (enable_fp16 && builder->platformHasFastFp16()) {
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
        SPDLOG_INFO("TensorRT: FP16 mode enabled");
    }

    // 5. 设置DLA核心（如果可用）
    if (enable_dla && builder->getNbDLACores() > 0) {
        config->setDLACore(dla_core);
        SPDLOG_INFO("TensorRT: DLA core {} enabled", dla_core);
    }

    // 6. 设置内存限制（使用可用显存的90%）
    size_t free_mem, total_mem;
    cudaMemGetInfo(&free_mem, &total_mem);
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, free_mem * 0.9);

    // 7. 解析ONNX模型
    auto parser =
        std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, g_trt_logger));
    if (!parser) {
        SPDLOG_ERROR("Failed to create ONNX parser");
        return std::unexpected("TensorRT: failed to create ONNX parser");
    }

    if (!parser->parseFromFile(
            onnx_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kINFO))) {
        SPDLOG_ERROR("Failed to parse ONNX model: {}", onnx_path.string());
        return std::unexpected("TensorRT: failed to parse ONNX model " + onnx_path.string());
    }

    // 8. 构建序列化网络（这是最耗时的步骤）
    auto plan =
        std::unique_ptr<nvinfer1::IHostMemory>(builder->buildSerializedNetwork(*network, *config));
    if (!plan) {
        SPDLOG_ERROR("Failed to build serialized network");
        return std::unexpected("TensorRT: failed to build serialized network");
    }

    // 9. 缓存引擎到文件
    if (!cache_dir.empty()) {
        fs::create_directories(cache_dir);

        // 生成缓存文件名：{model_name}_sm{compute_capability}[_fp16].engine
        std::string cache_name = onnx_path.stem().string();
        cache_name += "_sm";
        cache_name += compute_capability;
        if (enable_fp16)
            cache_name += "_fp16";
        cache_name += ".engine";

        fs::path cache_path = cache_dir / cache_name;

        // 写入文件
        std::ofstream cache_file(cache_path, std::ios::binary);
        if (cache_file.is_open()) {
            cache_file.write(static_cast<const char*>(plan->data()), plan->size());
            SPDLOG_INFO("Cached TensorRT engine to: {}", cache_path.string());
        }
    }

    // 10. 反序列化引擎
    auto* engine = runtime->deserializeCudaEngine(plan->data(), plan->size());
    if (!engine) {
        SPDLOG_ERROR("Failed to deserialize CUDA engine from ONNX");
        return std::unexpected("TensorRT: failed to deserialize CUDA engine from ONNX build");
    }

    return std::unique_ptr<nvinfer1::ICudaEngine>(engine);
}

/**
 * @brief 计算两个边界框的IoU (Intersection over Union)
 *
 * ## IoU定义
 *
 * IoU是衡量两个边界框重叠程度的指标，定义为：
 *
 * ```
 * IoU = 交集面积 / 并集面积
 *     = intersection_area / (area_A + area_B - intersection_area)
 * ```
 *
 * IoU取值范围：[0, 1]
 * - IoU = 0: 两框不重叠
 * - IoU = 1: 两框完全重合
 * - IoU > 0.5: 通常认为有明显重叠
 *
 * ## 边界情况处理
 *
 * - 并集面积为0或NaN时返回0（避免除零错误）
 * - 使用cv::Rect2f的&运算符计算交集
 * - cv::Rect2f的area()自动处理负宽高（返回0）
 *
 * @param a 第一个边界框
 * @param b 第二个边界框
 * @return IoU值，范围[0, 1]
 */
[[nodiscard]] inline float calculate_iou(const cv::Rect2f& a, const cv::Rect2f& b) noexcept {
    // 计算交集矩形
    const cv::Rect2f inter = a & b;
    const float inter_area = inter.area();

    // 计算并集面积
    const float union_area = a.area() + b.area() - inter_area;

    // 边界情况：并集面积为0或NaN
    if (union_area <= 0.0f || std::isnan(union_area)) {
        return 0.0f;
    }

    return inter_area / union_area;
}

/**
 * @brief NMS（Non-Maximum Suppression）算法
 *
 * ## 算法原理
 *
 * NMS用于去除重叠的检测框，保留置信度最高的框：
 *
 * 1. **预排序**：输入已按置信度降序排序的检测框
 * 2. **贪心选择**：从高到低遍历，保留当前框
 * 3. **抑制重叠**：抑制与当前框IoU > threshold的所有框
 * 4. **重复迭代**：直到所有框处理完毕
 *
 * ## 算法复杂度
 *
 * - 时间复杂度：O(n²)，需要计算所有框对的IoU
 * - 空间复杂度：O(n)，需要suppressed标记数组
 *
 * 虽然有更快的NMS算法（如Fast NMS、Cluster-NMS），但：
 * - 装甲板检测场景下n通常很小（< 100）
 * - O(n²)的实现简单、正确性易验证
 * - 性能差异在毫秒级别，对总体性能影响小
 *
 * ## 为什么输入要预先排序？
 *
 * 虽然可以在函数内部排序，但要求调用者预先排序有以下优势：
 * - 调用者可能有其他用途需要排序后的结果
 * - 避免重复排序（如多次调用NMS）
 * - 函数职责更单一
 *
 * ## 为什么返回索引而不是检测结果？
 *
 * 返回索引而不是检测结果：
 * - 避免不必要的拷贝
 * - 调用者可以根据索引访问其他信息（如关键点、类别）
 * - 更灵活，可以结合其他过滤条件
 *
 * ## 参数选择
 *
 * | nms_threshold | 效果                               | 适用场景               |
 * |---------------|------------------------------------|------------------------|
 * | 0.3           | 严格，去除更多重叠框               | 密集场景，需要精细区分 |
 * | 0.5           | 中等，平衡保留和去除               | 大多数场景（默认）     |
 * | 0.7           | 宽松，只去除高度重叠的框           | 稀疏场景，避免漏检     |
 *
 * @param boxes 边界框列表（已按置信度降序排序）
 * @param confidences 置信度列表（已排序）
 * @param nms_threshold IoU阈值，超过此值的框将被抑制
 * @return 保留的检测框索引列表
 */
[[nodiscard]] std::vector<int> nms_merge_sorted_bboxes(
    const std::vector<cv::Rect2f>& boxes, const std::vector<float>& confidences,
    float nms_threshold) {

    const size_t n = boxes.size();
    std::vector<int> keep;                  // 保留的索引列表
    std::vector<bool> suppressed(n, false); // 抑制标记

    // 遍历所有检测框（已按置信度排序）
    for (size_t i = 0; i < n; ++i) {
        // 如果已被抑制，跳过
        if (suppressed[i])
            continue;

        // 保留当前框（置信度最高的未抑制框）
        keep.push_back(static_cast<int>(i));

        // 抑制与当前框重叠度高的框
        for (size_t j = i + 1; j < n; ++j) {
            if (suppressed[j])
                continue;

            // 计算IoU
            float iou = calculate_iou(boxes[i], boxes[j]);

            // 如果IoU超过阈值，抑制框j
            if (iou > nms_threshold) {
                suppressed[j] = true;
            }
        }
    }

    return keep;
}

} // anonymous namespace

// ============================================================================
// Constructor/Destructor
// ============================================================================

TrtBackend::TrtBackend(Config config) noexcept
    : config_(std::move(config)) {
    impl_ = std::make_unique<Impl>();
}

TrtBackend::~TrtBackend() noexcept = default;

TrtBackend::TrtBackend(TrtBackend&&) noexcept = default;

TrtBackend& TrtBackend::operator=(TrtBackend&&) noexcept = default;

// ============================================================================
// 工厂方法 — 构造即初始化
// ============================================================================

/**
 * @brief 工厂方法：构造完全初始化的TrtBackend实例
 *
 * ## 设计哲学：构造即初始化
 *
 * 传统C++对象构造模式存在"半初始化"问题：
 * ```cpp
 * TrtBackend backend;  // 半初始化状态
 * backend.init();      // 可能失败，对象处于无效状态
 * ```
 *
 * 本实现采用"构造即初始化"模式：
 * ```cpp
 * auto backend = TrtBackend::create(config);
 * if (!backend) {
 *     // 处理错误
 * }
 * // backend已经完全初始化，立即可用
 * ```
 *
 * ## 初始化流程
 *
 * 1. **设置CUDA设备**：根据config.device_id选择GPU
 * 2. **创建Runtime**：TensorRT运行时环境
 * 3. **加载/构建引擎**：
 *    - 优先加载已序列化的引擎文件
 *    - 如果不存在，从ONNX构建并缓存
 * 4. **获取I/O信息**：提取输入输出张量名称和维度
 * 5. **分配GPU内存**：输入输出缓冲区
 * 6. **创建并发资源**：多个ExecutionContext和CUDA Stream
 *
 * ## 错误处理策略
 *
 * - 所有可能失败的系统调用都返回std::expected
 * - 错误信息包含：失败操作、系统错误、相关上下文
 * - 如果初始化失败，不产生部分初始化的对象
 *
 * ## 性能考虑
 *
 * - 首次启动：需要构建引擎（5-60秒）
 * - 后续启动：加载缓存引擎（< 1秒）
 * - 多Stream初始化：略增加启动时间，但显著提高并发性能
 *
 * @param config 配置参数
 * @return 成功返回初始化完成的TrtBackend实例，失败返回错误信息
 */
std::expected<TrtBackend, std::string> TrtBackend::create(Config config) noexcept {
    TrtBackend backend(std::move(config));

    // 1. 设置CUDA设备
    cudaError_t cuda_err = cudaSetDevice(backend.config_.device_id);
    if (cuda_err != cudaSuccess) {
        SPDLOG_ERROR(
            "Failed to set CUDA device {}: {}", backend.config_.device_id,
            cudaGetErrorString(cuda_err));
        return std::unexpected(
            "TensorRT: failed to set CUDA device " + std::to_string(backend.config_.device_id)
            + ": " + cudaGetErrorString(cuda_err));
    }

    // 2. 创建TensorRT Runtime
    backend.impl_->runtime_ =
        std::unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(g_trt_logger));
    if (!backend.impl_->runtime_) {
        SPDLOG_ERROR("Failed to create TensorRT runtime");
        return std::unexpected(
            "TensorRT: failed to set CUDA device " + std::to_string(backend.config_.device_id)
            + ": " + cudaGetErrorString(cuda_err));
    }

    // 3. 加载或构建引擎
    fs::path engine_path(backend.config_.engine_path);
    fs::path model_path(backend.config_.model_path);
    fs::path cache_dir(backend.config_.engine_cache_dir);

    std::unique_ptr<nvinfer1::ICudaEngine> engine;

    // 3.1 尝试加载现有引擎文件
    if (!engine_path.empty() && fs::exists(engine_path)) {
        SPDLOG_INFO("Loading TensorRT engine from: {}", engine_path.string());
        auto result = load_engine(backend.impl_->runtime_.get(), engine_path);
        if (!result) {
            SPDLOG_ERROR("Failed to load engine, will try building from ONNX if available");
        } else {
            engine = std::move(result.value());
        }
    }

    // 3.2 如果加载失败，从ONNX构建
    if (!engine && !model_path.empty() && fs::exists(model_path)) {
        SPDLOG_INFO("Building TensorRT engine from ONNX: {}", model_path.string());
        auto result = build_engine_from_onnx(
            backend.impl_->runtime_.get(), model_path, cache_dir,
            backend.config_.compute_capability, backend.config_.enable_fp16,
            backend.config_.enable_dla, backend.config_.dla_core);
        if (!result) {
            SPDLOG_ERROR("Failed to build engine from ONNX");
            return std::unexpected("TensorRT: failed to build engine from ONNX");
        }
        engine = std::move(result.value());
    }

    // 3.3 确保引擎可用
    if (!engine) {
        SPDLOG_ERROR("No valid engine or model found");
        return std::unexpected("TensorRT: no valid engine or model file found");
    }

    backend.impl_->engine_ = std::move(engine);

    // 4. 获取输入输出张量信息
    const int num_tensors = backend.impl_->engine_->getNbIOTensors();
    for (int i = 0; i < num_tensors; ++i) {
        const char* name = backend.impl_->engine_->getIOTensorName(i);
        auto mode        = backend.impl_->engine_->getTensorIOMode(name);
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            backend.impl_->input_name_ = name;
            auto dims                  = backend.impl_->engine_->getTensorShape(name);
            backend.impl_->input_size_ = INPUT_W * INPUT_H * 3 * sizeof(float);
            SPDLOG_INFO("Input: {} dims=[{}]", name, dims.d[0]);
        } else {
            backend.impl_->output_name_ = name;
            auto dims                   = backend.impl_->engine_->getTensorShape(name);
            // 输出形状：[1, max_det, 14]
            backend.impl_->max_detections_ = dims.d[1];
            backend.impl_->output_size_ =
                backend.impl_->max_detections_ * OUTPUT_DIM * sizeof(float);
            SPDLOG_INFO("Output: {} dims=[1, {}, {}]", name, dims.d[1], dims.d[2]);
        }
    }

    // 5. 分配设备内存（输入缓冲区）
    cuda_err = cudaMalloc(&backend.impl_->device_input_, backend.impl_->input_size_);
    if (cuda_err != cudaSuccess) {
        SPDLOG_ERROR("Failed to allocate device input buffer: {}", cudaGetErrorString(cuda_err));
        return std::unexpected(
            "TensorRT: failed to set CUDA device " + std::to_string(backend.config_.device_id)
            + ": " + cudaGetErrorString(cuda_err));
    }

    // 5. 分配设备内存（输出缓冲区）
    cuda_err = cudaMalloc(&backend.impl_->device_output_, backend.impl_->output_size_);
    if (cuda_err != cudaSuccess) {
        SPDLOG_ERROR("Failed to allocate device output buffer: {}", cudaGetErrorString(cuda_err));
        return std::unexpected(
            "TensorRT: failed to set CUDA device " + std::to_string(backend.config_.device_id)
            + ": " + cudaGetErrorString(cuda_err));
    }

    // 6. 调整Host缓冲区大小
    backend.impl_->host_output_.resize(backend.impl_->max_detections_ * OUTPUT_DIM);
    backend.impl_->host_input_.resize(INPUT_W * INPUT_H * 3);

    // 7. 创建执行上下文和CUDA流（支持并发推理）
    int num_streams = backend.config_.num_streams;
    backend.impl_->contexts_.reserve(num_streams);
    backend.impl_->streams_.reserve(num_streams);

    for (int i = 0; i < num_streams; ++i) {
        // 创建ExecutionContext（可以并发执行）
        auto* ctx = backend.impl_->engine_->createExecutionContext();
        if (!ctx) {
            SPDLOG_ERROR("Failed to create execution context {}", i);
            return std::unexpected(
                "TensorRT: failed to set CUDA device " + std::to_string(backend.config_.device_id)
                + ": " + cudaGetErrorString(cuda_err));
        }
        backend.impl_->contexts_.emplace_back(ctx);

        // 创建CUDA Stream（独立的命令队列）
        cudaStream_t stream;
        cuda_err = cudaStreamCreate(&stream);
        if (cuda_err != cudaSuccess) {
            SPDLOG_ERROR("Failed to create CUDA stream {}: {}", i, cudaGetErrorString(cuda_err));
            return std::unexpected(
                "TensorRT: failed to set CUDA device " + std::to_string(backend.config_.device_id)
                + ": " + cudaGetErrorString(cuda_err));
        }
        backend.impl_->streams_.push_back(stream);
    }

    SPDLOG_INFO("TensorRT backend initialized with {} streams", num_streams);
    return backend;
}

// ============================================================================
// 检测方法
// ============================================================================

/**
 * @brief 执行装甲板检测
 *
 * ## 完整的检测流程
 *
 * ### 1. 预处理阶段 (Preprocessing)
 *
 * - **Letterbox变换**：保持长宽比缩放到640x640
 * - **CUDA加速**：在GPU端完成预处理，减少数据传输
 * - **归一化**：将像素值从[0,255]归一化到[0,1]
 *
 * ### 2. 推理阶段 (Inference)
 *
 * - **异步执行**：使用CUDA stream异步推理
 * - **TensorRT优化**：利用层融合、精度优化等技术
 * - **多Stream并发**：支持多线程并发推理
 *
 * ### 3. 后处理阶段 (Postprocessing)
 *
 * - **解析输出**：从[1, max_det, 14]张量提取检测结果
 * - **置信度过滤**：移除低置信度检测
 * - **坐标映射**：将模型坐标转换回原图坐标
 * - **关键点重排**：调整关键点顺序为顺时针方向
 * - **NMS**：去除重复检测框
 * - **Top-K限制**：只保留置信度最高的K个检测
 *
 * ## CUDA预处理优化原理
 *
 * 传统CPU预处理：
 * ```
 * CPU: OpenCV resize/padding (耗时)
 *   ↓
 * cudaMemcpy (Host→Device, 传输耗时)
 *   ↓
 * GPU: 推理
 * ```
 *
 * CUDA预处理：
 * ```
 * CPU: 原图上传
 *   ↓
 * cudaMemcpyAsync (异步传输)
 *   ↓
 * GPU: CUDA kernel (letterbox + 归一化) → 推理
 * ```
 *
 * 优势：
 * - 减少CPU-GPU传输量
 * - 利用GPU并行计算
 * - 异步执行，提高吞吐量
 *
 * ## 输出格式说明
 *
 * 模型输出形状：[1, max_det, 14]
 *
 * 每个检测包含14个值：
 * - [0-3]: bbox (x_center, y_center, width, height)
 * - [4]: confidence (置信度)
 * - [5]: class_id (类别索引，0-63)
 * - [6-13]: 4个关键点坐标 (x, y) × 4
 *
 * @param image 输入图像（BGR格式）
 * @param color 目标颜色（当前版本已禁用颜色过滤）
 * @return 成功返回检测结果列表，失败返回错误信息
 */
TrtBackend::DetectionResult
    TrtBackend::detect_impl(const cv::Mat& image, ArmorColor color) noexcept {
    if (!impl_) {
        return std::unexpected("TensorRT backend not initialized");
    }

    if (image.empty()) {
        return std::unexpected("TensorRT backend received empty image");
    }

    // ==================== 1. Round-Robin调度 ====================
    // 使用原子变量实现无锁的round-robin调度，分配ExecutionContext和Stream
    uint32_t ctx_idx    = impl_->next_context_.fetch_add(1) % impl_->contexts_.size();
    auto* ctx           = impl_->contexts_[ctx_idx].get();
    cudaStream_t stream = impl_->streams_[ctx_idx];

    // ==================== 2. 计算Letterbox参数 ====================
    const int src_w = image.cols;
    const int src_h = image.rows;

    // 计算缩放因子（保持长宽比）
    float scale =
        std::min(static_cast<float>(INPUT_W) / src_w, static_cast<float>(INPUT_H) / src_h);
    int rw    = static_cast<int>(src_w * scale); // 缩放后的宽度
    int rh    = static_cast<int>(src_h * scale); // 缩放后的高度
    int pad_l = (INPUT_W - rw) / 2;              // 左侧padding
    int pad_t = (INPUT_H - rh) / 2;              // 顶部padding

    // ==================== 3. 图像格式处理 ====================
    // AT模型期望BGR输入（swap_rb=false）
    cv::Mat rgb_image;
    if (image.channels() == 3) {
        rgb_image = image; // 保持BGR格式
    } else if (image.channels() == 1) {
        cv::cvtColor(image, rgb_image, cv::COLOR_GRAY2BGR);
    } else {
        return std::unexpected("TensorRT backend received image with unsupported channel count");
    }

#if TALOS_HAS_CUDA_RUNTIME
    // ==================== 4. CUDA预处理（如果可用） ====================
    // 检查图像是否连续存储
    bool is_contiguous = (rgb_image.step == rgb_image.cols * rgb_image.elemSize());

    if (is_contiguous) {
        // 连续内存：直接异步上传
        cudaMemcpyAsync(
            impl_->device_input_, rgb_image.ptr<unsigned char>(),
            rgb_image.total() * rgb_image.elemSize(), cudaMemcpyHostToDevice, stream);

        // 启动letterbox CUDA kernel
        constexpr float NORM   = 1.0f / 255.0f;
        constexpr bool SWAP_RB = false; // AT模型期望BGR
        launch_letterbox_shared(
            static_cast<unsigned char*>(impl_->device_input_), src_w, src_h,
            static_cast<float*>(impl_->device_input_), INPUT_W, INPUT_H, scale, pad_t, pad_l, NORM,
            SWAP_RB, stream);
    } else {
        // 非连续内存（如ROI）：使用pitch上传
        size_t pitch = rgb_image.step;
        cudaMemcpyAsync(
            impl_->device_input_, rgb_image.ptr<unsigned char>(), rgb_image.rows * pitch,
            cudaMemcpyHostToDevice, stream);

        constexpr float NORM   = 1.0f / 255.0f;
        constexpr bool SWAP_RB = false;
        launch_letterbox_pitched(
            static_cast<unsigned char*>(impl_->device_input_), pitch, src_w, src_h,
            static_cast<float*>(impl_->device_input_), INPUT_W, INPUT_H, scale, pad_t, pad_l, NORM,
            SWAP_RB, stream);
    }
#else
    // ==================== 5. CPU预处理（备用方案） ====================
    cv::Mat letterboxed;
    cv::Size resized_size(rw, rh);

    // 使用OpenCV resize
    cv::resize(rgb_image, letterboxed, resized_size);

    // 创建填充后的图像
    cv::Mat final_image(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(114, 114, 114));

    // 将缩放后的图像复制到中心
    cv::Rect roi_rect(pad_l, pad_t, rw, rh);
    letterboxed.copyTo(final_image(roi_rect));

    // 转换为float并归一化（NCHW格式）
    std::vector<cv::Mat> channels(3);
    cv::split(final_image, channels);

    std::vector<float> host_input(INPUT_W * INPUT_H * 3);
    constexpr float NORM = 1.0f / 255.0f;
    for (int c = 0; c < 3; ++c) {
        for (int i = 0; i < INPUT_W * INPUT_H; ++i) {
            host_input[c * INPUT_W * INPUT_H + i] = channels[c].data[i] * NORM;
        }
    }

    // 拷贝到GPU
    cudaMemcpyAsync(
        impl_->device_input_, host_input.data(), INPUT_W * INPUT_H * 3 * sizeof(float),
        cudaMemcpyHostToDevice, stream);
#endif

    // ==================== 6. 执行推理 ====================
    // 使用TensorRT 8.5+ API
    ctx->setTensorAddress(impl_->input_name_.c_str(), impl_->device_input_);
    ctx->setTensorAddress(impl_->output_name_.c_str(), impl_->device_output_);
    bool success = ctx->enqueueV3(stream);
    if (!success) {
        SPDLOG_ERROR("TensorRT inference failed");
        return std::unexpected("TensorRT inference failed (enqueueV3)");
    }

    // ==================== 7. 异步拷贝输出 ====================
    cudaMemcpyAsync(
        impl_->host_output_.data(), impl_->device_output_, impl_->output_size_,
        cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream); // 同步等待推理完成

    // ==================== 8. 后处理 ====================
    std::vector<ArmorDetection> detections;

    // 输出格式：[1, max_det, 14]
    const float* output = impl_->host_output_.data();

    std::vector<cv::Rect2f> boxes;
    std::vector<float> confidences;
    std::vector<std::array<cv::Point2f, 4>> corners;
    std::vector<ClassMapping> class_infos;

    // 遍历所有检测
    for (int i = 0; i < impl_->max_detections_; ++i) {
        const float* row = output + i * OUTPUT_DIM;

        // 8.1 置信度过滤
        float conf = row[4];
        if (!std::isfinite(conf) || conf < config_.confidence_threshold) {
            continue;
        }

        // 8.2 提取边界框
        float x = row[0];
        float y = row[1];
        float w = row[2];
        float h = row[3];

        if (!std::isfinite(x) || !std::isfinite(y) || w <= 0.0f || h <= 0.0f) {
            continue;
        }

        // 8.3 提取类别ID
        int cls_id = static_cast<int>(row[5]);
        if (cls_id < 0 || cls_id >= 64) {
            continue;
        }

        // 8.4 类别解码（64类 → 颜色+数字）
        auto class_info = g_class_mappings[cls_id];

        // 8.5 提取并重排关键点
        std::array<cv::Point2f, 4> pts;
        for (int k = 0; k < 4; ++k) {
            float kx                   = row[6 + 2 * k];
            float ky                   = row[6 + 2 * k + 1];
            pts[Impl::keypoint_map[k]] = cv::Point2f(kx, ky);
        }

        // 8.6 坐标逆变换：从模型坐标映射回原图坐标
        for (auto& pt : pts) {
            pt.x = (pt.x - pad_l) / scale;
            pt.y = (pt.y - pad_t) / scale;
        }

        // 8.7 边界框坐标逆变换
        cv::Rect2f box(x - w / 2, y - h / 2, w, h);
        box.x = (box.x - pad_l) / scale;
        box.y = (box.y - pad_t) / scale;
        box.width /= scale;
        box.height /= scale;

        boxes.push_back(box);
        confidences.push_back(conf);
        corners.push_back(pts);
        class_infos.push_back(class_info);
    }

    // ==================== 9. 应用NMS ====================
    if (!boxes.empty()) {
        // 9.1 按置信度降序排序
        std::vector<int> indices(boxes.size());
        std::ranges::iota(indices.begin(), indices.end(), 0);
        std::ranges::sort(indices.begin(), indices.end(), [&confidences](int a, int b) {
            return confidences[a] > confidences[b];
        });

        // 9.2 重排检测结果
        std::vector<cv::Rect2f> sorted_boxes(boxes.size());
        std::vector<std::array<cv::Point2f, 4>> sorted_corners(boxes.size());
        std::vector<ClassMapping> sorted_classes(boxes.size());
        std::vector<float> sorted_confidences(boxes.size());

        for (size_t i = 0; i < indices.size(); ++i) {
            sorted_boxes[i]       = boxes[indices[i]];
            sorted_corners[i]     = corners[indices[i]];
            sorted_classes[i]     = class_infos[indices[i]];
            sorted_confidences[i] = confidences[indices[i]];
        }

        // 9.3 执行NMS
        std::vector<int> keep =
            nms_merge_sorted_bboxes(sorted_boxes, sorted_confidences, config_.nms_threshold);

        // 9.4 构建最终检测结果
        detections.reserve(keep.size());
        for (int idx : keep) {
            detections.emplace_back(
                sorted_corners[idx], sorted_classes[idx].name, sorted_classes[idx].color,
                sorted_confidences[idx]);
        }

        // 9.5 应用Top-K限制
        if (static_cast<int>(detections.size()) > config_.top_k) {
            detections.resize(config_.top_k);
        }
    }

    return detections;
}

} // namespace fcs::L2
