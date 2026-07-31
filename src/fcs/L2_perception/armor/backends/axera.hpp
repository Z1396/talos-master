#pragma once

/**
 * @file axera.hpp
 * @brief AX650 NPU 推理后端头文件
 *
 * 本文件实现了基于 AX650 芯片的神经网络推理后端，专用于装甲板检测任务。
 *
 * 核心特性：
 * 1. AX650 NPU 硬件推理：利用专用 AI 加速芯片实现高效模型推理
 * 2. IVPS 硬件预处理：使用 Image Video Processing System 进行硬件加速的图像预处理
 *    - 支持硬件 resize、color space conversion 等操作
 *    - 相比 OpenCV CPU 实现，延迟更低，抖动更小（p99 降低 2-3ms）
 * 3. concat-predecode 输出格式：模型输出已经过预解码，需要特殊解码逻辑
 * 4. tile-based NMS：基于空间分块的非极大值抑制算法，优化检测后处理性能
 *
 * 与 TensorRT/ONNXRuntime 后端的区别：
 * - 专用硬件：运行在 AX650 芯片上，而非通用 GPU
 * - 硬件预处理：集成 IVPS 专用硬件进行预处理，减少 CPU 负载
 * - 部署环境：适用于嵌入式边缘设备，功耗低、集成度高
 * - 输出格式：使用 concat-predecode 格式，需要在 CPU 上完成部分解码
 *
 * 适用场景：
 * - 嵌入式机器人视觉系统（如自瞄系统）
 * - 边缘 AI 推理应用
 * - 需要低功耗、小体积部署的视觉系统
 */

#include "../config.hpp"
#include "base.hpp"
#include "core/armor_types.hpp"

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

// AX650 NPU API - Axera AI 引擎核心接口
#include "ax_engine_api.h"
// AX650 系统级 API - 内存管理、设备初始化等
#include "ax_sys_api.h"

// AX650 IVPS (Image Video Processing System) API - 硬件加速预处理接口
// IVPS 是 AX650 芯片内置的图像/视频处理硬件单元，可实现零 CPU 负载的图像变换
#include "ax_ivps_api.h"
#include "ax_ivps_type.h"

namespace quanta {
class AxSysModule;
}

namespace fcs::L2 {

// ============================================================================
// AX650 NPU 推理后端实现
// ============================================================================

/**
 * @class AxeraBackend
 * @brief 基于 AX650 NPU 的神经网络检测后端
 *
 * 该类封装了 AX650 芯片的 NPU 推理能力，提供了完整的装甲板检测流水线：
 *
 * 核心流程：
 * 1. 硬件初始化（init）
 *    - 加载 .axmodel 模型文件到 NPU
 *    - 分配 NPU 输入/输出缓冲区（物理地址连续的 DMA 缓冲区）
 *    - 初始化 IVPS 硬件预处理单元（可选）
 *    - 构建 anchor grid 用于后处理解码
 *
 * 2. 图像预处理（preprocess）
 *    - IVPS 硬件路径（优先）：使用 IVPS 硬件进行 resize、BGR2RGB
 *      - 优势：零 CPU 负载，延迟低且稳定（消除 cache flush 系统调用抖动）
 *      - 实现：直接将 IVPS 输出写入 NPU 输入缓冲区，避免中间拷贝
 *    - OpenCV CPU 路径（后备）：当 IVPS 初始化失败或输入不合法时降级
 *
 * 3. NPU 推理（infer）
 *    - 同步执行模型推理（AX_ENGINE_RunSync）
 *    - 输入：resize 后的 BGR/RGB 图像张量
 *    - 输出：concat-predecode 格式的特征张量
 *
 * 4. 后处理解码（postprocess_concat_predecode）
 *    - 解析 concat-predecode 输出格式
 *    - anchor-based 关键点解码（关键点坐标 + grid offset + stride）
 *    - 颜色分类、pair_id 分类
 *    - 置信度过滤、top-k 选择
 *
 * 5. 非极大值抑制（nms）
 *    - tile-based NMS：将图像划分为 2x2 分块，只在相邻分块间计算 IoU
 *    - 优化：减少 IoU 计算量，从 O(N²) 降低到 O(N)（N 为检测数）
 *
 * 资源管理：
 * - RAII：所有硬件资源在析构函数中自动释放
 * - Move-only：禁止拷贝，只允许移动语义（硬件资源不可共享）
 * - 异常安全：构造失败时自动清理已分配资源
 *
 * 性能特性：
 * - 延迟：硬件预处理路径 p99 < 5ms（含推理）
 * - 内存：使用物理连续缓冲区，避免拷贝
 * - 并发：支持 NPU 并行运行模式（nParallelRun=1）
 */
class AxeraBackend : public DetectorBackendBase<AxeraBackend> {
public:
    using DetectionResult = std::expected<std::vector<ArmorDetection>, std::string>;
    using Config          = ArmorAxeraConfig;

    /**
     * @brief 工厂方法：创建并初始化 AX650 NPU 后端
     *
     * 实现了 "Construction IS Initialization" 模式，保证返回的对象已完全初始化。
     *
     * 初始化流程：
     * 1. 检查模型文件是否存在
     * 2. 初始化 AxSysModule（系统级模块，管理 NPU/IVPS 共享资源）
     * 3. 初始化 IVPS 硬件预处理单元（可选，失败后降级到 OpenCV）
     * 4. 初始化 AX_ENGINE NPU 引擎
     * 5. 加载 .axmodel 模型文件
     * 6. 创建推理上下文、分配输入/输出缓冲区
     * 7. 构建 anchor grid 用于后处理解码
     * 8. 执行 warmup 推理（预热 NPU，避免首次推理延迟）
     *
     * @param config 配置参数，包含模型路径、输入尺寸、阈值等
     * @return 成功返回初始化完成的后端对象，失败返回错误信息
     *
     * 错误处理：
     * - 模型文件不存在
     * - NPU 初始化失败（驱动问题、权限问题）
     * - IVPS 初始化失败（会降级到 OpenCV，不返回错误）
     * - 内存分配失败（NPU/IVPS 缓冲区）
     * - 模型加载失败（格式错误、不兼容）
     */
    [[nodiscard]] static std::expected<AxeraBackend, std::string> create(Config config) noexcept;

    /**
     * @brief 析构函数：释放所有硬件资源
     *
     * 资源释放顺序：
     * 1. 释放 NPU 输入/输出缓冲区（free_io）
     * 2. 销毁 AX_ENGINE 句柄
     * 3. 反初始化 AX_ENGINE
     * 4. 反初始化 IVPS
     * 5. 释放 AxSysModule 引用
     */
    ~AxeraBackend();

    // 禁止拷贝语义：硬件资源不可共享
    AxeraBackend(AxeraBackend&&) noexcept;
    AxeraBackend& operator=(AxeraBackend&&) noexcept;
    AxeraBackend(const AxeraBackend&)            = delete;
    AxeraBackend& operator=(const AxeraBackend&) = delete;

    /**
     * @brief 执行装甲板检测（同步接口）
     *
     * 检测流程：
     * 1. 计算预处理上下文（缩放比例、原始尺寸）
     * 2. 执行预处理（IVPS 硬件或 OpenCV CPU）
     * 3. 执行 NPU 推理
     * 4. 解码 concat-predecode 输出
     * 5. 执行 tile-based NMS
     *
     * @param image 输入图像（BGR 格式，CV_8UC3）
     * @param color 目标颜色过滤（只返回指定颜色的装甲板）
     * @return 成功返回检测列表，失败返回错误信息
     *
     * 线程安全：单实例不可跨线程调用（NPU 资源非线程安全）
     */
    [[nodiscard]] DetectionResult detect_impl(const cv::Mat& image, ArmorColor color) noexcept;

    /// 获取当前配置参数
    [[nodiscard]] const Config& get_config() const noexcept { return config_; }

private:
    /// 私有构造函数，使用工厂方法创建
    explicit AxeraBackend(Config config) noexcept;

    /**
     * @brief 初始化 NPU 和 IVPS 硬件资源
     *
     * 执行完整的硬件初始化流程，由工厂方法调用。
     *
     * @return 成功返回 void，失败返回错误信息
     */
    [[nodiscard]] std::expected<void, std::string> init() noexcept;

    /**
     * @struct PreprocContext
     * @brief 预处理上下文，记录图像变换参数
     *
     * 用于后处理时将模型坐标映射回原始图像坐标。
     */
    struct PreprocContext {
        ArmorColor detect_color;    ///< 目标检测颜色（颜色过滤）
        float scale_x      = 1.0f;  ///< X 轴缩放比例：orig_w / input_width
        float scale_y      = 1.0f;  ///< Y 轴缩放比例：orig_h / input_height
        int orig_w         = 0;     ///< 原始图像宽度
        int orig_h         = 0;     ///< 原始图像高度
        bool is_four_three = false; ///< 原始图像是否为 4:3 宽高比
    };

    /**
     * @brief 构建预处理上下文
     *
     * 计算从原始图像到模型输入尺寸的缩放参数，用于后处理坐标映射。
     *
     * @param image 输入图像
     * @param color 目标检测颜色
     * @return 预处理上下文
     */
    [[nodiscard]] PreprocContext preprocess(const cv::Mat& image, ArmorColor color) const noexcept;

    /**
     * @brief OpenCV CPU 预处理（后备路径）
     *
     * 使用 cv::resize 进行图像缩放，直接写入 NPU 输入缓冲区。
     * 适用于 IVPS 初始化失败或输入不合法的情况。
     *
     * 性能：p50 ~0.5ms 增加，但无系统调用抖动。
     *
     * @param image 输入图像（BGR 格式）
     */
    void preprocess_image(const cv::Mat& image) noexcept;

    /**
     * @brief 执行 NPU 推理
     *
     * 同步执行模型推理，假设输入缓冲区已准备就绪。
     *
     * @return 成功返回 true，失败返回 false
     */
    [[nodiscard]] bool infer() noexcept;

    /**
     * @struct DecodeGrid
     * @brief Anchor Grid 用于 concat-predecode 输出解码
     *
     * concat-predecode 模型的输出不包含 anchor grid 信息，
     * 需要在 CPU 上预先计算并缓存，用于关键点坐标解码。
     *
     * 解码公式：coord = (raw_coord + grid_offset) * stride * scale
     * - raw_coord: 模型输出的原始坐标（相对于 anchor 的偏移）
     * - grid_offset: anchor 中心偏移（0.5）
     * - stride: 特征图下采样步长（8/16/32）
     * - scale: 预处理缩放比例
     */
    struct DecodeGrid {
        std::vector<float> grid_x;  ///< 所有 anchor 的 X 坐标（grid 单位）
        std::vector<float> grid_y;  ///< 所有 anchor 的 Y 坐标（grid 单位）
        std::vector<float> strides; ///< 所有 anchor 对应的 stride
    };

    /**
     * @struct DecodedCandidate
     * @brief 解码后的候选检测
     *
     * 存储单个检测的解码结果，用于后续 NMS 处理。
     */
    struct DecodedCandidate {
        std::array<cv::Point2f, 4> corners{};   ///< 四个角点坐标（原始图像坐标系）
        ArmorName name   = ArmorName::Invalid;  ///< 装甲板类型名称
        ArmorColor color = ArmorColor::Neutral; ///< 装甲板颜色
        float confidence = 0.0f;                ///< 置信度（sigmoid 后）
        int pair_id      = -1;                  ///< pair_id（用于 NMS 分类）
    };

    /**
     * @struct RankedAnchor
     * @brief 排序后的 anchor（用于 top-k 选择）
     *
     * 在后处理第一阶段，筛选出置信度最高的 top-k anchors。
     */
    struct RankedAnchor {
        float best_obj_logit = -std::numeric_limits<float>::infinity(); ///< 最高的 obj logit
        int pair_id          = -1;                                      ///< 对应的 pair_id
        int anchor_idx       = -1;                                      ///< anchor 索引
    };

    /// Tile-based NMS 的分块尺寸（2x2 分块）
    static constexpr int TILE_GRID_SIZE = 2;

    /**
     * @brief 构建 anchor grid（初始化时调用一次）
     *
     * 遍历所有 stride（8/16/32），为每个 anchor 计算其 grid 坐标。
     * 结果缓存到 cached_grid_ 成员变量，避免每帧重复计算。
     *
     * @return DecodeGrid 结构，包含所有 anchor 的 grid 坐标和 stride
     */
    [[nodiscard]] DecodeGrid build_decode_grid() const noexcept;

    /// 缓存的 anchor grid（初始化时构建，后续推理复用）
    DecodeGrid cached_grid_;

    // 可复用的后处理缓冲区（mutable: 逻辑常量性不受影响）
    mutable std::vector<RankedAnchor> postproc_ranked_buf_;         ///< top-k 排序缓冲区
    mutable std::vector<DecodedCandidate> postproc_candidates_buf_; ///< 候选检测缓冲区
    mutable std::vector<uint8_t> nms_removed_buf_;                  ///< NMS 移除标记缓冲区
    mutable std::array<std::vector<int>, TILE_GRID_SIZE * TILE_GRID_SIZE>
        nms_tile_bins_buf_;                                         ///< 分块 bins
    mutable std::vector<ArmorDetection> nms_result_buf_;            ///< NMS 结果缓冲区

    /**
     * @brief 解码 concat-predecode 输出并执行后处理
     *
     * concat-predecode 格式说明：
     * 模型输出已预解码为 [num_anchors, output_dim] 形状，按通道排列：
     * - raw_boxes [4]: 边界框偏移（不使用，改为关键点）
     * - color_logits [num_colors]: 颜色分类 logits
     * - obj_logits [num_pairs]: pair_id 分类 logits
     * - raw_kpts [num_kpts * 2]: 关键点坐标偏移
     *
     * 解码算法：
     * 1. 遍历所有 anchor，找出 obj_logits 最高的 pair_id
     * 2. 应用 logit threshold 过滤低置信度检测
     * 3. 使用最小堆维护 top-k 检测
     * 4. 解码关键点坐标：kpt = (raw_kpt + grid_offset) * stride * scale
     * 5. 应用颜色过滤（只保留目标颜色的检测）
     * 6. 执行 tile-based NMS
     *
     * 性能优化：
     * - logit threshold 先于 sigmoid（避免计算低置信度检测）
     * - 预分配缓冲区（避免动态内存分配）
     * - 复用解码 grid（避免重复计算）
     *
     * @param ctx 预处理上下文
     * @return 检测结果列表
     */
    [[nodiscard]] std::vector<ArmorDetection>
        postprocess_concat_predecode(const PreprocContext& ctx) const noexcept;

    /**
     * @brief 基于分块的非极大值抑制（Tile-based NMS）
     *
     * 算法原理：
     * 1. 将图像划分为 TILE_GRID_SIZE x TILE_GRID_SIZE 分块（2x2）
     * 2. 根据检测中心点分配到对应分块
     * 3. 对每个检测，只检查相邻分块（3x3 窗口）内的其他检测
     * 4. 计算 IoU，移除重叠度高的检测
     *
     * 优势：
     * - 减少计算量：从 O(N²) 降低到 O(N)（N 为检测数）
     * - 空间局部性：相邻分块的数据在内存中连续，缓存友好
     * - 并行友好：可扩展到多线程实现
     *
     * IoU 计算方式：
     * - 使用四边形边界框计算 IoU（近似）
     * - 只在相同 pair_id 间计算 IoU（避免不同类别互相抑制）
     *
     * @param detections 候选检测列表
     * @param orig_w 原始图像宽度
     * @param orig_h 原始图像高度
     * @return NMS 后的检测结果
     */
    [[nodiscard]] std::vector<ArmorDetection>
        nms(std::vector<DecodedCandidate>& detections, int orig_w, int orig_h) const noexcept;

private:
    Config config_;                                      ///< 配置参数
    bool initialized_                 = false;           ///< 是否已初始化
    bool warned_non_four_three_input_ = false;           ///< 是否已警告非 4:3 输入
    std::shared_ptr<quanta::AxSysModule> ax_sys_module_; ///< 系统模块引用（管理共享资源）

    // AX650 NPU 资源
    AX_ENGINE_HANDLE handle_ = nullptr; ///< AX_ENGINE 句柄
    AX_ENGINE_IO_T io_data_{};          ///< 输入/输出缓冲区描述符
    std::vector<char> model_buffer_;    ///< 模型文件内存缓冲区

    // IVPS (Image Video Processing System) 硬件预处理资源
    /**
     * IVPS 硬件预处理原理：
     * - AX650 芯片内置专用图像处理硬件，可独立执行 resize、color conversion 等操作
     * - CPU 只需将图像数据拷贝到物理连续缓冲区，配置 IVPS 参数，启动硬件
     * - IVPS 通过 DMA 直接访问内存，无需 CPU 参与
     * - 输出可直接写入 NPU 输入缓冲区，实现零拷贝流水线
     *
     * 优势：
     * - 零 CPU 负载：预处理阶段 CPU 可执行其他任务
     * - 低延迟：硬件加速比 CPU 快 2-3 倍
     * - 低抖动：消除 cache flush 系统调用的不确定延迟（p99 降低 2-3ms）
     *
     * 内存布局：
     * - 源缓冲区（ivps_src_*）：存储原始图像，物理地址连续，stride 对齐
     * - 目标缓冲区：直接使用 NPU 输入缓冲区，避免中间拷贝
     */
    bool ivps_initialized_       = false;        ///< IVPS 是否已初始化
    AX_U64 ivps_src_phy_addr_    = 0;            ///< 源缓冲区物理地址（DMA 访问）
    AX_VOID* ivps_src_vir_addr_  = nullptr;      ///< 源缓冲区虚拟地址（CPU 访问）
    AX_U32 ivps_src_buffer_size_ = 0;            ///< 源缓冲区大小（字节）
    AX_U32 ivps_src_stride_      = 0;            ///< 源缓冲区 stride（对齐后）
    int ivps_max_src_width_      = 0;            ///< 最大源图像宽度
    int ivps_max_src_height_     = 0;            ///< 最大源图像高度
    AX_IVPS_ASPECT_RATIO_T ivps_aspect_ratio_{}; ///< 缓存的宽高比配置

    /// 初始化 IVPS 硬件预处理单元
    [[nodiscard]] std::expected<void, std::string> init_ivps() noexcept;
    /// 反初始化 IVPS
    void deinit_ivps() noexcept;
    /// 分配 IVPS 缓冲区
    [[nodiscard]] std::expected<void, std::string>
        allocate_ivps_buffers(int max_src_width, int max_src_height) noexcept;
    /// 释放 IVPS 缓冲区
    void free_ivps_buffers() noexcept;
    /// 使用 IVPS 硬件预处理图像
    void preprocess_image_ivps(const cv::Mat& image) noexcept;
};

} // namespace fcs::L2
