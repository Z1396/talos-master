/**
 * @file axera.cpp
 * @brief AX650 NPU 推理后端实现文件
 *
 * 本文件实现了基于 AX650 芯片的神经网络推理后端，包含完整的硬件初始化、
 * IVPS 硬件预处理、NPU 推理、后处理解码和 tile-based NMS 等核心逻辑。
 *
 * 主要模块：
 * 1. 硬件资源管理（RAII 封装）
 * 2. IVPS 硬件预处理（零 CPU 负载）
 * 3. concat-predecode 输出解码算法
 * 4. tile-based NMS 算法实现
 *
 * 关键技术点：
 * - 物理连续缓冲区管理：NPU/IVPS 需要 DMA 可访问的物理连续内存
 * - 非缓存缓冲区策略：消除 cache flush 系统调用抖动
 * - anchor-based 解码：concat-predecode 模型的特殊解码逻辑
 * - 空间分块 NMS：优化 IoU 计算复杂度
 */

#include "L2_perception/armor/backends/axera.hpp"

#include "L2_perception/armor/config.hpp"
#include "core/armor_types.hpp"
#include "core/types.hpp"
#include "quanta/encode_backend/ax/ax_venc_raii.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace fcs::L2 {

namespace {

namespace {
/**
 * @brief 将 AX650 模块 ID 转换为可读字符串
 *
 * AX650 芯片包含多个硬件模块，每个模块有独立的错误码空间。
 * 该函数用于错误诊断，将模块 ID 转换为人类可读的名称。
 *
 * 模块列表：
 * - ISP: Image Signal Processor（图像信号处理器）
 * - NPU: Neural Processing Unit（神经网络处理单元）
 * - IVPS: Image Video Processing System（图像视频处理系统）
 * - VENC/VDEC: Video Encoder/Decoder（视频编解码器）
 * - SYS: System（系统级模块）
 */
[[nodiscard]] const char* ax_module_name(int id) noexcept {
    switch (id) {
    case 0x01: return "AX_ID_ISP";
    case 0x02: return "AX_ID_CE";
    case 0x03: return "AX_ID_VO";
    case 0x04: return "AX_ID_VDSP";
    case 0x06: return "AX_ID_NPU";
    case 0x07: return "AX_ID_VENC";
    case 0x08: return "AX_ID_VDEC";
    case 0x09: return "AX_ID_JENC";
    case 0x0A: return "AX_ID_JDEC";
    case 0x0B: return "AX_ID_SYS";
    case 0x0D: return "AX_ID_IVPS";
    case 0x12: return "AX_ID_USER";
    default: return "AX_ID_?";
    }
}

/**
 * @brief 将 AX650 错误码转换为可读字符串
 *
 * AX650 SDK 定义了统一的错误码格式：
 * - 高 16 位：模块 ID（module）
 * - 中 8 位：子模块 ID（sub_module）
 * - 低 8 位：错误类型（err_id）
 *
 * 常见错误类型：
 * - ILLEGAL_PARAM: 参数非法（如奇数尺寸传给 IVPS）
 * - NOT_INIT: 模块未初始化
 * - NOMEM: 内存不足
 * - NOBUF: 缓冲区不足
 * - TIMED_OUT: 超时
 */
[[nodiscard]] const char* ax_err_name(int id) noexcept {
    switch (id) {
    case 0x01: return "AX_ERR_INVALID_MODID";
    case 0x02: return "AX_ERR_INVALID_DEVID";
    case 0x03: return "AX_ERR_INVALID_GRPID";
    case 0x04: return "AX_ERR_INVALID_CHNID";
    case 0x05: return "AX_ERR_INVALID_PIPEID";
    case 0x0A: return "AX_ERR_ILLEGAL_PARAM";
    case 0x0B: return "AX_ERR_NULL_PTR";
    case 0x0C: return "AX_ERR_BAD_ADDR";
    case 0x10: return "AX_ERR_SYS_NOTREADY";
    case 0x11: return "AX_ERR_BUSY";
    case 0x12: return "AX_ERR_NOT_INIT";
    case 0x13: return "AX_ERR_NOT_CONFIG";
    case 0x14: return "AX_ERR_NOT_SUPPORT";
    case 0x15: return "AX_ERR_NOT_PERM";
    case 0x16: return "AX_ERR_EXIST";
    case 0x17: return "AX_ERR_UNEXIST";
    case 0x18: return "AX_ERR_NOMEM";
    case 0x19: return "AX_ERR_NOBUF";
    case 0x1A: return "AX_ERR_NOT_MATCH";
    case 0x20: return "AX_ERR_BUF_EMPTY";
    case 0x21: return "AX_ERR_BUF_FULL";
    case 0x22: return "AX_ERR_QUEUE_EMPTY";
    case 0x23: return "AX_ERR_QUEUE_FULL";
    case 0x27: return "AX_ERR_TIMED_OUT";
    case 0x28: return "AX_ERR_FLOW_END";
    case 0x29: return "AX_ERR_UNKNOWN";
    default: return "AX_ERR_?";
    }
}
} // namespace

/**
 * @brief 将 AX650 错误码转换为详细的错误信息字符串
 *
 * 解析错误码的各个字段，生成人类可读的错误信息。
 *
 * 错误码格式：0xMMSSSEEE
 * - MM: 模块 ID
 * - SSS: 子模块 ID
 * - EEE: 错误类型
 *
 * @param err AX650 API 返回的错误码（负值表示错误）
 * @return 格式化的错误字符串，如 "AX_ID_NPU/0 AX_ERR_ILLEGAL_PARAM(0x0A) [raw=0x0606000A]"
 */
std::string ax_error_string(AX_S32 err) noexcept {
    if (err >= 0)
        return std::format("OK(0x{:X})", static_cast<unsigned>(err));

    const unsigned code       = static_cast<unsigned>(err);
    const unsigned module     = (code >> 16) & 0xFF;
    const unsigned sub_module = (code >> 8) & 0xFF;
    const unsigned err_id     = code & 0xFF;

    return fmt::format(
        "{}/{} {}(0x{:02X}) [raw=0x{:08X}]", ax_module_name(static_cast<int>(module)), sub_module,
        ax_err_name(static_cast<int>(err_id)), err_id, code);
}

/**
 * @brief 快速 sigmoid 函数（带提前过滤优化）
 *
 * sigmoid(x) = 1 / (1 + exp(-x))
 *
 * 优化策略：
 * - 当 x > 6.0 时，sigmoid(x) ≈ 1.0（误差 < 0.0025）
 * - 当 x < -6.0 时，sigmoid(x) ≈ 0.0（误差 < 0.0025）
 * - 提前过滤避免昂贵的 exp 计算
 *
 * 性能：相比标准 sigmoid，减少约 60% 的 exp 调用。
 *
 * @param x 输入值（logit）
 * @return sigmoid 后的值（范围 [0, 1]）
 */
inline float fast_sigmoid(float x) noexcept {
    if (x > 6.0f)
        return 1.0f;
    if (x < -6.0f)
        return 0.0f;
    return 1.0f / (1.0f + std::exp(-x));
}

/// AX650 内存分配对齐要求（字节）：DMA 传输需要物理地址对齐
constexpr int AX_CMM_ALIGN_SIZE = 128;
/// AX650 内存分配会话名称：用于内存池管理
constexpr const char* AX_CMM_SESSION_NAME = "ax-samples-cmm";
/// Anchor grid 中心偏移：用于解码 concat-predecode 输出
constexpr float GRID_OFFSET = 0.5f;
/// 4:3 宽高比常量
constexpr float FOUR_THIRTS = 4.0f / 3.0f;
/// 4:3 宽高比检测容差
constexpr float FOUR_THIRTS_TOLERANCE = 0.01f;
/// 关键点顺序映射：将模型输出的关键点顺序转换为标准顺序
/// 模型输出顺序：左上、右上、右下、左下（0, 1, 2, 3）
/// 标准顺序：左上、左下、右下、右上（0, 3, 2, 1）
constexpr std::array<int, 4> KPT_ORDER = {0, 3, 2, 1};

/**
 * @enum AX_ENGINE_ALLOC_BUFFER_STRATEGY_T
 * @brief NPU 缓冲区分配策略
 *
 * - DEFAULT: 使用硬件默认策略
 * - CACHED: 使用缓存缓冲区（需要手动 flush）
 *
 * 本实现使用非缓存策略（DEFAULT）避免 cache flush 系统调用抖动。
 */
enum class AX_ENGINE_ALLOC_BUFFER_STRATEGY_T {
    DEFAULT = 0,
    CACHED  = 1,
};

/**
 * @brief pair_id 到 ArmorName 的映射表
 *
 * pair_id 是模型输出的分类 ID，对应不同的装甲板类型。
 * 编码规则：
 * - 0-5: 小装甲板（s 后缀）
 * - 6-11: 大装甲板（b 后缀）
 *
 * 特殊说明：
 * - pair_id 0 和 6 都映射到 Sentry（哨兵）
 * - pair_id 5 和 11 分别对应前哨站和基地
 */
static constexpr std::array<ArmorName, 12> PAIR_TO_ARMOR{
    ArmorName::Sentry,   // 0: Gs (Guard small)
    ArmorName::Two,      // 1: 2s
    ArmorName::Three,    // 2: 3s
    ArmorName::Four,     // 3: 4s
    ArmorName::Five,     // 4: 5s
    ArmorName::Outpost,  // 5: Os (Outpost small)
    ArmorName::Sentry,   // 6: Gb (Guard big)
    ArmorName::One,      // 7: 1b
    ArmorName::Three,    // 8: 3b
    ArmorName::Four,     // 9: 4b
    ArmorName::Five,     // 10: 5b
    ArmorName::BaseLarge // 11: Bb (Base big)
};

// ====================================================================
// AXERA NPU 内存管理（手动管理策略）
// ====================================================================
//
// 背景：Axera SDK 使用 C API，要求使用 C 数组而非 std::vector
//
// C API 结构体定义：
//   typedef struct AX_ENGINE_IO_T {
//       AX_ENGINE_IO_BUFFER_T* pInputs;   // ← 需要 C 数组
//       uint32_t nInputSize;
//       AX_ENGINE_IO_BUFFER_T* pOutputs;  // ← 需要 C 数组
//       uint32_t nOutputSize;
//   } AX_ENGINE_IO_T;
//
// 为什么不能用 std::vector？
//   - C 结构体不能包含有非平凡构造函数的 C++ 对象
//   - std::vector 的内存布局与 C 数组不同
//   - 将 std::vector 传递给 C API 是未定义行为（UB）
//
// 所有权模型：
//   - AxeraBackend 类拥有 io_data_（RAII 管理）
//   - 析构函数调用 free_io()，内部使用 delete[] 释放 C 数组
//   - Move-only 语义防止双重释放
//   - 错误路径正确清理部分分配的资源
//
// 异常安全：
//   - ✅ 大部分错误路径正确清理
//   - ⚠️ 极端情况：如果第二个 new[] 抛出异常，第一个数组会泄漏（仅 OOM 时）
//
// 结论：由于 C API 约束，手动 new[] 在这里是可接受的
//
// ====================================================================

/**
 * @brief 释放指定索引之前的硬件内存
 *
 * 当分配失败时，需要清理已分配的前置缓冲区。
 *
 * @param io_buf 缓冲区数组指针
 * @param index 失败位置的索引（释放 [0, index) 范围）
 */
void free_io_index(AX_ENGINE_IO_BUFFER_T* io_buf, size_t index) noexcept {
    for (size_t i = 0; i < index; ++i) {
        AX_ENGINE_IO_BUFFER_T* p_buf = io_buf + i;
        AX_SYS_MemFree(p_buf->phyAddr, p_buf->pVirAddr);
    }
}

/**
 * @brief 释放 NPU 输入/输出缓冲区（RAII 清理函数）
 *
 * 清理流程：
 * 1. 遍历所有输入缓冲区，调用 AX_SYS_MemFree 释放物理内存
 * 2. 使用 delete[] 释放 C 数组（pInputs）
 * 3. 对输出缓冲区执行相同操作
 * 4. 重置 io 结构体
 *
 * 注意：必须使用 delete[] 而非 delete，因为使用 new[] 分配。
 */
void free_io(AX_ENGINE_IO_T* io) noexcept {
    if (io->pInputs != nullptr) {
        for (size_t j = 0; j < io->nInputSize; ++j) {
            AX_ENGINE_IO_BUFFER_T* p_buf = io->pInputs + j;
            AX_SYS_MemFree(p_buf->phyAddr, p_buf->pVirAddr);
        }
        delete[] io->pInputs;  // ← 删除 C 数组（非 vector！）
    }
    if (io->pOutputs != nullptr) {
        for (size_t j = 0; j < io->nOutputSize; ++j) {
            AX_ENGINE_IO_BUFFER_T* p_buf = io->pOutputs + j;
            AX_SYS_MemFree(p_buf->phyAddr, p_buf->pVirAddr);
        }
        delete[] io->pOutputs; // ← 删除 C 数组（非 vector！）
    }
    *io = {};
}

/**
 * @brief 准备 NPU 输入/输出缓冲区
 *
 * 分配并初始化 NPU 推理所需的硬件缓冲区。
 *
 * 分配策略：
 * 1. 输入缓冲区（非缓存）：
 *    - 使用 AX_SYS_MemAlloc 分配非缓存内存
 *    - 优势：消除 AX_SYS_MflushCache 系统调用抖动
 *    - 原理：NPU 通过 DMA 读取（物理地址），IVPS 通过 DMA 写入，都绕过缓存
 *    - CPU 回退路径（cv::resize）直接写入 DDR，吞吐量略降但无系统调用延迟
 *    - 性能影响：p50 可能增加 ~0.5ms，但 p99 降低 2-3ms（关键优化）
 *
 * 2. 输出缓冲区（缓存）：
 *    - 使用 AX_SYS_MemAllocCached 分配缓存内存
 *    - 原因：输出数据需要 CPU 读取（后处理），缓存加速读取
 *    - 注意：需要手动 flush cache（但本实现未 flush，因为只读）
 *
 * 3. 并行运行模式（nParallelRun=1）：
 *    - 启用 NPU 并行运行，重叠 DMA 输入传输与上一个子操作的执行
 *    - 可降低有效推理延迟
 *    - 不支持该特性的固件版本会忽略此设置
 *
 * @param info 模型输入/输出元信息
 * @param io_data 输入/输出缓冲区描述符（输出参数）
 * @param strategy 缓冲区分配策略（输入、输出）
 * @return 成功返回 void，失败返回错误信息
 */
std::expected<void, std::string> prepare_io(
    AX_ENGINE_IO_INFO_T* info, AX_ENGINE_IO_T* io_data,
    [[maybe_unused]] std::pair<AX_ENGINE_ALLOC_BUFFER_STRATEGY_T, AX_ENGINE_ALLOC_BUFFER_STRATEGY_T>
        strategy) noexcept {
    // 初始化结构体
    *io_data         = {};
    io_data->pInputs = new AX_ENGINE_IO_BUFFER_T[info->nInputSize];
    memset(io_data->pInputs, 0, sizeof(AX_ENGINE_IO_BUFFER_T) * info->nInputSize);
    io_data->nInputSize = info->nInputSize;

    // 分配输入缓冲区（非缓存）
    for (int i = 0; i < static_cast<int>(info->nInputSize); ++i) {
        auto meta     = info->pInputs[i];
        auto* buffer  = &io_data->pInputs[i];
        buffer->nSize = meta.nSize;

        const auto ret = AX_SYS_MemAlloc(
            reinterpret_cast<AX_U64*>(&buffer->phyAddr), &buffer->pVirAddr, meta.nSize,
            AX_CMM_ALIGN_SIZE, reinterpret_cast<const AX_S8*>(AX_CMM_SESSION_NAME));
        if (ret != 0) {
            free_io_index(io_data->pInputs, i);
            delete[] io_data->pInputs;
            io_data->pInputs = nullptr;
            return std::unexpected(
                "Axera NPU input buffer allocation failed for index " + std::to_string(i));
        }
    }

    // 分配输出缓冲区（缓存）
    io_data->pOutputs = new AX_ENGINE_IO_BUFFER_T[info->nOutputSize];
    memset(io_data->pOutputs, 0, sizeof(AX_ENGINE_IO_BUFFER_T) * info->nOutputSize);
    io_data->nOutputSize = info->nOutputSize;

    for (int i = 0; i < static_cast<int>(info->nOutputSize); ++i) {
        auto meta     = info->pOutputs[i];
        auto* buffer  = &io_data->pOutputs[i];
        buffer->nSize = meta.nSize;

        const auto ret = AX_SYS_MemAllocCached(
            reinterpret_cast<AX_U64*>(&buffer->phyAddr), &buffer->pVirAddr, meta.nSize,
            AX_CMM_ALIGN_SIZE, reinterpret_cast<const AX_S8*>(AX_CMM_SESSION_NAME));
        if (ret != 0) {
            free_io_index(io_data->pInputs, io_data->nInputSize);
            free_io_index(io_data->pOutputs, i);
            delete[] io_data->pInputs;
            delete[] io_data->pOutputs;
            io_data->pInputs  = nullptr;
            io_data->pOutputs = nullptr;
            return std::unexpected(
                "Axera NPU output buffer allocation failed for index " + std::to_string(i));
        }
    }

    // 启用 NPU 并行运行模式
    io_data->nParallelRun = 1;

    return {};
}

bool read_file(const std::string& path, std::vector<char>& data) {
    std::fstream fs(path, std::ios::in | std::ios::binary);
    if (!fs.is_open()) {
        return false;
    }

    fs.seekg(std::ios::end);
    const auto fs_end = fs.tellg();
    fs.seekg(std::ios::beg);
    const auto fs_beg = fs.tellg();

    const auto file_size = static_cast<size_t>(fs_end - fs_beg);
    data.reserve(file_size);
    data.insert(data.end(), std::istreambuf_iterator<char>(fs), std::istreambuf_iterator<char>());
    return true;
}

constexpr float clampf(float value, float lo, float hi) noexcept {
    return value < lo ? lo : (value > hi ? hi : value);
}

bool is_four_three(int width, int height) noexcept {
    if (width <= 0 || height <= 0) {
        return false;
    }
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    return std::fabs(aspect - FOUR_THIRDS) <= FOUR_THIRDS_TOLERANCE;
}

// O(1) bounding rect from 4 corner points — avoids cv::boundingRect allocation
cv::Rect2f bounding_rect_4pt(const std::array<cv::Point2f, 4>& pts) noexcept {
    const float x_min = std::min({pts[0].x, pts[1].x, pts[2].x, pts[3].x});
    const float x_max = std::max({pts[0].x, pts[1].x, pts[2].x, pts[3].x});
    const float y_min = std::min({pts[0].y, pts[1].y, pts[2].y, pts[3].y});
    const float y_max = std::max({pts[0].y, pts[1].y, pts[2].y, pts[3].y});
    return {x_min, y_min, x_max - x_min, y_max - y_min};
}

float pts_iou(const std::array<cv::Point2f, 4>& a, const std::array<cv::Point2f, 4>& b) noexcept {
    const cv::Rect2f ra = bounding_rect_4pt(a);
    const cv::Rect2f rb = bounding_rect_4pt(b);
    const float inter   = (ra & rb).area();
    if (inter <= 0.0f) {
        return 0.0f;
    }
    const float union_area = ra.area() + rb.area() - inter;
    return union_area > 0.0f ? inter / union_area : 0.0f;
}

/// 整数裁剪到指定范围
int clamp_int(int value, int lo, int hi) noexcept {
    return value < lo ? lo : (value > hi ? hi : value);
}

/**
 * @brief 向上对齐到指定边界
 *
 * @param value 输入值
 * @param alignment 对齐边界
 * @return 对齐后的值
 */
AX_U32 align_up_u32(AX_U32 value, AX_U32 alignment) noexcept {
    if (alignment == 0U) {
        return value;
    }
    return ((value + alignment - 1U) / alignment) * alignment;
}

/**
 * @brief 计算 BGR888 格式的对齐 stride
 *
 * BGR888 格式要求：
 * - 每像素 3 字节（B、G、R）
 * - stride 必须是 3 的倍数（像素对齐）
 * - stride 必须满足硬件对齐要求（48 字节，即 16 像素）
 *
 * 参考实现：ax-video-sdk src/common/ax_image_processor_ax650.cpp MakeAlignedDescriptor()
 *
 * @param width 图像宽度（像素）
 * @return 对齐后的 stride（字节）
 */
AX_U32 bgr888_stride_aligned(int width) noexcept {
    // RGB/BGR24 使用 16 像素对齐：16 * 3 = 48 字节
    // 这样 stride 仍然是 3 的倍数（像素对齐）
    constexpr AX_U32 ALIGN = 48U;
    return align_up_u32(static_cast<AX_U32>(std::max(width, 0)) * 3U, ALIGN);
}

/**
 * @brief 全局 IVPS 目标缓冲区（临时工作区）
 *
 * 注意：这是全局变量，假设只有一个 AxeraBackend 实例活跃。
 * 在当前使用场景下，这个假设成立（单后端实例）。
 *
 * 为什么使用全局变量？
 * - 避免 axera.hpp 添加成员变量（保持头文件简洁）
 * - IVPS 目标缓冲区与 NPU 张量缓冲区分开管理
 *
 * 未来改进：如果需要多实例，应改为成员变量。
 */
AX_U64 g_ivps_dst_phy_addr    = 0;
void* g_ivps_dst_vir_addr     = nullptr;
AX_U32 g_ivps_dst_buffer_size = 0;
AX_U32 g_ivps_dst_stride      = 0;
int g_ivps_dst_width          = 0;
int g_ivps_dst_height         = 0;

/// 释放全局 IVPS 目标缓冲区
void free_ivps_dst_buffer() noexcept {
    if (g_ivps_dst_vir_addr != nullptr && g_ivps_dst_phy_addr != 0) {
        AX_SYS_MemFree(g_ivps_dst_phy_addr, g_ivps_dst_vir_addr);
    }
    g_ivps_dst_phy_addr    = 0;
    g_ivps_dst_vir_addr    = nullptr;
    g_ivps_dst_buffer_size = 0;
    g_ivps_dst_stride      = 0;
    g_ivps_dst_width       = 0;
    g_ivps_dst_height      = 0;
}

/**
 * @brief 分配全局 IVPS 目标缓冲区
 *
 * 分配策略：
 * - 如果已有足够大的缓冲区，直接复用
 * - 否则释放旧缓冲区，分配新缓冲区
 * - 使用 4KB 对齐（0x1000）满足硬件要求
 *
 * @param width 目标宽度
 * @param height 目标高度
 * @return 成功返回 void，失败返回错误信息
 */
std::expected<void, std::string> allocate_ivps_dst_buffer(int width, int height) noexcept {
    if (width <= 0 || height <= 0) {
        return std::unexpected(fmt::format("invalid IVPS dst size: {}x{}", width, height));
    }

    const AX_U32 stride = bgr888_stride_aligned(width);
    const AX_U32 size   = stride * static_cast<AX_U32>(height);

    // 检查是否可复用现有缓冲区
    if (g_ivps_dst_vir_addr != nullptr && g_ivps_dst_width == width && g_ivps_dst_height == height
        && g_ivps_dst_stride == stride && g_ivps_dst_buffer_size >= size) {
        return {};
    }

    // 释放旧缓冲区
    free_ivps_dst_buffer();

    // 分配新缓冲区（4KB 对齐）
    constexpr AX_U32 ALIGN = 0x1000;
    const auto ret         = AX_SYS_MemAlloc(
        &g_ivps_dst_phy_addr, &g_ivps_dst_vir_addr, size, ALIGN,
        reinterpret_cast<const AX_S8*>("ivps-dst"));

    if (ret != AX_SUCCESS || g_ivps_dst_vir_addr == nullptr) {
        g_ivps_dst_phy_addr    = 0;
        g_ivps_dst_vir_addr    = nullptr;
        g_ivps_dst_buffer_size = 0;
        g_ivps_dst_stride      = 0;
        g_ivps_dst_width       = 0;
        g_ivps_dst_height      = 0;
        return std::unexpected(fmt::format("AX_SYS_MemAlloc ivps-dst: {}", ax_error_string(ret)));
    }

    g_ivps_dst_buffer_size = size;
    g_ivps_dst_stride      = stride;
    g_ivps_dst_width       = width;
    g_ivps_dst_height      = height;

    SPDLOG_INFO("IVPS dst buffer allocated: {}x{} stride={} size={}", width, height, stride, size);
    return {};
}

} // namespace

AxeraBackend::AxeraBackend(Config config) noexcept
    : config_(std::move(config)) {}

AxeraBackend::~AxeraBackend() {
    if (handle_ != nullptr) {
        free_io(&io_data_);
        AX_ENGINE_DestroyHandle(handle_);
        handle_ = nullptr;
        AX_ENGINE_Deinit();
    }
    deinit_ivps();
    ax_sys_module_.reset();
    initialized_ = false;
}

AxeraBackend::AxeraBackend(AxeraBackend&& other) noexcept
    : cached_grid_(std::move(other.cached_grid_))
    , postproc_ranked_buf_(std::move(other.postproc_ranked_buf_))
    , postproc_candidates_buf_(std::move(other.postproc_candidates_buf_))
    , nms_removed_buf_(std::move(other.nms_removed_buf_))
    , nms_tile_bins_buf_(std::move(other.nms_tile_bins_buf_))
    , config_(std::move(other.config_))
    , initialized_(other.initialized_)
    , warned_non_four_three_input_(other.warned_non_four_three_input_)
    , ax_sys_module_(std::move(other.ax_sys_module_))
    , handle_(other.handle_)
    , io_data_(other.io_data_)
    , model_buffer_(std::move(other.model_buffer_))
    , ivps_initialized_(other.ivps_initialized_)
    , ivps_src_phy_addr_(other.ivps_src_phy_addr_)
    , ivps_src_vir_addr_(other.ivps_src_vir_addr_)
    , ivps_src_buffer_size_(other.ivps_src_buffer_size_)
    , ivps_src_stride_(other.ivps_src_stride_)
    , ivps_max_src_width_(other.ivps_max_src_width_)
    , ivps_max_src_height_(other.ivps_max_src_height_)
    , ivps_aspect_ratio_(other.ivps_aspect_ratio_) {
    other.initialized_                 = false;
    other.warned_non_four_three_input_ = false;
    other.handle_                      = nullptr;
    other.io_data_                     = {};
    other.model_buffer_.clear();
    other.ivps_initialized_     = false;
    other.ivps_src_phy_addr_    = 0;
    other.ivps_src_vir_addr_    = nullptr;
    other.ivps_src_buffer_size_ = 0;
    other.ivps_src_stride_      = 0;
}

AxeraBackend& AxeraBackend::operator=(AxeraBackend&& other) noexcept {
    if (this == &other) {
        return *this;
    }

    if (handle_ != nullptr) {
        free_io(&io_data_);
        AX_ENGINE_DestroyHandle(handle_);
        AX_ENGINE_Deinit();
    }
    deinit_ivps();
    ax_sys_module_.reset();
    initialized_ = false;

    cached_grid_                 = std::move(other.cached_grid_);
    postproc_ranked_buf_         = std::move(other.postproc_ranked_buf_);
    postproc_candidates_buf_     = std::move(other.postproc_candidates_buf_);
    nms_removed_buf_             = std::move(other.nms_removed_buf_);
    nms_tile_bins_buf_           = std::move(other.nms_tile_bins_buf_);
    config_                      = std::move(other.config_);
    initialized_                 = other.initialized_;
    warned_non_four_three_input_ = other.warned_non_four_three_input_;
    ax_sys_module_               = std::move(other.ax_sys_module_);
    handle_                      = other.handle_;
    io_data_                     = other.io_data_;
    model_buffer_                = std::move(other.model_buffer_);
    ivps_initialized_            = other.ivps_initialized_;
    ivps_src_phy_addr_           = other.ivps_src_phy_addr_;
    ivps_src_vir_addr_           = other.ivps_src_vir_addr_;
    ivps_src_buffer_size_        = other.ivps_src_buffer_size_;
    ivps_src_stride_             = other.ivps_src_stride_;
    ivps_max_src_width_          = other.ivps_max_src_width_;
    ivps_max_src_height_         = other.ivps_max_src_height_;
    ivps_aspect_ratio_           = other.ivps_aspect_ratio_;

    other.initialized_                 = false;
    other.warned_non_four_three_input_ = false;
    other.handle_                      = nullptr;
    other.io_data_                     = {};
    other.model_buffer_.clear();
    other.ivps_initialized_     = false;
    other.ivps_src_phy_addr_    = 0;
    other.ivps_src_vir_addr_    = nullptr;
    other.ivps_src_buffer_size_ = 0;
    other.ivps_src_stride_      = 0;
    return *this;
}

// ============================================================================
// Factory — Construction IS Initialization
// ============================================================================

std::expected<AxeraBackend, std::string> AxeraBackend::create(Config config) noexcept {
    AxeraBackend backend(std::move(config));
    const auto& cfg = backend.get_config();
    if (!std::filesystem::exists(cfg.model_path)) {
        return std::unexpected(fmt::format("axmodel not found: {}", cfg.model_path));
    }

    auto init_result = backend.init();
    if (!init_result) {
        return std::unexpected(init_result.error());
    }

    return backend;
}

std::expected<void, std::string> AxeraBackend::init() noexcept {
    if (initialized_) {
        return {};
    }

    const auto& cfg = get_config();
    if (!std::filesystem::exists(cfg.model_path)) {
        return std::unexpected(fmt::format("axmodel not found: {}", cfg.model_path));
    }

    bool engine_initialized  = false;
    auto cleanup_failed_init = [&]() {
        if (io_data_.pInputs != nullptr || io_data_.pOutputs != nullptr) {
            free_io(&io_data_);
            io_data_ = {};
        }
        if (handle_ != nullptr) {
            AX_ENGINE_DestroyHandle(handle_);
            handle_ = nullptr;
        }
        if (engine_initialized) {
            AX_ENGINE_Deinit();
            engine_initialized = false;
        }
        deinit_ivps();
        ax_sys_module_.reset();
        initialized_ = false;
    };

    auto ax_sys_module = quanta::AxSysModule::acquire();
    if (!ax_sys_module) {
        return std::unexpected(std::move(ax_sys_module.error()));
    }
    ax_sys_module_ = std::move(*ax_sys_module);

    auto ivps = init_ivps();
    if (!ivps) {
        SPDLOG_WARN("init_ivps: {}, falling back to OpenCV preprocessing", ivps.error());
    } else {
        SPDLOG_INFO("IVPS initialized for hardware-accelerated resize");
    }

    AX_ENGINE_NPU_ATTR_T npu_attr{};
    npu_attr.eHardMode = AX_ENGINE_VIRTUAL_NPU_DISABLE;
    auto ret           = AX_ENGINE_Init(&npu_attr);
    if (ret != 0) {
        cleanup_failed_init();
        return std::unexpected(fmt::format("AX_ENGINE_Init: {}", ax_error_string(ret)));
    }
    engine_initialized = true;

    if (!read_file(cfg.model_path, model_buffer_)) {
        cleanup_failed_init();
        return std::unexpected(fmt::format("read {}: {}", cfg.model_path, ax_error_string(ret)));
    }

    ret = AX_ENGINE_CreateHandle(&handle_, model_buffer_.data(), model_buffer_.size());
    if (ret != 0) {
        cleanup_failed_init();
        return std::unexpected(fmt::format("AX_ENGINE_CreateHandle: {}", ax_error_string(ret)));
    }

    ret = AX_ENGINE_CreateContext(handle_);
    if (ret != 0) {
        cleanup_failed_init();
        return std::unexpected(fmt::format("AX_ENGINE_CreateContext: {}", ax_error_string(ret)));
    }

    AX_ENGINE_IO_INFO_T* io_info = nullptr;
    ret                          = AX_ENGINE_GetIOInfo(handle_, &io_info);
    if (ret != 0) {
        cleanup_failed_init();
        return std::unexpected(fmt::format("AX_ENGINE_GetIOInfo: {}", ax_error_string(ret)));
    }

    SPDLOG_INFO("axmodel: {}", cfg.model_path);
    const int predecode_output_dim = 4 + cfg.num_colors + cfg.num_pairs + cfg.num_kpts * 2;
    int expected_anchors           = 0;
    for (const int s : cfg.strides) {
        expected_anchors += (cfg.input_width / s) * (cfg.input_height / s);
    }
    SPDLOG_INFO(
        "expecting concat-predecode model: input={}x{}, output_dim={}, "
        "anchors={}",
        cfg.input_width, cfg.input_height, predecode_output_dim, expected_anchors);
    for (int i = 0; i < static_cast<int>(io_info->nInputSize); ++i) {
        const auto& in = io_info->pInputs[i];
        SPDLOG_INFO("  input[{}]: name={}, size={} bytes", i, in.pName, in.nSize);
    }
    for (int i = 0; i < static_cast<int>(io_info->nOutputSize); ++i) {
        const auto& out = io_info->pOutputs[i];
        SPDLOG_INFO("  output[{}]: name={}, size={} bytes", i, out.pName, out.nSize);
    }

    auto io_result = prepare_io(
        io_info, &io_data_,
        std::make_pair(
            AX_ENGINE_ALLOC_BUFFER_STRATEGY_T::CACHED, AX_ENGINE_ALLOC_BUFFER_STRATEGY_T::CACHED));
    if (!io_result) {
        cleanup_failed_init();
        return std::unexpected(io_result.error());
    }

    cached_grid_ = build_decode_grid();
    SPDLOG_INFO("cached decode grid with {} anchors", cached_grid_.grid_x.size());

    if (io_data_.nInputSize > 0 && io_data_.pInputs[0].pVirAddr != nullptr) {
        auto* warmup_input = static_cast<uint8_t*>(io_data_.pInputs[0].pVirAddr);
        std::memset(warmup_input, 114, static_cast<size_t>(io_data_.pInputs[0].nSize));
        // Non-cached buffer: no flush needed.
    }
    ret = AX_ENGINE_RunSync(handle_, &io_data_);
    if (ret != AX_SUCCESS) {
        cleanup_failed_init();
        return std::unexpected(fmt::format("warmup AX_ENGINE_RunSync: {}", ax_error_string(ret)));
    }

    initialized_ = true;
    return {};
}

AxeraBackend::PreprocContext
    AxeraBackend::preprocess(const cv::Mat& image, ArmorColor color) const noexcept {
    PreprocContext ctx;
    ctx.orig_w        = image.cols;
    ctx.orig_h        = image.rows;
    ctx.scale_x       = static_cast<float>(image.cols) / static_cast<float>(config_.input_width);
    ctx.scale_y       = static_cast<float>(image.rows) / static_cast<float>(config_.input_height);
    ctx.is_four_three = is_four_three(image.cols, image.rows);
    ctx.detect_color  = color;
    return ctx;
}

void AxeraBackend::preprocess_image(const cv::Mat& image) noexcept {
    auto* npu_input = static_cast<uint8_t*>(io_data_.pInputs[0].pVirAddr);

    cv::Mat npu_input_mat(config_.input_height, config_.input_width, CV_8UC3, npu_input);
    cv::resize(
        image, npu_input_mat, cv::Size(config_.input_width, config_.input_height), 0, 0,
        cv::INTER_LINEAR);

    // Input buffer is non-cached; no flush needed — CPU writes land directly in DDR.
}

/**
 * @brief 初始化 IVPS 硬件预处理单元
 *
 * IVPS (Image Video Processing System) 是 AX650 芯片内置的专用图像处理硬件，
 * 可独立执行 resize、color space conversion 等操作，零 CPU 负载。
 *
 * 初始化流程：
 * 1. 调用 AX_IVPS_Init 初始化 IVPS 硬件
 * 2. 配置 aspect ratio 参数（STRETCH 模式，直接缩放无 letterbox）
 *
 * STRETCH 模式说明：
 * - 直接将源图像缩放到目标尺寸，不保持宽高比
 * - 适用于 4:3 输入到 4:3 模型输入的场景（无畸变）
 * - 对于非 4:3 输入，会有畸变（已在 detect_impl 中警告）
 *
 * @return 成功返回 void，失败返回错误信息
 *
 * 注意：IVPS 初始化失败不会导致后端初始化失败，而是降级到 OpenCV CPU 预处理。
 */
std::expected<void, std::string> AxeraBackend::init_ivps() noexcept {
    const auto ret = AX_IVPS_Init();
    if (ret != AX_SUCCESS) {
        ivps_initialized_ = false;
        return std::unexpected(fmt::format("AX_IVPS_Init: {}", ax_error_string(ret)));
    }

    // 配置 aspect ratio：STRETCH 模式（直接缩放，无 letterbox）
    ivps_aspect_ratio_.eMode      = AX_IVPS_ASPECT_RATIO_STRETCH;
    ivps_aspect_ratio_.eAligns[0] = AX_IVPS_ASPECT_RATIO_HORIZONTAL_CENTER;
    ivps_aspect_ratio_.eAligns[1] = AX_IVPS_ASPECT_RATIO_VERTICAL_CENTER;
    ivps_aspect_ratio_.nBgColor   = 0xFF00FF; // 紫色背景（STRETCH 模式下不显示）

    ivps_initialized_ = true;
    return {};
}

void AxeraBackend::deinit_ivps() noexcept {
    if (!ivps_initialized_) {
        return;
    }
    free_ivps_buffers();
    // 不清理 AX_POOL，可能是共享资源
    AX_IVPS_Deinit();
    ivps_initialized_ = false;
}

std::expected<void, std::string>
    AxeraBackend::allocate_ivps_buffers(int max_src_width, int max_src_height) noexcept {
    free_ivps_buffers();

    ivps_max_src_width_  = max_src_width;
    ivps_max_src_height_ = max_src_height;

    // BGR888 stride must be a multiple of 3; align to 48 bytes (16 pixels × 3 bytes/pixel)
    // to satisfy both the 3-byte pixel packing constraint and hardware alignment requirements.
    // Reference: ax-video-sdk src/common/ax_image_processor_ax650.cpp MakeAlignedDescriptor()
    constexpr int ALIGN         = 48;
    const AX_U32 aligned_stride = bgr888_stride_aligned(max_src_width);
    ivps_src_stride_            = aligned_stride;
    ivps_src_buffer_size_       = aligned_stride * static_cast<AX_U32>(max_src_height);

    // Use non-cached allocation: CPU writes once, IVPS DMA reads once.
    // Cached buffer would require AX_SYS_MflushCache syscall per frame (~0.1-0.2ms)
    // with no cache reuse benefit.  Non-cached writes are ~10-20% slower per byte
    // but eliminate syscall jitter entirely — same tradeoff as NPU input buffer (line 232).
    const auto ret = AX_SYS_MemAlloc(
        &ivps_src_phy_addr_, &ivps_src_vir_addr_, ivps_src_buffer_size_, ALIGN,
        reinterpret_cast<const AX_S8*>("ivps-src"));

    if (ret != AX_SUCCESS || ivps_src_vir_addr_ == nullptr) {
        ivps_src_phy_addr_    = 0;
        ivps_src_vir_addr_    = nullptr;
        ivps_src_buffer_size_ = 0;
        ivps_src_stride_      = 0;
        return std::unexpected(fmt::format("AX_SYS_MemAlloc: {}", ax_error_string(ret)));
    }

    SPDLOG_INFO(
        "IVPS buffers allocated: {}x{} (stride={}) -> {}x{}", max_src_width, max_src_height,
        aligned_stride, config_.input_width, config_.input_height);
    return {};
}

void AxeraBackend::free_ivps_buffers() noexcept {
    if (ivps_src_vir_addr_ != nullptr && ivps_src_phy_addr_ != 0) {
        AX_SYS_MemFree(ivps_src_phy_addr_, ivps_src_vir_addr_);
    }
    ivps_src_phy_addr_    = 0;
    ivps_src_vir_addr_    = nullptr;
    ivps_src_buffer_size_ = 0;
    ivps_src_stride_      = 0;

    free_ivps_dst_buffer();
}

/**
 * @brief 使用 IVPS 硬件预处理图像
 *
 * 算法流程：
 * 1. 检查 IVPS 是否初始化（未初始化降级到 OpenCV）
 * 2. 检查输入合法性（非空、CV_8UC3）
 * 3. 奇偶对齐：VPP 硬件要求偶数尺寸，裁剪最后一行/列
 * 4. 分配/复用源缓冲区（按需增长）
 * 5. 拷贝图像数据到源缓冲区（stride 对齐）
 * 6. 配置 IVPS 参数（源帧、目标帧）
 * 7. 执行硬件 resize（AX_IVPS_CropResizeVpp）
 *
 * 直接写入 NPU 输入缓冲区优化：
 * - 优势：避免中间拷贝，降低延迟
 * - 安全性：768 * 3 = 2304 已满足 BGR888 48 字节 stride 约定
 * - 结果：消除额外 staging-buffer 拷贝，延迟减半
 *
 * 错误处理：
 * - IVPS 失败自动降级到 OpenCV CPU 路径
 * - 日志记录失败原因，便于调试
 *
 * @param image 输入图像（BGR 格式）
 *
 * 注意：NPU 输入缓冲区是非缓存的，无需 cache flush。
 */
void AxeraBackend::preprocess_image_ivps(const cv::Mat& image) noexcept {
    if (!ivps_initialized_) {
        preprocess_image(image);
        return;
    }

    if (image.empty() || image.type() != CV_8UC3) {
        SPDLOG_WARN(
            "preprocess_image_ivps expects non-empty CV_8UC3 BGR image, got type={} "
            "size={}x{}; using opencv",
            image.type(), image.cols, image.rows);
        preprocess_image(image);
        return;
    }

    // VPP 硬件约束：某些 MSP 构建拒绝奇数尺寸（AX_ERR_ILLEGAL_PARAM）
    // 解决方案：裁剪最后一行/列（损失可忽略，避免 CPU 回退）
    const int hw_src_w = image.cols & ~1;
    const int hw_src_h = image.rows & ~1;
    if (hw_src_w <= 0 || hw_src_h <= 0) {
        SPDLOG_WARN(
            "IVPS source size becomes invalid after even alignment: original={}x{}, hw={}x{}; "
            "using opencv",
            image.cols, image.rows, hw_src_w, hw_src_h);
        preprocess_image(image);
        return;
    }

    // 检查源缓冲区是否需要增长（按需分配）
    if (hw_src_w > ivps_max_src_width_ || hw_src_h > ivps_max_src_height_
        || ivps_src_vir_addr_ == nullptr) {
        auto buffer_result = allocate_ivps_buffers(hw_src_w, hw_src_h);
        if (!buffer_result) {
            static bool warned = false;
            if (!warned) {
                SPDLOG_WARN("allocate_ivps_buffers: {}, using opencv", buffer_result.error());
                warned = true;
            }
            preprocess_image(image);
            return;
        }
    }

    // 检查 NPU 输入缓冲区是否可用
    const size_t packed_row  = static_cast<size_t>(config_.input_width) * 3U;
    const size_t packed_size = packed_row * static_cast<size_t>(config_.input_height);
    if (io_data_.pInputs == nullptr || io_data_.pInputs[0].pVirAddr == nullptr
        || static_cast<size_t>(io_data_.pInputs[0].nSize) < packed_size) {
        SPDLOG_WARN(
            "NPU input buffer invalid/small for direct IVPS dst: nSize={}, required={}; using "
            "opencv",
            io_data_.pInputs != nullptr ? io_data_.pInputs[0].nSize : 0, packed_size);
        preprocess_image(image);
        return;
    }

    // 配置源帧描述符
    AX_VIDEO_FRAME_T src_frame{};
    src_frame.u32Width                      = static_cast<AX_U32>(hw_src_w);
    src_frame.u32Height                     = static_cast<AX_U32>(hw_src_h);
    src_frame.u32PicStride[0]               = ivps_src_stride_;
    src_frame.enImgFormat                   = AX_FORMAT_BGR888;
    src_frame.u64PhyAddr[0]                 = ivps_src_phy_addr_;
    src_frame.u64VirAddr[0]                 = reinterpret_cast<AX_U64>(ivps_src_vir_addr_);
    src_frame.u32FrameSize                  = ivps_src_buffer_size_;
    src_frame.stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;

    // 拷贝图像数据到源缓冲区（stride 对齐）
    auto* src_buffer       = static_cast<uint8_t*>(ivps_src_vir_addr_);
    const size_t src_bytes = static_cast<size_t>(hw_src_w) * 3U;
    for (int y = 0; y < hw_src_h; ++y) {
        std::memcpy(
            src_buffer + static_cast<size_t>(y) * ivps_src_stride_, image.ptr(y), src_bytes);
    }

    // 配置目标帧描述符（直接使用 NPU 输入缓冲区）
    AX_VIDEO_FRAME_T dst_frame{};
    dst_frame.u32Width        = config_.input_width;
    dst_frame.u32Height       = config_.input_height;
    dst_frame.u32PicStride[0] = static_cast<AX_U32>(packed_row);
    dst_frame.enImgFormat     = AX_FORMAT_BGR888;
    dst_frame.u64PhyAddr[0]   = io_data_.pInputs[0].phyAddr;
    dst_frame.u64VirAddr[0]   = reinterpret_cast<AX_U64>(io_data_.pInputs[0].pVirAddr);
    dst_frame.u32FrameSize    = static_cast<AX_U32>(packed_size);
    dst_frame.stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;

    // 执行硬件 resize
    const auto ret = AX_IVPS_CropResizeVpp(&src_frame, &dst_frame, &ivps_aspect_ratio_);
    if (ret != AX_SUCCESS) {
        SPDLOG_WARN(
            "AX_IVPS_CropResizeVpp direct-dst failed: {}, src={}x{} stride={} frameSize={}, "
            "dst={}x{} stride={} frameSize={} npuSize={}; using opencv",
            ax_error_string(ret), src_frame.u32Width, src_frame.u32Height,
            src_frame.u32PicStride[0], src_frame.u32FrameSize, dst_frame.u32Width,
            dst_frame.u32Height, dst_frame.u32PicStride[0], dst_frame.u32FrameSize,
            io_data_.pInputs[0].nSize);
        preprocess_image(image);
        return;
    }

    static bool logged_ivps_success = false;
    if (!logged_ivps_success) {
        SPDLOG_INFO(
            "IVPS hardware resize working direct to NPU input! ({}x{} effective from {}x{} -> "
            "{}x{}, src_stride={}, dst_stride={})",
            hw_src_w, hw_src_h, image.cols, image.rows, config_.input_width, config_.input_height,
            ivps_src_stride_, dst_frame.u32PicStride[0]);
        logged_ivps_success = true;
    }

    // NPU 输入缓冲区是非缓存的，无需 cache flush
}

bool AxeraBackend::infer() noexcept {
    if (handle_ == nullptr) {
        return false;
    }

    const auto ret = AX_ENGINE_RunSync(handle_, &io_data_);
    if (ret != 0) {
        SPDLOG_ERROR("infer: AX_ENGINE_RunSync: {}", ax_error_string(ret));
        return false;
    }
    return true;
}

/**
 * @brief 构建 anchor grid（初始化时调用一次）
 *
 * concat-predecode 模型的输出不包含 anchor grid 信息，
 * 需要在 CPU 上预先计算并缓存。
 *
 * 算法：
 * 1. 计算总 anchor 数量（遍历所有 stride）
 * 2. 为每个 anchor 计算其 grid 坐标（x + 0.5, y + 0.5）
 * 3. 存储 grid_x、grid_y、stride 到缓存
 *
 * Grid 坐标计算：
 * - 对于 stride=8 的特征图，每个 grid 位置对应输入图像 8x8 像素区域
 * - grid_x = x + 0.5：anchor 中心位于 grid cell 中心
 *
 * 解码公式：
 * coord = (raw_coord + grid_offset) * stride * scale
 *
 * @return DecodeGrid 结构，包含所有 anchor 的 grid 坐标
 */
AxeraBackend::DecodeGrid AxeraBackend::build_decode_grid() const noexcept {
    // 计算总 anchor 数量
    int total_anchors = 0;
    for (const int stride : config_.strides) {
        total_anchors += (config_.input_width / stride) * (config_.input_height / stride);
    }

    DecodeGrid grid;
    grid.grid_x.reserve(total_anchors);
    grid.grid_y.reserve(total_anchors);
    grid.strides.reserve(total_anchors);

    // 为每个 stride 构建特征图 grid
    for (const int stride : config_.strides) {
        const int feat_h = config_.input_height / stride;
        const int feat_w = config_.input_width / stride;
        for (int y = 0; y < feat_h; ++y) {
            for (int x = 0; x < feat_w; ++x) {
                // grid_x/y = 位置索引 + 中心偏移（0.5）
                grid.grid_x.push_back(static_cast<float>(x) + GRID_OFFSET);
                grid.grid_y.push_back(static_cast<float>(y) + GRID_OFFSET);
                grid.strides.push_back(static_cast<float>(stride));
            }
        }
    }

    return grid;
}

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
 * 2. 应用 logit threshold 过滤低置信度检测（先于 sigmoid，避免计算）
 * 3. 使用最小堆维护 top-k 检测
 * 4. 解码关键点坐标：kpt = (raw_kpt + grid_offset) * stride * scale
 * 5. 应用颜色过滤（只保留目标颜色的检测）
 * 6. 执行 tile-based NMS
 *
 * 性能优化：
 * - logit threshold 先于 sigmoid（避免计算低置信度检测）
 * - 预分配缓冲区（避免动态内存分配）
 * - 复用解码 grid（避免重复计算）
 * - 最小堆优化：O(N log K) 而非 O(N log N) 排序
 *
 * @param ctx 预处理上下文（缩放参数）
 * @return 检测结果列表
 */
std::vector<ArmorDetection>
    AxeraBackend::postprocess_concat_predecode(const PreprocContext& ctx) const noexcept {
    // 计算输出维度
    const int predecode_box_dim  = 4;
    const int predecode_obj_dim  = config_.num_pairs;
    const int predecode_kpts_dim = config_.num_kpts * 2;
    const int predecode_output_dim =
        predecode_box_dim + config_.num_colors + predecode_obj_dim + predecode_kpts_dim;

    // 查找有效的输出缓冲区（选择最大的）
    const AX_ENGINE_IO_BUFFER_T* output_buf = nullptr;
    const size_t min_output_bytes = static_cast<size_t>(predecode_output_dim) * sizeof(float);

    for (int i = 0; i < static_cast<int>(io_data_.nOutputSize); ++i) {
        const AX_ENGINE_IO_BUFFER_T& buf = io_data_.pOutputs[i];
        if (buf.pVirAddr == nullptr || static_cast<size_t>(buf.nSize) < min_output_bytes) {
            continue;
        }
        if (static_cast<size_t>(buf.nSize) % (sizeof(float) * predecode_output_dim) != 0U) {
            continue;
        }
        if (output_buf == nullptr || buf.nSize > output_buf->nSize) {
            output_buf = &buf;
        }
    }

    if (output_buf == nullptr) {
        SPDLOG_ERROR("No valid concat-predecode output tensor found");
        return {};
    }

    // 计算 anchor 数量
    const int num_anchors = static_cast<int>(
        static_cast<size_t>(output_buf->nSize) / (sizeof(float) * predecode_output_dim));
    if (num_anchors != static_cast<int>(cached_grid_.grid_x.size())) {
        SPDLOG_ERROR(
            "Anchor count mismatch: output={} grid={}", num_anchors, cached_grid_.grid_x.size());
        return {};
    }

    // 获取各通道指针（跳过 raw_boxes）
    const float* output       = static_cast<const float*>(output_buf->pVirAddr);
    const float* color_logits = output + predecode_box_dim * num_anchors;
    const float* obj_logits   = color_logits + config_.num_colors * num_anchors;
    const float* raw_kpts     = obj_logits + predecode_obj_dim * num_anchors;

    // 计算 logit threshold（先于 sigmoid，避免计算低置信度检测）
    const float confidence_threshold = static_cast<float>(config_.confidence_threshold);
    const float logit_threshold = std::log(confidence_threshold / (1.0f - confidence_threshold));

    static bool logged_threshold = false;
    if (!logged_threshold) {
        SPDLOG_INFO(
            "confidence_threshold={}, logit_threshold={:.2f}", confidence_threshold,
            logit_threshold);
        logged_threshold = true;
    }

    // 第一阶段：使用最小堆维护 top-k anchors
    const size_t pre_nms_top_k =
        static_cast<size_t>(std::min(config_.top_k, std::max(num_anchors, 1)));
    auto min_heap_comp = [](const RankedAnchor& lhs, const RankedAnchor& rhs) {
        return lhs.best_obj_logit > rhs.best_obj_logit;
    };

    postproc_ranked_buf_.clear();
    if (postproc_ranked_buf_.capacity() < pre_nms_top_k) {
        postproc_ranked_buf_.reserve(pre_nms_top_k);
    }
    auto& ranked = postproc_ranked_buf_;

    for (int anchor_idx = 0; anchor_idx < num_anchors; ++anchor_idx) {
        // 找出 obj_logits 最高的 pair_id
        float best_obj_logit = obj_logits[anchor_idx];
        int pair_id          = 0;
        for (int cls = 1; cls < config_.num_pairs; ++cls) {
            const float score = obj_logits[cls * num_anchors + anchor_idx];
            if (score > best_obj_logit) {
                best_obj_logit = score;
                pair_id        = cls;
            }
        }

        // 应用 logit threshold 过滤
        if (best_obj_logit < logit_threshold) {
            continue;
        }

        // 使用最小堆维护 top-k
        const RankedAnchor item{best_obj_logit, pair_id, anchor_idx};
        if (ranked.size() < pre_nms_top_k) {
            ranked.push_back(item);
            std::push_heap(ranked.begin(), ranked.end(), min_heap_comp);
            continue;
        }
        if (!ranked.empty() && item.best_obj_logit > ranked.front().best_obj_logit) {
            std::pop_heap(ranked.begin(), ranked.end(), min_heap_comp);
            ranked.back() = item;
            std::push_heap(ranked.begin(), ranked.end(), min_heap_comp);
        }
    }

    // 排序 top-k 检测（降序）
    std::sort(ranked.begin(), ranked.end(), [](const RankedAnchor& lhs, const RankedAnchor& rhs) {
        return lhs.best_obj_logit > rhs.best_obj_logit;
    });

    // 第二阶段：解码关键点坐标
    const float max_x   = static_cast<float>(std::max(ctx.orig_w - 1, 0));
    const float max_y   = static_cast<float>(std::max(ctx.orig_h - 1, 0));
    const float scale_x = ctx.scale_x;
    const float scale_y = ctx.scale_y;

    postproc_candidates_buf_.clear();
    if (postproc_candidates_buf_.capacity() < ranked.size()) {
        postproc_candidates_buf_.reserve(ranked.size());
    }
    auto& candidates = postproc_candidates_buf_;

    for (const RankedAnchor& ranked_anchor : ranked) {
        // sigmoid 转换置信度
        const float confidence = fast_sigmoid(ranked_anchor.best_obj_logit);

        const int anchor_idx = ranked_anchor.anchor_idx;
        const float grid_x   = cached_grid_.grid_x[anchor_idx];
        const float grid_y   = cached_grid_.grid_y[anchor_idx];
        const float stride   = cached_grid_.strides[anchor_idx];

        // 颜色分类（找出最高置信度的颜色）
        int best_color_id   = 0;
        float best_color_sc = color_logits[anchor_idx];
        for (int color_idx = 1; color_idx < config_.num_colors; ++color_idx) {
            const float score = color_logits[color_idx * num_anchors + anchor_idx];
            if (score > best_color_sc) {
                best_color_sc = score;
                best_color_id = color_idx;
            }
        }

        // 颜色过滤：只保留目标颜色的检测
        if (static_cast<ArmorColor>(best_color_id) != ctx.detect_color) {
            continue;
        }

        // 解码关键点坐标
        std::array<cv::Point2f, 4> model_corners{};
        for (int k = 0; k < config_.num_kpts; ++k) {
            const float kx = raw_kpts[(k * 2 + 0) * num_anchors + anchor_idx];
            const float ky = raw_kpts[(k * 2 + 1) * num_anchors + anchor_idx];

            // 解码公式：coord = (raw_coord + grid_offset) * stride * scale
            model_corners[k].x = clampf((kx + grid_x) * stride * scale_x, 0.0f, max_x);
            model_corners[k].y = clampf((ky + grid_y) * stride * scale_y, 0.0f, max_y);
        }

        // 重排关键点顺序（通过 KPT_ORDER）
        std::array<cv::Point2f, 4> ordered_corners{};
        for (int k = 0; k < config_.num_kpts; ++k) {
            ordered_corners[k] = model_corners[KPT_ORDER[k]];
        }

        // 验证：拒绝退化检测（零面积）
        const cv::Rect2f br = bounding_rect_4pt(ordered_corners);
        if (br.width <= 0.0f || br.height <= 0.0f) [[unlikely]] {
            continue;
        }

        // 构建候选检测
        DecodedCandidate candidate;
        candidate.corners    = ordered_corners;
        candidate.name       = PAIR_TO_ARMOR[ranked_anchor.pair_id];
        candidate.color      = static_cast<ArmorColor>(best_color_id);
        candidate.confidence = confidence;
        candidate.pair_id    = ranked_anchor.pair_id;
        candidates.push_back(std::move(candidate));
    }

    // 第三阶段：执行 tile-based NMS
    return nms(candidates, ctx.orig_w, ctx.orig_h);
}

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
 *   - 传统 NMS：每个检测与所有其他检测比较
 *   - Tile-based：每个检测只与相邻分块内的检测比较
 * - 空间局部性：相邻分块的数据在内存中连续，缓存友好
 * - 并行友好：可扩展到多线程实现（每个分块独立处理）
 *
 * IoU 计算方式：
 * - 使用四边形边界框计算 IoU（近似）
 *   - 真实四边形 IoU 计算复杂，边界框近似精度足够
 * - 只在相同 pair_id 间计算 IoU（避免不同类别互相抑制）
 *
 * 实现细节：
 * - 预分配缓冲区：避免动态内存分配抖动
 * - 按置信度排序：优先保留高置信度检测
 * - 移除标记：使用 removed 数组标记已移除的检测
 *
 * @param detections 候选检测列表
 * @param orig_w 原始图像宽度
 * @param orig_h 原始图像高度
 * @return NMS 后的检测结果
 */
std::vector<ArmorDetection> AxeraBackend::nms(
    std::vector<DecodedCandidate>& detections, int orig_w, int orig_h) const noexcept {
    if (detections.empty()) {
        return {};
    }

    // 计算分块尺寸
    const float tile_w = static_cast<float>(orig_w) / TILE_GRID_SIZE;
    const float tile_h = static_cast<float>(orig_h) / TILE_GRID_SIZE;

    // 清空分块 bins
    for (auto& bin : nms_tile_bins_buf_) {
        bin.clear();
    }
    auto& tile_bins = nms_tile_bins_buf_;

    // 将检测分配到对应分块
    for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
        const auto& det = detections[i];
        // 计算检测中心点
        const cv::Point2f center =
            (det.corners[0] + det.corners[1] + det.corners[2] + det.corners[3]) * 0.25f;
        // 计算分块索引（裁剪到有效范围）
        const int tx = clamp_int(static_cast<int>(center.x / tile_w), 0, TILE_GRID_SIZE - 1);
        const int ty = clamp_int(static_cast<int>(center.y / tile_h), 0, TILE_GRID_SIZE - 1);
        tile_bins[ty * TILE_GRID_SIZE + tx].push_back(i);
    }

    const float nms_thresh = static_cast<float>(config_.nms_threshold);

    // 初始化移除标记
    nms_removed_buf_.assign(detections.size(), 0);
    auto& removed = nms_removed_buf_;

    // 预分配结果缓冲区
    nms_result_buf_.clear();
    nms_result_buf_.reserve(std::min(detections.size(), static_cast<size_t>(config_.top_k)));

    // 按置信度顺序处理检测
    for (size_t i = 0;
         i < detections.size() && nms_result_buf_.size() < static_cast<size_t>(config_.top_k);
         ++i) {
        if (removed[i] != 0) {
            continue; // 跳过已移除的检测
        }

        // 添加当前检测到结果列表
        ArmorDetection det(
            detections[i].corners, detections[i].name, detections[i].color,
            detections[i].confidence);
        if (det.area <= 0) {
            continue; // 跳过无效检测
        }
        nms_result_buf_.push_back(std::move(det));

        // 找出当前检测所属的分块
        const auto& corners_i = detections[i].corners;
        const cv::Point2f center_i =
            (corners_i[0] + corners_i[1] + corners_i[2] + corners_i[3]) * 0.25f;
        const int tx = clamp_int(static_cast<int>(center_i.x / tile_w), 0, TILE_GRID_SIZE - 1);
        const int ty = clamp_int(static_cast<int>(center_i.y / tile_h), 0, TILE_GRID_SIZE - 1);

        // 只检查相邻分块（3x3 窗口）
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const int nx = tx + dx;
                const int ny = ty + dy;
                // 跳过越界的分块
                if (nx < 0 || nx >= TILE_GRID_SIZE || ny < 0 || ny >= TILE_GRID_SIZE) {
                    continue;
                }
                const int neighbor_tile = ny * TILE_GRID_SIZE + nx;
                // 遍历相邻分块内的所有检测
                for (const int j : tile_bins[neighbor_tile]) {
                    // 跳过自身、已移除、不同 pair_id 的检测
                    if (j <= static_cast<int>(i) || removed[j] != 0
                        || detections[i].pair_id != detections[j].pair_id) {
                        continue;
                    }
                    // 计算 IoU，如果超过阈值则移除
                    if (pts_iou(detections[i].corners, detections[j].corners) > nms_thresh) {
                        removed[j] = 1;
                    }
                }
            }
        }
    }

    return std::move(nms_result_buf_);
}

AxeraBackend::DetectionResult
    AxeraBackend::detect_impl(const cv::Mat& image, ArmorColor color) noexcept {
    const auto ctx = preprocess(image, color);
    if (!ctx.is_four_three && !warned_non_four_three_input_) {
        SPDLOG_WARN(
            "Input {}x{} is not 4:3; direct resize to {}x{} will be used "
            "without letterbox",
            image.cols, image.rows, config_.input_width, config_.input_height);
        warned_non_four_three_input_ = true;
    }

    if (ivps_initialized_) {
        preprocess_image_ivps(image);
    } else {
        preprocess_image(image);
    }

    if (!infer()) {
        return std::unexpected("Axera NPU inference failed (AX_ENGINE_RunSync)");
    }

    return postprocess_concat_predecode(ctx);
}

} // namespace fcs::L2
