#pragma once

// ---------------------------------------------------------------------------
// AX VENC encoder configuration and frame conversion utilities
// ---------------------------------------------------------------------------
// 文档说明：将原本适配libx265的EncodeParams编码参数映射到AX系列硬件VENC编码器
// 完整保留原有调参逻辑意图，下方是x265参数与AX硬件编码器参数一一对应映射表
//
//   x265 parameter           AX VENC mapping
//   ───────────────────────────────────────────────────────────
//   enVBR=false              H265CBR 恒定码率模式，target_bitrate 映射到 u32BitRate
//   enVBR=true               H265VBR 可变码率模式，max_bit_rate 映射到 u32MaxBitRate
//   min_bit_rate             AX H265VBR 无对应硬件参数，仅解析不使用
//   gop_size                 u32Gop 编码IDR关键帧间隔
//   framerate                stFrameRate.fDstFrameRate 输出帧率
//   max_width/max_height     u32PicWidthSrc / u32PicHeightSrc 编码最大支持分辨率
//   intra_refresh            stIntraRefresh 帧内刷新防闪烁配置
//   chroma_qp_offset         s32IntraQpDelta 色度QP偏移近似等效实现
//   filter.aq_strength       CTB 宏块码率控制模式 QUALITY_RATE 画质优先
//   filter.denoise_luma      IVE硬件高斯亮度降噪，在IVPS缩放/色彩转换后执行
//   filter.denoise_chroma    IVE硬件高斯色度降噪，作用于NV12交错UV平面
//   filter.sharpen*_luma     IVE高提升亮度锐化算子，恢复图像边缘细节
//   filter.sharpen*_chroma   拆分NV12 UV平面分别锐化，处理完成后重新合并UV
//   filter.luma_levels       IVE多阈值亮度量化分层，降低动态范围压缩码率
//   filter.chroma_levels     拆分UV独立量化色度，再合并回NV12
//   filter.qp_delta_map      AX_VENC_SendFrameEx 逐帧16x16宏块QP偏移图，局部动态码率
//   filter.saturation        AX编码后端无对应接口，饱和度调整需ISP图像信号处理器调参
//   crf                      硬件实时编码不支持CRF恒定画质，仅使用CBR/VBR码率控制
//   filter.psy_rd/trellis    仅x265软件心理视觉优化，AX硬件编码器无对应功能
//   preset / tune            硬件编码流水线固定，无速度/画质预设可调
//   me / subme / rd / rdoq   运动估计、率失真优化全部硬件内部固化，不可配置
// ---------------------------------------------------------------------------

// AX VENC RAII自动生命周期管理封装（通道、模块自动销毁）
#include "quanta/encode_backend/ax/ax_venc_raii.hpp"
// 通用流编码器抽象基类、编码包结构体EncodedPacket、参数结构体EncodeParams/FilterParams
#include "quanta/stream_encoder.hpp"

// C++标准库
#include <algorithm>    // std::min/std::max/std::clamp 数值限制、缩放计算
#include <cmath>        // 浮点数学运算，缩放比例、像素滤波计算
#include <cstdint>      // 固定宽度整数类型，跨平台统一uint/int
#include <cstring>      // 内存清零、拷贝 memset/memcpy
#include <expected>     // C++23 预期返回类型，统一错误/成功返回值
#include <format>       // 格式化字符串，错误日志打印
#include <string>       // 错误信息存储字符串
#include <vector>       // 动态数组（本头文件未直接使用，预留上层依赖）

// C语言AX底层驱动头文件，extern C隔离C++名称修饰
extern "C" {
#include "ax_sys_api.h"  // AX全局系统内存CMM分配/释放、缓存刷新接口
#include "ax_venc_comm.h"// VENC编码器通道、码率控制、帧结构体枚举定义
}

namespace quanta {

// ---------------------------------------------------------------------------
// 分辨率适配工具：输入分辨率等比缩小至最大分辨率限制，强制输出宽高为偶数
// ---------------------------------------------------------------------------
/**
 * @brief 等比例缩放图像尺寸，保证输出不超过max_w/max_h，宽高强制偶数（硬件编码约束）
 * @param src_w 原始输入宽度
 * @param src_h 原始输入高度
 * @param max_w 编码硬件支持最大宽度
 * @param max_h 编码硬件支持最大高度
 * @param out_w 输出适配后宽度（出参）
 * @param out_h 输出适配后高度（出参）
 */
inline void fit_dimensions(int src_w, int src_h, int max_w, int max_h, int& out_w, int& out_h) {
    // 计算宽高缩放比例，取最小比例保证完整画面放入最大分辨率
    double scale = std::min(static_cast<double>(max_w) / src_w, static_cast<double>(max_h) / src_h);
    // 缩放比例≥1，原图尺寸小于限制，直接使用原图尺寸
    if (scale >= 1.0) {
        out_w = src_w;
        out_h = src_h;
        return;
    }
    // 缩放后尺寸向下对齐偶数，AX VENC要求宽高必须偶数
    out_w = (static_cast<int>(src_w * scale) / 2) * 2;
    out_h = (static_cast<int>(src_h * scale) / 2) * 2;
    // 边界保护：防止缩放后尺寸为0，硬件最小支持2像素
    if (out_w == 0)
        out_w = 2;
    if (out_h == 0)
        out_h = 2;
}

// ---------------------------------------------------------------------------
// BGR24 转 NV12 4:2:0半平面YUV说明
// ---------------------------------------------------------------------------
// 转换使用BT.601标准RGB转YUV系数
// NV12内存布局：完整亮度Y平面(w×h) + 交错UV色度平面(w/2 × h/2，每个像素U/V成对存储)
// 所有行宽stride强制64字节对齐，AX VENC硬件DMA硬性要求

// ---------------------------------------------------------------------------
// CMM物理连续内存 NV12帧封装结构体（适配AX硬件DMA）
// ---------------------------------------------------------------------------
// AX650 VENC硬件仅支持CMM物理连续内存做输入帧，标准堆内存malloc/std::vector无法用于硬件DMA
// 帧内存必须通过AX_SYS_MemAllocCached分配带缓存CMM内存，提交编码前必须刷新CPU缓存
// ---------------------------------------------------------------------------

// VENC帧行宽对齐64字节常量
constexpr int kAxVencStrideAlign = 64;
// CMM内存分配对齐粒度64字节
constexpr AX_U32 kCmmAlignment   = 64;

/// RAII 生命周期管理NV12帧CMM内存块
/// 单块连续CMM内存，Y平面、UV平面分段存放于同一块物理内存
struct Nv12Frame {
    AX_U64 phy_addr   = 0;       // CMM内存物理地址，硬件DMA使用
    void* vir_addr    = nullptr; // CPU虚拟访问地址，软件读写像素
    AX_U32 alloc_size = 0;       // CMM整块分配总字节大小

    int width            = 0;    // 图像有效像素宽度
    int height           = 0;    // 图像有效像素高度
    int y_stride         = 0;    // Y平面每行字节（64字节对齐）
    int uv_stride        = 0;    // UV平面每行字节，NV12与Y平面行宽一致
    size_t y_plane_size  = 0;    // Y平面总字节数
    size_t uv_plane_size = 0;    // UV交错平面总字节数

    // 默认构造，成员全部零初始化
    Nv12Frame() = default;

    // 禁止拷贝：CMM物理内存独占所有权，不可复制
    Nv12Frame(const Nv12Frame&)            = delete;
    Nv12Frame& operator=(const Nv12Frame&) = delete;

    // 移动构造：转移CMM内存所有权，原对象清空内存信息
    Nv12Frame(Nv12Frame&& other) noexcept
        : phy_addr(other.phy_addr)
        , vir_addr(other.vir_addr)
        , alloc_size(other.alloc_size)
        , width(other.width)
        , height(other.height)
        , y_stride(other.y_stride)
        , uv_stride(other.uv_stride)
        , y_plane_size(other.y_plane_size)
        , uv_plane_size(other.uv_plane_size) {
        other.phy_addr   = 0;
        other.vir_addr   = nullptr;
        other.alloc_size = 0;
    }

    // 移动赋值：先释放当前内存，接管传入对象内存
    Nv12Frame& operator=(Nv12Frame&& other) noexcept {
        if (this != &other) {
            destroy();
            phy_addr         = other.phy_addr;
            vir_addr         = other.vir_addr;
            alloc_size       = other.alloc_size;
            width            = other.width;
            height           = other.height;
            y_stride         = other.y_stride;
            uv_stride        = other.uv_stride;
            y_plane_size     = other.y_plane_size;
            uv_plane_size    = other.uv_plane_size;
            other.phy_addr   = 0;
            other.vir_addr   = nullptr;
            other.alloc_size = 0;
        }
        return *this;
    }

    // 析构自动释放CMM内存，防止内存泄漏
    ~Nv12Frame() { destroy(); }

    /// 释放当前持有的CMM物理内存，重置所有成员
    void destroy() noexcept {
        if (vir_addr != nullptr && phy_addr != 0) {
            AX_SYS_MemFree(phy_addr, vir_addr);
        }
        phy_addr   = 0;
        vir_addr   = nullptr;
        alloc_size = 0;
    }

    /// 分配指定分辨率NV12帧CMM缓存，复用逻辑：尺寸完全匹配直接复用内存
    /// @param w 图像宽度
    /// @param h 图像高度
    /// @return 成功返回空expected，失败携带错误字符串
    [[nodiscard]] std::expected<void, std::string> allocate(int w, int h) noexcept {
        // 先释放原有内存
        destroy();

        width     = w;
        height    = h;
        // 向上对齐64字节计算Y行宽
        y_stride  = ((w + kAxVencStrideAlign - 1) / kAxVencStrideAlign) * kAxVencStrideAlign;
        uv_stride = y_stride;

        // 计算Y平面、UV平面字节大小
        y_plane_size  = static_cast<size_t>(y_stride) * h;
        uv_plane_size = static_cast<size_t>(uv_stride) * (h / 2);
        alloc_size    = static_cast<AX_U32>(y_plane_size + uv_plane_size);

        // 分配带CPU缓存的CMM物理连续内存，命名用于系统调试追踪
        AX_S32 ret = AX_SYS_MemAllocCached(
            &phy_addr, &vir_addr, alloc_size, kCmmAlignment,
            const_cast<AX_S8*>(reinterpret_cast<const AX_S8*>("QuantaNV12")));
        if (ret != AX_SUCCESS) {
            // 分配失败清空内存标记，返回错误信息
            phy_addr   = 0;
            vir_addr   = nullptr;
            alloc_size = 0;
            return std::unexpected(
                std::format(
                    "AX_SYS_MemAllocCached failed for NV12 {}x{} ({} bytes): ret=0x{:X}", w, h,
                    alloc_size, static_cast<unsigned>(ret)));
        }

        // 整块内存清零，对齐填充区域必须置0，防止垃圾像素干扰编码
        std::memset(vir_addr, 0, alloc_size);
        return {};
    }

    /// 刷新CPU缓存：软件写入像素后，同步到硬件DMA可见内存
    void flush_cache() noexcept {
        if (phy_addr != 0 && vir_addr != nullptr && alloc_size != 0) {
            AX_SYS_MflushCache(phy_addr, vir_addr, alloc_size);
        }
    }

    /// 失效CPU缓存：硬件写入帧后，CPU读取前调用，避免缓存脏数据
    [[nodiscard]] AX_S32 invalidate_cache() noexcept {
        if (phy_addr != 0 && vir_addr != nullptr && alloc_size != 0) {
            return AX_SYS_MinvalidateCache(phy_addr, vir_addr, alloc_size);
        }
        return AX_SUCCESS;
    }

    /// 单独刷新UV色度平面缓存（滤波后单独更新UV时使用）
    void flush_uv_cache() noexcept {
        if (phy_addr != 0 && vir_addr != nullptr && uv_plane_size != 0) {
            AX_SYS_MflushCache(uv_phy_addr(), uv_data(), static_cast<AX_U32>(uv_plane_size));
        }
    }

    /// 单独失效UV平面缓存
    [[nodiscard]] AX_S32 invalidate_uv_cache() noexcept {
        if (phy_addr != 0 && vir_addr != nullptr && uv_plane_size != 0) {
            return AX_SYS_MinvalidateCache(
                uv_phy_addr(), uv_data(), static_cast<AX_U32>(uv_plane_size));
        }
        return AX_SUCCESS;
    }

    // --- 平面读写指针、物理地址访问接口 ---
    // Y平面起始地址为整块内存首地址
    [[nodiscard]] uint8_t* y_data() noexcept { return static_cast<uint8_t*>(vir_addr); }
    [[nodiscard]] const uint8_t* y_data() const noexcept {
        return static_cast<const uint8_t*>(vir_addr);
    }
    [[nodiscard]] AX_U64 y_phy_addr() const noexcept { return phy_addr; }

    // UV平面起始地址：跳过完整Y平面字节
    [[nodiscard]] uint8_t* uv_data() noexcept {
        return static_cast<uint8_t*>(vir_addr) + y_plane_size;
    }
    [[nodiscard]] const uint8_t* uv_data() const noexcept {
        return static_cast<const uint8_t*>(vir_addr) + y_plane_size;
    }
    [[nodiscard]] AX_U64 uv_phy_addr() const noexcept { return phy_addr + y_plane_size; }

    // 平面字节大小获取
    [[nodiscard]] size_t y_size() const noexcept { return y_plane_size; }
    [[nodiscard]] size_t uv_size() const noexcept { return uv_plane_size; }
};

/// IVPS+IVE 硬件图像处理流水线上下文
/// 保存IVPS模块句柄、IVE算子句柄、缩放拉伸配置、流水线阶段开关
struct AxFilterChain {
    std::shared_ptr<IvpsModule> ivps_module; // IVPS缩放/色彩转换硬件模块
    std::shared_ptr<IveModule> ive_module;   // IVE图像滤波硬件算子模块
    AX_IVPS_ASPECT_RATIO_T aspect_ratio{};   // 缩放拉伸填充模式配置
    bool stage_resize_then_csc = false;      // true=先缩放再色彩转换；false=先色彩转换再缩放
    bool use_split_chroma      = false;      // 是否拆分UV平面独立色度滤波
    bool use_ive_filters       = false;      // 是否启用IVE硬件滤波（降噪/锐化/量化）
};

/**
 * @brief 根据滤波参数构建完整硬件图像处理流水线上下文
 * @param filter 前端滤波参数（降噪、锐化、量化、边缘等）
 * @param src_width 输入原始BGR宽度
 * @param src_height 输入原始BGR高度
 * @param dst_width 编码输出NV12宽度
 * @param dst_height 编码输出NV12高度
 * @return 流水线配置结构体，失败返回错误字符串
 */
[[nodiscard]] std::expected<AxFilterChain, std::string> build_ax_filter_chain(
    const FilterParams& filter, int src_width, int src_height, int dst_width,
    int dst_height) noexcept;

// ---------------------------------------------------------------------------
// AX VENC 编码通道属性构建工具函数
// ---------------------------------------------------------------------------

/// 构建HEVC实时流媒体编码通道完整配置结构体
/// 将上层EncodeParams参数全部映射至AX硬件VENC通道属性
[[nodiscard]] std::expected<AX_VENC_CHN_ATTR_T, std::string>
    build_hevc_channel_attr(const EncodeParams& params, int width, int height) noexcept;

/// 根据编码参数构建码率控制RC配置（CBR/VBR、QP区间、GOP、AQ模式）
[[nodiscard]] AX_VENC_RC_ATTR_T build_rc_attr(const EncodeParams& params, int framerate) noexcept;

/// 构建VUI视频可用性信息参数，写入BT.709色域元数据，对齐FFmpeg旧滤镜逻辑
[[nodiscard]] AX_VENC_VUI_PARAM_T build_vui_param() noexcept;

/// 根据参数生成帧内周期性刷新配置，抑制画面编码闪烁
[[nodiscard]] AX_VENC_INTRA_REFRESH_T build_intra_refresh(const EncodeParams& params) noexcept;

// ---------------------------------------------------------------------------
// 帧提交辅助工具函数
// ---------------------------------------------------------------------------

/// 使用预转换完成的NV12帧填充AX底层帧描述结构体
/// 自动填充CMM物理地址，硬件DMA编码必须依赖物理地址
/// @param nv12 预处理完成NV12帧
/// @param pts 帧显示时间戳
/// @param seq_num 帧序列号（默认0）
/// @return 底层硬件帧信息结构体
[[nodiscard]] AX_VIDEO_FRAME_INFO_T
    make_frame_info(const Nv12Frame& nv12, int64_t pts, uint64_t seq_num = 0) noexcept;

/// 从VENC输出码流结构体提取完整编码包
/// 拷贝硬件内部码流缓冲区至新分配内存，拷贝完成后释放硬件缓冲区
/// @param chn VENC编码通道RAII句柄
/// @param stream 硬件输出码流结构体
/// @return 封装好的EncodedPacket编码数据包，失败返回错误信息
[[nodiscard]] std::expected<EncodedPacket, std::string>
    extract_packet(VencChannel& chn, AX_VENC_STREAM_T& stream) noexcept;

/// 解析Axera平台底层错误码，转换为人类可读字符串
/// Axera错误码格式（AX_DEF_ERR宏定义规则）：
///   0x80000000 | (模块ID << 16) | (子模块ID << 8) | 细分错误ID
/// 示例输出："AX_ID_VENC:2 AX_ERR_ILLEGAL_PARAM(0x0A)"
[[nodiscard]] std::string ax_error_string(AX_S32 err) noexcept;

} // namespace quanta