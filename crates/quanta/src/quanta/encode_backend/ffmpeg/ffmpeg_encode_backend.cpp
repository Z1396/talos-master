#include "quanta/encode_backend/encoder_backend.hpp"   // 编码器后端抽象基类定义
#include "quanta/encode_backend/ffmpeg/ffmpeg_raii.hpp" // FFmpeg资源RAII智能指针封装（FramePtr/CodecCtxPtr/FilterGraphPtr/PacketPtr）
#include "quanta/paramset/hevc_annexb.hpp"              // HEVC VPS/SPS/PPS参数集导出工具、状态结构体

// C++标准库头文件
#include <algorithm>    // std::min/std::max/std::clamp/std::copy_n 数值限制、内存拷贝
#include <array>        // 固定大小数组，复用帧缓存池
#include <cmath>        // 浮点运算、四舍五入lround
#include <cstdio>       // 标准输出打印日志fprintf(stderr)
#include <cstring>      // 内存拷贝memcpy、字符串操作
#include <deque>        // 编码包输出队列，FIFO缓存已编码NALU
#include <format>       // C++23格式化字符串，拼接滤镜参数、x265私有参数
#include <memory>       // 智能指针unique_ptr管理后端实例
#include <optional>     // 可选返回值，poll_packet无包时返回nullopt
#include <string>       // 错误信息、滤镜字符串、x265参数字符串存储
#include <utility>      // std::move 转移对象所有权

// 终端IO头文件（本文件未实际使用，项目全局统一引入）
#include <termios.h>

// FFmpeg C语言底层API，extern C消除C++名称修饰冲突
extern "C" {
#include <libavcodec/avcodec.h>        // 编码器、AVCodecContext、AVPacket、AVFrame编码核心接口
#include <libavfilter/buffersink.h>    // 滤镜图输出接收器
#include <libavfilter/buffersrc.h>     // 滤镜图输入源
#include <libavutil/avutil.h>          // 基础工具：错误码解析、AVRational时间基
#include <libavutil/imgutils.h>        // 图像内存分配工具
#include <libavutil/opt.h>             // 编码器私有参数配置av_opt_set
}

namespace quanta {
namespace { // 匿名命名空间，内部工具函数仅当前编译单元可见，不对外暴露

/**
 * @brief 分辨率适配工具：等比缩放原图尺寸，限制输出不超过max_w/max_h，强制输出宽高为偶数
 * @param src_w 原始输入宽度
 * @param src_h 原始输入高度
 * @param max_w 编码最大允许宽度上限
 * @param max_h 编码最大允许高度上限
 * @param out_w 输出适配后宽度（出参）
 * @param out_h 输出适配后高度（出参）
 */
void fit_dimensions(int src_w, int src_h, int max_w, int max_h, int& out_w, int& out_h) {
    // 计算宽、高两个方向缩放比例，取最小比例保证画面完整不裁剪
    double scale = std::min(static_cast<double>(max_w) / src_w, static_cast<double>(max_h) / src_h);
    // 缩放比例≥1，原图尺寸小于上限，直接使用原图分辨率
    if (scale >= 1.0) {
        out_w = src_w;
        out_h = src_h;
        return;
    }
    // 缩放后尺寸向下对齐2，HEVC编码硬性要求宽高偶数
    out_w = (static_cast<int>(src_w * scale) / 2) * 2;
    out_h = (static_cast<int>(src_h * scale) / 2) * 2;
    // 边界保护：防止缩放后尺寸为0，硬件编码器最小支持2像素
    if (out_w == 0)
        out_w = 2;
    if (out_h == 0)
        out_h = 2;
}

/**
 * @brief FFmpeg滤镜流水线上下文结构体
 * 承载完整滤镜图、输入/输出滤镜上下文，管理图像预处理流水线
 */
struct FilterChain {
    FilterGraphPtr graph;               // RAII托管滤镜图智能指针，自动销毁avfilter_graph
    AVFilterContext* src_ctx  = nullptr; // buffersrc输入滤镜上下文，原始BGR帧送入入口
    AVFilterContext* sink_ctx = nullptr;  // buffersink输出滤镜上下文，滤镜处理后YUV帧出口
};

/**
 * @brief 色彩元数据结构体，统一封装视频色域、色彩空间、色度采样位置信息
 * 用于写入编码器VUI元数据，播放器识别色彩渲染标准
 */
struct ColorParams {
    AVColorSpace colorspace                 = AVCOL_SPC_UNSPECIFIED;        // 色彩空间（BT709/BT601等）
    AVColorRange range                      = AVCOL_RANGE_UNSPECIFIED;        // 像素值范围：全范围/电视标准范围
    AVColorPrimaries color_primaries        = AVCOL_PRI_UNSPECIFIED;          // 色彩原色标准
    AVColorTransferCharacteristic color_trc = AVCOL_TRC_UNSPECIFIED;          // 伽马转换曲线
    AVChromaLocation chroma_location        = AVCHROMA_LOC_UNSPECIFIED;        // 色度采样偏移位置
};

/**
 * @brief 判断浮点滤镜参数是否启用（非0），浮点误差1e-6
 * @param value 滤镜强度参数
 * @return true 数值有效，启用滤镜；false 接近0，跳过该滤镜
 */
[[nodiscard]] bool filter_value_enabled(double value) noexcept {
    constexpr double kEpsilon = 1e-6;
    return std::abs(value) > kEpsilon;
}

/**
 * @brief 判断正数滤镜参数是否启用，仅判断大于极小值
 * @param value 正数强度参数
 * @return true 参数>1e-6，启用滤镜
 */
[[nodiscard]] bool positive_filter_enabled(double value) noexcept {
    constexpr double kEpsilon = 1e-6;
    return value > kEpsilon;
}

/**
 * @brief 判断高斯降噪是否开启：核尺寸大于1才生效
 * @param params 高斯降噪配置参数
 * @return true 需要插入降噪滤镜
 */
[[nodiscard]] bool gaussian_denoise_enabled(const GaussianDenoiseParams& params) noexcept {
    return params.kernel_size > 1;
}

/**
 * @brief 获取X方向降噪Sigma，未配置则默认1.0
 * @param params 高斯降噪参数
 * @return X方向模糊强度
 */
[[nodiscard]] double denoise_sigma_x(const GaussianDenoiseParams& params) noexcept {
    return positive_filter_enabled(params.sigma_x) ? params.sigma_x : 1.0;
}

/**
 * @brief 获取Y方向降噪Sigma，未配置则复用X方向值
 * @param params 高斯降噪参数
 * @param sigma_x 已计算X方向sigma
 * @return Y方向模糊强度
 */
[[nodiscard]] double denoise_sigma_y(const GaussianDenoiseParams& params, double sigma_x) noexcept {
    return positive_filter_enabled(params.sigma_y) ? params.sigma_y : sigma_x;
}

/**
 * @brief 转换高斯参数为hqdn3d空间降噪强度，线性映射3*(σx+σy)
 * @param params 高斯降噪配置
 * @return hqdn3d luma/spatial强度值
 */
[[nodiscard]] double hqdn3d_spatial_strength(const GaussianDenoiseParams& params) noexcept {
    // 降噪未开启直接返回0，不插入滤镜
    if (!gaussian_denoise_enabled(params))
        return 0.0;

    const double sigma_x = denoise_sigma_x(params);
    const double sigma_y = denoise_sigma_y(params, sigma_x);
    return 3.0 * (sigma_x + sigma_y);
}

/**
 * @brief 动态构建完整FFmpeg滤镜流水线字符串
 * 流水线顺序：格式转换→色彩元数据写入→帧率转换→缩放→饱和度→降噪→多阶锐化
 * @param src_w 原始输入宽度
 * @param src_h 原始输入高度
 * @param sar 原始像素宽高比
 * @param time_base 输入时间基
 * @param input_frame_rate 原始输入帧率
 * @param output_frame_rate 输出编码目标帧率
 * @param pix_fmt 输入原始像素格式（BGR24）
 * @param dst_w 编码输出分辨率宽
 * @param dst_h 编码输出分辨率高
 * @param fp 前端滤镜总开关+降噪/锐化/饱和度参数
 * @param color 色彩元数据配置
 * @return 成功返回FilterChain滤镜上下文，失败返回错误字符串
 */
std::expected<FilterChain, std::string> build_filter_chain(
    int src_w, int src_h, AVRational sar, AVRational time_base, AVRational input_frame_rate,
    AVRational output_frame_rate, AVPixelFormat pix_fmt, int dst_w, int dst_h,
    const FilterParams& fp, const ColorParams& color = {}) {

    // 1. 创建空滤镜图RAII智能指针
    auto graph = alloc_filter_graph();
    if (!graph)
        return std::unexpected("Cannot alloc filter graph");

    // 2. 获取内置输入/输出滤镜定义
    const AVFilter* buf_src  = avfilter_get_by_name("buffer");
    const AVFilter* buf_sink = avfilter_get_by_name("buffersink");
    if (!buf_src || !buf_sink)
        return std::unexpected("Missing buffer/buffersink filter");

    AVFilterContext* src_ctx  = nullptr;
    AVFilterContext* sink_ctx = nullptr;

    // 3. 拼接buffer输入滤镜初始化参数，描述原始输入帧属性
    std::string args = std::format(
        "video_size={}x{}:pix_fmt={}:sar={}/{}:time_base={}/{}:frame_rate={}/{}", src_w, src_h,
        static_cast<int>(pix_fmt), sar.num, sar.den, time_base.num, time_base.den,
        input_frame_rate.num, input_frame_rate.den);

    // 4. 创建输入滤镜上下文
    int ret =
        avfilter_graph_create_filter(&src_ctx, buf_src, "in", args.c_str(), nullptr, graph.get());
    if (ret < 0)
        return std::unexpected("Cannot create buffersrc: " + averr(ret));

    // 5. 创建输出滤镜上下文
    ret = avfilter_graph_create_filter(&sink_ctx, buf_sink, "out", nullptr, nullptr, graph.get());
    if (ret < 0)
        return std::unexpected("Cannot create buffersink: " + averr(ret));

    // 6. 拼接滤镜链字符串，逗号分隔串行滤镜
    std::string filters;
    // 内部工具：追加滤镜，自动处理前置逗号分隔符
    auto append_filter = [&filters](const std::string& filter) {
        if (!filters.empty()) {
            filters += ",";
        }
        filters += filter;
    };

    // 流水线第一步：强制转换为YUV420P，libx265仅支持该格式输入
    append_filter("format=yuv420p");
    // 写入色彩元数据到帧，VUI自动携带色域信息
    append_filter(
        std::format(
            "setparams=range={}:color_primaries={}:color_trc={}:colorspace={}:"
            "chroma_location={}",
            static_cast<int>(color.range), static_cast<int>(color.color_primaries),
            static_cast<int>(color.color_trc), static_cast<int>(color.colorspace),
            static_cast<int>(color.chroma_location)));
    // 可选帧率转换：输入帧率≠输出帧率时插入fps滤镜做帧插值/丢帧
    if (output_frame_rate.num > 0 && output_frame_rate.den > 0) {
        append_filter(
            std::format("fps=fps={}/{}:round=near", output_frame_rate.num, output_frame_rate.den));
    }
    // 缩放至编码目标分辨率
    append_filter(std::format("scale={}:{}", dst_w, dst_h));

    // 饱和度调节滤镜，仅开启且饱和度≠1.0时插入eq滤镜
    if (fp.enable && fp.enable_saturation && filter_value_enabled(fp.saturation - 1.0)) {
        append_filter(std::format("eq=saturation={:.2f}", fp.saturation));
    }

    // 读取降噪四个维度强度：亮度空间、色度空间、亮度时间、色度时间
    const double denoise_luma =
        (fp.enable && fp.enable_denoise_luma) ? hqdn3d_spatial_strength(fp.denoise_luma) : 0.0;
    const double denoise_chroma =
        (fp.enable && fp.enable_denoise_chroma) ? hqdn3d_spatial_strength(fp.denoise_chroma) : 0.0;
    const double denoise_tl = (fp.enable && fp.enable_denoise_tl) ? fp.denoise_tl : 0.0;
    const double denoise_tc = (fp.enable && fp.enable_denoise_tc) ? fp.denoise_tc : 0.0;
    // 任意降噪参数有效则插入hqdn3d时空降噪滤镜
    if (positive_filter_enabled(denoise_luma) || positive_filter_enabled(denoise_chroma)
        || positive_filter_enabled(denoise_tl) || positive_filter_enabled(denoise_tc)) {
        append_filter(
            std::format(
                "hqdn3d={:.1f}:{:.1f}:{:.1f}:{:.1f}", denoise_luma, denoise_chroma, denoise_tl,
                denoise_tc));
    }

    // 一阶5x5锐化unsharp
    const double sharpen1_luma = (fp.enable && fp.enable_sharpen1_luma) ? fp.sharpen1_luma : 0.0;
    const double sharpen1_chroma =
        (fp.enable && fp.enable_sharpen1_chroma) ? fp.sharpen1_chroma : 0.0;
    if (filter_value_enabled(sharpen1_luma) || filter_value_enabled(sharpen1_chroma)) {
        append_filter(std::format("unsharp=5:5:{:.1f}:5:5:{:.1f}", sharpen1_luma, sharpen1_chroma));
    }

    // 二阶5x5锐化叠加
    const double sharpen2_luma = (fp.enable && fp.enable_sharpen2_luma) ? fp.sharpen2_luma : 0.0;
    const double sharpen2_chroma =
        (fp.enable && fp.enable_sharpen2_chroma) ? fp.sharpen2_chroma : 0.0;
    if (filter_value_enabled(sharpen2_luma) || filter_value_enabled(sharpen2_chroma)) {
        append_filter(std::format("unsharp=5:5:{:.1f}:5:5:{:.1f}", sharpen2_luma, sharpen2_chroma));
    }

    // 三阶3x3精细锐化叠加
    const double sharpen3_luma = (fp.enable && fp.enable_sharpen3_luma) ? fp.sharpen3_luma : 0.0;
    const double sharpen3_chroma =
        (fp.enable && fp.enable_sharpen3_chroma) ? fp.sharpen3_chroma : 0.0;
    if (filter_value_enabled(sharpen3_luma) || filter_value_enabled(sharpen3_chroma)) {
        append_filter(std::format("unsharp=3:3:{:.1f}:3:3:{:.1f}", sharpen3_luma, sharpen3_chroma));
    }

    // 7. 分配滤镜输入输出链表结构体
    AVFilterInOut* outputs = avfilter_inout_alloc();
    AVFilterInOut* inputs  = avfilter_inout_alloc();
    if (!outputs || !inputs) {
        avfilter_inout_free(&outputs);
        avfilter_inout_free(&inputs);
        return std::unexpected("Cannot alloc filter inouts");
    }

    // 绑定输入滤镜输出端到graph解析器
    outputs->name       = av_strdup("in");
    outputs->filter_ctx = src_ctx;
    outputs->pad_idx    = 0;
    outputs->next       = nullptr;

    // 绑定输出滤镜输入端到graph解析器
    inputs->name       = av_strdup("out");
    inputs->filter_ctx = sink_ctx;
    inputs->pad_idx    = 0;
    inputs->next       = nullptr;

    // 8. 解析滤镜字符串构建连接关系
    ret = avfilter_graph_parse_ptr(graph.get(), filters.c_str(), &inputs, &outputs, nullptr);
    // 释放临时链表内存
    avfilter_inout_free(&outputs);
    avfilter_inout_free(&inputs);
    if (ret < 0)
        return std::unexpected("Cannot parse filter graph: " + averr(ret));

    // 9. 配置滤镜图，分配内部缓冲区、校验链路合法性
    ret = avfilter_graph_config(graph.get(), nullptr);
    if (ret < 0)
        return std::unexpected("Cannot config filter graph: " + averr(ret));

    // 10. 返回完整滤镜上下文
    return FilterChain{
        .graph    = std::move(graph),
        .src_ctx  = src_ctx,
        .sink_ctx = sink_ctx,
    };
}

/**
 * @brief 创建libx265编码器上下文，将上层EncodeParams完整映射至AVCodecContext与x265私有参数
 * @param width 编码输出宽度
 * @param height 编码输出高度
 * @param time_base 编码时间基
 * @param frame_rate 输出帧率
 * @param params 上层编码总参数（码率、GOP、画质、滤镜、实时模式开关）
 * @param sample_aspect_ratio 像素宽高比
 * @param color 色彩VUI元数据
 * @param repeat_headers 实时流：每IDR帧重复输出VPS/SPS/PPS
 * @param global_header 是否全局头模式（文件存储用，实时流关闭）
 * @return 成功返回CodecCtxPtr RAII编码器上下文，失败返回错误字符串
 */
std::expected<CodecCtxPtr, std::string> create_encoder_context(
    int width, int height, AVRational time_base, AVRational frame_rate, const EncodeParams& params,
    AVRational sample_aspect_ratio = {1, 1}, const ColorParams& color = {},
    bool repeat_headers = false, bool global_header = false) {

    // 1. 查找libx265软件编码器
    const auto* enc_codec = avcodec_find_encoder_by_name("libx265");
    if (!enc_codec)
        return std::unexpected("libx265 encoder not found");

    // 2. RAII智能指针分配编码器上下文
    auto enc_ctx = alloc_codec_ctx(enc_codec);
    if (!enc_ctx)
        return std::unexpected("Cannot allocate encoder context");

    // 标记是否实时低延迟推流模式（repeat_headers开启、无全局头=RTSP/RTMP实时流）
    const bool realtime_stream = repeat_headers && !global_header;
    // 码率边界保护，最小1kbps
    const auto bitrate_bps     = std::max<int64_t>(1'000, params.target_bitrate);
    const auto bitrate_kbps =
        std::max(1, static_cast<int>(std::lround(static_cast<double>(bitrate_bps) / 1000.0)));

    // 3. 基础编码器全局参数配置
    // 实时流启用CBR恒定码率，文件模式CRF画质模式置0
    enc_ctx->bit_rate               = realtime_stream ? bitrate_bps : 0;
    enc_ctx->rc_max_rate            = realtime_stream ? bitrate_bps : 0;
    enc_ctx->rc_min_rate            = realtime_stream ? bitrate_bps : 0;
    enc_ctx->width                  = width;
    enc_ctx->height                 = height;
    enc_ctx->time_base              = time_base;
    enc_ctx->framerate              = frame_rate;
    enc_ctx->gop_size               = params.gop_size;
    enc_ctx->max_b_frames           = 0; // 实时流禁用B帧降低延迟
    enc_ctx->pix_fmt                = AV_PIX_FMT_YUV420P; // x265唯一支持输入格式
    enc_ctx->profile                = AV_PROFILE_HEVC_MAIN; // HEVC主档次
    enc_ctx->sample_aspect_ratio    = sample_aspect_ratio;
    // 色彩VUI元数据写入
    enc_ctx->colorspace             = color.colorspace;
    enc_ctx->color_range            = color.range;
    enc_ctx->color_primaries        = color.color_primaries;
    enc_ctx->color_trc              = color.color_trc;
    enc_ctx->chroma_sample_location = color.chroma_location;
    // 全局头标记：文件存储封装MP4使用，实时流关闭
    if (global_header)
        enc_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    // 实时流VBV缓冲延迟预算0.25秒
    constexpr double kLatencyBudgetSeconds = 0.25;

    // 计算实际帧率浮点值
    const double frame_rate_fps =
        frame_rate.num > 0 && frame_rate.den > 0
            ? static_cast<double>(frame_rate.num) / static_cast<double>(frame_rate.den)
            : static_cast<double>(params.framerate);
    // GOP一周期时长（秒）
    const double gop_seconds =
        frame_rate_fps > 0.0 ? static_cast<double>(params.gop_size) / frame_rate_fps : 0.0;
    // VBV缓冲窗口：实时流固定0.25s，文件模式限制1~1.5s GOP时长
    const double desired_vbv_window_seconds =
        realtime_stream ? kLatencyBudgetSeconds : std::clamp(gop_seconds, 1.0, 1.5);
    // 计算VBV缓冲大小（kb）
    const auto vbv_bufsize = std::max(
        1, static_cast<int>(
               std::lround(static_cast<double>(bitrate_kbps) * desired_vbv_window_seconds)));
    // 实时流启用VBV缓冲约束CBR，文件模式关闭
    enc_ctx->rc_buffer_size = realtime_stream ? vbv_bufsize * 1000 : 0;

    // 预设preset参数（ultrafast~veryslow）
    if (!params.preset.empty())
        av_opt_set(enc_ctx->priv_data, "preset", params.preset.c_str(), 0);
    // 实时流强制zerolatency调参，无自定义tune则使用默认低延迟
    if (realtime_stream) {
        const char* tune = params.tune.empty() ? "zerolatency" : params.tune.c_str();
        av_opt_set(enc_ctx->priv_data, "tune", tune, 0);
    }

    // x265内部私有参数固定配置
    const auto min_keyint   = std::max(1, params.gop_size / 2); // 最小I帧间隔
    const int frame_threads = realtime_stream ? 1 : 0;          // 实时流单线程降低延迟
    const int ref_count     = realtime_stream ? 1 : 10;         // 参考帧数量
    const int limit_refs    = realtime_stream ? 0 : 3;          // 参考帧限制
    const char* me_mode     = "star";                           // 运动估计算法
    const int subme_level   = 7;                                // 子像素运动估计层级
    const int rd_level      = 5;                                // 率失真优化层级
    const int rdoq_level    = 2;                                // 量化率失真优化
    const int amp_enabled   = 1;                                // 自适应多变换开启
    const int rect_enabled  = 1;                                // 矩形运动估计开启
    const int cb_qp_offset  = std::max(0, params.chroma_qp_offset); // Cb色度QP偏移
    const int cr_qp_offset  = std::max(0, params.chroma_qp_offset); // Cr色度QP偏移

    // 读取前端心理视觉优化参数
    const auto& fp           = params.filter;
    const double psy_rd      = (fp.enable && fp.enable_psy_rd) ? fp.psy_rd : 0.0;
    const double psy_trellis = (fp.enable && fp.enable_psy_trellis) ? fp.psy_trellis : 0.0;
    std::string x265_opts;

    // 分支1：实时CBR推流模式参数
    if (realtime_stream) {
        const int lookahead = std::clamp(params.lookahead, 0, 4); // 前瞻帧限制0~4低延迟
        x265_opts           = std::format(
            "bitrate={}:vbv-maxrate={}:vbv-bufsize={}:vbv-init=0.9:strict-cbr=1:"
                      "keyint={}:min-keyint={}:scenecut={}:open-gop=0:intra-refresh={}:"
                      "rc-lookahead={}:bframes={}:frame-threads={}:"
                      "ref={}:limit-refs={}:me={}:subme={}:rd={}:rdoq-level={}:"
                      "rect={}:amp={}:aq-mode={}:aq-strength={}:aq-motion=0:cutree=0:"
                      "cbqpoffs={}:crqpoffs={}:psy-rd={}:psy-rdoq={}:qcomp=0.5:qpstep=8:"
                      "deblock=0,0:no-sao=1:info=0:vui-timing-info=0:vui-hrd-info=0:repeat-headers={}",
            bitrate_kbps, bitrate_kbps, vbv_bufsize, params.gop_size, min_keyint,
            params.enScenecut ? 1 : 0, params.intra_refresh ? 1 : 0, lookahead, params.bframe,
            frame_threads, ref_count, limit_refs, me_mode, subme_level, rd_level, rdoq_level,
            rect_enabled, amp_enabled, params.enAqMode ? 1 : 0, params.aq_strength, cb_qp_offset,
            cr_qp_offset, psy_rd, psy_trellis, repeat_headers ? 1 : 0);
    } else {
        // 分支2：文件存储CRF恒定画质模式
        const int lookahead = params.lookahead;

        x265_opts = std::format(
            "crf={}:vbv-maxrate={}:vbv-bufsize={}:"
            "keyint={}:min-keyint={}:scenecut=5:open-gop=0:rc-lookahead={}:bframes=0:frame-threads="
            "{}:"
            "ref={}:limit-refs={}:me={}:subme={}:rd={}:rdoq-level={}:"
            "rect={}:amp={}:"
            "aq-mode=3:aq-strength={}:aq-motion=1:cutree=1:"
            "cbqpoffs={}:crqpoffs={}:psy-rd={}:psy-rdoq={}:"
            "deblock=1,0:no-sao=1:repeat-headers={}",
            params.crf, bitrate_kbps, vbv_bufsize, params.gop_size, min_keyint, lookahead,
            frame_threads, ref_count, limit_refs, me_mode, subme_level, rd_level, rdoq_level,
            rect_enabled, amp_enabled, params.aq_strength, cb_qp_offset, cr_qp_offset, psy_rd,
            psy_trellis, repeat_headers ? 1 : 0);
    }
    // 写入x265私有参数字符串
    av_opt_set(enc_ctx->priv_data, "x265-params", x265_opts.c_str(), 0);

    // 打开编码器上下文，完成初始化
    if (avcodec_open2(enc_ctx.get(), enc_codec, nullptr) < 0)
        return std::unexpected("Cannot open encoder");

    return enc_ctx;
}

} // namespace 匿名内部工具函数结束

// ===========================================================================
// FfmpegBackend — FFmpeg/libx265 软件编码器后端实现
// 继承EncoderBackend抽象基类，统一对外编码接口
// ===========================================================================
class FfmpegBackend final : public EncoderBackend {
public:
    // BGR输入帧复用池大小，3帧环形缓存
    static constexpr size_t kInputFramePoolSize = 3;
    // buffersrc标记：送入帧后保持帧引用，避免帧被自动销毁
    static constexpr int kBufferSrcFlags        = AV_BUFFERSRC_FLAG_KEEP_REF;

    /**
     * @brief 后端工厂静态创建函数，构造实例并执行初始化流水线
     * @param params 编码总参数
     * @param src_width 原始输入宽度
     * @param src_height 原始输入高度
     * @param framerate 输入帧率
     * @return 成功返回unique_ptr后端实例，失败返回错误字符串，捕获所有标准异常
     */
    [[nodiscard]] static std::expected<std::unique_ptr<FfmpegBackend>, std::string>
        create(EncodeParams params, int src_width, int src_height, int framerate) noexcept try {
        // 分配后端实例，私有构造仅静态函数可调用
        auto backend = std::unique_ptr<FfmpegBackend>(new FfmpegBackend(std::move(params)));
        // 执行完整初始化：分辨率适配、滤镜图构建、编码器创建、帧池分配
        auto result  = backend->setup(src_width, src_height, framerate);
        if (!result)
            return std::unexpected(std::move(result.error()));
        return backend;
    } catch (const std::exception& e) {
        // 捕获构造/初始化阶段所有C++异常，包装错误返回
        return std::unexpected(std::format("FfmpegBackend::create: {}", e.what()));
    }

    // 析构默认自动调用RAII智能指针释放FFmpeg资源
    ~FfmpegBackend() override = default;

    // 禁用拷贝，编码器独占滤镜、编码器上下文、帧池资源
    FfmpegBackend(const FfmpegBackend&)            = delete;
    FfmpegBackend& operator=(const FfmpegBackend&) = delete;
    // 允许移动语义，转移后端所有权
    FfmpegBackend(FfmpegBackend&&)                 = default;
    FfmpegBackend& operator=(FfmpegBackend&&)      = default;

    /**
     * @brief 推送一帧原始BGR24图像进入编码流水线
     * 流程：环形帧池拷贝BGR数据→送入滤镜图→循环拉取处理后YUV帧→送入编码器→缓存编码包
     * @param data BGR24原始像素内存首地址
     * @param linesize 输入单行字节宽度
     * @param pts 帧显示时间戳
     * @return 成功空expected，失败携带错误信息
     */
    std::expected<void, std::string>
        push_frame(const uint8_t* data, int linesize, int64_t pts) noexcept override try {
        // 取环形池下一空闲BGR帧缓存
        AVFrame* bgr_frame = bgr_frames_[next_bgr_frame_].get();
        next_bgr_frame_    = (next_bgr_frame_ + 1) % bgr_frames_.size();

        // 标记帧可写，清除内部只读引用锁
        int ret = av_frame_make_writable(bgr_frame);
        if (ret < 0)
            return std::unexpected("Cannot make BGR frame writable: " + averr(ret));

        // 写入帧时间戳
        bgr_frame->pts = pts;

        // 逐行拷贝BGR像素，区分输入行宽与帧内部行宽是否对齐
        if (linesize == input_row_bytes_ && bgr_frame->linesize[0] == input_row_bytes_) {
            // 内存连续整块拷贝
            std::memcpy(
                bgr_frame->data[0], data, static_cast<size_t>(input_row_bytes_) * src_height_);
        } else {
            // 行宽不一致，逐行拷贝避免跨行列错位
            for (int y = 0; y < src_height_; ++y) {
                std::memcpy(
                    bgr_frame->data[0] + y * bgr_frame->linesize[0], data + y * linesize,
                    input_row_bytes_);
            }
        }

        // 将BGR帧送入滤镜图输入口
        ret = av_buffersrc_add_frame_flags(chain_.src_ctx, bgr_frame, kBufferSrcFlags);
        if (ret < 0)
            return std::unexpected("Filter push failed: " + averr(ret));

        // 循环拉取滤镜处理完成的YUV帧，送入编码器编码
        while (av_buffersink_get_frame(chain_.sink_ctx, filt_frame_.get()) >= 0) {
            // 原子交换关键帧请求标记，置false
            const bool force_keyframe = std::exchange(force_keyframe_requested_, false);
            // 编码处理后帧
            auto r                    = encode_filtered_frame(filt_frame_.get(), force_keyframe);
            if (!r)
                return r;
            // 释放帧内部引用，复用缓存
            av_frame_unref(filt_frame_.get());
        }

        return {};
    } catch (const std::exception& e) {
        return std::unexpected(std::format("FfmpegBackend::push_frame: {}", e.what()));
    }

    /**
     * @brief 异步请求下一帧强制生成IDR关键帧
     */
    void request_keyframe() noexcept override { force_keyframe_requested_ = true; }

    /**
     * @brief 轮询读取已编码NALU包，无包返回std::nullopt
     * @return 编码包结构体，包含裸流、时间戳、关键帧标记
     */
    std::optional<EncodedPacket> poll_packet() noexcept override {
        if (packet_queue_.empty())
            return std::nullopt;
        // 队首出队转移所有权
        auto pkt = std::move(packet_queue_.front());
        packet_queue_.pop_front();
        return pkt;
    }

    /**
     * @brief 编码器冲刷：发送空帧输出所有缓存未输出编码包，流结束收尾调用
     * @return 冲刷结果，失败返回错误信息
     */
    std::expected<void, std::string> flush() noexcept override try {
        // 推送空帧触发编码器收尾
        int ret = avcodec_send_frame(enc_ctx_.get(), nullptr);
        if (ret < 0)
            return std::unexpected("Flush send failed: " + averr(ret));

        // 循环读取剩余缓存码流
        while (true) {
            ret = avcodec_receive_packet(enc_ctx_.get(), enc_pkt_.get());
            // EOF/无包退出循环
            if (ret == AVERROR_EOF || ret == AVERROR(EAGAIN))
                break;
            if (ret < 0)
                return std::unexpected("Flush recv failed: " + averr(ret));

            // 编码包入队缓存
            enqueue_packet(enc_pkt_.get());
            av_packet_unref(enc_pkt_.get());
        }

        return {};
    } catch (const std::exception& e) {
        return std::unexpected(std::format("FfmpegBackend::flush: {}", e.what()));
    }

    // 获取当前编码参数只读引用
    const EncodeParams& params() const noexcept override { return params_; }
    // 获取输出编码分辨率宽高对
    std::pair<int, int> dimensions() const noexcept override { return {out_width_, out_height_}; }

private:
    // 私有构造，仅静态create函数可实例化
    explicit FfmpegBackend(EncodeParams params)
        : params_(std::move(params)) {}

    /**
     * @brief 完整初始化流水线：分辨率计算→滤镜图构建→编码器创建→帧缓存池分配
     * @param src_width 原始输入宽
     * @param src_height 原始输入高
     * @param framerate 输入帧率
     * @return 初始化成功空expected，失败返回错误字符串
     */
    std::expected<void, std::string> setup(int src_width, int src_height, int framerate) {
        src_width_  = src_width;
        src_height_ = src_height;
        framerate_  = framerate;

        // 计算输出适配分辨率
        int out_w{}, out_h{};
        fit_dimensions(src_width, src_height, params_.max_width, params_.max_height, out_w, out_h);
        out_width_  = out_w;
        out_height_ = out_h;

        const AVRational time_base  = {1, framerate};
        const AVRational frame_rate = {framerate, 1};

        // 构建完整预处理滤镜流水线
        auto filter_result = build_filter_chain(
            src_width, src_height, {1, 1}, time_base, frame_rate, {0, 1}, AV_PIX_FMT_BGR24, out_w,
            out_h, params_.filter);
        if (!filter_result)
            return std::unexpected("Cannot build filter chain: " + filter_result.error());
        chain_ = std::move(filter_result.value());

        // 创建libx265编码器上下文，实时流开启repeat_headers
        auto enc_result =
            create_encoder_context(out_w, out_h, time_base, frame_rate, params_, {1, 1}, {}, true);
        if (!enc_result)
            return std::unexpected(enc_result.error());
        enc_ctx_ = std::move(enc_result.value());

        // 分配复用帧、包缓存
        filt_frame_ = alloc_frame();
        enc_pkt_    = alloc_packet();
        if (!filt_frame_ || !enc_pkt_)
            return std::unexpected("Cannot allocate reusable frame/packet");

        // 批量初始化环形BGR帧池
        for (auto& frame : bgr_frames_) {
            frame = alloc_frame();
            if (!frame)
                return std::unexpected("Cannot allocate reusable BGR frame");
            frame->format = AV_PIX_FMT_BGR24;
            frame->width  = src_width_;
            frame->height = src_height_;
            int ret       = av_frame_get_buffer(frame.get(), 0);
            if (ret < 0)
                return std::unexpected("Cannot allocate BGR frame buffer: " + averr(ret));
        }

        // 单行BGR字节：宽度×3通道
        input_row_bytes_ = src_width_ * 3;

        // 控制台打印初始化日志
        std::fprintf(
            stderr, "[quanta::stream][ffmpeg] %dx%d @ %dfps -> %dx%d, target %d bps\n", src_width,
            src_height, framerate, out_w, out_h, params_.target_bitrate);
        return {};
    }

    /**
     * @brief 送入滤镜处理完成YUV帧至编码器，读取输出码流包
     * @param frame 滤镜输出YUV420P帧
     * @param force_keyframe 是否强制本帧IDR关键帧
     * @return 编码执行结果
     */
    std::expected<void, std::string> encode_filtered_frame(AVFrame* frame, bool force_keyframe) {
        // 标记强制关键帧
        if (force_keyframe) {
            frame->flags |= AV_FRAME_FLAG_KEY;
            frame->pict_type = AV_PICTURE_TYPE_I;
        } else {
            frame->flags &= ~AV_FRAME_FLAG_KEY;
            frame->pict_type = AV_PICTURE_TYPE_NONE;
        }
        // 帧送入编码器
        int ret = avcodec_send_frame(enc_ctx_.get(), frame);
        if (ret < 0)
            return std::unexpected("Encode send: " + averr(ret));

        // 循环读取编码器输出NALU包
        while (ret >= 0) {
            ret = avcodec_receive_packet(enc_ctx_.get(), enc_pkt_.get());
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                break;
            if (ret < 0)
                return std::unexpected("Encode recv: " + averr(ret));

            // 编码包入队缓存
            enqueue_packet(enc_pkt_.get());
            av_packet_unref(enc_pkt_.get());
        }
        return {};
    }

    /**
     * @brief 拷贝AVPacket裸流数据，封装为上层EncodedPacket，自动导出HEVC VPS/SPS/PPS参数集
     * @param pkt FFmpeg原始编码包
     */
    void enqueue_packet(const AVPacket* pkt) {
        EncodedPacket encoded;
        // 分配裸流内存并拷贝
        if (pkt->size > 0) {
            encoded.data = std::unique_ptr<uint8_t[]>(new uint8_t[pkt->size]);
            std::copy_n(pkt->data, pkt->size, encoded.data.get());
        }
        encoded.size     = static_cast<size_t>(pkt->size);
        encoded.pts      = pkt->pts;
        encoded.keyframe = (pkt->flags & AV_PKT_FLAG_KEY) != 0;

        // 导出一次VPS/SPS/PPS参数集写入本地文件
        const bool was_exported = parameter_set_state_.exported;
        const auto result       = export_hevc_parameter_sets_once(encoded, parameter_set_state_);
        if (!result) {
            std::fprintf(
                stderr, "[quanta::stream][ffmpeg] export VPS/SPS/PPS failed: %s\n",
                result.error().c_str());
        } else if (!was_exported && parameter_set_state_.exported) {
            std::fprintf(
                stderr, "[quanta::stream][ffmpeg] VPS/SPS/PPS exported to "
                        "output/quanta_vps_sps_pps.hevc\n");
        }

        // 非空包入队缓存
        if (encoded.size > 0) {
            packet_queue_.push_back(std::move(encoded));
        }
    }

    EncodeParams params_; // 全局编码参数副本

    int src_width_  = 0;  // 原始输入分辨率宽
    int src_height_ = 0;  // 原始输入分辨率高
    int out_width_  = 0;  // 编码输出分辨率宽
    int out_height_ = 0;  // 编码输出分辨率高
    int framerate_  = 0;  // 输入帧率

    FilterChain chain_;               // 完整预处理滤镜流水线上下文
    CodecCtxPtr enc_ctx_;             // libx265编码器RAII上下文
    std::array<FramePtr, kInputFramePoolSize> bgr_frames_; // BGR原始帧环形缓存池
    FramePtr filt_frame_;            // 滤镜输出YUV复用帧
    PacketPtr enc_pkt_;              // 编码器输出包复用缓存
    std::deque<EncodedPacket> packet_queue_; // 已编码NALU输出队列
    HevcParameterSetExportState parameter_set_state_; // HEVC参数集导出状态（仅导出一次）
    int input_row_bytes_           = 0; // BGR单行字节宽度
    size_t next_bgr_frame_         = 0; // 环形帧池索引指针
    bool force_keyframe_requested_ = false; // 下一帧强制IDR标记
};

// ===========================================================================
// 后端工厂对外接口
// ===========================================================================
/**
 * @brief 对外统一创建FFmpeg软件编码器后端接口，向上层抽象工厂暴露
 * @param params 编码参数
 * @param src_width 原始输入宽
 * @param src_height 原始输入高
 * @param framerate 输入帧率
 * @return 编码器后端抽象基类智能指针，统一多后端兼容接口
 */
std::expected<std::unique_ptr<EncoderBackend>, std::string> create_ffmpeg_backend(
    EncodeParams params, int src_width, int src_height, int framerate) noexcept {
    auto backend = FfmpegBackend::create(std::move(params), src_width, src_height, framerate);
    if (!backend)
        return std::unexpected(std::move(backend.error()));
    return std::unique_ptr<EncoderBackend>(std::move(*backend));
}

} // namespace quanta