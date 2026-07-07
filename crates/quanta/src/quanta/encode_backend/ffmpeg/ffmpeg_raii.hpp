#pragma once
// 头文件保护宏，防止头文件被重复多次包含

// C++标准库头文件
#include <memory>  // std::unique_ptr 独占智能指针，实现FFmpeg资源自动释放RAII
#include <string>  // 存储错误描述字符串，averr函数返回人类可读错误信息

// FFmpeg C语言底层API头文件，extern C 消除C++名称修饰冲突
extern "C" {
#include <libavcodec/avcodec.h>        // 编码器/解码器上下文、AVFrame、AVPacket 编解码核心API
#include <libavfilter/avfilter.h>      // 滤镜图基础结构体AVFilterGraph
#include <libavfilter/buffersink.h>    // 滤镜输出接收器 buffersink
#include <libavfilter/buffersrc.h>     // 滤镜输入源 buffersrc
#include <libavformat/avformat.h>      // 媒体封装解封装上下文 AVFormatContext，文件IO、流解析
#include <libavutil/avutil.h>          // FFmpeg基础工具：错误码解析、AVRational、辅助工具
#include <libavutil/opt.h>             // 编码器/滤镜私有参数配置 av_opt_set
#include <libswscale/swscale.h>       // 图像缩放、像素格式转换 SwsContext 软件缩放上下文
}

namespace quanta {
// 自定义删除器结构体：配合std::unique_ptr实现RAII自动释放FFmpeg原生C资源
// 所有FFmpeg原生资源必须使用对应API释放，不能直接delete/free，因此自定义deleter

/**
 * @brief 输入媒体文件AVFormatContext 解封装上下文删除器
 * 读取本地文件/网络流的输入上下文，销毁标准流程：avformat_close_input
 */
struct FmtCtxDeleter {
    // 重载()运算符，unique_ptr析构时自动调用
    void operator()(AVFormatContext* ctx) const {
        // 判空防止空指针调用底层API崩溃
        if (ctx)
            avformat_close_input(&ctx);
    }
};

/**
 * @brief 输出媒体封装AVFormatContext 封装上下文删除器
 * 用于推流/写入本地文件输出，销毁流程：先关闭IO上下文avio_closep，再释放格式上下文
 */
struct FmtOutCtxDeleter {
    void operator()(AVFormatContext* ctx) const {
        if (ctx) {
            // 存在IO缓存流则先关闭文件/网络IO句柄
            if (ctx->pb)
                avio_closep(&ctx->pb);
            // 释放封装上下文内存
            avformat_free_context(ctx);
        }
    }
};

/**
 * @brief AVCodecContext 编解码器上下文删除器
 * 编码器/解码器上下文统一释放接口 avcodec_free_context
 */
struct CodecCtxDeleter {
    void operator()(AVCodecContext* ctx) const { avcodec_free_context(&ctx); }
};

/**
 * @brief AVFrame 图像帧缓存删除器
 * 存放原始YUV/BGR像素帧，释放接口 av_frame_free
 */
struct FrameDeleter {
    void operator()(AVFrame* f) const { av_frame_free(&f); }
};

/**
 * @brief AVPacket 编码NALU包删除器
 * 存放压缩码流数据包，释放接口 av_packet_free
 */
struct PacketDeleter {
    void operator()(AVPacket* p) const { av_packet_free(&p); }
};

/**
 * @brief SwsContext 图像缩放/格式转换上下文删除器
 * swscale缩放转换器释放接口 sws_freeContext
 */
struct SwsCtxDeleter {
    void operator()(SwsContext* ctx) const { sws_freeContext(ctx); }
};

/**
 * @brief AVFilterGraph 滤镜图上下文删除器
 * 预处理滤镜流水线容器释放接口 avfilter_graph_free
 */
struct FilterGraphDeleter {
    void operator()(AVFilterGraph* g) const { avfilter_graph_free(&g); }
};

// 类型别名：封装unique_ptr+自定义删除器，对外提供简洁RAII智能指针类型
// 输入解封装上下文智能指针
using FmtCtxPtr      = std::unique_ptr<AVFormatContext, FmtCtxDeleter>;
// 输出封装/推流上下文智能指针
using FmtOutPtr      = std::unique_ptr<AVFormatContext, FmtOutCtxDeleter>;
// 编解码器上下文智能指针
using CodecCtxPtr    = std::unique_ptr<AVCodecContext, CodecCtxDeleter>;
// 原始图像帧智能指针
using FramePtr       = std::unique_ptr<AVFrame, FrameDeleter>;
// 编码压缩包智能指针
using PacketPtr      = std::unique_ptr<AVPacket, PacketDeleter>;
// 图像缩放转换上下文智能指针
using SwsCtxPtr      = std::unique_ptr<SwsContext, SwsCtxDeleter>;
// FFmpeg滤镜图智能指针
using FilterGraphPtr = std::unique_ptr<AVFilterGraph, FilterGraphDeleter>;

/**
 * @brief 打开媒体输入文件/流，创建输入解封装上下文并包装为RAII智能指针
 * @param path 媒体文件路径/RTSP/RTMP流地址
 * @return FmtCtxPtr 成功返回托管上下文；失败返回空unique_ptr（底层打开失败）
 */
inline FmtCtxPtr open_input(const char* path) {
    AVFormatContext* raw = nullptr;
    // 打开输入媒体，参数：上下文二级指针、路径、指定解封装器、额外参数
    if (avformat_open_input(&raw, path, nullptr, nullptr) != 0)
        // 打开失败返回空智能指针，自动释放空指针无操作
        return nullptr;
    // 成功将原生裸指针交给unique_ptr托管，离开作用域自动调用FmtCtxDeleter销毁
    return FmtCtxPtr(raw);
}

/**
 * @brief 根据指定编码器/解码器分配CodecContext，RAII托管
 * @param codec AVCodec 编码器/解码器裸指针
 * @return CodecCtxPtr 托管编解码上下文
 */
inline CodecCtxPtr alloc_codec_ctx(const AVCodec* codec) {
    // avcodec_alloc_context3 分配编解码上下文内存
    return CodecCtxPtr(avcodec_alloc_context3(codec));
}

/**
 * @brief 分配空AVFrame图像帧，RAII自动释放
 * @return FramePtr 帧智能指针
 */
inline FramePtr alloc_frame() { return FramePtr(av_frame_alloc()); }

/**
 * @brief 分配空AVPacket编码包，RAII自动释放
 * @return PacketPtr 包智能指针
 */
inline PacketPtr alloc_packet() { return PacketPtr(av_packet_alloc()); }

/**
 * @brief 分配空滤镜图AVFilterGraph，RAII自动销毁
 * @return FilterGraphPtr 滤镜图智能指针
 */
inline FilterGraphPtr alloc_filter_graph() { return FilterGraphPtr(avfilter_graph_alloc()); }

/**
 * @brief 将FFmpeg数字错误码转换为人类可读字符串
 * @param code FFmpeg底层接口返回的int错误码（负数）
 * @return std::string 错误描述文本
 */
inline std::string averr(int code) {
    // 128字节缓冲区存放错误文本
    char buf[128]{};
    // av_strerror：根据错误码填充缓冲区
    av_strerror(code, buf, sizeof(buf));
    // 转为C++字符串返回
    return std::string(buf);
}

/**
 * @brief 创建SwsContext图像缩放+像素格式转换上下文，RAII托管
 * @param src_w 输入图像宽度
 * @param src_h 输入图像高度
 * @param src_fmt 输入像素格式（AV_PIX_FMT_BGR24/AV_PIX_FMT_YUV420P等）
 * @param dst_w 输出图像宽度
 * @param dst_h 输出图像高度
 * @param dst_fmt 输出目标像素格式
 * @return SwsCtxPtr 缩放转换器智能指针，失败内部为nullptr
 */
inline SwsCtxPtr create_sws(
    int src_w, int src_h, AVPixelFormat src_fmt, int dst_w, int dst_h, AVPixelFormat dst_fmt) {
    // sws_getContext 创建缩放上下文，使用双线性插值SWS_BILINEAR，无自定义滤镜/参数
    return SwsCtxPtr(sws_getContext(
        src_w, src_h, src_fmt, dst_w, dst_h, dst_fmt, SWS_BILINEAR, nullptr, nullptr, nullptr));
}

} // namespace quanta