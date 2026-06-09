#pragma once

#include <memory>
#include <string>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

namespace quanta {

struct FmtCtxDeleter {
    void operator()(AVFormatContext* ctx) const {
        if (ctx)
            avformat_close_input(&ctx);
    }
};

struct FmtOutCtxDeleter {
    void operator()(AVFormatContext* ctx) const {
        if (ctx) {
            if (ctx->pb)
                avio_closep(&ctx->pb);
            avformat_free_context(ctx);
        }
    }
};

struct CodecCtxDeleter {
    void operator()(AVCodecContext* ctx) const { avcodec_free_context(&ctx); }
};

struct FrameDeleter {
    void operator()(AVFrame* f) const { av_frame_free(&f); }
};

struct PacketDeleter {
    void operator()(AVPacket* p) const { av_packet_free(&p); }
};

struct SwsCtxDeleter {
    void operator()(SwsContext* ctx) const { sws_freeContext(ctx); }
};

struct FilterGraphDeleter {
    void operator()(AVFilterGraph* g) const { avfilter_graph_free(&g); }
};

using FmtCtxPtr      = std::unique_ptr<AVFormatContext, FmtCtxDeleter>;
using FmtOutPtr      = std::unique_ptr<AVFormatContext, FmtOutCtxDeleter>;
using CodecCtxPtr    = std::unique_ptr<AVCodecContext, CodecCtxDeleter>;
using FramePtr       = std::unique_ptr<AVFrame, FrameDeleter>;
using PacketPtr      = std::unique_ptr<AVPacket, PacketDeleter>;
using SwsCtxPtr      = std::unique_ptr<SwsContext, SwsCtxDeleter>;
using FilterGraphPtr = std::unique_ptr<AVFilterGraph, FilterGraphDeleter>;

inline FmtCtxPtr open_input(const char* path) {
    AVFormatContext* raw = nullptr;
    if (avformat_open_input(&raw, path, nullptr, nullptr) != 0)
        return nullptr;
    return FmtCtxPtr(raw);
}

inline CodecCtxPtr alloc_codec_ctx(const AVCodec* codec) {
    return CodecCtxPtr(avcodec_alloc_context3(codec));
}

inline FramePtr alloc_frame() { return FramePtr(av_frame_alloc()); }
inline PacketPtr alloc_packet() { return PacketPtr(av_packet_alloc()); }

inline FilterGraphPtr alloc_filter_graph() { return FilterGraphPtr(avfilter_graph_alloc()); }

inline std::string averr(int code) {
    char buf[128]{};
    av_strerror(code, buf, sizeof(buf));
    return std::string(buf);
}

inline SwsCtxPtr create_sws(
    int src_w, int src_h, AVPixelFormat src_fmt, int dst_w, int dst_h, AVPixelFormat dst_fmt) {
    return SwsCtxPtr(sws_getContext(
        src_w, src_h, src_fmt, dst_w, dst_h, dst_fmt, SWS_BILINEAR, nullptr, nullptr, nullptr));
}

} // namespace quanta
