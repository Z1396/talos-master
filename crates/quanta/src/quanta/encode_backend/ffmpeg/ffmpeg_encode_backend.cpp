#include "quanta/encode_backend/encoder_backend.hpp"
#include "quanta/encode_backend/ffmpeg/ffmpeg_raii.hpp"
#include "quanta/paramset/hevc_annexb.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <format>
#include <memory>
#include <optional>
#include <string>
#include <termios.h>
#include <utility>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}

namespace quanta {
namespace {

void fit_dimensions(int src_w, int src_h, int max_w, int max_h, int& out_w, int& out_h) {
    double scale = std::min(static_cast<double>(max_w) / src_w, static_cast<double>(max_h) / src_h);
    if (scale >= 1.0) {
        out_w = src_w;
        out_h = src_h;
        return;
    }
    out_w = (static_cast<int>(src_w * scale) / 2) * 2;
    out_h = (static_cast<int>(src_h * scale) / 2) * 2;
    if (out_w == 0)
        out_w = 2;
    if (out_h == 0)
        out_h = 2;
}

struct FilterChain {
    FilterGraphPtr graph;
    AVFilterContext* src_ctx  = nullptr;
    AVFilterContext* sink_ctx = nullptr;
};

struct ColorParams {
    AVColorSpace colorspace                 = AVCOL_SPC_UNSPECIFIED;
    AVColorRange range                      = AVCOL_RANGE_UNSPECIFIED;
    AVColorPrimaries color_primaries        = AVCOL_PRI_UNSPECIFIED;
    AVColorTransferCharacteristic color_trc = AVCOL_TRC_UNSPECIFIED;
    AVChromaLocation chroma_location        = AVCHROMA_LOC_UNSPECIFIED;
};

[[nodiscard]] bool filter_value_enabled(double value) noexcept {
    constexpr double kEpsilon = 1e-6;
    return std::abs(value) > kEpsilon;
}

[[nodiscard]] bool positive_filter_enabled(double value) noexcept {
    constexpr double kEpsilon = 1e-6;
    return value > kEpsilon;
}

[[nodiscard]] bool gaussian_denoise_enabled(const GaussianDenoiseParams& params) noexcept {
    return params.kernel_size > 1;
}

[[nodiscard]] double denoise_sigma_x(const GaussianDenoiseParams& params) noexcept {
    return positive_filter_enabled(params.sigma_x) ? params.sigma_x : 1.0;
}

[[nodiscard]] double denoise_sigma_y(const GaussianDenoiseParams& params, double sigma_x) noexcept {
    return positive_filter_enabled(params.sigma_y) ? params.sigma_y : sigma_x;
}

[[nodiscard]] double hqdn3d_spatial_strength(const GaussianDenoiseParams& params) noexcept {
    if (!gaussian_denoise_enabled(params))
        return 0.0;

    const double sigma_x = denoise_sigma_x(params);
    const double sigma_y = denoise_sigma_y(params, sigma_x);
    return 3.0 * (sigma_x + sigma_y);
}

std::expected<FilterChain, std::string> build_filter_chain(
    int src_w, int src_h, AVRational sar, AVRational time_base, AVRational input_frame_rate,
    AVRational output_frame_rate, AVPixelFormat pix_fmt, int dst_w, int dst_h,
    const FilterParams& fp, const ColorParams& color = {}) {

    auto graph = alloc_filter_graph();
    if (!graph)
        return std::unexpected("Cannot alloc filter graph");

    const AVFilter* buf_src  = avfilter_get_by_name("buffer");
    const AVFilter* buf_sink = avfilter_get_by_name("buffersink");
    if (!buf_src || !buf_sink)
        return std::unexpected("Missing buffer/buffersink filter");

    AVFilterContext* src_ctx  = nullptr;
    AVFilterContext* sink_ctx = nullptr;

    std::string args = std::format(
        "video_size={}x{}:pix_fmt={}:sar={}/{}:time_base={}/{}:frame_rate={}/{}", src_w, src_h,
        static_cast<int>(pix_fmt), sar.num, sar.den, time_base.num, time_base.den,
        input_frame_rate.num, input_frame_rate.den);

    int ret =
        avfilter_graph_create_filter(&src_ctx, buf_src, "in", args.c_str(), nullptr, graph.get());
    if (ret < 0)
        return std::unexpected("Cannot create buffersrc: " + averr(ret));

    ret = avfilter_graph_create_filter(&sink_ctx, buf_sink, "out", nullptr, nullptr, graph.get());
    if (ret < 0)
        return std::unexpected("Cannot create buffersink: " + averr(ret));

    std::string filters;
    auto append_filter = [&filters](const std::string& filter) {
        if (!filters.empty()) {
            filters += ",";
        }
        filters += filter;
    };

    append_filter("format=yuv420p");
    append_filter(
        std::format(
            "setparams=range={}:color_primaries={}:color_trc={}:colorspace={}:"
            "chroma_location={}",
            static_cast<int>(color.range), static_cast<int>(color.color_primaries),
            static_cast<int>(color.color_trc), static_cast<int>(color.colorspace),
            static_cast<int>(color.chroma_location)));
    if (output_frame_rate.num > 0 && output_frame_rate.den > 0) {
        append_filter(
            std::format("fps=fps={}/{}:round=near", output_frame_rate.num, output_frame_rate.den));
    }
    append_filter(std::format("scale={}:{}", dst_w, dst_h));

    if (fp.enable && fp.enable_saturation && filter_value_enabled(fp.saturation - 1.0)) {
        append_filter(std::format("eq=saturation={:.2f}", fp.saturation));
    }

    const double denoise_luma =
        (fp.enable && fp.enable_denoise_luma) ? hqdn3d_spatial_strength(fp.denoise_luma) : 0.0;
    const double denoise_chroma =
        (fp.enable && fp.enable_denoise_chroma) ? hqdn3d_spatial_strength(fp.denoise_chroma) : 0.0;
    const double denoise_tl = (fp.enable && fp.enable_denoise_tl) ? fp.denoise_tl : 0.0;
    const double denoise_tc = (fp.enable && fp.enable_denoise_tc) ? fp.denoise_tc : 0.0;
    if (positive_filter_enabled(denoise_luma) || positive_filter_enabled(denoise_chroma)
        || positive_filter_enabled(denoise_tl) || positive_filter_enabled(denoise_tc)) {
        append_filter(
            std::format(
                "hqdn3d={:.1f}:{:.1f}:{:.1f}:{:.1f}", denoise_luma, denoise_chroma, denoise_tl,
                denoise_tc));
    }

    const double sharpen1_luma = (fp.enable && fp.enable_sharpen1_luma) ? fp.sharpen1_luma : 0.0;
    const double sharpen1_chroma =
        (fp.enable && fp.enable_sharpen1_chroma) ? fp.sharpen1_chroma : 0.0;
    if (filter_value_enabled(sharpen1_luma) || filter_value_enabled(sharpen1_chroma)) {
        append_filter(std::format("unsharp=5:5:{:.1f}:5:5:{:.1f}", sharpen1_luma, sharpen1_chroma));
    }

    const double sharpen2_luma = (fp.enable && fp.enable_sharpen2_luma) ? fp.sharpen2_luma : 0.0;
    const double sharpen2_chroma =
        (fp.enable && fp.enable_sharpen2_chroma) ? fp.sharpen2_chroma : 0.0;
    if (filter_value_enabled(sharpen2_luma) || filter_value_enabled(sharpen2_chroma)) {
        append_filter(std::format("unsharp=5:5:{:.1f}:5:5:{:.1f}", sharpen2_luma, sharpen2_chroma));
    }

    const double sharpen3_luma = (fp.enable && fp.enable_sharpen3_luma) ? fp.sharpen3_luma : 0.0;
    const double sharpen3_chroma =
        (fp.enable && fp.enable_sharpen3_chroma) ? fp.sharpen3_chroma : 0.0;
    if (filter_value_enabled(sharpen3_luma) || filter_value_enabled(sharpen3_chroma)) {
        append_filter(std::format("unsharp=3:3:{:.1f}:3:3:{:.1f}", sharpen3_luma, sharpen3_chroma));
    }

    AVFilterInOut* outputs = avfilter_inout_alloc();
    AVFilterInOut* inputs  = avfilter_inout_alloc();
    if (!outputs || !inputs) {
        avfilter_inout_free(&outputs);
        avfilter_inout_free(&inputs);
        return std::unexpected("Cannot alloc filter inouts");
    }

    outputs->name       = av_strdup("in");
    outputs->filter_ctx = src_ctx;
    outputs->pad_idx    = 0;
    outputs->next       = nullptr;

    inputs->name       = av_strdup("out");
    inputs->filter_ctx = sink_ctx;
    inputs->pad_idx    = 0;
    inputs->next       = nullptr;

    ret = avfilter_graph_parse_ptr(graph.get(), filters.c_str(), &inputs, &outputs, nullptr);
    avfilter_inout_free(&outputs);
    avfilter_inout_free(&inputs);
    if (ret < 0)
        return std::unexpected("Cannot parse filter graph: " + averr(ret));

    ret = avfilter_graph_config(graph.get(), nullptr);
    if (ret < 0)
        return std::unexpected("Cannot config filter graph: " + averr(ret));

    return FilterChain{
        .graph    = std::move(graph),
        .src_ctx  = src_ctx,
        .sink_ctx = sink_ctx,
    };
}

std::expected<CodecCtxPtr, std::string> create_encoder_context(
    int width, int height, AVRational time_base, AVRational frame_rate, const EncodeParams& params,
    AVRational sample_aspect_ratio = {1, 1}, const ColorParams& color = {},
    bool repeat_headers = false, bool global_header = false) {

    const auto* enc_codec = avcodec_find_encoder_by_name("libx265");
    if (!enc_codec)
        return std::unexpected("libx265 encoder not found");

    auto enc_ctx = alloc_codec_ctx(enc_codec);
    if (!enc_ctx)
        return std::unexpected("Cannot allocate encoder context");

    const bool realtime_stream = repeat_headers && !global_header;
    const auto bitrate_bps     = std::max<int64_t>(1'000, params.target_bitrate);
    const auto bitrate_kbps =
        std::max(1, static_cast<int>(std::lround(static_cast<double>(bitrate_bps) / 1000.0)));
    enc_ctx->bit_rate               = realtime_stream ? bitrate_bps : 0;
    enc_ctx->rc_max_rate            = realtime_stream ? bitrate_bps : 0;
    enc_ctx->rc_min_rate            = realtime_stream ? bitrate_bps : 0;
    enc_ctx->width                  = width;
    enc_ctx->height                 = height;
    enc_ctx->time_base              = time_base;
    enc_ctx->framerate              = frame_rate;
    enc_ctx->gop_size               = params.gop_size;
    enc_ctx->max_b_frames           = 0;
    enc_ctx->pix_fmt                = AV_PIX_FMT_YUV420P;
    enc_ctx->profile                = AV_PROFILE_HEVC_MAIN;
    enc_ctx->sample_aspect_ratio    = sample_aspect_ratio;
    enc_ctx->colorspace             = color.colorspace;
    enc_ctx->color_range            = color.range;
    enc_ctx->color_primaries        = color.color_primaries;
    enc_ctx->color_trc              = color.color_trc;
    enc_ctx->chroma_sample_location = color.chroma_location;
    if (global_header)
        enc_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    constexpr double kLatencyBudgetSeconds = 0.25;

    const double frame_rate_fps =
        frame_rate.num > 0 && frame_rate.den > 0
            ? static_cast<double>(frame_rate.num) / static_cast<double>(frame_rate.den)
            : static_cast<double>(params.framerate);
    const double gop_seconds =
        frame_rate_fps > 0.0 ? static_cast<double>(params.gop_size) / frame_rate_fps : 0.0;
    const double desired_vbv_window_seconds =
        realtime_stream ? kLatencyBudgetSeconds : std::clamp(gop_seconds, 1.0, 1.5);
    const auto vbv_bufsize = std::max(
        1, static_cast<int>(
               std::lround(static_cast<double>(bitrate_kbps) * desired_vbv_window_seconds)));
    enc_ctx->rc_buffer_size = realtime_stream ? vbv_bufsize * 1000 : 0;

    if (!params.preset.empty())
        av_opt_set(enc_ctx->priv_data, "preset", params.preset.c_str(), 0);
    if (realtime_stream) {
        const char* tune = params.tune.empty() ? "zerolatency" : params.tune.c_str();
        av_opt_set(enc_ctx->priv_data, "tune", tune, 0);
    }

    const auto min_keyint   = std::max(1, params.gop_size / 2);
    const int frame_threads = realtime_stream ? 1 : 0;
    const int ref_count     = realtime_stream ? 1 : 10;
    const int limit_refs    = realtime_stream ? 0 : 3;
    const char* me_mode     = "star";
    const int subme_level   = 7;
    const int rd_level      = 5;
    const int rdoq_level    = 2;
    const int amp_enabled   = 1;
    const int rect_enabled  = 1;
    const int cb_qp_offset  = std::max(0, params.chroma_qp_offset);
    const int cr_qp_offset  = std::max(0, params.chroma_qp_offset);

    const auto& fp           = params.filter;
    const double psy_rd      = (fp.enable && fp.enable_psy_rd) ? fp.psy_rd : 0.0;
    const double psy_trellis = (fp.enable && fp.enable_psy_trellis) ? fp.psy_trellis : 0.0;
    std::string x265_opts;
    if (realtime_stream) {
        const int lookahead = std::clamp(params.lookahead, 0, 4);
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
    av_opt_set(enc_ctx->priv_data, "x265-params", x265_opts.c_str(), 0);

    if (avcodec_open2(enc_ctx.get(), enc_codec, nullptr) < 0)
        return std::unexpected("Cannot open encoder");

    return enc_ctx;
}

} // namespace

// ===========================================================================
// FfmpegBackend — FFmpeg/libx265 software encoder backend
// ===========================================================================

class FfmpegBackend final : public EncoderBackend {
public:
    static constexpr size_t kInputFramePoolSize = 3;
    static constexpr int kBufferSrcFlags        = AV_BUFFERSRC_FLAG_KEEP_REF;

    [[nodiscard]] static std::expected<std::unique_ptr<FfmpegBackend>, std::string>
        create(EncodeParams params, int src_width, int src_height, int framerate) noexcept try {
        auto backend = std::unique_ptr<FfmpegBackend>(new FfmpegBackend(std::move(params)));
        auto result  = backend->setup(src_width, src_height, framerate);
        if (!result)
            return std::unexpected(std::move(result.error()));
        return backend;
    } catch (const std::exception& e) {
        return std::unexpected(std::format("FfmpegBackend::create: {}", e.what()));
    }

    ~FfmpegBackend() override = default;

    FfmpegBackend(const FfmpegBackend&)            = delete;
    FfmpegBackend& operator=(const FfmpegBackend&) = delete;
    FfmpegBackend(FfmpegBackend&&)                 = default;
    FfmpegBackend& operator=(FfmpegBackend&&)      = default;

    std::expected<void, std::string>
        push_frame(const uint8_t* data, int linesize, int64_t pts) noexcept override try {
        AVFrame* bgr_frame = bgr_frames_[next_bgr_frame_].get();
        next_bgr_frame_    = (next_bgr_frame_ + 1) % bgr_frames_.size();

        int ret = av_frame_make_writable(bgr_frame);
        if (ret < 0)
            return std::unexpected("Cannot make BGR frame writable: " + averr(ret));

        bgr_frame->pts = pts;

        // Copy BGR data row-by-row
        if (linesize == input_row_bytes_ && bgr_frame->linesize[0] == input_row_bytes_) {
            std::memcpy(
                bgr_frame->data[0], data, static_cast<size_t>(input_row_bytes_) * src_height_);
        } else {
            for (int y = 0; y < src_height_; ++y) {
                std::memcpy(
                    bgr_frame->data[0] + y * bgr_frame->linesize[0], data + y * linesize,
                    input_row_bytes_);
            }
        }

        // Push through filter graph
        ret = av_buffersrc_add_frame_flags(chain_.src_ctx, bgr_frame, kBufferSrcFlags);
        if (ret < 0)
            return std::unexpected("Filter push failed: " + averr(ret));

        // Pull filtered frames and encode
        while (av_buffersink_get_frame(chain_.sink_ctx, filt_frame_.get()) >= 0) {
            const bool force_keyframe = std::exchange(force_keyframe_requested_, false);
            auto r                    = encode_filtered_frame(filt_frame_.get(), force_keyframe);
            if (!r)
                return r;
            av_frame_unref(filt_frame_.get());
        }

        return {};
    } catch (const std::exception& e) {
        return std::unexpected(std::format("FfmpegBackend::push_frame: {}", e.what()));
    }

    void request_keyframe() noexcept override { force_keyframe_requested_ = true; }

    std::optional<EncodedPacket> poll_packet() noexcept override {
        if (packet_queue_.empty())
            return std::nullopt;
        auto pkt = std::move(packet_queue_.front());
        packet_queue_.pop_front();
        return pkt;
    }

    std::expected<void, std::string> flush() noexcept override try {
        // Drain encoder: send NULL frame
        int ret = avcodec_send_frame(enc_ctx_.get(), nullptr);
        if (ret < 0)
            return std::unexpected("Flush send failed: " + averr(ret));

        while (true) {
            ret = avcodec_receive_packet(enc_ctx_.get(), enc_pkt_.get());
            if (ret == AVERROR_EOF || ret == AVERROR(EAGAIN))
                break;
            if (ret < 0)
                return std::unexpected("Flush recv failed: " + averr(ret));

            enqueue_packet(enc_pkt_.get());
            av_packet_unref(enc_pkt_.get());
        }

        return {};
    } catch (const std::exception& e) {
        return std::unexpected(std::format("FfmpegBackend::flush: {}", e.what()));
    }

    const EncodeParams& params() const noexcept override { return params_; }
    std::pair<int, int> dimensions() const noexcept override { return {out_width_, out_height_}; }

private:
    explicit FfmpegBackend(EncodeParams params)
        : params_(std::move(params)) {}

    std::expected<void, std::string> setup(int src_width, int src_height, int framerate) {
        src_width_  = src_width;
        src_height_ = src_height;
        framerate_  = framerate;

        // Compute output dimensions (fit within max bounds, snap to even)
        int out_w{}, out_h{};
        fit_dimensions(src_width, src_height, params_.max_width, params_.max_height, out_w, out_h);
        out_width_  = out_w;
        out_height_ = out_h;

        const AVRational time_base  = {1, framerate};
        const AVRational frame_rate = {framerate, 1};

        // Build filter chain: BGR24 input -> format=yuv420p -> filters -> scale -> output
        auto filter_result = build_filter_chain(
            src_width, src_height, {1, 1}, time_base, frame_rate, {0, 1}, AV_PIX_FMT_BGR24, out_w,
            out_h, params_.filter);
        if (!filter_result)
            return std::unexpected("Cannot build filter chain: " + filter_result.error());
        chain_ = std::move(filter_result.value());

        // Create encoder
        auto enc_result =
            create_encoder_context(out_w, out_h, time_base, frame_rate, params_, {1, 1}, {}, true);
        if (!enc_result)
            return std::unexpected(enc_result.error());
        enc_ctx_ = std::move(enc_result.value());

        filt_frame_ = alloc_frame();
        enc_pkt_    = alloc_packet();
        if (!filt_frame_ || !enc_pkt_)
            return std::unexpected("Cannot allocate reusable frame/packet");

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

        input_row_bytes_ = src_width_ * 3;

        std::fprintf(
            stderr, "[quanta::stream][ffmpeg] %dx%d @ %dfps -> %dx%d, target %d bps\n", src_width,
            src_height, framerate, out_w, out_h, params_.target_bitrate);
        return {};
    }

    std::expected<void, std::string> encode_filtered_frame(AVFrame* frame, bool force_keyframe) {
        if (force_keyframe) {
            frame->flags |= AV_FRAME_FLAG_KEY;
            frame->pict_type = AV_PICTURE_TYPE_I;
        } else {
            frame->flags &= ~AV_FRAME_FLAG_KEY;
            frame->pict_type = AV_PICTURE_TYPE_NONE;
        }
        int ret = avcodec_send_frame(enc_ctx_.get(), frame);
        if (ret < 0)
            return std::unexpected("Encode send: " + averr(ret));

        while (ret >= 0) {
            ret = avcodec_receive_packet(enc_ctx_.get(), enc_pkt_.get());
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                break;
            if (ret < 0)
                return std::unexpected("Encode recv: " + averr(ret));

            enqueue_packet(enc_pkt_.get());
            av_packet_unref(enc_pkt_.get());
        }
        return {};
    }

    void enqueue_packet(const AVPacket* pkt) {
        EncodedPacket encoded;
        if (pkt->size > 0) {
            encoded.data = std::unique_ptr<uint8_t[]>(new uint8_t[pkt->size]);
            std::copy_n(pkt->data, pkt->size, encoded.data.get());
        }
        encoded.size     = static_cast<size_t>(pkt->size);
        encoded.pts      = pkt->pts;
        encoded.keyframe = (pkt->flags & AV_PKT_FLAG_KEY) != 0;

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

        if (encoded.size > 0) {
            packet_queue_.push_back(std::move(encoded));
        }
    }

    EncodeParams params_;

    int src_width_  = 0;
    int src_height_ = 0;
    int out_width_  = 0;
    int out_height_ = 0;
    int framerate_  = 0;

    FilterChain chain_;
    CodecCtxPtr enc_ctx_;
    std::array<FramePtr, kInputFramePoolSize> bgr_frames_;
    FramePtr filt_frame_;
    PacketPtr enc_pkt_;
    std::deque<EncodedPacket> packet_queue_;
    HevcParameterSetExportState parameter_set_state_;
    int input_row_bytes_           = 0;
    size_t next_bgr_frame_         = 0;
    bool force_keyframe_requested_ = false;
};

// ===========================================================================
// Factory
// ===========================================================================

std::expected<std::unique_ptr<EncoderBackend>, std::string> create_ffmpeg_backend(
    EncodeParams params, int src_width, int src_height, int framerate) noexcept {
    auto backend = FfmpegBackend::create(std::move(params), src_width, src_height, framerate);
    if (!backend)
        return std::unexpected(std::move(backend.error()));
    return std::unique_ptr<EncoderBackend>(std::move(*backend));
}

} // namespace quanta
