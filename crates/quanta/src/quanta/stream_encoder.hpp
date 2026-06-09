#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

// Forward declarations - keep FFmpeg out of the public header.
extern "C" {
struct AVFormatContext;
struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;
struct AVStream;
}

namespace quanta {

// ---------------------------------------------------------------------------
// Post-scale pre-processing profile:
// suppress texture detail, keep controllable chroma, recover main contours.
// ---------------------------------------------------------------------------
struct GaussianDenoiseParams {
    int kernel_size = 5;
    double sigma_x  = 1.0;
    double sigma_y  = 1.0;
};

struct FilterParams {
    bool enable = true;
    // FFmpeg fallback only. The AX hardware path ignores saturation.
    bool enable_saturation = true;
    double saturation      = 1.0;
    // Spatial Gaussian luma denoise on the scaled frame.
    bool enable_denoise_luma = true;
    GaussianDenoiseParams denoise_luma{
        .kernel_size = 3,
        .sigma_x     = 1.5,
        .sigma_y     = 1.5,
    };
    // Spatial Gaussian chroma denoise on the scaled NV12 UV plane. AX keeps
    // U/V channels separate by using same-channel byte offsets.
    bool enable_denoise_chroma = true;
    GaussianDenoiseParams denoise_chroma{
        .kernel_size = 3,
        .sigma_x     = 1.5,
        .sigma_y     = 1.5,
    };
    // Keep temporal NR disabled by default to avoid target ghosting and added latency.
    // FFmpeg fallback 的时间降噪参数，分别偏向 luma temporal 和 chroma
    // temporal。原理是参考前后帧/历史帧抑制随机噪声。移动目标上容易出现拖影、残影
    bool enable_denoise_tl = true;
    double denoise_tl      = 0.0;
    bool enable_denoise_tc = true;
    double denoise_tc      = 0.0;
    // Light SHP-style luma edge recovery after NR.
    // 亮度锐化。AX 路径会把三个非负值相加，然后做一次 high-boost 5x5：
    // out = src + amount * (src - blur5x5(src))
    bool enable_sharpen1_luma = true;
    double sharpen1_luma      = 2.0;
    bool enable_sharpen2_luma = true;
    double sharpen2_luma      = 0.0;
    bool enable_sharpen3_luma = true;
    double sharpen3_luma      = 0.0;
    // 亮度锐化。AX 路径会把三个非负值相加，然后做一次 high-boost 5x5
    bool enable_sharpen1_chroma = true;
    double sharpen1_chroma      = 2.0;
    bool enable_sharpen2_chroma = true;
    double sharpen2_chroma      = 0.0;
    bool enable_sharpen3_chroma = true;
    double sharpen3_chroma      = 0.0;
    // Quantize luma into flat blocks; chroma_levels=0 keeps full 8-bit UV.
    bool enable_luma_quantization   = false;
    int luma_levels                 = 32;
    bool enable_chroma_quantization = false;
    int chroma_levels               = 0;
    bool enable_contour             = false;
    double contour_strength         = 1.0;
    int contour_low_thresh          = 36;
    int contour_high_thresh         = 96;
    int contour_width               = 3;
    // AX VENC QP delta map: experimental; keep normal CBR transport behavior by default.
    bool qp_delta_map       = false;
    int qp_edge_delta       = 0;
    int qp_interior_delta   = 16;
    bool enable_psy_rd      = true; // FFmpeg fallback only.
    double psy_rd           = 0.0;  // FFmpeg fallback only.
    bool enable_psy_trellis = true; // FFmpeg fallback only.
    double psy_trellis      = 0.0;  // FFmpeg fallback only.
};

// ---------------------------------------------------------------------------
// Encode parameters for ultra-low bitrate transmission
// ---------------------------------------------------------------------------
struct EncodeParams {
    int max_width      = 200;    // auto-downscale if wider
    int max_height     = 200;    // auto-downscale if taller
    int gop_size       = 15;     // short recovery period; avoid long P-frame corruption chains
    int framerate      = 30;     // target output framerate
    std::string preset = "fast"; // keep the original rate allocation baseline
    std::string tune   = "ssim";
    bool intra_refresh = false;  // periodic IDR lets a lossy stream recover without feedback

    // CRF + VBV rate control (original working approach)
    int crf              = 52;     // quality target; VBV caps actual bitrate
    int lookahead        = 0;      // realtime transport must emit immediately
    int chroma_qp_offset = 0;      // base chroma penalty; encoder internally protects Cr over Cb
    int target_bitrate   = 45'000; // bps
    bool enVBR           = false;  // AX only: true selects H265VBR, false selects H265CBR
    int min_bit_rate     = 0;      // bps; reserved, AX H265VBR has no min-bitrate field
    int max_bit_rate     = 50'000; // bps
    int vbv_bufsize      = 10'000; // bits
    int refresh_num      = 4;
    // For libx265
    int bframe         = 0;
    bool enScenecut    = true;
    bool enAqMode      = true;
    double aq_strength = 0.4; // FFmpeg fallback; AX uses explicit QP map instead.

    FilterParams filter{};
};

// ---------------------------------------------------------------------------
// Encoded HEVC access unit (Annex B format)
// ---------------------------------------------------------------------------
struct EncodedPacketNalu {
    size_t packet_offset = 0;
    size_t packet_size   = 0;
    uint8_t type         = 0;
};

struct EncodedPacket {
    std::unique_ptr<uint8_t[]> data; // Owned byte buffer
    size_t size   = 0;
    int64_t pts   = 0;
    bool keyframe = false;
    std::vector<EncodedPacketNalu> nalus;

    EncodedPacket()                                    = default;
    EncodedPacket(EncodedPacket&&) noexcept            = default;
    EncodedPacket& operator=(EncodedPacket&&) noexcept = default;
    EncodedPacket(const EncodedPacket&)                = delete;
    EncodedPacket& operator=(const EncodedPacket&)     = delete;

    /// Access encoded bytes for transmission.
    [[nodiscard]] std::span<const uint8_t> bytes() const noexcept { return {data.get(), size}; }
};

class EncoderBackend;

// ---------------------------------------------------------------------------
// StreamEncoder: realtime HEVC encoding with BGR24 frame input
// ---------------------------------------------------------------------------
class StreamEncoder {
public:
    /// Factory: construct a fully initialized encoder.
    /// Returns the encoder on success, or an error string on failure.
    [[nodiscard]] static std::expected<StreamEncoder, std::string>
        create(EncodeParams params, int src_width, int src_height, int framerate) noexcept;

    ~StreamEncoder();

    StreamEncoder(StreamEncoder&&) noexcept;
    StreamEncoder& operator=(StreamEncoder&&) noexcept;
    StreamEncoder(const StreamEncoder&)            = delete;
    StreamEncoder& operator=(const StreamEncoder&) = delete;

    std::expected<void, std::string>
        push_frame(const uint8_t* data, int linesize, int64_t pts) noexcept;

    void request_keyframe() noexcept;

    std::optional<EncodedPacket> poll_packet() noexcept;

    std::expected<void, std::string> flush() noexcept;

    /// Get encoding parameters.
    [[nodiscard]] const EncodeParams& params() const;

    /// Get output dimensions.
    [[nodiscard]] std::pair<int, int> dimensions() const;

private:
    explicit StreamEncoder(std::unique_ptr<EncoderBackend> backend);
    std::unique_ptr<EncoderBackend> backend_;
};

} // namespace quanta
