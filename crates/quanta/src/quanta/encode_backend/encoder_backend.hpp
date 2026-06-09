#pragma once

#include "quanta/stream_encoder.hpp"

#include <expected>
#include <memory>
#include <string>
#include <utility>

namespace quanta {

// ---------------------------------------------------------------------------
// Abstract encoder backend interface
// ---------------------------------------------------------------------------
// Implementations: FFmpeg (libx265), AX VENC (hardware).
// The StreamEncoder PIMPL delegates all encoding work to a backend instance.
// ---------------------------------------------------------------------------
class EncoderBackend {
public:
    virtual ~EncoderBackend() = default;

    virtual std::expected<void, std::string>
        push_frame(const uint8_t* data, int linesize, int64_t pts) noexcept = 0;

    virtual void request_keyframe() noexcept = 0;

    virtual std::optional<EncodedPacket> poll_packet() noexcept = 0;

    virtual std::expected<void, std::string> flush() noexcept = 0;

    virtual const EncodeParams& params() const noexcept = 0;

    virtual std::pair<int, int> dimensions() const noexcept = 0;
};

// ---------------------------------------------------------------------------
// Backend factories
// ---------------------------------------------------------------------------
// Each backend is compiled in its own translation unit.
// The default backend is selected at compile time based on platform capability.

/// Create FFmpeg/libx265 software encoder backend (always available).
[[nodiscard]] std::expected<std::unique_ptr<EncoderBackend>, std::string> create_ffmpeg_backend(
    EncodeParams params, int src_width, int src_height, int framerate) noexcept;

#if TALOS_HAS_AXERA
/// Create AX VENC hardware encoder backend (only when Axera SDK is available).
[[nodiscard]] std::expected<std::unique_ptr<EncoderBackend>, std::string>
    create_ax_backend(EncodeParams params, int src_width, int src_height, int framerate) noexcept;
#endif

/// Create the optimal backend for the current platform.
/// Prefers AX VENC when available, falls back to FFmpeg otherwise.
[[nodiscard]] inline std::expected<std::unique_ptr<EncoderBackend>, std::string>
    create_optimal_backend(
        EncodeParams params, int src_width, int src_height, int framerate) noexcept {
#if TALOS_HAS_AXERA
    return create_ax_backend(params, src_width, src_height, framerate);
#endif
    return create_ffmpeg_backend(std::move(params), src_width, src_height, framerate);
}

} // namespace quanta
