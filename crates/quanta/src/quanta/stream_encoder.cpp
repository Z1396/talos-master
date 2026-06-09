#include "quanta/stream_encoder.hpp"
#include "quanta/encode_backend/encoder_backend.hpp"

namespace quanta {

StreamEncoder::StreamEncoder(std::unique_ptr<EncoderBackend> backend)
    : backend_(std::move(backend)) {}

StreamEncoder::~StreamEncoder() = default;

StreamEncoder::StreamEncoder(StreamEncoder&&) noexcept            = default;
StreamEncoder& StreamEncoder::operator=(StreamEncoder&&) noexcept = default;

std::expected<StreamEncoder, std::string> StreamEncoder::create(
    EncodeParams params, int src_width, int src_height, int framerate) noexcept {
    auto backend = create_optimal_backend(std::move(params), src_width, src_height, framerate);
    if (!backend)
        return std::unexpected(std::move(backend.error()));
    return StreamEncoder(std::move(*backend));
}

std::expected<void, std::string>
    StreamEncoder::push_frame(const uint8_t* data, int linesize, int64_t pts) noexcept {
    return backend_->push_frame(data, linesize, pts);
}

void StreamEncoder::request_keyframe() noexcept { backend_->request_keyframe(); }

std::optional<EncodedPacket> StreamEncoder::poll_packet() noexcept {
    return backend_->poll_packet();
}

std::expected<void, std::string> StreamEncoder::flush() noexcept { return backend_->flush(); }

[[nodiscard]] const EncodeParams& StreamEncoder::params() const { return backend_->params(); }

[[nodiscard]] std::pair<int, int> StreamEncoder::dimensions() const {
    return backend_->dimensions();
}

} // namespace quanta
