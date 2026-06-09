#include "runtime/stream_encode.hpp"

#include "core/channel_topics.hpp"
#include "core/runtime.hpp"
#include "core/types.hpp"
#include "quanta/stream_encoder.hpp"
#include "quanta/stream_transport.hpp"
#include "runtime/stream_send.hpp"

#include <atomic>
#include <cstdint>
#include <fmt/core.h>
#include <limits>
#include <memory>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>

namespace fcs::runtime {
namespace {

struct QuantaEncodeState {
    quanta::EncodeParams params;
    std::optional<quanta::StreamEncoder> encoder;
    std::atomic<uint16_t> next_seq{0};
    int64_t next_pts = 0;
    int src_width    = 0;
    int src_height   = 0;
    int framerate    = 0;
    std::string last_error{};
};

void log_quanta_error_once(QuantaEncodeState& state, std::string message) noexcept {
    if (state.last_error == message)
        return;
    state.last_error = std::move(message);
    SPDLOG_WARN("stream_encode: {}", state.last_error);
}

[[nodiscard]] uint16_t take_next_seq(std::atomic<uint16_t>& next_seq) noexcept {
    auto current = next_seq.load(std::memory_order_relaxed);
    for (;;) {
        const auto next = current == std::numeric_limits<uint16_t>::max()
                            ? uint16_t{0}
                            : static_cast<uint16_t>(current + 1);
        if (next_seq.compare_exchange_weak(
                current, next, std::memory_order_relaxed, std::memory_order_relaxed)) {
            return current;
        }
    }
}

[[nodiscard]] bool
    ensure_quanta_encoder(QuantaEncodeState& state, int src_width, int src_height) noexcept {
    if (src_width <= 0 || src_height <= 0) {
        log_quanta_error_once(
            state, fmt::format("invalid source image dimensions: {}x{}", src_width, src_height));
        return false;
    }

    if (state.encoder && state.src_width == src_width && state.src_height == src_height
        && state.framerate == state.params.framerate) {
        return true;
    }

    auto encoder =
        quanta::StreamEncoder::create(state.params, src_width, src_height, state.params.framerate);
    if (!encoder) {
        log_quanta_error_once(
            state, fmt::format("quanta stream encoder create: {}", encoder.error()));
        state.encoder.reset();
        return false;
    }

    const auto [out_width, out_height] = encoder->dimensions();
    state.encoder.emplace(std::move(*encoder));
    state.next_pts   = 0;
    state.src_width  = src_width;
    state.src_height = src_height;
    state.framerate  = state.params.framerate;
    state.last_error.clear();

    SPDLOG_INFO(
        "quanta stream encoder initialized: {}x{} -> {}x{} (limit {}x{}, {} bps @ {} fps)",
        src_width, src_height, out_width, out_height, state.params.max_width,
        state.params.max_height, state.params.target_bitrate, state.params.framerate);
    return true;
}

} // namespace

std::expected<void, std::string> register_quanta_stream_systems(
    talos::World& world, talos::Scheduler& scheduler, const quanta::EncodeParams& encode_params,
    int src_width, int src_height) {
    auto encoder = quanta::StreamEncoder::create(
        encode_params, src_width, src_height, encode_params.framerate);
    if (!encoder) {
        return std::unexpected(fmt::format("quanta stream encoder create: {}", encoder.error()));
    }

    auto state    = std::make_shared<QuantaEncodeState>();
    state->params = encode_params;
    state->encoder.emplace(std::move(*encoder));
    state->src_width  = src_width;
    state->src_height = src_height;
    state->framerate  = encode_params.framerate;

    if (!world.has_resource<quanta::QuantaPacketQueue>()) {
        auto& queue = world.emplace_resource<quanta::QuantaPacketQueue>(
            quanta::QuantaPacketQueue::kDefaultMaxPackets);
        static_cast<void>(queue);
    }

    scheduler.add_system<talos::fixed_rate<30, 3>>(
        "stream_encode",
        [state](
            talos::spmc<ImageFrame, ImageChannelTopic> image_in,
            talos::res_mut<quanta::QuantaPacketQueue> packet_queue, core::capabilities cap) {
            if (!core::capable(*cap, core::Capability::Quanta)) {
                return;
            }

            auto frame = image_in.read();
            if (!frame) {
                return;
            }
            if (!ensure_quanta_encoder(*state, frame->image.cols, frame->image.rows)) {
                return;
            }

            const int64_t pts = state->next_pts++;
            auto push_result  = state->encoder->push_frame(
                frame->image.data, static_cast<int>(frame->image.step[0]), pts);
            if (!push_result) {
                SPDLOG_WARN("stream_encode push frame: {}", push_result.error());
                return;
            }

            while (auto packet = state->encoder->poll_packet()) {
                const auto seq = take_next_seq(state->next_seq);
                auto batch     = quanta::build_quanta_packets(*packet, seq);
                if (!batch) {
                    SPDLOG_WARN("stream_encode packetize: {}", batch.error());
                    continue;
                }

                const auto bytes     = packet->size;
                const auto fragments = batch->packets.size();
                if (!packet_queue->push_frame(std::move(*batch))) {
                    SPDLOG_WARN(
                        "stream_encode dropped encoded frame: seq = {}, bytes = {}, fragments = "
                        "{}, max_fragments = {}",
                        seq, bytes, fragments, quanta::QuantaPacketQueue::kDefaultMaxPackets);
                    continue;
                }

                SPDLOG_DEBUG(
                    "Quanta frame queued: seq = {}, fragments = {}, bytes = {}", seq, fragments,
                    bytes);
            }
        });

    register_quanta_stream_send_system(scheduler);
    const auto [out_width, out_height] = state->encoder->dimensions();
    SPDLOG_INFO(
        "quanta stream systems initialized: {}x{} -> {}x{} @ {}fps, packet_queue_max={}, "
        "frame_buffer={}",
        src_width, src_height, out_width, out_height, encode_params.framerate,
        quanta::QuantaPacketQueue::kDefaultMaxPackets,
        quanta::QuantaPacketQueue::kFrameBufferDepth);
    return {};
}

} // namespace fcs::runtime
