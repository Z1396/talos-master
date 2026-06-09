#include "quanta/stream_transport.hpp"
#include "quanta/stream_encoder.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <utility>

namespace quanta {

QuantaPacketQueue::QuantaPacketQueue(size_t max_packets)
    : frame_buffer_(make_frame_snapshot_endpoints())
    , max_packets_(max_packets == 0 ? kDefaultMaxPackets : max_packets) {
    stats_.max_packets      = max_packets_;
    stats_.max_frame_queues = kFrameBufferDepth;
}

bool QuantaPacketQueue::push_frame(QuantaPacketBatch batch) {
    if (batch.packets.empty())
        return false;

    if (batch.packets.size() > max_packets_) {
        std::lock_guard lock(stats_mutex_);
        stats_.frames_dropped++;
        stats_.oversized_frame_drops++;
        return false;
    }

    const auto packet_count = batch.packets.size();
    PacketFrameQueue frame{
        .frame_id = next_frame_id_++,
        .packets  = {},
        .keyframe = batch.keyframe,
    };
    for (auto& packet : batch.packets) {
        frame.packets.push_back(std::move(packet));
    }

    producer_history_.push_back(std::move(frame));
    std::size_t dropped_frames = 0;
    const auto last_ingested   = last_ingested_frame_id_.load(std::memory_order_acquire);
    while (producer_history_.size() > kFrameBufferDepth) {
        if (producer_history_.front().frame_id > last_ingested) {
            ++dropped_frames;
        }
        producer_history_.pop_front();
    }

    {
        std::lock_guard lock(stats_mutex_);
        stats_.frames_enqueued++;
        stats_.packets_enqueued += packet_count;
        stats_.frames_dropped += dropped_frames;
        stats_.frame_buffer_drops += dropped_frames;
    }
    update_stats_from_producer();
    publish_producer_history();
    return true;
}

std::optional<DequeuedQuantaPacket> QuantaPacketQueue::pop_packet() {
    ingest_latest_snapshot();

    if (current_packets_.empty() && !consumer_frames_.empty()) {
        auto frame = std::move(consumer_frames_.front());
        consumer_frames_.pop_front();
        current_keyframe_ = frame.keyframe;
        current_packets_  = std::move(frame.packets);
    }

    if (current_packets_.empty()) {
        current_keyframe_ = false;
        update_stats_from_consumer();
        return std::nullopt;
    }

    auto packet = std::move(current_packets_.front());
    current_packets_.pop_front();

    {
        std::lock_guard lock(stats_mutex_);
        stats_.packets_dequeued++;
    }
    if (current_packets_.empty()) {
        current_keyframe_ = false;
    }
    update_stats_from_consumer();
    return DequeuedQuantaPacket{
        .packet            = std::move(packet),
        .remaining_packets = consumer_packet_count(),
        .remaining_frames  = consumer_frame_count(),
    };
}

std::vector<QuantaPacket> QuantaPacketQueue::take_all() {
    std::vector<QuantaPacket> out;
    while (auto packet = pop_packet()) {
        out.push_back(std::move(packet->packet));
    }
    return out;
}

QuantaPacketQueueStats QuantaPacketQueue::stats() const {
    std::lock_guard lock(stats_mutex_);
    auto copy = stats_;
    return copy;
}

auto QuantaPacketQueue::make_frame_snapshot_endpoints() -> FrameSnapshotEndpoints {
    auto [writer, reader] = FrameSnapshotBuffer::create();
    return FrameSnapshotEndpoints{
        .writer = std::move(writer),
        .reader = std::move(reader),
    };
}

void QuantaPacketQueue::publish_producer_history() {
    PacketFrameSnapshot snapshot;
    snapshot.frames.reserve(producer_history_.size());
    for (const auto& frame : producer_history_) {
        snapshot.frames.push_back(frame);
    }
    frame_buffer_.writer.write(std::move(snapshot));
}

void QuantaPacketQueue::ingest_latest_snapshot() {
    auto snapshot = frame_buffer_.reader.read();
    if (!snapshot) {
        return;
    }

    auto last_ingested = last_ingested_frame_id_.load(std::memory_order_acquire);
    for (auto& frame : snapshot->frames) {
        if (frame.frame_id <= last_ingested) {
            continue;
        }
        last_ingested = frame.frame_id;
        consumer_frames_.push_back(std::move(frame));
    }
    last_ingested_frame_id_.store(last_ingested, std::memory_order_release);

    trim_consumer_frames();
    update_stats_from_consumer();
}

void QuantaPacketQueue::trim_consumer_frames() {
    std::size_t dropped_frames    = 0;
    const auto active_frame_count = current_packets_.empty() ? 0U : 1U;
    while (active_frame_count + consumer_frames_.size() > kFrameBufferDepth) {
        consumer_frames_.pop_front();
        ++dropped_frames;
    }

    if (dropped_frames == 0) {
        return;
    }

    std::lock_guard lock(stats_mutex_);
    stats_.frames_dropped += dropped_frames;
    stats_.frame_buffer_drops += dropped_frames;
}

size_t QuantaPacketQueue::consumer_packet_count() const {
    size_t count = current_packets_.size();
    for (const auto& frame : consumer_frames_) {
        count += frame.packets.size();
    }
    return count;
}

size_t QuantaPacketQueue::consumer_frame_count() const {
    return consumer_frames_.size() + (current_packets_.empty() ? 0U : 1U);
}

void QuantaPacketQueue::update_stats_from_producer() {
    size_t packet_count = 0;
    for (const auto& frame : producer_history_) {
        packet_count += frame.packets.size();
    }

    std::lock_guard lock(stats_mutex_);
    stats_.queued_packets   = packet_count;
    stats_.queued_frames    = producer_history_.size();
    stats_.current_keyframe = !producer_history_.empty() && producer_history_.back().keyframe;
}

void QuantaPacketQueue::update_stats_from_consumer() {
    std::lock_guard lock(stats_mutex_);
    stats_.queued_packets   = consumer_packet_count();
    stats_.queued_frames    = consumer_frame_count();
    stats_.current_keyframe = current_keyframe_;
}

std::expected<QuantaPacketBatch, std::string>
    build_quanta_packets(const EncodedPacket& packet, uint16_t seq) {
    if (packet.size == 0 || packet.data == nullptr)
        return std::unexpected("encoded packet is empty");

    const size_t fragment_count = (packet.size + static_cast<size_t>(QuantaPacket::MAX_PAYLOAD) - 1)
                                / static_cast<size_t>(QuantaPacket::MAX_PAYLOAD);
    if (fragment_count == 0 || fragment_count > std::numeric_limits<uint8_t>::max()) {
        return std::unexpected("encoded packet requires too many QuantaPacket fragments");
    }

    QuantaPacketBatch batch;
    batch.keyframe = packet.keyframe;
    batch.packets.reserve(fragment_count);

    size_t offset = 0;
    for (size_t i = 0; i < fragment_count; ++i) {
        const size_t payload_size =
            std::min(static_cast<size_t>(QuantaPacket::MAX_PAYLOAD), packet.size - offset);

        QuantaPacket out;
        out.seq            = seq;
        out.fragment_index = static_cast<uint8_t>(i);
        out.fragment_count = static_cast<uint8_t>(fragment_count);
        out.keyframe       = packet.keyframe;
        out.data.resize(static_cast<size_t>(QuantaPacket::HEADER_SIZE) + payload_size);

        out.data[0] = static_cast<uint8_t>(seq & 0xffu);
        out.data[1] = static_cast<uint8_t>((seq >> 8) & 0xffu);
        out.data[2] = out.fragment_index;
        out.data[3] = out.fragment_count;
        out.data[4] = out.keyframe ? 1U : 0U;
        std::memcpy(
            out.data.data() + QuantaPacket::HEADER_SIZE, packet.data.get() + offset, payload_size);

        offset += payload_size;
        batch.packets.push_back(std::move(out));
    }

    return batch;
}

} // namespace quanta
