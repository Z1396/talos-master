#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <expected>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "primitive/spsc_triple_buffer.hpp"

namespace quanta {

#pragma pack(push, 1)
struct QuantaPacketHeader {
    uint16_t seq;
    uint8_t frag_idx;
    uint8_t frag_total;
    uint8_t keyframe;
};
#pragma pack(pop)

struct EncodedPacket;

struct QuantaPacket {
    static constexpr int HEADER_SIZE = 5;
    static constexpr int MTU         = 298;
    static constexpr int MAX_PAYLOAD = MTU - HEADER_SIZE;

    std::vector<uint8_t> data;
    uint16_t seq           = 0;
    uint8_t fragment_index = 0;
    uint8_t fragment_count = 0;
    bool keyframe          = false;

    [[nodiscard]] std::span<const uint8_t> bytes() const noexcept { return data; }
};

static_assert(sizeof(QuantaPacketHeader) == QuantaPacket::HEADER_SIZE);

struct QuantaPacketBatch {
    std::vector<QuantaPacket> packets;
    bool keyframe = false;
};

struct QuantaPacketQueueStats {
    uint64_t frames_enqueued       = 0;
    uint64_t frames_dropped        = 0;
    uint64_t idr_protected_drops   = 0;
    uint64_t oversized_frame_drops = 0;
    uint64_t frame_buffer_drops    = 0;
    uint64_t packets_enqueued      = 0;
    uint64_t packets_dequeued      = 0;
    size_t queued_packets          = 0;
    size_t queued_frames           = 0;
    size_t max_packets             = 0;
    size_t max_frame_queues        = 0;
    bool current_keyframe          = false;
};

struct DequeuedQuantaPacket {
    QuantaPacket packet;
    size_t remaining_packets = 0;
    size_t remaining_frames  = 0;
};

class QuantaPacketQueue {
    struct PacketFrameQueue {
        uint64_t frame_id = 0;
        std::deque<QuantaPacket> packets;
        bool keyframe = false;
    };

    struct PacketFrameSnapshot {
        std::vector<PacketFrameQueue> frames;
    };

    using FrameSnapshotBuffer = talos::primitive::SpscTripleBuffer<PacketFrameSnapshot>;
    using FrameSnapshotWriter = FrameSnapshotBuffer::template Write<PacketFrameSnapshot>;
    using FrameSnapshotReader = FrameSnapshotBuffer::template Read<PacketFrameSnapshot>;

    struct FrameSnapshotEndpoints {
        FrameSnapshotWriter writer;
        FrameSnapshotReader reader;
    };

public:
    static constexpr size_t kDefaultMaxPackets = 15;
    static constexpr size_t kFrameBufferDepth  = 30;

    explicit QuantaPacketQueue(size_t max_packets = kDefaultMaxPackets);

    QuantaPacketQueue(const QuantaPacketQueue&)            = delete;
    QuantaPacketQueue& operator=(const QuantaPacketQueue&) = delete;

    bool push_frame(QuantaPacketBatch batch);
    std::optional<DequeuedQuantaPacket> pop_packet();
    std::vector<QuantaPacket> take_all();

    [[nodiscard]] QuantaPacketQueueStats stats() const;

private:
    [[nodiscard]] static FrameSnapshotEndpoints make_frame_snapshot_endpoints();

    void publish_producer_history();
    void ingest_latest_snapshot();
    void trim_consumer_frames();
    [[nodiscard]] size_t consumer_packet_count() const;
    [[nodiscard]] size_t consumer_frame_count() const;
    void update_stats_from_producer();
    void update_stats_from_consumer();

    mutable std::mutex stats_mutex_;
    QuantaPacketQueueStats stats_{};
    FrameSnapshotEndpoints frame_buffer_;
    std::deque<PacketFrameQueue> producer_history_;
    std::deque<PacketFrameQueue> consumer_frames_;
    std::deque<QuantaPacket> current_packets_;
    size_t max_packets_     = kDefaultMaxPackets;
    bool current_keyframe_  = false;
    uint64_t next_frame_id_ = 1;
    std::atomic<uint64_t> last_ingested_frame_id_{0};
};

[[nodiscard]] std::expected<QuantaPacketBatch, std::string>
    build_quanta_packets(const EncodedPacket& packet, uint16_t seq);

} // namespace quanta
