#include "quanta/stream_encoder.hpp"
#include "quanta/stream_transport.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>

namespace {

quanta::EncodedPacket make_encoded_packet(size_t size, bool keyframe) {
    quanta::EncodedPacket packet;
    packet.data     = std::make_unique<uint8_t[]>(size);
    packet.size     = size;
    packet.keyframe = keyframe;
    for (size_t i = 0; i < size; ++i) {
        packet.data[i] = static_cast<uint8_t>(i & 0xffu);
    }
    return packet;
}

} // namespace

TEST(QuantaPacket, BuildsQuantaPacketHeaderAndFragmentsAnnexBPayload) {
    auto encoded =
        make_encoded_packet(static_cast<size_t>(quanta::QuantaPacket::MAX_PAYLOAD) + 7, true);

    auto batch = quanta::build_quanta_packets(encoded, 0x1234);
    ASSERT_TRUE(batch.has_value()) << batch.error();
    ASSERT_TRUE(batch->keyframe);
    ASSERT_EQ(batch->packets.size(), 2U);

    const auto& first = batch->packets.front();
    EXPECT_EQ(first.data.size(), static_cast<size_t>(quanta::QuantaPacket::MTU));
    EXPECT_EQ(first.data[0], 0x34);
    EXPECT_EQ(first.data[1], 0x12);
    EXPECT_EQ(first.data[2], 0);
    EXPECT_EQ(first.data[3], 2);
    EXPECT_EQ(first.data[4], 1);
    EXPECT_EQ(first.data[quanta::QuantaPacket::HEADER_SIZE], 0);

    const auto& second = batch->packets.back();
    EXPECT_EQ(second.data.size(), static_cast<size_t>(quanta::QuantaPacket::HEADER_SIZE) + 7U);
    EXPECT_EQ(second.data[0], 0x34);
    EXPECT_EQ(second.data[1], 0x12);
    EXPECT_EQ(second.data[2], 1);
    EXPECT_EQ(second.data[3], 2);
    EXPECT_EQ(second.data[4], 1);
    EXPECT_EQ(
        second.data[quanta::QuantaPacket::HEADER_SIZE],
        static_cast<uint8_t>(quanta::QuantaPacket::MAX_PAYLOAD & 0xff));
}

TEST(QuantaPacketQueue, SendsFrameQueuesPacketByPacket) {
    quanta::QuantaPacketQueue queue;

    auto two_packet_frame = quanta::build_quanta_packets(
        make_encoded_packet(static_cast<size_t>(quanta::QuantaPacket::MAX_PAYLOAD) + 7, false), 1);
    auto one_packet_frame = quanta::build_quanta_packets(make_encoded_packet(12, true), 2);
    ASSERT_TRUE(two_packet_frame.has_value());
    ASSERT_TRUE(one_packet_frame.has_value());

    EXPECT_TRUE(queue.push_frame(std::move(*two_packet_frame)));
    EXPECT_TRUE(queue.push_frame(std::move(*one_packet_frame)));

    auto packet = queue.pop_packet();
    ASSERT_TRUE(packet.has_value());
    EXPECT_EQ(packet->packet.seq, 1);
    EXPECT_EQ(packet->packet.fragment_index, 0);
    EXPECT_EQ(packet->remaining_packets, 2U);
    EXPECT_EQ(packet->remaining_frames, 2U);
    auto stats = queue.stats();
    EXPECT_EQ(stats.queued_packets, 2U);
    EXPECT_EQ(stats.queued_frames, 2U);

    packet = queue.pop_packet();
    ASSERT_TRUE(packet.has_value());
    EXPECT_EQ(packet->packet.seq, 1);
    EXPECT_EQ(packet->packet.fragment_index, 1);
    EXPECT_EQ(packet->remaining_packets, 1U);
    EXPECT_EQ(packet->remaining_frames, 1U);
    stats = queue.stats();
    EXPECT_EQ(stats.queued_packets, 1U);
    EXPECT_EQ(stats.queued_frames, 1U);

    packet = queue.pop_packet();
    ASSERT_TRUE(packet.has_value());
    EXPECT_EQ(packet->packet.seq, 2);
    EXPECT_TRUE(packet->packet.keyframe);
    EXPECT_EQ(packet->remaining_packets, 0U);
    EXPECT_EQ(packet->remaining_frames, 0U);
    stats = queue.stats();
    EXPECT_EQ(stats.queued_packets, 0U);
    EXPECT_EQ(stats.queued_frames, 0U);

    EXPECT_FALSE(queue.pop_packet().has_value());

    stats = queue.stats();
    EXPECT_EQ(stats.packets_dequeued, 3U);
    EXPECT_EQ(stats.queued_packets, 0U);
    EXPECT_EQ(stats.queued_frames, 0U);
}

TEST(QuantaPacketQueue, BuffersLatestFiveFrameQueues) {
    quanta::QuantaPacketQueue queue;

    for (uint16_t seq = 1; seq <= 6; ++seq) {
        auto frame = quanta::build_quanta_packets(make_encoded_packet(12, false), seq);
        ASSERT_TRUE(frame.has_value());
        EXPECT_TRUE(queue.push_frame(std::move(*frame)));
    }

    auto packets = queue.take_all();
    ASSERT_EQ(packets.size(), quanta::QuantaPacketQueue::kFrameBufferDepth);
    for (size_t i = 0; i < packets.size(); ++i) {
        EXPECT_EQ(packets[i].seq, static_cast<uint16_t>(i + 2));
    }

    const auto stats = queue.stats();
    EXPECT_EQ(stats.frames_enqueued, 6U);
    EXPECT_EQ(stats.packets_dequeued, quanta::QuantaPacketQueue::kFrameBufferDepth);
}

TEST(QuantaPacketQueue, DropsFramesLargerThanSixPackets) {
    quanta::QuantaPacketQueue queue;
    auto previous  = quanta::build_quanta_packets(make_encoded_packet(12, false), 8);
    auto oversized = quanta::build_quanta_packets(
        make_encoded_packet(static_cast<size_t>(quanta::QuantaPacket::MAX_PAYLOAD) * 7, false), 9);
    ASSERT_TRUE(previous.has_value());
    ASSERT_TRUE(oversized.has_value());
    ASSERT_EQ(oversized->packets.size(), 7U);

    EXPECT_TRUE(queue.push_frame(std::move(*previous)));
    EXPECT_FALSE(queue.push_frame(std::move(*oversized)));

    auto packets = queue.take_all();
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(packets.front().seq, 8);

    const auto stats = queue.stats();
    EXPECT_EQ(stats.oversized_frame_drops, 1U);
}
