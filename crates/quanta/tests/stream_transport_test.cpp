// 流编码器、传输通道头文件
#include "quanta/stream_encoder.hpp"
#include "quanta/stream_transport.hpp"

// GoogleTest 单元测试框架
#include <gtest/gtest.h>

// 基础整数类型、智能指针
#include <cstdint>
#include <memory>

namespace {
/**
 * @brief 构造测试用编码裸码流包
 * @param size 原始码流字节长度
 * @param keyframe 是否关键帧
 * @return 填充递增测试数据的 EncodedPacket
 */
quanta::EncodedPacket make_encoded_packet(size_t size, bool keyframe) {
    quanta::EncodedPacket packet;
    // 动态分配uint8_t数组存储码流
    packet.data     = std::make_unique<uint8_t[]>(size);
    packet.size     = size;
    packet.keyframe = keyframe;
    // 填充测试数据：0~255循环递增，方便分片校验
    for (size_t i = 0; i < size; ++i) {
        packet.data[i] = static_cast<uint8_t>(i & 0xffu);
    }
    return packet;
}

} // 匿名测试命名空间，隔离测试辅助函数

/**
 * @brief 测试：QuantaPacket 头部生成、超大码流自动分片逻辑
 * 场景：码流长度 = 单包最大负载 +7，会拆分为2个分片包
 * 校验点：
 * 1. 批量分包返回有效，标记关键帧
 * 2. 分片数量=2
 * 3. 包头字节序、分片索引、总分片数、关键帧标记正确性
 * 4. 分片负载数据连续性
 */
TEST(QuantaPacket, BuildsQuantaPacketHeaderAndFragmentsAnnexBPayload) {
    // 构造超大码流：MAX_PAYLOAD +7 字节，必然产生2个分片
    auto encoded =
        make_encoded_packet(static_cast<size_t>(quanta::QuantaPacket::MAX_PAYLOAD) + 7, true);

    // 按流序列号0x1234拆分码流，生成分片包批量
    auto batch = quanta::build_quanta_packets(encoded, 0x1234);
    // 断言分包无错误
    ASSERT_TRUE(batch.has_value()) << batch.error();
    // 批量标记为关键帧（继承原始packet标记）
    ASSERT_TRUE(batch->keyframe);
    // 预期2个分片
    ASSERT_EQ(batch->packets.size(), 2U);

    // ===== 第一个分片（第0片）校验 =====
    const auto& first = batch->packets.front();
    // 包总长度 = 头部 + 完整MAX_PAYLOAD负载
    EXPECT_EQ(first.data.size(), static_cast<size_t>(quanta::QuantaPacket::MTU));
    // 包头0~1：小端存储流序列号 0x1234 → 低字节0x34，高字节0x12
    EXPECT_EQ(first.data[0], 0x34);
    EXPECT_EQ(first.data[1], 0x12);
    // 包头2：分片索引 fragment_index = 0
    EXPECT_EQ(first.data[2], 0);
    // 包头3：本帧总分片数量 =2
    EXPECT_EQ(first.data[3], 2);
    // 包头4：关键帧标记 1=true
    EXPECT_EQ(first.data[4], 1);
    // 头部后第一个负载字节 = 0，与填充逻辑一致
    EXPECT_EQ(first.data[quanta::QuantaPacket::HEADER_SIZE], 0);

    // ===== 第二个分片（第1片）校验 =====
    const auto& second = batch->packets.back();
    // 包长度 = 头部 + 剩余7字节负载
    EXPECT_EQ(second.data.size(), static_cast<size_t>(quanta::QuantaPacket::HEADER_SIZE) + 7U);
    // 流序列号不变
    EXPECT_EQ(second.data[0], 0x34);
    EXPECT_EQ(second.data[1], 0x12);
    // 分片索引 = 1
    EXPECT_EQ(second.data[2], 1);
    // 总分片数依旧2
    EXPECT_EQ(second.data[3], 2);
    // 关键帧标记1
    EXPECT_EQ(second.data[4], 1);
    // 分片首负载字节 = MAX_PAYLOAD & 0xff，对应原始码流偏移MAX_PAYLOAD处的值
    EXPECT_EQ(
        second.data[quanta::QuantaPacket::HEADER_SIZE],
        static_cast<uint8_t>(quanta::QuantaPacket::MAX_PAYLOAD & 0xff));
}

/**
 * @brief 测试：PacketQueue 帧入队、分片逐包出队逻辑
 * 流程：
 * 1. 推入2帧：2分片非关键帧、1分片关键帧
 * 2. 依次pop_packet取出分片，校验分片序号、剩余包/帧计数、序列号
 * 3. 队列为空pop返回空，校验出队统计指标
 */
TEST(QuantaPacketQueue, SendsFrameQueuesPacketByPacket) {
    quanta::QuantaPacketQueue queue;

    // 构造两分片非关键帧，序列号1
    auto two_packet_frame = quanta::build_quanta_packets(
        make_encoded_packet(static_cast<size_t>(quanta::QuantaPacket::MAX_PAYLOAD) + 7, false), 1);
    // 构造单分片关键帧，序列号2
    auto one_packet_frame = quanta::build_quanta_packets(make_encoded_packet(12, true), 2);
    // 分包接口返回合法批量
    ASSERT_TRUE(two_packet_frame.has_value());
    ASSERT_TRUE(one_packet_frame.has_value());

    // 推入两帧到队列
    EXPECT_TRUE(queue.push_frame(std::move(*two_packet_frame)));
    EXPECT_TRUE(queue.push_frame(std::move(*one_packet_frame)));

    // ========== 取出第1个分片：seq=1，分片0 ==========
    auto packet = queue.pop_packet();
    ASSERT_TRUE(packet.has_value());
    EXPECT_EQ(packet->packet.seq, 1);
    EXPECT_EQ(packet->packet.fragment_index, 0);
    // 当前帧剩余分片 2-1=1
    EXPECT_EQ(packet->remaining_packets, 2U);
    // 队列剩余总帧数 2
    EXPECT_EQ(packet->remaining_frames, 2U);
    // 读取队列统计
    auto stats = queue.stats();
    EXPECT_EQ(stats.queued_packets, 2U);
    EXPECT_EQ(stats.queued_frames, 2U);

    // ========== 取出第2个分片：seq=1，分片1 ==========
    packet = queue.pop_packet();
    ASSERT_TRUE(packet.has_value());
    EXPECT_EQ(packet->packet.seq, 1);
    EXPECT_EQ(packet->packet.fragment_index, 1);
    // 当前帧无剩余分片
    EXPECT_EQ(packet->remaining_packets, 1U);
    // 队列剩余1帧
    EXPECT_EQ(packet->remaining_frames, 1U);
    stats = queue.stats();
    EXPECT_EQ(stats.queued_packets, 1U);
    EXPECT_EQ(stats.queued_frames, 1U);

    // ========== 取出第3个分片：seq=2，完整关键帧 ==========
    packet = queue.pop_packet();
    ASSERT_TRUE(packet.has_value());
    EXPECT_EQ(packet->packet.seq, 2);
    EXPECT_TRUE(packet->packet.keyframe);
    // 无剩余分片、无剩余帧
    EXPECT_EQ(packet->remaining_packets, 0U);
    EXPECT_EQ(packet->remaining_frames, 0U);
    stats = queue.stats();
    EXPECT_EQ(stats.queued_packets, 0U);
    EXPECT_EQ(stats.queued_frames, 0U);

    // 队列已空，pop返回std::nullopt
    EXPECT_FALSE(queue.pop_packet().has_value());

    // 全局统计校验：累计出队3个包，当前队列0
    stats = queue.stats();
    EXPECT_EQ(stats.packets_dequeued, 3U);
    EXPECT_EQ(stats.queued_packets, 0U);
    EXPECT_EQ(stats.queued_frames, 0U);
}

/**
 * @brief 测试：队列帧缓冲深度限制（最多缓存最新5帧）
 * 推入6帧，仅保留后5帧，丢弃最早第1帧
 */
TEST(QuantaPacketQueue, BuffersLatestFiveFrameQueues) {
    quanta::QuantaPacketQueue queue;

    // 连续推入序列号1~6共6帧
    for (uint16_t seq = 1; seq <= 6; ++seq) {
        auto frame = quanta::build_quanta_packets(make_encoded_packet(12, false), seq);
        ASSERT_TRUE(frame.has_value());
        EXPECT_TRUE(queue.push_frame(std::move(*frame)));
    }

    // 一次性取出队列全部缓存帧
    auto packets = queue.take_all();
    // 缓冲深度kFrameBufferDepth=5，仅保留5帧
    ASSERT_EQ(packets.size(), quanta::QuantaPacketQueue::kFrameBufferDepth);
    // 丢弃seq=1，剩余seq=2,3,4,5,6
    for (size_t i = 0; i < packets.size(); ++i) {
        EXPECT_EQ(packets[i].seq, static_cast<uint16_t>(i + 2));
    }

    // 统计校验：入队6帧，出队5个分片
    const auto stats = queue.stats();
    EXPECT_EQ(stats.frames_enqueued, 6U);
    EXPECT_EQ(stats.packets_dequeued, quanta::QuantaPacketQueue::kFrameBufferDepth);
}

/**
 * @brief 测试：超过最大分片数的超大帧直接丢弃
 * 规则：单帧拆分后分片数>6则push_frame返回false并丢弃
 */
TEST(QuantaPacketQueue, DropsFramesLargerThanSixPackets) {
    quanta::QuantaPacketQueue queue;
    // 普通帧：1个分片，seq=8
    auto previous  = quanta::build_quanta_packets(make_encoded_packet(12, false), 8);
    // 超大码流：MAX_PAYLOAD*7，拆分7个分片，超限丢弃
    auto oversized = quanta::build_quanta_packets(
        make_encoded_packet(static_cast<size_t>(quanta::QuantaPacket::MAX_PAYLOAD) * 7, false), 9);
    ASSERT_TRUE(previous.has_value());
    ASSERT_TRUE(oversized.has_value());
    // 校验该帧生成7个分片，触发丢弃规则
    ASSERT_EQ(oversized->packets.size(), 7U);

    // 正常帧入队成功
    EXPECT_TRUE(queue.push_frame(std::move(*previous)));
    // 超大分片帧入队失败，被丢弃
    EXPECT_FALSE(queue.push_frame(std::move(*oversized)));

    // 取出全部缓存，仅保留seq=8的一帧
    auto packets = queue.take_all();
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(packets.front().seq, 8);

    // 统计：超大帧丢弃计数+1
    const auto stats = queue.stats();
    EXPECT_EQ(stats.oversized_frame_drops, 1U);
}