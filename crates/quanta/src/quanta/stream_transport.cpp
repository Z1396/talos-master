#include "quanta/stream_transport.hpp"  // 传输队列、分片包、统计结构体、快照缓冲区声明
#include "quanta/stream_encoder.hpp"    // EncodedPacket 编码裸流包定义

// C++标准库头文件
#include <algorithm>    // std::min 分片大小计算
#include <cstring>      // std::memcpy 二进制内存拷贝
#include <limits>       // std::numeric_limits 类型最大值限制校验
#include <utility>      // std::move 转移容器所有权，减少拷贝开销

namespace quanta {

/**
 * @brief 数据包队列构造函数
 * @param max_packets 单帧最大分片包数量上限，0使用默认常量kDefaultMaxPackets
 * 内部创建帧快照读写分离缓冲区，初始化统计指标
 */
QuantaPacketQueue::QuantaPacketQueue(size_t max_packets)
    // 初始化帧快照读写端（生产者写、消费者读无锁环形缓冲区）
    : frame_buffer_(make_frame_snapshot_endpoints())
    // 最大单帧分片包数：入参0则使用默认值，否则使用传入上限
    , max_packets_(max_packets == 0 ? kDefaultMaxPackets : max_packets) {
    // 初始化统计结构体固定配置
    stats_.max_packets      = max_packets_;
    stats_.max_frame_queues = kFrameBufferDepth;
}

/**
 * @brief 生产者入队一整帧分片包批次
 * @param batch 一帧拆分后的所有分片包批量容器，标记是否为关键帧
 * @return true 入队成功；false 入队丢弃（空包/分片超限/缓冲区溢出丢弃）
 * 核心逻辑：分片校验→封装帧结构→写入生产者历史快照缓冲区→溢出丢弃旧帧→更新统计
 */
bool QuantaPacketQueue::push_frame(QuantaPacketBatch batch) {
    // 批次无任何分片包，直接返回失败，不处理
    if (batch.packets.empty())
        return false;

    // 当前帧分片数量超过单帧最大包限制，直接丢弃整帧，统计丢帧指标
    if (batch.packets.size() > max_packets_) {
        // 加锁保护多线程统计变量
        std::lock_guard lock(stats_mutex_);
        stats_.frames_dropped++;
        stats_.oversized_frame_drops++;
        return false;
    }

    // 记录当前帧分片总数，用于统计入队包数量
    const auto packet_count = batch.packets.size();
    // 封装单帧所有分片的帧结构体，分配全局自增唯一帧ID
    PacketFrameQueue frame{
        .frame_id = next_frame_id_++,  // 全局单调递增帧序号，用于区分新旧帧
        .packets  = {},
        .keyframe = batch.keyframe,    // 标记该帧是否为IDR关键帧
    };
    // 转移所有分片包所有权到帧容器，避免拷贝
    for (auto& packet : batch.packets) {
        frame.packets.push_back(std::move(packet));
    }

    // 生产者历史双端队列存入当前完整帧
    producer_history_.push_back(std::move(frame));
    std::size_t dropped_frames = 0;
    // 读取消费者最后已消费的帧ID（无锁原子内存序acquire保证可见性）
    const auto last_ingested   = last_ingested_frame_id_.load(std::memory_order_acquire);
    // 生产者历史缓冲区超出最大深度，丢弃最旧帧
    while (producer_history_.size() > kFrameBufferDepth) {
        // 未被消费者读取的旧帧才计入缓冲区溢出丢帧统计
        if (producer_history_.front().frame_id > last_ingested) {
            ++dropped_frames;
        }
        producer_history_.pop_front();
    }

    // 加锁更新全局统计指标：入队帧、入队分片包、缓冲区溢出丢帧
    {
        std::lock_guard lock(stats_mutex_);
        stats_.frames_enqueued++;
        stats_.packets_enqueued += packet_count;
        stats_.frames_dropped += dropped_frames;
        stats_.frame_buffer_drops += dropped_frames;
    }
    // 刷新生产者侧实时队列统计
    update_stats_from_producer();
    // 将生产者最新快照写入无锁读写缓冲区，供消费者线程拉取
    publish_producer_history();
    return true;
}

/**
 * @brief 消费者非阻塞弹出单个分片包
 * @return 存在分片包返回DequeuedQuantaPacket（分片包+剩余缓存数量）；无包返回std::nullopt
 * 流程：拉取生产者最新帧快照→填充本地消费者帧缓存→弹出当前帧首分片包
 */
std::optional<DequeuedQuantaPacket> QuantaPacketQueue::pop_packet() {
    // 拉取生产者最新完整帧快照，同步生产者新入队数据
    ingest_latest_snapshot();

    // 当前正在消费的帧分片已全部取完，从消费者帧缓存取出下一整帧
    if (current_packets_.empty() && !consumer_frames_.empty()) {
        auto frame = std::move(consumer_frames_.front());
        consumer_frames_.pop_front();
        // 标记当前正在消费的帧是否为关键帧
        current_keyframe_ = frame.keyframe;
        // 接管该帧所有分片包到当前消费队列
        current_packets_  = std::move(frame.packets);
    }

    // 无任何可消费分片包，清空关键帧标记，更新统计并返回空
    if (current_packets_.empty()) {
        current_keyframe_ = false;
        update_stats_from_consumer();
        return std::nullopt;
    }

    // 取出当前帧第一个分片包，转移所有权
    auto packet = std::move(current_packets_.front());
    current_packets_.pop_front();

    // 加锁统计出队分片包计数
    {
        std::lock_guard lock(stats_mutex_);
        stats_.packets_dequeued++;
    }
    // 当前帧分片全部取完，重置关键帧标记
    if (current_packets_.empty()) {
        current_keyframe_ = false;
    }
    // 更新消费者侧实时缓存统计
    update_stats_from_consumer();
    // 返回分片包+剩余缓存数量信息
    return DequeuedQuantaPacket{
        .packet            = std::move(packet),
        .remaining_packets = consumer_packet_count(),
        .remaining_frames  = consumer_frame_count(),
    };
}

/**
 * @brief 一次性取出队列中所有剩余分片包，清空消费缓存
 * @return 所有未消费分片包数组，清空本地消费者缓存
 */
std::vector<QuantaPacket> QuantaPacketQueue::take_all() {
    std::vector<QuantaPacket> out;
    // 循环持续弹出分片包直到队列为空
    while (auto packet = pop_packet()) {
        out.push_back(std::move(packet->packet));
    }
    return out;
}

/**
 * @brief 线程安全读取当前队列全量运行统计快照
 * @return 复制一份独立的队列统计结构体，避免多线程读写竞争
 */
QuantaPacketQueueStats QuantaPacketQueue::stats() const {
    std::lock_guard lock(stats_mutex_);
    auto copy = stats_;
    return copy;
}

/**
 * @brief 创建帧快照无锁缓冲区读写端（生产者写、消费者读分离）
 * @return FrameSnapshotEndpoints 包含writer、reader一对句柄
 */
auto QuantaPacketQueue::make_frame_snapshot_endpoints() -> FrameSnapshotEndpoints {
    // 调用无锁快照缓冲区静态创建方法，分离读写句柄
    auto [writer, reader] = FrameSnapshotBuffer::create();
    return FrameSnapshotEndpoints{
        .writer = std::move(writer),
        .reader = std::move(reader),
    };
}

/**
 * @brief 将生产者当前所有缓存帧快照写入无锁缓冲区，供消费者同步
 */
void QuantaPacketQueue::publish_producer_history() {
    PacketFrameSnapshot snapshot;
    // 预分配内存，避免多次扩容
    snapshot.frames.reserve(producer_history_.size());
    // 拷贝生产者所有缓存帧到快照
    for (const auto& frame : producer_history_) {
        snapshot.frames.push_back(frame);
    }
    // 写入无锁缓冲区，覆盖上一版快照，消费者仅读取最新一版
    frame_buffer_.writer.write(std::move(snapshot));
}

/**
 * @brief 消费者拉取生产者最新帧快照，同步新增未消费帧到本地消费者缓存
 * 原子帧ID保证多线程帧顺序严格有序，不会重复消费、不会漏帧
 */
void QuantaPacketQueue::ingest_latest_snapshot() {
    // 读取缓冲区最新快照，无新快照直接返回
    auto snapshot = frame_buffer_.reader.read();
    if (!snapshot) {
        return;
    }

    // 原子加载上一次消费到的最大帧ID
    auto last_ingested = last_ingested_frame_id_.load(std::memory_order_acquire);
    // 遍历快照内所有帧，只处理未消费过的新帧
    for (auto& frame : snapshot->frames) {
        if (frame.frame_id <= last_ingested) {
            continue;
        }
        // 更新已消费最大帧ID
        last_ingested = frame.frame_id;
        // 转移帧所有权到消费者本地缓存
        consumer_frames_.push_back(std::move(frame));
    }
    // 原子存储最新消费帧ID，release内存序同步对生产者可见
    last_ingested_frame_id_.store(last_ingested, std::memory_order_release);

    // 裁剪消费者本地缓存，不超过最大帧深度限制
    trim_consumer_frames();
    // 更新消费者侧实时统计
    update_stats_from_consumer();
}

/**
 * @brief 裁剪消费者本地帧缓存，限制总缓存帧数量不超过kFrameBufferDepth
 * 超出上限则丢弃最旧未消费帧，同步统计丢帧指标
 */
void QuantaPacketQueue::trim_consumer_frames() {
    std::size_t dropped_frames    = 0;
    // 当前正在消费的帧占用1个深度，其余缓存帧累加计算总占用深度
    const auto active_frame_count = current_packets_.empty() ? 0U : 1U;
    // 循环丢弃队头旧帧直到总缓存帧数不超限
    while (active_frame_count + consumer_frames_.size() > kFrameBufferDepth) {
        consumer_frames_.pop_front();
        ++dropped_frames;
    }

    // 无丢弃帧直接返回，无需更新统计
    if (dropped_frames == 0) {
        return;
    }

    // 加锁更新缓冲区溢出丢帧统计
    std::lock_guard lock(stats_mutex_);
    stats_.frames_dropped += dropped_frames;
    stats_.frame_buffer_drops += dropped_frames;
}

/**
 * @brief 统计消费者侧所有待消费分片包总数量
 * @return 当前缓存内所有未弹出分片包总数
 */
size_t QuantaPacketQueue::consumer_packet_count() const {
    size_t count = current_packets_.size();
    // 累加所有缓存帧内分片包数量
    for (const auto& frame : consumer_frames_) {
        count += frame.packets.size();
    }
    return count;
}

/**
 * @brief 统计消费者侧待消费完整帧总数量
 * @return 未消费完整帧总数（含当前正在读取的帧）
 */
size_t QuantaPacketQueue::consumer_frame_count() const {
    // 缓存队列帧数量 + 当前正在读取的1帧（如有）
    return consumer_frames_.size() + (current_packets_.empty() ? 0U : 1U);
}

/**
 * @brief 更新生产者侧实时缓存统计（入队后调用）
 * 统计当前生产者缓冲区缓存包、帧数量、最新帧是否关键帧
 */
void QuantaPacketQueue::update_stats_from_producer() {
    size_t packet_count = 0;
    // 累加生产者所有缓存帧分片包总数
    for (const auto& frame : producer_history_) {
        packet_count += frame.packets.size();
    }

    // 加锁更新统计变量
    std::lock_guard lock(stats_mutex_);
    stats_.queued_packets   = packet_count;
    stats_.queued_frames    = producer_history_.size();
    // 判断最后一帧是否为关键帧
    stats_.current_keyframe = !producer_history_.empty() && producer_history_.back().keyframe;
}

/**
 * @brief 更新消费者侧实时缓存统计（出队/同步快照后调用）
 * 统计当前消费者可读取缓存包、帧、当前帧是否关键帧
 */
void QuantaPacketQueue::update_stats_from_consumer() {
    std::lock_guard lock(stats_mutex_);
    stats_.queued_packets   = consumer_packet_count();
    stats_.queued_frames    = consumer_frame_count();
    stats_.current_keyframe = current_keyframe_;
}

// ===================== 分片包构建工具函数 =====================
/**
 * @brief 将单个HEVC/H265编码裸流包，按传输最大载荷分片拆分生成一批QuantaPacket传输分片包
 * @param packet 编码器输出完整EncodedPacket裸流
 * @param seq 全局视频流序列号，同一流固定不变
 * @return 分片批量容器，携带分片头部、分片索引、总分片数、关键帧标记
 */
std::expected<QuantaPacketBatch, std::string>
    build_quanta_packets(const EncodedPacket& packet, uint16_t seq) {
    // 空编码包直接返回错误
    if (packet.size == 0 || packet.data == nullptr)
        return std::unexpected("encoded packet is empty");

    // 计算需要分片数量：向上取整，总字节 / 单分片最大载荷
    const size_t fragment_count = (packet.size + static_cast<size_t>(QuantaPacket::MAX_PAYLOAD) - 1)
                                / static_cast<size_t>(QuantaPacket::MAX_PAYLOAD);
    // 分片数量0 或 超过uint8_t最大值255（头部仅1字节存储分片总数），分片超限报错
    if (fragment_count == 0 || fragment_count > std::numeric_limits<uint8_t>::max()) {
        return std::unexpected("encoded packet requires too many QuantaPacket fragments");
    }

    QuantaPacketBatch batch;
    batch.keyframe = packet.keyframe;
    // 预分配分片容器内存
    batch.packets.reserve(fragment_count);

    size_t offset = 0; // 原始裸流读取偏移
    // 循环生成每一片分片包
    for (size_t i = 0; i < fragment_count; ++i) {
        // 本分片有效载荷长度，末尾分片取剩余不足MAX_PAYLOAD字节
        const size_t payload_size =
            std::min(static_cast<size_t>(QuantaPacket::MAX_PAYLOAD), packet.size - offset);

        QuantaPacket out;
        // 填充分片头部元数据
        out.seq            = seq;
        out.fragment_index = static_cast<uint8_t>(i);
        out.fragment_count = static_cast<uint8_t>(fragment_count);
        out.keyframe       = packet.keyframe;
        // 分配完整包内存：头部固定长度 + 载荷长度
        out.data.resize(static_cast<size_t>(QuantaPacket::HEADER_SIZE) + payload_size);

        // 序列化2字节序列号（小端序）
        out.data[0] = static_cast<uint8_t>(seq & 0xffu);
        out.data[1] = static_cast<uint8_t>((seq >> 8) & 0xffu);
        // 分片索引
        out.data[2] = out.fragment_index;
        // 总分片数量
        out.data[3] = out.fragment_count;
        // 关键帧标记 0/1
        out.data[4] = out.keyframe ? 1U : 0U;
        // 拷贝裸流载荷到分片包头部后方
        std::memcpy(
            out.data.data() + QuantaPacket::HEADER_SIZE, packet.data.get() + offset, payload_size);

        // 偏移前进，处理下一分片
        offset += payload_size;
        batch.packets.push_back(std::move(out));
    }

    return batch;
}

} // namespace quanta