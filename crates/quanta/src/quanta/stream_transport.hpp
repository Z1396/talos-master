#pragma once
// 头文件保护宏，防止头文件重复包含引发重定义编译错误

// C++标准库头文件
#include <atomic>        // 原子类型，无锁多线程帧ID同步
#include <cstddef>       // size_t 标准无符号长度类型
#include <cstdint>       // 固定宽度整数 uint8_t/uint16_t/uint64_t 跨平台统一字节
#include <deque>         // 双端队列，存储帧分片包缓存
#include <expected>      // C++23 预期返回类型，统一承载结果+错误信息
#include <mutex>         // 互斥锁，保护统计变量多线程读写
#include <optional>      // 可选返回值，无包时返回空
#include <span>          // 只读连续内存视图，快速读取包二进制数据
#include <string>        // 存储错误描述字符串
#include <vector>        // 动态数组，存储分片包、帧快照

// 自研单生产者单消费者三缓冲无锁队列，用于帧快照读写分离
#include "primitive/spsc_triple_buffer.hpp"

namespace quanta {

// 结构体1字节对齐关闭，严格按1字节紧凑打包，保证网络头部5字节固定长度
#pragma pack(push, 1)
/**
 * @brief 传输分片包固定头部结构体，网络二进制序列化直接映射
 * 总长度固定5字节，对应QuantaPacket::HEADER_SIZE
 */
struct QuantaPacketHeader {
    uint16_t seq;        // 2字节：视频流全局序列号，同一视频流全程不变，用于多路流区分
    uint8_t frag_idx;    // 1字节：当前分片索引，从0开始
    uint8_t frag_total;  // 1字节：当前完整帧总分片数量
    uint8_t keyframe;    // 1字节：关键帧标记 0=非IDR 1=IDR关键帧
};
#pragma pack(pop) // 恢复默认结构体对齐规则

// 前置声明，仅引用不完整定义，避免循环头文件依赖
struct EncodedPacket;

/**
 * @brief 单路网络传输分片包完整封装结构
 * 基于MTU限制拆分大编码裸流，每个实例代表一个UDP分片数据包
 */
struct QuantaPacket {
    // 固定头部字节长度，对应QuantaPacketHeader 5字节
    static constexpr int HEADER_SIZE = 5;
    // 链路MTU最大传输单元，单个UDP包总字节上限
    static constexpr int MTU         = 298;
    // 分片有效载荷最大字节 = MTU总长度 - 头部固定5字节
    static constexpr int MAX_PAYLOAD = MTU - HEADER_SIZE;

    std::vector<uint8_t> data;  // 完整二进制缓冲区：前5字节头部 + 后面裸流载荷
    uint16_t seq           = 0; // 流序列号，与头部seq一致
    uint8_t fragment_index = 0; // 当前分片索引 0~255
    uint8_t fragment_count = 0; // 本帧总分片数量
    bool keyframe          = false; // 是否属于IDR关键帧分片

    /**
     * @brief 获取包完整二进制只读视图，无需拷贝内存
     * @return std::span<const uint8_t> 覆盖data全部内存
     * noexcept 无异常抛出
     * [[nodiscard]] 强制接收返回值，防止丢弃数据视图
     */
    [[nodiscard]] std::span<const uint8_t> bytes() const noexcept { return data; }
};

// 静态断言：校验头部结构体严格5字节，防止打包对齐错误导致网络解析错乱
static_assert(sizeof(QuantaPacketHeader) == QuantaPacket::HEADER_SIZE);

/**
 * @brief 一帧完整编码图像对应的所有分片包批量容器
 * 一帧裸流拆分后所有QuantaPacket统一存放，批量入队队列
 */
struct QuantaPacketBatch {
    std::vector<QuantaPacket> packets; // 本帧所有分片包数组
    bool keyframe = false;             // 整帧是否为IDR关键帧
};

/**
 * @brief 队列运行时全量统计指标结构体
 * 记录入队、出队、各类丢帧、实时缓存数量、关键帧状态
 * 多线程加锁拷贝读取，用于监控推流链路负载与丢包情况
 */
struct QuantaPacketQueueStats {
    uint64_t frames_enqueued       = 0;  // 累计成功入队总帧数
    uint64_t frames_dropped        = 0;  // 累计所有类型丢弃总帧数
    uint64_t idr_protected_drops   = 0;  // 预留IDR保护丢帧统计（扩展字段）
    uint64_t oversized_frame_drops = 0;  // 单帧分片数量超限丢弃帧数
    uint64_t frame_buffer_drops    = 0;  // 缓冲区深度超限溢出丢弃帧数
    uint64_t packets_enqueued      = 0;  // 累计入队总分片包数量
    uint64_t packets_dequeued      = 0;  // 累计出队发送总分片包数量
    size_t queued_packets          = 0;  // 当前队列待发送分片包总数
    size_t queued_frames           = 0;  // 当前队列待发送完整帧总数
    size_t max_packets             = 0;  // 单帧允许最大分片包上限
    size_t max_frame_queues        = 0;  // 队列最大缓存帧深度上限
    bool current_keyframe          = false; // 队列队首待发送帧是否为关键帧
};

/**
 * @brief 出队单个分片包返回结构体
 * 携带分片包本体 + 当前队列剩余缓存数量，上层发送器可感知链路积压
 */
struct DequeuedQuantaPacket {
    QuantaPacket packet;      // 取出的单个分片包
    size_t remaining_packets = 0; // 队列剩余未发送分片包总数
    size_t remaining_frames  = 0; // 队列剩余未发送完整帧总数
};

/**
 * @brief 分片包收发双端无锁队列核心类
 * 架构：SPSC三缓冲帧快照分离生产者/消费者，仅统计变量加锁，高并发低延迟
 * 生产者：编码线程 push_frame 批量入队一帧分片
 * 消费者：网络发送线程 pop_packet 单包出队发送
 */
class QuantaPacketQueue {
    /**
     * @brief 单帧所有分片包容器，内部队列存储单元
     */
    struct PacketFrameQueue {
        uint64_t frame_id = 0;                // 全局单调递增唯一帧ID，用于时序同步
        std::deque<QuantaPacket> packets;     // 本帧所有分片包双端队列
        bool keyframe = false;                // 该帧是否为IDR关键帧
    };

    /**
     * @brief 帧快照结构体，生产者批量写入、消费者批量读取的完整帧缓存镜像
     */
    struct PacketFrameSnapshot {
        std::vector<PacketFrameQueue> frames; // 快照内全部缓存帧
    };

    // 引入自研单生产者单消费者三缓冲无锁队列模板，存储帧快照
    using FrameSnapshotBuffer = talos::primitive::SpscTripleBuffer<PacketFrameSnapshot>;
    // 快照写入端类型（生产者线程专用）
    using FrameSnapshotWriter = FrameSnapshotBuffer::template Write<PacketFrameSnapshot>;
    // 快照读取端类型（消费者线程专用）
    using FrameSnapshotReader = FrameSnapshotBuffer::template Read<PacketFrameSnapshot>;

    /**
     * @brief 快照读写端配对句柄，封装三缓冲读写对象
     */
    struct FrameSnapshotEndpoints {
        FrameSnapshotWriter writer; // 生产者写句柄
        FrameSnapshotReader reader; // 消费者读句柄
    };

public:
    // 静态常量定义
    // 单帧默认最大分片包数量，超过则整帧丢弃
    static constexpr size_t kDefaultMaxPackets = 15;
    // 队列最大缓存帧深度上限，超出自动丢弃旧帧防内存膨胀
    static constexpr size_t kFrameBufferDepth  = 30;

    /**
     * @brief 队列构造函数
     * @param max_packets 单帧分片包上限，不传使用默认15
     */
    explicit QuantaPacketQueue(size_t max_packets = kDefaultMaxPackets);

    // 禁用拷贝构造、拷贝赋值，队列独占资源不可复制
    QuantaPacketQueue(const QuantaPacketQueue&)            = delete;
    QuantaPacketQueue& operator=(const QuantaPacketQueue&) = delete;

    /**
     * @brief 生产者批量入队一整帧所有分片包
     * @param batch 一帧分片包批量容器
     * @return true 入队成功；false 入队丢弃（空包/分片超限/缓冲区溢出）
     */
    bool push_frame(QuantaPacketBatch batch);

    /**
     * @brief 消费者非阻塞取出单个分片包
     * @return 有分片包返回DequeuedQuantaPacket；无待发送包返回std::nullopt
     */
    std::optional<DequeuedQuantaPacket> pop_packet();

    /**
     * @brief 一次性取出队列所有剩余分片包，清空全部消费缓存
     * @return 所有未发送分片包数组
     */
    std::vector<QuantaPacket> take_all();

    /**
     * @brief 线程安全读取当前队列完整统计快照
     * @return 拷贝独立统计结构体，多线程读写无竞争
     */
    [[nodiscard]] QuantaPacketQueueStats stats() const;

private:
    /**
     * @brief 静态创建三缓冲快照读写端配对句柄
     * @return FrameSnapshotEndpoints 包含writer、reader
     */
    [[nodiscard]] static FrameSnapshotEndpoints make_frame_snapshot_endpoints();

    /**
     * @brief 生产者将当前本地缓存所有帧打包为快照，写入三缓冲无锁队列，供消费者同步
     */
    void publish_producer_history();

    /**
     * @brief 消费者拉取生产者最新帧快照，同步新增未消费帧到本地消费缓存
     * 依靠原子帧ID保证时序有序、不漏帧、不重复消费
     */
    void ingest_latest_snapshot();

    /**
     * @brief 裁剪消费者本地帧缓存，限制总缓存帧数不超过kFrameBufferDepth，溢出丢弃最旧帧
     */
    void trim_consumer_frames();

    /**
     * @brief 统计消费者侧所有待发送分片包总数量
     * @return 所有缓存分片包总数
     */
    [[nodiscard]] size_t consumer_packet_count() const;

    /**
     * @brief 统计消费者侧待发送完整帧总数量
     * @return 未消费完整帧总数（包含当前正在读取的帧）
     */
    [[nodiscard]] size_t consumer_frame_count() const;

    /**
     * @brief 生产者入队后刷新生产者侧实时缓存统计
     */
    void update_stats_from_producer();

    /**
     * @brief 消费者出队/同步快照后刷新消费者侧实时缓存统计
     */
    void update_stats_from_consumer();

    // 统计变量互斥锁，所有stats_读写加锁保护，mutable允许const函数内加锁
    mutable std::mutex stats_mutex_;
    // 队列统计指标存储
    QuantaPacketQueueStats stats_{};
    // 三缓冲快照读写句柄配对
    FrameSnapshotEndpoints frame_buffer_;
    // 生产者线程本地帧缓存，入队帧先存入此处，再打包快照写入三缓冲
    std::deque<PacketFrameQueue> producer_history_;
    // 消费者线程本地缓存帧队列，存储从快照同步过来的完整帧
    std::deque<PacketFrameQueue> consumer_frames_;
    // 当前正在逐包读取的单帧分片包队列
    std::deque<QuantaPacket> current_packets_;
    // 单帧允许最大分片包数量上限
    size_t max_packets_     = kDefaultMaxPackets;
    // 当前正在读取的帧是否为关键帧标记
    bool current_keyframe_  = false;
    // 下一入队帧全局自增ID，从1开始单调递增
    uint64_t next_frame_id_ = 1;
    // 原子变量：消费者最后已消费完成的最大帧ID，acquire/release内存序同步多线程可见性
    std::atomic<uint64_t> last_ingested_frame_id_{0};
};

/**
 * @brief 将编码器输出完整EncodedPacket裸流，按MTU自动分片拆分生成QuantaPacket批量包
 * @param packet HEVC编码裸流包
 * @param seq 视频流全局序列号，同一视频流固定不变
 * @return 分片批量容器，失败携带错误字符串
 */
[[nodiscard]] std::expected<QuantaPacketBatch, std::string>
    build_quanta_packets(const EncodedPacket& packet, uint16_t seq);

} // namespace quanta