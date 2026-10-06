#pragma once
// 头文件保护，防止重复包含导致重定义

// ============================================================================
// 跨机网络 IPC 线协议定义（Windows Rust 仿真 ↔ Linux NUC 上的 talos_ipc_bridge）
//
// 背景：共享内存只能在同一台机器内互通；仿真跑在 Windows、Talos 跑在 Linux 小电脑
// 时，用"一根网线局域网 + 网络包"替代 mmap 文件，由 talos_ipc_bridge 做落地：
//   收网络包 → 写入本机 /tmp 共享内存（三缓冲） → Talos 主程序零改动照常消费
//
// 传输分工：
//   UDP（桥监听 control_port）：小数据包
//     位姿×5、底盘观测、真值批量、相机内参、运行状态、心跳
//   TCP（桥监听 image_port）：JPEG 图像流（长度前缀分帧，一帧一包）
//   UDP（桥→仿真源地址）：云台指令回传
//
// 结构体载荷直接复用 shm_layout.hpp 的内存布局（双方均 x86/ARM 小端，
// 直接 memcpy 结构体字节，不做网络字节序转换）；
// 所有 timestamp_ns 由桥接收时改写为 NUC 本机时钟，统一时域（替代 NTP 对时）
// ============================================================================

// 定长整型
#include <cstdint>
// offsetof 偏移校验
#include <cstddef>

namespace ipc::net {

// 协议魔数：区别于共享内存的 SHM_MAGIC(0x54414C05)
static constexpr uint32_t NET_MAGIC = 0x54414C06; // ASCII "TAL06"
// 协议版本：结构布局变更时递增，接收端校验不匹配则丢弃
static constexpr uint16_t NET_VERSION = 1;

// 桥默认监听端口（可在启动参数覆盖）
static constexpr uint16_t DEFAULT_CONTROL_PORT = 9571; // UDP 控制口
static constexpr uint16_t DEFAULT_IMAGE_PORT   = 9572; // TCP 图像口

// UDP 单包载荷上限（真值批量 1664B 为最大合法载荷，预留余量）
static constexpr size_t NET_MAX_PAYLOAD = 2048;

// ====================== 包类型枚举 ======================
enum class PacketType : uint8_t {
    Pose          = 1, // 位姿（payload=PoseMeta 64B，header.index 区分5路通道）
    ChassisObserv = 2, // 底盘观测（payload=ChassisObservation 128B）
    GroundTruth   = 3, // 仿真真值批量（payload=GroundTruthBatch 1664B）
    CameraInfo    = 4, // 相机内参（payload=CameraInfo 128B）
    RuntimeState  = 5, // 自瞄运行状态（payload=RuntimeState 64B）
    Heartbeat     = 6, // 心跳（payload=0B，仅保活/记录仿真地址）
    GimbalCmd     = 7, // 云台指令回传（payload=GimbalCmd 32B，桥→仿真方向）
    ImageJpeg     = 8, // JPEG 图像（仅 TCP 通道，帧头见 ImageFrameHeader）
};

// ====================== UDP 控制包头 ======================
/**
 * @brief UDP 包头，32字节，紧跟 payload（结构体字节，不做额外封装）
 * 字段布局手工填充对齐，无隐式 padding，C++/Rust 逐字段写入保持一致
 */
struct PacketHeader {
    uint32_t magic;        // 偏移0  ：NET_MAGIC 校验
    uint16_t version;      // 偏移4  ：NET_VERSION
    uint16_t payload_len;  // 偏移6  ：载荷字节数
    uint8_t  type;         // 偏移8  ：PacketType
    uint8_t  index;        // 偏移9  ：位姿通道下标（Pose 包用，其他填0）
    uint16_t _pad;         // 偏移10 ：对齐填充
    uint32_t _pad2;        // 偏移12 ：对齐填充
    uint64_t seq;          // 偏移16 ：发送序号，单调递增（诊断丢包用）
    uint64_t timestamp_ns; // 偏移24 ：发送方时间戳（桥会改写为 NUC 时钟）
};
static_assert(sizeof(PacketHeader) == 32, "PacketHeader must be 32 bytes");
static_assert(offsetof(PacketHeader, seq) == 16, "PacketHeader.seq offset mismatch");
static_assert(offsetof(PacketHeader, timestamp_ns) == 24, "PacketHeader.timestamp_ns offset mismatch");

// ====================== TCP 图像帧头 ======================
/**
 * @brief TCP 图像流帧头，40字节，后跟 jpeg_len 字节 JPEG 压缩数据
 * 一帧一个包：先发帧头，紧接着发 JPEG 字节，不夹杂其他数据
 */
struct ImageFrameHeader {
    uint32_t magic;        // 偏移0  ：NET_MAGIC 校验
    uint8_t  type;         // 偏移4  ：PacketType::ImageJpeg
    uint8_t  _pad[3];      // 偏移5  ：对齐填充
    uint32_t width;        // 偏移8  ：原始图像宽（解码前校验用）
    uint32_t height;       // 偏移12 ：原始图像高
    uint32_t jpeg_len;     // 偏移16 ：JPEG 数据字节数（1~8MB 合法）
    uint32_t _pad2;        // 偏移20 ：对齐填充
    uint64_t seq;          // 偏移24 ：帧序号（与位姿 frame_seq 对齐）
    uint64_t timestamp_ns; // 偏移32 ：帧时间戳（桥会改写为 NUC 时钟）
};
static_assert(sizeof(ImageFrameHeader) == 40, "ImageFrameHeader must be 40 bytes");
static_assert(offsetof(ImageFrameHeader, seq) == 24, "ImageFrameHeader.seq offset mismatch");
static_assert(offsetof(ImageFrameHeader, timestamp_ns) == 32, "ImageFrameHeader.timestamp_ns offset mismatch");

// JPEG 载荷合法性边界：小于1B或超过8MB视为恶意/损坏包直接断连
static constexpr uint32_t JPEG_MIN_LEN = 1;
static constexpr uint32_t JPEG_MAX_LEN = 8 * 1024 * 1024;

} // namespace ipc::net
