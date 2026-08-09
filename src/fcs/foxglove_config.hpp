// 头文件保护，避免重复包含造成编译错误
#pragma once

// 引入数据流编码参数配置，用于Foxglove消息编码
#include "quanta/stream_encoder.hpp"

// 标准定长整型，用于端口、枚举底层类型
#include <cstdint>
// 标准字符串，存放地址、文件路径
#include <string>

// 项目顶层命名空间
namespace fcs {

/**
 * @brief Foxglove 数据传输方式枚举
 * 底层使用 uint8_t 节约内存，限定取值范围
 */
enum class FoxgloveTransport : uint8_t {
    WebSocket,  // 实时 WebSocket 传输：在线可视化，Foxglove Studio 实时连接查看数据
    Mcap        // 本地 MCAP 文件录制：将数据存为mcap格式离线文件，后续回放分析
};

/**
 * @brief Foxglove 可视化服务全局配置结构体
 * 控制可视化服务启停、通信方式、监听地址、端口、文件路径、消息编码等参数
 */
struct FoxgloveConfig {
    bool enabled{true};                // 是否启用Foxglove可视化服务，默认开启
    FoxgloveTransport transport{FoxgloveTransport::WebSocket}; // 默认使用WebSocket实时传输
    std::string host{"0.0.0.0"};       // 监听地址，0.0.0.0 表示监听本机所有网卡
    uint16_t port{8765};                // 服务监听端口，默认8765
    std::string mcap_path{};           // MCAP文件保存路径，仅传输模式为Mcap时生效
    quanta::EncodeParams quanta{};     // 可视化消息流的编码配置（压缩、格式等）
    /*{} 是列表初始化（值初始化）。
    创建一个 quanta 类型的临时对象，所有成员自动零初始化。*/
};

} // namespace fcs