// 云台手性通信、坐标系相关数据结构定义
#include "chiral/gimbal.hpp"
// spdlog日志系统全局初始化、标准IO流劫持钩子
#include "spdlog_hook.hpp"
// MCU设备USB通信驱动：USB设备创建、同步数据包发送、通信句柄封装
#include "talos_gimbal/mcu_device.hpp"

// 原子变量：无锁多线程/信号处理全局标记
#include <atomic>
// C++高精度时钟、时长类型
#include <chrono>
// POSIX系统信号捕获：Ctrl+C、进程终止信号
#include <csignal>
// 线程休眠接口
#include <thread>

// 时间字面量语法糖：100ms、1s 快速书写
using namespace std::chrono_literals;

// ===================== 全局程序运行标记 =====================
// std::atomic 保证信号处理函数、主线程无锁安全读写
// true = 程序持续循环发包；false = 收到终止信号，退出主循环
std::atomic<bool> g_running{true};

/**
 * @brief 全局信号捕获回调函数
 * @param sig 触发的系统信号编号
 * 捕获两种信号：
 * SIGINT ：终端按下 Ctrl+C
 * SIGTERM：系统/kill命令正常终止进程
 * 功能：将全局运行标记置false，优雅退出发包循环，不会暴力中断USB通信
 */
void signal_handler(int) {
    g_running = false;
}

/**
 * @brief Talos USB MCU 数据流测试工具主程序
 * 业务用途：
 * 1. 建立USB与下位机MCU的同步通信通道
 * 2. 固定100ms间隔（约30FPS）循环发送自定义二进制测试数据包
 * 3. 数据包携带固定标识字符串、帧计数时间戳，用于Foxglove Quanta高速流链路调试
 * 4. 支持Ctrl+C优雅退出，退出时发送一帧空包收尾，打印总发送帧数、运行耗时
 * 协议细节：自定义帧头SoF、帧尾EoF，12bit长度编码复用包ID高4bit
 */
int main() {
    // ===================== 1. 注册系统信号捕获处理器 =====================
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 初始化全局spdlog日志（统一格式、输出渠道）
    init_logger();
    SPDLOG_INFO("=== Talos Playground ===");
    SPDLOG_INFO("Press Ctrl+C to quit");
    // 记录程序启动时刻，用于结束时计算总运行毫秒数
    const auto start = std::chrono::system_clock::now();

    SPDLOG_INFO("creating shared memory regions...");
    // 创建USB通信设备句柄：VID=0x0483 STM设备厂商ID，无额外配置参数
    // create_usb 返回 std::expected<McuDeviceHandle, std::string>
    auto client = talos_gimbal::McuDeviceHandle::create_usb(0x0483, std::nullopt);
    // USB设备打开失败（无设备、权限不足、占用），打印致命日志并退出
    if (!client) {
        SPDLOG_CRITICAL("{}", client.error());
        return 1;
    }

    // 全局帧计数器：记录一共发送了多少轮完整数据包
    uint64_t frame_count = 0;
    // 298字节固定测试二进制缓冲区，预置固定标识字符串
    std::array<uint8_t, 298> bytes{};

    // 填充头部标识 "TALOS!"
    bytes[0] = 'T';
    bytes[1] = 'A';
    bytes[2] = 'L';
    bytes[3] = 'O';
    bytes[4] = 'S';
    bytes[5] = '!';

    // 填充标识 "PraySky1337"
    bytes[7]  = 'P';
    bytes[8]  = 'r';
    bytes[9]  = 'a';
    bytes[10] = 'y';
    bytes[11] = 'S';
    bytes[12] = 'k';
    bytes[13] = 'y';
    bytes[14] = '1';
    bytes[15] = '3';
    bytes[16] = '3';
    bytes[17] = '7';

    // 填充标识 "RoboMaster"
    bytes[51] = 'R';
    bytes[52] = 'o';
    bytes[53] = 'b';
    bytes[54] = 'o';
    bytes[55] = 'M';
    bytes[56] = 'a';
    bytes[57] = 's';
    bytes[58] = 't';
    bytes[59] = 'e';
    bytes[60] = 'r';

    // 填充标识 "ACTOR"
    bytes[184] = 'A';
    bytes[185] = 'C';
    bytes[186] = 'T';
    bytes[187] = 'O';
    bytes[188] = 'R';

    // 填充标识 "Thinker"
    bytes[190] = 'T';
    bytes[191] = 'h';
    bytes[192] = 'i';
    bytes[193] = 'n';
    bytes[194] = 'k';
    bytes[195] = 'e';
    bytes[196] = 'r';

    // 填充尾部标识 "ACK"
    bytes[295] = 'A';
    bytes[296] = 'C';
    bytes[297] = 'K';

    SPDLOG_INFO("starting maeikn loop...");

    // ===================== 协议长度编码 hack 说明 =====================
    // 协议限制：长度仅分配12bit存储空间，分包ID占用低4bit，长度高4bit复用ID高4位
    // 拆分规则：len低8bit单独存放，高4bit塞进ID的高4位
    constexpr auto data_length_raw = sizeof(talos_gimbal::SendQuantaData::data);
    // 静态断言：总数据长度必须小于4096（12bit最大值）
    static_assert(data_length_raw < 4096);
    // 取出长度高4bit
    constexpr uint8_t data_length_hi = data_length_raw >> 8;
    // 校验高4bit不溢出（仅允许4bit 0~15）
    static_assert((data_length_hi & 0b1111) == data_length_hi);
    // 长度低8bit，放入header.len字段
    constexpr uint8_t data_length_protocol = data_length_raw & 0b11111111;
    // 组合ID：高4bit=长度高位，低4bit=固定通道ID 0x04
    constexpr uint8_t data_id_protocol = (data_length_hi << 4) | 0x04;

    // 单包最大自定义数据块字节数
    constexpr std::size_t kMaxCustomBlockBytes = 298;

    // ===================== 主发包循环 =====================
    while (g_running) {
        frame_count++; // 每轮循环帧计数自增

        const auto* ptr = bytes.data();
        std::size_t remaining = bytes.size();
        std::size_t offset    = 0;

        // 分片循环（此处缓冲区刚好298，只会执行一次分片）
        while (remaining > 0) {
            const std::size_t this_len = std::min(remaining, kMaxCustomBlockBytes);

            // 构造完整Quanta高速流数据包
            talos_gimbal::SendQuantaData packet{
                .header =
                    {
                             .sof = talos_gimbal::HeaderFrame::SoF(), // 帧起始标记
                             .len = data_length_protocol,            // 长度低8bit
                             .id  = data_id_protocol,                // 组合后的通道ID
                             },
                .time_stamp = static_cast<uint32_t>(frame_count), // 帧计数时间戳
                .data =
                    {
                             .custom_byte_block_len = 298, // 自定义数据块长度
                             .custom_byte_block     = {},  // 自定义二进制缓冲区
                             },
                .eof = talos_gimbal::HeaderFrame::EoF(), // 帧结束标记
            };

            // 拷贝测试二进制数据进包内自定义缓冲区
            std::memcpy(packet.data.custom_byte_block, ptr + offset, this_len);
            // 缓冲区19号位置写入当前帧计数，用于接收端校验帧连续性
            packet.data.custom_byte_block[19] = frame_count;

            // 同步阻塞发送USB数据包
            if (auto result =
                    client->send_sync(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
                !result) {
                // 发送失败打印警告（USB断开、下位机无响应等）
                SPDLOG_WARN("send: {}", result.error());
            }

            offset += this_len;
            remaining -= this_len;
        }
        // 休眠100ms，控制发包帧率≈30FPS
        std::this_thread::sleep_for(100ms);
    }

    // ===================== 收到退出信号后，发送收尾空包 =====================
    talos_gimbal::SendQuantaData packet{
        .header =
            {
                     .sof = talos_gimbal::HeaderFrame::SoF(),
                     .len = data_length_protocol,
                     .id  = data_id_protocol,
                     },
        .time_stamp = static_cast<uint32_t>(frame_count),
        .data =
            {
                     .custom_byte_block_len = static_cast<uint16_t>(300),
                     .custom_byte_block     = {},
            },
        .eof = talos_gimbal::HeaderFrame::EoF(),
    };
    // 清空缓冲区为全0，作为结束标记包
    std::memset(packet.data.custom_byte_block, 0, 300);

    // 发送收尾空包，通知下位机/可视化工具数据流结束
    if (auto result = client->send_sync(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
        !result) {
        SPDLOG_WARN("send: {}", result.error());
    }

    // ===================== 程序退出清理与统计打印 ===========
    SPDLOG_INFO(
        "shutting down... ({} frames, {}ms)", frame_count,
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now() - start)
            .count());
    return 0;
}// namespace talos_gimbal