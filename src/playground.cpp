#include "chiral/gimbal.hpp"
#include "spdlog_hook.hpp"
#include "talos_gimbal/mcu_device.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

using namespace std::chrono_literals;

// Global signal handler
std::atomic<bool> g_running{true};

void signal_handler(int) { g_running = false; }

int main() {
    // Setup signal handler
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    init_logger();
    SPDLOG_INFO("=== Talos Playground ===");
    SPDLOG_INFO("Press Ctrl+C to quit");
    const auto start = std::chrono::system_clock::now();

    SPDLOG_INFO("creating shared memory regions...");
    auto client = talos_gimbal::McuDeviceHandle::create_usb(0x0483, std::nullopt);
    if (!client) {
        SPDLOG_CRITICAL("{}", client.error());
        return 1;
    }

    uint64_t frame_count = 0;
    std::array<uint8_t, 298> bytes{};

    bytes[0] = 'T';
    bytes[1] = 'A';
    bytes[2] = 'L';
    bytes[3] = 'O';
    bytes[4] = 'S';
    bytes[5] = '!';

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

    bytes[184] = 'A';
    bytes[185] = 'C';
    bytes[186] = 'T';
    bytes[187] = 'O';
    bytes[188] = 'R';

    bytes[190] = 'T';
    bytes[191] = 'h';
    bytes[192] = 'i';
    bytes[193] = 'n';
    bytes[194] = 'k';
    bytes[195] = 'e';
    bytes[196] = 'r';

    bytes[295] = 'A';
    bytes[296] = 'C';
    bytes[297] = 'K';

    // Main loop
    SPDLOG_INFO("starting maeikn loop...");

    // evil hack for extended data length
    constexpr auto data_length_raw = sizeof(talos_gimbal::SendQuantaData::data);
    // maximum extended data length is stored as u12
    static_assert(data_length_raw < 4096);
    // lower bits
    // higher bits, 4bits available
    constexpr uint8_t data_length_hi = data_length_raw >> 8;
    // double check for only 4bits
    static_assert((data_length_hi & 0b1111) == data_length_hi);
    constexpr uint8_t data_length_protocol = data_length_raw & 0b11111111;
    // actual id is 0x04
    constexpr uint8_t data_id_protocol = (data_length_hi << 4) | 0x04;

    constexpr std::size_t kMaxCustomBlockBytes = 298;

    while (g_running) {
        // 测试 2: 接收指令
        frame_count++;

        const auto* ptr = bytes.data();

        std::size_t remaining = bytes.size();
        std::size_t offset    = 0;

        while (remaining > 0) {
            const std::size_t this_len = std::min(remaining, kMaxCustomBlockBytes);

            talos_gimbal::SendQuantaData packet{
                .header =
                    {
                             .sof = talos_gimbal::HeaderFrame::SoF(),
                             .len = data_length_protocol,
                             .id  = data_id_protocol,
                             },
                // ffs
                .time_stamp = static_cast<uint32_t>(frame_count),
                .data =
                    {
                             .custom_byte_block_len = 298,
                             .custom_byte_block     = {},
                             },
                .eof = talos_gimbal::HeaderFrame::EoF(),
            };

            std::memcpy(packet.data.custom_byte_block, ptr + offset, this_len);
            packet.data.custom_byte_block[19] = frame_count;
            if (auto result =
                    client->send_sync(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
                !result) {
                SPDLOG_WARN("send: {}", result.error());
            }

            offset += this_len;
            remaining -= this_len;
        }
        std::this_thread::sleep_for(100ms); // ~30 FPS
    }

    talos_gimbal::SendQuantaData packet{
        .header =
            {
                     .sof = talos_gimbal::HeaderFrame::SoF(),
                     .len = data_length_protocol,
                     .id  = data_id_protocol,
                     },
        // ffs
        .time_stamp = static_cast<uint32_t>(frame_count),
        .data =
            {
                     .custom_byte_block_len = static_cast<uint16_t>(300),
                     .custom_byte_block     = {},
                     },
        .eof = talos_gimbal::HeaderFrame::EoF(),
    };
    std::memset(packet.data.custom_byte_block, 0, 300);

    if (auto result = client->send_sync(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
        !result) {
        SPDLOG_WARN("send: {}", result.error());
    }

    // Cleanup
    SPDLOG_INFO(
        "shutting down... ({} frames, {}ms)", frame_count,
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now() - start)
            .count());
    return 0;
}
