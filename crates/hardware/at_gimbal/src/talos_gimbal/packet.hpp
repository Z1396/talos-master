#pragma once
#include <cstdint>
#include <fmt/base.h>
#include <fmt/format.h>
#include <iomanip>
#include <iostream>
#include <string_view>

#include "magic_enum.hpp"

namespace talos_gimbal {

// 机器人自身颜色 (来自裁判系统)
enum class Color : uint8_t {
    Red  = 0,
    Blue = 1,
};

#pragma pack(1)
struct HeaderFrame {
    static constexpr uint8_t SoF() { return 0x5A; }

    static constexpr uint8_t EoF() { return 0xA5; }

    uint8_t sof; // 0x5A
    uint8_t len;
    uint8_t id;  // 0x01
};

struct ReceiveImuData {
    HeaderFrame header;
    uint32_t time_stamp;
    struct {
        Color self_color;
        float bullet_speed;
        float yaw;
        float pitch;
        float roll;
        float yaw_vel;
        float pitch_vel;
        float roll_vel;
    } data;
    uint8_t eof; // 0xA5
};

// 0x03 sent by mcu
struct ReceiveCapabilitiesData {
    HeaderFrame header;
    uint32_t time_stamp;
    struct {
        // 0: 未开启跟随 1: 开启跟随
        uint8_t following;
        // 0: 未开启开符模式 1: 开启开符模式
        uint8_t power_rune;
        // 0: 未开启 Quanta 图传 1: 开启 Quanta 图传
        uint8_t quanta;
    } data;
    uint8_t eof; // 0xA5
};

// 0x04 sent by talos
struct SendQuantaData {
    HeaderFrame header;
    uint32_t time_stamp;
    struct {
        uint16_t custom_byte_block_len;
        uint8_t custom_byte_block[298];
    } data;
    uint8_t eof;                                        // 0xA5
};

struct ReceiveSimpleImuData {
    HeaderFrame header;                                 // id = 0x03
    uint32_t time_stamp;
    struct {
        Color self_color;
        float yaw;
        float pitch;
        float roll;
    } data;
    uint8_t eof;                                        // 0xA5
};

struct SendVisionData {
    HeaderFrame header;
    struct {
        bool fire_advice;
        float target_yaw;
        float target_pitch;
        float ref_yaw_v;
        float ref_pitch_v;
        float ref_yaw_a;
        float ref_pitch_a;
        float distance;
    } data;
    uint8_t eof;
};
static_assert(sizeof(bool) == sizeof(uint8_t));

struct SendSimpleVisionData {
    HeaderFrame header;                                 // 0x04
    struct {
        float target_yaw;
    } data;
    uint8_t eof;
};

#pragma pack()

inline void print_imu(const ReceiveImuData& imu) {
    constexpr float RAD2DEG = 180.0f / 3.1415926535f;

    std::cout << std::fixed << std::setprecision(3);

    std::cout << "\r" << std::string(120, ' ') << "\r"; // 清空整行

    std::cout << "IMU | stamp: " << std::setw(8) << imu.time_stamp << " ms | "
              << "color: " << magic_enum::enum_name(imu.data.self_color) << " | "
              << "yaw: " << std::setw(7) << imu.data.yaw * RAD2DEG << "° | "
              << "pitch: " << std::setw(7) << imu.data.pitch * RAD2DEG << "° | "
              << "roll: " << std::setw(7) << imu.data.roll * RAD2DEG << "° | "
              << "ωy: " << std::setw(6) << imu.data.yaw_vel << " | "
              << "ωp: " << std::setw(6) << imu.data.pitch_vel << " | "
              << "ωr: " << std::setw(6) << imu.data.roll_vel << std::flush;
}

} // namespace talos_gimbal

// ============================================================================
// fmt::formatter specializations
// ============================================================================

namespace fmt {

template <>
struct formatter<talos_gimbal::Color> : formatter<std::string_view> {
    auto format(talos_gimbal::Color c, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(c), ctx);
    }
};

} // namespace fmt
