#pragma once
// 头文件保护，防止同一头文件被多次重复包含，替代传统 #ifndef / #define / #endif 写法

#include <cstdint>
// C++标准固定宽度整数头文件，提供 uint8_t / uint16_t / uint32_t 等平台无关整型
#include <fmt/base.h>
#include <fmt/format.h>
// fmt 第三方格式化库，替代难用的 std::cout / printf，高性能、类型安全格式化输出
#include <iomanip>
// C++标准IO操纵器，提供 setprecision / setw / fixed 等控制台格式化工具
#include <iostream>
// C++标准控制台输入输出流 std::cout / std::cin / std::flush
#include <string_view>
// C++17 只读字符串视图，不持有内存，轻量高效，无拷贝开销

#include "magic_enum.hpp"
// 第三方反射库：无宏、无运行时开销的枚举字符串反射，可直接把枚举值转对应名字符串

namespace talos_gimbal {
// 命名空间：云台通信协议专用命名空间，隔离全局命名冲突，所有协议结构体、枚举都放在此处

// 机器人自身颜色 (来自裁判系统)
// 对应裁判系统给出的己方队伍颜色标识
enum class Color : uint8_t {
    Red  = 0,   // 红方机器人，枚举底层存储为1字节无符号整型
    Blue = 1,   // 蓝方机器人
};

#pragma pack(1)
// 内存对齐指令：强制下面所有结构体按1字节紧凑对齐，**无内存填充Padding**
// 串口/网口二进制通信必须开启，保证结构体内存布局和协议帧格式完全一致
struct HeaderFrame {
    // 帧头静态常量函数，返回帧起始标识字节 0x5A
    static constexpr uint8_t SoF() { return 0x5A; }
    // 帧尾静态常量函数，返回帧结束标识字节 0xA5
    static constexpr uint8_t EoF() { return 0xA5; }

    uint8_t sof;    // 帧起始符，固定 0x5A，用于解析时识别一帧数据开始
    uint8_t len;    // 整帧数据长度字段，存储当前帧总字节长度
    uint8_t id;     // 帧功能ID，区分不同业务数据包（IMU/视觉/能力帧等）
};

// 【帧ID 0x01 接收IMU全量数据帧】
// MCU向上位机/视觉发送完整陀螺仪、姿态、弹速、队伍颜色数据
struct ReceiveImuData {
    HeaderFrame header;     // 统一协议帧头，所有数据包共用头部结构
    uint32_t time_stamp;    // MCU时间戳，单位ms，4字节无符号整型，记录数据采集时刻
    struct {
        Color self_color;  // 己方队伍颜色，枚举类型Red/Blue
        float bullet_speed; // 裁判系统下发的弹丸射速，单位m/s
        float yaw;          // 云台偏航角（水平旋转），单位弧度rad
        float pitch;        // 云台俯仰角（上下抬头），单位弧度rad
        float roll;         // 云台横滚角（机身侧翻），单位弧度rad
        float yaw_vel;     // 偏航角速度，rad/s
        float pitch_vel;   // 俯仰角速度，rad/s
        float roll_vel;    // 横滚角速度，rad/s
    } data;                 // 业务数据载荷段，封装所有业务变量
    uint8_t eof; // 帧结束符，固定0xA5，解析时校验帧完整性
};

// 【帧ID 0x03 MCU向上位机发送机器人功能开关状态帧】
// 下发各类功能使能标志：跟随、打符、图传开关
struct ReceiveCapabilitiesData {
    HeaderFrame header;
    uint32_t time_stamp;    // MCU毫秒时间戳
    struct {
        // 0: 未开启云台跟随模式 1: 开启云台跟随自瞄
        uint8_t following;
        // 0: 未开启能量机关打符模式 1: 开启自动打符逻辑
        uint8_t power_rune;
        // 0: 未开启Quanta高速图传 1: 开启高速自定义图传通道
        uint8_t quanta;
    } data;
    uint8_t eof; // 帧尾0xA5
};

// 【帧ID 0x04 上位机向MCU发送Quanta自定义图传数据包】
// 视觉端下发大段自定义二进制字节流，用于高速图传、图像特征传输
struct SendQuantaData {
    HeaderFrame header;
    uint32_t time_stamp;
    struct {
        uint16_t custom_byte_block_len; // 自定义字节块有效长度，2字节
        uint8_t custom_byte_block[298]; // 自定义二进制缓冲区，固定298字节容量
    } data;
    uint8_t eof; // 帧结束标志
};

// 简化版IMU姿态帧，精简数据量，高频低负载传输（帧ID 0x03）
struct ReceiveSimpleImuData {
    HeaderFrame header;                                 // id = 0x03
    uint32_t time_stamp;
    struct {
        Color self_color; // 己方颜色
        float yaw;        // 偏航角 rad
        float pitch;      // 俯仰角 rad
        float roll;       // 横滚角 rad
    } data;
    uint8_t eof;                                        // 0xA5
};

// 视觉发给MCU的完整目标预测帧
// 输出瞄准目标角度、角速度/加速度前馈、距离、开火使能建议
struct SendVisionData {
    HeaderFrame header;
    struct {
        bool fire_advice;      // 开火建议标志：true允许开火，false禁止开火
        float target_yaw;      // 目标预测偏航角 rad
        float target_pitch;    // 目标预测俯仰角 rad
        float ref_yaw_v;      // 偏航角速度前馈
        float ref_pitch_v;    // 俯仰角速度前馈
        float ref_yaw_a;      // 偏航角加速度前馈
        float ref_pitch_a;    // 俯仰角加速度前馈
        float distance;       // 目标距离 m
    } data;
    uint8_t eof;
};
// 静态断言：强制bool占1字节，保证串口协议内存对齐不出现长度错乱
static_assert(sizeof(bool) == sizeof(uint8_t));

// 极简视觉输出帧，仅下发目标偏航角，超低带宽高频发送（帧ID 0x04）
struct SendSimpleVisionData {
    HeaderFrame header;                                 // 0x04
    struct {
        float target_yaw; // 仅输出目标水平偏航角
    } data;
    uint8_t eof;
};

#pragma pack()
// 取消1字节紧凑对齐，恢复编译器默认内存对齐规则，避免后续无关结构体异常填充

/**
 * @brief 打印完整IMU姿态数据到控制台，用于调试可视化
 * @param imu 接收的完整IMU数据包引用
 */
inline void print_imu(const ReceiveImuData& imu) {
    // 弧度转角度常量：180 / π
    constexpr float RAD2DEG = 180.0f / 3.1415926535f;

    // 设置浮点数固定小数格式，保留3位小数输出
    std::cout << std::fixed << std::setprecision(3);

    // \r 回车回到行首，填充120个空格清空整行，实现单行刷新不滚屏
    std::cout << "\r" << std::string(120, ' ') << "\r";

    // 格式化打印全部IMU信息：时间戳、队伍颜色、三轴角度、三轴角速度
    // setw(n) 设置输出占位宽度，对齐控制台文本
    std::cout << "IMU | stamp: " << std::setw(8) << imu.time_stamp << " ms | "
              << "color: " << magic_enum::enum_name(imu.data.self_color) << " | "
              << "yaw: " << std::setw(7) << imu.data.yaw * RAD2DEG << "° | "
              << "pitch: " << std::setw(7) << imu.data.pitch * RAD2DEG << "° | "
              << "roll: " << std::setw(7) << imu.data.roll * RAD2DEG << "° | "
              << "ωy: " << std::setw(6) << imu.data.yaw_vel << " | "
              << "ωp: " << std::setw(6) << imu.data.pitch_vel << " | "
              << "ωr: " << std::setw(6) << imu.data.roll_vel << std::flush;
    // std::flush 强制刷新IO缓冲区，立刻打印到终端，不缓存延迟
}

} // namespace talos_gimbal

// ============================================================================
// fmt 库格式化特化：让 fmt::print 直接打印 Color 枚举名称，无需手动转换
// ============================================================================
namespace fmt {

// 模板特化：为 talos_gimbal::Color 实现 fmt 格式化器
template <>
struct formatter<talos_gimbal::Color> : formatter<std::string_view> {
    /**
     * @brief 实现枚举格式化逻辑
     * @param c 队伍颜色枚举
     * @param ctx fmt格式化上下文
     * @return 格式化迭代器
     */
    auto format(talos_gimbal::Color c, format_context& ctx) const {
        // magic_enum 获取枚举对应字符串名称，复用string_view格式化器输出
        return formatter<std::string_view>::format(magic_enum::enum_name(c), ctx);
    }
};

} // namespace fmt