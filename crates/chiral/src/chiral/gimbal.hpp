// 头文件保护宏，防止头文件重复包含
#pragma once

// 标准时间库，获取系统时钟纳秒时间戳
#include <chrono>
// 固定宽度整数类型，uint8_t、int64_t
#include <cstdint>
// fmt基础格式化头文件
#include <fmt/base.h>
// fmt完整格式化接口
#include <fmt/format.h>
// 字符串视图，无拷贝只读字符串
#include <string_view>

// 引入上一段IPC双向通信端点定义
#include "chiral/chiral_endpoint.hpp"
// 魔法枚举库：自动枚举转字符串、反射枚举名称
#include "magic_enum.hpp"

// 云台IPC通信业务命名空间
namespace talos::chiral::gimbal {

// 静态编译期断言：bool占用1字节，保证跨进程内存布局一致
static_assert(sizeof(bool) == sizeof(uint8_t));

/**
 * @brief 机器人阵营颜色枚举，来自裁判系统串口数据
 * uint8_t 底层存储，保证共享内存内存对齐固定大小
 */
enum class Color : uint8_t {
    Red  = 0,  // 红方机器人
    Blue = 1,  // 蓝方机器人
};

/**
 * @brief MCU云台上报数据结构体
 * 云台硬件板卡 → 主控程序 上行数据
 * 存储云台实时姿态、角速度、弹速、机器人颜色、系统时间戳
 */
struct McuData {
    // 系统时钟纳秒级时间戳，epoch起始到当前的纳秒数
    int64_t timestamp_ns_system_clock;
    // 本方机器人颜色 红/蓝
    Color self_color;
    // 子弹初速度，单位m/s，默认构造默认值114514.0占位
    float bullet_speed;
    // 云台当前绝对偏航角 yaw 弧度
    float yaw;
    // 云台当前俯仰角 pitch 弧度
    float pitch;
    // 云台滚转 roll 弧度（竞赛云台一般固定0）
    float roll;
    // 云台偏航角速度 rad/s
    float yaw_vel;
    // 云台俯仰角速度 rad/s
    float pitch_vel;
    // 云台滚转角速度 rad/s
    float roll_vel;

    // 默认无参构造，成员默认随机内存值
    McuData() = default;

    /**
     * @brief 带参数构造，自动填充系统时间戳、默认弹速
     * @param self_color_ 本方阵营颜色
     * @param yaw_ 当前偏航角
     * @param pitch_ 当前俯仰角
     * @param roll_ 当前滚转角
     * @param yaw_vel_ 偏航角速度
     * @param pitch_vel_ 俯仰角速度
     * @param roll_vel_ 滚转角速度
     */
    McuData(
        Color self_color_, float yaw_, float pitch_, float roll_, float yaw_vel_, float pitch_vel_,
        float roll_vel_) noexcept
        : self_color(self_color_)
        // 子弹速度默认填充测试占位值
        , bullet_speed(114514.0)
        , yaw(yaw_)
        , pitch(pitch_)
        , roll(roll_)
        , yaw_vel(yaw_vel_)
        , pitch_vel(pitch_vel_)
        , roll_vel(roll_vel_) {
        // 获取系统当前时钟
        const auto now = std::chrono::system_clock::now();
        // 转换为自纪元起纳秒时长，转int64存入时间戳
        auto t =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        timestamp_ns_system_clock = t;
    }
};

/**
 * @brief 主控下发云台控制请求结构体
 * 主控程序 → 云台MCU板卡 下行控制指令
 * 包含目标角度、角速度/加速度约束、目标距离、有效性标记
 */
struct McuRequest {
    // 下发指令时系统纳秒时间戳
    int64_t timestamp_ns_system_clock;
    // 开火建议标志 true=允许开火，false=禁止开火
    bool fire_advice;
    // 目标偏航角 弧度
    float target_yaw;
    // 目标俯仰角 弧度
    float target_pitch;
    // 云台偏航轴参考最大角速度 rad/s
    float ref_yaw_v;
    // 云台俯仰轴参考最大角速度 rad/s
    float ref_pitch_v;
    // 云台偏航轴参考最大角加速度 rad/s²
    float ref_yaw_a;
    // 云台俯仰轴参考最大角加速度 rad/s²
    float ref_pitch_a;
    // 目标敌方距离 m
    float distance;

    /**
     * @brief 判断当前控制指令是否有效
     * @return distance>0 返回true，有效指令；距离≤0无效丢弃
     */
    bool valid() const noexcept { return distance > 0.0; }
};

} // namespace talos::chiral::gimbal

// fmt格式化库自定义格式化器命名空间
namespace fmt {

/**
 * @brief 为Color枚举特化fmt格式化器，打印时输出枚举名字符串Red/Blue
 * 继承std::string_view格式化器，复用字符串输出逻辑
 */
template <>
struct formatter<talos::chiral::gimbal::Color> : formatter<std::string_view> {
    /**
     * @brief 格式化枚举为字符串
     * @param c 颜色枚举
     * @param ctx fmt输出上下文
     * @return 格式化迭代器
     */
    auto format(talos::chiral::gimbal::Color c, format_context& ctx) const {
        // magic_enum::enum_name 将枚举转为编译期字符串视图
        return formatter<std::string_view>::format(magic_enum::enum_name(c), ctx);
    }
};

} // namespace fmt

namespace talos::chiral::gimbal {

/**
 * @brief Talos主控端IPC双向端点
 * Outgoing=McuRequest 主控下发云台指令（写）
 * Incoming=McuData 云台MCU上报姿态数据（读）
 */
using TalosEndpoint  = ipc::ChiralEndpoint<McuRequest, McuData>;

/**
 * @brief 云台MCU板卡端IPC双向端点
 * Outgoing=McuData 云台上报姿态（写）
 * Incoming=McuRequest 接收主控下发控制指令（读）
 */
using GimbalEndpoint = ipc::ChiralEndpoint<McuData, McuRequest>;

} // namespace talos::chiral::gimbal

// IPC共享内存名称特化命名空间
namespace talos::chiral::ipc {

/**
 * @brief 模板特化：McuRequest 控制指令共享内存文件名
 * 写通道：主控创建 /chiral_gimbal_request
 */
template <>
struct ShmName<gimbal::McuRequest> {
    // 共享内存POSIX名称，全局唯一
    static constexpr const char* value = "/chiral_gimbal_request";
};

/**
 * @brief 模板特化：McuData 云台上报数据共享内存文件名
 * 写通道：云台MCU创建 /chiral_gimbal_data
 */
template <>
struct ShmName<gimbal::McuData> {
    static constexpr const char* value = "/chiral_gimbal_data";
};

} // namespace talos::chiral::ipc