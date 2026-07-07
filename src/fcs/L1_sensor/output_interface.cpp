#include "L1_sensor/output_interface.hpp"
// 新talos chiral上层云台控制端点
#include "chiral/gimbal.hpp"
// 激光雷达Quanta数据包定义
#include "quanta/stream_transport.hpp"
// 共享内存IPC客户端
#include "shm_client.hpp"
// 串口/USB云台MCU底层设备句柄、通信数据包
#include "talos_gimbal/mcu_device.hpp"
#include "talos_gimbal/packet.hpp"

// 日志打印
#include <spdlog/spdlog.h>
// 内存拷贝memcpy
#include <cstring>
// std::min/max等算法
#include <algorithm>
// 标准库基础
#include <chrono>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <variant>
// fmt格式化库
#include <fmt/core.h>
#include <fmt/std.h>

namespace fcs::L1 {

namespace {
// 弧度转角度系数 180/π
constexpr double kRadToDeg = 180.0 / M_PI;
} // 匿名常量命名空间

// ============================================================================
// IpcOutput 共享内存输出实现（多进程仿真，下发指令到其他进程）
// ============================================================================

/**
 * @brief IPC共享内存输出构造
 * @param client 共享内存客户端智能指针，所有权移动
 */
IpcOutput::IpcOutput(std::shared_ptr<ipc::ShmClient> client) noexcept
    : client_(std::move(client)) {}

/**
 * @brief 下发武器瞄准指令到共享内存
 * @param cmd L5层输出武器瞄准指令（yaw/pitch/距离/开火标志）
 */
void IpcOutput::send(const L5::WeaponCommand& cmd) const noexcept {
    // [[unlikely]] 编译器分支预测提示：空客户端是极少发生的异常分支
    if (!client_) [[unlikely]] {
        SPDLOG_ERROR("IpcOutput::send: client is null, dropping weapon command");
        return;
    }

    // 弧度转角度，pitch取负：坐标系对齐
    const auto yaw_deg    = static_cast<float>(cmd.yaw * kRadToDeg);
    const auto pitch_deg  = -static_cast<float>(cmd.pitch * kRadToDeg);
    const auto distance_m = static_cast<float>(cmd.distance);

    // 调用共享内存接口发送云台指令
    client_->send_gimbal_cmd(yaw_deg, pitch_deg, distance_m, cmd.fire);
}

/**
 * @brief 激光雷达数据包IPC转发，当前无实现空操作
 */
void IpcOutput::send_quanta(const quanta::QuantaPacket& pakcet) const noexcept {
    /// No-op
}

// ============================================================================
// McuOutput 串口/USB直连云台MCU硬件输出
// 区分串口精简数据包、USB全量数据包两种协议
// ============================================================================

/**
 * @brief 静态编译期校验：通信包是平凡可拷贝类型，允许memcpy二进制发送
 * 只有trivially_copyable类型才能安全使用reinterpret_cast+memcpy序列化二进制报文
 */
static_assert(std::is_trivially_copyable_v<talos_gimbal::SendSimpleVisionData>);
static_assert(std::is_trivially_copyable_v<talos_gimbal::SendVisionData>);
static_assert(std::is_trivially_copyable_v<talos_gimbal::SendQuantaData>);

/**
 * @brief MCU云台输出构造
 * @param camera 海康采集器独占指针
 * @param info 相机配置
 * @param profile 相机硬件参数模板
 * @param exposure_time 曝光时长微秒
 * @param device_name 可选设备标识名
 */
McuOutput::McuOutput(std::shared_ptr<talos_gimbal::McuDeviceHandle> device) noexcept
    : device_(std::move(device)) {}

/**
 * @brief 下发武器瞄准指令给MCU云台
 * 串口设备发送精简单角度包；USB设备发送完整姿态+角速度+加速度包
 */
void McuOutput::send(const L5::WeaponCommand& cmd) const noexcept {
    // 异常分支标记[[unlikely]]
    if (!device_) [[unlikely]] {
        SPDLOG_ERROR("McuOutput::send: device is null, dropping weapon command");
        return;
    }
    if (!device_->is_connected()) [[unlikely]] {
        SPDLOG_ERROR("McuOutput::send: device disconnected, dropping weapon command");
        return;
    }

    // 串口模式：仅下发目标yaw简易数据包
    if (device_->is_serial()) {
        talos_gimbal::SendSimpleVisionData packet{
            .header =
                {
                         .sof = talos_gimbal::HeaderFrame::SoF(), // 帧起始标识
                         .len = sizeof(talos_gimbal::SendSimpleVisionData::data), // 数据长度
                         .id  = 0x04, // 简易视觉指令包ID
                         },
            .data =
                {
                         .target_yaw = static_cast<float>(cmd.yaw * kRadToDeg),
                         },
            .eof = talos_gimbal::HeaderFrame::EoF(), // 帧结束标识
        };
        // 同步阻塞发送二进制报文
        if (auto result =
                device_->send_sync(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
            !result) {
            SPDLOG_WARN("send simple vision data: {}", result.error());
        }
    } else {
        // USB模式：下发完整视觉指令（俯仰、角速度、加速度、距离、开火标志）
        talos_gimbal::SendVisionData packet{
            .header =
                {
                         .sof = talos_gimbal::HeaderFrame::SoF(),
                         .len = sizeof(talos_gimbal::SendVisionData::data),
                         .id  = 0x02, // 完整视觉指令包ID
                         },
            .data =
                {
                         .fire_advice  = cmd.fire, // 开火使能
                         .target_yaw   = static_cast<float>(cmd.yaw * kRadToDeg),
                         .target_pitch = -static_cast<float>(cmd.pitch * kRadToDeg),
                         .ref_yaw_v    = static_cast<float>(cmd.yaw_vel * kRadToDeg), // 偏航角速度
                         .ref_pitch_v  = -static_cast<float>(cmd.pitch_vel * kRadToDeg), // 俯仰角速度
                         .ref_yaw_a    = static_cast<float>(cmd.yaw_accel * kRadToDeg), // 偏航角加速度
                         .ref_pitch_a  = -static_cast<float>(cmd.pitch_accel * kRadToDeg), // 俯仰角加速度
                         .distance     = static_cast<float>(cmd.distance), // 目标距离
                         },
            .eof = talos_gimbal::HeaderFrame::EoF(),
        };
        if (auto result =
                device_->send_sync(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
            !result) {
            SPDLOG_WARN("send vision data: {}", result.error());
        }
    }
}

/**
 * @brief 转发激光雷达Quanta自定义数据包到MCU（仅USB设备支持）
 * 利用包头ID高4bit存储扩展数据长度，hack实现变长自定义字节块
 */
void McuOutput::send_quanta(const quanta::QuantaPacket& packet) const noexcept {
    if (!device_) [[unlikely]] {
        return;
    }
    if (!device_->is_connected()) [[unlikely]] {
        return;
    }
    // 串口不支持扩展自定义数据包，直接返回
    if (device_->is_serial()) {
        return;
    }
    // 扩展数据长度hack逻辑：原始data长度拆分为高低4bit存入包ID
    constexpr auto data_length_raw = sizeof(talos_gimbal::SendQuantaData::data);
    static_assert(data_length_raw < 4096); // 总长度不超过12bit（0~4095）
    constexpr uint8_t data_length_hi = data_length_raw >> 8; // 高4bit
    static_assert((data_length_hi & 0b1111) == data_length_hi); // 校验仅占用4bit
    constexpr uint8_t data_length_protocol = data_length_raw & 0b11111111; // 低8bit
    constexpr uint8_t data_id_protocol = (data_length_hi << 4) | 0x04; // 拼接ID

    // 自定义字节块最大长度
    constexpr std::size_t kMaxCustomBlockBytes =
        sizeof(std::declval<talos_gimbal::SendQuantaData>().data.custom_byte_block);

    const auto bytes = packet.bytes();
    // 空包/超长包直接丢弃
    if (bytes.empty() || bytes.size() > kMaxCustomBlockBytes) {
        return;
    }

    // 静态自增序列号
    static uint32_t s = 0;
    talos_gimbal::SendQuantaData out{
        .header =
            {
                     .sof = talos_gimbal::HeaderFrame::SoF(),
                     .len = data_length_protocol,
                     .id  = data_id_protocol,
                     },
        .time_stamp = static_cast<uint32_t>(s++),
        .data =
            {
                     .custom_byte_block_len = static_cast<uint16_t>(bytes.size()),
                     .custom_byte_block     = {},
                     },
        .eof = talos_gimbal::HeaderFrame::EoF(),
    };

    // 拷贝雷达二进制载荷到报文
    std::memcpy(out.data.custom_byte_block, bytes.data(), bytes.size());
    SPDLOG_DEBUG("send_quanta: bytes.size() = {}, seq = {}", bytes.size(), packet.seq);
    // 同步发送
    device_->send_sync(reinterpret_cast<uint8_t*>(&out), sizeof(out));
}

// ============================================================================
// ChiralOutput 新一代Chiral协议云台输出
// 全新序列化结构体，统一完整姿态+角速度+加速度下发
// ============================================================================

/**
 * @brief Chiral云台输出构造
 * @param device 新一代Chiral云台通信端点共享指针
 */
ChiralOutput::ChiralOutput(std::shared_ptr<talos::chiral::gimbal::TalosEndpoint> device) noexcept
    : device_(std::move(device)) {}

/**
 * @brief 下发武器瞄准完整指令到Chiral云台
 */
void ChiralOutput::send(const L5::WeaponCommand& cmd) const noexcept {
    if (!device_) [[unlikely]] {
        SPDLOG_ERROR("ChiralOutput::send: device is null, dropping weapon command");
        return;
    }

    // 封装Chiral协议请求包，统一坐标系转换
    device_->write(
        talos::chiral::gimbal::McuRequest{
            .timestamp_ns_system_clock = static_cast<int64_t>(cmd.timestamp_ns),
            .fire_advice               = cmd.fire,
            .target_yaw                = static_cast<float>(cmd.yaw * kRadToDeg),
            .target_pitch              = -static_cast<float>(cmd.pitch * kRadToDeg),
            .ref_yaw_v                 = static_cast<float>(cmd.yaw_vel * kRadToDeg),
            .ref_pitch_v               = -static_cast<float>(cmd.pitch_vel * kRadToDeg),
            .ref_yaw_a                 = static_cast<float>(cmd.yaw_accel * kRadToDeg),
            .ref_pitch_a               = -static_cast<float>(cmd.pitch_accel * kRadToDeg),
            .distance                  = static_cast<float>(cmd.distance),
        });
}

/**
 * @brief Chiral协议暂不支持激光雷达数据包转发，空实现
 */
void ChiralOutput::send_quanta(const quanta::QuantaPacket& packet) const noexcept {
    (void)packet;
    /// No-op
}

// ============================================================================
// OutputInterface 顶层统一输出接口（策略模式，std::variant多态）
// 上层业务仅依赖此类，底层自动切换IPC/Mcu/Chiral三种输出
// ============================================================================

/**
 * @brief 顶层输出接口构造，传入任意一种输出实现变体
 */
OutputInterface::OutputInterface(OutputMode mode) noexcept
    : mode_(std::move(mode)) {}

/**
 * @brief 统一下发武器瞄准指令，std::visit自动分发到底层实现
 * std::visit 遍历variant内部存储的输出类型，调用对应send()
 */
void OutputInterface::send(const L5::WeaponCommand& cmd) noexcept {
    std::visit([&cmd](auto& output) { output.send(cmd); }, mode_);
}

// ============================================================================
// 静态工厂方法：统一创建各类输出顶层接口
// ============================================================================

/**
 * @brief 创建IPC共享内存输出顶层接口
 * @param client 共享内存客户端
 * @return 封装IpcOutput的OutputInterface，无错误直接返回
 */
auto OutputInterface::create_ipc(std::shared_ptr<ipc::ShmClient> client) noexcept
    -> std::expected<OutputInterface, std::string> {
    return OutputInterface(IpcOutput(std::move(client)));
}

/**
 * @brief 创建Chiral新一代云台输出顶层接口
 * @param client Chiral云台端点共享指针
 * @return OutputInterface封装ChiralOutput
 */
auto OutputInterface::create_chiral(
    std::shared_ptr<talos::chiral::gimbal::TalosEndpoint> client) noexcept
    -> std::expected<OutputInterface, std::string> {
    return OutputInterface(ChiralOutput(std::move(client)));
}

/**
 * @brief 创建USB直连MCU云台输出顶层接口
 * @param vendor_id USB厂商ID
 * @param product_id 可选产品ID过滤
 * @return 成功返回McuOutput封装接口；失败返回设备打开错误字符串
 */
auto OutputInterface::create_usb(
    const uint16_t vendor_id, const std::optional<uint16_t> product_id) noexcept
    -> std::expected<OutputInterface, std::string> {
    auto device_result = talos_gimbal::McuDeviceHandle::create_usb(vendor_id, product_id);
    if (!device_result) {
        return std::unexpected(std::move(device_result).error());
    }
    return OutputInterface(McuOutput(
        std::make_shared<talos_gimbal::McuDeviceHandle>(std::move(device_result).value())));
}

/**
 * @brief 创建串口RS485 MCU云台输出顶层接口
 * @param device_path 串口设备路径 /dev/ttyUSBx
 * @param baud_rate 波特率
 * @return 成功返回McuOutput封装接口；失败返回串口打开错误
 */
auto OutputInterface::create_serial(const std::string& device_path, int baud_rate) noexcept
    -> std::expected<OutputInterface, std::string> {
    auto device_result = talos_gimbal::McuDeviceHandle::create_serial(device_path, baud_rate);
    if (!device_result) {
        return std::unexpected(std::move(device_result).error());
    }
    return OutputInterface(McuOutput(
        std::make_shared<talos_gimbal::McuDeviceHandle>(std::move(device_result).value())));
}

} // namespace fcs::L1