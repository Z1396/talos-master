// 头文件保护宏，防止头文件被多次重复包含
#pragma once

// 串口底层实现、STM32通信协议解析器
#include "serial.hpp"
#include "stm32.hpp"

// 智能指针、多类型变体容器
#include <memory>
#include <variant>

// 云台MCU通信顶层命名空间
namespace talos_gimbal {

// 类型别名：USB 连接STM32设备实现类
using UsbDevice    = Stm32Impl<Stm32Parser>;
// 类型别名：串口(/dev/ttyUSBx)连接STM32设备实现类
using SerialDevice = SerialImpl<Stm32Parser>;

/**
 * @brief MCU设备统一句柄封装
 * 对外统一接口，屏蔽底层是USB直连还是串口TTL连接STM32云台主控
 * 基于 std::variant 实现多态，无需虚函数、无运行时多态开销
 */
class McuDeviceHandle {
public:
    // 禁用拷贝构造、拷贝赋值：设备句柄不可复制（独占设备资源）
    McuDeviceHandle(const McuDeviceHandle&)                        = delete;
    auto operator=(const McuDeviceHandle&) -> McuDeviceHandle&     = delete;
    // 允许移动构造、移动赋值：句柄可以转移所有权
    McuDeviceHandle(McuDeviceHandle&&) noexcept                    = default;
    auto operator=(McuDeviceHandle&&) noexcept -> McuDeviceHandle& = default;

    /**
     * @brief 创建USB模式云台设备
     * @param vendor_id USB厂商VID
     * @param product_id 可选USB设备PID，空则自动匹配同VID设备
     * @return 成功返回设备句柄，失败返回错误信息字符串
     * [[nodiscard]] 强制调用方接收返回值，防止忽略连接失败
     * noexcept 函数不会抛出C++异常
     */
    [[nodiscard]] static std::expected<McuDeviceHandle, std::string>
        create_usb(uint16_t vendor_id, std::optional<uint16_t> product_id) noexcept {
        // 新建USB设备实例
        auto device = std::make_unique<UsbDevice>();
        // 执行USB设备连接
        if (auto result = device->connect(vendor_id, product_id); !result) {
            // 连接失败，格式化错误信息返回
            return std::unexpected(
                fmt::format(
                    "connect usb mcu(vendor_id={:#06x}, product_id={}): {}", vendor_id,
                    product_id ? fmt::format("{:#06x}", *product_id) : std::string("auto"),
                    std::move(result).error()));
        }
        // 连接成功，构造句柄并返回（移动智能指针，所有权转移）
        return McuDeviceHandle(std::move(device));
    }

    /**
     * @brief 创建串口TTL模式云台设备
     * @param device_path 串口路径 /dev/ttyUSB0 /dev/ttyACM0
     * @param baud_rate 波特率 115200/460800等
     * @return 成功返回句柄，失败返回错误字符串
     */
    [[nodiscard]] static std::expected<McuDeviceHandle, std::string>
        create_serial(const std::string& device_path, int baud_rate) noexcept {
        auto device = std::make_unique<SerialDevice>();
        if (auto result = device->connect(device_path, baud_rate); !result) {
            return std::unexpected(
                fmt::format(
                    "connect serial mcu(path={}, baud_rate={}): {}", device_path, baud_rate,
                    std::move(result).error()));
        }
        return McuDeviceHandle(std::move(device));
    }

    /**
     * @brief 轮询处理底层设备收发事件（接收MCU回传数据、解析协议包）
     * 统一封装，自动分发到USB/Serial底层实现
     */
    void handle_events() noexcept {
        // std::visit：遍历variant内存储的设备类型，执行对应逻辑
        std::visit([](auto& d) { d->handle_events(); }, device_);
    }

    /**
     * @brief 同步阻塞发送云台指令数据包
     * @param data 待发送字节缓冲区
     * @param size 数据长度
     * @param timeout_ms 发送超时毫秒，默认500ms
     * @return 成功空，失败携带错误信息
     */
    [[nodiscard]] std::expected<void, std::string> send_sync(
        const uint8_t* data, const size_t size, const unsigned timeout_ms = 500) const noexcept {
        return std::visit(
            [&](const auto& d) { return d->send_sync(data, size, timeout_ms); }, device_);
    }

    /**
     * @brief 判断设备当前是否正常连接
     */
    bool is_connected() const noexcept {
        return std::visit([](const auto& d) { return d->is_connected(); }, device_);
    }

    /**
     * @brief 判断当前底层设备是否为串口类型
     */
    bool is_serial() const noexcept {
        // 检查variant当前存储的类型是否为SerialDevice独占智能指针
        return std::holds_alternative<std::unique_ptr<SerialDevice>>(device_);
    }

private:
    // 私有构造函数，仅静态create接口可构造句柄，外部不能直接实例化
    // 构造USB设备句柄
    explicit McuDeviceHandle(std::unique_ptr<UsbDevice> device) noexcept
        : device_(std::move(device)) {}

    // 构造串口设备句柄
    explicit McuDeviceHandle(std::unique_ptr<SerialDevice> device) noexcept
        : device_(std::move(device)) {}

    /**
     * 核心变体存储：二选一存储USB设备/串口设备智能指针
     * std::variant 替代虚函数多态，无vtable开销，编译期类型判定，性能更高
     */
    std::variant<std::unique_ptr<UsbDevice>, std::unique_ptr<SerialDevice>> device_;
};

} // namespace talos_gimbal