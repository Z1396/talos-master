#pragma once
// 头文件保护，防止同一头文件被多次重复包含，避免重复定义编译错误

// C++23 标准库：用于函数返回【成功/错误】结果类型
#include <expected>
// libusb 底层USB设备操作C库API
#include <libusb/libusb.h>
// 可选值容器，代表“可能存在/不存在”的uint16设备PID
#include <optional>
// 无所有权只读字节视图，用于安全传递数据包，无需拷贝内存
#include <span>
// 字符串，用于错误信息、设备描述文本
#include <string>

// 云台通信数据包结构体定义
#include "packet.hpp"

// fmt格式化库，用于拼接错误日志、打印信息
#include <fmt/core.h>

namespace talos_gimbal {
/**
 * @brief C++20 Concept：约束通用通信设备（串口/USB设备统一接口规范）
 * @tparam T 待约束的设备实现类（SerialImpl / UsbImpl）
 *
 * 作用：编译期强制校验设备类必须实现统一接口，保证上层业务代码可多态替换串口/USB
 * 模板约束分为几大类：生命周期约束、接口函数约束
 *
 * requires(参数列表) 内列出调用该类接口所需的参数类型，用于语法校验
 * 参数说明：
 *  T& device：设备实例引用
 *  uint16_t vendor_id：USB厂商VID
 *  std::optional<uint16_t> product_id：可选设备PID，可传空匹配所有同VID设备
 *  uint8_t* data：待发送原始字节缓冲区
 *  size_t size：发送数据长度
 *  unsigned timeout：收发超时毫秒数
 */
template <typename T>
concept Device = requires(
    T& device, uint16_t vendor_id, std::optional<uint16_t> product_id, uint8_t* data, size_t size,
    unsigned timeout) {
    // 约束1：必须支持析构（RAII资源自动释放，类似Rust Drop trait）
    /*std::destructible<T>	能否合法调用析构函数（核心）
    std::trivially_destructible<T>	可析构 + 析构是平凡（trivial）
    无自定义析构、无带非平凡析构成员 / 基类
    std::nothrow_destructible<T>	可析构 + 析构保证 noexcept(true)*/
    requires std::destructible<T>;

    // 约束2：禁止拷贝构造/拷贝赋值（设备是独占硬件资源，不能共享）
    requires !std::copyable<T>;

    // 约束3：connect 连接接口
    // 输入：VID、可选PID；无异常抛出；返回 std::expected<void, string>（成功空值/错误字符串）
    /*这一段只出现在 requires 约束块里面，叫复合需求（compound requirement），
    是 C++20 Concept 专门用来校验函数调用行为的语法，普通函数声明不能这么写。*/
    {
        device.connect(vendor_id, product_id)
    } noexcept -> std::same_as<std::expected<void, std::string>>;

    // 约束4：同步发送接口 send_sync
    // 输入：数据指针、长度、超时；无异常；返回统一错误类型
    {
        device.send_sync(data, size, timeout)
    } noexcept -> std::same_as<std::expected<void, std::string>>;

    // 约束5：事件轮询接口 handle_events
    // 外部循环定时调用，非阻塞读取设备数据、解析数据包；无返回、不抛异常
    { device.handle_events() } noexcept -> std::same_as<void>;
};

/**
 * @brief C++20 Concept：约束数据包解析器
 * @tparam T 解析器类
 *
 * 作用：统一串口/USB的数据解析接口，解耦底层IO与上层业务逻辑
 * 底层设备收到完整帧后，调用 Parser::parse 处理原始字节流
 */
template <typename T>
concept Parser = requires(const std::span<std::byte> data) {
    // 约束：必须拥有静态无返回parse函数，接收只读字节span
    { T::parse(data) } -> std::same_as<void>;
};

} // namespace talos_gimbal