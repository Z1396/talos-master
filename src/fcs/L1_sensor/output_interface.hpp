#pragma once
// L5武器控制指令结构体 WeaponCommand
#include "L5_weapon/fire_control.hpp"
// Chiral云台底层端点通信封装
#include "chiral/gimbal.hpp"
// 高速数据流包 QuantaPacket 传输调试/遥测数据
#include "quanta/stream_transport.hpp"
// 共享内存IPC客户端（模拟器通信）
#include "shm_client.hpp"
// STM32 MCU串口/USB设备操作句柄
#include "talos_gimbal/mcu_device.hpp"

// 原子序列号（多线程发包计数）
#include <atomic>
// C++23 无异常错误返回类型
#include <expected>
// 智能指针管理设备句柄生命周期
#include <memory>
// 可选值（串口路径、PID可选参数）
#include <optional>
// 串口设备路径字符串
#include <string>
// C++17 类型安全变体，替代虚基类多态
#include <variant>

namespace fcs::L1 {

// ============================================================================
// IpcOutput：共享内存输出，对接Rust仿真模拟器
// 场景：电脑仿真调试，不连接真实云台硬件
// ============================================================================
class IpcOutput {
public:
    /**
     * @brief 构造共享内存输出端
     * @param client 共享内存客户端智能指针，外部持有生命周期
     * @noexcept 构造不会抛出异常
     */
    explicit IpcOutput(std::shared_ptr<ipc::ShmClient> client) noexcept;

    /**
     * @brief 发送武器射击/云台控制指令到共享内存
     * @param cmd L5层标准化武器指令
     * @const 只读操作，不修改当前对象状态
     */
    void send(const L5::WeaponCommand& cmd) const noexcept;

    /**
     * @brief 发送高速遥测数据流包（图像/滤波日志/调试数据）
     * @param packet Quanta标准流数据包
     */
    void send_quanta(const quanta::QuantaPacket& packet) const noexcept;

private:
    // 共享内存通信客户端句柄，shared_ptr延长生命周期
    std::shared_ptr<ipc::ShmClient> client_;
};

// ============================================================================
// McuOutput：真实STM32云台MCU输出（USB/串口通用）
// 特性：
// 1. 仅支持移动语义，禁止拷贝（串口设备不可多份持有）
// 2. atomic 线程安全数据包序列号，多线程并发发包不重复
// ============================================================================
class McuOutput {
public:
    /**
     * @brief 构造MCU串口/USB输出设备
     * @param device STM32设备句柄智能指针
     */
    explicit McuOutput(std::shared_ptr<talos_gimbal::McuDeviceHandle> device) noexcept;

    /**
     * @brief 移动构造，转移设备句柄所有权
     * @param other 源McuOutput对象
     * 同步转移设备句柄与原子序列号计数（relaxed宽松内存序仅保证原子读取）
     */
    McuOutput(McuOutput&& other) noexcept
        : device_(std::move(other.device_))
        , quanta_seq_(other.quanta_seq_.load(std::memory_order_relaxed)) {}

    /**
     * @brief 移动赋值运算符，转移设备句柄与序列号
     */
    auto operator=(McuOutput&& other) noexcept -> McuOutput& {
        device_ = std::move(other.device_);
        quanta_seq_.store(
            other.quanta_seq_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        return *this;
    }

    // 删除拷贝构造、拷贝赋值：串口硬件设备不能被复制，避免多线程竞争、双重释放
    McuOutput(const McuOutput&)                    = delete;
    auto operator=(const McuOutput&) -> McuOutput& = delete;

    /**
     * @brief 下发云台/射击指令到MCU
     */
    void send(const L5::WeaponCommand& cmd) const noexcept;

    /**
     * @brief 下发遥测数据流包，内部自动自增原子序列号用于分包校验
     */
    void send_quanta(const quanta::QuantaPacket& packet) const noexcept;

private:
    // STM32 MCU通信设备句柄
    std::shared_ptr<talos_gimbal::McuDeviceHandle> device_;
    /**
     * @brief Quanta数据包序列号，多线程并发发包计数
     * mutable：const send_quanta函数内部可修改，仅做计数器无需同步屏障，使用relaxed内存序
     * memory_order_relaxed：只保证原子读写，无跨线程数据同步开销，纯计数场景最优性能
     */
    mutable std::atomic<uint32_t> quanta_seq_{0};
};

// ============================================================================
// ChiralOutput：自研Chiral高速云台协议输出
// 用于新型自研云台驱动链路，独立于传统STM32串口MCU
// ============================================================================
class ChiralOutput {
public:
    /**
     * @brief 构造Chiral云台通信端点输出
     * @param device Chiral协议通信端点智能指针
     */
    explicit ChiralOutput(std::shared_ptr<talos::chiral::gimbal::TalosEndpoint> device) noexcept;

    /**
     * @brief 发送标准化武器控制指令
     */
    void send(const L5::WeaponCommand& cmd) const noexcept;

    /**
     * @brief 发送Quanta高速遥测数据包
     */
    void send_quanta(const quanta::QuantaPacket& packet) const noexcept;

private:
    std::shared_ptr<talos::chiral::gimbal::TalosEndpoint> device_;
};

// ============================================================================
// OutputInterface：顶层统一输出抽象接口（核心门面类）
// 设计模式：访问者模式 + std::variant 静态多态，无虚函数开销
// 对外仅暴露一套send接口，底层自动分发至 Ipc/Mcu/Chiral 任意一种设备
// ============================================================================
class OutputInterface {
public:
    /**
     * @brief 输出模式变体：三种底层设备类型三选一，运行时确定活跃类型
     * 替代传统抽象基类+虚函数，编译期类型完备，无RTTI、无vtable性能损耗
     */
    using OutputMode = std::variant<IpcOutput, McuOutput, ChiralOutput>;

    // 禁止无参构造，必须指定输出设备类型
    OutputInterface() = delete;

    /**
     * @brief 直接用构造好的设备变体初始化统一输出接口
     */
    explicit OutputInterface(OutputMode mode) noexcept;

    // 禁止拷贝：通信设备资源独占，不允许复制接口
    OutputInterface(const OutputInterface&)                        = delete;
    auto operator=(const OutputInterface&) -> OutputInterface&     = delete;
    // 允许移动构造/移动赋值：支持接口所有权转移
    OutputInterface(OutputInterface&&) noexcept                    = default;
    auto operator=(OutputInterface&&) noexcept -> OutputInterface& = default;

    /**
     * @brief 统一发送武器控制指令门面接口
     * 内部std::visit自动匹配variant内活跃设备，调用对应send实现
     */
    void send(const L5::WeaponCommand& cmd) noexcept;

    /**
     * @brief 统一发送遥测数据包门面接口
     * @param packet 右值传参，允许内部移动转移数据包内存，减少拷贝
     * std::visit + constexpr if 编译期分支匹配设备类型，跳过空monostate保护
     */
    void send_quanta(quanta::QuantaPacket&& packet) noexcept {
        std::visit(
            [&packet](auto& output) {
                // 编译期判断：排除variant空占位monostate（本项目不会触发）
                if constexpr (!std::is_same_v<std::decay_t<decltype(output)>, std::monostate>) {
                    output.send_quanta(packet);
                }
            },
            mode_);
    }

    // ====================== 静态工厂函数：创建设备接口，返回std::expected处理打开失败 ======================
    /**
     * @brief 创建共享内存仿真输出接口
     * @param client 已初始化的共享内存客户端
     * @return expected<成功接口对象, 错误字符串>，无异常抛出，嵌入式友好
     */
    [[nodiscard]] static std::expected<OutputInterface, std::string>
        create_ipc(std::shared_ptr<ipc::ShmClient> client) noexcept;

    /**
     * @brief 创建Chiral自研云台输出接口
     * @param client Chiral通信端点句柄
     */
    [[nodiscard]] static std::expected<OutputInterface, std::string>
        create_chiral(std::shared_ptr<talos::chiral::gimbal::TalosEndpoint> client) noexcept;

    /**
     * @brief 创建USB虚拟串口MCU设备（STM32）
     * @param vendor_id USB厂商VID，默认STM32标准VID
     * @param product_id 可选PID，为空自动匹配第一个STM32设备
     */
    [[nodiscard]] static std::expected<OutputInterface, std::string> create_usb(
        uint16_t vendor_id = talos_gimbal::Stm32Impl<talos_gimbal::Stm32Parser>::VID,
        std::optional<uint16_t> product_id = std::nullopt) noexcept;

    /**
     * @brief 创建物理串口MCU设备（Linux /dev/ttyS4）
     * @param device_path 串口设备路径
     * @param baud_rate 波特率默认115200
     */
    [[nodiscard]] static std::expected<OutputInterface, std::string> create_serial(
        const std::string& device_path = "/dev/ttyS4", int baud_rate = 115200) noexcept;

private:
    // 存储当前激活的输出设备变体（IpcOutput/McuOutput/ChiralOutput三选一）
    OutputMode mode_;
};

} // namespace fcs::L1