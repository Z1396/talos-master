#pragma once

#include "L5_weapon/fire_control.hpp"
#include "chiral/gimbal.hpp"
#include "quanta/stream_transport.hpp"
#include "shm_client.hpp"
#include "talos_gimbal/mcu_device.hpp"

#include <atomic>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <variant>

namespace fcs::L1 {

// ============================================================================
// IpcOutput - 共享内存输出 (用于 Rust 模拟器)
// ============================================================================

class IpcOutput {
public:
    explicit IpcOutput(std::shared_ptr<ipc::ShmClient> client) noexcept;

    void send(const L5::WeaponCommand& cmd) const noexcept;
    void send_quanta(const quanta::QuantaPacket& packet) const noexcept;

private:
    std::shared_ptr<ipc::ShmClient> client_;
};

// ============================================================================
// McuOutput - MCU 输出 (USB/Serial 统一)
// ============================================================================

class McuOutput {
public:
    explicit McuOutput(std::shared_ptr<talos_gimbal::McuDeviceHandle> device) noexcept;

    McuOutput(McuOutput&& other) noexcept
        : device_(std::move(other.device_))
        , quanta_seq_(other.quanta_seq_.load(std::memory_order_relaxed)) {}

    auto operator=(McuOutput&& other) noexcept -> McuOutput& {
        device_ = std::move(other.device_);
        quanta_seq_.store(
            other.quanta_seq_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        return *this;
    }

    McuOutput(const McuOutput&)                    = delete;
    auto operator=(const McuOutput&) -> McuOutput& = delete;

    void send(const L5::WeaponCommand& cmd) const noexcept;
    void send_quanta(const quanta::QuantaPacket& packet) const noexcept;

private:
    std::shared_ptr<talos_gimbal::McuDeviceHandle> device_;
    mutable std::atomic<uint32_t> quanta_seq_{0};
};

// ============================================================================
// ChiralOutput - Chiral 输出
// ============================================================================

class ChiralOutput {
public:
    explicit ChiralOutput(std::shared_ptr<talos::chiral::gimbal::TalosEndpoint> device) noexcept;

    void send(const L5::WeaponCommand& cmd) const noexcept;
    void send_quanta(const quanta::QuantaPacket& packet) const noexcept;

private:
    std::shared_ptr<talos::chiral::gimbal::TalosEndpoint> device_;
};

// ============================================================================
// OutputInterface - 输出接口
// ============================================================================

class OutputInterface {
public:
    using OutputMode = std::variant<IpcOutput, McuOutput, ChiralOutput>;

    OutputInterface() = delete;
    explicit OutputInterface(OutputMode mode) noexcept;

    OutputInterface(const OutputInterface&)                        = delete;
    auto operator=(const OutputInterface&) -> OutputInterface&     = delete;
    OutputInterface(OutputInterface&&) noexcept                    = default;
    auto operator=(OutputInterface&&) noexcept -> OutputInterface& = default;

    void send(const L5::WeaponCommand& cmd) noexcept;
    void send_quanta(quanta::QuantaPacket&& packet) noexcept {
        std::visit(
            [&packet](auto& output) {
                if constexpr (!std::is_same_v<std::decay_t<decltype(output)>, std::monostate>) {
                    output.send_quanta(packet);
                }
            },
            mode_);
    }

    [[nodiscard]] static std::expected<OutputInterface, std::string>
        create_ipc(std::shared_ptr<ipc::ShmClient> client) noexcept;

    [[nodiscard]] static std::expected<OutputInterface, std::string>
        create_chiral(std::shared_ptr<talos::chiral::gimbal::TalosEndpoint> client) noexcept;

    [[nodiscard]] static std::expected<OutputInterface, std::string> create_usb(
        uint16_t vendor_id = talos_gimbal::Stm32Impl<talos_gimbal::Stm32Parser>::VID,
        std::optional<uint16_t> product_id = std::nullopt) noexcept;

    [[nodiscard]] static std::expected<OutputInterface, std::string> create_serial(
        const std::string& device_path = "/dev/ttyS4", int baud_rate = 115200) noexcept;

private:
    OutputMode mode_;
};

} // namespace fcs::L1
