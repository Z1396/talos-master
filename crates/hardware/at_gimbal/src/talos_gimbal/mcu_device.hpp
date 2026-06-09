#pragma once

#include "serial.hpp"
#include "stm32.hpp"

#include <memory>
#include <variant>

namespace talos_gimbal {

using UsbDevice    = Stm32Impl<Stm32Parser>;
using SerialDevice = SerialImpl<Stm32Parser>;

class McuDeviceHandle {
public:
    McuDeviceHandle(const McuDeviceHandle&)                        = delete;
    auto operator=(const McuDeviceHandle&) -> McuDeviceHandle&     = delete;
    McuDeviceHandle(McuDeviceHandle&&) noexcept                    = default;
    auto operator=(McuDeviceHandle&&) noexcept -> McuDeviceHandle& = default;

    [[nodiscard]] static std::expected<McuDeviceHandle, std::string>
        create_usb(uint16_t vendor_id, std::optional<uint16_t> product_id) noexcept {
        auto device = std::make_unique<UsbDevice>();
        if (auto result = device->connect(vendor_id, product_id); !result) {
            return std::unexpected(
                fmt::format(
                    "connect usb mcu(vendor_id={:#06x}, product_id={}): {}", vendor_id,
                    product_id ? fmt::format("{:#06x}", *product_id) : std::string("auto"),
                    std::move(result).error()));
        }
        return McuDeviceHandle(std::move(device));
    }

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

    void handle_events() noexcept {
        std::visit([](auto& d) { d->handle_events(); }, device_);
    }

    [[nodiscard]] std::expected<void, std::string> send_sync(
        const uint8_t* data, const size_t size, const unsigned timeout_ms = 500) const noexcept {
        return std::visit(
            [&](const auto& d) { return d->send_sync(data, size, timeout_ms); }, device_);
    }

    bool is_connected() const noexcept {
        return std::visit([](const auto& d) { return d->is_connected(); }, device_);
    }

    bool is_serial() const noexcept {
        return std::holds_alternative<std::unique_ptr<SerialDevice>>(device_);
    }

private:
    explicit McuDeviceHandle(std::unique_ptr<UsbDevice> device) noexcept
        : device_(std::move(device)) {}

    explicit McuDeviceHandle(std::unique_ptr<SerialDevice> device) noexcept
        : device_(std::move(device)) {}

    std::variant<std::unique_ptr<UsbDevice>, std::unique_ptr<SerialDevice>> device_;
};

} // namespace talos_gimbal
