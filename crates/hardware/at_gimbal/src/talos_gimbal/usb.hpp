#pragma once

#include <expected>
#include <libusb/libusb.h>
#include <optional>
#include <span>
#include <string>

#include "packet.hpp"

#include <fmt/core.h>

namespace talos_gimbal {
template <typename T>
concept Device = requires(
    T& device, uint16_t vendor_id, std::optional<uint16_t> product_id, uint8_t* data, size_t size,
    unsigned timeout) {
    // impl Drop
    requires std::destructible<T>;

    // impl !Copy
    requires !std::copyable<T>;

    {
        device.connect(vendor_id, product_id)
    } noexcept -> std::same_as<std::expected<void, std::string>>;
    {
        device.send_sync(data, size, timeout)
    } noexcept -> std::same_as<std::expected<void, std::string>>;
    { device.handle_events() } noexcept -> std::same_as<void>;
};

template <typename T>
concept Parser = requires(const std::span<std::byte> data) {
    { T::parse(data) } -> std::same_as<void>;
};

} // namespace talos_gimbal
