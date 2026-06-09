#pragma once

#include "L1_sensor/parcel.hpp"
#include "camera_config.hpp"
#include "hik_camera.hpp"
#include "shm_client.hpp"

#include <chrono>
#include <expected>
#include <fmt/core.h>
#include <fmt/std.h>
#include <memory>
#include <optional>
#include <string>
#include <variant>

namespace fcs::L1 {

struct Timeout {};
struct Disconnected {
    std::string ctx;
};
struct ProducerNotAvailable {};

using InputError = std::variant<
    Timeout, Disconnected, ProducerNotAvailable, hikcamera::CameraError, ipc::ShmError>;

// ============================================================================
// IpcInput - 共享内存输入
// ============================================================================

class IpcInput {
public:
    explicit IpcInput(std::shared_ptr<ipc::ShmClient> client, CameraConfig info) noexcept;

    ~IpcInput();

    IpcInput(const IpcInput&)            = delete;
    IpcInput& operator=(const IpcInput&) = delete;

    IpcInput(IpcInput&&) noexcept            = default;
    IpcInput& operator=(IpcInput&&) noexcept = default;

    [[nodiscard]] std::optional<Frame> try_recv() const noexcept;
    [[nodiscard]] std::expected<Frame, InputError>
        recv(std::chrono::milliseconds timeout) const noexcept;
    [[nodiscard]] const CameraConfig& camera_info() const noexcept;

private:
    std::shared_ptr<ipc::ShmClient> client_;
    CameraConfig info_;
};

// ============================================================================
// HikInput - HIK 相机输入
// ============================================================================

class HikInput {
public:
    explicit HikInput(
        std::unique_ptr<hikcamera::ImageCapturer> camera, CameraConfig info,
        hikcamera::ImageCapturer::CameraProfile profile,
        std::chrono::duration<float, std::micro> exposure_time,
        std::optional<std::string> device_name) noexcept;

    HikInput(const HikInput&)            = delete;
    HikInput& operator=(const HikInput&) = delete;

    HikInput(HikInput&&) noexcept            = default;
    HikInput& operator=(HikInput&&) noexcept = default;

    [[nodiscard]] std::optional<Frame> try_recv() noexcept;
    [[nodiscard]] std::expected<Frame, InputError> recv(std::chrono::milliseconds timeout) noexcept;
    [[nodiscard]] const CameraConfig& camera_info() const noexcept;

private:
    [[nodiscard]] uint64_t now_ns() const noexcept;

    std::unique_ptr<hikcamera::ImageCapturer> camera_;
    CameraConfig info_;
    hikcamera::ImageCapturer::CameraProfile profile_;
    uint64_t exposure_time_ns_;
    uint64_t seq_{0};
    std::optional<std::string> device_name_;
    static constexpr uint64_t kReconnectRetryLimit = 25565;
    uint64_t retry_{kReconnectRetryLimit};
};

// ============================================================================
// CameraInterface - 相机接口
// ============================================================================

class CameraInterface {
public:
    using InputMode = std::variant<IpcInput, HikInput>;

    CameraInterface() = delete;
    explicit CameraInterface(InputMode mode) noexcept;

    CameraInterface(const CameraInterface&)                = delete;
    CameraInterface& operator=(const CameraInterface&)     = delete;
    CameraInterface(CameraInterface&&) noexcept            = default;
    CameraInterface& operator=(CameraInterface&&) noexcept = default;

    [[nodiscard]] std::optional<Frame> try_recv() noexcept;
    [[nodiscard]] std::expected<Frame, InputError> recv(std::chrono::milliseconds timeout) noexcept;
    [[nodiscard]] const CameraConfig& camera_info() const noexcept;

    [[nodiscard]] static std::expected<CameraInterface, InputError>
        create_ipc(std::shared_ptr<ipc::ShmClient> client) noexcept;
    [[nodiscard]] static std::expected<CameraInterface, InputError>
        create_hik(const CameraConfig& config) noexcept;

private:
    InputMode mode_;
};

} // namespace fcs::L1

// ============================================================================
// fmt::formatter specializations
// ============================================================================

namespace fmt {

template <>
struct formatter<fcs::L1::Timeout> : formatter<std::string_view> {
    auto format(const fcs::L1::Timeout, format_context& ctx) const {
        return formatter<std::string_view>::format("timeout", ctx);
    }
};

template <>
struct formatter<fcs::L1::Disconnected> : formatter<std::string_view> {
    auto format(const fcs::L1::Disconnected d, format_context& ctx) const {
        return formatter<std::string_view>::format(fmt::format("disconnected: {}", d.ctx), ctx);
    }
};

template <>
struct formatter<fcs::L1::ProducerNotAvailable> : formatter<std::string_view> {
    auto format(const fcs::L1::ProducerNotAvailable, format_context& ctx) const {
        return formatter<std::string_view>::format("producer_not_available", ctx);
    }
};

template <>
struct formatter<fcs::L1::InputError> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    auto format(const fcs::L1::InputError& err, format_context& ctx) const {
        return std::visit(
            [&ctx](const auto& e) { return fmt::format_to(ctx.out(), "{}", e); }, err);
    }
};

} // namespace fmt
