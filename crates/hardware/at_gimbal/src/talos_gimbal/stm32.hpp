#pragma once

#include "talos_gimbal/packet.hpp"
#include "usb.hpp"

// STL
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <expected>
#include <mutex>
#include <string>
#include <thread>
// spdlog
#include <spdlog/spdlog.h>

/* ------------------- error ------------------- */
namespace talos_gimbal {
[[nodiscard]] inline const char* libusb_error_text(const int err) noexcept {
    return libusb_strerror(err);
}

template <class F>
class scoped_exit {
public:
    explicit scoped_exit(F f)
        : f_(f)
        , enabled_(true) {}
    ~scoped_exit() {
        if (enabled_)
            f_();
    }
    void disable() { enabled_ = false; }

    scoped_exit(const scoped_exit&)            = delete;
    scoped_exit& operator=(const scoped_exit&) = delete;
    scoped_exit(scoped_exit&&)                 = delete;
    scoped_exit& operator=(scoped_exit&&)      = delete;

private:
    F f_;
    bool enabled_;
};

class Stm32Parser {
public:
    // 静态指针，指向外部的 IMU 数据存储 (由外部管理生命周期)
    static inline ReceiveImuData* latest_imu{nullptr};
    static inline ReceiveCapabilitiesData* latest_capabilities{nullptr};

    Stm32Parser(const Stm32Parser&)            = delete;
    Stm32Parser& operator=(const Stm32Parser&) = delete;
    static void parse(const std::span<const std::byte> data) noexcept {
        if (const auto size = data.size_bytes(); size < sizeof(HeaderFrame)) [[unlikely]] {
            SPDLOG_WARN("frame too small: expected {}, got {}", sizeof(HeaderFrame), size);
            return;
        }
        const auto header_frame = reinterpret_cast<const HeaderFrame*>(data.data());
        if (header_frame->sof != HeaderFrame::SoF()) [[unlikely]] {
            SPDLOG_WARN(
                "frame header invalid: expected {:#x}, got {:#x}", HeaderFrame::SoF(),
                header_frame->sof);
            return;
        }
        if (header_frame->id == 0x01) {
            if (latest_imu && data.size_bytes() >= sizeof(ReceiveImuData)) {
                const auto* received = reinterpret_cast<const ReceiveImuData*>(data.data());
                *latest_imu          = *received;
            }
        } else if (header_frame->id == 0x03) {
            if (latest_capabilities && data.size_bytes() == sizeof(ReceiveCapabilitiesData)) {
                const auto* received =
                    reinterpret_cast<const ReceiveCapabilitiesData*>(data.data());
                *latest_capabilities = *received;
            }
            if (latest_imu && data.size_bytes() >= sizeof(ReceiveSimpleImuData)) {
                const auto* received   = reinterpret_cast<const ReceiveSimpleImuData*>(data.data());
                latest_imu->header     = received->header;
                latest_imu->time_stamp = received->time_stamp;
                latest_imu->data.self_color   = received->data.self_color;
                latest_imu->data.bullet_speed = 0.0f;
                latest_imu->data.yaw          = received->data.yaw;
                latest_imu->data.pitch        = received->data.pitch;
                latest_imu->data.roll         = received->data.roll;
                latest_imu->data.yaw_vel      = 0.0f;
                latest_imu->data.pitch_vel    = 0.0f;
                latest_imu->data.roll_vel     = 0.0f;
                latest_imu->eof               = received->eof;
            }
        }
    }
};
static_assert(Parser<Stm32Parser>);

template <Parser Parser>
struct Stm32Impl {
    static constexpr uint16_t VID          = 0x0483;
    static constexpr uint8_t EP_OUT        = 0x01;
    static constexpr uint8_t EP_IN         = 0x81;
    static constexpr uint8_t INTERFACE_NUM = 1;

    explicit Stm32Impl() = default;
    ~Stm32Impl() noexcept { shutdown(); }

    Stm32Impl(const Stm32Impl&)            = delete;
    Stm32Impl& operator=(const Stm32Impl&) = delete;
    Stm32Impl(Stm32Impl&&)                 = delete;
    Stm32Impl& operator=(Stm32Impl&&)      = delete;

    // ─────────────────────────────────────────────
    // 公共接口
    // ─────────────────────────────────────────────

    [[nodiscard]] std::expected<void, std::string> connect(
        const uint16_t vendor_id           = VID,
        std::optional<uint16_t> product_id = std::nullopt) noexcept {
        if (!ctx_) {
            const auto rc = libusb_init_context(&ctx_, nullptr, 0);
            if (rc != LIBUSB_SUCCESS) {
                return std::unexpected(
                    fmt::format("libusb_init_context: {}", libusb_error_text(rc)));
            }
        }

        if (!product_id) {
            auto result = find_device(ctx_, vendor_id);
            if (!result) {
                return std::unexpected(
                    fmt::format(
                        "find usb device(vendor_id={:#06x}): {}", vendor_id, result.error()));
            }
            product_id = *result;
        }
        vid_ = vendor_id;
        pid_ = *product_id;

        if (auto result = open_device(vendor_id, *product_id); !result) {
            return std::unexpected(std::move(result).error());
        }

        if (auto result = setup_transfer(); !result) {
            return std::unexpected(std::move(result).error());
        }

        if (auto result = register_hotplug(vendor_id, *product_id); !result) {
            return std::unexpected(std::move(result).error());
        }

        first_rx_ = true;
        state_.store(State::Connected, std::memory_order_release);
        SPDLOG_INFO("usb mcu connected (vendor_id={:#06x}, product_id={:#06x})", vid_, pid_);
        return {};
    }

    void handle_events() noexcept {
        if (state_.load(std::memory_order_acquire) == State::Disconnected)
            return;

        timeval tv{0, 0};
        libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);

        if (state_.load(std::memory_order_acquire) == State::NeedsReconnect) {
            reconnect();
        }
    }

    [[nodiscard]] std::expected<void, std::string> send_sync(
        const uint8_t* data, const size_t size, const unsigned timeout_ms = 500) const noexcept {
        std::lock_guard lock(device_mtx_);
        if (state_.load(std::memory_order_acquire) != State::Connected || !handle_) {
            return std::unexpected(
                fmt::format(
                    "send_sync(vendor_id={:#06x}, product_id={:#06x}): device not connected", vid_,
                    pid_));
        }
        int actual    = 0;
        const auto rc = libusb_bulk_transfer(
            handle_, EP_OUT, const_cast<uint8_t*>(data), static_cast<int>(size), &actual,
            timeout_ms);
        if (rc != LIBUSB_SUCCESS) {
            return std::unexpected(
                fmt::format(
                    "libusb_bulk_transfer(endpoint={:#04x}, size={}, timeout={}ms): {}", EP_OUT,
                    size, timeout_ms, libusb_error_text(rc)));
        }
        if (actual != static_cast<int>(size)) {
            return std::unexpected(
                fmt::format(
                    "libusb_bulk_transfer: incomplete write (expected {}, wrote {})", size,
                    actual));
        }
        return {};
    }

    bool is_connected() const noexcept {
        return state_.load(std::memory_order_acquire) == State::Connected;
    }

private:
    // ─────────────────────────────────────────────
    // 状态机
    // ─────────────────────────────────────────────
    enum class State { Disconnected, Connected, NeedsReconnect };

    // ─────────────────────────────────────────────
    // 内部实现
    // ─────────────────────────────────────────────

    [[nodiscard]] std::expected<void, std::string>
        open_device(const uint16_t vid, const uint16_t pid) noexcept {
        handle_ = libusb_open_device_with_vid_pid(ctx_, vid, pid);
        if (!handle_) {
            return std::unexpected(
                fmt::format(
                    "libusb_open_device_with_vid_pid(vendor_id={:#06x}, product_id={:#06x}): {}",
                    vid, pid, libusb_error_text(LIBUSB_ERROR_NO_DEVICE)));
        }

        // Auto-detach kernel driver (libusb 1.0.16+)
        libusb_set_auto_detach_kernel_driver(handle_, 1);

        const auto claim_rc =
            static_cast<libusb_error>(libusb_claim_interface(handle_, INTERFACE_NUM));
        if (claim_rc != LIBUSB_SUCCESS) {
            libusb_close(handle_);
            handle_ = nullptr;
            return std::unexpected(
                fmt::format(
                    "libusb_claim_interface(interface={}, vendor_id={:#06x}, product_id={:#06x}): "
                    "{}",
                    INTERFACE_NUM, vid, pid, libusb_error_text(claim_rc)));
        }
        return {};
    }

    [[nodiscard]] std::expected<void, std::string> setup_transfer() noexcept {
        rx_transfer_ = libusb_alloc_transfer(0);
        if (!rx_transfer_) {
            return std::unexpected(
                fmt::format(
                    "libusb_alloc_transfer(vendor_id={:#06x}, product_id={:#06x}): {}", vid_, pid_,
                    libusb_error_text(LIBUSB_ERROR_NO_MEM)));
        }

        libusb_fill_bulk_transfer(
            rx_transfer_, handle_, EP_IN, rx_buf_.data(), rx_buf_.size(),
            &Stm32Impl::on_rx_complete, this, 0);

        const auto submit_rc = static_cast<libusb_error>(libusb_submit_transfer(rx_transfer_));
        if (submit_rc != LIBUSB_SUCCESS) {
            libusb_free_transfer(rx_transfer_);
            rx_transfer_ = nullptr;
            return std::unexpected(
                fmt::format(
                    "libusb_submit_transfer(endpoint={:#04x}, vendor_id={:#06x}, "
                    "product_id={:#06x}): {}",
                    EP_IN, vid_, pid_, libusb_error_text(submit_rc)));
        }
        {
            std::lock_guard lock(mtx_);
            transfer_active_ = true;
        }
        return {};
    }

    [[nodiscard]] std::expected<void, std::string>
        register_hotplug(const uint16_t vid, const uint16_t pid) noexcept {
        if (!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
            return std::unexpected(
                fmt::format(
                    "libusb hotplug unsupported(vendor_id={:#06x}, product_id={:#06x})", vid, pid));
        }

        auto rc = libusb_hotplug_register_callback(
            ctx_,
            static_cast<libusb_hotplug_event>(
                LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT | LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED),
            LIBUSB_HOTPLUG_NO_FLAGS, vid, pid, LIBUSB_HOTPLUG_MATCH_ANY, &Stm32Impl::on_hotplug,
            this, &hp_handle_);
        if (rc == LIBUSB_SUCCESS) {
            return {};
        }
        return std::unexpected(
            fmt::format(
                "libusb_hotplug_register_callback(vendor_id={:#06x}, product_id={:#06x}): {}", vid,
                pid, libusb_error_text(rc)));
    }

    // ─────────────────────────────────────────────
    // 回调 (静态)
    // ─────────────────────────────────────────────

    static void LIBUSB_CALL on_rx_complete(libusb_transfer* tr) noexcept {
        auto self = static_cast<Stm32Impl*>(tr->user_data);

        if (tr->status == LIBUSB_TRANSFER_CANCELLED) [[unlikely]] {
            self->notify_transfer_done();
            return;
        }

        if (self->state_.load(std::memory_order_acquire) != State::Connected) [[unlikely]] {
            self->notify_transfer_done();
            return;
        }

        if (tr->status != LIBUSB_TRANSFER_COMPLETED) [[unlikely]] {
            SPDLOG_WARN("usb rx transfer failed (status={})", static_cast<int>(tr->status));
            self->state_.store(State::NeedsReconnect, std::memory_order_release);
            self->notify_transfer_done();
            return;
        }

        // 处理数据
        if (tr->actual_length > 0 && !self->first_rx_) [[likely]] {
            Parser::parse(std::span(reinterpret_cast<std::byte*>(tr->buffer), tr->actual_length));
        }
        self->first_rx_ = false;

        // 继续接收
        if (libusb_submit_transfer(tr) != LIBUSB_SUCCESS) {
            SPDLOG_WARN("usb rx resubmit failed");
            self->state_.store(State::NeedsReconnect, std::memory_order_release);
            self->notify_transfer_done();
        }
    }

    static int LIBUSB_CALL on_hotplug(
        libusb_context*, libusb_device*, const libusb_hotplug_event event, void* user) noexcept {
        auto self = static_cast<Stm32Impl*>(user);
        if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
            SPDLOG_INFO(
                "usb mcu disconnected (vendor_id={:#06x}, product_id={:#06x})", self->vid_,
                self->pid_);
            self->state_.store(State::NeedsReconnect, std::memory_order_release);
        } else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
            if (self->state_.load(std::memory_order_acquire) == State::NeedsReconnect) {
                SPDLOG_INFO("usb mcu re-appeared, waiting for enumeration...");
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
        }
        return 0;
    }

    // ─────────────────────────────────────────────
    // 资源管理
    // ─────────────────────────────────────────────

    void notify_transfer_done() noexcept {
        std::lock_guard lock(mtx_);
        transfer_active_ = false;
        cv_.notify_one();
    }

    void wait_transfer_done() noexcept {
        std::unique_lock lock(mtx_);
        while (transfer_active_) {
            lock.unlock();
            timeval tv{0, 10000};
            libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);
            lock.lock();
        }
    }

    void close_device() noexcept {
        std::lock_guard lock(device_mtx_);
        close_device_locked();
    }

    void close_device_locked() noexcept {
        if (rx_transfer_) {
            bool should_cancel = false;
            {
                std::lock_guard lock(mtx_);
                should_cancel = transfer_active_;
            }
            if (should_cancel) {
                const auto rc = libusb_cancel_transfer(rx_transfer_);
                if (rc != LIBUSB_SUCCESS && rc != LIBUSB_ERROR_NOT_FOUND) {
                    SPDLOG_WARN("libusb_cancel_transfer: {}", libusb_error_text(rc));
                }
                wait_transfer_done();
            }
            libusb_free_transfer(rx_transfer_);
            rx_transfer_ = nullptr;
        }
        if (handle_) {
            libusb_release_interface(handle_, INTERFACE_NUM);
            libusb_close(handle_);
            handle_ = nullptr;
        }
    }

    void reconnect() noexcept {
        SPDLOG_INFO("usb mcu reconnecting (vendor_id={:#06x}, product_id={:#06x})...", vid_, pid_);
        std::lock_guard lock(device_mtx_);
        close_device_locked();
        if (auto open_result = open_device(vid_, pid_); !open_result) {
            SPDLOG_WARN("usb mcu reconnect failed: {}", open_result.error());
            return;
        }
        if (auto setup_result = setup_transfer(); !setup_result) {
            SPDLOG_WARN("usb mcu reconnect setup failed: {}", setup_result.error());
            return;
        }
        first_rx_ = true;
        state_.store(State::Connected, std::memory_order_release);
        SPDLOG_INFO("usb mcu reconnected (vendor_id={:#06x}, product_id={:#06x})", vid_, pid_);
    }

    void shutdown() noexcept {
        const auto previous_state = state_.exchange(State::Disconnected, std::memory_order_acq_rel);
        if (previous_state == State::Disconnected && !ctx_) {
            return;
        }
        SPDLOG_INFO("usb mcu shutting down");
        if (hp_handle_ && ctx_) {
            libusb_hotplug_deregister_callback(ctx_, hp_handle_);
            hp_handle_ = 0;
        }
        close_device();
        if (ctx_) {
            libusb_exit(ctx_);
            ctx_ = nullptr;
        }
    }

    [[nodiscard]] static std::expected<uint16_t, std::string>
        find_device(libusb_context* ctx, const uint16_t vid) noexcept {
        libusb_device** list;
        auto cnt = libusb_get_device_list(ctx, &list);
        if (cnt < 0) {
            return std::unexpected(
                fmt::format(
                    "libusb_get_device_list(vendor_id={:#06x}): {}", vid,
                    libusb_error_text(static_cast<int>(cnt))));
        }

        scoped_exit cleanup{[&] { libusb_free_device_list(list, 1); }};

        uint16_t found_pid = 0;
        int count          = 0;

        for (ssize_t i = 0; i < cnt; ++i) {
            libusb_device_descriptor desc{};
            const auto desc_rc =
                static_cast<libusb_error>(libusb_get_device_descriptor(list[i], &desc));
            if (desc_rc != LIBUSB_SUCCESS) {
                continue;
            }
            if (desc.idVendor != vid)
                continue;
            found_pid = desc.idProduct;
            if (++count > 1) {
                return std::unexpected(
                    fmt::format(
                        "find usb device(vendor_id={:#06x}): multiple product ids matched", vid));
            }
        }

        if (count == 1) {
            return found_pid;
        }
        return std::unexpected(
            fmt::format("find usb device(vendor_id={:#06x}): no matching device found", vid));
    }

    // ─────────────────────────────────────────────
    // 成员变量
    // ─────────────────────────────────────────────
    libusb_context* ctx_                      = nullptr;
    libusb_device_handle* handle_             = nullptr;
    libusb_transfer* rx_transfer_             = nullptr;
    libusb_hotplug_callback_handle hp_handle_ = 0;

    std::array<uint8_t, 64> rx_buf_{};
    uint16_t vid_ = 0;
    uint16_t pid_ = 0;

    std::atomic<State> state_{State::Disconnected};
    bool first_rx_ = true;

    // Protects libusb handle ownership while the writer thread and event thread share the device.
    mutable std::mutex device_mtx_;

    // Transfer 取消同步：reconnect/close_device 在事件循环内调用，
    // mutex + condvar 保证 cancel-then-wait 和回调之间的正确同步。
    std::mutex mtx_;
    std::condition_variable cv_;
    bool transfer_active_ = false;
};
static_assert(Device<Stm32Impl<Stm32Parser>>);
} // namespace talos_gimbal
