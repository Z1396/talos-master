// 头文件保护，防止重复包含
#pragma once

// 自定义云台协议包定义、USB底层封装
#include "talos_gimbal/packet.hpp"
#include "usb.hpp"

// C++标准库
#include <array>               // 固定大小接收缓冲区
#include <atomic>              // 无锁状态机原子变量
#include <chrono>              // 延时处理热插拔重连
#include <condition_variable>  // 异步传输同步等待
#include <cstddef>
#include <expected>            // 现代C++错误处理，替代异常
#include <mutex>               // 多线程互斥锁
#include <string>
#include <thread>              // 线程休眠

// 日志库
#include <spdlog/spdlog.h>

namespace talos_gimbal {

/**
 * @brief 将libusb错误码转为可读字符串
 * @param err libusb返回错误码
 * @return 静态错误描述文本
 */
[[nodiscard]] inline const char* libusb_error_text(const int err) noexcept {
    return libusb_strerror(err);
}

/**
 * @brief 作用域自动执行清理回调 RAII 工具类
 * 离开作用域自动执行传入的回调函数，用于资源自动释放
 * 可手动 disable 取消执行
 * 禁止拷贝/移动，仅临时局部对象使用
 */
template <class F>
class scoped_exit {
public:
    // 传入清理函数
    explicit scoped_exit(F f)
        : f_(f)
        , enabled_(true) {}

    // 析构自动执行清理函数
    ~scoped_exit() {
        if (enabled_)
            f_();
    }

    // 取消自动执行
    void disable() { enabled_ = false; }

    // 禁用拷贝、移动，只允许栈临时变量
    scoped_exit(const scoped_exit&)            = delete;
    scoped_exit& operator=(const scoped_exit&) = delete;
    scoped_exit(scoped_exit&&)                 = delete;
    scoped_exit& operator=(scoped_exit&&)      = delete;

private:
    F f_;        // 存储清理回调
    bool enabled_;// 是否启用自动执行
};

/**
 * @brief STM32云台数据包解析器
 * 静态全局指针绑定外部IMU、能力数据存储，USB回调收到数据后自动解析填充
 * 满足通用Parser约束，可被Stm32Impl模板复用
 */
class Stm32Parser {
public:
    // 静态全局指针：外部全局数据存储，生命周期由上层调度管理
    static inline ReceiveImuData* latest_imu{nullptr};
    static inline ReceiveCapabilitiesData* latest_capabilities{nullptr};

    // 禁用拷贝
    Stm32Parser(const Stm32Parser&)            = delete;
    Stm32Parser& operator=(const Stm32Parser&) = delete;

    /**
     * @brief 原始USB字节流解析入口
     * @param data USB原始接收字节span
     * 逻辑：校验帧头SoF → 根据帧ID分别解析IMU/设备能力包
     */
    static void parse(const std::span<const std::byte> data) noexcept {
        // 数据长度小于帧头大小，非法短包直接丢弃
        if (const auto size = data.size_bytes(); size < sizeof(HeaderFrame)) [[unlikely]] {
            SPDLOG_WARN("frame too small: expected {}, got {}", sizeof(HeaderFrame), size);
            return;
        }
        // 强转协议帧头部
        const auto header_frame = reinterpret_cast<const HeaderFrame*>(data.data());
        // 校验帧起始标志SoF
        if (header_frame->sof != HeaderFrame::SoF()) [[unlikely]] {
            SPDLOG_WARN(
                "frame header invalid: expected {:#x}, got {:#x}", HeaderFrame::SoF(),
                header_frame->sof);
            return;
        }

        // 帧ID 0x01：完整IMU数据包
        if (header_frame->id == 0x01) {
            // 外部IMU缓存有效且包长度足够
            if (latest_imu && data.size_bytes() >= sizeof(ReceiveImuData)) {
                const auto* received = reinterpret_cast<const ReceiveImuData*>(data.data());
                *latest_imu          = *received;
            }
        }
        // 帧ID 0x03：设备能力 + 简易IMU数据包
        else if (header_frame->id == 0x03) {
            // 解析设备能力信息
            if (latest_capabilities && data.size_bytes() == sizeof(ReceiveCapabilitiesData)) {
                const auto* received =
                    reinterpret_cast<const ReceiveCapabilitiesData*>(data.data());
                *latest_capabilities = *received;
            }
            // 解析简易IMU数据，填充完整IMU结构体缺失字段（速度置0）
            if (latest_imu && data.size_bytes() >= sizeof(ReceiveSimpleImuData)) {
                const auto* received   = reinterpret_cast<const ReceiveSimpleImuData*>(data.data());
                latest_imu->header     = received->header;
                latest_imu->time_stamp = received->time_stamp;
                latest_imu->data.self_color   = received->data.self_color;
                latest_imu->data.bullet_speed = 0.0f; // 简易包无弹速
                latest_imu->data.yaw          = received->data.yaw;
                latest_imu->data.pitch        = received->data.pitch;
                latest_imu->data.roll         = received->data.roll;
                latest_imu->data.yaw_vel      = 0.0f; // 简易包无角速度
                latest_imu->data.pitch_vel    = 0.0f;
                latest_imu->data.roll_vel     = 0.0f;
                latest_imu->eof               = received->eof;
            }
        }
    }
};
// 编译期校验：满足Parser概念约束，可作为模板参数传入Stm32Impl
static_assert(Parser<Stm32Parser>);

/**
 * @brief USB STM32设备底层驱动模板实现
 * @tparam Parser 数据包解析器（Stm32Parser）
 * 基于libusb异步批量传输，支持热插拔、断线自动重连、同步发送指令
 */
template <Parser Parser>
struct Stm32Impl {
    // STM32 USB设备固定硬件参数
    static constexpr uint16_t VID          = 0x0483;    // ST官方厂商ID
    static constexpr uint8_t EP_OUT        = 0x01;      // 输出端点（主机→MCU）
    static constexpr uint8_t EP_IN         = 0x81;      // 输入端点（MCU→主机）
    static constexpr uint8_t INTERFACE_NUM = 1;         // USB接口号

    // 默认构造
    explicit Stm32Impl() = default;
    // 析构自动执行关闭释放资源
    ~Stm32Impl() noexcept { shutdown(); }

    // 禁止拷贝、移动：USB设备独占资源，不可转移/复制
    Stm32Impl(const Stm32Impl&)            = delete;
    Stm32Impl& operator=(const Stm32Impl&) = delete;
    Stm32Impl(Stm32Impl&&)                 = delete;
    Stm32Impl& operator=(Stm32Impl&&)      = delete;

    // ====================== 对外公共接口 ======================
    /**
     * @brief 打开USB设备，初始化libusb上下文、注册异步接收、热插拔回调
     * @param vendor_id 厂商VID，默认0x0483
     * @param product_id 可选PID，为空自动扫描同VID设备
     * @return 成功空，失败携带错误字符串
     */
    [[nodiscard]] std::expected<void, std::string> connect(
        const uint16_t vendor_id           = VID,
        std::optional<uint16_t> product_id = std::nullopt) noexcept {
        // 初始化libusb全局上下文
        if (!ctx_) {
            const auto rc = libusb_init_context(&ctx_, nullptr, 0);
            if (rc != LIBUSB_SUCCESS) {
                return std::unexpected(
                    fmt::format("libusb_init_context: {}", libusb_error_text(rc)));
            }
        }

        // 未指定PID，自动遍历USB设备查找唯一匹配PID
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

        // 打开VID/PID对应USB设备句柄
        if (auto result = open_device(vendor_id, *product_id); !result) {
            return std::unexpected(std::move(result).error());
        }

        // 初始化异步批量接收传输
        if (auto result = setup_transfer(); !result) {
            return std::unexpected(std::move(result).error());
        }

        // 注册USB热插拔回调（设备拔出/重新插入触发）
        if (auto result = register_hotplug(vendor_id, *product_id); !result) {
            return std::unexpected(std::move(result).error());
        }

        first_rx_ = true;
        // 原子更新状态为已连接
        state_.store(State::Connected, std::memory_order_release);
        SPDLOG_INFO("usb mcu connected (vendor_id={:#06x}, product_id={:#06x})", vid_, pid_);
        return {};
    }

    /**
     * @brief 单次USB事件轮询，处理异步接收回调、断线重连检测
     * 上层调度固定频率调用（如200Hz）
     */
    void handle_events() noexcept {
        // 已断开直接返回
        if (state_.load(std::memory_order_acquire) == State::Disconnected)
            return;

        // 非阻塞处理libusb异步事件
        timeval tv{0, 0};
        libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);

        // 标记需要重连，执行重连逻辑
        if (state_.load(std::memory_order_acquire) == State::NeedsReconnect) {
            reconnect();
        }
    }

    /**
     * @brief 同步阻塞批量发送数据到STM32
     * @param data 待发送字节缓冲区
     * @param size 数据长度
     * @param timeout_ms 发送超时毫秒
     * @return 发送结果，失败返回错误信息
     */
    [[nodiscard]] std::expected<void, std::string> send_sync(
        const uint8_t* data, const size_t size, const unsigned timeout_ms = 500) const noexcept {
        // 互斥锁保护USB写操作，多线程发送串行化
        std::lock_guard lock(device_mtx_);
        // 设备未连接直接失败
        if (state_.load(std::memory_order_acquire) != State::Connected || !handle_) {
            return std::unexpected(
                fmt::format(
                    "send_sync(vendor_id={:#06x}, product_id={:#06x}): device not connected", vid_,
                    pid_));
        }
        int actual    = 0;
        // 同步批量输出传输
        const auto rc = libusb_bulk_transfer(
            handle_, EP_OUT, const_cast<uint8_t*>(data), static_cast<int>(size), &actual,
            timeout_ms);
        if (rc != LIBUSB_SUCCESS) {
            return std::unexpected(
                fmt::format(
                    "libusb_bulk_transfer(endpoint={:#04x}, size={}, timeout={}ms): {}", EP_OUT,
                    size, timeout_ms, libusb_error_text(rc)));
        }
        // 实际写入字节不足，视为发送失败
        if (actual != static_cast<int>(size)) {
            return std::unexpected(
                fmt::format(
                    "libusb_bulk_transfer: incomplete write (expected {}, wrote {})", size,
                    actual));
        }
        return {};
    }

    /**
     * @brief 查询设备是否处于正常连接状态
     */
    bool is_connected() const noexcept {
        return state_.load(std::memory_order_acquire) == State::Connected;
    }

private:
    // ====================== 内部状态机定义 ======================
    enum class State {
        Disconnected,   // 断开
        Connected,      // 正常连接
        NeedsReconnect  // 断线，等待重连
    };

    // ====================== 私有内部工具函数 ======================
    /**
     * @brief 根据VID/PID打开USB设备、抢占接口
     */
    [[nodiscard]] std::expected<void, std::string>
        open_device(const uint16_t vid, const uint16_t pid) noexcept {
        // 打开设备句柄
        handle_ = libusb_open_device_with_vid_pid(ctx_, vid, pid);
        if (!handle_) {
            return std::unexpected(
                fmt::format(
                    "libusb_open_device_with_vid_pid(vendor_id={:#06x}, product_id={:#06x}): {}",
                    vid, pid, libusb_error_text(LIBUSB_ERROR_NO_DEVICE)));
        }

        // 自动脱离内核驱动，避免占用设备
        libusb_set_auto_detach_kernel_driver(handle_, 1);

        // 抢占USB接口，独占设备
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

    /**
     * @brief 初始化异步批量接收传输，循环等待MCU下发数据
     */
    [[nodiscard]] std::expected<void, std::string> setup_transfer() noexcept {
        // 分配libusb异步传输结构体
        rx_transfer_ = libusb_alloc_transfer(0);
        if (!rx_transfer_) {
            return std::unexpected(
                fmt::format(
                    "libusb_alloc_transfer(vendor_id={:#06x}, product_id={:#06x}): {}", vid_, pid_,
                    libusb_error_text(LIBUSB_ERROR_NO_MEM)));
        }

        // 填充异步接收传输配置：缓冲区、回调、用户指针this
        libusb_fill_bulk_transfer(
            rx_transfer_, handle_, EP_IN, rx_buf_.data(), rx_buf_.size(),
            &Stm32Impl::on_rx_complete, this, 0);

        // 提交异步接收传输，后台持续等待MCU数据
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

    /**
     * @brief 注册USB热插拔回调，监听设备拔出/插入事件
     */
    [[nodiscard]] std::expected<void, std::string>
        register_hotplug(const uint16_t vid, const uint16_t pid) noexcept {
        // 平台不支持热插拔能力直接返回失败
        if (!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
            return std::unexpected(
                fmt::format(
                    "libusb hotplug unsupported(vendor_id={:#06x}, product_id={:#06x})", vid, pid));
        }

        // 注册热插拔监听回调
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

    // ====================== 静态回调函数（libusb C风格回调） ======================
    /**
     * @brief USB异步接收完成回调
     * 收到MCU数据、传输失败、取消传输都会触发
     */
    static void LIBUSB_CALL on_rx_complete(libusb_transfer* tr) noexcept {
        auto self = static_cast<Stm32Impl*>(tr->user_data);

        // 传输被主动取消，通知等待线程
        if (tr->status == LIBUSB_TRANSFER_CANCELLED) [[unlikely]] {
            self->notify_transfer_done();
            return;
        }

        // 设备已断开，放弃处理
        if (self->state_.load(std::memory_order_acquire) != State::Connected) [[unlikely]] {
            self->notify_transfer_done();
            return;
        }

        // 传输异常失败，标记需要重连
        if (tr->status != LIBUSB_TRANSFER_COMPLETED) [[unlikely]] {
            SPDLOG_WARN("usb rx transfer failed (status={})", static_cast<int>(tr->status));
            self->state_.store(State::NeedsReconnect, std::memory_order_release);
            self->notify_transfer_done();
            return;
        }

        // 有效接收数据，交给Parser解析（首次数据包跳过，过滤上电乱码）
        if (tr->actual_length > 0 && !self->first_rx_) [[likely]] {
            Parser::parse(std::span(reinterpret_cast<std::byte*>(tr->buffer), tr->actual_length));
        }
        self->first_rx_ = false;

        // 重新提交异步接收，持续等待下一包数据
        if (libusb_submit_transfer(tr) != LIBUSB_SUCCESS) {
            SPDLOG_WARN("usb rx resubmit failed");
            self->state_.store(State::NeedsReconnect, std::memory_order_release);
            self->notify_transfer_done();
        }
    }

    /**
     * @brief USB热插拔事件回调：设备拔出/重新插入触发
     */
    static int LIBUSB_CALL on_hotplug(
        libusb_context*, libusb_device*, const libusb_hotplug_event event, void* user) noexcept {
        auto self = static_cast<Stm32Impl*>(user);
        // 设备拔出，标记重连状态
        if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
            SPDLOG_INFO(
                "usb mcu disconnected (vendor_id={:#06x}, product_id={:#06x})", self->vid_,
                self->pid_);
            self->state_.store(State::NeedsReconnect, std::memory_order_release);
        }
        // 设备重新插入，延时等待USB枚举完成
        else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
            if (self->state_.load(std::memory_order_acquire) == State::NeedsReconnect) {
                SPDLOG_INFO("usb mcu re-appeared, waiting for enumeration...");
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
        }
        return 0;
    }

    // ====================== 资源同步与释放工具 ======================
    /**
     * @brief 标记异步传输完成，唤醒等待的条件变量
     */
    void notify_transfer_done() noexcept {
        std::lock_guard lock(mtx_);
        transfer_active_ = false;
        cv_.notify_one();
    }

    /**
     * @brief 阻塞等待异步传输回调执行完毕，用于安全释放传输资源
     */
    void wait_transfer_done() noexcept {
        std::unique_lock lock(mtx_);
        while (transfer_active_) {
            lock.unlock();
            // 轮询libusb事件，防止死等
            timeval tv{0, 10000};
            libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);
            lock.lock();
        }
    }

    /**
     * @brief 外层调用关闭设备（带锁）
     */
    void close_device() noexcept {
        std::lock_guard lock(device_mtx_);
        close_device_locked();
    }

    /**
     * @brief 无锁内部关闭USB设备、销毁异步传输
     */
    void close_device_locked() noexcept {
        if (rx_transfer_) {
            bool should_cancel = false;
            {
                std::lock_guard lock(mtx_);
                should_cancel = transfer_active_;
            }
            // 取消正在运行的异步接收传输
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
        // 释放USB接口占用，关闭设备句柄
        if (handle_) {
            libusb_release_interface(handle_, INTERFACE_NUM);
            libusb_close(handle_);
            handle_ = nullptr;
        }
    }

    /**
     * @brief 断线自动重连逻辑：关闭旧设备，重新打开、初始化异步接收
     */
    void reconnect() noexcept {
        SPDLOG_INFO("usb mcu reconnecting (vendor_id={:#06x}, product_id={:#06x})...", vid_, pid_);
        std::lock_guard lock(device_mtx_);
        close_device_locked();
        // 重新打开设备
        if (auto open_result = open_device(vid_, pid_); !open_result) {
            SPDLOG_WARN("usb mcu reconnect failed: {}", open_result.error());
            return;
        }
        // 重建异步接收传输
        if (auto setup_result = setup_transfer(); !setup_result) {
            SPDLOG_WARN("usb mcu reconnect setup failed: {}", setup_result.error());
            return;
        }
        first_rx_ = true;
        state_.store(State::Connected, std::memory_order_release);
        SPDLOG_INFO("usb mcu reconnected (vendor_id={:#06x}, product_id={:#06x})", vid_, pid_);
    }

    /**
     * @brief 完整销毁USB上下文、注销热插拔、关闭设备
     */
    void shutdown() noexcept {
        const auto previous_state = state_.exchange(State::Disconnected, std::memory_order_acq_rel);
        if (previous_state == State::Disconnected && !ctx_) {
            return;
        }
        SPDLOG_INFO("usb mcu shutting down");
        // 注销热插拔监听
        if (hp_handle_ && ctx_) {
            libusb_hotplug_deregister_callback(ctx_, hp_handle_);
            hp_handle_ = 0;
        }
        close_device();
        // 释放libusb全局上下文
        if (ctx_) {
            libusb_exit(ctx_);
            ctx_ = nullptr;
        }
    }

    /**
     * @brief 扫描USB设备列表，根据VID自动匹配唯一PID
     * 多个同VID设备/无设备返回错误
     */
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

        // RAII自动释放设备列表
        scoped_exit cleanup{[&] { libusb_free_device_list(list, 1); }};

        uint16_t found_pid = 0;
        int count          = 0;

        // 遍历所有USB设备，筛选匹配VID
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
            // 匹配到第二个设备，多设备冲突直接报错
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

    // ====================== 成员变量 ======================
    libusb_context* ctx_                      = nullptr;        // libusb全局上下文
    libusb_device_handle* handle_             = nullptr;        // USB设备句柄
    libusb_transfer* rx_transfer_             = nullptr;        // 异步接收传输结构体
    libusb_hotplug_callback_handle hp_handle_ = 0;              // 热插拔回调句柄

    std::array<uint8_t, 64> rx_buf_{};                         // 64字节USB接收缓冲区
    uint16_t vid_ = 0;                                         // 缓存设备厂商ID
    uint16_t pid_ = 0;                                         // 缓存设备产品ID

    std::atomic<State> state_{State::Disconnected};            // 无锁状态机
    bool first_rx_ = true;                                     // 标记首次接收包，过滤上电乱码

    mutable std::mutex device_mtx_;                            // 发送接口互斥锁，保护设备句柄

    // 异步传输销毁同步锁+条件变量
    std::mutex mtx_;
    std::condition_variable cv_;
    bool transfer_active_ = false;                             // 异步传输是否正在运行
};
// 编译期校验：满足Device概念约束，可被上层McuDeviceHandle variant托管
static_assert(Device<Stm32Impl<Stm32Parser>>);

} // namespace talos_gimbal