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
    * @brief 连接自定义Bulk USB单片机设备（libusb主机端代码，运行在电脑侧）
    * [[nodiscard]]：必须处理返回值，不能忽略错误
    * @param vendor_id 厂商VID，默认使用类内部宏VID
    * @param product_id 产品PID，std::optional，可以不传；不传就自动搜索该VID下唯一设备
    * @return std::expected<void, std::string> void代表成功；string存放错误描述
    * @noexcept 不抛C++异常，全部用std::expected返回错误
    */
    [[nodiscard]] std::expected<void, std::string> connect(
            const uint16_t vendor_id           = VID,
            std::optional<uint16_t> product_id = std::nullopt) noexcept {

            // ctx_：libusb上下文句柄，整个libusb会话的环境，一个程序一般一个ctx
            if (!ctx_) {
                // 初始化libusb上下文
                /*1. `libusb_context **ctx`
                输出参数，返回 libusb 上下文指针。
                > **libusb_context：整个 libusb 会话的全局环境**。
                > 设备列表、事件轮询、异步 transfer、热插拔全部绑定在这个 ctx 上。
                > 一个进程一般创建**一个 ctx 就够**，不要反复创建销毁。
                2. `const libusb_init_option *options`：初始化选项数组，不需要就传`nullptr`
                3. `int num_options`：选项数组元素个数，没选项填`0`*/
                const auto rc = libusb_init_context(&ctx_, nullptr, 0);
                if (rc != LIBUSB_SUCCESS) {
                    // 初始化失败，包装错误文本返回
                    return std::unexpected(
                        fmt::format("libusb_init_context: {}", libusb_error_text(rc)));
                }
            }

            // ---------------- 如果外部没有传入PID：自动扫描USB总线，找这个VID下面的设备 ----------------
            if (!product_id) {
                // find_device：遍历本机全部USB设备，给定VID，返回匹配的PID；找不到返回error
                auto result = find_device(ctx_, vendor_id);
                if (!result) {
                    return std::unexpected(
                        fmt::format(
                            "find usb device(vendor_id={:#06x}): {}", vendor_id, result.error()));
                }
                // 拿到搜索出来的PID，存入optional
                product_id = *result;
            }

            // 保存本次连接使用的VID PID到成员变量
            vid_ = vendor_id;
            pid_ = *product_id;

            // ---------------- 根据VID+PID打开USB设备，拿到设备句柄 ----------------
            if (auto result = open_device(vendor_id, *product_id); !result) {
                // 打开失败（设备不存在、权限不足、被其他程序占用）直接返回错误
                return std::unexpected(std::move(result).error());
            }

            // ---------------- 设置异步Bulk传输，准备接收单片机上传的数据(EP‑IN 0x81) ----------------
            // libusb异步transfer：不会阻塞线程，底层提交USB请求，有数据自动触发回调
            if (auto result = setup_transfer(); !result) {
                return std::unexpected(std::move(result).error());
            }

            // ---------------- 注册热插拔回调：设备拔下来、重新插上自动触发回调函数 ----------------
            if (auto result = register_hotplug(vendor_id, *product_id); !result) {
                return std::unexpected(std::move(result).error());
            }

            first_rx_ = true; // 标记：还没有收到第一帧数据

            // 原子变量修改连接状态：多线程安全，标记设备为Connected已连接
            state_.store(State::Connected, std::memory_order_release);

            SPDLOG_INFO("usb mcu connected (vendor_id={:#06x}, product_id={:#06x})", vid_, pid_);
            // std::expected<void>成功返回空对象
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
    * @brief setup_transfer：配置并提交Bulk‑IN异步接收传输
    * 作用：向libusb提交一个后台接收请求，等待单片机从EP_IN(0x81)发来数据包
    * 这是libusb异步模型核心：提交一次transfer，回调触发后通常要再次提交，实现持续收数据
    * @return std::expected<void,std::string> 成功空；失败携带错误信息
    * @noexcept 不抛C++异常
    */
    [[nodiscard]] std::expected<void, std::string> setup_transfer() noexcept {
            //----------------------------------------------------------------------
            // API：libusb_alloc_transfer
            // 原型：libusb_transfer *libusb_alloc_transfer(int iso_packets);
            // 参数 iso_packets：等时包数量；Bulk传输填0
            // 功能：在堆上分配一个 libusb_transfer 异步传输结构体
            // 这个结构体保存端点、缓冲区、回调函数、用户自定义指针等全部传输上下文
            // 返回 nullptr = 内存分配失败
            // 注意：分配出来必须手动 libusb_free_transfer 释放，否则内存泄漏
            //----------------------------------------------------------------------
            rx_transfer_ = libusb_alloc_transfer(0);
            if (!rx_transfer_) {
                return std::unexpected(
                    fmt::format(
                        "libusb_alloc_transfer(vendor_id={:#06x}, product_id={:#06x}): {}", vid_, pid_,
                        libusb_error_text(LIBUSB_ERROR_NO_MEM)));
            }

            //----------------------------------------------------------------------
            // API：libusb_fill_bulk_transfer
            // 原型：
            // void libusb_fill_bulk_transfer(
            //     libusb_transfer *transfer,
            //     libusb_device_handle *dev_handle,
            //     unsigned char endpoint,
            //     unsigned char *buffer,
            //     int length,
            //     libusb_transfer_cb_fn callback,
            //     void *user_data,
            //     unsigned int timeout
            // );
            // 作用：给transfer结构体填充Bulk批量传输的全部参数，不用手动去写结构体每个字段
            // 参数逐个解释：
            // 1. rx_transfer_：刚刚分配好的transfer对象
            // 2. handle_：libusb设备打开后的句柄 open_device得到
            // 3. EP_IN：输入端点地址，本例0x81（MCU → PC）
            // 4. rx_buf_.data()：接收缓冲区，收到的数据会写到这块内存
            // 5. rx_buf_.size()：缓冲区字节大小（USB2.0 Bulk全速最大64字节）
            // 6. &Stm32Impl::on_rx_complete：传输完成回调函数指针，中断/事件线程调用
            // 7. this：user_data 用户自定义指针；回调里面会把这个指针传回来，拿到类实例
            // 8. timeout：超时毫秒；填0代表**永不超时**，一直等待数据包到来
            //----------------------------------------------------------------------
            libusb_fill_bulk_transfer(
                rx_transfer_, handle_, EP_IN, rx_buf_.data(), rx_buf_.size(),
                &Stm32Impl::on_rx_complete, this, 0);

            //----------------------------------------------------------------------
            // API：libusb_submit_transfer
            // 原型：int libusb_submit_transfer(libusb_transfer *transfer);
            // 功能：把填好参数的transfer提交给libusb内核，进入后台排队
            // ⚠️提交之后，控制权交给libusb，**本函数不会阻塞！立刻返回**
            // 当：收到数据包 / 设备拔出 / 出错，就会触发你设置的回调 on_rx_complete
            // 返回值：LIBUSB_SUCCESS 提交成功；负数错误码提交失败
            // 常见提交失败原因：接口没有claim、端点号错误、设备已经断开
            //----------------------------------------------------------------------
            const auto submit_rc = static_cast<libusb_error>(libusb_submit_transfer(rx_transfer_));
            if (submit_rc != LIBUSB_SUCCESS) {
                // 提交失败，必须手动释放transfer，防止内存泄漏
                libusb_free_transfer(rx_transfer_);
                rx_transfer_ = nullptr;
                return std::unexpected(
                    fmt::format(
                        "libusb_submit_transfer(endpoint={:#04x}, vendor_id={:#06x}, "
                        "product_id={:#06x}): {}",
                        EP_IN, vid_, pid_, libusb_error_text(submit_rc)));
            }

            // 加锁标记：异步transfer已经提交激活，多线程访问transfer_active_需要互斥
            {
                std::lock_guard lock(mtx_);
                transfer_active_ = true;
            }

            return {};
    }


    /**
    * @brief register_hotplug 注册USB热插拔回调
    * 作用：监听目标VID/PID设备：设备插上(ARRIVED)、设备拔下(LEFT)触发回调on_hotplug
    * @param vid 要监听的厂商ID
    * @param pid 要监听的产品ID
    * @return std::expected<void,std::string> 成功返回空；失败携带错误信息
    * @noexcept 不抛出C++异常
    */
    [[nodiscard]] std::expected<void, std::string>
    register_hotplug(const uint16_t vid, const uint16_t pid) noexcept {
        // 查询libusb库当前运行环境是否支持热插拔回调能力
        if (!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
            // 当前平台不支持热插拔，返回错误信息
            return std::unexpected(
                fmt::format(
                    "libusb hotplug unsupported(vendor_id={:#06x}, product_id={:#06x})", vid, pid));
        }

        // 调用libusb接口注册热插拔回调，监听设备插入、拔出事件
        auto rc = libusb_hotplug_register_callback(
            ctx_,                                               // libusb全局上下文对象
            static_cast<libusb_hotplug_event>(               // 需要监听的事件掩码
                LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT |                  // 监听设备拔出事件
                LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED),               // 监听设备插入事件
            LIBUSB_HOTPLUG_NO_FLAGS,                         // 无额外热插拔标志位
            vid,                                         // 过滤：只监听该厂商VID的设备
            pid,                                        // 过滤：只监听该产品PID的设备
            LIBUSB_HOTPLUG_MATCH_ANY,                    // 设备类别不做过滤，全部匹配
            &Stm32Impl::on_hotplug,                          // 热插拔触发的静态回调函数
            this,                                        // 用户自定义指针，回调会回传该类实例指针
            &hp_handle_);                          // 输出：热插拔句柄，注销回调时使用

        // 判断热插拔回调是否注册成功
        if (rc == LIBUSB_SUCCESS) {
            // 注册成功，std::expected<void>返回成功
            return {};
        }
        // 注册失败，把libusb错误码转为文本，封装到std::unexpected返回
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
    * @brief 扫描本机全部USB设备，根据VID查找设备PID
    * @param ctx libusb全局上下文
    * @param vid 要匹配的USB厂商ID
    * @return std::expected<uint16_t, std::string>
    *         成功：返回找到的PID；失败返回错误字符串
    * @noexcept 不抛出C++异常，全部通过expected返回错误
    * @static 静态工具函数，不依赖类成员
    */
    [[nodiscard]] static std::expected<uint16_t, std::string>
    find_device(libusb_context* ctx, const uint16_t vid) noexcept {
        // list：输出参数，接收libusb填充的设备指针数组
        libusb_device** list;

        /**
        * API: libusb_get_device_list
        * 函数原型：ssize_t libusb_get_device_list(libusb_context *ctx, libusb_device ***list);
        * 功能：扫描USB总线，获取当前系统上**所有USB设备**，写入list数组
        * 返回值：成功返回设备数量(>=0)；失败返回负数libusb错误码
        * 注意：list是libusb在堆上分配的数组！用完必须调用 libusb_free_device_list 释放，否则内存泄漏
        */
        auto cnt = libusb_get_device_list(ctx, &list);
        if (cnt < 0) {
            return std::unexpected(
                fmt::format(
                    "libusb_get_device_list(vendor_id={:#06x}): {}", vid,
                    libusb_error_text(static_cast<int>(cnt))));
        }

        /**
        * scoped_exit RAII守卫（自定义工具类）
        * 离开作用域的时候自动执行lambda：libusb_free_device_list(list, 1)
        * 第二个参数 1：同时增加所有设备引用计数递减；这是标准用法。
        * 不管函数正常return、异常return，一定会执行释放，防止漏写free造成内存泄漏。
        */
        scoped_exit cleanup{[&] { libusb_free_device_list(list, 1); }};

        uint16_t found_pid = 0; // 保存找到的产品PID
        int count          = 0; // 统计匹配该VID的设备数量

        // 遍历全部扫描出来的USB设备
        for (ssize_t i = 0; i < cnt; ++i) {
            libusb_device_descriptor desc{}; // USB设备描述符结构体，存放vid/pid、版本等信息

            /**
            * API: libusb_get_device_descriptor
            * 原型：int libusb_get_device_descriptor(libusb_device *dev, libusb_device_descriptor *desc);
            * 功能：从libusb_device设备句柄读取【设备描述符】，输出到desc结构体
            * 设备描述符：每个USB设备都有，固定包含 idVendor(VID), idProduct(PID)
            * 返回：LIBUSB_SUCCESS(0)成功；负数错误码。
            */
            const auto desc_rc =
                static_cast<libusb_error>(libusb_get_device_descriptor(list[i], &desc));
            // 获取描述符失败，跳过这个设备
            if (desc_rc != LIBUSB_SUCCESS) {
                continue;
            }

            // VID不匹配，跳过
            if (desc.idVendor != vid)
                continue;

            // VID匹配上，记录PID，计数+1
            found_pid = desc.idProduct;
            ++count;

            // 业务逻辑：同一个VID下**不能插2个同厂商设备**，检测到多个直接报错
            if (count > 1) {
                return std::unexpected(
                    fmt::format(
                        "find usb device(vendor_id={:#06x}): multiple product ids matched", vid));
            }
        }

        // 恰好找到1台设备，返回PID
        if (count == 1) {
            return found_pid;
        }
        // count == 0，没有找到任何匹配VID的USB设备
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