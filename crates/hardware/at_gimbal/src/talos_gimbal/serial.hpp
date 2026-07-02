#pragma once
// 头文件保护，防止重复包含

#include "usb.hpp" // 工程USB公共头，本文件未实际使用预留

// STL标准库
#include <algorithm>    // std::find std::exchange 缓存查找/移动赋值
#include <cerrno>      // 全局错误码 errno
#include <cstdint>     // 固定宽度整数 uint8_t uint32_t
#include <cstring>     // strerror 错误文本转换
#include <expected>    // C++23 标准预期类型，正常/错误分离，替代异常
#include <string>      // 设备路径、错误信息字符串
#include <vector>      // 接收环形缓存 rx_buf_

// POSIX Linux 终端系统调用头
#include <fcntl.h>     // open O_RDWR O_NONBLOCK 打开文件标志
#include <termios.h>   // 串口配置结构体 termios 波特率/流控/原始模式
#include <unistd.h>    // close read write tcflush 基础IO系统调用

// IO多路复用 poll 非阻塞事件监听
#include <poll.h>

// 日志库 spdlog
#include <spdlog/spdlog.h>

namespace talos_gimbal {

// ============================================================================
// FileDescriptor — RAII 封装POSIX文件句柄，自动管理串口fd生命周期
// 核心：离开作用域自动close，禁止拷贝，仅支持移动语义，杜绝句柄泄漏
// ============================================================================
class FileDescriptor {
public:
    // 默认构造：无效句柄 -1
    FileDescriptor() noexcept = default;

    // 传入有效文件描述符构造
    explicit FileDescriptor(int fd) noexcept
        : fd_(fd) {}

    // 析构：存在有效fd则调用系统close释放资源
    ~FileDescriptor() noexcept {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    // 拷贝构造函数 = delete：禁用对象拷贝
    //函数声明 = delete;，C++11 引入，作用：显式告诉编译器：这个函数不允许被调用，一旦调用直接编译报错。
    FileDescriptor(const FileDescriptor&)            = delete;
    // 拷贝赋值运算符 = delete：禁用对象赋值拷贝
    FileDescriptor& operator=(const FileDescriptor&) = delete;

    // 移动构造函数：接管另一个 FileDescriptor 的文件句柄，转移所有权
    // noexcept：保证不会抛出异常，符合资源移动规范
    FileDescriptor(FileDescriptor&& other) noexcept
        // 先把对方持有的 fd 复制到当前对象
        : fd_(other.fd_)
    {
        // 将原对象的 fd 置为无效值 -1，原对象不再拥有句柄
        other.fd_ = -1;
    }

    // 移动赋值运算符：转移句柄所有权，同时释放自身旧资源
    FileDescriptor& operator=(FileDescriptor&& other) noexcept {
        // 防止自移动：fd = std::move(fd) 这种非法操作
        if (this != &other) {
            // 如果当前对象本身持有合法文件句柄，先关闭，避免资源泄漏
            if (fd_ >= 0) {
                ::close(fd_);
            }
            // 接管传入对象的文件描述符
            fd_       = other.fd_;
            // 原对象失去所有权，标记为无效
            other.fd_ = -1;
        }
        return *this;
    }

    // 获取原始fd，只读不转移所有权
    [[nodiscard]] int get() const noexcept { return fd_; }
    // 判断句柄是否有效 >=0
    [[nodiscard]] bool valid() const noexcept { return fd_ >= 0; }

    // 释放所有权：返回fd并将自身置空，外部接管关闭责任
    int release() noexcept { return std::exchange(fd_, -1); }

private:
    // 文件描述符，默认-1无效
    int fd_ = -1;
};

// ============================================================================
// SerialImpl 串口核心实现模板类
// 模板参数 Parser：外部传入二进制帧解析器，解耦串口IO与业务协议解析
// 特性：非阻塞接收、带超时同步发送、poll事件轮询、自动帧拆分、RAII安全
// ============================================================================
template <Parser Parser>
class SerialImpl {
public:
    // 默认构造
    SerialImpl() = default;

    // 析构：FileDescriptor成员自动释放串口，无需手动写关闭逻辑
    ~SerialImpl() noexcept = default;

    // 禁用拷贝
    SerialImpl(const SerialImpl&)            = delete;
    SerialImpl& operator=(const SerialImpl&) = delete;

    // 移动构造：转移句柄、设备路径、接收缓存所有权
    SerialImpl(SerialImpl&& other) noexcept
        : fd_(std::move(other.fd_))
        , device_path_(std::move(other.device_path_))
        , rx_buf_(std::move(other.rx_buf_)) {}

    // 移动赋值
    SerialImpl& operator=(SerialImpl&& other) noexcept {
        if (this != &other) {
            fd_          = std::move(other.fd_);
            device_path_ = std::move(other.device_path_);
            rx_buf_      = std::move(other.rx_buf_);
        }
        return *this;
    }

    /**
     * @brief 打开并配置串口设备
     * @param device_path 串口路径 默认 /dev/ttyS4
     * @param baud_rate 波特率 默认115200
     * @return std::expected<void, string> 成功无值，失败携带错误字符串
     * @noexcept 全程无抛出异常，错误通过返回值传递
     */
    [[nodiscard]] std::expected<void, std::string>
        connect(const std::string& device_path = "/dev/ttyS4", int baud_rate = 115200) noexcept {
        // 打开串口：读写 | 不分配控制终端 | 非阻塞IO
        auto fd = ::open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            // 打开失败，返回格式化错误信息（设备路径+系统错误描述）
            return std::unexpected(
                fmt::format("open serial device(path={}): {}", device_path, std::strerror(errno)));
        }

        termios tty{};
        // 读取当前终端配置
        if (::tcgetattr(fd, &tty) != 0) {
            // fd尚未交给RAII托管，手动关闭防止句柄泄漏
            ::close(fd);
            return std::unexpected(
                fmt::format("tcgetattr serial device(path={}): {}", device_path, std::strerror(errno)));
        }

        // 配置为原始串口模式：关闭所有终端加工（回显、换行转换、信号）
        ::cfmakeraw(&tty);

        // 串口标准 8N1 配置
        tty.c_cflag &= ~CSTOPB;        // 1停止位
        tty.c_cflag &= ~CRTSCTS;       // 关闭硬件流控RTS/CTS
        tty.c_cflag |= CLOCAL | CREAD; // 本地模式、启用接收

        // 关闭软件XON/XOFF流控
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);

        // 读超时配置：VMIN=0 VTIME=0.1s
        // 无数据不阻塞，有任意字节立即返回
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;

        // 转换波特率为POSIX speed_t常量
        speed_t speed = baud_to_speed(baud_rate);
        ::cfsetispeed(&tty, speed); // 输入波特率
        ::cfsetospeed(&tty, speed); // 输出波特率

        // 应用终端配置 TCSANOW 立即生效
        if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
            ::close(fd);
            return std::unexpected(
                fmt::format("tcsetattr serial device(path={}, baud_rate={}): {}",
                            device_path, baud_rate, std::strerror(errno)));
        }

        // 清空输入输出缓冲区残留数据
        ::tcflush(fd, TCIOFLUSH);
        // 转移fd所有权到RAII容器
        fd_          = FileDescriptor(fd);
        device_path_ = device_path;
        SPDLOG_INFO("serial mcu connected (path={}, baud_rate={})", device_path, baud_rate);
        return {};
    }

    /**
     * @brief 断开串口，RAII自动关闭fd
     */
    void disconnect() noexcept { fd_ = FileDescriptor(-1); }

    /**
     * @brief 串口事件轮询入口，外部定时调用（主线程/IO线程）
     * 逻辑：poll监听可读事件 → 批量读取存入缓存 → 循环解析完整帧交给Parser
     */
    void handle_events() noexcept {
        // 未连接直接返回
        if (!fd_.valid())
            return;

        // poll配置：监听POLLIN可读事件，超时0 非阻塞
        struct pollfd pfd{.fd = fd_.get(), .events = POLLIN, .revents = 0};
        // 循环读取所有就绪数据，避免单次读取缓冲区残留
        while (::poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
            uint8_t tmp[512]; // 单次最大读取512字节
            auto n = ::read(fd_.get(), tmp, sizeof(tmp));
            if (n > 0) {
                // 读取到数据追加至接收缓存
                rx_buf_.insert(rx_buf_.end(), tmp, tmp + n);
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                // 真正IO错误（非无数据），打印日志并断开串口
                SPDLOG_WARN("serial read error(path={}): {}", device_path_, std::strerror(errno));
                disconnect();
                return;
            }
        }

        // 循环解析：一帧解析完成后继续处理剩余缓存，直到无完整帧
        while (try_parse_frame()) {}
    }

    /**
     * @brief 同步阻塞发送接口，带超时机制
     * @param data 待发送字节数组
     * @param size 数据长度
     * @param timeout_ms 发送超时 单位ms 默认500
     * @return 成功空值，失败错误字符串
     */
    [[nodiscard]] std::expected<void, std::string> send_sync(
        const uint8_t* data, const size_t size, const unsigned timeout_ms = 500) const noexcept {
        if (!fd_.valid()) {
            return std::unexpected(fmt::format("send_sync(path={}): device not connected", device_path_));
        }

        size_t written      = 0; // 已发送字节计数
        // 获取单调时钟绝对截止时间（不受系统时间修改影响）
        const auto deadline = [] {
            struct timespec ts;
            ::clock_gettime(CLOCK_MONOTONIC, &ts);
            return ts;
        }();
        // 转换为纳秒时间戳
        const auto deadline_ns = deadline.tv_sec * 1'000'000'000LL + deadline.tv_nsec
                               + static_cast<long long>(timeout_ms) * 1'000'000LL;

        // 循环发送直到全部字节写完
        while (written < size) {
            // poll监听可写POLLOUT，单次等待10ms
            struct pollfd pfd{.fd = fd_.get(), .events = POLLOUT, .revents = 0};
            if (::poll(&pfd, 1, 10) <= 0) {
                // poll无就绪，判断是否超时
                struct timespec now;
                ::clock_gettime(CLOCK_MONOTONIC, &now);
                if (now.tv_sec * 1'000'000'000LL + now.tv_nsec >= deadline_ns) {
                    return std::unexpected(
                        fmt::format("serial write timeout(path={}, timeout={}ms, written {}/{}): exceeded deadline",
                                    device_path_, timeout_ms, written, size));
                }
                continue;
            }

            // 缓冲区可写，执行write
            auto n = ::write(fd_.get(), data + written, size - written);
            if (n < 0) {
                // 非阻塞缓冲区满，重试；其他错误返回失败
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    continue;
                return std::unexpected(
                    fmt::format("serial write(path={}): {}", device_path_, std::strerror(errno)));
            }
            written += static_cast<size_t>(n);
        }

        // 阻塞等待所有数据物理发送完成，清空输出缓冲
        ::tcdrain(fd_.get());
        return {};
    }

    // 判断串口是否正常连接
    bool is_connected() const noexcept { return fd_.valid(); }

private:
    /**
     * @brief 波特率数字转POSIX标准speed_t枚举
     * @param baud 输入波特率整数
     * @return termios所需速度常量，不匹配默认115200
     */
    static speed_t baud_to_speed(int baud) noexcept {
        switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B115200;
        }
    }

    /**
     * @brief 从rx_buf_缓存中尝试解析一帧完整协议数据
     * 协议帧格式：
     * HeaderFrame(3字节帧头) + uint32_t时间戳(4) + payload(可变长header->len) + EoF单字节尾
     * @return true 解析出一帧，缓存截断；false 无完整帧等待更多数据
     */
    bool try_parse_frame() noexcept {
        // 1. 查找帧起始标志SoF
        auto sof_it = std::find(rx_buf_.begin(), rx_buf_.end(), HeaderFrame::SoF());
        if (sof_it == rx_buf_.end()) {
            // 无帧头，清空全部脏数据
            rx_buf_.clear();
            return false;
        }

        // 丢弃帧头前的乱码垃圾字节
        if (sof_it != rx_buf_.begin()) {
            rx_buf_.erase(rx_buf_.begin(), sof_it);
        }

        // 缓存长度不足帧头，等待后续数据
        if (rx_buf_.size() < sizeof(HeaderFrame))
            return false;

        const auto* header = reinterpret_cast<const HeaderFrame*>(rx_buf_.data());
        // 校验帧头魔数，不匹配删除首字节重新查找
        if (header->sof != HeaderFrame::SoF()) {
            rx_buf_.erase(rx_buf_.begin());
            return true;
        }

        // 计算整帧总字节长度
        const size_t frame_size = sizeof(HeaderFrame) + sizeof(uint32_t) + header->len + 1;

        // 缓存数据不足完整一帧，等待下次读取
        if (rx_buf_.size() < frame_size)
            return false;

        // 校验末尾EoF结束标志
        if (rx_buf_[frame_size - 1] != HeaderFrame::EoF()) {
            rx_buf_.erase(rx_buf_.begin());
            return true;
        }

        // 帧校验全部通过，调用外部模板Parser解析业务数据
        Parser::parse(
            std::span<const std::byte>(
                reinterpret_cast<const std::byte*>(rx_buf_.data()), frame_size));

        // 删除已解析完成的帧字节
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + static_cast<ptrdiff_t>(frame_size));
        return true;
    }

    // RAII串口文件句柄
    FileDescriptor fd_;
    // 串口设备路径 /dev/ttyXX
    std::string device_path_;
    // 接收环形缓存，存储未解析原始字节流
    std::vector<uint8_t> rx_buf_;
};

} // namespace talos_gimbal