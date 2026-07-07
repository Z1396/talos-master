#pragma once
// 帧结构体 Frame 定义
#include "L1_sensor/parcel.hpp"
// 相机参数配置结构体 CameraConfig
#include "camera_config.hpp"
// 海康相机采集封装类 ImageCapturer、相机错误类型 CameraError
#include "hik_camera.hpp"
// 共享内存IPC客户端 ShmClient、ShmError 错误类型
#include "shm_client.hpp"

// 时间单位 std::chrono::milliseconds / micro
#include <chrono>
// std::expected 承载帧/错误
#include <expected>
// fmt格式化日志核心
#include <fmt/core.h>
// fmt chrono时间格式化支持
#include <fmt/std.h>
// 独占/共享智能指针
#include <memory>
// std::optional 无阻塞接收空帧标记
#include <optional>
// 字符串，错误上下文、设备名称
#include <string>
// std::variant 多类型错误、多输入模式存储
#include <variant>

namespace fcs::L1 {

/**
 * @brief 接收超时错误标记空结构体
 */
struct Timeout {};

/**
 * @brief 连接断开错误
 * @param ctx 断开上下文描述文本
 */
struct Disconnected {
    std::string ctx;
};

/**
 * @brief 共享内存生产者（图像发送端）未启动/不存在
 */
struct ProducerNotAvailable {};

/**
 * @brief 输入统一错误变体类型，容纳所有图像读取失败场景
 * 包含：超时、连接断开、生产者不存在、海康相机硬件错误、共享内存IPC错误
 */
using InputError = std::variant<
    Timeout, Disconnected, ProducerNotAvailable, hikcamera::CameraError, ipc::ShmError>;

// ============================================================================
// IpcInput 共享内存图像输入实现类
// 从IPC共享内存读取其他进程输出的图像帧
// ============================================================================
class IpcInput {
public:
    /**
     * @brief 构造IPC图像输入
     * @param client 共享内存客户端共享指针
     * @param info 相机标定/分辨率等配置信息
     * @ noexcept 无抛异常
     */
    explicit IpcInput(std::shared_ptr<ipc::ShmClient> client, CameraConfig info) noexcept;

    /**
     * @brief 析构，释放共享内存客户端资源
     */
    ~IpcInput();

    // 禁用拷贝构造、拷贝赋值，共享内存资源不可复制
    IpcInput(const IpcInput&)            = delete;
    IpcInput& operator=(const IpcInput&) = delete;

    // 允许移动构造、移动赋值，资源所有权转移
    IpcInput(IpcInput&&) noexcept            = default;
    IpcInput& operator=(IpcInput&&) noexcept = default;

    /**
     * @brief 无阻塞尝试读取一帧图像
     * @return std::optional<Frame> 有帧返回有效对象，无帧返回std::nullopt
     */
    [[nodiscard]] std::optional<Frame> try_recv() const noexcept;

    /**
     * @brief 带超时阻塞读取图像帧
     * @param timeout 阻塞等待超时时间
     * @return 成功：图像帧；失败：InputError错误变体
     */
    [[nodiscard]] std::expected<Frame, InputError>
        recv(std::chrono::milliseconds timeout) const noexcept;

    /**
     * @brief 获取当前绑定相机配置只读引用
     */
    [[nodiscard]] const CameraConfig& camera_info() const noexcept;

private:
    std::shared_ptr<ipc::ShmClient> client_; ///< 共享内存IPC客户端
    CameraConfig info_;                      ///< 相机参数配置
};

// ============================================================================
// HikInput 海康硬件相机直接采集输入类
// 直连海康SDK，硬件采集图像帧
// ============================================================================
class HikInput {
public:
    /**
     * @brief 构造海康相机采集输入
     * @param camera 海康相机采集器独占指针
     * @param info 相机配置
     * @param profile 相机曝光/增益等参数配置模板
     * @param exposure_time 曝光时长（微秒）
     * @param device_name 可选设备标识名，用于日志区分
     */
    explicit HikInput(
        std::unique_ptr<hikcamera::ImageCapturer> camera, CameraConfig info,
        hikcamera::ImageCapturer::CameraProfile profile,
        std::chrono::duration<float, std::micro> exposure_time,
        std::optional<std::string> device_name) noexcept;

    // 禁用拷贝
    HikInput(const HikInput&)            = delete;
    HikInput& operator=(const HikInput&) = delete;

    // 允许移动
    HikInput(HikInput&&) noexcept            = default;
    HikInput& operator=(HikInput&&) noexcept = default;

    /**
     * @brief 无阻塞尝试读取硬件相机帧
     * @return 有帧返回Frame，无帧nullopt
     */
    [[nodiscard]] std::optional<Frame> try_recv() noexcept;

    /**
     * @brief 带超时阻塞读取相机帧，相机断连自动重试重连
     * @param timeout 阻塞超时
     * @return 图像帧 / InputError错误
     */
    [[nodiscard]] std::expected<Frame, InputError> recv(std::chrono::milliseconds timeout) noexcept;

    /**
     * @brief 获取相机配置只读引用
     */
    [[nodiscard]] const CameraConfig& camera_info() const noexcept;

private:
    /**
     * @brief 获取当前系统纳秒级时间戳，用于帧时序、重连计时
     */
    [[nodiscard]] uint64_t now_ns() const noexcept;

    std::unique_ptr<hikcamera::ImageCapturer> camera_; ///< 海康相机采集器实例
    CameraConfig info_;                                 ///< 相机参数
    hikcamera::ImageCapturer::CameraProfile profile_;  ///< 相机硬件参数模板
    uint64_t exposure_time_ns_;                         ///< 曝光时长（纳秒）
    uint64_t seq_{0};                                   ///< 图像帧序列号，自增
    std::optional<std::string> device_name_;            ///< 设备别名（可选）
    static constexpr uint64_t kReconnectRetryLimit = 25565; ///< 最大重连重试计数上限
    uint64_t retry_{kReconnectRetryLimit};              ///< 当前剩余重连重试次数
};

// ============================================================================
// CameraInterface 相机统一顶层接口（策略模式）
// 上层业务仅依赖此类，底层自动区分IPC共享内存/直连海康相机
// ============================================================================
class CameraInterface {
public:
    /// 输入模式变体：二选一 IpcInput / HikInput
    using InputMode = std::variant<IpcInput, HikInput>;

    // 无参构造禁用，必须指定输入模式
    CameraInterface() = delete;
    /**
     * @brief 构造顶层相机接口，传入对应输入实现
     * @param mode IPC或海康相机实例变体
     */
    explicit CameraInterface(InputMode mode) noexcept;

    // 禁用拷贝
    CameraInterface(const CameraInterface&)                = delete;
    CameraInterface& operator=(const CameraInterface&)     = delete;
    // 允许移动
    CameraInterface(CameraInterface&&) noexcept            = default;
    CameraInterface& operator=(CameraInterface&&) noexcept = default;

    /**
     * @brief 无阻塞读取帧，自动转发到底层Ipc/Hik实现
     */
    [[nodiscard]] std::optional<Frame> try_recv() noexcept;

    /**
     * @brief 带超时阻塞读取帧，统一错误类型InputError
     */
    [[nodiscard]] std::expected<Frame, InputError> recv(std::chrono::milliseconds timeout) noexcept;

    /**
     * @brief 获取当前使用相机配置
     */
    [[nodiscard]] const CameraConfig& camera_info() const noexcept;

    /**
     * @brief 静态工厂：创建IPC共享内存模式相机接口
     * @param client 共享内存客户端
     * @return 顶层CameraInterface实例 / 错误
     */
    [[nodiscard]] static std::expected<CameraInterface, InputError>
        create_ipc(std::shared_ptr<ipc::ShmClient> client) noexcept;

    /**
     * @brief 静态工厂：创建直连海康相机模式接口
     * @param config 相机完整配置
     * @return CameraInterface / 相机初始化错误
     */
    [[nodiscard]] static std::expected<CameraInterface, InputError>
        create_hik(const CameraConfig& config) noexcept;

private:
    InputMode mode_; ///< 底层输入实现变体（IPC / Hik）
};

} // namespace fcs::L1

// ============================================================================
// fmt 格式化特化：支持直接打印InputError变体所有错误类型日志
// ============================================================================
namespace fmt {

/**
 * @brief Timeout 错误格式化输出字符串 "timeout"
 */
template <>
struct formatter<fcs::L1::Timeout> : formatter<std::string_view> {
    auto format(const fcs::L1::Timeout, format_context& ctx) const {
        return formatter<std::string_view>::format("timeout", ctx);
    }
};

/**
 * @brief Disconnected 断开错误，输出上下文信息 "disconnected: xxx"
 */
template <>
struct formatter<fcs::L1::Disconnected> : formatter<std::string_view> {
    auto format(const fcs::L1::Disconnected d, format_context& ctx) const {
        return formatter<std::string_view>::format(fmt::format("disconnected: {}", d.ctx), ctx);
    }
};

/**
 * @brief ProducerNotAvailable 生产者不存在打印固定文本
 */
template <>
struct formatter<fcs::L1::ProducerNotAvailable> : formatter<std::string_view> {
    auto format(const fcs::L1::ProducerNotAvailable, format_context& ctx) const {
        return formatter<std::string_view>::format("producer_not_available", ctx);
    }
};

/**
 * @brief InputError 总变体格式化，visit分发到对应错误类型格式化器
 */
template <>
struct formatter<fcs::L1::InputError> {
    // 无格式控制符
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    auto format(const fcs::L1::InputError& err, format_context& ctx) const {
        // std::visit 自动匹配variant内部存储的错误类型，调用对应format
        return std::visit(
            [&ctx](const auto& e) { return fmt::format_to(ctx.out(), "{}", e); }, err);
    }
};

} // namespace fmt