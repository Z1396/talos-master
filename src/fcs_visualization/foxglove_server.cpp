// Foxglove 可视化服务实现头文件
#include "foxglove_server.hpp"

// 项目错误码、异常处理工具
#include <error.hpp>
// Foxglove 底层服务、通道基础封装
#include <server.hpp>
// 日志库
#include <spdlog/spdlog.h>
// 系统信息工具（主机名、时间等）
#include <system_info.hpp>

// 项目命名空间：fcs 项目 -> 可视化模块
namespace fcs::visualization {

// ============================================================================
// 内部细节实现：创建数据输出端(Sink)
// 匿名命名空间 detail：仅本文件内部可见，对外隐藏实现细节
// ============================================================================
namespace detail {

/**
 * @brief 创建 WebSocket 服务端（实时可视化推流）
 * @param config Foxglove 整体配置
 * @param context Foxglove 全局上下文
 * @return 成功返回 WebSocketServer 对象，失败返回错误字符串
 */
[[nodiscard]] std::expected<::foxglove::WebSocketServer, std::string>
    create_websocket_sink(const FoxgloveConfig& config, const ::foxglove::Context& context) {
    // 初始化 WebSocket 服务配置项
    ::foxglove::WebSocketServerOptions ws;
    ws.context      = context;        // 绑定全局上下文
    ws.name         = "talos";        // 服务名称标识
    ws.host         = config.host;     // 监听IP，对应配置文件 host
    ws.port         = config.port;     // 监听端口，对应配置文件 port
    ws.session_id   = "talos-forever";// 会话ID，用于客户端连接标识
    // 启用时间同步能力（上位机与设备时间对齐）
    ws.capabilities = foxglove::WebSocketServerCapabilities::Time;

    // 创建 WebSocket 服务实例
    auto server = ::foxglove::WebSocketServer::create(std::move(ws));
    if (server.has_value())
        return std::move(server.value());
    
    // 创建失败：拼接错误信息返回
    return std::unexpected(
        fmt::format("WebSocketServer: {}", ::foxglove::strerror(server.error())));
}

/**
 * @brief 创建 MCAP 文件录制端（数据落盘存储）
 * @param config Foxglove 配置
 * @param context Foxglove 全局上下文
 * @return 成功返回 McapWriter 写入器，失败返回错误字符串
 */
[[nodiscard]] std::expected<::foxglove::McapWriter, std::string>
    create_mcap_sink(const FoxgloveConfig& config, const ::foxglove::Context& context) {
    // 校验：录制文件路径不能为空
    if (config.mcap_path.empty()) {
        return std::unexpected("McapWriter: mcap_path must not be empty");
    }

    // 解析文件路径，自动递归创建上级目录
    const auto path = std::filesystem::path(config.mcap_path);
    if (const auto parent = path.parent_path(); !parent.empty()) {
        std::error_code ec;
        // 创建目录，已存在不会报错
        std::filesystem::create_directories(parent, ec);
        if (ec) {
            return std::unexpected(
                fmt::format(
                    "create MCAP parent directory '{}': {}", parent.string(), ec.message()));
        }
    }

    // 初始化 MCAP 写入器配置
    ::foxglove::McapWriterOptions options;
    options.context             = context;
    options.path                = config.mcap_path; // 录制文件路径
    options.truncate            = true;              // 覆盖已有文件
    // 通道过滤：不录制 /debug/ekf_heatmap 话题数据
    options.sink_channel_filter = [](const ::foxglove::ChannelDescriptor& channel) {
        return channel.topic() != "/debug/ekf_heatmap";
    };

    // 创建 MCAP 写入实例
    auto writer = ::foxglove::McapWriter::create(options);
    if (writer.has_value())
        return std::move(writer.value());
    
    return std::unexpected(fmt::format("McapWriter: {}", ::foxglove::strerror(writer.error())));
}

} // namespace detail

// ============================================================================
// FoxgloveServer 主类：可视化服务核心实现
// 职责：管理 WebSocket 实时推流、MCAP 离线录制、消息队列、发送线程、各类数据通道
// ============================================================================

/**
 * @brief 构造函数
 * @param ConstructorToken 私有构造令牌：禁止外部直接 new，强制使用工厂方法创建（防误用）
 * @param context Foxglove 全局上下文
 * @param channels 所有数据通道集合（图像、标定、TF坐标变换等）
 * @param publisher 系统信息发布器
 * @param websocket_server WebSocket 服务实例（可选）
 * @param mcap_writer MCAP 文件写入实例（可选）
 *
 * 流程：初始化成员、启动消息发送工作线程
 */
FoxgloveServer::FoxgloveServer(
    ConstructorToken, ::foxglove::Context context, FoxgloveChannels channels,
    std::optional<::foxglove::SystemInfoPublisher> publisher,
    std::optional<::foxglove::WebSocketServer> websocket_server,
    std::optional<::foxglove::McapWriter> mcap_writer)
    : context_(std::move(context))
    , channels_(std::move(channels))
    , publisher_(std::move(publisher))
    , websocket_server_(std::move(websocket_server))
    , mcap_writer_(std::move(mcap_writer))
    // 标记输出端是否存活，共享原子变量，多线程安全
    , sink_alive_(std::make_shared<std::atomic<bool>>(true)) {
    // 设置 Foxglove 底层日志级别
    ::foxglove::setLogLevel(::foxglove::LogLevel::Info);

    // 单独处理 TF 坐标变换通道：移入专属导出器，原通道置空
    if (channels_.tf_ch) {
        tf_exporter_.emplace(std::move(*channels_.tf_ch));
        channels_.tf_ch.reset();
    }

    server_initialized_ = true;
    // 启动消息发送后台线程，循环消费消息队列
    sender_thread_      = std::thread(&FoxgloveServer::message_sender_thread, this);
}

/**
 * @brief 析构函数：安全停止服务、回收资源
 * 流程：标记停止 -> 等待线程退出 -> 消费队列剩余消息 -> 关闭 WebSocket/MCAP
 */
FoxgloveServer::~FoxgloveServer() {
    // 原子布尔：防止析构函数被多次调用
    bool expected = false;
    if (!shutdown_done_.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
        return;

    // 标记输出端、线程停止
    sink_alive_->store(false, std::memory_order_release);
    should_stop_.store(true, std::memory_order_release);

    // 停止系统信息发布
    if (publisher_) {
        publisher_->stop();
    }

    // 等待发送线程正常退出
    if (sender_thread_.joinable())
        sender_thread_.join();

    // 消费队列中残余消息，保证数据不丢失
    FoxgloveMessage msg;
    while (message_queue_.try_pop(msg)) {
        dispatch_message(msg);
    }

    // 关闭 WebSocket 服务
    if (websocket_server_) {
        if (auto r = websocket_server_->stop(); r != ::foxglove::FoxgloveError::Ok) {
            SPDLOG_ERROR("Foxglove WebSocket stop failed: {}", ::foxglove::strerror(r));
        }
    }

    // 关闭 MCAP 文件写入（刷盘、收尾）
    if (mcap_writer_) {
        if (auto r = mcap_writer_->close(); r != ::foxglove::FoxgloveError::Ok) {
            SPDLOG_ERROR("Foxglove MCAP close failed: {}", ::foxglove::strerror(r));
        }
    }
}

// ============================================================================
// 对外公有接口
// ============================================================================

/**
 * @brief 发布相机标定参数（上位机可视化相机内参/畸变参数）
 * @param width 图像宽度
 * @param height 图像高度
 * @param camera_matrix 相机内参矩阵 3x3
 * @param distortion 畸变系数
 * @param timestamp_ns 时间戳(纳秒)
 */
void FoxgloveServer::publish_camera_calibration(
    uint32_t width, uint32_t height, const std::array<double, 9>& camera_matrix,
    const std::vector<double>& distortion, uint64_t timestamp_ns) {
    // 相机标定通道未初始化则直接返回
    if (!channels_.camera_calib_ch)
        return;

    // 广播当前时间戳，同步上位机时钟
    if (websocket_server_) {
        websocket_server_->broadcastTime(timestamp_ns);
    }

    // 组装 Foxglove 标准相机标定消息体
    ::foxglove::schemas::CameraCalibration calib;
    calib.timestamp        = timestamp_from_ns(timestamp_ns); // 纳秒转标准时间格式
    calib.frame_id         = "camera_optical_frame";          // 坐标系名称
    calib.width            = width;
    calib.height           = height;
    calib.distortion_model = "plumb_bob";                     // 畸变模型：经典针孔相机模型
    calib.d.assign(distortion.begin(), distortion.end());     // 畸变系数
    calib.k = camera_matrix;                                  // 相机内参矩阵K
    calib.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};  // 旋转矩阵R（单位矩阵）
    // 投影矩阵P
    calib.p = {
        camera_matrix[0], 0.0, camera_matrix[2], 0.0,
        0.0, camera_matrix[4], camera_matrix[5], 0.0,
        0.0, 0.0, 1.0, 0.0};

    // 写入标定数据到对应通道，对外发布
    channels_.camera_calib_ch->log(calib, timestamp_ns);
}

// ============================================================================
// 私有内部方法
// ============================================================================

/**
 * @brief 消息分发总入口
 * @param msg 通用 Foxglove 消息（多类型变体）
 * std::visit + 变体类型：根据消息实际类型，分发到对应处理函数
 */
void FoxgloveServer::dispatch_message(const FoxgloveMessage& msg) noexcept {
    std::visit([this](const auto& m) { dispatch_one(m); }, msg);
}

/**
 * @brief 消息发送后台线程主循环
 * 逻辑：不断从队列取消息 -> 分发发送；收到停止信号则退出
 */
void FoxgloveServer::message_sender_thread() {
    while (true) {
        FoxgloveMessage msg;
        // 非阻塞取消息，有消息立即分发
        if (message_queue_.try_pop(msg)) {
            dispatch_message(msg);
            continue;
        }

        // 检测停止标记，退出线程
        if (should_stop_.load(std::memory_order_acquire)) {
            break;
        }

        // 队列为空，让出CPU，降低空转占用
        std::this_thread::yield();
    }
}

// ============================================================================
// 工厂方法：统一创建 FoxgloveServer 实例（推荐外部调用入口）
// 设计模式：工厂模式，封装对象创建逻辑，解耦配置与实例
// ============================================================================

/**
 * @brief 工厂核心创建函数
 * @param config Foxglove 完整配置
 * @return 成功返回服务智能指针，失败返回错误信息
 */
[[nodiscard]] std::expected<std::unique_ptr<FoxgloveServer>, std::string>
    FoxgloveServerFactory::create(FoxgloveConfig config) {
    // 创建 Foxglove 全局上下文
    ::foxglove::Context context = ::foxglove::Context::create();

    std::optional<::foxglove::WebSocketServer> websocket_server;
    std::optional<::foxglove::McapWriter> mcap_writer;

    // 根据传输类型，创建对应输出端
    switch (config.transport) {
    case FoxgloveTransport::WebSocket: {
        // 创建实时 WebSocket 推流服务
        auto server = detail::create_websocket_sink(config, context);
        if (!server)
            return std::unexpected(server.error());
        /**server
        server 是 expected 对象，重载了解引用 operator*；
        *server 得到内部存储的 WebSocketServer& 左值引用。
        std::move(*server)
        把左值引用强制转换成右值引用，允许调用移动赋值运算符。*/
        websocket_server = std::move(*server);
        break;
    }
    case FoxgloveTransport::Mcap: {
        // 创建离线 MCAP 文件录制服务
        auto writer = detail::create_mcap_sink(config, context);
        if (!writer)
            return std::unexpected(writer.error());
        mcap_writer = std::move(*writer);
        break;
    }
    }

    // 初始化所有注册的数据通道（图像、TF、标定等）
    FoxgloveChannels channels;
    if (auto r = detail::init_all_channels(channels, context, config.transport, ChannelRegistry{});
        !r) {
        return std::unexpected(std::move(r.error()));
    }

    // 创建系统信息发布器（发布设备状态、时间等）
    auto options    = foxglove::SystemInfoOptions{};
    options.context = context;
    auto publisher  = foxglove::SystemInfoPublisher::create(std::move(options));
    if (!publisher) {
        return std::unexpected(std::string(foxglove::strerror(publisher.error())));
    }

    // 调用私有构造函数，生成 FoxgloveServer 实例并包装为 unique_ptr 返回
    return std::make_unique<FoxgloveServer>(
        FoxgloveServer::ConstructorToken{}, std::move(context), std::move(channels),
        std::move(publisher.value()), std::move(websocket_server), std::move(mcap_writer));
}

/**
 * @brief 对外简易创建接口（完整配置版本）
 */
[[nodiscard]] std::expected<std::unique_ptr<FoxgloveServer>, std::string>
    create_foxglove_server(FoxgloveConfig config) {
    return FoxgloveServerFactory::create(std::move(config));
}

/**
 * @brief 极简创建接口（仅指定IP+端口，默认 WebSocket 模式）
 */
[[nodiscard]] std::expected<std::unique_ptr<FoxgloveServer>, std::string>
    create_foxglove_server(uint16_t port, std::string host) {
    return create_foxglove_server(
        FoxgloveConfig{
            .transport = FoxgloveTransport::WebSocket,
            .host      = std::move(host),
            .port      = port,
        });
}

} // namespace fcs::visualization