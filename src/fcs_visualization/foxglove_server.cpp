#include "foxglove_server.hpp"

#include <error.hpp>
#include <server.hpp>
#include <spdlog/spdlog.h>
#include <system_info.hpp>

namespace fcs::visualization {

// ============================================================================
// Detail — Sink Creation
// ============================================================================

namespace detail {

[[nodiscard]] std::expected<::foxglove::WebSocketServer, std::string>
    create_websocket_sink(const FoxgloveConfig& config, const ::foxglove::Context& context) {
    ::foxglove::WebSocketServerOptions ws;
    ws.context      = context;
    ws.name         = "talos";
    ws.host         = config.host;
    ws.port         = config.port;
    ws.session_id   = "talos-forever";
    ws.capabilities = foxglove::WebSocketServerCapabilities::Time;

    auto server = ::foxglove::WebSocketServer::create(std::move(ws));
    if (server.has_value())
        return std::move(server.value());
    return std::unexpected(
        fmt::format("WebSocketServer: {}", ::foxglove::strerror(server.error())));
}

[[nodiscard]] std::expected<::foxglove::McapWriter, std::string>
    create_mcap_sink(const FoxgloveConfig& config, const ::foxglove::Context& context) {
    if (config.mcap_path.empty()) {
        return std::unexpected("McapWriter: mcap_path must not be empty");
    }

    const auto path = std::filesystem::path(config.mcap_path);
    if (const auto parent = path.parent_path(); !parent.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
        if (ec) {
            return std::unexpected(
                fmt::format(
                    "create MCAP parent directory '{}': {}", parent.string(), ec.message()));
        }
    }

    ::foxglove::McapWriterOptions options;
    options.context             = context;
    options.path                = config.mcap_path;
    options.truncate            = true;
    options.sink_channel_filter = [](const ::foxglove::ChannelDescriptor& channel) {
        return channel.topic() != "/debug/ekf_heatmap";
    };

    auto writer = ::foxglove::McapWriter::create(options);
    if (writer.has_value())
        return std::move(writer.value());
    return std::unexpected(fmt::format("McapWriter: {}", ::foxglove::strerror(writer.error())));
}

} // namespace detail

// ============================================================================
// FoxgloveServer — Constructor / Destructor
// ============================================================================

FoxgloveServer::FoxgloveServer(
    ConstructorToken, ::foxglove::Context context, FoxgloveChannels channels,
    std::optional<::foxglove::SystemInfoPublisher> publisher,
    std::optional<::foxglove::WebSocketServer> websocket_server,
    std::optional<::foxglove::McapWriter> mcap_writer)
    : context_(std::move(context))
    , channels_(std::move(channels))
    , publisher_(std::move(publisher))
    , websocket_server_(std::move(websocket_server))
    , mcap_writer_(std::move(mcap_writer)) {
    ::foxglove::setLogLevel(::foxglove::LogLevel::Info);
    if (channels_.tf_ch) {
        tf_exporter_.emplace(std::move(*channels_.tf_ch));
        channels_.tf_ch.reset();
    }
    server_initialized_ = true;
    sender_thread_      = std::thread(&FoxgloveServer::message_sender_thread, this);
}

FoxgloveServer::~FoxgloveServer() {
    bool expected = false;
    if (!shutdown_done_.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
        return;

    sink_alive_->store(false, std::memory_order_release);
    should_stop_.store(true, std::memory_order_release);
    if (publisher_) {
        publisher_->stop();
    }
    if (sender_thread_.joinable())
        sender_thread_.join();

    // Drain remaining messages before teardown
    FoxgloveMessage msg;
    while (message_queue_.try_pop(msg)) {
        dispatch_message(msg);
    }

    if (websocket_server_) {
        if (auto r = websocket_server_->stop(); r != ::foxglove::FoxgloveError::Ok) {
            SPDLOG_ERROR("Foxglove WebSocket stop failed: {}", ::foxglove::strerror(r));
        }
    }
    if (mcap_writer_) {
        if (auto r = mcap_writer_->close(); r != ::foxglove::FoxgloveError::Ok) {
            SPDLOG_ERROR("Foxglove MCAP close failed: {}", ::foxglove::strerror(r));
        }
    }
}

// ============================================================================
// FoxgloveServer — Public Methods
// ============================================================================

void FoxgloveServer::publish_camera_calibration(
    uint32_t width, uint32_t height, const std::array<double, 9>& camera_matrix,
    const std::vector<double>& distortion, uint64_t timestamp_ns) {
    if (!channels_.camera_calib_ch)
        return;
    if (websocket_server_) {
        websocket_server_->broadcastTime(timestamp_ns);
    }
    ::foxglove::schemas::CameraCalibration calib;
    calib.timestamp        = timestamp_from_ns(timestamp_ns);
    calib.frame_id         = "camera_optical_frame";
    calib.width            = width;
    calib.height           = height;
    calib.distortion_model = "plumb_bob";
    calib.d.assign(distortion.begin(), distortion.end());
    calib.k = camera_matrix;
    calib.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    calib.p = {
        camera_matrix[0],
        0.0,
        camera_matrix[2],
        0.0,
        0.0,
        camera_matrix[4],
        camera_matrix[5],
        0.0,
        0.0,
        0.0,
        1.0,
        0.0};
    channels_.camera_calib_ch->log(calib, timestamp_ns);
}

// ============================================================================
// FoxgloveServer — Private Methods
// ============================================================================

void FoxgloveServer::dispatch_message(const FoxgloveMessage& msg) noexcept {
    std::visit([this](const auto& m) { dispatch_one(m); }, msg);
}

void FoxgloveServer::message_sender_thread() {
    while (true) {
        FoxgloveMessage msg;
        if (message_queue_.try_pop(msg)) {
            dispatch_message(msg);
            continue;
        }

        if (should_stop_.load(std::memory_order_acquire)) {
            break;
        }

        std::this_thread::yield();
    }
}

// ============================================================================
// Factory Implementation — descriptor-driven
// ============================================================================

[[nodiscard]] std::expected<std::unique_ptr<FoxgloveServer>, std::string>
    FoxgloveServerFactory::create(FoxgloveConfig config) {
    ::foxglove::Context context = ::foxglove::Context::create();

    std::optional<::foxglove::WebSocketServer> websocket_server;
    std::optional<::foxglove::McapWriter> mcap_writer;

    switch (config.transport) {
    case FoxgloveTransport::WebSocket: {
        auto server = detail::create_websocket_sink(config, context);
        if (!server)
            return std::unexpected(server.error());
        websocket_server = std::move(*server);
        break;
    }
    case FoxgloveTransport::Mcap: {
        auto writer = detail::create_mcap_sink(config, context);
        if (!writer)
            return std::unexpected(writer.error());
        mcap_writer = std::move(*writer);
        break;
    }
    }

    // Initialize all channels from the descriptor registry
    FoxgloveChannels channels;
    if (auto r = detail::init_all_channels(channels, context, config.transport, ChannelRegistry{});
        !r) {
        return std::unexpected(std::move(r.error()));
    }
    auto options    = foxglove::SystemInfoOptions{};
    options.context = context;
    auto publisher  = foxglove::SystemInfoPublisher::create(std::move(options));
    if (!publisher) {
        return std::unexpected(std::string(foxglove::strerror(publisher.error())));
    }
    return std::make_unique<FoxgloveServer>(
        FoxgloveServer::ConstructorToken{}, std::move(context), std::move(channels),
        std::move(publisher.value()), std::move(websocket_server), std::move(mcap_writer));
}

[[nodiscard]] std::expected<std::unique_ptr<FoxgloveServer>, std::string>
    create_foxglove_server(FoxgloveConfig config) {
    return FoxgloveServerFactory::create(std::move(config));
}

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
