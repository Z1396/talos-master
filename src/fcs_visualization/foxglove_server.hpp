#pragma once

#include "foxglove_types.hpp"

#include "foxglove_export.hpp"
#include "frame.hpp"

#include <atomic>
#include <expected>
#include <memory>
#include <string>
#include <system_info.hpp>
#include <thread>

#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <tbb/concurrent_queue.h>

#include <foxglove/error.hpp>
#include <foxglove/foxglove.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/server.hpp>
#include <utility.hpp>

namespace fcs::visualization {

class FoxgloveServer;
class FoxgloveServerFactory;

namespace detail {

// ============================================================================
// Channel Creation Helpers — driven by descriptors
// ============================================================================

template <typename Def>
[[nodiscard]] inline std::expected<void, std::string> init_typed_channel(
    std::optional<typename Def::channel_type>& member, const ::foxglove::Context& context) {
    auto result = Def::channel_type::create(std::string(Def::topic), context);
    if (result.has_value()) {
        member = std::move(*result);
        return {};
    }
    return std::unexpected(fmt::format("{}: {}", Def::topic, ::foxglove::strerror(result.error())));
}

template <typename Def>
[[nodiscard]] inline std::expected<void, std::string> init_raw_channel(
    std::optional<::foxglove::RawChannel>& member, const ::foxglove::Context& context) {
    auto result = ::foxglove::RawChannel::create(
        std::string(Def::topic), std::string(Def::encoding), {}, context);
    if (result.has_value()) {
        member = std::move(*result);
        return {};
    }
    return std::unexpected(fmt::format("{}: {}", Def::topic, ::foxglove::strerror(result.error())));
}

template <typename Def>
[[nodiscard]] inline std::expected<void, std::string>
    init_channel_for(FoxgloveChannels& channels, const ::foxglove::Context& context) {
    if constexpr (requires { Def::transport; }) {
        // Skip channels that don't match the current transport.
        // Caller must set transport on the descriptor or check separately.
    }
    if constexpr (Def::is_raw) {
        return init_raw_channel<Def>(channels.*(Def::member), context);
    } else {
        return init_typed_channel<Def>(channels.*(Def::member), context);
    }
}

// Transport-aware initialization: skip if descriptor has a transport constraint
template <typename Def>
[[nodiscard]] inline std::expected<void, std::string> init_channel_for(
    FoxgloveChannels& channels, const ::foxglove::Context& context,
    FoxgloveTransport active_transport) {
    if constexpr (requires { Def::transport; }) {
        if (Def::transport != active_transport)
            return {};
    }
    return init_channel_for<Def>(channels, context);
}

// Fold-expression: initialize all channels in a descriptor tuple
template <typename... Defs>
[[nodiscard]] inline std::expected<void, std::string> init_all_channels(
    FoxgloveChannels& channels, const ::foxglove::Context& context, FoxgloveTransport transport,
    std::tuple<Defs...>) {
    std::expected<void, std::string> result;
    bool ok =
        ((result = init_channel_for<Defs>(channels, context, transport), result.has_value())
         && ...);
    if (!ok)
        return result;
    return {};
}

// ============================================================================
// Sink Creation
// ============================================================================

[[nodiscard]] std::expected<::foxglove::WebSocketServer, std::string>
    create_websocket_sink(const FoxgloveConfig& config, const ::foxglove::Context& context);

[[nodiscard]] std::expected<::foxglove::McapWriter, std::string>
    create_mcap_sink(const FoxgloveConfig& config, const ::foxglove::Context& context);

} // namespace detail

// ============================================================================
// Factory & Free Functions
// ============================================================================

class FoxgloveServerFactory {
public:
    [[nodiscard]] static std::expected<std::unique_ptr<FoxgloveServer>, std::string>
        create(FoxgloveConfig config);
};

[[nodiscard]] std::expected<std::unique_ptr<FoxgloveServer>, std::string>
    create_foxglove_server(FoxgloveConfig config);

[[nodiscard]] std::expected<std::unique_ptr<FoxgloveServer>, std::string>
    create_foxglove_server(uint16_t port, std::string host);

// ============================================================================
// FoxgloveServer
// ============================================================================

class FoxgloveServer {
private:
    struct ConstructorToken {
        explicit ConstructorToken() = default;
    };
    friend class FoxgloveServerFactory;

public:
    explicit FoxgloveServer(
        ConstructorToken, ::foxglove::Context context, FoxgloveChannels channels,
        std::optional<::foxglove::SystemInfoPublisher> publisher,
        std::optional<::foxglove::WebSocketServer> websocket_server,
        std::optional<::foxglove::McapWriter> mcap_writer);

    ~FoxgloveServer();

    FoxgloveServer(const FoxgloveServer&)            = delete;
    FoxgloveServer& operator=(const FoxgloveServer&) = delete;
    FoxgloveServer(FoxgloveServer&&)                 = delete;
    FoxgloveServer& operator=(FoxgloveServer&&)      = delete;

    [[nodiscard]] bool is_initialized() const noexcept { return server_initialized_; }
    void enqueue_message(FoxgloveMessage msg) noexcept { message_queue_.push(std::move(msg)); }
    [[nodiscard]] std::shared_ptr<std::atomic<bool>> sink_alive() const noexcept {
        return sink_alive_;
    }

    void publish_camera_calibration(
        uint32_t width, uint32_t height, const std::array<double, 9>& camera_matrix,
        const std::vector<double>& distortion, uint64_t timestamp_ns = 0);

    void publish_tf(const fast_tf::CoordinateSystem& tf_buffer, uint64_t timestamp_ns) noexcept {
        if (tf_exporter_) {
            tf_exporter_->publish_all(tf_buffer, timestamp_ns);
        }
    }

private:
    // Descriptor-driven dispatch — no ChannelTraits needed
    template <typename Def>
    void dispatch_one(const FoxgloveMsg<Def>& m) noexcept {
        auto& opt_ch = channels_.*(Def::member);
        if (!opt_ch)
            return;
        using Payload = typename Def::payload_type;
        detail::PayloadLogger<Payload>::log(*opt_ch, m.payload);
    }

    void dispatch_message(const FoxgloveMessage& msg) noexcept;
    void message_sender_thread();

    bool server_initialized_{false};
    std::atomic<bool> should_stop_{false};
    std::atomic<bool> shutdown_done_{false};
    std::shared_ptr<std::atomic<bool>> sink_alive_{std::make_shared<std::atomic<bool>>(true)};
    ::foxglove::Context context_{};
    FoxgloveChannels channels_;
    std::optional<::foxglove::SystemInfoPublisher> publisher_;
    std::optional<::foxglove::WebSocketServer> websocket_server_;
    std::optional<::foxglove::McapWriter> mcap_writer_;
    std::optional<fast_tf::FoxgloveExporter> tf_exporter_;
    alignas(64) tbb::concurrent_queue<FoxgloveMessage> message_queue_;
    std::thread sender_thread_;
};

} // namespace fcs::visualization
