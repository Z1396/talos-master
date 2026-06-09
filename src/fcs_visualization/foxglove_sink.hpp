#pragma once

#include "foxglove_server.hpp"
#include "foxglove_types.hpp"

#include <atomic>
#include <chrono>
#include <memory>

#include <spdlog/common.h>
#include <spdlog/details/log_msg.h>
#include <spdlog/sinks/base_sink.h>
#include <utility.hpp>

namespace fcs::visualization {

// ============================================================================
// FoxgloveSink - spdlog sink → Foxglove Log panel
// ============================================================================
//
// Thread-safety: inherits mutex from base_sink<mt>.
// Lifetime-safe: shared_ptr<atomic<bool>> outlives FoxgloveServer.
//   FoxgloveServer::~FoxgloveServer sets the flag to false first,
//   then joins the sender thread — so no dangling access is possible.
//
// Usage:
//   auto server = fcs::try_create_foxglove_server(cfg);
//   fcs::attach_foxglove_sink(*server);
// ============================================================================

template <typename Mutex>
class FoxgloveSink final : public spdlog::sinks::base_sink<Mutex> {
public:
    FoxgloveSink(FoxgloveServer& server, std::shared_ptr<std::atomic<bool>> alive) noexcept
        : server_(server)
        , alive_(std::move(alive)) {}

protected:
    void sink_it_(const spdlog::details::log_msg& msg) override {
        // FoxgloveServer already destroyed — silently drop
        if (!alive_->load(std::memory_order_acquire))
            return;

        const auto now = std::chrono::system_clock::now();
        const auto ns  = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());

        ::foxglove::schemas::Log log;
        log.timestamp = timestamp_from_ns(ns);
        log.level     = to_foxglove_level(msg.level);
        log.message   = std::string(msg.payload.data(), msg.payload.size());
        log.name      = std::string(msg.logger_name.data(), msg.logger_name.size());

        if (msg.source.filename != nullptr)
            log.file = msg.source.filename;
        if (msg.source.line >= 0)
            log.line = static_cast<uint32_t>(msg.source.line);

        LogMessage wrapper;
        wrapper.payload = std::move(log);
        server_.enqueue_message(FoxgloveMessage{std::move(wrapper)});
    }

    void flush_() override {}

private:
    [[nodiscard]] static ::foxglove::schemas::Log::LogLevel
        to_foxglove_level(spdlog::level::level_enum lvl) noexcept {
        using Spd = spdlog::level::level_enum;
        using Fg  = ::foxglove::schemas::Log::LogLevel;
        switch (lvl) {
        case Spd::trace: [[fallthrough]];
        case Spd::debug: return Fg::DEBUG;
        case Spd::info: return Fg::INFO;
        case Spd::warn: return Fg::WARNING;
        case Spd::err: return Fg::ERROR;
        case Spd::critical: return Fg::FATAL;
        default: return Fg::UNKNOWN;
        }
    }

    FoxgloveServer& server_;
    std::shared_ptr<std::atomic<bool>> alive_;
};

/// Attach a Foxglove log sink to the default spdlog logger.
/// Must be called after init_logger() and after FoxgloveServer creation.
inline void attach_foxglove_sink(FoxgloveServer& server) noexcept {
    auto sink   = std::make_shared<FoxgloveSink<std::mutex>>(server, server.sink_alive());
    auto logger = spdlog::default_logger();
    if (logger)
        logger->sinks().push_back(std::move(sink));
}

} // namespace fcs::visualization
