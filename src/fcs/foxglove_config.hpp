#pragma once

#include "quanta/stream_encoder.hpp"

#include <cstdint>
#include <string>

namespace fcs {

enum class FoxgloveTransport : uint8_t {
    WebSocket,
    Mcap,
};

struct FoxgloveConfig {
    bool enabled{true};
    FoxgloveTransport transport{FoxgloveTransport::WebSocket};
    std::string host{"0.0.0.0"};
    uint16_t port{8765};
    std::string mcap_path{};
    quanta::EncodeParams quanta{};
};

} // namespace fcs
