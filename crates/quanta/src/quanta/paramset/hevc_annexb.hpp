#pragma once

#include "quanta/stream_encoder.hpp"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <string>

namespace quanta {

struct HevcParameterSetExportState {
    bool exported = false;
};

[[nodiscard]] bool is_hevc_idr_nalu(uint8_t type) noexcept;
[[nodiscard]] bool is_hevc_parameter_set_nalu(uint8_t type) noexcept;

[[nodiscard]] std::expected<bool, std::string>
    strip_hevc_parameter_sets(EncodedPacket& packet) noexcept;

[[nodiscard]] std::expected<void, std::string> export_hevc_parameter_sets_once(
    const EncodedPacket& packet, HevcParameterSetExportState& state) noexcept;

} // namespace quanta
