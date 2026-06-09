#pragma once

#include "quanta/stream_encoder.hpp"
#include "scheduler/thin.hpp"

#include <expected>
#include <string>

namespace fcs::runtime {

[[nodiscard]] std::expected<void, std::string> register_quanta_stream_systems(
    talos::World& world, talos::Scheduler& scheduler, const quanta::EncodeParams& encode_params,
    int src_width, int src_height);

} // namespace fcs::runtime
