#pragma once

namespace fcs::visualization {
[[nodiscard]] inline ::foxglove::schemas::Timestamp timestamp_from_ns(uint64_t ns) noexcept {
    ::foxglove::schemas::Timestamp t;
    t.sec  = static_cast<uint32_t>(ns / 1'000'000'000ULL);
    t.nsec = static_cast<uint32_t>(ns % 1'000'000'000ULL);
    return t;
}

[[nodiscard]] inline std::vector<uint8_t> json_to_bytes(const std::string& json_str) {
    return {
        reinterpret_cast<const uint8_t*>(json_str.data()),
        reinterpret_cast<const uint8_t*>(json_str.data() + json_str.size())};
}
} // namespace fcs::visualization
