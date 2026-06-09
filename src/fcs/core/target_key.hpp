#pragma once

#include "core/armor_types.hpp"

#include <cstddef>
#include <cstdint>

namespace fcs::core {

struct TargetKey {
    ArmorName name{ArmorName::Invalid};
    ArmorColor color{ArmorColor::Neutral};

    [[nodiscard]] bool operator==(const TargetKey& other) const noexcept {
        return name == other.name && color == other.color;
    }
};

struct TargetKeyHash {
    [[nodiscard]] auto operator()(const TargetKey& key) const noexcept -> std::size_t {
        const auto packed =
            (static_cast<std::uint32_t>(key.name) << 8U) | static_cast<std::uint32_t>(key.color);
        return std::hash<std::uint32_t>{}(packed);
    }
};

} // namespace fcs::core
