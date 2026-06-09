#pragma once
#include "core/armor_types.hpp"
#include "scheduler/system/components.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <fmt/base.h>
#include <span>
#include <vector>

namespace fcs::core {

struct ImuState {
    uint64_t timestamp_ns{0};
    double yaw{0.0};
    double pitch{0.0};
    double roll{0.0};
    double yaw_vel{0.0};
    double pitch_vel{0.0};
    double roll_vel{0.0};
};

struct ControlTransformSnapshot {
    bool present{false};
    std::array<double, 3> translation{};
    std::array<double, 4> quaternion{};
};

struct ControlResourceSnapshot {
    uint64_t sample_timestamp_ns{0};
    ImuState imu{};
    ArmorColor detecting_color{ArmorColor::Blue};
    double bullet_speed_raw{0.0};
    double bullet_speed{0.0};
    ControlTransformSnapshot odom_to_gimbal_pitch{};
    ControlTransformSnapshot gimbal_to_camera_link{};
    ControlTransformSnapshot odom_to_camera_optical{};
    ControlTransformSnapshot odom_to_muzzle{};
};
using namespace talos::scheduler::system;
using imu_state     = res<ImuState>;
using imu_state_mut = res_mut<ImuState>;

using detecting_color     = res<ArmorColor>;
using detecting_color_mut = res_mut<ArmorColor>;

enum Capability {
    Armor  = 0,
    Rune   = 1,
    Ldm    = 2,
    Quanta = 3,
};

using CapabilityMask = uint8_t;

[[nodiscard]] constexpr auto capability_bit(const Capability cap) noexcept -> CapabilityMask {
    return static_cast<CapabilityMask>(1U << static_cast<unsigned>(cap));
}

[[nodiscard]] inline auto capability_mask(const std::span<const Capability> capabilities) noexcept
    -> CapabilityMask {
    CapabilityMask mask = 0;
    for (const auto capability : capabilities) {
        mask = static_cast<CapabilityMask>(mask | capability_bit(capability));
    }
    return mask;
}

struct CapabilityState {
    explicit CapabilityState(const std::span<const Capability> initial) noexcept
        : mask_(capability_mask(initial)) {}

    explicit CapabilityState(const CapabilityMask initial) noexcept
        : mask_(initial) {}

    CapabilityState(const CapabilityState&)                    = delete;
    auto operator=(const CapabilityState&) -> CapabilityState& = delete;
    CapabilityState(CapabilityState&&)                         = delete;
    auto operator=(CapabilityState&&) -> CapabilityState&      = delete;

    [[nodiscard]] auto load() const noexcept -> CapabilityMask {
        return mask_.load(std::memory_order_acquire);
    }

    void store(const CapabilityMask next) noexcept { mask_.store(next, std::memory_order_release); }

private:
    std::atomic<CapabilityMask> mask_;
};

using capabilities     = res<CapabilityState>;
using capabilities_mut = res_mut<CapabilityState>;

struct FollowingState {
    [[nodiscard]] auto load() const noexcept -> bool {
        return active_.load(std::memory_order_acquire);
    }

    void store(const bool active) noexcept { active_.store(active, std::memory_order_release); }

private:
    std::atomic<bool> active_{false};
};

using following     = res<FollowingState>;
using following_mut = res_mut<FollowingState>;

inline constexpr std::array all_capabilities{
    Capability::Armor, Capability::Rune, Capability::Ldm, Capability::Quanta};

[[nodiscard]] inline auto active_capabilities(const CapabilityState& state)
    -> std::vector<Capability> {
    std::vector<Capability> active;
    const auto mask = state.load();
    for (const auto capability : all_capabilities) {
        if ((mask & capability_bit(capability)) != 0U) {
            active.push_back(capability);
        }
    }
    return active;
}

[[nodiscard]] inline auto capable(const CapabilityState& state, const Capability cap) noexcept
    -> bool {
    return (state.load() & capability_bit(cap)) != 0U;
}

} // namespace fcs::core

namespace fmt {
template <>
struct formatter<fcs::core::Capability> : formatter<std::string_view> {
    auto format(const fcs::core::Capability c, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(c), ctx);
    }
};
} // namespace fmt
