#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <fmt/core.h>
#include <optional>
#include <string>

#include "L2_perception/armor/config.hpp"
#include "L2_perception/ldm/ldm_config.hpp"
#include "L2_perception/rune/rune_config.hpp"
#include "L3_estimation/energy_meter/energy_meter_config.hpp"
#include "L3_estimation/ldm_naive/ldm_naive_config.hpp"
#include "L3_estimation/tracker/config.hpp"
#include "L4_planning/config.hpp"
#include "L5_weapon/config.hpp"
#include "camera_config.hpp"
#include "core/armor_types.hpp"
#include "core/runtime.hpp"
#include "core/trajectory/config.hpp"
#include "euler.hpp"
#include "quanta/stream_encoder.hpp"
#include <magic_enum.hpp>

namespace fcs {

enum HardwareBackend { Direct, Daedalus, Chiral, CameraOnly };

struct CapturerConfig {
    bool enabled{false};
    std::string output_dir{};
    uint64_t reserved_free_bytes{5ULL * 1024ULL * 1024ULL * 1024ULL};
};

using degree = double;

struct RobotExtrinsicConfig {
    struct gimbal_yaw_t {
        struct gimbal_pitch_t {
            struct camera_link_t {
                Eigen::Vector3d translation{};
                degree roll{};
                degree pitch{};
                degree yaw{};
                math_fuxk::Ros2EulerRotd rotation() const noexcept {
                    return math_fuxk::rpy(roll, pitch, yaw);
                }
            };
            struct muzzle_link_t {
                Eigen::Vector3d translation{};
                degree roll{};
                degree pitch{};
                degree yaw{};

                math_fuxk::Ros2EulerRotd rotation() const noexcept {
                    return math_fuxk::rpy(roll, pitch, yaw);
                }
            };
            Eigen::Vector3d translation{};
            camera_link_t camera_link{};
            muzzle_link_t muzzle_link{};
        };
        gimbal_pitch_t gimbal_pitch{};
    };

    gimbal_yaw_t gimbal_yaw{};
};

enum class McuBackend : uint8_t {
    Usb,
    Serial,
};

struct McuConfig {
    McuBackend mcu_backend{McuBackend::Usb};
    uint16_t mcu_vendor_id{0x0483};
    std::optional<uint16_t> mcu_product_id{std::nullopt};
    std::string serial_device{"/dev/ttyS4"};
    int serial_baud_rate{115200};
    bool mcu_authoritative_self_color{true};
    bool mcu_authoritative_bullet_speed{true};
    double bullet_speed_default{22.0};
    double bullet_speed_min{25.0};
    double bullet_speed_max{15.0};
};

struct HardwareConfig {
    toml_helper::required<CameraConfig> camera{};
    toml_helper::required<McuConfig> mcu{};
    toml_helper::required<RobotExtrinsicConfig> extrinsic{};
    bool chiral{false};
};

struct VisionConfig {
    ArmorColor detect_color;
    toml_helper::required<std::vector<core::Capability>> capabilities;
    quanta::EncodeParams quanta{};
    quanta::FilterParams quanta_filter{};
    core::trajectory::TrajectoryConfig trajectory{};
    toml_helper::flatten<L2::ArmorDetectorConfig> armor{};
    rune::RuneDetectorConfig rune_detector{};
    L2::ldm::LdmDetectorConfig ldm{};

    toml_helper::flatten<L3::L3Config> l3{};
    energy_meter::EnergyMeterL3Config energy_meter{};
    L3::ldm::NaiveLdmConfig naive_ldm{};

    toml_helper::flatten<L4::L4Config> l4{};

    toml_helper::flatten<L5::L5Config> l5{};

    [[nodiscard]] const L3::TrackerConfig& tracker() const noexcept { return l3->tracker; }
    [[nodiscard]] const L5::WeaponControllerConfig& weapon() const noexcept {
        return l5->mpc_weapon;
    }
    [[nodiscard]] L3::TrackerConfig& tracker() noexcept { return l3->tracker; }
    [[nodiscard]] L5::WeaponControllerConfig& weapon() noexcept { return l5->mpc_weapon; }
};

} // namespace fcs

// ============================================================================
// fmt::formatter specializations
// ============================================================================

namespace fmt {

template <>
struct formatter<fcs::McuBackend> : formatter<std::string_view> {
    auto format(fcs::McuBackend b, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(b), ctx);
    }
};

template <>
struct formatter<fcs::HardwareBackend> : formatter<std::string_view> {
    auto format(fcs::HardwareBackend b, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(b), ctx);
    }
};

} // namespace fmt
