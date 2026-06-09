#pragma once

#include <expected>
#include <string>

#include "camera_config.hpp"
#include "runtime/config_loader.hpp"
#include "scheduler/thin.hpp"

namespace fcs {
struct HardwareConfig;
} // namespace fcs

namespace fcs::runtime {

struct L1L2SetupResult {
    CameraConfig camera_config;
};

// Creates and registers:
// - IMU reader system
// - Camera reader system (publishes ImageFrame)
// - Weapon output system (consumes L5::WeaponCommand)
// - L2 detector backend + PnP solver resources
// - CameraConfig resource
//
// `hardware` must be non-null when `daedalus == false`.
[[nodiscard]] std::expected<L1L2SetupResult, std::string>
    setup_l1(talos::World& world, talos::Scheduler& scheduler, hardware::HardwareBackendConfig cfg);

} // namespace fcs::runtime
