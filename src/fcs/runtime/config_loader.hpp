#pragma once

#include "config.hpp"
#include "foxglove_config.hpp"
#include "runtime/capturer.hpp"
#include "scheduler/thin.hpp"

#include <expected>
#include <map>
#include <optional>
#include <string>
#include <string_view>

namespace fcs {

namespace hardware {

enum Transport { Direct, Chiral };

struct DaedalusConfig {
    double bullet_speed{25.0};
    RobotExtrinsicConfig extrinsic{};
};

struct DirectConfig {
    bool camera_only;
    Transport transport;
    HardwareConfig hardware;
};

using HardwareBackendConfig = std::variant<DaedalusConfig, DirectConfig>;

}; // namespace hardware

/// Per-hostname preset entry in [preset] table.
struct PresetEntry {
    HardwareBackend backend{HardwareBackend::Direct};
    toml_helper::required<std::string> robot;
    toml_helper::required<std::string> vision;
};

// Entry-point configuration parsed from at_vision.toml
struct LaunchConfig {
    toml_helper::flatten<PresetEntry> fallback_preset;
    std::map<std::string, PresetEntry> preset;
    hardware::DaedalusConfig daedalus;
    FoxgloveConfig foxglove;
    CapturerConfig capturer;
    talos::scheduler::SchedulerConfig scheduler;
};

// Fully-resolved runtime configuration after merging base + override TOML
struct RuntimeConfig {
    hardware::HardwareBackendConfig backend;
    FoxgloveConfig foxglove;
    CapturerConfig capturer;
    VisionConfig vision;
    runtime::CapturerLaunchContext launch;
    talos::scheduler::SchedulerConfig scheduler;
};

/// Load and merge all configuration files from the given entry path.
/// When [preset] is present in the entry config, uses the system hostname
/// to auto-select robot and vision presets. Falls back to explicit fields otherwise.
/// Returns fully-resolved RuntimeConfig or a formatted error string.
[[nodiscard]] std::expected<RuntimeConfig, std::string> load_config(std::string_view path);

} // namespace fcs
