#pragma once

#include "config.hpp"
#include "scheduler/thin.hpp"

#include <string>

namespace fcs::runtime {

struct CapturerLaunchContext {
    HardwareBackend backend;
    std::string robot;
    std::string vision;
};

void register_runtime_capturer_system(
    talos::Scheduler& scheduler, const CapturerConfig& config, const CapturerLaunchContext& launch);

} // namespace fcs::runtime
