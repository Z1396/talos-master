#pragma once

#include "runtime/config_loader.hpp"
#include "scheduler/thin.hpp"

#include <expected>
#include <string>

namespace fcs {

/// Bootstrap the full FCS pipeline into the given World and Scheduler.
///
/// Inserts all resources, registers L1-L5 systems, and calls scheduler.build().
/// The caller must handle Foxglove setup separately (it lives in fcs_visualization,
/// which is not linked by the fcs library).
[[nodiscard]] std::expected<void, std::string>
    boot(talos::Scheduler& scheduler, RuntimeConfig&& config);

} // namespace fcs
