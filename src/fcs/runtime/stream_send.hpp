#pragma once

#include "scheduler/thin.hpp"

namespace fcs::runtime {

void register_quanta_stream_send_system(talos::Scheduler& scheduler);

} // namespace fcs::runtime
