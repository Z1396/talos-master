#include "runtime/stream_send.hpp"

#include "L1_sensor/output_interface.hpp"
#include "core/runtime.hpp"
#include "quanta/stream_transport.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>
#include <utility>

namespace fcs::runtime {

void register_quanta_stream_send_system(talos::Scheduler& scheduler) {
    scheduler.add_system<talos::fixed_rate<50, 3>>(
        "stream_send",
        [](talos::res_mut<quanta::QuantaPacketQueue> packet_queue,
           talos::res<std::unique_ptr<fcs::L1::OutputInterface>> output, core::capabilities cap) {
            if (!core::capable(*cap, core::Capability::Quanta) || !*output) {
                return;
            }

            auto dequeued = packet_queue->pop_packet();
            if (!dequeued) {
                SPDLOG_DEBUG("Packet queue is empty");
                return;
            }
            (*output)->send_quanta(std::move(dequeued->packet));
            SPDLOG_DEBUG(
                "Quanta packet sent: frag={}/{}, packets_left={}, frames_left={}",
                static_cast<unsigned int>(dequeued->packet.fragment_index) + 1U,
                static_cast<unsigned int>(dequeued->packet.fragment_count),
                dequeued->remaining_packets, dequeued->remaining_frames);
        });
}

} // namespace fcs::runtime
