#include "scheduler/rclcompat/timer_factory.hpp"

#include "scheduler/rclcompat/timer_constants.hpp"
#include "timer_system.hpp"
#include <utility>

namespace talos::scheduler::rclcompat {

std::unique_ptr<system::SystemBase> create_timer_system(
    std::string&& name, const Frequency frequency, std::function<void()> callback) {
    switch (frequency) {
    case Frequency::Hz_1: return std::make_unique<RclTimerSystem<1>>(name, std::move(callback));
    case Frequency::Hz_2: return std::make_unique<RclTimerSystem<2>>(name, std::move(callback));
    case Frequency::Hz_5: return std::make_unique<RclTimerSystem<5>>(name, std::move(callback));
    case Frequency::Hz_10: return std::make_unique<RclTimerSystem<10>>(name, std::move(callback));
    case Frequency::Hz_20: return std::make_unique<RclTimerSystem<20>>(name, std::move(callback));
    case Frequency::Hz_27: return std::make_unique<RclTimerSystem<27>>(name, std::move(callback));
    case Frequency::Hz_30: return std::make_unique<RclTimerSystem<30>>(name, std::move(callback));
    case Frequency::Hz_50: return std::make_unique<RclTimerSystem<50>>(name, std::move(callback));
    case Frequency::Hz_60: return std::make_unique<RclTimerSystem<60>>(name, std::move(callback));
    case Frequency::Hz_100: return std::make_unique<RclTimerSystem<100>>(name, std::move(callback));
    case Frequency::Hz_120: return std::make_unique<RclTimerSystem<120>>(name, std::move(callback));
    case Frequency::Hz_150: return std::make_unique<RclTimerSystem<150>>(name, std::move(callback));
    case Frequency::Hz_200: return std::make_unique<RclTimerSystem<200>>(name, std::move(callback));
    case Frequency::Hz_250: return std::make_unique<RclTimerSystem<250>>(name, std::move(callback));
    case Frequency::Hz_500: return std::make_unique<RclTimerSystem<500>>(name, std::move(callback));
    case Frequency::Hz_1000:
        return std::make_unique<RclTimerSystem<1000>>(name, std::move(callback));
    }
    std::unreachable();
}

} // namespace talos::scheduler::rclcompat
