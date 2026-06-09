#include "primitive/thread_affinity.hpp"

#include <cstring>

#include <fmt/format.h>

#ifdef __APPLE__
# include <mach/mach.h>
# include <mach/thread_policy.h>
# include <pthread.h>
#elif __linux__
# include <pthread.h>
# include <sched.h>
#endif

namespace talos::primitive {

auto ThreadAffinity::pin_to_core(const std::uint32_t core_id) noexcept -> AffinityResult {
    if (core_id >= get_num_cores()) {
        return std::unexpected(
            fmt::format("pin_to_core: core_id {} >= num_cores {}", core_id, get_num_cores()));
    }

#ifdef __linux__
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(static_cast<int>(core_id), &cpuset);

    pthread_t thread = pthread_self();
    const int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);

    if (result != 0) {
        return std::unexpected(
            fmt::format(
                "pin_to_core: pthread_setaffinity_np(core {}) failed: {}", core_id,
                std::strerror(result)));
    }
    return {};
#elif defined(__APPLE__) && defined(__arm64__)
    // Apple Silicon: THREAD_AFFINITY_POLICY is not supported.
    return {};
#elif defined(__APPLE__)
    // Intel Mac: affinity tag is a scheduling hint, not strict binding.
    thread_affinity_policy_data_t policy = {static_cast<integer_t>(core_id + 1)};
    const thread_port_t mach_thread      = pthread_mach_thread_np(pthread_self());

    const kern_return_t kr = thread_policy_set(
        mach_thread, THREAD_AFFINITY_POLICY, reinterpret_cast<thread_policy_t>(&policy),
        THREAD_AFFINITY_POLICY_COUNT);
    if (kr != KERN_SUCCESS) {
        return std::unexpected(
            fmt::format(
                "pin_to_core: thread_policy_set(affinity, tag={}) failed: {} (kern_return={})",
                core_id + 1, mach_error_string(kr), kr));
    }
    return {};
#else
    return std::unexpected("pin_to_core: unsupported platform");
#endif
}

auto ThreadAffinity::pin_thread_to_core(std::thread& thread, const std::uint32_t core_id) noexcept
    -> AffinityResult {
    if (core_id >= get_num_cores()) {
        return std::unexpected(
            fmt::format(
                "pin_thread_to_core: core_id {} >= num_cores {}", core_id, get_num_cores()));
    }

#ifdef __linux__
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(static_cast<int>(core_id), &cpuset);

    const int result = pthread_setaffinity_np(thread.native_handle(), sizeof(cpu_set_t), &cpuset);

    if (result != 0) {
        return std::unexpected(
            fmt::format(
                "pin_thread_to_core: pthread_setaffinity_np(core {}) failed: {}", core_id,
                std::strerror(result)));
    }
    return {};

#elif defined(__APPLE__)
    static_cast<void>(thread);
    static_cast<void>(core_id);
    return std::unexpected(
        "pin_thread_to_core: macOS does not support setting affinity from external thread");
#else
    static_cast<void>(thread);
    static_cast<void>(core_id);
    return std::unexpected("pin_thread_to_core: unsupported platform");
#endif
}

auto ThreadAffinity::set_realtime_priority(const RealtimeConfig config) noexcept -> AffinityResult {
#ifdef __linux__
    struct sched_param param{};
    param.sched_priority = static_cast<int>(config.priority);

    const int result = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (result != 0) {
        return std::unexpected(
            fmt::format(
                "set_realtime_priority: pthread_setschedparam(SCHED_FIFO, prio={}) failed: {}",
                config.priority, std::strerror(result)));
    }
    return {};

#elif defined(__APPLE__)
    thread_time_constraint_policy_data_t policy{};
    policy.period      = 0;
    policy.computation = static_cast<std::uint32_t>(config.computation_ns);
    policy.constraint  = static_cast<std::uint32_t>(config.constraint_ns);
    policy.preemptible = config.preemptible ? TRUE : FALSE;

    const thread_port_t mach_thread = pthread_mach_thread_np(pthread_self());
    const kern_return_t kr          = thread_policy_set(
        mach_thread, THREAD_TIME_CONSTRAINT_POLICY, reinterpret_cast<thread_policy_t>(&policy),
        THREAD_TIME_CONSTRAINT_POLICY_COUNT);

    if (kr != KERN_SUCCESS) {
        return std::unexpected(
            fmt::format(
                "set_realtime_priority: thread_policy_set(time_constraint, comp={}ns, "
                "constr={}ns) failed: {} (kern_return={})",
                config.computation_ns, config.constraint_ns, mach_error_string(kr), kr));
    }
    return {};
#else
    static_cast<void>(config);
    return std::unexpected("set_realtime_priority: unsupported platform");
#endif
}

auto ThreadAffinity::get_num_cores() noexcept -> std::uint32_t {
    return std::thread::hardware_concurrency();
}

auto ThreadAffinity::get_current_core() noexcept -> CoreIdResult {
#ifdef __linux__
    const int cpu = sched_getcpu();
    if (cpu < 0) {
        return std::unexpected("get_current_core: sched_getcpu() failed");
    }
    return static_cast<std::uint32_t>(cpu);
#elif defined(__APPLE__)
    return std::unexpected("get_current_core: not available on macOS");
#else
    return std::unexpected("get_current_core: unsupported platform");
#endif
}

} // namespace talos::primitive
