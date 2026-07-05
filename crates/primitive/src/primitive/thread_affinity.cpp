#include "primitive/thread_affinity.hpp"

// C标准库：strerror() 将系统错误码转为可读字符串
#include <cstring>
// fmt格式化库，用于拼接错误提示字符串，替代不安全sprintf
#include <fmt/format.h>

// 区分苹果macOS系统，引入Mach内核与pthread线程API
#ifdef __APPLE__
// Mach内核基础类型、返回码定义
# include <mach/mach.h>
// macOS线程调度、亲和性、实时策略结构体与常量
# include <mach/thread_policy.h>
// POSIX线程封装，mac底层线程依赖pthread
# include <pthread.h>
// Linux系统分支
#elif __linux__
// Linux POSIX线程接口
# include <pthread.h>
// Linux调度相关API：cpu_set_t、CPU_SET、sched_getcpu、SCHED_FIFO等
# include <sched.h>
#endif

// Talos框架底层基础工具命名空间
namespace talos::primitive {

/**
 * @brief 将【当前执行该函数的线程】绑定至指定CPU逻辑核心
 * @param core_id 目标逻辑CPU核心编号，无符号32位整型
 * @return AffinityResult = std::expected<void, std::string>
 *         成功：返回空{}；失败：std::unexpected携带错误描述文本
 * @noexcept 函数不抛出C++异常，所有错误通过返回值传递，适配实时无异常框架
 */
auto ThreadAffinity::pin_to_core(const std::uint32_t core_id) noexcept -> AffinityResult {
    // 合法性校验：目标核心编号不能大于等于系统总逻辑核心数，否则越界
    if (core_id >= get_num_cores()) {
        // 构造错误返回对象，格式化输出错误信息：传入非法core_id与系统最大核心数
        return std::unexpected(
            fmt::format("pin_to_core: core_id {} >= num_cores {}", core_id, get_num_cores()));
    }

// Linux平台专属实现分支
#ifdef __linux__
    // cpu_set_t：Linux内核CPU掩码类型，用来标记一组需要绑定的CPU核心
    cpu_set_t cpuset;
    // CPU_ZERO：清空掩码，初始化所有CPU标记为未选中
    CPU_ZERO(&cpuset);
    // CPU_SET：在掩码中标记指定core_id核心为选中状态
    // 强转int：内核API要求参数为int类型
    CPU_SET(static_cast<int>(core_id), &cpuset);

    // pthread_self()：获取当前调用线程的pthread原生句柄
    pthread_t self_tid = pthread_self();
    // pthread_setaffinity_np：Linux非标准扩展API，设置线程CPU亲和掩码
    // 参数1：目标线程句柄；参数2：掩码结构体字节大小；参数3：CPU掩码
    // 作用：强制内核仅将该线程调度到掩码内的CPU运行，严格锁核
    const int ret = pthread_setaffinity_np(self_tid, sizeof(cpu_set_t), &cpuset);
    // 系统调用返回非0代表操作失败，ret存储errno错误码
    if (ret != 0) {
        // 拼接错误信息，strerror把数字错误码转人类可读文本
        return std::unexpected(
            fmt::format("pin_to_core: pthread_setaffinity_np(core {}) failed: {}", core_id, std::strerror(ret)));
    }
    // 绑定成功，返回空expected，代表无错误
    return {};

// Apple Silicon M系列arm64芯片分支
#elif defined(__APPLE__) && defined(__arm64__)
    // M系列macOS内核移除THREAD_AFFINITY_POLICY亲和策略，不支持硬锁CPU核心
    // 无报错直接返回，仅无法实现绑核，不阻断业务运行
    return {};
// Intel x86架构Mac分支
#elif defined(__APPLE__)
    // thread_affinity_policy_data_t：macOS线程亲和策略结构体
    // macOS亲和tag从1开始，0代表无亲和约束，因此core_id需要+1偏移
    thread_affinity_policy_data_t policy = {static_cast<integer_t>(core_id + 1)};
    // pthread_mach_thread_np：将pthread句柄转换为Mach内核原生线程port
    thread_port_t mach_tid = pthread_mach_thread_np(pthread_self());

    // thread_policy_set：Mach内核API，设置线程调度策略
    // 参数1：mach线程port；参数2：策略类型CPU亲和；参数3：策略结构体指针；参数4：结构体成员数量
    kern_return_t kr = thread_policy_set(
        mach_tid,
        THREAD_AFFINITY_POLICY,
        reinterpret_cast<thread_policy_t>(&policy),
        THREAD_AFFINITY_POLICY_COUNT
    );

    // kr != KERN_SUCCESS 代表内核调用失败
    if (kr != KERN_SUCCESS) {
        // mach_error_string：将Mach内核数字返回码转为文字描述
        return std::unexpected(
            fmt::format("pin_to_core: thread_policy_set tag={} failed: {} (kr={})",
                        core_id + 1, mach_error_string(kr), kr));
    }
    // 亲和策略设置完成（仅调度提示，非严格锁核）
    return {};
// Windows/其他未适配操作系统
#else
    // 当前平台无CPU亲和实现，返回不支持平台错误
    return std::unexpected("pin_to_core: unsupported platform");
#endif
}

/**
 * @brief 传入外部std::thread线程对象，对其他线程设置CPU亲和绑定
 * @param thread 已启动运行的std::thread线程实例引用
 * @param core_id 目标绑定逻辑核心编号
 * @return AffinityResult 成功/错误字符串
 * @note macOS系统不允许跨线程修改调度策略，仅Linux可用
 */
auto ThreadAffinity::pin_thread_to_core(std::thread& thread, const std::uint32_t core_id) noexcept
    -> AffinityResult {
    // 前置校验核心ID合法性，避免无效参数
    if (core_id >= get_num_cores()) {
        return std::unexpected(
            fmt::format("pin_thread_to_core: core_id {} >= num_cores {}", core_id, get_num_cores()));
    }

#ifdef __linux__
    // 定义CPU掩码集合
    cpu_set_t cpuset;
    // 清空掩码所有标记位
    CPU_ZERO(&cpuset);
    // 选中指定核心写入掩码
    CPU_SET(static_cast<int>(core_id), &cpuset);

    // native_handle()：提取std::thread底层封装的pthread_t原生句柄，用于操作外部线程
    pthread_t tid = thread.native_handle();
    // 调用Linux系统API修改其他线程的CPU亲和掩码
    const int ret = pthread_setaffinity_np(tid, sizeof(cpu_set_t), &cpuset);
    // 判断系统调用失败
    if (ret != 0) {
        return std::unexpected(
            fmt::format("pin_thread_to_core pthread_setaffinity_np core {} err: {}", core_id, std::strerror(ret)));
    }
    // 外部线程绑核成功
    return {};

#elif defined(__APPLE__)
    // macOS系统限制：不支持在线程外部修改其他线程调度策略
    // static_cast<void>消除未使用参数编译警告
    static_cast<void>(thread);
    static_cast<void>(core_id);
    // 返回明确错误提示
    return std::unexpected("pin_thread_to_core: macOS cannot set affinity for external thread");
#else
    // 其他平台无实现，静默丢弃参数并返回错误
    static_cast<void>(thread);
    static_cast<void>(core_id);
    return std::unexpected("pin_thread_to_core: unsupported platform");
#endif
}

/**
 * @brief 将当前调用线程设置为实时调度模式，降低调度延迟，用于机器人控制/视觉任务
 * @param config RealtimeConfig实时调度配置结构体
 *        Linux：使用priority优先级；macOS：使用计算时长、截止约束纳秒参数
 * @return AffinityResult 成功无返回值，失败携带错误文本
 * @note Linux需要sudo权限或提升实时调度权限才能生效
 */
auto ThreadAffinity::set_realtime_priority(const RealtimeConfig config) noexcept -> AffinityResult {
#ifdef __linux__
    // sched_param：Linux调度参数结构体，存放实时优先级
    sched_param param{};
    // 将配置中的优先级写入调度参数，强转为int适配内核接口
    param.sched_priority = static_cast<int>(config.priority);

    // pthread_setschedparam：设置当前线程调度策略与优先级
    // 参数1：当前线程句柄；参数2：调度策略SCHED_FIFO（先进先出实时）；参数3：优先级参数
    int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    // 系统调用失败判断
    if (ret != 0) {
        return std::unexpected(
            fmt::format("set_realtime_priority SCHED_FIFO prio={} failed: {}",
                        config.priority, std::strerror(ret)));
    }
    // 实时调度设置成功
    return {};

#elif defined(__APPLE__)
    // macOS时间约束实时策略结构体，用于硬低延迟任务
    thread_time_constraint_policy_data_t policy{};
    policy.period      = 0; // 任务周期，0代表非周期性实时任务
    // 单次任务预期执行时长，单位纳秒
    policy.computation = static_cast<std::uint32_t>(config.computation_ns);
    // 任务硬截止约束时长，超过则内核优先调度该线程
    policy.constraint  = static_cast<std::uint32_t>(config.constraint_ns);
    // 是否允许被更高优先级实时线程抢占，布尔转内核TRUE/FALSE常量
    policy.preemptible = config.preemptible ? TRUE : FALSE;

    // 获取当前线程Mach内核port
    thread_port_t mach_tid = pthread_mach_thread_np(pthread_self());
    // 应用时间约束实时调度策略
    kern_return_t kr = thread_policy_set(
        mach_tid,
        THREAD_TIME_CONSTRAINT_POLICY,
        reinterpret_cast<thread_policy_t>(&policy),
        THREAD_TIME_CONSTRAINT_POLICY_COUNT
    );

    // 内核策略设置失败
    if (kr != KERN_SUCCESS) {
        return std::unexpected(
            fmt::format("set_realtime_policy comp={}ns constr={}ns err: {} kr={}",
                        config.computation_ns, config.constraint_ns, mach_error_string(kr), kr));
    }
    // macOS实时策略配置完成
    return {};
#else
    // 其他平台无实时调度实现，丢弃配置参数消除警告
    static_cast<void>(config);
    return std::unexpected("set_realtime_priority: unsupported platform");
#endif
}

/**
 * @brief 获取系统硬件逻辑CPU核心总数（包含超线程虚拟核心）
 * @return std::uint32_t 逻辑核心数量
 */
auto ThreadAffinity::get_num_cores() noexcept -> std::uint32_t {
    // C++标准跨平台接口，自动读取CPU硬件并发数，Linux/macOS通用
    return std::thread::hardware_concurrency();
}

/**
 * @brief 查询当前线程此刻正在运行的逻辑CPU核心ID
 * @return CoreIdResult = std::expected<uint32_t, string>
 *         成功返回核心编号；macOS/其他平台返回不支持错误
 */
auto ThreadAffinity::get_current_core() noexcept -> CoreIdResult {
#ifdef __linux__
    // sched_getcpu()：Linux专属系统调用，返回当前线程所在CPU编号，失败返回负数
    int cpu = sched_getcpu();
    // 系统调用出错判断
    if (cpu < 0) {
        return std::unexpected("get_current_core: sched_getcpu() failed");
    }
    // 转换为无符号整型封装进expected成功分支返回
    return static_cast<std::uint32_t>(cpu);
#elif defined(__APPLE__)
    // macOS无系统调用获取当前运行CPU，直接返回错误
    return std::unexpected("get_current_core: not available on macOS");
#else
    // 其余平台不支持该接口
    return std::unexpected("get_current_core: unsupported platform");
#endif
}

} // namespace talos::primitive