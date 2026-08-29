#include "scheduler.hpp"

// Intel TBB 多线程任务调度库，用于并行执行Compute计算系统
#include <tbb/task_arena.h>
#include <tbb/task_group.h>

// 工具头文件：C++模板类型名消繁，打印可读类型名
#include "demangle.hpp"
// 底层性能工具：执行耗时统计探针、CPU自旋等待、CPU核心亲和绑定工具
#include "primitive/performance_probe.hpp"
#include "primitive/spin.hpp"
#include "primitive/thread_affinity.hpp"

// C++标准库
#include <algorithm>         // std::any_of、std::sort、std::unique 通用算法
#include <bit>               // C++20 位运算工具：std::popcount、std::countr_zero
#include <cinttypes>         // PRIu64 格式化uint64_t打印宏定义
#include <cstdio>            // printf 控制台打印Mermaid图、系统信息
#include <functional>        // std::function 可调用对象、闭包存储
#include <limits>            // 数值极值模板类
#include <map>               // 有序关联容器，构建类型/通道名称映射
#include <nlohmann/json.hpp> // JSON序列化库，导出性能指标
#include <spdlog/spdlog.h>   // 日志库，输出警告/错误日志
#include <unordered_map>     // 哈希表，加速话题计数查找

namespace talos::scheduler {

// ============================================================================
// 内部通用辅助工具函数 Internal Helpers
// 纯静态工具，无成员依赖，用于打印、统计通道、构建绘图标签
// ============================================================================

/// 打印系统唤醒依赖链路
/// @param title 打印区块标题字符串
/// @param source_systems 源系统数组（FixedRate定时系统 / Compute计算系统）
/// @param affects_mask 每个源系统对应的下游目标系统位掩码数组
/// @param target_systems 目标系统数组，用于根据数组下标查询系统名称
template <typename SourceSystemEntry>
static void print_wake_chains(
    const char* title, const std::vector<SourceSystemEntry>& source_systems,
    const std::vector<std::uint64_t>& affects_mask,
    const std::vector<SystemEntry>& target_systems) {
    // 打印标题分隔行
    printf("%s\n", title);
    // 遍历每一个源系统
    for (std::size_t i = 0; i < source_systems.size(); ++i) {
        // 获取当前源系统的业务名称
        const auto& src_name = source_systems[i].system->meta().name;
        // 越界保护：下标超出掩码数组长度则置0，表示无下游依赖
        const auto affects = i < affects_mask.size() ? affects_mask[i] : 0U;

        printf("  [%s] -> ", src_name.c_str());
        // 掩码为0：没有任何下游依赖系统
        if (affects == 0) {
            printf("(none)");
        } else {
            bool first = true;
            // 遍历全部Compute目标系统，检查对应bit是否置1
            for (std::size_t j = 0; j < target_systems.size(); ++j) {
                if (affects & (1UL << j)) {
                    // 多个目标用逗号分隔，第一个不加逗号
                    if (!first)
                        printf(", ");
                    printf("[%s]", target_systems[j].system->meta().name.c_str());
                    first = false;
                }
            }
        }
        printf("\n");
    }
}

/// 通道IO统计结构体：记录单个系统的读写通道数量
struct ChannelIoCounts {
    std::size_t inputs  = 0; // 读通道总数（spsc_reader / spmc_reader）
    std::size_t outputs = 0; // 写通道总数（spsc_writer / spmc_writer）
};

/// 累加一组通道的读写数量，写入counts（noexcept 保证无异常抛出）
template <typename Channels>
static void
    accumulate_channel_io_counts(const Channels& channels, ChannelIoCounts& counts) noexcept {
    for (const auto& channel : channels) {
        // 根据通道类型区分输入/输出
        switch (channel.kind) {
        // 读通道，输入计数+1
        case channel_kind::spsc_reader:
        case channel_kind::spmc_reader: counts.inputs++; break;
        // 写通道，输出计数+1
        case channel_kind::spsc_writer:
        case channel_kind::spmc_writer: counts.outputs++; break;
        // 非法通道类型，直接终止程序，不可能走到此处
        default: std::unreachable();
        }
    }
}

/// 统计单个系统全部SPSC+SPMC通道的读写总数
static auto count_channel_io(const SystemMeta& meta) noexcept -> ChannelIoCounts {
    ChannelIoCounts counts;
    // 统计SPSC通道
    accumulate_channel_io_counts(meta.spsc_channels, counts);
    // 统计SPMC通道
    accumulate_channel_io_counts(meta.spmc_channels, counts);
    return counts;
}

/// 编译期常量函数：判断通道类型是否为写者通道
static constexpr auto is_writer_channel_kind(const channel_kind kind) noexcept -> bool {
    return kind == channel_kind::spsc_writer || kind == channel_kind::spmc_writer;
}

/// 判断一组通道内是否存在写者通道
template <typename Channels>
static auto has_writer_channels(const Channels& channels) noexcept -> bool {
    // 遍历通道，存在任意写通道返回true
    return std::any_of(channels.begin(), channels.end(), [](const auto& channel) {
        return is_writer_channel_kind(channel.kind);
    });
}

/// 判断系统是否拥有输出写通道（run返回true时会唤醒下游依赖系统）
static auto counts_written_calls(const SystemMeta& meta) noexcept -> bool {
    return has_writer_channels(meta.spsc_channels) || has_writer_channels(meta.spmc_channels);
}

/// 通道绘图标签结构体：存储消繁后的话题名、数据类型名
struct ChannelLabelNames {
    std::string topic_name; // 话题type_index消繁可读字符串
    std::string type_name;  // 数据承载类型type_index消繁可读字符串
};

/// 构建全局共享资源（ECS只读/可变组件）type_index → 可读名称映射，用于Mermaid绘图
template <typename Systems>
static auto build_resource_label_map(const Systems& systems)
    -> std::map<std::type_index, std::string> {
    std::map<std::type_index, std::string> resource_labels;
    for (const auto& entry : systems) {
        const auto& meta = entry.system->meta();
        // 遍历系统只读资源
        for (const auto& type : meta.reads) {
            // 消繁类型名并插入映射，已存在则不覆盖
            resource_labels.try_emplace(type, talos::scheduler::detail::demangle(type.name()));
        }
        // 遍历系统可变修改资源
        for (const auto& type : meta.writes) {
            resource_labels.try_emplace(type, talos::scheduler::detail::demangle(type.name()));
        }
    }
    return resource_labels;
}

/// 收集所有通道的唯一Key与消繁名称，存入有序map
template <typename Systems>
static void collect_channel_label_names(
    const Systems& systems, std::map<ChannelKey, ChannelLabelNames>& channel_names) {
    for (const auto& entry : systems) {
        const auto& meta = entry.system->meta();
        // 内部复用闭包，统一处理SPSC/SPMC两类通道
        auto collect = [&](const auto& channels) {
            for (const auto& channel : channels) {
                // 通道唯一键：数据类型 + 话题类型
                const ChannelKey key{channel.type, channel.topic};
                // 插入通道名称信息，key重复会覆盖旧值
                channel_names.try_emplace(
                    key, ChannelLabelNames{
                             .topic_name = talos::scheduler::detail::demangle(channel.topic.name()),
                             .type_name  = talos::scheduler::detail::demangle(channel.type.name()),
                         });
            }
        };
        collect(meta.spsc_channels);
        collect(meta.spmc_channels);
    }
}

/// 构建通道唯一key → Mermaid绘图展示标签映射
/// 规则：同一话题存在多种数据类型时，标签追加[类型名]区分
template <typename FixedSystems, typename ComputeSystems>
static auto build_channel_label_map(
    const FixedSystems& fixed_systems, const ComputeSystems& compute_systems)
    -> std::map<ChannelKey, std::string> {
    std::map<ChannelKey, ChannelLabelNames> channel_names;
    // 收集定时系统通道名称
    collect_channel_label_names(fixed_systems, channel_names);
    // 收集计算系统通道名称
    collect_channel_label_names(compute_systems, channel_names);

    // 统计每个话题名出现次数，用于判断是否需要追加类型后缀
    std::unordered_map<std::string, std::size_t> topic_name_counts;
    for (const auto& [_, names] : channel_names) {
        topic_name_counts[names.topic_name]++;
    }

    std::map<ChannelKey, std::string> channel_labels;
    for (const auto& [key, names] : channel_names) {
        std::string label = names.topic_name;
        // 同名话题存在多个数据类型，拼接类型名区分
        if (topic_name_counts[names.topic_name] > 1) {
            label += " [";
            label += names.type_name;
            label += "]";
        }
        channel_labels.emplace(key, std::move(label));
    }

    return channel_labels;
}

/// 拓扑未构建时打印Mermaid提示图
static void print_mermaid_rebuild_hint(const char* message) {
    printf("```mermaid\n");
    printf("flowchart LR\n\n");
    printf("    BuildRequired[\"%s\"]\n", message);
    printf("```\n");
}

// ============================================================================
// Scheduler 构造/析构函数 Constructor / Destructor
// ============================================================================

/// 调度器构造函数：接收全局配置，初始化性能统计起始时间戳
Scheduler::Scheduler(const SchedulerConfig config) noexcept
    : config_(config)
    , stats_start_time_(Clock::now()) {}

/// 调度器析构函数：停止调度，等待所有定时线程安全退出
Scheduler::~Scheduler() noexcept {
    stop();
    // 遍历所有定时线程，等待线程执行完毕回收资源
    for (auto& t : fixed_rate_threads_) {
        if (t.joinable()) {
            t.join();
        }
    }
}

/// FixedRate系统统计结构体移动构造（原子变量仅读取，不转移原子本身）
Scheduler::FixedRateStats::FixedRateStats(FixedRateStats&& other) noexcept
    // 宽松内存序读取原子计数
    : notify_count(other.notify_count.load(std::memory_order_relaxed))
    , execution_count(other.execution_count.load(std::memory_order_relaxed))
    , record_count(other.record_count.load(std::memory_order_relaxed))
    // 移动延迟直方图容器
    , latency_hist(std::move(other.latency_hist)) {}

/// Compute计算系统统计结构体移动构造
Scheduler::ComputeStats::ComputeStats(ComputeStats&& other) noexcept
    : run_count(other.run_count.load(std::memory_order_relaxed))
    , latency_hist(std::move(other.latency_hist)) {}

/// 获取顶层World资源容器（存储所有共享资源、通道缓冲区）
World& Scheduler::world() noexcept { return world_; }

/// 判断生命周期状态是否为运行中
bool Scheduler::is_running_state(const LifecycleState state) noexcept {
    return state == LifecycleState::Running;
}

/// 运行时保护校验：新增系统前必须保证调度器未启动
void Scheduler::ensure_not_running() const noexcept {
    // 原子加载生命周期状态（获取语义，同步其他线程修改）
    /*std::memory_order_acquire：内存序约束
    保证：本 load 之后的所有读写操作，不会被编译器 / CPU 重排到 load 前面；
    多线程场景下，能完整看到其他线程修改lifecycle_前的所有资源修改，避免数据可见性 bug。*/
    if (is_running_state(lifecycle_.load(std::memory_order_acquire))) {
        panic("add_system: system already running!");
    }
}

/// 标记拓扑为脏状态：新增系统后依赖图失效，需要重新build构建拓扑
void Scheduler::mark_topology_dirty() noexcept {
    // 释放语义存储，同步其他线程读取
    /*lifecycle_
    成员变量，类型 std::atomic<LifecycleState>
    原子类型，多线程安全共享调度器生命周期状态，无需手动加锁即可跨线程读写。
    .store(值, 内存序)
    原子写入接口：把第一个参数存入原子变量。
    第一个参数：要写入的枚举状态 LifecycleState::Configuring（配置初始化阶段）。
    std::memory_order_release 释放内存序
    核心规则：
    当前线程中，本句 store 之前的所有读写操作，不会被编译器 / CPU 重排到 store 之后；
    其他线程使用 memory_order_acquire 读取这个原子变量时，可以完整看到本线程在 store
    之前完成的所有数据修改。*/
    lifecycle_.store(LifecycleState::Configuring, std::memory_order_release);
}

// ============================================================================
// 调度器全生命周期管理 Lifecycle
// 注册系统、构建拓扑、启动、停止、热更系统核心逻辑
// ============================================================================

/// 注册业务系统：区分FixedRate定时系统/Compute计算系统存入对应数组
/// @param system 系统唯一所有权智能指针
/// @return std::expected 成功返回系统数组下标，失败返回调度器错误码
auto Scheduler::add_system(std::unique_ptr<SystemBase> system)
    -> std::expected<uint64_t, SchedulerError> {
    // 空系统直接崩溃，非法入参
    if (!system) {
        std::abort();
    }
    // 校验调度器未运行
    ensure_not_running();
    // 标记拓扑需要重建
    mark_topology_dirty();

    // 提取系统调度策略，封装系统存储条目
    const auto policy = system->meta().policy;
    SystemEntry entry{
        .system = std::move(system), // 转移系统所有权
        .policy = policy,
        .bound  = false,             // 标记通道是否完成绑定初始化
    };

    // 定时周期系统存入fixed数组
    if (policy.is_fixed_rate()) {
        fixed_rate_systems_.push_back(std::move(entry));
        // 返回当前数组最后一位下标
        return static_cast<uint64_t>(fixed_rate_systems_.size() - 1);
    }
    // 线程池并行/可视化计算系统存入compute数组
    if (policy.is_pool() || policy.is_visualization()) {
        compute_systems_.push_back(std::move(entry));
        return static_cast<uint64_t>(compute_systems_.size() - 1);
    }
    // 不存在其他调度策略，走到此处为代码逻辑错误
    std::unreachable();
}

/// 构建系统拓扑：校验通道冲突、构建依赖分层、初始化TBB线程池、绑定所有通道
auto Scheduler::build() -> BuildResult {
    // 运行中禁止重新构建拓扑
    if (is_running()) {
        return std::unexpected(BuildError{SchedulerError::AlreadyRunning});
    }

    // 第一步：生成拓扑快照，完成通道合法性校验、依赖环检测、分层计算
    /*扫描所有系统的通道读写
    → 校验合法性（多写者/多读者SPSC/孤儿读者）
    → 建依赖邻接掩码 adj_mask
    → Kahn 拓扑排序生成 levels
    → 环检测
    → 可达性校验
    → 预生成唤醒掩码 fixed_rate_affects / compute_affects*/
    auto topology_result = build_topology_snapshot();
    if (!topology_result) {
        // 拓扑校验失败，向上抛出错误
        return std::unexpected(topology_result.error());
    }

    // 仅第一次build时创建TBB任务池，避免重复创建
    if (!compute_arena_) {
        if (config_.compute_concurrency > 0) {
            // 用户指定并行核心数，创建固定并发TBB arena
            compute_arena_ =
                std::make_unique<tbb::task_arena>(static_cast<int>(config_.compute_concurrency));
        } else {
            // 自动适配CPU物理核心数量
            compute_arena_ = std::make_unique<tbb::task_arena>();
        }
    }

    // 统计数组扩容，与系统数量一一对应
    fixed_rate_stats_.resize(fixed_rate_systems_.size());
    compute_stats_.resize(compute_systems_.size());

    // 开启通道绑定模式：分配SPSC/SPMC通道缓冲区，禁止资源修改
    world_.open_channel_binding();
    // 内部闭包批量绑定所有未初始化的系统通道
    auto bind_all_systems = [&](auto& systems) {
        for (auto& entry : systems) {
            if (!entry.bound) {
                entry.system->bind(world_);
                entry.bound = true;
            }
        }
    };
    bind_all_systems(fixed_rate_systems_);
    bind_all_systems(compute_systems_);
    world_.close_channel_binding();

    // 绑定外部IO触发式计算源（外部信号主动唤醒Compute系统）
    bind_external_compute_sources();
    // 冻结共享资源内存布局，后续不可新增/删除资源
    world_.freeze_resource_structure();

    // 保存拓扑分层、系统依赖位掩码
    commit_topology(std::move(*topology_result));
    // 生命周期切换为已构建完成状态
    lifecycle_.store(LifecycleState::Built, std::memory_order_release);
    return {};
}

/// 启动调度器：创建所有定时线程，主线程阻塞执行Compute并行循环
auto Scheduler::run() -> std::expected<void, SchedulerError> {
    // CAS原子交换生命周期：仅Built状态允许切换为Running
    auto expected = LifecycleState::Built;
    /*CAS（Compare And
    Swap，比较并交换）原子操作，无锁并发编程核心原语，全程原子执行、不会被线程打断。
    函数行为分为两步，整体不可分割：
    1.比较：判断原子变量当前值是否等于传入的「预期值 expected」
    2.交换
    相等：把原子变量更新为「新值 desired」，返回 true；
    不相等：把原子变量当前值写入 expected，返回 false。*/
    if (!lifecycle_.compare_exchange_strong(
            expected, LifecycleState::Running, std::memory_order_acq_rel,
            std::memory_order_acquire)) 
    {
        // CAS交换失败，判断当前实际状态
        if (expected == LifecycleState::Running) {
            return std::unexpected(SchedulerError::AlreadyRunning);
        }
        // 未执行build，拓扑未初始化，禁止启动
        return std::unexpected(SchedulerError::NotBuilt);
    }

    // 批量创建所有FixedRate定时后台线程
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        auto& entry = fixed_rate_systems_[i];
        // 线程运行上下文，保存系统、调度器、生命周期等指针
        FixedRateContext ctx{
            .system       = entry.system.get(),
            .policy       = entry.policy,
            .lifecycle    = &lifecycle_,
            .scheduler    = this,
            .world        = &world_,
            .system_index = i,
        };
        // 生成后台线程，执行定时循环函数
        fixed_rate_threads_.emplace_back([ctx]() mutable { run_fixed_rate_thread(ctx); });
    }

    // 主线程阻塞，持续执行计算循环
    run_compute_loop();

    // 主线程循环退出后，等待所有定时线程安全结束
    for (auto& t : fixed_rate_threads_) {
        if (t.joinable()) {
            t.join();
        }
    }
    fixed_rate_threads_.clear();

    return {};
}

/// 停止调度器：生命周期切回Built，唤醒阻塞等待的计算循环
/* T 为原子封装类型，如 enum、int、bool
bool compare_exchange_strong(
    T& expected,          // 传入预期值，失败时会被改写为当前原子真实值
    T desired,             // CAS 成功后要写入的新值
    std::memory_order success,  // CAS 修改成功时的内存序
    std::memory_order failure  // CAS 修改失败时的内存序
) noexcept;*/
/*最简小例子
std::atomic<int> num = 5;
int exp = 5;
// 预期5，想改成10
bool ok = num.compare_exchange_strong(exp, 10);
num 现在是 5 == exp，修改成功 → num=10，ok=true

std::atomic<int> num = 8;
int exp = 5;
bool ok = num.compare_exchange_strong(exp, 10);
num=8≠5，修改失败 → exp 自动变成 8，ok=false*/
void Scheduler::stop() noexcept {
    // CAS将Running状态切回Built
    auto expected = LifecycleState::Running;
    lifecycle_.compare_exchange_strong(
        expected, LifecycleState::Built, std::memory_order_acq_rel, std::memory_order_acquire);

    // 清除暂停标记，唤醒等待的计算循环线程
    /*1. 最外层一对 {}：局部作用域（代码块）
    单独用大括号凭空造出一个临时作用域，作用只有一个：
    控制 lock_guard 的生命周期，出括号自动解锁。*/
    {
        std::lock_guard lock(pause_mutex_);
        pause_requested_.store(false, std::memory_order_release);
    }
    pause_cv_.notify_all();
}

/// 注册停止钩子函数：调度器关闭退出前执行自定义清理逻辑
void Scheduler::add_shutdown_hook(std::function<void()> hook) {
    shutdown_hooks_.push_back(std::move(hook));
}

/// 安全热新增系统：仅调度器未运行时可用，新增后自动重建拓扑
auto Scheduler::hot_add_system(std::unique_ptr<SystemBase> system) -> BuildResult {
    if (!system) {
        std::abort();
    }

    // 运行状态禁止安全热更
    if (is_running()) {
        panic(
            "hot_add_system() while scheduler is running is unsafe; use "
            "unsafe_hot_add_system() if you need the explicit escape hatch");
    }

    // 注册系统并重新构建拓扑
    if (auto result = add_system(std::move(system)); !result) {
        return std::unexpected(BuildError{result.error()});
    }
    return build();
}

/// 非安全热新增系统：调度器运行时暂停计算循环，新增系统、重建拓扑、恢复循环
auto Scheduler::unsafe_hot_add_system(std::unique_ptr<SystemBase> system) -> BuildResult {
    if (!system) {
        std::abort();
    }

    // 调度器未运行，降级调用安全热更接口
    if (!is_running()) {
        return hot_add_system(std::move(system));
    }

    // 请求暂停Compute主线程，阻塞直到循环进入暂停状态
    request_compute_loop_pause();

    // 封装新增系统条目，存入对应数组
    const auto policy = system->meta().policy;
    SystemEntry entry{
        .system = std::move(system),
        .policy = policy,
        .bound  = false,
    };

    const bool is_fixed_rate = policy.is_fixed_rate();
    if (is_fixed_rate) {
        fixed_rate_systems_.push_back(std::move(entry));
    } else if (policy.is_pool() || policy.is_visualization()) {
        compute_systems_.push_back(std::move(entry));
    } else {
        std::unreachable();
    }

    // 重新校验、生成拓扑快照
    auto topology_result = build_topology_snapshot();
    if (!topology_result) {
        // 拓扑构建失败，回滚：删除刚刚新增的系统
        if (is_fixed_rate) {
            fixed_rate_systems_.pop_back();
        } else {
            compute_systems_.pop_back();
        }
        // 恢复计算循环运行
        resume_compute_loop();
        return std::unexpected(topology_result.error());
    }

    // 扩容性能统计数组，匹配新增系统数量
    fixed_rate_stats_.resize(fixed_rate_systems_.size());
    compute_stats_.resize(compute_systems_.size());

    // 绑定新系统的通道缓冲区
    world_.open_channel_binding();
    if (is_fixed_rate) {
        auto& new_entry = fixed_rate_systems_.back();
        if (!new_entry.bound) {
            new_entry.system->bind(world_);
            new_entry.bound = true;
        }
        // 为新增定时系统启动独立后台线程
        const auto idx = fixed_rate_systems_.size() - 1;
        FixedRateContext ctx{
            .system       = new_entry.system.get(),
            .policy       = new_entry.policy,
            .lifecycle    = &lifecycle_,
            .scheduler    = this,
            .world        = &world_,
            .system_index = idx,
        };
        fixed_rate_threads_.emplace_back([ctx]() mutable { run_fixed_rate_thread(ctx); });
    } else {
        // 计算系统仅绑定通道，无独立线程，由主线程TBB池调度
        auto& new_entry = compute_systems_.back();
        if (!new_entry.bound) {
            new_entry.system->bind(world_);
            new_entry.bound = true;
        }
    }
    world_.close_channel_binding();
    bind_external_compute_sources();
    commit_topology(std::move(*topology_result));

    // 解除暂停，恢复计算循环执行
    resume_compute_loop();

    return {};
}

/// 判断调度器当前是否处于运行状态
auto Scheduler::is_running() const noexcept -> bool {
    return is_running_state(lifecycle_.load(std::memory_order_acquire));
}

/// 获取调度器当前生命周期状态
auto Scheduler::lifecycle_state() const noexcept -> LifecycleState {
    return lifecycle_.load(std::memory_order_acquire);
}

/// 保存拓扑快照：分层信息、定时系统依赖掩码、计算系统依赖掩码
void Scheduler::commit_topology(TopologySnapshot snapshot) noexcept {
    levels_             = std::move(snapshot.levels);
    fixed_rate_affects_ = std::move(snapshot.fixed_rate_affects);
    compute_affects_    = std::move(snapshot.compute_affects);
}

// ============================================================================
// 热更配套：暂停/恢复计算循环工具函数 Pause/Resume Helpers
// ============================================================================

/// 请求暂停计算主线程，阻塞等待循环进入暂停状态
void Scheduler::request_compute_loop_pause() noexcept {
    // 设置暂停请求标记
    {
        std::lock_guard lock(pause_mutex_);
        pause_requested_.store(true, std::memory_order_release);
    }
    // 条件变量阻塞，等待compute_loop置位paused标记
    {
        std::unique_lock lock(pause_mutex_);
        pause_cv_.wait(lock, [this] { return paused_.load(std::memory_order_acquire); });
    }
}

/// 恢复计算循环正常运行
void Scheduler::resume_compute_loop() noexcept {
    // 清除暂停请求标记
    {
        std::lock_guard lock(pause_mutex_);
        pause_requested_.store(false, std::memory_order_release);
    }
    // 唤醒等待在条件变量上的主线程
    pause_cv_.notify_all();
}

/// 累加所有FixedRate系统的唤醒触发总次数
std::uint64_t Scheduler::sum_notify_counts() const noexcept {
    std::uint64_t total = 0;
    for (const auto& s : fixed_rate_stats_) {
        total += s.notify_count.load(std::memory_order_relaxed);
    }
    return total;
}

/// 绑定外部计算源就绪信号槽：外部IO主动置位就绪位掩码唤醒系统
void Scheduler::bind_external_compute_sources() noexcept {
    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        // 判断系统是否为外部触发源
        if (auto* external = compute_systems_[i].system->as_external_compute(); external != nullptr) {
            // 绑定就绪位掩码与自身系统下标
            external->bind_external_ready_slot(&ready_systems_, i);
        }
    }
}

// ============================================================================
// 系统唤醒通知 Notification
// FixedRate系统执行完成后，标记下游Compute系统就绪
// ============================================================================

/// 唤醒指定下标定时系统的所有下游依赖计算系统
/// @param system_index 源FixedRate系统数组下标
void Scheduler::notify(const std::size_t system_index) noexcept {
    // 原子或操作，将下游依赖bit写入就绪掩码
    if (system_index < fixed_rate_affects_.size()) {
        ready_systems_.fetch_or(fixed_rate_affects_[system_index], std::memory_order_release);
    }
    // 累加该系统的唤醒触发统计计数
    if (system_index < fixed_rate_stats_.size()) {
        fixed_rate_stats_[system_index].notify_count.fetch_add(1, std::memory_order_relaxed);
    }
}

// ============================================================================
// 性能统计对外接口 Stats
// ============================================================================

/// 获取全局调度器性能统计快照
auto Scheduler::stats() const noexcept -> Stats {
    return Stats{
        .notify_count        = sum_notify_counts(),
        .compute_cycle_count = compute_cycles_.load(std::memory_order_relaxed),
    };
}

/// 控制台打印所有系统元数据、通道数量、执行分层、唤醒依赖链路
void Scheduler::print_systems() const {
    // 通用打印模板闭包，消除重复循环代码
    auto print_systems = [](const auto& systems, const char* type_name, auto print_fn) {
        printf("%s systems (%zu):\n", type_name, systems.size());
        for (const auto& entry : systems) {
            const auto& m = entry.system->meta();
            print_fn(entry, m);
        }
    };

    // 打印FixedRate定时系统信息：频率、CPU亲和、线程优先级、通道数量
    print_systems(fixed_rate_systems_, "FixedRate", [](const auto& entry, const auto& m) {
        printf(
            "  [%s] freq:%uHz affinity:%d priority:%d notify:%s spsc:%zu spmc:%zu\n",
            m.name.c_str(), entry.policy.frequency_hz, entry.policy.cpu_affinity,
            entry.policy.thread_priority, entry.policy.notifies ? "yes" : "no",
            m.spsc_channels.size(), m.spmc_channels.size());
    });

    // 打印Compute计算系统信息：读写通道、共享资源数量
    print_systems(
        compute_systems_, "Compute", [](const auto& entry [[maybe_unused]], const auto& m) {
            printf(
                "  [%s] spsc:%zu spmc:%zu res:%zu res_mut:%zu\n", m.name.c_str(),
                m.spsc_channels.size(), m.spmc_channels.size(), m.reads.size(), m.writes.size());
        });

    // 打印拓扑并行分层
    printf("Execution levels: %zu\n", levels_.size());
    for (std::size_t i = 0; i < levels_.size(); ++i) {
        printf("  Level %zu: ", i);
        for (const auto idx : levels_[i]) {
            printf("[%s] ", compute_systems_[idx].system->meta().name.c_str());
        }
        printf("\n");
    }

    // 筛选出带唤醒能力的定时系统，提取其下游依赖掩码
    std::vector<std::uint64_t> notifying_fixed_rate_affects(fixed_rate_systems_.size(), 0);
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        if (fixed_rate_systems_[i].policy.notifies) {
            notifying_fixed_rate_affects[i] =
                i < fixed_rate_affects_.size() ? fixed_rate_affects_[i] : 0U;
        }
    }

    // 打印两类唤醒依赖链路
    print_wake_chains(
        "Wake chains (notifying fixed_rate -> compute):", fixed_rate_systems_,
        notifying_fixed_rate_affects, compute_systems_);
    print_wake_chains(
        "Wake chains (compute -> compute):", compute_systems_, compute_affects_, compute_systems_);
}

/// 打印Mermaid唤醒触发依赖图（展示系统间唤醒传递关系）
void Scheduler::print_mermaid_wake_chains() const {
    // 拓扑未构建完成，输出提示Mermaid图
    if (lifecycle_state() == LifecycleState::Configuring) {
        print_mermaid_rebuild_hint(
            "Scheduler graph not built. Call build() to render wake chains.");
        return;
    }

    // Mermaid横向流程图
    printf("```mermaid\n");
    printf("flowchart LR\n\n");

    // 判断系统是否为外部信号触发源
    const auto is_external_compute_source = [&](const std::size_t idx) {
        return compute_systems_[idx].system->as_external_compute() != nullptr;
    };
    // 预构建通道绘图标签映射
    const auto channel_labels = build_channel_label_map(fixed_rate_systems_, compute_systems_);

    // 统计每个Compute系统入边、出边数量，用于系统分类
    std::vector<std::size_t> in_edges(compute_systems_.size(), 0);
    std::vector<std::size_t> out_edges(compute_systems_.size(), 0);

    // 统计每个计算系统的下游出边数量
    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        out_edges[i] = (i < compute_affects_.size()) ? std::popcount(compute_affects_[i]) : 0;
    }

    // 统计计算系统互相依赖的入边
    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if (i != j && (compute_affects_[i] & (1UL << j))) {
                in_edges[j]++;
            }
        }
    }

    // 累加定时系统指向计算系统的入边
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        if (!fixed_rate_systems_[i].policy.notifies) {
            continue;
        }
        auto affects = i < fixed_rate_affects_.size() ? fixed_rate_affects_[i] : 0U;
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if (affects & (1UL << j)) {
                in_edges[j]++;
            }
        }
    }

    // 根据入边/出边数量分类计算系统类型
    auto classify_system = [&](std::size_t idx) -> const char* {
        // 外部信号触发源
        if (is_external_compute_source(idx)) {
            return "external";
        }
        const bool has_out = out_edges[idx] > 0;
        const bool has_in  = in_edges[idx] > 0;

        if (has_out && has_in)
            return "pipeline"; // 有输入、有输出，中间流水线节点
        if (!has_out)
            return "sink";     // 只有输入无输出，终点节点
        return "internal";     // 只有输出无输入，起点节点
    };

    // 判断是否存在可唤醒外部的定时系统
    bool has_notifying_fixed_rate = false;
    for (const auto& entry : fixed_rate_systems_) {
        has_notifying_fixed_rate |= entry.policy.notifies;
    }
    if (has_notifying_fixed_rate) {
        printf("    subgraph ExternalWake[Notifying FixedRate Sources]\n");
        printf("        direction TB\n");
        for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
            if (!fixed_rate_systems_[i].policy.notifies) {
                continue;
            }
            const auto& name = fixed_rate_systems_[i].system->meta().name;
            const auto freq  = fixed_rate_systems_[i].policy.frequency_hz;
            printf("        E%d[\"%s<br/>%dHz\"]:::fixedStyle\n", (int)i, name.c_str(), freq);
        }
        printf("    end\n\n");
    }

    // 判断是否存在仅提供数据、不触发下游的静默定时系统
    bool has_silent_fixed_rate = false;
    for (const auto& sys : fixed_rate_systems_) {
        if (!sys.policy.notifies) {
            has_silent_fixed_rate = true;
            break;
        }
    }
    if (has_silent_fixed_rate) {
        printf("    subgraph SilentInputs[Silent FixedRate Side Inputs]\n");
        printf("        direction TB\n");
        for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
            if (fixed_rate_systems_[i].policy.notifies) {
                continue;
            }
            const auto& name = fixed_rate_systems_[i].system->meta().name;
            const auto freq  = fixed_rate_systems_[i].policy.frequency_hz;
            printf("        E%d[\"%s<br/>%dHz\"]:::fixedStyle\n", (int)i, name.c_str(), freq);
        }
        printf("    end\n\n");
    }

    // 按类型划分所有计算系统下标
    std::vector<std::size_t> external_compute_systems, pipeline_systems, sink_systems,
        internal_systems;
    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        const auto* type = classify_system(i);
        if (strcmp(type, "external") == 0)
            external_compute_systems.push_back(i);
        else if (strcmp(type, "sink") == 0)
            sink_systems.push_back(i);
        else if (strcmp(type, "internal") == 0)
            internal_systems.push_back(i);
        else
            pipeline_systems.push_back(i);
    }

    // 打印外部计算源子图
    if (!external_compute_systems.empty()) {
        printf("    subgraph ExternalCompute[External Compute Sources]\n");
        printf("        direction TB\n");
        for (auto i : external_compute_systems) {
            const auto& name = compute_systems_[i].system->meta().name;
            printf("        C%d[\"%s\"]:::inputStyle\n", (int)i, name.c_str());
        }
        printf("    end\n\n");
    }

    // 打印流水线中间节点子图
    if (!pipeline_systems.empty()) {
        printf("    subgraph Pipeline[Pipeline Systems]\n");
        printf("        direction TB\n");
        for (auto i : pipeline_systems) {
            const auto& name = compute_systems_[i].system->meta().name;
            printf("        C%d[\"%s\"]:::pipelineStyle\n", (int)i, name.c_str());
        }
        printf("    end\n\n");
    }

    // 打印终点Sink系统子图
    if (!sink_systems.empty()) {
        printf("    subgraph Sink[Sink Systems]\n");
        printf("        direction TB\n");
        for (auto i : sink_systems) {
            const auto& name = compute_systems_[i].system->meta().name;
            printf("        C%d[\"%s\"]:::outputStyle\n", (int)i, name.c_str());
        }
        printf("    end\n\n");
    }

    // 打印无输入内部起点系统子图
    if (!internal_systems.empty()) {
        printf("    subgraph Internal[Internal Compute Roots]\n");
        printf("        direction TB\n");
        for (auto i : internal_systems) {
            const auto& name = compute_systems_[i].system->meta().name;
            printf("        C%d[\"%s\"]:::orphanStyle\n", (int)i, name.c_str());
        }
        printf("    end\n\n");
    }

    /// 查找两个系统之间连通的所有通道话题名称
    auto get_channel_topics = [&](const SystemBase* writer,
                                  const SystemBase* reader) -> std::string {
        std::vector<std::string> topics;
        const auto& writer_meta = writer->meta();
        const auto& reader_meta = reader->meta();

        // 遍历两组系统的所有读写通道，匹配同类型同话题通道
        auto check_channels = [&](const auto& writer_channels, const auto& reader_channels) {
            for (const auto& wc : writer_channels) {
                // 仅匹配写通道
                if (wc.kind != channel_kind::spsc_writer && wc.kind != channel_kind::spmc_writer) {
                    continue;
                }
                for (const auto& rc : reader_channels) {
                    // 仅匹配读通道
                    if (rc.kind != channel_kind::spsc_reader
                        && rc.kind != channel_kind::spmc_reader) {
                        continue;
                    }
                    // 通道数据类型、话题完全匹配
                    if (wc.type == rc.type && wc.topic == rc.topic) {
                        const ChannelKey key{wc.type, wc.topic};
                        if (auto it = channel_labels.find(key); it != channel_labels.end()) {
                            topics.push_back(it->second);
                        }
                    }
                }
            }
        };

        // 组合所有通道匹配场景
        check_channels(writer_meta.spsc_channels, reader_meta.spsc_channels);
        check_channels(writer_meta.spsc_channels, reader_meta.spmc_channels);
        check_channels(writer_meta.spmc_channels, reader_meta.spmc_channels);

        // 无连通通道返回空字符串
        if (topics.empty()) {
            return "";
        }
        // 去重、排序通道名称
        std::sort(topics.begin(), topics.end());
        topics.erase(std::unique(topics.begin(), topics.end()), topics.end());
        // 单个通道直接返回名称
        if (topics.size() == 1) {
            return topics[0];
        }
        // 多通道用逗号拼接
        std::string result;
        for (const auto& t : topics) {
            if (!result.empty())
                result += ", ";
            result += t;
        }
        return result;
    };

    // 绘制定时系统 → 计算系统唤醒边
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        if (!fixed_rate_systems_[i].policy.notifies) {
            continue;
        }
        auto affects = i < fixed_rate_affects_.size() ? fixed_rate_affects_[i] : 0U;
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if (affects & (1UL << j)) {
                const auto* writer = fixed_rate_systems_[i].system.get();
                const auto* reader = compute_systems_[j].system.get();
                const auto topic   = get_channel_topics(writer, reader);
                if (topic.empty()) {
                    printf("    E%d --> C%d\n", (int)i, (int)j);
                } else {
                    printf("    E%d -->|\"%s\"| C%d\n", (int)i, topic.c_str(), (int)j);
                }
            }
        }
    }

    // 绘制计算系统互相依赖唤醒边
    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        auto affects = i < compute_affects_.size() ? compute_affects_[i] : 0U;
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if ((affects & (1UL << j)) && i != j) {
                const auto* writer       = compute_systems_[i].system.get();
                const auto* reader       = compute_systems_[j].system.get();
                const auto channel_topic = get_channel_topics(writer, reader);
                if (channel_topic.empty()) {
                    printf("    C%d --> C%d\n", (int)i, (int)j);
                } else {
                    printf("    C%d -->|\"%s\"| C%d\n", (int)i, channel_topic.c_str(), (int)j);
                }
            }
        }
    }

    printf("```\n");
}

// 输出Mermaid横向数据流图：系统+通道消息流+全局共享资源读写关系
void Scheduler::print_mermaid_data_flow() const {
    // Mermaid流程图起始标记
    printf("```mermaid\n");
    // LR = Left Right 横向布局，数据流从左到右
    printf("flowchart LR\n\n");

    // 内部lambda：判断指定下标Compute系统是否为外部信号触发源（外部IO主动唤醒）
    const auto is_external_compute_source = [&](const std::size_t idx) {
        // as_external_compute()非空代表该系统由外部硬件/网络信号驱动，不靠内部通道唤醒
        return compute_systems_[idx].system->as_external_compute() != nullptr;
    };
    // 预构建通道Key -> 绘图显示名称映射（同名话题多类型自动拼接类型后缀）
    const auto channel_labels = build_channel_label_map(fixed_rate_systems_, compute_systems_);

    // 定义10种不同颜色，每条唯一通道使用统一颜色，区分不同数据流
    const char* path_colors[] = {
        "#e53935", // Red 红
        "#1e88e5", // Blue 蓝
        "#43a047", // Green 绿
        "#fb8c00", // Orange 橙
        "#8e24aa", // Purple 紫
        "#00acc1", // Cyan 青
        "#fdd835", // Yellow 黄
        "#6d4c41", // Brown 棕
        "#546e7a", // Blue Grey 灰蓝
        "#d81b60", // Pink 粉
    };
    // 颜色数组总长度
    constexpr std::size_t num_colors = sizeof(path_colors) / sizeof(path_colors[0]);

    // ========================================================================
    // 数据结构定义：构建通道图、共享资源访问图
    // ========================================================================

    // 通道端点：标记一个读写端点属于哪个系统、是定时系统还是计算系统
    struct ChannelEndpoint {
        std::size_t system_idx; // 系统在数组中的下标
        bool is_fixed_rate;     // true=FixedRate定时系统；false=Compute计算系统
    };

    // 单条通道完整连接关系：同一个(type+topic)下所有写者、读者
    struct ChannelConnection {
        std::vector<ChannelEndpoint> writers; // 该通道所有发布/写者
        std::vector<ChannelEndpoint> readers; // 该通道所有订阅/读者
        std::type_index type;                 // 通道承载数据类型
        std::type_index topic;                // 通道话题标识类型
        channel_kind kind;                    // 通道类型：SPSC/SPMC Reader/Writer
    };

    // 哈希表：ChannelKey(数据类型+话题) → 完整通道连接信息，存储全部Pub/Sub通道
    std::unordered_map<ChannelKey, ChannelConnection, ChannelKeyHash> channel_graph;

    // 共享资源访问结构：全局资源被哪些系统读、哪些系统修改
    struct ResourceAccess {
        std::vector<ChannelEndpoint> readers;  // 只读该资源的系统
        std::vector<ChannelEndpoint> mutators; // 读写/修改该资源的系统
    };
    // 有序Map：资源type_index → 读写访问记录
    std::map<std::type_index, ResourceAccess> resource_graph;

    // 获取两类系统总数
    const std::size_t fixed_count   = fixed_rate_systems_.size();
    const std::size_t compute_count = compute_systems_.size();

    // 构建资源类型 -> 可读名称映射（消繁模板类型名）
    const auto fixed_resource_labels   = build_resource_label_map(fixed_rate_systems_);
    const auto compute_resource_labels = build_resource_label_map(compute_systems_);
    // 合并定时系统、计算系统的资源名称映射
    std::map<std::type_index, std::string> resource_labels = fixed_resource_labels;
    resource_labels.insert(compute_resource_labels.begin(), compute_resource_labels.end());

    // 内部工具lambda：遍历单个系统所有SPSC/SPMC通道、共享资源读写，填入两张图
    // idx：系统下标；is_fixed：是否定时系统；meta：系统元数据（通道、资源声明）
    auto process_system_channels = [&](std::size_t idx, bool is_fixed, const SystemMeta& meta) {
        ChannelEndpoint endpoint{idx, is_fixed};

        // 1. 处理系统内所有SPSC单生产者单消费者通道
        for (const auto& ch : meta.spsc_channels) {
            ChannelKey key{ch.type, ch.topic};
            // 查找该话题通道，不存在则新建ChannelConnection
            auto it = channel_graph.find(key);
            if (it == channel_graph.end()) {
                it = channel_graph
                         .emplace(
                             key,
                             ChannelConnection{
                                 .writers = {},
                                 .readers = {},
                                 .type    = ch.type,
                                 .topic   = ch.topic,
                                 .kind    = ch.kind})
                         .first;
            }

            // 根据通道类型区分写入写者列表/读者列表
            if (ch.kind == channel_kind::spsc_writer) {
                it->second.writers.push_back(endpoint);
            } else if (ch.kind == channel_kind::spsc_reader) {
                it->second.readers.push_back(endpoint);
            }
        }

        // 2. 处理系统内所有SPMC单生产者多消费者通道
        for (const auto& ch : meta.spmc_channels) {
            ChannelKey key{ch.type, ch.topic};
            auto it = channel_graph.find(key);
            if (it == channel_graph.end()) {
                it = channel_graph
                         .emplace(
                             key,
                             ChannelConnection{
                                 .writers = {},
                                 .readers = {},
                                 .type    = ch.type,
                                 .topic   = ch.topic,
                                 .kind    = ch.kind})
                         .first;
            }

            if (ch.kind == channel_kind::spmc_writer) {
                it->second.writers.push_back(endpoint);
            } else if (ch.kind == channel_kind::spmc_reader) {
                it->second.readers.push_back(endpoint);
            }
        }

        // 3. 处理全局共享资源只读声明
        for (const auto& type : meta.reads) {
            resource_graph[type].readers.push_back(endpoint);
        }
        // 4. 处理全局共享资源修改声明
        for (const auto& type : meta.writes) {
            resource_graph[type].mutators.push_back(endpoint);
        }
    };

    // 遍历全部定时系统，填充通道图、资源访问图
    for (std::size_t i = 0; i < fixed_count; ++i) {
        const auto& meta = fixed_rate_systems_[i].system->meta();
        process_system_channels(i, true, meta);
    }
    // 遍历全部计算系统，填充通道图、资源访问图
    for (std::size_t i = 0; i < compute_count; ++i) {
        const auto& meta = compute_systems_[i].system->meta();
        process_system_channels(i, false, meta);
    }

    // 根据系统发布/订阅行为分类Compute系统
    auto classify_compute_by_data_flow = [&](const std::size_t idx) -> const char* {
        // 外部信号驱动系统，单独分类
        if (is_external_compute_source(idx)) {
            return "external";
        }

        const auto& meta = compute_systems_[idx].system->meta();
        // has_pub：是否存在输出通道 或 修改共享资源（产生数据输出）
        const bool has_pub = std::any_of(
                                 meta.spsc_channels.begin(), meta.spsc_channels.end(),
                                 [](const auto& c) { return c.kind == channel_kind::spsc_writer; })
                          || std::any_of(
                                 meta.spmc_channels.begin(), meta.spmc_channels.end(),
                                 [](const auto& c) { return c.kind == channel_kind::spmc_writer; })
                          || !meta.writes.empty();
        // has_sub：是否存在输入通道 或 读取共享资源（依赖外部数据）
        const bool has_sub = std::any_of(
                                 meta.spsc_channels.begin(), meta.spsc_channels.end(),
                                 [](const auto& c) { return c.kind == channel_kind::spsc_reader; })
                          || std::any_of(
                                 meta.spmc_channels.begin(), meta.spmc_channels.end(),
                                 [](const auto& c) { return c.kind == channel_kind::spmc_reader; })
                          || !meta.reads.empty() || !meta.writes.empty();

        // 有输入有输出：流水线中间节点
        if (has_pub && has_sub)
            return "pipeline";
        // 只有输入无输出：终点Sink
        if (!has_pub && has_sub)
            return "sink";
        // 只有输出无输入：内部数据源起点
        if (has_pub && !has_sub)
            return "internal";
        // 无输入无输出：孤立无用系统
        return "orphan";
    };

    // 系统分类容器
    std::vector<std::size_t> notifying_fixed_systems, silent_fixed_systems,
        external_compute_systems, pipeline_systems, sink_systems, internal_systems, orphan_systems;

    // 划分定时系统：可唤醒下游 / 静默仅发数据不唤醒
    for (std::size_t i = 0; i < fixed_count; ++i) {
        if (fixed_rate_systems_[i].policy.notifies) {
            notifying_fixed_systems.push_back(i);
        } else {
            silent_fixed_systems.push_back(i);
        }
    }

    // 划分计算系统类别
    for (std::size_t i = 0; i < compute_count; ++i) {
        const auto* type = classify_compute_by_data_flow(i);
        if (strcmp(type, "external") == 0)
            external_compute_systems.push_back(i);
        else if (strcmp(type, "pipeline") == 0)
            pipeline_systems.push_back(i);
        else if (strcmp(type, "sink") == 0)
            sink_systems.push_back(i);
        else if (strcmp(type, "internal") == 0)
            internal_systems.push_back(i);
        else
            orphan_systems.push_back(i);
    }

    // 工具函数：打印单个系统Mermaid节点
    auto print_system_node = [&](std::size_t idx, bool is_fixed, const char* style) {
        const auto& meta = is_fixed ? fixed_rate_systems_[idx].system->meta()
                                    : compute_systems_[idx].system->meta();
        // E前缀=FixedRate定时系统；C前缀=Compute计算系统
        const char* prefix = is_fixed ? "E" : "C";
        if (is_fixed) {
            // 定时系统额外打印运行频率Hz
            const auto freq = fixed_rate_systems_[idx].policy.frequency_hz;
            printf(
                "        %s%d[\"%s<br/>%dHz\"]:::%s\n", prefix, (int)idx, meta.name.c_str(), freq,
                style);
        } else {
            // 计算系统仅打印名称
            printf("        %s%d[\"%s\"]:::%s\n", prefix, (int)idx, meta.name.c_str(), style);
        }
    };

    // 打印【可唤醒定时系统】子图
    if (!notifying_fixed_systems.empty()) {
        printf("    subgraph NotifyingFixed[Notifying FixedRate Sources]\n");
        for (auto idx : notifying_fixed_systems) {
            print_system_node(idx, true, "fixedStyle");
        }
        printf("    end\n\n");
    }

    // 打印【静默定时系统】子图（只发数据，不触发下游执行）
    if (!silent_fixed_systems.empty()) {
        printf("    subgraph SilentFixed[Silent FixedRate Side Inputs]\n");
        for (auto idx : silent_fixed_systems) {
            print_system_node(idx, true, "fixedStyle");
        }
        printf("    end\n\n");
    }

    // 打印【外部触发计算源】子图
    if (!external_compute_systems.empty()) {
        printf("    subgraph ComputeSources[External Compute Sources]\n");
        for (auto idx : external_compute_systems) {
            print_system_node(idx, false, "inputStyle");
        }
        printf("    end\n\n");
    }

    // 打印【流水线中间系统】子图
    if (!pipeline_systems.empty()) {
        printf("    subgraph Pipeline[Pipeline Systems]\n");
        for (auto idx : pipeline_systems) {
            print_system_node(idx, false, "pipelineStyle");
        }
        printf("    end\n\n");
    }

    // 打印【终点Sink系统】子图（无输出）
    if (!sink_systems.empty()) {
        printf("    subgraph Sink[Sink Systems]\n");
        for (auto idx : sink_systems) {
            print_system_node(idx, false, "outputStyle");
        }
        printf("    end\n\n");
    }

    // 打印【内部数据源系统】子图（无输入，仅输出）
    if (!internal_systems.empty()) {
        printf("    subgraph Internal[Internal Compute Roots]\n");
        for (auto idx : internal_systems) {
            print_system_node(idx, false, "orphanStyle");
        }
        printf("    end\n\n");
    }

    // 打印【孤立无交互系统】子图
    if (!orphan_systems.empty()) {
        printf("    subgraph Orphan[Orphan Systems]\n");
        for (auto idx : orphan_systems) {
            print_system_node(idx, false, "orphanStyle");
        }
        printf("    end\n\n");
    }

    // 分配共享资源绘图ID，给每个唯一资源分配R开头节点编号
    std::map<std::type_index, std::size_t> resource_indices;
    std::size_t next_resource_idx = 0;
    for (const auto& [type, access] : resource_graph) {
        // 跳过无任何系统读写的无效资源
        if (access.readers.empty() && access.mutators.empty()) {
            continue;
        }
        resource_indices.emplace(type, next_resource_idx++);
    }
    // 输出共享资源子图
    if (!resource_indices.empty()) {
        printf("    subgraph Resources[Shared Resources]\n");
        for (const auto& [type, idx] : resource_indices) {
            const auto label_it = resource_labels.find(type);
            if (label_it == resource_labels.end()) {
                continue;
            }
            printf(
                "        R%d[\"%s<br/>resource\"]:::resourceStyle\n", (int)idx,
                label_it->second.c_str());
        }
        printf("    end\n\n");
        // 注释说明虚线含义
        printf("    %% Dashed grey edges below indicate shared resource access only\n\n");
    }

    // ========================================================================
    // 绘制通道Pub/Sub彩色实线数据流
    // ========================================================================

    // 记录每个通道分配的颜色下标，同一通道所有连线保持同色
    std::map<ChannelKey, std::size_t> channel_color_map;
    std::size_t next_color_idx = 0;

    // 存储最终边文本 + 对应linkStyle样式
    std::vector<std::pair<std::string, std::string>> styled_edges;
    // 存储非法通道注释（多写者、无写者等警告）
    std::vector<std::string> channel_comments;

    // 遍历全部通道连接关系
    for (const auto& [key, conn] : channel_graph) {
        const auto label_it = channel_labels.find(key);
        if (label_it == channel_labels.end()) {
            continue;
        }
        const auto& channel_label = label_it->second;

        // 非法通道1：存在多个写者（SPSC/SPMC规范单写），跳过并打印注释警告
        if (conn.writers.size() > 1) {
            channel_comments.push_back(
                "    %% Skipped invalid channel \"" + channel_label + "\": multiple writers");
            continue;
        }
        // 非法通道2：无写者只有读者，无数据源，跳过
        if (conn.writers.empty()) {
            if (!conn.readers.empty()) {
                channel_comments.push_back(
                    "    %% Skipped invalid channel \"" + channel_label + "\": no writer");
            }
            continue;
        }
        // 无读者，无连线需求，直接跳过
        if (conn.readers.empty()) {
            continue;
        }

        // 获取唯一写者端点
        const auto& writer        = conn.writers.front();
        const char* writer_prefix = writer.is_fixed_rate ? "E" : "C";

        // 为该通道分配固定颜色下标，循环复用10种颜色
        if (!channel_color_map.contains(key)) {
            channel_color_map[key] = next_color_idx % num_colors;
            next_color_idx++;
        }
        const std::size_t color_idx = channel_color_map[key];

        // 写者连接每一个读者，生成边字符串存入styled_edges
        for (const auto& reader : conn.readers) {
            const char* reader_prefix = reader.is_fixed_rate ? "E" : "C";
            char edge_buf[256];
            // 生成：E0 -->|"话题名称"| C1 格式文本
            snprintf(
                edge_buf, sizeof(edge_buf), "    %s%d -->|\"%s\"| %s%d", writer_prefix,
                (int)writer.system_idx, channel_label.c_str(), reader_prefix,
                (int)reader.system_idx);
            // 绑定线条样式：对应颜色、粗实线
            styled_edges.emplace_back(
                edge_buf, "stroke:" + std::string(path_colors[color_idx]) + ",stroke-width:2px");
        }
    }

    // 处理共享资源灰色虚线（去重，避免重复绘制同一条资源访问边）
    std::vector<std::string> seen_resource_edge_defs;
    // 判断资源边是否已经生成过
    auto has_seen_resource_edge = [&](const std::string& edge_def) {
        return std::find(seen_resource_edge_defs.begin(), seen_resource_edge_defs.end(), edge_def)
            != seen_resource_edge_defs.end();
    };
    // 保存资源边，加入绘制列表
    auto remember_resource_edge = [&](std::string edge_def) {
        if (has_seen_resource_edge(edge_def)) {
            return;
        }
        seen_resource_edge_defs.push_back(edge_def);
        // 灰色虚线样式
        styled_edges.emplace_back(
            std::move(edge_def), "stroke:#7f8c8d,stroke-width:2px,stroke-dasharray: 5 5");
    };

    // 遍历所有全局共享资源，生成读写虚线
    for (const auto& [type, access] : resource_graph) {
        const auto resource_it = resource_indices.find(type);
        const auto label_it    = resource_labels.find(type);
        if (resource_it == resource_indices.end() || label_it == resource_labels.end()) {
            continue;
        }

        const auto resource_idx = resource_it->second;
        // 资源读：资源R → 系统（单向虚线）
        for (const auto& reader : access.readers) {
            const char* reader_prefix = reader.is_fixed_rate ? "E" : "C";
            char edge_buf[256];
            snprintf(
                edge_buf, sizeof(edge_buf), "    R%d -->|\"res\"| %s%d", (int)resource_idx,
                reader_prefix, (int)reader.system_idx);
            remember_resource_edge(edge_buf);
        }
        // 资源修改：双向虚线，系统读+写资源
        for (const auto& mutator : access.mutators) {
            const char* mutator_prefix = mutator.is_fixed_rate ? "E" : "C";
            // 资源 → 系统读
            char read_edge_buf[256];
            snprintf(
                read_edge_buf, sizeof(read_edge_buf), "    R%d -->|\"res_mut\"| %s%d",
                (int)resource_idx, mutator_prefix, (int)mutator.system_idx);
            remember_resource_edge(read_edge_buf);
            // 系统 → 资源写
            char write_edge_buf[256];
            snprintf(
                write_edge_buf, sizeof(write_edge_buf), "    %s%d -->|\"res_mut\"| R%d",
                mutator_prefix, (int)mutator.system_idx, (int)resource_idx);
            remember_resource_edge(write_edge_buf);
        }
    }

    // 输出非法通道警告注释
    for (const auto& comment : channel_comments) {
        printf("%s\n", comment.c_str());
    }
    // 空行分隔注释与连线
    if ((!channel_comments.empty() || !resource_indices.empty()) && !styled_edges.empty()) {
        printf("\n");
    }

    // 输出所有连线文本
    for (const auto& [edge_def, _] : styled_edges) {
        printf("%s\n", edge_def.c_str());
    }

    // 批量设置每条连线的样式（linkStyle N 样式）
    for (std::size_t i = 0; i < styled_edges.size(); ++i) {
        const auto& style = styled_edges[i].second;
        printf("    linkStyle %zu %s\n", i, style.c_str());
    }

    // Mermaid结束标记
    printf("```\n");
}

// 输出Mermaid拓扑分层执行图：展示Kahn排序后的并行执行层级、系统输入输出通道、层级依赖
void Scheduler::print_mermaid_execution_levels() const {
    // 拓扑未构建，直接输出提示图
    if (lifecycle_state() == LifecycleState::Configuring) {
        print_mermaid_rebuild_hint(
            "Scheduler graph not built. Call build() to render execution levels.");
        return;
    }

    printf("```mermaid\n");
    // LR横向布局，层级从左到右依次执行
    printf("flowchart LR\n\n");

    // 判断Compute系统是否外部触发源
    const auto is_external_compute_source = [&](const std::size_t idx) {
        return compute_systems_[idx].system->as_external_compute() != nullptr;
    };

    // 标记是否存在两类定时系统
    bool has_notifying_fixed_rate = false;
    bool has_silent_fixed_rate    = false;
    for (const auto& entry : fixed_rate_systems_) {
        has_notifying_fixed_rate |= entry.policy.notifies;
        has_silent_fixed_rate |= !entry.policy.notifies;
    }

    // 打印可唤醒定时系统子图
    if (has_notifying_fixed_rate) {
        printf("    subgraph Wakers[Notifying FixedRate Sources]\n");
        printf("        direction TB\n"); // 子图内部垂直排列
        for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
            if (!fixed_rate_systems_[i].policy.notifies) {
                continue;
            }
            const auto& name = fixed_rate_systems_[i].system->meta().name;
            const auto freq  = fixed_rate_systems_[i].policy.frequency_hz;
            printf(
                "        E%d[\"E%d: %s<br/>%dHz\"]:::fixedStyle\n", (int)i, (int)i, name.c_str(),
                freq);
        }
        printf("    end\n\n");
    }

    // 打印静默定时系统子图
    if (has_silent_fixed_rate) {
        printf("    subgraph SideInputs[Silent FixedRate Side Inputs]\n");
        printf("        direction TB\n");
        for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
            if (fixed_rate_systems_[i].policy.notifies) {
                continue;
            }
            const auto& name = fixed_rate_systems_[i].system->meta().name;
            const auto freq  = fixed_rate_systems_[i].policy.frequency_hz;
            printf(
                "        E%d[\"E%d: %s<br/>%dHz\"]:::fixedStyle\n", (int)i, (int)i, name.c_str(),
                freq);
        }
        printf("    end\n\n");
    }

    // 无执行层级直接提示
    if (levels_.empty()) {
        printf("    NoLevels[\"No execution levels computed\"]\n");
        printf("```\n");
        return;
    }

    // 统计每个Compute系统入边、出边数量，用于分类
    std::vector<std::size_t> in_edges(compute_systems_.size(), 0);
    std::vector<std::size_t> out_edges(compute_systems_.size(), 0);

    // 计算系统间互相唤醒出边总数
    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        out_edges[i] = (i < compute_affects_.size()) ? std::popcount(compute_affects_[i]) : 0;
    }

    // 统计系统间互相唤醒入边
    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if (i != j && (compute_affects_[i] & (1UL << j))) {
                in_edges[j]++;
            }
        }
    }

    // 叠加定时系统指向计算系统的入边
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        if (!fixed_rate_systems_[i].policy.notifies) {
            continue;
        }
        auto affects = i < fixed_rate_affects_.size() ? fixed_rate_affects_[i] : 0U;
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if (affects & (1UL << j)) {
                in_edges[j]++;
            }
        }
    }

    // 根据入/出边数量划分系统类型
    auto classify_by_edges = [&](std::size_t idx) -> const char* {
        if (is_external_compute_source(idx)) {
            return "external";
        }
        const bool has_out = out_edges[idx] > 0;
        const bool has_in  = in_edges[idx] > 0;

        if (has_out && has_in)
            return "pipeline"; // 中间流水线节点
        if (!has_out)
            return "sink";     // 终点无输出
        return "internal";     // 起点无输入
    };

    // 分层打印拓扑层级子图（Level 0 ~ Level N）
    for (std::size_t level_idx = 0; level_idx < levels_.size(); ++level_idx) {
        const auto& level = levels_[level_idx];
        printf(
            "    subgraph L%d[\"Level %d: %zu systems\"]\n", (int)level_idx, (int)level_idx,
            level.size());
        printf("        direction TB\n");

        // 打印层级内所有并行系统，附带输入/输出通道计数
        for (const auto sys_idx : level) {
            const auto& name = compute_systems_[sys_idx].system->meta().name;
            const auto& meta = compute_systems_[sys_idx].system->meta();
            // 统计该系统读写通道总数
            const auto io_counts = count_channel_io(meta);

            const auto* sys_type = classify_by_edges(sys_idx);
            // 分配绘图样式
            const char* style_class = (strcmp(sys_type, "external") == 0) ? "inputStyle"
                                    : (strcmp(sys_type, "sink") == 0)     ? "outputStyle"
                                                                          : "pipelineStyle";
            // 外部系统额外标记文字
            const char* extra = (strcmp(sys_type, "external") == 0) ? " [external]" : "";

            // 节点文本：系统名 + (输入通道数,输出通道数)
            printf(
                "        C%d[\"C%d: %s%s\\n(%d in, %d out)\"]:::%s\n", (int)sys_idx, (int)sys_idx,
                name.c_str(), extra, (int)io_counts.inputs, (int)io_counts.outputs, style_class);
        }

        printf("    end\n\n");
    }

    // ========================================================================
    // 绘制唤醒依赖连线
    // ========================================================================

    // 1. 定时系统 → 计算系统 唤醒边
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        if (!fixed_rate_systems_[i].policy.notifies) {
            continue;
        }
        auto affects = i < fixed_rate_affects_.size() ? fixed_rate_affects_[i] : 0U;
        for (std::size_t j = 0; j < compute_systems_.size(); ++j) {
            if (affects & (1UL << j)) {
                printf("    E%d --> C%d\n", (int)i, (int)j);
            }
        }
    }

    // 2. 计算系统互相唤醒边（包含跨层级直接依赖，不局限相邻层级）
    for (std::size_t src_idx = 0; src_idx < compute_systems_.size(); ++src_idx) {
        auto src_affects = src_idx < compute_affects_.size() ? compute_affects_[src_idx] : 0U;
        // 循环取出掩码每一个置1的bit
        while (src_affects != 0) {
            // countr_zero 获取最低置1位下标
            const auto dst_idx = static_cast<std::size_t>(std::countr_zero(src_affects));
            // 清除最低bit
            src_affects &= src_affects - 1;
            if (src_idx != dst_idx) {
                printf("    C%d --> C%d\n", (int)src_idx, (int)dst_idx);
            }
        }
    }

    printf("```\n");
    // 追加流水线性能统计注释
    printf("\n%% Pipeline Analysis:\n");

    // 统计各类计算系统数量
    std::size_t external_count = 0, pipeline_count = 0, sink_count = 0, internal_count = 0;
    for (std::size_t i = 0; i < compute_systems_.size(); ++i) {
        const auto* sys_type = classify_by_edges(i);
        if (strcmp(sys_type, "external") == 0)
            external_count++;
        else if (strcmp(sys_type, "sink") == 0)
            sink_count++;
        else if (strcmp(sys_type, "internal") == 0)
            internal_count++;
        else
            pipeline_count++;
    }

    // 输出统计注释
    printf(
        "%% - Notifying fixed_rate sources: %zu\n",
        std::count_if(
            fixed_rate_systems_.begin(), fixed_rate_systems_.end(),
            [](const auto& entry) { return entry.policy.notifies; }));
    printf(
        "%% - Silent fixed_rate side inputs: %zu\n",
        std::count_if(
            fixed_rate_systems_.begin(), fixed_rate_systems_.end(),
            [](const auto& entry) { return !entry.policy.notifies; }));
    printf("%% - Total depth: %zu levels\n", (std::size_t)levels_.size());
    printf(
        "%% - Compute system types: %zu external, %zu pipeline, %zu sink, %zu internal\n",
        external_count, pipeline_count, sink_count, internal_count);
    printf("%% - Max parallelism: ");
    // 查找并行度最大的层级（同一层可同时运行系统最多）
    std::size_t max_parallel = 0;
    std::size_t max_level    = 0;
    for (std::size_t i = 0; i < levels_.size(); ++i) {
        if (levels_[i].size() > max_parallel) {
            max_parallel = levels_[i].size();
            max_level    = i;
        }
    }
    printf("%zu systems at Level %zu\n", max_parallel, max_level);
}

// 控制台打印完整调度器性能统计（人类可读文本）
void Scheduler::print_stats() const {
    // 获取当前系统时钟时间戳
    const auto now = Clock::now();
    // 计算自调度器启动以来总运行秒数
    const auto elapsed_sec = std::chrono::duration<double>(now - stats_start_time_).count();
    // 原子读取计算循环总执行次数（宽松内存序，仅统计，无需强同步）
    const auto compute_cycles = compute_cycles_.load(std::memory_order_relaxed);
    // 所有计算循环总耗时（纳秒）
    const auto compute_total = compute_total_time_ns_.load(std::memory_order_relaxed);
    // 上一轮计算循环单次耗时（纳秒）
    const auto compute_last = compute_last_time_ns_.load(std::memory_order_relaxed);
    // 汇总所有定时系统触发下游唤醒总次数
    const auto notify_total = sum_notify_counts();

    printf("\n");
    printf("=== Scheduler Performance (%.1fs) ===\n", elapsed_sec);

    // 全局计算循环汇总指标
    printf(
        "Loop: %-4" PRIu64 " cycles (%.1f Hz)", compute_cycles,
        static_cast<double>(compute_cycles) / elapsed_sec);
    // 存在执行记录时打印平均耗时、最新单次耗时（转毫秒：ns / 1e6）
    if (compute_cycles > 0) {
        printf(
            " | avg: %.2f ms | last: %.2f ms",
            static_cast<double>(compute_total) / 1e6 / static_cast<double>(compute_cycles),
            static_cast<double>(compute_last) / 1e6);
    }
    // 全局唤醒总次数、平均唤醒频率
    printf(
        " | notifies: %-4" PRIu64 " (%.1f Hz)\n", notify_total,
        static_cast<double>(notify_total) / elapsed_sec);

    // 内部工具：打印延迟分位数统计（最小、p50中位数、p95、最大值）
    auto print_latency_stats = [](const auto& latency) {
        // 存在采样数据才输出
        if (latency.sample_count > 0) {
            printf(
                "   min:%.2f p50:%.2f p95:%.2f max:%.2f ms",
                static_cast<double>(latency.min_ns) / 1e6,
                static_cast<double>(latency.p50_ns) / 1e6,
                static_cast<double>(latency.p95_ns) / 1e6,
                static_cast<double>(latency.max_ns) / 1e6);
            printf("\n");
        }
    };

    // 统一遍历打印系统统计模板，消除重复循环代码
    // systems：系统数组；stats：对应性能统计数组；print_fn：单行打印自定义逻辑
    auto print_system_stats = [&](const auto& systems, const auto& stats, auto print_fn) {
        for (std::size_t i = 0; i < systems.size(); ++i) {
            const auto& entry     = systems[i];
            const auto& sys_stats = stats[i];
            print_fn(entry, sys_stats, elapsed_sec);
        }
    };

    // 打印FixedRate定时系统统计
    print_system_stats(
        fixed_rate_systems_, fixed_rate_stats_,
        [&](const auto& entry, const auto& sys_stats, double) {
            // 该系统触发下游唤醒次数
            const auto notify_cnt = sys_stats.notify_count.load(std::memory_order_relaxed);
            // 系统run函数实际执行总次数
            const auto exec_count = sys_stats.execution_count.load(std::memory_order_relaxed);
            // 有效记录次数：仅产生数据输出/触发唤醒时才计数
            const auto record_cnt = sys_stats.record_count.load(std::memory_order_relaxed);
            // 计算延迟直方图分位数数据
            const auto latency = sys_stats.latency_hist.compute();
            // 实际运行频率 = 有效记录次数 / 总运行时长
            const auto actual_hz = static_cast<double>(record_cnt) / elapsed_sec;
            // 判断系统是否存在输出通道/修改共享资源（会触发下游唤醒）
            const bool write_mode = counts_written_calls(entry.system->meta());

            // 打印系统名、实际频率/目标频率、计数类型、有效运行次数
            printf(
                "  %-30s@%.1fHz(%uHz) %s:%" PRIu64, entry.system->meta().name.c_str(), actual_hz,
                entry.policy.frequency_hz, write_mode ? "writes" : "runs", record_cnt);
            // 存在执行次数则追加打印
            if (write_mode) {
                printf(" exec:%" PRIu64, exec_count);
            }
            // 有唤醒下游记录则追加打印唤醒次数
            if (notify_cnt > 0) {
                printf(" notif:%" PRIu64, notify_cnt);
            }
            printf("\n");

            // 打印延迟分位数
            print_latency_stats(latency);
        });

    // 打印Compute计算系统统计
    print_system_stats(
        compute_systems_, compute_stats_, [&](const auto& entry, const auto& sys_stats, double) {
            // 系统run执行总次数
            const auto run_count = sys_stats.run_count.load(std::memory_order_relaxed);
            // 延迟统计
            const auto latency = sys_stats.latency_hist.compute();
            // 实际运行频率
            const auto actual_hz = static_cast<double>(run_count) / elapsed_sec;

            printf(
                "  %-30s@%.1fHz runs:%" PRIu64, entry.system->meta().name.c_str(), actual_hz,
                run_count);
            printf("\n");

            print_latency_stats(latency);
        });
    printf("======================================\n");
    printf("\n");
}

// 将全部性能指标序列化为JSON字符串，用于日志持久化、外部可视化工具读取
auto Scheduler::get_stats_json() const -> std::string {
    using json = nlohmann::json;

    // 计算总运行时长（秒）
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(Clock::now() - stats_start_time_);
    const double elapsed_sec = elapsed.count();

    // 读取全局计算循环原子统计
    const auto cycles       = compute_cycles_.load(std::memory_order_relaxed);
    const auto total_time   = compute_total_time_ns_.load(std::memory_order_relaxed);
    const auto last_time    = compute_last_time_ns_.load(std::memory_order_relaxed);
    const auto notify_total = sum_notify_counts();

    json root;
    root["runtime_seconds"] = elapsed_sec;

    // 填充计算循环全局指标
    json& compute_json           = root["compute_loop"];
    compute_json["cycles"]       = cycles;
    compute_json["frequency_hz"] = static_cast<double>(cycles) / elapsed_sec;
    compute_json["avg_time_ms"] =
        cycles > 0 ? (static_cast<double>(total_time) / 1e6 / static_cast<double>(cycles)) : 0.0;
    compute_json["last_time_ms"]   = static_cast<double>(last_time) / 1e6;
    compute_json["total_notifies"] = notify_total;

    // 内部工具：将完整延迟分位数批量写入JSON对象
    auto add_latency_stats = [](json& sys, const auto& latency, double) {
        sys["sampled_runs"] = latency.sample_count;
        sys["min_ms"]       = static_cast<double>(latency.min_ns) / 1e6;
        sys["p50_ms"]       = static_cast<double>(latency.p50_ns) / 1e6;
        sys["p95_ms"]       = static_cast<double>(latency.p95_ns) / 1e6;
        sys["p99_ms"]       = static_cast<double>(latency.p99_ns) / 1e6;
        sys["p999_ms"]      = static_cast<double>(latency.p999_ns) / 1e6;
        sys["mean_ms"]      = static_cast<double>(latency.mean_ns) / 1e6;
        sys["max_ms"]       = static_cast<double>(latency.max_ns) / 1e6;
        sys["stddev_ms"]    = static_cast<double>(latency.stddev_ns) / 1e6;
    };

    // 统一生成系统JSON数据模板，消除重复循环
    auto generate_system_stats = [&](const auto& systems, const auto& stats, json& arr,
                                     auto add_extra_fn) {
        for (std::size_t i = 0; i < systems.size(); ++i) {
            const auto& entry     = systems[i];
            const auto& sys_stats = stats[i];
            const auto latency    = sys_stats.latency_hist.compute();

            json sys;
            // 写入通用延迟指标
            add_latency_stats(sys, latency, elapsed_sec);
            // 写入系统类型专属扩展字段
            add_extra_fn(sys, entry, sys_stats);
            // 以系统名称为key存入JSON对象
            arr[entry.system->meta().name] = sys;
        }
    };

    // 填充定时系统JSON数组
    json& fixed_rate_arr = root["fixed_rate_systems"] = json::object();
    generate_system_stats(
        fixed_rate_systems_, fixed_rate_stats_, fixed_rate_arr,
        [elapsed_sec](json& sys, const auto& entry, const auto& stats) {
            const auto notify_cnt = stats.notify_count.load(std::memory_order_relaxed);
            const auto exec_count = stats.execution_count.load(std::memory_order_relaxed);
            const auto record_cnt = stats.record_count.load(std::memory_order_relaxed);
            const bool write_mode = counts_written_calls(entry.system->meta());
            sys["target_hz"]      = entry.policy.frequency_hz;
            sys["notifies"]       = notify_cnt;
            sys["executions"]     = exec_count;
            sys["runs"]           = record_cnt;
            sys["actual_hz"]      = static_cast<double>(record_cnt) / elapsed_sec;
            sys["count_mode"]     = write_mode ? "written_calls" : "run_calls";
        });

    // 填充计算系统JSON数组
    json& compute_arr = root["compute_systems"] = json::object();
    generate_system_stats(
        compute_systems_, compute_stats_, compute_arr,
        [elapsed_sec](json& sys, const auto&, const auto& stats) {
            const auto run_count = stats.run_count.load(std::memory_order_relaxed);
            sys["runs"]          = run_count;
            sys["actual_hz"]     = static_cast<double>(run_count) / elapsed_sec;
        });

    // 序列化JSON为字符串返回
    return root.dump();
}

void Scheduler::run_fixed_rate_thread(FixedRateContext ctx) {
    // 定义循环持续运行判断闭包
    // 读取外部传入的调度器生命周期原子状态，判断调度器是否仍处于可运行状态
    // memory_order_acquire：保证读到其他线程release写入的最新生命周期状态
    const auto keep_running = [&ctx] {
        return is_running_state(ctx.lifecycle->load(std::memory_order_acquire));
    };

    // ===================== 1、CPU核心绑定，线程隔离优化 =====================
    // 配置中cpu_affinity >= 0 代表需要将本线程绑定至指定逻辑CPU核心这里只是检查是否需要绑定，不执行绑核操作
    if (ctx.policy.cpu_affinity >= 0) {
        // 调用底层工具类将当前线程锁死到目标核心
        auto result = primitive::ThreadAffinity::pin_to_core(
            // 策略核心号是有符号int，转无符号uint32传给绑核接口
            static_cast<std::uint32_t>(ctx.policy.cpu_affinity));
        // 判断绑核操作是否失败
        if (!result) {
            // 输出警告日志：打印当前系统名称 + 底层返回的错误描述
            // 绑核失败属于非致命错误，仅告警不终止线程运行
            SPDLOG_WARN("{}: {}", ctx.system->meta().name, result.error());
        }
    }

    // ===================== 2、设置实时调度优先级（Linux SCHED_FIFO） =====================
    // thread_priority > 0 代表启用实时调度，0/负数使用系统默认分时调度
    /*优先级 50 vs 80 80 先跑 ，80 不让出 CPU，50 永远等着 
    优先级相同（都是 50） FIFO 排队 ：先就绪的先跑，跑完才轮到下一个 
    实时线程 vs 普通线程 实时线程 抢占 所有普通线程（普通线程优先级=0） 
    优先级 0（项目现状） 不走 SCHED_FIFO，用系统默认分时调度（SCHED_OTHER），大家公平抢 CPU*/
    if (ctx.policy.thread_priority > 0) {
        // 初始化实时调度配置结构体
        primitive::ThreadAffinity::RealtimeConfig rt_config{};
        // 将配置的实时优先级赋值，转为uint8_t适配底层接口参数类型
        rt_config.priority = static_cast<std::uint8_t>(ctx.policy.thread_priority);
        // 链式判断：调用设置实时优先级接口，失败则打印警告
        if (auto result = primitive::ThreadAffinity::set_realtime_priority(rt_config); !result) {
            SPDLOG_WARN("{}: {}", ctx.system->meta().name, result.error());
        }
    }

    // 读取配置的运行频率，单位Hz（每秒执行次数）
    const auto frequency = ctx.policy.frequency_hz;
    // 绑定当前系统专属的性能统计对象引用，用于记录循环耗时、延迟直方图
    auto& stats = ctx.scheduler->fixed_rate_stats_[ctx.system_index];
    // 判断当前系统是否属于输出型系统：执行完成后会产生数据传递给下游系统
    const bool write_mode = counts_written_calls(ctx.system->meta());
    // 执行完成统计闭包：统一处理计数、延迟统计、下游唤醒逻辑
    const auto record_execution = [&](const bool written, const std::uint64_t elapsed) {
        // 宽松内存序：仅本地统计自增，无跨线程同步需求，性能最优
        // 全局总执行次数 +1
        stats.execution_count.fetch_add(1, std::memory_order_relaxed);

        // 两种场景需要记录有效计算延迟：
        // 1. 系统本身不是输出型系统(write_mode=false)，每次执行都算有效计算
        // 2. 输出型系统，本次执行成功产出数据(written=true)
        if (!write_mode || written) {
            // 有效执行计数自增 数据产出次数 +1   
            stats.record_count.fetch_add(1, std::memory_order_relaxed);
            // 将本次执行耗时存入延迟直方图，用于统计平均/最大/分位延迟
            stats.latency_hist.record(elapsed);
        }

        // 满足两个条件则唤醒下游依赖系统：
        // 1. 本次执行产出有效输出数据 written=true
        // 2. 配置开启自动通知下游 notifies=true
        // [[likely]] 编译器提示：该分支是高频正常路径，优化分支预测
        if (written && ctx.policy.notifies) [[likely]] {
            // 调度器通知接口：将当前系统对应的就绪掩码置1，下游计算循环会检测并执行
            // notify 的作用 ： ready_systems_.fetch_or(1ULL << system_index) ，置位就绪位掩码，compute 线程池检测到后执行下游系统计算
            ctx.scheduler->notify(ctx.system_index);
        }
    };

    // ===================== 模式1：frequency=0 无固定周期，全速无限循环 =====================
    if (frequency == 0) {
        // 调度器未停止则持续循环执行系统逻辑
        while (keep_running()) {
            // 延迟探针：高精度计时工具，记录单次系统执行耗时
            primitive::LatencyProbe probe;
            // 执行当前系统业务逻辑，入参是全局世界数据；返回值bool代表是否产生输出数据
            const bool written = ctx.system->run(*ctx.world);
            // 获取从探针创建到当前的总执行耗时，单位纳秒
            const auto elapsed = probe.elapsed_ns();

            // 调用统计闭包，统一更新计数、延迟、下游唤醒
            record_execution(written, elapsed);
        }
        // 调度器停止，退出该线程函数
        return;
    }

    // ===================== 模式2：frequency>0 固定频率定时循环 =====================
    // [[assume(frequency > 0)]] 编译器提示，消除编译器除零警告、辅助优化
    [[assume(frequency > 0)]];
    // 计算单次周期纳秒时长：1秒(1e9纳秒) / 目标频率Hz
    const auto period =
        std::chrono::nanoseconds(1'000'000'000ULL / static_cast<std::uint64_t>(frequency));
    // 记录下一次需要执行任务的时间点，初始为当前时刻
    auto next_tick = Clock::now();

    // 调度器运行时持续定时执行
    while (keep_running()) [[likely]] {
        // 把下一次执行时间点向后推移一个完整周期
        next_tick += period;

        // 开启计时探针
        primitive::LatencyProbe probe;
        // 执行业务系统逻辑，获取是否产出输出
        const bool written = ctx.system->run(*ctx.world);
        // 读取本次执行耗时ns
        const auto elapsed = probe.elapsed_ns();

        // 统一执行统计、唤醒逻辑
        record_execution(written, elapsed);

        // 高精度阻塞休眠，直到下一个周期时间点再唤醒线程
        // 相比sleep_for：解决任务执行耗时导致周期漂移累积问题
        std::this_thread::sleep_until(next_tick);
    }
}

// ============================================================================
// Compute 主线程主循环（调度器主线程阻塞在此函数）
// 负责轮询就绪系统、自适应空闲退避、支持热更暂停、定时打印性能统计
// ============================================================================
void Scheduler::run_compute_loop() {
    // 自适应空闲退避三级策略：自旋 → 让出CPU → 短休眠
    // 设计目标：高负载有任务时零延迟、低负载无任务时降低CPU空转占用
    // 阶段1：短时自旋，CPU空跑，不放弃时间片，唤醒后立刻执行业务，延迟最低
    // 阶段2：std::this_thread::yield() 主动放弃当前CPU时间片，交给其他就绪线程
    // 阶段3：短时间休眠，内核收回CPU，彻底降低整机CPU使用率
    constexpr std::size_t SPIN_LIMIT  = 100;  // 第一阶段自旋最大累计空闲次数
    constexpr std::size_t YIELD_LIMIT = 1000; // 第二阶段yield让出CPU最大累计空闲次数
    constexpr auto SLEEP_DURATION= std::chrono::microseconds(10); // 第三阶段休眠时长 10微秒
    std::size_t idle_count            = 0; // 连续无任务空闲计数，用于切换三级退避策略

    // 读取配置：是否定时打印性能统计日志
    const auto should_print = config_.print_stats;
    // 统计打印固定间隔：5秒输出一次
    auto print_interval = std::chrono::seconds(5);
    // 记录上一次打印统计的时间点，用于间隔判断
    auto last_print = Clock::now();

    // 主循环：调度器处于Running运行状态时持续循环调度系统任务
    // [[likely]] 编译器提示：绝大多数情况条件为true，编译器优化分支预测
    while (is_running()) [[likely]] {
        // 【罕见分支】外部下发暂停指令，需要阻塞当前计算线程
        // [[unlikely]] 编译器提示：暂停属于低频事件，优化分支预测，不占用快速路径
        if (pause_requested_.load(std::memory_order_acquire)) [[unlikely]] {
            // 进入暂停逻辑，重置空闲计数器，退出退避状态
            idle_count = 0;

            // 上锁修改paused_原子标记，对外通知本计算线程已进入暂停阻塞状态
            // 独立{}局部作用域控制lock_guard生命周期，执行完自动解锁
            {
                // RAII互斥锁，构造上锁，出作用域自动析构解锁，异常安全
                std::lock_guard lock(pause_mutex_);
                // release内存序：本次标记修改对其他读取paused_的线程全局可见
                paused_.store(true, std::memory_order_release);
            }
            // 唤醒所有等待暂停状态的外部线程
            pause_cv_.notify_all();

            // 阻塞等待恢复信号，unique_lock支持条件变量wait自动解锁/重新上锁
            {
                std::unique_lock lock(pause_mutex_);
                // wait阻塞条件：两个任意满足其一即可唤醒线程
                // 1. pause_requested_恢复为false（收到恢复指令）
                // 2. 调度器整体停止is_running()=false（停机信号）
                pause_cv_.wait(lock, [this] {
                    // acquire读取保证拿到最新暂停标记与调度器运行状态
                    return !pause_requested_.load(std::memory_order_acquire) || !is_running();
                });
                // 线程被唤醒，清除“正在暂停”标记，release同步修改
                paused_.store(false, std::memory_order_release);
            }

            // 唤醒后校验调度器状态：如果已经下发停机指令，直接跳出主循环终止计算
            if (!is_running()) {
                break;
            }
            // 仅收到恢复暂停信号，回到循环头部继续正常调度
            continue;
        }

        // 原子交换操作：取出当前待执行系统掩码，同时把全局就绪掩码置0清空
        // acq_rel复合内存序：读旧掩码带acquire、写0清空带release，同步跨线程就绪标记
        /*exchange(new_val)：原子操作
        取出原子变量当前旧值；
        把原子变量直接赋值为 new_val；
        返回刚才拿到的旧值。*/
        const auto ready = ready_systems_.exchange(0, std::memory_order_acq_rel);

        // ready != 0 代表存在需要执行的ECS系统任务，正常业务路径（高频分支）
        if (ready != 0) [[likely]] {
            // 存在任务，清空连续空闲计数，退出退避逻辑
            idle_count = 0;

            // 延迟计时探针，用于统计单次系统分层执行耗时（纳秒精度）
            primitive::LatencyProbe probe;
            // 根据就绪掩码分层执行对应ECS系统，返回执行结果（包含本轮新产生的延迟就绪掩码）
            const auto result = run_compute_selective(ready);
            // 获取从probe创建到当前的总执行耗时（单位ns）
            const auto elapsed = probe.elapsed_ns();

            // 宽松内存序更新全局统计：仅内部读取统计，无跨线程同步依赖，性能最优
            // 全局总执行循环次数 +1
            compute_cycles_.fetch_add(1, std::memory_order_relaxed);
            // 累加本次执行耗时到总耗时统计
            compute_total_time_ns_.fetch_add(elapsed, std::memory_order_relaxed);
            // 覆盖更新上一轮单次循环耗时，用于实时观测单次延迟
            compute_last_time_ns_.store(elapsed, std::memory_order_relaxed);
            // 本轮系统执行过程中产生了延后执行的系统掩码，或运算合并回全局就绪标记等待下一轮调度
            if (result.deferred_mask != 0) {
                // release写屏障，保证新就绪掩码对下一轮exchange读取可见
                ready_systems_.fetch_or(result.deferred_mask, std::memory_order_release);
            }

            // 配置开启打印统计时，才执行间隔判断（低频分支）
            if (should_print) [[unlikely]] {
                // 获取当前高精度时间戳
                const auto now = Clock::now();
                // 判断距离上一次打印是否达到5秒打印间隔
                if (now - last_print >= print_interval) [[unlikely]] {
                    // 输出全部性能统计指标：循环次数、平均耗时、单次最大延迟等
                    print_stats();
                    // 更新本次打印时间戳，重置计时窗口
                    last_print = now;
                }
            }
        } else {
            // ready == 0，无任何就绪系统任务，进入三级空闲退避逻辑
            ++idle_count; // 连续空闲计数自增，用于切换退避阶段

            if (idle_count <= SPIN_LIMIT) {
                // 阶段1：短时间自旋等待，CPU空循环不放弃时间片
                // SPIN_HINT() 平台内置自旋提示指令，减少CPU功耗、优化缓存
                SPIN_HINT();
            } else if (idle_count <= YIELD_LIMIT) {
                // 阶段2：自旋次数超过阈值，主动让出当前CPU时间片
                // 内核调度其他就绪线程运行，本线程重新进入就绪队列等待调度
                std::this_thread::yield();
            } else {
                // 阶段3：长时间无任务，执行短休眠，内核挂起线程，大幅降低CPU占用
                std::this_thread::sleep_for(SLEEP_DURATION);
            }
        }
    }
    // 主循环正常退出 = 调度器标记为停止，执行所有注册的停机清理钩子函数
    for (auto& hook : shutdown_hooks_) {
        hook();
    }
}

// 根据就绪掩码，分层批量并行执行Compute系统，返回本轮执行结果（新就绪系统、延迟执行系统）
auto Scheduler::run_compute_selective(const std::uint64_t ready_mask) -> ComputeRoundResult {
    // 无计算系统直接返回空结果
    if (compute_systems_.empty()) [[unlikely]] {
        return {};
    }

    std::uint64_t pending_ready = ready_mask; // 当前等待执行的系统掩码
    std::uint64_t executed_mask = 0;          // 本轮已执行完成系统掩码
    std::uint64_t written_mask  = 0;          // 本轮执行后产生输出的系统掩码
    std::uint64_t deferred_mask = 0;          // 本轮无法执行、延后到下一轮的系统掩码

    // 合并新产生的就绪位，已执行过的系统延后执行
    auto enqueue_ready = [&](const std::uint64_t new_bits) {
        deferred_mask |= new_bits & executed_mask;
        pending_ready |= new_bits & ~executed_mask;
    };

    // 取出全局就绪池中新产生的就绪系统，合并到待执行队列
    enqueue_ready(ready_systems_.exchange(0, std::memory_order_acq_rel));

    // 存在待执行系统则循环分层执行
    while (pending_ready != 0) {
        bool ran_any = false;

        // 进入TBB任务调度域，多核并行执行
        compute_arena_->execute([&] {
            std::size_t to_run_buf[64]; // 单层待执行系统缓存（最多64个）

            // 按拓扑层级顺序遍历，保证依赖有序
            for (const auto& level : levels_) {
                // 中途可能产生新就绪系统，实时合并
                enqueue_ready(ready_systems_.exchange(0, std::memory_order_acq_rel));

                std::size_t to_run_count = 0;
                std::uint64_t level_mask = 0;
                // 筛选当前层级内处于就绪状态的系统
                for (const auto idx : level) {
                    const std::uint64_t bit = 1ULL << idx;
                    if ((pending_ready & bit) != 0) {
                        to_run_buf[to_run_count++] = idx;
                        level_mask |= bit;
                    }
                }

                // 当前层级无就绪系统，跳过
                if (to_run_count == 0) [[unlikely]] {
                    continue;
                }

                ran_any = true;
                // 从待执行掩码清除本层系统
                pending_ready &= ~level_mask;
                // 标记本层系统已执行
                executed_mask |= level_mask;

                std::atomic<std::uint64_t> level_written{0}; // 本层产生输出的系统掩码

                // 层级仅单个系统：直接串行执行，无需创建任务组
                if (to_run_count == 1) {
                    const auto idx = to_run_buf[0];
                    primitive::LatencyProbe probe;
                    const bool written = compute_systems_[idx].system->run(world_);
                    const auto elapsed = probe.elapsed_ns();

                    // 更新性能统计
                    compute_stats_[idx].run_count.fetch_add(1, std::memory_order_relaxed);
                    compute_stats_[idx].latency_hist.record(elapsed);
                    if (written) [[likely]] {
                        level_written.store(1ULL << idx, std::memory_order_relaxed);
                    }
                } else {
                    // 多个系统：创建TBB任务组，多核并行执行
                    tbb::task_group tg;
                    for (std::size_t i = 0; i < to_run_count; ++i) {
                        const auto idx = to_run_buf[i];
                        tg.run([this, idx, &level_written] {
                            primitive::LatencyProbe probe;
                            const bool written = compute_systems_[idx].system->run(world_);
                            const auto elapsed = probe.elapsed_ns();

                            compute_stats_[idx].run_count.fetch_add(1, std::memory_order_relaxed);
                            compute_stats_[idx].latency_hist.record(elapsed);
                            if (written) [[likely]] {
                                level_written.fetch_or(1ULL << idx, std::memory_order_relaxed);
                            }
                        });
                    }
                    // 阻塞等待本层所有并行任务完成
                    tg.wait();
                }

                // 合并本层产生输出的系统掩码
                const auto level_written_mask = level_written.load(std::memory_order_relaxed);
                written_mask |= level_written_mask;

                // 遍历所有产生输出的系统，取出其下游依赖系统掩码
                std::uint64_t cascade = 0;
                auto writers          = level_written_mask;
                while (writers != 0) {
                    const auto writer_idx = static_cast<std::size_t>(std::countr_zero(writers));
                    writers &= writers - 1;
                    cascade |= compute_affects_[writer_idx];
                }

                // 将下游依赖系统加入待执行队列
                enqueue_ready(cascade);
            }
        });

        // 执行层级中途新产生就绪系统，合并入队列
        enqueue_ready(ready_systems_.exchange(0, std::memory_order_acq_rel));
        // 本轮无任何系统执行，跳出循环
        if (!ran_any) [[unlikely]] {
            break;
        }
    }

    // 返回本轮执行结果：产生输出的系统、延后执行的系统
    return ComputeRoundResult{
        .written_mask  = written_mask,
        .deferred_mask = deferred_mask,
    };
}

// ============================================================================
// 依赖拓扑构建核心函数 build_topology_snapshot
// 7大阶段：数量校验 → 通道采集 → 通道合法性校验 → 构建依赖邻接掩码 → 外部源掩码计算 → Kahn拓扑分层
// → 环检测+可达性校验 → 生成唤醒掩码
// ============================================================================
// 函数定义：成员const函数 → 承诺本函数不会修改Scheduler对象本身
// auto 推导返回值，-> 后置返回类型写法
// std::expected<T,E>【C++23语法】：要么返回T成功值，要么返回E错误
auto Scheduler::build_topology_snapshot() const -> std::expected<TopologySnapshot, BuildError> {
    // 取出计算系统数组长度；后面要用uint64_t位掩码，因此上限64
    const auto n = compute_systems_.size();
    // 构造空拓扑快照对象，最终要返回给调度器运行时使用
    TopologySnapshot snapshot;

    // 硬上限校验：最多64个compute系统，因为依赖关系用uint64_t（64bit）存储
    if (n > TooManyComputeSystemsError::max_count) {
        // std::unexpected【C++23】：构造一个“失败”的expected，携带错误结构体
        return std::unexpected(TooManyComputeSystemsError{.count = n});
    }

    // ========================================================================
    // 阶段1：遍历所有系统，采集全部通道读写端点
    // ========================================================================
    // 【捕获lambda】判断通道类型是否是写者：spsc写 / spmc写
    const auto is_writer = [](channel_kind k) {
        return k == channel_kind::spsc_writer || k == channel_kind::spmc_writer;
    };

    // lambda：判断通道是不是SPSC系列（单生产者单消费者）
    const auto is_spsc = [](channel_kind k) {
        return k == channel_kind::spsc_reader || k == channel_kind::spsc_writer;
    };

    // 结构体：ChannelEndpoint = 一个系统对某条通道的一次读写使用记录
    struct ChannelEndpoint {
        std::size_t index;        // 系统在fixed_rate_systems_ / compute_systems_数组里的下标
        bool is_fixed_rate;       // true=定时系统，false=计算系统
    };

    // 结构体：单条通道的全局使用记录表
    struct ChannelUsage {
        channel_kind kind = channel_kind::local;       // 通道类型spsc/spmc/local
        std::vector<ChannelEndpoint> writers;          // 所有写这条通道的系统端点
        std::vector<ChannelEndpoint> readers;           // 所有读这条通道的系统端点
        std::string first_system;                      // 第一个注册该通道的系统名，报错提示用
    };

    // std::map：key=通道唯一键(类型+topic)，value=这条通道的读写使用情况
    std::map<ChannelKey, ChannelUsage> channels;

    // 捕获lambda：把ChannelEndpoint转成人类可读字符串，报错日志打印
    // [&] 【lambda捕获】引用捕获外层所有局部变量
    auto endpoint_name = [&](const ChannelEndpoint endpoint) -> std::string {
        // 分支：区分是定时系统数组 还是 计算系统数组，取出系统名字
        const auto& name = endpoint.is_fixed_rate
                             ? fixed_rate_systems_[endpoint.index].system->meta().name
                             : compute_systems_[endpoint.index].system->meta().name;
        // 定时系统额外标注后缀，方便日志区分
        if (endpoint.is_fixed_rate) {
            return fmt::format("{} (fixed_rate)", name);
        }
        return name;
    };

    // 【高阶捕获lambda】process_system_channels：批量扫描一组系统（fixed组 / compute组）
    // 入参systems：系统数组；is_fixed_rate标记这组是不是定时系统
    // 返回BuildResult（本质expected<void, BuildError>，代表成功/失败）
    auto process_system_channels = [&](const auto& systems,
                                       const bool is_fixed_rate) -> BuildResult {
        // 遍历本组所有系统，i是数组下标
        for (std::size_t i = 0; i < systems.size(); ++i) {
            // 拿到系统元信息meta：里面预先收集好这个系统声明的spsc/spmc通道列表
            const auto& meta = systems[i].system->meta();

            // 【嵌套lambda】处理单一条通道，校验+注册读写端点
            // expect_spsc=true代表当前通道是SPSC，false=SPMC
            auto process_channel = [&](const ChannelMeta& ch,
                                       const bool expect_spsc) -> BuildResult {
                // 构造通道唯一主键：通道类型 + topic名，同一个topic才能匹配
                ChannelKey key{ch.type, ch.topic};
                // 找到/自动创建该通道的使用记录（map[]不存在则默认构造）
                auto& usage              = channels[key];
                // 判断这条通道之前有没有注册过读写端点
                const bool has_endpoints = !usage.writers.empty() || !usage.readers.empty();

                // 如果通道已经存在，校验：不能同topic混用SPSC/SPMC
                if (has_endpoints) {
                    // expect_spsc是当前系统预期的类型；usage.kind是之前登记的类型
                    const auto is_expected_kind =
                        expect_spsc ? is_spsc(usage.kind) : !is_spsc(usage.kind);
                    if (!is_expected_kind) {
                        // 类型冲突，返回错误
                        return std::unexpected(
                            ChannelKindConflict{
                                .key           = ChannelKeyInfo(ch.type, ch.topic, expect_spsc),
                                .first_system  = usage.first_system,
                                .second_system = meta.name,
                            });
                    }
                } else {
                    // 首次见到这个通道：记录通道类型、第一个注册的系统名称
                    usage.kind         = ch.kind;
                    usage.first_system = meta.name;
                }

                // 构造当前系统的端点记录
                const ChannelEndpoint endpoint{
                    .index         = i,
                    .is_fixed_rate = is_fixed_rate,
                };
                // 判断是写者还是读者，推入对应vector
                if (is_writer(ch.kind)) {
                    usage.writers.push_back(endpoint);
                } else {
                    usage.readers.push_back(endpoint);
                }
                // 成功，返回无错误的expected<void>
                return {};
            };

            // 循环处理当前系统所有SPSC通道
            for (const auto& ch : meta.spsc_channels) {
                // 调用嵌套lambda；如果返回失败，直接向上传递错误
                if (auto result = process_channel(ch, true); !result) {
                    return result;
                }
            }
            // 循环处理当前系统所有SPMC通道
            for (const auto& ch : meta.spmc_channels) {
                if (auto result = process_channel(ch, false); !result) {
                    return result;
                }
            }
        }
        // 本组系统全部扫描完毕，无错误
        return {};
    };

    // 先扫描定时系统组（fixed_rate）
    if (auto result = process_system_channels(fixed_rate_systems_, true); !result) {
        // 把内部错误向上抛出
        return std::unexpected(result.error());
    }
    // 再扫描计算系统组（compute）
    if (auto result = process_system_channels(compute_systems_, false); !result) {
        return std::unexpected(result.error());
    }

    // ========================================================================
    // 阶段2：通道强约束合法性校验（报错阻断构建）
    // ========================================================================
    // lambda：批量把通道读写端点列表转为系统名字vector，用于报错信息
    auto collect_endpoint_names = [&](const std::vector<ChannelEndpoint>& endpoints) {
        std::vector<std::string> names;
        // 预分配内存，减少扩容开销
        names.reserve(endpoints.size());
        for (const auto endpoint : endpoints) {
            names.push_back(endpoint_name(endpoint));
        }
        return names;
    };

    // 遍历所有登记过的通道，执行静态契约检查
    for (const auto& [key, usage] : channels) {
        // 【结构化绑定 C++17语法】auto& [key,usage] 直接解pair的first/second

        // 规则1：任何通道不允许多写者
        if (usage.writers.size() > 1) {
            return std::unexpected(
                MultipleWritersError{
                    .key     = ChannelKeyInfo(key.type, key.topic, is_spsc(usage.kind)),
                    .writers = collect_endpoint_names(usage.writers),
                });
        }

        // 规则2：SPSC通道不允许多读者（单生产者单消费者）
        if (is_spsc(usage.kind) && usage.readers.size() > 1) {
            return std::unexpected(
                MultipleReadersError{
                    .key     = ChannelKeyInfo(key.type, key.topic, true),
                    .readers = collect_endpoint_names(usage.readers),
                });
        }

        // 规则3：只有读者、没有写者 → 孤儿读者，没有数据源，直接报错
        if (usage.writers.empty() && !usage.readers.empty()) {
            return std::unexpected(
                OrphanedReaderError{
                    .key     = ChannelKeyInfo(key.type, key.topic, is_spsc(usage.kind)),
                    .readers = collect_endpoint_names(usage.readers),
                });
        }

        // 软警告（不阻断）：计算系统写通道，但没人读；定时系统写而无读者合法
        if (!usage.writers.empty() && usage.readers.empty() && !usage.writers[0].is_fixed_rate) {
            SPDLOG_WARN(
                "[WARN] Channel has writer '{}' but no readers\n",
                compute_systems_[usage.writers[0].index].system->meta().name.c_str());
        }
    }

    // ========================================================================
    // 阶段3：构建计算系统间依赖邻接掩码、入度数组（仅计算系统互相依赖）
    // adj_mask[i] = uint64_t位掩码：bitN=1 → 系统i跑完可以唤醒系统N
    // in_degree[i] = 系统i剩余未完成的前置依赖数量，Kahn拓扑排序使用
    // ========================================================================
    std::vector<std::uint64_t> adj_mask(n, 0);
    std::vector<std::size_t> in_degree(n, 0);

    // 遍历所有通道，建立【计算系统→计算系统】依赖边
    for (const auto& [key, usage] : channels) {
        // 没有写者 或 没有读者 → 不存在依赖，跳过
        if (usage.writers.empty() || usage.readers.empty()) {
            continue;
        }

        // 取唯一写者（前面已经校验最多1个写者）
        const auto writer = usage.writers[0];
        // 如果写者是定时系统，不属于compute图节点，不加入拓扑依赖
        if (writer.is_fixed_rate) {
            continue;
        }

        // 遍历这条通道所有读者
        for (const auto reader : usage.readers) {
            // 只关心【计算系统读者】，排除定时系统；禁止自依赖
            if (!reader.is_fixed_rate && writer.index != reader.index) {
                // 把reader.index对应的bit置1
                const std::uint64_t bit = 1UL << reader.index;
                // 防重复建边：避免in_degree多次累加同一个依赖
                if ((adj_mask[writer.index] & bit) == 0) {
                    adj_mask[writer.index] |= bit;
                    ++in_degree[reader.index];
                }
            }
        }
    }

    // 外部触发源掩码：不需要其他compute系统唤醒、可以主动启动依赖链的系统
    std::uint64_t external_source_mask = 0;
    // 第一类外部源：标记as_external_compute()的计算系统（外部IO信号触发）
    for (std::size_t i = 0; i < n; ++i) {
        if (compute_systems_[i].system->as_external_compute() != nullptr) {
            external_source_mask |= (1ULL << i);
        }
    }
    // 第二类外部源：开启notifies的定时系统写者，可以主动唤醒下游compute系统
    for (const auto& [key, usage] : channels) {
        if (usage.writers.empty() || usage.readers.empty()) {
            continue;
        }

        const auto writer = usage.writers[0];
        // 必须是定时系统，并且policy.notifies=true（才具备唤醒能力）
        if (!writer.is_fixed_rate || !fixed_rate_systems_[writer.index].policy.notifies) {
            continue;
        }

        // 这条定时通道的所有计算读者，加入外部源掩码
        for (const auto reader : usage.readers) {
            if (!reader.is_fixed_rate) {
                external_source_mask |= (1ULL << reader.index);
            }
        }
    }

    // ========================================================================
    // 阶段4：Kahn拓扑排序，生成并行执行层级levels_
    // levels内同一层的系统互相无依赖，可以多核并行执行
    // ========================================================================
    snapshot.levels.clear();
    std::vector<std::size_t> current_level;

    // 初始化：所有入度=0的系统，作为第一层（无前置依赖）
    for (std::size_t i = 0; i < n; ++i) {
        if (in_degree[i] == 0) {
            current_level.push_back(i);
        }
    }
    // 记录处理过的节点总数，用于环检测
    std::size_t processed = 0;
    // Kahn主循环：逐层处理
    while (!current_level.empty()) {
        // 当前层级存入快照
        snapshot.levels.push_back(current_level);
        processed += current_level.size();

        std::vector<std::size_t> next_level;
        next_level.reserve(compute_systems_.size());
        // 遍历当前层所有节点，释放下游依赖
        for (std::size_t node : current_level) {
            std::uint64_t neighbors = adj_mask[node];
            // 【位运算循环】遍历掩码里所有为1的bit
            while (neighbors) {
                // std::countr_zero【C++20】：unsigned整数，统计末尾连续0的个数 → 拿到最低置1bit下标
                const std::size_t neighbor = std::countr_zero(neighbors);
                // 清除最低位1：经典bit trick
                neighbors &= neighbors - 1;
                // 下游入度减一；入度归零 → 全部前置依赖就绪，加入下一层
                if (--in_degree[neighbor] == 0) {
                    next_level.push_back(neighbor);
                }
            }
        }
        // 移动语义【std::move】：转移vector所有权，不拷贝内存
        current_level = std::move(next_level);
    }

    // ========================================================================
    // 阶段5：环检测
    // Kahn特性：处理过节点总数 < 总节点数 → 图存在环（循环依赖，永远跑不完）
    // 用DFS回溯找到环路，收集系统名用于报错
    // ========================================================================
        // Kahn拓扑排序后校验：processed是成功处理的节点总数
    // 如果处理完的节点数 < 总compute系统数量n → 说明图里存在环（循环依赖）
    if (processed < n) {
        // 保存检测到的环路系统名称，用于报错打印
        std::vector<std::string> cycle;

        // 标记节点是否被DFS访问过（全局访问标记）
        std::vector visited(n, false);
        // 标记节点是否在当前递归DFS栈里（核心：用来判断回边，区分普通后向边和环）
        std::vector in_stack(n, false);
        // 记录当前DFS正在走的路径（节点下标数组），找到环后截取这段路径
        std::vector<std::size_t> path;

        // 【语法】std::function 包装lambda，允许lambda内部递归调用自己；原生匿名lambda不能直接递归
        // 签名：入参node=系统下标，返回bool=true代表找到环
        std::function<bool(std::size_t)> find_cycle = [&](const std::size_t node) -> bool {
            // 当前节点标记为已访问
            visited[node]  = true;
            // 当前节点压入递归栈标记
            in_stack[node] = true;
            // 当前节点加入DFS路径栈
            path.push_back(node);

            // adj_mask[node]：当前节点下游所有依赖节点的uint64位掩码
            std::uint64_t neighbors = adj_mask[node];
            // 循环：遍历掩码里所有bit=1的下游节点
            while (neighbors) {
                // 【语法】std::countr_zero C++20：求无符号数末尾连续0个数，快速拿到最低置1bit的下标
                const std::size_t neighbor = std::countr_zero(neighbors);
                // 【语法】经典bit trick：neighbors & (neighbors-1) → 清除最低位的1，迭代遍历所有置位bit
                neighbors &= neighbors - 1;

                // 情况1：下游节点还没访问过 → 继续递归DFS
                if (!visited[neighbor]) {
                    // 递归搜索子节点；一旦子分支找到环，直接向上传递true
                    if (find_cycle(neighbor)) {
                        return true;
                    }
                }
                // 情况2：下游节点已经访问过，**并且还在当前递归栈内 → 找到回边 = 检测出环！**
                else if (in_stack[neighbor]) {
                    // 在path数组里找到环的起点neighbor
                    auto it = std::find(path.begin(), path.end(), neighbor);
                    // 从环起点一直到path末尾，全部存入cycle（环路主体）
                    for (; it != path.end(); ++it) {
                        cycle.push_back(compute_systems_[*it].system->meta().name);
                    }
                    // 补上起点，形成闭环展示
                    cycle.push_back(compute_systems_[neighbor].system->meta().name);
                    // 找到环，返回true终止所有递归
                    return true;
                }
            }

            // 回溯逻辑：离开当前节点递归分支，弹出路径、取消栈标记
            path.pop_back();
            in_stack[node] = false;
            // 本分支无环，返回false
            return false;
        };

        // 遍历所有系统，只要还没找到环，并且节点入度>0、未访问 → 启动DFS搜环
        for (std::size_t i = 0; i < n && cycle.empty(); ++i) {
            if (in_degree[i] > 0 && !visited[i]) {
                find_cycle(i);
            }
        }

        // 构造错误返回值：【语法】std::unexpected 构造expected的失败分支，携带环路信息
        return std::unexpected(DependencyCycleError{.cycle = std::move(cycle)});
    }

    // ========================================================================
    // 阶段6：可达性校验（孤立系统检测）
    // 含义：必须能从外部触发源（外部IO/带notify的定时系统）顺着依赖链走到所有compute系统
    // 存在永远无法触发、跑不起来的孤立系统 → 直接报错
    // 算法：BFS扩散标记所有可达节点，底层用uint64位掩码做高性能集合运算
    // ========================================================================
    // 构造掩码：所有compute系统对应的全集掩码
    // 分支：刚好64个系统直接取uint64最大值全1；否则 1ULL左移n位再-1，低n位全部置1
    const auto all_compute_mask =
        n == 64 ? std::numeric_limits<std::uint64_t>::max() : ((1ULL << n) - 1ULL);
    // reachable：BFS过程中已经确认可达的系统掩码
    std::uint64_t reachable = external_source_mask;
    // frontier：BFS本轮待扩散的前沿节点掩码（待遍历）
    std::uint64_t frontier  = external_source_mask;

    // BFS主循环：前沿不为0就持续扩散下游依赖
    while (frontier != 0) {
        // next：本轮前沿节点能直接触达的所有下游节点集合
        std::uint64_t next = 0;
        // 拷贝一份前沿掩码，用来逐个取出bit
        auto sources       = frontier;
        // 遍历sources里所有激活bit
        while (sources != 0) {
            // 拿到最低置1bit下标，也就是当前待处理系统索引
            const auto idx = static_cast<std::size_t>(std::countr_zero(sources));
            // 清除最低位1，处理下一个
            sources &= sources - 1;
            // 把当前系统的所有下游节点合并进next掩码（按位或）
            next |= adj_mask[idx];
        }
        // 【语法】~ 按位取反；next & ~reachable = 只保留新发现、还没标记可达的节点，作为下一轮前沿
        frontier = next & ~reachable;
        // 把新可达节点并入总可达集合
        reachable |= frontier;
    }

    // 校验：可达集合 是否等于 全部compute系统集合
    // 不相等 = 存在bit不在reachable里 → 有孤立系统，永远无法被调度触发
    if ((reachable & all_compute_mask) != all_compute_mask) {
        // 存放所有不可达系统名字，用于报错
        std::vector<std::string> unreachable;
        // 逐个遍历所有compute系统，检查对应bit是否在可达掩码内
        for (std::size_t i = 0; i < n; ++i) {
            const auto bit = 1ULL << i;
            // 当前系统不在可达集合里
            if ((reachable & bit) == 0) {
                unreachable.push_back(compute_systems_[i].system->meta().name);
            }
        }
        // 返回错误：携带不可达系统列表
        return std::unexpected(UnreachableComputeSystemsError{.systems = std::move(unreachable)});
    }

    // ========================================================================
    // 阶段7：预生成两套唤醒掩码，存入拓扑快照TopologySnapshot，**专供运行时调度器使用**
    // 1. fixed_rate_affects：定时系统写通道 → 能够直接唤醒哪些compute系统（掩码）
    // 2. compute_affects：compute系统写通道 → 能够直接唤醒哪些compute系统（掩码）
    // 核心目的：运行时不用再遍历map、不用再解析通道，直接位运算判断唤醒，性能极高
    // ========================================================================
    // 初始化fixed_rate_affects数组：长度和定时系统数量一致，初始值全部0
    snapshot.fixed_rate_affects.assign(fixed_rate_systems_.size(), 0);

    // 【语法】auto& meta_channels 泛型lambda（C++14通用lambda）
    // 功能辅助函数：遍历某个系统的输出通道，收集它能直接唤醒的compute系统，写入affects掩码
    auto collect_direct_readers = [&](const auto& meta_channels, std::uint64_t& affects) {
        // 遍历当前系统注册的所有通道元信息
        for (const auto& ch : meta_channels) {
            // 只处理【写通道】，读者通道不会产生下游唤醒
            if (is_writer(ch.kind)) {
                // 构造通道唯一键：通道类型 + topic名称
                ChannelKey key{ch.type, ch.topic};
                // 【语法】C++17 if内初始化：find查找通道，找到才进入分支
                if (auto it = channels.find(key); it != channels.end()) {
                    // 遍历这条通道所有读者端点
                    for (const auto reader : it->second.readers) {
                        // 只收集compute系统读者，定时系统之间的唤醒不在这套掩码管理
                        if (!reader.is_fixed_rate) {
                            // 把读者下标对应的bit置1，合并进掩码
                            affects |= (1UL << reader.index);
                        }
                    }
                }
            }
        }
    };

    // ========== 填充 fixed_rate_affects：定时系统 → compute 的唤醒掩码 ==========
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        // 单个定时系统的唤醒掩码，初始0
        std::uint64_t affects = 0;
        // 取出定时系统预注册的元信息（spsc/spmc通道列表）
        const auto& meta      = fixed_rate_systems_[i].system->meta();
        // 处理该系统所有SPSC输出通道，收集唤醒掩码
        collect_direct_readers(meta.spsc_channels, affects);
        // 处理该系统所有SPMC输出通道，收集唤醒掩码
        collect_direct_readers(meta.spmc_channels, affects);
        // 写入快照数组，运行时直接读取
        snapshot.fixed_rate_affects[i] = affects;
    }

    // ========== 填充 compute_affects：compute系统 → compute系统 的唤醒掩码 ==========
    // 初始化数组：长度等于compute系统总数n，初始全部0
    snapshot.compute_affects.assign(n, 0);
    // 【语法】C++17结构化绑定，遍历channels这个map，直接取出key和usage
    for (const auto& [key, usage] : channels) {
        // 这条通道没有写者、没有读者 → 不存在依赖唤醒，跳过
        if (usage.writers.empty() || usage.readers.empty()) {
            continue;
        }

        // 取出这条通道唯一写者（前面静态校验已经保证最多1个写者）
        const auto writer = usage.writers[0];
        // 写者是定时系统，不归compute_affects管理，跳过
        if (writer.is_fixed_rate) {
            continue;
        }

        // 遍历这条通道所有读者
        for (const auto reader : usage.readers) {
            // 只关心compute读者，并且禁止系统自依赖（自己写自己读不算跨系统唤醒）
            if (!reader.is_fixed_rate && writer.index != reader.index) {
                // 读者对应bit置1，写入compute互相唤醒掩码
                snapshot.compute_affects[writer.index] |= (1UL << reader.index);
            }
        }
    }

    // 全部静态校验、依赖构图、唤醒掩码生成完成
    // 返回拓扑快照给调度器，后续运行时完全依靠这份快照做系统唤醒、并行调度
    return snapshot;
}

} // namespace talos::scheduler
