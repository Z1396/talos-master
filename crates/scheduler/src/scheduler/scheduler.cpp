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
#include <algorithm>       // std::any_of、std::sort、std::unique 通用算法
#include <bit>             // C++20 位运算工具：std::popcount、std::countr_zero
#include <cinttypes>       // PRIu64 格式化uint64_t打印宏定义
#include <cstdio>          // printf 控制台打印Mermaid图、系统信息
#include <functional>      // std::function 可调用对象、闭包存储
#include <limits>          // 数值极值模板类
#include <map>             // 有序关联容器，构建类型/通道名称映射
#include <nlohmann/json.hpp>// JSON序列化库，导出性能指标
#include <spdlog/spdlog.h> // 日志库，输出警告/错误日志
#include <unordered_map>   // 哈希表，加速话题计数查找

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
        const auto affects   = i < affects_mask.size() ? affects_mask[i] : 0U;

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
    std::size_t inputs  = 0;  // 读通道总数（spsc_reader / spmc_reader）
    std::size_t outputs = 0;  // 写通道总数（spsc_writer / spmc_writer）
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
        auto collect     = [&](const auto& channels) {
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
    if (is_running_state(lifecycle_.load(std::memory_order_acquire))) {
        panic("add_system: system already running!");
    }
}

/// 标记拓扑为脏状态：新增系统后依赖图失效，需要重新build构建拓扑
void Scheduler::mark_topology_dirty() noexcept {
    // 释放语义存储，同步其他线程读取
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
        .bound  = false, // 标记通道是否完成绑定初始化
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
    if (!lifecycle_.compare_exchange_strong(
            expected, LifecycleState::Running, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
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
void Scheduler::stop() noexcept {
    // CAS将Running状态切回Built
    auto expected = LifecycleState::Running;
    lifecycle_.compare_exchange_strong(
        expected, LifecycleState::Built, std::memory_order_acq_rel, std::memory_order_acquire);

    // 清除暂停标记，唤醒等待的计算循环线程
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
        if (auto* external = compute_systems_[i].system->as_external_compute();
            external != nullptr) {
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
    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        if (!fixed_rate_systems_[i].policy.notifies) {
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
        std::size_t system_idx;  // 系统在数组中的下标
        bool is_fixed_rate;      // true=FixedRate定时系统；false=Compute计算系统
    };

    // 单条通道完整连接关系：同一个(type+topic)下所有写者、读者
    struct ChannelConnection {
        std::vector<ChannelEndpoint> writers; // 该通道所有发布/写者
        std::vector<ChannelEndpoint> readers;  // 该通道所有订阅/读者
        std::type_index type;                  // 通道承载数据类型
        std::type_index topic;                 // 通道话题标识类型
        channel_kind kind;                     // 通道类型：SPSC/SPMC Reader/Writer
    };

    // 哈希表：ChannelKey(数据类型+话题) → 完整通道连接信息，存储全部Pub/Sub通道
    std::unordered_map<ChannelKey, ChannelConnection, ChannelKeyHash> channel_graph;

    // 共享资源访问结构：全局资源被哪些系统读、哪些系统修改
    struct ResourceAccess {
        std::vector<ChannelEndpoint> readers;   // 只读该资源的系统
        std::vector<ChannelEndpoint> mutators;  // 读写/修改该资源的系统
    };
    // 有序Map：资源type_index → 读写访问记录
    std::map<std::type_index, ResourceAccess> resource_graph;

    // 获取两类系统总数
    const std::size_t fixed_count      = fixed_rate_systems_.size();
    const std::size_t compute_count    = compute_systems_.size();

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

        const auto& meta   = compute_systems_[idx].system->meta();
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
        const auto& meta   = is_fixed ? fixed_rate_systems_[idx].system->meta()
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
        for (std::size_t i = 0; i < level.size(); ++i) {
            const auto sys_idx   = level[i];
            const auto& name     = compute_systems_[sys_idx].system->meta().name;
            const auto& meta     = compute_systems_[sys_idx].system->meta();
            // 统计该系统读写通道总数
            const auto io_counts = count_channel_io(meta);

            const auto* sys_type    = classify_by_edges(sys_idx);
            // 分配绘图样式
            const char* style_class = (strcmp(sys_type, "external") == 0) ? "inputStyle"
                                    : (strcmp(sys_type, "sink") == 0)     ? "outputStyle"
                                                                          : "pipelineStyle";
            // 外部系统额外标记文字
            const char* extra       = (strcmp(sys_type, "external") == 0) ? " [external]" : "";

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
    const auto now            = Clock::now();
    // 计算自调度器启动以来总运行秒数
    const auto elapsed_sec    = std::chrono::duration<double>(now - stats_start_time_).count();
    // 原子读取计算循环总执行次数（宽松内存序，仅统计，无需强同步）
    const auto compute_cycles = compute_cycles_.load(std::memory_order_relaxed);
    // 所有计算循环总耗时（纳秒）
    const auto compute_total  = compute_total_time_ns_.load(std::memory_order_relaxed);
    // 上一轮计算循环单次耗时（纳秒）
    const auto compute_last   = compute_last_time_ns_.load(std::memory_order_relaxed);
    // 汇总所有定时系统触发下游唤醒总次数
    const auto notify_total   = sum_notify_counts();

    printf("\n");
    printf("=== Scheduler Performance (%.1fs) ===\n", elapsed_sec);

    // 全局计算循环汇总指标
    printf("Loop: %-4" PRIu64 " cycles (%.1f Hz)", compute_cycles, compute_cycles / elapsed_sec);
    // 存在执行记录时打印平均耗时、最新单次耗时（转毫秒：ns / 1e6）
    if (compute_cycles > 0) {
        printf(
            " | avg: %.2f ms | last: %.2f ms", compute_total / 1e6 / compute_cycles,
            compute_last / 1e6);
    }
    // 全局唤醒总次数、平均唤醒频率
    printf(" | notifies: %-4" PRIu64 " (%.1f Hz)\n", notify_total, notify_total / elapsed_sec);

    // 内部工具：打印延迟分位数统计（最小、p50中位数、p95、最大值）
    auto print_latency_stats = [](const auto& latency) {
        // 存在采样数据才输出
        if (latency.sample_count > 0) {
            printf(
                "   min:%.2f p50:%.2f p95:%.2f max:%.2f ms", latency.min_ns / 1e6,
                latency.p50_ns / 1e6, latency.p95_ns / 1e6, latency.max_ns / 1e6);
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
            const auto latency    = sys_stats.latency_hist.compute();
            // 实际运行频率 = 有效记录次数 / 总运行时长
            const auto actual_hz  = record_cnt / elapsed_sec;
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
            const auto latency   = sys_stats.latency_hist.compute();
            // 实际运行频率
            const auto actual_hz = run_count / elapsed_sec;

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
    json& compute_json             = root["compute_loop"];
    compute_json["cycles"]         = cycles;
    compute_json["frequency_hz"]   = cycles / elapsed_sec;
    compute_json["avg_time_ms"]    = cycles > 0 ? (total_time / 1e6 / cycles) : 0.0;
    compute_json["last_time_ms"]   = last_time / 1e6;
    compute_json["total_notifies"] = notify_total;

    // 内部工具：将完整延迟分位数批量写入JSON对象
    auto add_latency_stats = [](json& sys, const auto& latency, double) {
        sys["sampled_runs"] = latency.sample_count;
        sys["min_ms"]       = latency.min_ns / 1e6;
        sys["p50_ms"]       = latency.p50_ns / 1e6;
        sys["p95_ms"]       = latency.p95_ns / 1e6;
        sys["p99_ms"]       = latency.p99_ns / 1e6;
        sys["p999_ms"]      = latency.p999_ns / 1e6;
        sys["mean_ms"]      = latency.mean_ns / 1e6;
        sys["max_ms"]       = latency.max_ns / 1e6;
        sys["stddev_ms"]    = latency.stddev_ns / 1e6;
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
            sys["actual_hz"]      = record_cnt / elapsed_sec;
            sys["count_mode"]     = write_mode ? "written_calls" : "run_calls";
        });

    // 填充计算系统JSON数组
    json& compute_arr = root["compute_systems"] = json::object();
    generate_system_stats(
        compute_systems_, compute_stats_, compute_arr,
        [elapsed_sec](json& sys, const auto&, const auto& stats) {
            const auto run_count = stats.run_count.load(std::memory_order_relaxed);
            sys["runs"]          = run_count;
            sys["actual_hz"]     = run_count / elapsed_sec;
        });

    // 序列化JSON为字符串返回
    return root.dump();
}

// ============================================================================
// FixedRate 定时后台线程执行逻辑
// 每个FixedRate系统独立一条pthread线程，支持CPU亲和、实时优先级、固定周期/无限速两种模式
// ============================================================================
void Scheduler::run_fixed_rate_thread(FixedRateContext ctx) {
    // 运行状态判断闭包：读取调度器全局生命周期原子标记
    const auto keep_running = [&ctx] {
        return is_running_state(ctx.lifecycle->load(std::memory_order_acquire));
    };

    // 1. 绑定CPU核心亲和性，隔离线程到指定CPU，减少上下文切换
    if (ctx.policy.cpu_affinity >= 0) {
        auto result = primitive::ThreadAffinity::pin_to_core(
            static_cast<std::uint32_t>(ctx.policy.cpu_affinity));
        // 绑定失败仅打印警告，不终止线程
        if (!result) {
            SPDLOG_WARN("{}: {}", ctx.system->meta().name, result.error());
        }
    }

    // 2. 设置Linux实时调度优先级（SCHED_FIFO/SCHED_RR）
    if (ctx.policy.thread_priority > 0) {
        primitive::ThreadAffinity::RealtimeConfig rt_config{};
        rt_config.priority = static_cast<std::uint8_t>(ctx.policy.thread_priority);
        if (auto result = primitive::ThreadAffinity::set_realtime_priority(rt_config); !result) {
            SPDLOG_WARN("{}: {}", ctx.system->meta().name, result.error());
        }
    }

    const auto frequency        = ctx.policy.frequency_hz;
    // 绑定当前系统对应的性能统计对象引用
    auto& stats                 = ctx.scheduler->fixed_rate_stats_[ctx.system_index];
    // 判断系统是否存在输出通道（执行后会唤醒下游）
    const bool write_mode       = counts_written_calls(ctx.system->meta());
    // 执行结果记录闭包：更新计数、写入延迟直方图、按需触发下游唤醒
    const auto record_execution = [&](const bool written, const std::uint64_t elapsed) {
        stats.execution_count.fetch_add(1, std::memory_order_relaxed);
        // 仅产生有效输出时记录运行次数与延迟
        if (!write_mode || written) {
            stats.record_count.fetch_add(1, std::memory_order_relaxed);
            stats.latency_hist.record(elapsed);
        }
        // 产生输出 且 配置允许唤醒下游 → 调用notify置位就绪掩码
        if (written && ctx.policy.notifies) [[likely]] {
            ctx.scheduler->notify(ctx.system_index);
        }
    };

    // 模式1：频率0，无限制全速循环运行（无定时休眠）
    if (frequency == 0) {
        while (keep_running()) {
            primitive::LatencyProbe probe;
            // 执行系统业务逻辑，返回值：是否产生数据输出
            const bool written = ctx.system->run(*ctx.world);
            // 获取本次执行耗时纳秒
            const auto elapsed = probe.elapsed_ns();

            record_execution(written, elapsed);
        }
        return;
    }

    // 模式2：固定周期定时运行
    [[assume(frequency > 0)]]; // 编译器提示消除除零优化分支
    // 计算单周期纳秒时长：1s / 频率
    const auto period =
        std::chrono::nanoseconds(1'000'000'000ULL / static_cast<std::uint64_t>(frequency));
    // 下一次执行的时间点
    auto next_tick = Clock::now();

    while (keep_running()) [[likely]] {
        // 推进下一次定时点
        next_tick += period;

        primitive::LatencyProbe probe;
        const bool written = ctx.system->run(*ctx.world);
        const auto elapsed = probe.elapsed_ns();

        record_execution(written, elapsed);

        // 休眠至下一个周期，高精度定时
        std::this_thread::sleep_until(next_tick);
    }
}

// ============================================================================
// Compute 主线程主循环（调度器主线程阻塞在此函数）
// 负责轮询就绪系统、自适应空闲退避、支持热更暂停、定时打印性能统计
// ============================================================================
void Scheduler::run_compute_loop() {
    // 自适应空闲退避三级策略：自旋 → 让出CPU → 短休眠
    // 目的：低负载时降低CPU占用，高负载无延迟
    constexpr std::size_t SPIN_LIMIT  = 100;  // 自旋最大迭代次数
    constexpr std::size_t YIELD_LIMIT = 1000;  // 让出CPU最大迭代次数
    constexpr auto SLEEP_DURATION     = std::chrono::microseconds(10); // 低负载休眠时长
    std::size_t idle_count            = 0;     // 连续空闲计数

    // 是否开启定时打印性能统计
    const auto should_print = config_.print_stats;
    // 打印统计间隔5秒
    auto print_interval     = std::chrono::seconds(5);
    auto last_print         = Clock::now();

    // 调度器处于运行状态则持续循环
    while (is_running()) [[likely]] {
        // 【罕见分支】收到热新增系统暂停请求
        if (pause_requested_.load(std::memory_order_acquire)) [[unlikely]] {
            idle_count = 0;

            // 标记主线程已进入暂停状态，通知热更线程
            {
                std::lock_guard lock(pause_mutex_);
                paused_.store(true, std::memory_order_release);
            }
            pause_cv_.notify_all();

            // 阻塞等待恢复信号
            {
                std::unique_lock lock(pause_mutex_);
                pause_cv_.wait(lock, [this] {
                    return !pause_requested_.load(std::memory_order_acquire) || !is_running();
                });
                paused_.store(false, std::memory_order_release);
            }

            // 恢复后调度器已停止，直接退出循环
            if (!is_running()) {
                break;
            }
            continue;
        }

        // 原子交换取出当前所有就绪系统掩码，同时清空就绪标记
        const auto ready = ready_systems_.exchange(0, std::memory_order_acq_rel);

        if (ready != 0) [[likely]] {
            // 存在待执行任务，重置空闲计数器
            idle_count = 0;

            primitive::LatencyProbe probe;
            // 分层并行执行所有就绪系统
            const auto result  = run_compute_selective(ready);
            const auto elapsed = probe.elapsed_ns();

            // 更新全局计算循环统计
            compute_cycles_.fetch_add(1, std::memory_order_relaxed);
            compute_total_time_ns_.fetch_add(elapsed, std::memory_order_relaxed);
            compute_last_time_ns_.store(elapsed, std::memory_order_relaxed);
            // 本轮执行产生新就绪系统，合并回全局就绪掩码
            if (result.deferred_mask != 0) {
                ready_systems_.fetch_or(result.deferred_mask, std::memory_order_release);
            }

            // 到达打印间隔则输出性能统计
            if (should_print) [[unlikely]] {
                const auto now = Clock::now();
                if (now - last_print >= print_interval) [[unlikely]] {
                    print_stats();
                    last_print = now;
                }
            }
        } else {
            // 无就绪任务，进入三级退避
            ++idle_count;

            if (idle_count <= SPIN_LIMIT) {
                // 阶段1：CPU自旋，延迟最低
                SPIN_HINT();
            } else if (idle_count <= YIELD_LIMIT) {
                // 阶段2：让出CPU给其他线程
                std::this_thread::yield();
            } else {
                // 阶段3：短休眠，大幅降低CPU占用
                std::this_thread::sleep_for(SLEEP_DURATION);
            }
        }
    }
    // 调度器退出前，顺序执行所有注册的清理钩子
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

    std::uint64_t pending_ready = ready_mask;    // 当前等待执行的系统掩码
    std::uint64_t executed_mask = 0;            // 本轮已执行完成系统掩码
    std::uint64_t written_mask  = 0;            // 本轮执行后产生输出的系统掩码
    std::uint64_t deferred_mask = 0;            // 本轮无法执行、延后到下一轮的系统掩码

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
// 7大阶段：数量校验 → 通道采集 → 通道合法性校验 → 构建依赖邻接掩码 → 外部源掩码计算 → Kahn拓扑分层 → 环检测+可达性校验 → 生成唤醒掩码
// ============================================================================
auto Scheduler::build_topology_snapshot() const -> std::expected<TopologySnapshot, BuildError> {
    const auto n = compute_systems_.size();
    TopologySnapshot snapshot;

    // 限制最多64个Compute系统（uint64_t位掩码存储依赖）
    if (n > TooManyComputeSystemsError::max_count) {
        return std::unexpected(TooManyComputeSystemsError{.count = n});
    }

    // ========================================================================
    // 阶段1：遍历所有系统，采集全部通道读写端点
    // ========================================================================
    // 判断通道是否为写者
    const auto is_writer = [](channel_kind k) {
        return k == channel_kind::spsc_writer || k == channel_kind::spmc_writer;
    };

    // 判断通道是否SPSC类型
    const auto is_spsc = [](channel_kind k) {
        return k == channel_kind::spsc_reader || k == channel_kind::spsc_writer;
    };

    // 通道端点：标记属于定时/计算系统+下标
    struct ChannelEndpoint {
        std::size_t index;
        bool is_fixed_rate;
    };

    // 单条通道全量读写使用记录
    struct ChannelUsage {
        channel_kind kind = channel_kind::local;
        std::vector<ChannelEndpoint> writers;
        std::vector<ChannelEndpoint> readers;
        std::string first_system; // 首次注册该通道的系统名，用于冲突报错
    };

    std::map<ChannelKey, ChannelUsage> channels;

    // 格式化端点所属系统名称，报错时打印
    auto endpoint_name = [&](const ChannelEndpoint endpoint) -> std::string {
        const auto& name = endpoint.is_fixed_rate
                             ? fixed_rate_systems_[endpoint.index].system->meta().name
                             : compute_systems_[endpoint.index].system->meta().name;
        if (endpoint.is_fixed_rate) {
            return fmt::format("{} (fixed_rate)", name);
        }
        return name;
    };

    // 遍历一组系统的所有通道，填充全局通道记录表，校验SPSC/SPMC类型冲突
    auto process_system_channels = [&](const auto& systems,
                                       const bool is_fixed_rate) -> BuildResult {
        for (std::size_t i = 0; i < systems.size(); ++i) {
            const auto& meta = systems[i].system->meta();

            // 处理单条通道，校验类型冲突，写入读写端点
            auto process_channel = [&](const ChannelMeta& ch,
                                       const bool expect_spsc) -> BuildResult {
                ChannelKey key{ch.type, ch.topic};
                auto& usage              = channels[key];
                const bool has_endpoints = !usage.writers.empty() || !usage.readers.empty();

                // 通道已存在，校验SPSC/SPMC类型是否统一
                if (has_endpoints) {
                    const auto is_expected_kind =
                        expect_spsc ? is_spsc(usage.kind) : !is_spsc(usage.kind);
                    if (!is_expected_kind) {
                        return std::unexpected(
                            ChannelKindConflict{
                                .key           = ChannelKeyInfo(ch.type, ch.topic, expect_spsc),
                                .first_system  = usage.first_system,
                                .second_system = meta.name,
                            });
                    }
                } else {
                    // 首次注册该通道，记录类型与来源系统
                    usage.kind         = ch.kind;
                    usage.first_system = meta.name;
                }

                const ChannelEndpoint endpoint{
                    .index         = i,
                    .is_fixed_rate = is_fixed_rate,
                };
                // 区分写入者/读者列表
                if (is_writer(ch.kind)) {
                    usage.writers.push_back(endpoint);
                } else {
                    usage.readers.push_back(endpoint);
                }
                return {};
            };

            // 处理SPSC通道组
            for (const auto& ch : meta.spsc_channels) {
                if (auto result = process_channel(ch, true); !result) {
                    return result;
                }
            }
            // 处理SPMC通道组
            for (const auto& ch : meta.spmc_channels) {
                if (auto result = process_channel(ch, false); !result) {
                    return result;
                }
            }
        }
        return {};
    };

    // 先遍历定时系统通道，再遍历计算系统通道
    if (auto result = process_system_channels(fixed_rate_systems_, true); !result) {
        return std::unexpected(result.error());
    }
    if (auto result = process_system_channels(compute_systems_, false); !result) {
        return std::unexpected(result.error());
    }

    // ========================================================================
    // 阶段2：通道强约束合法性校验（报错阻断构建）
    // 1. 通道多写者  2.SPSC多读者  3.只有读者无写者
    // 仅计算系统无读者写者输出警告，不阻断
    // ========================================================================
    auto collect_endpoint_names = [&](const std::vector<ChannelEndpoint>& endpoints) {
        std::vector<std::string> names;
        names.reserve(endpoints.size());
        for (const auto endpoint : endpoints) {
            names.push_back(endpoint_name(endpoint));
        }
        return names;
    };

    for (const auto& [key, usage] : channels) {
        // 错误1：任意通道存在多个写者，直接报错
        if (usage.writers.size() > 1) {
            return std::unexpected(
                MultipleWritersError{
                    .key     = ChannelKeyInfo(key.type, key.topic, is_spsc(usage.kind)),
                    .writers = collect_endpoint_names(usage.writers),
                });
        }

        // 错误2：SPSC通道多个读者，单生产者单消费者规范不允许
        if (is_spsc(usage.kind) && usage.readers.size() > 1) {
            return std::unexpected(
                MultipleReadersError{
                    .key     = ChannelKeyInfo(key.type, key.topic, true),
                    .readers = collect_endpoint_names(usage.readers),
                });
        }

        // 错误3：通道只有读者，无任何写者，数据无来源
        if (usage.writers.empty() && !usage.readers.empty()) {
            return std::unexpected(
                OrphanedReaderError{
                    .key     = ChannelKeyInfo(key.type, key.topic, is_spsc(usage.kind)),
                    .readers = collect_endpoint_names(usage.readers),
                });
        }

        // 警告：计算系统写者、无任何读者（定时系统写者无读者合法，作为周期数据源）
        if (!usage.writers.empty() && usage.readers.empty() && !usage.writers[0].is_fixed_rate) {
            SPDLOG_WARN(
                "[WARN] Channel has writer '{}' but no readers\n",
                compute_systems_[usage.writers[0].index].system->meta().name.c_str());
        }
    }

    // ========================================================================
    // 阶段3：构建计算系统间依赖邻接掩码、入度数组（仅计算系统互相依赖）
    // adj_mask[i] = i执行完成后可唤醒的下游系统位掩码
    // in_degree[i] = i的前置依赖系统总数，用于Kahn拓扑排序
    // ========================================================================
    std::vector<std::uint64_t> adj_mask(n, 0);
    std::vector<std::size_t> in_degree(n, 0);

    for (const auto& [key, usage] : channels) {
        if (usage.writers.empty() || usage.readers.empty()) {
            continue;
        }

        const auto writer = usage.writers[0];
        // 定时系统写者不参与计算系统拓扑分层，跳过
        if (writer.is_fixed_rate) {
            continue;
        }

        // 遍历所有计算系统读者，建立依赖边
        for (const auto reader : usage.readers) {
            if (!reader.is_fixed_rate && writer.index != reader.index) {
                const std::uint64_t bit = 1UL << reader.index;
                if ((adj_mask[writer.index] & bit) == 0) {
                    adj_mask[writer.index] |= bit;
                    ++in_degree[reader.index];
                }
            }
        }
    }

    // 计算外部触发源掩码：两类可主动唤醒计算系统的源头
    // 1. 外部IO触发系统 as_external_compute()  2.可唤醒下游的定时系统写者
    std::uint64_t external_source_mask = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (compute_systems_[i].system->as_external_compute() != nullptr) {
            external_source_mask |= (1ULL << i);
        }
    }
    for (const auto& [key, usage] : channels) {
        if (usage.writers.empty() || usage.readers.empty()) {
            continue;
        }

        const auto writer = usage.writers[0];
        // 仅可唤醒下游的定时系统才作为外部源
        if (!writer.is_fixed_rate || !fixed_rate_systems_[writer.index].policy.notifies) {
            continue;
        }

        for (const auto reader : usage.readers) {
            if (!reader.is_fixed_rate) {
                external_source_mask |= (1ULL << reader.index);
            }
        }
    }

    // ========================================================================
    // 阶段4：Kahn拓扑排序，生成并行执行层级levels_
    // 同层级系统无依赖，可多核并行执行
    // ========================================================================
    snapshot.levels.clear();
    std::vector<std::size_t> current_level;

    // 初始入度为0的系统为第一层
    for (std::size_t i = 0; i < n; ++i) {
        if (in_degree[i] == 0) {
            current_level.push_back(i);
        }
    }

    std::size_t processed = 0;
    while (!current_level.empty()) {
        snapshot.levels.push_back(current_level);
        processed += current_level.size();

        std::vector<std::size_t> next_level;
        next_level.reserve(compute_systems_.size());
        // 遍历当前层级所有节点，减少下游入度
        for (std::size_t node : current_level) {
            std::uint64_t neighbors = adj_mask[node];
            while (neighbors) {
                const std::size_t neighbor = std::countr_zero(neighbors);
                neighbors &= neighbors - 1;
                // 下游依赖全部满足，加入下一层
                if (--in_degree[neighbor] == 0) {
                    next_level.push_back(neighbor);
                }
            }
        }
        current_level = std::move(next_level);
    }

    // ========================================================================
    // 阶段5：环检测
    // Kahn处理节点总数 < 总系统数 → 存在依赖环，DFS遍历提取环路报错
    // ========================================================================
    if (processed < n) {
        std::vector<std::string> cycle;

        std::vector visited(n, false);
        std::vector in_stack(n, false);
        std::vector<std::size_t> path;

        // DFS深度优先搜索查找环路
        std::function<bool(std::size_t)> find_cycle = [&](const std::size_t node) -> bool {
            visited[node]  = true;
            in_stack[node] = true;
            path.push_back(node);

            std::uint64_t neighbors = adj_mask[node];
            while (neighbors) {
                const std::size_t neighbor = std::countr_zero(neighbors);
                neighbors &= neighbors - 1;

                if (!visited[neighbor]) {
                    if (find_cycle(neighbor)) {
                        return true;
                    }
                } else if (in_stack[neighbor]) {
                    // 找到环路起点，截取路径生成环路名称列表
                    auto it = std::find(path.begin(), path.end(), neighbor);
                    for (; it != path.end(); ++it) {
                        cycle.push_back(compute_systems_[*it].system->meta().name);
                    }
                    cycle.push_back(compute_systems_[neighbor].system->meta().name);
                    return true;
                }
            }

            path.pop_back();
            in_stack[node] = false;
            return false;
        };

        // 从存在剩余入度的节点开始DFS
        for (std::size_t i = 0; i < n && cycle.empty(); ++i) {
            if (in_degree[i] > 0 && !visited[i]) {
                find_cycle(i);
            }
        }

        return std::unexpected(DependencyCycleError{.cycle = std::move(cycle)});
    }

    // ========================================================================
    // 阶段6：可达性校验（无外部源头可触发的孤立系统报错）
    // 从外部源BFS遍历，所有未被遍历到的系统不可触发执行
    // ========================================================================
    const auto all_compute_mask =
        n == 64 ? std::numeric_limits<std::uint64_t>::max() : ((1ULL << n) - 1ULL);
    std::uint64_t reachable = external_source_mask;
    std::uint64_t frontier  = external_source_mask;

    // BFS扩散所有可到达系统
    while (frontier != 0) {
        std::uint64_t next = 0;
        auto sources       = frontier;
        while (sources != 0) {
            const auto idx = static_cast<std::size_t>(std::countr_zero(sources));
            sources &= sources - 1;
            next |= adj_mask[idx];
        }
        frontier = next & ~reachable;
        reachable |= frontier;
    }

    // 存在不可达系统，收集名称返回错误
    if ((reachable & all_compute_mask) != all_compute_mask) {
        std::vector<std::string> unreachable;
        for (std::size_t i = 0; i < n; ++i) {
            const auto bit = 1ULL << i;
            if ((reachable & bit) == 0) {
                unreachable.push_back(compute_systems_[i].system->meta().name);
            }
        }
        return std::unexpected(UnreachableComputeSystemsError{.systems = std::move(unreachable)});
    }

    // ========================================================================
    // 阶段7：生成两套唤醒掩码存入拓扑快照
    // 1.fixed_rate_affects：定时系统写者直接下游计算系统掩码
    // 2.compute_affects：计算系统写者直接下游计算系统掩码
    // ========================================================================
    snapshot.fixed_rate_affects.assign(fixed_rate_systems_.size(), 0);

    // 遍历定时系统所有输出通道，收集直接读者
    auto collect_direct_readers = [&](const auto& meta_channels, std::uint64_t& affects) {
        for (const auto& ch : meta_channels) {
            if (is_writer(ch.kind)) {
                ChannelKey key{ch.type, ch.topic};
                if (auto it = channels.find(key); it != channels.end()) {
                    for (const auto reader : it->second.readers) {
                        if (!reader.is_fixed_rate) {
                            affects |= (1UL << reader.index);
                        }
                    }
                }
            }
        }
    };

    for (std::size_t i = 0; i < fixed_rate_systems_.size(); ++i) {
        std::uint64_t affects = 0;
        const auto& meta      = fixed_rate_systems_[i].system->meta();
        collect_direct_readers(meta.spsc_channels, affects);
        collect_direct_readers(meta.spmc_channels, affects);
        snapshot.fixed_rate_affects[i] = affects;
    }

    // 生成计算系统互相唤醒掩码
    snapshot.compute_affects.assign(n, 0);
    for (const auto& [key, usage] : channels) {
        if (usage.writers.empty() || usage.readers.empty()) {
            continue;
        }

        const auto writer = usage.writers[0];
        if (writer.is_fixed_rate) {
            continue;
        }

        for (const auto reader : usage.readers) {
            if (!reader.is_fixed_rate && writer.index != reader.index) {
                snapshot.compute_affects[writer.index] |= (1UL << reader.index);
            }
        }
    }

    // 全部校验通过，返回完整拓扑快照
    return snapshot;
}

} // namespace talos::scheduler



