// 头文件保护：防止头文件重复包含，标准现代 C++ 写法
#pragma once

/**
 * @file
 * @brief 数据驱动型任务执行调度器
 *
 * 本模块实现一套**基于数据流依赖**的机器人任务调度器，核心管理两类任务：
 * 1. fixed_rate 定频任务：独占线程、固定周期执行
 * 2. compute 计算任务：基于 TBB 线程池、被数据事件触发执行
 * 整体依靠**通道(Channel)依赖图**实现按需唤醒、拓扑序执行。
 *
 * ## 整体架构说明
 * - **FixedRate 定频系统**：独立专属线程运行，产生数据后可主动通知调度器
 * - **Compute 计算系统**：运行在 TBB 线程池，由上游数据通知触发执行
 * - **依赖图(Dependency graph)**：由数据通道连接关系自动构建，实现选择性唤醒
 * - **资源(Resources)**：仅作为共享状态存在，**不参与调度依赖、不产生调度边**
 *
 * ## 执行流程模型
 * 1. FixedRate 系统在各自专属线程中周期运行
 * 2. 当 FixedRate 系统向通道写入数据时，触发下游依赖的 Compute 系统
 * 3. Compute 系统按照**拓扑分层(levels)** 执行，直到所有就绪任务执行完毕
 * 4. TBB Arena 统一管理线程池并发数
 *
 * ## 构建阶段校验规则（调用 build() 时自动校验）
 * 调度器在构建依赖图时会做严格合法性检查，不满足则直接返回错误：
 * - 单通道仅允许**唯一写入者**（兼容 SPSC/SPMC 通道模型）
 * - 依赖图**禁止出现环路**（循环依赖会导致死锁/无限执行）
 * - 检测孤立读取者（有读无写的无效节点）
 * - 计算系统总数上限 64 个（基于 64 位位掩码做快速唤醒调度）
 */

// 性能探针：用于耗时统计、延迟直方图
#include "primitive/performance_probe.hpp"
// 调度器错误码、错误类型定义
#include "scheduler/error.hpp"
// 系统执行策略枚举、配置
#include "scheduler/system/execution_policy.hpp"
// 系统基类定义，所有任务系统的抽象父类
#include "scheduler/system/system.hpp"
// 轻量调度器前置声明/基础定义
#include "scheduler/thin.hpp"
// 全局世界容器：管理所有共享资源、数据通道 Channel
#include "scheduler/world.hpp"

// C++ 标准库
#include <atomic>          // 原子变量：无锁线程安全、状态标记、计数
#include <chrono>          // 高精度时间、定时、周期控制
#include <condition_variable> // 条件变量：线程阻塞/唤醒（热更、暂停）
#include <cstddef>         // 标准类型 size_t
#include <cstdint>         // 定长整数 uint8_t/uint64_t 等
#include <expected>        // C++23 错误返回：结果+错误信息
#include <functional>      // 可调用对象、回调函数（关机钩子）
#include <memory>          // 智能指针 unique_ptr/shared_ptr
#include <mutex>           // 互斥锁：临界区保护
#include <new>             // 内存对齐相关
#include <string>          // 字符串（系统名、日志、JSON 统计）
#include <thread>          // 标准线程
#include <utility>         // 移动语义、std::forward
#include <vector>          // 动态数组，存储系统、线程、统计数据

// 前置声明 TBB 内部命名空间，兼容不同 TBB 版本
namespace tbb::detail::d1 {
class task_arena;
} // namespace tbb::detail::d1

// 对外别名：简化 TBB 线程池 arena 使用
namespace tbb {
using task_arena = tbb::detail::d1::task_arena;
} // namespace tbb

// Talos 调度器主命名空间
namespace talos::scheduler {

// ============================================================================
// 内部结构体：SystemEntry 系统条目
// 作用：调度器内部存储单个注册系统的完整信息
// ============================================================================
/**
 * @brief 已注册系统的内部存储结构
 * 封装系统实例、执行策略、绑定状态，是调度器管理所有任务的基础单元
 */
struct SystemEntry {
    // 系统基类智能指针：持有具体业务系统实例
    std::unique_ptr<SystemBase> system;
    // 执行策略信息：区分 fixed_rate / compute / 可视化等类型
    PolicyInfo policy;
    // 标记该系统是否已完成资源/通道绑定（bind 阶段）
    bool bound = false;
};

// 导入 system 子命名空间，简化后续代码书写
using namespace system;

// ============================================================================
// 主类：Scheduler 核心调度器
// ============================================================================
/**
 * @brief 数据驱动任务调度器主类
 * 统一管理 定频任务(fixed_rate) + 池化计算任务(compute)，基于数据流依赖自动调度
 *
 * ## 使用示例
 * ```cpp
 * World world;
 * Scheduler scheduler(world);
 *
 * // 注册 30Hz 定频相机采集任务
 * scheduler.add_system<fixed_rate<30>>("camera", [](talos::spmc_mut<fcs::ImageFrame,
 * fcs::ImageChannelTopic> cam_out) {
 *     // 相机业务逻辑，向通道写入图像数据
 *     cam_out.write(...);
 * });
 *
 * // 构建依赖图、校验拓扑
 * auto result = scheduler.build();
 * if (!result) {
 *     // 处理构建失败错误
 * }
 *
 * scheduler.run();  // 阻塞运行，直到调用 stop() 停止
 * ```
 *
 * ## 线程安全约束
 * 1. **构造完成后、run() 之前**：单线程配置、注册系统、调用 build()
 * 2. build() 成功后：**资源结构冻结**
 *    - 已有资源内部数据可修改
 *    - 禁止新增/替换资源（系统会缓存资源裸指针，修改会导致野指针）
 * 3. 运行阶段：仅调用 stop()、查询状态、统计接口，禁止修改拓扑
 */
class Scheduler {
public:
    // 调度器使用的时钟类型：steady_clock 单调时钟（不受系统时间修改影响，适合定时/延时）
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;  // 时间点
    using Duration  = Clock::duration;    // 时间间隔

    /**
     * @brief 调度器生命周期状态枚举
     * 用 uint8_t 节省内存，原子变量无锁状态切换
     */
    enum class LifecycleState : std::uint8_t {
        Configuring, ///< 配置阶段：可新增系统、修改拓扑，必须先 build 才能运行
        Built,       ///< 构建完成：依赖图校验通过、拓扑就绪，等待启动
        Running,     ///< 运行中：定频线程、计算循环全部活跃
    };

    /**
     * @brief 调度器构造函数
     * @param config 调度器配置参数，默认空配置
     * noexcept 保证构造不抛异常
     1. explicit作用核心
     * 修饰单参数构造函数 / 转换构造函数，禁止编译器隐式类型转换，只允许显式调用构造。
     */
    explicit Scheduler(SchedulerConfig config = {}) noexcept;

    /**
     * @brief 析构函数
     * 自动回收线程、TBB 资源、所有系统实例
     */
    ~Scheduler() noexcept;

    // 禁用拷贝构造、拷贝赋值：调度器全局唯一，不允许复制
    Scheduler(const Scheduler&)            = delete;
    Scheduler& operator=(const Scheduler&) = delete;
    // 禁用移动构造、移动赋值：运行时拓扑/线程不允许转移所有权
    Scheduler(Scheduler&&)                 = delete;
    Scheduler& operator=(Scheduler&&)      = delete;

    // ========================================================================
    // 一、系统注册接口：向调度器添加任务系统
    // ========================================================================

    /**
     * @brief 模板接口：快速注册一个业务系统（最常用）
     * 自动推导执行策略、函数类型，内部生成 System 实例
     *
     * @tparam Policy 执行策略：fixed_rate / pool_compute / fixed_rate_silent 等
     * @tparam F 可调用对象类型（函数、Lambda、仿函数）
     * @param name 系统名称（调试、日志、可视化使用），右值移动避免拷贝
     * @param func 系统执行体（业务逻辑）
     */
    template <typename Policy = default_policy, typename F>
    auto add_system(std::string&& name, F&& func) -> void;

    /**
     * @brief 重载接口：添加自定义实现的系统（高级用法）
     * 允许外部手动继承 SystemBase 实现自定义绑定、运行逻辑
     *
     * @param system 外部构造好的系统基类智能指针
     * @return std::expected<系统ID, 错误信息> 成功返回系统下标，失败返回错误
     *
     * 约束说明：
     * 1. 仅调度器**非运行状态**可调用
     * 2. 系统策略由自身 meta().policy 决定
     * 3. 通道/资源必须在 bind() 阶段获取，run() 阶段禁止动态修改拓扑
     */
    [[nodiscard]] auto add_system(std::unique_ptr<SystemBase> system)
        -> std::expected<uint64_t, SchedulerError>;

    /**
     * @brief 获取全局 World 容器引用
     * World 是所有共享资源、数据通道的顶层容器
     *
     * 约束：
     * - build() 成功前可修改资源结构
     * - build() 后仅允许修改资源内部数据，**禁止新增/替换资源**
     */
    [[nodiscard]] World& world() noexcept;

    // ========================================================================
    // 二、生命周期接口：构建、启动、停止、热更新
    // ========================================================================

    /**
     * @brief 构建依赖图 + 拓扑校验（核心初始化步骤）
     * 执行全量合法性检查，生成任务分层、唤醒掩码
     *
     * 校验项：
     * - 依赖图无环路
     * - 通道单写入者约束
     * - 无孤立读取节点
     * - 所有计算系统都能被外部源触发
     *
     * 特性：
     * - 可重复调用（幂等），每次都会重新校验、重建拓扑
     * - 资源结构 build 后冻结
     * @return 构建结果：成功/详细错误
     * 线程安全：必须单线程调用，运行中调用会直接报错
     */
    [[nodiscard]] auto build() -> BuildResult;

    /**
     * @brief 启动调度器，阻塞当前线程直到 stop()
     * 拉起所有 fixed_rate 专属线程，启动 compute 计算主循环
     * @return 启动结果/生命周期错误
     */
    [[nodiscard]] auto run() -> std::expected<void, SchedulerError>;

    /**
     * @brief 优雅停止调度器
     * 状态切回 Built，定频线程执行完当前周期后正常退出
     * noexcept 无异常
     */
    void stop() noexcept;

    /**
     * @brief 注册关机回调钩子
     * 在 stop() 执行阶段、系统停止前按注册顺序依次执行
     * 用途：相机停流、硬件断电、资源释放等前置清理逻辑
     */
    void add_shutdown_hook(std::function<void()> hook);

    /**
     * @brief 安全热添加系统（运行前可用）
     * 等价于 add_system + build，**仅允许 run() 之前调用**
     * 运行中调用会直接失败，并提示使用 unsafe 接口
     * @param system 待添加系统
     * @return 构建结果
     */
    [[nodiscard]] auto hot_add_system(std::unique_ptr<SystemBase> system) -> BuildResult;

    /**
     * @brief 实验性 运行时热更接口（不安全）
     * 运行中可动态新增系统，仅暂停计算循环重建拓扑，**不保证全量线程安全**
     * 仅限调试、实验场景使用，正式业务禁止依赖
     */
    [[nodiscard]] auto unsafe_hot_add_system(std::unique_ptr<SystemBase> system) -> BuildResult;

    /**
     * @brief 查询调度器是否处于运行状态
     */
    [[nodiscard]] auto is_running() const noexcept -> bool;

    /**
     * @brief 获取当前完整生命周期状态
     */
    [[nodiscard]] auto lifecycle_state() const noexcept -> LifecycleState;

    // ========================================================================
    // 三、通知接口：定频系统主动触发下游计算任务
    // ========================================================================

    /**
     * @brief 通知调度器：当前定频系统已产生新数据
     * 由 fixed_rate 线程主动调用，通过**位掩码**快速唤醒下游依赖计算系统
     * @param system_index 定频系统在数组中的下标
     * 线程安全：可在任意定频任务线程中调用
     */
    void notify(std::size_t system_index) noexcept;

    // ========================================================================
    // 四、诊断、可视化、统计接口
    // ========================================================================

    /**
     * @brief 调度器全局运行统计信息
     */
    struct Stats {
        std::uint64_t notify_count        = 0; ///< 总通知次数
        std::uint64_t compute_cycle_count = 0; ///< 计算循环总执行次数
    };

    /**
     * @brief 获取运行时统计数据
     */
    [[nodiscard]] auto stats() const noexcept -> Stats;

    /**
     * @brief 打印所有已注册系统信息（名称、策略、通道数）到控制台
     */
    void print_systems() const;

    /**
     * @brief 输出 Mermaid 流程图：唤醒链
     * 展示「谁能唤醒谁」的触发关系，不含纯资源依赖
     * 要求：必须先执行 build() 生成拓扑
     */
    void print_mermaid_wake_chains() const;

    /**
     * @brief 输出 Mermaid 流程图：完整数据流 DAG
     * 包含系统、通道、共享资源，区分：
     * - 实线：通道数据流（触发调度）
     * - 虚线：共享资源访问（不触发调度）
     */
    void print_mermaid_data_flow() const;

    /**
     * @brief 输出 Mermaid 流程图：执行分层拓扑
     * 展示计算系统拓扑分层，同层系统可并行执行
     * 附带流水线深度、最大并行度分析
     */
    void print_mermaid_execution_levels() const;

    /**
     * @brief 打印性能统计、延迟直方图、吞吐率到控制台
     */
    void print_stats() const;

    /**
     * @brief 获取 JSON 格式性能统计字符串（便于外部监控、上位机解析）
     */
    [[nodiscard]] std::string get_stats_json() const;

// ========================================================================
// 私有成员：内部类型、方法、变量（外部不可访问）
// ========================================================================
private:
    // ------------------------------
    // 内部上下文结构体
    // ------------------------------
    /**
     * @brief 定频任务线程运行上下文
     * 传递给专属线程的所有依赖指针、下标、状态
     */
    struct FixedRateContext {
        SystemBase* system;               // 系统实例裸指针
        PolicyInfo policy;                // 执行策略
        std::atomic<LifecycleState>* lifecycle; // 全局生命周期状态指针
        Scheduler* scheduler;             // 调度器自身指针
        World* world;                     // 全局资源容器指针
        std::size_t system_index;         // 系统下标
    };

    /**
     * @brief 定频系统性能统计
     * alignas：硬件缓存行对齐，解决伪共享（多线程原子变量高频访问优化）
     */
    struct alignas(std::hardware_destructive_interference_size) FixedRateStats {
        std::atomic<std::uint64_t> notify_count{0};   // 触发通知次数
        std::atomic<std::uint64_t> execution_count{0};// 执行次数
        std::atomic<std::uint64_t> record_count{0};   // 数据产出次数
        primitive::LatencyHistogram latency_hist;     // 延迟直方图

        FixedRateStats() = default;

        // 移动构造：原子变量不可拷贝，手动加载赋值
        FixedRateStats(FixedRateStats&& other) noexcept;

        // 禁用拷贝、拷贝赋值、移动赋值
        FixedRateStats(const FixedRateStats&)            = delete;
        FixedRateStats& operator=(const FixedRateStats&) = delete;
        FixedRateStats& operator=(FixedRateStats&&)      = delete;
    };

    /**
     * @brief 计算系统性能统计
     */
    struct ComputeStats {
        std::atomic<std::uint64_t> run_count{0}; // 执行次数
        primitive::LatencyHistogram latency_hist;// 延迟直方图

        ComputeStats() = default;
        ComputeStats(ComputeStats&& other) noexcept;
        ComputeStats(const ComputeStats&)            = delete;
        ComputeStats& operator=(const ComputeStats&) = delete;
        ComputeStats& operator=(ComputeStats&&)      = delete;
    };

    // ------------------------------
    // 计算循环内部返回结构
    // ------------------------------
    /**
     * @brief 单次计算轮次执行结果
     */
    struct ComputeRoundResult {
        std::uint64_t written_mask  = 0;  // 本轮产生新数据的系统位掩码
        std::uint64_t deferred_mask = 0;  // 本轮延迟执行/未就绪系统位掩码
    };

    /**
     * @brief 拓扑快照：构建完成后的稳定拓扑数据
     * 用于运行时快速切换、热更后原子替换
     */
    struct TopologySnapshot {
        std::vector<std::vector<std::size_t>> levels;          // 计算系统分层
        std::vector<std::uint64_t> fixed_rate_affects;        // 定频系统影响掩码
        std::vector<std::uint64_t> compute_affects;           // 计算系统依赖掩码
    };

    // ------------------------------
    // 内部私有方法
    // ------------------------------
    /**
     * @brief 定频线程入口函数（静态）
     * 循环执行对应定频系统业务逻辑
     */
    static void run_fixed_rate_thread(FixedRateContext ctx);

    /**
     * @brief 计算任务主循环（持续运行，处理就绪计算任务）
     */
    void run_compute_loop();

    /**
     * @brief 选择性执行就绪计算系统（按位掩码筛选）
     * @param ready_mask 就绪系统位掩码
     * @return 本轮执行结果
     */
    [[nodiscard]] ComputeRoundResult run_compute_selective(std::uint64_t ready_mask);

    /**
     * @brief 生成拓扑快照：分析依赖、分层、生成掩码
     * @return 拓扑快照 或 构建错误
     */
    [[nodiscard]] auto build_topology_snapshot() const
        -> std::expected<TopologySnapshot, BuildError>;

    /**
     * @brief 原子提交新拓扑快照，运行时无缝切换
     */
    void commit_topology(TopologySnapshot snapshot) noexcept;

    /**
     * @brief 绑定外部计算源（如外部话题、网络数据等非系统触发源）
     */
    void bind_external_compute_sources() noexcept;

    /**
     * @brief 工具函数：判断状态是否为运行中
     */
    static bool is_running_state(LifecycleState state) noexcept;

    // 热更新：暂停/恢复计算循环
    void request_compute_loop_pause() noexcept;
    void resume_compute_loop() noexcept;

    /**
     * @brief 汇总所有定频系统的通知总数
     */
    [[nodiscard]] std::uint64_t sum_notify_counts() const noexcept;

    /**
     * @brief 校验：如果调度器正在运行则直接终止操作（防非法修改）
     */
    void ensure_not_running() const noexcept;

    /**
     * @brief 标记拓扑已脏：新增系统后需要重新 build
     */
    void mark_topology_dirty() noexcept;

    // ------------------------------
    // 核心成员变量
    // ------------------------------
    World world_{};                                   // 全局资源&通道容器
    SchedulerConfig config_;                          // 调度器配置

    // 系统列表：分两类存储
    std::vector<SystemEntry> fixed_rate_systems_;      // 定频系统数组
    std::vector<SystemEntry> compute_systems_;         // 计算系统数组

    std::vector<std::vector<std::size_t>> levels_;     // 计算系统拓扑分层（同层可并行）

    std::vector<std::jthread> fixed_rate_threads_;    // 定频专属线程（C++20 jthread 自动join）

    std::unique_ptr<tbb::task_arena> compute_arena_;  // TBB 线程池 Arena，管理计算并发

    std::atomic<LifecycleState> lifecycle_{LifecycleState::Configuring}; // 全局生命周期状态

    // 唤醒掩码（核心调度：64位位掩码，最多支持64个计算系统）
    std::vector<std::uint64_t> fixed_rate_affects_;    // 定频系统 -> 下游计算系统掩码
    std::vector<std::uint64_t> compute_affects_;       // 计算系统 -> 下游计算系统掩码
    std::atomic<std::uint64_t> ready_systems_{0};      // 当前已就绪、待执行的计算系统掩码

    // 全局统计
    std::atomic<std::uint64_t> compute_cycles_{0};
    std::atomic<std::uint64_t> compute_total_time_ns_{0};
    std::atomic<std::uint64_t> compute_last_time_ns_{0};
    std::vector<FixedRateStats> fixed_rate_stats_;
    std::vector<ComputeStats> compute_stats_;
    TimePoint stats_start_time_;

    // 热更新：计算循环暂停控制
    std::atomic<bool> pause_requested_{false};
    std::atomic<bool> paused_{false};
    std::condition_variable pause_cv_;
    std::mutex pause_mutex_;

    // 关机回调钩子列表
    std::vector<std::function<void()>> shutdown_hooks_;
};

// ============================================================================
// 模板函数实现：add_system 模板方法
// 放在头文件中（模板必须头文件实现）
// ============================================================================
template <typename Policy, typename F>
auto Scheduler:: add_system(std::string&& name, F&& func) -> void {
    // 校验：运行中禁止新增系统
    ensure_not_running();
    // 标记拓扑失效，下次 build 必须重新生成
    mark_topology_dirty();

    // 模板工厂：根据策略+函数生成对应系统实例
    /*前置背景：万能引用 F&& func
    函数形参是 F&& func，这是转发引用（万能引用），不是单纯右值引用：
    如果你传临时右值（匿名 lambda、临时对象）：F 推导为裸类型，F&& = 右值引用
    如果你传普通左值变量：F 推导为 Type&，折叠后 F&& = 左值引用
    单纯 func 本身永远是左值，哪怕原本传入的是临时对象。
    如果直接传 func，会强制走拷贝，丢失移动语义。
    2. std::forward 作用：还原原值的左右值属性
    std::forward<F>(func) 的唯一功能：
    根据模板参数 F，把 func 还原成当初传入时的左值 / 右值：
    当初传入右值 → forward 后变成右值引用，触发移动构造
    当初传入左值 → forward 后仍是左值引用，触发拷贝*/
    auto system = make_system<F, Policy>(std::move(name), std::forward<F>(func));
    /*1. 什么是折叠
    当模板推导出来的类型自带 &，再和函数形参的 && 拼在一起，编译器会合并、化简引用符号，这个合并过程叫引用折叠。
    模板形参：F&& func
    F 是推导出来的类型，分两种情况：左值、右值。
    2. 四条折叠规则（死记这 4 条）
    T& & → T&
    T& && → T&
    T&& & → T&
    T&& && → T&&
    口诀：只要有一个左值引用 &，结果一定是左值引用；只有两个 && 才是右值引用。*/
    // 生成执行策略信息
    auto policy = make_policy_info<Policy>();

    // 构造系统条目
    /*前缀 . + 成员名，显式指定给哪个字段赋值；
    赋值顺序可以和结构体内部定义顺序无关；
    没写的成员会执行值初始化（内置类型零初始化、类默认构造）；
    最后一行末尾允许带逗号 ,，编译器兼容，方便批量增删代码。*/
    SystemEntry entry{
        .system = std::move(system),
        .policy = policy,
        .bound  = false,
    };

    // 编译期分支判断：根据策略类型分到不同系统数组
    if constexpr (is_fixed_rate_policy_v<Policy>) {
        // 定频系统：加入定频列表
        fixed_rate_systems_.push_back(std::move(entry));
        return;
    }
    if constexpr (is_pool_policy_v<Policy> || is_visualization_policy_v<Policy>) {
        // 计算/可视化系统：加入计算列表
        compute_systems_.push_back(std::move(entry));
        return;
    }
    // 未知策略：走到这里直接终止（编译期/运行期非法分支）
    std::unreachable();
}

} // namespace talos::scheduler

// ============================================================================
// 全局别名：简化外部使用，无需逐层写命名空间
// ============================================================================
namespace talos {
using namespace talos::scheduler::system;
using talos::scheduler::BuildError;
using talos::scheduler::ChannelKindConflict;
using talos::scheduler::DependencyCycleError;
using talos::scheduler::MultipleReadersError;
using talos::scheduler::MultipleWritersError;
using talos::scheduler::OrphanedReaderError;
using talos::scheduler::Scheduler;
using talos::scheduler::SchedulerError;
using talos::scheduler::TooManyComputeSystemsError;
using talos::scheduler::UnreachableComputeSystemsError;
using talos::scheduler::World;
} // namespace talos