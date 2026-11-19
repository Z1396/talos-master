#pragma once

/**
 * @file
 * @brief 单生产者多消费者「单槽版本化」缓冲区（SPMC latest-value buffer）
 *
 * ⚠️ 命名说明：类名沿用了历史名称 SpmcTripleBuffer，但本实现**不是三槽轮换的三缓冲**，
 * 也**不是无锁结构**。实际模型是：
 *   一个由读写自旋锁保护的单槽 shared_ptr + 单调递增的版本号 generation。
 *
 * ## 设计核心
 *
 * - **读写自旋锁 RWSpinLock**：读线程之间互不阻塞；写线程独占，需等待在途读者全部退出
 * - 读逻辑：快速路径无锁看一眼版本号（仅作提示），版本更新才进入读锁拷贝快照
 * - 写逻辑：获取独占写锁 → 替换 shared_ptr → 版本号自增 → 解锁
 * - 数据语义：latest-value。读者只能拿到“最新一帧”，写入之间不排队，中间的版本会被丢弃
 *
 * ## 性能特征（数量级，随平台/负载变化）
 *
 * - 读：无新版本时仅一次原子读即返回；有新数据时走临界区 + 引用计数 + 一次数据拷贝
 * - 写：一次堆分配（make_shared）+ 等待在途读者退出 + 若干原子操作
 * - 读线程之间互不阻塞；但写线程会短暂阻塞读者，读者也会短暂阻塞写线程
 *
 * ## 与真正三缓冲的区别
 *
 * 真正三缓冲用 3 个独立槽位轮换，写者写自己的槽、读者读自己的槽，写不等待读者；
 * 但经典三缓冲只对 SPSC 安全——SPMC 下多个读者可能同时停在同一个槽，写者轮换后
 * 会覆写正在被读的槽。本实现用“单槽 + 锁 + shared_ptr 快照”换取 SPMC 正确性，
 * 代价是写会阻塞、每次写都要分配。
 *
 * ## 线程安全
 *
 * - Write 句柄：仅限单个生产者线程使用
 * - Read 句柄：**句柄本身不是线程安全的**（内部有 last_gen_ 状态），每个线程必须
 *   持有自己的句柄（用 clone() 分发）；多个独立句柄之间可并发读取、互不阻塞
 */

// 自旋提示指令 SPIN_HINT()
#include "spin.hpp"
// 原子变量
#include <atomic>
// assert：锁内快照不变量检查
#include <cassert>
// std::size_t
#include <cstddef>
// 固定宽度整数
#include <cstdint>
// 智能指针
#include <memory>
// 内存对齐、缓存行隔离
#include <new>
// 可选值容器，区分有无新数据
#include <optional>
// 移动/拷贝语义、pair
#include <utility>

namespace talos::primitive {

namespace detail {

// 缓存行字节数：优先使用标准常量，老平台/老编译器回退到主流值 64
#if defined(__cpp_lib_hardware_interference_size)
inline constexpr std::size_t cache_line_bytes = std::hardware_destructive_interference_size;
#else
inline constexpr std::size_t cache_line_bytes = 64;
#endif

} // namespace detail

/**
 * @brief 单写者、多读者「单槽版本化」缓冲区
 *
 * ⚠️ 类名为历史遗留：本类不是三槽轮换的三缓冲，也不是无锁结构，而是一个
 * 被读写自旋锁保护的单槽 shared_ptr + 版本号。语义是 latest-value：
 * 读者拿到的永远是最新一帧，写入之间不排队。
 *
 * 每个读者句柄独立维护版本标记，同一个版本只会被读取一次（read_current 除外）。
 *
 * ## 内存模型
 *
 * 内部存储 `shared_ptr<const T>`，生产者与消费者共享底层数据；read() 返回的是
 * 值拷贝，read_shared() 才是不拷贝的共享视图。
 *
 * ## 使用示例
 * auto [writer, reader] = SpmcTripleBuffer<Frame>::create();
 *
 * // 生产者线程：写入新帧
 * writer.write(Frame{...});
 *
 * // 消费者线程（每个消费者持有独立Read句柄）
 * auto reader2 = reader.clone();
 * if (auto frame = reader2.read()) {
 *     // 处理帧数据
 * }
 * ## 线程约束
 * - 仅允许单个写线程；任意数量读线程，但每个线程必须持有独立的 Read 句柄
 * - 写操作会阻塞，等待在途读线程退出读锁后才能真正写入
 * - 读线程之间完全无阻塞并发
 *
 * ## 性能复杂度
 *
 * - 写：O(1) 原子操作 + 一次 make_shared 堆分配
 * - 读：O(1) 原子操作 + （read）一次数据拷贝
 * - 内存占用：1 个 shared_ptr + 1 个版本号 + 锁状态，均缓存行对齐
 */
template <typename T>
/*std::copyable<T>
类型可拷贝：既能拷贝构造、拷贝赋值，也支持移动（可拷贝必然可移动）。
read() 需要返回 T 的拷贝，因此这里要求 copyable；只可移动的类型请改用 read_shared() 思路扩展。*/
requires std::copyable<T> class SpmcTripleBuffer {
    /**
     * @brief 读写分离自旋锁，优化多读者场景
     *
     * ## 状态编码规则（32位原子uint32_t）
     *
     * - Bit31：写者等待/持有锁标记 WRITER_BIT = 0x80000000
     * - Bit0~30：当前活跃读者计数 READER_MASK = 0x7FFFFFFF
     *
     * ## 算法逻辑
     *
     * - 读线程：原子递增读者计数，获取共享读锁
     * - 写线程：先置位写标记，自旋等待读者计数归零后获取独占写权限
     */
    //0x80000000u == ~0x7FFFFFFFu
    class RWSpinLock {
        // 最高位：写者占用标记
        static constexpr std::uint32_t WRITER_BIT = 0x80000000u;
        // 低31位：读者计数掩码
        static constexpr std::uint32_t READER_MASK = 0x7FFFFFFFu;
        // 锁状态原子变量
        std::atomic<std::uint32_t> state_{0};

    public:
        /**
         * @brief 获取共享读锁
         *
         * 自旋循环直到写标记清除，再原子递增读者计数。
         *
         * ## 线程安全
         *
         * 多读者并发调用安全，互不阻塞。
         */
        void read_lock() noexcept {
            while (true) {
                auto s = state_.load(std::memory_order_relaxed);
                // 如果存在写者持有锁，自旋等待
                if (s & WRITER_BIT) {
                    SPIN_HINT();
                    continue;
                }
                // CAS尝试读者计数+1
                if (state_.compare_exchange_weak(
                        s, s + 1, std::memory_order_acquire, std::memory_order_relaxed)) {
                    return;
                }
            }
        }

        /**
         * @brief 释放共享读锁，读者计数-1
         * release序，保证读操作数据对写线程可见
         */
        void read_unlock() noexcept { state_.fetch_sub(1, std::memory_order_release); }

        /**
         * @brief 获取独占写锁
         * 1. 先自旋抢占WRITER_BIT标记，阻止新读者进入
         * 2. 持续自旋等待所有现有读者释放锁（读者计数=0）
         */
        void write_lock() noexcept {
            // 第一步：抢占写标记，阻止新增读者
            while (true) {
                auto s = state_.load(std::memory_order_relaxed);
                // 已有其他写者持有标记，自旋等待
                if (s & WRITER_BIT) {
                    SPIN_HINT();
                    continue;
                }
                // CAS置位写者bit
                if (state_.compare_exchange_weak(
                        s, s | WRITER_BIT, std::memory_order_acquire, std::memory_order_relaxed)) {
                    break;
                }
            }
            // 第二步：等待所有活跃读者全部释放锁
            while ((state_.load(std::memory_order_acquire) & READER_MASK) != 0) {
                SPIN_HINT();
            }
        }

        /**
         * @brief 释放独占写锁，清除WRITER_BIT标记
         */
        void write_unlock() noexcept { state_.fetch_and(~WRITER_BIT, std::memory_order_release); }
    };

    /**
     * @brief 缓冲区全局共享状态结构体
     * 缓存行对齐隔离，避免伪共享
     */
    struct State {
        // 读写锁，缓存行对齐，防止与 generation 产生伪共享
        // alignas(常量表达式)：强制变量按指定字节数对齐，这里让 lock 独占一条缓存行
        alignas(detail::cache_line_bytes) RWSpinLock lock{};
        // 当前对外暴露的最新数据智能指针，多线程共享
        std::shared_ptr<const T> current{nullptr};
        // 版本号，每次写入自增，用于读者判断是否有新数据
        alignas(detail::cache_line_bytes) std::atomic<std::uint64_t> generation{0};
    };

public:
    // 前置声明读写句柄模板
    template <typename U>
    class Write;
    template <typename U>
    class Read;

    /**
     * @brief 生产者专属写句柄
     *
     * 全局仅允许单个写句柄，由生产者线程持有，提供线程安全写入接口。
     */
    template <typename U>
    class Write {
    public:
        // 禁用拷贝，仅允许移动
        Write(const Write&)                = delete;
        Write& operator=(const Write&)     = delete;
        Write(Write&&) noexcept            = default;
        Write& operator=(Write&&) noexcept = default;

        /**
         * @brief 写入新数据，发布给所有读者
         *
         * 原子更新全局current数据与版本号generation。
         *
         * ## 性能说明
         *
         * 每次写入都会 make_shared 做一次堆分配；随后短暂自旋等待在途读者退出读锁。
         * 函数标记为 noexcept，因此分配失败（bad_alloc）或 T 的移动/拷贝抛异常时会
         * 直接 std::terminate。
         *
         * ## 线程约束
         *
         * 仅可在唯一生产者线程调用，多写者未做同步，会产生数据竞争。
         *
         * @param data 待写入的数据，移动语义减少拷贝
         */
        void write(U data) noexcept {
            // 构造共享只读数据，生产者移交所有权
            auto ptr = std::make_shared<const U>(std::move(data));

            // 获取独占写锁，阻塞等待读者全部退出
            state_->lock.write_lock();
            // 更新全局最新数据暴露该类型的智能指针，多线程共享
            state_->current = std::move(ptr);
            // 版本号自增，release序保证读者能看到新数据
            state_->generation.fetch_add(1, std::memory_order_release);
            // 释放写锁，允许读者重新进入
            state_->lock.write_unlock();
        }

        /**
         * @brief 获取当前全局最新版本号
         */
        [[nodiscard]] std::uint64_t generation() const noexcept {
            return state_->generation.load(std::memory_order_acquire);
        }

        /**
         * @brief 判断句柄是否有效（绑定共享状态）
         */
        [[nodiscard]] bool valid() const noexcept { return state_ != nullptr; }
        explicit operator bool() const noexcept { return valid(); }

    private:
        friend class SpmcTripleBuffer;
        // 构造：绑定全局共享状态
        explicit Write(std::shared_ptr<State> s)
            : state_(std::move(s)) {}

        // 共享缓冲区全局状态
        std::shared_ptr<State> state_;
    };

    /**
     * @brief 消费者读句柄
     *
     * 支持拷贝、移动；每个读句柄独立维护last_gen_记录上一次读取的版本，过滤重复旧数据。
     *
     * ## 线程安全
     *
     * **单个句柄不是线程安全的**（last_gen_ 是普通成员），每个线程必须持有自己的句柄；
     * 用 clone() 给新消费者分发独立句柄。多个独立句柄之间并发读取互不阻塞。
     */
    template <typename U>
    class Read {
    public:
        // 拷贝构造：复制共享状态与本地已读版本
        Read(const Read& other)
            : state_(other.state_)
            , last_gen_(other.last_gen_) {}

        Read& operator=(const Read& other) {
            if (this != &other) {
                state_    = other.state_;
                last_gen_ = other.last_gen_;
            }
            return *this;
        }

        Read(Read&&) noexcept            = default;
        Read& operator=(Read&&) noexcept = default;

        /**
         * @brief 克隆当前读句柄
         * 生成新读者，继承当前已读版本标记，不会重复读取历史数据
         * @return 新独立Read句柄
         */
        [[nodiscard]] Read clone() const { return *this; }

        /**
         * @brief 读取新数据（仅返回未读过的新版本），返回值拷贝
         *
         * 算法流程：
         * 1. 快速路径：无锁读取全局版本号，若未更新直接返回空（仅作提示，不作为保证）
         * 2. 慢速路径：获取共享读锁，拷贝 shared_ptr 快照与版本号
         * 3. 解锁后更新本地 last_gen_，返回值拷贝
         *
         * 注意：`current` 与 `generation` 在写锁临界区内一起更新，读锁内读到的二者
         * 一定是同一版本的一致快照；且 generation 单调递增，因此不存在“快照比本地已读
         * 版本更旧”的情况（以 assert 表达该不变量，而不是运行时跳过，避免无谓丢帧）。
         * 大对象（图像/点云）请改用 read_shared() 避免拷贝。
         *
         * @return std::optional<U> 有新数据返回有效值，无新数据返回std::nullopt
         */
        [[nodiscard]] std::optional<U> read() noexcept {
            // 快速路径：无锁判断是否存在新版本，减少锁争抢
            if (const std::uint64_t gen = state_->generation.load(std::memory_order_acquire);
                gen <= last_gen_) {
                return std::nullopt;
            }

            // 获取共享读锁，其他读者可同时进入
            state_->lock.read_lock();

            // 拷贝当前全局数据智能指针（共享计数+1，无深拷贝）
            std::shared_ptr<const U> ptr   = state_->current;
            const std::uint64_t locked_gen = state_->generation.load(std::memory_order_acquire);

            state_->lock.read_unlock();

            // 不变量：快速路径已确保 gen_fast > last_gen_；generation 单调递增，且对同一
            // 原子对象后读到的值不会早于先读到的值，因此 locked_gen >= gen_fast > last_gen_。
            // 这两个操作数都是已确定的拷贝，与是否已释放锁无关。
            assert(locked_gen > last_gen_);

            // 缓冲区无任何数据
            if (!ptr) {
                return std::nullopt;
            }

            // 更新本地已读版本，返回数据拷贝
            last_gen_ = locked_gen;
            return *ptr;
        }

        /**
         * @brief 读取新数据（仅返回未读过的新版本），不拷贝数据本体
         *
         * 与 read() 的消费语义完全一致，只是返回 shared_ptr 快照而不是值拷贝，
         * 适合图像帧、点云等大对象。返回的指针在调用方持有期间保证数据不被回收或修改。
         *
         * @return 有新数据返回共享只读指针，无新数据返回 std::nullopt
         */
        [[nodiscard]] std::optional<std::shared_ptr<const U>> read_shared() noexcept {
            if (const std::uint64_t gen = state_->generation.load(std::memory_order_acquire);
                gen <= last_gen_) {
                return std::nullopt;
            }

            state_->lock.read_lock();

            std::shared_ptr<const U> ptr   = state_->current;
            const std::uint64_t locked_gen = state_->generation.load(std::memory_order_acquire);

            state_->lock.read_unlock();

            // 与 read() 相同的不变量检查（见上文说明）
            assert(locked_gen > last_gen_);

            if (!ptr) {
                return std::nullopt;
            }

            last_gen_ = locked_gen;
            return ptr;
        }

        /**
         * @brief 强制读取当前最新数据，无论是否已经读取过
         * 会同步更新本地last_gen_标记，下一次read()会跳过该版本
         */
        [[nodiscard]] std::optional<U> read_current() noexcept {
            state_->lock.read_lock();

            std::shared_ptr<const U> ptr = state_->current;
            std::uint64_t locked_gen     = state_->generation.load(std::memory_order_acquire);

            state_->lock.read_unlock();

            if (!ptr) {
                return std::nullopt;
            }

            last_gen_ = locked_gen;
            return *ptr;
        }

        /**
         * @brief 判断是否存在未读取的新版本数据
         *
         * 只是一次无锁探测（hint）：返回值可能在下一纳秒就因为生产者写入而失效，
         * 因此不要用它做“读之前一定有数据”的保证，直接调 read() 即可。
         */
        [[nodiscard]] bool has_new() const noexcept {
            return state_->generation.load(std::memory_order_acquire) > last_gen_;
        }

        /**
         * @brief 获取本读者上一次成功读取的版本号
         */
        [[nodiscard]] std::uint64_t last_generation() const noexcept { return last_gen_; }

        /**
         * @brief 判断句柄是否有效绑定缓冲区
         */
        [[nodiscard]] bool valid() const noexcept { return state_ != nullptr; }
        explicit operator bool() const noexcept { return valid(); }

    private:
        friend class SpmcTripleBuffer;
        explicit Read(std::shared_ptr<State> s)
            : state_(std::move(s)) {}

        // 共享全局缓冲区状态
        std::shared_ptr<State> state_;
        // 本地记录上一次读取的版本号，用于过滤重复数据
        std::uint64_t last_gen_{0};
    };

    // 禁用默认构造、拷贝
    SpmcTripleBuffer()                                   = delete;
    SpmcTripleBuffer(const SpmcTripleBuffer&)            = delete;
    SpmcTripleBuffer& operator=(const SpmcTripleBuffer&) = delete;

    /**
     * @brief 静态工厂函数，创建一对写句柄+读句柄
     * @return pair<Write<T>, Read<T>> 生产者、消费者初始句柄
     */
    [[nodiscard]] static std::pair<Write<T>, Read<T>> create() {
        auto state = std::make_shared<State>();
        return {Write<T>(state), Read<T>(state)};
    }
};

} // namespace talos::primitive