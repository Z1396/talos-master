#pragma once

/**
 * @file
 * @brief 无锁单生产者多消费者 三缓冲实现
 *
 * 本模块提供低延迟线程间数据传输的无锁SPMC（单生产者、多消费者）三缓冲结构
 *
 * ## 设计核心
 *
 * - **读写自旋锁RWSpinLock**：读线程之间互不阻塞，仅写线程会阻塞等待全部读线程释放锁
 * - 读逻辑：获取共享读锁（原子读者计数自增）
 * - 写逻辑：获取独占写锁，自旋等待所有读者计数归零
 *
 * ## 性能特征
 *
 * - **写操作**：约50~100ns（等待现有读者释放锁 + 原子交换版本号）
 * - **读操作**：约20~40ns（原子增减计数 + 数据拷贝）
 * - **读线程永远不会互相阻塞**（区别于普通排他自旋锁）
 *
 * ## 线程安全
 *
 * 全部接口线程安全；多个消费者可并发读取，彼此无阻塞。
 */

// 自旋提示指令 SPIN_HINT()
#include "spin.hpp"
// 原子变量
#include <atomic>
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

/**
 * @brief 无锁单生产者多消费者三缓冲容器
 *
 * 实现单写者、多读者线程安全数据传输。每个读者独立维护版本标记，各自持有一份一致数据视图。
 *
 * ## 内存模型
 *
 * 内部存储 `shared_ptr<const T>`，生产者与消费者共享底层数据，实现零拷贝数据传递。
 *
 * ## 使用示例
 *
 * ```cpp
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
 * ```
 *
 * ## 线程约束
 *
 * - 仅允许单个写线程；任意数量读线程
 * - 写操作会阻塞，等待所有读线程释放锁
 * - 读线程之间完全无阻塞并发
 *
 * ## 性能复杂度
 *
 * - 写：O(1) 原子操作
 * - 读：O(1) 原子操作
 * - 内存占用：3个shared_ptr + 原子变量缓存行对齐开销
 */
template <typename T>
/*std::movable<T>
类型可移动：支持移动构造、移动赋值，可以被 std::move 转移所有权；不一定支持拷贝。
std::copyable<T>
类型可拷贝：既能拷贝构造、拷贝赋值，也支持移动（可拷贝必然可移动）*/
requires(std::movable<T> && std::copyable<T>) class SpmcTripleBuffer {
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
        // 读写锁，缓存行对齐，防止与generation产生伪共享
        alignas(std::hardware_destructive_interference_size) RWSpinLock lock{};
        // 当前对外暴露的最新数据智能指针，多线程共享
        std::shared_ptr<const T> current{nullptr};
        // 版本号，每次写入自增，用于读者判断是否有新数据
        alignas(std::hardware_destructive_interference_size) std::atomic<std::uint64_t> generation{
            0};
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
         * 可能短暂自旋等待现有读者释放读锁；写入本体为O(1)原子操作。
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
            // 更新全局最新数据
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
     * 任意线程均可使用；多个读句柄并发读取互不阻塞。
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
         * @brief 读取新数据（仅返回未读过的新版本）
         *
         * 算法流程：
         * 1. 快速路径：无锁读取全局版本号，若未更新直接返回空
         * 2. 慢速路径：获取共享读锁，再次校验版本、拷贝数据
         * 3. 版本更新则更新本地last_gen_，返回数据拷贝
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
            std::shared_ptr<const U> ptr = state_->current;
            std::uint64_t locked_gen     = state_->generation.load(std::memory_order_acquire);

            state_->lock.read_unlock();

            // 缓冲区无任何数据
            if (!ptr) {
                return std::nullopt;
            }

            // 锁内读取后再次校验版本，防止快速路径过时
            if (locked_gen <= last_gen_) {
                return std::nullopt;
            }

            // 更新本地已读版本，返回数据拷贝
            last_gen_ = locked_gen;
            return *ptr;
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