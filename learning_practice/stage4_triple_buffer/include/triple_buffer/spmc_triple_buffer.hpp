// ===========================================================================
// spmc_triple_buffer.hpp - 无锁单生产者多消费者三缓冲
// 真实版对应: crates/primitive/src/primitive/spmc_triple_buffer.hpp
//
// 设计核心：RWSpinLock + generation 版本号 + shared_ptr 数据
//   - 读锁：读者计数自增，多读者并发、互不阻塞
//   - 写锁：置位写标记后自旋等待所有读者退出，独占更新数据
//   - 读者在锁内同时快照 current 与 generation，保证版本号与数据一致
//
// 核心语义：每个读者独立维护 last_gen_，read() 只返回未读过的新版本
//
// 教训记录（为什么必须锁内快照）：
//   曾尝试用 atomic<shared_ptr> 无锁简化，压力测试测出版本号与数据
//   错位竞态（读者拿到新版本号配旧数据，重复返回同一版本）——
//   真实版在读锁内同时快照 current 与 generation 正是为杜绝此问题
// ===========================================================================

#pragma once                     // 防止头文件被重复包含

// 自旋提示指令 SPIN_HINT()：告诉 CPU "我在自旋等待"，优化性能
#include "triple_buffer/spin.hpp"

#include <atomic>    // std::atomic - 原子操作，无锁编程的基础
#include <concepts>  // std::movable, std::copyable - C++20 概念约束
#include <cstdint>   // std::uint32_t, std::uint64_t - 固定宽度整数
#include <memory>    // std::shared_ptr - 共享所有权智能指针
#include <new>       // std::hardware_destructive_interference_size - 缓存行大小
#include <optional>  // std::optional - 表示"可能没有值"的返回值
#include <utility>   // std::pair, std::move - 工具函数

namespace talos::primitive {

// ===========================================================================
// SpmcTripleBuffer - 单生产者多消费者三缓冲
// 模板参数 T：要传输的数据类型，必须可移动且可拷贝
// ===========================================================================
template <typename T>
requires(std::movable<T> && std::copyable<T>)  // C++20 概念约束
class SpmcTripleBuffer {
    // =======================================================================
    // RWSpinLock - 读写自旋锁
    // 
    // 设计目标：
    //   - 多个读者可以同时进入（共享锁）
    //   - 写者独占访问（独占锁）
    //   - 无系统调用，纯自旋等待（适合短临界区）
    //   - 用单个原子整数编码状态，高效且一致
    // 
    // 状态编码（32位）：
    //   ┌─────────────────────────────────────────────────────────────┐
    //   │ Bit 31 (最高位)  │ Bit 0 ~ Bit 30 (低31位)                │
    //   ├─────────────────────────────────────────────────────────────┤
    //   │ 写者标记          │ 读者计数                               │
    //   │ 1 = 写者持有锁    │ 0 ~ 2^31-1 个读者正在读               │
    //   │ 0 = 无写者        │                                        │
    //   └─────────────────────────────────────────────────────────────┘
    // 
    // 为什么这样设计？
    //   1. 一个原子变量同时表示"写者状态"和"读者数量"
    //   2. CAS 操作可以一次性修改两个信息，保证一致性
    //   3. 无需两个原子变量，减少缓存行开销
    // =======================================================================
    class RWSpinLock {
        // WRITER_BIT = 0x80000000u = 二进制 1000...000 (第31位为1)
        // 用于检测和设置写者标记
        static constexpr std::uint32_t WRITER_BIT  = 0x80000000u;
        
        // READER_MASK = 0x7FFFFFFFu = 二进制 0111...111 (低31位全为1)
        // 用于提取读者计数（屏蔽写者标记位）
        static constexpr std::uint32_t READER_MASK = 0x7FFFFFFFu;
        
        // 原子状态变量，初始为 0（无写者，0 个读者）
        std::atomic<std::uint32_t> state_{0};

    public:
        // =============================================================
        // read_lock() - 获取共享读锁
        // 
        // 行为：
        //   1. 如果没有写者，读者计数 +1，进入临界区
        //   2. 如果有写者，自旋等待
        // 
        // 内存序：
        //   - acquire：保证此后的读操作能看到写者的写入
        //   - relaxed：读取 state 不需要强同步
        // =============================================================
        void read_lock() noexcept {
            while (true) {
                // 1. 加载当前状态（relaxed 足够，因为 CAS 会处理一致性）
                auto s = state_.load(std::memory_order_relaxed);
                
                // 2. 检查写者标记
                if (s & WRITER_BIT) {
                    // 有写者持有锁，自旋等待
                    // SPIN_HINT() 在 x86 上是 PAUSE 指令
                    // 作用：告诉 CPU 我们在自旋，优化功耗和性能
                    SPIN_HINT();
                    continue;
                }
                
                // 3. 尝试 CAS：将读者计数 +1
                //    compare_exchange_weak 可能失败（伪失败），但循环会重试
                //    第一个 s 是期望值，如果 state_ 还是 s，就改成 s+1
                //    CAS 成功后用 acquire 序，失败用 relaxed（因为要重试）
                if (state_.compare_exchange_weak(
                        s,                    // 期望值（引用，会被更新）
                        s + 1,                // 期望值 + 1（新值）
                        std::memory_order_acquire,   // 成功时的内存序
                        std::memory_order_relaxed)) { // 失败时的内存序
                    return;  // 成功获取读锁
                }
                // CAS 失败 → 重试循环
            }
        }

        // =============================================================
        // read_unlock() - 释放共享读锁
        // 
        // 行为：读者计数 -1
        // 内存序：release，保证此前的读操作对写者可见
        // =============================================================
        void read_unlock() noexcept {
            // fetch_sub 原子地减少读者计数
            // memory_order_release：之前的读操作不能重排到 store 之后
            state_.fetch_sub(1, std::memory_order_release);
        }

        // =============================================================
        // write_lock() - 获取独占写锁
        // 
        // 行为：
        //   1. 抢占写标记（阻止新读者进入）
        //   2. 等待所有现有读者退出（读者计数归零）
        // 
        // 为什么分两步？
        //   - 先设置标记阻止新读者，再等老读者退出
        //   - 防止"饥饿"：如果读者不断进入，写者永远等不到
        // =============================================================
        void write_lock() noexcept {
            // ---------- 第一步：抢占写标记 ----------
            while (true) {
                auto s = state_.load(std::memory_order_relaxed);
                
                // 如果已经有写者，自旋等待
                if (s & WRITER_BIT) {
                    SPIN_HINT();
                    continue;
                }
                
                // 尝试 CAS：设置写者标记（s | WRITER_BIT）
                // 注意：这里保留了读者计数（s 的低31位不变）
                if (state_.compare_exchange_weak(
                        s,
                        s | WRITER_BIT,  // 设置最高位为 1
                        std::memory_order_acquire,
                        std::memory_order_relaxed)) {
                    break;  // 成功抢占
                }
                // CAS 失败 → 重试
            }

            // ---------- 第二步：等待所有读者退出 ----------
            // 读者计数在低31位，用 READER_MASK 提取
            while ((state_.load(std::memory_order_acquire) & READER_MASK) != 0) {
                SPIN_HINT();  // 还有读者在读，自旋等待
            }
            // 此时：写者标记已设置，读者计数为 0
            // 写者可以独占修改数据
        }

        // =============================================================
        // write_unlock() - 释放独占写锁
        // 
        // 行为：清除写者标记，允许新读者进入
        // 内存序：release，保证写操作对后续读者可见
        // =============================================================
        void write_unlock() noexcept {
            // fetch_and(~WRITER_BIT)：原子地清除最高位
            // 等价于：state_ &= ~WRITER_BIT
            // memory_order_release：之前的写操作不能重排到 store 之后
            state_.fetch_and(~WRITER_BIT, std::memory_order_release);
        }
    };

    // =======================================================================
    // State - 缓冲区全局共享状态
    // 
    // 所有 Write 和 Read 句柄共享同一个 State 对象
    // 通过 shared_ptr 管理生命周期
    // =======================================================================
    struct State {
        // ---------- 读写锁 ----------
        // alignas 强制缓存行对齐，防止伪共享（False Sharing）
        // std::hardware_destructive_interference_size 通常是 64 字节
        // 确保 lock 独占一个缓存行，不被其他变量干扰
        alignas(std::hardware_destructive_interference_size) RWSpinLock lock{};

        // ---------- 当前数据 ----------
        // shared_ptr<const T>：多读者共享同一份数据，自动管理生命周期
        // const T：读者只能读，不能修改（不可变数据）
        // 为什么用 shared_ptr？
        //   - 多个读者可能同时持有引用
        //   - 写者更新后，旧数据自动销毁（引用计数归零时）
        std::shared_ptr<const T> current{nullptr};

        // ---------- 版本号 ----------
        // 每次写入自增，读者据此判断是否有新数据
        // 原子类型：读者和写者可能同时访问
        // 再次缓存行对齐，避免与 lock 在同一缓存行
        alignas(std::hardware_destructive_interference_size) 
            std::atomic<std::uint64_t> generation{0};
    };

public:
    // 前向声明 Write 和 Read 类（在类外部定义）
    template <typename U>
    class Write;
    template <typename U>
    class Read;

    // =======================================================================
    // Write 句柄 - 生产者（写者）
    // 
    // 特性：
    //   - 全局只能有一个 Write 句柄（单生产者）
    //   - 仅移动语义（不能拷贝）
    //   - 持有 State 的 shared_ptr
    // =======================================================================
    template <typename U>
    class Write {
    public:
        // ---------- 禁止拷贝 ----------
        // 写者句柄应该是唯一的，不能拷贝
        Write(const Write&) = delete;
        Write& operator=(const Write&) = delete;

        // ---------- 允许移动 ----------
        // 移动是安全的，转移 State 的所有权
        Write(Write&&) noexcept = default;
        Write& operator=(Write&&) noexcept = default;

        // =============================================================
        // write() - 写入新数据并发布给所有读者
        // 
        // 参数：U data - 要写入的数据（按值传递，调用者可以 move）
        // 
        // 流程：
        //   1. 在堆上创建数据的 shared_ptr（移动语义）
        //   2. 获取独占写锁
        //   3. 更新 current 指向新数据
        //   4. 版本号 +1
        //   5. 释放写锁
        // 
        // 为什么用 shared_ptr？
        //   - 写者更新后，旧数据可能还有读者在用
        //   - shared_ptr 自动管理引用计数，安全释放
        // 
        // noexcept：保证不抛异常（内存分配可能抛，但这里没处理）
        // =============================================================
        void write(U data) noexcept {
            // 1. 在堆上创建数据副本（用移动语义，高效）
            //    注意：这里是 const U，读者只能读
            auto ptr = std::make_shared<const U>(std::move(data));

            // 2. 获取独占写锁（等待所有读者退出）
            state_->lock.write_lock();

            // 3. 更新当前数据（移动 shared_ptr，引用计数转移）
            state_->current = std::move(ptr);

            // 4. 版本号 +1（release 序保证读者看到新版本时数据已就绪）
            //    fetch_add 原子地自增，返回旧值
            state_->generation.fetch_add(1, std::memory_order_release);

            // 5. 释放写锁（允许新读者进入）
            state_->lock.write_unlock();
        }

        // ---------- 有效性检查 ----------
        [[nodiscard]] bool valid() const noexcept { 
            return state_ != nullptr; 
        }
        explicit operator bool() const noexcept { 
            return valid(); 
        }

    private:
        // 只有 SpmcTripleBuffer 可以创建 Write 句柄
        friend class SpmcTripleBuffer;

        // 私有构造函数：从 State 创建
        explicit Write(std::shared_ptr<State> s)
            : state_(std::move(s)) {}

        // 指向共享 State 的智能指针
        std::shared_ptr<State> state_;
    };

    // =======================================================================
    // Read 句柄 - 消费者（读者）
    // 
    // 特性：
    //   - 可以有多个 Read 句柄（多消费者）
    //   - 支持拷贝/克隆（每个读者独立）
    //   - 每个读者独立记录 last_gen_（已读版本号）
    //   - read() 只返回未读过的新数据
    // =======================================================================
    template <typename U>
    class Read {
    public:
        // ---------- 拷贝构造 ----------
        // 拷贝时继承被拷贝者的已读版本号
        // 这样克隆的读者不会重复读取历史数据
        Read(const Read& other)
            : state_(other.state_)      // 共享同一个 State
            , last_gen_(other.last_gen_) // 继承已读版本
        {}

        // ---------- 拷贝赋值 ----------
        Read& operator=(const Read& other) {
            if (this != &other) {
                state_ = other.state_;
                last_gen_ = other.last_gen_;
            }
            return *this;
        }

        // ---------- 移动 ----------
        Read(Read&&) noexcept = default;
        Read& operator=(Read&&) noexcept = default;

        // ---------- 克隆读者 ----------
        // 返回当前读者的拷贝
        // 新读者继承已读版本，不会重复读旧数据
        [[nodiscard]] Read clone() const { 
            return *this; 
        }

        // =============================================================
        // read() - 读取新数据（核心函数！）
        // 
        // 返回值：std::optional<U>
        //   - 有新数据且未读过 → 返回数据（拷贝）
        //   - 无新数据或已读过 → 返回 nullopt
        // 
        // 设计原理：快速路径 + 慢速路径
        //   - 快速路径：无锁检查版本号，无新数据直接返回
        //   - 慢速路径：加读锁，快照 current 和 generation
        // 
        // 为什么分两步？
        //   大多数时候没有新数据，快速路径避免锁开销
        // 
        // 为什么必须在锁内快照 current 和 generation？
        //   防止"版本号-数据错位"竞态：
        //     ❌ 先读 generation，再读 current → 可能读到新版本号 + 旧数据
        //     ✅ 锁内同时读 current 和 generation → 保证一致性
        // =============================================================
        [[nodiscard]] std::optional<U> read() noexcept {
            // ---------- 快速路径：无锁检查 ----------
            // 检查当前版本号是否大于本地已读版本
            // memory_order_acquire：保证能看到写者的最新写入
            const auto gen = state_->generation.load(std::memory_order_acquire);
            if (gen <= last_gen_) {
                // 没有新数据，直接返回空（不加锁，极快！）
                return std::nullopt;
            }

            // ---------- 慢速路径：加锁读取 ----------
            // 注意：从快速路径到慢速路径之间，写者可能已经更新了数据
            // 所以必须在锁内重新检查

            // 1. 获取读锁（如果有写者则等待）
            state_->lock.read_lock();

            // 2. 在锁内快照 current 和 generation
            //    这一步保证了数据一致性：
            //    - 写者被锁阻塞，不会在快照期间修改数据
            //    - current 和 generation 必然匹配
            std::shared_ptr<const U> ptr = state_->current;
            const auto locked_gen = state_->generation.load(std::memory_order_acquire);

            // 3. 释放读锁（其他读者可以进入了）
            state_->lock.read_unlock();

            // ---------- 快照后的校验 ----------
            // 检查1：缓冲区是否为空（从未写入过数据）
            if (!ptr) {
                return std::nullopt;
            }

            // 检查2：快速路径的检查可能已经过时
            // 例如：快速路径读到 gen=5，但锁内读到时写者已经更新到 gen=6
            // 或者：另一个读者已经更新了我们的 last_gen_？（不会，每个读者独立）
            if (locked_gen <= last_gen_) {
                return std::nullopt;  // 数据已经被读过
            }

            // ---------- 更新状态，返回数据 ----------
            // 更新本地已读版本号
            last_gen_ = locked_gen;

            // 返回数据的拷贝（解引用 shared_ptr）
            // 为什么返回拷贝而不是引用？
            //   - 读者可能在不同线程，引用会悬空
            //   - 写者可能更新数据，旧数据会被销毁
            //   - 返回拷贝保证读者持有自己的数据副本
            return *ptr;
        }

        // =============================================================
        // read_current() - 强制读取当前最新数据
        // 
        // 与 read() 的区别：
        //   - read()：只返回未读过的新数据
        //   - read_current()：不管是否读过，强制读最新数据
        // 
        // 适用场景：
        //   - 需要当前最新值（如传感器读数）
        //   - 不在乎是否重复读取
        // =============================================================
        [[nodiscard]] std::optional<U> read_current() noexcept {
            // 加读锁，快照 current 和 generation
            state_->lock.read_lock();
            std::shared_ptr<const U> ptr = state_->current;
            const auto locked_gen = state_->generation.load(std::memory_order_acquire);
            state_->lock.read_unlock();

            if (!ptr) {
                return std::nullopt;
            }

            // 强制更新本地已读版本（即使版本没变）
            last_gen_ = locked_gen;
            return *ptr;
        }

        // =============================================================
        // has_new() - 检查是否有未读取的新数据
        // 
        // 无锁检查，非常快
        // 适用于：先检查再决定是否调用 read()
        // =============================================================
        [[nodiscard]] bool has_new() const noexcept {
            // 比较全局版本号和本地已读版本号
            // memory_order_acquire：确保看到写者的最新写入
            return state_->generation.load(std::memory_order_acquire) > last_gen_;
        }

        // ---------- 获取本地已读版本号 ----------
        [[nodiscard]] std::uint64_t last_generation() const noexcept {
            return last_gen_;
        }

        // ---------- 有效性检查 ----------
        [[nodiscard]] bool valid() const noexcept {
            return state_ != nullptr;
        }
        explicit operator bool() const noexcept {
            return valid();
        }

    private:
        // 只有 SpmcTripleBuffer 可以创建 Read 句柄
        friend class SpmcTripleBuffer;

        // 私有构造函数：从 State 创建
        explicit Read(std::shared_ptr<State> s)
            : state_(std::move(s)) {}

        // 指向共享 State 的智能指针
        std::shared_ptr<State> state_;

        // 本地已读版本号（每个读者独立维护）
        // 初始为 0，意味着版本号 0 的数据也会被读取（如果存在）
        std::uint64_t last_gen_{0};
    };

    // =======================================================================
    // SpmcTripleBuffer 类的构造/析构
    // =======================================================================
    
    // 禁止直接构造（只能通过静态工厂 create()）
    SpmcTripleBuffer() = delete;
    
    // 禁止拷贝
    SpmcTripleBuffer(const SpmcTripleBuffer&) = delete;
    SpmcTripleBuffer& operator=(const SpmcTripleBuffer&) = delete;

    // =======================================================================
    // create() - 静态工厂函数
    // 
    // 创建一对 Write 和 Read 句柄
    // 
    // 用法：
    //   auto [writer, reader] = SpmcTripleBuffer<int>::create();
    // 
    // 返回：std::pair<Write<T>, Read<T>>
    //   - first：写者句柄（唯一）
    //   - second：读者句柄（可以克隆多个）
    // =======================================================================
    [[nodiscard]] static std::pair<Write<T>, Read<T>> create() {
        // 1. 创建共享 State 对象（在堆上）
        auto state = std::make_shared<State>();

        // 2. 返回 Write 和 Read 句柄对，共享同一个 State
        //    Write 和 Read 的构造函数是私有的，但友元可以访问
        return {Write<T>(state), Read<T>(state)};
    }
};

}  // namespace talos::primitive