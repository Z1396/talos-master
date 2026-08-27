// ===========================================================================
// spsc_triple_buffer.hpp - 无锁单生产者单消费者三缓冲
// 真实版对应: crates/primitive/src/primitive/spsc_triple_buffer.hpp
//
// 三槽架构：写入槽 / 读取槽 / 中间交换槽，状态机位标记协调握手
// 线程约束：严格单写单读，多写/多读是数据竞争 UB
// ===========================================================================

#pragma once        // 防止头文件被重复包含

#include <array>    // std::array - 固定大小数组（3个槽位）
#include <atomic>   // std::atomic - 原子操作，无锁编程
#include <concepts> // std::movable - C++20 概念约束
#include <cstdint>  // uint8_t - 8位无符号整数
#include <memory>   // std::shared_ptr - 共享所有权
#include <new>      // std::hardware_destructive_interference_size
#include <optional> // std::optional - 可能没有值的返回值
#include <utility>  // std::move, std::pair

namespace talos::primitive {

// ===========================================================================
// SpscTripleBuffer - 单生产者单消费者三缓冲
//
// 模板参数 T：要传输的数据类型，必须可移动（movable）
//
// 核心设计：
//   1. 三个槽位：写入槽、读取槽、交换槽
//   2. 状态机：用原子变量协调槽位交换
//   3. 无锁：使用 CAS（Compare-And-Swap）实现无等待交换
//   4. 单写单读：严格线程约束，违反会导致数据竞争
// ===========================================================================
template <typename T>
requires(std::movable<T>) // T 必须可移动（不能只可拷贝）
class SpscTripleBuffer {
    // =======================================================================
    // 状态标记位
    // =======================================================================

    // FLAG_NEW = 0x80 = 二进制 1000 0000 (最高位)
    // 表示"有新数据可读"
    // 为什么用最高位？因为状态值的低2位存储槽位索引（0,1,2）
    // 最高位和低2位互不干扰
    static constexpr uint8_t FLAG_NEW = 0x80;

    // INDEX_MASK = 0x03 = 二进制 0000 0011 (低2位)
    // 用于提取槽位索引（0,1,2）
    // 因为三缓冲只有3个槽，2位足够表示 0,1,2
    static constexpr uint8_t INDEX_MASK = 0x03;

    // =======================================================================
    // State - 全局共享状态
    //
    // 所有成员都缓存行对齐，防止伪共享（False Sharing）
    // 为什么每个成员都 alignas？
    //   - 不同成员可能被不同线程访问（写线程/读线程）
    //   - 如果在同一缓存行，会导致缓存行"弹跳"（性能下降）
    // =======================================================================
    struct State {
        // ---------- 三槽数据存储 ----------
        // std::array<T, 3>：固定大小数组，3个槽位
        // 槽位索引：0, 1, 2
        // 缓存行对齐：防止 slots 被其他成员干扰
        /*std::hardware_destructive_interference_size：C++17 常量，通常是 64 字节（一个缓存行大小）
        为什么对齐？防止"伪共享"（False Sharing），即不同 CPU 核心访问同一缓存行导致性能下降*/
        alignas(std::hardware_destructive_interference_size) std::array<T, 3> slots{};
        // 槽内存储模板参数 T 类型的数据，调用方通过
        // SpscTripleBuffer<int> 等方式指定具体类型

        // ---------- 生产者写入槽下标 ----------
        // 仅写线程访问，无需原子操作
        // 注意：这不是原子变量！写者独占访问
        // 初始值 0：从槽0开始写
        alignas(std::hardware_destructive_interference_size) uint8_t write_idx{0};

        // ---------- 共享原子状态 ----------
        // 这是整个三缓冲的"心脏"！
        //
        // 编码格式（8位）：
        //   ┌─────────────────────────────────────────────────────┐
        //   │ Bit 7 (最高位)  │ Bit 0 ~ Bit 1 (低2位)           │
        //   ├─────────────────────────────────────────────────────┤
        //   │ FLAG_NEW        │ 交换槽下标 (0, 1, 2)            │
        //   │ 1 = 有新数据    │ 指向"中间槽"                    │
        //   │ 0 = 无新数据    │                                   │
        //   └─────────────────────────────────────────────────────┘
        //
        // 初始值 1：表示交换槽为槽1，且无新数据（FLAG_NEW=0）
        // 为什么初始是1？稍后解释
        alignas(std::hardware_destructive_interference_size) std::atomic<uint8_t> shared{1};

        // ---------- 消费者读取槽下标 ----------
        // 仅读线程访问，无需原子操作
        // 初始值 2：从槽2开始读
        alignas(std::hardware_destructive_interference_size) uint8_t read_idx{2};
    };

public:
    // 前向声明 Write 和 Read 类
    template <typename U>
    class Write;
    template <typename U>
    class Read;

    // =======================================================================
    // Write 句柄 - 生产者（写者）
    //
    // 特性：
    //   - 只能有一个 Write 句柄（单生产者）
    //   - 仅移动语义（不能拷贝）
    //   - 持有 State 的 shared_ptr
    //   - 写者独占访问 write_idx 和 slots[write_idx]
    // =======================================================================
    template <typename U>
    class Write {
    public:
        // ---------- 禁止拷贝 ----------
        Write(const Write&)            = delete;
        Write& operator=(const Write&) = delete;

        // ---------- 允许移动 ----------
        Write(Write&&) noexcept            = default;
        Write& operator=(Write&&) noexcept = default;

        // =============================================================
        // write() - 移动写入数据并自动发布
        //
        // 用法：writer.write(42);
        //
        // 流程：
        //   1. 将数据移动到当前写入槽
        //   2. 调用 publish() 发布
        //
        // 参数：U data - 按值传递，调用者可以 std::move
        // =============================================================
        void write(U data) noexcept {
            // 1. 将数据移动到写入槽
            //    state_->write_idx 是当前写入槽索引（0, 1, 2）
            //    std::move(data) 转移数据所有权，不拷贝
            //    state_->slots[state_->write_idx]：通过指针访问当前写入槽
            state_->slots[state_->write_idx] = std::move(data);

            // 2. 发布（通知消费者有新数据）
            publish();
        }

        // =============================================================
        // borrow_mut() - 借用写入槽的可变引用
        //
        // 用途：原位构造大对象，避免先构造再移动
        //
        // 使用流程：
        //   1. 调用 borrow_mut() 获取槽引用
        //   2. 在槽内直接构造/填充数据
        //   3. 调用 publish() 发布
        //
        // 示例：
        //   FrameData& slot = writer.borrow_mut();
        //   slot.seq = 1;
        //   slot.pixels.assign(1000, 42);
        //   writer.publish();
        //
        // 优点：0 拷贝，直接在目标内存构造
        // =============================================================
        [[nodiscard]] U& borrow_mut() noexcept {
            // 返回当前写入槽的引用
            return state_->slots[state_->write_idx];
        }

        // =============================================================
        // publish() - 发布当前写入槽的数据
        //
        // 这是最核心的操作！它做了两件事：
        //   1. 原子交换：将当前写入槽标记为"有新数据"
        //   2. 更新写入槽：取回旧的交换槽作为下一轮写入槽
        //
        // 状态转换：
        //   写入前：shared = [FLAG_NEW | exchange_idx]
        //   发布后：shared = [FLAG_NEW | write_idx]  (write_idx 变成新交换槽)
        //
        // 内存序：acq_rel（acquire + release）
        //   - release：保证槽内数据对消费者可见
        //   - acquire：保证能看到消费者的读取进度
        // =============================================================
        void publish() noexcept {
            // ---------- 步骤1：原子交换 ----------
            // 将 shared 从旧值交换为 (write_idx | FLAG_NEW)
            /*第1步：计算新值
            state_->write_idx | FLAG_NEW
            假设当前 write_idx = 0，FLAG_NEW = 0x80（128）
            计算：0 | 128 = 128
            新值 = 128

            第2步：原子交换（核心）
            cpp
            exchange(新值, acq_rel)
            这一刻发生的操作：
            读取 shared 的当前值（假设是 1）
            把 shared 改成新值（128）
            返回旧值（1）
            text
            执行前：shared = 1
            执行中：shared = 128（瞬间完成，原子的）
            执行后：old = 1（返回的是旧值）
            交换 = 读旧值 + 写新值，两步合在一起，中间不能被其他线程打断。

            第3步：保存旧值到变量
            cpp
            const auto old = 1
            old 拿到了旧值 1。*/
            const auto old = state_->shared.exchange(
                state_->write_idx | FLAG_NEW, // 新值：写入槽 + 新数据标记
                std::memory_order_acq_rel     // 内存序
            );

            // ---------- 步骤2：更新写入槽 ----------
            // 取回旧的交换槽作为下一轮写入槽
            // old & INDEX_MASK 提取低2位（旧的交换槽索引）
            //
            // 为什么？
            //   三缓冲的核心：三个槽轮流扮演"写入槽"、"交换槽"、"读取槽"
            //   写者发布后，当前写入槽变成新数据槽，旧的交换槽变成新的写入槽
            state_->write_idx = old & INDEX_MASK;
        }

        // ---------- 有效性检查 ----------
        [[nodiscard]] bool valid() const noexcept { return state_ != nullptr; }
        explicit operator bool() const noexcept { return valid(); }

    private:
        // 只有 SpscTripleBuffer 可以创建 Write 句柄
        friend class SpscTripleBuffer;

        // 私有构造函数
        explicit Write(std::shared_ptr<State> s)
            : state_(std::move(s)) {}

        std::shared_ptr<State> state_;
    };

    // =======================================================================
    // Read 句柄 - 消费者（读者）
    //
    // 特性：
    //   - 只能有一个 Read 句柄（单消费者）
    //   - 仅移动语义（不能拷贝）
    //   - 读者独占访问 read_idx 和 slots[read_idx]
    // =======================================================================
    template <typename U>
    class Read {
    public:
        // ---------- 禁止拷贝 ----------
        Read(const Read&)            = delete;
        Read& operator=(const Read&) = delete;

        // ---------- 允许移动 ----------
        Read(Read&&) noexcept            = default;
        Read& operator=(Read&&) noexcept = default;

        // =============================================================
        // read() - 尝试读取新数据
        //
        // 返回值：std::optional<U>
        //   - 有新数据 → 返回数据（移动语义，所有权转移）
        //   - 无新数据 → 返回 nullopt
        //
        // 设计要点：
        //   1. 用 CAS 原子地"消费"新数据
        //   2. 数据被 move 移出槽位（所有权转移）
        //   3. 读取后清除 FLAG_NEW 标记
        //
        // 为什么用 CAS 而不是简单的 exchange？
        //   防止"ABA 问题"：写者可能在读者读取过程中再次发布
        //   但 SPSC 中不会，因为只有一个写者一个读者
        //   CAS 主要用于处理竞态（写者刚好在读者读取时发布）
        // =============================================================
        [[nodiscard]] std::optional<U> read() noexcept {
            // ---------- 第一次尝试 ----------
            // 1. 加载共享状态（acquire 保证看到写者的写入）
            auto expected = state_->shared.load(std::memory_order_acquire);

            // 2. 检查是否有新数据
            if (!(expected & FLAG_NEW)) {
                // 无新数据，直接返回空
                return std::nullopt;
            }

            // 3. 提取生产者就绪槽下标
            //    expected 的低2位是"交换槽"索引
            //    当 FLAG_NEW 置位时，交换槽就是"有新数据"的槽
            auto ready_idx = expected & INDEX_MASK;

            // 4. CAS：将 shared 从 expected 改为 read_idx（清除 FLAG_NEW）
            //    这表示：消费者取走了数据，把当前读槽变成新的交换槽
            //
            //    为什么要改为 read_idx？
            //       - 读者刚读完，读槽里的数据已经被取走（空了）
            //       - 这个空槽可以成为新的交换槽
            //       - 写者下次发布时会取回这个槽作为写入槽
            /*伪代码：
            // compare_exchange_strong( expected【引用】, desired )
            if (shared == expected)
            {
                // ✅相等（成功）：把 desired（这里是read_idx）写入 shared
                shared = desired;
                return true;
            }
            else
            {
                // ❌不相等（失败）：把 shared 的当前值，覆盖给 expected
                expected = shared;
                return false;
            }*/
            if (!state_->shared.compare_exchange_strong(
                    expected,                  // 期望值（引用）
                    state_->read_idx,          // 新值：当前读槽（无 FLAG_NEW）
                    std::memory_order_acq_rel, // 成功时的内存序
                    std::memory_order_acquire  // 失败时的内存序
                    )) 
            {
                // ---------- CAS 失败：处理竞态 ----------
                // CAS 失败原因：写者刚好又发布了一轮数据
                // 此时 expected 已被更新为写者的新值

                // 检查是否还有新数据
                if (!(expected & FLAG_NEW)) {
                    return std::nullopt; // 被其他人读走了（SPSC 中不会发生）
                }

                // 提取新的就绪槽
                ready_idx = expected & INDEX_MASK;

                // 第二次 CAS 尝试（用 relaxed 失败序，因为已 acquire 过）
                if (!state_->shared.compare_exchange_strong(
                        expected, 
                        state_->read_idx, 
                        std::memory_order_acq_rel,
                        std::memory_order_relaxed)) {
                    // 第二次 CAS 也失败，放弃
                    // 这种情况极少发生，返回空
                    return std::nullopt;
                }
            }

            // ---------- 成功取出数据 ----------
            // 更新读槽索引为就绪槽（消费者切换到新数据槽）
            state_->read_idx = ready_idx;

            // 从槽中移动出数据（所有权转移给调用方）
            // std::move 后，槽中的值变为"未定义"状态（但安全）
            // 下次写者写入时会覆盖它
            return std::move(state_->slots[state_->read_idx]);
        }

        // ---------- 有效性检查 ----------
        [[nodiscard]] bool valid() const noexcept { return state_ != nullptr; }
        explicit operator bool() const noexcept { return valid(); }

    private:
        // 只有 SpscTripleBuffer 可以创建 Read 句柄
        friend class SpscTripleBuffer;

        // 私有构造函数
        explicit Read(std::shared_ptr<State> s)
            : state_(std::move(s)) {}

        std::shared_ptr<State> state_;
    };

    // =======================================================================
    // SpscTripleBuffer 类的构造/析构
    // =======================================================================

    // 禁止直接构造
    SpscTripleBuffer()                                   = delete;
    SpscTripleBuffer(const SpscTripleBuffer&)            = delete;  // 禁止拷贝构造
    SpscTripleBuffer& operator=(const SpscTripleBuffer&) = delete;  // 禁止赋值

    // =======================================================================
    // create() - 静态工厂函数
    //
    // 创建一对 Write 和 Read 句柄
    //
    // 用法：
    //   auto [writer, reader] = SpscTripleBuffer<int>::create();
    // =======================================================================
    [[nodiscard]] static std::pair<Write<T>, Read<T>> create() {
        auto state = std::make_shared<State>();
        return {Write<T>(state), Read<T>(state)};
    }
};

} // namespace talos::primitive