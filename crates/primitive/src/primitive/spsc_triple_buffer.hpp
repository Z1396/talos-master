#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <new>
#include <optional>

/**
 * @file
 * @brief 无锁单生产者单消费者 三缓冲
 *
 * 本模块提供双线程流水线专用低延迟无锁SPSC（单生产者、单消费者）三缓冲实现
 *
 * ## 设计原理
 *
 * - **三槽架构**：写入槽write_slot、读取槽read_slot、中间交换槽intermediate_slot
 * - 状态机：使用位标记协调生产者、消费者数据交换握手
 * - 移动语义：数据全部使用move转移所有权，零拷贝传输
 *
 * ## 内存序规则
 *
 * 发布操作使用 `memory_order_acq_rel`，保证写入的数据对消费者可见性先于标志位。
 *
 * ## 线程安全约束
 *
 * - **严格单写、单读双线程模型**，多写/多读会产生数据竞争UB
 * - 全程无互斥锁，纯原子操作实现无锁通信
 * - 严格遵循设计使用方式时无数据竞争
 */
namespace talos::primitive {

/**
 * @brief 无锁单生产者单消费者三缓冲容器
 *
 * 双线程专用通信通道，极致低延迟；三槽+原子状态机完成数据交接。
 *
 * ## 使用示例
 *
 * ```cpp
 * auto [writer, reader] = SpscTripleBuffer<Frame>::create();
 *
 * // 生产者线程：写入数据
 * writer.write(Frame{...});
 *
 * // 消费者线程：读取新数据
 * if (auto frame = reader.read()) {
 *     // 处理帧数据
 * }
 * ```
 *
 * ## 性能指标
 *
 * - **写入**：O(1) 原子操作，耗时约 20~30ns
 * - **读取**：O(1) 原子操作，耗时约 20~30ns
 * - **零拷贝**：全程移动语义，无深拷贝原始数据
 */
template <typename T>
requires(std::movable<T>) class SpscTripleBuffer {
    // 状态标记位：最高位标记存在新数据
    static constexpr uint8_t FLAG_NEW = 0x80;
    // 下标掩码：低2位存储槽位索引 0/1/2
    static constexpr uint8_t INDEX_MASK = 0x03;

    /**
     * @brief 缓冲区全局共享状态
     * 全部成员使用缓存行对齐隔离，消除多核伪共享（false sharing）
     */
    struct State {
        // 三槽数据存储，0/1/2 三个存储位
        alignas(std::hardware_destructive_interference_size) std::array<T, 3> slots{};
        // 生产者当前写入槽下标，仅写线程访问，无需原子
        alignas(std::hardware_destructive_interference_size) uint8_t write_idx{0};
        // 共享原子状态：高1位FLAG_NEW，低2位交换槽下标
        alignas(std::hardware_destructive_interference_size) std::atomic<uint8_t> shared{1};
        // 消费者当前读取槽下标，仅读线程访问，无需原子
        alignas(std::hardware_destructive_interference_size) uint8_t read_idx{2};
    };

public:
    // 前置声明读写句柄模板
    template <typename U>
    class Write;
    template <typename U>
    class Read;

    /**
     * @brief 生产者写入句柄，仅移动语义，禁止拷贝
     * 由唯一生产者线程持有
     */
    template <typename U>
    class Write {
    public:
        // 禁用拷贝构造、拷贝赋值
        Write(const Write&)            = delete;
        Write& operator=(const Write&) = delete;
        // 允许移动构造、移动赋值
        Write(Write&&) noexcept            = default;
        Write& operator=(Write&&) noexcept = default;

        /**
         * @brief 移动写入数据到缓冲槽，自动发布对消费者可见
         * @param data 待写入对象，右值移动，不产生拷贝
         *
         * 线程约束：仅生产者线程调用
         */
        void write(U data) noexcept {
            // 将数据移动写入当前生产者槽
            state_->slots[state_->write_idx] = std::move(data);
            // 发布更新，通知消费者有新数据
            publish();
        }

        /**
         * @brief 借用写入槽可变引用，原位构造数据避免额外move
         * @return 当前写入槽T的左值引用
         *
         * 使用流程：borrow_mut()构造对象 → publish()发布
         * 线程约束：仅生产者线程调用
         */
        [[nodiscard]] U& borrow_mut() noexcept { return state_->slots[state_->write_idx]; }

        /**
         * @brief 发布当前写入槽，原子置位新数据标记，通知消费者
         * 原子exchange 使用acq_rel内存序，保证数据先于标志位同步
         *
         * 逻辑：
         * 1. 交换全局shared状态，将当前write_idx+FLAG_NEW写入共享变量
         * 2. 旧状态提取下标，切换为下一轮写入槽
         */
        void publish() noexcept {
            // 原子交换，将当前写入下标+新数据标记写入共享状态
            const auto old =
                state_->shared.exchange(state_->write_idx | FLAG_NEW, std::memory_order_acq_rel);
            // 提取旧状态下标，切换写入槽，实现三槽轮换
            state_->write_idx = old & INDEX_MASK;
        }

        /**
         * @brief 判断句柄是否有效（绑定共享状态）
         */
        [[nodiscard]] bool valid() const noexcept { return state_ != nullptr; }
        // 布尔隐式转换，快速判有效
        explicit operator bool() const noexcept { return valid(); }

    private:
        friend class SpscTripleBuffer;
        // 构造：绑定全局共享状态智能指针
        explicit Write(std::shared_ptr<State> s)
            : state_(std::move(s)) {}
        std::shared_ptr<State> state_;
    };

    /**
     * @brief 消费者读取句柄，仅移动语义，禁止拷贝
     * 唯一消费者线程持有
     */
    template <typename U>
    class Read {
    public:
        // 禁用拷贝，仅允许移动
        Read(const Read&)                = delete;
        Read& operator=(const Read&)     = delete;
        Read(Read&&) noexcept            = default;
        Read& operator=(Read&&) noexcept = default;

        /**
         * @brief 尝试读取新数据，若无新数据返回std::nullopt
         * 完整无锁原子握手流程，数据move移出缓冲槽，所有权转移给调用方
         *
         * 线程约束：仅消费者线程调用
         * 风险说明：移除了has_new()/current()，规避TOCTOU/野引用UAB漏洞
         * @return 有新数据返回std::optional<U>，无新数据返回空
         */
        [[nodiscard]] std::optional<U> read() noexcept {
            // 先acquire加载共享状态，检查是否存在新数据标记
            auto expected = state_->shared.load(std::memory_order_acquire);
            if (!(expected & FLAG_NEW)) {
                return std::nullopt;
            }

            // 提取生产者就绪槽下标
            auto ready_idx = expected & INDEX_MASK;
            // CAS尝试将共享状态替换为当前读槽下标，清除FLAG_NEW
            if (!state_->shared.compare_exchange_strong(
                    expected, state_->read_idx, std::memory_order_acq_rel,
                    std::memory_order_acquire)) {
                // CAS失败，重新检查是否还有新数据
                if (!(expected & FLAG_NEW)) {
                    return std::nullopt;
                }
                ready_idx = expected & INDEX_MASK;
                // 二次CAS抢占共享状态
                if (!state_->shared.compare_exchange_strong(
                        expected, state_->read_idx, std::memory_order_acq_rel,
                        std::memory_order_relaxed)) {
                    return std::nullopt;
                }
            }

            // 更新消费者读取槽下标为生产者就绪槽
            state_->read_idx = ready_idx;
            // 移动取出槽内数据，所有权转移
            return std::move(state_->slots[state_->read_idx]);
        }

        /**
         * @brief 判断句柄是否有效绑定缓冲
         */
        [[nodiscard]] bool valid() const noexcept { return state_ != nullptr; }
        explicit operator bool() const noexcept { return valid(); }

    private:
        friend class SpscTripleBuffer;
        explicit Read(std::shared_ptr<State> s)
            : state_(std::move(s)) {}
        std::shared_ptr<State> state_;
    };

    // 禁用默认构造、拷贝构造、拷贝赋值
    SpscTripleBuffer()                                   = delete;
    SpscTripleBuffer(const SpscTripleBuffer&)            = delete;
    SpscTripleBuffer& operator=(const SpscTripleBuffer&) = delete;

    /**
     * @brief 静态工厂函数，创建一对生产者/消费者句柄
     * @return std::pair<Write<T>, Read<T>> 写句柄、读句柄配对
     */
    [[nodiscard]] static std::pair<Write<T>, Read<T>> create() {
        auto state = std::make_shared<State>();
        return {Write<T>(state), Read<T>(state)};
    }
};

} // namespace talos::primitive