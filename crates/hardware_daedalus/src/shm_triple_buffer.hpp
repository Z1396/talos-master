// ============================================================================
// 头文件保护：防止同一个头文件被多次包含
// 等效于传统的 #ifndef ... #define ... #endif
// pragma once 是更简洁的方式，大部分现代编译器都支持
// ============================================================================
#pragma once

// 包含共享内存布局定义：ImageTripleBuffer, PoseTripleBuffer, GimbalTripleBuffer 等结构体
#include "shm_layout.hpp"
// C++11 原子库：用于无锁操作的 std::atomic
#include <atomic>
// C++17 可选值：用于表示可能不存在的返回值 std::optional
#include <optional>

// ipc 命名空间：将所有进程间通信相关代码隔离
namespace ipc {

/**
 * @brief TripleBuffer 原子操作模板
 *
 * 这是一个无锁 SPSC (Single Producer Single Consumer) 通信原语。
 * 基于 talos-cpp 现有的 triple_buffer.hpp 设计，扩展用于跨进程通信。
 *
 * 状态编码 (uint8_t):
 *   Bit 7: FLAG_NEW - 有新数据
 *   Bits 0-1: ready_index - 就绪槽位索引 (0-2)
 *
 * @tparam BufferType 三缓冲结构体类型（如 ImageTripleBuffer）
 * @tparam SlotType   槽位数据类型（如 ImageMeta）
 */
template <typename BufferType, typename SlotType>
class TripleBufferOps {
public:
    /**
     * @brief 构造函数
     * @param buf 指向三缓冲结构体的指针（通常位于共享内存中）
     * 
     * 注意：buf 指向的内存必须是跨进程共享的（通过 mmap 映射）
     */
    explicit TripleBufferOps(BufferType* buf)
        : buf_(buf) {}  // 成员初始化列表：保存缓冲区指针

    // ============ 生产者 API ============

    /**
     * @brief 获取可写槽位的可变引用
     * @return 当前写入槽位的引用
     * 
     * 生产者私有方法：直接返回 write_idx 指向的槽位
     * 无需原子操作，因为 write_idx 只有生产者会修改
     * [[nodiscard]] 属性：如果返回值被忽略，编译器会发出警告
     */
    [[nodiscard]] SlotType& borrow_mut() noexcept {
        // 返回 slots[write_idx] 的可变引用
        // 生产者可以修改这个槽位的内容
        // noexcept 保证不会抛出异常（性能优化）
        return buf_->slots[buf_->write_idx];
    }

    /**
     * @brief 发布数据，使消费者可见
     * 
     * 原子地交换 write_idx 和 ready_idx，并设置 FLAG_NEW
     * 
     * 算法流程：
     * 1. 保存当前的 write_idx（即将变成新的就绪槽）
     * 2. 原子交换 state：新 state = write_idx | FLAG_NEW
     * 3. 获取旧的 state，提取旧的 ready_idx
     * 4. 将 write_idx 更新为旧的 ready_idx（回收旧的就绪槽用于写入）
     */
    void publish() noexcept {
        // exchange：原子操作，将 state 设置为 (write_idx | FLAG_NEW)
        // 返回修改前的旧值 old
        // std::memory_order_acq_rel：获取-释放语义
        //   - 对之前所有写操作有释放语义（生产者保证数据已写入）
        //   - 对之后所有读操作有获取语义（消费者能读到最新数据）
        const auto old =
            buf_->state.exchange(buf_->write_idx | FLAG_NEW, std::memory_order_acq_rel);
        
        // 更新 write_idx 为旧的 ready_idx（从 old 中提取）
        // 这样生产者下次写入时使用这个槽位
        // old & INDEX_MASK：取出低2位（就绪槽索引）
        buf_->write_idx = old & INDEX_MASK;
        
        // 注意：write_idx 更新在 state 交换之后
        // 即使此时消费者正在读取，也不会冲突
        // 因为消费者使用独立的 read_idx
    }

    // ============ 消费者 API ============

    /**
     * @brief 尝试获取最新数据
     * @return 如果有新数据，返回指向数据的指针；否则返回 nullopt
     * 
     * 使用 CAS 操作尝试获取就绪槽位，如果生产者同时在写入可能失败
     * 
     * 算法流程：
     * 1. 读取 state，检查 FLAG_NEW 标志
     * 2. 如果没有新数据，返回 nullopt
     * 3. 尝试 CAS：将 state 从 current 改为 read_idx（清除 FLAG_NEW）
     * 4. 如果 CAS 成功，获取就绪槽位
     * 5. 更新 read_idx 为旧的 ready_idx
     * 6. 如果 CAS 失败，说明生产者同时发布了新数据，重试一次
     */
    [[nodiscard]] std::optional<const SlotType*> borrow() noexcept {
        // 1. 原子读取 state（获取语义：保证看到生产者之前的所有写入）
        auto expected = buf_->state.load(std::memory_order_acquire);

        // 2. 检查是否有新数据
        // !(expected & FLAG_NEW)：如果第7位为0，说明没有新数据
        if (!(expected & FLAG_NEW)) {
            return std::nullopt;  // 返回空值（无数据）
        }

        // 3. 提取就绪槽索引（低2位）
        auto ready_idx = expected & INDEX_MASK;

        // 4. 尝试 CAS（Compare-And-Swap）：原子地更新 state
        // 期望值：expected（当前的 state）
        // 目标值：desired = read_idx（清除 FLAG_NEW，保留读索引）
        // 
        // compare_exchange_strong 伪代码：
        // if (state == expected) {
        //     state = desired;
        //     return true;
        // } else {
        //     expected = state;  // 更新 expected 为当前值
        //     return false;
        // }
        if (auto desired = buf_->read_idx; !buf_->state.compare_exchange_strong(
                expected, desired, 
                std::memory_order_acq_rel,  // 成功时的内存序：获取-释放
                std::memory_order_acquire)) {  // 失败时的内存序：获取
            // CAS 失败：生产者可能在同时发布数据
            
            // 重新检查：是否还有新数据？
            // expected 已被 compare_exchange_strong 更新为当前 state
            if (!(expected & FLAG_NEW)) {
                return std::nullopt;  // 数据已被其他消费者取走
            }
            
            // 还有新数据：提取新的就绪索引
            ready_idx = expected & INDEX_MASK;
            desired   = buf_->read_idx;  // 目标值不变
            
            // 第二次 CAS 尝试
            if (!buf_->state.compare_exchange_strong(
                    expected, desired,
                    std::memory_order_acq_rel,
                    std::memory_order_relaxed)) {  // 失败时用 relaxed（性能优化）
                // 第二次尝试也失败：生产者连续发布了两次新数据
                // 我们放弃这次读取，让消费者下次再读最新的
                return std::nullopt;
            }
        }

        // 5. CAS 成功：更新消费者的 read_idx 为旧的 ready_idx
        // 这样下次读取时从这个槽开始
        buf_->read_idx = ready_idx;

        // 6. 返回就绪槽位的常量指针
        // 消费者只能读取，不能修改（const 保证）
        return &buf_->slots[ready_idx];
    }

    /**
     * @brief 获取当前读取槽位 (不检查是否有新数据)
     * @return 当前读取槽位的常量引用
     * 
     * 直接返回 read_idx 指向的槽位
     * 适合在已经确认有新数据后使用（避免重复检查）
     */
    [[nodiscard]] const SlotType& current() const noexcept {
        return buf_->slots[buf_->read_idx];
    }

    /**
     * @brief 检查是否有新数据可读
     * @return true 如果有新数据，false 否则
     * 
     * 非侵入式检查：不修改任何状态
     * 消费者可以先用此函数检查，再决定是否调用 borrow()
     */
    [[nodiscard]] bool has_new_data() const noexcept {
        // 原子读取 state，检查 FLAG_NEW 标志位
        // 获取语义：保证看到生产者之前的所有写入
        return buf_->state.load(std::memory_order_acquire) & FLAG_NEW;
    }

private:
    BufferType* buf_;  // 指向三缓冲结构体的指针（共享内存地址）
};

// ============ 便捷类型别名 ============
// 为具体的三缓冲类型实例化模板，方便使用

// 图像三缓冲操作器：ImageTripleBuffer → ImageMeta
using ImageOps = TripleBufferOps<ImageTripleBuffer, ImageMeta>;

// 位姿三缓冲操作器：PoseTripleBuffer → PoseMeta
using PoseOps = TripleBufferOps<PoseTripleBuffer, PoseMeta>;

// 云台指令三缓冲操作器：GimbalTripleBuffer → GimbalCmd
using GimbalOps = TripleBufferOps<GimbalTripleBuffer, GimbalCmd>;

} // namespace ipc