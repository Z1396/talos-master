// ===========================================================================
// triple_buffer.hpp - SPMC 三缓冲无锁通道（Talos 简化版）
//
// 核心思想：
//   1. 三个缓冲槽位轮转，写者与读者互不阻塞
//   2. 用原子状态变量记录"最新可读槽位"和"是否有新数据"
//   3. 写者写完发布新槽位，读者交换自己的旧槽位获取新数据
//
// 数据一致性保证：
//   - 任意时刻，写者独占一个槽位写入
//   - 读者独占一个槽位读取
//   - 第三个槽位作为"最新发布"的中转
//   - 读者总能读到完整的最新一份数据（不会读到半写状态）
//
// 内存序选择：
//   - AcqRel：发布时 store+swap 需要 release（写可见性），读取需要 acquire
//   - Relaxed：仅在不需要同步的地方使用（如 index 计算）
// ===========================================================================
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <new>
#include <optional>
#include <type_traits>
#include <utility>

namespace spmc {

// 缓存行大小：64 字节，用于 alignas 防 false sharing
inline constexpr std::size_t CACHELINE = 64;

// ===========================================================================
// TripleBuffer<T>：单生产者多消费者三缓冲
// 限制：单个写者 + 单个读者（SPSC）；多读者需扩展（SPMC 见下文）
// ===========================================================================
template <typename T>
class TripleBuffer {
public:
    TripleBuffer() {
        // 初始化：写者持有槽0，读者持有槽1，槽2为发布中转
        write_idx_ = 0;
        read_idx_ = 1;
        // state_ 高位标记是否有新数据，低位记录最新发布槽位
        state_.store(2 | FLAG_NEW, std::memory_order_relaxed);
    }

    // 写者获取可写引用
    [[nodiscard]] T& write_slot() {
        return slots_[write_idx_];
    }

    // 写者发布：把自己写的槽位标记为"最新"
    void publish() {
        // swap：原子交换 write_idx 与 state 中的 published_idx
        // 设置 FLAG_NEW 标记，表示有新数据可读
        uint8_t old = state_.exchange(
            write_idx_ | FLAG_NEW, std::memory_order_acq_rel);
        write_idx_ = old & INDEX_MASK;  // 取回旧的 published 槽位作为下次写
    }

    // 读者尝试获取最新数据
    // 返回指向最新数据的指针，若无新数据返回 nullptr
    [[nodiscard]] const T* read() {
        uint8_t expected = state_.load(std::memory_order_acquire);
        if ((expected & FLAG_NEW) == 0) {
            return nullptr;  // 无新数据
        }
        // CAS：清除 FLAG_NEW，交换 read_idx 与 published_idx
        uint8_t desired = read_idx_;  // 把自己的槽位放回，取走 published
        if (state_.compare_exchange_strong(
                expected, desired, std::memory_order_acq_rel,
                std::memory_order_acquire)) {
            read_idx_ = expected & INDEX_MASK;
            return &slots_[read_idx_];
        }
        // CAS 失败：写者刚好又发布了，重试一次
        expected = state_.load(std::memory_order_acquire);
        if ((expected & FLAG_NEW) == 0) {
            return nullptr;
        }
        desired = read_idx_;
        if (state_.compare_exchange_strong(
                expected, desired, std::memory_order_acq_rel,
                std::memory_order_relaxed)) {
            read_idx_ = expected & INDEX_MASK;
            return &slots_[read_idx_];
        }
        return nullptr;
    }

    // 检查是否有新数据（不消费）
    [[nodiscard]] bool has_new() const noexcept {
        return (state_.load(std::memory_order_acquire) & FLAG_NEW) != 0;
    }

private:
    static constexpr uint8_t FLAG_NEW = 0x80;  // 高位：是否有新数据
    static constexpr uint8_t INDEX_MASK = 0x03; // 低2位：槽位索引 0/1/2

    // 三个槽位：alignas 防止 false sharing
    alignas(CACHELINE) T slots_[3];
    alignas(CACHELINE) std::atomic<uint8_t> state_;  // 原子状态：FLAG_NEW | published_idx
    alignas(CACHELINE) uint8_t write_idx_;  // 写者当前槽位（仅写者访问）
    alignas(CACHELINE) uint8_t read_idx_;   // 读者当前槽位（仅读者访问）
};

// ===========================================================================
// SPMC 扩展：多读者共享同一份数据（用 shared_ptr 引用计数）
// 写者发布后，所有读者共享同一份只读副本
// ===========================================================================
template <typename T>
class SpmcTripleBuffer {
public:
    SpmcTripleBuffer() = default;

    // 写者发布数据（拷贝到 shared_ptr）
    void publish(T value) {
        auto ptr = std::make_shared<T>(std::move(value));
        // 原子存储，读者通过 load 获取
        ptr_.store(ptr, std::memory_order_release);
    }

    // 读者读取最新数据（共享所有权，零拷贝）
    [[nodiscard]] std::shared_ptr<const T> read() const {
        return ptr_.load(std::memory_order_acquire);
    }

private:
    alignas(CACHELINE) std::atomic<std::shared_ptr<const T>> ptr_;
};

}  // namespace spmc
