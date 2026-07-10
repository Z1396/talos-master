// ===========================================================================
// channel.hpp - 通道与资源组件抽象（Talos components 简化版）
//
// 设计要点：
//   1. spmc<T>：单生产者多消费者通道（简化为直接存值）
//   2. res<T>：只读资源引用
//   3. res_mut<T>：可写资源引用
//   4. SystemParam concept：统一约束系统参数类型
// ===========================================================================
#pragma once

#include "ecs/world.hpp"

#include <concepts>
#include <optional>
#include <type_traits>
#include <utility>

namespace ecs {

// ---------------------------------------------------------------------------
// SPMC 通道：单生产者多消费者（简化版，用 optional 存最新值）
// 真实实现见 stage4 的三缓冲无锁通道
// ---------------------------------------------------------------------------
template <typename T>
class SpmcChannel {
public:
    // 写入：生产者调用
    void write(T value) {
        data_ = std::move(value);
    }

    // 读取：消费者调用，返回 optional
    [[nodiscard]] std::optional<T> read() const {
        if (data_) {
            return *data_;
        }
        return std::nullopt;
    }

private:
    std::optional<T> data_;
};

// ---------------------------------------------------------------------------
// 系统参数标签：标记组件类型（读写语义）
// ---------------------------------------------------------------------------
enum class ParamKind {
    ReadResource,   // res<T>：只读资源
    WriteResource,  // res_mut<T>：可写资源
    ReadChannel,    // spmc<T>：读通道
    WriteChannel,   // spmc_mut<T>：写通道
};

// ---------------------------------------------------------------------------
// 只读资源包装器
// ---------------------------------------------------------------------------
template <typename T>
struct Res {
    using value_type = T;
    static constexpr ParamKind kind = ParamKind::ReadResource;
    T* ptr;

    explicit Res(T* p) : ptr(p) {}
    [[nodiscard]] const T& get() const { return *ptr; }
};

// 可写资源包装器
template <typename T>
struct ResMut {
    using value_type = T;
    static constexpr ParamKind kind = ParamKind::WriteResource;
    T* ptr;

    explicit ResMut(T* p) : ptr(p) {}
    [[nodiscard]] T& get() { return *ptr; }
};

// 读通道包装器
template <typename T>
struct SpmcReader {
    using value_type = T;
    static constexpr ParamKind kind = ParamKind::ReadChannel;
    const SpmcChannel<T>* ptr;

    explicit SpmcReader(const SpmcChannel<T>* p) : ptr(p) {}
    [[nodiscard]] std::optional<T> read() const { return ptr->read(); }
};

// 写通道包装器
template <typename T>
struct SpmcWriter {
    using value_type = T;
    static constexpr ParamKind kind = ParamKind::WriteChannel;
    SpmcChannel<T>* ptr;

    explicit SpmcWriter(SpmcChannel<T>* p) : ptr(p) {}
    void write(T value) { ptr->write(std::move(value)); }
};

// ---------------------------------------------------------------------------
// Concept：统一约束系统参数类型
// ---------------------------------------------------------------------------
template <typename T>
concept SystemParam = requires {
    typename T::value_type;
    requires std::is_enum_v<decltype(T::kind)>;
};

}  // namespace ecs
