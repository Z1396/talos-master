#pragma once

/**
 * @file
 * @brief 通道抽象：基于三缓冲实现 SPSC / SPMC 消息通道
 *
 * 本模块封装统一通道类型，底层依赖三缓冲容器，并预留数据流图追踪埋点
 *
 * ## 通道类型说明
 *
 * - `SpscChannel<T>`：单生产者单消费者通道
 * - `SpmcChannel<T>`：单生产者多消费者通道
 * - `Channel<T>`：SpmcChannel<T> 的别名，用于历史代码兼容
 */

#include "spmc_triple_buffer.hpp"   // 底层 SPMC 三缓冲原生实现
#include "spsc_triple_buffer.hpp"   // 底层 SPSC 三缓冲原生实现

#include <concepts>                 // C++20 约束/概念，区分 SPSC / SPMC 能力
#include <optional>                 // 无值读取返回 std::optional
#include <utility>                  // std::move 移动语义

namespace talos::primitive {
// ============================================================================
// 内部概念：区分底层缓冲能力（SPMC支持克隆Reader，SPSC不支持）
// ============================================================================
namespace detail {

/**
 * @brief 概念约束：判断缓冲的Reader句柄是否支持 clone()
 *
 * SPMC 多消费者允许复制读取器；SPSC 单消费者唯一，不可复制
 */
template <typename Buffer, typename T>
concept HasClone = requires(typename Buffer::template Read<T>& r) {
    // 要求 r.clone() 返回同类型 Reader
    { r.clone() } -> std::same_as<typename Buffer::template Read<T>>;
};

/**
 * @brief 标记SPMC类缓冲：支持Reader克隆
 */
template <typename Buffer, typename T>
concept SpmcLike = HasClone<Buffer, T>;

/**
 * @brief 标记SPSC类缓冲：不支持Reader克隆，仅移动语义
 */
template <typename Buffer, typename T>
concept SpscLike = !HasClone<Buffer, T>;

} // namespace detail

// ============================================================================
// TrackedWriter：封装底层三缓冲写入句柄，增加数据流追踪能力
// ============================================================================

/**
 * @brief 带追踪标记的写入句柄
 *
 * 包装原生三缓冲 Write 句柄，预留链路追踪埋点，用于自动生成数据流图
 *
 * ## 线程安全
 * 完全继承底层缓冲的线程安全特性
 *
 * 模板参数：
 * - T：通道传输的数据类型
 * - Buffer：底层三缓冲实现（SpscTripleBuffer / SpmcTripleBuffer）
 */
template <typename T, typename Buffer>
class TrackedWriter {
    // 底层原生写入句柄类型
    using Inner = Buffer::template Write<T>;
    Inner inner_;

public:
    using value_type  = T;         // 通道承载数据类型
    using buffer_type = Buffer;    // 底层缓冲实现类型

    // 构造：接管原生Write句柄
    TrackedWriter(Inner w) noexcept
        : inner_(std::move(w)) {}

    // 禁用拷贝写入器（生产者唯一，不可复制）
    TrackedWriter(const TrackedWriter&)            = delete;
    TrackedWriter& operator=(const TrackedWriter&) = delete;
    // 允许移动语义
    TrackedWriter(TrackedWriter&&) noexcept        = default;
    TrackedWriter& operator=(TrackedWriter&&)      = default;

    /**
     * @brief 直接写入一份完整数据
     * @param data 待发送数据，内部移动避免拷贝
     */
    void write(T data) noexcept { inner_.write(std::move(data)); }

    /**
     * @brief SPSC专属接口：可变借用缓冲块，原地修改后publish发布
     * 仅底层Write具备borrow_mut()时才编译通过
     */
    auto& borrow_mut() noexcept requires requires(Inner& w) { w.borrow_mut(); } {
        return inner_.borrow_mut();
    }

    /**
     * @brief SPSC专属接口：提交修改后的缓冲帧对外可见
     */
    void publish() noexcept requires requires(Inner& w) { w.publish(); } { inner_.publish(); }

    /**
     * @brief SPMC专属接口：获取当前写入生成号（版本号）
     * 多消费者依靠生成号判断是否读到最新数据
     */
    [[nodiscard]] auto generation() const noexcept
        requires requires(const Inner& w) { w.generation(); } {
        return inner_.generation();
    }

    /// 判断句柄是否有效（底层缓冲未销毁）
    [[nodiscard]] bool valid() const noexcept { return inner_.valid(); }
    /// 布尔隐式转换，快速判断有效性
    explicit operator bool() const noexcept { return valid(); }

    // 高级接口：获取底层原生句柄，供底层操作扩展
    [[nodiscard]] Inner& inner() noexcept { return inner_; }
    [[nodiscard]] const Inner& inner() const noexcept { return inner_; }
};

// ============================================================================
// TrackedReader：封装底层三缓冲读取句柄，区分SPMC可拷贝 / SPSC仅移动
// ============================================================================

/**
 * @brief 带追踪标记的读取句柄
 *
 * 包装原生三缓冲 Read 句柄，统一对外读取接口，区分两种缓冲拷贝特性
 *
 * ## 拷贝语义
 * - SPMC：可拷贝，每个Reader独立维护自己的读取版本标记
 * - SPSC：仅允许移动，全局只能存在一个Reader
 *
 * ## 线程安全
 * 完全继承底层缓冲的线程安全特性
 *
 * 模板参数：
 * - T：通道传输的数据类型
 * - Buffer：底层三缓冲实现
 */
template <typename T, typename Buffer>
class TrackedReader {
    using Inner = Buffer::template Read<T>;
    Inner inner_;

public:
    using value_type  = T;
    using buffer_type = Buffer;

    TrackedReader(Inner r) noexcept
        : inner_(std::move(r)) {}

    // ---------------- SPMC 缓冲：支持拷贝构造/拷贝赋值 ----------------
    TrackedReader(const TrackedReader& other) requires detail::SpmcLike<Buffer, T>
        : inner_(other.inner_) {}

    TrackedReader& operator=(const TrackedReader& other) requires detail::SpmcLike<Buffer, T> {
        if (this != &other) {
            inner_ = other.inner_;
        }
        return *this;
    }

    // ---------------- SPSC 缓冲：禁用拷贝，仅移动 ----------------
    TrackedReader(const TrackedReader&) requires detail::SpscLike<Buffer, T>            = delete;
    TrackedReader& operator=(const TrackedReader&) requires detail::SpscLike<Buffer, T> = delete;

    // 两种缓冲都支持移动语义
    TrackedReader(TrackedReader&&) noexcept            = default;
    TrackedReader& operator=(TrackedReader&&) noexcept = default;

    /**
     * @brief 读取最新未消费数据
     * @return 有新数据返回包含T的optional，无新数据返回std::nullopt
     */
    [[nodiscard]] std::optional<T> read() noexcept { return inner_.read(); }

    /// 查询是否存在未读取的新帧
    [[nodiscard]] bool has_new() const noexcept { return inner_.has_new(); }

    /**
     * @brief SPMC专属：直接读取当前最新帧（不消耗版本标记，多消费者互不干扰）
     */
    [[nodiscard]] auto read_current() noexcept requires requires(Inner& r) { r.read_current(); } {
        return inner_.read_current();
    }

    /**
     * @brief SPMC专属：获取该Reader上一次读取的生成号
     */
    [[nodiscard]] auto last_generation() const noexcept
        requires requires(const Inner& r) { r.last_generation(); } {
        return inner_.last_generation();
    }

    /// 判断读取句柄是否有效
    [[nodiscard]] bool valid() const noexcept { return inner_.valid(); }
    explicit operator bool() const noexcept { return valid(); }

    // 高级接口：暴露底层原生读取句柄
    [[nodiscard]] Inner& inner() noexcept { return inner_; }
    [[nodiscard]] const Inner& inner() const noexcept { return inner_; }
};

// ============================================================================
// TrackedChannel：统一通道顶层封装，抹平SPSC/SPMC对外API差异
// ============================================================================

/**
 * @brief 统一通道顶层容器，封装一对Writer/Reader句柄
 *
 * 对外提供一套统一API，自动适配底层SPSC/SPMC缓冲，内置数据流追踪埋点
 *
 * ## 使用示例
 * ```cpp
 * // 创建单生产者多消费者通道
 * auto channel = primitive::make_spmc_channel<Frame>();
 * // 分离出读写句柄
 * auto [writer, reader] = channel.split();
 *
 * writer.write(frame);
 * if (auto frame = reader.read()) {
 *     // 处理图像帧
 * }
 * ```
 *
 * ## 接口区分
 * - split()：拆分通道，分离独立写入器与读取器
 * - clone_reader()：仅SPMC可用，复制出新的消费者读取器
 * - take_reader()：取出内部Reader，转移所有权
 *
 * ## 线程安全
 * 继承底层三缓冲的线程安全规则
 */
template <typename T, typename Buffer>
class TrackedChannel {
    using WriteInner = Buffer::template Write<T>;
    using ReadInner  = Buffer::template Read<T>;

    WriteInner writer_;
    ReadInner reader_;

public:
    using value_type  = T;
    using buffer_type = Buffer;
    using Writer      = TrackedWriter<T, Buffer>;
    using Reader      = TrackedReader<T, Buffer>;

    // 编译期常量，标记通道类型
    static constexpr bool is_spmc = detail::SpmcLike<Buffer, T>;
    static constexpr bool is_spsc = detail::SpscLike<Buffer, T>;

    // 构造：接收底层原生读写句柄
    TrackedChannel(WriteInner w, ReadInner r) noexcept
        : writer_(std::move(w))
        , reader_(std::move(r)) {}

    // 通道整体不可拷贝（生产者唯一）
    TrackedChannel(const TrackedChannel&)            = delete;
    TrackedChannel& operator=(const TrackedChannel&) = delete;
    // 允许移动整个通道对象
    TrackedChannel(TrackedChannel&&) noexcept        = default;
    TrackedChannel& operator=(TrackedChannel&&)      = default;

    // ========================================================================
    // 内置写入快捷接口（也可使用split()获取独立Writer）
    // ========================================================================
    void write(T data) noexcept { writer_.write(std::move(data)); }

    /// SPSC专属原地修改接口
    auto& borrow_mut() noexcept requires is_spsc { return writer_.borrow_mut(); }
    /// SPSC专属提交更新
    void publish() noexcept requires is_spsc { writer_.publish(); }

    /// SPMC专属获取当前写入生成号
    [[nodiscard]] auto generation() const noexcept requires is_spmc { return writer_.generation(); }

    // ========================================================================
    // SPMC独有：克隆读取器，新增消费者
    // ========================================================================
    [[nodiscard]] Reader clone_reader() const requires is_spmc { return Reader(reader_.clone()); }

    // ========================================================================
    // 取出读取器：转移内部Reader所有权，通道不再持有Reader
    // ========================================================================
    [[nodiscard]] Reader take_reader() { return Reader(std::move(reader_)); }

    // ========================================================================
    // split：分离通道，返回独立Writer + Reader，最常用接口
    // ========================================================================
    struct SplitResult {
        Writer writer;
        Reader reader;
    };

    [[nodiscard]] SplitResult split() {
        return SplitResult{Writer(std::move(writer_)), Reader(std::move(reader_))};
    }
};

// ============================================================================
// 通用工厂函数：创建带追踪的通道
// 使用示例：tracked_create<SpmcTripleBuffer, Frame>()
// ============================================================================

/**
 * @brief 通用通道创建工厂
 * @tparam Buffer 缓冲模板（SpmcTripleBuffer / SpscTripleBuffer）
 * @tparam T 传输数据类型
 * @return 封装完成的TrackedChannel统一通道对象
 */
template <template <typename> class Buffer, typename T>
[[nodiscard]] auto tracked_create() -> TrackedChannel<T, Buffer<T>> {
    // 调用底层缓冲静态create生成原生读写句柄
    auto [w, r] = Buffer<T>::create();
    return TrackedChannel<T, Buffer<T>>(std::move(w), std::move(r));
}

// ============================================================================
// 对外友好类型别名，屏蔽底层缓冲细节
// ============================================================================

/// 单生产者多消费者通道
template <typename T>
using SpmcChannel = TrackedChannel<T, SpmcTripleBuffer<T>>;

/// 单生产者单消费者通道
template <typename T>
using SpscChannel = TrackedChannel<T, SpscTripleBuffer<T>>;

// 便捷工厂函数，直接创建对应通道
template <typename T>
[[nodiscard]] auto make_spmc_channel() -> SpmcChannel<T> {
    return tracked_create<SpmcTripleBuffer, T>();
}

template <typename T>
[[nodiscard]] auto make_spsc_channel() -> SpscChannel<T> {
    return tracked_create<SpscTripleBuffer, T>();
}

// ============================================================================
// 历史兼容别名：Channel<T> 默认等价SPMC多消费者通道
// ============================================================================
template <typename T>
using Channel = SpmcChannel<T>;

} // namespace talos::primitive