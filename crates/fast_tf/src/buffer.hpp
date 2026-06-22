#pragma once

// 欧拉角转换工具（RPY/旋转矩阵互转）
#include "euler.hpp"
// SO3旋转群数学封装（四元数/旋转矩阵流形插值）
#include "groups/SO3.hpp"
// Eigen矩阵封装、基础线性代数工具
#include "matrix.hpp"
// 全局基础类型别名：timestamp_ns_t、fp_t、sec等
#include "types.hpp"
// 变换矩阵合法性校验函数
#include "validation.hpp"

// C++标准库
#include <array>           // 固定长度环形缓冲区底层存储
#include <cassert>         // 静态断言static_assert运行时断言
#include <concepts>        // C++20 概念约束，泛型逻辑分支
#include <cstdint>         // 固定宽度整数
#include <expected>        // C++23 错误返回类型，区分正常数据/错误字符串
#include <fmt/format.h>    // 高性能字符串格式化，日志报错拼接
#include <optional>        // 可选返回值，无数据返回nullopt
#include <tbb/spin_rw_mutex.h> // TBB自旋读写锁：读多写少场景低开销同步
#include <type_traits>     // 类型特征，移动noexcept判断
#include <utility>         // std::move 移动语义

namespace fast_tf {

// 时间单位别名
using sec            = std::size_t;      // 秒数，缓冲区时长范围单位
using sample_per_sec = std::size_t;      // 每秒采样点数，缓冲区密度

// ===================== 时序查询插值模式标记空类型（标签分发） =====================
/**
 * 四种时间戳查找策略，空结构体仅作编译期标签，无运行时开销
 */
struct exact_t {};          // 精确匹配：必须时间戳完全相等，否则报错
struct nearest_t {};        // 就近取值：取前后两个时间戳中距离更近的样本
struct interpolate_t {};    // 线性插值：按时间比例插值两个相邻样本
struct clamped_t {};        // 钳位截断：查询时间超出区间时返回首尾样本，不报错

// 全局常量实例，方便调用时直接传入
inline constexpr exact_t exact{};
inline constexpr nearest_t nearest{};
inline constexpr interpolate_t interpolate{};
inline constexpr clamped_t clamped{};

/**
 * @brief 概念约束：限定模板参数只能是上面四种查询模式
 */
template <typename Mode>
concept buffer_lookup_mode = std::same_as<Mode, exact_t> || std::same_as<Mode, nearest_t>
                          || std::same_as<Mode, interpolate_t> || std::same_as<Mode, clamped_t>;

/**
 * @brief 概念约束：缓冲区值类型必须配套实现BufferOps工具集
 * 要求Ops具备三个静态函数：
 *  1. identity()：返回单位初值（零变换/零旋转）
 *  2. interpolate(a,b,ratio)：两个样本按比例插值生成中间值
 *  3. is_valid(value)：校验数据是否合法有效（过滤非法变换）
 */
template <typename Ops, typename T>
concept buffer_value_ops = requires(const T& value, fp_t ratio) {
    { Ops::identity() } -> std::same_as<T>;
    { Ops::interpolate(value, value, ratio) } -> std::same_as<T>;
    { Ops::is_valid(value) } -> std::same_as<bool>;
};

// ===================== 泛型缓冲区操作特化模板 =====================
/**
 * @brief 缓冲区数据操作基模板，针对不同数据类型做特化实现
 * 对外统一接口：identity / interpolate / is_valid / with_rotation(可选)
 */
template <typename T>
struct BufferOps;

/**
 * @brief 特化1：变换矩阵 TransformMatrix<Scalar, FromFrame, ToFrame>
 * 支持齐次变换矩阵插值、合法性校验、仅修改旋转复用平移
 */
template <std::floating_point T, typename From, typename To>
struct BufferOps<TransformMatrix<T, From, To>> {
    using Transform = TransformMatrix<T, From, To>;

    /// 返回单位变换（无平移无旋转）
    static Transform identity() { return Transform{}; }

    /// 两个位姿矩阵线性插值（底层封装矩阵LERP）
    static Transform interpolate(const Transform& a, const Transform& b, fp_t ratio) {
        return Transform::lerp(a, b, static_cast<T>(ratio));
    }

    /// 校验变换矩阵是否合法（旋转正交、数值无NaN/无穷）
    static bool is_valid(const Transform& value) { return validate_transform(value).has_value(); }

    /**
     * @brief 复用原有平移，仅替换旋转欧拉角生成新变换
     * @param base 原始变换，平移保持不变
     * @param rot 新RPY旋转角
     * @return 平移不变、旋转更新后的新变换矩阵
     */
    static Transform with_rotation(const Transform& base, const math_fuxk::Ros2EulerRotd& rot) {
        const auto translation        = base.translation();
        const auto [roll, pitch, yaw] = rot.rpy();
        return Transform::from_rpy(
            static_cast<T>(roll), static_cast<T>(pitch), static_cast<T>(yaw), translation.x(),
            translation.y(), translation.z());
    }
};

/**
 * @brief 特化2：SO3三维旋转群（四元数旋转流形）
 * 使用球面线性插值Slerp，不能直接线性插值矩阵
 */
template <typename T>
struct BufferOps<group::SO3<T>> {
    /// 单位旋转（无旋转）
    static group::SO3<T> identity() { return group::SO3<T>{}; }

    /// SO3流形插值：log映射到向量空间插值再exp映射回旋转群
    static group::SO3<T> interpolate(const group::SO3<T>& a, const group::SO3<T>& b, fp_t ratio) {
        // a.inv() * b 得到a到b的旋转差，对数映射为角速度向量，缩放后指数还原旋转
        return a * group::SO3<T>::exp(group::SO3<T>::log(a.inv() * b) * ratio);
    }

    /// SO3四元数默认永远合法，无需校验
    static bool is_valid(const group::SO3<T>& value) {
        (void)value;
        return true;
    }
};

/**
 * @brief 特化3：球坐标/云台俯仰偏航类型 Spherial<T>
 * 自定义球面插值实现
 */
template <typename T>
struct BufferOps<Spherial<T>> {
    static Spherial<T> identity() { return Spherial<T>{}; }

    static Spherial<T> interpolate(const Spherial<T>& a, const Spherial<T>& b, fp_t ratio) {
        return Spherial<T>::lerp(a, b, ratio);
    }

    static bool is_valid(const Spherial<T>& value) {
        (void)value;
        return true;
    }
};

// ===================== 时序环形缓冲区核心模板类 =====================
/**
 * @brief 时序环形Buffer：存储带时间戳的位姿/旋转样本，支持多模式时序查询插值
 *
 * 模板参数说明：
 * @tparam T        存储数据类型（TransformMatrix / SO3 / Spherial）
 * @tparam Range    缓存时长(秒)，Range=0代表静态单样本缓冲区StaticBuffer
 * @tparam Density  每秒采样数量，总容量=Range*Density
 * @tparam Ops      配套数据操作工具类BufferOps<T>
 *
 * 两大模式：
 * 1. 动态时序缓冲：Range>0，环形覆盖旧样本，支持时序插值查询
 * 2. 静态缓冲：Range=0 && Density=1，仅保存单一样本，无视时间戳，所有查询返回固定值
 */
template <typename T, sec Range, sample_per_sec Density, typename Ops = BufferOps<T>>
class Buffer {
    // 编译期静态校验
    static_assert(Density > 0, "Density must be greater than 0");
    // 必须实现配套插值/校验操作
    static_assert(buffer_value_ops<Ops, T>, "Buffer ops are not defined for this value type");
    // 静态缓冲约束：仅允许Range=0且Density=1
    static_assert(
        Range > 0 || Density == 1, "Static Buffer only supports Range == 0 && Density == 1");

public:
    /**
     * @brief 对外返回的带时间戳数据结构体
     */
    struct Timestamped {
        T value;                // 存储的位姿/旋转数据
        timestamp_ns_t timestamp; // 对应纳秒时间戳
    };

    // 编译期常量标记缓冲区类型
    static constexpr bool is_static             = Range == 0;
    // 缓冲区总容量：静态固定1，动态=时长*每秒采样数
    static constexpr std::size_t capacity_value = is_static ? 1 : Range * Density;
    // 读写同步锁：TBB自旋读写锁，读多写少场景性能优于互斥锁
    using mutex_type                            = tbb::spin_rw_mutex;

    // 动态缓冲区默认构造
    Buffer() noexcept requires(!is_static) = default;

    /**
     * 静态缓冲区构造：仅存储一个固定样本，时间戳固定0
     */
    explicit Buffer(const T& value) noexcept requires(is_static)
        : buffer_{
              StoredSample{value, 0}
    } {}

    // 禁用拷贝构造/拷贝赋值：缓冲区包含锁，不可拷贝
    Buffer(const Buffer&)            = delete;
    Buffer& operator=(const Buffer&) = delete;

    /**
     * 移动构造：转移另一个缓冲区全部数据、索引、时间戳状态
     */
    Buffer(Buffer&& other) noexcept(std::is_nothrow_move_assignable_v<decltype(buffer_)>) {
        move_from(std::move(other));
    }

    /**
     * 移动赋值：安全转移缓冲区资源，避免锁竞争
     */
    Buffer&
        operator=(Buffer&& other) noexcept(std::is_nothrow_move_assignable_v<decltype(buffer_)>) {
        if (this != &other) {
            move_assign_from(std::move(other));
        }
        return *this;
    }

    /**
     * @brief 写入新时序样本（动态缓冲区专用）
     * @param timestamp_ns 样本纳秒时间戳
     * @param value 待存入数据
     * 逻辑：非法数据直接丢弃；时序倒退丢弃；环形覆盖旧样本，更新首尾时间戳
     */
    void push(timestamp_ns_t timestamp_ns, const T& value) noexcept requires(!is_static) {
        // 校验数据合法性，非法直接丢弃
        if (!Ops::is_valid(value)) {
            return;
        }

        // 写锁独占访问缓冲区
        mutex_type::scoped_lock lock(mutex_);
        // 新样本时序早于最新样本，时序回退，直接丢弃
        if (size_ != 0 && timestamp_ns <= newest_ts_) {
            return;
        }

        // 写入环形缓冲区头部位置
        buffer_[head_] = StoredSample{value, timestamp_ns};

        // 空缓冲区：当前样本为最早时间戳
        if (size_ == 0) {
            oldest_ts_ = timestamp_ns;
        }
        // 更新最新时间戳
        newest_ts_ = timestamp_ns;

        // 环形头指针前进，取模循环
        head_ = (head_ + 1) % capacity_value;

        if (size_ < capacity_value) {
            // 未满，有效样本数+1
            ++size_;
        } else {
            // 缓冲区已满，覆盖最旧样本，更新最早时间戳为下一个逻辑起点
            oldest_ts_ = buffer_[head_].timestamp;
        }
    }

    /**
     * @brief 仅更新旋转、平移复用最新样本的快捷写入接口
     * @param timestamp_ns 新时间戳
     * @param rot 新RPY旋转角
     * 适用：云台仅旋转、平移不变场景，减少外部构造完整变换矩阵代码
     */
    template <typename U = T>
    void push_rotate_only(timestamp_ns_t timestamp_ns, const math_fuxk::Ros2EulerRotd rot) noexcept
        requires(!is_static && requires(const U& value, const math_fuxk::Ros2EulerRotd& rotation) {
            { Ops::with_rotation(value, rotation) } -> std::same_as<U>;
        }) {
        // 默认单位变换
        T base = Ops::identity();
        // 读取缓冲区最新样本，复用平移部分
        if (auto latest_result = latest(); latest_result) {
            base = latest_result->value;
        }
        // 生成仅旋转更新的变换并写入
        push(timestamp_ns, Ops::with_rotation(base, rot));
    }

    /**
     * @brief 获取缓冲区最新写入的样本
     * @return 成功返回带时间戳数据；空缓冲区返回错误字符串
     */
    [[nodiscard]] std::expected<Timestamped, std::string> latest() const noexcept {
        // 静态缓冲区固定返回唯一样本
        if constexpr (is_static) {
            return Timestamped{buffer_[0].value, 0};
        }

        // 共享读锁，多线程并发读无阻塞
        mutex_type::scoped_lock lock(mutex_, false);
        if (size_ == 0) {
            return std::unexpected(
                fmt::format("Buffer::latest(): buffer is empty (capacity={})", capacity_value));
        }

        // 取逻辑最后一个样本（最新）封装对外结构体
        return materialize(at_logical(size_ - 1));
    }

    /**
     * @brief 核心时序查询函数：按指定模式查找对应时间戳数据
     * @tparam Mode 查询模式 exact/nearest/interpolate/clamped
     * @param ts 目标查询纳秒时间戳
     * @return 匹配/插值后的时序数据，或错误信息
     */
    template <buffer_lookup_mode Mode>
    [[nodiscard]] std::expected<Timestamped, std::string>
        lookup(timestamp_ns_t ts, Mode) const noexcept {
        // 静态缓冲区无视查询时间，直接返回固定样本
        if constexpr (is_static) {
            return Timestamped{buffer_[0].value, ts};
        }

        mutex_type::scoped_lock lock(mutex_, false);
        // 缓冲区空直接报错
        if (size_ == 0) {
            return std::unexpected(
                fmt::format(
                    "Buffer::lookup({}ns): buffer is empty (capacity={})", ts, capacity_value));
        }

        // 获取逻辑首尾样本
        const auto& oldest = at_logical(0);
        const auto& newest = at_logical(size_ - 1);

        // ========== 处理查询时间超出缓冲区区间 ==========
        if constexpr (std::same_as<Mode, clamped_t>) {
            // clamped模式：截断，超出直接返回首尾，不报错
            if (ts < oldest.timestamp) {
                return materialize(oldest);
            }
            if (ts > newest.timestamp) {
                return materialize(newest);
            }
        } else {
            // 其余模式：超出区间报错，禁止外推
            if (ts < oldest.timestamp) {
                return std::unexpected(
                    fmt::format(
                        "Buffer::lookup({}ns): past extrapolation required, "
                        "buffer range=[{}, {}], size={}, query_ts={}",
                        ts, oldest.timestamp, newest.timestamp, size_, ts));
            }
            if (ts > newest.timestamp) {
                return std::unexpected(
                    fmt::format(
                        "Buffer::lookup({}ns): future extrapolation required, "
                        "buffer range=[{}, {}], size={}, query_ts={}",
                        ts, oldest.timestamp, newest.timestamp, size_, ts));
            }
        }

        // 二分线性查找小于等于ts的最大时间戳样本下标（floor下界）
        const std::size_t lower_idx = find_floor_from_latest(ts);
        const auto& lower           = at_logical(lower_idx);
        // 精确匹配，直接返回
        if (lower.timestamp == ts) {
            return materialize(lower);
        }

        // ========== 按不同模式分支处理 ==========
        if constexpr (std::same_as<Mode, exact_t>) {
            // exact模式无完全匹配直接报错
            return std::unexpected(
                fmt::format(
                    "Buffer::lookup({}ns, exact): no exact match, "
                    "nearest floor={}ns, buffer range=[{}, {}]",
                    ts, lower.timestamp, oldest.timestamp, newest.timestamp));
        } else if constexpr (std::same_as<Mode, nearest_t>) {
            // nearest就近模式
            if (lower_idx + 1 >= size_) {
                // 下界已是最后一个样本，无下一个，直接返回
                return materialize(lower);
            }

            if (lower.timestamp > ts) {
                return materialize(oldest);
            }

            const auto& next = at_logical(lower_idx + 1);
            // 计算前后时间差，取更近样本
            const auto prev_delta = ts - lower.timestamp;
            const auto next_delta = next.timestamp - ts;
            return materialize(prev_delta <= next_delta ? lower : next);
        } else {
            // interpolate线性插值模式
            if (lower.timestamp > ts) {
                return materialize(oldest);
            }
            if (lower_idx + 1 >= size_) {
                return materialize(lower);
            }

            const auto& next = at_logical(lower_idx + 1);
            const auto dt    = next.timestamp - lower.timestamp;
            // 两个样本时间戳完全相同，直接取下界
            if (dt == 0) {
                return materialize(lower);
            }

            // 计算插值比例0~1，调用Ops插值函数生成中间值
            const fp_t ratio = static_cast<fp_t>(ts - lower.timestamp) / static_cast<fp_t>(dt);
            return Timestamped{Ops::interpolate(lower.value, next.value, ratio), ts};
        }
    }

    /**
     * @brief 获取缓冲区存储的完整时间区间[最早时间戳, 最新时间戳]
     * @return 空缓冲区返回nullopt，否则返回pair
     */
    [[nodiscard]] std::optional<std::pair<timestamp_ns_t, timestamp_ns_t>>
        time_range() const noexcept {
        if constexpr (is_static) {
            return std::make_pair(timestamp_ns_t{0}, timestamp_ns_t{0});
        }

        mutex_type::scoped_lock lock(mutex_, false);
        if (size_ == 0) {
            return std::nullopt;
        }
        return std::make_pair(oldest_ts_, newest_ts_);
    }

    /// 当前有效样本数量
    [[nodiscard]] std::size_t size() const noexcept {
        if constexpr (is_static) {
            return 1;
        }

        mutex_type::scoped_lock lock(mutex_, false);
        return size_;
    }

    /// 缓冲区总固定容量（编译期常量）
    [[nodiscard]] constexpr std::size_t capacity() const noexcept { return capacity_value; }

    /// 判断缓冲区是否存满
    [[nodiscard]] bool is_full() const noexcept {
        if constexpr (is_static) {
            return true;
        }

        mutex_type::scoped_lock lock(mutex_, false);
        return size_ == capacity_value;
    }

    /// 判断缓冲区是否无有效样本
    [[nodiscard]] bool is_empty() const noexcept {
        if constexpr (is_static) {
            return false;
        }

        mutex_type::scoped_lock lock(mutex_, false);
        return size_ == 0;
    }

    /// 查询目标时间戳是否落在缓冲区时序区间内
    [[nodiscard]] bool contains_time(timestamp_ns_t ts) const noexcept {
        if constexpr (is_static) {
            (void)ts;
            return true;
        }

        mutex_type::scoped_lock lock(mutex_, false);
        if (size_ == 0) {
            return false;
        }
        return ts >= oldest_ts_ && ts <= newest_ts_;
    }

    /**
     * @brief 清空动态缓冲区所有样本，重置索引与时间戳
     */
    void clear() noexcept requires(!is_static) {
        mutex_type::scoped_lock lock(mutex_);
        head_      = 0;
        size_      = 0;
        oldest_ts_ = 0;
        newest_ts_ = 0;
    }

private:
    /**
     * @brief 底层存储单样本结构：原始存储单元
     */
    struct StoredSample {
        T value{};
        timestamp_ns_t timestamp = 0;
    };

    /**
     * @brief 移动构造内部实现：转移源缓冲区全部数据
     * @param other 待移动的源缓冲区
     */
    void move_from(Buffer&& other) noexcept(std::is_nothrow_move_assignable_v<decltype(buffer_)>) {
        typename mutex_type::scoped_lock lock(other.mutex_);
        buffer_    = std::move(other.buffer_);
        head_      = other.head_;
        size_      = other.size_;
        oldest_ts_ = other.oldest_ts_;
        newest_ts_ = other.newest_ts_;
        // 置空原对象状态
        reset_moved_from(other);
    }

    /**
     * @brief 移动赋值内部实现：同时锁定this与other避免死锁
     */
    void move_assign_from(Buffer&& other) noexcept(
        std::is_nothrow_move_assignable_v<decltype(buffer_)>) {
        typename mutex_type::scoped_lock this_lock(mutex_);
        typename mutex_type::scoped_lock other_lock(other.mutex_);
        buffer_    = std::move(other.buffer_);
        head_      = other.head_;
        size_      = other.size_;
        oldest_ts_ = other.oldest_ts_;
        newest_ts_ = other.newest_ts_;
        reset_moved_from(other);
    }

    /**
     * @brief 重置已被移动走的缓冲区状态，避免析构异常
     */
    static void reset_moved_from(Buffer& other) noexcept {
        if constexpr (!is_static) {
            other.head_      = 0;
            other.size_      = 0;
            other.oldest_ts_ = 0;
            other.newest_ts_ = 0;
        }
    }

    /**
     * @brief 底层存储样本转换为对外Timestamped结构体
     */
    [[nodiscard]] Timestamped materialize(const StoredSample& sample) const {
        return Timestamped{sample.value, sample.timestamp};
    }

    /**
     * @brief 逻辑下标转环形数组物理下标
     * 逻辑下标0 = 最旧样本；逻辑下标size-1 = 最新样本
     */
    [[nodiscard]] const StoredSample& at_logical(std::size_t idx) const {
        const std::size_t physical = (head_ + capacity_value - size_ + idx) % capacity_value;
        return buffer_[physical];
    }

    /**
     * @brief 从最新样本向前线性查找第一个 <= ts 的样本下标（floor下界）
     * 时序数据严格递增，样本量小线性遍历性能足够
     */
    [[nodiscard]] std::size_t find_floor_from_latest(timestamp_ns_t ts) const {
        std::size_t idx = size_ - 1;
        while (idx > 0 && at_logical(idx).timestamp > ts) {
            --idx;
        }
        return idx;
    }

    // 底层固定数组环形存储
    std::array<StoredSample, capacity_value> buffer_{};
    std::size_t head_         = 0;      // 环形写入头指针（下一次写入位置）
    std::size_t size_         = 0;      // 当前有效样本总数
    timestamp_ns_t oldest_ts_ = 0;      // 缓冲区最旧样本时间戳
    timestamp_ns_t newest_ts_ = 0;      // 缓冲区最新样本时间戳
    mutable mutex_type mutex_;          // 读写自旋锁，const函数也可加锁 mutable
};

/**
 * @brief 静态单样本缓冲区别名
 * Range=0, Density=1，仅保存固定一个变换/旋转，无视时序插值查询
 */
template <typename T, typename Ops = BufferOps<T>>
using StaticBuffer = Buffer<T, 0, 1, Ops>;

} // namespace fast_tf