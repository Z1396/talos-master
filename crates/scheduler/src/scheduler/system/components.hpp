// 头文件保护：防止重复包含，标准写法
#pragma once

// 底层基础通道/队列实现
#include "primitive/channel.hpp"
// 类型判断、类型特征工具
#include <type_traits>
// 移动语义、完美转发
#include <utility>

// 项目命名空间：talos 项目 -> 调度器模块 -> 系统层组件
namespace talos::scheduler::system {

// ============================================================================
// 一、C++20 Concept 类型约束（编译期类型校验）
// 作用：强制模板参数满足指定特征，不符合则编译报错，提升代码健壮性
// ============================================================================

/**
 * @brief 概念：topic_type 话题标签类型约束
 * @tparam T 待校验类型
 * 规则：
 * 1. 类型是 void（无话题）
 * 2. 或是 空标签类型（sizeof(T) == 0，仅用作编译期标记，不存数据）
 * 用途：通信话题(Topic)是**空标签类型**，仅做类型区分、不占用内存
 */
template <typename T>
concept topic_type = std::is_same_v<T, void> || requires { sizeof(T) == 0; };

/**
 * @brief 概念：component_kind 调度器合法组件约束
 * @tparam T 待校验组件类型（spsc / spmc / res / local 等）
 * 要求：
 * 1. 内部必须嵌套类型 value_type（存储数据类型）
 * 2. 内部必须嵌套类型 topic_type（话题标签类型）
 * 3. value_type 不能是 void（必须承载有效数据）
 * 4. topic_type 必须满足上面的 topic_type 约束
 * 作用：统一校验所有通信/资源组件，保证接口规范一致
 */
template <typename T>
concept component_kind = requires {
    typename T::value_type;
    typename T::topic_type;
    requires !std::is_void_v<typename T::value_type>;
    requires topic_type<typename T::topic_type>;
};

/**
 * @brief 默认空话题标签
 * 当使用者不指定自定义话题时，默认使用该空类型
 * 空结构体，sizeof=0，仅作编译期类型标记，无运行时开销
 */
struct DefaultTopic {};

// ============================================================================
// 二、通道/组件类型枚举：区分组件大类
// ============================================================================

/**
 * @brief 组件类别枚举，标记当前组件是什么类型
 * 调度器依靠该枚举做依赖分析、唤醒策略、内存管理
 */
enum class channel_kind {
    spsc_reader,  ///< SPSC 队列 读端
    spsc_writer,  ///< SPSC 队列 写端
    spmc_reader,  ///< SPMC 队列 读端
    spmc_writer,  ///< SPMC 队列 写端
    res,          ///< 只读共享资源（多任务只读，不可修改）
    res_mut,      ///< 可写共享资源（多任务读写）
    local,        ///< 任务/系统局部变量（不共享、不参与依赖分析）
};

// ============================================================================
// 三、通用模板封装：统一读通道、写通道、资源、局部变量
// 设计思路：门面模式，对底层原始队列做一层包装，统一对外接口
// ============================================================================

/**
 * @brief 通用读通道模板（封装 SPSC/SPMC 读端）
 * @tparam T          存储的数据类型
 * @tparam Topic      话题标签类型，默认 DefaultTopic
 * @tparam UnderlyingChannel 底层原始读队列类型（来自 primitive::channel）
 * @tparam Kind       组件类型枚举
 * @tparam HasNew     是否拥有 has_new() 方法（SPMC 独有，判断是否有新数据）
 *
 * 职责：对接底层原始队列，对外提供统一的读接口
 */
template <
    typename T, 
    typename Topic = DefaultTopic, 
    typename UnderlyingChannel = void,
    channel_kind Kind = static_cast<channel_kind>(0), 
    bool HasNew = false>
    
struct basic_channel {
    // 对外暴露类型：遵循统一组件规范
    using value_type           = T;
    using topic_type           = Topic;
    // 组件类型常量
    static constexpr auto kind = Kind;

    // 指向底层原始队列的指针
    // 注释：由调度器框架保证指针永远合法，无需判空
    UnderlyingChannel* ptr_ = nullptr;

    /**
     * @brief 读取一条数据（弹出队列头部）
     * @return 取出的数据
     * noexcept：保证不抛异常
     */
    [[nodiscard]] auto read() noexcept { return ptr_->read(); }

    /**
     * @brief 读取当前最新数据（不弹出，仅查看）
     * 适用于只需要最新帧、丢弃历史数据的场景（视觉/传感器常用）
     * @return 当前最新数据
     */
    [[nodiscard]] auto read_current() noexcept { return ptr_->read_current(); }

    /**
     * @brief 判断是否存在未读取的新数据
     * requires HasNew：C++20 约束，仅 HasNew=true 时该方法才存在
     * 仅 SPMC 读端启用此接口
     */
    [[nodiscard]] bool has_new() const noexcept requires HasNew { return ptr_->has_new(); }

    /**
     * @brief 获取数据版本号(世代号)
     * 用于判断数据是否更新、做数据时序比对
     * 约束：仅底层队列拥有 last_generation 方法时，该接口才生效
     */
    [[nodiscard]] auto last_generation() const noexcept
        requires requires(const UnderlyingChannel& channel) { channel.last_generation(); } {
        return ptr_->last_generation();
    }
};

/**
 * @brief 通用写通道模板（封装 SPSC/SPMC 写端）
 * @tparam T          数据类型
 * @tparam Topic      话题标签
 * @tparam UnderlyingChannel 底层原始写队列
 * @tparam Kind       组件类型枚举
 */
template <
    typename T, typename Topic = DefaultTopic, typename UnderlyingChannel = void,
    channel_kind Kind = static_cast<channel_kind>(0)>
struct basic_writer {
    using value_type           = T;
    using topic_type           = Topic;
    static constexpr auto kind = Kind;

    UnderlyingChannel* ptr_ = nullptr;
    // 唤醒标记指针：由调度器框架管理
    // 写入数据后置为 true，调度器据此唤醒等待该数据的读任务
    bool* written_flag_     = nullptr;

    /**
     * @brief 向队列写入数据
     * @param value 待写入数据，使用 std::move 转移所有权，避免拷贝
     * [[likely]] 编译器优化：该分支大概率执行
     */
    void write(T value) noexcept {
        ptr_->write(std::move(value));
        if (written_flag_) [[likely]]
            *written_flag_ = true; // 标记有新数据，触发任务唤醒
    }
};

/**
 * @brief 通用共享资源模板（只读/可写全局资源）
 * @tparam T         资源数据类型
 * @tparam IsMutable  true=可读写资源，false=只读资源
 * @tparam Kind      组件类型枚举
 *
 * 用途：共享配置、全局状态、硬件句柄等**非队列型共享数据**
 */
template <typename T, bool IsMutable, channel_kind Kind>
struct basic_resource {
    using value_type           = T;
    // 资源不使用话题机制，topic_type 设为 void
    using topic_type           = void;
    static constexpr auto kind = Kind;

    // 指针类型：根据是否可写，自动选择 T* 或 const T*
    using PtrType = std::conditional_t<IsMutable, T*, const T*>;

    PtrType ptr_ = nullptr;

    // ===================== 只读资源 访问接口 =====================
    // 解引用取值
    [[nodiscard]] const T& operator*() const noexcept requires(!IsMutable) { return *ptr_; }
    // 箭头访问成员
    [[nodiscard]] const T* operator->() const noexcept requires(!IsMutable) { return ptr_; }

    // ===================== 可写资源 访问接口 =====================
    [[nodiscard]] T& operator*() noexcept requires IsMutable { return *ptr_; }
    [[nodiscard]] T* operator->() noexcept requires IsMutable { return ptr_; }

    // 可写资源的 const 重载：const 上下文也能只读访问
    [[nodiscard]] const T& operator*() const noexcept { return *ptr_; }
    [[nodiscard]] const T* operator->() const noexcept { return *ptr_; }
};

// ============================================================================
// 四、系统局部变量：不共享、不参与调度依赖
// ============================================================================

/**
 * @brief 系统/任务局部变量
 * @tparam T 变量类型
 * 特点：
 * 1. 仅当前任务/系统内部使用，**不跨任务共享**
 * 2. 不参与调度器依赖分析、不触发任务唤醒
 * 3. 纯本地私有数据
 */
template <typename T>
struct local {
    using value_type           = T;
    using topic_type           = void;
    static constexpr auto kind = channel_kind::local;

    T* ptr_ = nullptr;

    // 重载 * 和 -> 模仿原生指针用法，使用体验和普通指针一致
    [[nodiscard]] T& operator*() noexcept { return *ptr_; }
    [[nodiscard]] T* operator->() noexcept { return ptr_; }
    [[nodiscard]] const T& operator*() const noexcept { return *ptr_; }
    [[nodiscard]] const T* operator->() const noexcept { return *ptr_; }
};

// ============================================================================
// 五、对外公开 API：类型别名（简化使用，项目业务层直接用这些别名）
// 把通用模板绑定到底层具体队列，对外暴露简洁名称
// ============================================================================

// ------------------------------
// SPSC 单生产者单消费者队列
// ------------------------------
/**
 * @brief SPSC 读端：单写单读
 */
template <typename T, typename Topic = DefaultTopic>
using spsc = basic_channel<
    T, 
    Topic, 
    typename primitive::SpscChannel<T>::Reader, 
    channel_kind::spsc_reader,
    false  // SPSC 不需要 has_new() 接口
>;

/**
 * @brief SPSC 写端
 */
template <typename T, typename Topic = DefaultTopic>
using spsc_mut = basic_writer<
    T, 
    Topic, 
    typename primitive::SpscChannel<T>::Writer, 
    channel_kind::spsc_writer>;

// ------------------------------
// SPMC 单生产者多消费者队列
// ------------------------------
/**
 * @brief SPMC 读端：单写多读（支持多个任务同时读取）
 */
template <typename T, typename Topic = DefaultTopic>
using spmc = basic_channel<
    T,                                  // 参数1：消息负载类型
    Topic,                              // 参数2：话题类型
    typename primitive::SpmcChannel<T>::Reader, // 参数3：读取器实现类
    channel_kind::spmc_reader,          // 参数4：通道枚举类型标记
    true                                // 参数5：特性开关：开启has_new()接口
>;

/**
 * @brief SPMC 写端
 */
template <typename T, typename Topic = DefaultTopic>
using spmc_mut = basic_writer<
    T, 
    Topic, 
    typename primitive::SpmcChannel<T>::Writer, 
    channel_kind::spmc_writer>;

// ------------------------------
// 共享资源
// ------------------------------
/**
 * @brief 只读共享资源
 */
template <typename T>
using res = basic_resource<T, 
            false, 
            channel_kind::res>;

/**
 * @brief 可写共享资源
 */
template <typename T>
using res_mut = basic_resource<T, 
                true, 
                channel_kind::res_mut>;

// ------------------------------
// 语义化别名（业务常用）
// ------------------------------
/**
 * @brief 订阅：语义等价于 SPMC 读端（订阅话题数据）
 */
template <typename T, typename Topic = DefaultTopic>
using subscribe = spmc<T, Topic>;

/**
 * @brief 发布：语义等价于 SPMC 写端（向话题发布数据）
 */
template <typename T, typename Topic = DefaultTopic>
using publish = spmc_mut<T, Topic>;

// ============================================================================
// 六、静态断言：编译期校验所有组件符合规范
// 确保上面所有别名都满足 component_kind 约束，提前拦截类型错误
// ============================================================================
// 编译期校验所有对外公开的组件别名是否满足统一的 component_kind 约束
// 提前拦截类型不符合规范的错误，避免运行时才暴露问题
static_assert(component_kind<spsc<bool>>);      // 校验 SPSC 单生产者单消费者读端组件
static_assert(component_kind<spsc_mut<bool>>); // 校验 SPSC 单生产者单消费者写端组件
static_assert(component_kind<spmc<bool>>);      // 校验 SPMC 单生产者多消费者读端组件
static_assert(component_kind<spmc_mut<bool>>); // 校验 SPMC 单生产者多消费者写端组件
static_assert(component_kind<res<bool>>);      // 校验只读共享资源组件
static_assert(component_kind<res_mut<bool>>);   // 校验可写共享资源组件
static_assert(component_kind<local<bool>>);     // 校验系统/任务局部变量组件

} // namespace talos::scheduler::system