#pragma once

// 类型名反解析工具（把编译期 mangled 符号转为可读类型名，用于日志、报错）
#include "demangle.hpp"
// 全局错误定义、panic 致命错误函数
#include "error.hpp"
// 系统组件定义：资源读写句柄、SPSC/SPMC 通道封装、Topic 标签等
#include "system/components.hpp"
// 系统元数据：ChannelKey、哈希函数、默认通信标签等
#include "system/system_meta.hpp"

#include <atomic>        // 原子变量：无锁并发、状态标记、计数器
#include <concepts>      // C++20 概念，做类型约束（本文件少量使用）
#include <cstdint>       // 标准固定宽度整型
#include <memory>        // 智能指针 unique_ptr / shared_ptr / weak_ptr
#include <spdlog/spdlog.h> // 日志库，用于模块注册、调试打印
#include <type_traits>   // 类型特征：移动/拷贝、构造性判断、类型萃取
#include <typeindex>     // 运行时类型索引，作为哈希表 Key
#include <typeinfo>      // RTTI：typeid 获取运行时类型信息
#include <unordered_map> // 哈希表，存储异构资源、通信通道
#include <utility>       // 移动语义 std::forward、std::move
#include <vector>        // 动态数组，存放多消费者读端

// Talos 机器人框架：调度器顶层命名空间
namespace talos::scheduler {

// 预留 ROS 兼容层前置声明，通信发布槽位
namespace rclcompat {
template <typename T, typename Topic>
struct PubSlot;
} // namespace rclcompat

// ============================================================================
// 【仅可移动的类型擦除容器 UniqueAny】
// 设计目标：类似 std::any，但**强制仅移动、禁止拷贝**
// 适用场景：
// 1. 存放 unique_ptr、通信通道、硬件资源等【不可拷贝对象】
// 2. 异构容器统一存储，不需要业务类继承虚基类，降低侵入性
// 业务背景：机器狗电调、传感器、CAN通道、驱动资源大多不可拷贝，非常适配
// ============================================================================
class UniqueAny {
    // 内部抽象接口基类：类型擦除统一接口
    struct Concept {
        virtual ~Concept()                                  = default;
        // 获取内部真实类型信息
        virtual const std::type_info& type() const noexcept = 0;
    };

    // 模板实现类：承载具体业务类型，继承抽象接口
    template <typename T>
    struct Model final : Concept {
        T value;  // 真正存储的业务对象

        /**
         * @brief 原位构造内部对象
         * @tparam Args 构造参数包
         * @param args 转发构造参数
         * @note 继承原类型的 noexcept 属性，保证性能
         */
        template <typename... Args>
        explicit Model(std::in_place_t, Args&&... args) noexcept(
            std::is_nothrow_constructible_v<T, Args&&...>)
            : value(std::forward<Args>(args)...) {}

        // 实现基类接口：返回当前存储类型
        const std::type_info& type() const noexcept override { return typeid(T); }
    };

    // 指向类型擦除基类的智能指针，持有异构数据
    std::unique_ptr<Concept> ptr_;

public:
    UniqueAny()  = default;  // 默认构造：空容器
    ~UniqueAny() = default;  // 默认析构

    // 允许移动构造、移动赋值（核心特性：仅移动）
    UniqueAny(UniqueAny&&) noexcept            = default;
    UniqueAny& operator=(UniqueAny&&) noexcept = default;

    // 禁用拷贝构造、拷贝赋值
    // 原因：机器人硬件资源、通道句柄禁止拷贝，从语法层面强制约束
    UniqueAny(const UniqueAny&)            = delete;
    UniqueAny& operator=(const UniqueAny&) = delete;

    /**
     * @brief 静态工厂方法：创建并填充 UniqueAny
     * @tparam T 目标存储类型
     * @tparam Args 构造参数
     * @return 构造完成的容器对象
     */
    template <typename T, typename... Args>
    static UniqueAny make(Args&&... args) {
        UniqueAny out;
        out.emplace<T>(std::forward<Args>(args)...);
        return out;
    }

    /**
     * @brief 原位构造对象，替换容器原有内容
     * @tparam T 目标类型
     * @tparam Args 构造参数
     * @return 内部对象引用
     */
    template <typename T, typename... Args>
    T& emplace(Args&&... args) {
        ptr_ = std::make_unique<Model<T>>(std::in_place, std::forward<Args>(args)...);
        return static_cast<Model<T>*>(ptr_.get())->value;
    }

    /**
     * @brief 判断容器是否存有有效数据
     * @return true=非空，false=空
     */
    [[nodiscard]] bool has_value() const noexcept { return static_cast<bool>(ptr_); }

    /**
     * @brief 获取当前存储的运行时类型信息
     * @return 空容器返回 typeid(void)
     */
    [[nodiscard]] const std::type_info& type() const noexcept {
        return ptr_ ? ptr_->type() : typeid(void);
    }

    /**
     * @brief 安全类型转换（非const）
     * @tparam T 目标类型
     * @return 类型匹配返回对象指针，不匹配/空返回 nullptr
     */
    template <typename T>
    [[nodiscard]] T* get() noexcept {
        if (!ptr_ || ptr_->type() != typeid(T)) {
            return nullptr;
        }
        return &static_cast<Model<T>*>(ptr_.get())->value;
    }

    /**
     * @brief 安全类型转换（const 只读）
     * @tparam T 目标类型
     * @return 类型匹配返回只读指针，否则 nullptr
     */
    template <typename T>
    [[nodiscard]] const T* get() const noexcept {
        if (!ptr_ || ptr_->type() != typeid(T)) {
            return nullptr;
        }
        return &static_cast<const Model<T>*>(ptr_.get())->value;
    }

    /**
     * @brief 强制类型转换（非const）
     * @tparam T 目标类型
     * @return 内部对象引用
     * @note 类型不匹配直接调用 panic 终止程序，用于调度器核心逻辑（不允许类型错误）
     */
    template <typename T>
    [[nodiscard]] T& as() noexcept {
        auto* p = get<T>();
        if (!p) {
            panic(
                "UniqueAny bad cast: requested {} but stored {}",
                detail::demangle(typeid(T).name()), detail::demangle(type().name()));
        }
        return *p;
    }

    /**
     * @brief 强制类型转换（const 只读）
     * @tparam T 目标类型
     * @return 只读对象引用
     */
    template <typename T>
    [[nodiscard]] const T& as() const noexcept {
        auto* p = get<T>();
        if (!p) {
            panic(
                "UniqueAny bad cast: requested {} but stored {}",
                detail::demangle(typeid(T).name()), detail::demangle(type().name()));
        }
        return *p;
    }
};

// ============================================================================
// 【通用资源包装器 Resource】
// 作用：对任意硬件/业务资源做一层统一包裹，统一异构存储格式
// 场景：机器狗传感器、IMU、相机、电调参数、底盘配置等全局资源
// ============================================================================
template <typename T>
struct Resource {
    T value; // 被包装的真实资源

    /**
     * @brief 默认构造，要求类型支持默认初始化
     */
    Resource() noexcept(std::is_nothrow_default_constructible_v<T>)
        requires std::default_initializable<T>
        = default;

    /**
     * @brief 原位构造：直接转发参数构造内部资源
     */
    template <typename... Args>
    explicit Resource(std::in_place_t, Args&&... args) noexcept(
        std::is_nothrow_constructible_v<T, Args&&...>)
        : value(std::forward<Args>(args)...) {}

    /**
     * @brief 转发构造：接收右值资源，支持移动构造
     */
    template <typename U>
    explicit Resource(U&& val) noexcept(std::is_nothrow_constructible_v<T, U&&>)
        : value(std::forward<U>(val)) {}
};

/**
 * @brief 世界生命周期令牌
 * 作用：标记 World 全局容器存活状态，外部模块通过弱指针判断容器是否已销毁
 * 场景：机器人程序退出、模块卸载时做安全判空
 */
struct WorldLifetimeToken {};

// ============================================================================
// 【设计说明】
// World 是整个机器狗调度器的**全局根容器**
// 1. 使用 UniqueAny 做类型擦除，统一存储所有异构资源 + 通信通道
// 2. UniqueAny 仅移动、禁止拷贝，适配 CAN 通道、电调句柄、独占硬件资源
// 3. 对外接口强类型，类型转换仅发生在容器内部，上层业务无感知
// ============================================================================

// ============================================================================
// 【SPSC 通道存储结构】Single Producer Single Consumer
// 单生产者、单消费者通道：一对一通信
// 机器狗场景：主控 ↔ 单个电调、主控 ↔ 单个传感器
// 特点：读写端拆分，一旦被占用就不可二次绑定，保证一对一语义
// ============================================================================
template <typename T>
struct SpscStorage {
    using Channel    = primitive::SpscChannel<T>; // 底层 SPSC 队列/通道
    using Writer     = Channel::Writer;          // 生产者（写端）
    using Reader     = Channel::Reader;          // 消费者（读端）
    using value_type = T;                        // 通道传输数据类型

    std::unique_ptr<Writer> writer;  // 写端智能指针
    std::unique_ptr<Reader> reader;   // 读端智能指针
    bool writer_claimed = false;      // 标记：写端是否已被绑定占用
    bool reader_claimed = false;      // 标记：读端是否已被绑定占用

    /**
     * @brief 静态创建 SPSC 通道实例
     * @return 全新通道存储独占指针
     * 逻辑：创建底层通道 → 拆分为独立读写端 → 存入当前结构体
     */
    static std::unique_ptr<SpscStorage> create() {
        auto ch         = primitive::make_spsc_channel<T>();
        auto [w, r]     = ch.split(); // SPSC 核心：拆分读写端，强制一对一
        auto storage    = std::make_unique<SpscStorage>();
        storage->writer = std::make_unique<Writer>(std::move(w));
        storage->reader = std::make_unique<Reader>(std::move(r));
        return storage;
    }
};

// ============================================================================
// 【SPMC 通道存储结构】Single Producer Multi Consumer
// 单生产者、多消费者通道：一对多广播
// 机器狗高频场景：
// 1. IMU 数据广播给姿态解算、自瞄、可视化
// 2. 主控指令广播给多个腿关节电调（CAN 总线多节点）
// 特点：写端唯一，读端可无限克隆，天然适配 CAN 总线多从机
// ============================================================================
template <typename T>
struct SpmcStorage {
    using Channel    = primitive::SpmcChannel<T>; // 底层 SPMC 广播通道
    using Reader     = Channel::Reader;           // 消费者读端
    using value_type = T;                         // 传输数据类型

    std::unique_ptr<Channel> channel;                // 通道主体（唯一写端）
    std::vector<std::unique_ptr<Reader>> readers;    // 所有克隆的读端集合

    /**
     * @brief 静态创建 SPMC 通道实例
     */
    static std::unique_ptr<SpmcStorage> create() {
        auto storage     = std::make_unique<SpmcStorage>();
        storage->channel = std::make_unique<Channel>(primitive::make_spmc_channel<T>());
        return storage;
    }

    /**
     * @brief 克隆出新读端，支持多消费者订阅
     * @return 新增读端裸指针
     * 对应业务：多个电调/模块订阅同一份广播数据
     */
    Reader* add_reader() {
        auto reader = std::make_unique<Reader>(channel->clone_reader());
        auto* raw   = reader.get();
        readers.push_back(std::move(reader));
        return raw;
    }
};

// ============================================================================
// 【资源存储器 ResourceStore】
// 按「类型」全局管理所有机器人硬件/业务资源
// 底层：type_index → UniqueAny 哈希表，异构存储
// 约束：同类型资源全局唯一，重复创建直接 panic
// ============================================================================
class ResourceStore {
public:
    /**
     * @brief 原位创建并存入资源
     * @tparam T 资源类型
     * @tparam Args 构造参数
     * @return 资源本体引用
     * 规则：同类型资源只能存在一份，重复插入报错
     */
    template <typename T, typename... Args>
    [[nodiscard]] T& emplace(Args&&... args) {
        using U = std::remove_cvref_t<T>; // 去除 const/引用，拿到原始类型

        // 尝试插入：不存在则创建，存在则返回已有迭代器
        auto [it, inserted] = resources_.try_emplace(typeid(U));

        // 资源已存在，致命错误（调度器设计：全局资源单例）
        if (!inserted) [[unlikely]] {
            panic("Resource already exists: {}", detail::demangle(typeid(U).name()));
        }

        // 在类型擦除容器中原位构造 Resource 包装器
        auto& storage =
            it->second.template emplace<Resource<U>>(std::in_place, std::forward<Args>(args)...);

        return storage.value;
    }

    /**
     * @brief 判断指定类型资源是否存在
     */
    template <typename T>
    [[nodiscard]] bool contains() const noexcept {
        return resources_.contains(typeid(T));
    }

    /**
     * @brief 只读获取资源包装器指针
     * @return 不存在返回 nullptr
     */
    template <typename T>
    [[nodiscard]] const Resource<T>* get_storage() const noexcept {
        const auto it = resources_.find(typeid(T));
        if (it == resources_.end()) [[unlikely]] {
            return nullptr;
        }
        return it->second.template get<Resource<T>>();
    }

    /**
     * @brief 可写获取资源包装器指针
     */
    template <typename T>
    [[nodiscard]] Resource<T>* get_storage_mut() noexcept {
        const auto it = resources_.find(typeid(T));
        if (it == resources_.end()) [[unlikely]] {
            return nullptr;
        }
        return it->second.template get<Resource<T>>();
    }

    /**
     * @brief 只读获取资源本体，不存在直接 panic
     */
    template <typename T>
    [[nodiscard]] const T& get() const noexcept {
        auto* storage = get_storage<T>();
        if (!storage) [[unlikely]] {
            panic("Resource not found: {}", detail::demangle(typeid(T).name()));
        }
        return storage->value;
    }

    /**
     * @brief 可写获取资源本体，不存在直接 panic
     */
    template <typename T>
    [[nodiscard]] T& get_mut() noexcept {
        auto* storage = get_storage_mut<T>();
        if (!storage) [[unlikely]] {
            panic("Resource not found: {}", detail::demangle(typeid(T).name()));
        }
        return storage->value;
    }

private:
    // Key：运行时类型索引  Value：类型擦除容器
    std::unordered_map<std::type_index, UniqueAny> resources_;
};

// ============================================================================
// 【通道存储器 ChannelStore】
// 统一管理 SPSC / SPMC 通信通道（对应机器人 CAN、内部消息总线）
// 核心机制：**通道绑定阶段锁**
// 规则：通道读写端只能在调度器指定的「绑定阶段」创建/获取，运行时禁止修改
// 目的：保证机器狗运行时通道拓扑稳定，避免动态改拓扑导致通信异常
// ============================================================================
class ChannelStore {
public:
    /**
     * @brief 进入通道绑定阶段：计数器+1
     */
    void open_binding() noexcept { channel_binding_depth_.fetch_add(1, std::memory_order_acq_rel); }

    /**
     * @brief 退出通道绑定阶段：计数器-1
     */
    void close_binding() noexcept {
        channel_binding_depth_.fetch_sub(1, std::memory_order_acq_rel);
    }

    /**
     * @brief 判断当前是否处于合法绑定阶段
     * @return true=允许获取/创建通道
     */
    [[nodiscard]] bool binding_open() const noexcept {
        return channel_binding_depth_.load(std::memory_order_acquire) != 0;
    }

    /**
     * @brief 获取指定通道存储，不存在则自动创建
     * @tparam Storage 通道类型(SpscStorage/SpmcStorage)
     * @tparam Topic 通道标签（区分同数据类型、不同业务的通道）
     * @return 通道存储裸指针
     */
    template <typename Storage, typename Topic = system::DefaultTopic>
    [[nodiscard]] Storage* get_storage() noexcept {
        ensure_binding_open(); // 校验：非绑定阶段直接报错

        using T = typename Storage::value_type;
        // 通道唯一键：数据类型 + 业务标签（Topic）
        const system::ChannelKey key{typeid(T), typeid(Topic)};

        // 根据通道类型选择对应哈希表
        auto& storage_map   = select_storage_map<Storage>();
        auto [it, inserted] = storage_map.try_emplace(key);

        // 通道首次访问：自动创建实例
        if (inserted) {
            it->second.template emplace<std::unique_ptr<Storage>>(Storage::create());
        }

        auto* ptr = it->second.template get<std::unique_ptr<Storage>>();

        // 类型校验失败，报错
        if (!ptr || !*ptr) [[unlikely]] {
            panic(
                "Channel storage type mismatch: {}@{}", detail::demangle(typeid(T).name()),
                detail::demangle(typeid(Topic).name()));
        }

        return ptr->get();
    }

private:
    // SPSC 通道集合
    std::unordered_map<system::ChannelKey, UniqueAny, system::ChannelKeyHash> spsc_storage_;
    // SPMC 通道集合（对应 CAN 广播总线）
    std::unordered_map<system::ChannelKey, UniqueAny, system::ChannelKeyHash> spmc_storage_;
    // 原子计数器：通道绑定深度，控制绑定生命周期（无锁、线程安全）
    std::atomic<std::uint32_t> channel_binding_depth_{0};

    /**
     * @brief 强制校验绑定状态，运行时非法操作直接 panic
     */
    void ensure_binding_open() const noexcept {
        if (!binding_open()) [[unlikely]] {
            panic("Channel endpoints can only be claimed during scheduler-controlled bind phases");
        }
    }

    /**
     * @brief 编译期分支选择：区分 SPSC / SPMC 存储容器
     */
    template <typename Storage>
    [[nodiscard]] auto& select_storage_map() noexcept {
        using T = typename Storage::value_type;
        if constexpr (std::is_same_v<Storage, SpscStorage<T>>) {
            return spsc_storage_;
        } else {
            return spmc_storage_;
        }
    }
};

// ============================================================================
// 【World 全局世界容器】
// 整个 Talos 调度器的**根对象**，机器狗程序的核心容器
// 职责：
// 1. 统一管理所有全局资源（传感器、配置、硬件句柄）
// 2. 统一管理所有通信通道（SPSC/SPMC，对应 CAN、内部消息）
// 3. 控制资源/通道的生命周期、读写权限、拓扑冻结
// 4. 提供对外统一的资源、通道访问接口
// ============================================================================
using namespace system;
using namespace talos::scheduler::detail;

class World {
public:
    // ==================== 资源操作接口 ====================
    /**
     * @brief 移入资源，会校验资源结构是否允许修改
     * 阶段：仅初始化阶段可用，冻结后禁止调用
     */
    template <typename T>
    void insert_resource(T&& resource) {
        using U = std::remove_cvref_t<T>;
        SPDLOG_DEBUG("register {}", demangle(typeid(T).name()));
        ensure_resource_structure_mutable<U>();
        static_cast<void>(resources_.template emplace<U>(std::forward<T>(resource)));
    }

    /**
     * @brief 原位构造全局资源
     */
    template <typename T, typename... Args>
    [[nodiscard]] T& emplace_resource(Args&&... args) {
        using U = std::remove_cvref_t<T>;
        ensure_resource_structure_mutable<U>();
        return resources_.template emplace<U>(std::forward<Args>(args)...);
    }

    /**
     * @brief 不安全插入资源：跳过结构冻结检查
     * 用途：运行时动态新增资源（调试/特殊业务），正式比赛慎用
     */
    template <typename T>
    void unsafe_insert_resource(T&& resource) {
        using U = std::remove_cvref_t<T>;
        static_cast<void>(resources_.template emplace<U>(std::forward<T>(resource)));
    }

    /**
     * @brief 不安全原位构造资源：跳过冻结检查
     */
    template <typename T, typename... Args>
    [[nodiscard]] T& unsafe_emplace_resource(Args&&... args) {
        using U = std::remove_cvref_t<T>;
        return resources_.template emplace<U>(std::forward<Args>(args)...);
    }

    /**
     * @brief 冻结资源结构：禁止再新增/删除资源
     * 业务场景：机器狗初始化完成、进入运行态后调用，保证拓扑稳定
     */
    void freeze_resource_structure() noexcept;

    /**
     * @brief 查询资源结构是否已冻结
     */
    [[nodiscard]] bool resource_structure_frozen() const noexcept;

    /**
     * @brief 冻结资源标识（扩展接口）
     */
    void freeze_resource_identity() noexcept;

    /**
     * @brief 查询资源标识是否冻结
     */
    [[nodiscard]] bool resource_identity_frozen() const noexcept;

    /**
     * @brief 判断指定资源是否存在
     */
    template <typename T>
    [[nodiscard]] bool has_resource() const noexcept {
        return resources_.template contains<T>();
    }

    /**
     * @brief 获取只读资源句柄（res<T> 轻量包装）
     */
    template <typename T>
    [[nodiscard]] res<T> get_res() const noexcept {
        return res<T>{&resources_.template get<T>()};
    }

    /**
     * @brief 获取可写资源句柄（res_mut<T> 轻量包装）
     */
    template <typename T>
    [[nodiscard]] res_mut<T> get_res_mut() noexcept {
        return res_mut<T>{&resources_.template get_mut<T>()};
    }

    // ==================== 统一组件访问入口（模板分发） ====================
    /**
     * @brief 通用获取接口：根据传入的「标记类型」自动分发到对应实现
     * C++ 模板元编程：类型标签分发，简化上层调用
     */
    template <typename ComponentT>
    [[nodiscard]] auto get() noexcept {
        return get_impl(ComponentT{});
    }

    // 快捷接口：SPSC 读写端
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_spsc_reader() noexcept {
        return get<spsc<T, Topic>>();
    }
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_spsc_writer() noexcept {
        return get<spsc_mut<T, Topic>>();
    }

    // 快捷接口：SPMC 读写端（CAN 广播、多电调场景）
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_spmc_reader() noexcept {
        return get<spmc<T, Topic>>();
    }
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_spmc_writer() noexcept {
        return get<spmc_mut<T, Topic>>();
    }

private:
    // 友元：调度器、ROS兼容发布槽位可访问私有成员
    friend class Scheduler;
    template <typename T, typename Topic>
    friend struct rclcompat::PubSlot;

    // 通道绑定阶段控制（对外封装，由调度器调用）
    void open_channel_binding() noexcept { channels_.open_binding(); }
    void close_channel_binding() noexcept { channels_.close_binding(); }
    [[nodiscard]] bool channel_binding_open() const noexcept { return channels_.binding_open(); }

    /**
     * @brief 通用通道成员获取模板
     * 利用成员指针，复用代码获取通道内部读写对象
     */
    template <typename Ret, typename Topic, typename Storage, auto MemberPtr>
    [[nodiscard]] auto get_channel_member() noexcept {
        auto* s = get_channel_storage<Storage, Topic>();
        return Ret{(s->*MemberPtr).get()};
    }

    // ---------- 各类通道/资源 具体实现分发 ----------
    /**
     * @brief SPSC 读端实现：一对一，禁止重复绑定
     */
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_impl(spsc<T, Topic>) noexcept {
        auto* s = get_channel_storage<SpscStorage<T>, Topic>();
        if (s->reader_claimed) [[unlikely]] {
            panic(
                "SPSC reader already bound: {}@{}", demangle(typeid(T).name()),
                demangle(typeid(Topic).name()));
        }
        s->reader_claimed = true;
        return spsc<T, Topic>{s->reader.get()};
    }

    /**
     * @brief SPSC 写端实现：一对一，禁止重复绑定
     */
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_impl(spsc_mut<T, Topic>) noexcept {
        auto* s = get_channel_storage<SpscStorage<T>, Topic>();
        if (s->writer_claimed) [[unlikely]] {
            panic(
                "SPSC writer already bound: {}@{}", demangle(typeid(T).name()),
                demangle(typeid(Topic).name()));
        }
        s->writer_claimed = true;
        return spsc_mut<T, Topic>{s->writer.get()};
    }

    /**
     * @brief SPMC 读端实现：支持无限多消费者（CAN 多从机）
     */
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_impl(spmc<T, Topic>) noexcept {
        auto* s = get_channel_storage<SpmcStorage<T>, Topic>();
        return spmc<T, Topic>{s->add_reader()};
    }

    /**
     * @brief SPMC 写端实现：全局唯一生产者
     */
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_impl(spmc_mut<T, Topic>) noexcept {
        return get_channel_member<
            spmc_mut<T, Topic>, Topic, SpmcStorage<T>, &SpmcStorage<T>::channel>();
    }

    // 资源句柄分发
    template <typename T>
    [[nodiscard]] auto get_impl(res<T>) noexcept { return get_res<T>(); }
    template <typename T>
    [[nodiscard]] auto get_impl(res_mut<T>) noexcept { return get_res_mut<T>(); }

    /**
     * @brief 底层接口：获取通道存储指针
     */
    template <typename Storage, typename Topic = DefaultTopic>
    [[nodiscard]] auto* get_channel_storage() noexcept {
        return channels_.template get_storage<Storage, Topic>();
    }

private:
    ResourceStore resources_;                          // 全局资源管理器
    ChannelStore channels_;                            // 通信通道管理器（CAN/内部总线）
    std::atomic<bool> resource_structure_frozen_{false}; // 原子标记：资源结构是否冻结
    std::shared_ptr<WorldLifetimeToken> lifetime_token_; // 生命周期令牌

    /**
     * @brief 校验资源结构是否可修改
     * 冻结后普通新增资源直接报错，强制保证运行态拓扑稳定
     */
    template <typename T>
    void ensure_resource_structure_mutable() const noexcept {
        if (resource_structure_frozen_.load(std::memory_order_acquire)) [[unlikely]] {
            panic(
                "Mutating resource structure for '{}' after build() breaks scheduler invariants; "
                "use unsafe_insert_resource()/unsafe_emplace_resource() only if you need the "
                "explicit escape hatch",
                demangle(typeid(T).name()));
        }
    }

public:
    /**
     * @brief 获取生命周期弱令牌
     * 外部模块用弱指针监听 World 销毁，防止野指针访问
     */
    [[nodiscard]] std::weak_ptr<WorldLifetimeToken> lifetime_token() const noexcept;
};

// ==================== 业务别名（简化机器狗业务代码） ====================
/**
 * @brief publish/subscribe 对应 SPMC 发布/订阅
 * 直接对应：CAN 总线广播、IMU/姿态数据广播、主控指令下发给多电调
 */
template <typename T, typename Topic = DefaultTopic>
using publish = spmc_mut<T, Topic>;

template <typename T, typename Topic = DefaultTopic>
using subscribe = spmc<T, Topic>;

} // namespace talos::scheduler

// ============================================================================
// 顶层命名空间快捷别名，业务层无需嵌套长命名空间
// ============================================================================
namespace talos {
template <typename T, typename Topic = scheduler::DefaultTopic>
using subscribe = scheduler::spmc<T, Topic>;

template <typename T, typename Topic = scheduler::DefaultTopic>
using publish = scheduler::spmc_mut<T, Topic>;
} // namespace talos