#pragma once

// 基础工具头文件
#include "../demangle.hpp"
#include "../error.hpp"
#include "../system/system.hpp"
// 发布订阅所有权注册表
#include "registry.hpp"
// SystemMeta 流式建造器
#include "system_builder.hpp"

// 标准库
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace talos::scheduler::rclcompat {

// 全局命名别名，简化书写
using system::ChannelKey;
using system::DefaultTopic;
using system::ExternalComputeSource;
using system::fixed_rate;
using system::pool_compute;
using system::res;
using system::res_mut;
using system::spmc;
using system::spmc_mut;
using system::SystemBase;
using system::SystemMeta;

// 前置声明：订阅系统基类
class RclSubSystemBase;

/**
 * @brief 线程本地回调上下文，追踪当前正在执行的订阅回调
 *
 * 核心作用：约束一个Publisher仅能被**同一个订阅回调**调用publish，禁止跨回调混用发布器
 *
 * ## 执行流程示意图
 * ```
 * RclSubSystem::run()
 *   ├─ CallbackContextScope scope(this)  // 将当前系统写入线程本地上下文
 *   ├─ callback_(*msg)  ─────────────────┐
 *   │   └─ publisher.publish(msg)        │
 *   │       ├─ 首次调用：绑定当前回调为该发布器唯一所有者
 *   │       └─ 后续调用：校验调用方与所有者一致，不一致直接崩溃
 *   └─ ~CallbackContextScope()  // 析构恢复上下文，防止污染下一次回调
 * ```
 */
struct CallbackContext {
    // 当前正在执行的订阅系统指针
    RclSubSystemBase* current_system = nullptr;
};

/// 全局线程本地回调上下文：每个CPU线程拥有独立实例，无锁并发安全
extern thread_local CallbackContext g_callback_context;

/**
 * @brief RAII 作用域守卫，自动设置/恢复线程本地回调上下文
 * 离开作用域自动回滚上下文，异常安全，杜绝上下文残留污染
 */
struct CallbackContextScope {
    // 绑定当前线程独有的上下文实例
    CallbackContext* ctx_;
    // 进入作用域前保存的旧系统指针，析构用于恢复
    RclSubSystemBase* prev_system_;

    /**
     * @brief 构造：切换当前线程上下文为传入的订阅系统
     * @param current 正在执行回调的RclSubSystemBase
     */
    explicit CallbackContextScope(RclSubSystemBase* current) noexcept;

    /**
     * @brief 析构：RAII自动恢复上下文为进入作用域前的值
     */
    ~CallbackContextScope() noexcept;

    // 禁用拷贝、移动：作用域守卫不允许转移所有权
    CallbackContextScope(const CallbackContextScope&)            = delete;
    CallbackContextScope& operator=(const CallbackContextScope&) = delete;
};

/**
 * @brief 订阅系统公共基类 RclSubSystemBase
 * 统一封装系统元数据、元数据静态构建工具，供发布/订阅系统继承
 */
class RclSubSystemBase : public SystemBase {
public:
    // 虚析构，支持多态销毁派生类
    ~RclSubSystemBase() override = default;

    /**
     * @brief 获取系统完整元数据（调度策略、通道、话题类型）
     * @return 只读SystemMeta引用
     */
    const SystemMeta& meta() const noexcept override;

protected:
    /**
     * @brief 静态工具：统一构建SystemMeta，消除发布/订阅系统重复模板代码
     * @tparam Policy 调度执行策略（默认pool_compute线程池）
     * @param name 系统唯一名称
     * @param config_fn 回调函数，用于链式添加读写通道
     * @return 组装完成的SystemMeta对象
     */
    template <typename Policy = pool_compute>
    [[nodiscard]] static auto build_meta(std::string name, auto&& config_fn) noexcept
        -> SystemMeta {
        SystemMetaBuilder builder(std::move(name));
        // 设置执行策略
        builder.policy(system::make_policy_info<Policy>());
        // 执行通道配置回调（add_spmc_writer/add_spmc_reader）
        config_fn(builder);
        // 移动返回元数据
        return builder.build();
    }

    // 存储当前系统完整元数据
    SystemMeta meta_;
};

// ============================================================================
// PubSlot：发布器底层状态槽，由Node持有，多共享指针引用
// ============================================================================
/**
 * @brief 发布器底层数据槽 PubSlot
 * 归属Node管理，Publisher句柄、RclPubSystem共享引用；
 * 保存SPMC写入通道、绑定状态、待写标记、回调所有者约束等全部发布状态
 *
 * @tparam T 消息载荷类型
 * @tparam Topic 话题标签类型（默认DefaultTopic通用话题）
 */
template <typename T, typename Topic = DefaultTopic>
struct PubSlot {
    // SPMC多生产者多消费者写入通道
    std::optional<spmc_mut<T, Topic>> writer;
    // 是否完成World资源绑定，原子布尔
    std::atomic<bool> bound{false};
    // 是否存在待发送消息，等待RclPubSystem::run()消费
    std::atomic<bool> pending_write{false};
    // 调度器就绪掩码指针，用于外部唤醒系统
    std::atomic<std::uint64_t>* ready_systems = nullptr;
    // 当前系统在位掩码bit位
    std::uint64_t ready_bit                   = 0;

    // 回调所有权约束：当前独占该发布器的订阅系统
    std::atomic<RclSubSystemBase*> owner_callback{nullptr};
    // 是否完成自动绑定回调所有者标记
    std::atomic<bool> auto_bound{false};

    /**
     * @brief 将发布槽绑定到全局World资源容器
     * 由RclPubSystem::bind()调用，打开通道绑定并获取spmc_mut写入器
     * @param world 全局资源World
     */
    void bind(World& world) noexcept {
        world.open_channel_binding();
        // 获取对应话题的SPMC写入通道
        writer = world.get<spmc_mut<T, Topic>>();
        world.close_channel_binding();
        // 标记绑定完成，release序保证写入器对其他线程可见
        bound.store(true, std::memory_order_release);
    }

    /**
     * @brief 绑定外部就绪掩码，用于publish后唤醒调度系统
     * @param ready_mask 全局就绪位掩码数组
     * @param system_index 当前系统下标，计算掩码bit
     */
    void bind_ready_target(
        std::atomic<std::uint64_t>* ready_mask, const std::size_t system_index) noexcept {
        ready_systems = ready_mask;
        ready_bit     = 1ULL << system_index;
    }

    /**
     * @brief 执行消息发布，写入SPMC通道并标记待写、置位就绪掩码
     * @param msg 待发送消息，移动语义避免拷贝
     */
    void publish(T msg) noexcept {
        // 快速路径：已绑定通道
        if (bound.load(std::memory_order_acquire)) [[likely]] {
            // 写入SPMC共享通道
            writer->write(std::move(msg));
            // 标记存在待消费消息
            pending_write.store(true, std::memory_order_release);
            // 存在就绪掩码，置位对应bit唤醒调度器
            if (ready_systems != nullptr) [[likely]] {
                ready_systems->fetch_or(ready_bit, std::memory_order_release);
            }
        }
    }

    /**
     * @brief 判断发布槽是否完成绑定，可执行publish
     * @return true 已绑定World，通道可用
     */
    [[nodiscard]] bool ready() const noexcept { return bound.load(std::memory_order_acquire); }
};

// ============================================================================
// Publisher：仅移动型发布器句柄，上层业务使用
// ============================================================================
/**
 * @brief 仅移动语义发布器句柄 Publisher
 * 轻量化上层句柄，仅共享PubSlot底层状态；禁止拷贝，编译期强制单回调独占约束
 * Node销毁后，只要存在Publisher句柄，PubSlot仍有效，支持句柄生命周期超过Node
 *
 * @tparam T 消息类型
 * @tparam Topic 话题标签
 *
 * 使用示例
 * ```cpp
 * auto pub = node.create_publisher<ImageFrame, CameraTag>();
 * pub.publish(ImageFrame{});
 * // 仅允许移动，禁止拷贝
 * auto pub2 = std::move(pub);
 * pub2.publish(ImageFrame{});
 * ```
 */
template <typename T, typename Topic = DefaultTopic>
class Publisher {
public:
    // 默认空构造，生成无效句柄
    Publisher() noexcept = default;

    /**
     * @brief 构造发布器句柄，绑定底层槽、所有权注册表、频道唯一键
     * @param slot PubSlot共享智能指针
     * @param registry 频道所有权注册表共享指针
     * @param key 频道唯一键（消息类型+话题）
     */
    Publisher(
        std::shared_ptr<PubSlot<T, Topic>> slot, std::shared_ptr<OwnershipRegistry> registry,
        const ChannelKey key) noexcept
        : slot_(slot)
        , registry_(registry)
        , key_(key) {}

    // 禁用拷贝构造、拷贝赋值，仅允许移动
    Publisher(const Publisher&)            = delete;
    Publisher& operator=(const Publisher&) = delete;

    /**
     * @brief 移动构造，转移底层PubSlot所有权
     */
    Publisher(Publisher&& other) noexcept
        : slot_(std::move(other.slot_))
        , registry_(std::move(other.registry_))
        , key_(other.key_) {
        // 源句柄置空，变为无效
        other.slot_.reset();
        other.registry_.reset();
    }

    /**
     * @brief 移动赋值，转移底层资源，释放当前占用的频道抢占标记
     */
    Publisher& operator=(Publisher&& other) noexcept {
        if (this != &other) {
            // 当前持有有效槽，释放频道抢占
            if (slot_ && registry_) {
                registry_->release_claim(key_);
            }
            // 转移资源
            slot_     = std::move(other.slot_);
            registry_ = std::move(other.registry_);
            key_      = other.key_;
            // 源句柄置空
            other.slot_.reset();
            other.registry_.reset();
        }
        return *this;
    }

    /**
     * @brief 析构：释放频道独占抢占标记，允许其他回调抢占该发布器
     */
    ~Publisher() noexcept {
        if (slot_ && registry_) {
            registry_->release_claim(key_);
        }
    }

    /**
     * @brief 对外发布消息核心接口
     * 自动校验回调所有权：一个发布器仅允许被首次调用的订阅回调使用，跨回调调用直接panic崩溃
     * @param msg 待发送消息，移动语义
     * 崩溃场景：句柄已被移动失效 / 跨回调混用发布器
     */
    void publish(T msg) noexcept {
        // 校验句柄有效性，移动后空槽直接崩溃
        if (!slot_) [[unlikely]] {
            panic("Publisher::publish() called on invalid (moved-from) publisher");
        }

        // 获取当前线程正在执行的订阅系统
        auto* current_callback = g_callback_context.current_system;

        // 崩溃辅助lambda：打印所有权冲突日志并终止程序
        auto abort_ownership_error = [&](RclSubSystemBase* owner, RclSubSystemBase* used_by) {
            panic(
                "Publisher {}@{} owned by callback '{}', used by '{}'",
                scheduler::detail::demangle(typeid(T).name()),
                scheduler::detail::demangle(typeid(Topic).name()),
                owner ? owner->meta().name.c_str() : "<unknown>",
                used_by ? used_by->meta().name.c_str() : "<unknown>");
        };

        // 场景1：存在当前执行回调，且尚未自动绑定所有者
        if (current_callback && !slot_->auto_bound.load(std::memory_order_acquire)) {
            RclSubSystemBase* expected = nullptr;
            // CAS抢占所有者标记，绑定当前回调为唯一所有者
            if (slot_->owner_callback.compare_exchange_strong(
                    expected, current_callback, std::memory_order_release,
                    std::memory_order_acquire)) {
                slot_->auto_bound.store(true, std::memory_order_release);
            } else if (expected != current_callback) {
                // 已有其他回调绑定，所有权冲突崩溃
                abort_ownership_error(expected, current_callback);
            }
        } else if (current_callback) {
            // 场景2：已绑定所有者，校验当前回调与所有者一致
            auto* owner = slot_->owner_callback.load(std::memory_order_acquire);
            if (owner && owner != current_callback) {
                abort_ownership_error(owner, current_callback);
            }
        }

        // 所有权校验通过，底层执行发布
        slot_->publish(std::move(msg));
    }

    /**
     * @brief 判断句柄是否有效（未被移动清空）
     * @return true 持有有效PubSlot
     */
    [[nodiscard]] bool valid() const noexcept { return slot_ != nullptr; }

    /**
     * @brief 判断底层发布槽是否绑定World，可正常发布消息
     * @return true 通道绑定完成
     */
    [[nodiscard]] bool ready() const noexcept { return slot_ && slot_->ready(); }

private:
    // 底层发布槽共享智能指针
    std::shared_ptr<PubSlot<T, Topic>> slot_{};
    // 频道所有权注册表共享指针
    std::shared_ptr<OwnershipRegistry> registry_{};
    // 当前发布器对应频道唯一键
    ChannelKey key_{typeid(void), typeid(void)};
};

/**
 * @brief 发布系统 RclPubSystem：调度图中可见的外部计算源
 * 业务Publisher调用publish仅写入内存标记，本系统在调度循环中消费待写数据，推送至下游订阅通道
 * 单图唯一写入源，外部publish通过原子位唤醒该系统执行run()
 *
 * @tparam T 消息类型
 * @tparam Topic 话题标签
 * @tparam Policy 调度策略（默认线程池pool_compute）
 */
template <typename T, typename Topic = DefaultTopic, typename Policy = pool_compute>
class RclPubSystem
    : public RclSubSystemBase
    , public ExternalComputeSource {
public:
    /**
     * @brief 构造发布系统，自动生成系统元数据
     * @param name 系统名称
     * @param slot 共享PubSlot底层状态
     */
    explicit RclPubSystem(std::string name, std::shared_ptr<PubSlot<T, Topic>> slot) noexcept
        : slot_(slot) {
        // 调用基类静态工具构建元数据，添加SPMC写通道
        meta_ = build_meta<Policy>(std::move(name), [](auto& builder) {
            builder.add_spmc_writer(typeid(T), typeid(Topic));
        });
    }

    /**
     * @brief 调度器初始化绑定World资源，获取SPMC写入通道
     */
    void bind(World& world) noexcept override { slot_->bind(world); }

    /**
     * @brief RTTI多态转换：返回外部计算源基类指针
     */
    ExternalComputeSource* as_external_compute() noexcept override { return this; }

    /**
     * @brief 绑定全局就绪掩码，publish时置位bit唤醒调度器执行run
     * @param ready_systems 全局就绪原子掩码数组
     * @param system_index 当前系统下标
     */
    void bind_external_ready_slot(
        std::atomic<std::uint64_t>* ready_systems,
        const std::size_t system_index) noexcept override {
        slot_->bind_ready_target(ready_systems, system_index);
    }

    /**
     * @brief 调度周期执行函数，消费pending_write标记
     * 原子交换清空待写标记，返回true代表存在消息需要处理
     */
    bool run([[maybe_unused]] World& world) noexcept override {
        // 原子交换取出pending_write并置false，acq_rel同步读写
        return slot_->pending_write.exchange(false, std::memory_order_acq_rel);
    }

private:
    std::shared_ptr<PubSlot<T, Topic>> slot_;
};

/**
 * @brief 订阅系统 RclSubSystem：监听SPMC通道，执行业务回调
 * 调度周期读取通道新消息，RAII设置线程本地回调上下文，校验Publisher所有权
 *
 * @tparam T 消息类型
 * @tparam Topic 话题标签
 * @tparam Policy 调度策略（默认线程池pool_compute）
 */
template <typename T, typename Topic = DefaultTopic, typename Policy = pool_compute>
class RclSubSystem : public RclSubSystemBase {
public:
    // 回调函数类型：接收常量消息引用
    using Callback = std::function<void(const T&)>;

    /**
     * @brief 构造订阅系统，生成元数据、保存业务回调
     * @param name 系统名称
     * @param callback 消息到达回调
     */
    explicit RclSubSystem(std::string name, Callback callback) noexcept
        : callback_(std::move(callback)) {
        // 构建元数据，添加SPMC读通道
        meta_ = build_meta<Policy>(std::move(name), [](auto& builder) {
            builder.add_spmc_reader(typeid(T), typeid(Topic));
        });
    }

    /**
     * @brief 绑定World，获取SPMC读取通道
     */
    void bind(World& world) noexcept override { reader_ = world.get<spmc<T, Topic>>(); }

    /**
     * @brief 调度周期执行函数：读取新消息并执行业务回调
     * 读取消息时自动创建CallbackContextScope，标记当前回调上下文
     * @return false 无持续任务，单次消费
     */
    bool run([[maybe_unused]] World& world) noexcept override {
        // 无新消息直接返回
        if (!reader_.has_new()) {
            return false;
        }

        auto msg = reader_.read();
        if (msg) {
            // RAII 切换线程本地回调上下文，publish所有权校验依赖该标记
            CallbackContextScope scope(this);
            callback_(*msg);
        }

        return false;
    }

private:
    // 用户业务回调
    Callback callback_;
    // SPMC多消费者读取通道
    spmc<T, Topic> reader_;
};

// ============================================================================
// ResourceAccessor：全局World资源只读访问代理句柄
// ============================================================================
/**
 * @brief 全局资源访问代理句柄 ResourceAccessor
 * 可拷贝、可作为类成员存储，在订阅/定时器回调中安全读取全局World资源
 * 弱生命周期标记校验，World销毁后访问直接崩溃，杜绝野指针UB
 *
 * @tparam T 资源类型
 *
 * 使用示例
 * ```cpp
 * class Detector {
 *     ResourceAccessor<TfBuffer> tf_buffer_;
 *     Publisher<Detection> det_pub_;
 *
 *     void init(Node& node) {
 *         tf_buffer_ = node.create_resource<TfBuffer>();
 *         det_pub_ = node.create_publisher<Detection>();
 *
 *         node.create_subscription<Image>([this](const Image& img) {
 *             auto tf = tf_buffer_.get();  // 读取全局TF缓存资源
 *             det_pub_.publish(process(img, tf));
 *         });
 *     }
 * };
 * ```
 */
template <typename T>
class ResourceAccessor {
public:
    // 默认空构造，无效句柄
    ResourceAccessor() noexcept = default;

    /**
     * @brief 构造资源访问器
     * @param world 全局World裸指针（非持有，依赖生命周期令牌校验）
     * @param lifetime 弱生命周期令牌，检测World是否已销毁
     */
    explicit ResourceAccessor(World* world, std::weak_ptr<WorldLifetimeToken> lifetime) noexcept
        : world_(world)
        , lifetime_(std::move(lifetime)) {}

    /**
     * @brief 获取资源只读访问句柄 res<T>
     * 多线程并发安全，资源结构初始化完成后类型稳定
     * @return 只读资源代理
     */
    [[nodiscard]] res<T> get() const noexcept { return validate_world()->template get_res<T>(); }

    /**
     * @brief 获取资源读写访问句柄 res_mut<T>
     * 多线程可并发调用，但写入同步由业务自行保证
     * @return 读写资源代理
     */
    [[nodiscard]] res_mut<T> get_mut() const noexcept {
        return validate_world()->template get_res_mut<T>();
    }

    /**
     * @brief 判断访问器是否有效（World未销毁、绑定合法）
     * @return true 可正常访问资源
     */
    [[nodiscard]] bool valid() const noexcept { return world_ != nullptr && !lifetime_.expired(); }

    // 允许拷贝：多个访问器可绑定同一个World资源
    ResourceAccessor(const ResourceAccessor&)            = default;
    ResourceAccessor& operator=(const ResourceAccessor&) = default;

private:
    // 全局World裸指针
    World* world_ = nullptr;
    // 弱生命周期令牌，检测World销毁
    std::weak_ptr<WorldLifetimeToken> lifetime_;

    /**
     * @brief 内部校验World有效性，失效直接panic崩溃
     * @return 合法World指针
     */
    [[nodiscard]] World* validate_world() const noexcept {
        if (!world_ || lifetime_.expired()) [[unlikely]] {
            panic("ResourceAccessor::get() called on invalid accessor");
        }
        return world_;
    }
};

} // namespace talos::scheduler::rclcompat