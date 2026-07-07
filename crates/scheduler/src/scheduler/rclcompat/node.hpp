#pragma once

// 类型名反序列化工具
#include "../demangle.hpp"
// 调度器错误定义
#include "../error.hpp"
// 全局调度器核心
#include "../scheduler.hpp"
// ECS组件基础定义
#include "../system/components.hpp"
// 系统任务基类、世界资源容器
#include "../system/system.hpp"
// 发布订阅所有权注册表
#include "registry.hpp"
// RCL兼容层系统封装基类
#include "system.hpp"
// 定时器频率常量枚举
#include "timer_constants.hpp"

// 类型擦除任意容器，存储不同类型发布槽
#include <any>
// 标准库基础类型
#include <cstddef>
// 回调函数包装
#include <functional>
// 智能指针
#include <memory>
// 字符串
#include <string>
// 只读字符串视图，无拷贝
#include <string_view>
// 哈希表存储发布槽
#include <unordered_map>
// 移动语义、pair
#include <utility>
// 动态数组延迟缓存待注册系统
#include <vector>

namespace talos::scheduler::rclcompat {
// 简化命名空间，无需重复写system::
using namespace system;
using namespace talos::scheduler::detail;

// ============================================================================
// QoS 服务质量枚举
// ============================================================================
/**
 * @brief 简化版ROS2 QoS策略枚举
 * 仅实现Volatile临时策略
 */
enum class Qos {
    Volatile, ///< 无历史缓存，仅保留最新单条消息
};

// ============================================================================
// Node: ROS2兼容层节点容器，管理发布器、订阅器、定时器、资源访问器
// ============================================================================
/**
 * @brief ROS2风格兼容节点容器
 *
 * 一个Node管理一组发布器Publisher、订阅器Subscription、定时器Timer、资源访问器ResourceAccessor，统一管理生命周期。
 * 所有创建pub/sub/timer的接口仅缓存系统，调用finalize()才批量注册进调度器Scheduler。
 *
 * ## 线程安全约束
 *
 * 1. create_publisher / create_subscription / create_wall_timer / create_resource 必须**单线程**调用，不可多线程并发创建；
 * 2. finalize() 必须在 scheduler.run() 执行前调用；
 * 3. unsafe_finalize() 实验性接口，允许调度器运行时热添加系统，存在并发风险；
 * 4. Publisher::publish() 在finalize完成后多线程安全，唤醒内部发布系统执行回调。
 *
 * ## 生命周期说明
 *
 * Publisher句柄使用共享指针引用调度器侧PubSlot，因此Publisher生命周期可超过Node本身；
 * unsafe_finalize() 批量添加系统不具备事务性，中途失败会导致部分系统已注册、部分仍挂在Node缓存。
 */
class Node {
public:
    /**
     * @brief Node构造函数
     * @param name 节点名称，作为内部系统名前缀
     * @param scheduler 绑定全局调度器外部引用，外部保证生命周期长于Node
     * noexcept 无异常抛出
     */
    explicit Node(std::string name, Scheduler& scheduler) noexcept;

    // 禁用拷贝、移动：内部持有外部引用、共享注册表，禁止转移所有权
    Node(const Node&)            = delete;
    Node& operator=(const Node&) = delete;
    Node(Node&&)                 = delete;
    Node& operator=(Node&&)      = delete;

    /**
     * @brief 创建话题发布器 Publisher
     *
     * 返回仅可移动的Publisher句柄，底层PubSlot由Node持有，共享指针跨生命周期引用。
     *
     * @tparam T 消息载荷数据类型
     * @tparam Topic 话题标签类型，默认DefaultTopic通用话题
     * @param qos 服务质量策略，当前仅Volatile，未实际实现多策略
     * @return Publisher<T, Topic> 仅移动句柄，用于publish()发送消息
     *
     * 崩溃条件：同一频道（消息类型+话题标签）重复创建发布器直接panic终止程序
     */
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] Publisher<T, Topic> create_publisher([[maybe_unused]] Qos qos = Qos::Volatile) {
        // 组合频道唯一键：消息类型typeid + 话题typeid
        const ChannelKey key{typeid(T), typeid(Topic)};
        // 拼接系统唯一名称：节点名/pub/话题名/消息类型名
        const std::string system_name = make_system_name<T, Topic>("pub");

        // 检查当前频道是否已存在发布器，重复创建直接崩溃
        if (pub_slots_.contains(key)) {
            using namespace talos::scheduler;
            panic(
                "Node '{}': Publisher for channel {}@{} already exists", name_,
                demangle(key.type.name()), demangle(key.topic.name()));
        }

        // 构造共享指针PubSlot，std::any要求内部类型可拷贝，shared_ptr满足要求
        auto slot       = std::make_shared<PubSlot<T, Topic>>();
        pub_slots_[key] = slot;

        // 所有权注册表注册当前频道归属本节点系统
        registry_->register_owner(key, system_name);

        // 尝试抢占频道所有权，抢占失败直接崩溃
        if (!registry_->try_claim(key)) {
            using namespace talos::scheduler;
            panic(
                "Node '{}': Failed to claim publisher for channel {}@{}", name_,
                demangle(key.type.name()), demangle(key.topic.name()));
        }

        // 创建ROS发布系统，存入延迟缓存，finalize阶段统一注册调度器
        pending_systems_.push_back(std::make_unique<RclPubSystem<T, Topic>>(system_name, slot));

        // 返回发布器句柄，绑定共享槽、注册表、频道键
        return Publisher<T, Topic>(slot, registry_, key);
    }

    /**
     * @brief 创建话题订阅器 Subscription
     * 注册回调函数，调度器收到对应话题消息时自动执行回调
     *
     * @tparam T 消息载荷类型
     * @tparam Topic 话题标签类型，默认DefaultTopic
     * @param callback 消息到达回调函数 void(const T&)
     * @param qos 服务质量策略，预留参数未实现
     */
    template <typename T, typename Topic = DefaultTopic>
    void create_subscription(
        std::function<void(const T&)> callback, [[maybe_unused]] Qos qos = Qos::Volatile) {
        // 生成订阅系统唯一名称
        const std::string system_name = make_system_name<T, Topic>("sub");

        // 构造订阅系统存入延迟缓存
        pending_systems_.push_back(
            std::make_unique<RclSubSystem<T, Topic>>(system_name, std::move(callback)));
    }

    /**
     * @brief 创建周期墙时钟定时器（ROS wall_timer）
     * 定时器回调运行在独立固定频率线程，不受其他任务阻塞
     *
     * @param frequency 定时器频率枚举 Hz_1 ~ Hz_1000
     * @param callback 每周期触发回调，值接收支持移动捕获
     *
     * 使用示例见注释文档；回调采用值传递，方便外部移动闭包，大捕获列表建议内部引用捕获
     */
    void create_wall_timer(Frequency frequency, std::function<void()> callback);

    /**
     * @brief 创建世界资源访问器 ResourceAccessor
     * 返回可拷贝句柄，在定时器/订阅回调中安全读取全局World资源
     *
     * @tparam T 资源类型
     * @return ResourceAccessor<T> 资源只读访问句柄
     *
     * 约束：调度器第一次build/finalize成功后，禁止结构性修改资源；仅允许修改资源内部值
     * 使用示例见文档注释，用于检测、TF缓存等全局资源读取
     */
    template <typename T>
    [[nodiscard]] ResourceAccessor<T> create_resource() noexcept {
        auto& world = scheduler_.world();
        return ResourceAccessor<T>(&world, world.lifetime_token());
    }

    /**
     * @brief 判断全局World是否存在指定类型资源
     * @tparam T 资源类型
     * @return true 资源已存在，false 不存在
     */
    template <typename T>
    [[nodiscard]] bool has_resource() const noexcept {
        return scheduler_.world().has_resource<T>();
    }

    /**
     * @brief 向全局World插入资源（标准安全接口）
     * @tparam T 资源类型
     * @param resource 待插入资源，右值移动语义
     *
     * 约束：调度器首次构建完成后禁止结构性增删资源，系统会拒绝，防止任务缓存裸指针失效
     */
    template <typename T>
    void insert_resource(T&& resource) {
        scheduler_.world().insert_resource(std::forward<T>(resource));
    }

    /**
     * @brief 实验性资源插入逃逸接口，绕过构建后资源冻结限制
     * 调用者自行保证同步、指针稳定，存在数据竞争风险
     * @tparam T 资源类型
     * @param resource 待插入资源，移动语义
     */
    template <typename T>
    void unsafe_insert_resource(T&& resource) {
        scheduler_.world().unsafe_insert_resource(std::forward<T>(resource));
    }

    /**
     * @brief 获取节点名称只读视图
     * @return std::string_view 节点名，无内存拷贝
     */
    [[nodiscard]] std::string_view name() const noexcept;

    /**
     * @brief 标准安全完成节点构建，批量注册所有缓存系统到调度器
     *
     * 调用时机：所有pub/sub/timer创建完成、scheduler.run()之前
     * @return BuildResult 成功返回空，失败携带错误信息
     *
     * 非事务特性：中途添加失败时，已处理系统已注册进调度器，未处理系统保留在Node缓存，可重试
     * 线程约束：单线程调用；调度器运行时调用直接panic崩溃，提示使用unsafe_finalize
     */
    [[nodiscard]] BuildResult finalize();

    /**
     * @brief 实验性热更新构建接口，允许调度器运行时添加系统
     * 底层调用 Scheduler::unsafe_hot_add_system 动态追加任务，存在并发风险，仅特殊场景使用
     * @return BuildResult 批量添加结果
     */
    [[nodiscard]] BuildResult unsafe_finalize();

    /**
     * @brief 获取发布订阅所有权注册表
     * @return OwnershipRegistry& 注册表引用
     */
    [[nodiscard]] OwnershipRegistry& registry() noexcept;

    /**
     * @brief 高级底层接口：获取原始PubSlot裸指针
     * @tparam T 消息类型
     * @tparam Topic 话题标签
     * @return PubSlot<T, Topic>* 存在返回指针，不存在返回nullptr
     */
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] PubSlot<T, Topic>* get_pub_slot() noexcept {
        const ChannelKey key{typeid(T), typeid(Topic)};
        const auto it = pub_slots_.find(key);
        if (it == pub_slots_.end()) {
            return nullptr;
        }
        // any提取共享指针，返回内部裸指针
        return std::any_cast<std::shared_ptr<PubSlot<T, Topic>>>(it->second).get();
    }

private:
    // 节点名称
    std::string name_;
    // 外部全局调度器引用
    Scheduler& scheduler_;
    // 频道所有权注册表，共享指针多句柄共用
    std::shared_ptr<OwnershipRegistry> registry_ = std::make_shared<OwnershipRegistry>();

    // 发布槽哈希表：键=频道唯一标识，值=any擦除类型共享指针PubSlot
    std::unordered_map<ChannelKey, std::any, ChannelKeyHash> pub_slots_;

    // 延迟待注册系统缓存：pub/sub/timer全部存入此处，finalize统一注册调度器
    std::vector<std::unique_ptr<SystemBase>> pending_systems_;

    // finalize内部实现，区分是否允许调度器运行时添加
    [[nodiscard]] BuildResult finalize_impl(bool allow_running_finalize);

    /**
     * @brief 辅助函数：拼接系统唯一标识字符串
     * @tparam T 消息/资源类型
     * @tparam Topic 话题标签
     * @param kind 类型标识 "pub"/"sub"
     * @return 完整系统名 node/kind/话题名/消息类型名
     */
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] std::string make_system_name(std::string_view kind) const {
        return name_ + "/" + std::string(kind) + "/" + demangle(typeid(Topic).name()) + "/"
             + demangle(typeid(T).name());
    }
};

} // namespace talos::scheduler::rclcompat

// 别名命名空间，简化上层业务导入
namespace talos {
namespace rclcompat = scheduler::rclcompat;
} // namespace talos