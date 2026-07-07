// ROS兼容层节点定义
#include "scheduler/rclcompat/node.hpp"
// 定时器系统工厂函数
#include "scheduler/rclcompat/timer_factory.hpp"

// C++23 std::expected 错误返回容器
#include <expected>

namespace talos::scheduler::rclcompat {

/**
 * @brief ROS2 rcl 兼容层 Node 节点封装
 * 模拟 ROS2 rclcpp::Node 风格，提供定时器创建接口，延迟注册系统到调度器
 * 核心机制：
 * 1. create_wall_timer 仅缓存定时器系统，不立即加入调度器
 * 2. finalize() / unsafe_finalize() 批量把缓存的系统注册进调度器
 * 3. 区分调度器运行/停止两种状态，分别调用热添加/冷添加接口
 * 4. 安全校验：调度器运行时禁止普通finalize，防止并发数据竞争
 */

/**
 * @brief 构造函数
 * @param name 节点名称字符串，移动语义
 * @param scheduler 绑定全局调度器引用（外部生命周期保证有效）
 * noexcept 无异常抛出
 */
Node::Node(std::string name, Scheduler& scheduler) noexcept
    : name_(std::move(name))
    , scheduler_(scheduler) {}

/**
 * @brief 创建wall定时器（ROS风格周期定时器）
 * @param frequency 定时器运行频率
 * @param callback 定时器触发回调函数
 *
 * 逻辑：
 * 1. 自动生成唯一定时器系统名：节点名/timer/自增下标
 * 2. 工厂生成定时器System对象
 * 3. 仅存入pending_systems_延迟缓存，**不立刻注册进调度器**
 */
void Node::create_wall_timer(const Frequency frequency, std::function<void()> callback) {
    // 拼接定时器唯一标识，用当前待注册系统数量作为自增ID
    std::string system_name = name_ + "/timer/" + std::to_string(pending_systems_.size());

    // 工厂构造定时器系统实例
    auto system = create_timer_system(std::move(system_name), frequency, std::move(callback));
    // 放入延迟注册缓存列表
    pending_systems_.push_back(std::move(system));
}

/**
 * @brief 获取节点名称只读视图
 * @return std::string_view 节点名，无拷贝开销
 */
std::string_view Node::name() const noexcept { return name_; }

/**
 * @brief 安全完成节点构建，批量注册所有缓存系统
 * 禁止调度器运行时调用，会直接panic崩溃
 * @return BuildResult 调度器构建结果（std::expected空类型，携带错误码）
 */
BuildResult Node::finalize() { return finalize_impl(false); }

/**
 * @brief 非安全版本完成构建，允许调度器运行时调用
 * @deprecated 仅应急逃生通道，优先使用finalize()
 * @return BuildResult 调度器构建结果
 */
BuildResult Node::unsafe_finalize() { return finalize_impl(true); }

/**
 * @brief 获取节点所有权注册表（组件生命周期管理）
 * @return 注册表引用
 */
OwnershipRegistry& Node::registry() noexcept { return *registry_; }

/**
 * @brief 私有实现：批量注册缓存系统到调度器核心逻辑
 * @param allow_running_finalize true=允许调度器运行时执行，false=禁止
 * @return BuildResult 成功返回空值，失败返回错误信息
 */
BuildResult Node::finalize_impl(const bool allow_running_finalize) {
    // 安全校验：调度器正在运行 且 不允许运行时finalize，直接崩溃打印提示
    if (scheduler_.is_running() && !allow_running_finalize) {
        using namespace talos::scheduler;
        panic(
            "Node '{}': finalize() while scheduler is running is unsafe; use "
            "unsafe_finalize() if you need the explicit escape hatch",
            name_);
    }

    // 记录已成功注册的系统数量，用于失败时回滚删除
    std::size_t consumed = 0;
    // 遍历全部待注册定时器系统
    for (; consumed < pending_systems_.size(); ++consumed) {
        auto& sys = pending_systems_[consumed];
        if (scheduler_.is_running()) {
            // 调度器正在运行：热添加系统（无锁并发安全接口）
            if (auto result = scheduler_.unsafe_hot_add_system(std::move(sys)); !result) {
                // 添加失败：删除 [0, consumed+1] 已处理缓存项，回滚状态
                pending_systems_.erase(
                    pending_systems_.begin(),
                    pending_systems_.begin() + static_cast<std::ptrdiff_t>(consumed + 1));
                // 返回添加错误
                return result;
            }
        } else {
            // 调度器停止状态：冷添加系统（构建阶段标准接口）
            if (auto result = scheduler_.add_system(std::move(sys)); !result) {
                // 添加失败，回滚删除已处理缓存
                pending_systems_.erase(
                    pending_systems_.begin(),
                    pending_systems_.begin() + static_cast<std::ptrdiff_t>(consumed + 1));
                // 包装错误为BuildResult返回
                return std::unexpected(BuildError{result.error()});
            }
        }
    }
    // 全部系统注册成功，清空待注册缓存
    pending_systems_.clear();

    // 调度器未运行：执行全局构建流程（分配任务、排序、初始化）
    if (!scheduler_.is_running()) {
        return scheduler_.build();
    }

    // 调度器已在运行，无需全局build，返回成功空值
    return {};
}

} // namespace talos::scheduler::rclcompat