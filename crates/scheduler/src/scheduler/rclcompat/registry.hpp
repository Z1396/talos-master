#pragma once
// 头文件保护宏，防止重复包含重定义

// 系统元数据、ChannelKey 频道唯一键定义
#include "../system/system_meta.hpp"

// C++20 概念约束 std::invocable
#include <concepts>
// 有序映射：存储频道-所有者名称
#include <map>
// 互斥锁，多线程同步保护共享容器
#include <mutex>
// 有序集合：标记已抢占发布权的频道
#include <set>
// 字符串存储所有者名称
#include <string>
// 只读字符串视图，无拷贝开销
#include <string_view>
// 类型特征，判断可调用对象是否noexcept
#include <type_traits>

namespace talos::scheduler::rclcompat {

// ============================================================================
// OwnershipRegistry: 频道所有权全局注册表
// ============================================================================
/**
 * @brief 频道所有权管控注册表
 *
 * 运行时强制约束：**每个频道仅允许单个发布器**，管控频道归属、发布句柄抢占/释放生命周期
 * 所有接口互斥锁保护，多线程并发安全，用于ROS兼容层pub/sub生命周期校验、冲突检测
 */
class OwnershipRegistry {
public:
    /**
     * @brief 注册频道归属所有者系统名
     * @param key 频道唯一标识（消息类型+话题标签二元键 ChannelKey）
     * @param owner 拥有该频道的系统/节点名称
     *
     * 线程安全：内部互斥锁串行化读写，支持多线程并发调用
     * 业务规则：同一频道不可重复注册所有者，重复注册会触发内部崩溃日志终止程序
     */
    void register_owner(system::ChannelKey key, std::string owner) noexcept;

    /**
     * @brief 尝试抢占当前频道的独占发布句柄
     * 约束：一个频道同一时刻仅能被抢占一次，运行时强制Publisher仅可移动、不可拷贝
     * @param key 频道唯一标识
     * @return true 抢占成功；false 该频道已被其他发布器抢占
     * 线程安全：互斥锁保护集合读写
     */
    [[nodiscard]] bool try_claim(system::ChannelKey key) noexcept;

    /**
     * @brief 释放频道独占发布抢占标记
     * 调用时机：Publisher句柄销毁、移动转移所有权时执行
     * @param key 频道唯一标识
     * 释放后其他发布器可重新调用try_claim抢占该频道
     */
    void release_claim(system::ChannelKey key) noexcept;

    /**
     * @brief 断言校验：该频道必须存在注册所有者，否则直接崩溃退出
     * @param key 频道唯一标识
     * @param caller 调用方名称，崩溃日志用于定位违规调用代码
     * 崩溃场景：业务代码访问从未创建Publisher的非法频道
     * const修饰：只读查询，不修改内部容器
     */
    void assert_owner(system::ChannelKey key, std::string_view caller) const noexcept;

private:
    // 可变互斥锁，const成员函数也可上锁（只读查询仍需同步）
    mutable std::mutex lock_;
    // 有序映射：<频道键, 所有者系统名称>，全局记录每个频道归属节点
    std::map<system::ChannelKey, std::string> owners_;
    // 有序集合：存储当前已抢占发布权的频道，实现单发布器互斥约束
    std::set<system::ChannelKey> claimed_;

    /**
     * @brief 内部辅助模板：自动加锁执行回调函数，消除重复锁模板代码
     * @tparam F 可调用对象，满足std::invocable概念（无入参）
     * @param func 临界区执行逻辑lambda/函数
     * @return 回调函数返回值，类型自动推导
     * noexcept 传递：若传入函数无异常抛出，则整体无异常
     * 实现：std::lock_guard RAII自动上锁，作用域结束自动解锁
     */
    template <std::invocable F>
    auto with_lock(F&& func) const noexcept(std::is_nothrow_invocable_v<F>) -> decltype(func()) {
        std::lock_guard lock(lock_);
        return func();
    }
};

} // namespace talos::scheduler::rclcompat