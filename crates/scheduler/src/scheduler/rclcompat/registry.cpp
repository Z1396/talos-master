#pragma once
// 头文件保护宏，防止重复包含重定义

// 所有权注册表类定义
#include "scheduler/rclcompat/registry.hpp"
// 类型名反混淆工具
#include "scheduler/demangle.hpp"
// 全局panic崩溃工具、错误定义
#include "scheduler/error.hpp"

// spdlog 日志库，打印崩溃关键日志
#include <spdlog/spdlog.h>

// std::abort 程序终止函数
#include <cstdlib>

namespace talos::scheduler::rclcompat {

/**
 * @brief 频道所有权注册表 OwnershipRegistry
 * 作用：全局管控每一组消息类型+话题(ChannelKey)的归属权、独占发布抢占标记
 * 核心两个容器：
 * 1. owners_：记录每个ChannelKey对应的所有者节点系统名，**全局唯一绑定**
 * 2. claimed_：标记当前是否已有发布器抢占该频道，保证单发布器约束
 * 全部接口通过with_lock加锁，多线程并发安全
 */

/**
 * @brief 注册频道归属所有者，一个频道仅允许注册一次
 * @param key 频道唯一键：消息类型 + 话题标签
 * @param owner 归属节点/系统名称，移动语义
 * 逻辑：
 * 1. 加锁进入临界区
 * 2. 如果该key已存在所有者，打印致命日志并调用abort直接崩溃程序
 * 3. 无占用则写入所有者名称
 */
void OwnershipRegistry::register_owner(const system::ChannelKey key, std::string owner) noexcept {
    with_lock([&] {
        // 判断频道已被注册
        if (owners_.contains(key)) {
            // 打印CRITICAL致命日志，输出消息类型、话题、原有所有者、新冲突所有者
            SPDLOG_CRITICAL(
                "Channel {}@{} is already owned by '{}', but '{}' attempted to register",
                talos::scheduler::detail::demangle(key.type.name()),
                talos::scheduler::detail::demangle(key.topic.name()), owners_[key], owner);
            // 直接终止进程，不可恢复冲突
            std::abort();
        }
        // 写入所有者名称，移动避免拷贝
        owners_[key] = std::move(owner);
    });
}

/**
 * @brief 尝试抢占频道独占发布权
 * @param key 频道唯一键
 * @return true 抢占成功；false 已被其他发布器抢占
 * 线程安全：加锁原子检查+插入
 */
bool OwnershipRegistry::try_claim(const system::ChannelKey key) noexcept {
    return with_lock([&] {
        // 频道已被抢占，返回失败
        if (claimed_.contains(key)) {
            return false;
        }
        // 插入抢占标记，代表当前节点持有该频道唯一发布权
        claimed_.insert(key);
        return true;
    });
}

/**
 * @brief 释放频道独占发布权
 * @param key 频道唯一键
 * 移除claimed_内的抢占标记，允许其他节点后续抢占该频道
 */
void OwnershipRegistry::release_claim(const system::ChannelKey key) noexcept {
    with_lock([&] { claimed_.erase(key); });
}

/**
 * @brief 断言校验：当前频道必须存在注册所有者，否则直接panic崩溃
 * @param key 频道唯一键
 * @param caller 调用方名称，用于崩溃日志定位违规代码
 * 使用场景：读取/发布消息前校验频道合法性，禁止访问未注册的裸频道
 */
void OwnershipRegistry::assert_owner(
    const system::ChannelKey key, const std::string_view caller) const noexcept {
    with_lock([&] {
        // 查找所有者映射
        if (const auto it = owners_.find(key); it == owners_.end()) {
            using namespace talos::scheduler;
            // 调用全局panic工具，打印崩溃信息并终止程序
            panic(
                "Channel {}@{} is not registered, but '{}' attempted to access",
                talos::scheduler::detail::demangle(key.type.name()),
                talos::scheduler::detail::demangle(key.topic.name()), caller);
        }
    });
}

} // namespace talos::scheduler::rclcompat