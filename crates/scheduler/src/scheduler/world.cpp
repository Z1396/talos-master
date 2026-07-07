#pragma once
// 全局资源容器 World 头文件
#include "scheduler/world.hpp"

namespace talos::scheduler {

/**
 * @brief 冻结全局资源结构，禁止后续新增/删除资源类型
 * 调度器完成第一次build/finalize后调用，防止系统缓存资源裸指针失效
 * 原子写 release 内存序：确保冻结标记对所有读取线程可见
 */
void World::freeze_resource_structure() noexcept {
    resource_structure_frozen_.store(true, std::memory_order_release);
}

/**
 * @brief 查询当前资源结构是否已冻结
 * @return true 资源结构锁定，禁止增删资源；false 可动态插入/删除资源
 * 原子读 acquire 内存序：同步可见冻结标记的写入
 */
bool World::resource_structure_frozen() const noexcept {
    return resource_structure_frozen_.load(std::memory_order_acquire);
}

/**
 * @brief 兼容别名：冻结资源身份标识，底层复用冻结结构逻辑
 * 对外提供统一语义接口，resource identity 和 resource structure 共用一套冻结标记
 */
void World::freeze_resource_identity() noexcept { freeze_resource_structure(); }

/**
 * @brief 查询资源身份是否冻结，别名调用结构冻结查询
 */
bool World::resource_identity_frozen() const noexcept { return resource_structure_frozen(); }

/**
 * @brief 获取World生命周期弱令牌
 * @return 弱智能指针 WorldLifetimeToken
 * 用途：ResourceAccessor 持有该弱令牌，检测World是否销毁，杜绝野指针访问UB
 */
std::weak_ptr<WorldLifetimeToken> World::lifetime_token() const noexcept { return lifetime_token_; }

} // namespace talos::scheduler