// ===========================================================================
// world.hpp - 全局资源容器（Talos World 简化版）
//
// 设计要点：
//   1. ResourceStore 用 unordered_map<type_index, UniqueAny> 异构存储
//   2. insert_resource<T> 插入资源，get_resource<T> 取出
//   3. build() 后冻结结构：禁止新增资源（避免野指针）
//   4. 资源存储指针，系统缓存裸指针避免运行时哈希查询
// ===========================================================================
#pragma once

#include "ecs/unique_any.hpp"

#include <stdexcept>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>

namespace ecs {

class World {
public:
    World() = default;

    // 禁拷贝：World 是全局唯一根容器
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    // 插入资源：移动语义转移所有权
    template <typename T>
    void insert_resource(T&& resource) {
        if (frozen_) {
            throw std::runtime_error("World::insert_resource: structure frozen after build()");
        }
        auto type = std::type_index(typeid(std::decay_t<T>));
        store_[type] = UniqueAny(std::forward<T>(resource));
    }

    // 取出资源引用：不存在则抛异常
    template <typename T>
    [[nodiscard]] T& get_resource() {
        auto type = std::type_index(typeid(T));
        auto it = store_.find(type);
        if (it == store_.end()) {
            throw std::runtime_error("World::get_resource: resource not found");
        }
        return it->second.as<T>();
    }

    template <typename T>
    [[nodiscard]] const T& get_resource() const {
        auto type = std::type_index(typeid(T));
        auto it = store_.find(type);
        if (it == store_.end()) {
            throw std::runtime_error("World::get_resource: resource not found");
        }
        return it->second.as<T>();
    }

    // 检查资源是否存在
    template <typename T>
    [[nodiscard]] bool has_resource() const noexcept {
        return store_.find(std::type_index(typeid(T))) != store_.end();
    }

    // 冻结结构：build() 后调用，禁止再新增资源
    void freeze() noexcept { frozen_ = true; }

    [[nodiscard]] bool is_frozen() const noexcept { return frozen_; }

private:
    std::unordered_map<std::type_index, UniqueAny> store_;
    bool frozen_ = false;
};

}  // namespace ecs
