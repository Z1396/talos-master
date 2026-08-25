// ===========================================================================
// world.hpp - 全局资源容器（Talos World 简化版）
//
// 核心设计：
//   1. unordered_map<type_index, UniqueAny> 异构存储，按类型索引
//   2. insert_resource<T> 插入，get_resource<T> 取出（强类型引用）
//   3. freeze() 冻结结构：build() 后禁止新增资源
// ===========================================================================

#pragma once                  // 防止头文件被重复包含

#include "ecs/unique_any.hpp" // 引入 UniqueAny（类型擦除容器），这是 World 存储的基础

#include <stdexcept>          // std::runtime_error - 运行时异常
#include <typeindex>          // std::type_index - 类型的唯一标识符（基于 typeid）
#include <unordered_map>      // 哈希表 - 用类型作为 key，存储任意类型的资源
#include <utility>            // std::forward, std::move - 完美转发和移动语义

namespace ecs {               // 所有代码放在 ecs 命名空间

// ===========================================================================
// World - 全局资源容器
//
// 在 ECS 中，World 是"世界的根"，包含：
//   - 所有实体（Entity，通常用整数 ID 表示）
//   - 所有组件（Component，按类型组织）
//   - 所有资源（Resource，这里是重点，如配置、渲染器、输入状态等）
//
// 本简化版 World 只实现了"资源"（Resource）管理，实体和组件管理在其他文件中
// ===========================================================================
class World {
public:
    // 默认构造函数（无特殊初始化）
    World() = default;

    // 禁止拷贝：World 应该全局唯一，拷贝无意义且昂贵
    World(const World&)            = delete;
    World& operator=(const World&) = delete;

    // =========================================================================
    // insert_resource - 插入一个资源
    // 模板参数 T：资源类型（由传入参数自动推导）
    // 参数：T&& resource - 万能引用，支持左值和右值
    //
    // 工作原理：
    //   1. 检查是否已冻结（已冻结则不能再插入）
    //   2. 取类型 T 的 std::type_index 作为 key
    //   3. 用 std::forward 完美转发资源，存入 UniqueAny（类型擦除）
    // =========================================================================
    template <typename T>
    void insert_resource(T&& resource) {
        // 冻结检查：一旦 build() 后，资源结构不能再改变
        // 这保证了依赖分析的结果是有效的
        if (frozen_) {
            throw std::runtime_error("World frozen after build()");
        }

        // 计算资源的类型索引
        // std::decay_t<T> 会移除引用和 cv 限定符（const/volatile），得到原始类型
        // 比如：int& → int, const int → int
        auto type_idx = std::type_index(typeid(std::decay_t<T>));

        // 用 UniqueAny 擦除类型，存入哈希表
        // std::forward<T>(resource) 保持左右值属性（完美转发）
        // 如果是右值，UniqueAny 会移动它；如果是左值，会拷贝它
        store_[type_idx] = UniqueAny(std::forward<T>(resource));
    }

    // =========================================================================
    // get_resource - 取出资源引用（非 const 版本）
    // 模板参数 T：想要的资源类型（必须显式指定，如 world.get_resource<Config>()）
    // 返回：T& 强类型引用（如果类型匹配）
    // 异常：找不到类型时抛 std::runtime_error
    // =========================================================================
    template <typename T>
    [[nodiscard]] T& get_resource() { // [[nodiscard]] 警告用户不要忽略返回值
        // 用类型 T 的 type_index 作为 key 查找
        auto it = store_.find(std::type_index(typeid(T)));

        // 如果找不到，抛出异常
        if (it == store_.end()) {
            throw std::runtime_error("resource not found");
        }

        // 找到后，调用 UniqueAny::as<T>() 取出强类型引用
        // 这里会再次校验类型匹配（防止存储的类型和请求的类型不一致）
        return it->second.as<T>();
    }

    // =========================================================================
    // get_resource - 取出资源引用（const 版本）
    // 当 World 本身是 const 时调用，只能读取不能修改
    // =========================================================================
    template <typename T>
    [[nodiscard]] const T& get_resource() const {
        auto it = store_.find(std::type_index(typeid(T)));
        if (it == store_.end()) {
            throw std::runtime_error("resource not found");
        }
        return it->second.as<T>(); // 调用 const 版本的 UniqueAny::as()
    }

    // =========================================================================
    // freeze - 冻结 World
    // 调用后：is_frozen() 返回 true，且 insert_resource 会抛出异常
    // 用途：在 Scheduler::build() 中调用，锁定资源结构
    // noexcept：保证不会抛出异常（因为只是设一个 bool 标志）
    // =========================================================================
    void freeze() noexcept { frozen_ = true; }

    // =========================================================================
    // is_frozen - 检查是否已冻结
    // noexcept：只读操作不会抛异常
    // =========================================================================
    [[nodiscard]] bool is_frozen() const noexcept { return frozen_; }

private:
    // =========================================================================
    // store_ - 核心存储结构
    // 键（key）：std::type_index - 类型的唯一标识
    // 值（value）：UniqueAny - 类型擦除容器，可以存储任意类型
    //
    // 举例：
    //   store_[typeid(int)] = UniqueAny(42)
    //   store_[typeid(Config)] = UniqueAny(Config{...})
    //   store_[typeid(Renderer)] = UniqueAny(Renderer{...})
    // =========================================================================
    std::unordered_map<std::type_index, UniqueAny> store_;

    // frozen_ - 冻结标志
    // false：可以插入新资源
    // true：禁止插入新资源（但可以读取已有资源）
    bool frozen_ = false;
};

} // namespace ecs