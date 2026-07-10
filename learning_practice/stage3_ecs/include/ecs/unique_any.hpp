// ===========================================================================
// unique_any.hpp - 类型擦除容器（Talos UniqueAny 简化版）
//
// 设计要点：
//   1. 内部用 Concept/Model 模式擦除类型
//   2. 仅移动、禁拷贝（适配通道、硬件句柄等不可拷贝资源）
//   3. as<T>() 取出强类型引用，类型不匹配调用 panic
// ===========================================================================
#pragma once

#include <memory>
#include <stdexcept>
#include <typeindex>
#include <type_traits>
#include <utility>

namespace ecs {

// 类型擦除接口：内部 Concept 基类
class UniqueAny {
public:
    UniqueAny() = default;
    ~UniqueAny() = default;

    // 仅移动语义：禁用拷贝（资源独占）
    UniqueAny(const UniqueAny&) = delete;
    UniqueAny& operator=(const UniqueAny&) = delete;
    UniqueAny(UniqueAny&&) noexcept = default;
    UniqueAny& operator=(UniqueAny&&) noexcept = default;

    // 构造：传入任意类型，用 Model<Derived> 擦除
    template <typename T>
        requires(!std::same_as<std::decay_t<T>, UniqueAny>)
    explicit UniqueAny(T&& value)
        : ptr_(std::make_unique<Model<std::decay_t<T>>>(std::forward<T>(value)))
        , type_(std::type_index(typeid(std::decay_t<T>))) {}

    // 判断是否持有值
    [[nodiscard]] bool has_value() const noexcept { return ptr_ != nullptr; }

    // 获取运行时类型索引
    [[nodiscard]] std::type_index type() const noexcept { return type_; }

    // 取出强类型引用：类型不匹配直接抛异常（模拟 Talos 的 panic）
    template <typename T>
    [[nodiscard]] T& as() {
        if (type_ != std::type_index(typeid(T))) {
            throw std::runtime_error("UniqueAny::as(): type mismatch");
        }
        return static_cast<Model<T>*>(ptr_.get())->value;
    }

    template <typename T>
    [[nodiscard]] const T& as() const {
        if (type_ != std::type_index(typeid(T))) {
            throw std::runtime_error("UniqueAny::as(): type mismatch");
        }
        return static_cast<const Model<T>*>(ptr_.get())->value;
    }

private:
    // Concept 基类：类型擦除接口
    struct Concept {
        virtual ~Concept() = default;
    };

    // Model 模板：持有具体类型
    template <typename T>
    struct Model final : Concept {
        T value;
        explicit Model(T v) : value(std::move(v)) {}
    };

    std::unique_ptr<Concept> ptr_;
    std::type_index type_{typeid(void)};
};

}  // namespace ecs
