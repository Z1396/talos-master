// ===========================================================================
// unique_any.hpp - 类型擦除容器（Talos UniqueAny 简化版）
//
// 核心设计：
//   1. Concept/Model 模式擦除类型，异构资源统一存储
//   2. 仅移动语义（资源独占，适配通道/硬件句柄等不可拷贝类型）
//   3. as<T>() 强类型取出，类型不匹配抛异常（模拟 Talos panic）
// ===========================================================================
#pragma once                      // 防止头文件重复包含

#include <memory>                 // std::unique_ptr, std::make_unique
#include <stdexcept>              // std::runtime_error
#include <type_traits>            // std::decay_t, std::same_as
#include <typeindex>              // std::type_index (类型擦除后的类型标识)
#include <utility>                // std::forward, std::move

namespace ecs {

// ===========================================================================
// UniqueAny - 可存储任意类型的容器，且拥有独占所有权（仅移动）
// 
// 设计目标：像 std::any 一样能装任何类型，但只能移动不能拷贝，
//          适合管理不能复制的资源（如文件句柄、GPU纹理、网络连接等）
// ===========================================================================
class UniqueAny {
public:
    // ---------- 构造/析构 ----------
    UniqueAny()  = default;        // 默认构造：空容器，不包含任何数据
    ~UniqueAny() = default;        // 默认析构：unique_ptr 自动释放资源

    // ---------- 拷贝禁用（核心：独占所有权） ----------
    // 删除拷贝构造和拷贝赋值，确保不能复制
    UniqueAny(const UniqueAny&)                = delete;
    UniqueAny& operator=(const UniqueAny&)     = delete;
    
    // ---------- 移动启用 ----------
    // 默认的移动构造和移动赋值即可，unique_ptr 支持移动
    UniqueAny(UniqueAny&&) noexcept            = default;
    UniqueAny& operator=(UniqueAny&&) noexcept = default;

    // =========================================================================
    // 构造函数（模板）：从任意类型 T 构造 UniqueAny
    // 
    // 模板约束（requires C++20）：
    //   - 禁止从 UniqueAny 自身构造（避免混淆）
    //   - std::decay_t<T> 去除引用和 cv 限定符，得到纯类型
    // 
    // 关键操作：
    //   1. 用 Model<std::decay_t<T>> 包装实际值
    //   2. 存入 ptr_（基类指针，擦除具体类型）
    //   3. 记录 type_（运行时类型信息，用于后续类型检查）
    // =========================================================================
    template <typename T>
    requires(!std::same_as<std::decay_t<T>, UniqueAny>)  // 禁止从 UniqueAny 构造
    explicit UniqueAny(T&& value)                        // 完美转发
        : ptr_(std::make_unique<Model<std::decay_t<T>>>(std::forward<T>(value)))                  // 用 forward 保留值类别
        , type_(std::type_index(typeid(std::decay_t<T>))) {} 
        // typeid 获取实际类型的运行时信息，存入 type_index 便于比较

    // =========================================================================
    // as<T>() - 取出存储的值的引用（非 const 版本）
    // 
    // 工作流程：
    //   1. 检查存储的类型是否与请求的类型 T 匹配
    //   2. 不匹配 → 抛出异常（模拟 Talos 的 panic）
    //   3. 匹配 → 将 ptr_ 向下转型为 Model<T>*，返回内部的 value 引用
    // 
    // [[nodiscard]] 强制调用者使用返回值，防止误操作
    // =========================================================================
    template <typename T>
    [[nodiscard]] T& as() {
        // 运行时类型检查：用 type_index 比较存储的类型和请求的类型是否一致
        if (type_ != std::type_index(typeid(T))) {
            // 类型不匹配，抛出异常（类似 Talos 的 panic!）
            throw std::runtime_error("UniqueAny::as(): type mismatch");
        }
        // 安全向下转型：ptr_ 实际指向 Model<T>，用 static_cast 转回
        return static_cast<Model<T>*>(ptr_.get())->value;
    }

    // =========================================================================
    // as<T>() - 取出存储的值的引用（const 版本）
    // 
    // 用法相同，但返回 const 引用，防止外部修改
    // =========================================================================
    template <typename T>
    [[nodiscard]] const T& as() const {
        if (type_ != std::type_index(typeid(T))) {
            throw std::runtime_error("UniqueAny::as(): type mismatch");
        }
        // const 版本返回 const 引用
        return static_cast<const Model<T>*>(ptr_.get())->value;
    }

private:
    // =========================================================================
    // Concept（概念基类）
    // 
    // 作用：提供统一的虚接口，使得 unique_ptr<Concept> 可以指向任意 Model<T>
    // 
    // 这个类本身什么也不做，只提供虚析构函数，确保派生类能正确析构
    // 这就是"类型擦除"的关键：通过基类指针隐藏具体类型
    // =========================================================================
    struct Concept {
        virtual ~Concept() = default;    // 虚析构，确保派生类析构时调用正确
    };

    // =========================================================================
    // Model（模型类）
    // 
    // 模板类，每个 T 实例化一份，存储实际值
    // 
    // 设计模式：Concept/Model（也称为 Type Erasure 惯用法）
    //   - Concept：对外暴露的统一接口（这里只是虚析构）
    //   - Model<T>：针对具体类型的包装器，存储 T 类型的值
    // =========================================================================
    template <typename T>
    struct Model final : Concept {      // final 阻止进一步派生
        T value;                         // 存储的实际值
        
        explicit Model(T v)              // 构造函数，用 move 转移值
            : value(std::move(v)) {}     // 移动语义，避免拷贝
    };

    // ---------- 成员变量 ----------
    std::unique_ptr<Concept> ptr_;      // 指向实际数据的智能指针（基类指针擦除类型）
    std::type_index type_{typeid(void)}; // 存储实际类型的运行时信息（默认 void 表示空）
};

} // namespace ecs