#pragma once
// 头文件保护宏，防止重复包含造成重定义编译错误

// 标准元组，存储延迟构造的预绑定参数包
#include <tuple>
// std::forward 完美转发、std::move 所有权转移
#include <utility>

namespace talos::primitive {

/**
 * @brief 延迟构造包装器 lazy<T, PArgs...>
 * 作用：预先保存一组构造参数，推迟 T 的实例化；调用 operator() 时才真正 new T
 * 两种使用模式：
 * 1. 带预绑定参数 PArgs...：调用无参 operator()() 直接用保存参数构造 T
 * 2. 无预绑定参数 PArgs...=空：调用 operator()(Args&&...) 传入运行时参数构造 T
 * 返回 std::unique_ptr<T> 自动管理内存，RAII 自动释放
 * @tparam T 需要延迟构造的目标类型
 * @tparam PArgs... 预绑定构造参数包（编译期固定参数）
 */
template <typename T, typename... PArgs>
struct lazy {
private:
    // 存储预绑定参数：std::decay_t 去除引用/const/volatile，保存纯值类型
    std::tuple<std::decay_t<PArgs>...> args_;

public:
    /**
     * @brief 构造 lazy 延迟包装器，保存所有预绑定参数
     * @param p_args 任意数量预绑定构造参数，完美转发存入元组
     * constexpr 编译期可构造；noexcept 不抛出异常
     * explicit 禁止隐式类型转换
     */
    explicit constexpr lazy(PArgs&&... p_args) noexcept
        : args_(std::forward<PArgs>(p_args)...) {}

    /**
     * @brief 重载调用运算符：无参调用，使用预保存参数构造 T
     * 约束 requires(sizeof...(PArgs) > 0)：仅当存在预绑定参数时启用该重载
     * @return std::unique_ptr<T> 动态分配T实例智能指针
     * [[nodiscard]] 强制接收返回值，防止内存泄漏
     * noexcept 构造不抛异常
     */
    [[nodiscard]] std::unique_ptr<T> operator()() noexcept requires(sizeof...(PArgs) > 0) {
        // std::apply 解包元组，把所有预绑定参数传入 lambda
        return std::apply(
            [](auto&&... args) {
                // 完美转发元组内保存的参数，调用 T 构造函数，生成 unique_ptr
                return std::make_unique<T>(std::forward<decltype(args)>(args)...);
            },
            args_);
    }

    /**
     * @brief 重载调用运算符：接收运行时动态参数，无预绑定参数场景专用
     * 约束 requires(sizeof...(PArgs) == 0)：仅空预参数包时启用
     * @tparam Args... 运行时传入的构造参数包
     * @param args 运行时动态构造参数
     * @return std::unique_ptr<T> 动态分配T实例智能指针
     */
    template <typename... Args>
    [[nodiscard]] std::unique_ptr<T> operator()(Args&&... args) noexcept requires(sizeof...(PArgs) == 0) {
        // 直接转发运行时参数构造T，无预绑定元组参与
        return std::make_unique<T>(std::forward<Args>(args)...);
    }
};

/**
 * @brief 辅助工厂函数 make_lazy，简化 lazy<T, PArgs...> 构造，自动推导模板参数
 * @tparam T 目标延迟构造类型
 * @tparam PArgs... 预绑定参数类型包（编译器自动推导）
 * @param args 预绑定构造参数
 * @return constexpr lazy<T> 延迟包装实例
 */
template <typename T, typename... PArgs>
constexpr lazy<T> make_lazy(PArgs&&... args) {
    return lazy<T, PArgs...>(args...);
}

} // namespace talos::primitive