// ===========================================================================
// 练习2：std::expected 错误处理（C++23）
// 目标：实现安全除法，返回 expected<int, string>
// 学习要点：expected 构造、错误传播、and_then / or_else 单子操作
// ===========================================================================

#include <cmath>
#include <cstdint>
#include <expected>
#include <iostream>
#include <string>
#include <string_view>

// ---------------------------------------------------------------------------
// 1. 自定义错误类型：枚举类比字符串更结构化
// ---------------------------------------------------------------------------
enum class MathError {
    DivisionByZero,
    NegativeRoot,
};

// 为错误类型提供格式化输出
std::string to_string(MathError e) {
    switch (e) {
        case MathError::DivisionByZero: return "division by zero";
        case MathError::NegativeRoot:   return "negative square root";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// 2. 安全除法：返回 expected<T, E> 而非抛异常
// 与 Talos 项目一致：noexcept + expected 错误传播
// ---------------------------------------------------------------------------
[[nodiscard]] std::expected<int, MathError>
safe_divide(int a, int b) noexcept {
    if (b == 0) {
        return std::unexpected(MathError::DivisionByZero);
    }
    return a / b;
}

// ---------------------------------------------------------------------------
// 3. 安全平方根：链式调用前用 and_then 串联
// and_then 在成功时执行下一步，失败时直接透传错误
// ---------------------------------------------------------------------------
[[nodiscard]] std::expected<int, MathError>
safe_sqrt(int x) noexcept {
    if (x < 0) {
        return std::unexpected(MathError::NegativeRoot);
    }
    return static_cast<int>(std::sqrt(x));
}

// ---------------------------------------------------------------------------
// 4. 链式计算：(a/b) 再开根，任意一步失败都优雅传播
// ---------------------------------------------------------------------------

int main() {
    // 正常情况：16 / 4 = 4，sqrt(4) = 2
    auto result1 = safe_divide(16, 4).and_then([](int v) {
        return safe_sqrt(v);
    });
    if (result1) {
        std::cout << "result1 = " << *result1 << "\n";
    } else {
        std::cout << "result1 error: " << to_string(result1.error()) << "\n";
    }

    // 除零错误：透传至最终结果
    auto result2 = safe_divide(16, 0).and_then([](int v) {
        return safe_sqrt(v);
    });
    if (result2) {
        std::cout << "result2 = " << *result2 << "\n";
    } else {
        std::cout << "result2 error: " << to_string(result2.error()) << "\n";
    }

    // 负数开根：4/-4 = -1，sqrt(-1) 失败
    auto result3 = safe_divide(4, -4).and_then([](int v) {
        return safe_sqrt(v);
    });
    if (result3) {
        std::cout << "result3 = " << *result3 << "\n";
    } else {
        std::cout << "result3 error: " << to_string(result3.error()) << "\n";
    }

    // transform：对成功值做变换（不产生新错误）
    auto result4 = safe_divide(100, 5).transform([](int v) {
        return v * 2;  // 20 * 2 = 40
    });
    std::cout << "result4 = " << result4.value_or(-1) << "\n";

    // value_or：失败时给默认值
    auto result5 = safe_divide(10, 0);
    std::cout << "result5 = " << result5.value_or(0) << "\n";

    std::cout << "std::expected 演示完成\n";
    return 0;
}
