#include <string>
#include <expected>
#include <cmath>
#include <iostream>

// ---------------------------------------------------------------------------
// 1. 自定义错误类型：枚举类比字符串更结构化
// ---------------------------------------------------------------------------
enum class MathError {
    DivisionByZero,   // 除零
    NegativeRoot,     // 负数开根号
};

// 把枚举转成打印用字符串
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
safe_divide(int a, int b) noexcept
{
    if (b == 0)
    {
        // 出错：返回unexpected包装错误对象
        return std::unexpected(MathError::DivisionByZero);
    }
    // 正常：普通int，隐式构造expected<int,...>的成功分支
    return a / b;
}

// ---------------------------------------------------------------------------
// 3. 安全平方根：链式调用前用 and_then 串联
// and_then 在成功时执行下一步，失败时直接透传错误
// ---------------------------------------------------------------------------
[[nodiscard]] std::expected<int, MathError>
safe_sqrt(int x) noexcept
{
    if (x < 0)
    {
        return std::unexpected(MathError::NegativeRoot);
    }
    return static_cast<int>(std::sqrt(x));
}

// ---------------------------------------------------------------------------
// 4. 链式计算：(a/b) 再开根，任意一步失败都优雅传播
// ---------------------------------------------------------------------------

int main()
{
    // case1 正常流程：16 /4 =4；再sqrt(4)=2
    auto result1 = safe_divide(16, 4).and_then([](int v) {
        return safe_sqrt(v);
    });
    if (result1) // operator bool：true代表成功
    {
        std::cout << "result1 = " << *result1 << "\n"; // *解引用拿成功值
    }
    else
    {
        std::cout << "result1 error: " << to_string(result1.error()) << "\n";
    }


    // case2：除零错误，and_then里面lambda不会跑，错误直接透传
    auto result2 = safe_divide(16, 0).and_then([](int v) {
        return safe_sqrt(v);
    });
    if (result2)
    {
        std::cout << "result2 = " << *result2 << "\n";
    }
    else
    {
        std::cout << "result2 error: " << to_string(result2.error()) << "\n";
    }


    // case3：4 / (-4) = -1；除法成功，进入and_then调用safe_sqrt(-1) →负数开根号报错
    auto result3 = safe_divide(4, -4).and_then([](int v) {
        return safe_sqrt(v);
    });
    if (result3)
    {
        std::cout << "result3 = " << *result3 << "\n";
    }
    else
    {
        std::cout << "result3 error: " << to_string(result3.error()) << "\n";
    }


    // transform：对成功的值做变换；lambda返回普通值，**不能产生新错误**
    // safe_divide(100,5)=20 → transform λ：20*2=40
    auto result4 = safe_divide(100, 5).transform([](int v) {
        return v * 2;
    });
    // value_or：成功取内部值；失败返回传入的默认值-1
    std::cout << "result4 = " << result4.value_or(-1) << "\n";


    // value_or演示：除零失败，取默认0
    auto result5 = safe_divide(10, 0);
    std::cout << "result5 = " << result5.value_or(0) << "\n";


    std::cout << "std::expected 演示完成\n";
    return 0;
}


