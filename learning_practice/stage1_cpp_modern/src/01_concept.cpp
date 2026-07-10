// ===========================================================================
// 练习1：C++20 concept 类型约束
// 目标：自定义 Comparable<T> 概念，限制模板参数必须支持 <
// 学习要点：requires 表达式、requires 子句、concept 命名规范
// ===========================================================================

#include <compare>
#include <iostream>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// 1. 定义概念：concept 名称用 snake_case（与 Talos 项目风格一致）
// requires 表达式返回 bool，编译期判定类型 T 是否满足约束
// ---------------------------------------------------------------------------
template <typename T>
concept Comparable = requires(const T& a, const T& b) {
    // 要求 T 支持小于比较，且结果可转换为 bool
    { a < b } -> std::convertible_to<bool>;
    // 要求 T 支持三路比较（C++20 spaceship operator）
    { a <=> b };
};

// ---------------------------------------------------------------------------
// 2. 用 concept 约束模板函数：找容器中的最小元素
// requires 子句写法：在模板参数列表后用 requires 显式约束
// ---------------------------------------------------------------------------
template <typename T>
    requires Comparable<T>
constexpr T find_min(const std::vector<T>& vec) {
    if (vec.empty()) {
        throw std::invalid_argument("empty container");
    }
    T min_val = vec.front();
    for (const auto& v : vec) {
        if (v < min_val) {
            min_val = v;
        }
    }
    return min_val;
}

// ---------------------------------------------------------------------------
// 3. 用 concept 约束类模板：有序容器（仅接受可比较类型）
// typename T 后直接跟 concept 名，语法更简洁
// ---------------------------------------------------------------------------
template <Comparable T>
class SortedVector {
public:
    void push(T value) {
        // 利用 <=> 找到插入位置，保持有序
        auto it = std::lower_bound(data_.begin(), data_.end(), value);
        data_.insert(it, std::move(value));
    }

    const T& at(std::size_t i) const { return data_.at(i); }
    [[nodiscard]] std::size_t size() const noexcept { return data_.size(); }

private:
    std::vector<T> data_;
};

// ---------------------------------------------------------------------------
// 4. 多概念组合：要求同时满足 Comparable 且可默认构造
// ---------------------------------------------------------------------------
template <typename T>
concept ComparableAndDefaultable = Comparable<T> && std::default_initializable<T>;

// ---------------------------------------------------------------------------
// 5. 不满足 Comparable 的类型：用于验证编译期约束
// ---------------------------------------------------------------------------
struct NonComparable {};

int main() {
    // 整数向量
    std::vector<int> ints = {5, 2, 8, 1, 9, 3};
    std::cout << "min int: " << find_min(ints) << "\n";

    // 字符串向量
    std::vector<std::string> strs = {"banana", "apple", "cherry"};
    std::cout << "min str: " << find_min(strs) << "\n";

    // 有序容器
    SortedVector<int> sv;
    sv.push(5);
    sv.push(1);
    sv.push(3);
    std::cout << "sorted[0]: " << sv.at(0) << "\n";
    std::cout << "sorted[1]: " << sv.at(1) << "\n";
    std::cout << "sorted[2]: " << sv.at(2) << "\n";

    // 取消下面注释会触发编译错误，验证 concept 约束生效：
    // find_min(std::vector<NonComparable>{});
    // SortedVector<NonComparable> bad;

    std::cout << "concept 约束演示完成\n";
    return 0;
}
