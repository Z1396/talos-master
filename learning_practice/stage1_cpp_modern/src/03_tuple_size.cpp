// ===========================================================================
// 练习3：编译期元编程 - 计算 tuple 大小
// 目标：手写 tuple_size，理解模板递归与特化
// 学习要点：模板特化、编译期递归、if constexpr
// ===========================================================================

#include <iostream>
#include <tuple>
#include <type_traits>

// ---------------------------------------------------------------------------
// 1. 自定义编译期类型列表：TypeList<T...>
// 用于学习可变参数模板展开
// ---------------------------------------------------------------------------
template <typename... Ts>
struct TypeList {};

// 编译期计算 TypeList 长度：递归模板特化
template <typename List>
struct TypeListSize;

// 特化：空列表长度 0
template <>
struct TypeListSize<TypeList<>> {
    static constexpr std::size_t value = 0;
};

// 特化：首元素 + 尾列表递归
template <typename Head, typename... Tail>
struct TypeListSize<TypeList<Head, Tail...>> {
    static constexpr std::size_t value = 1 + TypeListSize<TypeList<Tail...>>::value;
};

// 便捷别名
template <typename List>
inline constexpr std::size_t type_list_size_v = TypeListSize<List>::value;

// ---------------------------------------------------------------------------
// 2. 编译期取 TypeList 第 N 个类型
// ---------------------------------------------------------------------------
template <std::size_t N, typename List>
struct TypeAt;

template <typename Head, typename... Tail>
struct TypeAt<0, TypeList<Head, Tail...>> {
    using type = Head;
};

template <std::size_t N, typename Head, typename... Tail>
struct TypeAt<N, TypeList<Head, Tail...>> {
    using type = typename TypeAt<N - 1, TypeList<Tail...>>::type;
};

// ---------------------------------------------------------------------------
// 3. 编译期判断类型是否在 TypeList 中
// ---------------------------------------------------------------------------
template <typename T, typename List>
struct Contains;

template <typename T>
struct Contains<T, TypeList<>> : std::false_type {};

template <typename T, typename Head, typename... Tail>
struct Contains<T, TypeList<Head, Tail...>>
    : std::bool_constant<std::is_same_v<T, Head> || Contains<T, TypeList<Tail...>>::value> {};

template <typename T, typename List>
inline constexpr bool contains_v = Contains<T, List>::value;

// ---------------------------------------------------------------------------
// 4. 运行期使用：用 std::tuple 配合 std::tuple_size_v
// ---------------------------------------------------------------------------
template <typename Tuple>
void print_tuple_info() {
    // std::tuple_size_v 是标准库提供的编译期大小
    constexpr std::size_t size = std::tuple_size_v<std::remove_cvref_t<Tuple>>;
    std::cout << "tuple size = " << size << "\n";
}

// 用 fold expression 在编译期展开参数包
template <typename... Ts>
constexpr auto sum_all(Ts... args) {
    return (... + args);  // C++17 折叠表达式
}

int main() {
    // TypeList 大小计算（编译期）
    using MyList = TypeList<int, double, char, float>;
    std::cout << "TypeList size = " << type_list_size_v<MyList> << "\n";

    // 取第 2 个类型（double）
    static_assert(std::is_same_v<typename TypeAt<1, MyList>::type, double>);

    // 判断类型是否在列表中
    static_assert(contains_v<int, MyList>);
    static_assert(!contains_v<bool, MyList>);
    std::cout << "contains int: " << std::boolalpha << contains_v<int, MyList> << "\n";
    std::cout << "contains bool: " << contains_v<bool, MyList> << "\n";

    // std::tuple 大小（标准库）
    std::tuple<int, double, char, float> t{1, 2.0, 'c', 3.0f};
    print_tuple_info<decltype(t)>();

    // 折叠表达式求和
    std::cout << "sum = " << sum_all(1, 2, 3, 4, 5) << "\n";

    std::cout << "模板元编程演示完成\n";
    return 0;
}
