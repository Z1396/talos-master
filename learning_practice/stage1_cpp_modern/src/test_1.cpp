#include <compare>   // 引入 spaceship operator （C++20）
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>

template <typename T>
concept Comparable = requires(const T& a, const T& b)
{
    { a < b } -> std::convertible_to<bool>;

};

template <typename T>
requires Comparable<T>
constexpr T find_min(const std::vector<T>& vec)
{
    if(vec.empty())
    {
        throw std::invalid_argument("empty container");
    }
    auto main_it = vec.cbegin();
    for(auto it = vec.cbegin(); it != vec.cend(); ++it)
    {
        if(*it < *main_it)
        {
            main_it = it;
        }
    }
    return *main_it;
}




int main()
{
    // 测试1 int求最小值
    std::vector<int> iv{5,2,8,1,9,3};
    std::cout << "min int = " << find_min(iv) << "\n";

    // 测试2 string字典序最小值
    std::vector<std::string> sv{"banana", "appleeee", "cherrye"};
    std::cout << "min str = " << find_min(sv) << "\n";

    // =====取消下面注释会编译报错，验证concept约束生效=====
    // find_min(std::vector<NonComparable>{});
    // SortedVector<NonComparable> bad_obj;

    std::cout << "demo done\n";
    return 0;
}

