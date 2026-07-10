// 阶段2：app 主程序
// 演示 crate 依赖链：app → math → io
#include "io/io.hpp"
#include "math/math.hpp"

#include <vector>

int main() {
    using namespace practice;

    // 使用 io crate
    io::print_line("main", "Stage2 CMake 项目启动");

    // 使用 math crate（间接依赖 io）
    std::vector<double> a = {1.0, 2.0, 3.0};
    std::vector<double> b = {4.0, 5.0, 6.0};

    auto c = math::add(a, b);
    io::print_line("add", "结果: [5, 7, 9]");

    double d = math::dot(a, b);
    io::print_line("dot", "结果: " + std::to_string(d));

    double s = math::sum(c);
    io::print_line("sum", "结果: " + std::to_string(s));

    io::print_line("main", "Stage2 演示完成");
    return 0;
}
