#include "math/math.hpp"
#include "io/io.hpp"  // 演示跨 crate 依赖

#include <stdexcept>

namespace practice::math {

std::vector<double> add(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) {
        // 调用 io crate 的功能，演示依赖传递
        practice::io::print_line("math", "size mismatch in add()");
        throw std::invalid_argument("size mismatch");
    }
    std::vector<double> result(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] + b[i];
    }
    return result;
}

double dot(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) {
        throw std::invalid_argument("size mismatch");
    }
    double result = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        result += a[i] * b[i];
    }
    return result;
}

double sum(const std::vector<double>& v) {
    double total = 0.0;
    for (double x : v) {
        total += x;
    }
    return total;
}

}  // namespace practice::math
