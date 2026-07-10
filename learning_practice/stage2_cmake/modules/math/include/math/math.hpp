#pragma once
// math crate 公共头文件
#include <vector>

namespace practice::math {

// 向量加法
[[nodiscard]] std::vector<double> add(const std::vector<double>& a, const std::vector<double>& b);

// 向量点积
[[nodiscard]] double dot(const std::vector<double>& a, const std::vector<double>& b);

// 向量求和
[[nodiscard]] double sum(const std::vector<double>& v);

}  // namespace practice::math
