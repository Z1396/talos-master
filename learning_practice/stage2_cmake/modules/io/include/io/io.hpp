#pragma once
// io crate 公共头文件
#include <string>
#include <string_view>

namespace practice::io {

// 打印带前缀的消息
void print_line(std::string_view prefix, std::string_view msg);

// 读取用户输入
[[nodiscard]] std::string read_input(std::string_view prompt);

}  // namespace practice::io
