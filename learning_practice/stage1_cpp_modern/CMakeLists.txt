# ===========================================================================
# 阶段1：C++ 现代特性实战 - 构建配置
# 功能：编译 5 个独立小程序，演示 concept / expected / 模板元编程 / CRTP
# 构建：cmake -B build -G Ninja && cmake --build build
# 运行：./build/01_concept  ./build/02_expected ...
# ===========================================================================
# 强制使用 Clang 21 + libstdc++13（std::expected 在 libstdc++13 中提供）
set(CMAKE_C_COMPILER /usr/bin/clang-21)
set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")

cmake_minimum_required(VERSION 3.20)
project(stage1_cpp_modern LANGUAGES CXX)

# 强制使用 C++23 标准，不允许编译器扩展
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 启用所有警告且视为错误（与 Talos 项目保持一致）
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# 列出 5 个独立练习程序：源文件名即目标名 
set(EXAMPLES
    01_concept        # concept 类型约束
    02_expected       # std::expected 错误处理
    03_tuple_size     # 编译期元编程
    04_crtp           # 奇异递归模板模式
    05_combined       # 综合演示
)

# 为每个程序创建独立可执行目标，便于单独运行测试
foreach(example IN LISTS EXAMPLES)
    add_executable(${example} src/${example}.cpp)
endforeach()
