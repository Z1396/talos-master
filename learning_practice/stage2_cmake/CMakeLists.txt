# ===========================================================================
# 阶段2：CMake 多模块项目 - 顶层 CMakeLists
# 演示自研 crate() 宏的模块化管理（Talos 风格简化版）
#
# 构建：cmake -B build -G Ninja -DCMAKE_CXX_COMPILER=clang++-21 && cmake --build build
# 运行：./build/app
# ===========================================================================
# 强制 Clang 21（与 Talos 项目一致）
set(CMAKE_C_COMPILER /usr/bin/clang-21)
set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")

cmake_minimum_required(VERSION 3.20)
project(stage2_cmake LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 引入自研 crate() 宏定义
include(${CMAKE_CURRENT_SOURCE_DIR}/crates.cmake)

# === 添加三个 crate 模块（依赖顺序：io ← math ← app）===
add_subdirectory(modules/io)     # 底层：I/O 工具
add_subdirectory(modules/math)   # 中层：数学库（依赖 io）
add_subdirectory(app)             # 顶层：应用（依赖 math）

# 打印构建信息（模拟 Talos 的版本注入）
message(STATUS "=== Stage2 CMake 项目配置完成 ===")
