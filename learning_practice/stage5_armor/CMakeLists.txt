# 强制 Clang 21 + libstdc++13
set(CMAKE_C_COMPILER /usr/bin/clang-21)
set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")

cmake_minimum_required(VERSION 3.20)
project(stage5_armor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# 装甲板检测 + EKF 跟踪库（头文件库）
add_library(armor INTERFACE)
target_include_directories(armor INTERFACE include)

# 演示程序
add_executable(armor_demo src/demo.cpp)
target_link_libraries(armor_demo PRIVATE armor)
