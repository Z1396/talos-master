# 强制 Clang 21 + libstdc++13
set(CMAKE_C_COMPILER /usr/bin/clang-21)
set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")

cmake_minimum_required(VERSION 3.20)
project(stage4_triple_buffer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# 三缓冲库（头文件库）
add_library(triple_buffer INTERFACE)
target_include_directories(triple_buffer INTERFACE include)

# 演示 + 压力测试
add_executable(triple_buffer_demo src/demo.cpp)
target_link_libraries(triple_buffer_demo PRIVATE triple_buffer)

# 多线程压力测试需要 pthread
find_package(Threads REQUIRED)
target_link_libraries(triple_buffer_demo PRIVATE Threads::Threads)
