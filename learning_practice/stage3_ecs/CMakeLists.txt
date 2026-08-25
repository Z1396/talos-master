# 强制 Clang 21 + libstdc++13
set(CMAKE_C_COMPILER /usr/bin/clang-21)
set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")

cmake_minimum_required(VERSION 3.20)
project(stage3_ecs LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS ON)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# ECS 库（头文件库）
add_library(ecs INTERFACE)
target_include_directories(ecs INTERFACE include)

# 测试程序
add_executable(ecs_demo src/main.cpp)
target_link_libraries(ecs_demo PRIVATE ecs)
