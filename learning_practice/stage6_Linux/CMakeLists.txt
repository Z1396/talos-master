cmake_minimum_required(VERSION 3.20)# 强制 Clang 21 + libstdc++13

set(CMAKE_C_COMPILER /usr/bin/clang-21)
set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")

project(stage6_Linux LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS ON)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(fmt REQUIRED)


# EKF 跟踪库（头文件库，照搬真实 L3_estimation/tracker 结构）
add_library(Linux INTERFACE)
target_include_directories(Linux INTERFACE include)

add_executable(realtime_priority_demo src/realtime_priority_demo.cpp)
target_link_libraries(realtime_priority_demo PRIVATE Linux fmt::fmt)
