# 强制 Clang 21 + libstdc++13
set(CMAKE_C_COMPILER /usr/bin/clang-21)
set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")

cmake_minimum_required(VERSION 3.20)
project(stage5_armor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS ON)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# 依赖：Eigen（矩阵运算）+ Ceres（Jet 自动微分，与主项目相同）
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

# EKF 跟踪库（头文件库，照搬真实 L3_estimation/tracker 结构）
add_library(armor INTERFACE)
target_include_directories(armor INTERFACE include)

# 演示程序
add_executable(armor_demo src/demo.cpp)
target_link_libraries(armor_demo PRIVATE armor Eigen3::Eigen Ceres::ceres)
