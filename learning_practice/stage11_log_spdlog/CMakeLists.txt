# ===========================================================================
# stage11_log_spdlog：log 模块（crates/log，149 行，1 个头文件）
#   - spdlog_hook.hpp  全局日志初始化（控制台 + 滚动文件双输出槽）
#                      + std::cout / std::cerr 重定向进 spdlog
# demo 直接 #include 项目真实头文件（include 指向 crates/log/src）。
# ===========================================================================

# 编译器选择：优先学习环境统一的 clang-21，不存在时回退系统默认
if(EXISTS /usr/bin/clang++-21)
    set(CMAKE_C_COMPILER /usr/bin/clang-21)
    set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
    set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")
endif()

cmake_minimum_required(VERSION 3.20)
project(stage11_log_spdlog LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS ON)

# ===========================================================================
# 依赖：spdlog（真实项目 CMake 由 fetch-content-helper 拉取 spdlog::spdlog）
# 本 stage 用 FetchContent 拉官方源码包，等价于项目的 spdlog::spdlog 目标
# ===========================================================================
include(FetchContent)
set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_BENCH OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    spdlog
    URL https://github.com/gabime/spdlog/archive/refs/tags/v1.15.3.tar.gz
)
FetchContent_MakeAvailable(spdlog)

# ===========================================================================
# log 模块（头文件库）：include 指向真实 crates/log/src
# ===========================================================================
add_library(talos_log INTERFACE)
target_include_directories(talos_log INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../crates/log/src
)
target_link_libraries(talos_log INTERFACE spdlog::spdlog)

# 演示程序：警告只作用于本 demo，不传染 spdlog
add_executable(log_spdlog_demo src/demo.cpp)
target_compile_options(log_spdlog_demo PRIVATE -Wall -Wextra -Werror)
target_link_libraries(log_spdlog_demo PRIVATE talos_log)
