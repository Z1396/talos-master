# ===========================================================================
# stage7_primitive_tools：primitive 模块其余 6 个组件
#   - lazy.hpp             延迟构造（参数包存 tuple，调用时才 new T）
#   - overloaded.hpp       std::visit 多 lambda 派发
#   - spin.hpp             SPIN_HINT（x86 _mm_pause / ARM yield）
#   - performance_probe    时延探针 + 8192 滑动窗口直方图（含 .cpp）
#   - system_info.hpp      用户名/主机名查询（含 .cpp）
#   - channel.hpp          三缓冲之上的通道封装（make_spsc/spmc_channel）
# demo 直接 #include 项目真实头文件；两个真实 .cpp 直接编译进目标。
# ===========================================================================

# 编译器选择：优先学习环境统一的 clang-21，不存在时回退系统默认
if(EXISTS /usr/bin/clang++-21)
    set(CMAKE_C_COMPILER /usr/bin/clang-21)
    set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
    set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")
endif()

cmake_minimum_required(VERSION 3.20)
project(stage7_primitive_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS ON)

# ===========================================================================
# primitive 模块库：真实源码（2 个 .cpp）+ 头文件目录
# ===========================================================================
set(PRIMITIVE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../crates/primitive/src)

add_library(talos_primitive STATIC
    ${PRIMITIVE_DIR}/primitive/performance_probe.cpp
    ${PRIMITIVE_DIR}/primitive/system_info.cpp
)
target_include_directories(talos_primitive PUBLIC ${PRIMITIVE_DIR})
target_compile_features(talos_primitive PUBLIC cxx_std_23)

# 演示程序：警告只作用于本 demo
# -Wno-interference-size：项目三缓冲头文件用 std::hardware_destructive_interference_size
# 做缓存行对齐，GCC 对其值稳定性有保留意见（提示用常量替代），此处豁免
add_executable(primitive_tools_demo src/demo.cpp)
target_compile_options(primitive_tools_demo PRIVATE -Wall -Wextra -Werror -Wno-interference-size)
target_link_libraries(primitive_tools_demo PRIVATE talos_primitive)
