# ===========================================================================
# stage10_math_so2：math 模块（crates/math，207 行 header-only）
#   - so2.hpp  SO(2) 旋转李群：角度自动归一化 [-π, π]
#   - euler.hpp ROS2 Z-Y-X RPY 欧拉角 <-> 四元数/旋转矩阵/SO3 互转
# demo 直接 #include 项目真实头文件（include 指向 crates/math/src），
# 不复制任何代码。
# ===========================================================================

# 编译器选择：优先学习环境统一的 clang-21 + libstdc++13，
# 不存在时回退系统默认编译器（不影响本 stage，全部为标准 C++23）
if(EXISTS /usr/bin/clang++-21)
    set(CMAKE_C_COMPILER /usr/bin/clang-21)
    set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
    set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")
endif()

cmake_minimum_required(VERSION 3.20)
project(stage10_math_so2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS ON)

# ===========================================================================
# 依赖：Eigen（euler.hpp / lieplusplus 需要）
# 真实项目用 find_package(Eigen3)；本 stage 先找系统版本，
# 找不到则 FetchContent 拉取 Eigen 3.4.0（仅头文件，不执行其 CMake）
# ===========================================================================
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
    message(STATUS "stage10: 系统未安装 Eigen，FetchContent 拉取 3.4.0（header-only）")
    include(FetchContent)
    # SOURCE_SUBDIR 指向不存在的目录：只下载解压，不 add_subdirectory
    FetchContent_Declare(
        eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
        SOURCE_SUBDIR __headers_only__
    )
    FetchContent_MakeAvailable(eigen)
    add_library(stage10_eigen INTERFACE)
    target_include_directories(stage10_eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
    set(EIGEN_TARGET stage10_eigen)
else()
    message(STATUS "stage10: 使用系统 Eigen ${Eigen3_VERSION}")
    set(EIGEN_TARGET Eigen3::Eigen)
endif()

# ===========================================================================
# math 模块（头文件库）：include 指向真实 crates/math/src 与
# 3dparty/lieplusplus（euler.hpp 依赖其 groups/SO3.hpp）
# ===========================================================================
add_library(talos_math INTERFACE)
target_include_directories(talos_math SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../crates/math/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3dparty/lieplusplus
)
target_link_libraries(talos_math INTERFACE ${EIGEN_TARGET})

# 演示程序：警告只作用于本 demo，不传染第三方依赖
add_executable(math_so2_demo src/demo.cpp)
target_compile_options(math_so2_demo PRIVATE -Wall -Wextra -Werror)
target_link_libraries(math_so2_demo PRIVATE talos_math)
