# ===========================================================================
# stage8_toml_reflection：toml 模块（crates/toml，INTERFACE header-only）
#   结构体反射式 TOML 解析：required 必填 / optional 可选 / flatten 扁平化 /
#   vector/array 容器扩展 / merge_configs 分层合并
# demo 直接 #include 项目真实头文件（include 指向 crates/toml/src）。
#
# 依赖（对齐项目主 CMake）：
#   - tomlplusplus（同一 commit 1c8b7466，header-only 模式）
#   - fmt 12.0.0（错误信息拼接；项目用本地 zip，这里 FetchContent 等价物）
#   - Boost.PFR 2.2.0（field_reflection.hpp 的反射后端，项目 3dparty 为空目录）
#   - magic_enum（项目 3dparty 本地头文件，枚举反射）
#   注意：Eigen 扩展（toml_helper_eigen.hpp）本 demo 不涉及，不引入 Eigen
# ===========================================================================

# 编译器选择：优先学习环境统一的 clang-21，不存在时回退系统默认
if(EXISTS /usr/bin/clang++-21)
    set(CMAKE_C_COMPILER /usr/bin/clang-21)
    set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
    set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")
endif()

cmake_minimum_required(VERSION 3.20)
project(stage8_toml_reflection LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS ON)

# ===========================================================================
# 依赖：全部 FetchContent（SOURCE_SUBDIR 指向不存在目录 = 只下载不执行其
# CMake，纯头文件使用）
# ===========================================================================
include(FetchContent)

# tomlplusplus：与项目 dependencies.cmake 同一 commit。
# 项目 toml/detail/common.hpp 强制 TOML_HEADER_ONLY=0 + TOML_EXCEPTIONS=0，
# 因此必须像主项目 toml_impl 一样把 src/toml.cpp 编成静态库
FetchContent_Declare(
    tomlplusplus
    URL https://github.com/marzer/tomlplusplus/archive/1c8b7466e4946fcc3bf20484c0e1d001202cca5a.tar.gz
    SOURCE_SUBDIR __headers_only__
)
FetchContent_MakeAvailable(tomlplusplus)

add_library(toml_impl STATIC ${tomlplusplus_SOURCE_DIR}/src/toml.cpp)
target_include_directories(toml_impl SYSTEM PUBLIC ${tomlplusplus_SOURCE_DIR}/include)
target_compile_definitions(toml_impl PUBLIC
    TOML_HEADER_ONLY=0
    TOML_EXCEPTIONS=0
)

# Boost.PFR：field_reflection.hpp 需要 names_as_array / tuple_size_v
FetchContent_Declare(
    boost_pfr
    URL https://github.com/boostorg/pfr/archive/refs/tags/2.2.0.tar.gz
    SOURCE_SUBDIR __headers_only__
)
FetchContent_MakeAvailable(boost_pfr)

# fmt：编译为静态库（与项目 fetch_dependency(fmt 12.0.0) 等价）
set(FMT_TEST OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    fmt
    URL https://github.com/fmtlib/fmt/archive/refs/tags/12.0.0.tar.gz
)
FetchContent_MakeAvailable(fmt)

# ===========================================================================
# toml 模块（头文件库）：include 指向真实 crates/toml/src + 3dparty/magic_enum
# ===========================================================================
add_library(talos_toml INTERFACE)
target_include_directories(talos_toml SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../crates/toml/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3dparty/magic_enum
    ${tomlplusplus_SOURCE_DIR}/include
    ${boost_pfr_SOURCE_DIR}/include
)
# magic_enum 枚举扫描范围限定 0~16（对齐项目主 CMake，压缩模板膨胀）
target_compile_definitions(talos_toml INTERFACE
    MAGIC_ENUM_RANGE_MIN=0
    MAGIC_ENUM_RANGE_MAX=16
)
target_link_libraries(talos_toml INTERFACE fmt::fmt toml_impl)

# 演示程序：警告只作用于本 demo
add_executable(toml_reflection_demo src/demo.cpp)
target_compile_options(toml_reflection_demo PRIVATE -Wall -Wextra -Werror)
target_link_libraries(toml_reflection_demo PRIVATE talos_toml)
