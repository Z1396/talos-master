# ===========================================================================
# stage9_fast_tf：fast_tf 模块（crates/fast_tf，2,713 行，1 个 .cpp）
#   - frame.hpp    帧树：DECL 宏 + frame concept + is_descendant_of 编译期校验
#   - matrix.hpp   强类型 SE(3)：TransformMatrix<T, A, B>（A/B 是帧类型参数）
#   - buffer.hpp   时序环形缓冲：exact/nearest/interpolate/clamped 四种查询
#   - validation   变换合法性校验（NaN/Inf/四元数归一化，.cpp 显式实例化）
#   - euler.hpp    来自 crates/math 的 ROS2 RPY 欧拉角
# demo 直接 #include 项目真实头文件（include 指向 crates/fast_tf/src），
# 真实 validation.cpp 编译进静态库，不复制任何代码。
#
# 依赖（对齐 crates/fast_tf 的 crate() 声明：talos_math fmt lieplusplus TBB opencv_core）：
#   - Eigen 3.4.0   FetchContent（euler.hpp / lieplusplus / matrix.hpp 需要）
#   - fmt 12.0.0    FetchContent（buffer/frame 错误信息拼接）
#   - oneTBB 2021.11.0 FetchContent——仅取头文件：buffer.hpp 只用
#     tbb::spin_rw_mutex，实测纯 header-only 可编译链接，无需编 TBB 库
#   - OpenCV：matrix.hpp 仅用 cv::Vec<T,3>（from_pnp 接口）。
#     为免拉整套 OpenCV，compat/opencv2/core/types.hpp 提供最小 cv::Vec 替身
# ===========================================================================

# 编译器选择：优先学习环境统一的 clang-21，其次 clang-18
# （frame.hpp 的 Fuck<Frame,T> 用了 C++23 deducing this 显式对象参数，
#   GCC 13 尚不支持，clang-18 起才完备），都不存在时回退系统默认
if(EXISTS /usr/bin/clang++-21)
    set(CMAKE_C_COMPILER /usr/bin/clang-21)
    set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
    set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")
elseif(EXISTS /usr/bin/clang++-18)
    set(CMAKE_C_COMPILER /usr/bin/clang-18)
    set(CMAKE_CXX_COMPILER /usr/bin/clang++-18)
    # clang-18 报告 __cpp_concepts=201907L（clang-19 起才为 202002L），
    # libstdc++-13 的 <expected> 以 >= 202002L 做门卫，会整个静默失效。
    # clang-18 的 concepts 实现已完整，这里显式覆盖该特性宏 + 关掉重定义警告。
    set(CMAKE_CXX_FLAGS "-stdlib=libstdc++ -D__cpp_concepts=202002L -Wno-builtin-macro-redefined")
endif()

cmake_minimum_required(VERSION 3.20)
project(stage9_fast_tf LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS ON)

include(FetchContent)

# ===========================================================================
# Eigen：优先系统版本，否则 FetchContent 3.4.0（header-only，不执行其 CMake）
# ===========================================================================
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
    message(STATUS "stage9: 系统未安装 Eigen，FetchContent 拉取 3.4.0（header-only）")
    FetchContent_Declare(
        eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
        SOURCE_SUBDIR __headers_only__
    )
    FetchContent_MakeAvailable(eigen)
    add_library(stage9_eigen INTERFACE)
    target_include_directories(stage9_eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
    set(EIGEN_TARGET stage9_eigen)
else()
    message(STATUS "stage9: 使用系统 Eigen ${Eigen3_VERSION}")
    set(EIGEN_TARGET Eigen3::Eigen)
endif()

# ===========================================================================
# oneTBB：只下载头文件（SOURCE_SUBDIR 指向不存在目录 = 不执行其 CMake）。
# buffer.hpp 仅使用 tbb::spin_rw_mutex（读写自旋锁），该头文件是
# header-only 实现，不需要链接 libtbb。
# ===========================================================================
FetchContent_Declare(
    onetbb
    URL https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.11.0.tar.gz
    SOURCE_SUBDIR __headers_only__
)
FetchContent_MakeAvailable(onetbb)

# ===========================================================================
# fmt 12.0.0：编译为静态库（与项目 fetch_dependency(fmt 12.0.0) 等价）
# ===========================================================================
set(FMT_TEST OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    fmt
    URL https://github.com/fmtlib/fmt/archive/refs/tags/12.0.0.tar.gz
)
FetchContent_MakeAvailable(fmt)

# ===========================================================================
# talos_math（crates/math 头文件库）+ lieplusplus（3dparty 李群实现）
# ===========================================================================
add_library(talos_math INTERFACE)
target_include_directories(talos_math SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../crates/math/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3dparty/lieplusplus
)
target_link_libraries(talos_math INTERFACE ${EIGEN_TARGET})

# ===========================================================================
# fast_tf 真实库：validation.cpp（显式实例化全部帧组合）+ 头文件目录。
# include 顺序：compat/（cv::Vec 替身）→ 真实源码目录
# ===========================================================================
set(FASTTF_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../crates/fast_tf/src)

add_library(talos_fast_tf STATIC ${FASTTF_DIR}/validation.cpp)
target_include_directories(talos_fast_tf SYSTEM PUBLIC
    ${FASTTF_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/compat
    ${onetbb_SOURCE_DIR}/include
)
target_compile_features(talos_fast_tf PUBLIC cxx_std_23)
target_link_libraries(talos_fast_tf PUBLIC talos_math fmt::fmt)

# 演示程序：警告只作用于本 demo
add_executable(fast_tf_demo src/demo.cpp)
target_compile_options(fast_tf_demo PRIVATE -Wall -Wextra -Werror)
target_link_libraries(fast_tf_demo PRIVATE talos_fast_tf)
