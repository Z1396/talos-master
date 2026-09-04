# ===========================================================================
# stage13_shm_ipc：共享内存 IPC（Talos ↔ Daedalus 零拷贝协作）学习案例
#
# 与 crates/hardware_daedalus/tests/shm_ipc_test.cpp 的分工：
#   那边 = gtest 单元测试（CI 跑，覆盖边界/错误路径）；
#   这边 = 学习案例，三个可独立运行的 demo：
#     1. ipc_demo         单进程三层功能自测（ShmRegion→三缓冲→ShmClient）
#     2. sim_producer     双进程演示·生产者端（模拟 Daedalus/Rust 仿真器）
#     3. vision_consumer  双进程演示·消费者端（模拟 Talos 视觉程序）
#     双进程 demo 真实验证 MAP_SHARED 跨进程可见性与 latest-wins 语义
#
# 依赖（与主项目 versions 一致）：
#   hardware_daedalus  直接 include 真实源码头文件（../../crates/.../src），零复制
#   fmt 12.0.0         shm_region.hpp 的 fmt::formatter<ShmError> 需要 fmt>=10
#                      的 const format 签名，系统 fmt 8 不兼容 → FetchContent
#   magic_enum         仓库自带 3dparty/magic_enum 单头文件，零拉取
#   OpenCV core        shm_client.hpp 需要 cv::Mat（零拷贝图像映射）
# ===========================================================================

# 编译器选择：优先学习环境统一的 clang-21，不存在时回退系统默认
if(EXISTS /usr/bin/clang++-21)
    set(CMAKE_C_COMPILER /usr/bin/clang-21)
    set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
    set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")
endif()

cmake_minimum_required(VERSION 3.20)
project(stage13_shm_ipc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

# ===========================================================================
# 依赖一：fmt 12.0.0（shm_region.hpp / demo 的 fmt::print）
# 优先复用 3dparty 缓存 zip（构建过主项目即存在），否则在线拉取
# ===========================================================================
set(FMT_ZIP ${CMAKE_CURRENT_SOURCE_DIR}/../../3dparty/fmt-12.0.0.zip)
if(EXISTS ${FMT_ZIP})
    FetchContent_Declare(fmt URL ${FMT_ZIP})
else()
    FetchContent_Declare(
        fmt URL https://github.com/fmtlib/fmt/releases/download/12.0.0/fmt-12.0.0.zip)
endif()
FetchContent_MakeAvailable(fmt)

# ===========================================================================
# 依赖二：magic_enum（仓库自带单头文件，ShmError 枚举反射打印）
# ===========================================================================
set(MAGIC_ENUM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../3dparty/magic_enum)
if(NOT EXISTS ${MAGIC_ENUM_DIR}/magic_enum.hpp)
    message(FATAL_ERROR "missing 3dparty/magic_enum/magic_enum.hpp in repo")
endif()

# ===========================================================================
# talos_ipc 模块（头文件库）：include 指向真实
# crates/hardware_daedalus/src —— demo #include "shm_client.hpp" 即与主程序
# 使用完全相同的 IPC API。SYSTEM：头文件内部告警不传染 demo 的 -Werror
# ===========================================================================
add_library(talos_ipc_learning INTERFACE)
target_include_directories(talos_ipc_learning SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../crates/hardware_daedalus/src
    ${MAGIC_ENUM_DIR}
)
target_link_libraries(talos_ipc_learning INTERFACE
    fmt::fmt
    ${OpenCV_LIBS}
    Threads::Threads
)

# 三个可执行目标：警告只作用于本 demo 源码
add_executable(ipc_demo src/ipc_demo.cpp)
target_compile_options(ipc_demo PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(ipc_demo PRIVATE talos_ipc_learning)

add_executable(sim_producer src/sim_producer.cpp)
target_compile_options(sim_producer PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(sim_producer PRIVATE talos_ipc_learning)

add_executable(vision_consumer src/vision_consumer.cpp)
target_compile_options(vision_consumer PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(vision_consumer PRIVATE talos_ipc_learning)
