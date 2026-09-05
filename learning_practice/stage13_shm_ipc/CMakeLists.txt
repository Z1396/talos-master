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
    set(CMAKE_C_COMPILER /usr/bin/clang-21)      # C 编译器
    set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)  # C++ 编译器
    set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")     # 使用 GNU 标准库（而非 libc++）
endif()

# 指定 CMake 最低版本为 3.20，项目名称，仅使用 C++ 语言
cmake_minimum_required(VERSION 3.20)
project(stage13_shm_ipc LANGUAGES CXX)

# 强制使用 C++23 标准，且必须支持
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)           # 允许编译器扩展（如 GNU 扩展）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # 生成 compile_commands.json 供工具使用

include(FetchContent)  # 用于自动下载依赖

# 查找系统依赖：线程库和 OpenCV（仅需 core 组件）
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

# ===========================================================================
# 依赖一：fmt 12.0.0（shm_region.hpp / demo 的 fmt::print）
# 优先复用 3dparty 缓存 zip（构建过主项目即存在），否则在线拉取
# ===========================================================================
set(FMT_ZIP ${CMAKE_CURRENT_SOURCE_DIR}/../../3dparty/fmt-12.0.0.zip)
if(EXISTS ${FMT_ZIP})
    # 如果本地已有 zip 包，直接用本地文件（加速构建）
    FetchContent_Declare(fmt URL ${FMT_ZIP})
else()
    # 否则从 GitHub 官方仓库下载
    FetchContent_Declare(
        fmt URL https://github.com/fmtlib/fmt/releases/download/12.0.0/fmt-12.0.0.zip)
endif()
FetchContent_MakeAvailable(fmt)  # 下载并构建 fmt 库

# ===========================================================================
# 依赖二：magic_enum（仓库自带单头文件，ShmError 枚举反射打印）
# ===========================================================================
set(MAGIC_ENUM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../3dparty/magic_enum)
if(NOT EXISTS ${MAGIC_ENUM_DIR}/magic_enum.hpp)
    # 如果缺失关键头文件，立即报错终止配置
    message(FATAL_ERROR "missing 3dparty/magic_enum/magic_enum.hpp in repo")
endif()

# ===========================================================================
# talos_ipc 模块（头文件库）：include 指向真实
# crates/hardware_daedalus/src —— demo #include "shm_client.hpp" 即与主程序
# 使用完全相同的 IPC API。SYSTEM：头文件内部告警不传染 demo 的 -Werror
# ===========================================================================
add_library(talos_ipc_learning INTERFACE)
# INTERFACE 库：只提供头文件和链接依赖，不生成编译产物
target_include_directories(talos_ipc_learning SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../crates/hardware_daedalus/src  # IPC 头文件路径
    ${MAGIC_ENUM_DIR}  # magic_enum 头文件路径
)
target_link_libraries(talos_ipc_learning INTERFACE
    fmt::fmt              # 格式化库
    ${OpenCV_LIBS}        # OpenCV 库（cv::Mat 所需）
    Threads::Threads      # 线程库（用于同步原语）
)

# 三个可执行目标：警告只作用于本 demo 源码
# ---------- demo1: 单进程功能自测 ----------
add_executable(ipc_demo src/ipc_demo.cpp)
# 开启严格编译警告，并将警告视为错误（确保代码质量）
target_compile_options(ipc_demo PRIVATE -Wall -Wextra -Wpedantic -Werror)
# ipc_demo 用 assert 做断言：Release 构建默认 -DNDEBUG 会把断言整个编译掉，
# 测试"假通过"。生成器表达式仅在 Release 配置注入 -UNDEBUG 强制保留断言
# 解释：$<$<CONFIG:Release>:-UNDEBUG> 表示如果配置是 Release，添加 -UNDEBUG 编译选项
# 这会覆盖默认的 -DNDEBUG，确保 assert 在 Release 下也生效
target_compile_options(ipc_demo PRIVATE $<$<CONFIG:Release>:-UNDEBUG>)
target_link_libraries(ipc_demo PRIVATE talos_ipc_learning)

# ---------- demo2: 双进程生产者 ----------
add_executable(sim_producer src/sim_producer.cpp)
target_compile_options(sim_producer PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(sim_producer PRIVATE talos_ipc_learning)

# ---------- demo3: 双进程消费者 ----------
add_executable(vision_consumer src/vision_consumer.cpp)
target_compile_options(vision_consumer PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(vision_consumer PRIVATE talos_ipc_learning)