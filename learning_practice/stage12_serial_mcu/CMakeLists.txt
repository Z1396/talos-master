# ===========================================================================
# stage12_serial_mcu：真实下位机（STM32 云台主控）串口通信实测程序
#
# 与 crates/hardware/at_gimbal/tests/serial_protocol_test.cpp 的分工：
#   那边 = PTY 伪终端【模拟】下位机的单元测试，无硬件、CI 可跑；
#   这边 = 部署在机器人上对【真实 STM32】运行的实测验收程序：
#     - 真实 /dev/ttyS*、真实波特率、真实 termios 配置
#     - 验收上行链路：IMU 帧率 / 时间戳单调性 / 帧完整性（SoF/EoF）
#     - 验收下行链路：视觉指令帧下发（可选，默认关闭——云台会真实转动）
#     - 验收稳定性：长时间运行统计、断线自动重连
#
# 依赖（与主项目 versions 完全一致）：
#   talos_gimbal  直接 include 真实源码头文件（../../crates/.../src）
#   fmt 12.0.0    优先复用 3dparty 缓存 zip（构建过主项目即存在），否则在线拉取
#   spdlog 1.15.1 同上；SPDLOG_FMT_EXTERNAL=ON 绑定上面的外部 fmt
#   magic_enum    仓库自带 3dparty/magic_enum 单头文件，零拉取
#   libusb 1.0.27 仅因 usb.hpp 头文件包含而依赖（Stm32Impl 模板不实例化不链接），
#                静态编译并关闭 udev，最小化构建要求
# ===========================================================================

# 编译器选择：优先学习环境统一的 clang-21，不存在时回退系统默认
if(EXISTS /usr/bin/clang++-21)
    set(CMAKE_C_COMPILER /usr/bin/clang-21)
    set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
    set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")
endif()

cmake_minimum_required(VERSION 3.20)
project(stage12_serial_mcu LANGUAGES C CXX) # C：libusb 源码是纯 C 编译

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS ON)

include(FetchContent)

find_package(Threads REQUIRED) # pthread：spdlog/libusb/周期节拍共同依赖

# ===========================================================================
# 依赖一：fmt 12.0.0（serial.hpp/packet.hpp 直接 fmt::format）
# ===========================================================================
set(FMT_ZIP ${CMAKE_CURRENT_SOURCE_DIR}/../../3dparty/fmt-12.0.0.zip)
if(EXISTS ${FMT_ZIP})
    # 离线优先：主项目 fetch-content-helper 已缓存到 3dparty
    FetchContent_Declare(fmt URL ${FMT_ZIP})
else()
    FetchContent_Declare(
        fmt URL https://github.com/fmtlib/fmt/releases/download/12.0.0/fmt-12.0.0.zip)
endif()
FetchContent_MakeAvailable(fmt)

# ===========================================================================
# 依赖二：spdlog 1.15.1（serial.hpp/stm32.hpp 的 SPDLOG_* 日志）
# SPDLOG_FMT_EXTERNAL=ON：使用上面的外部 fmt，避免内置 fmt 版本冲突，
# 与主项目 cmake/dependencies.cmake 的做法完全一致
# ===========================================================================
set(SPDLOG_FMT_EXTERNAL ON CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_BENCH OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(SPDLOG_ZIP ${CMAKE_CURRENT_SOURCE_DIR}/../../3dparty/spdlog-1.15.1.zip)
if(EXISTS ${SPDLOG_ZIP})
    FetchContent_Declare(spdlog URL ${SPDLOG_ZIP})
else()
    FetchContent_Declare(
        spdlog URL https://codeload.github.com/gabime/spdlog/zip/refs/tags/v1.15.1)
endif()
FetchContent_MakeAvailable(spdlog)

# ===========================================================================
# 依赖三：magic_enum（仓库自带单头文件，packet.hpp 枚举反射）
# ===========================================================================
set(MAGIC_ENUM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../3dparty/magic_enum)
if(NOT EXISTS ${MAGIC_ENUM_DIR}/magic_enum.hpp)
    message(FATAL_ERROR "missing 3dparty/magic_enum/magic_enum.hpp in repo")
endif()

# ===========================================================================
# 依赖四：libusb 1.0.30 源码
# usb.hpp（serial.hpp 包含它以获得 Parser 概念）include <libusb/libusb.h>，
# 故必须提供 libusb 头文件与少量符号（mcu_device variant 析构 Stm32Impl）。
# 照搬主项目 cmake/dependencies.cmake 的做法：拉源码手动编译静态库，
# 不依赖系统 libusb —— 机器人上无需额外 apt 安装。
# 注意：libusb v1.0.27 及更早版本没有 CMakeLists.txt，必须用 v1.0.30 源码。
# ===========================================================================
FetchContent_Declare(libusb
    URL https://codeload.github.com/libusb/libusb/tar.gz/refs/tags/v1.0.30)
# Populate 只下载展开源码，不 add_subdirectory（手动挑选 .c 编译）
FetchContent_Populate(libusb)

# 生成 libusb 编译所需的跨平台 config.h（与主项目完全一致）
set(LIBUSB_CFG_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/libusb)
file(MAKE_DIRECTORY ${LIBUSB_CFG_DIR})
file(WRITE ${LIBUSB_CFG_DIR}/config.h
"#pragma once
#define DEFAULT_VISIBILITY __attribute__ ((visibility (\"hidden\")))
#define ENABLE_LOGGING 1
#define HAVE_NFDS_T 1
#define HAVE_SYS_TIME_H 1
#define HAVE_PLATFORM_POSIX 1
#define PLATFORM_POSIX 1
#define PRINTF_FORMAT(a, b) __attribute__ ((__format__ (__printf__, a, b)))
#define _GNU_SOURCE 1
")
# Linux 平台专属宏
file(APPEND ${LIBUSB_CFG_DIR}/config.h
"#define HAVE_ASM_TYPES_H 1
#define HAVE_CLOCK_GETTIME 1
#define HAVE_EVENTFD 1
#define HAVE_PIPE2 1
#define HAVE_PTHREAD_CONDATTR_SETCLOCK 1
#define HAVE_TIMERFD 1
")

# 手动编译 libusb 静态库（核心源码 + Linux USB 底层实现）
add_library(talos_libusb_static STATIC
    ${libusb_SOURCE_DIR}/libusb/core.c
    ${libusb_SOURCE_DIR}/libusb/descriptor.c
    ${libusb_SOURCE_DIR}/libusb/hotplug.c
    ${libusb_SOURCE_DIR}/libusb/io.c
    ${libusb_SOURCE_DIR}/libusb/strerror.c
    ${libusb_SOURCE_DIR}/libusb/sync.c
    ${libusb_SOURCE_DIR}/libusb/os/events_posix.c
    ${libusb_SOURCE_DIR}/libusb/os/threads_posix.c
    ${libusb_SOURCE_DIR}/libusb/os/linux_usbfs.c
    ${libusb_SOURCE_DIR}/libusb/os/linux_netlink.c
)
# SYSTEM：头文件告警不传染 demo；PUBLIC：usb.hpp 能找到 <libusb/libusb.h>
target_include_directories(talos_libusb_static SYSTEM PUBLIC ${libusb_SOURCE_DIR})
target_include_directories(talos_libusb_static PRIVATE
    ${libusb_SOURCE_DIR}/libusb
    ${LIBUSB_CFG_DIR}
)
target_compile_definitions(talos_libusb_static PRIVATE _GNU_SOURCE)
target_compile_options(talos_libusb_static PRIVATE -w -fvisibility=hidden)
target_link_libraries(talos_libusb_static PUBLIC Threads::Threads rt)

# ===========================================================================
# talos_gimbal 模块（头文件库）：include 指向真实
# crates/hardware/at_gimbal/src —— demo #include "talos_gimbal/mcu_device.hpp"
# 即与主程序使用完全相同的通信 API，零拷贝、零重写
# ===========================================================================
add_library(talos_gimbal_learning INTERFACE)
target_include_directories(talos_gimbal_learning INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../crates/hardware/at_gimbal/src
    ${MAGIC_ENUM_DIR}
)
target_link_libraries(talos_gimbal_learning INTERFACE
    fmt::fmt
    spdlog::spdlog
    talos_libusb_static
    Threads::Threads
)

# 演示程序：警告只作用于本 demo
add_executable(serial_mcu_demo src/demo.cpp)
target_compile_options(serial_mcu_demo PRIVATE -Wall -Wextra -Werror)
target_link_libraries(serial_mcu_demo PRIVATE talos_gimbal_learning)
