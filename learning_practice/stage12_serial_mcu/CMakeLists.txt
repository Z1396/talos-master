# 编译器选择：优先学习环境统一的 clang-21，不存在时回退系统默认
if(EXISTS /usr/bin/clang++-21)
    set(CMAKE_C_COMPILER /usr/bin/clang-21)
    set(CMAKE_CXX_COMPILER /usr/bin/clang++-21)
    set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")
endif()

cmake_minimum_required(VERSION 3.20)
project(stage12_serial_mcu LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS ON)

include(FetchContent)
find_package(Threads REQUIRED)

# ============================================================================
# fmt 12.0.0
# ============================================================================
set(FMT_ZIP ${CMAKE_CURRENT_SOURCE_DIR}/../../3dparty/fmt-12.0.0.zip)
if(EXISTS ${FMT_ZIP})
    FetchContent_Declare(fmt URL ${FMT_ZIP})
else()
    FetchContent_Declare(
        fmt URL https://github.com/fmtlib/fmt/releases/download/12.0.0/fmt-12.0.0.zip)
endif()
FetchContent_MakeAvailable(fmt)

# ============================================================================
# spdlog 1.15.1
# ============================================================================
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

# ============================================================================
# magic_enum
# ============================================================================
set(MAGIC_ENUM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../3dparty/magic_enum)
if(NOT EXISTS ${MAGIC_ENUM_DIR}/magic_enum.hpp)
    message(FATAL_ERROR "missing 3dparty/magic_enum/magic_enum.hpp in repo")
endif()

# ============================================================================
# libusb 1.0.30
# ============================================================================
FetchContent_Declare(libusb
    URL https://codeload.github.com/libusb/libusb/tar.gz/refs/tags/v1.0.30)
FetchContent_Populate(libusb)

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
#define HAVE_ASM_TYPES_H 1
#define HAVE_CLOCK_GETTIME 1
#define HAVE_EVENTFD 1
#define HAVE_PIPE2 1
#define HAVE_PTHREAD_CONDATTR_SETCLOCK 1
#define HAVE_TIMERFD 1
")

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

target_include_directories(talos_libusb_static SYSTEM PUBLIC ${libusb_SOURCE_DIR})
target_include_directories(talos_libusb_static PRIVATE
    ${libusb_SOURCE_DIR}/libusb
    ${LIBUSB_CFG_DIR}
)
target_compile_definitions(talos_libusb_static PRIVATE _GNU_SOURCE)
target_compile_options(talos_libusb_static PRIVATE -w -fvisibility=hidden)
target_link_libraries(talos_libusb_static PUBLIC Threads::Threads rt)

# ============================================================================
# talos_gimbal
# ============================================================================
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

# ============================================================================
# 可执行文件
# ============================================================================
add_executable(serial_mcu_demo
    src/main.cpp
    src/options.cpp
    src/utils.cpp
    src/serial_test.cpp
)

target_compile_options(serial_mcu_demo PRIVATE -Wall -Wextra -Werror)
target_link_libraries(serial_mcu_demo PRIVATE talos_gimbal_learning)