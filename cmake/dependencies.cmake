set(CMAKE_WARN_DEPRECATED OFF)
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 OLD)
endif()

function(_talos_relocate_sysroot_usr_path out_var in_value)
    set(_fixed "${in_value}")

    if (NOT CMAKE_CROSSCOMPILING OR NOT TALOS_SYSROOT)
        set(${out_var} "${_fixed}" PARENT_SCOPE)
        return()
    endif ()

    if (_fixed MATCHES "^\\$<" OR NOT IS_ABSOLUTE "${_fixed}" OR EXISTS "${_fixed}")
        set(${out_var} "${_fixed}" PARENT_SCOPE)
        return()
    endif ()

    string(FIND "${_fixed}" "${TALOS_SYSROOT}/" _sysroot_prefix_pos)
    if (NOT _sysroot_prefix_pos EQUAL 0)
        set(${out_var} "${_fixed}" PARENT_SCOPE)
        return()
    endif ()

    string(LENGTH "${TALOS_SYSROOT}/" _sysroot_prefix_len)
    string(SUBSTRING "${_fixed}" ${_sysroot_prefix_len} -1 _relative_to_sysroot)

    if (_relative_to_sysroot MATCHES "^(include|lib|lib64|share|bin)(/.*)?$")
        set(_candidate "${TALOS_SYSROOT}/usr/${_relative_to_sysroot}")
        if (EXISTS "${_candidate}")
            set(_fixed "${_candidate}")
        endif ()
    endif ()

    set(${out_var} "${_fixed}" PARENT_SCOPE)
endfunction()

function(_talos_fix_imported_target_sysroot target)
    if (NOT TARGET "${target}")
        return()
    endif ()

    get_target_property(_is_imported "${target}" IMPORTED)
    if (NOT _is_imported)
        return()
    endif ()

    foreach(_prop IN ITEMS
            INTERFACE_INCLUDE_DIRECTORIES
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
            INTERFACE_LINK_DIRECTORIES
            INTERFACE_LINK_LIBRARIES)
        get_target_property(_values "${target}" ${_prop})
        if (NOT _values OR _values STREQUAL "${_prop}-NOTFOUND")
            continue()
        endif ()

        set(_rewritten_values "")
        set(_changed FALSE)
        foreach(_value IN LISTS _values)
            _talos_relocate_sysroot_usr_path(_rewritten_value "${_value}")
            list(APPEND _rewritten_values "${_rewritten_value}")
            if (NOT "${_rewritten_value}" STREQUAL "${_value}")
                set(_changed TRUE)
            endif ()
        endforeach ()

        if (_changed)
            set_property(TARGET "${target}" PROPERTY ${_prop} "${_rewritten_values}")
        endif ()
    endforeach ()

    set(_import_location_props IMPORTED_LOCATION IMPORTED_IMPLIB)
    get_target_property(_import_configs "${target}" IMPORTED_CONFIGURATIONS)
    if (_import_configs AND NOT _import_configs STREQUAL "_import_configs-NOTFOUND")
        foreach(_config IN LISTS _import_configs)
            string(TOUPPER "${_config}" _config_upper)
            list(APPEND _import_location_props
                "IMPORTED_LOCATION_${_config_upper}"
                "IMPORTED_IMPLIB_${_config_upper}")
        endforeach ()
    endif ()

    foreach(_prop IN LISTS _import_location_props)
        get_target_property(_value "${target}" ${_prop})
        if (NOT _value OR _value STREQUAL "${_prop}-NOTFOUND")
            continue()
        endif ()

        _talos_relocate_sysroot_usr_path(_rewritten_value "${_value}")
        if (NOT "${_rewritten_value}" STREQUAL "${_value}")
            set_property(TARGET "${target}" PROPERTY ${_prop} "${_rewritten_value}")
        endif ()
    endforeach ()
endfunction()

function(talos_fixup_cross_sysroot_imported_targets)
    if (NOT CMAKE_CROSSCOMPILING OR NOT TALOS_SYSROOT)
        return()
    endif ()

    get_cmake_property(_all_targets TARGETS)
    foreach(_target IN LISTS _all_targets)
        _talos_fix_imported_target_sysroot("${_target}")
    endforeach ()
endfunction()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d dnn)

# Include FetchContent helper
include(${CMAKE_CURRENT_LIST_DIR}/fetch-content-helper.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/ffmpeg.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/fetch-onnxruntime.cmake)

# fmt from local 12.0.0 (system 8.x and 10.x have consteval issues with Clang 21)
fetch_dependency(NAME fmt
                 ZIP_URL "${CMAKE_SOURCE_DIR}/3dparty/fmt-12.0.0.zip"
                 ZIP_NAME "fmt-12.0.0.zip")
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# spdlog from local 1.15.1 (system 1.9.2 incompatible with fmt 12)
set(SPDLOG_FMT_EXTERNAL ON CACHE BOOL "" FORCE)
set(SPDLOG_ENABLE_PCH ON CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_SHARED OFF CACHE BOOL "" FORCE)
fetch_dependency(NAME spdlog
                 ZIP_URL "${CMAKE_SOURCE_DIR}/3dparty/spdlog-1.15.1.zip"
                 ZIP_NAME "spdlog-1.15.1.zip")

# nlohmann_json from system
find_package(nlohmann_json REQUIRED)

# boost::pfr - use local git checkout (GitHub download times out)
add_library(Boost::pfr INTERFACE IMPORTED)
if(EXISTS "${CMAKE_SOURCE_DIR}/3dparty/boost-pfr/include/boost/pfr.hpp")
    target_include_directories(Boost::pfr INTERFACE ${CMAKE_SOURCE_DIR}/3dparty/boost-pfr/include)
else()
    set(_pfr_stub_dir "${CMAKE_BINARY_DIR}/_deps/boost_pfr-src/include")
    if(NOT EXISTS "${_pfr_stub_dir}/boost/pfr.hpp")
        file(MAKE_DIRECTORY "${_pfr_stub_dir}/boost")
        file(WRITE "${_pfr_stub_dir}/boost/pfr.hpp" "// boost::pfr stub\n#pragma once\n#include <tuple>\nnamespace boost::pfr {\ntemplate<class T> constexpr auto fields_as_tuple(T&&) { return std::tuple<>{}; }\ntemplate<class T> constexpr auto get_name(std::size_t) { return \"\"; }\n}\n")
    endif()
    target_include_directories(Boost::pfr INTERFACE ${_pfr_stub_dir})
endif()

# Eigen3 from system (downloaded 5.0.1 has Wcast-qual issues with Clang 21)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

fetch_dependency(NAME tomlplusplus
                 REPO marzer/tomlplusplus
                 VERSION 1c8b7466e4946fcc3bf20484c0e1d001202cca5a)

include(cmake/tbb.cmake)
find_package(TBB REQUIRED)
sanitize_tbb()

if (BUILD_TESTING)
    find_package(GTest REQUIRED)
endif ()
find_package(Ceres REQUIRED)

# TensorRT configuration with better error handling
find_package(CUDA QUIET)
if (CUDA_FOUND)
    message(STATUS "CUDA found at: ${CUDA_TOOLKIT_ROOT_DIR}")
else ()
    message(STATUS "CUDA not found - TensorRT backend will be disabled")
endif ()

find_package(TensorRT QUIET MODULE)
if (TensorRT_FOUND)
    message(STATUS "TensorRT found at: ${TensorRT_DIR}")
else ()
    message(STATUS "TensorRT not found - TensorRT backend will be disabled")
endif ()

find_package(benchmark QUIET)

find_package(PkgConfig REQUIRED)
talos_configure_ffmpeg()

if (TALOS_FFMPEG_ROOT AND EXISTS "${TALOS_FFMPEG_ROOT}/lib")
    list(APPEND CMAKE_BUILD_RPATH "${TALOS_FFMPEG_ROOT}/lib")
    list(APPEND CMAKE_INSTALL_RPATH "${TALOS_FFMPEG_ROOT}/lib")
    list(REMOVE_DUPLICATES CMAKE_BUILD_RPATH)
    list(REMOVE_DUPLICATES CMAKE_INSTALL_RPATH)
endif ()

fetch_dependency(NAME libusb
                 REPO libusb/libusb
                 VERSION v1.0.30)
FetchContent_GetProperties(libusb SOURCE_DIR TALOS_LIBUSB_SOURCE_DIR)

find_package(Threads REQUIRED)

set(TALOS_LIBUSB_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/libusb")
file(MAKE_DIRECTORY "${TALOS_LIBUSB_CONFIG_DIR}")
file(WRITE "${TALOS_LIBUSB_CONFIG_DIR}/config.h"
"/* Generated by Talos. Keep the MCU libusb build private to this process. */
#pragma once

#define DEFAULT_VISIBILITY __attribute__ ((visibility (\"hidden\")))
#define ENABLE_LOGGING 1
#define HAVE_NFDS_T 1
#define HAVE_SYS_TIME_H 1
#define PLATFORM_POSIX 1
#define PRINTF_FORMAT(a, b) __attribute__ ((__format__ (__printf__, a, b)))
#define _GNU_SOURCE 1
")

set(TALOS_LIBUSB_OS_SOURCES "")
set(TALOS_LIBUSB_OS_LIBS "")

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    file(APPEND "${TALOS_LIBUSB_CONFIG_DIR}/config.h"
"#define HAVE_ASM_TYPES_H 1
#define HAVE_CLOCK_GETTIME 1
#define HAVE_EVENTFD 1
#define HAVE_PIPE2 1
#define HAVE_PTHREAD_CONDATTR_SETCLOCK 1
#define HAVE_TIMERFD 1
")
    list(APPEND TALOS_LIBUSB_OS_SOURCES
        "${TALOS_LIBUSB_SOURCE_DIR}/libusb/os/linux_usbfs.c"
        "${TALOS_LIBUSB_SOURCE_DIR}/libusb/os/linux_netlink.c"
    )
    list(APPEND TALOS_LIBUSB_OS_LIBS rt)
elseif (APPLE)
    list(APPEND TALOS_LIBUSB_OS_SOURCES
        "${TALOS_LIBUSB_SOURCE_DIR}/libusb/os/darwin_usb.c"
    )
    list(APPEND TALOS_LIBUSB_OS_LIBS
        "-framework CoreFoundation"
        "-framework IOKit"
        "-framework Security"
    )
else ()
    message(FATAL_ERROR "talos_libusb_static is only configured for Linux and Darwin")
endif ()

add_library(talos_libusb_static STATIC
    "${TALOS_LIBUSB_SOURCE_DIR}/libusb/core.c"
    "${TALOS_LIBUSB_SOURCE_DIR}/libusb/descriptor.c"
    "${TALOS_LIBUSB_SOURCE_DIR}/libusb/hotplug.c"
    "${TALOS_LIBUSB_SOURCE_DIR}/libusb/io.c"
    "${TALOS_LIBUSB_SOURCE_DIR}/libusb/strerror.c"
    "${TALOS_LIBUSB_SOURCE_DIR}/libusb/sync.c"
    "${TALOS_LIBUSB_SOURCE_DIR}/libusb/os/events_posix.c"
    "${TALOS_LIBUSB_SOURCE_DIR}/libusb/os/threads_posix.c"
    ${TALOS_LIBUSB_OS_SOURCES}
)
target_include_directories(talos_libusb_static SYSTEM PUBLIC
    "${TALOS_LIBUSB_SOURCE_DIR}"
)
target_include_directories(talos_libusb_static PRIVATE
    "${TALOS_LIBUSB_SOURCE_DIR}/libusb"
    "${TALOS_LIBUSB_CONFIG_DIR}"
)
target_compile_definitions(talos_libusb_static PRIVATE _GNU_SOURCE)
if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    target_compile_options(talos_libusb_static PRIVATE -w -fvisibility=hidden)
endif ()
target_link_libraries(talos_libusb_static PUBLIC
    Threads::Threads
    ${TALOS_LIBUSB_OS_LIBS}
)
target_link_options(talos_libusb_static INTERFACE
    "$<$<AND:$<PLATFORM_ID:Linux>,$<NOT:$<BOOL:${ENABLE_ASAN}>>>:-Wl,--exclude-libs,ALL>"
)
set_target_properties(talos_libusb_static PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

talos_fixup_cross_sysroot_imported_targets()

# HIK Camera SDK configuration
set(HIK_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3dparty/hik_sdk")

if (EXISTS "${HIK_SDK_ROOT}")
    message(STATUS "HIK SDK found at: ${HIK_SDK_ROOT}")
else ()
    message(STATUS "HIK SDK not found at: ${HIK_SDK_ROOT}")
endif ()

# Axera AX650 SDK configuration
# SDK should be deployed to /soc/ with env vars set (see docs/ax650.md)
# CMAKE_PREFIX_PATH, CMAKE_INCLUDE_PATH, CMAKE_LIBRARY_PATH will be searched automatically

# Reset cached paths so re-configure follows the forced /soc/soc search roots.
unset(AXERA_INCLUDE_DIRS CACHE)
unset(AXERA_SYS_LIBRARY CACHE)
unset(AXERA_ENGINE_LIBRARY CACHE)
unset(AXERA_IVPS_LIBRARY CACHE)
unset(AXERA_IVE_LIBRARY CACHE)
unset(AXERA_VENC_LIBRARY CACHE)

find_path(AXERA_INCLUDE_DIRS
    NAMES ax_sys_api.h ax_engine_api.h
    PATH_SUFFIXES "soc/include" "include"
    DOC "Axera SDK include directory"
)

find_library(AXERA_SYS_LIBRARY
    NAMES ax_sys
    DOC "Axera system library"
)

find_library(AXERA_ENGINE_LIBRARY
    NAMES ax_engine
    DOC "Axera engine library"
)

find_library(AXERA_IVPS_LIBRARY
    NAMES ax_ivps
    DOC "Axera IVPS library"
)

find_library(AXERA_IVE_LIBRARY
    NAMES ax_ive
    DOC "Axera IVE library"
)

find_library(AXERA_VENC_LIBRARY
        NAMES ax_venc
        DOC "Axera VENC library"
)

if (AXERA_INCLUDE_DIRS AND AXERA_SYS_LIBRARY AND AXERA_ENGINE_LIBRARY AND AXERA_IVPS_LIBRARY AND AXERA_IVE_LIBRARY AND AXERA_VENC_LIBRARY)
    set(AXERA_LIBRARIES "${AXERA_SYS_LIBRARY}" "${AXERA_ENGINE_LIBRARY}" "${AXERA_IVPS_LIBRARY}" "${AXERA_IVE_LIBRARY}" "${AXERA_VENC_LIBRARY}")
    set(AXERA_FOUND TRUE)
    message(STATUS "Axera SDK found:")
    message(STATUS "  Include: ${AXERA_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${AXERA_LIBRARIES}")
else ()
    set(AXERA_FOUND FALSE)
    message(STATUS "Axera SDK not found - ensure /soc/ is deployed and env vars are set")
    message(STATUS "  See docs/ax650.md for deployment instructions")
endif ()

# Foxglove platform detection
if (APPLE AND CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
    set(FOXGLOVE_PLATFORM "aarch64-apple-darwin")
elseif (UNIX AND NOT APPLE)
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(FOXGLOVE_PLATFORM "aarch64-unknown-linux-gnu")
    else ()
        set(FOXGLOVE_PLATFORM "x86_64-unknown-linux-gnu")
    endif ()
else ()
    message(FATAL_ERROR "Unsupported platform. Only macOS and Linux are supported.")
endif ()

message(STATUS "Foxglove platform: ${FOXGLOVE_PLATFORM}")

set(FOXGLOVE_VERSION "v0.23.0")

# Fetch foxglove (custom URL + ZIP_NAME)
fetch_dependency(NAME foxglove
                 ZIP_URL "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2F${FOXGLOVE_VERSION}/foxglove-${FOXGLOVE_VERSION}-cpp-${FOXGLOVE_PLATFORM}.zip"
                 ZIP_NAME "foxglove-${FOXGLOVE_VERSION}-cpp-${FOXGLOVE_PLATFORM}.zip")

# Get foxglove source directory (FetchContent variable may not be set yet)
FetchContent_GetProperties(foxglove SOURCE_DIR foxglove_SOURCE_DIR)

# Foxglove SDK sources
#
# Avoid `file(GLOB ... CONFIGURE_DEPENDS ...)`: on some setups it forces Ninja
# to run the glob-check + CMake regeneration on *every* incremental build.
#
# This list is stable for the pinned SDK version above; update it only when bumping Foxglove.
set(FOXGLOVE_SOURCES
    "${foxglove_SOURCE_DIR}/src/channel.cpp"
    "${foxglove_SOURCE_DIR}/src/connection_graph.cpp"
    "${foxglove_SOURCE_DIR}/src/context.cpp"
    "${foxglove_SOURCE_DIR}/src/error.cpp"
    "${foxglove_SOURCE_DIR}/src/fetch_asset.cpp"
    "${foxglove_SOURCE_DIR}/src/foxglove.cpp"
    "${foxglove_SOURCE_DIR}/src/mcap.cpp"
    "${foxglove_SOURCE_DIR}/src/messages.cpp"
    "${foxglove_SOURCE_DIR}/src/parameter.cpp"
    "${foxglove_SOURCE_DIR}/src/service.cpp"
    "${foxglove_SOURCE_DIR}/src/websocket.cpp"
    "${foxglove_SOURCE_DIR}/src/system_info.cpp"
)

function(silence_target_warnings tgt)
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(${tgt} PRIVATE -Wno-everything)
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(${tgt} PRIVATE -w)
    elseif (MSVC)
        target_compile_options(${tgt} PRIVATE /W0)
    endif()
endfunction()
