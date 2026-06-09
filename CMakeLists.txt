cmake_minimum_required(VERSION 3.14)

include(cmake/crate-utils.cmake)

project(talos VERSION 0.1 LANGUAGES C CXX)

include(cmake/compiler-spec.cmake)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}" CACHE PATH "Install path prefix." FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(TALOS_INSTALL_BIN_DIR "${PROJECT_SOURCE_DIR}/bin")

get_filename_component(PROJECT_SOURCE_DIR_ABS "${CMAKE_SOURCE_DIR}" REALPATH)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-fmacro-prefix-map=${PROJECT_SOURCE_DIR_ABS}/=)
endif ()

# ============================================================================
# AddressSanitizer Option (Memory Leak Detection)
# ============================================================================
option(ENABLE_ASAN "Enable AddressSanitizer for memory leak detection" OFF)

if(ENABLE_ASAN)
    message(STATUS "AddressSanitizer: ENABLED")
    add_compile_options(
        -fsanitize=address
        -fno-omit-frame-pointer
        -fno-optimize-sibling-calls
        -g3
    )
    add_link_options(-fsanitize=address -g3)
    message(STATUS "  Compile flags: -fsanitize=address -fno-omit-frame-pointer -fno-optimize-sibling-calls -g3")
    message(STATUS "  Link flags:    -fsanitize=address -g3")
else()
    message(STATUS "AddressSanitizer: DISABLED")
endif()

# ============================================================================
# ThreadSanitizer Option (Data Race Detection)
# ============================================================================
option(ENABLE_TSAN "Enable ThreadSanitizer for data race detection" OFF)

if(ENABLE_TSAN)
    message(STATUS "ThreadSanitizer: ENABLED")
    add_compile_options(
        -fsanitize=thread
        -fno-omit-frame-pointer
        -g3
    )
    add_link_options(-fsanitize=thread -g3)
    message(STATUS "  Compile flags: -fsanitize=thread -fno-omit-frame-pointer -g3")
    message(STATUS "  Link flags:    -fsanitize=thread -g3")
else()
    message(STATUS "ThreadSanitizer: DISABLED")
endif()

# Add project cmake directory to module search path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# RPATH configuration for micromamba environment
if(APPLE)
    set(TALOS_INSTALL_RPATH "@loader_path")
else()
    set(TALOS_INSTALL_RPATH "$ORIGIN")
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
set(CMAKE_INSTALL_RPATH "${TALOS_INSTALL_RPATH}")

if(CMAKE_CXX_COMPILER MATCHES "micromamba")
    get_filename_component(MICROMAMBA_LIB_DIR ${CMAKE_CXX_COMPILER} DIRECTORY)
    get_filename_component(MICROMAMBA_LIB_DIR ${MICROMAMBA_LIB_DIR} DIRECTORY)
    set(MICROMAMBA_LIB_DIR "${MICROMAMBA_LIB_DIR}/lib")
    set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR};${MICROMAMBA_LIB_DIR}")
    list(APPEND CMAKE_INSTALL_RPATH "${MICROMAMBA_LIB_DIR}")
endif()

include(cmake/dependencies.cmake)
include(cmake/test.cmake)
include(cmake/calibration.cmake)

foreach(_runtime_target IN ITEMS fmt spdlog)
    if(TARGET "${_runtime_target}")
        set_target_properties(${_runtime_target} PROPERTIES
            BUILD_WITH_INSTALL_RPATH TRUE
            INSTALL_RPATH "${TALOS_INSTALL_RPATH}"
        )
    endif()
endforeach()

# ============================================================================
# Third-party libraries (header-only / vendored)
# ============================================================================

# tomlplusplus implementation
add_library(toml_impl STATIC
        ${tomlplusplus_SOURCE_DIR}/src/toml.cpp
)

crate_silence_warnings(toml_impl)

target_include_directories(toml_impl SYSTEM PUBLIC
        ${tomlplusplus_SOURCE_DIR}/include
)

target_compile_definitions(toml_impl PUBLIC
        TOML_HEADER_ONLY=0
        TOML_EXCEPTIONS=0
)

# Magic enum
add_library(magic_enum INTERFACE)
target_include_directories(magic_enum INTERFACE 3dparty/magic_enum)
target_compile_definitions(magic_enum INTERFACE
        MAGIC_ENUM_RANGE_MIN=0
        MAGIC_ENUM_RANGE_MAX=16
)

# Lie++ (SO3 manifold library)
add_library(lieplusplus INTERFACE)
target_include_directories(lieplusplus INTERFACE 3dparty/lieplusplus)

# Foxglove C++ wrapper library
add_library(foxglove_cpp SHARED ${FOXGLOVE_SOURCES})
crate_silence_warnings(foxglove_cpp)

target_include_directories(foxglove_cpp SYSTEM PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include_foxglove
        ${foxglove_SOURCE_DIR}/include
        ${foxglove_SOURCE_DIR}/include/foxglove
)

target_link_libraries(foxglove_cpp PUBLIC
        ${foxglove_SOURCE_DIR}/lib/libfoxglove.a
        "$<$<PLATFORM_ID:Darwin>:-framework IOKit>"
        "$<$<PLATFORM_ID:Darwin>:-framework Security>"
        "$<$<PLATFORM_ID:Darwin>:-framework CoreFoundation>"
        "$<$<PLATFORM_ID:Darwin>:-framework SystemConfiguration>"
        "$<$<PLATFORM_ID:Darwin>:-mmacosx-version-min=15.5>"
)

# ============================================================================
# Crates (Rust-style workspace)
# ============================================================================

# Core utilities
add_subdirectory(crates/toml)
add_subdirectory(crates/log)
add_subdirectory(crates/math)

# Math and geometry
add_subdirectory(crates/fast_tf)

# Scheduler framework
add_subdirectory(crates/primitive)
add_subdirectory(crates/scheduler)

# Ultra-low bitrate image transmission
add_subdirectory(crates/quanta)

# Chiral bidirectional communication
add_subdirectory(crates/chiral)

# Hardware drivers
add_subdirectory(crates/hardware/at_gimbal)
add_subdirectory(crates/hardware/hik_camera_driver)
add_subdirectory(crates/hardware_daedalus)

# ============================================================================
# FCS library (Fire Control System)
# ============================================================================

if(DEFINED ENV{TZ})
    set(TALOS_PREVIOUS_TZ "$ENV{TZ}")
    set(TALOS_HAD_PREVIOUS_TZ TRUE)
else()
    set(TALOS_HAD_PREVIOUS_TZ FALSE)
endif()

set(ENV{TZ} "UTC-8")
string(TIMESTAMP TALOS_BUILD_DATE "%Y-%m-%dT%H:%M:%S+0800")
if(TALOS_HAD_PREVIOUS_TZ)
    set(ENV{TZ} "${TALOS_PREVIOUS_TZ}")
else()
    unset(ENV{TZ})
endif()
find_package(Git QUIET)

set(TALOS_GIT_HASH "unknown")
set(TALOS_GIT_BRANCH "unknown")
set(TALOS_GIT_DIRTY_SUFFIX "")
set(TALOS_BUILD_HOST "unknown")

execute_process(
        COMMAND hostname
        OUTPUT_VARIABLE TALOS_BUILD_HOST_OUTPUT
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT TALOS_BUILD_HOST_OUTPUT STREQUAL "")
    set(TALOS_BUILD_HOST "${TALOS_BUILD_HOST_OUTPUT}")
endif()

if(Git_FOUND)
    execute_process(
            COMMAND "${GIT_EXECUTABLE}" -C "${PROJECT_SOURCE_DIR_ABS}" rev-parse --short=6 HEAD
            OUTPUT_VARIABLE TALOS_GIT_HASH_OUTPUT
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT TALOS_GIT_HASH_OUTPUT STREQUAL "")
        set(TALOS_GIT_HASH "${TALOS_GIT_HASH_OUTPUT}")
    endif()

    execute_process(
            COMMAND "${GIT_EXECUTABLE}" -C "${PROJECT_SOURCE_DIR_ABS}" rev-parse --abbrev-ref HEAD
            OUTPUT_VARIABLE TALOS_GIT_BRANCH_OUTPUT
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(TALOS_GIT_BRANCH_OUTPUT STREQUAL "" OR TALOS_GIT_BRANCH_OUTPUT STREQUAL "HEAD")
        execute_process(
                COMMAND "${GIT_EXECUTABLE}" -C "${PROJECT_SOURCE_DIR_ABS}" rev-parse --short=6 HEAD
                OUTPUT_VARIABLE TALOS_GIT_BRANCH_OUTPUT
                ERROR_QUIET
                OUTPUT_STRIP_TRAILING_WHITESPACE
        )
    endif()
    if(NOT TALOS_GIT_BRANCH_OUTPUT STREQUAL "")
        set(TALOS_GIT_BRANCH "${TALOS_GIT_BRANCH_OUTPUT}")
    endif()

    execute_process(
            COMMAND "${GIT_EXECUTABLE}" -C "${PROJECT_SOURCE_DIR_ABS}" status --porcelain --untracked-files=all
            OUTPUT_VARIABLE TALOS_GIT_STATUS_OUTPUT
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT TALOS_GIT_STATUS_OUTPUT STREQUAL "")
        set(TALOS_GIT_DIRTY_SUFFIX "+dirty")
    endif()
endif()

set(TALOS_GIT_COMMIT "${TALOS_GIT_HASH}${TALOS_GIT_DIRTY_SUFFIX}")

add_library(talos_build_info STATIC
        src/fcs/runtime/build_info.cpp
)

target_include_directories(talos_build_info PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs
)

target_compile_definitions(talos_build_info PRIVATE
        TALOS_BUILD_INFO_DATE="${TALOS_BUILD_DATE}"
        TALOS_BUILD_INFO_HOST="${TALOS_BUILD_HOST}"
        TALOS_BUILD_INFO_GIT_COMMIT="${TALOS_GIT_COMMIT}"
        TALOS_BUILD_INFO_GIT_BRANCH="${TALOS_GIT_BRANCH}"
)

set(FCS_SOURCES
        src/fcs/chiral/chiral_collector_system.cpp
        src/fcs/core/trajectory/solver/trajectory_solver.cpp
        src/fcs/core/types.cpp
        src/fcs/L1_sensor/camera_interface.cpp
        src/fcs/L1_sensor/output_interface.cpp
        src/fcs/runtime/boot.cpp
        src/fcs/runtime/capturer.cpp
        src/fcs/runtime/config_loader.cpp
        src/fcs/runtime/l1_l2_setup.cpp
        src/fcs/runtime/stream_encode.cpp
        src/fcs/runtime/stream_send.cpp
        src/fcs/L2_perception/armor/backend.cpp
        src/fcs/L2_perception/armor/systems.cpp
        src/fcs/L2_perception/ldm/ldm_detector.cpp
        src/fcs/L2_perception/ldm/ldm_systems.cpp
        src/fcs/L2_perception/rune/rune_detector.cpp
        src/fcs/L2_perception/rune/rune_systems.cpp
        src/fcs/L2_perception/armor/backends/traditional.cpp
        src/fcs/L2_perception/armor/backends/ort.cpp
        src/fcs/L2_perception/armor/backends/traditional_classifier.cpp
        src/fcs/L3_estimation/tracker_systems.cpp
        src/fcs/L3_estimation/extended_kalman_filter.cpp
        src/fcs/L3_estimation/energy_meter_solver/energy_meter_tracker.cpp
        src/fcs/L3_estimation/energy_meter_solver/voter.cpp
        src/fcs/L3_estimation/energy_meter/energy_meter_systems.cpp
        src/fcs/L3_estimation/ldm_naive/ldm_naive_systems.cpp
        src/fcs/L4_planning/aimer/aimer.cpp
        src/fcs/L4_planning/aimer/armor_target_decider.cpp
        src/fcs/L4_planning/aimer/aimer_systems.cpp
        src/fcs/L4_planning/planning_systems.cpp
        src/fcs/L4_planning/trajectory_builder.cpp
        src/fcs/L5_weapon/enhanced/trajectory_optimizer.cpp
        src/fcs/L5_weapon/enhanced/weapon_systems.cpp
)

if(CMAKE_CROSSCOMPILING)
    list(APPEND FCS_SOURCES cmake/toolchains/isoc23_shim.cpp)
endif()

if(TensorRT_FOUND)
    list(APPEND FCS_SOURCES src/fcs/L2_perception/armor/backends/tensor_rt.cpp)
    message(STATUS "TensorRT backend enabled")
else()
    message(STATUS "TensorRT backend disabled (TensorRT not found)")
endif()

if(AXERA_FOUND)
    list(APPEND FCS_SOURCES src/fcs/L2_perception/armor/backends/axera.cpp)
    message(STATUS "Axera backend enabled")
else()
    message(STATUS "Axera backend disabled (AXERA not found)")
endif()

add_library(fcs SHARED ${FCS_SOURCES})

target_include_directories(fcs PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs
        ${CMAKE_CURRENT_SOURCE_DIR}/crates/quanta/src
        $<$<TARGET_EXISTS:TensorRT::TensorRT>:${TensorRT_INCLUDE_DIRS}>
)

if(CUDA_FOUND)
    # Modern CMake: use imported targets instead of variables
    target_link_libraries(fcs PUBLIC ${CUDA_LIBRARIES})
    target_include_directories(fcs SYSTEM PUBLIC ${CUDA_INCLUDE_DIRS})
    target_compile_definitions(fcs PUBLIC TALOS_HAS_CUDA_RUNTIME=0)
    message(STATUS "Linking with CUDA runtime: ${CUDA_LIBRARIES}")
else()
    target_compile_definitions(fcs PUBLIC TALOS_HAS_CUDA_RUNTIME=0)
endif()

if(TensorRT_FOUND)
    target_compile_definitions(fcs PUBLIC TALOS_HAS_TENSORRT=1)
    message(STATUS "TensorRT support enabled")
else()
    target_compile_definitions(fcs PUBLIC TALOS_HAS_TENSORRT=0)
endif()

if(AXERA_FOUND)
    target_compile_definitions(fcs PUBLIC TALOS_HAS_AXERA=1)
    target_include_directories(fcs SYSTEM PUBLIC ${AXERA_INCLUDE_DIRS})
    target_link_libraries(fcs PUBLIC ${AXERA_LIBRARIES})
    message(STATUS "Axera support enabled")
else()
    target_compile_definitions(fcs PUBLIC TALOS_HAS_AXERA=0)
    message(STATUS "Axera support disabled")
endif()

target_compile_options(fcs PRIVATE -Wno-error=array-bounds)

target_link_libraries(fcs PUBLIC
        toml_std
        fast_tf
        scheduler
        quanta
        Eigen3::Eigen
        Ceres::ceres
        talos_gimbal
        hikcamera
        ipc
        chiral
        ${OpenCV_LIBS}
        onnxruntime::onnxruntime
        $<$<TARGET_EXISTS:TensorRT::TensorRT>:TensorRT::TensorRT>
        $<$<TARGET_EXISTS:TensorRT::nvonnxparser>:TensorRT::nvonnxparser>
PRIVATE
        lieplusplus
)

target_precompile_headers(fcs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs/pch.hpp)

# FCS visualization
add_library(fcs_visualization SHARED
        src/fcs_visualization/foxglove_server.cpp
        src/fcs_visualization/foxglove_systems.cpp
        src/fcs_visualization/scene_builder.cpp
        src/fcs_visualization/systems/debug_systems.cpp
        src/fcs_visualization/systems/l1_sensor_systems.cpp
        src/fcs_visualization/systems/l2_perception_systems.cpp
        src/fcs_visualization/systems/l3_estimation_systems.cpp
        src/fcs_visualization/systems/l4_planning_systems.cpp
        src/fcs_visualization/systems/rune_systems.cpp
        src/fcs_visualization/systems/ground_truth_systems.cpp
)

target_include_directories(fcs_visualization PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs
)

target_include_directories(fcs_visualization SYSTEM PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs_visualization
)

target_link_libraries(fcs_visualization PUBLIC
        fcs
        scheduler
        foxglove_cpp
        quanta
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
        Eigen3::Eigen
        spdlog::spdlog
)

target_precompile_headers(fcs_visualization PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs_visualization/pch.hpp)

# FCS tests (auto-discovered from src/fcs/tests/)
if(TALOS_BUILD_TESTING)
    file(GLOB FCS_TESTS "${CMAKE_CURRENT_SOURCE_DIR}/src/fcs/tests/*_test.cpp")
    foreach(test_file ${FCS_TESTS})
        get_filename_component(test_name ${test_file} NAME_WE)
        add_executable(${test_name} ${test_file})
        target_compile_definitions(${test_name} PRIVATE
            TALOS_SOURCE_DIR="${PROJECT_SOURCE_DIR_ABS}"
        )
        target_link_libraries(${test_name} PRIVATE
            GTest::gtest_main
            Threads::Threads
            fcs
            fast_tf
            nlohmann_json::nlohmann_json
        )
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
endif()

# ============================================================================
# Main executable
# ============================================================================

add_executable(${PROJECT_NAME}
        src/main.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        talos_build_info
        fcs
        fcs_visualization
        fast_tf
        talos_log
        toml_std
        spdlog::spdlog
        fmt::fmt
        "$<$<PLATFORM_ID:Darwin>:-mmacosx-version-min=15.5>"
)

add_executable(talos-playground src/playground.cpp)

target_include_directories(talos-playground PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/crates/hardware_daedalus/src
)

target_link_libraries(talos-playground PRIVATE
        fast_tf
        Eigen3::Eigen
        lieplusplus
        ipc
        talos_gimbal
        talos_log
        chiral
        spdlog::spdlog
        ${OpenCV_LIBS}
)

add_executable(talos-replay
        src/talos_replay.cpp
        src/fcs/runtime/replay.cpp
)

target_link_libraries(talos-replay PRIVATE
        talos_build_info
        talos_log
        talos_math
        toml_std
        ipc
        fast_tf
        nlohmann_json::nlohmann_json
        fmt::fmt
        spdlog::spdlog
        opencv_imgcodecs
        opencv_imgproc
        "$<$<PLATFORM_ID:Darwin>:-mmacosx-version-min=15.5>"
)

# ============================================================================
# MPC Benchmark: 2x TinyMPC (double/dynamic) vs DualSmallMpcSolver (float/fixed)
# ============================================================================
add_library(tinympc_bench STATIC
        3dparty/tinympc/admm.cpp
        3dparty/tinympc/tiny_api.cpp
        3dparty/tinympc/rho_benchmark.cpp
)
target_include_directories(tinympc_bench SYSTEM PUBLIC 3dparty/tinympc)
target_link_libraries(tinympc_bench PUBLIC Eigen3::Eigen)
crate_silence_warnings(tinympc_bench)
target_compile_options(tinympc_bench PRIVATE -Wno-header-hygiene)

add_executable(bench_mpc benchmark/bench_mpc.cpp)
target_include_directories(bench_mpc PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs/L5_weapon/enhanced
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs
        ${CMAKE_CURRENT_SOURCE_DIR}/3dparty
)
target_link_libraries(bench_mpc PRIVATE
        Eigen3::Eigen
        tinympc_bench
)

# ============================================================================
# Install
# ============================================================================

function(talos_install_shared_library path)
    if(EXISTS "${path}" OR IS_SYMLINK "${path}")
        install(CODE
            "file(INSTALL DESTINATION \"${TALOS_INSTALL_BIN_DIR}\" TYPE SHARED_LIBRARY FOLLOW_SYMLINK_CHAIN FILES \"${path}\")"
        )
    endif()
endfunction()

function(talos_collect_imported_shared_libraries out_var)
    set(_collected "")

    foreach(_target IN LISTS ARGN)
        if(NOT TARGET "${_target}")
            continue()
        endif()

        set(_props
            IMPORTED_LOCATION
            IMPORTED_IMPLIB
            INTERFACE_LINK_LIBRARIES
        )

        get_target_property(_import_configs "${_target}" IMPORTED_CONFIGURATIONS)
        if(_import_configs AND NOT _import_configs STREQUAL "_import_configs-NOTFOUND")
            foreach(_config IN LISTS _import_configs)
                string(TOUPPER "${_config}" _config_upper)
                list(APPEND _props
                    "IMPORTED_LOCATION_${_config_upper}"
                    "IMPORTED_IMPLIB_${_config_upper}"
                )
            endforeach()
        endif()

        foreach(_prop IN LISTS _props)
            get_target_property(_values "${_target}" ${_prop})
            if(NOT _values OR _values STREQUAL "${_prop}-NOTFOUND")
                continue()
            endif()

            foreach(_value IN LISTS _values)
                if((EXISTS "${_value}" OR IS_SYMLINK "${_value}")
                   AND IS_ABSOLUTE "${_value}"
                   AND _value MATCHES "\\.(so(\\..*)?|dylib)$")
                    list(APPEND _collected "${_value}")
                endif()
            endforeach()
        endforeach()
    endforeach()

    list(REMOVE_DUPLICATES _collected)
    set(${out_var} "${_collected}" PARENT_SCOPE)
endfunction()

set(TALOS_INSTALL_TARGETS
        foxglove_cpp
        quanta
        scheduler
        hikcamera
        fcs
        fcs_visualization
        ${PROJECT_NAME}
        talos-playground
        talos-replay
        bench_mpc
)

set(TALOS_OPTIONAL_INSTALL_TARGETS
        rm_calibration
)

foreach(_target IN LISTS TALOS_OPTIONAL_INSTALL_TARGETS)
    if(NOT TARGET "${_target}")
        list(REMOVE_ITEM TALOS_INSTALL_TARGETS "${_target}")
    endif()
endforeach()

if(TARGET rm_calibration)
    list(APPEND TALOS_INSTALL_TARGETS rm_calibration)
endif()

install(TARGETS ${TALOS_INSTALL_TARGETS}
        RUNTIME DESTINATION "${TALOS_INSTALL_BIN_DIR}"
        LIBRARY DESTINATION "${TALOS_INSTALL_BIN_DIR}"
)

set(TALOS_HIK_RUNTIME_LIB_DIR "")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(TALOS_HIK_RUNTIME_LIB_DIR "${HIK_SDK_ROOT}/lib/amd64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
        set(TALOS_HIK_RUNTIME_LIB_DIR "${HIK_SDK_ROOT}/lib/arm64")
    endif()
endif()

if(TALOS_HIK_RUNTIME_LIB_DIR AND EXISTS "${TALOS_HIK_RUNTIME_LIB_DIR}")
    file(GLOB TALOS_HIK_RUNTIME_LIBS
        "${TALOS_HIK_RUNTIME_LIB_DIR}/*.so"
        "${TALOS_HIK_RUNTIME_LIB_DIR}/*.so.*"
        "${TALOS_HIK_RUNTIME_LIB_DIR}/*.dylib"
    )
    list(FILTER TALOS_HIK_RUNTIME_LIBS EXCLUDE REGEX "/lib/[^/]*/lib(av|sw)")
    foreach(_lib IN LISTS TALOS_HIK_RUNTIME_LIBS)
        talos_install_shared_library("${_lib}")
    endforeach()
endif()

if(TALOS_FFMPEG_ROOT AND EXISTS "${TALOS_FFMPEG_ROOT}/lib")
    file(GLOB TALOS_FFMPEG_RUNTIME_LIBS
        "${TALOS_FFMPEG_ROOT}/lib/libav*.so"
        "${TALOS_FFMPEG_ROOT}/lib/libav*.so.*"
        "${TALOS_FFMPEG_ROOT}/lib/libav*.dylib"
        "${TALOS_FFMPEG_ROOT}/lib/libav*.dylib*"
        "${TALOS_FFMPEG_ROOT}/lib/libsw*.so"
        "${TALOS_FFMPEG_ROOT}/lib/libsw*.so.*"
        "${TALOS_FFMPEG_ROOT}/lib/libsw*.dylib"
        "${TALOS_FFMPEG_ROOT}/lib/libsw*.dylib*"
    )
else()
    talos_collect_imported_shared_libraries(TALOS_FFMPEG_RUNTIME_LIBS
        PkgConfig::AVCODEC
        PkgConfig::AVFORMAT
        PkgConfig::AVFILTER
        PkgConfig::SWSCALE
        PkgConfig::SWRESAMPLE
        PkgConfig::AVUTIL
    )
endif()

foreach(_lib IN LISTS TALOS_FFMPEG_RUNTIME_LIBS)
    talos_install_shared_library("${_lib}")
endforeach()

file(GLOB TALOS_FETCHED_RUNTIME_LIBS
    "${CMAKE_BINARY_DIR}/_deps/fmt-build/libfmt*.so"
    "${CMAKE_BINARY_DIR}/_deps/fmt-build/libfmt*.so.*"
    "${CMAKE_BINARY_DIR}/_deps/fmt-build/libfmt*.dylib"
    "${CMAKE_BINARY_DIR}/_deps/fmt-build/libfmt*.dylib*"
    "${CMAKE_BINARY_DIR}/_deps/spdlog-build/libspdlog*.so"
    "${CMAKE_BINARY_DIR}/_deps/spdlog-build/libspdlog*.so.*"
    "${CMAKE_BINARY_DIR}/_deps/spdlog-build/libspdlog*.dylib"
    "${CMAKE_BINARY_DIR}/_deps/spdlog-build/libspdlog*.dylib*"
    "${CMAKE_BINARY_DIR}/_deps/onnxruntime-src/lib/*.dylib*"
    "${CMAKE_BINARY_DIR}/_deps/onnxruntime-src/lib/*.so*"
)

foreach(_lib IN LISTS TALOS_FETCHED_RUNTIME_LIBS)
    talos_install_shared_library("${_lib}")
endforeach()
