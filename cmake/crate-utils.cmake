# Crate utilities for Rust-style C++ project organization
#
# Usage:
#   crate(
#       NAME <name>
#       TYPE <SHARED|STATIC|INTERFACE|EXECUTABLE>
#       [DEPS <dep1> <dep2> ...]
#       [TESTS test1.cpp test2.cpp ...]  # 可选：手动指定测试，否则自动发现 tests/*.cpp
#       [DEFINES <def1> <def2> ...]
#       [OPTIONS <opt1> <opt2> ...]
#       [LINK_OPTIONS <opt1> <opt2> ...]
#       [FEATURES <feature1> <feature2> ...]
#   )
#
# 测试自动发现：
#   - 如果未指定 TESTS，自动查找 crate_dir/tests/*_test.cpp
#   - 测试自动链接 GTest::gtest_main + Threads::Threads + crate_name

function(crate)
    cmake_parse_arguments(
        ARG
        ""                           # 选项
        "NAME;TYPE"                  # 单值参数
        "DEPS;SOURCES;PRIVATE_HEADERS;DEFINES;OPTIONS;LINK_OPTIONS;FEATURES;TESTS"  # 多值参数
        ${ARGN}
    )

    # 验证必需参数
    if(NOT ARG_NAME)
        message(FATAL_ERROR "crate(): NAME is required")
    endif()
    if(NOT ARG_TYPE)
        message(FATAL_ERROR "crate(): TYPE is required (SHARED|STATIC|INTERFACE|EXECUTABLE)")
    endif()

    # Include prefix 默认为 crate name
    if(NOT ARG_INCLUDE_PREFIX)
        set(ARG_INCLUDE_PREFIX "${ARG_NAME}")
    endif()

    # 固定目录结构: src/
    set(CRATE_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

    # 收集源文件（如果未手动指定）
    if(ARG_SOURCES)
        # 手动指定的源文件（支持相对路径）
        set(CRATE_SOURCES ${ARG_SOURCES})
    else()
        # 自动收集所有源文件
        file(GLOB_RECURSE CRATE_CPP_SOURCES "${CRATE_SRC_DIR}/*.cpp")
        file(GLOB_RECURSE CRATE_C_SOURCES   "${CRATE_SRC_DIR}/*.c")
        file(GLOB_RECURSE CRATE_CU_SOURCES  "${CRATE_SRC_DIR}/*.cu")
        set(CRATE_SOURCES ${CRATE_CPP_SOURCES} ${CRATE_C_SOURCES} ${CRATE_CU_SOURCES})
    endif()

    # 根据类型创建库或可执行文件
    if(ARG_TYPE STREQUAL "INTERFACE")
        add_library(${ARG_NAME} INTERFACE)
        target_include_directories(${ARG_NAME} INTERFACE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
            $<INSTALL_INTERFACE:include>
        )
        if(ARG_DEPS)
            target_link_libraries(${ARG_NAME} INTERFACE ${ARG_DEPS})
        endif()
    elseif(ARG_TYPE STREQUAL "EXECUTABLE")
        if(NOT CRATE_SOURCES)
            message(FATAL_ERROR "crate(${ARG_NAME}): No source files found in src/")
        endif()
        add_executable(${ARG_NAME} ${CRATE_SOURCES})
        # include 目录（自身）
        target_include_directories(${ARG_NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
            $<INSTALL_INTERFACE:include>
        )
        # 链接依赖
        if(ARG_DEPS)
            target_link_libraries(${ARG_NAME} PRIVATE ${ARG_DEPS})
        endif()
    else()
        # SHARED/STATIC 库
        add_library(${ARG_NAME} ${ARG_TYPE} ${CRATE_SOURCES})
        # include 目录（自身）
        target_include_directories(${ARG_NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
            $<INSTALL_INTERFACE:include>
        )
        # 链接依赖
        if(ARG_DEPS)
            target_link_libraries(${ARG_NAME} PUBLIC ${ARG_DEPS})
        endif()
    endif()

    # EXECUTABLE 和 SHARED/STATIC 库的额外配置
    if(NOT ARG_TYPE STREQUAL "INTERFACE")
        # 编译定义
        if(ARG_DEFINES)
            target_compile_definitions(${ARG_NAME} PRIVATE ${ARG_DEFINES})
        endif()

        # 编译选项
        if(ARG_OPTIONS)
            target_compile_options(${ARG_NAME} PRIVATE ${ARG_OPTIONS})
        endif()

        # 链接选项
        if(ARG_LINK_OPTIONS)
            target_link_options(${ARG_NAME} PRIVATE ${ARG_LINK_OPTIONS})
        endif()

        # C++ 特性
        if(ARG_FEATURES)
            target_compile_features(${ARG_NAME} PUBLIC ${ARG_FEATURES})
        endif()
    endif()

    # 自动添加导出宏定义 (对于 SHARED/STATIC 库)
    if(NOT ARG_TYPE STREQUAL "INTERFACE" AND NOT ARG_TYPE STREQUAL "EXECUTABLE")
        string(TOUPPER "${ARG_NAME}" CRATE_NAME_UPPER)
        string(REPLACE "::" "_" CRATE_NAME_UPPER "${CRATE_NAME_UPPER}")
        target_compile_definitions(${ARG_NAME} PRIVATE "${CRATE_NAME_UPPER}_EXPORTS")

        # 自动生成 export.hpp
        set(EXPORT_HPP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/export.hpp")
        if(NOT EXISTS "${EXPORT_HPP_FILE}")
            file(WRITE "${EXPORT_HPP_FILE}"
"#pragma once

// ${ARG_NAME} library export macros
#ifdef ${CRATE_NAME_UPPER}_EXPORTS
#define ${CRATE_NAME_UPPER}_API __attribute__((visibility(\"default\")))
#else
#define ${CRATE_NAME_UPPER}_API
#endif
")
            message(STATUS "Generated ${EXPORT_HPP_FILE}")
        endif()
    endif()

    # 自动检测并添加 PCH
    if(NOT ARG_TYPE STREQUAL "INTERFACE")
        set(PCH_FILE "${CRATE_SRC_DIR}/pch.hpp")
        if(EXISTS "${PCH_FILE}")
            target_precompile_headers(${ARG_NAME} PRIVATE "${PCH_FILE}")
        endif()
    endif()

    # 处理测试（自动发现 tests/ 目录或使用 TESTS 参数）
    _crate_add_tests(${ARG_NAME} "${CRATE_SRC_DIR}" "${ARG_TESTS}")
endfunction()

# =============================================================================
# 私有辅助函数: 处理测试
# =============================================================================
function(_crate_add_tests crate_name src_dir test_list)
    # 跳过测试构建（默认 OFF）
    if(NOT TALOS_BUILD_TESTING)
        return()
    endif()

    # 如果提供了 TESTS 参数，使用它
    if(test_list)
        foreach(test_file ${test_list})
            get_filename_component(test_name ${test_file} NAME_WE)
            _crate_create_test(${test_name} ${test_file} ${crate_name})
        endforeach()
        return()
    endif()

    # 否则自动发现 tests/ 目录
    set(tests_dir "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    if(EXISTS "${tests_dir}")
        file(GLOB test_files "${tests_dir}/*_test.cpp")
        foreach(test_file ${test_files})
            get_filename_component(test_name ${test_file} NAME_WE)
            _crate_create_test(${test_name} ${test_file} ${crate_name})
        endforeach()
    endif()
endfunction()

function(_crate_create_test test_name test_file crate_name)
    add_executable(${test_name} ${test_file})
    target_link_libraries(${test_name} PRIVATE
        GTest::gtest_main
        Threads::Threads
        ${crate_name}
    )
    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

# 辅助函数: 添加 PCH（预编译头）
function(crate_add_pch TARGET)
    cmake_parse_arguments(ARG "" "PCH_FILE" "" ${ARGN})
    if(NOT ARG_PCH_FILE)
        set(ARG_PCH_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/pch.hpp")
    endif()

    if(EXISTS "${ARG_PCH_FILE}")
        target_precompile_headers(${TARGET} PRIVATE "${ARG_PCH_FILE}")
    else()
        message(DEBUG "PCH file not found: ${ARG_PCH_FILE}")
    endif()
endfunction()

# 辅助函数: 抑制警告
function(crate_silence_warnings TARGET)
    if(MSVC)
        target_compile_options(${TARGET} PRIVATE /W0 /wd4100 /wd4201)
    else()
        target_compile_options(${TARGET} PRIVATE
            -Wno-unused-parameter
            -Wno-sign-compare
            -Wno-unused-variable
            -Wno-unused-but-set-variable
            -Wno-return-type-c-linkage
        )
    endif()
endfunction()
