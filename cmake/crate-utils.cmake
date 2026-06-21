# =============================================================================
# 模块说明：Crate 工具函数，模仿Rust crate的工程结构管理C++项目
# 设计思想：统一目录规范，用单一 crate() 宏替代零散 add_library/add_executable
# 固定目录约定：
#   当前CMakeLists同级 src/      存放库/程序源码
#   当前CMakeLists同级 tests/    存放单元测试文件（*_test.cpp）
# =============================================================================
# 使用文档
# Usage:
#   crate(
#       NAME <name>                     # 【必填】模块名称，目标名
#       TYPE <SHARED|STATIC|INTERFACE|EXECUTABLE>  # 【必填】目标类型
#       [DEPS <dep1> <dep2> ...]        # 依赖库列表
#       [SOURCES xxx.cpp ...]           # 手动指定源码，不填则自动扫描src下所有c/cpp/cu
#       [PRIVATE_HEADERS ...]           # 预留参数，当前脚本未使用
#       [DEFINES <def1> <def2> ...]     # 编译宏定义（PRIVATE）
#       [OPTIONS <opt1> <opt2> ...]     # 编译参数（PRIVATE）
#       [LINK_OPTIONS <opt1> <opt2> ...]# 链接参数（PRIVATE）
#       [FEATURES cxx_std_17 ...]       # C++标准/语言特性 PUBLIC
#       [TESTS test1.cpp test2.cpp ...] # 手动指定测试文件；不填自动扫描tests/*_test.cpp
#   )
#
# 测试自动发现规则：
#   1. 不写TESTS参数：自动查找当前目录 tests/ 下所有 *_test.cpp
#   2. 每个测试自动链接：GTest::gtest_main + Threads::Threads + 当前crate库
#   3. 全局开关 TALOS_BUILD_TESTING 控制是否编译单元测试
#
# 内置自动化特性：
# 1. 自动扫描 src/ 下 c/cpp/cu 源码
# 2. SHARED/STATIC库自动生成 export.hpp 导出符号宏
# 3. 存在 src/pch.hpp 自动启用预编译头PCH
# 4. 自动管理头文件目录（BUILD/INSTALL 双路径兼容安装）
# =============================================================================

# 主入口函数：crate()，对外暴露给业务CMakeLists调用
function(crate)
    # cmake_parse_arguments：解析本函数传入的命名参数
    # 格式：cmake_parse_arguments(前缀 无值开关列表 单值参数列表 多值参数列表 原始参数${ARGN})
    cmake_parse_arguments(
        ARG                                 # 解析后变量前缀，所有参数都会变成 ARG_xxx
        ""                                  # 无布尔开关（如 OPTIONAL 这种不带值参数）
        "NAME;TYPE"                         # 只能传单个值的参数：NAME、TYPE
        "DEPS;SOURCES;PRIVATE_HEADERS;DEFINES;OPTIONS;LINK_OPTIONS;FEATURES;TESTS" # 多值列表参数
        ${ARGN}                             # 接收调用时传入的所有参数
    )

    # ====================== 1. 校验必填参数 ======================
    # 必须指定 NAME
    if(NOT ARG_NAME)
        message(FATAL_ERROR "crate(): NAME is required")
    endif()
    # 必须指定 TYPE，限定四种合法类型
    if(NOT ARG_TYPE)
        message(FATAL_ERROR "crate(): TYPE is required (SHARED|STATIC|INTERFACE|EXECUTABLE)")
    endif()

    # ====================== 2. 预留参数：头文件安装前缀（当前未实际使用） ======================
    # 如果外部没传 INCLUDE_PREFIX，默认使用模块名 NAME
    if(NOT ARG_INCLUDE_PREFIX)
        set(ARG_INCLUDE_PREFIX "${ARG_NAME}")
    endif()

    # ====================== 3. 固定源码目录：强制 src/ 规范 ======================
    # CMAKE_CURRENT_SOURCE_DIR：当前执行crate()函数的CMakeLists所在目录
    set(CRATE_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

    # ====================== 4. 收集源码文件（二选一：手动指定 / 自动全局扫描） ======================
    if(ARG_SOURCES)
        # 用户手动传入SOURCES参数，直接使用用户提供的源码列表，不再自动扫描
        set(CRATE_SOURCES ${ARG_SOURCES})
    else()
        # 未手动指定源码，递归扫描 src/ 下所有支持文件
        file(GLOB_RECURSE CRATE_CPP_SOURCES "${CRATE_SRC_DIR}/*.cpp")  # C++文件
        file(GLOB_RECURSE CRATE_C_SOURCES   "${CRATE_SRC_DIR}/*.c")    # C文件
        file(GLOB_RECURSE CRATE_CU_SOURCES  "${CRATE_SRC_DIR}/*.cu")   # CUDA GPU文件
        # 合并所有源码到统一列表
        set(CRATE_SOURCES ${CRATE_CPP_SOURCES} ${CRATE_C_SOURCES} ${CRATE_CU_SOURCES})
    endif()

    # ====================== 5. 根据TYPE创建目标：INTERFACE库 / 可执行程序 / 普通静态/动态库 ======================
    if(ARG_TYPE STREQUAL "INTERFACE")
        # 场景：纯头文件库，无编译产物，仅提供头文件、依赖传递
        add_library(${ARG_NAME} INTERFACE)
        # 设置头文件目录
        target_include_directories(${ARG_NAME} INTERFACE
            # $<BUILD_INTERFACE>：编译阶段使用本地src目录
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
            # $<INSTALL_INTERFACE>：执行make install安装后，对外头文件路径
            $<INSTALL_INTERFACE:include>
        )
        # 如果存在依赖，INTERFACE传递给所有链接本库的上层目标
        if(ARG_DEPS)
            target_link_libraries(${ARG_NAME} INTERFACE ${ARG_DEPS})
        endif()

    elseif(ARG_TYPE STREQUAL "EXECUTABLE")
        # 场景：生成可执行程序
        # 可执行文件必须有源码，无源码直接报错
        if(NOT CRATE_SOURCES)
            message(FATAL_ERROR "crate(${ARG_NAME}): No source files found in src/")
        endif()
        add_executable(${ARG_NAME} ${CRATE_SOURCES})
        # 头文件目录 PUBLIC：自身使用，同时依赖本程序的测试代码也能看到头文件
        target_include_directories(${ARG_NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
            $<INSTALL_INTERFACE:include>
        )
        # 可执行程序依赖仅PRIVATE，不会向外传递
        if(ARG_DEPS)
            target_link_libraries(${ARG_NAME} PRIVATE ${ARG_DEPS})
        endif()

    else()
        # 场景：SHARED动态库 / STATIC静态库
        add_library(${ARG_NAME} ${ARG_TYPE} ${CRATE_SOURCES})
        # PUBLIC头文件：外部链接该库的代码自动包含src目录
        target_include_directories(${ARG_NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
            $<INSTALL_INTERFACE:include>
        )
        # PUBLIC依赖：外部链接本库时自动连带链接DEPS依赖
        if(ARG_DEPS)
            target_link_libraries(${ARG_NAME} PUBLIC ${ARG_DEPS})
        endif()
    endif()

    # ====================== 6. 非INTERFACE类型统一附加编译/链接配置 ======================
    # INTERFACE无编译过程，跳过以下配置
    if(NOT ARG_TYPE STREQUAL "INTERFACE")
        # 自定义编译宏定义（仅当前目标内部可见 PRIVATE）
        if(ARG_DEFINES)
            target_compile_definitions(${ARG_NAME} PRIVATE ${ARG_DEFINES})
        endif()

        # 自定义编译选项（-Wall / /W4 等）
        if(ARG_OPTIONS)
            target_compile_options(${ARG_NAME} PRIVATE ${ARG_OPTIONS})
        endif()

        # 自定义链接选项
        if(ARG_LINK_OPTIONS)
            target_link_options(${ARG_NAME} PRIVATE ${ARG_LINK_OPTIONS})
        endif()

        # C++语言特性（cxx_std_17/cxx_std_20 PUBLIC对外暴露）
        if(ARG_FEATURES)
            target_compile_features(${ARG_NAME} PUBLIC ${ARG_FEATURES})
        endif()
    endif()

    # ====================== 7. 动态/静态库：自动生成导出符号头文件 export.hpp ======================
    # 仅 SHARED/STATIC 库执行，可执行程序、头文件库跳过
    if(NOT ARG_TYPE STREQUAL "INTERFACE" AND NOT ARG_TYPE STREQUAL "EXECUTABLE")
        # 模块名转大写，用于宏名
        string(TOUPPER "${ARG_NAME}" CRATE_NAME_UPPER)
        # 若模块名带::命名空间，替换为下划线，避免宏非法字符
        string(REPLACE "::" "_" CRATE_NAME_UPPER "${CRATE_NAME_UPPER}")
        # 编译时定义 xxx_EXPORTS，用于区分编译库本身 / 使用库的外部程序
        target_compile_definitions(${ARG_NAME} PRIVATE "${CRATE_NAME_UPPER}_EXPORTS")

        # 自动生成 src/export.hpp 导出宏文件（不存在才生成，不覆盖用户已有文件）
        set(EXPORT_HPP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/export.hpp")
        if(NOT EXISTS "${EXPORT_HPP_FILE}")
            # 写入跨平台导出宏：Linux/macOS用visibility default，Windows需扩展此处
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

    # ====================== 8. 自动启用预编译头PCH ======================
    # 非INTERFACE库，若src/pch.hpp存在，自动绑定预编译头加速编译
    if(NOT ARG_TYPE STREQUAL "INTERFACE")
        set(PCH_FILE "${CRATE_SRC_DIR}/pch.hpp")
        if(EXISTS "${PCH_FILE}")
            target_precompile_headers(${ARG_NAME} PRIVATE "${PCH_FILE}")
        endif()
    endif()

    # ====================== 9. 自动创建单元测试（调用内部私有函数） ======================
    # 参数：模块名、源码目录、用户手动传入的TESTS列表
    _crate_add_tests(${ARG_NAME} "${CRATE_SRC_DIR}" "${ARG_TESTS}")
endfunction()

# =============================================================================
# 私有辅助函数：_crate_add_tests
# 功能：统一管理测试创建逻辑，内部使用，不对外暴露
# 参数：
#   crate_name：当前crate模块名
#   src_dir：源码目录（本脚本未用到）
#   test_list：用户手动传入的TESTS文件列表
# =============================================================================
function(_crate_add_tests crate_name src_dir test_list)
    # 全局总开关：TALOS_BUILD_TESTING=OFF 直接跳过所有测试编译
    if(NOT TALOS_BUILD_TESTING)
        return()
    endif()

    # 分支1：用户手动传入TESTS参数，使用指定测试文件
    if(test_list)
        foreach(test_file ${test_list})
            # 取文件名（去掉后缀）作为测试目标名
            get_filename_component(test_name ${test_file} NAME_WE)
            # 创建单个测试可执行文件
            _crate_create_test(${test_name} ${test_file} ${crate_name})
        endforeach()
        return()
    endif()

    # 分支2：未手动指定TESTS，自动扫描 tests/ 目录下 *_test.cpp
    set(tests_dir "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    if(EXISTS "${tests_dir}")
        # 匹配所有以 _test.cpp 结尾的测试文件
        file(GLOB test_files "${tests_dir}/*_test.cpp")
        foreach(test_file ${test_files})
            get_filename_component(test_name ${test_file} NAME_WE)
            _crate_create_test(${test_name} ${test_file} ${crate_name})
        endforeach()
    endif()
endfunction()

# =============================================================================
# 私有辅助函数：_crate_create_test
# 功能：创建单个GTest测试程序，并注册到CTest
# 参数：test_name 测试目标名；test_file 测试源码；crate_name 被测库名
# =============================================================================
function(_crate_create_test test_name test_file crate_name)
    # 创建测试可执行文件
    add_executable(${test_name} ${test_file})
    # 固定依赖：GTest主入口 + 线程库 + 当前业务库
    target_link_libraries(${test_name} PRIVATE
        GTest::gtest_main
        Threads::Threads
        ${crate_name}
    )
    # 将测试注册到CTest，执行 ctest 时自动运行
    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

# =============================================================================
# 对外辅助函数：crate_add_pch
# 手动给指定目标绑定预编译头，可选自定义pch文件路径
# 使用示例：crate_add_pch(my_lib PCH_FILE "common/pch.h")
# =============================================================================
function(crate_add_pch TARGET)
    # 解析传入参数
    cmake_parse_arguments(ARG "" "PCH_FILE" "" ${ARGN})
    # 未指定PCH_FILE则默认 src/pch.hpp
    if(NOT ARG_PCH_FILE)
        set(ARG_PCH_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/pch.hpp")
    endif()

    # 文件存在才启用预编译头
    if(EXISTS "${ARG_PCH_FILE}")
        target_precompile_headers(${TARGET} PRIVATE "${ARG_PCH_FILE}")
    else()
        message(DEBUG "PCH file not found: ${ARG_PCH_FILE}")
    endif()
endfunction()

# =============================================================================
# 对外辅助函数：crate_silence_warnings
# 一键关闭目标大量冗余警告，区分MSVC/ GCC/Clang编译器
# 使用示例：crate_silence_warnings(my_lib)
# =============================================================================
function(crate_silence_warnings TARGET)
    if(MSVC)
        # Windows MSVC：全局低警告等级，屏蔽特定冗余警告码
        target_compile_options(${TARGET} PRIVATE /W0 /wd4100 /wd4201)
    else()
        # GCC/Clang：关闭无用参数、无符号比较、未使用变量等警告
        target_compile_options(${TARGET} PRIVATE
            -Wno-unused-parameter
            -Wno-sign-compare
            -Wno-unused-variable
            -Wno-unused-but-set-variable
            -Wno-return-type-c-linkage
        )
    endif()
endfunction()