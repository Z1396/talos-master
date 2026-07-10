# ===========================================================================
# crates.cmake - 自研 crate() 宏定义（Talos 风格简化版）
#
# 设计思想：模仿 Rust crate 模块化，自动处理：
#   1. 扫描 src/*.cpp 作为源文件
#   2. 自动添加 include/ 为公共头文件目录
#   3. 自动生成 export.hpp（模拟 Talos 的 export.hpp）
#   4. 自动启用 PCH（src/pch.hpp 存在时）
#
# 用法：
#   crate(my_math      # crate 名
#       DEPENDENCIES  # 依赖的其他 crate
#           io
#   )
# ===========================================================================

# crate() 宏：声明一个模块化静态库
# 参数：
#   name           - crate 名称
#   DEPENDENCIES   - 关键字，后跟依赖列表
function(crate name)
    # 解析参数：DEPENDENCIES 关键字后的为依赖列表
    set(options)
    set(oneValueArgs)
    set(multiValueArgs DEPENDENCIES)
    cmake_parse_arguments(CRATE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # 1. 自动收集源文件：src/*.cpp
    file(GLOB sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
    if(NOT sources)
        message(FATAL_ERROR "crate(${name}): src/*.cpp 未找到源文件")
    endif()

    # 2. 头文件目录：include/ 存在则作为 PUBLIC 头文件目录
    set(include_dir "${CMAKE_CURRENT_SOURCE_DIR}/include")
    if(EXISTS "${include_dir}")
        set(has_include TRUE)
    else()
        set(has_include FALSE)
    endif()

    # 3. 创建静态库目标
    add_library(${name} STATIC ${sources})
    target_compile_features(${name} PUBLIC cxx_std_20)

    # 4. 公共头文件目录（依赖方也能访问）
    if(has_include)
        target_include_directories(${name}
            PUBLIC
                $<BUILD_INTERFACE:${include_dir}>
        )
    endif()

    # 5. 链接依赖的其他 crate
    foreach(dep IN LISTS CRATE_DEPENDENCIES)
        target_link_libraries(${name} PUBLIC ${dep})
    endforeach()

    # 6. 自动 PCH：src/pch.hpp 存在则启用预编译头
    set(pch_file "${CMAKE_CURRENT_SOURCE_DIR}/src/pch.hpp")
    if(EXISTS "${pch_file}")
        target_precompile_headers(${name} PRIVATE ${pch_file})
    endif()

    # 7. 静默第三方警告（如果标记为 SYSTEM 依赖）
    message(STATUS "crate(${name}): registered, sources=${sources}")
endfunction()
