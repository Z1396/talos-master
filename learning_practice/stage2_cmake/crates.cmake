# ===========================================================================
# crates.cmake - 自研 crate() 宏定义（Talos 风格简化版）
#
# 设计思想：模仿 Rust crate 模块化，自动处理：
#   1. 扫描 src/*.cpp 作为源文件
#   2. 自动添加 include/ 为公共头文件目录
#   3. 自动生成 export.hpp（模拟 Talos 的 export.hpp）【当前版本未实现该部分】
#   4. 自动启用 PCH（src/pch.hpp 存在时）
#
# 用法：
#   crate(my_math      # crate 名
#       DEPENDENCIES  # 依赖的其他 crate
#           io
#   )
# ===========================================================================


# crate() 函数：声明一个模块化静态库
# 参数：
#   name            - crate 名称，第一个位置参数
#   DEPENDENCIES    - 关键字参数，后跟依赖库/其他crate列表
function(crate name)
    # ---------- CMake关键字参数解析配置 ----------
    set(options)                     # options：布尔开关，这里不需要任何开关，置空
    set(oneValueArgs)                # oneValueArgs：单值关键字，这里不需要，置空
    set(multiValueArgs DEPENDENCIES) # multiValueArgs：多值关键字，识别DEPENDENCIES，后面可跟多个依赖

    # 解析传入的剩余参数 ${ARGN}，解析后的变量全部带上 CRATE_ 前缀
    # 例：DEPENDENCIES io → 变量 CRATE_DEPENDENCIES = "io"
    cmake_parse_arguments(CRATE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})


    # ---------- 步骤1：自动收集当前目录下 src/*.cpp 全部源码 ----------
    # GLOB：递归收集所有子目录下的 .cpp 文件
    # CONFIGURE_DEPENDS：告诉CMake，文件增删时，重新运行cmake配置阶段，不用手动删build
    # CMAKE_CURRENT_SOURCE_DIR：当前执行该函数的 CMakeLists.txt 所在源码目录
    file(GLOB sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

    # 如果没有找到任何cpp文件，直接报错终止编译，crate必须要有源码
    if(NOT sources)
        message(FATAL_ERROR "crate(${name}): src/*.cpp 未找到源文件")
    endif()


    # ---------- 步骤2：判断是否存在 include 头文件文件夹 ----------
    set(include_dir "${CMAKE_CURRENT_SOURCE_DIR}/include")
    if(EXISTS "${include_dir}")
        set(has_include TRUE)
    else()
        set(has_include FALSE)
    endif()


    # ---------- 步骤3：创建静态库目标，开启C++20标准 ----------
    add_library(${name} STATIC ${sources})   # 创建静态库，库名字就是传入的crate名字
    target_compile_features(${name} PUBLIC cxx_std_20) # PUBLIC：使用这个库的上层目标也会继承C++20


    # ---------- 步骤4：配置PUBLIC头文件路径 ----------
    # $<BUILD_INTERFACE:...>生成器表达式：仅在构建阶段生效，安装时不会导出该路径
    # PUBLIC：自己这个库能用，同时所有链接这个库的上层目标也能找到include下的头文件
    if(has_include)
        target_include_directories(${name}
            PUBLIC
                $<BUILD_INTERFACE:${include_dir}>
        )
    endif()


    # ---------- 步骤5：循环把DEPENDENCIES里面所有依赖链接到本库 ----------
    # CRATE_DEPENDENCIES 就是解析出来 DEPENDENCIES 后面跟的全部库名
    # PUBLIC：本库链接依赖，同时链接本库的上层也间接链接这些依赖
    foreach(dep IN LISTS CRATE_DEPENDENCIES)
        target_link_libraries(${name} PUBLIC ${dep})
    endforeach()


    # ---------- 步骤6：自动预编译头PCH ----------
    # 如果当前crate的src目录存在pch.hpp，自动作为私有预编译头
    set(pch_file "${CMAKE_CURRENT_SOURCE_DIR}/src/pch.hpp")
    if(EXISTS "${pch_file}")
        # PRIVATE：预编译头只作用于当前这个crate，不会传给依赖它的上层
        target_precompile_headers(${name} PRIVATE ${pch_file})
    endif()


    # ---------- 步骤7：打印日志，提示哪个crate注册成功，打印搜集到的源文件 ----------
    message(STATUS "crate(${name}): registered, sources=${sources}")
endfunction()
