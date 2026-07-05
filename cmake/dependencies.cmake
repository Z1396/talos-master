# 全局关闭CMake废弃API警告，消除编译时一堆deprecated提示
set(CMAKE_WARN_DEPRECATED OFF)

# CMP0135策略：FetchContent下载文件时，时间戳更新规则
# OLD行为：不自动刷新文件时间，避免重复重编译
# 判断当前CMake版本是否支持策略CMP0135
if(POLICY CMP0135)
    # 启用旧版本兼容行为
    cmake_policy(SET CMP0135 OLD)
endif()

# ====================== 交叉编译Sysroot路径修复工具函数组 ======================
# 函数1：把路径中本机sysroot绝对路径，替换为适配嵌入式sysroot的/usr路径
# out_var：输出修正后的路径变量名
# in_value：待修正的原始路径
# 自定义函数：交叉编译时，修正宿主机绝对sysroot路径，映射到嵌入式sysroot/usr标准目录
# 参数说明：
#   out_var：输出变量名，修正后的路径会赋值给这个外层变量
#   in_value：待修正的原始绝对路径字符串
function(_talos_relocate_sysroot_usr_path out_var in_value)
    # 临时变量保存输入路径，后续统一修改
    set(_fixed "${in_value}")

    # ========== 分支1：非交叉编译 / 未配置嵌入式sysroot，直接原路返回 ==========
    # CMAKE_CROSSCOMPILING：CMake内置变量，ON=当前是交叉编译模式
    # TALOS_SYSROOT：项目自定义变量，嵌入式平台的sysroot根目录（如/opt/arm-linux/sysroot）
    if (NOT CMAKE_CROSSCOMPILING OR NOT TALOS_SYSROOT)
        # PARENT_SCOPE：把值传递给调用函数的上层作用域变量 out_var
        set(${out_var} "${_fixed}" PARENT_SCOPE)
        return() # 终止当前函数，不再执行下方逻辑
    endif ()

    # ========== 分支2：三类无需修正的路径，直接返回原值 ==========
    # 匹配规则任意一条满足就跳过处理：
    # 1. ^\\$< ：以 $< 开头，是CMake生成器表达式（平台判断$<PLATFORM_ID>等）
    # 2. NOT IS_ABSOLUTE ：路径是相对路径，不是绝对路径
    # 3. EXISTS "${_fixed}" ：宿主机本地真实存在该文件，不需要映射sysroot
    if (_fixed MATCHES "^\\$<" OR NOT IS_ABSOLUTE "${_fixed}" OR EXISTS "${_fixed}")
        set(${out_var} "${_fixed}" PARENT_SCOPE)
        return()
    endif ()

    # ========== 分支3：判断输入路径是否以宿主机sysroot绝对路径开头 ==========
    # string(FIND 查找前缀起始下标，匹配成功返回0（从第0位开始）
    string(FIND "${_fixed}" "${TALOS_SYSROOT}/" _sysroot_prefix_pos)
    # 下标不等于0：路径不是以sysroot/开头，无需替换，直接返回
    if (NOT _sysroot_prefix_pos EQUAL 0)
        set(${out_var} "${_fixed}" PARENT_SCOPE)
        return()
    endif ()

    # ========== 截取sysroot/之后的相对子路径 ==========
    # 1. 获取 sysroot/ 前缀字符串长度
    string(LENGTH "${TALOS_SYSROOT}/" _sysroot_prefix_len)
    # 2. 截取：从前缀长度位置开始，截取到末尾（-1代表剩余全部字符）
    string(SUBSTRING "${_fixed}" ${_sysroot_prefix_len} -1 _relative_to_sysroot)

    # ========== 判断截取后的子路径是否是系统标准目录：include/lib/lib64/share/bin ==========
    # 正则匹配：以指定目录开头，后面可带子路径 /xxx
    if (_relative_to_sysroot MATCHES "^(include|lib|lib64|share|bin)(/.*)?$")
        # 拼接嵌入式标准路径 sysroot/usr/xxx
        set(_candidate "${TALOS_SYSROOT}/usr/${_relative_to_sysroot}")
        # 如果sysroot/usr下该目录真实存在，覆盖替换原路径
        if (EXISTS "${_candidate}")
            set(_fixed "${_candidate}")
        endif ()
    endif ()

    # ========== 最终输出修正完成的路径给外层调用处 ==========
    set(${out_var} "${_fixed}" PARENT_SCOPE)
endfunction()

# 函数2：处理单个导入库target（第三方预编译库，如onnx、海康SDK）
# 遍历库的头文件目录、链接目录、库文件路径，调用上面函数修正sysroot路径
# 功能函数：交叉编译时，批量修正【外部导入库(IMPORTED Target)】内部所有带sysroot的绝对路径
# 参数 target：待处理的导入库目标名（如 opencv::opencv、Eigen3::Eigen、预编译第三方库）
function(_talos_fix_imported_target_sysroot target)
    # 防御判断：如果这个目标根本不存在，直接退出，避免报错
    if (NOT TARGET "${target}")
        return()
    endif ()

    # 获取目标属性 IMPORTED，判断是否是外部导入库
    # IMPORTED=TRUE：代表不是当前工程源码编译出来，是find_package引入的预编译库
    get_target_property(_is_imported "${target}" IMPORTED)
    if (NOT _is_imported)
        # 本项目内部源码库无需修正路径，直接退出
        return()
    endif ()

    # ====================== 第一部分：修正INTERFACE对外暴露的路径属性 ======================
    # 四类需要处理的属性：头文件目录、系统头文件、链接库目录、依赖库列表
    foreach(_prop IN ITEMS
            INTERFACE_INCLUDE_DIRECTORIES
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
            INTERFACE_LINK_DIRECTORIES
            INTERFACE_LINK_LIBRARIES)
        # 取出当前属性存储的全部路径/依赖列表
        get_target_property(_values "${target}" ${_prop})
        # 属性不存在 / 为空，跳过本轮循环
        if (NOT _values OR _values STREQUAL "${_prop}-NOTFOUND")
            continue()
        endif ()

        set(_rewritten_values "") # 存放修正后的新路径列表
        set(_changed FALSE)       # 标记当前属性是否有路径被修改

        # 遍历该属性里每一条路径值
        foreach(_value IN LISTS _values)
            # 调用上一个工具函数，把sysroot/lib 自动转为 sysroot/usr/lib
            _talos_relocate_sysroot_usr_path(_rewritten_value "${_value}")
            # 将修正后的路径加入新列表
            list(APPEND _rewritten_values "${_rewritten_value}")
            # 只要任意一条路径发生变化，标记_changed为真
            if (NOT "${_rewritten_value}" STREQUAL "${_value}")
                set(_changed TRUE)
            endif ()
        endforeach ()

        # 如果有路径被修正，覆盖更新目标的原始属性
        if (_changed)
            set_property(TARGET "${target}" PROPERTY ${_prop} "${_rewritten_values}")
        endif ()
    endforeach ()

    # ====================== 第二部分：修正库本体文件(.so/.a/.dll)的绝对路径 ======================
    # 基础库路径属性：
    # IMPORTED_LOCATION：Linux/macOS 动态/静态库完整路径
    # IMPORTED_IMPLIB：Windows dll对应的导入lib
    set(_import_location_props IMPORTED_LOCATION IMPORTED_IMPLIB)

    # 获取该导入库支持的编译配置：Release / Debug / RelWithDebInfo 等
    get_target_property(_import_configs "${target}" IMPORTED_CONFIGURATIONS)
    # 如果存在多配置区分的库路径（Debug库、Release库分开存放）
    if (_import_configs AND NOT _import_configs STREQUAL "_import_configs-NOTFOUND")
        foreach(_config IN LISTS _import_configs)
            string(TOUPPER "${_config}" _config_upper) # 转大写：Release → RELEASE
            # 追加多配置专属属性：IMPORTED_LOCATION_RELEASE / IMPORTED_IMPLIB_DEBUG
            list(APPEND _import_location_props
                "IMPORTED_LOCATION_${_config_upper}"
                "IMPORTED_IMPLIB_${_config_upper}")
        endforeach ()
    endif ()

    # 遍历所有库文件路径属性，逐个修正sysroot路径
    foreach(_prop IN LISTS _import_location_props)
        get_target_property(_value "${target}" ${_prop})
        # 属性为空则跳过
        if (NOT _value OR _value STREQUAL "${_prop}-NOTFOUND")
            continue()
        endif ()

        # 路径转换
        _talos_relocate_sysroot_usr_path(_rewritten_value "${_value}")
        # 路径改动则更新属性
        if (NOT "${_rewritten_value}" STREQUAL "${_value}")
            set_property(TARGET "${target}" PROPERTY ${_prop} "${_rewritten_value}")
        endif ()
    endforeach ()
endfunction()

# 函数3：入口函数，遍历项目所有target，批量修复全部导入库sysroot路径
function(talos_fixup_cross_sysroot_imported_targets)
    # 非交叉编译/无sysroot，无需修复
    if (NOT CMAKE_CROSSCOMPILING OR NOT TALOS_SYSROOT)
        return()
    endif ()

    # 获取项目内所有注册的target
    get_cmake_property(_all_targets TARGETS)
    foreach(_target IN LISTS _all_targets)
        _talos_fix_imported_target_sysroot("${_target}")
    endforeach ()
endfunction()
# ====================== 交叉编译工具函数结束 ======================

# 系统自带OpenCV，必须安装，包含图像、标定、DNN推理模块
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d dnn)

# 引入自定义工具脚本：FetchContent拉取第三方库、FFmpeg配置
include(${CMAKE_CURRENT_LIST_DIR}/fetch-content-helper.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/ffmpeg.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/fetch-onnxruntime.cmake)

# 本地zip加载fmt 12.0.0，规避系统低版本fmt与Clang21 consteval语法冲突
fetch_dependency(NAME fmt
                 ZIP_URL "${CMAKE_SOURCE_DIR}/3dparty/fmt-12.0.0.zip"
                 ZIP_NAME "fmt-12.0.0.zip")
# fmt编译为静态库，不生成动态so
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# spdlog日志库，绑定外部fmt，避免内置fmt版本冲突，静态编译
set(SPDLOG_FMT_EXTERNAL ON CACHE BOOL "" FORCE)
set(SPDLOG_ENABLE_PCH ON CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_SHARED OFF CACHE BOOL "" FORCE)
fetch_dependency(NAME spdlog
                 ZIP_URL "${CMAKE_SOURCE_DIR}/3dparty/spdlog-1.15.1.zip"
                 ZIP_NAME "spdlog-1.15.1.zip")

# JSON解析库，使用系统安装版本
find_package(nlohmann_json REQUIRED)

# Boost PFR结构体反射库，无系统版本，本地3dparty目录提供头文件
add_library(Boost::pfr INTERFACE IMPORTED)
if(EXISTS "${CMAKE_SOURCE_DIR}/3dparty/boost-pfr/include/boost/pfr.hpp")
    # 本地存在头文件，直接引用
    target_include_directories(Boost::pfr INTERFACE ${CMAKE_SOURCE_DIR}/3dparty/boost-pfr/include)
else()
    # 不存在则生成空桩头文件，编译不报错，运行反射功能失效
    set(_pfr_stub_dir "${CMAKE_BINARY_DIR}/_deps/boost_pfr-src/include")
    if(NOT EXISTS "${_pfr_stub_dir}/boost/pfr.hpp")
        file(MAKE_DIRECTORY "${_pfr_stub_dir}/boost")
        file(WRITE "${_pfr_stub_dir}/boost/pfr.hpp" "// boost::pfr stub\n#pragma once\n#include <tuple>\nnamespace boost::pfr {\ntemplate<class T> constexpr auto fields_as_tuple(T&&) { return std::tuple<>{}; }\ntemplate<class T> constexpr auto get_name(std::size_t) { return \"\"; }\n}\n")
    endif()
    target_include_directories(Boost::pfr INTERFACE ${_pfr_stub_dir})
endif()

# 矩阵计算库Eigen，使用系统版本，下载新版会产生编译警告冲突
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# TOML配置文件解析库，从github拉取指定commit版本
fetch_dependency(NAME tomlplusplus
                 REPO marzer/tomlplusplus
                 VERSION 1c8b7466e4946fcc3bf20484c0e1d001202cca5a)

# TBB多线程并发库，自定义编译配置并清理警告
include(cmake/tbb.cmake)
find_package(TBB REQUIRED)
sanitize_tbb()

# 单元测试框架GTest，开启测试时才加载
if (BUILD_TESTING)
    find_package(GTest REQUIRED)
endif ()

# 非线性优化库Ceres
find_package(Ceres REQUIRED)

# CUDA显卡加速可选依赖，检测到则打印路径，未检测则关闭TensorRT推理后端
find_package(CUDA QUIET)
if (CUDA_FOUND)
    message(STATUS "CUDA found at: ${CUDA_TOOLKIT_ROOT_DIR}")
else ()
    message(STATUS "CUDA not found - TensorRT backend will be disabled")
endif ()

# TensorRT GPU推理加速库，可选依赖
find_package(TensorRT QUIET MODULE)
if (TensorRT_FOUND)
    message(STATUS "TensorRT found at: ${TensorRT_DIR}")
else ()
    message(STATUS "TensorRT not found - TensorRT backend will be disabled")
endif ()

# 性能基准测试库，可选
find_package(benchmark QUIET)

# 系统包管理工具，用于FFmpeg检测
find_package(PkgConfig REQUIRED)
# 执行FFmpeg自定义配置逻辑
talos_configure_ffmpeg()

# 如果指定FFmpeg根目录，把FFmpeg库路径追加到两套RPATH
if (TALOS_FFMPEG_ROOT AND EXISTS "${TALOS_FFMPEG_ROOT}/lib")
    list(APPEND CMAKE_BUILD_RPATH "${TALOS_FFMPEG_ROOT}/lib")
    list(APPEND CMAKE_INSTALL_RPATH "${TALOS_FFMPEG_ROOT}/lib")
    # 去重，避免rpath重复路径
    list(REMOVE_DUPLICATES CMAKE_BUILD_RPATH)
    list(REMOVE_DUPLICATES CMAKE_INSTALL_RPATH)
endif ()

# libusb USB设备通信库，拉取源码手动编译静态库，不使用系统动态库
fetch_dependency(NAME libusb
                 REPO libusb/libusb
                 VERSION v1.0.30)
FetchContent_GetProperties(libusb SOURCE_DIR TALOS_LIBUSB_SOURCE_DIR)

# 创建libusb编译生成配置头文件目录
set(TALOS_LIBUSB_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/libusb")
file(MAKE_DIRECTORY "${TALOS_LIBUSB_CONFIG_DIR}")
# 写入跨平台编译宏config.h
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

# 分系统填充USB平台专属源码、系统依赖库
set(TALOS_LIBUSB_OS_SOURCES "")
set(TALOS_LIBUSB_OS_LIBS "")

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Linux专属宏追加到config.h
    file(APPEND "${TALOS_LIBUSB_CONFIG_DIR}/config.h"
"#define HAVE_ASM_TYPES_H 1
#define HAVE_CLOCK_GETTIME 1
#define HAVE_EVENTFD 1
#define HAVE_PIPE2 1
#define HAVE_PTHREAD_CONDATTR_SETCLOCK 1
#define HAVE_TIMERFD 1
")
    # Linux USB底层实现文件
    list(APPEND TALOS_LIBUSB_OS_SOURCES
        "${TALOS_LIBUSB_SOURCE_DIR}/libusb/os/linux_usbfs.c"
        "${TALOS_LIBUSB_SOURCE_DIR}/libusb/os/linux_netlink.c"
    )
    # Linux依赖rt库
    list(APPEND TALOS_LIBUSB_OS_LIBS rt)
elseif (APPLE)
    # MacOS USB实现文件
    list(APPEND TALOS_LIBUSB_OS_SOURCES
        "${TALOS_LIBUSB_SOURCE_DIR}/libusb/os/darwin_usb.c"
    )
    # MacOS系统框架依赖
    list(APPEND TALOS_LIBUSB_OS_LIBS
        "-framework CoreFoundation"
        "-framework IOKit"
        "-framework Security"
    )
else ()
    # 不支持Windows/其他系统，直接报错终止配置
    message(FATAL_ERROR "talos_libusb_static is only configured for Linux and Darwin")
endif ()

# 编译libusb静态库
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
# 公共头文件目录
target_include_directories(talos_libusb_static SYSTEM PUBLIC
    "${TALOS_LIBUSB_SOURCE_DIR}"
)
# 私有内部头文件、生成的config.h
target_include_directories(talos_libusb_static PRIVATE
    "${TALOS_LIBUSB_SOURCE_DIR}/libusb"
    "${TALOS_LIBUSB_CONFIG_DIR}"
)
# 开启GNU拓展
target_compile_definitions(talos_libusb_static PRIVATE _GNU_SOURCE)
# 屏蔽libusb全部编译警告，隐藏内部符号
if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    target_compile_options(talos_libusb_static PRIVATE -w -fvisibility=hidden)
endif ()
# 链接线程库与系统USB依赖
target_link_libraries(talos_libusb_static PUBLIC
    Threads::Threads
    ${TALOS_LIBUSB_OS_LIBS}
)
# Linux静态链接时排除libusb内部符号导出，避免全局符号冲突
target_link_options(talos_libusb_static INTERFACE
    "$<$<AND:$<PLATFORM_ID:Linux>,$<NOT:$<BOOL:${ENABLE_ASAN}>>>:-Wl,--exclude-libs,ALL>"
)
# C11标准、开启位置无关代码（可嵌入动态库）
set_target_properties(talos_libusb_static PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# 交叉编译场景：批量修复所有第三方导入库sysroot路径
talos_fixup_cross_sysroot_imported_targets()

# 海康工业相机SDK路径定义，本地3dparty存放
set(HIK_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3dparty/hik_sdk")
if (EXISTS "${HIK_SDK_ROOT}")
    message(STATUS "HIK SDK found at: ${HIK_SDK_ROOT}")
else ()
    message(STATUS "HIK SDK not found at: ${HIK_SDK_ROOT}")
endif ()

# Axera AX650嵌入式NPU推理SDK配置，要求部署到/soc目录
unset(AXERA_INCLUDE_DIRS CACHE)
unset(AXERA_SYS_LIBRARY CACHE)
unset(AXERA_ENGINE_LIBRARY CACHE)
unset(AXERA_IVPS_LIBRARY CACHE)
unset(AXERA_IVE_LIBRARY CACHE)
unset(AXERA_VENC_LIBRARY CACHE)

# 查找Axera头文件
find_path(AXERA_INCLUDE_DIRS
    NAMES ax_sys_api.h ax_engine_api.h
    PATH_SUFFIXES "soc/include" "include"
    DOC "Axera SDK include directory"
)
# 查找各类NPU推理静态/动态库
find_library(AXERA_SYS_LIBRARY NAMES ax_sys DOC "Axera system library")
find_library(AXERA_ENGINE_LIBRARY NAMES ax_engine DOC "Axera engine library")
find_library(AXERA_IVPS_LIBRARY NAMES ax_ivps DOC "Axera IVPS library")
find_library(AXERA_IVE_LIBRARY NAMES ax_ive DOC "Axera IVE library")
find_library(AXERA_VENC_LIBRARY NAMES ax_venc DOC "Axera VENC library")

# 全部库头文件找到则开启Axera推理模块
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

# Foxglove可视化SDK平台判断（区分Mac M1/M2、x86 Linux、ARM Linux）
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

# 拉取对应平台预编译Foxglove可视化SDK压缩包
fetch_dependency(NAME foxglove
                 ZIP_URL "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2F${FOXGLOVE_VERSION}/foxglove-${FOXGLOVE_VERSION}-cpp-${FOXGLOVE_PLATFORM}.zip"
                 ZIP_NAME "foxglove-${FOXGLOVE_VERSION}-cpp-${FOXGLOVE_PLATFORM}.zip")
FetchContent_GetProperties(foxglove SOURCE_DIR foxglove_SOURCE_DIR)

# 手动罗列Foxglove源码，避免GLOB全局扫描导致每次构建重新cmake
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

# 工具函数：屏蔽第三方库所有编译警告，消除冗余告警
function(silence_target_warnings tgt)
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(${tgt} PRIVATE -Wno-everything)
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(${tgt} PRIVATE -w)
    elseif (MSVC)
        target_compile_options(${tgt} PRIVATE /W0)
    endif()
endfunction()