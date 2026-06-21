# ==============================================================================
# Talos 机器人火控系统(FCS) 根目录完整CMakeLists.txt
# 整体定位：C++23 跨平台机器人视觉弹道解算工程，支持PC/嵌入式ARM，多AI推理后端、内存竞态检测、Foxglove可视化、一键离线打包部署
# 工程分层：底层工具crates -> 第三方vendored库 -> 五层火控核心FCS -> 可视化模块 -> 可执行程序/测试/基准 -> 安装打包逻辑
# 支持平台：Linux x86_64/aarch64、macOS；支持交叉编译、micromamba开发环境、ASAN/TSAN调试
# ==============================================================================
cmake_minimum_required(VERSION 3.14)
# 最低CMake版本要求3.14，依赖FetchContent、生成器表达式、现代导入Target语法

# 引入自定义工程工具宏文件，封装屏蔽第三方告警、工具辅助函数
include(cmake/crate-utils.cmake)

# 项目基础定义：项目名talos，版本0.1，同时启用C/C++源码编译
project(talos VERSION 0.1 LANGUAGES C CXX)

# 加载全局编译器配置：统一GCC/Clang/AppleClang告警、优化、平台编译选项
include(cmake/compiler-spec.cmake)

# ------------------------------------------------------------------------------
# 全局安装路径覆盖逻辑
# CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT：标记用户未手动指定安装目录
# 默认不装到系统/usr/local，强制安装至项目根目录，实现工程内自包含部署
# ------------------------------------------------------------------------------
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    # FORCE强制覆盖缓存内默认路径，CACHE PATH给GUI工具显示说明
    set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}" CACHE PATH "Install path prefix." FORCE)
endif()

# ------------------------------------------------------------------------------
# 全局C++标准锁定配置
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 23)                # 启用C++23新标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)       # 编译器必须支持C++23，禁止自动降级
set(CMAKE_CXX_EXTENSIONS OFF)             # 关闭GNU/Clang编译器扩展，保证跨平台语法统一

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)     # 生成compile_commands.json，给Clangd/VSCode静态语法分析、LSP使用

# 设置编译阶段二进制输出目录：构建目录下bin文件夹，和最终安装产物隔离
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
# 定义安装时二进制/动态库存放目录：项目根目录bin
set(TALOS_INSTALL_BIN_DIR "${PROJECT_SOURCE_DIR}/bin")

# 获取项目源码绝对真实路径（消除软链接影响），用于编译日志脱敏
get_filename_component(PROJECT_SOURCE_DIR_ABS "${CMAKE_SOURCE_DIR}" REALPATH)

# GCC/Clang专属编译参数：路径脱敏，报错/日志中绝对源码路径替换为空，保护开发环境目录结构
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-fmacro-prefix-map=${PROJECT_SOURCE_DIR_ABS}/=)
endif ()

# ==============================================================================
# 调试模块1：AddressSanitizer 内存消毒器
# 检测：野指针、数组越界、内存泄漏、重复释放、栈溢出等内存错误
# ==============================================================================
option(ENABLE_ASAN "Enable AddressSanitizer for memory leak detection" OFF)

if(ENABLE_ASAN)
    message(STATUS "AddressSanitizer: ENABLED")
    # 全局编译参数
    add_compile_options(
        -fsanitize=address          # 开启地址内存检测
        -fno-omit-frame-pointer     # 保留栈帧指针，打印完整调用堆栈
        -fno-optimize-sibling-calls # 关闭尾调用优化，防止栈信息丢失
        -g3                         # 最高等级调试符号，包含宏、局部变量名
    )
    # 链接阶段必须同步开启sanitizer，否则检测失效
    add_link_options(-fsanitize=address -g3)
    message(STATUS "  Compile flags: -fsanitize=address -fno-omit-frame-pointer -fno-optimize-sibling-calls -g3")
    message(STATUS "  Link flags:    -fsanitize=address -g3")
else()
    message(STATUS "AddressSanitizer: DISABLED")
endif()

# ==============================================================================
# 调试模块2：ThreadSanitizer 线程消毒器
# 检测：多线程数据竞争、锁顺序死锁、无同步共享变量访问
# 注意：ASAN与TSAN不能同时开启，CMake未做互斥校验，使用时二选一
# ==============================================================================
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

# 将项目cmake目录加入CMake模块搜索路径，find_package可直接读取cmake下依赖配置文件
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ==============================================================================
# RPATH 动态链接库运行时搜索路径配置（工程离线运行核心）
# Linux $ORIGIN / macOS @loader_path 代表当前可执行/库自身所在目录
# 实现：拷贝bin文件夹到任意设备可直接运行，无需配置LD_LIBRARY_PATH
# ==============================================================================
# 区分操作系统设置基础rpath标识
if(APPLE)
    set(TALOS_INSTALL_RPATH "@loader_path")
else()
    set(TALOS_INSTALL_RPATH "$ORIGIN")
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)  # 将链接阶段依赖库路径写入rpath
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)        # 编译阶段构建产物启用$ORIGIN规则
set(CMAKE_INSTALL_RPATH "${TALOS_INSTALL_RPATH}")

# 适配micromamba conda开发环境：自动提取conda库目录注入rpath
if(CMAKE_CXX_COMPILER MATCHES "micromamba")
    # 编译器路径向上两级，定位micromamba环境根目录
    get_filename_component(MICROMAMBA_LIB_DIR ${CMAKE_CXX_COMPILER} DIRECTORY)
    get_filename_component(MICROMAMBA_LIB_DIR ${MICROMAMBA_LIB_DIR} DIRECTORY)
    set(MICROMAMBA_LIB_DIR "${MICROMAMBA_LIB_DIR}/lib")
    # 编译阶段rpath追加构建目录 + conda lib目录
    set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR};${MICROMAMBA_LIB_DIR}")
    # 打包安装时保留conda库搜索路径
    list(APPEND CMAKE_INSTALL_RPATH "${MICROMAMBA_LIB_DIR}")
endif()

# 加载依赖查找、单元测试、标定工具三套子cmake脚本
include(cmake/dependencies.cmake)
include(cmake/test.cmake)
include(cmake/calibration.cmake)

# 统一给fmt、spdlog第三方库设置RPATH打包属性
foreach(_runtime_target IN ITEMS fmt spdlog)
    if(TARGET "${_runtime_target}")
        set_target_properties(${_runtime_target} PROPERTIES
            BUILD_WITH_INSTALL_RPATH TRUE
            INSTALL_RPATH "${TALOS_INSTALL_RPATH}"
        )
    endif()
endforeach()

# ==============================================================================
# 内置第三方库 3dparty/ 目录（vendored源码内置，不依赖系统预装）
# ==============================================================================

# ------------------------------
# tomlplusplus TOML配置解析库
# 不使用header-only模式，编译静态库降低编译耗时；关闭异常适配嵌入式实时系统
# ------------------------------
add_library(toml_impl STATIC
        ${tomlplusplus_SOURCE_DIR}/src/toml.cpp
)
# 调用工具宏屏蔽第三方库内部编译告警
crate_silence_warnings(toml_impl)
# SYSTEM标记头文件目录，编译器忽略该目录所有警告
target_include_directories(toml_impl SYSTEM PUBLIC
        ${tomlplusplus_SOURCE_DIR}/include
)
# 编译宏：关闭仅头文件模式、禁用C++异常
target_compile_definitions(toml_impl PUBLIC
        TOML_HEADER_ONLY=0
        TOML_EXCEPTIONS=0
)

# ------------------------------
# magic_enum 编译期枚举反射头文件库
# 限制枚举数值范围0~16，减少模板实例化编译膨胀，适配机器人有限枚举ID
# ------------------------------
add_library(magic_enum INTERFACE)
target_include_directories(magic_enum INTERFACE 3dparty/magic_enum)
target_compile_definitions(magic_enum INTERFACE
        MAGIC_ENUM_RANGE_MIN=0
        MAGIC_ENUM_RANGE_MAX=16
)

# ------------------------------
# lieplusplus SO3李群几何计算库（四元数、旋转矩阵、姿态变换）
# 纯头文件INTERFACE库
# ------------------------------
add_library(lieplusplus INTERFACE)
target_include_directories(lieplusplus INTERFACE 3dparty/lieplusplus)

# ------------------------------
# foxglove_cpp Foxglove Studio可视化通信动态库
# 用于推送图像、检测框、轨迹、卡尔曼状态至可视化工具调试
# ------------------------------
add_library(foxglove_cpp SHARED ${FOXGLOVE_SOURCES})
crate_silence_warnings(foxglove_cpp)

target_include_directories(foxglove_cpp SYSTEM PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include_foxglove
        ${foxglove_SOURCE_DIR}/include
        ${foxglove_SOURCE_DIR}/include/foxglove
)

# 链接静态foxglove底层库，macOS额外链接系统框架并锁定最低系统版本
target_link_libraries(foxglove_cpp PUBLIC
        ${foxglove_SOURCE_DIR}/lib/libfoxglove.a
        "$<$<PLATFORM_ID:Darwin>:-framework IOKit>"
        "$<$<PLATFORM_ID:Darwin>:-framework Security>"
        "$<$<PLATFORM_ID:Darwin>:-framework CoreFoundation>"
        "$<$<PLATFORM_ID:Darwin>:-framework SystemConfiguration>"
        "$<$<PLATFORM_ID:Darwin>:-mmacosx-version-min=15.5>"
)

# ==============================================================================
# Crates 自研模块化组件（Rust风格分层静态库，低耦合基础能力）
# 依赖顺序从上至下：基础工具 -> 数学坐标变换 -> 调度框架 -> 通信图像传输 -> 硬件驱动
# ==============================================================================
# 基础工具层
add_subdirectory(crates/toml)       # toml配置文件封装层，封装toml_impl
add_subdirectory(crates/log)        # spdlog日志全局封装
add_subdirectory(crates/math)        # 通用数值数学工具
add_subdirectory(crates/fast_tf)     # 轻量化坐标变换TF库，相机/云台位姿转换

# 实时调度框架层
add_subdirectory(crates/primitive)   # 全局消息结构体、基础数据原语定义
add_subdirectory(crates/scheduler)  # 多任务实时调度器，工程核心运行框架

# 通信&图像传输层
add_subdirectory(crates/quanta)     # 超低码率图像压缩传输组件
add_subdirectory(crates/chiral)     # 双向IPC进程间通信模块

# 硬件驱动层
add_subdirectory(crates/hardware/at_gimbal)    # AT系列云台控制驱动
add_subdirectory(crates/hardware/hik_camera_driver) # 海康工业相机SDK封装
add_subdirectory(crates/hardware_daedalus)   # 底层硬件抽象统一接口

# ==============================================================================
# Build Info 编译信息注入模块
# 编译时抓取Git版本、编译时间、主机名，编译宏注入二进制，运行时打印版本信息
# ==============================================================================
# 临时保存用户原有时区，用于生成标准东八区时间戳，生成后恢复原时区
if(DEFINED ENV{TZ})
    set(TALOS_PREVIOUS_TZ "$ENV{TZ}")
    set(TALOS_HAD_PREVIOUS_TZ TRUE)
else()
    set(TALOS_HAD_PREVIOUS_TZ FALSE)
endif()

# 临时切换时区UTC-8（东八区），生成ISO8601标准构建时间戳
set(ENV{TZ} "UTC-8")
string(TIMESTAMP TALOS_BUILD_DATE "%Y-%m-%dT%H:%M:%S+0800")
# 恢复用户原始时区环境变量，避免污染编译环境
if(TALOS_HAD_PREVIOUS_TZ)
    set(ENV{TZ} "${TALOS_PREVIOUS_TZ}")
else()
    unset(ENV{TZ})
endif()

# 查找Git工具，抓取仓库版本信息
find_package(Git QUIET)

# 初始化版本信息默认值
set(TALOS_GIT_HASH "unknown")
set(TALOS_GIT_BRANCH "unknown")
set(TALOS_GIT_DIRTY_SUFFIX "")
set(TALOS_BUILD_HOST "unknown")

# 执行hostname命令获取编译主机名称
execute_process(
        COMMAND hostname
        OUTPUT_VARIABLE TALOS_BUILD_HOST_OUTPUT
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT TALOS_BUILD_HOST_OUTPUT STREQUAL "")
    set(TALOS_BUILD_HOST "${TALOS_BUILD_HOST_OUTPUT}")
endif()

# Git工具存在时，抓取commit、分支、工作区脏标记
if(Git_FOUND)
    # 获取6位短commit hash
    execute_process(
            COMMAND "${GIT_EXECUTABLE}" -C "${PROJECT_SOURCE_DIR_ABS}" rev-parse --short=6 HEAD
            OUTPUT_VARIABLE TALOS_GIT_HASH_OUTPUT
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT TALOS_GIT_HASH_OUTPUT STREQUAL "")
        set(TALOS_GIT_HASH "${TALOS_GIT_HASH_OUTPUT}")
    endif()

    # 获取当前分支名，分离HEAD时 fallback到commit hash
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

    # 检测工作区是否存在未提交修改，有则追加+dirty标记
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

# 拼接完整commit标识：短hash+dirty标记
set(TALOS_GIT_COMMIT "${TALOS_GIT_HASH}${TALOS_GIT_DIRTY_SUFFIX}")

# 静态库：编译信息实现文件
add_library(talos_build_info STATIC
        src/fcs/runtime/build_info.cpp
)
# 头文件检索路径
target_include_directories(talos_build_info PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs
)
# 注入编译宏，代码中直接使用字符串常量
target_compile_definitions(talos_build_info PRIVATE
        TALOS_BUILD_INFO_DATE="${TALOS_BUILD_DATE}"
        TALOS_BUILD_INFO_HOST="${TALOS_BUILD_HOST}"
        TALOS_BUILD_INFO_GIT_COMMIT="${TALOS_GIT_COMMIT}"
        TALOS_BUILD_INFO_GIT_BRANCH="${TALOS_GIT_BRANCH}"
)

# ==============================================================================
# FCS 五层火控核心业务动态库（工程核心逻辑）
# 分层定义：
# L1 传感器采集层：相机硬件采集、数据输出接口
# L2 感知层：装甲/能量机关/符文视觉检测，多推理后端
# L3 状态估计层：EKF卡尔曼滤波、目标跟踪、能量机关状态解算
# L4 规划层：目标选择、瞄准决策、弹道预规划
# L5 武器弹道层：MPC模型预测控制弹道优化、发射解算
# ==============================================================================
# 基础源码列表
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

# 交叉编译嵌入式设备时，添加C23标准兼容垫片文件
if(CMAKE_CROSSCOMPILING)
    list(APPEND FCS_SOURCES cmake/toolchains/isoc23_shim.cpp)
endif()

# NVIDIA TensorRT GPU推理后端，库存在则追加对应实现文件并启用宏
if(TensorRT_FOUND)
    list(APPEND FCS_SOURCES src/fcs/L2_perception/armor/backends/tensor_rt.cpp)
    message(STATUS "TensorRT backend enabled")
else()
    message(STATUS "TensorRT backend disabled (TensorRT not found)")
endif()

# Axera 端侧NPU嵌入式推理后端，存在则启用
if(AXERA_FOUND)
    list(APPEND FCS_SOURCES src/fcs/L2_perception/armor/backends/axera.cpp)
    message(STATUS "Axera backend enabled")
else()
    message(STATUS "Axera backend disabled (AXERA not found)")
endif()

# 编译火控核心动态库fcs
add_library(fcs SHARED ${FCS_SOURCES})

# 对外暴露头文件目录
target_include_directories(fcs PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs
        ${CMAKE_CURRENT_SOURCE_DIR}/crates/quanta/src
        # TensorRT仅库存在时暴露头文件
        $<$<TARGET_EXISTS:TensorRT::TensorRT>:${TensorRT_INCLUDE_DIRS}>
)

# CUDA运行时标记，存在则链接cuda库，统一宏标记
if(CUDA_FOUND)
    target_link_libraries(fcs PUBLIC ${CUDA_LIBRARIES})
    target_include_directories(fcs SYSTEM PUBLIC ${CUDA_INCLUDE_DIRS})
    target_compile_definitions(fcs PUBLIC TALOS_HAS_CUDA_RUNTIME=0)
    message(STATUS "Linking with CUDA runtime: ${CUDA_LIBRARIES}")
else()
    target_compile_definitions(fcs PUBLIC TALOS_HAS_CUDA_RUNTIME=0)
endif()

# TensorRT编译宏开关
if(TensorRT_FOUND)
    target_compile_definitions(fcs PUBLIC TALOS_HAS_TENSORRT=1)
    message(STATUS "TensorRT support enabled")
else()
    target_compile_definitions(fcs PUBLIC TALOS_HAS_TENSORRT=0)
endif()

# Axera NPU编译宏、头文件、链接库
if(AXERA_FOUND)
    target_compile_definitions(fcs PUBLIC TALOS_HAS_AXERA=1)
    target_include_directories(fcs SYSTEM PUBLIC ${AXERA_INCLUDE_DIRS})
    target_link_libraries(fcs PUBLIC ${AXERA_LIBRARIES})
    message(STATUS "Axera support enabled")
else()
    target_compile_definitions(fcs PUBLIC TALOS_HAS_AXERA=0)
    message(STATUS "Axera support disabled")
endif()

# 临时关闭数组越界告警，适配第三方数值库
target_compile_options(fcs PRIVATE -Wno-error=array-bounds)

# 依赖链路：PUBLIC对外暴露，PRIVATE仅内部实现使用
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
        # TensorRT库存在时才链接
        $<$<TARGET_EXISTS:TensorRT::TensorRT>:TensorRT::TensorRT>
        $<$<TARGET_EXISTS:TensorRT::nvonnxparser>:TensorRT::nvonnxparser>
PRIVATE
        lieplusplus
)

# 预编译头文件，大幅缩短编译耗时
target_precompile_headers(fcs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs/pch.hpp)

# ==============================================================================
# FCS可视化模块动态库 fcs_visualization
# 基于foxglove_cpp封装，分层推送感知/估计/规划调试数据，业务与可视化解耦，正式部署可裁剪
# ==============================================================================
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

# 对外头文件路径
target_include_directories(fcs_visualization PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs
)
# 第三方可视化头文件标记SYSTEM屏蔽告警
target_include_directories(fcs_visualization SYSTEM PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs_visualization
)

# 可视化模块依赖：核心FCS库+调度+foxglove通信+OpenCV图像
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
# 可视化模块预编译头
target_precompile_headers(fcs_visualization PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/fcs_visualization/pch.hpp)

# ==============================================================================
# FCS单元测试（GTest）
# 全局开关TALOS_BUILD_TESTING开启时，自动扫描src/fcs/tests下所有*_test.cpp生成测试程序，注册CTest用例
# ==============================================================================
if(TALOS_BUILD_TESTING)
    file(GLOB FCS_TESTS "${CMAKE_CURRENT_SOURCE_DIR}/src/fcs/tests/*_test.cpp")
    foreach(test_file ${FCS_TESTS})
        # 提取文件名作为测试程序名
        get_filename_component(test_name ${test_file} NAME_WE)
        add_executable(${test_name} ${test_file})
        # 注入源码根目录宏，测试用例读取配置文件使用
        target_compile_definitions(${test_name} PRIVATE
            TALOS_SOURCE_DIR="${PROJECT_SOURCE_DIR_ABS}"
        )
        # 测试依赖：GTest、线程库、核心FCS库、数学、json
        target_link_libraries(${test_name} PRIVATE
            GTest::gtest_main
            Threads::Threads
            fcs
            fast_tf
            nlohmann_json::nlohmann_json
        )
        # 注册CTest测试用例，支持ctest批量执行
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
endif()

# ==============================================================================
# 可执行程序集合
# 1. talos：主火控程序完整运行入口
# 2. talos-playground：算法/硬件独立调试沙盒
# 3. talos-replay：离线日志回放工具，复现比赛数据调试感知算法
# 4. bench_mpc：MPC弹道优化基准性能测试
# ==============================================================================
# 主程序 talos
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

# 调试沙盒 playground
add_executable(talos-playground src/playground.cpp)
# 硬件驱动私有头文件检索
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

# 离线回放工具 talos-replay
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

# ------------------------------
# MPC弹道优化基准测试模块
# 对比TinyMPC浮点求解器与自研定点MPC求解器性能
# ------------------------------
# TinyMPC第三方静态库封装
add_library(tinympc_bench STATIC
        3dparty/tinympc/admm.cpp
        3dparty/tinympc/tiny_api.cpp
        3dparty/tinympc/rho_benchmark.cpp
)
target_include_directories(tinympc_bench SYSTEM PUBLIC 3dparty/tinympc)
target_link_libraries(tinympc_bench PUBLIC Eigen3::Eigen)
crate_silence_warnings(tinympc_bench)
# 屏蔽头文件卫生告警
target_compile_options(tinympc_bench PRIVATE -Wno-header-hygiene)

# MPC性能测试可执行文件
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

# ==============================================================================
# Install 一键打包部署逻辑（核心工程交付能力）
# 自定义工具函数：自动收集所有动态库(.so/.dylib)，统一复制至bin目录，实现离线交付
# ==============================================================================
# ------------------------------
# 工具函数1：单个动态库安装，跟随符号链接复制真实库文件
# 参数path：动态库绝对路径
# ------------------------------
function(talos_install_shared_library path)
    if(EXISTS "${path}" OR IS_SYMLINK "${path}")
        install(CODE
            "file(INSTALL DESTINATION \"${TALOS_INSTALL_BIN_DIR}\" TYPE SHARED_LIBRARY FOLLOW_SYMLINK_CHAIN FILES \"${path}\")"
        )
    endif()
endfunction()

# ------------------------------
# 工具函数2：批量提取导入目标的所有动态库路径
# out_var：输出变量存储库路径列表；ARGN：输入多个导入Target名
# 遍历Target所有导入属性，筛选linux/mac动态库绝对路径，去重返回
# ------------------------------
function(talos_collect_imported_shared_libraries out_var)
    set(_collected "")

    foreach(_target IN LISTS ARGN)
        if(NOT TARGET "${_target}")
            continue()
        endif()

        # 需要检索的Target属性列表
        set(_props
            IMPORTED_LOCATION
            IMPORTED_IMPLIB
            INTERFACE_LINK_LIBRARIES
        )

        # 读取导入库配置，追加Debug/Release等分配置导入路径属性
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

        # 遍历所有属性，提取有效动态库路径
        foreach(_prop IN LISTS _props)
            get_target_property(_values "${_target}" ${_prop})
            if(NOT _values OR _values STREQUAL "${_prop}-NOTFOUND")
                continue()
            endif()

            foreach(_value IN LISTS _values)
                # 筛选存在、绝对路径、后缀为so/dylib的动态库
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

# 基础待安装目标集合：自有动态库、所有可执行程序
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

# 可选安装目标：标定程序，目标不存在则从安装列表移除
set(TALOS_OPTIONAL_INSTALL_TARGETS
        rm_calibration
)
foreach(_target IN LISTS TALOS_OPTIONAL_INSTALL_TARGETS)
    if(NOT TARGET "${_target}")
        list(REMOVE_ITEM TALOS_INSTALL_TARGETS "${_target}")
    endif()
endforeach()
# 标定程序存在则追加至安装列表
if(TARGET rm_calibration)
    list(APPEND TALOS_INSTALL_TARGETS rm_calibration)
endif()

# 批量安装编译产出：可执行(RUNTIME)、动态库(LIBRARY)全部输出至bin目录
install(TARGETS ${TALOS_INSTALL_TARGETS}
        RUNTIME DESTINATION "${TALOS_INSTALL_BIN_DIR}"
        LIBRARY DESTINATION "${TALOS_INSTALL_BIN_DIR}"
)

# ------------------------------
# 自动复制海康相机SDK运行时动态库
# 根据CPU架构x86_64/aarch64选择对应lib目录，过滤FFMPEG相关库不重复复制
# ------------------------------
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
    # 过滤av/sw开头FFMPEG库，避免重复拷贝
    list(FILTER TALOS_HIK_RUNTIME_LIBS EXCLUDE REGEX "/lib/[^/]*/lib(av|sw)")
    foreach(_lib IN LISTS TALOS_HIK_RUNTIME_LIBS)
        talos_install_shared_library("${_lib}")
    endforeach()
endif()

# ------------------------------
# FFMPEG音视频编解码运行库收集
# 优先读取TALOS_FFMPEG_ROOT预定义路径，无则通过pkgconfig导入目标提取库路径
# ------------------------------
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
    # 无预定义FFMPEG目录，自动提取pkgconfig导入目标的动态库
    talos_collect_imported_shared_libraries(TALOS_FFMPEG_RUNTIME_LIBS
        PkgConfig::AVCODEC
        PkgConfig::AVFORMAT
        PkgConfig::AVFILTER
        PkgConfig::SWSCALE
        PkgConfig::SWRESAMPLE
        PkgConfig::AVUTIL
    )
endif()
# 批量安装FFMPEG动态库
foreach(_lib IN LISTS TALOS_FFMPEG_RUNTIME_LIBS)
    talos_install_shared_library("${_lib}")
endforeach()

# ------------------------------
# FetchContent拉取的第三方库运行时库收集：fmt、spdlog、onnxruntime
# 从构建目录_deps下检索编译生成的so/dylib并复制
# ------------------------------
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
# 批量安装拉取的第三方动态库
foreach(_lib IN LISTS TALOS_FETCHED_RUNTIME_LIBS)
    talos_install_shared_library("${_lib}")
endforeach()