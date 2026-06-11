#!/usr/bin/env bash

# =============================================================================
# Talos 跨平台构建 & 运行脚本
# 支持平台与编译器约束（项目强要求）：
#   macOS  : Apple Clang 17+  + libc++  | 任意架构
#   Linux  : Clang 21+       + libstdc++(依赖 GCC 13+ 工具链) | x86_64 / arm64
#
# 用法：
#   ./build.sh [编译选项] [-- 程序运行参数]
# 示例见下方说明
# =============================================================================

# 开启严格模式：脚本健壮性保障
# -E：函数内错误也触发 ERR
# -e：命令出错立即退出
# -u：使用未定义变量直接报错
# -o pipefail：管道中任意命令失败，整条管道返回失败码
set -Eeuo pipefail

# -----------------------------------------------------------------------------
# 终端彩色输出定义
# -----------------------------------------------------------------------------
YELLOW='\033[33m'   # 黄色
GREEN='\033[32m'    # 绿色
RED='\033[31m'      # 红色
BLUE='\033[34m'     # 蓝色
DIM='\033[2m'       # 暗淡色
RESET='\033[0m'     # 恢复默认颜色

# 通用日志输出函数
# $1: 前缀符号  $2: 颜色  $3: 日志内容
status() {
    local symbol="$1"
    local color="$2"
    local msg="$3"
    echo -e "${color}${symbol} ${msg}${RESET}"
}

# 普通信息日志
info() {
    status "[i]" "$BLUE" "$1"
}

# 成功日志
ok() {
    status "[o]" "$GREEN" "$1"
}

# 警告日志
warn() {
    status "[!]" "$YELLOW" "$1"
}

# 错误并退出脚本
die() {
    status "[x]" "$RED" "$1"
    exit 1
}

# -----------------------------------------------------------------------------
# 全局默认参数（运行前初始值）
# -----------------------------------------------------------------------------
BUILD_TYPE="Release"               # 默认编译模式：Release 正式版
BUILD_TARGET="talos"               # 默认编译目标：主程序 talos
ENABLE_CALIBRATION="OFF"           # 是否开启标定模块：默认关闭
ENABLE_ASAN="OFF"                  # 是否开启地址消毒器(内存检测)：默认关闭
ENABLE_TSAN="OFF"                  # 是否开启线程消毒器(竞态检测)：默认关闭
ENABLE_TESTING="OFF"               # 是否编译&运行单元测试：默认关闭
RUN_AFTER_BUILD="ON"               # 编译完成后是否自动运行程序：默认开启
VERBOSE="OFF"                      # 是否开启 CMake 详细编译日志：默认关闭
REDACT_BUILD_OUTPUT="ON"           # 是否脱敏编译输出：默认开启
CROSS_COMPILE="OFF"                # 是否交叉编译(编译ARM板程序)：默认关闭
GENERATOR="Ninja"                  # CMake 构建生成器：固定使用 Ninja(高速构建)
BUILD_DIR="build"                  # 本地编译输出目录
RUN_ARGS=()                        # 程序运行时的启动参数

# 运行时自动探测变量
OS=""                              # 系统：macos / linux
ARCH=""                            # 架构：x86_64 / arm64
BUILD_JOBS=""                      # 编译并发任务数(CPU核心数)
CC=""                              # C 编译器路径
CXX=""                             # C++ 编译器路径
CMAKE_EXTRA_ARGS=()                # CMake 额外全局参数
CMAKE_CONFIGURE_ARGS=()            # CMake 配置阶段参数
CMAKE_BUILD_ARGS=()                # CMake 编译阶段参数
CMAKE_ENV_VARS=()                  # CMake 环境变量

# -----------------------------------------------------------------------------
# 工具函数库
# -----------------------------------------------------------------------------

# 判断命令是否存在
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# 探测当前操作系统
detect_os() {
    local os
    os="$(uname -s)"
    case "$os" in
        Darwin) echo "macos" ;;  # macOS 系统 uname -s 输出 Darwin
        Linux)  echo "linux" ;;
        *)      echo "unknown" ;;
    esac
}

# 探测当前CPU架构
detect_arch() {
    local arch
    arch="$(uname -m)"
    case "$arch" in
        x86_64)         echo "x86_64" ;;
        aarch64|arm64)  echo "arm64" ;;
        *)              echo "unknown" ;;
    esac
}

# 执行命令，取第一行输出，屏蔽错误
get_first_line() {
    "$@" 2>/dev/null | head -n1 || true
}

# 提取 Clang 主版本号（兼容 Apple Clang / 原生 Clang）
get_clang_major_version() {
    local version_str
    version_str="$(get_first_line "$1" --version)"

    # 匹配两种格式：
    # Apple clang version 17.0.0
    # clang version 21.0.0
    if [[ "$version_str" =~ Apple[[:space:]]clang[[:space:]]version[[:space:]]([0-9]+) ]]; then
        echo "${BASH_REMATCH[1]}"
    elif [[ "$version_str" =~ clang[[:space:]]version[[:space:]]([0-9]+) ]]; then
        echo "${BASH_REMATCH[1]}"
    else
        echo "0"
    fi
}

# 判断是否为 Apple 官方 Clang（macOS Xcode 自带）
is_apple_clang() {
    local version_str
    version_str="$(get_first_line "$1" --version)"
    [[ "$version_str" == Apple\ clang* ]]
}

# 提取 GCC 主版本号
get_gcc_major_version() {
    local version_str
    version_str="$(get_first_line "$1" --version)"
    if [[ "$version_str" =~ ([0-9]+)\.[0-9]+\.[0-9]+ ]]; then
        echo "${BASH_REMATCH[1]}"
    else
        echo "0"
    fi
}

# 获取CPU核心数，用于编译并发
num_cpus() {
    case "$OS" in
        macos) sysctl -n hw.ncpu ;;  # macOS 查核心数
        linux) nproc ;;              # Linux 查核心数
        *) echo 4 ;;                 # 未知系统默认4线程
    esac
}

# 数组拼接为字符串（分隔符 $1）
join_by() {
    local IFS="$1"
    shift
    echo "$*"
}

# 检查 Ninja 构建工具是否存在（项目强制依赖 Ninja）
ensure_generator() {
    if ! command_exists ninja; then
        die "Ninja not found. Please install ninja."
    fi
}

# -----------------------------------------------------------------------------
# 编译器选择逻辑
# -----------------------------------------------------------------------------

# macOS 平台编译器选择：优先 Xcode 自带 Apple Clang
select_macos_compiler() {
    local clangxx
    local clangc

    # 优先使用 xcrun 查找 Xcode 内编译器（标准 macOS 开发方式）
    if command_exists xcrun; then
        clangxx="$(xcrun --find clang++ 2>/dev/null || true)"
        clangc="$(xcrun --find clang 2>/dev/null || true)"
    else
        # 兜底：从系统 PATH 查找
        clangxx="$(command -v clang++ || true)"
        clangc="$(command -v clang || true)"
    fi

    # 校验编译器存在性
    [[ -n "$clangxx" && -x "$clangxx" ]] || die "clang++ not found. Install Xcode Command Line Tools."
    [[ -n "$clangc"  && -x "$clangc"  ]] || die "clang not found. Install Xcode Command Line Tools."

    # 检查版本：要求 Apple Clang >= 17
    local major
    major="$(get_clang_major_version "$clangxx")"

    if ! is_apple_clang "$clangxx"; then
        warn "Detected non-Apple clang on macOS: $(get_first_line "$clangxx" --version)"
        warn "This script officially supports Apple Clang on macOS."
    fi

    (( major >= 17 )) || die "Apple Clang 17+ required on macOS. Found: $(get_first_line "$clangxx" --version)"

    # 赋值全局编译器变量
    CC="$clangc"
    CXX="$clangxx"

    # 传给 CMake 指定编译器
    CMAKE_EXTRA_ARGS+=(
        "-DCMAKE_C_COMPILER=${CC}"
        "-DCMAKE_CXX_COMPILER=${CXX}"
    )
}

# Linux 平台选择 Clang：要求 Clang >= 21
select_linux_clang() {
    # 优先级：clang++-22 > clang++-21 > clang++
    local candidates=(
        clang++-22
        clang++-21
        clang++
    )

    local cxx_candidate=""
    local cc_candidate=""
    local major="0"

    for c in "${candidates[@]}"; do
        if command_exists "$c"; then
            major="$(get_clang_major_version "$c")"
            if (( major >= 21 )); then
                cxx_candidate="$(command -v "$c")"
                break
            fi
        fi
    done

    [[ -n "$cxx_candidate" ]] || die "clang++ 21+ not found. Please install clang-21 or newer."

    # 由 clang++ 对应找到 clang
    cc_candidate="${cxx_candidate/clang++/clang}"
    [[ -x "$cc_candidate" ]] || cc_candidate="$(command -v clang || true)"
    [[ -n "$cc_candidate" && -x "$cc_candidate" ]] || die "Matching clang not found for ${cxx_candidate}."

    CC="$cc_candidate"
    CXX="$cxx_candidate"
}

# Linux 绑定 GCC 工具链：Clang 使用 GCC 的 libstdc++（项目强制要求）
# 要求 GCC >= 13
select_linux_gcc_toolchain() {
    # GCC 优先级：g++-15 > g++-14 > g++-13 > g++
    local gcc_candidates=(
        g++-15
        g++-14
        g++-13
        g++
    )

    local gxx=""
    local gcc_major="0"

    for c in "${gcc_candidates[@]}"; do
        if command_exists "$c"; then
            gcc_major="$(get_gcc_major_version "$c")"
            if (( gcc_major >= 13 )); then
                gxx="$( command -v "$c")"
                break
            fi
        fi
    done

    [[ -n "$gxx" ]] || die "GCC/G++ 13+ toolchain required on Linux for a recent libstdc++."

    local gcc_root
    gcc_root="$(cd "$(dirname "$gxx")/.." && pwd)"

    info "Using GCC toolchain: $gxx"
    info "GCC version: $(get_first_line "$gxx" --version)"

    # 查找 libstdc++.so 路径
    local libstdcpp_path=""
    libstdcpp_path="$("$CXX" -print-file-name=libstdc++.so 2>/dev/null || true)"

    if [[ -z "$libstdcpp_path" || "$libstdcpp_path" == "libstdc++.so" ]]; then
        libstdcpp_path="$("$gxx" -print-file-name=libstdc++.so 2>/dev/null || true)"
    fi

    local libstdcpp_dir=""
    if [[ -n "$libstdcpp_path" && "$libstdcpp_path" != "libstdc++.so" ]]; then
        libstdcpp_dir="$(dirname "$libstdcpp_path")"
        info "libstdc++: $libstdcpp_path"
    else
        warn "Could not resolve libstdc++.so path directly; relying on GCC toolchain discovery."
    fi

    # 传递 CMake 参数：强制 Clang 使用 GCC 工具链 + libstdc++
    CMAKE_EXTRA_ARGS+=(
        "-DCMAKE_C_COMPILER=${CC}"
        "-DCMAKE_CXX_COMPILER=${CXX}"
        "-DCMAKE_C_FLAGS=--gcc-toolchain=${gcc_root}"
        "-DCMAKE_CXX_FLAGS=--gcc-toolchain=${gcc_root} -stdlib=libstdc++"
    )

    # 补充库搜索路径
    if [[ -n "$libstdcpp_dir" ]]; then
        export CMAKE_LIBRARY_PATH="${libstdcpp_dir}:${CMAKE_LIBRARY_PATH:-}"
    fi
}

# -----------------------------------------------------------------------------
# 平台整体配置入口
# -----------------------------------------------------------------------------

# macOS 完整配置
configure_macos() {
    info "Configuring for macOS..."
    select_macos_compiler
    BUILD_JOBS="$(num_cpus)"
    ok "macOS ready (Apple Clang + libc++)"
}

# Linux 完整配置
configure_linux() {
    info "Configuring for Linux..."
    select_linux_clang
    info "Compiler: $(get_first_line "$CXX" --version)"
    select_linux_gcc_toolchain
    BUILD_JOBS="$(num_cpus)"
    ok "Linux ready ($(basename "$CXX") + libstdc++ via GCC toolchain)"
}

# -----------------------------------------------------------------------------
# ASan 地址消毒器（内存泄漏、野指针、越界检测）运行时配置
# -----------------------------------------------------------------------------
configure_asan_runtime() {
    [[ "$ENABLE_ASAN" == "ON" ]] || return 0

    # 查找 llvm-symbolizer：将内存错误地址翻译为源码行号
    local symbolizer
    symbolizer="$(which llvm-symbolizer 2>/dev/null || which llvm-symbolizer-21 2>/dev/null || which llvm-symbolizer-22 2>/dev/null || true)"
    if [[ -n "$symbolizer" ]]; then
        export ASAN_SYMBOLIZER_PATH="$symbolizer"
        info "ASan symbolizer: ${symbolizer}"
    else
        warn "llvm-symbolizer not found - ASan reports will lack source-level symbolization"
    fi

    # Linux 开启内存泄漏检测，macOS 不支持 LSan，关闭
    local detect_leaks_val
    if [[ "$OS" == "linux" ]]; then
        detect_leaks_val="1"
    else
        detect_leaks_val="0"
    fi

    # ASan 运行时参数优化
    export ASAN_OPTIONS="symbolize=1:fast_unwind_on_malloc=0:detect_leaks=${detect_leaks_val}:malloc_context_size=30:print_suppressions=0:verbosity=0"
    # LSan 内存泄漏检测参数
    export LSAN_OPTIONS="report_objects=1:print_suppressions=0"

    ok "ASan runtime configured"
}

# -----------------------------------------------------------------------------
# TSan 线程消毒器（线程竞态、死锁检测）运行时配置
# -----------------------------------------------------------------------------
configure_tsan_runtime() {
    [[ "$ENABLE_TSAN" == "ON" ]] || return 0

    # 同样需要符号解析工具
    local symbolizer
    symbolizer="$(which llvm-symbolizer 2>/dev/null || which llvm-symbolizer-21 2>/dev/null || which llvm-symbolizer-22 2>/dev/null || true)"
    if [[ -n "$symbolizer" ]]; then
        export TSAN_SYMBOLIZER_PATH="$symbolizer"
        info "TSan symbolizer: ${symbolizer}"
    else
        warn "llvm-symbolizer not found - TSan reports will lack source-level symbolization"
    fi

    # TSan 运行时参数：加深调用栈、显示双方死锁栈、出错不立即退出
    export TSAN_OPTIONS="history_size=7:second_deadlock_stack=1:halt_on_error=0:verbosity=0"

    ok "TSan runtime configured"
}

# -----------------------------------------------------------------------------
# CMake 配置、编译、安装、运行核心流程
# -----------------------------------------------------------------------------

# CMake 配置：生成构建目录、缓存配置，避免重复无用重配
configure_cmake() {
    CMAKE_CONFIGURE_ARGS=(
        -DCMAKE_COLOR_DIAGNOSTICS=ON       # 开启彩色编译日志
        -G "$GENERATOR"                    # 指定构建生成器 Ninja
        -B "$BUILD_DIR"                    # 输出目录
        -S .                               # 源码根目录
        "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}" # 编译类型 Debug/Release
        "-DENABLE_CALIBRATION=${ENABLE_CALIBRATION}"
        "-DENABLE_ASAN=${ENABLE_ASAN}"
        "-DENABLE_TSAN=${ENABLE_TSAN}"
        "-DTALOS_BUILD_TESTING=${ENABLE_TESTING}"
        "${CMAKE_EXTRA_ARGS[@]}"
    )

    # 配置缓存文件：记录上一次 CMake 参数，参数不变则跳过 reconfigure
    local stamp_file="${BUILD_DIR}/.cmake_configure_args"
    local current_args="${CMAKE_CONFIGURE_ARGS[*]}"

    if [[ -f "$stamp_file" ]]; then
        local cached_args
        cached_args="$(cat "$stamp_file")"
        if [[ "$current_args" == "$cached_args" ]]; then
            info "CMake cache is up-to-date. Skipping reconfigure."
            return 0
        fi
        info "Build configuration changed. Reconfiguring..."
    fi

    info "Configuring CMake..."
    cmake "${CMAKE_CONFIGURE_ARGS[@]}"
    echo "$current_args" > "$stamp_file"
}

# 执行编译
build_target() {
    # 设置编译并发数
    export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}"

    CMAKE_BUILD_ARGS=(
        --build "$BUILD_DIR"
    )

    # 非测试模式：只编译指定目标；测试模式编译全部
    if [[ "$ENABLE_TESTING" == "OFF" ]]; then
        CMAKE_BUILD_ARGS+=(--target "${BUILD_TARGET}")
    fi

    # 开启详细日志
    if [[ "$VERBOSE" == "ON" ]]; then
        CMAKE_BUILD_ARGS+=(--verbose)
    fi

    if [[ "$ENABLE_TESTING" == "ON" ]]; then
        info "Building all targets (${BUILD_TYPE})..."
    else
        info "Building ${BUILD_TARGET} (${BUILD_TYPE})..."
    fi
    cmake "${CMAKE_BUILD_ARGS[@]}"

    ok "Build complete!"
}

# 交叉编译后安装文件到 bin 目录
install_target() {
    [[ "$CROSS_COMPILE" == "ON" ]] || return 0

    info "Installing to bin/ (cross-compile)..."
    cmake --install "$BUILD_DIR" --prefix bin
    ok "Install complete!"
}

# 编译完成后运行可执行程序
run_target() {
    [[ "$RUN_AFTER_BUILD" == "ON" ]] || return 0

    # 提前加载 ASan/TSan 环境变量
    configure_asan_runtime
    configure_tsan_runtime

    local exe="./${BUILD_DIR}/bin/${BUILD_TARGET}"
    [[ -x "$exe" ]] || die "Built executable not found: $exe"

    info "Starting ${BUILD_TARGET}..."
    if [[ "${#RUN_ARGS[@]}" -gt 0 ]]; then
        info "App args: $(join_by ' ' "${RUN_ARGS[@]}")"
    fi

    # exec 替换当前进程为目标程序
    exec "$exe"
}

# 运行单元测试 ctest
run_tests() {
    [[ "$ENABLE_TESTING" == "ON" ]] || return 0

    configure_asan_runtime
    configure_tsan_runtime

    info "Running tests..."
    # 并行执行测试、失败时输出详情
    ctest --test-dir "$BUILD_DIR" --output-on-failure --parallel "${BUILD_JOBS}"
    ok "All tests passed!"
}

# -----------------------------------------------------------------------------
# 命令行参数解析
# -----------------------------------------------------------------------------
parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --release)
                BUILD_TYPE="Release"
                shift
                ;;
            --debug)
                BUILD_TYPE="Debug"
                shift
                ;;
            --asan|-fsanitize=address)
                ENABLE_ASAN="ON"
                shift
                ;;
            --tsan|-fsanitize=thread)
                ENABLE_TSAN="ON"
                shift
                ;;
            --test)
                ENABLE_TESTING="ON"
                RUN_AFTER_BUILD="OFF"
                shift
                ;;
            -c|--calib)
                # 编译标定程序
                BUILD_TARGET="rm_calibration"
                ENABLE_CALIBRATION="ON"
                shift
                ;;
            -x|--ax|-ax|-cx|--cross)
                # 开启交叉编译（编译 aarch64 嵌入式板卡程序）
                CROSS_COMPILE="ON"
                CMAKE_EXTRA_ARGS+=(
                    "-DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/aarch64-linux-gnu-clang.cmake"
                    "-DTALOS_TARGET_TRIPLE=aarch64-unknown-linux-gnu"
                    "-DTALOS_LLVM_ROOT=$(brew --prefix llvm)"
                    "-DTALOS_GNU_TOOLCHAIN_ROOT=$(brew --prefix aarch64-unknown-linux-gnu)/toolchain"
                    "-DTALOS_SYSROOT=/Users/blackjack/sysroots/ax650n"
                    "-DBUILD_TESTING=OFF"
                    "-DCMAKE_CROSSCOMPILING=ON"
                )
                shift
                ;;
            -p|--playground|--play)
                # 编译演示/测试程序
                BUILD_TARGET="talos-playground"
                shift
                ;;
            --no-run)
                # 编译后不自动运行
                RUN_AFTER_BUILD="OFF"
                shift
                ;;
            --verbose)
                # 开启详细编译日志
                VERBOSE="ON"
                shift
                ;;
            --no-redact-build-output)
                REDACT_BUILD_OUTPUT="OFF"
                shift
                ;;
            --redact-build-output)
                REDACT_BUILD_OUTPUT="ON"
                shift
                ;;
            --)
                # -- 之后全部作为程序运行参数
                shift
                RUN_ARGS+=("$@")
                break
                ;;
            *)
                # 未知参数归入运行参数
                RUN_ARGS+=("$1")
                shift
                ;;
        esac
    done
}

# -----------------------------------------------------------------------------
# 脚本主入口
# -----------------------------------------------------------------------------
main() {
    # 解析启动参数
    parse_args "$@"

    echo ""
    status "🤖" "$BLUE" "Talos Build & Run"
    echo ""

    # 探测系统与架构
    OS="$(detect_os)"
    ARCH="$(detect_arch)"

    [[ "$OS"   != "unknown" ]] || die "Unsupported OS: $(uname -s)"
    [[ "$ARCH" != "unknown" ]] || die "Unsupported architecture: $(uname -m)"

    # 检查 Ninja
    ensure_generator

    # 交叉编译使用独立目录 build-cross
    if [[ "$CROSS_COMPILE" == "ON" ]]; then
        BUILD_DIR="build-cross"
    fi

    # 互斥判断：ASan 与 TSan 不能同时开启
    if [[ "$ENABLE_ASAN" == "ON" && "$ENABLE_TSAN" == "ON" ]]; then
        die "Cannot enable both ASan and TSan simultaneously. Choose one."
    fi

    # 打印当前配置概览
    info "Platform: ${OS} (${ARCH})"
    info "Build type: ${BUILD_TYPE}"
    info "Build dir: ${BUILD_DIR}"
    info "Cross-compile: ${CROSS_COMPILE}"
    info "ASAN: ${ENABLE_ASAN}"
    info "TSAN: ${ENABLE_TSAN}"
    info "Target: ${BUILD_TARGET}"
    info "Run after build: ${RUN_AFTER_BUILD}"
    info "Build log redaction: ${REDACT_BUILD_OUTPUT}"

    # 分平台初始化
    case "$OS" in
        macos) configure_macos ;;
        linux) configure_linux ;;
        *) die "Unsupported platform" ;;
    esac

    info "CC  = ${CC}"
    info "CXX = ${CXX}"
    info "Jobs = ${BUILD_JOBS}"

    # 完整执行流水线：配置 → 编译 → 安装(交叉编译) → 测试 → 运行
    configure_cmake
    build_target
    install_target
    run_tests
    run_target
}

# 启动主函数
main "$@"