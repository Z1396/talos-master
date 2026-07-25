#!/usr/bin/env bash
# 【脚本解释器声明】
# #!/usr/bin/env bash
# 1. 告诉系统：使用环境变量里找到的 bash 解释器执行本脚本
# 2. 跨平台兼容性优于直接写 #!/bin/bash，适配 bash 不在 /bin 下的系统

# =============================================================================
# Talos 项目 跨平台构建 & 运行 主脚本 build.sh
# 项目编译器硬性约束（业务规则）：
#   macOS  : Apple Clang 17+  + libc++ 标准库 | 支持任意CPU架构
#   Linux  : Clang 21+       + libstdc++ 标准库(依赖 GCC 13+ 工具链) | x86_64 / arm64
#
# 脚本通用用法：
#   ./build.sh [编译选项] [-- 程序运行参数]
# 示例：
#   ./build.sh --debug --asan          # Debug模式 + 开启内存检测
#   ./build.sh --release -- ./a b c    # Release编译，运行程序并传入参数 a b c
# =============================================================================

# -------------------------- 严格模式开启（Shell 脚本健壮性核心） --------------------------
# set 是 Shell 内置命令，用来修改 Shell 运行行为
set -Eeuo pipefail
# 逐个解释参数：
# -E：Shell 函数内部发生错误时，也会触发 ERR 陷阱（和 -e 配合使用）
# -e：任何命令执行返回非0状态码(失败)，脚本**立即退出**，防止错误继续扩散
# -u：使用「未定义的变量」时直接报错退出，避免变量为空引发诡异BUG
# -o pipefail：管道命令（cmd1 | cmd2 | cmd3）中，只要任意一段失败，整条管道返回失败码
# 作用：强制脚本严谨，提前暴露错误，是工业级 Shell 脚本标准配置

# -----------------------------------------------------------------------------
# 终端 ANSI 彩色输出码定义（终端颜色控制）
# ANSI 转义序列格式：\033[ 样式;颜色 m 文本 \033[0m
# \033 等价 ESC 按键，终端识别为控制符
# -----------------------------------------------------------------------------
YELLOW='\033[33m'   # 前景色：黄色（警告色）
GREEN='\033[32m'    # 前景色：绿色（成功色）
RED='\033[31m'      # 前景色：红色（错误色）
BLUE='\033[34m'     # 前景色：蓝色（普通信息色）
DIM='\033[2m'       # 文本暗淡效果
RESET='\033[0m'     # 重置所有颜色/样式为终端默认（必须加，否则后续文字一直带颜色）

# -----------------------------------------------------------------------------
# 通用日志基础函数：统一封装彩色输出逻辑
# 入参：
#   $1 = 日志前缀符号
#   $2 = 颜色变量
#   $3 = 日志正文内容
# local：声明**局部变量**，仅当前函数内有效，不污染全局变量
# echo -e：-e 启用「转义字符解析」（才能识别 \033 颜色码）
# -----------------------------------------------------------------------------
status() {
    local symbol="$1"
    local color="$2"
    local msg="$3"
    echo -e "${color}${symbol} ${msg}${RESET}"
}

# 普通信息日志（蓝色）
info() {
    # 调用基础日志函数，固定前缀、颜色
    status "[i]" "$BLUE" "$1"
}

# 成功日志（绿色）
ok() {
    status "[o]" "$GREEN" "$1"
}

# 警告日志（黄色）
warn() {
    status "[!]" "$YELLOW" "$1"
}

# 错误日志 + 直接退出脚本（红色）
# exit 1：脚本退出，返回状态码 1（标识执行失败）
die() {
    status "[x]" "$RED" "$1"
    exit 1
}

# -----------------------------------------------------------------------------
# 全局默认变量区：脚本运行初始状态、编译配置、路径、开关
# 所有变量为全局变量，整个脚本所有函数均可读取/修改
# -----------------------------------------------------------------------------
BUILD_TYPE="Release"               # 编译模式：Release(发布版)/Debug(调试版)，默认Release
BUILD_TARGET="talos"               # 默认编译目标产物名（可执行文件）
ENABLE_CALIBRATION="OFF"           # 标定模块开关：ON/OFF，默认关闭
ENABLE_ASAN="OFF"                  # ASAN 地址消毒器(内存泄漏/野指针/越界检测)开关
ENABLE_TSAN="OFF"                  # TSAN 线程消毒器(线程竞态/死锁检测)开关
ENABLE_TESTING="OFF"               # 单元测试编译&运行开关
RUN_AFTER_BUILD="ON"               # 编译完成后是否自动运行程序
VERBOSE="OFF"                      # CMake 详细编译日志开关
REDACT_BUILD_OUTPUT="ON"           # 编译日志脱敏开关
CROSS_COMPILE="OFF"                # 交叉编译开关（编译ARM嵌入式程序）
GENERATOR="Ninja"                  # CMake 构建生成器：固定使用 Ninja（高速构建工具）
BUILD_DIR="build"                  # 本地编译产物输出目录
RUN_ARGS=()                        # 数组：存储**程序运行时**的启动参数

# 运行时自动探测变量（脚本执行中动态赋值）
OS=""                              # 系统类型：macos / linux
ARCH=""                            # CPU架构：x86_64 / arm64(aarch64)
BUILD_JOBS=""                      # 编译并发线程数（等于CPU核心数）
CC=""                              # C 编译器完整路径（全局）
CXX=""                             # C++ 编译器完整路径（全局）
CMAKE_EXTRA_ARGS=()                # 数组：CMake 全局附加参数
CMAKE_CONFIGURE_ARGS=()            # 数组：CMake 配置阶段参数
CMAKE_BUILD_ARGS=()                # 数组：CMake 编译阶段参数
CMAKE_ENV_VARS=()                  # 数组：CMake 环境变量

# -----------------------------------------------------------------------------
# 工具函数库：通用底层工具，被上层逻辑反复调用
# -----------------------------------------------------------------------------

# 【工具函数】判断一个命令/程序是否在系统 PATH 中可用
# 语法：command_exists 命令名
# command -v 命令：查找命令所在路径
# >/dev/null 2>&1：丢弃标准输出+标准错误，只保留执行状态码
# 函数无显式 return，Shell 以「最后一条命令的状态码」作为返回值
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# 【工具函数】探测当前操作系统
# uname -s：Shell 内置命令，输出系统内核名称
# case ... esac：Shell 多分支条件判断，等价其他语言 switch-case
detect_os() {
    local os
    os="$(uname -s)"  # $() 命令替换：执行命令，将输出赋值给变量
    case "$os" in
        Darwin) echo "macos" ;;  # macOS 系统 uname -s 固定输出 Darwin
        Linux)  echo "linux" ;; # Linux 系统输出 Linux
        *)      echo "unknown" ;; # * 通配符：匹配所有其他未知系统
    esac
}

# 【工具函数】探测当前CPU架构
# uname -m：输出机器硬件架构
detect_arch() {
    local arch
    arch="$(uname -m)"
    case "$arch" in
        x86_64)         echo "x86_64" ;; # 传统64位PC架构
        aarch64|arm64)  echo "arm64" ;;  # ARM64 架构（嵌入式/苹果芯片）
        *)              echo "unknown" ;;
    esac
}

# 【工具函数】执行传入命令，只取输出第一行，屏蔽所有错误
# "$@"：代表传入函数的**所有参数**，原样拼接为一条命令执行
# | head -n1：管道，将前序命令输出交给 head，-n1 只取第一行
# || true：前序命令执行失败时，强制返回成功状态码，防止脚本中断
get_first_line() {
    "$@" 2>/dev/null | head -n1 || true
}

# 【工具函数】提取 Clang 主版本号（兼容 Apple Clang / 官方 Clang）
# Bash 专属正则匹配语法：[[ 字符串 =~ 正则 ]]
# [[:space:]]：POSIX 标准空白符（空格/制表符，兼容性优于直接写空格）
# ([0-9]+)：捕获分组，匹配1个及以上数字，结果存入 ${BASH_REMATCH} 数组
# ${BASH_REMATCH[1]}：取出第一个捕获分组内容（即主版本号）
get_clang_major_version() {
    local version_str
    # 执行 编译器 --version，取第一行版本信息
    version_str="$(get_first_line "$1" --version)"

    # 匹配 macOS Apple Clang 格式：Apple clang version 17.0.0
    if [[ "$version_str" =~ Apple[[:space:]]clang[[:space:]]version[[:space:]]([0-9]+) ]]; then
        echo "${BASH_REMATCH[1]}"
    # 匹配 Linux 标准 Clang 格式：clang version 21.0.0
    elif [[ "$version_str" =~ clang[[:space:]]version[[:space:]]([0-9]+) ]]; then
        echo "${BASH_REMATCH[1]}"
    # 匹配失败，返回 0 作为错误标记
    else
        echo "0"
    fi
}

# 【工具函数】判断当前 Clang 是否为 macOS Xcode 自带的 Apple Clang
# [[ 字符串 == 通配符 ]]：Bash 字符串通配匹配
# Apple\ clang*：以 Apple clang 开头的字符串（\ 转义空格）
is_apple_clang() {
    local version_str
    version_str="$(get_first_line "$1" --version)"
    [[ "$version_str" == Apple\ clang* ]]
}

# 【工具函数】提取 GCC 主版本号
# 正则：([0-9]+)\.[0-9]+\.[0-9]+ 匹配 主版本.次版本.修订版本
# 只捕获最前面的主版本数字
get_gcc_major_version() {
    local version_str
    version_str="$(get_first_line "$1" --version)"
    if [[ "$version_str" =~ ([0-9]+)\.[0-9]+\.[0-9]+ ]]; then
        echo "${BASH_REMATCH[1]}"
    else
        echo "0"
    fi
}

# 【工具函数】获取CPU核心数，作为编译并发线程数
num_cpus() {
    case "$OS" in
        macos) sysctl -n hw.ncpu ;;  # macOS 使用 sysctl 查询CPU核心
        linux) nproc ;;              # Linux 专用 nproc 命令查询核心数
        *) echo 4 ;;                 # 未知系统默认4线程编译
    esac
}

# 【工具函数】数组拼接为单个字符串
# 用法：join_by "分隔符" 数组元素...
# IFS：Shell 内部字段分隔符，临时修改为传入的分隔符
join_by() {
    local IFS="$1"
    shift          # 移除第一个参数（分隔符），剩余全为数组内容
    echo "$*"      # $*：将剩余参数用当前 IFS 拼接输出
}

# 【工具函数】检查 Ninja 构建工具是否存在（项目强制依赖 Ninja）
ensure_generator() {
    if ! command_exists ninja; then
        # ! 取反：命令不存在则报错退出
        die "Ninja not found. Please install ninja."
    fi
}

# -----------------------------------------------------------------------------
# 编译器选择逻辑：分平台自动筛选符合版本要求的编译器
# -----------------------------------------------------------------------------

# macOS 平台编译器选择：优先 Xcode 自带 Apple Clang（项目强制要求）
select_macos_compiler() {
    local clangxx
    local clangc

    # xcrun：macOS Xcode 工具，专门查找Xcode内部编译器/工具路径
    if command_exists xcrun; then
        clangxx="$(xcrun --find clang++ 2>/dev/null || true)"
        clangc="$(xcrun --find clang 2>/dev/null || true)"
    else
        # 兜底：从系统环境变量 PATH 查找 clang++ / clang
        clangxx="$(command -v clang++ || true)"
        clangc="$(command -v clang || true)"
    fi

    # [[ -x 文件路径 ]]：判断文件存在 且 拥有可执行权限
    [[ -n "$clangxx" && -x "$clangxx" ]] || die "clang++ not found. Install Xcode Command Line Tools."
    [[ -n "$clangc"  && -x "$clangc"  ]] || die "clang not found. Install Xcode Command Line Tools."

    # 提取 Clang 主版本号
    local major
    major="$(get_clang_major_version "$clangxx")"

    # 非 Apple Clang 给出警告（项目官方只支持 Apple Clang）
    if ! is_apple_clang "$clangxx"; then
        warn "Detected non-Apple clang on macOS: $(get_first_line "$clangxx" --version)"
        warn "This script officially supports Apple Clang on macOS."
    fi

    # (( 表达式 ))：Bash 整数算术判断，比较版本号大小
    (( major >= 17 )) || die "Apple Clang 17+ required on macOS. Found: $(get_first_line "$clangxx" --version)"

    # 赋值全局编译器变量
    CC="$clangc"
    CXX="$clangxx"

    # 数组追加 +=()：向 CMake 参数数组添加编译器配置
    CMAKE_EXTRA_ARGS+=(
        "-DCMAKE_C_COMPILER=${CC}"
        "-DCMAKE_CXX_COMPILER=${CXX}"
    )
}

# Linux 平台选择 Clang 编译器：要求 Clang >= 21
# 优先级：clang++-22 > clang++-21 > clang++
select_linux_clang() {
    # 定义候选编译器数组
    local candidates=(
        clang++-22
        clang++-21
        clang++
    )

    local cxx_candidate=""
    local cc_candidate=""
    local major="0"

    # for 遍历数组：${candidates[@]} 展开数组所有元素
    for c in "${candidates[@]}"; do
        if command_exists "$c"; then
            major="$(get_clang_major_version "$c")"
            # 版本≥21 才符合要求
            if (( major >= 21 )); then
                cxx_candidate="$(command -v "$c")"
                break  # break：跳出 for 循环，只取第一个符合条件的编译器
            fi
        fi
    done

    # 遍历完无符合条件编译器，报错退出
    [[ -n "$cxx_candidate" ]] || die "clang++ 21+ not found. Please install clang-21 or newer."

    # 字符串替换 ${变量/旧串/新串}：把 clang++ 替换为 clang，快速得到对应C编译器
    cc_candidate="${cxx_candidate/clang++/clang}"
    # 替换后的路径不可执行，则全局重新查找 clang
    [[ -x "$cc_candidate" ]] || cc_candidate="$(command -v clang || true)"
    # 二次强校验：必须存在可执行的 clang
    [[ -n "$cc_candidate" && -x "$cc_candidate" ]] || die "Matching clang not found for ${cxx_candidate}."

    # 赋值全局编译器变量
    CC="$cc_candidate"
    CXX="$cxx_candidate"
}

# Linux 平台绑定 GCC 工具链：Clang 借用 GCC 的头文件 + libstdc++ 标准库
# 项目强制要求：Clang 编译，但使用 GCC 13+ 的标准库
select_linux_gcc_toolchain() {
    # GCC 候选优先级数组
    local gcc_candidates=(
        g++-15
        g++-14
        g++-13
        g++
    )

    local gxx=""
    local gcc_major="0"

    # 遍历筛选 GCC >= 13
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

    # 计算 GCC 工具链根目录
    # dirname 路径：获取文件所在目录
    # /.. 进入上一级目录
    # cd 路径 && pwd：切换目录并输出绝对路径
    local gcc_root
    gcc_root="$(cd "$(dirname "$gxx")/.." && pwd)"

    info "Using GCC toolchain: $gxx"
    info "GCC version: $(get_first_line "$gxx" --version)"

    # 编译器内置参数 -print-file-name=xxx：查询库文件真实路径
    local libstdcpp_path=""
    libstdcpp_path="$("$CXX" -print-file-name=libstdc++.so 2>/dev/null || true)"

    # 查询失败则改用 g++ 重新查询
    if [[ -z "$libstdcpp_path" || "$libstdcpp_path" == "libstdc++.so" ]]; then
        libstdcpp_path="$("$gxx" -print-file-name=libstdc++.so 2>/dev/null || true)"
    fi

    # 提取标准库所在目录
    local libstdcpp_dir=""
    if [[ -n "$libstdcpp_path" && "$libstdcpp_path" != "libstdc++.so" ]]; then
        libstdcpp_dir="$(dirname "$libstdcpp_path")"
        info "libstdc++: $libstdcpp_path"
    else
        warn "Could not resolve libstdc++.so path directly; relying on GCC toolchain discovery."
    fi

    # 追加 CMake 参数：让 Clang 绑定 GCC 工具链、使用 libstdc++
    CMAKE_EXTRA_ARGS+=(
        "-DCMAKE_C_COMPILER=${CC}"
        "-DCMAKE_CXX_COMPILER=${CXX}"
        "-DCMAKE_C_FLAGS=--gcc-toolchain=${gcc_root}"
        "-DCMAKE_CXX_FLAGS=--gcc-toolchain=${gcc_root} -stdlib=libstdc++"
    )

    # 将库目录加入 CMake 库搜索路径
    # ${CMAKE_LIBRARY_PATH:-}：变量不存在则取空字符串，避免报错
    if [[ -n "$libstdcpp_dir" ]]; then
        export CMAKE_LIBRARY_PATH="${libstdcpp_dir}:${CMAKE_LIBRARY_PATH:-}"
    fi
}

# -----------------------------------------------------------------------------
# 平台整体配置入口
# -----------------------------------------------------------------------------

# macOS 完整平台配置
configure_macos() {
    info "Configuring for macOS..."
    select_macos_compiler
    BUILD_JOBS="$(num_cpus)"
    ok "macOS ready (Apple Clang + libc++)"
}

# Linux 完整平台配置
configure_linux() {
    info "Configuring for Linux..."
    select_linux_clang
    info "Compiler: $(get_first_line "$CXX" --version)"
    select_linux_gcc_toolchain
    BUILD_JOBS="$(num_cpus)"
    ok "Linux ready ($(basename "$CXX") + libstdc++ via GCC toolchain)"
}

# -----------------------------------------------------------------------------
# ASan 地址消毒器 运行时环境配置（内存检测）
# -----------------------------------------------------------------------------
configure_asan_runtime() {
    # 开关关闭则直接返回，不执行后续逻辑
    [[ "$ENABLE_ASAN" == "ON" ]] || return 0

    # 查找 llvm-symbolizer：将内存错误地址翻译成源码行号（符号解析工具）
    local symbolizer
    symbolizer="$(which llvm-symbolizer 2>/dev/null || which llvm-symbolizer-21 2>/dev/null || which llvm-symbolizer-22 2>/dev/null || true)"
    if [[ -n "$symbolizer" ]]; then
        export ASAN_SYMBOLIZER_PATH="$symbolizer"  # export 导出为环境变量
        info "ASan symbolizer: ${symbolizer}"
    else
        warn "llvm-symbolizer not found - ASan reports will lack source-level symbolization"
    fi

    # 区分系统：macOS 不支持内存泄漏检测 LSan
    local detect_leaks_val
    if [[ "$OS" == "linux" ]]; then
        detect_leaks_val="1"
    else
        detect_leaks_val="0"
    fi

    # 导出 ASan 运行时参数（环境变量）
    export ASAN_OPTIONS="symbolize=1:fast_unwind_on_malloc=0:detect_leaks=${detect_leaks_val}:malloc_context_size=30:print_suppressions=0:verbosity=0"
    # LSan 内存泄漏检测参数
    export LSAN_OPTIONS="report_objects=1:print_suppressions=0"

    ok "ASan runtime configured"
}

# -----------------------------------------------------------------------------
# TSan 线程消毒器 运行时环境配置（线程竞态/死锁检测）
# -----------------------------------------------------------------------------
configure_tsan_runtime() {
    [[ "$ENABLE_TSAN" == "ON" ]] || return 0

    # 同样依赖符号解析工具
    local symbolizer
    symbolizer="$(which llvm-symbolizer 2>/dev/null || which llvm-symbolizer-21 2>/dev/null || which llvm-symbolizer-22 2>/dev/null || true)"
    if [[ -n "$symbolizer" ]]; then
        export TSAN_SYMBOLIZER_PATH="$symbolizer"
        info "TSan symbolizer: ${symbolizer}"
    else
        warn "llvm-symbolizer not found - TSan reports will lack source-level symbolization"
    fi

    # TSan 运行时优化参数
    export TSAN_OPTIONS="history_size=7:second_deadlock_stack=1:halt_on_error=0:verbosity=0"

    ok "TSan runtime configured"
}

# -----------------------------------------------------------------------------
# CMake 核心流程：配置 → 编译 → 安装 → 测试 → 运行
# -----------------------------------------------------------------------------

# CMake 配置：生成构建目录、缓存参数，避免重复配置
configure_cmake() {
    # 组装 CMake 配置参数数组
    CMAKE_CONFIGURE_ARGS=(
        -DCMAKE_COLOR_DIAGNOSTICS=ON       # 开启 CMake 彩色编译日志
        -G "$GENERATOR"                    # -G 指定构建生成器（Ninja）
        -B "$BUILD_DIR"                    # -B 指定构建输出目录
        -S .                               # -S 指定源码根目录（当前目录）
        "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}" # 编译类型 Debug/Release
        "-DENABLE_CALIBRATION=${ENABLE_CALIBRATION}"
        "-DENABLE_ASAN=${ENABLE_ASAN}"
        "-DENABLE_TSAN=${ENABLE_TSAN}"
        "-DTALOS_BUILD_TESTING=${ENABLE_TESTING}"
        "${CMAKE_EXTRA_ARGS[@]}"           # 追加全局CMake参数
    )

    # 配置缓存文件：记录上一次CMake参数，参数不变则跳过重配
    local stamp_file="${BUILD_DIR}/.cmake_configure_args"
    local current_args="${CMAKE_CONFIGURE_ARGS[*]}"

    # 缓存文件存在，对比新旧参数
    if [[ -f "$stamp_file" ]]; then
        local cached_args
        cached_args="$(cat "$stamp_file")"  # cat 读取文件内容
        if [[ "$current_args" == "$cached_args" ]]; then
            info "CMake cache is up-to-date. Skipping reconfigure."
            return 0
        fi
        info "Build configuration changed. Reconfiguring..."
    fi

    info "Configuring CMake..."
    cmake "${CMAKE_CONFIGURE_ARGS[@]}"
    echo "$current_args" > "$stamp_file"  # > 重定向：将内容写入缓存文件
}

# 执行编译构建
build_target() {
    # 设置 CMake 编译并发线程数（环境变量）
    export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}"

    CMAKE_BUILD_ARGS=(
        --build "$BUILD_DIR"  # cmake --build 指定构建目录
    )

    # 非测试模式：只编译指定目标；测试模式编译全部目标
    if [[ "$ENABLE_TESTING" == "OFF" ]]; then
        CMAKE_BUILD_ARGS+=(--target "${BUILD_TARGET}")
    fi

    # 开启详细编译日志
    if [[ "$VERBOSE" == "ON" ]]; then
        CMAKE_BUILD_ARGS+=(--verbose)
    fi

    # 执行编译命令
    if [[ "$ENABLE_TESTING" == "ON" ]]; then
        info "Building all targets (${BUILD_TYPE})..."
    else
        info "Building ${BUILD_TARGET} (${BUILD_TYPE})..."
    fi
    cmake "${CMAKE_BUILD_ARGS[@]}"

    ok "Build complete!"
}

# 交叉编译专用：将产物安装到 bin 目录
install_target() {
    # 非交叉编译直接返回
    [[ "$CROSS_COMPILE" == "ON" ]] || return 0

    info "Installing to bin/ (cross-compile)..."
    cmake --install "$BUILD_DIR" --prefix bin
    ok "Install complete!"
}

# 编译完成后自动运行可执行程序
run_target() {
    [[ "$RUN_AFTER_BUILD" == "ON" ]] || return 0

    # 提前加载 ASan/TSan 环境变量
    configure_asan_runtime
    configure_tsan_runtime

    # 拼接可执行文件路径
    local exe="./${BUILD_DIR}/bin/${BUILD_TARGET}"
    [[ -x "$exe" ]] || die "Built executable not found: $exe"

    info "Starting ${BUILD_TARGET}..."
    # 输出程序启动参数
    if [[ "${#RUN_ARGS[@]}" -gt 0 ]]; then
        info "App args: $(join_by ' ' "${RUN_ARGS[@]}")"
    fi

    # exec：用目标程序替换当前Shell进程，脚本不再继续执行
    exec "$exe"
}

# 运行单元测试 ctest
run_tests() {
    [[ "$ENABLE_TESTING" == "ON" ]] || return 0

    configure_asan_runtime
    configure_tsan_runtime

    info "Running tests..."
    # ctest：CMake 自带测试工具，并行执行、失败输出详情
    ctest --test-dir "$BUILD_DIR" --output-on-failure --parallel "${BUILD_JOBS}"
    ok "All tests passed!"
}

# -----------------------------------------------------------------------------
# 命令行参数解析：解析 ./build.sh 后跟的启动参数
# while [[ $# -gt 0 ]]：$# 代表当前参数个数，大于0则循环解析
# shift：参数左移，丢弃第一个参数，处理下一个
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
                BUILD_TARGET="rm_calibration"
                ENABLE_CALIBRATION="ON"
                shift
                ;;
            -x|--ax|-ax|-cx|--cross)
                # 开启交叉编译，注入交叉编译工具链文件、系统根目录等CMake参数
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
                BUILD_TARGET="talos-playground"
                shift
                ;;
            --no-run)
                RUN_AFTER_BUILD="OFF"
                shift
                ;;
            --verbose)
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
                # 特殊标记 -- ：后面**所有内容**全部作为程序运行参数，不再解析编译参数
                shift
                RUN_ARGS+=("$@")
                break
                ;;
            *)
                # 无法识别的参数，统一归入程序运行参数
                RUN_ARGS+=("$1")
                shift
                ;;
        esac
    done
}

# -----------------------------------------------------------------------------
# 脚本主入口函数（程序执行起点）
# -----------------------------------------------------------------------------
main() {
    # 第一步：解析命令行传入的所有参数
    # parse_args：自定义函数名（bash 脚本里你自己定义的参数解析函数，不是系统内置命令！）
    # "$@"：Shell 内置特殊变量
    # 重点： "$@" 是什么
    # $@ = 脚本接收到的全部命令行参数
    # 加双引号 "$@"：保留参数内部空格，参数独立拆分（标准正确写法）
    # 举例子：
    # 运行脚本
    # bash
    # 运行
    # ./build.sh --mode release --name "my robot"
    # 此时：
    # "$@" 等价于 "--mode" "release" "--name" "my robot"
    # ❌ 坑：不要写成 $@ 不带引号，带空格参数会被错误切分；
    # ❌ 不要写成 "$*"，所有参数合并成单个字符串，无法正常循环解析。
    parse_args "$@"

    echo ""
    status "🤖" "$BLUE" "Talos Build & Run"
    echo ""

    # 探测系统与架构
    OS="$(detect_os)"
    ARCH="$(detect_arch)"

    # 校验系统/架构合法性
    [[ "$OS"   != "unknown" ]] || die "Unsupported OS: $(uname -s)"
    [[ "$ARCH" != "unknown" ]] || die "Unsupported architecture: $(uname -m)"

    # 强制检查 Ninja 工具
    ensure_generator

    # 交叉编译使用独立编译目录 build-cross
    if [[ "$CROSS_COMPILE" == "ON" ]]; then
        BUILD_DIR="build-cross"
    fi

    # 互斥校验：ASan 和 TSan 不能同时开启（两者冲突）
    if [[ "$ENABLE_ASAN" == "ON" && "$ENABLE_TSAN" == "ON" ]]; then
        die "Cannot enable both ASan and TSan simultaneously. Choose one."
    fi

    # 打印当前全局配置概览
    info "Platform: ${OS} (${ARCH})"
    info "Build type: ${BUILD_TYPE}"
    info "Build dir: ${BUILD_DIR}"
    info "Cross-compile: ${CROSS_COMPILE}"
    info "ASAN: ${ENABLE_ASAN}"
    info "TSAN: ${ENABLE_TSAN}"
    info "Target: ${BUILD_TARGET}"
    info "Run after build: ${RUN_AFTER_BUILD}"
    info "Build log redaction: ${REDACT_BUILD_OUTPUT}"

    # 根据系统分支执行平台配置
    case "$OS" in
        macos) configure_macos ;;
        linux) configure_linux ;;
        *) die "Unsupported platform" ;;
    esac

    # 打印最终选中的编译器
    info "CC  = ${CC}"
    info "CXX = ${CXX}"
    info "Jobs = ${BUILD_JOBS}"

    # 标准流水线：配置 → 编译 → 安装(交叉编译) → 测试 → 运行
    configure_cmake
    build_target
    install_target
    run_tests
    run_target
}

# 调用主函数，将脚本所有启动参数传递给 main
main "$@"