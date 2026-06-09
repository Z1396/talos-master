# Talos 项目部署操作文档

> 本文档记录 Talos 上位机在 Ubuntu 22.04 (x86_64) 上的完整部署流程，包含所有踩坑记录与解决方案。
>
> 适用版本：Talos master 分支，CMake 3.22，Clang 21.1.8，GCC 13.4.0
>
> 部署日期：2026-06-08

---

## 目录

1. [环境准备](#1-环境准备)
2. [基础依赖安装](#2-基础依赖安装)
3. [三方库手动下载](#3-三方库手动下载)
4. [代码修改记录](#4-代码修改记录)
5. [项目编译](#5-项目编译)
6. [程序运行](#6-程序运行)
7. [常见问题处理](#7-常见问题处理)
8. [关键操作点与注意事项](#8-关键操作点与注意事项)

---

## 1. 环境准备

### 1.1 硬件与系统要求

| 项目 | 最低要求 | 推荐 |
|------|----------|------|
| 系统 | Ubuntu 22.04 LTS (x86_64) | 同左 |
| 磁盘 | 10 GB 可用空间 | 20 GB+ |
| 内存 | 4 GB | 8 GB+ |
| 编译器 | Clang 21+ 或 GCC 13+ | Clang 21.1.8 |
| 网络 | 可访问 GitHub / 镜像源（部分依赖需下载） | 稳定网络 |

### 1.2 系统初始化

```bash
# 更新软件源
sudo apt update
```

> ⚠️ **关键点**：本机用户名 `pldx`，所有路径基于 `/home/pldx/Desktop/talos-master`。

---

## 2. 基础依赖安装

### 2.1 安装系统编译器与基础工具

```bash
# 安装 CMake、Ninja、基础 C++ 工具链
sudo apt install -y cmake ninja-build libomp-dev \
                    libeigen3-dev libopencv-dev \
                    libspdlog-dev libfmt-dev nlohmann-json3-dev
```

**已安装版本（本环境实测）**：
- libfmt 8.1.1（**版本过旧**，需手动升级）
- libspdlog 1.9.2（**版本过旧**，需手动升级）
- nlohmann-json 3.10.5
- OpenCV 4.5.5

### 2.2 安装 GCC 13 工具链

> **踩坑记录**：`./build.sh` 要求 GCC 13+，但 Ubuntu 22.04 默认源不提供 g++-13。必须添加 Toolchain PPA。

```bash
# 添加 ubuntu-toolchain-r/test PPA
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install -y g++-13
```

**验证**：
```bash
g++ --version
# 期望：g++ (Ubuntu 13.4.0-6ubuntu1~22~ppa2) 13.4.0
```

### 2.3 安装/验证 Clang 21

```bash
# Clang 21 通过 LLVM 源安装（apt.llvm.org）
# 本机已安装 /usr/bin/clang-21
clang-21 --version
# 期望：Ubuntu clang version 21.1.8
```

> ⚠️ **关键点**：Ubuntu 22.04 默认源中 `clang++` 软链接未建立，建议使用 `clang-21` + `clang++-21` 全名调用。`build.sh` 内部已自动选择 `clang-21`。

### 2.4 安装其他可选依赖

```bash
# TBB 线程库（OpenVINO 自带，本机已包含）
# Ceres 优化库
sudo apt install -y libceres-dev libgflags-dev libgoogle-glog-dev libsuitesparse-dev
# FFmpeg（用于图传）
sudo apt install -y libavcodec-dev libavformat-dev libavutil-dev libswscale-dev
```

---

## 3. 三方库手动下载

> **踩坑记录**：本机网络无法稳定访问 GitHub（SSL EOF、连接超时），必须手动下载所有三方库到 `3dparty/` 目录。

### 3.1 创建 3dparty 目录

```bash
cd /home/pldx/Desktop/talos-master
mkdir -p 3dparty
```

### 3.2 下载 ONNXRuntime 1.26.0

```bash
# 浏览器或代理下载（建议使用 gitee 镜像或 ghproxy）
# 原地址：https://github.com/microsoft/onnxruntime/releases/download/v1.26.0/onnxruntime-linux-x64-gpu-1.26.0.tgz
# 放到：
# /home/pldx/Desktop/talos-master/3dparty/onnxruntime-linux-x64-gpu-1.26.0.tgz

cd /home/pldx/Desktop/talos-master/3dparty
# 若有可用 curl，尝试：
# curl -L -o onnxruntime-linux-x64-gpu-1.26.0.tgz \
#   https://github.com/microsoft/onnxruntime/releases/download/v1.26.0/onnxruntime-linux-x64-gpu-1.26.0.tgz
```

### 3.3 下载 fmt 12.0.0

> **踩坑记录**：系统 libfmt 8.1.1 缺少 `fmt/std.h` 和 `fmt/base.h`；libfmt 10.x 与 Clang 21 的 consteval 实现冲突。必须使用 fmt 12.x。

```bash
cd /home/pldx/Desktop/talos-master/3dparty
curl -L -o fmt-12.0.0.zip https://github.com/fmtlib/fmt/archive/refs/tags/12.0.0.zip --max-time 60
```

**验证**：
```bash
ls -la fmt-12.0.0.zip
# 文件应 > 500KB
```

### 3.4 下载 spdlog 1.15.1

> **踩坑记录**：系统 libspdlog 1.9.2 不认识 fmt 12 引入的 `fmt::basic_runtime` 类型，会编译失败。

```bash
cd /home/pldx/Desktop/talos-master/3dparty
curl -L -o spdlog-1.15.1.zip https://github.com/gabime/spdlog/archive/refs/tags/v1.15.1.zip --max-time 60
```

### 3.5 下载 boost-pfr 1.90.0

> **踩坑记录**：boost::pfr 没有系统包，GitHub ZIP 下载在受限网络下超时。使用 git clone 成功（受限于 git 协议可达性）。

```bash
cd /home/pldx/Desktop/talos-master/3dparty
git clone --depth 1 --branch boost-1.90.0 https://github.com/boostorg/pfr.git boost-pfr
```

**验证**：
```bash
ls boost-pfr/include/boost/pfr.hpp
# 必须存在
```

### 3.6 跳过下载的库

以下库无需手动下载（本机已缓存或系统已安装）：

- **tomlplusplus**：本机 `3dparty/` 已有 `tomlplusplus-1c8b7466e4946fcc3bf20484c0e1d001202cca5a.zip`
- **Eigen3**：使用系统 `libeigen3-dev` 3.4.0
- **FFmpeg**：本机已有 `ffmpeg-master-latest-linux64-gpl-shared.tar.xz`
- **libusb**：本机已有 `libusb-1.0.30.zip`
- **HIK SDK**：本机已有 `3dparty/hik_sdk/`
- **foxglove**：本机已有 `foxglove-v0.23.0-cpp-x86_64-unknown-linux-gnu.zip`
- **magic_enum**：源码内置于 `3dparty/magic_enum/`

---

## 4. 代码修改记录

### 4.1 修改 `cmake/dependencies.cmake`

**目的**：让 CMake 使用本地/系统库，跳过受限的 GitHub 下载。

**修改位置**：`/home/pldx/Desktop/talos-master/cmake/dependencies.cmake`

#### 修改 A：fmt 改用本地 12.0.0

```cmake
# 替换第 115-119 行的 Eigen3+fmt 配置段
# 旧内容（参考）：
# fetch_dependency(NAME Eigen3
#                  REPO gitlab.com/libeigen/eigen
#                  VERSION 5.0.1)
# set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
# fetch_dependency(NAME fmt
#                  REPO fmtlib/fmt
#                  VERSION 12.1.0)
# set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
# set(SPDLOG_FMT_EXTERNAL ON CACHE BOOL "" FORCE)
# set(SPDLOG_ENABLE_PCH ON CACHE BOOL "" FORCE)
# set(SPDLOG_BUILD_SHARED ON CACHE BOOL "" FORCE)
# fetch_dependency(NAME spdlog
#                  REPO gabime/spdlog
#                  VERSION fb1227486b860c673c5cfdc49359707e30c6b5f8)
# fetch_dependency(NAME nlohmann_json
#                  REPO nlohmann/json
#                  VERSION 3946872265598aed5a7aea68cad4d9d1f168bd4b)
# set(_prev_build_testing "${BUILD_TESTING}")
# set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
# fetch_dependency(NAME boost_pfr
#                  REPO boostorg/pfr
#                  VERSION boost-1.90.0)
# set(BUILD_TESTING "${_prev_build_testing}" CACHE BOOL "" FORCE)
```

**替换为**：

```cmake
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
```

### 4.2 修改 `config/vision_base.toml`（可选）

> **踩坑记录**：若 ONNXRuntime 链接有问题，可临时切到 Traditional 后端。

```bash
# 若使用 Traditional 后端：
sed -i 's/backend_type = "OnnxRuntime"/backend_type = "Traditional"/' \
  /home/pldx/Desktop/talos-master/config/vision_base.toml
```

> 本机编译时已保留 `OnnxRuntime` 模式，编译通过；用户可按需切换。

---

## 5. 项目编译

### 5.1 清理旧的构建

```bash
cd /home/pldx/Desktop/talos-master
rm -rf build
```

### 5.2 执行编译

```bash
./build.sh
```

### 5.3 编译参数（可选）

| 参数 | 作用 |
|------|------|
| `./build.sh --debug` | Debug 模式 |
| `./build.sh --asan` | 启用 AddressSanitizer |
| `./build.sh --tsan` | 启用 ThreadSanitizer |
| `./build.sh --no-run` | 编译但不运行 |

### 5.4 编译成功标志

```
[o] Build complete!
[i] Starting talos...
[INFO] Foxglove WebSocket enabled on ws://0.0.0.0:8765
```

编译产物：`bin/talos`

---

## 6. 程序运行

### 6.1 默认运行

```bash
cd /home/pldx/Desktop/talos-master
./bin/talos
```

### 6.2 配置文件

主配置：`at_vision.toml`（项目根目录）

```toml
# 选择后端模式
backend = "Direct"       # 直接硬件模式
# backend = "Daedalus"   # 模拟器模式
# backend = "CameraOnly" # 仅相机
# backend = "Chiral"     # 双向通信

robot = "infantry_4"
vision = "daedalus_vision"
```

### 6.3 不接硬件启动

如本机无 STM32 单片机，程序会报：
```
critical setup l1: connect usb mcu: find usb device: no matching device found
```

这是**预期行为**，系统将退出。需接 USB MCU 或使用模拟器后端。

### 6.4 模拟器模式（推荐用于开发）

```bash
# 1. 修改 at_vision.toml
sed -i 's/backend = "Direct"/backend = "Daedalus"/' at_vision.toml

# 2. 启动 Rust 模拟器（需单独安装）

# 3. 启动 Talos
./bin/talos
```

### 6.5 Foxglove 可视化

程序启动后，浏览器访问：
```
http://localhost:8765
```

需安装 [Foxglove Studio](https://foxglove.dev/) 客户端连接 WebSocket。

---

## 7. 常见问题处理

### 7.1 `clang-21` 找不到

```
E: 软件包 clang-21 没有可安装候选
```

**解决**：使用 LLVM 官方源安装：
```bash
wget https://apt.llvm.org/llvm.sh -O /tmp/llvm.sh
sudo bash /tmp/llvm.sh 21
sudo apt install -y clang-21 libclang-21-dev
```

### 7.2 `g++-13` 找不到

**解决**：见 §2.2，添加 `ubuntu-toolchain-r/test` PPA。

### 7.3 `clang++` 命令找不到

**解决**：本机使用 `clang++-21` 全名，`build.sh` 内部已处理；不需要额外建立软链接。

### 7.4 GitHub 下载超时/SSL 错误

```
curl: (28) Operation timed out
OpenSSL SSL_read: error:0A000126
```

**解决**：使用 §3 的手动下载方案。优先使用 `git clone`（受 git 协议可达性影响小），否则用浏览器+手动复制到 `3dparty/`。

### 7.5 ONNXRuntime `SYSTEM` 标志报错

```
At least one entry of URL is a path (invalid in a list)
```

> **原因**：CMake 3.22 + Clang 21 下 `SYSTEM` 标志的 bug。

**解决**：本机已通过手动下载到 `3dparty/` + 修改 `dependencies.cmake` 解决。无需额外操作。

### 7.6 fmt 编译错误

```
error: call to consteval function ... is not a constant expression
```

> **原因**：fmt 10.2.x 与 Clang 21 存在 consteval 兼容性问题。

**解决**：使用 fmt 12.0.0（见 §3.3）。

### 7.7 spdlog 编译错误

```
error: no member named 'basic_runtime' in namespace 'fmt'
```

> **原因**：系统 libspdlog 1.9.2 不认识 fmt 12 新类型。

**解决**：使用 spdlog 1.15.1（见 §3.4）。

### 7.8 boost::pfr 找不到

```
fatal error: 'boost/pfr.hpp' file not found
```

**解决**：使用 §3.5 的 git clone 方式，或检查 `3dparty/boost-pfr/include/boost/pfr.hpp` 是否存在。

### 7.9 Eigen 5.0.1 `-Wcast-qual` 错误

```
error: cast from 'const unsigned int *' to 'int *' drops const qualifier
```

> **原因**：Eigen 5.0.1 在 Clang 21 + `-Werror=cast-qual` 下报错。

**解决**：使用系统 `libeigen3-dev` 3.4.0（见 §4.1 修改）。

### 7.10 USB MCU 找不到

```
no matching device found
```

**说明**：本机无 STM32 单片机，需：
- 接真实 MCU，或
- 改用 `Daedalus` 后端 + 模拟器

---

## 8. 关键操作点与注意事项

### 8.1 ⚠️ 必须执行

1. ✅ 添加 Toolchain PPA 安装 `g++-13`
2. ✅ 在 `3dparty/` 手动放置 `fmt-12.0.0.zip`、`spdlog-1.15.1.zip`、`onnxruntime-linux-x64-gpu-1.26.0.tgz`
3. ✅ `git clone` boost-pfr 到 `3dparty/boost-pfr/`
4. ✅ 修改 `cmake/dependencies.cmake`（§4.1）
5. ✅ 编译前清理 `build/` 目录

### 8.2 ⚠️ 注意事项

1. **网络受限环境**：所有 GitHub 下载都可能失败，**优先使用 git clone**，否则手动下载。
2. **fmt 与 spdlog 版本强相关**：必须同时使用新版（12.0.0 + 1.15.1），混搭会失败。
3. **Eigen 5.0.1 与 Clang 21 不兼容**：必须用系统 3.4.0。
4. **清理 build 目录**：每次修改 `dependencies.cmake` 后必须 `rm -rf build`。
5. **无硬件运行**：默认后端 `Direct` 必须接 STM32 MCU，否则程序启动后立即退出。
6. **Foxglove WebSocket**：默认端口 8765，确认防火墙开放。

### 8.3 🔄 完整部署检查清单

| 步骤 | 命令/操作 | 状态 |
|------|-----------|------|
| 1 | `apt update` | ☐ |
| 2 | 安装 g++-13（PPA） | ☐ |
| 3 | 安装 Clang 21 | ☐ |
| 4 | 安装基础依赖 | ☐ |
| 5 | 下载 ONNXRuntime | ☐ |
| 6 | 下载 fmt-12.0.0 | ☐ |
| 7 | 下载 spdlog-1.15.1 | ☐ |
| 8 | git clone boost-pfr | ☐ |
| 9 | 修改 `dependencies.cmake` | ☐ |
| 10 | `rm -rf build && ./build.sh` | ☐ |
| 11 | 验证 Foxglove 端口 | ☐ |
| 12 | 选择后端运行 | ☐ |

---

## 附录 A：版本信息表

| 组件 | 版本 | 来源 |
|------|------|------|
| Ubuntu | 22.04.5 LTS | 系统 |
| Linux Kernel | 6.x | 系统 |
| CMake | 3.22.1 | 系统 |
| Ninja | 1.10+ | 系统 |
| GCC | 13.4.0 | ubuntu-toolchain-r PPA |
| Clang | 21.1.8 | apt.llvm.org |
| OpenCV | 4.5.5 | 系统 /usr/local |
| fmt | 12.0.0 | 手动下载 |
| spdlog | 1.15.1 | 手动下载 |
| Eigen3 | 3.4.0 | 系统 libeigen3-dev |
| boost-pfr | 1.90.0 | git clone |
| ONNXRuntime | 1.26.0 (GPU) | 手动下载 |
| TBB | OpenVINO 2024.6 | /opt/intel/openvino_2024.6.0 |
| Ceres | 2.0.0 | 系统 |

## 附录 B：失败记录汇总

| 序号 | 错误 | 根因 | 解决 |
|------|------|------|------|
| 1 | clang-21 找不到 | 默认源不含 | 添加 PPA |
| 2 | g++-13 找不到 | 默认源不含 | 添加 ubuntu-toolchain-r PPA |
| 3 | GCC/G++ 13+ required | build.sh 检查 | 安装 g++-13 |
| 4 | ONNXRuntime 下载失败 | GitHub SSL | 手动下载 |
| 5 | ONNXRuntime SYSTEM 标志 | CMake 3.22 bug | 改用 ZIP_URL |
| 6 | fmt/std.h 找不到 | 系统 fmt 8.1 太旧 | 升级到 fmt 12.0.0 |
| 7 | fmt consteval 错误 | fmt 10 + Clang 21 | 升级到 fmt 12.0.0 |
| 8 | spdlog basic_runtime | 系统 spdlog 1.9 旧 | 升级到 spdlog 1.15.1 |
| 9 | boost::pfr 找不到 | 未下载 | git clone |
| 10 | boost::pfr stub 不足 | 用了空 stub | 改用真实头文件 |
| 11 | Eigen Wcast-qual | Eigen 5.0.1 + Clang 21 | 改用系统 Eigen 3.4.0 |
| 12 | USB MCU 找不到 | 缺硬件 | 改用 Daedalus 后端 |

---

## 附录 C：构建脚本逻辑说明

`./build.sh` 内部流程：

1. 检测平台（Linux x86_64）
2. 选择编译器：优先 `clang-21`，要求 GCC 13+ 提供 libstdc++
3. 验证 libstdc++ 路径：`/usr/lib/gcc/x86_64-linux-gnu/13/libstdc++.so`
4. 导出 `--gcc-toolchain=/usr` 给 Clang
5. CMake 配置：检测已缓存三方库
6. Ninja 编译（jobs=8）
7. 编译成功后自动运行 `./bin/talos`

可通过 `./build.sh --no-run` 跳过自动运行。

---

## 附录 D：目录结构

```
talos-master/
├── 3dparty/                          # 三方库（手动下载）
│   ├── onnxruntime-linux-x64-gpu-1.26.0.tgz
│   ├── fmt-12.0.0.zip
│   ├── spdlog-1.15.1.zip
│   ├── boost-pfr/                    # git clone
│   ├── tomlplusplus-*.zip
│   ├── ffmpeg-*.tar.xz
│   ├── libusb-*.zip
│   ├── hik_sdk/
│   ├── foxglove-*.zip
│   └── magic_enum/
├── cmake/
│   ├── dependencies.cmake            # ★ 已修改
│   ├── fetch-content-helper.cmake
│   ├── fetch-onnxruntime.cmake
│   └── ffmpeg.cmake
├── config/
│   ├── at_vision.toml                # 入口配置
│   ├── vision_base.toml
│   ├── vision/
│   └── robot/
├── src/
│   ├── main.cpp
│   ├── fcs/                          # 火控系统
│   └── fcs_visualization/            # Foxglove 桥
├── crates/                           # 核心库
│   ├── primitive/
│   ├── scheduler/
│   ├── fast_tf/
│   ├── math/
│   ├── toml/
│   ├── log/
│   ├── hardware/
│   ├── hardware_daedalus/
│   ├── chiral/
│   └── quanta/
├── bin/                              # 编译产物
│   └── talos
└── build/                            # 构建目录
```

---

## 附录 E：联系方式

部署过程中遇到新问题，可参考：
- 项目 README.md
- 项目 CLAUDE.md
- 项目 docs/ 目录

---

**文档结束**
