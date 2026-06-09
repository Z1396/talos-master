# macOS 交叉编译到 AX650N

目标板示例：

```text
Linux ax650-omni 5.15.73 aarch64 GNU/Linux
```

这类目标是 `Linux + aarch64 + glibc + libstdc++`。在 macOS 上不能直接复用 Homebrew 的
`OpenCV`、`TBB`、`Ceres` 等宿主机包；必须准备一套 **面向目标板的 Linux sysroot**，并让
CMake、pkg-config、链接器都只从那套 sysroot 里找头文件和库。

## 结论先说

1. `build.sh` 目前是原生构建脚本，不适合 macOS -> Linux 交叉编译。
2. 交叉编译必须提供：
   - `aarch64-linux-gnu` 交叉编译器
   - 从板子导出的 rootfs/sysroot
   - sysroot 内的目标版 `OpenCV`、`oneTBB`、`Ceres`、`libusb`、`FFmpeg`
   - AX SDK 对应的 `/soc` 目录
3. 宿主 macOS 上安装的 `brew install opencv tbb` 对这个目标没有帮助。

## 1. 准备交叉编译器

如果你本地主力编译器是 `clang21`，也没问题，但要注意：

- `clang21` 只是前端，不会替你提供 Linux 目标的 `glibc/libstdc++/crt`
- 你仍然需要一套 `aarch64-linux-gnu` 的 GNU cross toolchain
- 交叉构建时必须显式带上 `--target`、`--sysroot`、`--gcc-toolchain`

推荐准备两套东西：

1. LLVM/Clang 21
2. GNU aarch64 交叉工具链，负责 `libstdc++`、启动对象和 binutils

一种常见安装方式是通过 Homebrew 的 cross toolchain：

```bash
brew tap messense/macos-cross-toolchains
brew install aarch64-unknown-linux-gnu
```

安装完成后应能看到：

```bash
aarch64-unknown-linux-gnu-gcc --version
aarch64-unknown-linux-gnu-g++ --version
```

如果你的工具链前缀是 `aarch64-linux-gnu-`，下面命令里把 triple 改掉即可。

## 2. 从 AX650N 导出 sysroot

最稳妥的做法是直接从目标板拷根文件系统。至少要包含：

- `/lib`
- `/usr/include`
- `/usr/lib`
- `/usr/share/pkgconfig`
- `/usr/lib/pkgconfig`
- `/usr/lib/aarch64-linux-gnu/pkgconfig`（如果有）
- `/soc`

示例：

```bash
mkdir -p ~/sysroots/ax650n
rsync -a root@ax650.local:/lib ~/sysroots/ax650n/
rsync -a root@ax650.local:/usr ~/sysroots/ax650n/
rsync -a root@ax650.local:/soc ~/sysroots/ax650n/
```

如果板子上没有开发包，只拷到 `.so` 不够；你还需要把对应的头文件和 `.pc` 文件一起带出来。

拷完之后，建议立刻规整 sysroot 里的绝对 symlink：

```bash
./scripts/normalize_sysroot_symlinks.sh ~/sysroots/ax650n
```

很多目标系统里的 linker name 会长这样：

```text
usr/lib/aarch64-linux-gnu/libm.so -> /lib/aarch64-linux-gnu/libm.so.6
```

在目标机根目录下这是合法的，但在 macOS 本地作为离线 sysroot 使用时，
`lld`/`clang` 会把它当成宿主机绝对路径，导致动态库入口失效，随后错误地回退到 `libm.a`
这类静态归档，最终冒出类似：

```text
undefined symbol: __frexpl
```

把这些绝对 symlink 改成 sysroot 内部的相对 symlink 后，这类问题会一起消失。

## 3. 检查 ABI，别盲编

`uname -a` 只能说明内核和架构，不能说明用户态 ABI 是否匹配。至少再确认两件事：

```bash
ssh root@ax650.local 'ldd --version'
ssh root@ax650.local 'strings /usr/lib*/libstdc++.so.6 | grep GLIBCXX_ | tail'
```

你本地交叉工具链用的 `glibc/libstdc++` 版本，必须不高于板子实际提供的版本，否则程序会在目标板启动时报
`GLIBCXX_* not found` 或 `GLIBC_* not found`。

## 4. 目标依赖怎么准备

Talos 当前配置里会在目标侧查找这些库：

- `OpenCV`
- `TBB`
- `Ceres`
- `libusb-1.0`
- `FFmpeg`
- 可选：`TensorRT`
- 可选：AX SDK (`/soc`)

### OpenCV / TBB / Ceres / libusb

你有两个选择：

1. 直接从板子导出开发包
   前提是板子上本来就安装了这些库的头文件、`.so`、`.pc`、`*Config.cmake`。
2. 用同一套交叉工具链把这些依赖先编到 sysroot
   这是更可控的做法。

不要把 macOS 的 `.dylib` 或 Homebrew 头文件塞给这次构建，它们和 Linux 目标 ABI 不兼容。

### FFmpeg

项目默认在 Linux 上优先尝试拉取 BtbN 的 FFmpeg 共享库。交叉编译时更建议你直接把目标版 FFmpeg 放进
sysroot，并保证 pkg-config 能找到它；这样版本和 ABI 更可控。

## 5. AX SDK

AX 后端通过下面这些路径查找：

- include: `/soc/soc/include`
- libs: `/soc/lib`

如果你的 sysroot 是从板子根目录拷出来的，那么它们应分别位于：

- `${TALOS_SYSROOT}/soc/soc/include`
- `${TALOS_SYSROOT}/soc/lib`

仓库已有部署说明见 [docs/ax650.md](./ax650.md)。

## 6. 配置并编译 Talos

仓库里现在提供了两个工具链文件：

`cmake/toolchains/aarch64-linux-gnu.cmake`
`cmake/toolchains/aarch64-linux-gnu-clang.cmake`

先导出 sysroot：

```bash
export TALOS_SYSROOT="$HOME/sysroots/ax650n"
```

如果你走 GCC/G++ 前端，直接用：

```bash
cmake -S . -B build-ax650 \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/aarch64-linux-gnu.cmake \
  -DTALOS_TARGET_TRIPLE=aarch64-unknown-linux-gnu \
  -DBUILD_TESTING=OFF

cmake --build build-ax650 --target talos -j8
```

如果你走 `clang21` 前端，改用：

```bash
cmake -S . -B build-ax650 \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/aarch64-linux-gnu-clang.cmake \
  -DTALOS_TARGET_TRIPLE=aarch64-unknown-linux-gnu \
  -DTALOS_LLVM_ROOT=/opt/homebrew/opt/llvm \
  -DTALOS_GNU_TOOLCHAIN_ROOT="$(brew --prefix aarch64-unknown-linux-gnu)/toolchain" \
  -DBUILD_TESTING=OFF

cmake --build build-ax650 --target talos -j8
```

如果你的 GNU 交叉工具链前缀不同，例如 `aarch64-linux-gnu-*`，改成：

```bash
cmake -S . -B build-ax650 \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/aarch64-linux-gnu.cmake \
  -DTALOS_TARGET_TRIPLE=aarch64-linux-gnu \
  -DBUILD_TESTING=OFF
```

`clang21` 版同理，把 `TALOS_TARGET_TRIPLE` 改成 `aarch64-linux-gnu`。

如果你传的是 Homebrew formula 前缀而不是实际 GCC 根，工具链文件现在也会自动把它规整到
`.../toolchain`。但命令行上直接传 `.../toolchain` 更清晰。

如果你的编译器 triple 是 `aarch64-unknown-linux-gnu`，但 sysroot 来自 Debian/Ubuntu，
里面的包目录通常还是 `usr/lib/aarch64-linux-gnu`。仓库里的工具链文件现在会同时探测这两种目录名，
并自动为 `OpenCV`、`TBB`、`Ceres`、`GTest` 预填 `*_DIR`。

## 7. 常见报错与处理

### `OpenCV not found` / `TBB not found`

说明 sysroot 里没有对应目标包，或者缺少 `OpenCVConfig.cmake` / `tbbConfig.cmake` / `.pc` 文件。
先修 sysroot，不要退回去用宿主 macOS 的包。

### `GLIBCXX_* not found`

说明你链接时使用的 `libstdc++` 比板子运行时更新。换一套更老的交叉工具链，或者把目标板运行时升级到匹配版本。

### `ax_sys_api.h: No such file or directory`

说明 sysroot 中没有 `${TALOS_SYSROOT}/soc/soc/include`，或者 `/soc` 没有一起拷出来。

### `libax_sys.so: cannot open shared object file`

这是目标板运行时问题，不是交叉编译本身的问题。需要在板子上保证：

```bash
export LD_LIBRARY_PATH=/soc/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH="/root/talos/build/_deps/talos-ffmpeg/ffmpeg-master-latest-linuxarm64-gpl-shared/lib:/root/talos/3dparty/hik_sdk/lib/arm64:$LD_LIBRARY_PATH"
```

## 8. 当前仓库里已处理的交叉编译坑

为了让这套流程可用，仓库已经做了两处修正：

1. 交叉编译时不再强制加 `-march=native -mtune=native`
2. `GTest` 只在 `BUILD_TESTING=ON` 时才要求存在

这样默认的 `Release + BUILD_TESTING=OFF` 交叉构建不会再被宿主机优化选项和测试依赖卡住。
