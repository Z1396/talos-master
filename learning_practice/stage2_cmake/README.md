# Stage2：CMake 多模块项目（crate 宏教学）

模仿 Rust crate 模块化思想，用自研 `crate()` CMake 宏管理多模块 C++ 项目的教学示例（Talos 风格简化版）。

## 学习目标

1. 理解 CMake 多模块项目的组织方式（顶层 + 子目录 CMakeLists）
2. 掌握 `target_include_directories` / `target_link_libraries` 的 PUBLIC 传递语义
3. 学会编写 CMake 函数宏（`function()` + `cmake_parse_arguments`）
4. 了解 Talos 项目 `crates/` 模块化管理的底层原理

## 目录结构

```
stage2_cmake/
├── CMakeLists.txt            # 顶层：编译器/标准设置，按依赖顺序添加子目录
├── crates.cmake              # 核心：crate() 宏定义
├── modules/
│   ├── io/                   # 底层 crate：I/O 工具
│   │   ├── CMakeLists.txt    #   crate(io)
│   │   ├── include/io/io.hpp #   公共头文件（PUBLIC 对外可见）
│   │   └── src/io.cpp
│   └── math/                 # 中层 crate：数学库
│       ├── CMakeLists.txt    #   crate(math DEPENDENCIES io)
│       ├── include/math/math.hpp
│       └── src/math.cpp
└── app/
    ├── CMakeLists.txt        # 顶层应用：链接 math
    └── src/main.cpp
```

依赖链：`app → math → io`（app 无需声明 io，PUBLIC 链接自动传递）。

## 编译运行

```bash
cd learning_practice/stage2_cmake

# ① 配置（生成 Ninja 构建文件）
cmake -B build -G Ninja

# ② 编译（产物：build/modules/*/lib*.a + build/app/app）
cmake --build build

# ③ 运行（注意可执行文件在 build/app/ 目录下）
./build/app/app
```

预期输出：

```
[main] Stage2 CMake 项目启动
[add] 结果: [5, 7, 9]
[dot] 结果: 32.000000
[sum] 结果: 21.000000
[main] Stage2 演示完成
```

全清重来：`rm -rf build`。

## crate() 宏原理

`crates.cmake` 中的 `crate(name [DEPENDENCIES dep...])` 自动完成：

| 步骤 | 做的事 | 对应的手写 CMake |
|------|--------|-----------------|
| 1 | 扫描 `src/*.cpp` 作为源文件 | `file(GLOB sources CONFIGURE_DEPENDS src/*.cpp)` |
| 2 | 创建静态库目标 | `add_library(name STATIC ${sources})` |
| 3 | 强制 C++20 | `target_compile_features(name PUBLIC cxx_std_20)` |
| 4 | `include/` 设为公共头目录 | `target_include_directories(name PUBLIC include)` |
| 5 | 链接依赖并传递 | `target_link_libraries(name PUBLIC dep...)` |
| 6 | `src/pch.hpp` 存在则启用 PCH | `target_precompile_headers(name PRIVATE src/pch.hpp)` |

**PUBLIC 是关键**：math 以 PUBLIC 方式链接 io，所以 app 链接 math 时自动获得 io 的头文件路径和符号——这就是"依赖传递"。若改成 PRIVATE，app 将看不到 io 的头文件。

**CONFIGURE_DEPENDS 的作用**：新增/删除 `src/*.cpp` 后，下次构建会自动重新扫描（构建日志中的 `Re-checking globbed directories...`）。

## 如何新增一个模块

以新增 `network` crate（依赖 io）为例：

1. 建目录：

```
modules/network/
├── CMakeLists.txt
├── include/network/network.hpp   # namespace practice::network { ... }
└── src/network.cpp
```

2. `CMakeLists.txt` 只需一行：

```cmake
crate(network DEPENDENCIES io)
```

3. 顶层 `CMakeLists.txt` 注册（注意放在依赖它的模块之前）：

```cmake
add_subdirectory(modules/network)
```

4. 使用方直接链接：`target_link_libraries(app PRIVATE network)`。

## 与 Talos 真实项目的对应关系

| 本示例 | Talos 项目 |
|--------|-----------|
| `crate()` 宏自动收集源文件 | 各 `crates/xxx/CMakeLists.txt` 的模块封装 |
| `include/` PUBLIC 头目录 | 各 crate 的 `include/` 目录 |
| `src/pch.hpp` 自动 PCH | fcs 库的 PCH 加速（50-70%） |
| STATIC 静态库 | Talos 中 primitive/scheduler 等用 SHARED |
| `DEPENDENCIES` 传递 | `target_link_libraries(... PUBLIC ...)` |

区别：Talos 的库类型更多样（INTERFACE/SHARED）、有 export.hpp 导出宏管理、通过顶层 CMake 统一注册，但"一个模块一个目录、显式声明依赖、头文件 PUBLIC 对外"的思想完全一致。
