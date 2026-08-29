# stage9_fast_tf —— 强类型坐标变换树

对齐真实模块：`crates/fast_tf/src`（2,713 行，1 个 .cpp），demo 直接
`#include` 项目真实头文件，真实 `validation.cpp` 编译进静态库，不复制任何代码。

## 功能分析

| 文件 | 核心功能 | 关键技术点 |
|------|----------|-----------|
| `frame.hpp` | 帧树：`DECL` 宏定义 world→odom→gimbal_yaw→gimbal_pitch→camera_link→camera_optical / muzzle_link | 帧标签是**空类型**（编译期元信息，零运行时开销）；`frame` concept 校验；`is_descendant_of` 编译期递归查父链；`Fuck<Frame, T>` 强类型数值包裹 |
| `matrix.hpp` | `TransformMatrix<T, A, B>`：A/B 是帧类型参数的 SE(3) 变换 | `T<A,B> * T<B,C> = T<A,C>`、`inv(T<A,B>) = T<B,A>` 在**类型系统**里表达；静态工厂 `from_rpy / from_translation / from_rvec_tvec / from_quaternion`；`lerp`（slerp+线性平移）与 `lerp_se3`（流形测地线）双插值 |
| `buffer.hpp` | 时序环形缓冲 `Buffer<T, Range, Density>` | 四种查询模式标签分发：`exact`（必须精确匹配）/ `nearest`（就近）/ `interpolate`（按比例插值）/ `clamped`（超界钳位）；TBB 自旋读写锁（读多写少）；非法数据在 `push` 处就被门卫丢弃；`BufferOps<T>` 特化：矩阵 LERP、SO3 用 log-exp 流形插值 |
| `validation.cpp/.hpp` | 变换合法性校验 | NaN/Inf 逐分量检测；四元数归一化容差 0.01（对齐 ROS tf2）；`.cpp` 对全部帧组合**显式实例化**（提速 + 防链接错误） |
| `euler.hpp`（来自 crates/math） | ROS2 Z-Y-X RPY 欧拉角 | `quat()/matrix()/so3()` 互转，万向锁分支处理 |

### 为什么"强类型"值得 2,700 行

把 odom 系的向量传给 camera 系的函数，普通库（tf2 等）要等运行时才炸；
fast_tf 里每个坐标系是独立的 C++ 类型，`lookup<camera, odom>`（方向反了，
odom 不是 camera 的后代）在 `static_assert` 直接编译失败。坐标错乱是机器人
对瞄系统最难查的 bug 类别，这套设计把它整个挪到了编译期。

demo 中 `boot.cpp:112-167` 的 `init_coordinate_system` 复刻了实战初始化流程。

## 依赖说明（对齐 crate() 声明 talos_math / fmt / lieplusplus / TBB / opencv_core）

- **Eigen 3.4.0**：FetchContent（系统没有时）
- **fmt 12.0.0**：FetchContent 静态库（错误信息拼接）
- **oneTBB 2021.11.0**：仅取头文件。`buffer.hpp` 只用 `tbb::spin_rw_mutex`，
  实测纯 header-only 可编译链接，无需编 TBB 库
- **OpenCV**：`matrix.hpp` 只用了 `cv::Vec<T,3>`（`from_pnp` 接口参数），
  为免拉整套 OpenCV，`compat/opencv2/core/types.hpp` 提供 `cv::Vec` 最小替身；
  机器上有真 OpenCV 时移除该 include 目录即可无缝切回

## 运行方法

```bash
cd learning_practice/stage9_fast_tf
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/fast_tf_demo      # 断言失败时以非零退出码结束
```

### 编译器说明（沙箱实测踩坑记录）

- `frame.hpp` 的 `Fuck<Frame, T>` 用了 **C++23 deducing this**（显式对象参数
  `operator*(this Self&& self, ...)`），GCC 13 尚不支持 → 必须用 clang
  （优先 clang-21，学习沙箱回退 clang-18）。
- clang-18 + libstdc++-13 组合还有一个坑：clang-18 报告
  `__cpp_concepts = 201907L`（clang-19 起才是 202002L），而 libstdc++-13 的
  `<expected>` 以 `__cpp_concepts >= 202002L` 做门卫 → `std::expected` /
  `std::unexpected` 整个静默失效。CMake 里对 clang-18 分支显式加
  `-D__cpp_concepts=202002L` 覆盖（clang-18 的 concepts 实现已完整，安全）。

## 测试清单与预期输出

| 测试 | 内容 | 预期 |
|------|------|------|
| 测试1 | 帧树静态初始化（复刻 boot.cpp）+ `lookup` 查平移 | `T_world_camera` 平移 = (0.05, 0.02, 0.11)；`T_world_muzzle` = (0.2, 0, 0.05) |
| 测试2 | 编译期帧安全：`static_assert` + 复合/求逆代数 + 注释里的 3 类编译报错示例 | `inv` 复合回环 = 单位矩阵 |
| 测试3 | `lookup<odom, camera_link>` vs 手算 Eigen 4x4 连乘 | Frobenius 误差 < 1e-12；点 (1,2,3) 变换两途径误差 < 1e-9 |
| 测试4 | 插值：yaw 0→0.2rad 查 0.5s | `interpolate` yaw = 0.1（两端均值）；`nearest(0.6s)` = 0.2；`exact(0.5s)` 报错；`clamped(2s)` = 0.2；`interpolate(2s)` 报外推错误 |
| 测试5 | 校验 fail-fast + 自愈 | NaN 平移/旋转报错（带字段名与完整数值）；2 倍缩放矩阵被 SO3 构造函数**自愈**（读源码确认：强制归一化，模长恒 1）；`push(NaN)` 被丢弃，lookup 报 "empty" |
| 测试6 | `lookup_clamped` 超界钳位 | 2s 查询钳位到最新样本 yaw=0.2；普通 lookup 同时刻报 "future extrapolation" |

预期输出（节选）：

```
=== 测试1：帧树静态初始化（复刻 boot.cpp）===
  T_world_camera 平移 = (0.05, 0.02, 0.11)
  T_world_muzzle 平移 = (0.2, 0, 0.05)
测试1通过

=== 测试4：时间戳插值 + 四种查询模式 ===
  lookup<odom,gimbal_yaw>(0.5s)：yaw = 0.1（两端均值 0.1）
  nearest(0.6s)：yaw = 0.2（离 1s 样本更近，整样本返回）
  exact(0.5s)：按预期报错 —— ...
  clamped(2s)：钳位到最新样本，yaw = 0.2
...
全部测试通过（PASS）
```

## 不建独立 stage 的 4 个模块（收尾说明）

| 模块 | 不建原因 | 替代学习方案 |
|------|----------|-------------|
| quanta | 编码后端绑定特定硬件平台（ax_encode_backend 2,507 行），依赖 FFmpeg/厂商编解码，无硬件跑不出结果 | 读 `foxglove_systems.cpp` 的传输模式分支理解 MCP/WebSocket 双路 |
| hardware（hik_camera/at_gimbal） | 需要真实相机/串口硬件 + 厂商 SDK | stage6 已覆盖 thread_affinity；串口协议看 talos_gimbal/packet.hpp 的 static_assert 设计 |
| hardware_daedalus | 共享内存对端是 Daedalus 仿真进程，单独跑无意义 | 可选进阶：用 POSIX shm 自己写对端喂帧 |
| fcs_visualization | 依赖 Foxglove 服务 + 全流水线数据 | 已在 l1/l2 可视化系统上做过完整注释分析 |

## 学习产出顺序（stage7-11 收官）

stage10_math（SO2 热身）→ stage11_log → stage7_primitive → stage8_toml →
**stage9_fast_tf（最难：Eigen 连乘 + ring buffer 插值 + 编译期帧树）**
