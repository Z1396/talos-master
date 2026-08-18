# Talos 项目指南

> 本文档是 Talos 项目的完整参考手册，按模块分章节组织。当前已完成**坐标系系统**章节，其他模块章节将逐步补充。

---

## 目录

- [一、坐标系系统（fast_tf）](#一坐标系系统fast_tf)
  - [1.1 设计目标](#11-设计目标)
  - [1.2 坐标系树定义](#12-坐标系树定义)
  - [1.3 坐标轴方向规定](#13-坐标轴方向规定)
  - [1.4 坐标单位](#14-坐标单位)
  - [1.5 时序环形缓冲区](#15-时序环形缓冲区)
  - [1.6 坐标变换查询算法](#16-坐标变换查询算法)
  - [1.7 插值策略](#17-插值策略)
  - [1.8 静态变换与动态变换](#18-静态变换与动态变换)
  - [1.9 与其他系统的坐标对应关系](#19-与其他系统的坐标对应关系)
  - [1.10 业务 API 速查](#110-业务-api-速查)
  - [1.11 常见错误与排查](#111-常见错误与排查)
- [二、调度器系统（scheduler）](#二调度器系统scheduler) *[待补充]*
- [三、FCS 流水线](#三fcs-流水线) *[待补充]*
- [四、感知层（L2）](#四感知层l2) *[待补充]*
- [五、估计层（L3）](#五估计层l3) *[待补充]*
- [六、规划层（L4）](#六规划层l4) *[待补充]*
- [七、武器层（L5）](#七武器层l5) *[待补充]*
- [八、可视化系统（Foxglove）](#八可视化系统foxglove) *[待补充]*
- [九、配置系统](#九配置系统) *[待补充]*
- [十、构建与部署](#十构建与部署) *[待补充]*

---

## 一、坐标系系统（fast_tf）

**代码位置**：`crates/fast_tf/src/`
**核心文件**：
- `frame.hpp` — 坐标系树定义、查询 API
- `buffer.hpp` — 时序环形缓冲区
- `matrix.hpp` — 强类型变换矩阵
- `euler.hpp` — 欧拉角工具
- `groups/SO3.hpp` — SO3 旋转群数学

### 1.1 设计目标

Talos 是机器人火控系统，云台在高速转动，相机曝光和处理之间存在时间差。坐标系系统的核心目标是回答一个问题：

> **"在任意时刻 t，任意两个坐标系之间的变换矩阵是什么？"**

为了精确解算目标位置，必须查询"图像曝光那一瞬间"的机器人姿态，而不是当前姿态。这要求系统具备时序查询和插值能力。

### 1.2 坐标系树定义

项目用强类型 C++ 结构体表达坐标系树。每个坐标系是一个空结构体（仅作编译期类型标签），通过 `ancestor` 类型别名建立父子关系。

**树结构**（[frame.hpp:120-132](file:///home/pldx/Desktop/talos-master/crates/fast_tf/src/frame.hpp#L120-L132)）：

```
world (根)
 └── odom (里程计)
      └── gimbal_yaw (云台偏航)
           └── gimbal_pitch (云台俯仰)
                ├── camera_link (相机安装座)
                │    └── camera_optical (相机光学帧)
                └── muzzle_link (枪口发射点)
```

**定义方式**（宏批量生成）：

```cpp
DECL_ROOT(world);                                    // 根帧，ancestor = void
DECL(odom, world);                                   // 父: world
DECL(gimbal_yaw, odom);                              // 父: odom
DECL(gimbal_pitch, gimbal_yaw);                      // 父: gimbal_yaw
DECL(camera_link, gimbal_pitch);                     // 父: gimbal_pitch
DECL_WITH_ID(camera_optical, camera_link, "camera_optical_frame");  // 父: camera_link
DECL(muzzle_link, gimbal_pitch);                     // 父: gimbal_pitch
```

**业务代码使用的短别名**（[frame.hpp:140-146](file:///home/pldx/Desktop/talos-master/crates/fast_tf/src/frame.hpp#L140-L146)）：

| 别名 | 实际类型 | 父帧 |
|------|---------|------|
| `fast_tf::world` | `world_fuxk_frame` | —（根） |
| `fast_tf::odom` | `odom_fuxk_frame` | world |
| `fast_tf::gimbal` | `gimbal_yaw_fuxk_frame` | odom |
| `fast_tf::gimbal_pitch` | `gimbal_pitch_fuxk_frame` | gimbal_yaw |
| `fast_tf::camera` | `camera_link_fuxk_frame` | gimbal_pitch |
| `fast_tf::camera_optical` | `camera_optical_fuxk_frame` | camera_link |
| `fast_tf::muzzle` | `muzzle_link_fuxk_frame` | gimbal_pitch |

> **注意**：`fast_tf::gimbal` 实际指向 `gimbal_yaw_fuxk_frame`（云台偏航帧），不是"云台整体"。`fast_tf::camera` 指向 `camera_link_fuxk_frame`（机械安装座），光学帧是 `camera_optical`。

### 1.3 坐标轴方向规定

项目遵循 **ROS REP-103 标准**（右手坐标系）：

| 坐标系 | X 轴 | Y 轴 | Z 轴 |
|--------|------|------|------|
| `world` / `odom` | 机器人正前方 | 机器人左方 | 机器人上方 |
| `gimbal_yaw` | 云台偏航指向 | — | — |
| `gimbal_pitch` | 云台俯仰指向 | — | — |
| `camera_link` | 相机机械安装座 X | — | — |
| `camera_optical` | **前方（光轴方向）** | **左方** | **上方** |
| `muzzle_link` | 枪管指向（弹道方向） | — | — |

**特殊约定**：`camera_optical` 采用 ROS 光学帧标准——**Z 轴朝前**（沿光轴），与常规坐标系（X 朝前）不同。这是 ROS 社区的标准约定，处理相机数据时务必注意。

### 1.4 坐标单位

- **平移**：米（m），使用 `fp_t`（即 `double`）
- **旋转**：弧度（rad），`math_fuxk::Ros2EulerRotd` 用 RPY（Roll-Pitch-Yaw）表示
- **时间戳**：纳秒（ns），类型 `timestamp_ns_t`（即 `std::uint64_t`）

### 1.5 时序环形缓冲区

**核心思想**：不存"每个帧的绝对位姿"，而是存"每条边（父→子变换）的时序历史"。

**`CoordinateSystem` 容器**（[frame.hpp:346-349](file:///home/pldx/Desktop/talos-master/crates/fast_tf/src/frame.hpp#L346-L349)）：

```cpp
using EphemeralBuffer = Buffer<EdgeTransform<T>, 5, 500>;
//                                   ↑     ↑    ↑
//                              父→子变换  5秒  500样本/秒 → 容量2500

using CoordinateSystem = std::tuple<
    EphemeralBuffer<odom>,            // world→odom 边的时序历史
    EphemeralBuffer<gimbal_yaw>,      // odom→gimbal_yaw
    EphemeralBuffer<gimbal_pitch>,    // gimbal_yaw→gimbal_pitch
    EphemeralBuffer<camera_link>,     // gimbal_pitch→camera_link
    EphemeralBuffer<camera_optical>,  // camera_link→camera_optical
    EphemeralBuffer<muzzle_link>>;    // gimbal_pitch→muzzle_link
```

**缓冲区特性**（[buffer.hpp](file:///home/pldx/Desktop/talos-master/crates/fast_tf/src/buffer.hpp)）：

| 属性 | 值 |
|------|-----|
| 容量 | 5秒 × 500样本/秒 = 2500 个样本 |
| 存储 | `std::array`（栈分配，零堆分配） |
| 同步 | TBB `spin_rw_mutex`（读多写少，读共享/写独占） |
| 淘汰策略 | 环形覆盖，5 秒外的旧数据自动淘汰 |
| 时间戳约束 | 严格单调递增，乱序样本丢弃 |

**写入逻辑**（[buffer.hpp:237-270](file:///home/pldx/Desktop/talos-master/crates/fast_tf/src/buffer.hpp#L237-L270)）：

```cpp
void push(timestamp_ns_t timestamp_ns, const T& value) {
    if (!Ops::is_valid(value)) return;          // 非法数据丢弃
    mutex_type::scoped_lock lock(mutex_);       // 独占写锁
    if (timestamp_ns <= newest_ts_) return;     // 时序倒退丢弃
    buffer_[head_] = {value, timestamp_ns};     // 写入head位置
    head_ = (head_ + 1) % capacity_value;       // head环形前进
    // 满了则覆盖最旧样本
}
```

### 1.6 坐标变换查询算法

**核心 API**：`fast_tf::lookup<Target, Source>(system, timestamp_ns)`

**数学原理**：从 Source 向上爬树到 Target，逐层左乘边变换：

```
T<Target, Source> = T<Target, Parent> × T<Parent, Child> × ... × T<Son, Source>
```

**示例**：查询"曝光时刻相机系到 odom 系的变换"

```cpp
auto tf = fast_tf::lookup<fast_tf::odom, fast_tf::camera_optical>(
    system, image_timestamp_ns);
// 等价于:
// T<odom, camera_optical> = T<odom,gimbal_yaw> × T<gimbal_yaw,gimbal_pitch>
//                         × T<gimbal_pitch,camera_link> × T<camera_link,camera_optical>
// 每层都用 image_timestamp_ns 插值，保证整条链是同一瞬间的姿态
```

**递归实现**（[frame.hpp:503-537](file:///home/pldx/Desktop/talos-master/crates/fast_tf/src/frame.hpp#L503-L537)）：

```cpp
template <frame Target, frame Source, frame CurrentFrame>
constexpr auto lookup(const CoordinateSystem& system,
                      TransformMatrixd<CurrentFrame, Source> acc,
                      timestamp_ns_t ns) {
    static_assert(is_descendant_of<Source, Target>());  // 编译期校验
    
    if constexpr (std::is_same_v<CurrentFrame, Target>) {
        return acc;                                      // 爬到目标，返回累积结果
    } else {
        const auto& buffer = buffer_of<CurrentFrame>(system);
        auto result = buffer.lookup(ns, interpolate);    // 该时刻的边变换
        return lookup<Target, Source>(system, result->value * acc, ns);  // 左乘递归
    }
}
```

**执行过程**（以 `lookup<odom, camera_optical>(sys, t)` 为例）：

```
第1层: CurrentFrame=camera_optical, acc=I
    → 插值 T<camera_link, camera_optical>(t) [静态边]
    → acc = T_cl_co

第2层: CurrentFrame=camera_link
    → 插值 T<gimbal_pitch, camera_link>(t) [静态边]
    → acc = T_gp_cl × T_cl_co

第3层: CurrentFrame=gimbal_pitch
    → 插值 T<gimbal_yaw, gimbal_pitch>(t) [★动态边：曝光瞬间俯仰角]
    → acc = T_gy_gp × T_gp_cl × T_cl_co

第4层: CurrentFrame=gimbal_yaw
    → 插值 T<odom, gimbal_yaw>(t) [★动态边：曝光瞬间偏航角]
    → acc = T_odom_gy × ...

第5层: CurrentFrame=odom == Target
    → 返回 T<odom, camera_optical>(t) ✓
```

**关键**：每层递归用**同一时间戳**插值，保证整条链是同一瞬间的姿态——这就是时序一致性。

### 1.7 插值策略

四种查询模式（[buffer.hpp:37-47](file:///home/pldx/Desktop/talos-master/crates/fast_tf/src/buffer.hpp#L37-L47)）：

| 模式 | 标签 | 行为 | 适用场景 |
|------|------|------|---------|
| 精确匹配 | `fast_tf::exact` | 必须时间戳完全相等，否则报错 | 严格同步场景 |
| 就近取值 | `fast_tf::nearest` | 取前后更近的样本 | 容忍小误差 |
| **线性插值** | `fast_tf::interpolate` | 按时间比例插值（**项目主用**） | 大多数场景 |
| 钳位截断 | `fast_tf::clamped` | 超区间返回边界样本，不报错 | 对标 ROS TF2 TimePointZero |

**插值算法按数据类型特化**（[buffer.hpp:78-154](file:///home/pldx/Desktop/talos-master/crates/fast_tf/src/buffer.hpp#L78-L154)）：

| 数据类型 | 插值方法 | 原理 |
|---------|---------|------|
| `TransformMatrix` | 矩阵 LERP | 旋转平移分别线性插值 |
| `SO3`（四元数） | **流形 Slerp** | `a * exp(log(a⁻¹b) × ratio)`，保证插值结果仍是合法旋转 |
| `Spherial`（球坐标） | 球面 LERP | 球面线性插值 |

> **为什么 SO3 不能直接 LERP？** 线性插值两个旋转矩阵会破坏正交性（结果不再是旋转）。必须走 log-exp 映射在流形上走"最短弧"。

### 1.8 静态变换与动态变换

**静态缓冲区**（`StaticBuffer`，[buffer.hpp:596-597](file:///home/pldx/Desktop/talos-master/crates/fast_tf/src/buffer.hpp#L596-L597)）：

```cpp
template <typename T>
using StaticBuffer = Buffer<T, 0, 1>;  // Range=0, Density=1，只存1个样本
```

用于**永不变化的变换**，查询时无视时间戳直接返回：

| 边 | 类型 | 说明 |
|----|------|------|
| `gimbal_pitch → camera_link` | 静态 | 相机安装偏移（机械固定） |
| `camera_link → camera_optical` | 静态 | 光学帧旋转（ROS标准约定） |
| `gimbal_pitch → muzzle_link` | 静态 | 枪口安装偏移 |

**动态缓冲区**（`EphemeralBuffer`）：

| 边 | 类型 | 更新频率 |
|----|------|---------|
| `world → odom` | 动态 | 启动时设置一次 |
| `odom → gimbal_yaw` | 动态 | 云台偏航反馈（高频） |
| `gimbal_yaw → gimbal_pitch` | 动态 | 云台俯仰反馈（高频） |

### 1.9 与其他系统的坐标对应关系

**与 ROS TF2 的对应**：

| Talos 坐标系 | ROS 标准名 | 说明 |
|-------------|-----------|------|
| `world` | `world` / `map` | 全局世界坐标系 |
| `odom` | `odom` | 里程计坐标系 |
| `gimbal_yaw` | `gimbal_yaw` | 云台偏航 |
| `gimbal_pitch` | `gimbal_pitch` | 云台俯仰 |
| `camera_link` | `camera_link` | 相机机械座 |
| `camera_optical` | `camera_optical_frame` | 光学帧（ROS标准后缀） |
| `muzzle_link` | `muzzle_link` | 枪口 |

**与 Foxglove 可视化的对应**：

`fast_tf` 通过 [foxglove_export.hpp](file:///home/pldx/Desktop/talos-master/crates/fast_tf/src/foxglove_export.hpp) 将坐标系树发布到 Foxglove，前端能显示完整的 TF 树和 3D 场景。

**与 PnP 解算的对应**：

`armor_solver` 解算出的装甲板位姿在 `camera_optical` 坐标系下，通过 `lookup<odom, camera_optical>` 变换到 `odom` 系后才能用于跟踪和瞄准。

### 1.10 业务 API 速查

**写入变换**：

```cpp
// 写入完整 SE3 变换
fast_tf::update<fast_tf::gimbal>(system, transform, timestamp_ns);

// 仅更新旋转（平移复用最新样本）
fast_tf::update_rotate_only<fast_tf::gimbal>(system, rpy_rotation, timestamp_ns);
```

**查询变换**：

```cpp
// 线性插值查询（主用）
auto tf = fast_tf::lookup<fast_tf::odom, fast_tf::camera_optical>(system, t_ns);

// 钳位查询（超区间不报错）
auto tf = fast_tf::lookup_clamped<fast_tf::odom, fast_tf::camera_optical>(system, t_ns);

// 使用结果
if (tf.has_value()) {
    auto point_in_odom = tf.value() * point_in_camera;
}
```

**强类型保护**：

```cpp
Fuck<fast_tf::camera_optical, Eigen::Vector3d> p_cam = ...;
Fuck<fast_tf::odom, Eigen::Vector3d> p_odom = tf.value() * p_cam;  // ✓ 合法

p_cam + p_odom;  // ❌ 编译报错！不同坐标系不能相加
```

### 1.11 常见错误与排查

| 错误信息 | 原因 | 解决方法 |
|---------|------|---------|
| `static_assert: Source must be a descendant of Target` | 查询方向反了（如 `lookup<camera, odom>`） | 确认 Target 是祖先、Source 是后代 |
| `past extrapolation required` | 查询时间戳早于缓冲区最早样本 | 检查时间戳是否正确，或用 `lookup_clamped` |
| `future extrapolation required` | 查询时间戳晚于缓冲区最新样本 | 确认写入时间戳与查询时间戳的时序 |
| `buffer is empty` | 该边的缓冲区还没有数据 | 检查 boot 阶段是否初始化了该边 |
| `failed at edge X -> Y` | 某条边的插值失败 | 查看具体是哪条边，检查该边的缓冲区数据 |

---

## 二、调度器系统（scheduler）

*[待补充]*

预留内容：
- 调度器编程模型（System、Channel、Resource）
- 执行策略（fixed_rate / pool_compute / pool_visualization）
- 通道组件类型（spsc / spmc / res / local）
- build 阶段流程
- 运行时调度机制

---

## 三、FCS 流水线

*[待补充]*

预留内容：
- 五级流水线概览（L1-L5）
- 核心数据流主链路
- 通道定义总表
- 系统明细表

---

## 四、感知层（L2）

*[待补充]*

预留内容：
- 装甲检测（armor）
- LDM 吊射检测（ldm）
- Rune 能量机关检测（rune）
- PnP 位姿解算

---

## 五、估计层（L3）

*[待补充]*

预留内容：
- 装甲跟踪器（tracker）
- EKF 滤波
- LDM naive 跟踪
- 能量机关估计（energy_meter）

---

## 六、规划层（L4）

*[待补充]*

预留内容：
- 瞄准器（aimer）
- 目标选择逻辑
- 云台规划
- 弹道构建

---

## 七、武器层（L5）

*[待补充]*

预留内容：
- 火控解算
- 发射判定
- 摩擦轮控制

---

## 八、可视化系统（Foxglove）

*[待补充]*

预留内容：
- Foxglove WebSocket server
- 通道类型与 topic 定义
- 场景构建
- 调试数据发布

---

## 九、配置系统

*[待补充]*

预留内容：
- TOML 配置文件结构
- RuntimeConfig 解析
- 资源注册（insert_resource / emplace_resource）

---

## 十、构建与部署

*[待补充]*

预留内容：
- 构建脚本（build.sh）
- CMake 配置
- 依赖管理
- 交叉编译
- 部署流程

---

## 文档维护说明

- **坐标系章节**：已完成，基于 `crates/fast_tf/` 代码
- **其他章节**：预留框架，待逐步补充
- **更新原则**：代码变更时同步更新文档，保持文档与代码一致
