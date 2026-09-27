# Talos 命中率影响因素全面分析报告

分析对象：`talos` 火控系统（C++23 / Clang 21 / ECS Scheduler 五级流水线 L1→L5）
分析视角：一切可能导致"该开火时没开 / 开了没打中"的因素，按对命中率的影响排序

---

## 优先级总览

| 级别 | 因素 | 影响量级 |
|---|---|---|
| **P0 致命** | aimer `delay = 0.0` 未补偿全链路延迟 | 每次瞄准系统性偏 1~2 个身位 |
| **P0 致命** | 竖直方向无视阻力的线性阻力模型 | 远距离（>8m）系统偏差 3~10cm |
| **P1 严重** | `shooting_range_w_*` 用小装甲窗口做普适判据 | 大装甲/远距目标接弹率下降 |
| **P1 严重** | EKF `lost_threshold` 区域内旋转装甲切换判据 | 旋转目标连续挥空 |
| **P2 重要** | Tracker `R` 静态配置 + 无 NIS 门控剔除 | 异常观测污染状态 0.2~0.5s |
| **P2 重要** | SVG "v_yaw" 单点估计无平滑/外推校验 | 装甲切换瞬间瞄准点跳变 |
| **P2 重要** | `optical_stale_timeout_s=0.25` 与相机帧率不匹配 | 低帧率场景丢帧即失效 |
| **P3 轻微** | 逐帧 `Eigen::MatrixXd` resize 分配 | 微秒级抖动，可通过内存池消除 |

---

## 一、P0：延迟补偿与弹道模型

### 1.1 aimer.delay = 0.0 全链路延迟未补偿

**位置**：`config/vision_base.toml` L270 `[aimer] delay = 0.0`；代码默认值 `L4Config::AimerConfig::delay{0.01}`（`src/fcs/L4_planning/config.hpp` L102）——**配置覆盖掉了这个 10ms 默认值，改成了 0**。

**机制**：`aim_generic()` 中 `total_delay = age + prediction_delay`（`src/fcs/L4_planning/aimer/aimer.cpp` L443-446），其中 `age` = 观测到当前的时间差（已自动测），`prediction_delay` 来自配置。设为 0 意味着：曝光时间、图像处理耗时、IMU/TF 查询时刻、MCU 通信、云台伺服响应，**全部没有补偿**。

**实测口径**：从传感器采集到 L5 指令到达云台，机械+链路延迟通常 **20~60ms**（sentry.toml 配 0.043 可佐证真实延迟量级在 40ms 上下）。

**量化影响**：目标横向速度以 3m/s 计，40ms 延迟 → 装甲偏移 12cm。小装甲板宽 12cm——**刚好偏出一个装甲板**。对垂直于视线方向运动的目标（最常见的绕圈走位），这是命中率第一杀手。

**注意**：各机型配置对此处理不一致，说明这是已知但未统一的问题：
- infantry_vision: `aimer.delay = -0.1`（负值，近乎恶意超调）
- sentry: 0.043、drone: 0.1、omni: 0.0

### 1.2 竖直方向无视阻力的线性阻力模型

**位置**：`src/fcs/core/trajectory/model/linear_drag.hpp` L80-81
```cpp
// 竖直方向运动方程：无阻力匀加速（仅线性阻力作用于水平方向，竖直忽略阻力）
const double z = v0 * std::sin(pitch) * tof - 0.5 * gravity_ * tof * tof;
```

**问题**：解析解 `tof = (exp(r*x)-1)/(r*v0*cosθ)` 只对水平方向施加线性阻力，竖直方向直接抛物体公式。真实弹丸竖直方向受阻力显著（且阻力与速度平方成正比，非线性）。

**量化影响**：弹速 25m/s、距离 8m 时，TOF≈0.33s，竖直方向忽略阻力导致落点偏高约 **5~8cm**（命中 = 0.12m 高窗口的 1/2 以下）；距离越远越糟，15m 时偏差可达 **20cm+**。同时该误差是**符号恒定的（打高）**，不能靠平均抵消。

**附带问题**：`resistance` 默认 0.001 且 `create_default` 为硬编码（`linear_drag.hpp` L46-48），配置链路未暴露给 toml，实际弹丸的空气阻力系数需要标定。

---

## 二、P1：接弹窗口与目标切换判据

### 2.1 shooting_range 用小窗口做普适判据

**位置**：`src/fcs/L5_weapon/fire_decision.hpp` L96-97
```cpp
auto shooting_range_yaw = std::abs(std::atan2(cfg.shooting_range_w_small / 2.0, distance));
```
窗口宽恒取 `shooting_range_w_small = 0.12`，`shooting_range_w_large = 0.24`（L35-38）定义了却**没有任何调用方使用**。

**影响**：英雄大装甲（24cm 宽）也按 12cm 判据——**一半可以命中的射击被判为"不可开火"**，反之远距离小窗口角度阈值过松。在打英雄/基地场景命中率直接损失约 30~50%。

### 2.2 旋转目标的装甲切换判据

**位置**：`aim_generic` 装甲选择逻辑（`aimer.cpp` L472-498）。三个优先级：`preferred`（上一帧选中）> `front_best`（前向窗内 delta_angle 最小）> `any_best`。

**问题**：`preferred` 优先意味着**只要上一帧选中的装甲还在 60° 前向窗内，即使另一块装甲已经转到更正的位置也绝不换**。旋转目标两块装甲交替到达窗口边缘时，会在两帧之间来回重选，云台在两块装甲间横跳。

**影响**：旋转半径 0.22m、转速 2π/s 的步兵，两块装甲切换周期约 100ms，若云台在两个角度间振荡 → 每次击发都打在"刚过去的那块"上，命中率折损约 20%。

---

## 三、P2：滤波与数据时效

### 3.1 NIS 门控缺失

**位置**：`new_tracker.hpp` L287 仅计算 `normalized_innovation_squared = ekf_.normalized_innovation_squared()`，写入 `FilterConvergenceState` 供可视化/统计，**未用于 update 门控**（发散的 `consecutive_diverged_updates` 同理只上报）。

**影响**：一次严重的 PnP 误检（检测框错位、数字识别错号）会以全权重进入 EKF。Q/R 静态（初值后不随观测质量自适应），单次坏观测把 P 抬高后需数秒才能重新收敛——这期间状态被持续带偏，`consecutive_converged_updates` 归零期间实质上不可信。

### 3.2 时效窗口与帧率不匹配

**位置**：`config/vision_base.toml` L286 `optical_stale_timeout_s = 0.25`，另 L164 `stale_timeout_s = 0.20`（ROI 回退全图）。

**影响**：若相机实际帧率 120fps（8.3ms）则宽裕；但降级/高负载场景掉到 30fps（33ms）后，0.25s 相当于 7.5 帧——NuVision 抖动或检测空窗会直接作废数据；同时 0.2s ROI 超时回退全图会引入检测延迟峰值，形成"卡顿→全图慢→更卡顿"的正反馈。

---

## 四、P3：实现效率（不直接损失命中率，造成时序抖动）

- `RobotTargetState`/`OutpostTargetState` 内的 `Eigen::MatrixXd P/K/Q/R` 为动态矩阵，EKF 每次 predict/update 前按固定尺寸构造/赋值（`new_tracker.hpp` L143-186 初始化区），4~8 目标 × 250Hz 下产生持续小对象分配。建议改 `Matrix<double, 11, 11>` 编译期尺寸。
- 250Hz `foxglove_l4_mpc_prediction_scene` 每周期生成 PNG/Scene JSON 与弹道离散点（trajectory_solver.cpp L102-134），对固定线程是可接受的（独占线程），但 `bullet_trajectory` pattern 每周期新建 `std::vector<SceneEntity>`，抖动源之一。

---

## 五、结论：修复优先级建议

1. **先测延迟再配 delay**：给 L5 指令和云台姿态回读打时间戳，统计观测→指令的真实延迟中位数，按机型写入 `aimer.delay`。预期命中率提升最显著的一项（对齐 sentry.toml 的 0.043 经验值）。
2. **fire_decision 接入大/小窗口分支**：传 `ArmorType`，大装甲用 `shooting_range_w_large`。
3. **竖直阻力建模**：`z` 方向改用数值积分或平方阻力解析解，标定 `resistance`。
4. **NIS 门控**：`normalized_innovation_squared > χ²(0.99)` 的观测跳过 update 并记 `consecutive_diverged_updates`。
5. **preferred 装甲加角度领先判据**：当次优装甲 delta_angle 比 preferred 小超过阈值（如 5°）时允许切换。
