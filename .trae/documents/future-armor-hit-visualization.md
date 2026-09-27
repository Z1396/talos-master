# 实现击打时刻预测装甲板可视化（FutureArmorSnapshot）

## Context

项目中 L4 的目标选择/弹道解算已经有可视化（`/solver/scene` 画选中目标球体、`/debug/mpc_prediction_scene` 画 MPC 轨迹+子弹弹道线），但**缺少"弹丸飞行时间 TOF 之后装甲板会在哪"的预测可视化**。作者已预留了两个未使用的符号：

- `l4_planning_systems.cpp` L37-43：`FutureArmorSnapshot` 结构体（center / linear_velocity / angular_velocity / armors 位姿数组），注释写明"预留，用于目标装甲未来位姿可视化"
- `tactical_palette.hpp` L192：`FUTURE_ARMOR` 颜色常量（预测未来装甲位置颜色）

目标：在 `/debug/mpc_prediction_scene`（`foxglove_l4_mpc_prediction_scene`，250Hz，已画子弹弹道线）中叠加"当前时刻 + cmd->tof 时刻"预测出的装甲板立方体，并与弹道线终点直观对齐——一眼看出弹丸到达时装甲板转到哪、能否命中。

## 实现方案（全部改动集中在 l4_planning_systems.cpp，共 3 处）

### 1. 补 include

```cpp
#include "vis_helpers.hpp"   // make_armor_cube / cls_to_armor_type（同 l3_estimation_systems.cpp 用法）
```

### 2. 在 `l4_detail` 命名空间新增预测函数（使用预留的 FutureArmorSnapshot）

```cpp
/// 用预留结构体：把选中目标状态线性外推 dt 秒，输出击打时刻装甲板位姿
[[nodiscard]] inline std::optional<FutureArmorSnapshot>
    predict_future_armors(const ::fcs::L4::SelectedTargetSnapshot& snapshot, double dt) noexcept;
```

逻辑（复用现有数学，不新造轮子）：
- 前置条件：`snapshot.has_target()` 且 `dt > 0`，否则返回 nullopt
- **机器人目标**：拷贝 `tracker.robot_state()`（RobotTargetState，11维EKF状态）→ `position += velocity*dt`、`yaw += v_yaw*dt` → 调用其 `armor_poses()` 得 4 块 `[x,y,z,yaw]`（复用 L3 的位姿计算，含 radius0/radius1/z1 几何）
- **前哨站目标**：拷贝 `outpost_state()` → 二维 position 外推、yaw 外推、`z[i] += vz*dt` → `armor_poses()` 得 3 块
- 填充 `FutureArmorSnapshot`：center=外推后中心、linear_velocity/angular_velocity 直接取自状态、armors=外推后位姿数组

### 3. 在 `foxglove_l4_mpc_prediction_scene` 系统中渲染

- 系统增加一个输入通道：`talos::spmc<::fcs::L4::SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic> selected_target_in`（同 `foxglove_l4_gimbal_scene` L749 的用法，`read_current()` 读取）
- 渲染条件：`cmd && snapshot && snapshot->has_target() && cmd->tof > 0`
- 调用 `predict_future_armors(*snapshot, cmd->tof)` 得到预测快照
- 生成一个实体（id `"l4_future_armors"`，frame `fast_tf::odom`，timestamp 取 cmd->timestamp_ns）：
  - 每块预测装甲板：复用 `vis_helpers` 的 `make_armor_cube(pose, tilt, height)`（大/小装甲尺寸用 `cls_to_armor_type(target_name)` 判断，与现有代码一致），画完覆盖 `cube.color = tac::L4::FUTURE_ARMOR`（半透明，区分当前时刻装甲）
  - **选中的装甲板**（`snapshot->selected_armor_id`）：颜色改为高亮（`tac::L4::PREDICTION_AMBER`），并加 billboard 文本 `fmt::format("hit t+{:.0f}ms", cmd->tof*1000)`，标识"弹丸到达时击中的是这块"
- 实体 push 进现有 `entities` 数组，随 `publish_scene_if_nonempty<MpcPredictionSceneMessage>` 一起发布，无需新增话题/通道定义

## 关键复用清单

| 复用项 | 位置 |
|---|---|
| `FutureArmorSnapshot` 预留结构体 | l4_planning_systems.cpp L38 |
| `FUTURE_ARMOR` 预留颜色 | tactical_palette.hpp L192 |
| `make_armor_cube` / `cls_to_armor_type` | src/fcs_visualization/systems/vis_helpers.hpp |
| `armor_poses()`（Robot/Outpost 两套） | src/fcs/L3_estimation/tracker/types.hpp |
| `SelectedTargetSnapshot`（含完整 TrackerOutput + selected_armor_id） | src/fcs/L4_planning/selected_target_snapshot.hpp |
| `WeaponCommand::tof` | src/fcs/L5_weapon/fire_control.hpp L77 |
| `SelectedTargetSnapshotChannelTopic` 通道 | core/channel_topics.hpp（gimbal scene 已用） |

## 不做的事

- 不改任何 L3/L4/L5 业务逻辑（纯可视化，线性外推即 EKF 的 CV 运动学假设，与 aimer 预测一致）
- 不新增 Foxglove 话题/消息类型定义
- 不动 vis_helpers.hpp（颜色直接在调用处覆盖 `cube.color`）

## 验证

1. `./build.sh --release` 编译通过（Clang 21，零新增警告）
2. 启动 Daedalus 仿真器 + talos，Foxglove 添加 3D 面板订阅 `/debug/mpc_prediction_scene`：
   - 跟踪运动目标时可见半透明"虚影"装甲板随目标旋转/平移前移
   - 高亮装甲板应落在子弹弹道线终点附近
   - 目标切换/丢失时虚影消失，无残留实体
3. 回放 MCAP 对比：`/track/scene`（当前装甲）与虚影的差值应约等于 v_yaw × tof 的旋转量
