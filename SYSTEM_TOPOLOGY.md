# Talos FCS 系统拓扑图

> 本文档整理 Talos 火控系统所有 ECS 系统的注册关系、通道读写与数据流链路。
> 数据来源：`src/fcs/runtime/boot.cpp`、`src/fcs/core/channel_topics.hpp`、各层 `systems.cpp`、`src/fcs_visualization/systems/`。

---

## 一、核心数据流主链路

```
                        ┌─────────────────────────────────────────────────────────┐
                        │                    L1 传感器层                          │
                        │  camera_reader(250Hz)──►ImageChannelTopic              │
                        │  imu_reader(1000Hz) ──►RuntimeControlStateChannelTopic │
                        └──────────────┬──────────────────────┬───────────────────┘
                                       │ Image                │ ControlState
                    ┌──────────────────┼──────────────────┐   │
                    ▼                  ▼                  ▼   │
         ┌──────────────────┐ ┌────────────────┐ ┌──────────────────┐
         │   装甲支线 L2     │ │  LDM吊射支线    │ │   Rune能量机关    │
         │                  │ │                │ │                  │
         │ armor_detector   │ │ ldm_detector   │ │ rune_detector    │
         │  (fixed 200Hz)   │ │ (fixed 200Hz)  │ │ (pool_compute)   │
         │   读Image         │ │  读Image        │ │  读Image         │
         │   写Detection     │ │ 写LdmDetection  │ │ 写RuneObservation│
         │       │          │ │ 写LdmMeasurement │ │ 写RuneDebugFrame │
         │       ▼          │ │     │    │      │ └──────┬───────────┘
         │ armor_solver     │ │     │    │      │        │
         │  (fixed 200Hz)   │ │     │    │      │        ▼
         │  读Detection     │ │     │    │      │ energy_meter(L3)
         │  写Measurement   │ │     │    │      │ (pool_compute)
         │       │          │ │     │    │      │ 读RuneObservation
         └───────┼──────────┘ │     │    │      │ 写EnergyMeterState
                 │            │     │    │      └────────┬─────────┘
                 │            │     │    │               │
                 ▼            │     │    │               │
         ┌──────────────────┐ │     │    │               │
         │   装甲支线 L3     │ │     │    │               │
         │ armor_tracker    │ │     │    │               │
         │ (fixed 250Hz)    │ │     │    │               │
         │  读Measurement   │ │     ▼    ▼               │
         │  写TrackerOutput │ │ ldm_naive_tracker(L3)     │
         └───────┬──────────┘ │ (fixed 250Hz)             │
                 │            │ 读LdmMeasurement          │
                 │            │ 写LdmState(默认通道)       │
                 │            └──────┬────────────────────┘
                 │                   │
                 │    ┌──────────────┘
                 ▼    ▼
        ┌──────────────────────────────────────────────────┐
        │              L4 规划层 — l4_aimer                 │
        │              (fixed 250Hz)                        │
        │  读: TrackerOutput + EnergyMeterState + LdmState  │
        │  写: SelectedTargetSnapshot                       │
        │      TargetSelectionTrace                         │
        │      ControlIntent                                │
        └──────────────────────┬───────────────────────────┘
                               │ ControlIntent
                               ▼
        ┌──────────────────────────────────────────────────┐
        │         L5 武器层 — enhanced_weapon_control       │
        │              (fixed 250Hz)                        │
        │  读: ControlIntent                                │
        │  写: WeaponCommand                                │
        └──────────────────────┬───────────────────────────┘
                               │ WeaponCommand
                               ▼
                    ┌──────────────────┐
                    │  weapon_output    │
                    │  (L1, fixed 250Hz)│
                    │  读WeaponCommand  │
                    │  → 下发硬件MCU     │
                    └──────────────────┘
```

---

## 二、通道定义总表（15 个 Topic）

| Topic | 承载数据 | 流向 |
|-------|---------|------|
| ImageChannelTopic | ImageFrame | L1 相机 → L2 检测器 |
| DetectionChannelTopic | ArmorDetectionBatch | L2 检测器 → L2 解算器 |
| MeasurementChannelTopic | ArmorMeasurementBatch | L2 解算器 → L3 跟踪器 |
| LdmDetectionChannelTopic | LdmDetection | L2 LDM检测器 → 可视化 |
| LdmMeasurementChannelTopic | LdmMeasurement | L2 LDM检测器 → L3 LDM跟踪 |
| TrackerOutputChannelTopic | TrackerOutputs | L3 跟踪器 → L4 规划器 |
| RuneObservationChannelTopic | RuneObservation | L2 Rune检测 → L3 energy_meter |
| RuneDebugFrameChannelTopic | RuneDebugFrame | L2 Rune检测 → 可视化/录制 |
| EnergyMeterStateChannelTopic | EnergyMeterState | L3 energy_meter → L4 规划器 |
| ControlIntentChannelTopic | ControlIntent(Track/Shot/Hold) | L4 规划器 → L5 武器层 |
| SelectedTargetSnapshotChannelTopic | SelectedTargetSnapshot | L4 → 录制/可视化/手性 |
| TargetSelectionTraceChannelTopic | TargetSelectionTrace | L4 → 可视化/录制 |
| WeaponCommandChannelTopic | WeaponCommand | L5 武器层 → L1 硬件输出 |
| RuntimeControlStateChannelTopic | ControlResourceSnapshot | L1 IMU → 录制/可视化 |
| GroundTruthBatchChannelTopic | GroundTruthBatch | 外部IPC → 可视化 |

---

## 三、核心系统明细表（18 个）

| # | 系统名 | 层 | 执行策略 | 读通道 | 写通道 |
|---|--------|----|---------|--------|--------|
| 1 | camera_reader | L1 | fixed_rate<250> | — | ImageChannelTopic |
| 2 | imu_reader | L1 | fixed_rate<1000> | — | RuntimeControlStateChannelTopic |
| 3 | weapon_output | L1 | fixed_rate<250> | WeaponCommandChannelTopic | —(→硬件MCU) |
| 4 | armor_detector | L2 | fixed_rate<200> | ImageChannelTopic | DetectionChannelTopic |
| 5 | armor_solver | L2 | fixed_rate<200> | DetectionChannelTopic | MeasurementChannelTopic |
| 6 | ldm_detector | L2 | fixed_rate<200> | ImageChannelTopic | LdmDetection + LdmMeasurement |
| 7 | rune_detector | L2 | pool_compute | ImageChannelTopic | RuneObservation + RuneDebugFrame |
| 8 | armor_tracker | L3 | fixed_rate<250> | MeasurementChannelTopic | TrackerOutputChannelTopic |
| 9 | ldm_naive_tracker | L3 | fixed_rate<250> | LdmMeasurementChannelTopic | LdmState(默认通道) |
| 10 | energy_meter | L3 | pool_compute | RuneObservationChannelTopic | EnergyMeterStateChannelTopic |
| 11 | l4_aimer | L4 | fixed_rate<250> | TrackerOutput + EnergyMeterState + LdmState | SelectedTargetSnapshot + TargetSelectionTrace + ControlIntent |
| 12 | l4_readback_roi | L4 | pool_compute | SelectedTargetSnapshotChannelTopic | res_mut\<TrackerReadbackCache\> |
| 13 | enhanced_weapon_control | L5 | fixed_rate<250> | ControlIntentChannelTopic | WeaponCommandChannelTopic |

---

## 四、辅助系统（5 个）

| # | 系统名 | 执行策略 | 读通道 | 输出 |
|---|--------|---------|--------|------|
| 14 | chiral_collector | pool_compute | SelectedTargetSnapshot | 共享内存(左右手同步) |
| 15 | stream_encode | fixed_rate<30,3> | ImageChannelTopic | H.265视频流(Foxglove) |
| 16 | runtime_control_recorder | fixed_rate_silent<250> | ControlIntent + WeaponCommand + ControlState | Mcap文件 |
| 17 | runtime_camera_recorder | fixed_rate_silent<20> | Image + WeaponCommand | Mcap文件 |
| 18 | runtime_capturer | fixed_rate_silent<4> | 几乎所有通道(12个) | Mcap文件 |

---

## 五、Foxglove 可视化系统群（28 个）

全部读通道 → 输出到 Foxglove WebSocket，绝大部分用 `pool_compute`，少数用 `fixed_rate`。

| 分组 | 系统名 | 策略 | 读通道 |
|------|--------|------|--------|
| L1 | foxglove_l1_image_pub | pool_compute | ImageChannelTopic |
| L2 装甲 | foxglove_l2_detection_pub | pool_compute | DetectionChannelTopic |
| | foxglove_l2_measurement_scene | pool_compute | MeasurementChannelTopic |
| | foxglove_l2_measurement_pub | pool_compute | MeasurementChannelTopic |
| | foxglove_debug_l2_pnp_pub | pool_compute | DetectionChannelTopic |
| | foxglove_debug_l2_nn_confidence_pub | pool_compute | DetectionChannelTopic |
| L2 LDM | foxglove_l2_ldm_scene | pool_compute | LdmMeasurementChannelTopic |
| | foxglove_l2_ldm_detection_json | pool_compute | LdmDetectionChannelTopic |
| | foxglove_l2_ldm_measurement_json | pool_compute | LdmMeasurementChannelTopic |
| L3 装甲 | foxglove_l3_tracker_scene | pool_compute | TrackerOutputChannelTopic |
| | foxglove_l3_association_scene | pool_compute | TrackerOutputChannelTopic |
| | foxglove_l3_ekf_heatmap | pool_compute | TrackerOutputChannelTopic |
| L3 LDM | foxglove_l3_ldm_tracker_scene | pool_compute | LdmState |
| | foxglove_l3_ldm_tracker_json | pool_compute | LdmState |
| L3 Rune | foxglove_rune_scene | pool_compute | RuneObservationChannelTopic |
| | foxglove_rune_ekf_scene | pool_compute | EnergyMeterStateChannelTopic |
| | foxglove_rune_debug_images | fixed_rate<30> | RuneDebugFrameChannelTopic |
| | foxglove_rune_debug_json | pool_compute | RuneDebugFrameChannelTopic |
| | foxglove_debug_energy_meter_pub | pool_compute | EnergyMeterStateChannelTopic |
| L4 | foxglove_solver_target_pub | pool_compute | SelectedTargetSnapshotChannelTopic |
| | foxglove_target_selection_pub | pool_compute | TargetSelectionTraceChannelTopic |
| | foxglove_l4_gimbal_cmd_pub | pool_compute | WeaponCommand + ControlIntent |
| | foxglove_l4_mpc_trajectory_pub | pool_compute | WeaponCommand + ControlIntent |
| | foxglove_l4_gimbal_scene | pool_compute | ControlIntent + SelectedTarget + Trace |
| | foxglove_l4_mpc_prediction_scene | fixed_rate<250> | WeaponCommand |
| 调试 | foxglove_res_pub | pool_compute | 资源状态 |
| | foxglove_debug_perf_stats_pub | fixed_rate_silent<100> | 性能统计 |
| 真值 | foxglove_ground_truth_pub | pool_compute | GroundTruthBatchChannelTopic |

---

## 六、Foxglove 可视化数据流链路

### 6.1 链路全景图

```
  核心系统输出                 SPMC 通道                Foxglove 系统              输出类型              Foxglove 前端
 ────────────────         ────────────────         ────────────────          ──────────────         ──────────────

 camera_reader ──► ImageChannelTopic ──────► foxglove_l1_image_pub ───► JPEG图像 + TF + 标定 ──► 图像面板/3D场景
                                          ├─► stream_encode ──────────► H.265视频流 ──────────► 视频面板
                                          ├─► runtime_camera_recorder ► Mcap文件 ────────────► 离线回放

 armor_detector ─► DetectionChannelTopic ├── foxglove_l2_detection_pub ──► JSON(DebugArmors) ──► JSON面板
                                          ├── foxglove_debug_l2_pnp_pub ──► JSON(PnPSolver) ────► JSON面板
                                          └── foxglove_debug_l2_nn_conf ─► JSON(NNConfidence) ─► JSON面板

 armor_solver ───► MeasurementChannel ├── foxglove_l2_measurement_scene ► 3D场景(SceneMessage) ► 3D场景面板
                                     ├── foxglove_l2_measurement_pub ──► JSON(Measurement) ───► JSON面板
                                     └── foxglove_debug_l2_pnp_pub ────► JSON(PnPSolver) ─────► JSON面板

 ldm_detector ───► LdmDetectionChannel ├── foxglove_l2_ldm_detection_json ► JSON(LdmDetection) ► JSON面板
                 ► LdmMeasurementChannel├── foxglove_l2_ldm_scene ────────► 3D场景(SceneMessage) ► 3D场景面板
                                      └── foxglove_l2_ldm_measurement_json ► JSON(LdmMeasurement)► JSON面板

 rune_detector ──► RuneObservationChannel ├─ foxglove_rune_scene ────────► 3D场景(RuneScene) ──► 3D场景面板
                 ► RuneDebugFrameChannel  ├── foxglove_rune_ekf_scene ───► 3D场景(RuneEkfScene) ► 3D场景面板
                                        ├── foxglove_rune_debug_images ──► 图像(JPEG) ──────────► 图像面板
                                        └── foxglove_rune_debug_json ───► JSON(RuneDebug) ─────► JSON面板

 armor_tracker ──► TrackerOutputChannel ├── foxglove_l3_tracker_scene ──► 3D场景(TrackScene) ──► 3D场景面板
                                      ├── foxglove_l3_association_scene ► 3D场景(Association) ─► 3D场景面板
                                      └── foxglove_l3_ekf_heatmap ──────► 图像(热力图) ────────► 图像面板

 ldm_naive ──────► LdmState(默认通道) ├── foxglove_l3_ldm_tracker_scene ► 3D场景(LdmTrackScene)► 3D场景面板
                                    └── foxglove_l3_ldm_tracker_json ──► JSON(LdmState) ──────► JSON面板

 energy_meter ───► EnergyMeterStateChannel ├── foxglove_rune_ekf_scene ──► 3D场景(RuneEkfScene) ► 3D场景面板
                                          └── foxglove_debug_energy_meter ► JSON(EnergyMeter) ─► JSON面板

 l4_aimer ───────► SelectedTargetSnapshot ├── foxglove_solver_target_pub ─► JSON(Target) ──────► JSON面板
                 ► TargetSelectionTrace  ├─ foxglove_target_selection_pub ► JSON(TargetTrace) ─► JSON面板
                 ► ControlIntent         ├─ foxglove_l4_gimbal_scene ─────► 3D场景(GimbalScene) ► 3D场景面板
                                        └─ foxglove_l4_mpc_prediction_scene► 3D场景(MpcPrediction)►3D场景面板

 enhanced_weapon ► WeaponCommandChannel ├── foxglove_l4_gimbal_cmd_pub ──► JSON(GimbalCmd) ────► JSON面板
                                    ├── foxglove_l4_mpc_trajectory_pub ► JSON(MpcTrajectory) ─► JSON面板
                                    └── foxglove_l4_mpc_prediction_scene► 3D场景(MpcPrediction) ► 3D场景面板

 (调度器内部) ────────────────────────────► foxglove_res_pub ──────────► JSON(Resource) ──────► JSON面板
                                          └── foxglove_debug_perf_stats_pub ► JSON(PerfStats) ─► JSON面板

 (外部IPC) ────► GroundTruthBatchChannel ► foxglove_ground_truth_pub ──► JSON(GroundTruth) ───► JSON面板
```

### 6.2 Foxglove 输出类型分类

| 输出类型 | 发送方法 | 系统数 | 说明 |
|---------|---------|--------|------|
| 3D 场景 | `publish_scene_if_nonempty<T>` | 9 | 绘制包围盒/轨迹/模型/线条，前端 3D 面板展示 |
| JSON 消息 | `publish_json_message<T>` | 16 | 结构化调试数据，前端 JSON 面板/图表展示 |
| 图像 | `enqueue_message` / `publish_jpeg_image` | 3 | JPEG 压缩图像或热力图，前端图像面板展示 |
| TF 坐标变换 | `publish_tf` | 1 | 坐标系树，前端 3D 场景依赖 |
| 相机标定 | `publish_camera_calibration` | 1 | 内参/畸变，前端去畸变/投影用 |
| H.265 视频 | `enqueue_message` | 1 | 视频流，前端视频面板 |

### 6.3 Foxglove 输出明细表

| Foxglove 系统 | 输出类型 | Message 类型 | Foxglove 前端面板 |
|---------------|---------|-------------|------------------|
| foxglove_l1_image_pub | 图像+TF+标定 | CompressedImage / FrameTransform / CameraCalibration | 图像面板 / 3D场景 |
| foxglove_l2_detection_pub | JSON | DebugArmorsMessage | JSON 面板 |
| foxglove_l2_measurement_scene | 3D 场景 | SceneMessage | 3D 场景面板 |
| foxglove_l2_measurement_pub | JSON | MeasurementMessage | JSON 面板 |
| foxglove_debug_l2_pnp_pub | JSON | PnPSolverMessage | JSON 面板 |
| foxglove_debug_l2_nn_confidence_pub | JSON | NNConfidenceMessage | JSON 面板 |
| foxglove_l2_ldm_scene | 3D 场景 | SceneMessage | 3D 场景面板 |
| foxglove_l2_ldm_detection_json | JSON | LdmDetectionMessage | JSON 面板 |
| foxglove_l2_ldm_measurement_json | JSON | LdmMeasurementMessage | JSON 面板 |
| foxglove_l3_tracker_scene | 3D 场景 | TrackSceneMessage | 3D 场景面板 |
| foxglove_l3_association_scene | 3D 场景 | AssociationSceneMessage | 3D 场景面板 |
| foxglove_l3_ekf_heatmap | 图像 | CompressedImage(热力图) | 图像面板 |
| foxglove_l3_ldm_tracker_scene | 3D 场景 | LdmTrackSceneMessage | 3D 场景面板 |
| foxglove_l3_ldm_tracker_json | JSON | LdmStateMessage | JSON 面板 |
| foxglove_rune_scene | 3D 场景 | RuneSceneMessage | 3D 场景面板 |
| foxglove_rune_ekf_scene | 3D 场景 | RuneEkfSceneMessage | 3D 场景面板 |
| foxglove_rune_debug_images | 图像 | CompressedImage | 图像面板 |
| foxglove_rune_debug_json | JSON | RuneDebugMessage | JSON 面板 |
| foxglove_solver_target_pub | JSON | TargetMessage | JSON 面板 |
| foxglove_target_selection_pub | JSON | TargetSelectionTraceMessage | JSON 面板 |
| foxglove_l4_gimbal_cmd_pub | JSON | GimbalCmdMessage | JSON 面板 |
| foxglove_l4_mpc_trajectory_pub | JSON | MpcTrajectoryMessage | JSON 面板 |
| foxglove_l4_gimbal_scene | 3D 场景 | GimbalSceneMessage | 3D 场景面板 |
| foxglove_l4_mpc_prediction_scene | 3D 场景 | MpcPredictionSceneMessage | 3D 场景面板 |
| foxglove_res_pub | JSON | ResourceMessage | JSON 面板 |
| foxglove_debug_energy_meter_pub | JSON | EnergyMeterMessage | JSON 面板 |
| foxglove_debug_perf_stats_pub | JSON | PerfStatsMessage | JSON 面板 |
| foxglove_ground_truth_pub | JSON | GroundTruthMessage | JSON 面板 |

### 6.4 Foxglove 链路特征

**数据消费模式**：所有 Foxglove 系统通过 `talos::spmc<T, Topic>` 只读通道订阅核心系统输出，通过 `talos::res<std::shared_ptr<FoxgloveServer>>` 资源获取 FoxgloveServer 句柄发送数据。

**触发方式**：
- `pool_compute`（25个）：依赖触发，上游通道有新数据时调度器自动触发
- `fixed_rate<30>`（1个）：foxglove_rune_debug_images 定频 30Hz 推送调试图像
- `fixed_rate<250>`（1个）：foxglove_l4_mpc_prediction_scene 定频 250Hz 插值 MPC 轨迹
- `fixed_rate_silent<100>`（1个）：foxglove_debug_perf_stats_pub 定频 100Hz 采集性能统计

**输出汇聚**：所有 Foxglove 系统的输出最终汇聚到同一个 `FoxgloveServer` 实例，通过 WebSocket 推送到 Foxglove Studio 前端。

**通道扇出**：`TrackerOutputChannelTopic` 被 3 个 Foxglove 系统消费，`MeasurementChannelTopic` 被 4 个消费，`WeaponCommandChannelTopic` 被 3 个消费。

---

## 七、关键拓扑特征

### 三条并行支线

在 L2 分叉、L3 各自滤波、L4 汇聚：

- **装甲支线**：detector → solver → tracker（fixed_rate 主链路，200/200/250Hz）
- **Rune 能量机关**：detector → energy_meter（pool_compute 旁路）
- **LDM 吊射**：detector → naive_tracker（fixed_rate 旁路）

### 数据汇聚点

`l4_aimer` 同时读三条支线的输出，按优先级（装甲 > 能量机关 > LDM）决策，输出 `ControlIntent`（variant: Track/Shot/Hold）。

### 闭环反馈

`l4_readback_roi` 读 `SelectedTargetSnapshot` 写 `res_mut<TrackerReadbackCache>`，L2 的 `armor_detector` 读这个缓存裁剪 ROI，形成 L4→L2 的反向数据流。

### 消费者扇出

- `ImageChannelTopic` 有 5 个消费者（detector×3 + stream_encode + capturer）
- `WeaponCommandChannelTopic` 有 6 个消费者（weapon_output + capturer×2 + foxglove×3）

### 执行策略规律

| 策略 | 用途 | 典型系统 |
|------|------|---------|
| fixed_rate | 关键路径，独占线程定频触发 | 相机/检测/跟踪/瞄准/武器 |
| fixed_rate_silent | 非关键定频，不通知下游 | 录制系统 |
| pool_compute | TBB线程池，依赖触发 | 可视化/Rune检测/energy_meter |

关键路径（相机/检测/跟踪/瞄准/武器）全部 `fixed_rate`，非关键路径（可视化/录制/Rune检测/energy_meter）用 `pool_compute` 或 `fixed_rate_silent`。
