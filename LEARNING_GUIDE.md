# Talos 项目源码阅读指南

> 本文档按学习顺序列出所有关键源文件，链接可直接点击跳转。
>
> 编号共 **224 个文件**，按从宏观到微观、从框架到业务的顺序排列。
> 建议按编号顺序阅读，每读完一个阶段做一次阶段性总结。

---

## 阶段一：项目概览（先建立全局印象）

1. [README.md](./README.md)
2. [AGENTS.md](./AGENTS.md) — 编码哲学与规范（必读）
3. [CMakeLists.txt](./CMakeLists.txt) — 构建系统、依赖、编译选项
4. [at_vision.toml](./at_vision.toml) — 入口配置文件
5. [build.sh](./build.sh) — 一键构建脚本
6. [.clang-format](./.clang-format) — 代码格式化规则
7. [.clang-tidy](./.clang-tidy) — 静态检查规则

---

## 阶段二：核心框架——调度器与原语（理解 System 怎么跑起来的）

### 入口与启动

8. [src/main.cpp](./src/main.cpp) — 程序主入口
9. [src/fcs/runtime/boot.hpp](./src/fcs/runtime/boot.hpp) — boot 函数声明
10. [src/fcs/runtime/boot.cpp](./src/fcs/runtime/boot.cpp) — boot 函数实现（核心引导流程）

### 调度器核心

11. [crates/scheduler/src/scheduler/scheduler.hpp](./crates/scheduler/src/scheduler/scheduler.hpp) — Scheduler 类接口
12. [crates/scheduler/src/scheduler/scheduler.cpp](./crates/scheduler/src/scheduler/scheduler.cpp) — Scheduler 实现
13. [crates/scheduler/src/scheduler/world.hpp](./crates/scheduler/src/scheduler/world.hpp) — World 资源/通道容器
14. [crates/scheduler/src/scheduler/world.cpp](./crates/scheduler/src/scheduler/world.cpp) — World 实现
15. [crates/scheduler/src/scheduler/thin.hpp](./crates/scheduler/src/scheduler/thin.hpp) — 薄封装工具

### System 模型

16. [crates/scheduler/src/scheduler/system/system.hpp](./crates/scheduler/src/scheduler/system/system.hpp) — System 执行模型
17. [crates/scheduler/src/scheduler/system/system_meta.hpp](./crates/scheduler/src/scheduler/system/system_meta.hpp) — System 元信息与依赖描述
18. [crates/scheduler/src/scheduler/system/components.hpp](./crates/scheduler/src/scheduler/system/components.hpp) — 通道/资源组件（spmc、res 等）
19. [crates/scheduler/src/scheduler/system/execution_policy.hpp](./crates/scheduler/src/scheduler/system/execution_policy.hpp) — 执行策略（fixed_rate、pool_compute）

### 调度器辅助

20. [crates/scheduler/src/scheduler/error.hpp](./crates/scheduler/src/scheduler/error.hpp) — 调度器错误类型
21. [crates/scheduler/src/scheduler/error.cpp](./crates/scheduler/src/scheduler/error.cpp)
22. [crates/scheduler/src/scheduler/error_formatter.hpp](./crates/scheduler/src/scheduler/error_formatter.hpp)
23. [crates/scheduler/src/scheduler/error_formatter.cpp](./crates/scheduler/src/scheduler/error_formatter.cpp)
24. [crates/scheduler/src/scheduler/demangle.hpp](./crates/scheduler/src/scheduler/demangle.hpp)
25. [crates/scheduler/src/scheduler/demangle.cpp](./crates/scheduler/src/scheduler/demangle.cpp)

### 原语库（通道与并发）

26. [crates/primitive/src/primitive/channel.hpp](./crates/primitive/src/primitive/channel.hpp) — 统一通道抽象
27. [crates/primitive/src/primitive/spsc_triple_buffer.hpp](./crates/primitive/src/primitive/spsc_triple_buffer.hpp) — SPSC 三重缓冲
28. [crates/primitive/src/primitive/spmc_triple_buffer.hpp](./crates/primitive/src/primitive/spmc_triple_buffer.hpp) — SPMC 三重缓冲
29. [crates/primitive/src/primitive/overloaded.hpp](./crates/primitive/src/primitive/overloaded.hpp) — std::visit 重载辅助
30. [crates/primitive/src/primitive/lazy.hpp](./crates/primitive/src/primitive/lazy.hpp) — 延迟构造
31. [crates/primitive/src/primitive/spin.hpp](./crates/primitive/src/primitive/spin.hpp) — 自旋等待原语
32. [crates/primitive/src/primitive/performance_probe.hpp](./crates/primitive/src/primitive/performance_probe.hpp) — 性能探针
33. [crates/primitive/src/primitive/system_info.hpp](./crates/primitive/src/primitive/system_info.hpp) — 系统信息
34. [crates/primitive/src/primitive/thread_affinity.hpp](./crates/primitive/src/primitive/thread_affinity.hpp) — 线程亲和性

---

## 阶段三：FCS 核心类型与通道拓扑（数据流"地图"）

35. [src/fcs/core/channel_topics.hpp](./src/fcs/core/channel_topics.hpp) — ★ 通道 Topic 总定义（数据流地图）
36. [src/fcs/core/types.hpp](./src/fcs/core/types.hpp) — FCS 基础类型
37. [src/fcs/core/types.cpp](./src/fcs/core/types.cpp)
38. [src/fcs/core/types_pnp.hpp](./src/fcs/core/types_pnp.hpp) — PnP 类型
39. [src/fcs/core/armor_types.hpp](./src/fcs/core/armor_types.hpp) — 装甲板领域类型
40. [src/fcs/core/target_key.hpp](./src/fcs/core/target_key.hpp) — 目标身份 key
41. [src/fcs/core/time.hpp](./src/fcs/core/time.hpp) — 时间类型
42. [src/fcs/core/runtime.hpp](./src/fcs/core/runtime.hpp) — 运行时全局上下文

### 弹道模型

43. [src/fcs/core/trajectory/resource.hpp](./src/fcs/core/trajectory/resource.hpp) — 弹道全局资源
44. [src/fcs/core/trajectory/config.hpp](./src/fcs/core/trajectory/config.hpp) — 弹道配置
45. [src/fcs/core/trajectory/reference_trajectory.hpp](./src/fcs/core/trajectory/reference_trajectory.hpp) — 参考轨迹
46. [src/fcs/core/trajectory/solver/trajectory_solver.hpp](./src/fcs/core/trajectory/solver/trajectory_solver.hpp) — 弹道求解器
47. [src/fcs/core/trajectory/solver/trajectory_solver.cpp](./src/fcs/core/trajectory/solver/trajectory_solver.cpp)
48. [src/fcs/core/trajectory/solver/solver_interfaces.hpp](./src/fcs/core/trajectory/solver/solver_interfaces.hpp)
49. [src/fcs/core/trajectory/model/ballistic_model.hpp](./src/fcs/core/trajectory/model/ballistic_model.hpp) — 弹道模型
50. [src/fcs/core/trajectory/model/ideal.hpp](./src/fcs/core/trajectory/model/ideal.hpp) — 理想弹道
51. [src/fcs/core/trajectory/model/linear_drag.hpp](./src/fcs/core/trajectory/model/linear_drag.hpp) — 线性阻力弹道

---

## 阶段四：Runtime 运行时支撑

52. [src/fcs/runtime/config_loader.hpp](./src/fcs/runtime/config_loader.hpp) — 配置加载器
53. [src/fcs/runtime/config_loader.cpp](./src/fcs/runtime/config_loader.cpp)
54. [src/fcs/runtime/l1_l2_setup.hpp](./src/fcs/runtime/l1_l2_setup.hpp) — L1/L2 初始化辅助
55. [src/fcs/runtime/l1_l2_setup.cpp](./src/fcs/runtime/l1_l2_setup.cpp)
56. [src/fcs/runtime/build_info.hpp](./src/fcs/runtime/build_info.hpp) — 编译构建信息
57. [src/fcs/runtime/capturer.hpp](./src/fcs/runtime/capturer.hpp) — 数据录制抓包
58. [src/fcs/runtime/capturer.cpp](./src/fcs/runtime/capturer.cpp)
59. [src/fcs/runtime/stream_encode.hpp](./src/fcs/runtime/stream_encode.hpp) — 图像编码
60. [src/fcs/runtime/stream_encode.cpp](./src/fcs/runtime/stream_encode.cpp)
61. [src/fcs/runtime/stream_send.hpp](./src/fcs/runtime/stream_send.hpp)
62. [src/fcs/runtime/stream_send.cpp](./src/fcs/runtime/stream_send.cpp)
63. [src/fcs/runtime/replay.hpp](./src/fcs/runtime/replay.hpp) — 数据回放
64. [src/fcs/runtime/replay.cpp](./src/fcs/runtime/replay.cpp)

---

## 阶段五：L1 传感器层

65. [src/fcs/L1_sensor/camera_interface.hpp](./src/fcs/L1_sensor/camera_interface.hpp) — 相机接口
66. [src/fcs/L1_sensor/camera_interface.cpp](./src/fcs/L1_sensor/camera_interface.cpp)
67. [src/fcs/L1_sensor/output_interface.hpp](./src/fcs/L1_sensor/output_interface.hpp) — 硬件输出接口
68. [src/fcs/L1_sensor/output_interface.cpp](./src/fcs/L1_sensor/output_interface.cpp)
69. [src/fcs/L1_sensor/parcel.hpp](./src/fcs/L1_sensor/parcel.hpp) — 数据 parcel

---

## 阶段六：L2 感知层

### 装甲检测核心

70. [src/fcs/L2_perception/armor/systems.hpp](./src/fcs/L2_perception/armor/systems.hpp) — 装甲检测系统注册
71. [src/fcs/L2_perception/armor/systems.cpp](./src/fcs/L2_perception/armor/systems.cpp)
72. [src/fcs/L2_perception/armor/backend.hpp](./src/fcs/L2_perception/armor/backend.hpp) — 推理后端抽象
73. [src/fcs/L2_perception/armor/backend.cpp](./src/fcs/L2_perception/armor/backend.cpp)
74. [src/fcs/L2_perception/armor/solver.hpp](./src/fcs/L2_perception/armor/solver.hpp) — PnP 解算器
75. [src/fcs/L2_perception/armor/config.hpp](./src/fcs/L2_perception/armor/config.hpp) — 装甲检测配置
76. [src/fcs/L2_perception/armor/readback_roi.hpp](./src/fcs/L2_perception/armor/readback_roi.hpp) — ROI 裁剪

### 推理后端

77. [src/fcs/L2_perception/armor/backends/base.hpp](./src/fcs/L2_perception/armor/backends/base.hpp) — 后端基类
78. [src/fcs/L2_perception/armor/backends/ort.hpp](./src/fcs/L2_perception/armor/backends/ort.hpp) — ONNXRuntime 后端
79. [src/fcs/L2_perception/armor/backends/ort.cpp](./src/fcs/L2_perception/armor/backends/ort.cpp)
80. [src/fcs/L2_perception/armor/backends/tensor_rt.hpp](./src/fcs/L2_perception/armor/backends/tensor_rt.hpp) — TensorRT 后端
81. [src/fcs/L2_perception/armor/backends/tensor_rt.cpp](./src/fcs/L2_perception/armor/backends/tensor_rt.cpp)
82. [src/fcs/L2_perception/armor/backends/axera.hpp](./src/fcs/L2_perception/armor/backends/axera.hpp) — Axera NPU 后端
83. [src/fcs/L2_perception/armor/backends/axera.cpp](./src/fcs/L2_perception/armor/backends/axera.cpp)
84. [src/fcs/L2_perception/armor/backends/traditional.hpp](./src/fcs/L2_perception/armor/backends/traditional.hpp) — 传统算法后端
85. [src/fcs/L2_perception/armor/backends/traditional.cpp](./src/fcs/L2_perception/armor/backends/traditional.cpp)
86. [src/fcs/L2_perception/armor/backends/traditional_classifier.hpp](./src/fcs/L2_perception/armor/backends/traditional_classifier.hpp)
87. [src/fcs/L2_perception/armor/backends/traditional_classifier.cpp](./src/fcs/L2_perception/armor/backends/traditional_classifier.cpp)
88. [src/fcs/L2_perception/armor/backends/traditional_types.hpp](./src/fcs/L2_perception/armor/backends/traditional_types.hpp)
89. [src/fcs/L2_perception/armor/backends/axera_preprocess.hpp](./src/fcs/L2_perception/armor/backends/axera_preprocess.hpp)

### LDM 大符检测

90. [src/fcs/L2_perception/ldm/ldm_systems.hpp](./src/fcs/L2_perception/ldm/ldm_systems.hpp) — LDM 系统注册
91. [src/fcs/L2_perception/ldm/ldm_systems.cpp](./src/fcs/L2_perception/ldm/ldm_systems.cpp)
92. [src/fcs/L2_perception/ldm/ldm_detector.hpp](./src/fcs/L2_perception/ldm/ldm_detector.hpp) — LDM 检测器
93. [src/fcs/L2_perception/ldm/ldm_detector.cpp](./src/fcs/L2_perception/ldm/ldm_detector.cpp)
94. [src/fcs/L2_perception/ldm/ldm_solver.hpp](./src/fcs/L2_perception/ldm/ldm_solver.hpp)
95. [src/fcs/L2_perception/ldm/ldm_geometry.hpp](./src/fcs/L2_perception/ldm/ldm_geometry.hpp)
96. [src/fcs/L2_perception/ldm/ldm_config.hpp](./src/fcs/L2_perception/ldm/ldm_config.hpp)
97. [src/fcs/L2_perception/ldm/types.hpp](./src/fcs/L2_perception/ldm/types.hpp)

### 能量机关检测

98. [src/fcs/L2_perception/rune/rune_systems.hpp](./src/fcs/L2_perception/rune/rune_systems.hpp) — Rune 系统注册
99. [src/fcs/L2_perception/rune/rune_systems.cpp](./src/fcs/L2_perception/rune/rune_systems.cpp)
100. [src/fcs/L2_perception/rune/rune_detector.hpp](./src/fcs/L2_perception/rune/rune_detector.hpp) — 能量机关检测器
101. [src/fcs/L2_perception/rune/rune_detector.cpp](./src/fcs/L2_perception/rune/rune_detector.cpp)
102. [src/fcs/L2_perception/rune/rune_config.hpp](./src/fcs/L2_perception/rune/rune_config.hpp)
103. [src/fcs/L2_perception/rune/rune_detector_types.hpp](./src/fcs/L2_perception/rune/rune_detector_types.hpp)
104. [src/fcs/L2_perception/rune/types.hpp](./src/fcs/L2_perception/rune/types.hpp)

### L2 公共

105. [src/fcs/L2_perception/common/geometry.hpp](./src/fcs/L2_perception/common/geometry.hpp) — 几何工具

---

## 阶段七：L3 估计层

### 装甲板跟踪

106. [src/fcs/L3_estimation/tracker_systems.hpp](./src/fcs/L3_estimation/tracker_systems.hpp) — 跟踪系统注册
107. [src/fcs/L3_estimation/tracker_systems.cpp](./src/fcs/L3_estimation/tracker_systems.cpp)
108. [src/fcs/L3_estimation/manager.hpp](./src/fcs/L3_estimation/manager.hpp) — 跟踪管理器
109. [src/fcs/L3_estimation/extended_kalman_filter.cpp](./src/fcs/L3_estimation/extended_kalman_filter.cpp) — EKF 实现

### 跟踪器内部

110. [src/fcs/L3_estimation/tracker/config.hpp](./src/fcs/L3_estimation/tracker/config.hpp) — 跟踪配置
111. [src/fcs/L3_estimation/tracker/types.hpp](./src/fcs/L3_estimation/tracker/types.hpp) — 跟踪类型
112. [src/fcs/L3_estimation/tracker/extended_kalman_filter.hpp](./src/fcs/L3_estimation/tracker/extended_kalman_filter.hpp) — EKF
113. [src/fcs/L3_estimation/tracker/invariant_extended_kalman_filter.hpp](./src/fcs/L3_estimation/tracker/invariant_extended_kalman_filter.hpp) — IEKF
114. [src/fcs/L3_estimation/tracker/data_associator.hpp](./src/fcs/L3_estimation/tracker/data_associator.hpp) — 数据关联
115. [src/fcs/L3_estimation/tracker/acceleration_motion_model.hpp](./src/fcs/L3_estimation/tracker/acceleration_motion_model.hpp)
116. [src/fcs/L3_estimation/tracker/new_motion_model.hpp](./src/fcs/L3_estimation/tracker/new_motion_model.hpp)
117. [src/fcs/L3_estimation/tracker/new_tracker.hpp](./src/fcs/L3_estimation/tracker/new_tracker.hpp)
118. [src/fcs/L3_estimation/tracker/util.hpp](./src/fcs/L3_estimation/tracker/util.hpp)
119. [src/fcs/L3_estimation/tracker/vis_helpers.hpp](./src/fcs/L3_estimation/tracker/vis_helpers.hpp)

### 能量机关估计

120. [src/fcs/L3_estimation/energy_meter/energy_meter_systems.hpp](./src/fcs/L3_estimation/energy_meter/energy_meter_systems.hpp)
121. [src/fcs/L3_estimation/energy_meter/energy_meter_systems.cpp](./src/fcs/L3_estimation/energy_meter/energy_meter_systems.cpp)
122. [src/fcs/L3_estimation/energy_meter/energy_meter_config.hpp](./src/fcs/L3_estimation/energy_meter/energy_meter_config.hpp)
123. [src/fcs/L3_estimation/energy_meter/types.hpp](./src/fcs/L3_estimation/energy_meter/types.hpp)
124. [src/fcs/L3_estimation/energy_meter_solver/energy_meter_tracker.hpp](./src/fcs/L3_estimation/energy_meter_solver/energy_meter_tracker.hpp)
125. [src/fcs/L3_estimation/energy_meter_solver/energy_meter_tracker.cpp](./src/fcs/L3_estimation/energy_meter_solver/energy_meter_tracker.cpp)
126. [src/fcs/L3_estimation/energy_meter_solver/motion_model.hpp](./src/fcs/L3_estimation/energy_meter_solver/motion_model.hpp)
127. [src/fcs/L3_estimation/energy_meter_solver/voter.hpp](./src/fcs/L3_estimation/energy_meter_solver/voter.hpp)
128. [src/fcs/L3_estimation/energy_meter_solver/voter.cpp](./src/fcs/L3_estimation/energy_meter_solver/voter.cpp)
129. [src/fcs/L3_estimation/energy_meter_solver/types.hpp](./src/fcs/L3_estimation/energy_meter_solver/types.hpp)

### LDM 简易跟踪

130. [src/fcs/L3_estimation/ldm_naive/ldm_naive_systems.hpp](./src/fcs/L3_estimation/ldm_naive/ldm_naive_systems.hpp)
131. [src/fcs/L3_estimation/ldm_naive/ldm_naive_systems.cpp](./src/fcs/L3_estimation/ldm_naive/ldm_naive_systems.cpp)
132. [src/fcs/L3_estimation/ldm_naive/ldm_tracker.hpp](./src/fcs/L3_estimation/ldm_naive/ldm_tracker.hpp)
133. [src/fcs/L3_estimation/ldm_naive/ldm_kinematic_model.hpp](./src/fcs/L3_estimation/ldm_naive/ldm_kinematic_model.hpp)
134. [src/fcs/L3_estimation/ldm_naive/ldm_kinematic_params.hpp](./src/fcs/L3_estimation/ldm_naive/ldm_kinematic_params.hpp)
135. [src/fcs/L3_estimation/ldm_naive/ldm_naive_config.hpp](./src/fcs/L3_estimation/ldm_naive/ldm_naive_config.hpp)
136. [src/fcs/L3_estimation/ldm_naive/types.hpp](./src/fcs/L3_estimation/ldm_naive/types.hpp)

---

## 阶段八：L4 规划层

137. [src/fcs/L4_planning/planning_systems.hpp](./src/fcs/L4_planning/planning_systems.hpp) — 规划系统注册
138. [src/fcs/L4_planning/planning_systems.cpp](./src/fcs/L4_planning/planning_systems.cpp)
139. [src/fcs/L4_planning/config.hpp](./src/fcs/L4_planning/config.hpp) — 规划配置
140. [src/fcs/L4_planning/control_intent.hpp](./src/fcs/L4_planning/control_intent.hpp) — 控制意图（variant 指令）
141. [src/fcs/L4_planning/plan_source.hpp](./src/fcs/L4_planning/plan_source.hpp) — 规划来源
142. [src/fcs/L4_planning/selected_target_snapshot.hpp](./src/fcs/L4_planning/selected_target_snapshot.hpp) — 选中目标快照
143. [src/fcs/L4_planning/target_selection_trace.hpp](./src/fcs/L4_planning/target_selection_trace.hpp) — 目标选择诊断

### 瞄准器

144. [src/fcs/L4_planning/aimer/aimer_systems.hpp](./src/fcs/L4_planning/aimer/aimer_systems.hpp) — 瞄准系统
145. [src/fcs/L4_planning/aimer/aimer_systems.cpp](./src/fcs/L4_planning/aimer/aimer_systems.cpp)
146. [src/fcs/L4_planning/aimer/aimer.hpp](./src/fcs/L4_planning/aimer/aimer.hpp) — 瞄准器
147. [src/fcs/L4_planning/aimer/aimer.cpp](./src/fcs/L4_planning/aimer/aimer.cpp)
148. [src/fcs/L4_planning/aimer/aimer_utils.hpp](./src/fcs/L4_planning/aimer/aimer_utils.hpp)
149. [src/fcs/L4_planning/aimer/armor_target_decider.hpp](./src/fcs/L4_planning/aimer/armor_target_decider.hpp) — 装甲目标决策
150. [src/fcs/L4_planning/aimer/armor_target_decider.cpp](./src/fcs/L4_planning/aimer/armor_target_decider.cpp)
151. [src/fcs/L4_planning/aimer/fsm.hpp](./src/fcs/L4_planning/aimer/fsm.hpp) — 瞄准 FSM
152. [src/fcs/L4_planning/aimer/types.hpp](./src/fcs/L4_planning/aimer/types.hpp)

### 弹道构建 & 云台规划

153. [src/fcs/L4_planning/trajectory_builder.hpp](./src/fcs/L4_planning/trajectory_builder.hpp) — 弹道构建
154. [src/fcs/L4_planning/trajectory_builder.cpp](./src/fcs/L4_planning/trajectory_builder.cpp)
155. [src/fcs/L4_planning/gimbal_planner/types.hpp](./src/fcs/L4_planning/gimbal_planner/types.hpp) — 云台规划类型
156. [src/fcs/L4_planning/common/transform_utils.hpp](./src/fcs/L4_planning/common/transform_utils.hpp) — 坐标变换工具

---

## 阶段九：L5 武器层

157. [src/fcs/L5_weapon/enhanced/weapon_systems.hpp](./src/fcs/L5_weapon/enhanced/weapon_systems.hpp) — 武器系统注册
158. [src/fcs/L5_weapon/enhanced/weapon_systems.cpp](./src/fcs/L5_weapon/enhanced/weapon_systems.cpp)
159. [src/fcs/L5_weapon/enhanced/trajectory_optimizer.hpp](./src/fcs/L5_weapon/enhanced/trajectory_optimizer.hpp) — 轨迹优化器
160. [src/fcs/L5_weapon/enhanced/trajectory_optimizer.cpp](./src/fcs/L5_weapon/enhanced/trajectory_optimizer.cpp)
161. [src/fcs/L5_weapon/enhanced/dual_mpc_osqp_solver.hpp](./src/fcs/L5_weapon/enhanced/dual_mpc_osqp_solver.hpp) — MPC 求解器
162. [src/fcs/L5_weapon/enhanced/dual_small_mpc_solver.hpp](./src/fcs/L5_weapon/enhanced/dual_small_mpc_solver.hpp)
163. [src/fcs/L5_weapon/fire_control.hpp](./src/fcs/L5_weapon/fire_control.hpp) — 火控核心
164. [src/fcs/L5_weapon/fire_decision.hpp](./src/fcs/L5_weapon/fire_decision.hpp) — 开火决策
165. [src/fcs/L5_weapon/config.hpp](./src/fcs/L5_weapon/config.hpp) — 武器层配置

---

## 阶段十：坐标变换库 fast_tf

166. [crates/fast_tf/src/frame.hpp](./crates/fast_tf/src/frame.hpp) — 坐标系 frame 类型
167. [crates/fast_tf/src/types.hpp](./crates/fast_tf/src/types.hpp) — typed pose/vector/transform
168. [crates/fast_tf/src/matrix.hpp](./crates/fast_tf/src/matrix.hpp) — 矩阵与变换运算
169. [crates/fast_tf/src/buffer.hpp](./crates/fast_tf/src/buffer.hpp) — 变换缓冲
170. [crates/fast_tf/src/validation.hpp](./crates/fast_tf/src/validation.hpp) — 坐标变换验证
171. [crates/fast_tf/src/validation.cpp](./crates/fast_tf/src/validation.cpp)
172. [crates/fast_tf/src/foxglove_export.hpp](./crates/fast_tf/src/foxglove_export.hpp) — Foxglove 适配

---

## 阶段十一：硬件驱动

### 云台

173. [crates/hardware/at_gimbal/src/talos_gimbal/mcu_device.hpp](./crates/hardware/at_gimbal/src/talos_gimbal/mcu_device.hpp) — MCU 设备
174. [crates/hardware/at_gimbal/src/talos_gimbal/packet.hpp](./crates/hardware/at_gimbal/src/talos_gimbal/packet.hpp) — 通信数据包
175. [crates/hardware/at_gimbal/src/talos_gimbal/serial.hpp](./crates/hardware/at_gimbal/src/talos_gimbal/serial.hpp) — 串口通信
176. [crates/hardware/at_gimbal/src/talos_gimbal/stm32.hpp](./crates/hardware/at_gimbal/src/talos_gimbal/stm32.hpp) — STM32 协议
177. [crates/hardware/at_gimbal/src/talos_gimbal/usb.hpp](./crates/hardware/at_gimbal/src/talos_gimbal/usb.hpp) — USB 通信

### HIK 相机

178. [crates/hardware/hik_camera_driver/src/hik_camera.hpp](./crates/hardware/hik_camera_driver/src/hik_camera.hpp) — HIK 相机驱动
179. [crates/hardware/hik_camera_driver/src/hik_camera.cpp](./crates/hardware/hik_camera_driver/src/hik_camera.cpp)
180. [crates/hardware/hik_camera_driver/src/hik_camera_stub.cpp](./crates/hardware/hik_camera_driver/src/hik_camera_stub.cpp) — 空实现桩

### Daedalus 模拟器

181. [crates/hardware_daedalus/src/shm_layout.hpp](./crates/hardware_daedalus/src/shm_layout.hpp) — 共享内存布局
182. [crates/hardware_daedalus/src/shm_region.hpp](./crates/hardware_daedalus/src/shm_region.hpp) — 共享内存区域
183. [crates/hardware_daedalus/src/shm_client.hpp](./crates/hardware_daedalus/src/shm_client.hpp) — 共享内存客户端
184. [crates/hardware_daedalus/src/shm_triple_buffer.hpp](./crates/hardware_daedalus/src/shm_triple_buffer.hpp) — 共享内存三重缓冲

---

## 阶段十二：其余 crates

### 数学

185. [crates/math/src/so2.hpp](./crates/math/src/so2.hpp) — SO(2) 旋转群
186. [crates/math/src/euler.hpp](./crates/math/src/euler.hpp) — 欧拉角

### TOML 配置

187. [crates/toml/src/toml_helper.hpp](./crates/toml/src/toml_helper.hpp) — TOML 辅助
188. [crates/toml/src/toml/core.hpp](./crates/toml/src/toml/core.hpp) — TOML 核心
189. [crates/toml/src/toml/type_wrappers.hpp](./crates/toml/src/toml/type_wrappers.hpp) — 类型包装
190. [crates/toml/src/field_reflection.hpp](./crates/toml/src/field_reflection.hpp) — 编译期字段反射

### 日志

191. [crates/log/src/spdlog_hook.hpp](./crates/log/src/spdlog_hook.hpp) — 日志钩子

### Quanta 视频流

192. [crates/quanta/src/quanta/stream_transport.hpp](./crates/quanta/src/quanta/stream_transport.hpp) — 视频流传输
193. [crates/quanta/src/quanta/stream_transport.cpp](./crates/quanta/src/quanta/stream_transport.cpp)
194. [crates/quanta/src/quanta/stream_encoder.hpp](./crates/quanta/src/quanta/stream_encoder.hpp) — 视频流编码
195. [crates/quanta/src/quanta/stream_encoder.cpp](./crates/quanta/src/quanta/stream_encoder.cpp)

### 手性采集

196. [crates/chiral/src/chiral/chiral_endpoint.hpp](./crates/chiral/src/chiral/chiral_endpoint.hpp)
197. [crates/chiral/src/chiral/shm_layout.hpp](./crates/chiral/src/chiral/shm_layout.hpp) — 手性共享内存布局

---

## 阶段十三：可视化

198. [src/fcs_visualization/foxglove_server.hpp](./src/fcs_visualization/foxglove_server.hpp) — Foxglove 服务
199. [src/fcs_visualization/foxglove_server.cpp](./src/fcs_visualization/foxglove_server.cpp)
200. [src/fcs_visualization/foxglove_sink.hpp](./src/fcs_visualization/foxglove_sink.hpp) — 数据接收器
201. [src/fcs_visualization/foxglove_systems.hpp](./src/fcs_visualization/foxglove_systems.hpp) — 可视化系统注册
202. [src/fcs_visualization/foxglove_systems.cpp](./src/fcs_visualization/foxglove_systems.cpp)
203. [src/fcs_visualization/foxglove_types.hpp](./src/fcs_visualization/foxglove_types.hpp) — 可视化类型
204. [src/fcs_visualization/scene_builder.hpp](./src/fcs_visualization/scene_builder.hpp) — 场景构建
205. [src/fcs_visualization/scene_builder.cpp](./src/fcs_visualization/scene_builder.cpp)
206. [src/fcs_visualization/system_helpers.hpp](./src/fcs_visualization/system_helpers.hpp)
207. [src/fcs_visualization/tactical_palette.hpp](./src/fcs_visualization/tactical_palette.hpp) — 战术配色
208. [src/fcs_visualization/utility.hpp](./src/fcs_visualization/utility.hpp)
209. [src/fcs_visualization/systems/base.hpp](./src/fcs_visualization/systems/base.hpp) — 可视化系统基类

---

## 阶段十四：测试

210. [src/fcs/tests/config_parse_test.cpp](./src/fcs/tests/config_parse_test.cpp)
211. [src/fcs/tests/tracker_manager_test.cpp](./src/fcs/tests/tracker_manager_test.cpp)
212. [src/fcs/tests/trajectory_optimizer_test.cpp](./src/fcs/tests/trajectory_optimizer_test.cpp)
213. [src/fcs/tests/ldm_detector_solver_test.cpp](./src/fcs/tests/ldm_detector_solver_test.cpp)
214. [src/fcs/tests/aimer_phase_test.cpp](./src/fcs/tests/aimer_phase_test.cpp)
215. [src/fcs/tests/at_legacy_pnp_solver_test.cpp](./src/fcs/tests/at_legacy_pnp_solver_test.cpp)
216. [src/fcs/tests/at_legacy_readback_roi_test.cpp](./src/fcs/tests/at_legacy_readback_roi_test.cpp)
217. [src/fcs/tests/capturer_test.cpp](./src/fcs/tests/capturer_test.cpp)
218. [src/fcs/tests/ldm_config_test.cpp](./src/fcs/tests/ldm_config_test.cpp)
219. [src/fcs/tests/ldm_inekf_test.cpp](./src/fcs/tests/ldm_inekf_test.cpp)
220. [src/fcs/tests/read_config_test.cpp](./src/fcs/tests/read_config_test.cpp)
221. [crates/scheduler/tests/scheduler_ecs_test.cpp](./crates/scheduler/tests/scheduler_ecs_test.cpp) — 调度器 ECS 测试
222. [crates/scheduler/tests/triple_buffer_test.cpp](./crates/scheduler/tests/triple_buffer_test.cpp) — 三重缓冲测试
223. [crates/scheduler/tests/primitives_test.cpp](./crates/scheduler/tests/primitives_test.cpp)

---

## 阶段十五：调试入口

224. [src/playground.cpp](./src/playground.cpp) — 调试沙盒

---

## 附录：关键概念速查

| 概念 | 一句话解释 | 代码位置 |
|------|-----------|---------|
| **System** | 声明依赖的计算单元，是调度器调度的基本单位 | [#16](./crates/scheduler/src/scheduler/system/system.hpp) |
| **World** | 全局资源+通道的容器，初始化后冻结 | [#13](./crates/scheduler/src/scheduler/world.hpp) |
| **Channel** | System 间无锁通信管道（SPSC/SPMC 三重缓冲） | [#26](./crates/primitive/src/primitive/channel.hpp) |
| **Resource** | 全局共享数据（只读 res / 读写 res_mut），带版本号 | [#18](./crates/scheduler/src/scheduler/system/components.hpp) |
| **boot()** | 唯一初始化入口：注入资源→注册系统→冻结拓扑 | [#10](./src/fcs/runtime/boot.cpp) |
| **Topic** | 通道的类型安全标签（空 struct），防止通道误用 | [#35](./src/fcs/core/channel_topics.hpp) |
| **Variant** | 互斥状态表达（如多种硬件后端、多种指令类型） | 全项目广泛使用 |
| **Expected** | 错误传播方式（替代异常/错误码） | 全项目广泛使用 |
| **fast_tf** | 类型安全坐标变换库，防止坐标系误用 | [阶段十](#阶段十坐标变换库-fast_tf) |

---

> **学习建议：**
> 1. 先理解调度器（阶段二），再理解业务（阶段五~九）
> 2. 从 `channel_topics.hpp`（#35）入手理解数据流骨架
> 3. 从 `boot.cpp`（#10）追踪系统启动顺序
> 4. 关注 `variant` 和 `expected` 两种核心类型模式
> 5. 善用 Foxglove 可视化理解运行时数据流