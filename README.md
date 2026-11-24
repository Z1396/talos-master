<div align="center">

# Talos

**RoboMaster 上位机自瞄**

_如同守卫克里特岛的青铜巨人，以钢铁之躯承载自瞄意志。_

[![C++23](https://img.shields.io/badge/C%2B%2B-23-00599C.svg?style=for-the-badge&logo=cplusplus)](https://isocpp.org/)
[![OpenCV](https://img.shields.io/badge/opencv-4.x-5C3EE8.svg?style=for-the-badge&logo=opencv)](https://opencv.org/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg?style=for-the-badge)](LICENSE)

</div>

---

## Talos

Talos 是一个 RoboMaster 自瞄上位机。

它把传感输入、目标感知、状态估计、瞄准规划、开火决策和控制输出组织为一条火控流水线。

系统由运行时、调度器、视觉模块、配置系统、可视化链路、模拟器接口和图传组件共同组成。

## 架构

Talos 使用五级火控结构。

| 阶段 | 模块 | 职责 |
| :---: | --- | --- |
| **L1** | Sensor | 传感输入与设备接入 |
| **L2** | Perception | 目标识别与观测提取 |
| **L3** | Estimation | 状态估计与目标建模 |
| **L4** | Planning | 瞄准决策与解算规划 |
| **L5** | Weapon | 开火决策与控制输出 |

## 运行时

Talos 的运行时围绕资源和系统构建。

`System` 声明自己读取和写入的资源。
调度器根据这些声明建立执行关系。
数据更新后，相关系统被驱动执行。

```text
Resource ──read──▶ System ──write──▶ Resource
```

这种结构用于保持模块边界、数据依赖和执行顺序一致。

## 平台

| 平台                        |        架构        | 工具链                                  | 状态 |
| ------------------------- | :--------------: | ------------------------------------ | -- |
| **PC**                    |      x86_64      | LLVM clang-21 + libstdc++13          | ✅  |
| **NVIDIA Jetson Orin NX** |      aarch64     | LLVM clang-21 + libstdc++13          | ✅  |
| **Axera AX650N**          |      aarch64     | LLVM clang-21 + libstdc++13          | ✅  |
| **Mac**                   | aarch64 / x86_64 | Apple Clang + libc++                 | ✅  |
| **Android / Termux**      |      aarch64     | LLVM clang-21 + libc++ / bionic libc | 🧪 |

> ✅：已验证
> 🧪：实验性支持

## 依赖

| 依赖                                                | 版本       | 用途         |
| ------------------------------------------------- | -------- | ---------- |
| [CMake](https://cmake.org)                        | >= 3.14  | 构建系统       |
| [oneTBB](https://uxlfoundation.github.io/oneTBB)  | 2021.9+  | 任务调度与线程池   |
| [Eigen3](https://libeigen.gitlab.io)              | 3.4+     | 线性代数       |
| [OpenCV](https://opencv.org)                      | 4.x      | 图像处理       |
| [Ceres](https://ceres-solver.org)                 | 2.x      | 非线性优化与自动微分 |
| [FFmpeg](https://ffmpeg.org)                      | 8.x      | 图传编码与解码    |
| [ONNXRuntime](https://onnxruntime.ai)              | >= 1.26 | 推理后端       |
| [TensorRT](https://developer.nvidia.com/tensorrt) | optional | 推理后端       |

## 快速开始

```bash
bash build.sh
```

程序从入口配置加载运行时、视觉算法和机器人硬件配置。

## 配置

Talos 使用 TOML 作为配置格式。

```text
at_vision.toml              # 入口配置
config/vision_base.toml     # 视觉基础配置
config/vision/<name>.toml   # 视觉算法配置
config/robot/<name>.toml    # 机器人硬件配置
```

配置按覆盖语义合并。
更具体的配置覆盖基础配置。

## 可视化

Talos 集成 Foxglove，用于观察图像、识别结果、目标状态、规划信息与运行时调试数据。

## 模拟器

Talos 可与 Daedalus 协同工作。

[Daedalus](https://github.com/Blackjack200/bevy_robomaster_simulator)

## 状态

Talos 正在迭代。

## 致谢

Talos 的开发受到以下项目启发：

* [bevy](https://bevyengine.org/) - System 调度模型
* [oneTBB](https://uxlfoundation.github.io/oneTBB) - 任务调度与并行执行
* [toml++](https://marzer.github.io/tomlplusplus/) - TOML 解析
* [Foxglove](https://foxglove.dev/) - 机器人可视化

感谢以下战队与开发者的开源分享（排名不分先后）：

* 同济大学 SuperPower 战队 - [sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25)
* 华南师范大学 PIONEER 陈君 - [rm_vision](https://github.com/chenjunnn/rm_vision)
* 中南大学 FYT 战队 - [FYT2024_vision](https://github.com/CSU-FYT-Vision/FYT2024_vision)
* 武汉科技大学 崇实战队 - [wust_vision](https://github.com/WUST-RM/wust_vision)
* 沈阳航空航天大学 T-UP 战队 - [TUP-InfantryVision-2022](https://github.com/TUP-RM/TUP-InfantryVision-2022)

感谢 RoboMaster 社区长期以来的工程探索与知识传承。

## 本仓库新增功能

以下功能由 [@Z1396](https://github.com/Z1396) 在原项目基础上新增或改进，原作者与上游项目的贡献保持不变：

* **ImuOnly 调试后端**：无需相机，直接按配置波形生成云台指令并下发 MCU，用于调试伺服响应；支持小陀螺、左右平移、边走边转三种模式，默认禁止开火。
* **机动自适应 EKF 过程噪声**：检测到目标机动（新息异常、速度跳变、速度过零）时临时放大过程噪声与速度协方差，改善目标反向瞬间的预测滞后，并提供新息、触发原因等诊断可视化。
* **角速度 / 角加速度前馈**：云台指令增加一阶、二阶前馈补偿，提升高速运动目标的跟踪响应。
* **OpenVINO GPU 推理**：ONNXRuntime 接入 OpenVINO GPU 后端，降低推理耗时。
* **距离分档开火容差**：按目标距离调整允许开火的误差门限，减少远距离误开火。
* **图像与位姿按帧号配对同步**：保证检测图像与云台位姿在时间上对齐。
* **全链路延迟补偿修正**：修正时间戳类型、后端配置、射击容差与延迟补偿。
* **检测帧率优化**：调整输入分辨率、ROI、后处理与推理配置，检测帧率由约 12–19Hz 提升至约 31Hz。
* **瞄准状态机与弹道解算优化**：完善瞄准阶段切换与弹道求解逻辑。
* **跨机仿真网络桥**：新增仿真数据桥接与可视化坐标轴。
* **调试可视化**：传统装甲检测二值化、类型识别调试帧、装甲调试可视化等。
* **相机与仿真修复**：修复仿真模式下相机分辨率为 0 导致编码器启动失败等问题。
* **代码注释与文档**：为各分层模块补充详细注释，提升可读性。

> 本段仅记录本仓库新增内容，原项目介绍、架构与致谢保持原样。

---

<div align="center">

**用 🪓 和 💀 构建**

</div>
