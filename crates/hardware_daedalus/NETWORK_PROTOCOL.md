# 跨机仿真网络协议 v1（Rust 仿真端对接文档）

Windows Rust 仿真 ↔ Linux NUC 上的 `talos_ipc_bridge`，替代原来的 mmap 共享内存。

## 部署与端口

```text
Windows 仿真                              NUC (Linux)
┌──────────────────┐    网线局域网    ┌─────────────────────┐
│ UdpSocket ───────│──UDP:9571──────▶│ talos_ipc_bridge    │──▶ /tmp/talos_ipc_meta
│            ◀─────│──UDP 源地址回传──│ (9571 UDP+9572 TCP) │──▶ /tmp/talos_ipc_image_pool
│ TcpStream ───────│──TCP:9572──────▶│                     │
└──────────────────┘                 └─────────────────────┘
```

- **先启动桥，再启动 talos 主程序**（桥创建共享内存，talos 照常 `ShmClient::connect()`）
- 桥启动参数：`talos_ipc_bridge [控制端口=9571] [图像端口=9572]`
- 仿真 UDP socket **只建一个**：发数据用 `send_to(NUC_IP:9571)`，同时在这个 socket 上
  `recv` 云台指令回传（桥按收到包的源地址回发）
- 两机静态 IP 同网段即可（如 `192.168.2.1/24`），无需网关

## 字节序与结构体

双方均为小端（x86/ARM），直接 `memcpy` 结构体字节，**不做网络字节序转换**。
载荷结构体与 `shm_layout.hpp` 完全一致（Rust 端沿用原共享内存的 repr(C) 定义）：
`PoseMeta(64B)`、`ChassisObservation(128B)`、`GroundTruthBatch(1664B)`、
`CameraInfo(128B)`、`RuntimeState(64B)`、`GimbalCmd(32B)`。

## UDP 控制包（仿真 → 桥，端口 9571）

```text
┌─ PacketHeader 32B ────────────────┐┌─ payload ─────┐
│ magic    u32 = 0x54414C06         ││ 结构体字节     │
│ version  u16 = 1                  ││               │
│ payload_len u16                   ││               │
│ type     u8                       ││               │
│ index    u8  （Pose 包填通道号）   ││               │
│ _pad     u16 + _pad2 u32          ││               │
│ seq      u64 （发送计数，诊断用）  ││               │
│ timestamp_ns u64                  ││               │
└───────────────────────────────────┘└───────────────┘
```

| type | payload | 发送时机 |
|---|---|---|
| 1 Pose | PoseMeta 64B | 每渲染帧×5路（index：0云台 1odom 2枪口 3相机 4底盘观测） |
| 2 ChassisObserv | ChassisObservation 128B | 每渲染帧 |
| 3 GroundTruth | GroundTruthBatch 1664B | 每渲染帧（单包 <64KB，无需分片） |
| 4 CameraInfo | CameraInfo 128B | 启动时 + 变更时 |
| 5 RuntimeState | RuntimeState 64B | 状态变更时 |
| 6 Heartbeat | 空 | ≥10Hz 保活（同时让桥记录回传地址） |

> 桥会把所有 timestamp_ns **改写为 NUC 本机时钟**后落地，统一时域，无需 NTP 对时。

## 云台指令回传（桥 → 仿真，UDP 发到仿真源地址）

同一个 32B `PacketHeader`（`type=7`）+ `GimbalCmd 32B`。
仿真在自己发送用的 UDP socket 上 `recv` 即可；桥以 200Hz 轮询转发。

## TCP JPEG 图像流（仿真 → 桥，端口 9572）

一帧两段式写入 `TcpStream`：

```text
┌─ ImageFrameHeader 40B ────┐┌─ JPEG 数据 ────────┐
│ magic u32 = 0x54414C06    ││ jpeg_len 字节      │
│ type  u8 = 8              ││ （RGB 帧编码，     │
│ _pad   [3]                ││  质量 70~85 足够） │
│ width u32 = 1440          ││                    │
│ height u32 = 1080         ││                    │
│ jpeg_len u32              ││                    │
│ _pad2  u32                ││                    │
│ seq   u64                 ││                    │
│ timestamp_ns u64          ││                    │
└───────────────────────────┘└────────────────────┘
```

硬性约束：**分辨率必须恰好 1440×1080**（桥按固定图像池布局落地，其他尺寸丢弃）。
JPEG 用 `image` crate 的 `JpegEncoder::new_with_quality(80)` 或 `turbojpeg`。
断线后桥自动等重连，仿真直接重连 `TcpStream::connect` 即可。

## Rust 侧最小改造示例

```rust
let sock = UdpSocket::bind("0.0.0.0:0")?;
let nuc = "192.168.2.200:9571";
// 位姿（其余通道同型）
let mut pkt = [0u8; 32 + 64];
fill_header(&mut pkt[..32], 1 /*Pose*/, pose_index as u8, 64, seq);
pkt[32..].copy_from_slice(bytemuck::bytes_of(&pose_meta));
sock.send_to(&pkt, nuc)?;
// 云台指令回传（同 socket 非阻塞收）
sock.set_read_timeout(Some(Duration::from_millis(1)))?;
if let Ok((cmd_pkt, _)) = sock.recv_from(&mut buf) { parse_gimbal_cmd(&buf[..cmd_pkt]); }
// 图像
let mut tcp = TcpStream::connect("192.168.2.200:9572")?;
tcp.write_all(&frame_header)?;
tcp.write_all(&jpeg_bytes)?;
```

## 验证清单

1. `ping` 通 → 2. NUC 上先跑 `talos_ipc_bridge` → 3. 仿真启动发包
4. 桥终端出现 `image sender connected` 与 `images: N` 计数
5. SSH 另开终端启动 `talos`，Foxglove 里 `/image` 出仿真画面、3D 场景随位姿动
