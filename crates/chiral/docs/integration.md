# Chiral 开发者对接手册

本文档面向外部对接开发者。`chiral` 提供基于 POSIX 共享内存的双向 IPC 端点，采用 SPSC 三重缓冲 + 序列锁保证无撕裂读写。

---

## 1. 架构概览

```
  Talos 侧                              外部模块侧
┌──────────────┐                    ┌──────────────────┐
│ TalosEndpoint│  ──write(T)──►    │ Navigation/       │
│              │  ◄──read()───     │ GimbalEndpoint    │
│              │                    │                   │
│  writes Out  │  共享内存 (SHM)    │  reads Incoming   │
│  reads Inc   │◄════════════════► │  writes Out       │
│              │                    │  reads Inc        │
└──────────────┘                    └──────────────────┘
```

每个 `ChiralEndpoint<Outgoing, Incoming>` 持有两个单向通道：

- **Outgoing**（Writer）：本端创建 SHM 并写入
- **Incoming**（Reader）：按需打开对端 SHM 并读取，支持对端重启后自动重连

---

## 2. 源文件与头文件

| 头文件 | 内容 |
|--------|------|
| `chiral/shm_layout.hpp` | `ShmRegion` (RAII SHM 区域)、`ShmHeader`、`ShmLayout<T>`、`ShmError`、`ShmName<T>` |
| `chiral/shm_triple_buffer.hpp` | `TripleBufferLayout<T>` (三槽布局 + 序列锁)、`TripleBufferOps` (原子操作) |
| `chiral/chiral_endpoint.hpp` | `ChannelWriter<T>`、`ChannelReader<T>`、`ChiralEndpoint<Out, Inc>` |
| `chiral/navigation.hpp` | 导航域数据类型 (`TalosData`, `NavigationData`) + 端点别名 + SHM 名称特化 |
| `chiral/gimbal.hpp` | 云台域数据类型 (`McuData`, `McuRequest`) + 端点别名 + SHM 名称特化 |

外部对接只需包含对应的域头文件：

- 导航对接：`#include "chiral/navigation.hpp"`
- 云台对接：`#include "chiral/gimbal.hpp"`

---

## 3. 两个对接域

### 3.1 导航域 (Navigation)

命名空间：`talos::chiral::navigation`

| 端点 | 类型 | Talos 侧写入 | Talos 侧读取 |
|------|------|-------------|-------------|
| `TalosEndpoint` | `ChiralEndpoint<TalosData, NavigationData>` | `TalosData` | `NavigationData` |
| `NavigationEndpoint` | `ChiralEndpoint<NavigationData, TalosData>` | `NavigationData` | `TalosData` |

SHM 名称：

| 数据类型 | SHM 对象名 |
|----------|-----------|
| `TalosData` | `/chiral_nav_talos` |
| `NavigationData` | `/chiral_nav_navigation` |

### 3.2 云台域 (Gimbal)

命名空间：`talos::chiral::gimbal`

| 端点 | 类型 | Talos 侧写入 | Talos 侧读取 |
|------|------|-------------|-------------|
| `TalosEndpoint` | `ChiralEndpoint<McuRequest, McuData>` | `McuRequest` | `McuData` |
| `GimbalEndpoint` | `ChiralEndpoint<McuData, McuRequest>` | `McuData` | `McuRequest` |

SHM 名称：

| 数据类型 | SHM 对象名 |
|----------|-----------|
| `McuRequest` | `/chiral_gimbal_request` |
| `McuData` | `/chiral_gimbal_data` |

---

## 4. 数据结构

### 4.1 导航域 — TalosData

```cpp
namespace talos::chiral::navigation {

enum TargetStateKind : uint8_t { Robot = 0, Outpost = 1 };

enum class TrackerStatus : uint8_t { Idle = 0, Detecting = 1, Tracking = 2, TempLost = 3 };
enum class ArmorColor : uint8_t { Blue = 0, Red = 1, Neutral = 2, Purple = 3 };
enum class ArmorName : uint8_t { Sentry = 0, One, Two, Three, Four, Five, Outpost, Base, BaseLarge, Invalid };

struct TalosData {
    TargetStateKind state_kind;
    TargetState     state;        // 包含 robot / outpost 子状态
    Transform<odom, gimbal_yaw>   gimbal_link;
    Transform<gimbal_yaw, muzzle> muzzle_link;
    Transform<gimbal_yaw, camera> camera_link;
};

struct NavigationData {
    uint64_t timestamp_ns = 0;
};

} // namespace talos::chiral::navigation
```

`TargetState` 包含：

- 目标元信息：`status`, `color`, `name`
- `RobotState`：位置、速度、yaw、角速度、装甲板半径、高度
- `OutpostState`：位置、速度、yaw、角速度、z 系数

### 4.2 云台域 — McuData / McuRequest

```cpp
namespace talos::chiral::gimbal {

enum class Color : uint8_t { Red = 0, Blue = 1 };

struct McuData {
    int64_t timestamp_ns_system_clock;
    Color  self_color;
    float  bullet_speed;
    float  yaw, pitch, roll;
    float  yaw_vel, pitch_vel, roll_vel;
};

struct McuRequest {
    int64_t timestamp_ns_system_clock;
    bool   fire_advice;
    float  target_yaw, target_pitch;
    float  ref_yaw_v, ref_pitch_v;
    float  ref_yaw_a, ref_pitch_a;
    float  distance;
};

} // namespace talos::chiral::gimbal
```

单位约定：

- 长度：米
- 角度：弧度
- 角速度：弧度/秒

---

## 5. 接入要求

- C++23（建议 `-std=c++2b`）
- 编译时包含头文件目录：`crates/chiral/src`
- 并发模型：SPSC（单 Writer + 单 Reader，每方向独立）
- 依赖：`fmt`、`magic_enum`（仅云台域）

---

## 6. API 用法

### 6.1 创建端点

```cpp
#include "chiral/navigation.hpp"
using namespace talos::chiral::navigation;

// Talos 侧
auto talos = TalosEndpoint::create();   // returns expected<unique_ptr<TalosEndpoint>, ShmError>

// 外部导航模块侧
auto remote = NavigationEndpoint::create();
```

- `create()` 创建 outgoing 方向的 SHM 并写入 header
- incoming 方向的 SHM 在首次 `read_new()` / `read_latest()` 时按需打开

### 6.2 读写

```cpp
// 写入
TalosData data{};
data.gimbal_link.translation.x = 1.0;
talos->write(data);                         // void, noexcept

// 读取新数据
if (auto inbound = talos->read_new()) {     // optional<NavigationData>
    // 消费 *inbound
}

// 读取最新快照
auto latest = talos->read_latest();         // optional<NavigationData>
```

启动顺序无关：任一侧均可先 `create()`。Reader 侧在对端启动前 `read_new()` 返回 `std::nullopt`。

### 6.3 对端重启自动重连

`ChiralEndpoint` 内部维护 lazy reader，检测到对端 SHM 被 unlink 后自动重新 open：

```cpp
auto talos = TalosEndpoint::create();

{
    auto remote = NavigationEndpoint::create();
    remote->write(NavigationData{.timestamp_ns = 1});
    auto r1 = talos->read_new();  // 有数据
}
// remote 析构，SHM 被 unlink

auto r2 = talos->read_new();      // nullopt（对端已断）

auto remote2 = NavigationEndpoint::create();
remote2->write(NavigationData{.timestamp_ns = 2});
auto r3 = talos->read_new();      // 自动重连，读到 2
```

### 6.4 最小外部 Reader 示例（导航域）

```cpp
#include "chiral/navigation.hpp"
#include <chrono>
#include <iostream>
#include <thread>

int main() {
    using namespace talos::chiral::navigation;

    // 外部导航模块：写入 NavigationData，读取 TalosData
    auto remote = NavigationEndpoint::create();
    if (!remote) {
        std::cerr << "create failed, err=" << static_cast<int>(remote.error()) << "\n";
        return 1;
    }

    for (int i = 0; i < 100; ++i) {
        if (auto data = (*remote)->read_new()) {
            std::cout << "status=" << static_cast<int>(data->state.status)
                      << " x=" << data->state.robot.position.x << "\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
```

---

## 7. API 语义

| 方法 | 返回类型 | 语义 |
|------|---------|------|
| `create()` | `expected<unique_ptr<ChiralEndpoint>, ShmError>` | 创建 outgoing SHM |
| `write(const T&)` | `void` | 写入一个完整快照并发布 |
| `read_new()` | `optional<T>` | 有新数据返回快照拷贝；无新数据返回 `nullopt` |
| `read_latest()` | `optional<T>` | 返回当前最新稳定快照拷贝（即使已消费过） |

TOCTOU 约束（对端点级 API）：

- 端点公共 API 不暴露 `has_new_data()`
- 必须直接调用 `read_new()` 并以 optional 结果判定

唯一允许模式：

```cpp
if (auto data = endpoint->read_new()) {
    // 消费 *data
}
```

---

## 8. 无撕裂保证

三重缓冲每个槽位都有序列锁 (`slot_seq[i]`)：

1. Writer 写入前递增 `slot_seq` 为奇数（写入中）
2. Writer 写完后递增 `slot_seq` 为偶数（稳定）
3. Reader 读取时做前后两次 `slot_seq` 一致性校验
4. 仅在一致且偶数时返回快照

因此 Reader 不会拿到"半写入数据"。

SHM 布局：

```
ShmHeader (64 bytes, alignas 64)
├── magic:    0x544C4454 ("TLDT")
├── version:  2
└── padding
TripleBufferLayout<T>
├── state:        atomic<uint8_t>  (bit7=FLAG_NEW, bit0-1=ready_idx)
├── write_idx:    uint8_t
├── read_idx:     uint8_t
├── _pad
├── slot_seq[3]:  atomic<uint64_t> (序列锁)
└── slots[3]:     T (三个数据槽位)
```

---

## 9. 错误码

```cpp
enum class ShmError : uint8_t {
    OpenFailed      = 0,  // shm_open 创建失败
    TruncateFailed  = 1,  // ftruncate 失败
    MapFailed       = 2,  // mmap 失败
    NotFound        = 3,  // Reader 打开时对端尚未创建
    InvalidMagic    = 4,  // magic 不匹配
    VersionMismatch = 5,  // 协议版本不匹配
};
```

排障建议：

1. 确认对端已运行并 `create()` 成功
2. 确认读写双方使用同一版本头文件
3. 清理残留 SHM 对象后重试：
   - 导航域：`shm_unlink("/chiral_nav_talos")` + `shm_unlink("/chiral_nav_navigation")`
   - 云台域：`shm_unlink("/chiral_gimbal_request")` + `shm_unlink("/chiral_gimbal_data")`
4. 若改过数据类型定义，读写端必须一起重新编译

---

## 10. 验收命令

在仓库根目录执行：

```bash
cmake --build build --target chiral_endpoint_test triple_buffer_simple_test -j 8
./build/bin/chiral_endpoint_test
./build/bin/triple_buffer_simple_test
```

> 注意：两个测试操作相同的命名 SHM 端点，不可并行执行。CMake 已通过 `RESOURCE_LOCK chiral_shm` 保证串行。

通过标准：

- 全部测试 PASS
- 无 `Data tearing detected` 报告

---

## 11. 兼容性策略

- 当前尚未发布 v1，接口仍处于 pre-v1 阶段，允许破坏性调整
- 数据结构（`TalosData`、`McuData` 等）视为协议结构体，字段变更属于协议变更
- 对接方建议固定版本（commit/tag）以避免 ABI 偏差

---

## 12. ShmName 扩展指南

定义新的对接域需要为数据类型特化 `ShmName<T>`：

```cpp
// 在 talos::chiral::ipc 命名空间中特化
namespace talos::chiral::ipc {

template <>
struct ShmName<MyDataType> {
    static constexpr const char* value = "/chiral_my_domain_data";
};

} // namespace talos::chiral::ipc
```

然后用 `ChiralEndpoint` 组合双向通道：

```cpp
using MyTalosSide  = ipc::ChiralEndpoint<MyOutgoing, MyIncoming>;
using MyRemoteSide = ipc::ChiralEndpoint<MyIncoming, MyOutgoing>;
```
