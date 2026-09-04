# stage13_shm_ipc —— 共享内存 IPC：仿真程序零拷贝协作

对齐真实模块：`crates/hardware_daedalus/src/`（4 个 .hpp），demo 直接
`#include` 项目真实头文件，**零复制**。这套 IPC 是 Talos（C++ 视觉自瞄）与
Daedalus（Rust 仿真器）两个独立进程互相协作的全部通信通道。

## 通信模型

```
Daedalus 仿真器 (Rust, 生产者)                Talos 视觉程序 (C++, 消费者)
  memmap2 创建 /tmp 两块文件      ──────►      ShmClient::connect()
  ├─ talos_ipc_meta      3,712 B  控制区       ├─ 魔数/版本校验 → 订阅
  └─ talos_ipc_image_pool 13.4 MB 像素池       ├─ recv_image()  零拷贝 cv::Mat
                                               └─ send_gimbal_cmd() 回发云台指令
```

数据流：仿真器发布**图像 / 5 路位姿 / 真值 / 心跳**，视觉端消费后回发
**云台指令**（yaw/pitch/距离/开火建议），形成"仿真 → 算法 → 控制 → 仿真"闭环。

## 功能分析

| 文件 | 核心功能 | 关键技术点 |
|------|----------|-----------|
| `shm_region.hpp` | `/tmp` 文件式 mmap RAII | `create`/`open`/`create_or_open` 三模式；**owner 标记**决定析构是否 unlink（消费者析构不删文件）；move-only 防双重 munmap；`std::expected` + fmt/magic_enum 特化直接打印错误枚举 |
| `shm_layout.hpp` | 跨 C++/Rust ABI 内存布局 | 每个结构体 `alignas` + 手动 `_pad` + `static_assert` 锁死大小与偏移（`ShmMetaRegion` 必须 3,712 B）；三缓冲 `state` 原子量独占 64B 缓存行防伪共享 |
| `shm_triple_buffer.hpp` | SPSC 无锁三缓冲 | `uint8_t state` 位编码：bit7=`FLAG_NEW`、bit0-1=就绪槽下标；发布用 `exchange`、借用用 CAS（失败最多重试 2 次，生产者过快则放弃本帧=latest-wins）；全程无锁无系统调用 |
| `shm_client.hpp` | 双端门面 | `connect()` 消费者 / `create()` 生产者；零拷贝 `recv_image`（cv::Mat 直接指向共享内存像素，**下次 recv 后失效**）；心跳 + `wait_for_producer` 防连到残留旧内存 |

### 为什么是"文件 mmap"而不是 shm_open / Unix socket

- **跨语言**：Rust `memmap2` 就是文件映射，`/tmp/名字` 两端天然对齐；
  POSIX `shm_open`（`/dev/shm`）语义相同但 Rust 端要额外依赖
- **零拷贝大图像**：1440×1080×3 ≈ 4.5 MB/帧，socket 要拷贝两次；共享内存
  消费者直接读像素地址
- **latest-wins 语义**：视觉流只关心最新帧，三缓冲天然丢弃被覆盖的旧帧，
  不会像队列那样阻塞生产者

### 三缓冲状态位编码

```
state (atomic<uint8_t>):  [ FLAG_NEW | ready_idx(2bit) ]     初始 = 0b0000_0001
publish():  exchange(write_idx | FLAG_NEW) → 旧 state 的低 2 bit 变成新 write_idx
borrow():   有 FLAG_NEW 才 CAS 拿走 ready_idx，把自己的 read_idx 还回去
→ 一次原子交换同时完成"发布 + 换槽"，生产者永远不等待消费者
```

## 依赖说明

- **hardware_daedalus 真实头文件**：`target_include_directories` 指向
  `../../crates/hardware_daedalus/src`（SYSTEM 属性：头文件内部告警不传染 demo 的 -Werror）
- **fmt 12.0.0**：`3dparty/fmt-12.0.0.zip` FetchContent（`shm_region.hpp` 的
  `formatter<ShmError>` 需要 fmt≥10 的 const format 签名，系统 fmt 8 不兼容）
- **magic_enum**：仓库自带 `3dparty/magic_enum` 单头文件
- **OpenCV core**：`find_package`（系统 4.5.5 即可，`shm_client.hpp` 需要 cv::Mat）

## 运行方法

```bash
cd learning_practice/stage13_shm_ipc
cmake -B build && cmake --build build
./build/ipc_demo          # 单进程三层自测，全部通过退出码 0
```

> **构建类型注意**：`ipc_demo` 用 `assert` 断言，Release 构建默认 `-DNDEBUG`
> 会把断言整个编译掉导致"假通过"。CMakeLists 已对 Release 注入 `-UNDEBUG`
> 强制保留断言，任意构建类型均可放心使用。

### 双进程协作演示（本 stage 核心）

```bash
# 终端 1：仿真程序（生产者，30fps 合成图像 + 正弦位姿 + 心跳）
./build/sim_producer

# 终端 2：视觉程序（消费者，零拷贝收帧 + 假自瞄 P 控制 + 回发云台指令）
./build/vision_consumer
```

**必须终端 1 先启动**：消费者 `connect()` 要求共享内存已存在，否则报
`NotFound` 退出。

真实闭环验证点：
- 消费者统计"收帧 N = 下发指令 N"，生产者同步回显"收到指令 N 条"，两端计数一致
- 消费者 60fps 轮询 > 生产者 30fps 发布 → 丢帧 0；反向压测（消费者 sleep）
  可观察到 seq 跳变计入"丢帧"——latest-wins 的正常表现
- 生产者 Ctrl+C 退出 → 消费者 1 秒内打印"心跳超时"告警并退出

### 已知坑点：kill -9 生产者会残留共享文件

`ShmRegion` 靠 **owner 析构** 删除 `/tmp/talos_ipc_*`。`kill -9`（或崩溃）
跳过析构，文件残留且**魔数/版本校验完全合法**——消费者随后 connect 会成功，
然后卡在 `wait_for_producer()` 直到 10s 心跳超时才退出。手动清理：

```bash
rm -f /tmp/talos_ipc_meta /tmp/talos_ipc_image_pool
```

真实项目对应缓解手段就是 `wait_for_producer` 这道闸：宁可超时退出，
也不消费僵尸内存里的旧数据。

## 测试清单与预期输出（ipc_demo）

| 层 | 内容 | 预期 |
|----|------|------|
| 1 ShmRegion | create 零初始化 / 双映射可见 / NotFound / InvalidSize / owner 析构删文件 / move 转移所有权 | 6 项 OK |
| 2 三缓冲 | 初始无数据 / publish→borrow 往返 / FLAG_NEW 清除 / latest-wins / 三槽轮换 10000 次 | 5 项 OK |
| 3 ShmClient | 魔数·版本篡改拒绝连接 / 图像像素抽检 / **第三映射改写像素验证零拷贝** / 五路位姿隔离 / 云台指令读回 / 心跳存活·死亡 | 7 项 OK |
| 收尾 | `/tmp/talos_ipc_*` 无残留 | `[OK] /tmp 共享文件已清理` |

预期输出（节选）：

```
---- [2-TripleBuffer 无锁三缓冲原语] ----
  [OK] 初始状态无新数据，borrow 返回 nullopt
  [OK] latest-wins：连发两帧只保留最新
  [OK] 三槽轮换压力 10000 次发布/消费全对
...
==== 全部通过：ShmRegion + TripleBuffer + ShmClient ====
```
