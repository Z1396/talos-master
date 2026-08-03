# Talos 三缓冲架构详解

> 本文档系统讲解 Talos 项目中 SPSC / SPMC 三缓冲的设计动机、架构原理与具体实现。
>
> **源码位置**：
> - [crates/primitive/src/primitive/spsc_triple_buffer.hpp](./crates/primitive/src/primitive/spsc_triple_buffer.hpp)
> - [crates/primitive/src/primitive/spmc_triple_buffer.hpp](./crates/primitive/src/primitive/spmc_triple_buffer.hpp)
> - [crates/primitive/src/primitive/channel.hpp](./crates/primitive/src/primitive/channel.hpp)

---

## 目录

- [1. 设计动机：为什么需要缓冲](#1-设计动机为什么需要缓冲)
- [2. 缓冲方案演进：单/双/三缓冲对比](#2-缓冲方案演进单双三缓冲对比)
- [3. SPSC 三缓冲架构详解](#3-spsc-三缓冲架构详解)
- [4. SPMC 三缓冲架构详解](#4-spmc-三缓冲架构详解)
- [5. 两种实现对比](#5-两种实现对比)
- [6. 项目中的实际应用](#6-项目中的实际应用)
- [7. 关键设计要点总结](#7-关键设计要点总结)

---

## 1. 设计动机：为什么需要缓冲

### 1.1 问题背景

机器人自瞄系统中，生产者与消费者速度天然不匹配：

```
相机（生产者）：250Hz 产帧，每 4ms 一帧
检测算法（消费者）：150Hz 处理，每 6ms 一帧
```

若直接对接，会出现：

```
无缓冲：
时间→  0ms    4ms    8ms    12ms
相机:  产帧1  产帧2  产帧3  产帧4
检测:  ────处理帧1(6ms)────>────处理帧2(6ms)────>
                    ↑
              相机产帧2时检测还在忙
              帧2被覆盖或丢失
```

### 1.2 缓冲的核心目标

1. **解耦速度**：生产者永不阻塞，消费者按自己节奏取数据
2. **保最新**：消费者总是拿到最新帧，旧帧可丢弃
3. **零等待**：生产者写完就走，消费者有新数据就读，无新数据就跳过

---

## 2. 缓冲方案演进：单/双/三缓冲对比

### 2.1 方案 1：单缓冲（1 个槽）

```
槽：[帧?]
相机写 → 检测读 → 相机再写 → 检测再读
```

**问题**：相机写时检测不能读，检测读时相机不能写 → **必须加锁互斥**，相机要等检测读完才能写下一帧。

### 2.2 方案 2：双缓冲（2 个槽轮换）

```
槽A：[帧1]  槽B：[帧2]
相机写A → 切换 → 检测读A，相机写B → 切换 → 检测读B，相机写A
```

**残留问题**：切换瞬间仍有竞争——
- 相机写完 A 想切到 B，但 B 还在被检测读 → 相机要等
- 检测想读 A，但相机正在写 A → 检测要等

### 2.3 方案 3：三缓冲（3 个槽）—— 关键突破

```
槽0：[?]  槽1：[?]  槽2：[?]
```

**核心思想**：永远保证**生产者有一个空闲槽可写**，**消费者有一个稳定槽可读**，两者通过第三个槽"交接"。

**三槽动态角色分配**：

| 角色 | 含义 | 访问者 |
|---|---|---|
| 写槽 | 生产者正在写的槽 | 仅生产者 |
| 读槽 | 消费者正在读的槽 | 仅消费者 |
| 中间槽 | 上次刚写完的数据，等待消费者来取 | 交接缓冲 |

**关键性质**：任何时候三个槽角色不同，生产者和消费者**永远碰不到同一个槽**，所以**完全无锁**。

---

## 3. SPSC 三缓冲架构详解

> **SPSC** = Single Producer Single Consumer（单生产者单消费者）
>
> **源码**：[spsc_triple_buffer.hpp](./crates/primitive/src/primitive/spsc_triple_buffer.hpp)

### 3.1 数据结构

```cpp
template <typename T>
requires(std::movable<T>) class SpscTripleBuffer {
    static constexpr uint8_t FLAG_NEW   = 0x80;  // 高1位：新数据标记
    static constexpr uint8_t INDEX_MASK = 0x03;  // 低2位：槽下标 0/1/2

    struct State {
        alignas(hardware_destructive_interference_size) std::array<T, 3> slots{};
        alignas(hardware_destructive_interference_size) uint8_t write_idx{0};   // 生产者私有
        alignas(hardware_destructive_interference_size) std::atomic<uint8_t> shared{1};
        alignas(hardware_destructive_interference_size) uint8_t read_idx{2};    // 消费者私有
    };
};
```

**关键设计点**：

1. **单字节编码两件事**：`shared` 字节同时编码「新数据标记」+「中间槽下标」

   ```
   shared 字节：[ 1 bit ] [ 2 bits ]
                 FLAG_NEW  INDEX
                 有新数据?  中间槽是几号?
   ```

2. **缓存行对齐**：所有成员 `alignas(64)`，消除多核伪共享（false sharing）

3. **私有变量无需原子**：`write_idx` 仅生产者访问，`read_idx` 仅消费者访问

### 3.2 写入流程

```cpp
void write(U data) noexcept {
    slots[write_idx] = std::move(data);   // 1. 数据写到 write_idx 槽
    publish();                             // 2. 发布
}

void publish() noexcept {
    // 原子交换：把"当前 write_idx + 新数据标记"写入 shared
    const auto old = shared.exchange(write_idx | FLAG_NEW, acq_rel);
    // 旧 shared 的低 2 位就是新的 write_idx（轮换）
    write_idx = old & INDEX_MASK;
}
```

**流程解读**：

1. 数据写入当前 `write_idx` 槽
2. 原子 `exchange` 把"我刚写的槽"变成新的中间槽（同时置位新数据标记）
3. 旧的中间槽变成下一个 `write_idx`（轮换）

### 3.3 读取流程

```cpp
[[nodiscard]] std::optional<U> read() noexcept {
    auto expected = shared.load(acquire);                  // 1. 读 shared
    if (!(expected & FLAG_NEW))                            // 2. 没新数据？
        return std::nullopt;                               //    返回空

    // 3. CAS 循环：清除新数据标记
    while (!shared.compare_exchange_weak(
        expected,
        expected & ~FLAG_NEW,
        acq_rel)) {
        if (!(expected & FLAG_NEW)) return std::nullopt;
    }

    // 4. 切换读槽到中间槽
    read_idx = expected & INDEX_MASK;
    return std::move(slots[read_idx]);                     // 5. 移动取出数据
}
```

**流程解读**：

1. `acquire` 加载 `shared`，检查 `FLAG_NEW`
2. 无新数据直接返回 `nullopt`（非阻塞）
3. CAS 清除 `FLAG_NEW`（防止重复消费）
4. 读槽切到中间槽，`move` 取出数据

### 3.4 三槽轮换全景示例

```
时刻 0：初始
  槽0:空  槽1:空  槽2:空
  write_idx=0  shared=0x01(中间槽1,无新)  read_idx=2
                          ↓ 相机 write(frame1)

时刻 1：相机刚写完
  槽0:frame1  槽1:空  槽2:空
  write_idx=1  shared=0x80(中间槽0,有新)  read_idx=2
  └ 槽0是中间槽(刚写完)，槽1是相机下一个目标
    槽2是检测正在读(旧的)，三者互不干扰
                          ↓ 检测 read()

时刻 2：检测取走 frame1
  槽0:已消费  槽1:空  槽2:空
  write_idx=1  shared=0x00(中间槽0,无新)  read_idx=0
  └ 检测正在读槽0(虽然已消费但还在用)
    相机准备写槽1，互不干扰
                          ↓ 相机 write(frame2)

时刻 3：相机写完 frame2
  槽0:旧  槽1:frame2  槽2:空
  write_idx=0  shared=0x81(中间槽1,有新)  read_idx=0
  └ 槽1是中间槽，槽0重新变写槽(检测已读完)
```

**核心规律**：`write_idx` 永远指向「中间槽 + 1」（模 3 轮换），`read_idx` 永远指向中间槽——生产者和消费者**永不碰同一个槽**。

### 3.5 零拷贝借用接口

```cpp
[[nodiscard]] U& borrow_mut() noexcept {
    return slots[write_idx];  // 直接返回写入槽的可变引用
}
```

**使用场景**：原地构造对象，避免额外的 `move` 操作

```cpp
auto& frame = writer.borrow_mut();  // 拿到写入槽引用
frame.timestamp = now();             // 原地填充字段
frame.image = capture();             // 原地构造
writer.publish();                    // 发布
```

---

## 4. SPMC 三缓冲架构详解

> **SPMC** = Single Producer Multi Consumer（单生产者多消费者）
>
> **源码**：[spmc_triple_buffer.hpp](./crates/primitive/src/primitive/spmc_triple_buffer.hpp)

### 4.1 为什么 SPSC 三槽方案不能直接用于 SPMC

三槽轮换假设**单一消费者**（只有一个人 `move` 取走数据）。多消费者场景下：

- 消费者 A `move` 取走槽 0 → 槽 0 空了
- 消费者 B 也想读这一帧 → 没了！

**解决思路**：改用 `shared_ptr` 快照 + 版本号，所有消费者共享同一份快照的引用计数。

### 4.2 数据结构

```cpp
struct State {
    std::shared_ptr<const T> snapshot;     // 当前数据快照
    std::atomic<uint64_t> generation{0};   // 版本号，每次写+1
    RWSpinLock lock;                        // 读写锁（多读者场景）
};
```

**RWSpinLock 设计**（单原子变量同时管读写）：

```cpp
static constexpr uint32_t WRITER_BIT  = 0x80000000;  // 最高位：写者位
static constexpr uint32_t READER_MASK = 0x7FFFFFFF;  // 低31位：读者计数
```

- 写者独占：置 `WRITER_BIT`，读者全部退出
- 多读者并发：低 31 位计数，互不阻塞

### 4.3 写入流程

```cpp
void write(T data) noexcept {
    auto new_snap = std::make_shared<T>(std::move(data));  // 1. 新建 shared_ptr
    {
        std::lock_guard lock(writer_lock_);                 // 2. 独占写锁
        snapshot = std::move(new_snap);                     // 3. 替换快照
    }
    generation.fetch_add(1, release);                       // 4. 版本号+1
}
```

### 4.4 读取流程（每个消费者独立）

```cpp
[[nodiscard]] std::optional<T> read() noexcept {
    auto gen = generation.load(acquire);                    // 1. 读版本号
    if (gen == my_last_gen) return std::nullopt;            // 2. 没新数据
    {
        std::shared_lock lock(reader_lock_);                // 3. 共享读锁
        auto data = *snapshot;                              // 4. 拷贝数据
    }
    my_last_gen = gen;                                      // 5. 更新自己的版本
    return data;
}
```

**多消费者关键点**：

- 每个消费者维护**自己的 `my_last_gen`**，互不干扰
- 快照由 `shared_ptr` 持有，引用计数自动管理
- 最后一个消费者用完，快照自动释放

### 4.5 快速路径优化

```cpp
// 无锁快速路径：仅比较版本号，不拿锁
auto gen = generation.load(acquire);
if (gen == my_last_gen) return std::nullopt;  // 直接返回，零开销
```

只有在版本号变化时才进入慢速路径（加读锁拷贝），大幅降低 contention。

---

## 5. 两种实现对比

| 维度 | SPSC 三缓冲 | SPMC 三缓冲 |
|---|---|---|
| **槽数** | 3 个固定槽 | 1 个 shared_ptr 快照 |
| **内存分配** | 零分配（预分配 3 槽） | 每次写 `make_shared` |
| **同步机制** | 1 个原子字节 exchange/CAS | generation + RWSpinLock |
| **多消费者** | ❌ 不支持 | ✅ 支持 |
| **延迟** | ~20ns | ~50ns（含分配） |
| **拷贝次数** | 0（move 语义） | 1（拷贝快照） |
| **适用场景** | 相机→检测（一对一） | IMU→多系统（一对多） |
| **缓存行对齐** | 是（防伪共享） | 是 |

---

## 6. 项目中的实际应用

### 6.1 通道封装层

[channel.hpp](./crates/primitive/src/primitive/channel.hpp) 在三缓冲之上封装了统一接口：

```cpp
// SPSC 通道别名
template <typename T>
using SpscChannel = TrackedChannel<T, SpscTripleBuffer<T>>;

// SPMC 通道别名
template <typename T>
using SpmcChannel = TrackedChannel<T, SpmcTripleBuffer<T>>;
```

**对外统一 API**：

| 接口 | SPSC 行为 | SPMC 行为 |
|---|---|---|
| `write(data)` | 写槽 + publish | make_shared + 版本号+1 |
| `read()` | CAS 取最新帧 | 版本号检查 + 拷贝快照 |
| `clone_reader()` | ❌ 禁用（单消费者） | ✅ 克隆新消费者 |
| `borrow_mut()` | ✅ 原地修改（零拷贝） | ❌ 无 |

### 6.2 调度器中的使用

系统通过 `spmc<T, Topic>` / `spsc<T, Topic>` 声明通道依赖，bind 阶段从 World 获取句柄：

```cpp
scheduler.add_system<talos::fixed_rate<250>>(
    "camera_reader",
    [](talos::spmc_mut<ImageFrame, ImageChannelTopic> cam_out) {
        auto frame = camera->recv(1s);
        cam_out.write(std::move(*frame));  // 底层走 SPMC 三缓冲
    });

scheduler.add_system<talos::pool_compute>(
    "armor_detection",
    [](talos::spmc<ImageFrame, ImageChannelTopic> img_in,
       talos::spmc_mut<ArmorDetectionBatch, DetectionChannelTopic> det_out) {
        auto frame = img_in.read();  // 底层走 SPMC 三缓冲 read()
        if (!frame) return;
        // ... 检测逻辑
        det_out.write(std::move(batch));
    });
```

### 6.3 实际数据流

```
camera_reader (250Hz)
    │ spmc_mut<ImageFrame, ImageChannelTopic>
    │ ↓ 底层：SpmcTripleBuffer::write() → make_shared + generation++
    │
    ▼
[ImageChannelTopic SPMC 三缓冲]
    │ ↑ 底层：SpmcTripleBuffer::read() → 版本号检查 + 拷贝快照
    │
    ├─> armor_detection (150Hz)  ── 拿最新帧
    ├─> ldm_detection (100Hz)    ── 拿同一帧
    └─> rune_detection (50Hz)    ── 拿同一帧
         （多消费者共享同一快照，引用计数自动管理）
```

### 6.4 性能数据

| 指标 | SPSC | SPMC |
|---|---|---|
| 写入耗时 | 20~30ns | 50~80ns（含 make_shared） |
| 读取耗时 | 20~30ns | 30~50ns |
| 吞吐量 | 百万级/秒 | 十万级/秒 |
| 内存开销 | 3 × sizeof(T) | 1 × sizeof(T) + shared_ptr 控制块 |

---

## 7. 关键设计要点总结

### 7.1 为什么 SPSC 用三槽轮换

1. **零分配**：3 个槽预分配，运行时无 `new`/`delete`
2. **零拷贝**：`std::move` 语义，数据所有权转移
3. **无锁**：单个原子 `exchange`/`CAS` 完成同步
4. **抗伪共享**：缓存行对齐隔离

### 7.2 为什么 SPMC 用 shared_ptr 快照

1. **多消费者共享**：引用计数自动管理生命周期
2. **版本号轻量检查**：无新数据时零开销
3. **读写锁粒度细**：多读者并发，仅写者独占

### 7.3 内存序选择

| 操作 | 内存序 | 原因 |
|---|---|---|
| `publish()` 的 exchange | `acq_rel` | 既要保证数据写入对消费者可见（release），又要看到消费者的读取状态（acquire） |
| `read()` 的 load | `acquire` | 看到 publish 的 release 写入，保证读到完整数据 |
| `read()` 的 CAS | `acq_rel` | 既要清除标记（写），又要看到最新状态（读） |
| `generation` 写 | `release` | 配合消费者 `acquire` 读，保证快照可见 |
| `write_idx`/`read_idx` | 无需原子 | 私有变量，单线程访问 |

### 7.4 安全性保证

1. **无 TOCTOU**：移除了 `has_new()` + `current()` 组合接口（会有 use-after-free 风险），统一用 `read()` 原子握手
2. **无数据竞争**：三槽轮换保证生产者消费者永不碰同一槽
3. **无死锁**：纯原子操作 + 读写锁（写者优先），无嵌套锁

### 7.5 适用边界

| 场景 | 推荐方案 |
|---|---|
| 一对一高频数据流（相机→检测） | SPSC 三缓冲 |
| 一对多广播（IMU→多系统） | SPMC 三缓冲 |
| 跨进程通信 | ❌ 不适用（考虑共享内存 + 三缓冲） |
| 需要历史数据保留 | ❌ 不适用（只保留最新帧） |

---

## 参考文献

- [Lock-free triple buffering](https://www.reddit.com/r/gamedev/comments/1v4ybp/lockfree_triple_buffering/) — 经典三缓冲讨论
- [C++20 atomic reference](https://en.cppreference.com/w/cpp/atomic/atomic) — 原子操作内存序
- [False sharing](https://en.wikipedia.org/wiki/False_sharing) — 缓存行伪共享问题

---

> **文档版本**：v1.0
> **最后更新**：2026-08-04
> **源码版本**：对应 `crates/primitive/src/primitive/` 当前实现
