# stage13_shm_ipc —— 共享内存 IPC 学习案例方案

## Context（背景）

Talos 与 Daedalus 两套仿真程序通过**零拷贝共享内存 IPC** 协作：Rust 仿真器作为生产者写入图像/位姿/真值，C++ 视觉程序作为消费者订阅并回发云台指令。实现位于 `crates/hardware_daedalus/src/`（4 个头文件）：

| 文件 | 职责 |
|------|------|
| `shm_region.hpp` | `/tmp` 文件式 mmap RAII（create/open/create_or_open，owner 析构删文件，兼容 Rust memmap2） |
| `shm_layout.hpp` | 跨 C++/Rust ABI 布局：魔数/版本、图像池常量、无锁三缓冲结构、`static_assert` 锁定偏移 |
| `shm_triple_buffer.hpp` | SPSC 无锁三缓冲：`state` 位编码（bit7=FLAG_NEW，bit0-1=就绪槽），exchange/CAS 发布借用 |
| `shm_client.hpp` | ShmClient：connect() 消费者 / create() 生产者、零拷贝 recv_image、5 路位姿、云台指令下发、心跳保活 |

用户希望在 `learning_practice/` 中新增一个与现有 12 个 stage 风格一致的学习案例（下一个编号 **stage13**），帮助掌握这套 IPC。

## 方案：新建 `learning_practice/stage13_shm_ipc/`

采用 **stage9 模式**：demo 直接 `#include` 项目真实头文件，不复制任何代码；再用**两个独立可执行程序真实演示"仿真程序互相协作"**（双进程验证 MAP_SHARED 跨进程可见性，替代 Rust 端的角色）。

### 目录结构

```
learning_practice/stage13_shm_ipc/
├── CMakeLists.txt
├── README.md
└── src/
    ├── ipc_demo.cpp          # 单进程功能测试（三层，assert 风格）
    ├── sim_producer.cpp      # 仿真程序（生产者角色，模拟 Daedalus/Rust 端）
    └── vision_consumer.cpp   # 视觉程序（消费者角色，模拟 Talos 端）
```

### 1. CMakeLists.txt

- 编译器：优先 clang-21，不存在时回退默认（照抄 stage12 的 `if(EXISTS ...)` 模式）
- C++23、`-Wall -Wextra -Wpedantic -Werror`、`CMAKE_EXPORT_COMPILE_COMMANDS ON`
- 依赖（全部环境实测可用）：
  - fmt 12.0.0：`FetchContent` 指向 `../../3dparty/fmt-12.0.0.zip`（stage12 同款写法，`shm_region.hpp` 的 fmt 格式化器需要 fmt≥10 的 const format 签名，系统 fmt 8 不兼容）
  - magic_enum：include `../../3dparty/magic_enum`
  - OpenCV：`find_package(OpenCV REQUIRED COMPONENTS core)`（系统 4.5.5 实测可用，`shm_client.hpp` 需要 cv::Mat）
  - `find_package(Threads REQUIRED)`
- `add_library(talos_ipc_learning INTERFACE)`，include 目录指向 `../../crates/hardware_daedalus/src`
- 三个可执行目标：`ipc_demo`、`sim_producer`、`vision_consumer`，均链接该库

### 2. src/ipc_demo.cpp —— 单进程三层功能测试

文件头注释块说明被测对象与测试清单（stage4 风格），assert + 中文逐行注释：

- **第一层 ShmRegion**：create 零初始化；同进程 create+open 双映射写入互相可见；NotFound/InvalidSize 错误路径；owner 析构删 `/tmp` 文件、非 owner 保留；move 语义转移所有权
- **第二层 三缓冲原语**：初始无新数据 borrow 返回 nullopt；publish→borrow 往返；消费后 FLAG_NEW 清除（同帧不读两次）；连发只留最新（latest-wins）；三槽轮换压力
- **第三层 ShmClient 全链路**：connect 魔数/版本校验（篡改头部拒绝连接）；publish_image→recv_image 零拷贝像素抽检；publish_pose→recv_pose 五路通道隔离；send_gimbal_cmd→GimbalOps 读回；心跳 is_producer_alive/wait_for_producer
- 每个用例前后清理 `/tmp/talos_ipc_meta`、`/tmp/talos_ipc_image_pool`（对齐 `shm_ipc_test.cpp` 夹具思路，但用 assert 不用 gtest）

### 3. src/sim_producer.cpp —— 仿真程序（生产者）

复刻 Daedalus 的角色：
- `ShmClient::create()` 建立共享内存（注释说明：真实项目中这一步由 Rust memmap2 完成，字节布局由 shm_layout.hpp 的 static_assert 保证两端一致）
- 循环 ~30fps：生成合成图像（cv::Mat 渐变+递增序号图案）→ `publish_image`；正弦轨迹云台/相机位姿 → `publish_pose`；`update_heartbeat`
- 同时用 `GimbalOps` 从 `gimbal_cmd` 通道读回视觉端指令，fmt::print 显示"仿真收到：yaw/pitch/开火建议"，形成闭环演示
- Ctrl+C 退出（signal 处理置原子标志），析构删除共享文件

### 4. src/vision_consumer.cpp —— 视觉程序（消费者）

复刻 Talos 视觉端角色：
- `ShmClient::connect()` + `wait_for_producer()` 等待仿真上线
- 轮询 `recv_image()`：零拷贝 Mat 直接读共享内存像素，统计帧率/丢帧（seq 跳变检测）
- `recv_pose()` 读取位姿，简单 P 控制算出目标角度（假自瞄：朝图像序号方向摆动），`send_gimbal_cmd` 下发
- 心跳超时检测：生产者消失时打印告警退出

### 5. README.md（stage9 格式）

章节：功能分析表（4 个真实头文件的核心技术点）→ 为什么用文件 mmap 而非 POSIX shm_open/Unix socket（跨 Rust/C++ 语言互通、零拷贝大图像、latest-wins 语义适合视觉流）→ 三缓冲状态位编码图解与内存序说明 → 依赖说明 → 运行方法（ipc_demo + 双终端 producer/consumer 协作演示）→ 测试清单与预期输出。

## 涉及文件

- 新增：`learning_practice/stage13_shm_ipc/{CMakeLists.txt, README.md, src/ipc_demo.cpp, src/sim_producer.cpp, src/vision_consumer.cpp}`
- 只读引用（不修改）：`crates/hardware_daedalus/src/shm_*.hpp`、`3dparty/fmt-12.0.0.zip`、`3dparty/magic_enum/`

## 验证

```bash
cd learning_practice/stage13_shm_ipc
cmake -B build && cmake --build build
./build/ipc_demo                      # 全部断言通过，退出码 0
# 双终端协作验证：
./build/sim_producer                  # 终端1
./build/vision_consumer               # 终端2：应看到帧率统计 + 生产者终端回显云台指令
```

编译零警告（-Werror），ipc_demo 结束后 `/tmp/talos_ipc_*` 无残留文件。
