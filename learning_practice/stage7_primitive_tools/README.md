# stage7：primitive 模块其余 6 个组件

对应真实代码：[crates/primitive/src/primitive/](../../crates/primitive/src/primitive/)

| 组件 | 功能 | 关键技术点 |
|---|---|---|
| [lazy.hpp](../../crates/primitive/src/primitive/lazy.hpp) | 延迟构造：参数包存 `std::tuple`，调用时才 `new T` | `std::apply` 解包、C++20 `requires` 区分两种重载 |
| [overloaded.hpp](../../crates/primitive/src/primitive/overloaded.hpp) | `std::visit` 多 lambda 派发 | C++17 pack expansion 继承 + `using Ts::operator()...` |
| [spin.hpp](../../crates/primitive/src/primitive/spin.hpp) | 自旋等待提示宏 | x86 `_mm_pause` / ARM `yield`，降功耗防流水线冲刷 |
| [performance_probe.hpp](../../crates/primitive/src/primitive/performance_probe.hpp) | 时延探针 + 直方图 | `steady_clock` 计时；8192 滑动窗口 + 原子 min/max，无锁 |
| [system_info.hpp](../../crates/primitive/src/primitive/system_info.hpp) | 用户名/主机名查询 | `getlogin_r`/`gethostname`，`std::expected` 返回 |
| [channel.hpp](../../crates/primitive/src/primitive/channel.hpp) | 三缓冲之上的通道封装 | `make_spsc_channel`/`make_spmc_channel` → `split()`；SPMC 可 `clone_reader()` |

## 读源码 + 验证时发现的真实问题（比测试本身更有价值）

1. **`make_lazy` 是坏的**：返回类型声明为 `lazy<T>`（丢了 `PArgs...`），
   内部却构造 `lazy<T, PArgs...>` —— 传任何预绑定参数都无法通过编译。
   预绑定路径只能显式写模板参数直接构造 `lazy<T, int, std::string>`。
2. **`getlogin_r` 在无登录会话的环境（容器/沙箱/CI）返回 ENOTTY**：
   `get_username()` 走 `std::expected` 错误分支。demo 对两个分支都做断言
   （错误分支要求 error 文本非空），这是 expected 语义的活教材。
3. **GCC 对 `std::hardware_destructive_interference_size` 有保留意见**
   （`-Winterference-size`，值可能随 `-mtune` 变化）：demo 编译选项豁免。
4. **分位数下标是 `n*p/100` 向下取整**：1050 个样本里塞 50 个毛刺（4.76%），

   `p95_idx = 1050*95/100 = 997 < 1000` → p95 仍是主体值 100，
   p99（下标 1039）才命中 10000 —— "5% 的尾部到底够不够命中 p95"要算过才知道。

## 运行方法

```bash
cd learning_practice/stage7_primitive_tools
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/primitive_tools_demo    # 全部通过返回 0
```

无外部依赖：`performance_probe.cpp` / `system_info.cpp` 两个真实源文件
直接编译进 demo（primitive crate 本身就是准 INTERFACE 组织方式）。

## 预期输出

```
=== 测试1：lazy 延迟构造 ===
  预绑定路径: 调用前构造次数 = 0，调用后 = 1（延迟生效）
  运行时路径: 构造次数 +1，unique_ptr 托管
测试1通过

=== 测试2：overloaded + variant 分发 ===
  直连后端: 曝光 2000us
  Daedalus 后端: 共享内存 /dev/shm/talos_frames
  Foxglove 后端: WebSocket 端口 8765
测试2通过

=== 测试3：SPIN_HINT 自旋提示 ===
  100 万次空转（含 volatile 加法）:
  无提示耗时: 737 us
  加提示耗时: 16172 us
  （PAUSE 不保证更快，但显著降低自旋功耗与流水线冲刷）
测试3通过

=== 测试4：LatencyProbe + LatencyHistogram ===
  分布: 1000 个 100ns + 50 个 10000ns
  count=1050  min=100  p50=100  p95=100  p99=10000  max=10000  mean=571  stddev=2108.29
  LatencyProbe 分段: 忙等段 267 us > 空闲段 40 ns
测试4通过

=== 测试5：SystemInfo ===
  username 不可用（无登录会话）: get_username: getlogin_r failed: Inappropriate ioctl for device
  hostname = <你的机器名>
测试5通过

=== 测试6：channel 通道封装 ===
  SPSC: write/read 42 -> borrow_mut/publish 99 均正确
  SPMC: generation=1，两个消费者各自读到 frame#1
测试6通过

=== stage7 primitive 模块全部测试通过 ===
```

## 测试清单（src/demo.cpp）

| 测试 | 断言要点 |
|---|---|
| 1 lazy | 预绑定/运行时两条路径，构造次数调用前 0、调用后恰 +1 |
| 2 overloaded | variant 三策略各命中对应 lambda，输出含各自字段值 |
| 3 SPIN_HINT | 100 万次空转加/不加 PAUSE 定性对比（只断言耗时 > 0） |
| 4 性能探针 | 已知分布：count=1050、min=100、p50=100、p99=max=10000、mean=571 |
| 5 SystemInfo | expected 双分支：有值非空 / 无值则 error 文本非空 |
| 6 channel | SPSC write/read + borrow_mut/publish；SPMC 双消费者独立读同一帧、generation=1 |
