## Talos Scheduler

Talos Scheduler 是一个面向实时机器人系统的 C++23 数据流任务调度器。

它将程序组织为一组 `System`。每个 `System` 声明自己读取哪些资源、写入哪些资源。调度器根据这些声明构建依赖图，并在运行时驱动任务执行。

```text
Resource ──read──▶ System ──write──▶ Resource
````

调度器的目标很简单：

让数据流决定执行顺序。

## 核心模型

Talos Scheduler 中有三个核心概念。

**System** 是一个任务。
它可以采集相机、运行检测、更新状态估计，也可以输出控制指令。

**Resource** 是系统之间共享的数据。
它可以是一帧图像、一次检测结果、一个目标状态，或者一份全局配置。

**Channel** 是带读写语义的资源访问方式。
它描述数据是一对一传递、一对多广播，还是作为共享状态存在。

```text
┌────────┐       frame        ┌──────────┐     detection     ┌─────────┐
│ Camera │ ─────────────────▶ │ Detector │ ────────────────▶ │ Tracker │
└────────┘                    └──────────┘                   └─────────┘
```

## 最小例子

```cpp
#include <scheduler/scheduler.hpp>
#include <scheduler/system/components.hpp>

using namespace talos;

struct ImageFrame {};
struct Detection {};

struct CameraTopic {};
struct DetectionTopic {};

int main() {
    World world;
    Scheduler scheduler(world);

    scheduler.add_system<fixed_rate<30, 0, 0>>("camera",
        [](spsc_mut<ImageFrame, CameraTopic> output) {
            ImageFrame frame = capture_camera();
            output.write(std::move(frame));
        });

    scheduler.add_system("detector",
        [](spsc<ImageFrame, CameraTopic> input,
           spmc_mut<Detection, DetectionTopic> output) {
            auto frame = input.read();
            if (!frame) {
                return;
            }

            Detection detection = detect(*frame);
            output.write(std::move(detection));
        });

    auto result = scheduler.build();
    if (!result) {
        return 1;
    }

    scheduler.run();
    return 0;
}
```

这段代码做了三件事：

```text
camera   以 30Hz 采集图像
detector 在新图像到达后被唤醒
build()  在运行前检查数据流是否合法
```

## 通道类型

| 类型                        | 语义        | 用途            | 保证        |
| ------------------------- | --------- | ------------- | --------- |
| `spsc<T>` / `spsc_mut<T>` | 单生产者，单消费者 | 点对点数据流        | 无锁三重缓冲    |
| `spmc<T>` / `spmc_mut<T>` | 单生产者，多消费者 | 广播与多订阅        | 多读者共享读取   |
| `res<T>` / `res_mut<T>`   | 共享资源      | 全局状态          | 由调度拓扑约束访问 |
| `local<T>`                | 系统本地状态    | per-system 缓存 | 仅当前系统访问   |

读访问使用不带 `_mut` 的类型。

写访问使用带 `_mut` 的类型。

例如：

```cpp
spsc<ImageFrame, CameraTopic>       // 读取图像
spsc_mut<ImageFrame, CameraTopic>   // 写入图像
```

## 内存模型

通道读写使用 acquire-release 语义。

```text
write() / publish()  release
read()               acquire
```

这保证写入端发布的数据，对读取端可见。

该模型适用于 x86-64 和 ARM64。

## 执行策略

### `fixed_rate<FreqHz, CPU, Priority>`

使用独占线程定频执行。

适合相机、IMU、控制输出等周期性任务。

```cpp
scheduler.add_system<fixed_rate<30, 0, 0>>("camera", [](auto out) {
    out.write(capture());
});
```

### `fixed_rate_silent<FreqHz, CPU, Priority>`

使用独占线程定频执行，但不唤醒下游系统。

适合高频状态刷新。

```cpp
scheduler.add_system<fixed_rate_silent<500, 1, 0>>("imu", [](auto out) {
    out.write(read_imu());
});
```

### `pool_compute`

默认执行策略。

系统由上游数据写入触发，并在线程池中执行。

```cpp
scheduler.add_system("processor", [](auto in, auto out) {
    auto data = in.read();
    if (data) {
        out.write(process(*data));
    }
});
```

## 构建期验证

`Scheduler::build()` 会在运行前检查数据流。

只有通过验证的调度图才能进入运行状态。

| 验证项      | 捕获的问题          | 错误类型                             |
| -------- | -------------- | -------------------------------- |
| 单写者      | 多个系统写入同一通道     | `MultipleWritersError`           |
| SPSC 单读者 | SPSC 通道被多个系统读取 | `MultipleReadersError`           |
| 无孤儿读取    | 读取了没有写者的通道     | `OrphanedReaderError`            |
| 无循环依赖    | 系统之间存在循环数据依赖   | `DependencyCycleError`           |
| 全系统可达    | 计算系统没有唤醒来源     | `UnreachableComputeSystemsError` |
| 规模限制     | 计算系统超过 64 个    | `TooManyComputeSystemsError`     |

```cpp
auto result = scheduler.build();

if (!result) {
    std::visit([](auto&& err) {
        using T = std::decay_t<decltype(err)>;

        if constexpr (std::is_same_v<T, DependencyCycleError>) {
            // 处理循环依赖
        } else if constexpr (std::is_same_v<T, MultipleWritersError>) {
            // 处理多写者冲突
        }
    }, result.error());
}
```

## 运行状态

调度器具有明确的生命周期。

```text
Configuring ──build()──▶ Built ──run()──▶ Running
     ▲                                      │
     └──────────────── stop() ◀────────────┘
```

`build()` 成功后，系统拓扑被冻结。

运行时不再动态创建或销毁通道。

## 可视化与调试


```cpp
// 打印唤醒链：
scheduler.print_mermaid_wake_chains();

// 打印执行层级：
scheduler.print_mermaid_execution_levels();

// 打印性能统计：
scheduler.print_stats();

// 导出 JSON 统计：
auto json = scheduler.get_stats_json();
```

这些接口用于观察调度图、唤醒关系、执行层级和运行时性能。

## 设计边界

Talos Scheduler 负责调度拓扑、资源访问语义和任务执行。
