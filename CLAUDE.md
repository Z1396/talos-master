# Talos C++ 项目代码指南

> **代码定位**：Talos 是一个以 ML-style 建模、RAII 生命周期和类型安全数据流为核心的机器人视觉框架。
>
> **架构定位**：系统运行在自定义调度器之上，通过 5 级 FCS（Fire Control System）流水线组织计算；模块依赖由 DAG
> 显式描述，并在执行前完成分析，从结构上杜绝隐式共享状态与数据竞争。
>
> **核心原则**：用 `struct` 表达 product type，用 `std::variant` 表达 sum type，用 `std::expected<T, std::string>`
> 表达可恢复失败，用 RAII owner 表达资源生命周期；所有边界数据必须先解析为强类型，核心域中非法状态不可表示。

---

## 代码目录结构

```
talos-cpp/
├── crates/                      # 核心库
│   ├── primitive/               # 基础原语库（通道、ADT 辅助、线程/性能工具）
│   │   └── src/primitive/
│   │       ├── channel.hpp              # 统一通道抽象
│   │       ├── spsc_triple_buffer.hpp   # SPSC 三重缓冲
│   │       ├── spmc_triple_buffer.hpp   # SPMC 三重缓冲
│   │       ├── lazy.hpp                 # 延迟构造
│   │       ├── overloaded.hpp           # std::visit 重载辅助
│   │       ├── spin.hpp                 # 自旋等待原语
│   │       ├── performance_probe.hpp    # 性能探针
│   │       ├── system_info.hpp          # 系统信息查询
│   │       └── thread_affinity.hpp      # 线程亲和性设置
│   ├── scheduler/               # 调度器核心（World、System、DAG 依赖分析）
│   │   ├── src/scheduler/
│   │       ├── world.hpp      # World: 资源和通道容器
│   │       ├── scheduler.hpp/.cpp  # Scheduler: 调度器核心
│   │       ├── thin.hpp            # 轻量工具/薄封装
│   │       ├── demangle.*          # 类型名反混淆，错误诊断用
│   │       ├── error.*             # 调度器错误类型/错误传播
│   │       ├── error_formatter.*   # 调度器错误格式化
│   │   │   ├── system/             # System 执行模型与元信息
│   │   │   │   ├── components.hpp  # 通道/资源组件类型
│   │   │   │   ├── execution_policy.hpp # 执行策略
│   │   │   │   ├── system_meta.hpp # System 元信息与依赖描述
│   │   │   │   └── system.hpp      # System 执行模型
│   ├── fast_tf/                 # 类型安全坐标变换
│   ├── math/                    # 数学工具（SO2、Euler）
│   ├── toml/                    # TOML 配置解析
│   ├── log/                     # 日志工具
│   └── hardware/                # 硬件驱动
│       ├── at_gimbal/           # 云台控制
│       └── hik_camera_driver/   # HIK 相机驱动（x86_64）
├── src/
│   ├── fcs/                         # 火控系统主体（5 级流水线 + 标定 + 运行时）
│   │   ├── config.hpp               # FCS 总配置
│   │   ├── camera_config.hpp        # 相机配置
│   │   ├── foxglove_config.hpp      # 可视化配置
│   │   ├── core/                    # 共享领域类型、topics、时间、PNP、弹道
│   │   │   ├── channel_topics.hpp   # 调度器通道 topics
│   │   │   ├── armor_types.hpp      # 装甲板领域类型
│   │   │   ├── target_key.hpp       # 目标身份 key
│   │   │   ├── time.hpp             # 时间类型
│   │   │   ├── types.hpp/.cpp       # FCS 基础类型
│   │   │   ├── types_pnp.hpp        # PNP 类型
│   │   │   ├── math/                # FCS 数学辅助
│   │   │   └── trajectory/          # 弹道模型
│   │   ├── calibration/             # 相机内参、ChArUco、棋盘格、手眼标定
│   │   ├── chiral/                  # Chiral 数据采集/记录系统
│   │   ├── L1_sensor/               # 采集层：相机输入、输出接口、数据 parcel
│   │   ├── L2_perception/           # 感知层：armor、rune、ldm、common
│   │   ├── L3_estimation/           # 估计层：tracker、EKF、能量机关、LDM naive
│   │   ├── L4_planning/             # 轨迹规划层：aimer、gimbal planner、目标选择、弹道构建
│   │   ├── L5_weapon/               # 武器/火控层：fire decision、fire control
│   │   ├── runtime/                 # 启动、配置加载、采集器、L1/L2 注册
│   │   └── tests/                   # FCS 单元测试与算法回归测试
│   ├── fcs_visualization/           # Foxglove 可视化
│   │   ├── foxglove_server.*        # Foxglove WebSocket server
│   │   ├── foxglove_sink.hpp        # 可视化消息 sink
│   │   ├── foxglove_systems.*       # 可视化系统注册
│   │   ├── foxglove_types.hpp       # 可视化类型
│   │   ├── scene_builder.*          # 场景构建
│   │   ├── system_helpers.hpp       # 可视化系统辅助
│   │   ├── tactical_palette.hpp     # 战术配色
│   │   ├── utility.hpp              # 可视化工具
│   │   └── systems/                 # 各流水线层的可视化系统
│   ├── main.cpp                     # 主程序入口
│   ├── playground.cpp               # 实验入口
│   └── quanta_ipc_demo.cpp          # Quanta IPC demo
├── config/                      # 配置文件
├── 3dparty/                     # 第三方库
└── cmake/                       # CMake 模块
```

---

## 库结构

| 库名                | 类型        | 说明                       |
|-------------------|-----------|--------------------------|
| toml              | INTERFACE | TOML 配置解析辅助              |
| log               | INTERFACE | 日志工具封装                   |
| math              | INTERFACE | 数学工具（SO2、Euler）          |
| primitive         | SHARED    | 原语库（通道、lazy、spin、性能探针）   |
| scheduler         | SHARED    | 调度器                      |
| fast_tf           | SHARED    | 类型安全坐标变换                 |
| at_gimbal         | INTERFACE | 云台控制                     |
| hik_camera_driver | SHARED    | HIK 相机驱动（仅 Linux x86_64） |
| hardware_daedalus | SHARED    | 硬件抽象层（共享内存）              |
| fcs               | SHARED    | 火控系统（使用 PCH 加速 50-70%）   |
| fcs_visualization | SHARED    | Foxglove 可视化             |

---

## 调度器编程模型

### System 定义

```cpp
// System 通过通道 (Channel) 和资源 (Resource) 访问数据
struct MySystem {
  // 通道访问
  spsc<Input, TagInput> reader;       // SPSC 只读
  spmc_mut<Output, TagOutput> writer; // SPMC 只写

  // 资源访问
  res<Config> config;                 // 只读资源（版本追踪）
  res_mut<State> state;               // 可写资源（写后版本号自增）
  local<Temp> temp;                   // 系统本地变量（不参与依赖分析）

  // bind(): 预创建通道（初始化期调用）
  void bind(World& world);

  // run(): 执行系统逻辑
  void run(World& world);
};
```

### 执行策略

| 策略                                       | 说明               | 用途            |
|------------------------------------------|------------------|---------------|
| `fixed_rate<Freq, CPU, Priority>`        | 独占线程，定频触发，通知下游   | 数据源（如相机）      |
| `fixed_rate_silent<Freq, CPU, Priority>` | 独占线程，定频触发，不通知    | 高频静默更新（如 IMU） |
| `pool_compute`                           | TBB 线程池，依赖触发（默认） | 数据处理系统        |
| `pool_visualization`                     | 可视化系统专用          | TODO          |

### 通道组件

| 组件                 | 说明              |
|--------------------|-----------------|
| `spsc<T, Tag>`     | SPSC 只读（点对点通信）  |
| `spsc_mut<T, Tag>` | SPSC 只写         |
| `spmc<T, Tag>`     | SPMC 只读（可拷贝）    |
| `spmc_mut<T, Tag>` | SPMC 只写         |
| `res<T>`           | 只读资源（版本追踪）      |
| `res_mut<T>`       | 可写资源（写后版本号自增）   |
| `local<T>`         | 系统本地变量（不参与依赖分析） |

## FCS 流水线代码组织

`src/fcs/` 是火控系统主体。除 5 级流水线外，还包含标定、运行时启动、配置加载、测试与数据采集辅助模块。

```text
src/fcs/
├── core/              # 共享领域类型、通道 topics、时间、PNP、弹道模型
├── calibration/       # 相机内参、ChArUco、棋盘格、手眼标定
├── chiral/            # Chiral 数据采集/记录系统
├── L1_sensor/         # 采集层：相机输入、输出接口、parcel
├── L2_perception/     # 感知层：armor、rune、ldm、common
├── L3_estimation/     # 估计层：tracker、EKF、能量机关、LDM naive
├── L4_planning/       # 规划层：aimer、gimbal planner、目标选择、弹道构建
├── L5_weapon/         # 武器层：fire decision、fire control、轨迹优化
├── runtime/           # boot、capturer、config loader、系统注册
└── tests/             # FCS 单元测试与算法回归测试
```

## Fast_TF 坐标系

`fast_tf` 是 Talos 的类型安全坐标变换库，用强类型 frame 替代字符串坐标系，防止把不同坐标系下的向量、位姿、变换误用。

```text
crates/fast_tf/src/
├── frame.hpp           # 坐标系 frame 类型
├── types.hpp           # typed pose / vector / transform
├── matrix.hpp          # 矩阵与变换运算
├── buffer.hpp          # 变换缓冲
├── validation.*        # 坐标变换验证
├── export.hpp          # fast_tf 导出宏
└── foxglove_export.hpp # Foxglove 可视化导出适配
```

## C++ 开发规范

### 核心哲学

1. **用 ML 思维写 C++**
    - 先建模数据，再写控制流
    - 先定义状态空间，再实现状态转移
    - 先让非法状态不可表示，再考虑运行时检查
    - 优先使用代数数据类型表达业务语义，而不是用 `class + bool + enum + nullptr` 拼状态机

2. **代数数据类型是核心抽象**
    - `struct` 表达 product type：多个字段同时存在
    - `std::variant` 表达 sum type：互斥状态只能选其一
    - `std::optional<T>` 表达值缺失
    - `std::expected<T, std::string>` 表达成功或失败

3. **RAII 是唯一资源管理方式**
    - 禁止手动 `new/delete`
    - 禁止裸指针表达所有权
    - 禁止 `*_initialized` / ownership flag
    - 资源必须由 owner / guard / token 类型持有
    - 第三方 C API 资源优先用 `std::unique_ptr<T, Deleter>`

4. **Parse, Don’t Validate**
    - 边界数据进入核心域前必须解析为强类型
    - 核心域不接受半合法数据
    - 不允许 primitive obsession 污染业务层
    - 解析失败必须返回 `std::expected<T, std::string>`

5. **编译期检查优先**
    - 用 `concepts` 约束模板接口
    - 用 `std::variant + std::visit` 穷尽处理状态分支
    - enum switch 开启 `-Wswitch-enum`
    - 禁止 `default` 掩盖新增状态
    - 禁止 RTTI：`dynamic_cast` / `typeid`

### DO / DON'T

| DO                                          | DON'T                           |
|---------------------------------------------|---------------------------------|
| `std::unique_ptr` / `std::shared_ptr` 表达所有权 | 裸指针表达所有权                        |
| `std::unique_ptr<T, Deleter>` 管理第三方资源       | 手动 `destroy()` / `close()`      |
| `std::optional<T>` 表达缺失                     | 空字符串、`-1`、`nullptr` 表达缺失        |
| `std::span<T>` 表达连续视图                       | 裸指针 + size                      |
| `std::expected<T, std::string>` 表达错误        | exception / bool / 错误码          |
| `std::variant` 表达互斥状态                       | enum + nullable payload         |
| `enum class` 表达封闭枚举                         | 普通 enum / 字符串状态                 |
| 强类型包装业务概念                                   | 到处传 `int` / `double` / `string` |
| 模板 + concepts                               | `std::function` / 运行时检查         |

### 状态建模

互斥状态必须用 `std::variant`，禁止用多个 bool、nullable pointer、magic enum 拼状态机。

```cpp
struct Loading {};
struct Ready {
    ModelHandle model;
};
struct Failed {
    std::string reason;
};
using ModelState = std::variant<Loading, Ready, Failed>;
```

禁止：

```cpp
bool initialized;
bool failed;
ModelHandle* model;
std::string error;
```

处理 variant 必须集中、穷尽：

```cpp
return std::visit(
    overloaded{
        [](const Loading&) -> Result {
            return std::unexpected("model is still loading");
        },
        [](const Ready& ready) -> Result {
            return run(ready.model);
        },
        [](const Failed& failed) -> Result {
            return std::unexpected(failed.reason);
        }
    },
    state
);
```

### 错误处理

* 内部错误统一使用 `std::expected<T, std::string>`
* 禁止自定义错误枚举 / 错误码
* 禁止用 `bool` 表达多义性错误

- **异常是外部系统边界**，不允许在系统内部传播。所有内部错误通过 `std::expected` 传播

* 返回 `std::expected` 的函数必须 `noexcept`
* 只允许 `catch(const std::exception&)`
* 禁止 `catch(...)`

错误必须在 error site 内联构造，使用 `fmt::format`，并携带完整上下文：

```cpp
return std::unexpected(fmt::format(
    "create session(path={}): {}",
    path.value(),
    reason
));
```

系统错误必须转为人类可读文本：

```cpp
std::strerror(errno)      // Linux
mach_error_string(kr)    // macOS
```

禁止裸错误码：

```cpp
return std::unexpected("errno=22");        // bad
return std::unexpected("kern_return=46");  // bad
```

### 构造与初始化

1. 构造函数必须 `noexcept`
    * 构造完成的对象必须处于有效状态
    * 构造函数不得执行可能失败的系统调用、IO、模型加载、GPU 初始化
    * 构造函数不得抛异常
2. 可能失败的初始化必须放入 `static create() noexcept`
    * 返回 `std::expected<T, std::string>`
    * 构造函数设为 `private` 或只接收已经安全取得的资源
    * 所有失败点必须携带完整上下文

```cpp
class ModelRunner {
public:
    static auto create(ModelPath path) noexcept
        -> std::expected<ModelRunner, std::string>;
    ModelRunner(ModelRunner&&) noexcept = default;
    auto operator=(ModelRunner&&) noexcept -> ModelRunner& = default;
    ModelRunner(const ModelRunner&) = delete;
    auto operator=(const ModelRunner&) -> ModelRunner& = delete;
private:
    explicit ModelRunner(SessionHandle session) noexcept
        : session_(std::move(session)) {}
    SessionHandle session_;
};
```

禁止半初始化对象。
禁止用 `*_initialized` 标志冒充生命周期管理。

### 第三方资源

第三方 C API 资源优先使用 `std::unique_ptr<T, Deleter>`：

```cpp
struct OrtSessionDeleter {
    void operator()(OrtSession* p) const noexcept {
        if (p != nullptr) {
            OrtReleaseSession(p);
        }
    }
};
using OrtSessionHandle = std::unique_ptr<OrtSession, OrtSessionDeleter>;
```

非指针资源必须定义专门 owner 类型：

```cpp
class FileDescriptor {
public:
    explicit FileDescriptor(int fd) noexcept;
    FileDescriptor(FileDescriptor&&) noexcept;
    auto operator=(FileDescriptor&&) noexcept -> FileDescriptor&;
    FileDescriptor(const FileDescriptor&) = delete;
    auto operator=(const FileDescriptor&) -> FileDescriptor& = delete;
    ~FileDescriptor() noexcept;
    [[nodiscard]]
    auto get() const noexcept -> int;
private:
    int fd_;
};
```

禁止让调用方手动调用 `destroy()` / `close()` / `release()` 完成正常生命周期管理。

### 接口设计

* 单参数构造函数必须 `explicit`
* 业务概念必须强类型包装
* 优先模板，而非 `std::function`
* 模板参数必须用 C++20 concepts 约束
* CRTP、策略模式、工厂签名必须通过 `concept` 检查 `noexcept` 和返回类型
* 禁止运行时能力查询：`supports_xxx()` / `name()`
* 禁止向后兼容废代码，废弃接口直接删除
* 禁止 `std::any`，除 World / Registry 内部实现外

能力应进入类型系统：

```cpp
using Backend = std::variant<CpuBackend, CudaFp32Backend, CudaFp16Backend, TensorRtInt8Backend>;
```

不要写：

```cpp
if (backend.supports_fp16()) { ... }
```

### 控制流

enum switch 必须穷尽：

```cpp
switch (kind) {
    case DeviceKind::Cpu:
        return "cpu";
    case DeviceKind::Cuda:
        return "cuda";
    case DeviceKind::TensorRt:
        return "tensorrt";
}
std::abort(); // only unreachable logic error
```

`std::abort()` 只允许用于不可恢复的逻辑错误，例如理论不可达分支或 invariant 被破坏。

以下情况必须返回 `std::expected`，禁止 `abort()`：

* 文件不存在
* 权限不足
* 模型加载失败
* 资源分配失败
* 第三方库返回失败
* 用户输入非法

### 所有权传递

非拷贝组件必须显式 move：

```cpp
auto system = MySystem::create(std::move(reader));
```

禁止：

```cpp
auto system = MySystem::create(reader);
```

Lambda 捕获非拷贝组件：

```cpp
auto fn = [&reader]() mutable {
    return reader.read();
};
auto fn = [reader = std::move(reader)]() mutable {
    return reader.read();
};
```

禁止隐式复制资源对象。

### 常见模式和反模式

| 模式          | 	正确	                                              | 错误                             |
|-------------|---------------------------------------------------|--------------------------------|
| 传递非拷贝组件     | 	std::move(component)                             | 	my_system(component)          |
| Lambda 捕获组件 | 	[&reader] mutable / [reader = std::move(reader)] | 	[reader]                      |
| 表达互斥状态      | 	std::variant<A, B, C>	                           | enum + optional payload        |
| 表达资源生命周期    | 	RAII owner                                       | 	initialized flag              |
| 表达错误	       | std::expected<T, std::string>                     | 	exception / bool / error code |

### 依赖库版本

| 依赖                                                | 版本       | 用途         |
|---------------------------------------------------|----------|------------|
| [oneTBB](https://uxlfoundation.github.io/oneTBB)  | 2021.9+  | 任务调度与线程池   |
| [Eigen3](https://libeigen.gitlab.io)              | 3.4+     | 线性代数       |
| [OpenCV](https://opencv.org)                      | 4.x      | 图像处理       |
| [Ceres](https://ceres-solver.org)                 | 2.x      | 非线性优化与自动微分 |
| [FFmpeg](https://ffmpeg.org)                      | 8.x      | 图传编码与解码    |
| [spdlog](https://github.com/gabime/spdlog)        | 1.17     | 日志         |
| [ONNXRuntime](https://onnxruntime.ai)             | >= 1.26  | 推理后端       |
| [TensorRT](https://developer.nvidia.com/tensorrt) | optional | 推理后端       |

> 用类型系统建模世界，用 RAII 管理资源，用 expected 显式传播失败，用 variant 表达选择，用 ML 思维驯服 C++。
