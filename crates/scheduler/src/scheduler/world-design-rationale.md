# World 资源池设计剖析：为什么是 `map<type_index, UniqueAny>` + `Resource<U>` 壳

> 本文整理 World 全局资源池的类型擦除设计，回答一个核心问题：
> **为什么存一个资源要套四层（`resources_` → `UniqueAny` → `Model<T>` → `Resource<T>` → 真身），每一层分别是谁逼出来的？**

## 一、全景：四层套娃

```
resources_（登记表：map<type_index, UniqueAny>）
 └─ 值: UniqueAny ────────────────── "大箱子"（类型擦除容器，仅移动）
     └─ ptr_: unique_ptr<Concept> ── "擦除指针"（指向抽象基类）
         └─ Model<Resource<CameraConfig>> ── "内胆"（模板实现类）
             └─ value: Resource<CameraConfig> ── "业务壳"
                 └─ value: CameraConfig ──────── "真身"（要存的资源）
```

| 层 | 代码位置 | 职责 |
|---|---|---|
| `resources_` | `ResourceStore` 成员 | 登记表：按类型索引所有资源 |
| `UniqueAny` | `world.hpp` L80 起 | 统一容器：任意类型装进同一种箱型，**仅移动** |
| `Concept` / `Model<T>` | L44 / L57 | 虚函数类型擦除：让"万能箱"成为可能 |
| `Resource<U>` | L218 | 业务协议壳：存取钥匙统一 + 生命周期挂载点 |

## 二、逐层归因：每一层是谁逼出来的

| 层 | 真正的原因 | 换个数据结构还需要吗 |
|---|---|---|
| `Model`/`Concept`（虚函数擦除） | **异构需求**：一张容器装任意类型，而 C++ 没有运行时反射、没有万物皆 Object 的基类 | 需要。与容器选型无关 |
| `UniqueAny` 禁拷贝（仅移动） | **硬件句柄语义**：CAN 通道、电调、相机句柄天然独占，拷贝 = 两个主人抢一个硬件 | 需要。与 map 无关 |
| 自研 UniqueAny（不用 `std::any`） | `std::any` 要求可拷贝，与上一条冲突；且有额外开销 | 需要 |
| `Resource<U>` 壳 | 业务协议：存取钥匙统一 + 生命周期/冻结防护挂载点 | 需要 |
| `type_index` 当键（map 的贡献） | 按类型 O(1) 查找（`res<T>` 每帧都查，hot path） | ← map 真正贡献的部分 |

**一句话归因：根源是"异构 + C++ 没有反射 + 硬件句柄禁拷贝"三件事；map 只负责"按 typeid 索引"这一步。**

### 反证：其他语言怎么做同样的事

```python
# Python：万物皆 Object，一个 dict 直接装，零套娃
resources = {}
resources[CameraConfig] = camera_config
```

Python/Java 不需要 Model/UniqueAny/Resource 这些层，因为语言自带统一基类。
C++ 里这些"套娃"本质是**手动补上语言缺失的运行时类型系统**：

- `Concept/Model` 补"统一基类"（虚函数擦除）
- `typeid` 键补"运行时类型识别"
- `Resource` 补"访问协议"

## 三、核心机制详解

### 3.1 UniqueAny 的类型擦除（`world.hpp` L44-137）

```cpp
struct Concept {                       // 抽象基类：只暴露 type()
    virtual const std::type_info& type() const = 0;
};

template <typename T>
struct Model final : Concept {         // 模板实现类：承载具体业务类型
    T value;
    const std::type_info& type() const noexcept override { return typeid(T); }
};

class UniqueAny {
    std::unique_ptr<Concept> ptr_;     // 指向堆上 Model<T>
    ...
};
```

**为什么 UniqueAny 必须仅移动、真身必须在堆上**：
`ptr_` 指向堆上的 `Model<T>`，`UniqueAny` 移动只是换指针——真身地址终生不变。
这从语言层面强制了"资源只有一个所有者"；若允许拷贝，两个 UniqueAny 的 `ptr_` 各指一份副本，硬件句柄语义立刻崩坏。

**为什么不用 `std::any`**：`std::any` 要求存储类型可拷贝。相机句柄、CAN 通道、电调这类独占资源不可拷贝，`std::any` 直接装不下——这就是自研 UniqueAny 的直接动因。

### 3.2 Resource 壳（`world.hpp` L218-249）

```cpp
template <typename T>
struct Resource {
    T value;   // 真实资源
    Resource() requires std::default_initializable<T> = default;
    template <typename... Args>
    explicit Resource(std::in_place_t, Args&&... args);   // 原位构造
    template <typename U>
    explicit Resource(U&& val);                           // 转发构造
};
```

薄壳、零开销、没有逻辑。它的存在理由：

1. **存取钥匙统一**：写入侧 `emplace<Resource<U>>` 与读取侧 `get<Resource<T>>` 必须用同一种类型做匹配键，任意一侧直接用裸 `U` 都会导致另一侧 `UniqueAny bad cast` panic
2. **生命周期挂载点**：调度器"资源 build 后冻结"的防护（lifetime token / 冻结检查）发生在壳层——`get_storage<T>()` 返回 `Resource<T>*` 而非 `T*`，就是为了在壳上做检查。裸 `U` 无处挂载
3. **扩展性**：今天壳里只有 `value`，明天要加冻结标记/版本号，全部资源自动带上

### 3.3 存取完整链路（`world.hpp` L364-417）

```
emplace_resource<CameraConfig>(args...)
   └─ resources_.emplace<CameraConfig>
        ├─ try_emplace(typeid(CameraConfig))    ← 开格（已存在则 panic：单例铁律）
        └─ it->second.emplace<Resource<CameraConfig>>(in_place, args...)
               └─ UniqueAny::emplace (L115)
                    └─ ptr_ = make_unique<Model<Resource<CameraConfig>>>(...)
                         └─ Resource::Resource(in_place, args...) → value(args...)

get_storage<CameraConfig>()
   └─ find(typeid(CameraConfig))                ← 同一个键，必然命中
        └─ get<Resource<CameraConfig>>()        ← 同一个壳
             └─ 返回 Resource<CameraConfig>* → .value → CameraConfig&
```

`std::in_place` 的作用：空结构体全局实例，作为**重载歧义的暗号**——
没有它，空参会误触默认构造、带参会因类型不匹配直接编译失败。
它是给编译器重载决议看的类型标签，零运行时开销。

### 3.4 insert_resource vs emplace_resource（L621-662）

| | `insert_resource` | `emplace_resource` |
|---|---|---|
| 手里有什么 | 现成对象 | 构造参数 |
| 构造次数 | 2 次（调用方 1 次 + 容器内拷贝/移动 1 次） | **1 次**（容器内存里直接出生） |
| 返回值 | void（手里本来就有） | `T&`（刚出生的，还给你立刻用） |
| 选型口诀 | 手里有现成对象 → insert | 只有构造参数（或想省拷贝）→ emplace |

本质：insert 的签名收 `T&&`（已存在的对象引用），容器必然要产生独立副本，"搬一次"结构上不可避免；
emplace 收的是"还不是对象"的构造实参，才有可能让构造直接发生在容器内存上。

## 四、map 被选中的理由（数据结构层的贡献）

1. **O(1) 按类型查**：`res<T>` 每帧都在查，hot path
2. **try_emplace 天然支持"同类型单例"**：不存在则建、存在则报，一次调用完成，且 key 冲突时**不构造 value**（对比 emplace：冲突时白构造再丢弃）
3. **条目稳定**：插入其他键不影响已有键的迭代器/引用

若换成 sorted vector 二分查找，`Model/UniqueAny/Resource` 一层都不用动——
**容器选型与类型擦除正交，可独立演进。**

## 五、语义级调用方式对照（boot.cpp 实例）

```cpp
// 需要返回值：马上要往里填外参
auto& coordinate_system = world.emplace_resource<fast_tf::CoordinateSystem>();

// 不需要返回值：static_cast<void> 显式丢弃（emplace_resource 带 [[nodiscard]]）
static_cast<void>(world.emplace_resource<core::CapabilityState>(...));

// 纯注册：手里有现成对象
world.insert_resource(server);
```

规律：**返回值接不接，取决于 boot 自己接下来要不要直接操作这个资源**。
要 → 接引用；不要 → `static_cast<void>` 显式丢弃（压制 `[[nodiscard]]` 警告 + 声明意图）。

## 六、一句话总结

> **`resources_` 管查找，`UniqueAny` 管统一容器，`Model` 管类型擦除，`Resource` 管业务协议，`value` 才是货。**
> 套娃的 80% 来自"C++ 手写类型擦除 + 硬件资源独占语义"，map 只负责最后"按 typeid 索引"这一步。
