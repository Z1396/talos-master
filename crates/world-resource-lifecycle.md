# World 资源池：一次 insert / get 的完整生命周期

> 以项目中真实运行的链路为例：`FoxgloveServer` 从 `main.cpp` 存入资源池，到可视化系统运行时取出使用。
> 所有代码位置均为真实文件与行号，可对照源码阅读。

---

## 0. 核心思想（一句话）

**用"类型本身"当钥匙（`typeid`），把资源存进一张全局登记表（map），存和取各自独立推导类型——类型相同则钥匙必然配对，拿到的是同一份对象。**

资源池的数据结构：

```cpp
resources_ : map<type_index, UniqueAny>
//           └─ 键：类型的身份证（typeid(U)）
//                        └─ 值：类型擦除的"盒子"，内部装 Resource<U>{ .value = 资源本体 }
```

---

## 1. 第 1 站：数据从哪来（main.cpp）

```cpp
// main.cpp L125
std::shared_ptr<FoxgloveServer> server = std::move(*server_result);
// 此时：栈上有个变量 server，堆上有真正的 FoxgloveServer 对象

// main.cpp L131
scheduler.world().insert_resource(server);   // ← 数据从这里进入资源池
```

---

## 2. 第 2 站：insert_resource 的壳（world.hpp L619-623）

```cpp
template <typename T>                        // 编译器推导：T = shared_ptr<FoxgloveServer>&（左值）
void insert_resource(T&& resource) {
    using U = std::remove_cvref_t<T>;        // L623：剥壳 → U = shared_ptr<FoxgloveServer>
```

`U` 从此确定。**这个 U 后面每一站都会用到——它就是数据的"门牌号"。**

---

## 3. 第 3 站：try_emplace 占位（world.hpp L368）

```cpp
auto [it, inserted] = resources_.try_emplace(typeid(U));
//                                                └─┬─┘
//              实际传入的是：typeid(shared_ptr<FoxgloveServer>)
```

执行时 `resources_` 这个 map 的内容变化：

```
执行前：
  { }                                    ← 空表

执行后：
  { typeid(shared_ptr<FoxgloveServer>) : 空盒子 }
   └── 就这一条新记录，it 指向它，inserted=true
```

`try_emplace` 的两条规则：

| 情况 | 行为 | 返回 |
|---|---|---|
| key 不存在 | 新建条目（空盒子） | `it` 指向新条目，`inserted = true` |
| key 已存在 | 什么都不动 | `it` 指向旧条目，`inserted = false` |

`auto [it, inserted]` 是结构化绑定，等价于：

```cpp
auto result = resources_.try_emplace(typeid(U));
auto it = result.first;        // 条目定位器（迭代器）
auto inserted = result.second; // 是否本次新建
```

---

## 4. 第 4 站：往盒子里装数据（world.hpp L387-390）

```cpp
// 资源已存在 → 重复注册 → 直接 panic（全局单例铁律）
if (!inserted) [[unlikely]] {
    panic("Resource already exists: {}", demangle(typeid(U).name()));
}

// 在盒子内部原地构造 Resource<U>，实参就是 main 传来的 server
auto& storage =
    it->second.template emplace<Resource<U>>(std::in_place, std::forward<Args>(args)...);

return storage.value;   // 返回容器内对象的引用
```

逐个零件：

| 片段 | 含义 |
|---|---|
| `it->second` | 条目的值部分——那个空盒子（UniqueAny） |
| `.template` | 消歧义：告诉编译器 `<` 是模板实参，不是小于号 |
| `emplace<Resource<U>>(...)` | 在盒子内部**原地**构造 Resource 包装器 |
| `std::in_place` | 原地构造标记：用后面的参数直接调构造函数，零临时对象、零搬运 |
| `std::forward<Args>(args)...` | 完美转发构造参数（左值拷贝、右值移动） |

盒子里的最终形态：

```cpp
Resource<shared_ptr<FoxgloveServer>> {
    .value = server 的拷贝   // 拷贝的是 shared_ptr（引用计数+1），FoxgloveServer 本体没动
}
```

**为什么敢丢弃返回值**：重复插入走 panic、冻结后调用也 panic，成功是唯一到达返回值的路径，所以 `insert_resource` 里的 `static_cast<void>(...)` 可以放心丢。

---

## 5. 第 5 站：运行时系统声明要用（可视化系统）

```cpp
// l3_estimation_systems.cpp
app.add_system<talos::pool_compute>("foxglove_l3_ekf_heatmap",
    [](talos::spmc<...> tracker_in,
       talos::res<std::shared_ptr<FoxgloveServer>> server) {   // ← 声明"我要这个资源"
        ...
});
```

调度器 bind 阶段看到这个参数，去 World 取句柄（world.hpp L707-710）：

```cpp
template <typename T>
[[nodiscard]] res<T> get_res() const noexcept {
    return res<T>{&resources_.template get<T>()};   // T 又是 shared_ptr<FoxgloveServer>！
}
```

---

## 6. 第 6 站：get 内部——与第 3 站镜像对称（world.hpp L410-415）

```cpp
template <typename T>
[[nodiscard]] const Resource<T>* get_storage() const noexcept {
    const auto it = resources_.find(typeid(T));     // ← L411：还是那个门牌号！
    if (it == resources_.end()) [[unlikely]] {      // ← 没注册过：返回空
        return nullptr;
    }
    return it->second.template get<Resource<T>>();  // ← 打开同一个盒子，取出同一张卡
}
```

**关键对照**——存与取用的是**同一个键**：

```
第 3 站（存）：try_emplace( typeid(shared_ptr<FoxgloveServer>) )   ← U 来自 insert_resource 的推导
第 6 站（取）：find        ( typeid(shared_ptr<FoxgloveServer>) )   ← T 来自系统参数的推导

两个 typeid 是同一种类型的身份证 → 必然相等 → find 必然命中第 3 站占的坑
```

这就是为什么 main 里 `insert_resource(server)` 存的东西，系统里写个 `res<shared_ptr<FoxgloveServer>>` 参数就能拿到——**类型推导保证了钥匙必然配对**，map 只负责保管。

---

## 7. 第 7 站：系统里实际使用

```cpp
// 系统 lambda 里
server->enqueue_message(std::move(msg));   // 通过句柄调用 FoxgloveServer 的方法
```

`server->` 最终摸到的堆上对象，**就是第 1 站 main 创建的那个 FoxgloveServer**——中途没有任何新对象被拷贝构造，只有 shared_ptr 的引用计数涨过。

---

## 8. 内存视角：数据到底存到哪了

### 情况 A：shared_ptr 资源（对象本体根本没动）

```
main 栈区                          堆区
┌─────────────────┐
│ server           │───┐        ┌──────────────────────────┐
│ (shared_ptr 对象)│   ├───────►│ 控制块 {计数=2}           │
└─────────────────┘   │        │ FoxgloveServer 本体       │ ← 它从来没动过！
                      │        └──────────────────────────┘
World 内部             │                    ▲
┌─────────────────┐   │                    │
│ resources_ 条目  │   │                    │
│  键: typeid(...) │   └────────────────────┤
│  值: 盒子        │───► 拷贝了一份 shared_ptr┘  ← insert_resource 真正做的事
└─────────────────┘         (引用计数 1→2)
```

### 情况 B：值类型资源（数据真的搬家了）

```cpp
insert_resource(std::move(some_config));   // 右值
```

移动构造：数据从旧位置搬到 UniqueAny 在堆上新开的内存，**World 的盒子成为它唯一的主人**。移动构造只搬指针不搬数据，成本极低。

### 汇总

```
main 栈上: scheduler 对象
             └─ 里面有个 World
                  └─ 里面有个 resources_ (map)
                       └─ 每个条目 = { 类型钥匙, 盒子(UniqueAny) }
                            └─ 盒子在【堆】上挂着一小块内存
                                 └─ 里面装着 Resource<U>{ .value = 你的资源 }
```

- **传智能指针**：本体在堆上没动，map 里只多了个"遥控器"副本
- **传值/右值**：数据本体被移动构造进 UniqueAny 的堆内存，归 World 管
- **取的时候**：拿同一个 typeid 找到同一个盒子，取出 `.value` 的引用——还是那一块内存

---

## 9. 全链一张图

```
main.cpp                          world.hpp 资源池                 系统运行时
─────────                          ──────────────                  ─────────
server (shared_ptr) ──insert──► map{ typeid(S) : 盒子 ─► 卡片.value } ◄──find── res<shared_ptr<S>>
      │                                （同一个键 typeid(S)）              │
      └──────────────── 堆上同一个 FoxgloveServer 对象 ◄────────────────────┘
```

七站速查：

| 站点 | 文件:行号 | 动作 |
|---|---|---|
| 1 | main.cpp:125,131 | 创建 server，调 insert_resource |
| 2 | world.hpp:619-623 | 推导 T、剥壳得 U |
| 3 | world.hpp:368 | try_emplace(typeid(U)) 占坑 |
| 4 | world.hpp:387-390 | 盒内原地构造 Resource\<U\>，返回 .value |
| 5 | l3_estimation_systems.cpp | 系统参数声明 res\<T\> |
| 6 | world.hpp:410-415 | find(typeid(T)) 打开同一个盒子 |
| 7 | 系统 lambda 内 | 通过句柄操作同一个堆上对象 |

---

## 10. 相关机制备忘

- **重复注册 → panic**：`emplace` 中 `if (!inserted) [[unlikely]] panic(...)`，保证全局资源单例
- **结构冻结**：`scheduler.build()` 之后资源结构冻结，再调 insert_resource 会 panic（见 world.hpp 生命周期图 L599-608）
- **`[[nodiscard]]`**：`emplace` 返回 `T&`，强制调用方接收，丢掉返回值编译报错
- **`[[unlikely]]`**：标记冷路径（错误分支），让编译器优化偏向正常路径
- **`.template` 消歧义**：依赖类型的成员模板调用必须写 `.template`，否则 `<` 被当作小于号
- **demangle**：错误信息里用 `demangle(typeid(U).name())` 把编译器修饰名还原成可读类型名（crates/scheduler/src/scheduler/demangle.cpp）
