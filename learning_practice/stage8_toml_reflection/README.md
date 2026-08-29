# stage8：toml 模块 —— 结构体反射式 TOML 解析

对应真实代码：[crates/toml/src/](../../crates/toml/src/)（INTERFACE header-only）
骨架取自 [container_example.cpp](../../crates/toml/examples/container_example.cpp)
（该示例本身就是六能力全演示），改造成 4 个带断言的 test。

## 功能分析

定义好 struct 之后 `from_table<T>(table)` 自动从 TOML 表填充，
不写一行解析代码。反射遍历由 [field_reflection.hpp](../../crates/toml/src/field_reflection.hpp)
（Boost.PFR 后端，`names_as_array` + `tuple_size_v`）完成，字段语义四种：

| 字段写法 | 语义 | 缺失时行为 |
|---|---|---|
| `int fps{60};` | 普通成员 | 用构造默认值 |
| `required<int> baud{};` | 必填 | 解析失败（fail-fast） |
| `std::optional<int> port{};` | 可选 | `nullopt`，与"显式填 0"可区分 |
| `flatten<CameraIntrinsics>` | serde 扁平化 | 子结构体字段直接是顶层 key |

另有 `merge_configs(base, override)` 分层合并、`vector`/`array` 容器扩展
（[toml/ext/containers.hpp](../../crates/toml/src/toml/ext/containers.hpp)）、
Eigen 扩展（本 demo 不涉及）。

## 读 demo 验证时发现的两个真实行为（写进了断言）

1. **`from_table` 是严格模式**：表里存在结构体没有的 key 也报错
   （`Unread keys: 'modle'(string)`）—— 拼错的配置项不会被静默忽略，
   这是防拼写错误的一道保险；也因此 merge 测试的 base 层不能塞
   目标结构体以外的 key。
2. **必填错误信息带完整字段路径**：`serial: Missing key 'baud'` ——
   嵌套子表能定位到具体字段，对应项目"配置缺字段直接退出"的规范。

## 运行方法

```bash
cd learning_practice/stage8_toml_reflection
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/toml_reflection_demo
```

依赖（CMake FetchContent 自动拉取，对齐项目 `dependencies.cmake`）：
tomlplusplus（同 commit `1c8b7466`，因项目 `common.hpp` 强制
`TOML_HEADER_ONLY=0` 而编译为静态库）、fmt 12.0.0、Boost.PFR 2.2.0
（项目 `3dparty/boost-pfr` 是空目录，主工程会退化为空桩）、
magic_enum（项目本地头文件）。

## 预期输出

```
=== 测试1：全字段解析成功 ===
  Team Color: red  (TOML 覆盖默认 blue)
  Max Speed : 8 m/s
  Camera IDs: [0, 1, 2]
  Camera Intrinsics: fx=920, fy=918, cx=640, cy=512.5  (flatten 打平)
  Serial: baud=115200, port=2, timeout_ms=200  (默认值)
测试1通过

=== 测试2：required 缺失 fail-fast ===
  解析失败（预期）: serial: Missing key 'baud'
  错误信息包含字段名 baud → fail-fast 生效
测试2通过

=== 测试3：optional 未填写 vs 显式赋值 ===
  未填写 cx : has_value = false (nullopt)
  显式 cx=0: has_value = true, 值 = 0
  两种状态可区分（哨兵值做不到）
测试3通过

=== 测试4：merge_configs 分层合并 ===
  base 单独解析: 失败（缺 fx，符合预期）
  合并后解析   : 成功
  fps = 120  (override 覆盖 base 的 60)
  model = MV-CA013  (base 保留)
  fx = 920  (override 层) + fy = 918  (base 层)
  拼错 key 检测: Unread keys: 'modle'(string)
测试4通过

=== stage8 toml 模块全部测试通过 ===
```

## 测试清单（src/demo.cpp）

| 测试 | 断言要点 |
|---|---|
| 1 全字段解析 | 默认值/覆盖、嵌套子表、flatten、vector/array 全部命中 |
| 2 required 缺失 | 解析失败且错误串含字段名 `baud` |
| 3 optional 语义 | 缺省 = `nullopt`；显式 `cx=0` = 有值且为 0 |
| 4 merge 分层合并 | 同名 key override 胜出、未覆盖保留 base；两层各出一半 required 可拼成完整配置；拼错 key 报 Unread |
