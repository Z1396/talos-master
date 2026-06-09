# toml-helper 快速参考

TOML 配置解析 - `READ` 宏, `merge_configs`, `read_from`, `from_table`

## 30秒速查

| 我想...              | 代码                                             |
| -------------------- | ------------------------------------------------ |
| **读取必需字段**     | `READ(tbl, variable);`                           |
| **读取可选字段**     | `READ_OPT(tbl, variable);`                       |
| **平铺读取嵌套结构** | `READ_FLATTEN(tbl, nested_struct);`              |
| **反射式读取对象**   | `auto cfg = toml_helper::from_table<Config>(tbl);` |
| **写入已有对象**     | `toml_helper::read_into(tbl, cfg);`（完整解析，不是 patch） |
| **检查是否有未读键** | `ensure_all_read(tbl, "context");`               |
| **合并两个配置文件** | `merge_configs(base.table(), override.table());` |
| **解析 TOML 文件**   | `auto tbl = toml::parse_file("path.toml");`      |
| **读取嵌套表**       | `READ(tbl, inner_struct);` (自动递归)            |
| **读取枚举**         | `READ(tbl, enum_var);` (支持字符串/数字)         |
| **读取 std::optional** | `READ_OPT(tbl, optional_var);` (缺失键时为空) |
| **读取 std::vector** | `READ(tbl, vector_var);` (动态数组)              |
| **读取 std::array**  | `READ(tbl, array_var);` (固定大小数组)           |

## 头文件

```cpp
#include "toml/core.hpp"
#include "toml/ext/containers.hpp" // std::optional / std::vector / std::array
#include "toml/ext/eigen.hpp"      // 需要 Eigen 扩展时再引入
```

兼容旧代码时，仍可使用 `toml_helper.hpp`、`toml_helper_containers.hpp`、`toml_helper_eigen.hpp`。

---

## 反射式无样板写法

```cpp
#include "toml/core.hpp"
#include "toml/ext/containers.hpp"

struct CameraIntrinsics {
    toml_helper::required<double> fx{};
    toml_helper::required<double> fy{};
    std::optional<double> cx{};
    double cy{540.0};
};

struct CameraConfig {
    std::string model{"HIK"};
    toml_helper::flatten<CameraIntrinsics> camera{};
    int fps{60};
};

auto base = toml::parse_file("camera_base.toml");
auto override_tbl = toml::parse_file("camera_override.toml");
auto merged = toml_helper::merge_configs(base.table(), override_tbl.table());
auto cfg = toml_helper::from_table<CameraConfig>(*merged);
```

反射模式的缺失键语义：

- plain `T` 缺失时保留成员默认值
- `toml_helper::required<T>` 缺失时报错
- `std::optional<T>` 缺失时得到 `nullopt`
- `toml_helper::flatten<T>` 从当前表直接消费 `T` 的字段
- 文件级 `merge_configs(base, override)` 仍然适合做 base + override 组合；`required<T>` 适合表达必须显式提供的字段

旧的 `read_from + READ/READ_OPT/READ_FLATTEN` 仍然可用，适合保留特殊解析逻辑的类型。

---

## 完整配置系统示例

```cpp
#include "toml/core.hpp"
#include "toml/ext/containers.hpp"
#include <optional>
#include <vector>

// 场景: 装甲板追踪器配置系统
// - 支持多层级嵌套配置
// - 支持可选参数和默认值
// - 支持多文件合并 (base + robot + 算法)
// - 支持枚举类型 (字符串/数字)

enum class TrackerType {
    JPDA,        // 联合概率数据关联
    NearestNeighbor,
    MultipleHypothesis
};

enum class MotionModel {
    CV,          // 恒速模型
    CA,          // 恒加速模型
    CT           // 恒转速率模型
};

struct MotionModelConfig {
    double process_noise_pos = 0.1;      // 位置过程噪声
    double process_noise_vel = 0.5;      // 速度过程噪声
    double measurement_noise = 0.05;     // 测量噪声

    std::expected<void, std::string> read_from(const toml::table& tbl) noexcept {
        // 读取必需字段 (缺失则报错)
        READ(tbl, process_noise_pos);
        READ(tbl, process_noise_vel);
        READ(tbl, measurement_noise);

        return {};
    }
};

// 中层配置: 单个目标类型的追踪参数
struct TargetConfig {
    // 追踪器参数
    double lost_threshold = 0.5;         // 丢失阈值
    double tracking_threshold = 2.0;     // 追踪阈值
    double matcher_gate = 1.0;           // 关联门限

    // 嵌套的运动模型配置
    MotionModelConfig model;

    std::expected<void, std::string> read_from(const toml::table& tbl) noexcept {
        // 读取必需字段
        READ(tbl, lost_threshold);
        READ(tbl, tracking_threshold);
        READ(tbl, matcher_gate);
        // 读取嵌套表 (自动递归解析)
        READ(tbl, model);
        return {};
    }
};

// 外层配置: 完整追踪器配置
struct TrackerConfig {
    // 不同目标的配置
    TargetConfig robot;                  // 机器人目标
    TargetConfig outpost;                // 前哨站目标
    TargetConfig hero;                   // 英雄目标

    // 全局开关
    bool inekf_enabled = false;          // 是否启用 InEKF
    bool enable_visualization = false;   // 是否可视化

    // 追踪器类型 (枚举)
    TrackerType tracker_type = TrackerType::JPDA;

    // 可选的调试参数
    std::optional<std::string> log_file;      // 日志文件路径
    std::optional<int> debug_level;           // 调试级别

    std::expected<void, std::string> read_from(const toml::table& tbl) noexcept {
        // 读取嵌套配置 (自动递归)
        READ(tbl, robot);          // 读取 [robot] 表
        READ(tbl, outpost);        // 读取 [outpost] 表
        READ(tbl, hero);           // 读取 [hero] 表

        // 读取全局开关
        READ(tbl, inekf_enabled);
        READ(tbl, enable_visualization);

        // 读取枚举 (支持字符串或数字)
        READ(tbl, tracker_type);

        // 读取可选字段 (缺失不影响)
        READ_OPT(tbl, log_file);
        READ_OPT(tbl, debug_level);

        return {};
    }
};

// ============================================================================
// 辅助配置: 相机内参 (平铺结构)
// ============================================================================

struct CameraIntrinsics {
    int width = 1920;
    int height = 1080;
    double fx = 1000.0;
    double fy = 1000.0;
    double cx = 960.0;
    double cy = 540.0;

    std::expected<void, std::string> read_from(const toml::table& tbl) noexcept {
        // 平铺读取: 所有字段在同一个表中,无需 [camera] 子表
        READ(tbl, width);
        READ(tbl, height);
        READ(tbl, fx);
        READ(tbl, fy);
        READ(tbl, cx);
        READ(tbl, cy);

        return {};
    }
};

// ============================================================================
// 平铺配置包装: 在外层表中使用平铺内参
// ============================================================================

struct VisionConfig {
    std::string camera_model = "HIK-2030";

    // 平铺嵌入: camera 参数直接在 vision 表中
    CameraIntrinsics camera;

    // 其他配置
    double exposure_time = 5000.0;
    double gain = 10.0;

    std::expected<void, std::string> read_from(const toml::table& tbl) noexcept {
        READ(tbl, camera_model);
        READ_FLATTEN(tbl, camera);      // 平铺读取 camera 配置
        READ(tbl, exposure_time);
        READ(tbl, gain);
        return {};
    }
};

// ============================================================================
// 配置加载函数: 单文件
// ============================================================================

std::expected<TrackerConfig, std::string>
    load_tracker_config(const std::string& path) {

    // ------------------------------------------------------------
    // 解析 TOML 文件
    // ------------------------------------------------------------
    auto tbl_result = toml::parse_file(path);
    if (!tbl_result) {
        return std::unexpected("Failed to parse TOML: " + tbl_result.error());
    }

    // ------------------------------------------------------------
    // 读取配置
    // ------------------------------------------------------------
    TrackerConfig config;
    auto result = config.read_from(tbl_result.table());

    if (!result) {
        return std::unexpected(result.error());
    }

    // ------------------------------------------------------------
    // 检查是否有未读的键 (防止配置拼写错误)
    // ------------------------------------------------------------
    auto check = toml_helper::ensure_all_read(tbl_result.table(), "tracker");
    if (!check) {
        return std::unexpected(check.error());
    }

    return config;
}

// ============================================================================
// 配置加载函数: 多文件合并
// ============================================================================

std::expected<TrackerConfig, std::string>
    load_merged_tracker_config(
        const std::string& base_path,
        const std::string& robot_path,
        const std::string& algorithm_path
    ) {
    // ------------------------------------------------------------
    // 解析基础配置 (通用参数)
    // ------------------------------------------------------------
    auto base_tbl = toml::parse_file(base_path);
    if (!base_tbl) {
        return std::unexpected("Failed to parse base config");
    }

    // ------------------------------------------------------------
    // 解析机器人配置 (硬件相关)
    // ------------------------------------------------------------
    auto robot_tbl = toml::parse_file(robot_path);
    if (!robot_tbl) {
        return std::unexpected("Failed to parse robot config");
    }

    // ------------------------------------------------------------
    // 解析算法配置 (算法相关)
    // ------------------------------------------------------------
    auto algo_tbl = toml::parse_file(algorithm_path);
    if (!algo_tbl) {
        return std::unexpected("Failed to parse algorithm config");
    }

    // ------------------------------------------------------------
    // 合并配置: base + robot + algorithm
    // 后面的覆盖前面的
    // ------------------------------------------------------------
    auto merged = toml_helper::merge_configs(
        base_tbl.table(),
        robot_tbl.table()
    );

    if (!merged) {
        return std::unexpected(merged.error());
    }

    auto merged_again = toml_helper::merge_configs(
        *merged,
        algo_tbl.table()
    );

    if (!merged_again) {
        return std::unexpected(merged_again.error());
    }

    // ------------------------------------------------------------
    // 读取合并后的配置
    // ------------------------------------------------------------
    TrackerConfig config;
    auto result = config.read_from(*merged_again);

    if (!result) {
        return std::unexpected(result.error());
    }

    // ------------------------------------------------------------
    // 检查未读键
    // ------------------------------------------------------------
    auto check = toml_helper::ensure_all_read(*merged_again, "merged_tracker");
    if (!check) {
        return std::unexpected(check.error());
    }

    return config;
}

// ============================================================================
// 配置数组类型
// ============================================================================

struct FilterConfig {
    std::vector<int> kernel_size{3, 3};
    std::vector<double> weights;

    std::expected<void, std::string> read_from(const toml::table& tbl) noexcept {
        READ(tbl, kernel_size);
        READ(tbl, weights);
        return {};
    }
};

struct PreprocessConfig {
    FilterConfig blur_filter;
    FilterConfig edge_filter;

    std::expected<void, std::string> read_from(const toml::table& tbl) noexcept {
        READ(tbl, blur_filter);
        READ(tbl, edge_filter);
        return {};
    }
};
```

---

## 对应的 TOML 配置文件

```toml
# ============================================================================
# config/tracker_base.toml (基础配置)
# ============================================================================

[robot]
lost_threshold = 0.5
tracking_threshold = 2.0
matcher_gate = 1.0

[robot.model]
process_noise_pos = 0.1
process_noise_vel = 0.5
measurement_noise = 0.05

[outpost]
lost_threshold = 1.0
tracking_threshold = 3.0
matcher_gate = 1.5

[outpost.model]
process_noise_pos = 0.05
process_noise_vel = 0.3
measurement_noise = 0.02

[hero]
lost_threshold = 0.3
tracking_threshold = 1.5
matcher_gate = 0.8

[hero.model]
process_noise_pos = 0.08
process_noise_vel = 0.4
measurement_noise = 0.03

inekf_enabled = true
enable_visualization = true

# 枚举支持字符串 (推荐)
tracker_type = "JPDA"

# 或数字 (等价)
# tracker_type = 0
```

```toml
# ============================================================================
# config/robot/hero.toml (机器人配置,覆盖基础配置)
# ============================================================================

# 覆盖 hero 的 matcher_gate
[hero]
matcher_gate = 0.5

# 新增调试配置
log_file = "/var/log/tracker.log"
debug_level = 2
```

```toml
# ============================================================================
# config/vision.toml (相机配置,平铺结构)
# ============================================================================

[camera]
model = "HIK-2030"
width = 1920
height = 1080
fx = 1200.0
fy = 1200.0
cx = 960.0
cy = 540.0

# 暴光参数 (在同一层级)
exposure_time = 8000.0
gain = 8.0
```

---

## 容器类型支持

`toml_helper` 支持以下标准容器类型的自动解析：

### std::optional<T>

表示可选值。对于标准 TOML 配置:

1. **键不存在** → `std::nullopt`
2. **有效值** → `T`

实现层也会把 `toml::node_type::none` 映射为 `std::nullopt`。

```cpp
struct CameraConfig {
    std::optional<std::string> serial_number;  // 可选序列号
    std::optional<int> exposure;               // 可选曝光时间
    std::optional<double> gain;                // 可选增益

    std::expected<void, std::string> read_from(const toml::table& tbl) noexcept {
        READ_OPT(tbl, serial_number);
        READ_OPT(tbl, exposure);
        READ_OPT(tbl, gain);
        return {};
    }
};
```

**对应的 TOML:**

```toml
# serial_number 缺失 → std::nullopt
exposure = 8000
gain = 1.25
```

### std::vector<T>

动态大小数组,支持任意长度:

```cpp
struct MultiCameraConfig {
    std::vector<int> camera_ids;               // 相机 ID 列表
    std::vector<std::string> camera_models;    // 相机型号列表
    std::vector<double> exposure_times;        // 曝光时间列表

    std::expected<void, std::string> read_from(const toml::table& tbl) noexcept {
        READ(tbl, camera_ids);
        READ(tbl, camera_models);
        READ(tbl, exposure_times);
        return {};
    }
};
```

**对应的 TOML:**

```toml
camera_ids = [0, 1, 2]
camera_models = ["HIK-2030", "HIK-2030", "Basler"]
exposure_times = [5000.0, 8000.0, 10000.0]
```

**嵌套 ReadFrom 对象:**

```cpp
struct Point {
    int x, y;
    std::expected<void, std::string> read_from(const toml::table& tbl) noexcept {
        READ(tbl, x);
        READ(tbl, y);
        return {};
    }
};

struct PathConfig {
    std::vector<Point> waypoints;  // 航点列表

    std::expected<void, std::string> read_from(const toml::table& tbl) noexcept {
        READ(tbl, waypoints);
        return {};
    }
};
```

**对应的 TOML:**

```toml
waypoints = [
    { x = 0, y = 0 },
    { x = 1, y = 1 },
    { x = 2, y = 2 }
]
```

### std::array<T, N>

固定大小数组,**严格验证元素数量**:

```cpp
struct PoseConfig {
    std::array<double, 3> position;     // [x, y, z] 必须恰好 3 个元素
    std::array<double, 4> orientation;  // [w, x, y, z] 必须恰好 4 个元素
    std::array<int, 2> image_size;      // [width, height] 必须恰好 2 个元素

    std::expected<void, std::string> read_from(const toml::table& tbl) noexcept {
        READ(tbl, position);
        READ(tbl, orientation);
        READ(tbl, image_size);
        return {};
    }
};
```

**对应的 TOML:**

```toml
position = [1.0, 2.0, 3.0]
orientation = [1.0, 0.0, 0.0, 0.0]
image_size = [1920, 1080]
```

**错误示例:**

```toml
# ❌ 错误: position 需要恰好 3 个元素,但只有 2 个
position = [1.0, 2.0]

# ❌ 错误: image_size 需要恰好 2 个元素,但有 3 个
image_size = [1920, 1080, 30]
```

### 嵌套容器组合

支持复杂的嵌套容器结构:

```cpp
struct RobotGroupConfig {
    std::vector<std::string> names;                       // 机器人名称列表
    std::vector<std::array<double, 3>> positions;         // 每个机器人的位置
    std::optional<std::vector<double>> speeds;            // 整组速度配置 (可选)

    std::expected<void, std::string> read_from(const toml::table& tbl) noexcept {
        READ(tbl, names);
        READ(tbl, positions);
        READ_OPT(tbl, speeds);
        return {};
    }
};
```

**对应的 TOML:**

```toml
names = ["robot1", "robot2", "robot3"]
positions = [
    [0.0, 0.0, 0.0],
    [1.0, 1.0, 0.0],
    [2.0, 0.0, 0.0]
]
speeds = [1.5, 0.0, 2.0]
```

### 容器元素的递归支持

所有容器类型都支持递归解析,只要元素类型实现了 `Deserialize`:

- ✅ `std::vector<int>` - 基本类型
- ✅ `std::vector<std::string>` - 字符串
- ✅ `std::vector<ReadFromStruct>` - 嵌套结构体
- ✅ `std::vector<std::optional<int>>` - 嵌套 optional
- ✅ `std::array<MyConfig, 3>` - 固定数量配置对象
- ✅ `std::optional<std::vector<int>>` - 可选的向量

---

## 类型支持总结

| 类型               | 支持 | 说明                           |
| ------------------ | ---- | ------------------------------ |
| bool               | ✅   | 布尔值                         |
| 算术类型 (int, float) | ✅ | 数值,支持隐式转换            |
| std::string        | ✅   | 字符串                         |
| toml::date/time    | ✅   | 日期时间类型                   |
| enum class         | ✅   | 枚举,支持字符串/数字           |
| ReadFrom 结构体    | ✅   | 递归嵌套解析                   |
| Eigen 矩阵         | ✅   | 固定大小矩阵                   |
| **std::optional<T>**  | ✅   | **可选值 (缺失键时为空)**      |
| **std::vector<T>**    | ✅   | **动态数组**                   |
| **std::array<T, N>**  | ✅   | **固定大小数组 (严格验证)**    |
| std::variant<...>  | ❌   | 暂不支持 (计划中)              |
| std::map<K, V>    | ❌   | 暂不支持                       |
