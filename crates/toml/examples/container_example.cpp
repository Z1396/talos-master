// toml_helper 基于反射自动解析配置的示例代码注释
//
// 本示例完整演示 toml_helper 工具库六大核心能力：
// 1. 普通成员变量带默认值：配置缺失key时使用构造初始化值
// 2. toml_helper::required<T> 强制必填字段，缺失直接解析报错
// 3. std::optional<T> 真正可选字段，配置有无均可，区分“未填写”与“显式赋值”
// 4. 标准容器：std::vector 动态数组、std::array 固定长度数组自动解析
// 5. toml_helper::flatten<T> 扁平化结构体，serde风格，子结构体字段不嵌套前缀
// 6. merge_configs 配置合并：基础默认配置 + 覆盖层配置，后者覆盖前者同名key

#include <array>         // 固定长度数组 std::array
#include <fmt/format.h>  // fmt格式化打印、字符串拼接
#include <iostream>      // 标准输入输出流 std::cout / std::cerr
#include <optional>      // 标准可选值 std::optional
#include <vector>        // 动态变长数组 std::vector

// TOML核心解析库基础API
#include "toml/core.hpp"
// TOML容器扩展解析（vector/array/optional等容器自动序列化反序列化）
#include "toml/ext/containers.hpp"

/**
 * @brief 相机内参子结构体
 * 用于演示 flatten 扁平化嵌套结构体、required必填、optional可选、基础默认值
 */
struct CameraIntrinsics {
    // 必填浮点字段，TOML中无fx键则解析直接失败抛出错误
    toml_helper::required<double> fx{};
    // 必填浮点fy，缺失报错
    toml_helper::required<double> fy{};
    // 真正可选字段：TOML无cx则为std::nullopt，有则存数值，区分“未配置”和“显式填0”
    std::optional<double> cx{};
    // 普通基础类型，构造默认值540.0；TOML无cy键自动使用该默认值
    double cy{540.0};
};

/**
 * @brief 机器人总配置根结构体
 * 顶层配置，包含字符串、数值、动态数组、定长数组、扁平化子结构体
 */
struct RobotConfig {
    // 普通字符串，默认队伍颜色blue，TOML无team_color时使用blue
    std::string team_color{"blue"};
    // 可选最大速度，不写则为空optional
    std::optional<double> max_speed{};

    // 动态整数数组，TOML数组映射vector<int>
    std::vector<int> camera_ids;
    // 动态字符串数组，机器人盟友名称列表
    std::vector<std::string> allies;

    // 3维定长数组：机器人世界坐标 x y z
    std::array<double, 3> position{};
    // 4维定长数组：四元数姿态 w x y z
    std::array<double, 4> orientation{};

    // flatten 扁平化子结构体：CameraIntrinsics内部fx/fy/cx/cy直接作为顶层key，
    // 不会生成camera.fx嵌套路径，等价于serde(flatten)
    toml_helper::flatten<CameraIntrinsics> camera{};
};

int main() {
    // 基础层TOML配置字符串：全局默认基础配置
    constexpr std::string_view base_toml = R"(
        max_speed = 8.0
        camera_ids = [0, 1, 2]
        allies = ["robot1", "robot2"]
        position = [1.0, 2.0, 3.0]
        orientation = [1.0, 0.0, 0.0, 0.0]
        cy = 540.0
    )";

    // 覆盖层TOML：用户自定义覆写配置，合并时会覆盖base同名key
    constexpr std::string_view override_toml = R"(
        fx = 920.0
        fy = 918.0
        cx = 640.0
    )";

    // 解析基础配置文本，返回expected<table>
    auto base = toml::parse(base_toml);
    // 解析失败打印错误退出
    if (!base) {
        std::cerr << "Failed to parse base TOML\n";
        return 1;
    }

    // 解析覆盖层配置
    auto override_cfg = toml::parse(override_toml);
    if (!override_cfg) {
        std::cerr << "Failed to parse override TOML\n";
        return 1;
    }

    // 合并两层配置：base为底层默认，override_cfg同名键会覆盖base的值
    // 返回expected<table>，合并失败携带错误信息
    auto merged = toml_helper::merge_configs(base.table(), override_cfg.table());
    if (!merged) {
        std::cerr << fmt::format("Failed to merge config: {}\n", merged.error());
        return 1;
    }

    // 从合并后的TOML表反射解析填充RobotConfig结构体
    // 自动识别required/optional/vector/array/flatten各类标记成员
    auto config = toml_helper::from_table<RobotConfig>(*merged);
    // 解析失败（必填字段缺失、类型不匹配等）打印错误退出
    if (!config) {
        std::cerr << fmt::format("Failed to read config: {}\n", config.error());
        return 1;
    }

    // 格式化打印完整解析后的配置
    std::cout << "=== Robot Configuration ===\n";
    // 队伍颜色（默认blue，本示例无覆盖）
    std::cout << fmt::format("Team Color: {}\n", config->team_color);
    // 最大速度：optional判断是否存在，存在输出数值，否则显示<none>
    std::cout << fmt::format(
        "Max Speed: {}\n",
        config->max_speed ? fmt::format("{} m/s", *config->max_speed) : "<none>");
    // 相机ID数组，fmt::join拼接逗号分隔
    std::cout << fmt::format("Camera IDs: [{}]\n", fmt::join(config->camera_ids, ", "));
    // 盟友列表字符串数组
    std::cout << fmt::format("Allies: [{}]\n", fmt::join(config->allies, ", "));
    // 三维位置数组打印
    std::cout << fmt::format(
        "Position: [{}, {}, {}]\n", config->position[0], config->position[1], config->position[2]);
    // 四元数姿态数组打印
    std::cout << fmt::format(
        "Orientation: [{}, {}, {}, {}]\n", config->orientation[0], config->orientation[1],
        config->orientation[2], config->orientation[3]);
    // 扁平化相机内参：fx/fy是required，cx可选，cy带默认值
    std::cout << fmt::format(
        "Camera Intrinsics: fx={}, fy={}, cx={}, cy={}\n", config->camera->fx.get(),
        config->camera->fy.get(),
        config->camera->cx ? fmt::format("{}", *config->camera->cx) : std::string("<none>"),
        fmt::format("{}", config->camera->cy));

    return 0;
}