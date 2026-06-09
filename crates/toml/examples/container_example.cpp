// Example usage of reflection-based parsing in toml_helper.
//
// This example demonstrates how to use:
// - plain T with member initializers for missing-key defaults
// - toml_helper::required<T> for required fields
// - std::optional<T> for true optional fields
// - std::vector<T> and std::array<T, N>
// - toml_helper::flatten<T> for serde-like flattened structs
// - merge_configs(base, override) for file-level defaults and overrides

#include <array>
#include <fmt/format.h>
#include <iostream>
#include <optional>
#include <vector>

#include "toml/core.hpp"
#include "toml/ext/containers.hpp"

struct CameraIntrinsics {
    toml_helper::required<double> fx{};
    toml_helper::required<double> fy{};
    std::optional<double> cx{};
    double cy{540.0};
};

struct RobotConfig {
    std::string team_color{"blue"};
    std::optional<double> max_speed{};

    std::vector<int> camera_ids;
    std::vector<std::string> allies;

    std::array<double, 3> position{};
    std::array<double, 4> orientation{};

    toml_helper::flatten<CameraIntrinsics> camera{};
};

int main() {
    constexpr std::string_view base_toml = R"(
        max_speed = 8.0
        camera_ids = [0, 1, 2]
        allies = ["robot1", "robot2"]
        position = [1.0, 2.0, 3.0]
        orientation = [1.0, 0.0, 0.0, 0.0]
        cy = 540.0
    )";

    constexpr std::string_view override_toml = R"(
        fx = 920.0
        fy = 918.0
        cx = 640.0
    )";

    auto base = toml::parse(base_toml);
    if (!base) {
        std::cerr << "Failed to parse base TOML\n";
        return 1;
    }

    auto override_cfg = toml::parse(override_toml);
    if (!override_cfg) {
        std::cerr << "Failed to parse override TOML\n";
        return 1;
    }

    auto merged = toml_helper::merge_configs(base.table(), override_cfg.table());
    if (!merged) {
        std::cerr << fmt::format("Failed to merge config: {}\n", merged.error());
        return 1;
    }

    auto config = toml_helper::from_table<RobotConfig>(*merged);
    if (!config) {
        std::cerr << fmt::format("Failed to read config: {}\n", config.error());
        return 1;
    }

    std::cout << "=== Robot Configuration ===\n";
    std::cout << fmt::format("Team Color: {}\n", config->team_color);
    std::cout << fmt::format(
        "Max Speed: {}\n",
        config->max_speed ? fmt::format("{} m/s", *config->max_speed) : "<none>");
    std::cout << fmt::format("Camera IDs: [{}]\n", fmt::join(config->camera_ids, ", "));
    std::cout << fmt::format("Allies: [{}]\n", fmt::join(config->allies, ", "));
    std::cout << fmt::format(
        "Position: [{}, {}, {}]\n", config->position[0], config->position[1], config->position[2]);
    std::cout << fmt::format(
        "Orientation: [{}, {}, {}, {}]\n", config->orientation[0], config->orientation[1],
        config->orientation[2], config->orientation[3]);
    std::cout << fmt::format(
        "Camera Intrinsics: fx={}, fy={}, cx={}, cy={}\n", config->camera->fx.get(),
        config->camera->fy.get(),
        config->camera->cx ? fmt::format("{}", *config->camera->cx) : std::string("<none>"),
        fmt::format("{}", config->camera->cy));

    return 0;
}
