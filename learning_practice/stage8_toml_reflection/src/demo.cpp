// ===========================================================================
// 阶段8：toml 模块 —— 结构体反射式 TOML 解析
//
// 文件对齐真实项目（crates/toml/src/）：
//   - toml/core.hpp + toml/ext/containers.hpp
//     from_table<T> 模板遍历结构体成员自动填充（反射辅助 field_reflection.hpp
//     底层是 Boost.PFR），四种字段语义：
//       普通成员        缺失用构造默认值
//       required<T>     必填，缺失 → 解析失败（fail-fast，对应项目
//                       "配置缺字段直接退出"的规范）
//       std::optional  可选，区分"未填写"与"显式赋值"
//       flatten<T>     serde 风格打平，子结构体字段直接是顶层 key
//   - merge_configs   分层合并：覆盖层同名 key 生效，未覆盖 key 保留 base
//
// 骨架取自 crates/toml/examples/container_example.cpp（六能力全演示），
// 改造成 4 个带断言的 test。
//
// 测试清单
// 测试1：全字段解析成功（普通默认值/required/optional/flatten/
//        vector/array 容器），对照示例输出
// 测试2：删掉 required 字段 → 解析失败，错误信息带字段名（fail-fast）
// 测试3：optional 语义：缺省 = nullopt vs 显式赋 0，两者可区分
// 测试4：merge_configs：覆盖层同名 key 生效、未覆盖 key 保留 base，
//        两层各出一半 required 字段也能合并成完整配置
// ===========================================================================

// TOML 基础解析（toml::parse）+ 反射核心（from_table）
#include "toml/core.hpp"
// 容器扩展（vector / array 反射）
#include "toml/ext/containers.hpp"

#include <array>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// 全局统一简化命名空间
using namespace toml_helper;

// ===========================================================================
// 轻量断言：失败打印位置并累计，main 末尾以非零退出码结束
// ===========================================================================
static int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::cerr << "  [CHECK 失败] " #cond "  (" << __FILE__ << ":" \
                      << __LINE__ << ")\n";                               \
            ++g_failures;                                                \
        }                                                                 \
    } while (0)

// ===========================================================================
// 结构体定义（对齐 crates/toml/tests/reflection_test.cpp 的三层结构）
// ===========================================================================

/// 串口子配置：baud 必填 / port 可选 / timeout_ms 默认 200
struct SerialConfig {
    required<int>       baud{};
    std::optional<int>  port{};
    int                 timeout_ms{200};
};

/// 相机内参（flatten 用）：fx/fy 必填 / cx 可选 / cy 默认 540
struct CameraIntrinsics {
    required<double>    fx{};
    required<double>    fy{};
    std::optional<double> cx{};
    double              cy{540.0};
};

/// 顶层相机配置：model 默认 HIK / 嵌套子表 [serial] / intrinsics 扁平化
struct CameraConfig {
    std::string                 model{"HIK"};
    SerialConfig                serial{};
    flatten<CameraIntrinsics>   intrinsics{};
    int                         fps{60};
};

/// 机器人总配置（对齐 container_example.cpp，覆盖容器能力）
struct RobotConfig {
    std::string               team_color{"blue"};
    std::optional<double>     max_speed{};
    std::vector<int>          camera_ids;
    std::vector<std::string>  allies;
    std::array<double, 3>     position{};
    std::array<double, 4>     orientation{};
};

// ===========================================================================
// 测试1：全字段解析成功 —— 六种能力一次跑通
// ===========================================================================
void test_full_parse() {
    std::cout << "=== 测试1：全字段解析成功 ===\n";

    const auto parsed = toml::parse(R"(
        model = "MV-CA013"
        fps = 120
        fx = 920.0
        fy = 918.0
        cx = 640.0
        cy = 512.5

        [serial]
        baud = 115200
        port = 2
    )");
    CHECK(static_cast<bool>(parsed)); // parse_result 用 operator bool

    auto result = from_table<CameraConfig>(parsed.table());
    CHECK(result.has_value());
    if (!result) {
        std::cout << "  错误: " << result.error() << "\n";
        return;
    }

    // 普通 string：TOML 覆盖默认 "HIK"
    CHECK(result->model == "MV-CA013");
    // 普通 int：TOML 覆盖默认 60
    CHECK(result->fps == 120);
    // 嵌套子表 [serial]
    CHECK(result->serial.baud.get() == 115200);
    CHECK(result->serial.port.has_value() && *result->serial.port == 2);
    CHECK(result->serial.timeout_ms == 200); // TOML 缺失 → 默认值
    // flatten 扁平化：fx/fy/cx/cy 直接是顶层 key
    CHECK(result->intrinsics->fx.get() == 920.0);
    CHECK(result->intrinsics->fy.get() == 918.0);
    CHECK(result->intrinsics->cx.has_value() && *result->intrinsics->cx == 640.0);
    CHECK(result->intrinsics->cy == 512.5);

    // 容器扩展（RobotConfig）
    const auto robot = toml::parse(R"(
        team_color = "red"
        max_speed = 8.0
        camera_ids = [0, 1, 2]
        allies = ["robot1", "robot2"]
        position = [1.0, 2.0, 3.0]
        orientation = [1.0, 0.0, 0.0, 0.0]
    )");
    CHECK(static_cast<bool>(robot));
    auto rc = from_table<RobotConfig>(robot.table());
    CHECK(rc.has_value());
    if (rc) {
        CHECK(rc->team_color == "red");
        CHECK(rc->max_speed.has_value() && *rc->max_speed == 8.0);
        CHECK((rc->camera_ids == std::vector<int>{0, 1, 2}));
        CHECK((rc->allies == std::vector<std::string>{"robot1", "robot2"}));
        CHECK((rc->position == std::array<double, 3>{1.0, 2.0, 3.0}));
        CHECK((rc->orientation == std::array<double, 4>{1.0, 0.0, 0.0, 0.0}));
    }

    std::cout << "  Team Color: " << rc->team_color << "  (TOML 覆盖默认 blue)\n";
    std::cout << "  Max Speed : " << *rc->max_speed << " m/s\n";
    std::cout << "  Camera IDs: [" << rc->camera_ids[0] << ", " << rc->camera_ids[1]
              << ", " << rc->camera_ids[2] << "]\n";
    std::cout << "  Camera Intrinsics: fx=" << result->intrinsics->fx.get()
              << ", fy=" << result->intrinsics->fy.get()
              << ", cx=" << *result->intrinsics->cx
              << ", cy=" << result->intrinsics->cy << "  (flatten 打平)\n";
    std::cout << "  Serial: baud=" << result->serial.baud.get()
              << ", port=" << *result->serial.port
              << ", timeout_ms=" << result->serial.timeout_ms << "  (默认值)\n";
    std::cout << "测试1通过\n\n";
}

// ===========================================================================
// 测试2：required 字段缺失 → 解析失败，错误信息带字段名（fail-fast）
// 对应项目规范：启动时配置缺字段直接退出，绝不带默认值凑合跑
// ===========================================================================
void test_required_missing() {
    std::cout << "=== 测试2：required 缺失 fail-fast ===\n";

    // 删掉了 [serial].baud（required<int>）
    const auto parsed = toml::parse(R"(
        model = "MV-CA013"
        fx = 920.0
        fy = 918.0

        [serial]
        port = 2
    )");
    CHECK(static_cast<bool>(parsed)); // parse_result 用 operator bool

    const auto result = from_table<CameraConfig>(parsed.table());
    CHECK(!result.has_value());                     // 必须失败
    CHECK(result.error().find("baud")               // 错误信息必须带字段名
          != std::string::npos);

    std::cout << "  解析失败（预期）: " << result.error() << "\n";
    std::cout << "  错误信息包含字段名 baud → fail-fast 生效\n";
    std::cout << "测试2通过\n\n";
}

// ===========================================================================
// 测试3：optional 语义 —— "未填写"与"显式赋 0"可区分
// 这是 std::optional 相比"用 0/‐1 当哨兵值"的本质优势
// ===========================================================================
void test_optional_semantics() {
    std::cout << "=== 测试3：optional 未填写 vs 显式赋值 ===\n";

    // cx 未填写 → nullopt
    const auto no_cx = toml::parse(R"(
        fx = 920.0
        fy = 918.0
    )");
    CHECK(static_cast<bool>(no_cx));
    auto r1 = from_table<CameraConfig>(no_cx.table());
    CHECK(r1.has_value());
    if (r1) {
        CHECK(!r1->intrinsics->cx.has_value()); // 未填写 = nullopt
    }

    // cx 显式填 0 → 有值且为 0（不是"未填写"！）
    const auto zero_cx = toml::parse(R"(
        fx = 920.0
        fy = 918.0
        cx = 0.0
    )");
    CHECK(static_cast<bool>(zero_cx));
    auto r2 = from_table<CameraConfig>(zero_cx.table());
    CHECK(r2.has_value());
    if (r2) {
        CHECK(r2->intrinsics->cx.has_value());   // 显式赋值 = 有值
        CHECK(*r2->intrinsics->cx == 0.0);       // 且值就是 0
    }

    std::cout << "  未填写 cx : has_value = "
              << (r1->intrinsics->cx.has_value() ? "true" : "false") << " (nullopt)\n";
    std::cout << "  显式 cx=0: has_value = "
              << (r2->intrinsics->cx.has_value() ? "true" : "false")
              << ", 值 = " << *r2->intrinsics->cx << "\n";
    std::cout << "  两种状态可区分（哨兵值做不到）\n";
    std::cout << "测试3通过\n\n";
}

// ===========================================================================
// 测试4：merge_configs 分层合并
// base 出 fy（内参），override 出 fx —— 两层各出一半 required 字段，
// 合并后 from_table 成功；同名 key override 胜出，未覆盖 key 保留 base
// ===========================================================================
void test_merge_configs() {
    std::cout << "=== 测试4：merge_configs 分层合并 ===\n";

    // 注意：base 里不能放 CameraConfig 没有的 key —— from_table 严格模式
    // 会把未读 key 当错误（"Unread keys: 'team_color'"），这本身就是
    // 反射解析防拼写错误的一道保险
    const auto base = toml::parse(R"(
        model = "MV-CA013"
        fps = 60
        fy = 918.0
        cy = 480.0
    )");
    CHECK(static_cast<bool>(base));

    const auto override_cfg = toml::parse(R"(
        fps = 120
        fx = 920.0
        cx = 640.0
    )");
    CHECK(static_cast<bool>(override_cfg));

    // 合并：override 同名 key 覆盖 base
    auto merged = merge_configs(base.table(), override_cfg.table());
    CHECK(merged.has_value());

    // 合并前 base 缺 fx（required）→ 单独解析必然失败
    auto base_only = from_table<CameraConfig>(base.table());
    CHECK(!base_only.has_value());

    // 合并后 fx（override）+ fy（base）凑齐全部 required → 成功
    auto result = from_table<CameraConfig>(*merged);
    CHECK(result.has_value());
    if (result) {
        CHECK(result->fps == 120);              // 同名 key：override 胜出
        CHECK(result->model == "MV-CA013");     // 未覆盖：保留 base
        CHECK(result->intrinsics->fx.get() == 920.0); // override 层提供
        CHECK(result->intrinsics->fy.get() == 918.0); // base 层提供
        CHECK(result->intrinsics->cx.has_value());
        CHECK(result->intrinsics->cy == 480.0);      // base 层提供
    }

    // 额外发现：from_table 严格模式 —— 表里存在结构体没有的 key 也算错
    // （防配置文件拼写错误的保险：拼错的 key 不会被静默忽略）
    const auto typo = toml::parse(R"(
        fx = 920.0
        fy = 918.0
        modle = "MV-CA013"   # ← 拼错：应为 model
    )");
    CHECK(static_cast<bool>(typo));
    const auto typo_result = from_table<CameraConfig>(typo.table());
    CHECK(!typo_result.has_value());
    CHECK(typo_result.error().find("Unread") != std::string::npos);

    std::cout << "  base 单独解析: 失败（缺 fx，符合预期）\n";
    std::cout << "  合并后解析   : 成功\n";
    std::cout << "  fps = " << result->fps << "  (override 覆盖 base 的 60)\n";
    std::cout << "  model = " << result->model << "  (base 保留)\n";
    std::cout << "  fx = " << result->intrinsics->fx.get() << "  (override 层) + "
              << "fy = " << result->intrinsics->fy.get() << "  (base 层)\n";
    std::cout << "  拼错 key 检测: " << typo_result.error() << "\n";
    std::cout << "测试4通过\n\n";
}

// ===========================================================================
// 主函数：依次运行所有测试，任一断言失败返回非零
// ===========================================================================
int main() {
    test_full_parse();       // 1. 全字段解析成功
    test_required_missing(); // 2. required 缺失 fail-fast
    test_optional_semantics(); // 3. optional 未填写 vs 显式赋值
    test_merge_configs();    // 4. 分层合并

    if (g_failures == 0) {
        std::cout << "=== stage8 toml 模块全部测试通过 ===\n";
        return 0;
    }
    std::cerr << "=== stage8 失败断言数: " << g_failures << " ===\n";
    return 1;
}
