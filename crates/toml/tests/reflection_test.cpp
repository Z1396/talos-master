#include <gtest/gtest.h>

// fmt格式化库，用于打印required包装器、拼接错误信息
#include <fmt/format.h>

// 标准可选值 std::optional
#include <optional>
// 标准字符串
#include <string>
// std::move 移动语义
#include <utility>
// 动态数组std::vector（容器反射测试）
#include <vector>

// TOML基础扩展（标量、结构体反射基础解析）
#include "toml/ext/core.hpp"
// TOML容器扩展（vector/array/map反射）
#include "toml/ext/containers.hpp"

namespace {
// 全局统一简化命名空间，省略toml_helper::前缀
using namespace toml_helper;

/**
 * @brief 无默认构造类型，用于测试required<T>包装器自动补齐默认构造
 * 原生类型删除默认构造函数，无法直接实例化；
 * required<NoDefault>内部存储层提供默认初始化，外层可无参构造
 */
struct NoDefault {
    // 删除默认构造，原生类型不能 NoDefault{}
    NoDefault() = delete;
    // 仅支持带参数构造
    explicit NoDefault(int value)
        : value(value) {}

    int value;
};

// 编译期静态断言校验：原生类型无默认构造
static_assert(!std::default_initializable<NoDefault>);
// required包装后支持无参默认构造，符合语法要求
static_assert(std::default_initializable<required<NoDefault>>);

/**
 * @brief 串口子配置结构体
 * required<int> baud：波特率必填，TOML缺失直接报错
 * std::optional<int> port：端口可选，不写则无值
 * int timeout_ms{200}：普通基础类型，缺失使用默认值200
 */
struct SerialConfig {
    required<int> baud{};
    std::optional<int> port{};
    int timeout_ms{200};
};

/**
 * @brief 相机内参扁平化子结构体
 * fx/fy 像素焦距必填；cx可选；cy默认540
 */
struct CameraIntrinsics {
    required<double> fx{};
    required<double> fy{};
    std::optional<double> cx{};
    double cy{540.0};
};

/**
 * @brief 顶层相机总配置结构体
 * model：相机型号默认HIK
 * SerialConfig serial：嵌套子表 [serial]
 * flatten<CameraIntrinsics> intrinsics：扁平化，fx/fy/cx/cy直接顶层key，无嵌套前缀
 * fps 帧率默认60
 */
struct CameraConfig {
    std::string model{"HIK"};
    SerialConfig serial{};
    flatten<CameraIntrinsics> intrinsics{};
    int fps{60};
};

/**
 * @brief 路径点结构体，二维坐标，用于vector容器反射测试
 */
struct Waypoint {
    double x = 0.0;
    double y = 0.0;
};

/**
 * @brief 路径配置，包含std::vector<Waypoint>反射数组
 */
struct PathConfig {
    std::vector<Waypoint> points;
};

/**
 * @brief 枚举类型，用于枚举字面量/整数解析测试
 */
enum class PowerMode {
    Off = 0,
    On  = 1,
};

/**
 * @brief 测试1：from_table 基础读取，校验required/optional/普通默认值/flatten嵌套全部正常解析
 * 测试要点：
 * 1. 普通字符串默认值被TOML覆盖；
 * 2. required字段必须存在，读取成功；
 * 3. optional存在则赋值；
 * 4. 普通int缺失使用构造默认；
 * 5. flatten扁平化子结构体字段直接顶层读取；
 * 6. ensure_all_read 校验无冗余未读key
 */
TEST(TomlReflection, FromTableReadsRequiredOptionalAndPlainDefaultFields) {
    // 基础TOML文本：顶层字段+[serial]子表，扁平化fx/fy/cy顶层key
    const auto parsed = toml::parse(R"(
        model = "MV-CA013"
        fps = 60
        fx = 900.0
        fy = 901.0
        cx = 320.0

        [serial]
        baud = 115200
        port = 2
    )");
    // 解析TOML无语法错误
    ASSERT_TRUE(parsed);

    // 反射完整解析CameraConfig顶层结构体
    auto result = from_table<CameraConfig>(parsed.table());
    // 解析不能报错
    ASSERT_TRUE(result);

    // model被TOML覆盖为MV-CA013，不是默认HIK
    EXPECT_EQ(result->model, "MV-CA013");
    // fps匹配TOML 60
    EXPECT_EQ(result->fps, 60);
    // serial.baud必填，取值115200
    EXPECT_EQ(result->serial.baud.get(), 115200);
    // port存在optional有值
    ASSERT_TRUE(result->serial.port.has_value());
    EXPECT_EQ(*result->serial.port, 2);
    // timeout_ms无TOML键，使用默认200
    EXPECT_EQ(result->serial.timeout_ms, 200);
    // 扁平化fx/fy/cx/cy直接读取顶层
    EXPECT_DOUBLE_EQ(result->intrinsics->fx.get(), 900.0);
    EXPECT_DOUBLE_EQ(result->intrinsics->fy.get(), 901.0);
    ASSERT_TRUE(result->intrinsics->cx.has_value());
    EXPECT_DOUBLE_EQ(*result->intrinsics->cx, 320.0);
    // cy无TOML键，使用默认540
    EXPECT_DOUBLE_EQ(result->intrinsics->cy, 540.0);

    // 校验table内所有key均被读取，无冗余无效配置项
    auto check = ensure_all_read(parsed.table(), "camera");
    ASSERT_TRUE(check);
}

/**
 * @brief 测试2：read_into 覆写已有结构体，缺失字段重置为默认值
 * read_into 逻辑：
 * 1. 先将目标结构体全部重置为成员初始默认值；
 * 2. 再用TOML表覆盖存在的key；
 * 3. TOML不存在的字段保持默认，不保留旧对象残留值
 */
TEST(TomlReflection, ReadIntoReplacesExistingObjectAndResetsByFieldPolicy) {
    // TOML仅提供fx/fy/serial.baud，其余字段缺失
    const auto parsed = toml::parse(R"(
        fx = 700.0
        fy = 710.0

        [serial]
        baud = 115200
    )");
    ASSERT_TRUE(parsed);

    // 预先构造结构体，填充大量非默认残留值
    CameraConfig config;
    config.model             = "existing";
    config.serial.port       = 4;
    config.serial.timeout_ms = 200;
    config.intrinsics->cx    = 300.0;
    config.intrinsics->cy    = 540.0;
    config.fps               = 144;

    // 覆写已有config对象
    auto result = read_into(parsed.table(), config);
    ASSERT_TRUE(result);

    // model无TOML键，重置为默认"HIK"，不再保留旧值existing
    EXPECT_EQ(config.model, "HIK");
    // baud被TOML覆盖
    EXPECT_EQ(config.serial.baud.get(), 115200);
    // port无TOML键，重置为std::nullopt，旧值4丢失
    EXPECT_FALSE(config.serial.port.has_value());
    // timeout_ms默认200，无覆盖不变
    EXPECT_EQ(config.serial.timeout_ms, 200);
    // fx/fy被TOML覆盖
    EXPECT_DOUBLE_EQ(config.intrinsics->fx.get(), 700.0);
    EXPECT_DOUBLE_EQ(config.intrinsics->fy.get(), 710.0);
    // cx无TOML键，重置为空optional，旧300丢弃
    EXPECT_FALSE(config.intrinsics->cx.has_value());
    // cy默认540不变
    EXPECT_DOUBLE_EQ(config.intrinsics->cy, 540.0);
    // fps无TOML键，重置为默认60，旧144丢失
    EXPECT_EQ(config.fps, 60);

    auto check = ensure_all_read(parsed.table(), "camera");
    ASSERT_TRUE(check);
}

/**
 * @brief 测试3：read_into 解析失败时，目标结构体完全不被修改（原子事务）
 * 解析中途报错，不会半写污染原有对象
 */
TEST(TomlReflection, ReadIntoFailureDoesNotMutateDestination) {
    // TOML缺失必填fy，解析必然失败
    const auto parsed = toml::parse(R"(
        fx = 700.0

        [serial]
        baud = 115200
    )");
    ASSERT_TRUE(parsed);

    // 预填充结构体所有字段为自定义值
    CameraConfig config;
    config.model             = "existing";
    config.serial.baud       = 9600;
    config.serial.port       = 4;
    config.serial.timeout_ms = 250;
    config.intrinsics->fx    = 1.0;
    config.intrinsics->fy    = 2.0;
    config.intrinsics->cx    = 300.0;
    config.fps               = 144;

    // 执行覆写解析，预期失败
    auto result = read_into(parsed.table(), config);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "intrinsics: Missing key 'fy'");

    // 全部字段保持原有预填充值，无任何修改
    EXPECT_EQ(config.model, "existing");
    EXPECT_EQ(config.serial.baud.get(), 9600);
    ASSERT_TRUE(config.serial.port.has_value());
    EXPECT_EQ(*config.serial.port, 4);
    EXPECT_EQ(config.serial.timeout_ms, 250);
    EXPECT_DOUBLE_EQ(config.intrinsics->fx.get(), 1.0);
    EXPECT_DOUBLE_EQ(config.intrinsics->fy.get(), 2.0);
    ASSERT_TRUE(config.intrinsics->cx.has_value());
    EXPECT_DOUBLE_EQ(*config.intrinsics->cx, 300.0);
    EXPECT_EQ(config.fps, 144);
}

/**
 * @brief 测试4：flatten扁平化结构体缺失required字段，报错携带层级前缀intrinsics:
 * 区分顶层字段与扁平化嵌套字段的报错上下文路径
 */
TEST(TomlReflection, MissingRequiredFieldInsideFlattenIsPrefixed) {
    // TOML缺失扁平化必填fx，fy存在
    const auto parsed = toml::parse(R"(
        model = "MV-CA013"
        fy = 901.0

        [serial]
        baud = 115200
        port = 2
    )");
    ASSERT_TRUE(parsed);

    auto result = from_table<CameraConfig>(parsed.table());
    ASSERT_FALSE(result);
    // 错误上下文携带intrinsics: 标识扁平化子结构体内缺失key
    EXPECT_NE(result.error().find("intrinsics: Missing key 'fx'"), std::string::npos);
}

/**
 * @brief 测试5：take<T>读取嵌套table，子table存在未读取冗余key直接报错
 * ensure_all_tables_read开启，table内所有键必须被结构体成员读取，多余key视为配置错误
 */
TEST(TomlReflection, TakeReflectiveObjectRejectsUnreadKeysInNestedTable) {
    // serial子表额外存在extra字段，无对应结构体成员
    const auto parsed = toml::parse(R"(
        serial = { baud = 115200, port = 7, extra = 1 }
    )");
    ASSERT_TRUE(parsed);

    // take读取后删除table键，校验子表无未读key
    auto result = take<SerialConfig>(parsed.table(), "serial");
    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("serial: unread keys"), std::string::npos);
    EXPECT_NE(result.error().find("'extra'(integer)"), std::string::npos);
}

/**
 * @brief 测试6：顶层table存在未读取冗余key，from_table直接报错
 */
TEST(TomlReflection, FromTableRejectsUnreadTopLevelKeys) {
    // 顶层多余extra字段，无对应CameraConfig成员
    const auto parsed = toml::parse(R"(
        model = "MV-CA013"
        extra = 1
        fx = 900.0
        fy = 901.0

        [serial]
        baud = 115200
    )");
    ASSERT_TRUE(parsed);

    auto result = from_table<CameraConfig>(parsed.table());
    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("Unread keys"), std::string::npos);
    EXPECT_NE(result.error().find("'extra'(integer)"), std::string::npos);
}

/**
 * @brief 测试7：普通基础字段无TOML键，使用构造默认值，不报错
 */
TEST(TomlReflection, PlainFieldMayBeMissingAndKeepsInitializer) {
    struct PlainDefaultConfig {
        // 无required/optional，缺失使用默认7
        int port = 7;
    };

    // 空TOML，无任何键
    const auto parsed = toml::parse("");
    ASSERT_TRUE(parsed);

    auto result = from_table<PlainDefaultConfig>(parsed.table());
    ASSERT_TRUE(result);
    EXPECT_EQ(result->port, 7);
}

/**
 * @brief 测试8：required<T>字段TOML无对应key，解析失败
 */
TEST(TomlReflection, RequiredFieldMustBePresent) {
    struct RequiredConfig {
        required<int> port{};
    };

    const auto parsed = toml::parse("");
    ASSERT_TRUE(parsed);

    auto result = from_table<RequiredConfig>(parsed.table());
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Missing key 'port'");
}

/**
 * @brief 测试9：required包装器无值时调用get()/*/-> 抛出std::bad_optional_access
 * required内部基于optional实现，未赋值时访问成员触发异常
 */
TEST(TomlReflection, RequiredWrapperThrowsOnEmptyAccess) {
    required<int> port{};
    const required<int>& const_port = port;

    // 无有效值标记
    EXPECT_FALSE(port.has_value());
    // 四种访问方式全部抛异常
    EXPECT_THROW((void)port.get(), std::bad_optional_access);
    EXPECT_THROW((void)const_port.get(), std::bad_optional_access);
    EXPECT_THROW((void)port.operator->(), std::bad_optional_access);
    EXPECT_THROW((void)const_port.operator->(), std::bad_optional_access);
    EXPECT_THROW((void)*port, std::bad_optional_access);
}

/**
 * @brief 测试10：std::optional<T> 构造默认值不生效，TOML缺失则为std::nullopt
 * 区分普通int（默认生效）与optional<int>（默认初始化仅占位，无TOML键为空）
 */
TEST(TomlReflection, OptionalFieldMayBeMissingEvenWithEngagedInitializer) {
    struct OptionalConfig {
        // 构造默认7，但TOML缺失时依然是nullopt
        std::optional<int> port = 7;
    };

    const auto parsed = toml::parse("");
    ASSERT_TRUE(parsed);

    auto result = from_table<OptionalConfig>(parsed.table());
    ASSERT_TRUE(result);
    // 无TOML键，optional为空，不使用构造赋值7
    EXPECT_FALSE(result->port.has_value());
}

/**
 * @brief 测试11：merge_configs 分层配置合并，override覆盖base同名key，不冲突保留base
 * base：底层默认配置；override：上层用户自定义覆盖配置
 */
TEST(TomlReflection, MergeConfigsKeepsFileLevelOverrideSemantics) {
    // 基础层完整默认配置
    const auto base = toml::parse(R"(
        model = "MV-CA013"
        fps = 120
        fx = 900.0
        fy = 901.0
        cy = 540.0

        [serial]
        baud = 115200
        timeout_ms = 200
    )");
    ASSERT_TRUE(base);

    // 覆盖层仅写port，其余复用base
    const auto override_cfg = toml::parse(R"(
        [serial]
        port = 2
    )");
    ASSERT_TRUE(override_cfg);

    // 合并两层配置，override优先级更高
    auto merged = merge_configs(base.table(), override_cfg.table());
    ASSERT_TRUE(merged);

    auto result = from_table<CameraConfig>(*merged);
    ASSERT_TRUE(result);
    // model/fps/fx/fy/cy/baud/timeout_ms 继承base
    EXPECT_EQ(result->model, "MV-CA013");
    EXPECT_EQ(result->fps, 120);
    // port被override新增
    ASSERT_TRUE(result->serial.port.has_value());
    EXPECT_EQ(*result->serial.port, 2);
    EXPECT_EQ(result->serial.timeout_ms, 200);
    EXPECT_DOUBLE_EQ(result->intrinsics->cy, 540.0);
}

/**
 * @brief 测试12：std::vector<自定义反射结构体> 容器数组解析
 * TOML数组内嵌套table，自动映射Waypoint反射结构体
 */
TEST(TomlReflection, ContainerParsersSupportReflectiveElementTypes) {
    // 数组包含两个嵌套table点位
    const auto parsed = toml::parse(R"(
        points = [
            { x = 1.0, y = 2.0 },
            { x = 3.0, y = 4.0 }
        ]
    )");
    ASSERT_TRUE(parsed);

    auto result = from_table<PathConfig>(parsed.table());
    ASSERT_TRUE(result);
    // 数组长度2
    ASSERT_EQ(result->points.size(), 2U);
    // 坐标匹配
    EXPECT_DOUBLE_EQ(result->points[0].x, 1.0);
    EXPECT_DOUBLE_EQ(result->points[1].y, 4.0);

    auto check = ensure_all_read(parsed.table(), "path");
    ASSERT_TRUE(check);
}

/**
 * @brief 测试13：required/flatten包装器运算符、赋值、移动、fmt格式化完整可用性
 * 验证包装器重载运算符、拷贝赋值、移动语义、fmt打印全部正常
 */
TEST(TomlReflection, WrappersSupportOperatorsAssignmentsAndFormatting) {
    // 赋值构造required
    required<int> engaged{7};
    const required<int>& const_engaged = engaged;

    EXPECT_TRUE(engaged.has_value());
    EXPECT_EQ(engaged.get(), 7);
    EXPECT_EQ(const_engaged.get(), 7);
    // 解引用运算符
    EXPECT_EQ(*engaged, 7);
    EXPECT_EQ(*const_engaged, 7);

    // 隐式转const引用
    int& as_ref             = engaged;
    const int& as_const_ref = const_engaged;
    EXPECT_EQ(as_ref, 7);
    EXPECT_EQ(as_const_ref, 7);

    // -> 修改内部值
    *engaged.operator->() = 9;
    EXPECT_EQ(engaged.get(), 9);
    // fmt格式化直接打印内部数值
    EXPECT_EQ(fmt::format("{}", engaged), "9");

    // 移动语义取出内部值
    required<std::string> moved{std::string("ready")};
    std::string moved_value = std::move(moved).get();
    EXPECT_EQ(moved_value, "ready");

    // flatten包装器赋值、->、*运算符
    flatten<CameraIntrinsics> intrinsics{};
    intrinsics->fx                  = 1.5;
    (*intrinsics).fy                = 2.5;
    // 转底层结构体引用
    CameraIntrinsics& as_intrinsics = intrinsics;
    EXPECT_DOUBLE_EQ(as_intrinsics.fx.get(), 1.5);
    EXPECT_DOUBLE_EQ(as_intrinsics.fy.get(), 2.5);

    // 整体结构体赋值给flatten包装器
    CameraIntrinsics replacement{};
    replacement.fx = 3.5;
    replacement.fy = 4.5;
    replacement.cx = 5.5;
    intrinsics     = replacement;
    EXPECT_DOUBLE_EQ(intrinsics->fx.get(), 3.5);
    EXPECT_DOUBLE_EQ(intrinsics->fy.get(), 4.5);
    ASSERT_TRUE(intrinsics->cx.has_value());
    EXPECT_DOUBLE_EQ(*intrinsics->cx, 5.5);
}

/**
 * @brief 测试14：反射结构体顶层Deserialize四种读取API：read/read_optional/take/take_optional
 * read：键必须存在；read_optional：键可选；take读取后删除key；take_optional可选删除
 */
TEST(TomlReflection, ReflectiveDeserializeSupportsReadOptionalAndTakeOptional) {
    // 顶层[camera]完整table，read强制读取
    const auto parsed_direct = toml::parse(R"(
        [camera]
        model = "MV-CA013"
        fps = 90
        fx = 800.0
        fy = 801.0

        [camera.serial]
        baud = 115200
    )");
    ASSERT_TRUE(parsed_direct);

    auto direct = Deserialize<CameraConfig>::read(parsed_direct.table(), "camera");
    ASSERT_TRUE(direct);
    EXPECT_EQ(direct->model, "MV-CA013");
    EXPECT_EQ(direct->fps, 90);
    EXPECT_EQ(direct->serial.baud.get(), 115200);

    // read_optional：键存在返回optional包裹结构体
    const auto parsed_optional = toml::parse(R"(
        [camera]
        model = "MV-CA013"
        fps = 90
        fx = 800.0
        fy = 801.0

        [camera.serial]
        baud = 115200
    )");
    ASSERT_TRUE(parsed_optional);

    auto optional_read =
        Deserialize<CameraConfig>::read_optional(parsed_optional.table(), "camera");
    ASSERT_TRUE(optional_read);
    ASSERT_TRUE(optional_read->has_value());
    EXPECT_EQ((*optional_read)->serial.baud.get(), 115200);

    // read_optional：键不存在返回外层nullopt
    const auto parsed_missing = toml::parse("");
    ASSERT_TRUE(parsed_missing);

    auto missing_optional =
        Deserialize<CameraConfig>::read_optional(parsed_missing.table(), "missing");
    ASSERT_TRUE(missing_optional);
    EXPECT_FALSE(missing_optional->has_value());

    // take_optional：读取后删除table内camera键
    auto parsed_take = toml::parse(R"(
        [camera]
        model = "MV-CA013"
        fx = 800.0
        fy = 801.0

        [camera.serial]
        baud = 115200
    )");
    ASSERT_TRUE(parsed_take);

    auto taken = Deserialize<CameraConfig>::take_optional(parsed_take.table(), "camera");
    ASSERT_TRUE(taken);
    ASSERT_TRUE(taken->has_value());
    EXPECT_EQ((*taken)->serial.baud.get(), 115200);
    // 键已被删除，get返回nullptr
    EXPECT_EQ(parsed_take.table().get("camera"), nullptr);
}

/**
 * @brief 测试15：Deserialize读取非table类型节点，返回类型不匹配错误
 */
TEST(TomlReflection, ReflectiveDeserializeRejectsNonTableValues) {
    // camera是数字，不是table
    const auto parsed = toml::parse("camera = 5");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<CameraConfig>::read(parsed.table(), "camera");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid value for key 'camera': expected table");
}

/**
 * @brief 测试16：required<T>专用Deserialize四种读取API兼容
 */
TEST(TomlReflection, RequiredDeserializerSupportsOptionalForms) {
    const auto parsed = toml::parse("value = 42");
    ASSERT_TRUE(parsed);

    // read强制读取required
    auto direct = Deserialize<required<int>>::read(parsed.table(), "value");
    ASSERT_TRUE(direct);
    EXPECT_EQ(direct->get(), 42);

    // read_optional
    auto optional_read = Deserialize<required<int>>::read_optional(parsed.table(), "value");
    ASSERT_TRUE(optional_read);
    ASSERT_TRUE(optional_read->has_value());
    EXPECT_EQ((*optional_read)->get(), 42);

    // take_optional，读取后删除key
    auto parsed_take = toml::parse("value = 42");
    ASSERT_TRUE(parsed_take);

    auto taken = Deserialize<required<int>>::take_optional(parsed_take.table(), "value");
    ASSERT_TRUE(taken);
    ASSERT_TRUE(taken->has_value());
    EXPECT_EQ((*taken)->get(), 42);
    EXPECT_EQ(parsed_take.table().get("value"), nullptr);

    // read_optional键不存在返回外层nullopt
    const auto missing = toml::parse("");
    ASSERT_TRUE(missing);

    auto missing_optional = Deserialize<required<int>>::read_optional(missing.table(), "value");
    ASSERT_TRUE(missing_optional);
    EXPECT_FALSE(missing_optional->has_value());
}

/**
 * @brief 测试17：枚举类型解析，支持整数字面量，拒绝非法字符串/数组类型
 */
TEST(TomlReflection, EnumDeserializerAcceptsIntegralValuesAndRejectsInvalidInputs) {
    // 合法整数1对应PowerMode::On
    const auto parsed_int = toml::parse("mode = 1");
    ASSERT_TRUE(parsed_int);

    auto int_result = read<PowerMode>(parsed_int.table(), "mode");
    ASSERT_TRUE(int_result);
    EXPECT_EQ(*int_result, PowerMode::On);

    // 非法字符串，无法转枚举
    const auto parsed_bad_string = toml::parse("mode = 'invalid'");
    ASSERT_TRUE(parsed_bad_string);

    auto bad_string = read<PowerMode>(parsed_bad_string.table(), "mode");
    ASSERT_FALSE(bad_string);
    EXPECT_NE(
        bad_string.error().find("Invalid enum value for key 'mode': 'invalid'"), std::string::npos);

    // 数组类型，枚举仅支持int/string字面量
    const auto parsed_bad_type = toml::parse("mode = [1]");
    ASSERT_TRUE(parsed_bad_type);

    auto bad_type = read<PowerMode>(parsed_bad_type.table(), "mode");
    ASSERT_FALSE(bad_type);
    EXPECT_NE(
        bad_type.error().find("expected enum as string/integer, got array"), std::string::npos);
}

/**
 * @brief 测试18：三种字段类型（required/optional/plain）解析类型不匹配时统一透传错误
 */
TEST(TomlReflection, ReflectiveFieldKindsPropagateDeserializerFailures) {
    struct RequiredValueConfig {
        required<int> value{};
    };
    struct OptionalValueConfig {
        std::optional<int> value{};
    };
    struct PlainValueConfig {
        int value{};
    };

    // value为字符串，期望int，全部三种字段报错一致
    const auto parsed = toml::parse("value = 'oops'");
    ASSERT_TRUE(parsed);

    auto required_result = from_table<RequiredValueConfig>(parsed.table());
    ASSERT_FALSE(required_result);
    EXPECT_EQ(
        required_result.error(), "Invalid value for key 'value': expected integer, got string");

    const auto parsed_optional = toml::parse("value = 'oops'");
    ASSERT_TRUE(parsed_optional);

    auto optional_result = from_table<OptionalValueConfig>(parsed_optional.table());
    ASSERT_FALSE(optional_result);
    EXPECT_EQ(
        optional_result.error(), "Invalid value for key 'value': expected integer, got string");

    const auto parsed_plain = toml::parse("value = 'oops'");
    ASSERT_TRUE(parsed_plain);

    auto plain_result = from_table<PlainValueConfig>(parsed_plain);
    ASSERT_FALSE(plain_result);
    EXPECT_EQ(plain_result.error(), "Invalid value for key 'value': expected integer, got string");
}

/**
 * @brief 测试19：嵌套table缺失、扁平化子字段缺失，报错携带完整层级上下文路径
 */
TEST(TomlReflection, ReflectiveDeserializeReportsNestedAndMissingTableErrors) {
    // [camera]存在，内部扁平化fx缺失
    const auto parsed_nested = toml::parse(R"(
        [camera]
        fy = 801.0

        [camera.serial]
        baud = 115200
    )");
    ASSERT_TRUE(parsed_nested);

    auto nested_result = read<CameraConfig>(parsed_nested.table(), "camera");
    ASSERT_FALSE(nested_result);
    EXPECT_NE(
        nested_result.error().find("camera: intrinsics: Missing key 'fx'"), std::string::npos);

    // 顶层无camera子table
    const auto parsed_missing = toml::parse("");
    ASSERT_TRUE(parsed_missing);

    auto missing_result = read<CameraConfig>(parsed_missing.table(), "camera");
    ASSERT_FALSE(missing_result);
    EXPECT_EQ(missing_result.error(), "Missing table 'camera'");

    // camera是数字，不是table
    const auto parsed_take = toml::parse("camera = 5");
    ASSERT_TRUE(parsed_take);

    auto take_result = Deserialize<CameraConfig>::take(parsed_take.table(), "camera");
    ASSERT_FALSE(take_result);
    EXPECT_EQ(take_result.error(), "Invalid value for key 'camera': expected table");
}

/**
 * @brief 测试20：required专用Deserialize各类读取接口，解析失败统一透传错误
 */
TEST(TomlReflection, RequiredDeserializerPropagatesErrorsAndMissingStates) {
    // 类型错误字符串
    const auto parsed_invalid = toml::parse("value = 'oops'");
    ASSERT_TRUE(parsed_invalid);

    auto read_result = Deserialize<required<int>>::read(parsed_invalid.table(), "value");
    ASSERT_FALSE(read_result);
    EXPECT_EQ(read_result.error(), "Invalid value for key 'value': expected integer, got string");

    const auto parsed_optional_invalid = toml::parse("value = 'oops'");
    ASSERT_TRUE(parsed_optional_invalid);

    auto read_optional_result =
        Deserialize<required<int>>::read_optional(parsed_optional_invalid.table(), "value");
    ASSERT_FALSE(read_optional_result);
    EXPECT_EQ(
        read_optional_result.error(),
        "Invalid value for key 'value': expected integer, got string");

    const auto parsed_take_invalid = toml::parse("value = 'oops'");
    ASSERT_TRUE(parsed_take_invalid);

    auto take_optional_result =
        Deserialize<required<int>>::take_optional(parsed_take_invalid.table(), "value");
    ASSERT_FALSE(take_optional_result);
    EXPECT_EQ(
        take_optional_result.error(),
        "Invalid value for key 'value': expected integer, got string");

    // 键不存在，read_optional返回外层nullopt
    const auto parsed_missing = toml::parse("");
    ASSERT_TRUE(parsed_missing);

    auto take_optional_missing =
        Deserialize<required<int>>::take_optional(parsed_missing.table(), "value");
    ASSERT_TRUE(take_optional_missing);
    EXPECT_FALSE(take_optional_missing->has_value());
}

/**
 * @brief 测试21：read_optional/take_optional 读取结构体时，table存在冗余未读key也会报错
 * 无论是否optional读取，只要开启ensure_all_tables_read，存在未读key一律失败
 */
TEST(TomlReflection, ReflectiveOptionalEntryPointsPropagateUnreadAndParseErrors) {
    // camera顶层存在extra冗余key
    const auto parsed_unread = toml::parse(R"(
        [camera]
        model = "MV-CA013"
        fx = 800.0
        fy = 801.0
        extra = 1

        [camera.serial]
        baud = 115200
    )");
    ASSERT_TRUE(parsed_unread);

    auto read_result = Deserialize<CameraConfig>::read(parsed_unread.table(), "camera");
    ASSERT_FALSE(read_result);
    EXPECT_NE(read_result.error().find("camera: unread keys"), std::string::npos);

    // read_optional 同样检测冗余key
    const auto parsed_optional_unread = toml::parse(R"(
        [camera]
        model = "MV-CA013"
        fx = 800.0
        fy = 801.0
        extra = 1

        [camera.serial]
        baud = 115200
    )");
    ASSERT_TRUE(parsed_optional_unread);

    auto read_optional_unread =
        Deserialize<CameraConfig>::read_optional(parsed_optional_unread.table(), "camera");
    ASSERT_FALSE(read_optional_unread);
    EXPECT_NE(read_optional_unread.error().find("camera: unread keys"), std::string::npos);

    // key类型错误，read_optional报错
    const auto parsed_optional_invalid = toml::parse("camera = 5");
    ASSERT_TRUE(parsed_optional_invalid);

    auto read_optional_invalid =
        Deserialize<CameraConfig>::read_optional(parsed_optional_invalid.table(), "camera");
    ASSERT_FALSE(read_optional_invalid);
    EXPECT_EQ(read_optional_invalid.error(), "Invalid value for key 'camera': expected table");

    // take_optional 存在冗余key报错
    const auto parsed_take_unread = toml::parse(R"(
        [camera]
        model = "MV-CA013"
        fx = 800.0
        fy = 801.0
        extra = 1

        [camera.serial]
        baud = 115200
    )");
    ASSERT_TRUE(parsed_take_unread);

    auto take_optional_unread =
        Deserialize<CameraConfig>::take_optional(parsed_take_unread.table(), "camera");
    ASSERT_FALSE(take_optional_unread);
    EXPECT_NE(take_optional_unread.error().find("camera: unread keys"), std::string::npos);

    // take_optional key类型错误
    const auto parsed_take_invalid = toml::parse("camera = 5");
    ASSERT_TRUE(parsed_take_invalid);

    auto take_optional_invalid =
        Deserialize<CameraConfig>::take_optional(parsed_take_invalid.table(), "camera");
    ASSERT_FALSE(take_optional_invalid);
    EXPECT_EQ(take_optional_invalid.error(), "Invalid value for key 'camera': expected table");

    // take_optional 键不存在返回外层nullopt
    const auto parsed_missing = toml::parse("");
    ASSERT_TRUE(parsed_missing);

    auto take_optional_missing =
        Deserialize<CameraConfig>::take_optional(parsed_missing.table(), "camera");
    ASSERT_TRUE(take_optional_missing);
    EXPECT_FALSE(take_optional_missing->has_value());
}

} // namespace