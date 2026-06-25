#include <gtest/gtest.h>

#include <fmt/format.h>

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "toml/ext/core.hpp"
#include "toml/ext/containers.hpp"

namespace {

using namespace toml_helper;

struct NoDefault {
    NoDefault() = delete;
    explicit NoDefault(int value)
        : value(value) {}

    int value;
};

static_assert(!std::default_initializable<NoDefault>);
static_assert(std::default_initializable<required<NoDefault>>);

struct SerialConfig {
    required<int> baud{};
    std::optional<int> port{};
    int timeout_ms{200};
};

struct CameraIntrinsics {
    required<double> fx{};
    required<double> fy{};
    std::optional<double> cx{};
    double cy{540.0};
};

struct CameraConfig {
    std::string model{"HIK"};
    SerialConfig serial{};
    flatten<CameraIntrinsics> intrinsics{};
    int fps{60};
};

struct Waypoint {
    double x = 0.0;
    double y = 0.0;
};

struct PathConfig {
    std::vector<Waypoint> points;
};

enum class PowerMode {
    Off = 0,
    On  = 1,
};

TEST(TomlReflection, FromTableReadsRequiredOptionalAndPlainDefaultFields) {
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
    ASSERT_TRUE(parsed);

    auto result = from_table<CameraConfig>(parsed.table());
    ASSERT_TRUE(result);

    EXPECT_EQ(result->model, "MV-CA013");
    EXPECT_EQ(result->fps, 60);
    EXPECT_EQ(result->serial.baud.get(), 115200);
    ASSERT_TRUE(result->serial.port.has_value());
    EXPECT_EQ(*result->serial.port, 2);
    EXPECT_EQ(result->serial.timeout_ms, 200);
    EXPECT_DOUBLE_EQ(result->intrinsics->fx.get(), 900.0);
    EXPECT_DOUBLE_EQ(result->intrinsics->fy.get(), 901.0);
    ASSERT_TRUE(result->intrinsics->cx.has_value());
    EXPECT_DOUBLE_EQ(*result->intrinsics->cx, 320.0);
    EXPECT_DOUBLE_EQ(result->intrinsics->cy, 540.0);

    auto check = ensure_all_read(parsed.table(), "camera");
    ASSERT_TRUE(check);
}

TEST(TomlReflection, ReadIntoReplacesExistingObjectAndResetsByFieldPolicy) {
    const auto parsed = toml::parse(R"(
        fx = 700.0
        fy = 710.0

        [serial]
        baud = 115200
    )");
    ASSERT_TRUE(parsed);

    CameraConfig config;
    config.model             = "existing";
    config.serial.port       = 4;
    config.serial.timeout_ms = 200;
    config.intrinsics->cx    = 300.0;
    config.intrinsics->cy    = 540.0;
    config.fps               = 144;

    auto result = read_into(parsed.table(), config);
    ASSERT_TRUE(result);

    EXPECT_EQ(config.model, "HIK");
    EXPECT_EQ(config.serial.baud.get(), 115200);
    EXPECT_FALSE(config.serial.port.has_value());
    EXPECT_EQ(config.serial.timeout_ms, 200);
    EXPECT_DOUBLE_EQ(config.intrinsics->fx.get(), 700.0);
    EXPECT_DOUBLE_EQ(config.intrinsics->fy.get(), 710.0);
    EXPECT_FALSE(config.intrinsics->cx.has_value());
    EXPECT_DOUBLE_EQ(config.intrinsics->cy, 540.0);
    EXPECT_EQ(config.fps, 60);

    auto check = ensure_all_read(parsed.table(), "camera");
    ASSERT_TRUE(check);
}

TEST(TomlReflection, ReadIntoFailureDoesNotMutateDestination) {
    const auto parsed = toml::parse(R"(
        fx = 700.0

        [serial]
        baud = 115200
    )");
    ASSERT_TRUE(parsed);

    CameraConfig config;
    config.model             = "existing";
    config.serial.baud       = 9600;
    config.serial.port       = 4;
    config.serial.timeout_ms = 250;
    config.intrinsics->fx    = 1.0;
    config.intrinsics->fy    = 2.0;
    config.intrinsics->cx    = 300.0;
    config.fps               = 144;

    auto result = read_into(parsed.table(), config);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "intrinsics: Missing key 'fy'");

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

TEST(TomlReflection, MissingRequiredFieldInsideFlattenIsPrefixed) {
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
    EXPECT_NE(result.error().find("intrinsics: Missing key 'fx'"), std::string::npos);
}

TEST(TomlReflection, TakeReflectiveObjectRejectsUnreadKeysInNestedTable) {
    const auto parsed = toml::parse(R"(
        serial = { baud = 115200, port = 7, extra = 1 }
    )");
    ASSERT_TRUE(parsed);

    auto result = take<SerialConfig>(parsed.table(), "serial");
    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("serial: unread keys"), std::string::npos);
    EXPECT_NE(result.error().find("'extra'(integer)"), std::string::npos);
}

TEST(TomlReflection, FromTableRejectsUnreadTopLevelKeys) {
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

TEST(TomlReflection, PlainFieldMayBeMissingAndKeepsInitializer) {
    struct PlainDefaultConfig {
        int port = 7;
    };

    const auto parsed = toml::parse("");
    ASSERT_TRUE(parsed);

    auto result = from_table<PlainDefaultConfig>(parsed.table());
    ASSERT_TRUE(result);
    EXPECT_EQ(result->port, 7);
}

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

TEST(TomlReflection, RequiredWrapperThrowsOnEmptyAccess) {
    required<int> port{};
    const required<int>& const_port = port;

    EXPECT_FALSE(port.has_value());
    EXPECT_THROW((void)port.get(), std::bad_optional_access);
    EXPECT_THROW((void)const_port.get(), std::bad_optional_access);
    EXPECT_THROW((void)port.operator->(), std::bad_optional_access);
    EXPECT_THROW((void)const_port.operator->(), std::bad_optional_access);
    EXPECT_THROW((void)*port, std::bad_optional_access);
}

TEST(TomlReflection, OptionalFieldMayBeMissingEvenWithEngagedInitializer) {
    struct OptionalConfig {
        std::optional<int> port = 7;
    };

    const auto parsed = toml::parse("");
    ASSERT_TRUE(parsed);

    auto result = from_table<OptionalConfig>(parsed.table());
    ASSERT_TRUE(result);
    EXPECT_FALSE(result->port.has_value());
}

TEST(TomlReflection, MergeConfigsKeepsFileLevelOverrideSemantics) {
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

    const auto override_cfg = toml::parse(R"(
        [serial]
        port = 2
    )");
    ASSERT_TRUE(override_cfg);

    auto merged = merge_configs(base.table(), override_cfg.table());
    ASSERT_TRUE(merged);

    auto result = from_table<CameraConfig>(*merged);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->model, "MV-CA013");
    EXPECT_EQ(result->fps, 120);
    ASSERT_TRUE(result->serial.port.has_value());
    EXPECT_EQ(*result->serial.port, 2);
    EXPECT_EQ(result->serial.timeout_ms, 200);
    EXPECT_DOUBLE_EQ(result->intrinsics->cy, 540.0);
}

TEST(TomlReflection, ContainerParsersSupportReflectiveElementTypes) {
    const auto parsed = toml::parse(R"(
        points = [
            { x = 1.0, y = 2.0 },
            { x = 3.0, y = 4.0 }
        ]
    )");
    ASSERT_TRUE(parsed);

    auto result = from_table<PathConfig>(parsed.table());
    ASSERT_TRUE(result);
    ASSERT_EQ(result->points.size(), 2U);
    EXPECT_DOUBLE_EQ(result->points[0].x, 1.0);
    EXPECT_DOUBLE_EQ(result->points[1].y, 4.0);

    auto check = ensure_all_read(parsed.table(), "path");
    ASSERT_TRUE(check);
}

TEST(TomlReflection, WrappersSupportOperatorsAssignmentsAndFormatting) {
    required<int> engaged{7};
    const required<int>& const_engaged = engaged;

    EXPECT_TRUE(engaged.has_value());
    EXPECT_EQ(engaged.get(), 7);
    EXPECT_EQ(const_engaged.get(), 7);
    EXPECT_EQ(*engaged, 7);
    EXPECT_EQ(*const_engaged, 7);

    int& as_ref             = engaged;
    const int& as_const_ref = const_engaged;
    EXPECT_EQ(as_ref, 7);
    EXPECT_EQ(as_const_ref, 7);

    *engaged.operator->() = 9;
    EXPECT_EQ(engaged.get(), 9);
    EXPECT_EQ(fmt::format("{}", engaged), "9");

    required<std::string> moved{std::string("ready")};
    std::string moved_value = std::move(moved).get();
    EXPECT_EQ(moved_value, "ready");

    flatten<CameraIntrinsics> intrinsics{};
    intrinsics->fx                  = 1.5;
    (*intrinsics).fy                = 2.5;
    CameraIntrinsics& as_intrinsics = intrinsics;
    EXPECT_DOUBLE_EQ(as_intrinsics.fx.get(), 1.5);
    EXPECT_DOUBLE_EQ(as_intrinsics.fy.get(), 2.5);

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

TEST(TomlReflection, ReflectiveDeserializeSupportsReadOptionalAndTakeOptional) {
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

    const auto parsed_missing = toml::parse("");
    ASSERT_TRUE(parsed_missing);

    auto missing_optional =
        Deserialize<CameraConfig>::read_optional(parsed_missing.table(), "missing");
    ASSERT_TRUE(missing_optional);
    EXPECT_FALSE(missing_optional->has_value());

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
    EXPECT_EQ(parsed_take.table().get("camera"), nullptr);
}

TEST(TomlReflection, ReflectiveDeserializeRejectsNonTableValues) {
    const auto parsed = toml::parse("camera = 5");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<CameraConfig>::read(parsed.table(), "camera");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid value for key 'camera': expected table");
}

TEST(TomlReflection, RequiredDeserializerSupportsOptionalForms) {
    const auto parsed = toml::parse("value = 42");
    ASSERT_TRUE(parsed);

    auto direct = Deserialize<required<int>>::read(parsed.table(), "value");
    ASSERT_TRUE(direct);
    EXPECT_EQ(direct->get(), 42);

    auto optional_read = Deserialize<required<int>>::read_optional(parsed.table(), "value");
    ASSERT_TRUE(optional_read);
    ASSERT_TRUE(optional_read->has_value());
    EXPECT_EQ((*optional_read)->get(), 42);

    auto parsed_take = toml::parse("value = 42");
    ASSERT_TRUE(parsed_take);

    auto taken = Deserialize<required<int>>::take_optional(parsed_take.table(), "value");
    ASSERT_TRUE(taken);
    ASSERT_TRUE(taken->has_value());
    EXPECT_EQ((*taken)->get(), 42);
    EXPECT_EQ(parsed_take.table().get("value"), nullptr);

    const auto missing = toml::parse("");
    ASSERT_TRUE(missing);

    auto missing_optional = Deserialize<required<int>>::read_optional(missing.table(), "value");
    ASSERT_TRUE(missing_optional);
    EXPECT_FALSE(missing_optional->has_value());
}

TEST(TomlReflection, EnumDeserializerAcceptsIntegralValuesAndRejectsInvalidInputs) {
    const auto parsed_int = toml::parse("mode = 1");
    ASSERT_TRUE(parsed_int);

    auto int_result = read<PowerMode>(parsed_int.table(), "mode");
    ASSERT_TRUE(int_result);
    EXPECT_EQ(*int_result, PowerMode::On);

    const auto parsed_bad_string = toml::parse("mode = 'invalid'");
    ASSERT_TRUE(parsed_bad_string);

    auto bad_string = read<PowerMode>(parsed_bad_string.table(), "mode");
    ASSERT_FALSE(bad_string);
    EXPECT_NE(
        bad_string.error().find("Invalid enum value for key 'mode': 'invalid'"), std::string::npos);

    const auto parsed_bad_type = toml::parse("mode = [1]");
    ASSERT_TRUE(parsed_bad_type);

    auto bad_type = read<PowerMode>(parsed_bad_type.table(), "mode");
    ASSERT_FALSE(bad_type);
    EXPECT_NE(
        bad_type.error().find("expected enum as string/integer, got array"), std::string::npos);
}

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

    auto plain_result = from_table<PlainValueConfig>(parsed_plain.table());
    ASSERT_FALSE(plain_result);
    EXPECT_EQ(plain_result.error(), "Invalid value for key 'value': expected integer, got string");
}

TEST(TomlReflection, ReflectiveDeserializeReportsNestedAndMissingTableErrors) {
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

    const auto parsed_missing = toml::parse("");
    ASSERT_TRUE(parsed_missing);

    auto missing_result = read<CameraConfig>(parsed_missing.table(), "camera");
    ASSERT_FALSE(missing_result);
    EXPECT_EQ(missing_result.error(), "Missing table 'camera'");

    const auto parsed_take = toml::parse("camera = 5");
    ASSERT_TRUE(parsed_take);

    auto take_result = Deserialize<CameraConfig>::take(parsed_take.table(), "camera");
    ASSERT_FALSE(take_result);
    EXPECT_EQ(take_result.error(), "Invalid value for key 'camera': expected table");
}

TEST(TomlReflection, RequiredDeserializerPropagatesErrorsAndMissingStates) {
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

    const auto parsed_missing = toml::parse("");
    ASSERT_TRUE(parsed_missing);

    auto take_optional_missing =
        Deserialize<required<int>>::take_optional(parsed_missing.table(), "value");
    ASSERT_TRUE(take_optional_missing);
    EXPECT_FALSE(take_optional_missing->has_value());
}

TEST(TomlReflection, ReflectiveOptionalEntryPointsPropagateUnreadAndParseErrors) {
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

    const auto parsed_optional_invalid = toml::parse("camera = 5");
    ASSERT_TRUE(parsed_optional_invalid);

    auto read_optional_invalid =
        Deserialize<CameraConfig>::read_optional(parsed_optional_invalid.table(), "camera");
    ASSERT_FALSE(read_optional_invalid);
    EXPECT_EQ(read_optional_invalid.error(), "Invalid value for key 'camera': expected table");

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

    const auto parsed_take_invalid = toml::parse("camera = 5");
    ASSERT_TRUE(parsed_take_invalid);

    auto take_optional_invalid =
        Deserialize<CameraConfig>::take_optional(parsed_take_invalid.table(), "camera");
    ASSERT_FALSE(take_optional_invalid);
    EXPECT_EQ(take_optional_invalid.error(), "Invalid value for key 'camera': expected table");

    const auto parsed_missing = toml::parse("");
    ASSERT_TRUE(parsed_missing);

    auto take_optional_missing =
        Deserialize<CameraConfig>::take_optional(parsed_missing.table(), "camera");
    ASSERT_TRUE(take_optional_missing);
    EXPECT_FALSE(take_optional_missing->has_value());
}

} // namespace
