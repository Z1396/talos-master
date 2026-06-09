#include <gtest/gtest.h>

#include <array>
#include <optional>
#include <vector>

#include "toml/core.hpp"
#include "toml/ext/containers.hpp"

namespace {

using namespace toml_helper;

struct Point {
    int x = 0;
    int y = 0;
};

struct MacroConfig {
    int gain{7};
    std::optional<int> exposure = 42;

    [[nodiscard]] std::expected<void, std::string> read_from(const toml::table& table) noexcept {
        READ(table, gain);
        READ_OPT(table, exposure);
        return {};
    }
};

TEST(TomlContainers, OptionalReadRequiresKey) {
    const auto parsed = toml::parse("");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::optional<int>>::read(parsed.table(), "value");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Missing key 'value'");
}

TEST(TomlContainers, OptionalReadParsesValue) {
    const auto parsed = toml::parse("value = 42");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::optional<int>>::read(parsed.table(), "value");
    ASSERT_TRUE(result);
    ASSERT_TRUE(result->has_value());
    EXPECT_EQ(**result, 42);
}

TEST(TomlContainers, ReflectiveReadIntoClearsOptionalWhenKeyMissing) {
    struct Config {
        int gain{7};
        std::optional<int> exposure = 42;
    };

    const auto parsed = toml::parse("");
    ASSERT_TRUE(parsed);

    Config config{};
    auto result = read_into(parsed.table(), config);
    ASSERT_TRUE(result);
    EXPECT_EQ(config.gain, 7);
    EXPECT_FALSE(config.exposure.has_value());
}

TEST(TomlContainers, ReflectiveReadIntoOptionalWhenPresentOverwritesValue) {
    struct Config {
        int gain{};
        std::optional<int> exposure = 42;
    };

    const auto parsed = toml::parse(R"(
        gain = 7
        exposure = 5
    )");
    ASSERT_TRUE(parsed);

    Config config{};
    auto result = read_into(parsed.table(), config);
    ASSERT_TRUE(result);
    EXPECT_EQ(config.gain, 7);
    ASSERT_TRUE(config.exposure.has_value());
    EXPECT_EQ(*config.exposure, 5);
}

TEST(TomlContainers, ReadFromReadOptClearsOptionalWhenKeyMissing) {
    const auto parsed = toml::parse("");
    ASSERT_TRUE(parsed);

    MacroConfig config{};
    auto result = read_into(parsed.table(), config);
    ASSERT_TRUE(result);
    EXPECT_EQ(config.gain, 7);
    EXPECT_FALSE(config.exposure.has_value());
}

TEST(TomlContainers, ReadIntoFailureDoesNotMutateReadFromObject) {
    struct RequiredMacroConfig {
        required<int> mode{};
        int gain{7};

        [[nodiscard]] std::expected<void, std::string>
            read_from(const toml::table& table) noexcept {
            READ(table, mode);
            READ(table, gain);
            return {};
        }
    };

    const auto parsed = toml::parse("gain = 5");
    ASSERT_TRUE(parsed);

    RequiredMacroConfig config{};
    config.mode = 3;
    config.gain = 42;

    auto result = read_into(parsed.table(), config);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Missing key 'mode'");
    EXPECT_EQ(config.mode.get(), 3);
    EXPECT_EQ(config.gain, 42);
}

TEST(TomlContainers, ReadFromReadPopulatesRequiredAndPlainFieldsOnSuccess) {
    struct RequiredMacroConfig {
        required<int> mode{};
        int gain{7};

        [[nodiscard]] std::expected<void, std::string>
            read_from(const toml::table& table) noexcept {
            READ(table, mode);
            READ(table, gain);
            return {};
        }
    };

    const auto parsed = toml::parse(R"(
        mode = 3
        gain = 5
    )");
    ASSERT_TRUE(parsed);

    RequiredMacroConfig config{};
    auto result = read_into(parsed.table(), config);
    ASSERT_TRUE(result);
    EXPECT_EQ(config.mode.get(), 3);
    EXPECT_EQ(config.gain, 5);
}

TEST(TomlContainers, ReadFromReadPropagatesPlainFieldParseErrors) {
    const auto parsed = toml::parse("gain = 'oops'");
    ASSERT_TRUE(parsed);

    MacroConfig config{};
    auto result = read_into(parsed.table(), config);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid value for key 'gain': expected integer, got string");
}

TEST(TomlContainers, VectorReadParsesScalars) {
    const auto parsed = toml::parse("values = [1, 2, 3, 4, 5]");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::vector<int>>::read(parsed.table(), "values");
    ASSERT_TRUE(result);
    EXPECT_EQ(result->size(), 5U);
    EXPECT_EQ((*result)[0], 1);
    EXPECT_EQ((*result)[4], 5);
}

TEST(TomlContainers, VectorReadParsesNestedArrays) {
    const auto parsed = toml::parse(R"(
        waypoints = [
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [2.0, 0.0, 0.0]
        ]
    )");
    ASSERT_TRUE(parsed);

    auto result =
        Deserialize<std::vector<std::array<double, 3>>>::read(parsed.table(), "waypoints");
    ASSERT_TRUE(result);
    EXPECT_EQ(result->size(), 3U);
    EXPECT_DOUBLE_EQ((*result)[1][0], 1.0);
    EXPECT_DOUBLE_EQ((*result)[2][1], 0.0);
}

TEST(TomlContainers, VectorTakeRejectsUnreadKeysInNestedTables) {
    const auto parsed = toml::parse(R"(
        points = [
            { x = 1, y = 2, extra = 3 }
        ]
    )");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::vector<Point>>::take(parsed.table(), "points");
    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("points[0]: unread keys"), std::string::npos);
    EXPECT_NE(result.error().find("'extra'(integer)"), std::string::npos);
}

TEST(TomlContainers, VectorReadAlsoRejectsUnreadKeysInNestedTables) {
    const auto parsed = toml::parse(R"(
        points = [
            { x = 1, y = 2, extra = 3 }
        ]
    )");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::vector<Point>>::read(parsed.table(), "points");
    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("points[0]: unread keys"), std::string::npos);
    EXPECT_NE(result.error().find("'extra'(integer)"), std::string::npos);
}

TEST(TomlContainers, ArrayReadValidatesSize) {
    const auto parsed = toml::parse("values = [1, 2]");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::array<int, 3>>::read(parsed.table(), "values");
    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("expected 3 elements, got 2"), std::string::npos);
}

TEST(TomlContainers, ArrayReadParsesNestedTables) {
    const auto parsed = toml::parse(R"(
        triangle = [
            { x = 0, y = 0 },
            { x = 1, y = 0 },
            { x = 0, y = 1 }
        ]
    )");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::array<Point, 3>>::read(parsed.table(), "triangle");
    ASSERT_TRUE(result);
    EXPECT_EQ((*result)[0].x, 0);
    EXPECT_EQ((*result)[1].x, 1);
    EXPECT_EQ((*result)[2].y, 1);
}

TEST(TomlContainers, ReadOptionalForOptionalReturnsOuterPresence) {
    const auto parsed = toml::parse("value = 9");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::optional<int>>::read_optional(parsed.table(), "value");
    ASSERT_TRUE(result);
    ASSERT_TRUE(result->has_value());
    ASSERT_TRUE((*result)->has_value());
    EXPECT_EQ(**result, 9);
}

TEST(TomlContainers, ReadOptionalVectorMissingKeyReturnsOuterNullopt) {
    const auto parsed = toml::parse("");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::vector<int>>::read_optional(parsed.table(), "values");
    ASSERT_TRUE(result);
    EXPECT_FALSE(result->has_value());
}

TEST(TomlContainers, ReadOptionalVectorParsesPresentValue) {
    const auto parsed = toml::parse("values = [1, 2, 3]");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::vector<int>>::read_optional(parsed.table(), "values");
    ASSERT_TRUE(result);
    ASSERT_TRUE(result->has_value());
    EXPECT_EQ((*result)->size(), 3U);
    EXPECT_EQ((*result)->at(2), 3);
}

TEST(TomlContainers, OptionalReadRejectsInvalidInnerType) {
    const auto parsed = toml::parse("value = 'oops'");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::optional<int>>::read(parsed.table(), "value");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid value for key 'value': expected integer, got string");
}

TEST(TomlContainers, VectorReadRejectsNonArrayValue) {
    const auto parsed = toml::parse("values = 1");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::vector<int>>::read(parsed.table(), "values");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid value for key 'values': expected array, got integer");
}

TEST(TomlContainers, VectorTakeOptionalErasesValueOnSuccess) {
    auto parsed = toml::parse("values = [1, 2, 3]");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::vector<int>>::take_optional(parsed.table(), "values");
    ASSERT_TRUE(result);
    ASSERT_TRUE(result->has_value());
    EXPECT_EQ((*result)->size(), 3U);
    EXPECT_EQ(parsed.table().get("values"), nullptr);
}

TEST(TomlContainers, ArrayReadRejectsNonArrayValue) {
    const auto parsed = toml::parse("triangle = 1");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::array<Point, 3>>::read(parsed.table(), "triangle");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid value for key 'triangle': expected array, got integer");
}

TEST(TomlContainers, ArrayReadRejectsNestedElementErrors) {
    const auto parsed = toml::parse(R"(
        triangle = [
            { x = 0, y = 0 },
            { x = 1, y = 0, extra = 5 },
            { x = 0, y = 1 }
        ]
    )");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::array<Point, 3>>::read(parsed.table(), "triangle");
    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("triangle[1]: unread keys"), std::string::npos);
    EXPECT_NE(result.error().find("'extra'(integer)"), std::string::npos);
}

TEST(TomlContainers, ArrayTakeOptionalErasesValueOnSuccess) {
    auto parsed = toml::parse(R"(
        triangle = [
            { x = 0, y = 0 },
            { x = 1, y = 0 },
            { x = 0, y = 1 }
        ]
    )");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::array<Point, 3>>::take_optional(parsed.table(), "triangle");
    ASSERT_TRUE(result);
    ASSERT_TRUE(result->has_value());
    EXPECT_EQ((*result)->at(1).x, 1);
    EXPECT_EQ(parsed.table().get("triangle"), nullptr);
}

} // namespace
