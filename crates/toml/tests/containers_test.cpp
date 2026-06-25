#include <gtest/gtest.h>

// 标准库容器头文件
#include <array>
#include <optional>
#include <vector>

// toml++ 核心解析库
#include "toml/core.hpp"
// 自定义扩展：vector/array/optional/结构体反射反序列化逻辑
#include "toml/ext/containers.hpp"

namespace { // 匿名命名空间，隔离测试代码，不污染全局

using namespace toml_helper; // 自定义toml工具命名空间

// 简单聚合结构体：平面坐标点，用于测试数组嵌套结构体反序列化
struct Point {
    int x = 0;
    int y = 0;
};

// 业务配置结构体示例：内置read_from自定义读取接口
struct MacroConfig {
    // 必填基础int字段，默认值7
    int gain{7};
    // 可选字段：std::optional，缺失key时清空，存在则覆盖
    std::optional<int> exposure = 42;

    /**
     * @brief 自定义toml读取接口，框架会自动识别该接口实现反射反序列化
     * @param table 待读取的toml根表
     * @return std::expected<void, string> 成功无错误；失败返回错误信息
     * @noexcept 不抛出异常，错误使用expected传递
     */
    [[nodiscard]] std::expected<void, std::string> read_from(const toml::table& table) noexcept {
        // 宏READ：读取必填字段，key缺失/类型错误直接返回失败
        READ(table, gain);
        // 宏READ_OPT：读取optional可选字段
        // key存在：覆盖optional值；key缺失：optional重置为std::nullopt
        READ_OPT(table, exposure);
        return {}; // 成功，无错误
    }
};

// ===================== 测试用例1：std::optional 读取，key缺失返回错误 =====================
TEST(TomlContainers, OptionalReadRequiresKey) {
    // 解析空toml文本，无任何键值
    const auto parsed = toml::parse("");
    // 断言解析文件成功（空文件也算合法toml）
    ASSERT_TRUE(parsed);

    // 调用通用反序列化类：读取key=value，目标类型std::optional<int>
    auto result = Deserialize<std::optional<int>>::read(parsed.table(), "value");
    // read方法对optional是【强制要求key存在】，缺失则失败
    ASSERT_FALSE(result);
    // 校验错误文案：提示缺失value键
    EXPECT_EQ(result.error(), "Missing key 'value'");
}

// ===================== 测试用例2：std::optional 存在key，正常解析数值 =====================
TEST(TomlContainers, OptionalReadParsesValue) {
    // 带value = 42的toml配置
    const auto parsed = toml::parse("value = 42");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::optional<int>>::read(parsed.table(), "value");
    // 读取成功
    ASSERT_TRUE(result);
    // optional内部存在有效值
    ASSERT_TRUE(result->has_value());
    // 取出optional包裹的int，校验等于42
    EXPECT_EQ(**result, 42);
}

// ===================== 测试用例3：反射读取结构体，optional字段无key则清空 =====================
TEST(TomlContainers, ReflectiveReadIntoClearsOptionalWhenKeyMissing) {
    // 局部配置结构体，包含普通int + optional<int>
    struct Config {
        int gain{7};
        std::optional<int> exposure = 42;
    };

    // 空toml，gain、exposure两个key全部缺失
    const auto parsed = toml::parse("");
    ASSERT_TRUE(parsed);

    Config config{};
    // read_into：基于PFR反射，自动遍历结构体所有字段填充toml数据
    auto result = read_into(parsed.table(), config);
    ASSERT_TRUE(result);
    // gain无对应key，保留构造默认值7
    EXPECT_EQ(config.gain, 7);
    // exposure无对应key，READ_OPT逻辑清空optional，无值
    EXPECT_FALSE(config.exposure.has_value());
}

// ===================== 测试用例4：结构体optional字段存在key，覆盖原有值 =====================
TEST(TomlContainers, ReflectiveReadIntoOptionalWhenPresentOverwritesValue) {
    struct Config {
        int gain{};
        std::optional<int> exposure = 42;
    };

    // toml内存在gain和exposure两个键
    const auto parsed = toml::parse(R"(
        gain = 7
        exposure = 5
    )");
    ASSERT_TRUE(parsed);

    Config config{};
    auto result = read_into(parsed.table(), config);
    ASSERT_TRUE(result);
    // 覆盖int gain
    EXPECT_EQ(config.gain, 7);
    // optional被赋值，存在有效值
    EXPECT_TRUE(config.exposure.has_value());
    // 原值42被toml的5覆盖
    EXPECT_EQ(*config.exposure, 5);
}

// ===================== 测试用例5：自定义read_from结构体，缺失key清空optional =====================
TEST(TomlContainers, ReadFromReadOptClearsOptionalWhenKeyMissing) {
    const auto parsed = toml::parse("");
    ASSERT_TRUE(parsed);

    MacroConfig config{};
    // 调用read_into，自动调用结构体内部自定义read_from接口
    auto result = read_into(parsed.table(), config);
    ASSERT_TRUE(result);
    // gain无key，保留默认7
    EXPECT_EQ(config.gain, 7);
    // exposure无key，READ_OPT置空optional
    EXPECT_FALSE(config.exposure.has_value());
}

// ===================== 测试用例6：读取失败时，不修改原始对象任何成员 =====================
TEST(TomlContainers, ReadIntoFailureDoesNotMutateReadFromObject) {
    struct RequiredMacroConfig {
        // required<T> 自定义包装类型：强制必填，缺失key直接读取失败
        required<int> mode{};
        int gain{7};

        [[nodiscard]] std::expected<void, std::string>
            read_from(const toml::table& table) noexcept {
            READ(table, mode); // mode是必填项
            READ(table, gain);
            return {};
        }
    };

    // toml只提供gain，缺失必填mode键
    const auto parsed = toml::parse("gain = 5");
    ASSERT_TRUE(parsed);

    RequiredMacroConfig config{};
    // 预先手动赋值，修改默认值
    config.mode = 3;
    config.gain = 42;

    auto result = read_into(parsed.table(), config);
    // 读取失败（缺少mode）
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Missing key 'mode'");
    // 核心校验：读取失败，结构体原有数据完全不变
    EXPECT_EQ(config.mode.get(), 3);
    EXPECT_EQ(config.gain, 42);
}

// ===================== 测试用例7：自定义read_from，全部key存在正常填充 =====================
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

    // toml同时存在mode、gain两个必填键
    const auto parsed = toml::parse(R"(
        mode = 3
        gain = 5
    )");
    ASSERT_TRUE(parsed);

    RequiredMacroConfig config{};
    auto result = read_into(parsed.table(), config);
    ASSERT_TRUE(result);
    // required<int> mode 读取到3
    EXPECT_EQ(config.mode.get(), 3);
    // gain覆盖为5
    EXPECT_EQ(config.gain, 5);
}

// ===================== 测试用例8：普通字段类型不匹配，返回解析错误 =====================
TEST(TomlContainers, ReadFromReadPropagatesPlainFieldParseErrors) {
    // gain配置字符串，要求int，类型不匹配
    const auto parsed = toml::parse("gain = 'oops'");
    ASSERT_TRUE(parsed);

    MacroConfig config{};
    auto result = read_into(parsed.table(), config);
    ASSERT_FALSE(result);
    // 错误信息提示：gain期望整数，实际是字符串
    EXPECT_EQ(result.error(), "Invalid value for key 'gain': expected integer, got string");
}

// ===================== 测试用例9：std::vector<int> 一维数组正常解析 =====================
TEST(TomlContainers, VectorReadParsesScalars) {
    // toml一维数字数组
    const auto parsed = toml::parse("values = [1, 2, 3, 4, 5]");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::vector<int>>::read(parsed.table(), "values");
    ASSERT_TRUE(result);
    // 数组长度5
    EXPECT_EQ(result->size(), 5U);
    EXPECT_EQ((*result)[0], 1);
    EXPECT_EQ((*result)[4], 5);
}

// ===================== 测试用例10：vector嵌套std::array二维数组解析 =====================
TEST(TomlContainers, VectorReadParsesNestedArrays) {
    // 二维浮点数组，每个元素固定3个浮点数
    const auto parsed = toml::parse(R"(
        waypoints = [
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [2.0, 0.0, 0.0]
        ]
    )");
    ASSERT_TRUE(parsed);

    // 目标类型 vector<array<double,3>>
    auto result =
        Deserialize<std::vector<std::array<double, 3>>>::read(parsed.table(), "waypoints");
    ASSERT_TRUE(result);
    EXPECT_EQ(result->size(), 3U);
    EXPECT_DOUBLE_EQ((*result)[1][0], 1.0);
    EXPECT_DOUBLE_EQ((*result)[2][1], 0.0);
}

// ===================== 测试用例11：take读取vector，结构体存在多余未读key报错 =====================
TEST(TomlContainers, VectorTakeRejectsUnreadKeysInNestedTables) {
    // Point结构体仅x/y，toml内多出extra字段
    const auto parsed = toml::parse(R"(
        points = [
            { x = 1, y = 2, extra = 3 }
        ]
    )");
    ASSERT_TRUE(parsed);

    // take：读取后会从toml table中删除该key；严格校验无多余字段
    auto result = Deserialize<std::vector<Point>>::take(parsed.table(), "points");
    ASSERT_FALSE(result);
    // 错误包含数组下标0、多余key extra
    EXPECT_NE(result.error().find("points[0]: unread keys"), std::string::npos);
    EXPECT_NE(result.error().find("'extra'(integer)"), std::string::npos);
}

// ===================== 测试用例12：read读取vector，嵌套结构体多余key同样报错 =====================
TEST(TomlContainers, VectorReadAlsoRejectsUnreadKeysInNestedTables) {
    const auto parsed = toml::parse(R"(
        points = [
            { x = 1, y = 2, extra = 3 }
        ]
    )");
    ASSERT_TRUE(parsed);

    // read：仅读取，不删除原key；同样严格校验结构体无多余字段
    auto result = Deserialize<std::vector<Point>>::read(parsed.table(), "points");
    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("points[0]: unread keys"), std::string::npos);
    EXPECT_NE(result.error().find("'extra'(integer)"), std::string::npos);
}

// ===================== 测试用例13：std::array固定长度校验，元素数量不符报错 =====================
TEST(TomlContainers, ArrayReadValidatesSize) {
    // array<int,3> 需要3个数字，toml仅提供2个
    const auto parsed = toml::parse("values = [1, 2]");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::array<int, 3>>::read(parsed.table(), "values");
    ASSERT_FALSE(result);
    // 错误提示期望3个，实际2个
    EXPECT_NE(result.error().find("expected 3 elements, got 2"), std::string::npos);
}

// ===================== 测试用例14：array存储自定义结构体，正常解析 =====================
TEST(TomlContainers, ArrayReadParsesNestedTables) {
    // 数组内是Point结构体表格
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

// ===================== 测试用例15：read_optional 外层optional包裹int =====================
TEST(TomlContainers, ReadOptionalForOptionalReturnsOuterPresence) {
    const auto parsed = toml::parse("value = 9");
    ASSERT_TRUE(parsed);

    // read_optional：外层类型是optional<T>，key存在则返回std::optional<T>有值
    auto result = Deserialize<std::optional<int>>::read_optional(parsed.table(), "value");
    ASSERT_TRUE(result);
    // result是std::expected<std::optional<int>, err>
    ASSERT_TRUE(result->has_value());
    // 内层optional包含数字9
    ASSERT_TRUE((*result)->has_value());
    EXPECT_EQ(**result, 9);
}

// ===================== 测试用例16：read_optional读取vector，key缺失返回外层nullopt =====================
TEST(TomlContainers, ReadOptionalVectorMissingKeyReturnsOuterNullopt) {
    // 空toml，无values键
    const auto parsed = toml::parse("");
    ASSERT_TRUE(parsed);

    // 返回expected<optional<vector<int>>>
    auto result = Deserialize<std::vector<int>>::read_optional(parsed.table(), "values");
    ASSERT_TRUE(result);
    // key缺失，外层optional无值
    EXPECT_FALSE(result->has_value());
}

// ===================== 测试用例17：read_optional vector存在key正常解析 =====================
TEST(TomlContainers, ReadOptionalVectorParsesPresentValue) {
    const auto parsed = toml::parse("values = [1, 2, 3]");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::vector<int>>::read_optional(parsed.table(), "values");
    ASSERT_TRUE(result);
    ASSERT_TRUE(result->has_value());
    EXPECT_EQ((*result)->size(), 3U);
    EXPECT_EQ((*result)->at(2), 3);
}

// ===================== 测试用例18：optional<int> key存在但类型错误，返回失败 =====================
TEST(TomlContainers, OptionalReadRejectsInvalidInnerType) {
    // value是字符串，目标int不匹配
    const auto parsed = toml::parse("value = 'oops'");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::optional<int>>::read(parsed.table(), "value");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid value for key 'value': expected integer, got string");
}

// ===================== 测试用例19：vector期望数组，实际标量类型报错 =====================
TEST(TomlContainers, VectorReadRejectsNonArrayValue) {
    // values = 1 是单个数字，不是数组
    const auto parsed = toml::parse("values = 1");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::vector<int>>::read(parsed.table(), "values");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid value for key 'values': expected array, got integer");
}

// ===================== 测试用例20：take_optional读取vector，读取成功后删除toml内原key =====================
TEST(TomlContainers, VectorTakeOptionalErasesValueOnSuccess) {
    auto parsed = toml::parse("values = [1, 2, 3]");
    ASSERT_TRUE(parsed);

    // take_optional：读取成功后，table内删除values键
    auto result = Deserialize<std::vector<int>>::take_optional(parsed.table(), "values");
    ASSERT_TRUE(result);
    ASSERT_TRUE(result->has_value());
    EXPECT_EQ((*result)->size(), 3U);
    // 原table中values已被移除，get返回nullptr
    EXPECT_EQ(parsed.table().get("values"), nullptr);
}

// ===================== 测试用例21：array期望数组，传入单个数字报错 =====================
TEST(TomlContainers, ArrayReadRejectsNonArrayValue) {
    const auto parsed = toml::parse("triangle = 1");
    ASSERT_TRUE(parsed);

    auto result = Deserialize<std::array<Point, 3>>::read(parsed.table(), "triangle");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid value for key 'triangle': expected array, got integer");
}

// ===================== 测试用例22：array嵌套结构体存在多余未读key报错 =====================
TEST(TomlContainers, ArrayReadRejectsNestedElementErrors) {
    // 第二个Point多出extra字段
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

// ===================== 测试用例23：take_optional读取array，成功后删除toml键 =====================
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
    // 读取后table中triangle被清除
    EXPECT_EQ(parsed.table().get("triangle"), nullptr);
}

} // namespace