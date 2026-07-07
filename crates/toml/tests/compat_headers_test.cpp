#pragma once
// GoogleTest 单元测试框架头文件
#include <gtest/gtest.h>

// 标准可选值 std::optional，解析返回值使用
#include <optional>
// 动态数组 std::vector，测试容器解析
#include <vector>

// 新版统一主头文件，包含核心解析逻辑
#include "toml_helper.hpp"
// 旧版独立容器扩展头文件（兼容层）
#include "toml_helper_containers.hpp"

/**
 * @brief 兼容性测试：旧版分拆式头文件仍能正常导出核心+容器解析API
 * 业务背景：项目早期将核心、容器解析拆分为两个头文件，后续合并为单一toml_helper.hpp
 * 为保证历史业务代码不用修改include，保留旧头文件做兼容转发，本用例验证兼容层生效
 * 测试逻辑：
 * 1. 解析一段极简TOML文本；
 * 2. 用toml_helper::read读取基础字符串字段；
 * 3. 用toml_helper::read读取std::vector<int>数组容器；
 * 4. 断言解析成功、数值匹配，证明两个头文件的API都正常暴露可用
 */
TEST(TomlCompatHeaders, LegacyFlatHeadersStillExposeCoreAndContainers) {
    // 原始TOML文本字面量：包含字符串基础字段、int数组容器字段
    const auto parsed = toml::parse(R"(
        label = "robot"
        values = [1, 2, 3]
    )");
    // 断言TOML文本解析无语法错误
    ASSERT_TRUE(parsed);

    // 读取基础字符串字段 label，调用核心解析API
    auto label = toml_helper::read<std::string>(parsed.table(), "label");
    // 读取结果必须成功（expected无错误）
    ASSERT_TRUE(label);
    // 校验字符串内容匹配"robot"
    EXPECT_EQ(*label, "robot");

    // 读取std::vector<int>数组容器字段 values，容器解析API可用
    auto values = toml_helper::read<std::vector<int>>(parsed.table(), "values");
    // 数组解析成功
    ASSERT_TRUE(values);
    // 数组长度等于3
    EXPECT_EQ(values->size(), 3U);
    // 数组第二个元素等于2，校验数组解析下标正确性
    EXPECT_EQ((*values)[1], 2);
}