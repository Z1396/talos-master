#pragma once
// GoogleTest 单元测试框架
#include <gtest/gtest.h>

// Eigen 线性代数库核心头文件，固定尺寸矩阵/向量定义
#include <Eigen/Core>

// TOML Eigen扩展官方头文件（标准规范头）
#include "toml/ext/eigen.hpp"
// 历史兼容旧版Eigen解析头文件（遗留别名头）
#include "toml_helper_eigen.hpp"

/**
 * @brief 兼容性回归测试：两套Eigen头文件均可正常解析Eigen固定尺寸矩阵/向量
 * 业务背景：
 * 1. 早期库使用独立 toml_helper_eigen.hpp 提供Eigen反序列化；
 * 2. 新版统一将扩展放入 toml/ext/eigen.hpp 规范扩展路径；
 * 3. 保留旧头文件做兼容转发，保证老项目无需修改include；
 * 测试逻辑：
 * - 解析单行TOML浮点数组；
 * - 使用 toml_helper::read 读取Eigen::Matrix<double,3,1>三维列向量；
 * - 校验三个维度数值完全匹配，证明两套头文件的Eigen反序列化特化均生效
 */
TEST(TomlEigenHeaders, CanonicalAndLegacyHeadersParseFixedSizeMatrices) {
    // 解析单行TOML文本：一维浮点数组对应3维向量
    const auto parsed = toml::parse("vector = [1.0, 2.0, 3.0]");
    // 断言TOML语法解析无错误
    ASSERT_TRUE(parsed);

    // 类型别名：3行1列double固定尺寸列向量 Eigen::Vector3d
    using Vec3 = Eigen::Matrix<double, 3, 1>;

    // 顶层通用读取API，自动匹配Eigen固定矩阵反序列化特化
    auto result = toml_helper::read<Vec3>(parsed.table(), "vector");
    // 读取不能报错（数组长度、类型匹配、Eigen特化存在）
    ASSERT_TRUE(result);
    // 分别校验向量三个分量数值精准相等（浮点专用断言）
    EXPECT_DOUBLE_EQ((*result)(0), 1.0);
    EXPECT_DOUBLE_EQ((*result)(1), 2.0);
    EXPECT_DOUBLE_EQ((*result)(2), 3.0);
}