#include <gtest/gtest.h>

#include <Eigen/Core>

#include "toml/ext/eigen.hpp"
#include "toml_helper_eigen.hpp"

TEST(TomlEigenHeaders, CanonicalAndLegacyHeadersParseFixedSizeMatrices) {
    const auto parsed = toml::parse("vector = [1.0, 2.0, 3.0]");
    ASSERT_TRUE(parsed);

    using Vec3 = Eigen::Matrix<double, 3, 1>;

    auto result = toml_helper::read<Vec3>(parsed.table(), "vector");
    ASSERT_TRUE(result);
    EXPECT_DOUBLE_EQ((*result)(0), 1.0);
    EXPECT_DOUBLE_EQ((*result)(1), 2.0);
    EXPECT_DOUBLE_EQ((*result)(2), 3.0);
}
