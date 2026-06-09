#include <gtest/gtest.h>

#include <optional>
#include <vector>

#include "toml_helper.hpp"
#include "toml_helper_containers.hpp"

TEST(TomlCompatHeaders, LegacyFlatHeadersStillExposeCoreAndContainers) {
    const auto parsed = toml::parse(R"(
        label = "robot"
        values = [1, 2, 3]
    )");
    ASSERT_TRUE(parsed);

    auto label = toml_helper::read<std::string>(parsed.table(), "label");
    ASSERT_TRUE(label);
    EXPECT_EQ(*label, "robot");

    auto values = toml_helper::read<std::vector<int>>(parsed.table(), "values");
    ASSERT_TRUE(values);
    EXPECT_EQ(values->size(), 3U);
    EXPECT_EQ((*values)[1], 2);
}
