#include <gtest/gtest.h>

#include <expected>
#include <filesystem>
#include <string>

#include "config.hpp"
#include "toml_helper.hpp"

namespace {

std::filesystem::path source_root() { return std::filesystem::path(TALOS_SOURCE_DIR); }

template <typename T>
std::expected<T, std::string> parse_config_file(const std::filesystem::path& path) {
    auto parsed = toml::parse_file(path.string());
    if (!parsed) {
        return std::unexpected(path.string() + ": " + std::string(parsed.error().description()));
    }

    auto config = toml_helper::from_table<T>(parsed.table());
    if (!config) {
        return std::unexpected(path.string() + ": " + config.error());
    }
    return *config;
}

TEST(LdmConfig, VisionBaseCarriesExpectedGeometryDefaults) {
    auto result = parse_config_file<fcs::VisionConfig>(source_root() / "config/vision_base.toml");
    ASSERT_TRUE(result.has_value()) << result.error();

    const auto& ldm = result->ldm;
    EXPECT_EQ(ldm.target_color, fcs::ArmorColor::Red);
    EXPECT_GE(ldm.min_blob_area_px, 1);
    EXPECT_GE(ldm.min_pairs_for_detection, 1);

    EXPECT_DOUBLE_EQ(ldm.rmse_stable_threshold_px, 8.0);
    EXPECT_DOUBLE_EQ(ldm.rmse_constrained_threshold_px, 8.0);
    EXPECT_DOUBLE_EQ(ldm.max_pose_angle_rad, 0.872664626);
    EXPECT_GT(ldm.min_preliminary_candidate_score, 0.0);
    EXPECT_GE(ldm.min_preliminary_candidate_score_two_pair, ldm.min_preliminary_candidate_score);
    EXPECT_GT(ldm.min_two_pair_mean_center_dy_px, 0.0);

    EXPECT_DOUBLE_EQ(ldm.geometry.octagon_side_length_m, 0.020711);
    EXPECT_DOUBLE_EQ(ldm.geometry.octagon_circumradius_m, 0.02706);
    EXPECT_DOUBLE_EQ(ldm.geometry.pair_center_separation_m, 0.036514);
    EXPECT_DOUBLE_EQ(ldm.geometry.window_length_m, 0.012);
    EXPECT_DOUBLE_EQ(ldm.geometry.window_width_m, 0.009618);
    EXPECT_DOUBLE_EQ(ldm.geometry.volume_height_m, 0.067);
    EXPECT_DOUBLE_EQ(ldm.geometry.detectable_center_z_range_m[0], -0.01);
    EXPECT_DOUBLE_EQ(ldm.geometry.detectable_center_z_range_m[1], 0.01);
}

} // namespace
