#include <gtest/gtest.h>

#include <array>
#include <expected>
#include <filesystem>
#include <fmt/format.h>
#include <string>

#include "config.hpp"
#include "core/trajectory/resource.hpp"
#include "runtime/l1_l2_setup.hpp"
#include "scheduler/scheduler.hpp"
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

template <typename Derived, std::size_t N>
void expect_matrix_values(
    const Eigen::MatrixBase<Derived>& actual, const std::array<double, N>& expected) {
    ASSERT_EQ(static_cast<std::size_t>(actual.size()), expected.size());
    for (Eigen::Index i = 0; i < actual.size(); ++i) {
        EXPECT_DOUBLE_EQ(actual(i), expected[static_cast<std::size_t>(i)]) << "index=" << i;
    }
}

TEST(ConfigParse, HeroRobotTomlMatchesExpectedSemanticsAndValues) {
    auto result = parse_config_file<fcs::HardwareConfig>(source_root() / "config/robot/hero.toml");
    ASSERT_TRUE(result.has_value()) << result.error();
    const auto& config = *result;

    expect_matrix_values(
        config.camera->camera_matrix,
        std::array<double, 9>{
            1784.40785518, 0.0, 709.26908080, 0.0, 1784.39799730, 556.61031728, 0.0, 0.0, 1.0});
    expect_matrix_values(
        config.camera->distort_coefficient,
        std::array<double, 5>{-0.05413741, 0.13077699, -0.00008913, 0.00029836, -0.05802791});
    EXPECT_EQ(config.camera->width, 1440u);
    EXPECT_EQ(config.camera->height, 1080u);

    EXPECT_FALSE(config.camera->profile.trigger_mode);
    EXPECT_FALSE(config.camera->profile.invert_image);
    EXPECT_EQ(config.camera->profile.exposure_time_us, 5000u);
    EXPECT_DOUBLE_EQ(config.camera->profile.gain, 16.7);
    EXPECT_EQ(config.camera->profile.rotate_angle, fcs::RotateType::None);

    expect_matrix_values(
        config.extrinsic->gimbal_yaw.gimbal_pitch.translation,
        std::array<double, 3>{-0.06492, 0.0, 0.164});
    expect_matrix_values(
        config.extrinsic->gimbal_yaw.gimbal_pitch.camera_link.translation,
        std::array<double, 3>{0.2403, 0.0, -0.0547});
    expect_matrix_values(
        config.extrinsic->gimbal_yaw.gimbal_pitch.muzzle_link.translation,
        std::array<double, 3>{0.1598, 0.0, 0.0});
    EXPECT_DOUBLE_EQ(config.extrinsic->gimbal_yaw.gimbal_pitch.muzzle_link.roll, 0.0);
    EXPECT_DOUBLE_EQ(config.extrinsic->gimbal_yaw.gimbal_pitch.muzzle_link.pitch, 0.8);
    EXPECT_DOUBLE_EQ(config.extrinsic->gimbal_yaw.gimbal_pitch.muzzle_link.yaw, -0.3);

    EXPECT_EQ(config.mcu->mcu_vendor_id, 0x0483);
    EXPECT_FALSE(config.mcu->mcu_product_id.has_value());
    EXPECT_TRUE(config.mcu->mcu_authoritative_self_color);
    EXPECT_TRUE(config.mcu->mcu_authoritative_bullet_speed);
    EXPECT_DOUBLE_EQ(config.mcu->bullet_speed_default, 11.0);
    EXPECT_DOUBLE_EQ(config.mcu->bullet_speed_min, 10.0);
    EXPECT_DOUBLE_EQ(config.mcu->bullet_speed_max, 20.0);
}

TEST(ConfigParse, InlineHelpersAndFormattersBehaveAsExpected) {
    auto hardware_result =
        parse_config_file<fcs::HardwareConfig>(source_root() / "config/robot/hero.toml");
    ASSERT_TRUE(hardware_result.has_value()) << hardware_result.error();
    const auto& hardware = *hardware_result;

    auto vision_result =
        parse_config_file<fcs::VisionConfig>(source_root() / "config/vision_base.toml");
    ASSERT_TRUE(vision_result.has_value()) << vision_result.error();
    const auto& vision = *vision_result;

    EXPECT_EQ(fmt::format("{}", hardware.camera->profile.rotate_angle), "None");

    const auto muzzle_rotation = hardware.extrinsic->gimbal_yaw.gimbal_pitch.muzzle_link.rotation();
    const auto [roll, pitch, yaw] = muzzle_rotation.rpy();
    EXPECT_DOUBLE_EQ(roll, 0.0);
    EXPECT_DOUBLE_EQ(pitch, 0.8);
    EXPECT_DOUBLE_EQ(yaw, -0.3);

    const auto tracker_ptr = vision.l3->tracker_ptr();
    ASSERT_TRUE(tracker_ptr);
    EXPECT_DOUBLE_EQ(tracker_ptr->robot.matcher_gate, vision.tracker().robot.matcher_gate);
    EXPECT_DOUBLE_EQ(tracker_ptr->outpost.model.yaw_log_k, 0.005);
    EXPECT_TRUE(vision.quanta_filter.enable_denoise_luma);
    EXPECT_EQ(vision.quanta_filter.denoise_luma.kernel_size, 5);
    EXPECT_DOUBLE_EQ(vision.quanta_filter.denoise_luma.sigma_x, 1.0);
    EXPECT_DOUBLE_EQ(vision.quanta_filter.denoise_luma.sigma_y, 1.0);
    EXPECT_TRUE(vision.quanta_filter.enable_denoise_chroma);
    EXPECT_EQ(vision.quanta_filter.denoise_chroma.kernel_size, 3);
    EXPECT_DOUBLE_EQ(vision.quanta_filter.denoise_chroma.sigma_x, 1.0);
    EXPECT_DOUBLE_EQ(vision.quanta_filter.denoise_chroma.sigma_y, 1.0);
    EXPECT_TRUE(vision.quanta_filter.enable_luma_quantization);
    EXPECT_EQ(vision.quanta_filter.luma_levels, 16);
    EXPECT_FALSE(vision.quanta.enVBR);
    EXPECT_EQ(vision.quanta.target_bitrate, 45000);
    EXPECT_EQ(vision.quanta.min_bit_rate, 0);
    EXPECT_EQ(vision.quanta.max_bit_rate, 50000);

    EXPECT_TRUE(
        fcs::L5::is_on_target(vision.l5->fire_decision, 0.0, 0.0, 0.001, 0.001, 100.0).fire);
    EXPECT_FALSE(fcs::L5::is_on_target(vision.l5->fire_decision, 0.0, 0.0, 0.2, 0.2, 100.0).fire);
}

TEST(ConfigParse, MutableAccessorsAndTrajectoryHelpersUseWrappedConfigFields) {
    auto vision_result =
        parse_config_file<fcs::VisionConfig>(source_root() / "config/vision_base.toml");
    ASSERT_TRUE(vision_result.has_value()) << vision_result.error();
    auto vision = *vision_result;

    vision.tracker().robot.matcher_gate = 12.0;
    EXPECT_DOUBLE_EQ(vision.l3->tracker.robot.matcher_gate, 12.0);

    vision.weapon().enable_debug = false;
    EXPECT_FALSE(vision.l5->mpc_weapon.enable_debug);

    fcs::core::trajectory::TrajectoryConfig ideal{};
    ideal.model->type    = fcs::core::trajectory::model::ModelType::Ideal;
    ideal.model->gravity = 9.81;
    auto ideal_solver    = fcs::core::trajectory::solver::create_solver(ideal);
    ASSERT_NE(ideal_solver, nullptr);

    fcs::core::trajectory::TrajectoryConfig linear{};
    linear.model->type       = fcs::core::trajectory::model::ModelType::LinearDrag;
    linear.model->gravity    = 9.79;
    linear.model->resistance = 0.001;
    auto linear_solver       = fcs::core::trajectory::solver::create_solver(linear);
    ASSERT_NE(linear_solver, nullptr);

    talos::World world;
    talos::Scheduler scheduler(world);
    fcs::core::trajectory::register_resource(scheduler, std::move(linear));
}

TEST(ConfigParse, SetupL1CoversUsbMcuWrapperBranchesBeforeStubCameraFails) {
    auto hardware_result =
        parse_config_file<fcs::HardwareConfig>(source_root() / "config/robot/hero.toml");
    ASSERT_TRUE(hardware_result.has_value()) << hardware_result.error();
    auto hardware = *hardware_result;

    {
        talos::World world;
        talos::Scheduler scheduler(world);
        auto result = fcs::runtime::setup_l1(world, scheduler, false, false, &hardware);
        ASSERT_FALSE(result.has_value());
        EXPECT_NE(result.error().find("failed connecting to mcu"), std::string::npos);
    }

    hardware.mcu->mcu_product_id = 0x1234;
    {
        talos::World world;
        talos::Scheduler scheduler(world);
        auto result = fcs::runtime::setup_l1(world, scheduler, false, false, &hardware);
        ASSERT_FALSE(result.has_value());
        EXPECT_NE(result.error().find("failed connecting to mcu"), std::string::npos);
    }
}

} // namespace
