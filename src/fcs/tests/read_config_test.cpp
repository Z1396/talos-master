#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "runtime/config_loader.hpp"

namespace {

std::filesystem::path source_root() { return std::filesystem::path(TALOS_SOURCE_DIR); }

class ScopedCurrentPath {
public:
    explicit ScopedCurrentPath(const std::filesystem::path& path)
        : old_path_(std::filesystem::current_path()) {
        std::filesystem::current_path(path);
    }

    ~ScopedCurrentPath() { std::filesystem::current_path(old_path_); }

private:
    std::filesystem::path old_path_;
};

void write_file(const std::filesystem::path& path, std::string_view contents) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream(path) << contents;
}

void copy_file_from_repo(const std::filesystem::path& from, const std::filesystem::path& to) {
    std::filesystem::create_directories(to.parent_path());
    std::filesystem::copy_file(
        source_root() / from, to, std::filesystem::copy_options::overwrite_existing);
}

std::filesystem::path make_temp_dir(std::string_view name) {
    const auto unique = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto path   = std::filesystem::temp_directory_path()
                    / ("talos-read-config-" + std::string(name) + "-" + std::to_string(unique));
    std::filesystem::create_directories(path);
    return path;
}

TEST(ReadConfig, RepositoryLaunchConfigLoadsAndMergesOverrideSemantics) {
    const ScopedCurrentPath cwd(source_root());

    auto result = fcs::load_config("at_vision.toml");
    ASSERT_TRUE(result.has_value()) << result.error();

    auto config = std::move(*result);
    EXPECT_TRUE(config.launch.daedalus);
    ASSERT_TRUE(config.hardware.has_value());
    EXPECT_DOUBLE_EQ(config.hardware->mcu->bullet_speed_default, 24.0);

    EXPECT_TRUE(config.foxglove.enabled);
    EXPECT_EQ(config.foxglove.transport, fcs::FoxgloveTransport::WebSocket);
    EXPECT_EQ(config.foxglove.host, "0.0.0.0");
    EXPECT_EQ(config.foxglove.port, 8765);
    EXPECT_FALSE(config.foxglove.mcap_path.empty());
    EXPECT_EQ(config.foxglove.quanta.target_bitrate, 100500);
    EXPECT_FALSE(config.foxglove.quanta.enVBR);

    EXPECT_EQ(config.foxglove.quanta.preset, "fast");
    EXPECT_EQ(config.foxglove.quanta.tune, "ssim");
    EXPECT_TRUE(config.foxglove.quanta.intra_refresh);

    EXPECT_FALSE(config.capturer.enabled);
    EXPECT_EQ(config.capturer.output_dir, "record");

    EXPECT_EQ(config.vision.trajectory.model->type, fcs::core::trajectory::model::ModelType::Ideal);
    EXPECT_DOUBLE_EQ(config.vision.trajectory.model->gravity, 9.81);
    EXPECT_TRUE(config.vision.at_legacy->traditional.advanced_binary);
    EXPECT_EQ(config.vision.at_legacy->default_detect_color, fcs::ArmorColor::Blue);
}

TEST(ReadConfig, MissingRequiredLaunchFieldReturnsEntryConfigError) {
    const auto temp_dir = make_temp_dir("missing-launch-field");
    {
        const ScopedCurrentPath cwd(temp_dir);
        std::ofstream(temp_dir / "at_vision.toml") << "daedalus = false\nvision = 'std'\n";

        auto result = fcs::load_config("at_vision.toml");
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error(), "entry config: Missing key 'robot'");
    }
    std::filesystem::remove_all(temp_dir);
}

TEST(ReadConfig, NonDaedalusLaunchReturnsHardwareAndMergedBaseVision) {
    const auto temp_dir = make_temp_dir("non-daedalus-success");
    copy_file_from_repo("config/vision_base.toml", temp_dir / "config/vision_base.toml");
    copy_file_from_repo("config/robot/hero.toml", temp_dir / "config/robot/hero.toml");
    write_file(temp_dir / "config/vision/std.toml", "");
    write_file(temp_dir / "at_vision.toml", "daedalus = false\nrobot = 'hero'\nvision = 'std'\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_TRUE(result.has_value()) << result.error();

        auto config = std::move(*result);
        EXPECT_FALSE(config.launch.daedalus);
        ASSERT_TRUE(config.hardware.has_value());
        EXPECT_TRUE(config.foxglove.enabled);
        EXPECT_EQ(config.foxglove.transport, fcs::FoxgloveTransport::WebSocket);
        EXPECT_EQ(
            config.vision.trajectory.model->type,
            fcs::core::trajectory::model::ModelType::LinearDrag);
    }

    std::filesystem::remove_all(temp_dir);
}

TEST(ReadConfig, CapturerRequiresOutputDirWhenEnabled) {
    const auto temp_dir = make_temp_dir("capturer-missing-output-dir");
    copy_file_from_repo("config/vision_base.toml", temp_dir / "config/vision_base.toml");
    copy_file_from_repo("config/robot/hero.toml", temp_dir / "config/robot/hero.toml");
    write_file(temp_dir / "config/vision/std.toml", "");
    write_file(
        temp_dir / "at_vision.toml",
        "daedalus = false\nrobot = 'hero'\nvision = 'std'\n[capturer]\nenabled = true\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(
            result.error(),
            "entry config: capturer.output_dir is required when capturer.enabled=true");
    }

    std::filesystem::remove_all(temp_dir);
}

TEST(ReadConfig, McapTransportRequiresPath) {
    const auto temp_dir = make_temp_dir("mcap-missing-path");
    copy_file_from_repo("config/vision_base.toml", temp_dir / "config/vision_base.toml");
    copy_file_from_repo("config/robot/hero.toml", temp_dir / "config/robot/hero.toml");
    write_file(temp_dir / "config/vision/std.toml", "");
    write_file(
        temp_dir / "at_vision.toml",
        "daedalus = false\nrobot = 'hero'\nvision = 'std'\n[foxglove]\ntransport = 'Mcap'\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(
            result.error(),
            "entry config: foxglove.mcap_path is required when foxglove.transport=\"Mcap\"");
    }

    std::filesystem::remove_all(temp_dir);
}

TEST(ReadConfig, McapTransportParsesWhenPathProvided) {
    const auto temp_dir = make_temp_dir("mcap-success");
    copy_file_from_repo("config/vision_base.toml", temp_dir / "config/vision_base.toml");
    copy_file_from_repo("config/robot/hero.toml", temp_dir / "config/robot/hero.toml");
    write_file(temp_dir / "config/vision/std.toml", "");
    write_file(
        temp_dir / "at_vision.toml", "daedalus = false\n"
                                     "robot = 'hero'\n"
                                     "vision = 'std'\n"
                                     "[foxglove]\n"
                                     "transport = 'Mcap'\n"
                                     "mcap_path = 'logs/out.mcap'\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_TRUE(result.has_value()) << result.error();

        auto config = std::move(*result);
        EXPECT_EQ(config.foxglove.transport, fcs::FoxgloveTransport::Mcap);
        EXPECT_EQ(config.foxglove.mcap_path, "logs/out.mcap");
        EXPECT_EQ(config.foxglove.port, 8765);
        EXPECT_EQ(config.foxglove.host, "0.0.0.0");
    }

    std::filesystem::remove_all(temp_dir);
}

TEST(ReadConfig, QuantaOverridesParseWhenProvided) {
    const auto temp_dir = make_temp_dir("quanta-success");
    copy_file_from_repo("config/vision_base.toml", temp_dir / "config/vision_base.toml");
    copy_file_from_repo("config/robot/hero.toml", temp_dir / "config/robot/hero.toml");
    write_file(temp_dir / "config/vision/std.toml", "");
    write_file(
        temp_dir / "at_vision.toml", "daedalus = false\n"
                                     "robot = 'hero'\n"
                                     "vision = 'std'\n"
                                     "[foxglove]\n"
                                     "transport = 'WebSocket'\n"
                                     "[foxglove.quanta]\n"
                                     "target_bitrate = 64000\n"
                                     "enVBR = true\n"
                                     "min_bit_rate = 32000\n"
                                     "max_bit_rate = 96000\n"
                                     "preset = 'medium'\n"
                                     "tune = 'fastdecode'\n"
                                     "intra_refresh = true\n"
                                     "lookahead = 8\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_TRUE(result.has_value()) << result.error();

        auto config = std::move(*result);
        EXPECT_EQ(config.foxglove.quanta.target_bitrate, 64000);
        EXPECT_TRUE(config.foxglove.quanta.enVBR);
        EXPECT_EQ(config.foxglove.quanta.min_bit_rate, 32000);
        EXPECT_EQ(config.foxglove.quanta.max_bit_rate, 96000);
        EXPECT_EQ(config.foxglove.quanta.preset, "medium");
        EXPECT_EQ(config.foxglove.quanta.tune, "fastdecode");
        EXPECT_TRUE(config.foxglove.quanta.intra_refresh);
        EXPECT_EQ(config.foxglove.quanta.lookahead, 8);
    }

    std::filesystem::remove_all(temp_dir);
}

TEST(ReadConfig, InvalidMergedVisionConfigIsReported) {
    const auto temp_dir = make_temp_dir("invalid-vision-config");
    write_file(temp_dir / "config/vision_base.toml", "backend_type = [1]\n");
    write_file(temp_dir / "config/vision/std.toml", "");
    write_file(temp_dir / "at_vision.toml", "daedalus = true\nrobot = 'hero'\nvision = 'std'\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_FALSE(result.has_value());
        const auto& error = result.error();
        EXPECT_NE(error.find("vision config:"), std::string::npos);
        EXPECT_NE(error.find("backend_type"), std::string::npos);
    }

    std::filesystem::remove_all(temp_dir);
}

TEST(ReadConfig, InvalidHardwareConfigIsReported) {
    const auto temp_dir = make_temp_dir("invalid-hardware-config");
    copy_file_from_repo("config/vision_base.toml", temp_dir / "config/vision_base.toml");
    write_file(temp_dir / "config/vision/std.toml", "");
    write_file(temp_dir / "config/robot/hero.toml", "");
    write_file(temp_dir / "at_vision.toml", "daedalus = false\nrobot = 'hero'\nvision = 'std'\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error(), "hardware config: Missing key 'camera'");
    }

    std::filesystem::remove_all(temp_dir);
}

} // namespace
