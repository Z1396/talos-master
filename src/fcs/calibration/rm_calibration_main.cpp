/**
 * @file rm_calibration_main.cpp
 * @brief RM Calibration Tool - Camera intrinsic and hand-eye calibration
 *
 * Usage:
 *   ./rm_calibration --config calibration.toml
 *   ./rm_calibration --mode Intrinsic
 *   ./rm_calibration --mode Handeye --intrinsic camera_intrinsic.toml
 *   ./rm_calibration --daedalus  # Use daedalus simulator
 */

#include "calibration/calibration_board.hpp"
#include "calibration/calibration_config.hpp"
#include "calibration/calibration_systems.hpp"
#include "calibration/chessboard_detector.hpp"
#include "calibration/handeye_calibrator.hpp"
#include "calibration/intrinsic_calibrator.hpp"
#include "camera_config.hpp"
#include "core/armor_types.hpp"
#include "core/trajectory/resource.hpp"
#include "spdlog_hook.hpp"

#include "config.hpp"
#include "core/channel_topics.hpp"
#include "core/time.hpp"
#include "foxglove_server.hpp"
#include "foxglove_sink.hpp"
#include "runtime/l1_l2_setup.hpp"
#include "scheduler/scheduler.hpp"
#include "toml_helper.hpp"
#include "toml_helper_eigen.hpp"

#include "scheduler/error_formatter.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <spdlog/spdlog.h>
#include <thread>

using namespace fcs::calibration;

// Global signal handler
std::atomic<bool> g_running{true};

void signal_handler(int) { g_running = false; }

void print_usage() {
    std::cout << R"(
RM Calibration Tool - Camera Intrinsic and Hand-Eye Calibration

Usage:
  rm_calibration [options]

Options:
  --config <path>       Configuration file path (default: calibration.toml)
  --mode <mode>         Override calibration mode: Intrinsic, Handeye, Full
  --intrinsic <path>    Load existing intrinsic calibration (required for Handeye mode)
  --daedalus            Use daedalus simulator instead of real hardware
  --help                Show this help message

Examples:
  # Run intrinsic calibration with real hardware
  rm_calibration --mode Intrinsic

  # Run intrinsic calibration with daedalus simulator
  rm_calibration --mode Intrinsic --daedalus

  # Run hand-eye calibration with existing intrinsics
  rm_calibration --mode Handeye --intrinsic camera_intrinsic.toml

  # Run full calibration
  rm_calibration --mode Full

Press 'Space' to capture sample, 'c' to run calibration, 'q' to quit.
)";
}

int main(int argc, char* argv[]) {
    init_logger();
    // Parse command line arguments
    std::string config_path = "calibration.toml";
    std::string intrinsic_path;
    std::optional<CalibrationMode> mode_override;
    std::optional<bool> daedalus_override;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage();
            return 0;
        } else if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--mode" && i + 1 < argc) {
            std::string mode_str = argv[++i];
            auto mode            = magic_enum::enum_cast<CalibrationMode>(mode_str);
            if (mode) {
                mode_override = *mode;
            } else {
                SPDLOG_ERROR("Invalid mode: {}", mode_str);
                return 1;
            }
        } else if (arg == "--intrinsic" && i + 1 < argc) {
            intrinsic_path = argv[++i];
        } else if (arg == "--daedalus") {
            daedalus_override = true;
        }
    }

    // Setup signal handler
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    SPDLOG_INFO("RM Calibration Tool starting...");
    SPDLOG_INFO("Config file: {}", config_path);

    // Load configuration
    auto config_result = CalibrationConfig::load_from_file(config_path);
    if (!config_result) {
        SPDLOG_ERROR("Failed to load config: {}", config_result.error());
        return 1;
    }
    auto config = std::make_shared<CalibrationConfig>(std::move(*config_result));

    // Apply mode override
    if (mode_override) {
        config->mode = *mode_override;
    }

    // Apply daedalus override
    if (daedalus_override) {
        config->input.daedalus = *daedalus_override;
    }

    const bool use_daedalus = config->input.daedalus;

    SPDLOG_INFO("Calibration mode: {}", magic_enum::enum_name(config->mode));
    SPDLOG_INFO("Input source: {}", use_daedalus ? "daedalus simulator" : "real hardware");
    SPDLOG_INFO("Board type: {}", magic_enum::enum_name(config->board.type));
    SPDLOG_INFO(
        "Board size: {}x{}, square: {}mm", config->board.width, config->board.height,
        config->board.square_size * 1000);

    // Create calibration board detector
    auto board_result = create_board(config->board, config->charuco);
    if (!board_result) {
        SPDLOG_ERROR("Failed to create board detector: {}", board_result.error());
        return 1;
    }
    auto board = std::shared_ptr<CalibrationBoard>(std::move(*board_result));

    // Create calibrators
    auto intrinsic_calibrator =
        std::make_shared<IntrinsicCalibrator>(config->capture, config->intrinsic);
    auto handeye_calibrator = std::make_shared<HandEyeCalibrator>(config->capture, config->handeye);

    // Load existing intrinsic if provided or required
    auto intrinsic_result = std::make_shared<IntrinsicResult>();
    if (!intrinsic_path.empty()) {
        auto loaded = toml::parse_file(intrinsic_path);
        if (!loaded) {
            SPDLOG_ERROR("Failed to load intrinsic file: {}", intrinsic_path);
            return 1;
        }

        if (auto cam = loaded.table()["camera"].as_table()) {
            auto cam_cfg = toml_helper::from_table<fcs::CameraConfig>(*cam);
            if (cam_cfg) {
                intrinsic_result->camera_matrix       = cam_cfg->camera_matrix;
                intrinsic_result->distort_coefficient = cam_cfg->distort_coefficient;
                intrinsic_result->width               = cam_cfg->width;
                intrinsic_result->height              = cam_cfg->height;
                SPDLOG_INFO("Loaded intrinsic calibration from {}", intrinsic_path);
            }
        }
    }

    // Validate requirements
    if (config->mode == CalibrationMode::Handeye && intrinsic_result->width == 0) {
        SPDLOG_ERROR("Handeye mode requires intrinsic calibration. Use --intrinsic <path>");
        return 1;
    }

    // Create calibration status
    auto status            = std::make_shared<CalibrationStatus>();
    status->state          = CalibrationState::Idle;
    status->target_samples = config->capture.min_samples;

    // Setup scheduler
    talos::World world;
    talos::Scheduler scheduler(world);

    // Create TF buffer
    [[maybe_unused]] auto& tf_buffer = world.emplace_resource<fast_tf::CoordinateSystem>();

    // Create hardware config if using real hardware
    fcs::HardwareConfig hardware_config;
    hardware_config.camera          = fcs::CameraConfig{};
    hardware_config.camera->width   = config->width;
    hardware_config.camera->height  = config->height;
    hardware_config.camera->profile = config->profile;
    hardware_config.mcu             = fcs::McuConfig{};
    hardware_config.extrinsic       = fcs::RobotExtrinsicConfig{};
    world.insert_resource(fcs::core::trajectory::bullet_speed_data{.bullet_speed = 25.0});
    world.insert_resource(fcs::ArmorColor::Blue);

    // Setup L1_L2 runtime (camera + IMU)
    auto setup_result = fcs::runtime::setup_l1(
        world, scheduler, /*mock_mcu=*/false, use_daedalus,
        use_daedalus ? nullptr : &hardware_config);
    if (!setup_result) {
        SPDLOG_ERROR("Failed to setup L1/L2: {}", setup_result.error());
        return 1;
    }

    // Initialize Foxglove server (reuse fcs::FoxgloveConfig from calibration config)
    std::shared_ptr<fcs::visualization::FoxgloveServer> foxglove_server;
    if (config->foxglove.enabled) {
        auto server_result = fcs::visualization::create_foxglove_server(config->foxglove);
        if (!server_result) {
            SPDLOG_WARN("Foxglove transport failed to initialize: {}", server_result.error());
        } else {
            foxglove_server = std::move(*server_result);
            fcs::visualization::attach_foxglove_sink(*foxglove_server);
            world.insert_resource(foxglove_server);
            if (config->foxglove.transport == fcs::FoxgloveTransport::WebSocket) {
                SPDLOG_INFO(
                    "Foxglove WebSocket started on {}:{}", config->foxglove.host,
                    config->foxglove.port);
            } else {
                SPDLOG_INFO("Foxglove MCAP started at {}", config->foxglove.mcap_path);
            }
        }
    }

    // Insert resources
    world.insert_resource(config);
    world.insert_resource(status);
    world.insert_resource(intrinsic_result);

    // Register calibration systems
    register_calibration_systems(
        scheduler, config, board, intrinsic_calibrator, handeye_calibrator, intrinsic_result);

    // Build scheduler
    if (auto build_result = scheduler.build(); !build_result) {
        SPDLOG_CRITICAL("Failed to build scheduler: {}", build_result.error());
        return 1;
    }

    SPDLOG_INFO("Calibration system built");
    SPDLOG_INFO("Starting scheduler... (Ctrl+C to quit)");

    // Set initial state to capturing
    status->state = CalibrationState::Capturing;

    // Run scheduler in a background thread
    std::thread scheduler_thread([&scheduler]() {
        if (const auto result = scheduler.run(); !result) {
            SPDLOG_ERROR("Scheduler error occurred");
        }
    });

    // Main loop - handle keyboard input and check progress
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Check if we have enough samples
        uint32_t current_samples = 0;
        if (config->mode == CalibrationMode::Intrinsic) {
            current_samples = intrinsic_calibrator->sample_count();
        } else if (config->mode == CalibrationMode::Handeye) {
            current_samples = handeye_calibrator->sample_count();
        }

        // Auto-calibrate when target reached (for automatic mode)
        if (config->capture.auto_capture && current_samples >= config->capture.min_samples
            && status->state == CalibrationState::Capturing) {

            SPDLOG_INFO("Target samples reached, running calibration...");
            status->state = CalibrationState::Calibrating;

            if (config->mode == CalibrationMode::Intrinsic) {
                auto result =
                    intrinsic_calibrator->calibrate(cv::Size(config->width, config->height));
                if (result) {
                    *intrinsic_result = *result;
                    if (auto save_result =
                            save_intrinsic_result(*result, config->output.intrinsic_path);
                        !save_result) {
                        SPDLOG_ERROR("Failed to save intrinsic: {}", save_result.error());
                    } else {
                        SPDLOG_INFO(
                            "Intrinsic calibration saved to {}", config->output.intrinsic_path);
                        SPDLOG_INFO("RMS error: {:.4f} pixels", result->rms_error);
                        status->state = CalibrationState::Completed;
                    }
                } else {
                    SPDLOG_ERROR("Calibration failed: {}", result.error());
                    status->state = CalibrationState::Failed;
                }
            } else if (config->mode == CalibrationMode::Handeye) {
                auto result = handeye_calibrator->calibrate();
                if (result) {
                    if (auto save_result = save_handeye_result(
                            *result, config->output.extrinsic_path, config->handeye.method);
                        !save_result) {
                        SPDLOG_ERROR("Failed to save handeye: {}", save_result.error());
                    } else {
                        print_handeye_result(*result);
                        SPDLOG_INFO(
                            "Hand-eye calibration saved to {}", config->output.extrinsic_path);
                        status->state = CalibrationState::Completed;
                    }
                } else {
                    SPDLOG_ERROR("Calibration failed: {}", result.error());
                    status->state = CalibrationState::Failed;
                }
            }
            if (status->state == CalibrationState::Completed) {
                // Stop after completing calibration
                g_running = false;
            }
        }
    }

    // Cleanup
    SPDLOG_INFO("Shutting down...");
    scheduler.stop();
    if (scheduler_thread.joinable()) {
        scheduler_thread.join();
    }

    return 0;
}
