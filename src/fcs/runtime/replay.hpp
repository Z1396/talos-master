#pragma once

#include <chrono>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace fcs::runtime {

struct ReplayOptions {
    std::optional<std::filesystem::path> input_path{};
    double speed{1.0};
    bool loop{false};
    std::chrono::milliseconds startup_offset{0};
    std::optional<size_t> max_images{};
};

[[nodiscard]] auto replay_usage(std::string_view program) -> std::string;

[[nodiscard]] auto parse_replay_options(int argc, char** argv) noexcept
    -> std::expected<ReplayOptions, std::string>;

[[nodiscard]] auto run_replay(const ReplayOptions& options) noexcept
    -> std::expected<void, std::string>;

} // namespace fcs::runtime
