#include "runtime/build_info.hpp"
#include "runtime/replay.hpp"
#include "spdlog_hook.hpp"

#include <iostream>

int main(int argc, char** argv) {
    init_logger();
    hook_cstream();

    const auto build = fcs::build_info();
    SPDLOG_INFO(
        "talos-replay build version={}@{} git={} host={}", build.git_branch, build.build_date,
        build.git_commit, build.build_host);

    const auto options = fcs::runtime::parse_replay_options(argc, argv);
    if (!options) {
        std::cerr << options.error() << '\n';
        if (options.error().rfind("usage:", 0) == 0) {
            return 0;
        }
        std::cerr << fcs::runtime::replay_usage(argc > 0 ? argv[0] : "talos-replay") << '\n';
        return 1;
    }

    const auto replay = fcs::runtime::run_replay(*options);
    if (!replay) {
        SPDLOG_CRITICAL("{}", replay.error());
        return 1;
    }

    return 0;
}
