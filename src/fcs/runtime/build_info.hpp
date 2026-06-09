#pragma once

#include <string_view>

namespace fcs {

struct BuildInfo {
    std::string_view build_date;
    std::string_view build_host;
    std::string_view git_commit;
    std::string_view git_branch;
};

[[nodiscard]] auto build_info() noexcept -> BuildInfo;

} // namespace fcs
