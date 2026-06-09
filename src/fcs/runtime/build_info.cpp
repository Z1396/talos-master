#include "runtime/build_info.hpp"

#ifndef TALOS_BUILD_INFO_DATE
# define TALOS_BUILD_INFO_DATE "unknown"
#endif

#ifndef TALOS_BUILD_INFO_GIT_COMMIT
# define TALOS_BUILD_INFO_GIT_COMMIT "unknown"
#endif

#ifndef TALOS_BUILD_INFO_HOST
# define TALOS_BUILD_INFO_HOST "unknown"
#endif

#ifndef TALOS_BUILD_INFO_GIT_BRANCH
# define TALOS_BUILD_INFO_GIT_BRANCH "unknown"
#endif

namespace fcs {

[[nodiscard]] auto build_info() noexcept -> BuildInfo {
    return BuildInfo{
        .build_date = TALOS_BUILD_INFO_DATE,
        .build_host = TALOS_BUILD_INFO_HOST,
        .git_commit = TALOS_BUILD_INFO_GIT_COMMIT,
        .git_branch = TALOS_BUILD_INFO_GIT_BRANCH,
    };
}

} // namespace fcs
