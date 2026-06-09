#include "primitive/system_info.hpp"

#include <array>
#include <cerrno>
#include <cstring>

#include <unistd.h>

namespace talos::primitive {

auto SystemInfo::get_username() noexcept -> Result {
    static constexpr std::size_t kBufSize = 256;
    static std::array<char, kBufSize> buf{};

    const int ret = getlogin_r(buf.data(), kBufSize);
    if (ret != 0) {
        return std::unexpected(
            std::string("get_username: getlogin_r failed: ") + std::strerror(errno));
    }
    return std::string_view{buf.data()};
}

auto SystemInfo::get_hostname() noexcept -> Result {
    static constexpr std::size_t kBufSize = 256;
    static std::array<char, kBufSize> buf{};

    const int ret = gethostname(buf.data(), kBufSize);
    if (ret != 0) {
        return std::unexpected(
            std::string("get_hostname: gethostname failed: ") + std::strerror(errno));
    }
    return std::string_view{buf.data()};
}

} // namespace talos::primitive
