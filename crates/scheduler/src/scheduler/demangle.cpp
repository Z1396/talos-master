#include "scheduler/demangle.hpp"

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <cxxabi.h>

namespace talos::scheduler::detail {

std::string demangle(const char* name) noexcept {
    if (!name) {
        return "<null>";
    }

    int status = -1;
    char* p    = abi::__cxa_demangle(name, nullptr, nullptr, &status);

    std::string s;
    if (status == 0 && p) {
        s = p;
        std::free(p);
    } else {
        s = name;
    }

    std::size_t pos;
    while ((pos = s.find("::")) != std::string::npos) {
        std::size_t start = pos;
        while (start > 0
               && (std::isalnum(static_cast<unsigned char>(s[start - 1])) || s[start - 1] == '_')) {
            --start;
        }
        s.erase(start, pos - start + 2);
    }

    const char* patterns[] = {", std::allocator<", ", std::char_traits<", ", std::less<",
                              ", std::equal_to<",  ", std::hash<",        ", std::default_delete<"};
    for (const char* pat : patterns) {
        std::size_t p = 0;
        while ((p = s.find(pat, p)) != std::string::npos) {
            std::size_t end = s.find('>', p + std::strlen(pat));
            if (end != std::string::npos) {
                s.erase(p, end - p + 1);
            } else {
                break;
            }
        }
    }

    std::string cleaned;
    bool in_template = false;
    for (std::size_t i = 0; i < s.size(); ++i) {
        char c = s[i];
        if (c == '<') {
            in_template = true;
            cleaned += c;
            continue;
        }
        if (c == '>') {
            in_template = false;
            cleaned += c;
            continue;
        }
        if (in_template && (c == ' ' || (c == ',' && (i + 1 < s.size() && s[i + 1] == ' ')))) {
            if (c == ',' && i + 1 < s.size() && s[i + 1] == ' ') {
                ++i;
            }
            continue;
        }
        cleaned += c;
    }

    std::size_t start = cleaned.find_first_not_of(" \t");
    if (start == std::string::npos) {
        return "<empty>";
    }
    std::size_t end = cleaned.find_last_not_of(" \t");
    cleaned         = cleaned.substr(start, end - start + 1);

    return cleaned.empty() ? "<unknown>" : cleaned;
}

} // namespace talos::scheduler::detail
