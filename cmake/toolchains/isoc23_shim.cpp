#include <cstdlib>

#if defined(__GNUC__) || defined(__clang__)
# define TALOS_DEFAULT_VISIBILITY __attribute__((visibility("default")))
#else
# define TALOS_DEFAULT_VISIBILITY
#endif

extern "C" {

TALOS_DEFAULT_VISIBILITY long int __isoc23_strtol(const char* nptr, char** endptr, int base) {
    return std::strtol(nptr, endptr, base);
}

TALOS_DEFAULT_VISIBILITY unsigned long int
    __isoc23_strtoul(const char* nptr, char** endptr, int base) {
    return std::strtoul(nptr, endptr, base);
}

TALOS_DEFAULT_VISIBILITY long long int __isoc23_strtoll(const char* nptr, char** endptr, int base) {
    return std::strtoll(nptr, endptr, base);
}

TALOS_DEFAULT_VISIBILITY unsigned long long int
    __isoc23_strtoull(const char* nptr, char** endptr, int base) {
    return std::strtoull(nptr, endptr, base);
}

} // extern "C"
