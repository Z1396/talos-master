#pragma once

#include <Eigen/Core>
#include <array>
#include <concepts>
#include <cstdint>
#include <expected>
#include <fmt/format.h>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#ifndef MAGIC_ENUM_RANGE_MIN
# define MAGIC_ENUM_RANGE_MIN 0
#endif
#ifndef MAGIC_ENUM_RANGE_MAX
# define MAGIC_ENUM_RANGE_MAX 16
#endif
#include <magic_enum.hpp>

#define TOML_HEADER_ONLY 0
#define TOML_EXCEPTIONS  0
#include <toml++/toml.hpp>
