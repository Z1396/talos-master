# System library dependencies - use instead of fetching from GitHub
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)

# boost::pfr has no system package, fetch from a mirror
include(FetchContent)
FetchContent_Declare(
  boost_pfr
  URL https://archives.boost.io/release/1.90.0/source/pfr.hpp
  DOWNLOAD_NO_EXTRACT TRUE
)
