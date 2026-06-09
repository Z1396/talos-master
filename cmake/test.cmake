# Build testing (default OFF for faster builds)
option(TALOS_BUILD_TESTING "Build tests" OFF)

if(NOT TALOS_BUILD_TESTING)
    return()
endif()

find_package(GTest REQUIRED)

# Sanitizers (optional - enable via CMake flags)
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(ENABLE_COVERAGE "Enable code coverage" OFF)

# Sanitizer mutual exclusion check
if(ENABLE_ASAN AND ENABLE_TSAN)
    message(FATAL_ERROR "Cannot enable both AddressSanitizer and ThreadSanitizer simultaneously")
endif()

if(ENABLE_ASAN)
    message(STATUS "AddressSanitizer enabled")
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
endif()

if(ENABLE_TSAN)
    message(STATUS "ThreadSanitizer enabled")
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
    add_link_options(-fsanitize=thread)
endif()

if(ENABLE_COVERAGE)
    message(STATUS "Coverage enabled")
endif()

enable_testing()
