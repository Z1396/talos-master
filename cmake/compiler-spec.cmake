option(ENABLE_SCCACHE "Enable compiler caching via sccache" ON)

if (ENABLE_SCCACHE)
    find_program(SCCACHE_PROGRAM sccache)

    if (SCCACHE_PROGRAM)
        message(STATUS "Sccache found at: ${SCCACHE_PROGRAM}")
        if (NOT CMAKE_C_COMPILER_LAUNCHER)
            set(CMAKE_C_COMPILER_LAUNCHER ${SCCACHE_PROGRAM})
        endif ()
        if (NOT CMAKE_CXX_COMPILER_LAUNCHER)
            set(CMAKE_CXX_COMPILER_LAUNCHER ${SCCACHE_PROGRAM})
        endif ()
    else ()
        message(STATUS "sccache not found, building without compiler cache")
    endif ()
else ()
    message(STATUS "sccache disabled")
endif ()

add_compile_options(-fPIC)

# Common optimization options (work for both GCC and Clang)
#
# Host-native tuning is only valid for native builds. During cross-compilation it
# can either be rejected outright or silently tune code for the build host
# instead of the target SoC.
if (CMAKE_CROSSCOMPILING)
  # Target: ARM Cortex-A55 (aarch64, ARMv8.2-A)
  #   8× in-order cores, 2-wide decode, NEON + LSE atomics + dotprod
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    add_compile_options(
        -mcpu=cortex-a55
        -mno-outline-atomics
        -ffp-contract=fast
    )
    add_link_options(-Wl,--gc-sections)
  endif ()
else ()
  add_compile_options(
        -march=native
        -mtune=native
  )
endif ()

add_compile_options(
    -funroll-loops
    -fvectorize
    -fslp-vectorize
    -fno-math-errno
    -fno-omit-frame-pointer
    -fno-plt

    -fdata-sections
    -ffunction-sections
)

# Common warning options (work for both GCC and Clang)
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Werror=pedantic

    -Werror=enum-conversion
    # -Wswitch-default
    -Werror=implicit-fallthrough

    -Werror=misleading-indentation
    -Werror=non-virtual-dtor
    -Werror=overloaded-virtual
    -Werror=suggest-override

    -Werror=null-dereference
    -Werror=array-bounds
    -Werror=vla
    -Werror=alloca
    -Werror=pointer-arith
    -Wformat=2
    -Werror=format
    -Werror=format-security
    -Werror=cast-align
    -Werror=cast-qual
    -Werror=redundant-decls
    -Werror=undef

    -Werror=return-type
    -Werror=uninitialized
)

# Clang-specific warning options
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        # Type safety
        -Werror=bool-conversion
        -Werror=string-plus-int

        -Werror=header-hygiene
        -Werror=move
        -Werror=assign-enum
        -Werror=bad-function-cast

        -Wrange-loop-analysis
        -Wthread-safety
        -Wthread-safety-analysis
    )
endif()
