set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES
    TALOS_SYSROOT
    TALOS_TARGET_TRIPLE
    TALOS_LLVM_ROOT
    TALOS_GNU_TOOLCHAIN_ROOT
    TALOS_CLANG_RESOURCE_DIR)

set(TALOS_SYSROOT "$ENV{TALOS_SYSROOT}" CACHE PATH "Target aarch64 Linux sysroot")
if (NOT TALOS_SYSROOT)
    message(FATAL_ERROR
        "TALOS_SYSROOT is not set. Point it at the copied target rootfs "
        "(for example /opt/sysroots/ax650n-rootfs).")
endif ()

if (NOT IS_DIRECTORY "${TALOS_SYSROOT}")
    message(FATAL_ERROR "TALOS_SYSROOT does not exist: ${TALOS_SYSROOT}")
endif ()

get_filename_component(TALOS_SYSROOT "${TALOS_SYSROOT}" REALPATH)

set(TALOS_TARGET_TRIPLE "aarch64-unknown-linux-gnu" CACHE STRING
    "GNU target triple used for clang cross-compilation")
set(TALOS_LLVM_ROOT "" CACHE PATH
    "Optional LLVM/Clang installation root (should contain bin/clang and bin/clang++)")
set(TALOS_GNU_TOOLCHAIN_ROOT "" CACHE PATH
    "GNU cross-toolchain root used by clang to locate libstdc++, crt objects and binutils")
set(TALOS_CLANG_RESOURCE_DIR "" CACHE PATH
    "Target Clang resource directory that contains lib/linux/libclang_rt.*")

function(_talos_normalize_gnu_toolchain_root out_var input_root)
    set(_root "${input_root}")
    if (IS_DIRECTORY "${_root}/toolchain/bin" AND IS_DIRECTORY "${_root}/toolchain/lib/gcc")
        set(_root "${_root}/toolchain")
    endif ()
    if (_root AND IS_DIRECTORY "${_root}")
        get_filename_component(_root "${_root}" REALPATH)
    endif ()
    set(${out_var} "${_root}" PARENT_SCOPE)
endfunction()

if (TALOS_GNU_TOOLCHAIN_ROOT)
    _talos_normalize_gnu_toolchain_root(TALOS_GNU_TOOLCHAIN_ROOT "${TALOS_GNU_TOOLCHAIN_ROOT}")
endif ()

set(_talos_program_roots "")
if (TALOS_LLVM_ROOT)
    list(APPEND _talos_program_roots "${TALOS_LLVM_ROOT}")
endif ()
if (EXISTS "/opt/homebrew/opt/llvm/bin")
    list(APPEND _talos_program_roots "/opt/homebrew/opt/llvm")
endif ()
if (EXISTS "/usr/local/opt/llvm/bin")
    list(APPEND _talos_program_roots "/usr/local/opt/llvm")
endif ()
if (TALOS_GNU_TOOLCHAIN_ROOT)
    list(APPEND _talos_program_roots "${TALOS_GNU_TOOLCHAIN_ROOT}")
endif ()

foreach(_talos_root IN LISTS _talos_program_roots)
    if (IS_DIRECTORY "${_talos_root}/bin")
        list(PREPEND CMAKE_PROGRAM_PATH "${_talos_root}/bin")
    endif ()
endforeach ()

find_program(TALOS_CLANG
    NAMES clang-21 clang
    REQUIRED)
find_program(TALOS_CLANGXX
    NAMES clang++-21 clang++
    REQUIRED)

set(CMAKE_C_COMPILER "${TALOS_CLANG}" CACHE FILEPATH "Clang C compiler" FORCE)
set(CMAKE_CXX_COMPILER "${TALOS_CLANGXX}" CACHE FILEPATH "Clang C++ compiler" FORCE)
set(CMAKE_C_COMPILER_TARGET "${TALOS_TARGET_TRIPLE}")
set(CMAKE_CXX_COMPILER_TARGET "${TALOS_TARGET_TRIPLE}")

if (NOT TALOS_GNU_TOOLCHAIN_ROOT)
    find_program(TALOS_GCC_DRIVER
        NAMES
            ${TALOS_TARGET_TRIPLE}-gcc
            aarch64-linux-gnu-gcc
        REQUIRED)
    get_filename_component(_talos_gcc_bindir "${TALOS_GCC_DRIVER}" DIRECTORY)
    get_filename_component(TALOS_GNU_TOOLCHAIN_ROOT "${_talos_gcc_bindir}/.." REALPATH)
    _talos_normalize_gnu_toolchain_root(TALOS_GNU_TOOLCHAIN_ROOT "${TALOS_GNU_TOOLCHAIN_ROOT}")
endif ()

if (NOT IS_DIRECTORY "${TALOS_GNU_TOOLCHAIN_ROOT}")
    message(FATAL_ERROR "TALOS_GNU_TOOLCHAIN_ROOT does not exist: ${TALOS_GNU_TOOLCHAIN_ROOT}")
endif ()

set(CMAKE_C_COMPILER_EXTERNAL_TOOLCHAIN "${TALOS_GNU_TOOLCHAIN_ROOT}")
set(CMAKE_CXX_COMPILER_EXTERNAL_TOOLCHAIN "${TALOS_GNU_TOOLCHAIN_ROOT}")
set(CMAKE_SYSROOT "${TALOS_SYSROOT}")

# ------------------------------------------------------------------------------
# Target clang resource-dir inside sysroot
#
# Important:
#   --sysroot does not redirect clang's resource-dir.
#   compiler-rt / clang_rt must be wired explicitly.
# ------------------------------------------------------------------------------

if (NOT TALOS_CLANG_RESOURCE_DIR)
    file(GLOB _talos_clang_resource_candidates
        LIST_DIRECTORIES TRUE
        "${CMAKE_SYSROOT}/lib/clang/*"
        "${CMAKE_SYSROOT}/usr/lib/clang/*")

    list(SORT _talos_clang_resource_candidates COMPARE NATURAL ORDER DESCENDING)

    foreach(_candidate IN LISTS _talos_clang_resource_candidates)
        if (IS_DIRECTORY "${_candidate}/lib/linux")
            set(TALOS_CLANG_RESOURCE_DIR "${_candidate}" CACHE PATH
                "Auto-detected target Clang resource directory" FORCE)
            break()
        endif ()
    endforeach ()
endif ()

if (NOT TALOS_CLANG_RESOURCE_DIR)
    message(FATAL_ERROR
        "Could not find target Clang resource dir under sysroot. "
        "Expected something like ${CMAKE_SYSROOT}/lib/clang/<version>/lib/linux.")
endif ()

if (NOT IS_DIRECTORY "${TALOS_CLANG_RESOURCE_DIR}")
    message(FATAL_ERROR
        "TALOS_CLANG_RESOURCE_DIR does not exist: ${TALOS_CLANG_RESOURCE_DIR}")
endif ()

get_filename_component(TALOS_CLANG_RESOURCE_DIR "${TALOS_CLANG_RESOURCE_DIR}" REALPATH)

set(_talos_clang_rt_linux_dir "${TALOS_CLANG_RESOURCE_DIR}/lib/linux")
set(_talos_clang_rt_triple_dir "${TALOS_CLANG_RESOURCE_DIR}/lib/${TALOS_TARGET_TRIPLE}")

if (NOT EXISTS "${_talos_clang_rt_linux_dir}/libclang_rt.builtins-aarch64.a")
    message(FATAL_ERROR
        "Target compiler-rt builtins not found in: "
        "${_talos_clang_rt_linux_dir}")
endif ()

if (EXISTS "${_talos_clang_rt_linux_dir}/libclang_rt.asan-aarch64.a")
    set(TALOS_HAS_ASAN_RUNTIME TRUE CACHE BOOL
        "Target compiler-rt has ASan runtime" FORCE)
else ()
    set(TALOS_HAS_ASAN_RUNTIME FALSE CACHE BOOL
        "Target compiler-rt has ASan runtime" FORCE)
endif ()

set(TALOS_MULTIARCH_DIRS
    "${TALOS_TARGET_TRIPLE}"
    "aarch64-linux-gnu")
list(REMOVE_DUPLICATES TALOS_MULTIARCH_DIRS)

foreach(_talos_arch IN LISTS TALOS_MULTIARCH_DIRS)
    if (IS_DIRECTORY "${CMAKE_SYSROOT}/usr/lib/${_talos_arch}")
        set(CMAKE_LIBRARY_ARCHITECTURE "${_talos_arch}" CACHE STRING
            "Target library architecture inside the sysroot" FORCE)
        break()
    endif ()
endforeach ()

set(_talos_common_target_flags
    "--target=${TALOS_TARGET_TRIPLE} --sysroot=${CMAKE_SYSROOT} --gcc-toolchain=${TALOS_GNU_TOOLCHAIN_ROOT} -resource-dir=${TALOS_CLANG_RESOURCE_DIR}")

set(CMAKE_C_FLAGS_INIT "${_talos_common_target_flags}")
set(CMAKE_CXX_FLAGS_INIT "${_talos_common_target_flags} -stdlib=libstdc++")

set(_talos_linker_flags "${_talos_common_target_flags} -fuse-ld=lld")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${_talos_linker_flags}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${_talos_linker_flags}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${_talos_linker_flags}")

find_program(TALOS_LLVM_AR
    NAMES llvm-ar
    NO_CACHE)
if (TALOS_LLVM_AR)
    set(CMAKE_AR "${TALOS_LLVM_AR}" CACHE FILEPATH "Archiver" FORCE)
endif ()

find_program(TALOS_LLVM_RANLIB
    NAMES llvm-ranlib
    NO_CACHE)
if (TALOS_LLVM_RANLIB)
    set(CMAKE_RANLIB "${TALOS_LLVM_RANLIB}" CACHE FILEPATH "Ranlib" FORCE)
endif ()

list(PREPEND CMAKE_FIND_ROOT_PATH
    "${CMAKE_SYSROOT}"
    "${CMAKE_SYSROOT}/usr"
    "${CMAKE_SYSROOT}/usr/local"
    "${CMAKE_SYSROOT}/soc")

set(_talos_prefix_paths
    "${CMAKE_SYSROOT}"
    "${CMAKE_SYSROOT}/usr"
    "${CMAKE_SYSROOT}/usr/local"
    "${CMAKE_SYSROOT}/soc")

foreach(_talos_arch IN LISTS TALOS_MULTIARCH_DIRS)
    list(APPEND _talos_prefix_paths
        "${CMAKE_SYSROOT}/usr/lib/${_talos_arch}"
        "${CMAKE_SYSROOT}/usr/lib/${_talos_arch}/cmake"
        "${CMAKE_SYSROOT}/usr/local/lib/${_talos_arch}"
        "${CMAKE_SYSROOT}/usr/local/lib/${_talos_arch}/cmake")
endforeach ()

set(CMAKE_PREFIX_PATH
    "${_talos_prefix_paths}"
    CACHE STRING "Cross-compilation prefix roots" FORCE)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(_talos_pkgconfig_dirs
    "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig"
    "${CMAKE_SYSROOT}/usr/lib/pkgconfig"
    "${CMAKE_SYSROOT}/usr/share/pkgconfig"
    "${CMAKE_SYSROOT}/lib/aarch64-linux-gnu/pkgconfig"
    "${CMAKE_SYSROOT}/lib/pkgconfig"
    "${CMAKE_SYSROOT}/soc/lib/pkgconfig")
list(JOIN _talos_pkgconfig_dirs ":" _talos_pkgconfig_libdir)

set(ENV{PKG_CONFIG_SYSROOT_DIR} "${CMAKE_SYSROOT}")
set(ENV{PKG_CONFIG_LIBDIR} "${_talos_pkgconfig_libdir}")
unset(ENV{PKG_CONFIG_PATH})

function(_talos_seed_package_dir package)
    if (DEFINED ${package}_DIR AND EXISTS "${${package}_DIR}")
        return()
    endif ()

    foreach(_config_name IN LISTS ARGN)
        set(_config_matches "")
        foreach(_talos_arch IN LISTS TALOS_MULTIARCH_DIRS)
            file(GLOB _config_matches LIST_DIRECTORIES FALSE
                "${CMAKE_SYSROOT}/usr/lib/${_talos_arch}/cmake/*/${_config_name}"
                "${CMAKE_SYSROOT}/usr/local/lib/${_talos_arch}/cmake/*/${_config_name}"
                "${CMAKE_SYSROOT}/usr/lib/cmake/*/${_config_name}"
                "${CMAKE_SYSROOT}/usr/local/lib/cmake/*/${_config_name}"
                "${CMAKE_SYSROOT}/usr/share/*/${_config_name}")
            if (_config_matches)
                list(GET _config_matches 0 _config_match)
                get_filename_component(_config_dir "${_config_match}" DIRECTORY)
                set(${package}_DIR "${_config_dir}" CACHE PATH
                    "Auto-detected ${package} package directory in sysroot" FORCE)
                return()
            endif ()
        endforeach ()
    endforeach ()
endfunction()

_talos_seed_package_dir(OpenCV OpenCVConfig.cmake opencv-config.cmake)
_talos_seed_package_dir(TBB TBBConfig.cmake tbb-config.cmake oneTBBConfig.cmake)
_talos_seed_package_dir(Ceres CeresConfig.cmake ceres-config.cmake)
_talos_seed_package_dir(GTest GTestConfig.cmake gtest-config.cmake)

message(STATUS "Cross-compiling for ${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR} with clang")
message(STATUS "Target sysroot: ${CMAKE_SYSROOT}")
message(STATUS "Target triple: ${TALOS_TARGET_TRIPLE}")
message(STATUS "Clang C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Clang CXX compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "GNU toolchain root: ${TALOS_GNU_TOOLCHAIN_ROOT}")
message(STATUS "Target library architecture: ${CMAKE_LIBRARY_ARCHITECTURE}")
message(STATUS "Target Clang resource dir: ${TALOS_CLANG_RESOURCE_DIR}")
message(STATUS "Target clang_rt linux dir: ${_talos_clang_rt_linux_dir}")
message(STATUS "Target ASan runtime: ${TALOS_HAS_ASAN_RUNTIME}")

if (EXISTS "${_talos_clang_rt_triple_dir}/libclang_rt.asan.a")
    message(STATUS "Target clang_rt triple dir: ${_talos_clang_rt_triple_dir}")
else ()
    message(STATUS
        "Target clang_rt triple dir is not populated: ${_talos_clang_rt_triple_dir}. "
        "This is OK if clang uses lib/linux layout.")
endif ()
