# FetchContent helper: fetch dependencies with automatic local caching
#
# Usage:
#   # By version tag (GitHub)
#   fetch_dependency(NAME fmt REPO fmtlib/fmt VERSION 12.1.0)
#
#   # By version tag (GitLab)
#   fetch_dependency(NAME eigen3 REPO gitlab.com/libeigen/eigen VERSION 5.0.1)
#
#   # By commit hash
#   fetch_dependency(NAME fmt REPO fmtlib/fmt COMMIT abc123)
#
#   # By git (supports local repo)
#   fetch_dependency(NAME fmt GIT_REPOSITORY https://github.com/fmtlib/fmt.git GIT_TAG 12.1.0)
#
#   # Custom ZIP URL
#   fetch_dependency(NAME foo ZIP_URL https://... ZIP_NAME foo.zip)
#
# All downloaded files are cached to 3dparty/ for offline builds.

include(FetchContent)

# =============================================================================
# Private helpers
# =============================================================================

function(_download_and_cache zip_url zip_name)
    set(local_zip "${CMAKE_CURRENT_SOURCE_DIR}/3dparty/${zip_name}")

    if(EXISTS "${local_zip}")
        # Check for corrupted zero-byte file
        file(SIZE "${local_zip}" file_size)
        if(file_size EQUAL 0)
            message(WARNING "Cached file ${local_zip} is corrupted (0 bytes), re-downloading...")
            file(REMOVE "${local_zip}")
        else()
            return()
        endif()
    endif()

    file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/3dparty")

    message(STATUS "Downloading ${zip_name} to 3dparty/")
    file(DOWNLOAD "${zip_url}" "${local_zip}" STATUS status LOG log)

    list(GET status 0 status_code)
    if(NOT status_code EQUAL 0)
        message(FATAL_ERROR "Failed to download ${zip_url}:\n${log}")
    endif()

    message(STATUS "Cached to: ${local_zip}")
endfunction()

function(_fetch_dependency_from_zip name zip_url zip_name)
    _download_and_cache("${zip_url}" "${zip_name}")
    set(local_zip "${CMAKE_CURRENT_SOURCE_DIR}/3dparty/${zip_name}")
    message(STATUS "Using cached ${name}: ${local_zip}")
    #FetchContent_Declare(${name} URL "${local_zip}" SYSTEM)
    FetchContent_Declare(${name} URL "${local_zip}")
    FetchContent_MakeAvailable(${name})
endfunction()

function(_fetch_dependency_from_git name repo tag)
    if(EXISTS "${repo}")
        message(STATUS "Using local git repo for ${name}: ${repo}")
    else()
        message(STATUS "Cloning ${name} from ${repo} (tag: ${tag})")
    endif()

    FetchContent_Declare(${name}
        GIT_REPOSITORY ${repo}
        GIT_TAG ${tag}
        GIT_SHALLOW TRUE
        SYSTEM
        GIT_SUBMODULES_RECURSE TRUE
    )
    FetchContent_MakeAvailable(${name})
endfunction()

function(_build_archive_url out_var repo ref)
    if(repo MATCHES "github.com/")
        set(${out_var} "https://${repo}/archive/${ref}.zip" PARENT_SCOPE)
    elseif(repo MATCHES "gitlab.com/")
        string(REGEX REPLACE ".*/([^/]+)/?$" "\\1" project_name "${repo}")
        string(REPLACE "/" "-" project_name_safe "${project_name}")
        set(${out_var} "https://${repo}/-/archive/${ref}/${project_name_safe}-${ref}.zip" PARENT_SCOPE)
    else()
        set(${out_var} "https://github.com/${repo}/archive/${ref}.zip" PARENT_SCOPE)
    endif()
endfunction()

# =============================================================================
# Public API
# =============================================================================

function(fetch_dependency)
    cmake_parse_arguments(ARG
        ""
        "NAME;REPO;VERSION;COMMIT;GIT_REPOSITORY;GIT_TAG;ZIP_URL;ZIP_NAME"
        ""
        ${ARGN})

    if(NOT ARG_NAME)
        message(FATAL_ERROR "fetch_dependency: NAME is required")
    endif()

    # Git mode
    if(ARG_GIT_REPOSITORY)
        if(NOT ARG_GIT_TAG)
            message(FATAL_ERROR "fetch_dependency: GIT_REPOSITORY requires GIT_TAG")
        endif()
        _fetch_dependency_from_git("${ARG_NAME}" "${ARG_GIT_REPOSITORY}" "${ARG_GIT_TAG}")
        return()
    endif()

    # ZIP mode: build filename and URL
    if(NOT ARG_ZIP_NAME)
        if(ARG_COMMIT)
            set(ARG_ZIP_NAME "${ARG_NAME}-${ARG_COMMIT}.zip")
        elseif(ARG_VERSION)
            string(REPLACE "v" "" _version_clean "${ARG_VERSION}")
            set(ARG_ZIP_NAME "${ARG_NAME}-${_version_clean}.zip")
        else()
            message(FATAL_ERROR "fetch_dependency: Must specify VERSION, COMMIT, GIT_REPOSITORY, or ZIP_URL")
        endif()
    endif()

    if(NOT ARG_ZIP_URL)
        if(NOT ARG_REPO)
            message(FATAL_ERROR "fetch_dependency: REPO required for archive download")
        endif()

        set(_ref "${ARG_COMMIT}")
        if(NOT _ref)
            set(_ref "${ARG_VERSION}")
        endif()

        _build_archive_url(ARG_ZIP_URL "${ARG_REPO}" "${_ref}")
    endif()

    _fetch_dependency_from_zip("${ARG_NAME}" "${ARG_ZIP_URL}" "${ARG_ZIP_NAME}")
endfunction()
