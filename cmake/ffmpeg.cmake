include_guard(GLOBAL)

option(TALOS_PREFER_BTBN_FFMPEG "Prefer the latest shared FFmpeg build from BtbN before falling back to system pkg-config" ON)

function(_talos_ffmpeg_cache_dir out_dir)
    set(${out_dir} "${CMAKE_SOURCE_DIR}/3dparty" PARENT_SCOPE)
endfunction()

function(_talos_ffmpeg_btbn_asset out_url out_archive_name)
    if (NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(${out_url} "" PARENT_SCOPE)
        set(${out_archive_name} "" PARENT_SCOPE)
        return()
    endif ()

    if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(_archive_name "ffmpeg-master-latest-linux64-gpl-shared.tar.xz")
    elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
        set(_archive_name "ffmpeg-master-latest-linuxarm64-gpl-shared.tar.xz")
    else ()
        set(_archive_name "")
    endif ()

    if (_archive_name)
        set(${out_url} "https://github.com/BtbN/FFmpeg-Builds/releases/download/latest/${_archive_name}" PARENT_SCOPE)
        set(${out_archive_name} "${_archive_name}" PARENT_SCOPE)
    else ()
        set(${out_url} "" PARENT_SCOPE)
        set(${out_archive_name} "" PARENT_SCOPE)
    endif ()
endfunction()

function(_talos_ffmpeg_find_root extracted_dir out_root)
    if (EXISTS "${extracted_dir}/lib/pkgconfig")
        set(${out_root} "${extracted_dir}" PARENT_SCOPE)
        return()
    endif ()

    file(GLOB _children LIST_DIRECTORIES TRUE RELATIVE "${extracted_dir}" "${extracted_dir}/*")
    foreach(_child IN LISTS _children)
        if (IS_DIRECTORY "${extracted_dir}/${_child}" AND EXISTS "${extracted_dir}/${_child}/lib/pkgconfig")
            set(${out_root} "${extracted_dir}/${_child}" PARENT_SCOPE)
            return()
        endif ()
    endforeach ()

    set(${out_root} "" PARENT_SCOPE)
endfunction()

function(_talos_ffmpeg_download_latest_checksum asset_name out_checksum)
    set(_checksums_url "https://github.com/BtbN/FFmpeg-Builds/releases/download/latest/checksums.sha256")
    _talos_ffmpeg_cache_dir(_cache_dir)
    set(_checksums_path "${_cache_dir}/btbn-ffmpeg-checksums.sha256")
    set(_checksums_tmp_path "${_checksums_path}.tmp")

    file(MAKE_DIRECTORY "${_cache_dir}")

    file(DOWNLOAD
        "${_checksums_url}"
        "${_checksums_tmp_path}"
        STATUS _download_status
        LOG _download_log
        INACTIVITY_TIMEOUT 60
        TIMEOUT 300)

    list(GET _download_status 0 _download_code)
    if (_download_code EQUAL 0)
        file(REMOVE "${_checksums_path}")
        file(RENAME "${_checksums_tmp_path}" "${_checksums_path}")
    else ()
        file(REMOVE "${_checksums_tmp_path}")
        message(WARNING
            "Failed to download BtbN FFmpeg checksums from ${_checksums_url}; "
            "falling back to system FFmpeg.\n${_download_log}")
        set(${out_checksum} "" PARENT_SCOPE)
        return()
    endif ()

    file(STRINGS "${_checksums_path}" _checksum_lines)
    foreach(_line IN LISTS _checksum_lines)
        string(FIND "${_line}" "${asset_name}" _asset_pos)
        if (_asset_pos LESS 0)
            continue()
        endif ()

        string(REGEX MATCH "^([0-9A-Fa-f]+)[ \t]+" _hash_match "${_line}")
        if (_hash_match)
            set(_parsed_hash "${CMAKE_MATCH_1}")
            string(LENGTH "${_parsed_hash}" _parsed_hash_length)
            if (_parsed_hash_length EQUAL 64)
                set(${out_checksum} "${_parsed_hash}" PARENT_SCOPE)
                return()
            endif ()
        endif ()

        message(WARNING
            "Found a checksum entry for ${asset_name}, but could not parse its SHA256 value; "
            "falling back to system FFmpeg.")
        set(${out_checksum} "" PARENT_SCOPE)
        return()
    endforeach ()

    message(WARNING
        "BtbN FFmpeg checksums did not contain an entry for ${asset_name}; "
        "falling back to system FFmpeg.")
    set(${out_checksum} "" PARENT_SCOPE)
endfunction()

function(_talos_ffmpeg_extract_archive archive_path archive_hash out_root)
    set(_extract_dir "${CMAKE_BINARY_DIR}/_deps/talos-ffmpeg")
    set(_extract_stamp "${_extract_dir}/.talos_ffmpeg_sha256")
    set(_needs_extract TRUE)

    if (EXISTS "${_extract_stamp}")
        file(READ "${_extract_stamp}" _current_hash)
        string(STRIP "${_current_hash}" _current_hash)
        if (_current_hash STREQUAL "${archive_hash}")
            set(_needs_extract FALSE)
        endif ()
    endif ()

    if (_needs_extract)
        file(REMOVE_RECURSE "${_extract_dir}")
        file(MAKE_DIRECTORY "${_extract_dir}")

        execute_process(
            COMMAND "${CMAKE_COMMAND}" -E tar xJf "${archive_path}"
            WORKING_DIRECTORY "${_extract_dir}"
            RESULT_VARIABLE _extract_status
            ERROR_VARIABLE _extract_error
        )

        if (NOT _extract_status EQUAL 0)
            message(WARNING
                "Failed to extract BtbN FFmpeg archive ${archive_path}; "
                "falling back to system FFmpeg.\n${_extract_error}")
            set(${out_root} "" PARENT_SCOPE)
            return()
        endif ()

        file(WRITE "${_extract_stamp}" "${archive_hash}\n")
    endif ()

    _talos_ffmpeg_find_root("${_extract_dir}" _ffmpeg_root)
    if (NOT _ffmpeg_root)
        message(WARNING
            "Extracted BtbN FFmpeg archive ${archive_path}, but no lib/pkgconfig directory was found; "
            "falling back to system FFmpeg.")
        set(${out_root} "" PARENT_SCOPE)
        return()
    endif ()

    set(${out_root} "${_ffmpeg_root}" PARENT_SCOPE)
endfunction()

function(_talos_try_btbn_ffmpeg out_root)
    _talos_ffmpeg_btbn_asset(_archive_url _archive_name)
    if (NOT _archive_url)
        message(WARNING
            "BtbN FFmpeg latest builds are unavailable for ${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}; "
            "falling back to system FFmpeg.")
        set(${out_root} "" PARENT_SCOPE)
        return()
    endif ()

    _talos_ffmpeg_cache_dir(_cache_dir)
    set(_cache_archive "${_cache_dir}/${_archive_name}")

    if (EXISTS "${_cache_archive}")
        file(SIZE "${_cache_archive}" _cache_archive_size)
        if (_cache_archive_size GREATER 0)
            file(SHA256 "${_cache_archive}" _cached_hash)
            message(STATUS "Using cached BtbN FFmpeg archive: ${_cache_archive}")
            _talos_ffmpeg_extract_archive("${_cache_archive}" "${_cached_hash}" _ffmpeg_root)
            set(${out_root} "${_ffmpeg_root}" PARENT_SCOPE)
            return()
        endif ()

        message(WARNING
            "Cached BtbN FFmpeg archive ${_cache_archive} is empty; re-downloading latest build.")
        file(REMOVE "${_cache_archive}")
    endif ()

    _talos_ffmpeg_download_latest_checksum("${_archive_name}" _expected_hash)
    if (NOT _expected_hash)
        set(${out_root} "" PARENT_SCOPE)
        return()
    endif ()

    file(MAKE_DIRECTORY "${_cache_dir}")
    set(_download_tmp "${_cache_archive}.tmp")

    message(STATUS "Downloading latest BtbN FFmpeg build: ${_archive_url}")
    file(DOWNLOAD
        "${_archive_url}"
        "${_download_tmp}"
        STATUS _download_status
        LOG _download_log
        SHOW_PROGRESS
        INACTIVITY_TIMEOUT 60
        TIMEOUT 900)

    list(GET _download_status 0 _download_code)
    if (NOT _download_code EQUAL 0)
        file(REMOVE "${_download_tmp}")
        message(WARNING
            "Failed to download BtbN FFmpeg archive from ${_archive_url}; "
            "falling back to system FFmpeg.\n${_download_log}")
        set(${out_root} "" PARENT_SCOPE)
        return()
    endif ()

    file(SHA256 "${_download_tmp}" _downloaded_hash)
    if (NOT _downloaded_hash STREQUAL _expected_hash)
        file(REMOVE "${_download_tmp}")
        message(WARNING
            "Downloaded BtbN FFmpeg archive ${_archive_url}, but its SHA256 was ${_downloaded_hash} instead of ${_expected_hash}; "
            "falling back to system FFmpeg.")
        set(${out_root} "" PARENT_SCOPE)
        return()
    endif ()

    file(REMOVE "${_cache_archive}")
    file(RENAME "${_download_tmp}" "${_cache_archive}")

    _talos_ffmpeg_extract_archive("${_cache_archive}" "${_expected_hash}" _ffmpeg_root)
    set(${out_root} "${_ffmpeg_root}" PARENT_SCOPE)
endfunction()

function(_talos_define_btbn_ffmpeg_target target_name library_name ffmpeg_root)
    set(_library_path "${ffmpeg_root}/lib/${library_name}")
    if (NOT EXISTS "${_library_path}")
        message(WARNING
            "BtbN FFmpeg was extracted to ${ffmpeg_root}, but ${_library_path} is missing; "
            "falling back to system FFmpeg.")
        set(TALOS_FFMPEG_BTBN_TARGETS_READY FALSE PARENT_SCOPE)
        return()
    endif ()

    add_library(${target_name} INTERFACE IMPORTED GLOBAL)
    set_target_properties(${target_name} PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ffmpeg_root}/include"
        INTERFACE_LINK_LIBRARIES "${_library_path}"
    )
endfunction()

function(_talos_define_btbn_ffmpeg_targets ffmpeg_root out_ready)
    set(TALOS_FFMPEG_BTBN_TARGETS_READY TRUE)

    _talos_define_btbn_ffmpeg_target(PkgConfig::AVCODEC libavcodec.so "${ffmpeg_root}")
    _talos_define_btbn_ffmpeg_target(PkgConfig::AVFORMAT libavformat.so "${ffmpeg_root}")
    _talos_define_btbn_ffmpeg_target(PkgConfig::AVFILTER libavfilter.so "${ffmpeg_root}")
    _talos_define_btbn_ffmpeg_target(PkgConfig::SWSCALE libswscale.so "${ffmpeg_root}")
    _talos_define_btbn_ffmpeg_target(PkgConfig::SWRESAMPLE libswresample.so "${ffmpeg_root}")
    _talos_define_btbn_ffmpeg_target(PkgConfig::AVUTIL libavutil.so "${ffmpeg_root}")

    set(${out_ready} "${TALOS_FFMPEG_BTBN_TARGETS_READY}" PARENT_SCOPE)
endfunction()

function(talos_configure_ffmpeg)
    set(_saved_pkg_config_path "$ENV{PKG_CONFIG_PATH}")
    set(_ffmpeg_root "")

    if (TALOS_PREFER_BTBN_FFMPEG)
        _talos_try_btbn_ffmpeg(_ffmpeg_root)
    endif ()

    if (_ffmpeg_root)
        _talos_define_btbn_ffmpeg_targets("${_ffmpeg_root}" _btbn_targets_ready)
        if (_btbn_targets_ready)
            message(STATUS "Using BtbN FFmpeg from: ${_ffmpeg_root}")
            set(TALOS_FFMPEG_SOURCE "btbn" CACHE INTERNAL "Resolved FFmpeg source")
            set(TALOS_FFMPEG_ROOT "${_ffmpeg_root}" CACHE INTERNAL "Resolved FFmpeg root directory")
            set(ENV{PKG_CONFIG_PATH} "${_saved_pkg_config_path}")
            return()
        endif ()
    endif ()

    if (_saved_pkg_config_path)
        set(ENV{PKG_CONFIG_PATH} "${_saved_pkg_config_path}")
    else ()
        unset(ENV{PKG_CONFIG_PATH})
    endif ()

    message(STATUS "Using system FFmpeg via pkg-config")
    set(TALOS_FFMPEG_SOURCE "system" CACHE INTERNAL "Resolved FFmpeg source")
    set(TALOS_FFMPEG_ROOT "" CACHE INTERNAL "Resolved FFmpeg root directory")

    pkg_check_modules(AVCODEC REQUIRED IMPORTED_TARGET libavcodec)
    pkg_check_modules(AVFORMAT REQUIRED IMPORTED_TARGET libavformat)
    pkg_check_modules(AVFILTER REQUIRED IMPORTED_TARGET libavfilter)
    pkg_check_modules(SWSCALE REQUIRED IMPORTED_TARGET libswscale)
    pkg_check_modules(SWRESAMPLE REQUIRED IMPORTED_TARGET libswresample)
    pkg_check_modules(AVUTIL REQUIRED IMPORTED_TARGET libavutil)

    if (_saved_pkg_config_path)
        set(ENV{PKG_CONFIG_PATH} "${_saved_pkg_config_path}")
    else ()
        unset(ENV{PKG_CONFIG_PATH})
    endif ()
endfunction()
