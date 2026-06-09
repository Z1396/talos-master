# FFmpeg targets are configured centrally in cmake/dependencies.cmake.

crate(
    NAME quanta
    TYPE SHARED
    SOURCES
        src/quanta/stream_encoder.cpp
        src/quanta/stream_transport.cpp
        src/quanta/encode_backend/ffmpeg/ffmpeg_encode_backend.cpp
        src/quanta/paramset/hevc_annexb.cpp
    DEPS
        primitive
        PkgConfig::AVCODEC
        PkgConfig::AVFORMAT
        PkgConfig::AVFILTER
        PkgConfig::SWSCALE
        PkgConfig::SWRESAMPLE
        PkgConfig::AVUTIL
)
if(AXERA_FOUND)
    target_compile_definitions(quanta PUBLIC TALOS_HAS_AXERA=1)
    target_include_directories(quanta SYSTEM PUBLIC ${AXERA_INCLUDE_DIRS})
    target_sources(quanta PRIVATE
        # src/tool/ax_log_redirect.cpp
        src/quanta/encode_backend/ax/ax_encode_backend.cpp
    )
    # spdlog::spdlog is needed by ax_log_redirect.cpp.
    # Must appear BEFORE ${AXERA_LIBRARIES} so our AX_SYS_LogPrint wins at link time.
    target_link_libraries(quanta PUBLIC spdlog::spdlog ${AXERA_LIBRARIES})
    message(STATUS "Axera venc enabled (spdlog log intercept active)")
else()
    target_compile_definitions(quanta PUBLIC TALOS_HAS_AXERA=0)
    message(STATUS "Axera venc disabled")
endif()
