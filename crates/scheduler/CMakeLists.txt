crate(
    NAME scheduler
    TYPE SHARED
    DEPS
        primitive
        magic_enum
        talos_log
        fmt::fmt
        TBB::tbb
        nlohmann_json::nlohmann_json
)

# rclcompat 测试需要额外的 include 目录
if(TARGET rclcompat_test)
    target_include_directories(rclcompat_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
endif()
