option(ENABLE_CALIBRATION "Enable rm_calibration" OFF)

if (ENABLE_CALIBRATION)
    message(STATUS "Camera calibration module enabled")

    # OpenCV aruco module is required for ChArUco detection
    find_package(OpenCV REQUIRED COMPONENTS calib3d)

    # Check if aruco module is available (optional)
    find_package(OpenCV QUIET COMPONENTS aruco)
    if(OpenCV_aruco_FOUND)
        message(STATUS "OpenCV aruco module found - ChArUco support enabled")
        set(HAVE_OPENCV_ARUCO ON)
    else()
        message(STATUS "OpenCV aruco module not found - ChArUco support disabled")
        set(HAVE_OPENCV_ARUCO OFF)
    endif()

    add_executable(rm_calibration
    src/fcs/calibration/rm_calibration_main.cpp
    src/fcs/calibration/chessboard_detector.cpp
    src/fcs/calibration/charuco_detector.cpp
    src/fcs/calibration/intrinsic_calibrator.cpp
    src/fcs/calibration/handeye_calibrator.cpp
    )

    # Add foxglove headers
    target_include_directories(rm_calibration SYSTEM PRIVATE
    ${foxglove_SOURCE_DIR}/include
    ${foxglove_SOURCE_DIR}/include/foxglove
)

    target_link_libraries(rm_calibration PRIVATE
    fcs
    fcs_visualization
    fast_tf
    talos_log
    toml_std
    spdlog::spdlog
    fmt::fmt
    ${OpenCV_LIBS}
    Eigen3::Eigen
    talos_libusb_static
    foxglove_cpp
    ${foxglove_SOURCE_DIR}/lib/libfoxglove.a
    "$<$<PLATFORM_ID:Darwin>:-framework Security>"
    "$<$<PLATFORM_ID:Darwin>:-framework CoreFoundation>"
    "$<$<PLATFORM_ID:Darwin>:-framework SystemConfiguration>"
    "$<$<PLATFORM_ID:Darwin>:-mmacosx-version-min=15.5>"
)

    # Link aruco if available
    if(HAVE_OPENCV_ARUCO)
        target_link_libraries(rm_calibration PRIVATE opencv_aruco)
        target_compile_definitions(rm_calibration PRIVATE HAVE_OPENCV_ARUCO)
    endif()

    # C++23 required for std::expected
    target_compile_features(rm_calibration PRIVATE cxx_std_23)

    set_target_properties(rm_calibration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

    message(STATUS "rm_calibration target configured")
endif()
