# 编译开关：是否启用相机标定工具 rm_calibration，默认关闭不编译
# option(参数名 "说明文本" 默认值)
option(ENABLE_CALIBRATION "Enable rm_calibration" OFF)

# 判断：只有用户手动开启标定模块才执行下方所有编译、依赖、链接逻辑
if (ENABLE_CALIBRATION)
    # 控制台打印日志，告知用户标定模块已启用
    message(STATUS "Camera calibration module enabled")

    # 1. 强制查找 OpenCV，必须包含 calib3d 标定模块（棋盘格、手眼标定核心算法）
    # REQUIRED：找不到 OpenCV / calib3d 模块直接终止CMake配置，报错退出
    find_package(OpenCV REQUIRED COMPONENTS calib3d)

    # 2. 静默查找 OpenCV aruco 模块（用于ChArUco标定板，可选依赖）
    # QUIET：找不到aruco不会抛致命错误，仅静默返回未找到状态
    find_package(OpenCV QUIET COMPONENTS aruco)
    if(OpenCV_aruco_FOUND)
        # 找到aruco模块，开启ChArUco码支持
        message(STATUS "OpenCV aruco module found - ChArUco support enabled")
        set(HAVE_OPENCV_ARUCO ON) # 自定义标记变量，供后续编译宏/链接判断使用
    else()
        # 无aruco模块，禁用ChArUco相关代码分支
        message(STATUS "OpenCV aruco module not found - ChArUco support disabled")
        set(HAVE_OPENCV_ARUCO OFF)
    endif()

    # 3. 生成可执行程序 rm_calibration（相机标定工具主程序）
    # 填入该工具全部cpp源码文件
    add_executable(rm_calibration
        src/fcs/calibration/rm_calibration_main.cpp    # 程序入口main函数
        src/fcs/calibration/chessboard_detector.cpp    # 普通棋盘格识别
        src/fcs/calibration/charuco_detector.cpp       # ChArUco标定板识别
        src/fcs/calibration/intrinsic_calibrator.cpp   # 相机内参标定
        src/fcs/calibration/handeye_calibrator.cpp      # 手眼标定（相机-机械臂外参）
    )

    # 4. 给标定程序添加 foxglove 可视化库头文件搜索路径
    # SYSTEM：标记为系统头文件，编译器会屏蔽该目录下的警告
    # PRIVATE：仅当前rm_calibration目标可见，不会传递给其他依赖本程序的库
    target_include_directories(rm_calibration SYSTEM PRIVATE
        ${foxglove_SOURCE_DIR}/include
        ${foxglove_SOURCE_DIR}/include/foxglove
    )

    # 5. 批量链接所有依赖库（PRIVATE：依赖仅作用于当前可执行文件）
    target_link_libraries(rm_calibration PRIVATE
        fcs                     # 项目底层基础框架库
        fcs_visualization       # 项目可视化通用组件
        fast_tf                 # 坐标变换TF库
        talos_log               # 项目日志封装
        toml_std                # TOML配置文件解析库
        spdlog::spdlog          # 高性能日志第三方库
        fmt::fmt                # 格式化字符串库
        ${OpenCV_LIBS}          # 全部OpenCV找到的库（包含calib3d）
        Eigen3::Eigen           # 线性代数矩阵计算（标定矩阵运算核心）
        talos_libusb_static     # USB静态库，相机USB通信
        foxglove_cpp            # Foxglove可视化客户端封装
        ${foxglove_SOURCE_DIR}/lib/libfoxglove.a # Foxglove静态库文件

        # macOS 平台专属链接条件编译，$<PLATFORM_ID:Darwin> 生成器表达式
        # 仅Mac系统编译时追加系统框架
        "$<$<PLATFORM_ID:Darwin>:-framework Security>"
        "$<$<PLATFORM_ID:Darwin>:-framework CoreFoundation>"
        "$<$<PLATFORM_ID:Darwin>:-framework SystemConfiguration>"
        # Mac最低系统版本限制
        "$<$<PLATFORM_ID:Darwin>:-mmacosx-version-min=15.5>"
    )

    # 6. 如果检测到aruco模块，追加链接+代码宏定义
    if(HAVE_OPENCV_ARUCO)
        target_link_libraries(rm_calibration PRIVATE opencv_aruco)
        # 定义编译宏 HAVE_OPENCV_ARUCO，源码中可用 #ifdef 区分有无ChArUco
        target_compile_definitions(rm_calibration PRIVATE HAVE_OPENCV_ARUCO)
    endif()

    # 7. 指定编译C++23标准，该工具依赖std::expected等C++23新特性
    target_compile_features(rm_calibration PRIVATE cxx_std_23)

    # 8. 设置程序输出路径：编译完成后二进制文件放到 build/bin 目录
    set_target_properties(rm_calibration PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )

    # 打印完成提示日志
    message(STATUS "rm_calibration target configured")
endif()