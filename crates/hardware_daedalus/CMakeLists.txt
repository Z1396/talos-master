crate(
    NAME ipc
    TYPE INTERFACE
    DEPS
        opencv_core
        magic_enum
        fmt::fmt
)

# ============================================================================
# talos_ipc_bridge：跨机仿真网络桥可执行程序
# 场景：Rust 仿真跑 Windows、Talos 跑 Linux，网线组局域网
# 职责：UDP/TCP 收仿真网络包 → 写入本机 /tmp 共享内存（Talos 主程序零改动）
# 依赖：ipc 接口库（shm_client/shm_layout/net_protocol）+ OpenCV 解码 + POSIX 线程
# ============================================================================
add_executable(talos_ipc_bridge
    src/ipc_bridge_main.cpp
)
target_include_directories(talos_ipc_bridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(talos_ipc_bridge PRIVATE
    ipc
    opencv_core
    opencv_imgcodecs
    fmt::fmt
    magic_enum
    pthread
)

# 注意：tests/ 下的 *_test.cpp 由 crate() 的 _crate_add_tests() 自动扫描并
# 创建同名可执行目标 + CTest 条目（见 cmake/crate-utils.cmake），
# 因此这里不再显式 add_executable/add_test，避免目标重名导致 CMake 配置失败。
