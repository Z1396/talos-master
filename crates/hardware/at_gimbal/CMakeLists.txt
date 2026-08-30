crate(
    NAME talos_gimbal
    TYPE INTERFACE
    DEPS
        talos_libusb_static
        # serial.hpp/stm32.hpp 直接 include spdlog 日志与 fmt 格式化，
        # packet.hpp 依赖 magic_enum 枚举反射。
        # 此前缺失声明：主程序因 fcs 恰好同时链接 spdlog 而侥幸编译通过，
        # 但 tests/ 下自动生成的测试目标只链接本 crate，会因缺头文件而失败。
        spdlog::spdlog
        magic_enum
)
