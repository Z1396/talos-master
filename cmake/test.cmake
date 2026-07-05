# ====================== 模块功能说明 ======================
# 本段 CMake 脚本专门管理项目单元测试编译、GTest 依赖、内存/线程检测工具(ASAN/TSAN)、代码覆盖率
# 默认关闭测试编译，加快日常业务代码构建速度，仅开发调试时手动开启
# 放在单独的 tests/CMakeLists.txt 或者主 CMakeLists.txt 测试分支
# ==========================================================

# 构建测试总开关：默认 OFF，不编译测试代码，提升编译速度
# option(变量名 "提示描述" 默认值)
option(TALOS_BUILD_TESTING "Build tests" OFF)

# 判断：如果测试总开关关闭，直接退出当前文件，不再执行下方所有测试相关逻辑
# return() 在当前 CMake 文件中终止执行，不退出整个项目构建
if(NOT TALOS_BUILD_TESTING)
    return()
endif()

# 查找 GoogleTest 单元测试框架，REQUIRED 代表找不到则直接报错终止构建
# 开启测试后必须依赖 GTest，没有测试框架无法编译用例
find_package(GTest REQUIRED)

# ---------------------- 内存/线程检测工具 可选开关 ----------------------
# 地址消毒器：检测内存越界、野指针、内存泄漏、堆缓冲区溢出
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
# 线程消毒器：检测多线程数据竞争、锁顺序颠倒、未同步共享变量
option(ENABLE_TSAN "Enable ThreadSanitizer" OFF)
# 代码覆盖率：统计单元测试覆盖了多少业务代码行数
option(ENABLE_COVERAGE "Enable code coverage" OFF)

# ASAN 和 TSAN 互斥校验：两者底层插桩逻辑冲突，不能同时开启
if(ENABLE_ASAN AND ENABLE_TSAN)
    # FATAL_ERROR：打印错误信息并立刻终止 CMake 配置流程，无法继续构建
    message(FATAL_ERROR "Cannot enable both AddressSanitizer and ThreadSanitizer simultaneously")
endif()

# 开启 ASAN 内存检测逻辑
if(ENABLE_ASAN)
    # STATUS 普通日志，告知用户当前启用 ASAN
    message(STATUS "AddressSanitizer enabled")
    # 给全局编译参数追加 ASAN 插桩 + 保留调用栈（方便定位崩溃堆栈）
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    # 链接阶段也要传入 sanitize 参数，链接器会加载 ASAN 运行时库
    add_link_options(-fsanitize=address)
endif()

# 开启 TSAN 线程竞争检测逻辑
if(ENABLE_TSAN)
    message(STATUS "ThreadSanitizer enabled")
    # TSAN 编译插桩，同样保留完整调用栈
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
    # 链接 TSAN 运行时库
    add_link_options(-fsanitize=thread)
endif()

# 开启覆盖率仅打印日志，此处未添加编译参数，一般配合 gcov/lcov 额外补充
if(ENABLE_COVERAGE)
    message(STATUS "Coverage enabled")
endif()

# CMake 内置测试系统激活命令
# 必须调用 enable_testing() 后，才能使用 add_test()、ctest 命令执行单元测试
enable_testing()