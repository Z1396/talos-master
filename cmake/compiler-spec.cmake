# 定义编译选项：是否启用 sccache 编译缓存，默认开启 ON
# sccache：编译加速工具，缓存编译产物，重复编译大幅提速
option(ENABLE_SCCACHE "Enable compiler caching via sccache" ON)

# 判断用户是否开启 sccache 功能
if (ENABLE_SCCACHE)
    # find_program：在系统PATH中查找 sccache 可执行程序，结果存入 SCCACHE_PROGRAM
    find_program(SCCACHE_PROGRAM sccache)

    # 如果成功找到 sccache 程序
    if (SCCACHE_PROGRAM)
        # 打印状态日志，输出 sccache 可执行文件路径
        message(STATUS "Sccache found at: ${SCCACHE_PROGRAM}")
        # CMAKE_C_COMPILER_LAUNCHER：C编译器启动器，前置包装编译器（这里用sccache包装gcc/clang）
        # 仅当用户未手动指定编译器启动器时，才自动赋值，避免覆盖用户自定义配置
        if (NOT CMAKE_C_COMPILER_LAUNCHER)
            set(CMAKE_C_COMPILER_LAUNCHER ${SCCACHE_PROGRAM})
        endif ()
        # 同理配置 C++ 编译器启动器
        if (NOT CMAKE_CXX_COMPILER_LAUNCHER)
            set(CMAKE_CXX_COMPILER_LAUNCHER ${SCCACHE_PROGRAM})
        endif ()
    else ()
        # 系统未安装 sccache，输出提示，不启用缓存正常编译
        message(STATUS "sccache not found, building without compiler cache")
    endif ()
else ()
    # 用户主动关闭 ENABLE_SCCACHE 选项，打印提示
    message(STATUS "sccache disabled")
endif ()

# 全局添加编译参数：生成位置无关代码（Position-Independent Code）
# 作用：用于动态库 .so / 共享库编译，否则链接动态库会报错
add_compile_options(-fPIC)

# ====================== 通用编译优化参数（GCC / Clang 双编译器兼容） ======================
# 注释说明：
# -march=native / -mtune=native：针对本机CPU架构优化指令集，仅本地编译可用
# 交叉编译场景（编译ARM/aarch64开发板固件）不能使用，否则会生成仅主机能跑的二进制，开发板无法运行
if (CMAKE_CROSSCOMPILING)
    # CMAKE_CROSSCOMPILING 为 ON：当前是交叉编译环境，目标架构 aarch64/arm64 Cortex-A55
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        # 指定目标CPU为 Cortex-A55，开启对应ARMv8.2-A指令集、NEON浮点、LSE原子、dotprod向量扩展
        add_compile_options(
            -mcpu=cortex-a55
            # 禁用原子操作 outline 分离优化，减少分支跳转，提升原子性能
            -mno-outline-atomics
            # 浮点收缩运算快速模式：允许编译器合并连续浮点乘加，牺牲极小浮点精度换取性能
            -ffp-contract=fast
        )
        # 链接阶段参数：--gc-sections 丢弃未使用的代码段/数据段，减小最终可执行文件体积
        add_link_options(-Wl,--gc-sections)
    endif ()
else ()
    # 本地本机编译，根据当前CPU自动开启全部原生指令集优化
    add_compile_options(
        -march=native
        -mtune=native
    )
endif ()

# 全局通用优化编译参数，所有编译目标生效
add_compile_options(
    # 循环展开：编译器自动展开短循环，减少循环跳转开销，提升CPU流水线利用率
    -funroll-loops
    # 开启自动向量化优化（基础向量优化）
    -fvectorize
    # 开启SLP向量化：多语句并行向量化，连续标量运算打包SIMD指令
    -fslp-vectorize
    # 关闭数学库errno全局变量同步，浮点运算更快，不依赖C标准库错误标记
    -fno-math-errno
    # 保留栈帧指针（frame pointer），方便gdb性能采样、栈回溯调试；生产环境可关闭换性能
    -fno-omit-frame-pointer
    # 禁用PLT过程链接表，减少动态链接跳转开销，适合静态链接/内部库
    -fno-plt

    # 函数/数据分段存放，配合链接器 --gc-sections 自动删除未使用代码
    -fdata-sections
    -ffunction-sections
)

# ====================== 全局严格警告检测（GCC/Clang 通用） ======================
# 所有警告全部升级为编译错误，强制规范代码，杜绝隐性bug
add_compile_options(
    # 基础警告：开启绝大多数常规代码隐患警告
    -Wall
    # 扩展警告：补充更多边界、类型、逻辑警告
    -Wextra
    # 严格遵循C/C++标准，禁止非标准扩展语法
    -Wpedantic
    # 标准违规直接报错，不允许忽略
    -Werror=pedantic

    # 枚举隐式类型转换报错（enum转int/反向转换）
    -Werror=enum-conversion
    # -Wswitch-default：可选，强制switch覆盖全部enum分支，这里注释关闭
    # case无break隐式穿透直接报错
    -Werror=implicit-fallthrough

    # 缩进误导代码逻辑报错
    -Werror=misleading-indentation
    # 带虚函数的类无虚析构函数报错（内存泄漏根源）
    -Werror=non-virtual-dtor
    # 父类虚函数重写时参数/签名不匹配报错
    -Werror=overloaded-virtual
    # 重写虚函数必须加 override 关键字，漏写直接报错
    -Werror=suggest-override

    # 空指针解引用静态检测报错
    -Werror=null-dereference
    # 数组越界静态检查报错
    -Werror=array-bounds
    # 禁止可变长度数组VLA（C99特性，C++不标准，栈溢出风险）
    -Werror=vla
    # 禁止alloca栈动态分配（栈溢出高危）
    -Werror=alloca
    # 禁止指针算术运算（裸指针偏移易越界）
    -Werror=pointer-arith
    # printf/scanf格式化字符串完整检测等级2
    -Wformat=2
    # 格式化字符串错误直接报错
    -Werror=format
    # 防止格式化字符串注入漏洞（安全编译选项）
    -Werror=format-security
    # 指针类型转换对齐不匹配报错（ARM架构极易触发崩溃）
    -Werror=cast-align
    # const/volatile限定符转换丢失报错
    -Werror=cast-qual
    # 重复声明变量/函数报错
    -Werror=redundant-decls
    # 使用未定义宏时报错
    -Werror=undef

    # 函数必须所有分支有return返回值
    -Werror=return-type
    # 局部变量未初始化就读取直接报错
    -Werror=uninitialized
)

# ====================== Clang 编译器专属严格警告 ======================
# 判断当前编译器是否为 Clang / AppleClang
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        # bool和其他数值隐式转换报错
        -Werror=bool-conversion
        # 字符串字面量 + int 非法拼接报错（如 "abc" + 1）
        -Werror=string-plus-int

        # 头文件卫生检测：防止循环头文件、缺失头文件依赖
        -Werror=header-hygiene
        # 移动语义使用错误检测
        -Werror=move
        # 枚举赋值不匹配类型报错
        -Werror=assign-enum
        # 函数指针强制转换类型不匹配报错
        -Werror=bad-function-cast

        # C++11范围for循环隐患检测（迭代器失效、拷贝开销）
        -Wrange-loop-analysis
        # 线程安全静态分析（需配合thread_safety注解）
        -Wthread-safety
        # 完整线程竞争、数据竞争静态检查
        -Wthread-safety-analysis
    )
endif()