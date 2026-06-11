// 引入本模块对应的头文件，里面定义了 BuildInfo 结构体
#include "runtime/build_info.hpp"

// =============================================================================
// 预处理宏兜底定义：防止外部未传宏时编译报错
// 语法：#ifndef 宏名 → 如果这个宏没有被定义，就执行下面 #define
// 作用：编译脚本/CI/CMake 会在编译时动态注入这些宏；没注入就默认填 "unknown"
// =============================================================================

// 编译日期：如果外部未定义 TALOS_BUILD_INFO_DATE，默认字符串 "unknown"
#ifndef TALOS_BUILD_INFO_DATE
# define TALOS_BUILD_INFO_DATE "unknown"
#endif

// Git 提交哈希(commit id)：代码版本标识
#ifndef TALOS_BUILD_INFO_GIT_COMMIT
# define TALOS_BUILD_INFO_GIT_COMMIT "unknown"
#endif

// 编译主机名/编译机器标识（在哪台机器上编译的程序）
#ifndef TALOS_BUILD_INFO_HOST
# define TALOS_BUILD_INFO_HOST "unknown"
#endif

// Git 分支名（当前代码所在分支，如 main / dev / feature）
#ifndef TALOS_BUILD_INFO_GIT_BRANCH
# define TALOS_BUILD_INFO_GIT_BRANCH "unknown"
#endif

// 命名空间 fcs：项目模块隔离，避免全局命名冲突
namespace fcs {

/**
 * @brief 获取程序编译信息接口
 * @return BuildInfo 结构体，包含编译时间、编译机器、Git 版本等信息
 * 
 * 关键字说明：
 * 1. [[nodiscard]] 属性：C++17 标准属性
 *    强制要求调用方必须接收返回值；如果调用后丢弃返回值，编译器直接报警告/错误
 *    防止有人误写：build_info(); 白白调用不取值
 * 
 * 2. auto 返回值推导：C++14 及以上
 *    自动推导返回类型为 BuildInfo，不用重复写类型名
 * 
 * 3. noexcept：声明函数**不会抛出异常**
 *    编译器可做异常、性能优化；内部抛异常会直接终止程序
 * 
 * 4. -> BuildInfo：后置返回类型写法（函数返回类型后置声明）
 */
[[nodiscard]] auto build_info() noexcept -> BuildInfo {
    // C++17 聚合体 列表初始化 / 字段名初始化（指定成员赋值）
    // 按字段名给 BuildInfo 结构体每个成员赋值，可读性极强，顺序无关
    return BuildInfo{
        .build_date  = TALOS_BUILD_INFO_DATE,    // 编译日期
        .build_host  = TALOS_BUILD_INFO_HOST,     // 编译主机
        .git_commit  = TALOS_BUILD_INFO_GIT_COMMIT,// Git 提交号
        .git_branch  = TALOS_BUILD_INFO_GIT_BRANCH // Git 分支名
    };
}

} // namespace fcs