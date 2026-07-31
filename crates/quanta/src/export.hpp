#pragma once

/**
 * @file export.hpp
 * @brief Quanta库导出宏定义
 *
 * 定义跨平台动态库导出/导入宏，用于控制符号的可见性。
 * 这个宏确保Quanta库的公共API可以被外部项目正确链接和使用。
 *
 * ## 动态库符号可见性
 *
 * 默认情况下，动态库（.so/.dll）的符号对调用者不可见。
 * 需要通过特殊属性显式标记哪些符号应该导出（对外可见）。
 *
 * ## 平台差异
 *
 * | 平台 | 导出属性 | 导入属性 |
 * |------|---------|---------|
 * | Linux | `__attribute__((visibility("default")))` | 无需标记 |
 * | Windows | `__declspec(dllexport)` | `__declspec(dllimport)` |
 * | macOS | `__attribute__((visibility("default")))` | 无需标记 |
 *
 * ## 使用方法
 *
 * ```cpp
 * // 在库源文件中（编译quanta库时）
 * #define QUANTA_EXPORTS  // CMakeLists.txt中定义
 * #include "quanta/export.hpp"
 *
 * class QUANTA_API MyClass {
 *     // 此类对外可见
 * };
 *
 * QUANTA_API void my_function();
 * // 此函数对外可见
 * ```
 *
 * ```cpp
 * // 在外部项目中（使用quanta库时）
 * #include "quanta/export.hpp"
 *
 * // QUANTA_EXPORTS未定义，QUANTA_API为空
 * // 编译器根据动态库导入规则自动处理
 * MyClass obj;  // 正确链接到库中的符号
 * ```
 *
 * ## CMake配置
 *
 * ```cmake
 * # 库编译目标
 * add_library(quanta SHARED ...)
 * target_compile_definitions(quanta PRIVATE QUANTA_EXPORTS)  # 定义导出宏
 *
 * # 使用库的项目
 * target_link_libraries(my_app PRIVATE quanta)  # 自动处理导入
 * ```
 *
 * ## Linux符号可见性详解
 *
 * ### 默认可见性
 * - **hidden**：符号仅在库内部可见，外部无法链接
 * - **default**：符号对外可见，可被动态链接器解析
 *
 * ### 性能影响
 * - **导出符号少**：减少动态链接时间、降低二进制体积
 * - **导出符号多**：增加兼容性风险、延长加载时间
 *
 * ### 最佳实践
 * - 仅导出公共API（类、函数）
 * - 内部实现细节使用hidden（默认）
 * - 使用PIMPL模式隐藏实现细节
 *
 * ## Windows特殊处理
 *
 * Windows平台需要区分导出和导入：
 * - **编译库时**：使用`__declspec(dllexport)`
 * - **使用库时**：使用`__declspec(dllimport)`
 *
 * 这是通过QUANTA_EXPORTS宏控制的：
 * - **定义时**：编译库，使用导出属性
 * - **未定义时**：使用库，使用导入属性（Windows）或无需标记（Linux）
 *
 * @note Linux/macOS使用visibility属性，Windows使用declspec属性
 * @note QUANTA_API在导出时为visibility("default")，导入时为空
 */

/**
 * @brief Quanta库API导出/导入宏
 *
 * 根据QUANTA_EXPORTS宏定义自动选择导出或导入属性：
 *
 * - **QUANTA_EXPORTS已定义**（编译quanta库时）：
 *   - Linux/macOS：`__attribute__((visibility("default")))`
 *   - Windows：`__declspec(dllexport)`
 *   - 效果：符号导出，对外可见
 *
 * - **QUANTA_EXPORTS未定义**（使用quanta库时）：
 *   - Linux/macOS：空宏（无需标记）
 *   - Windows：`__declspec(dllimport)`（需要显式导入）
 *   - 效果：符号导入，链接到库
 *
 * ## 使用示例
 *
 * ```cpp
 * // 导出类
 * class QUANTA_API AxSysModule {
 * public:
 *     void public_method();  // 对外可见
 * private:
 *     void internal_method();  // 仅内部可见（即使类被导出）
 * };
 *
 * // 导出函数
 * QUANTA_API std::expected<AxSysModule, std::string> create_module();
 *
 * // 导出变量（不推荐，建议使用函数访问）
 * QUANTA_API extern const int kConstantValue;
 * ```
 *
 * ## 常见错误
 *
 * 1. **忘记标记QUANTA_API**：链接错误（undefined symbol）
 * 2. **标记内部函数**：增加导出表体积、暴露实现细节
 * 3. **Windows未定义QUANTA_EXPORTS**：链接错误或运行时崩溃
 *
 * @see CMakeLists.txt QUANTA_EXPORTS宏定义位置
 */
#ifdef QUANTA_EXPORTS
// 编译quanta库：导出符号
# define QUANTA_API __attribute__((visibility("default")))
#else
// 使用quanta库：导入符号（Linux/macOS无需标记）
# define QUANTA_API
#endif