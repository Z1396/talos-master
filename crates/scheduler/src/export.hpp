#pragma once
// 头文件保护宏，防止该头文件被重复多次包含，避免重定义编译错误

// 调度器库导出符号控制宏注释
#ifdef SCHEDULER_EXPORTS
// 编译库本体时定义 SCHEDULER_EXPORTS，标记对外导出的API符号
# define SCHEDULER_API __attribute__((visibility("default")))
#else
// 外部项目链接该库时，不定义 SCHEDULER_EXPORTS，宏为空，无额外属性修饰
# define SCHEDULER_API
#endif