// 头文件保护，防止头文件被多次重复包含
#pragma once

// 海康相机库动态库导出宏定义
#ifdef HIKCAMERA_EXPORTS
// 编译当前动态库(.so)时定义该宏，将函数/类标记为对外可见
# define HIKCAMERA_API __attribute__((visibility("default")))
#else
// 外部项目链接该动态库时，宏为空，不添加任何属性
# define HIKCAMERA_API
#endif