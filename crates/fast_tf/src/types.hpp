#pragma once

#include <cstdint>

/// Timestamp in nanoseconds
// 类型别名：纳秒时间戳，用uint64无符号64位整数存储
// uint64_t最大可以存接近184亿秒，纳秒单位足够存几百年时间，不会溢出
using timestamp_ns_t = std::uint64_t;

/// Floating point type used throughout fast_tf
// fast‑tf整个库全局使用的浮点数类型
// fp_t 是 type alias，现在别名绑定 double（双精度64位浮点数）
// 如果以后想整个库改成float单精度，只需要改这里一行，全部代码自动切换
using fp_t = double;
