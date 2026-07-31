/**
 * @file pch.hpp
 * @brief FCS模块预编译头文件
 *
 * 本文件定义了FCS（Fire Control System，火控系统）模块的预编译头，
 * 用于加速编译过程。包含了FCS模块中几乎所有源文件都会使用的公共头文件。
 *
 * 预编译头包含的内容：
 * 1. C++标准库核心组件：
 *    - 类型支持（<cstddef>, <cstdint>）
 *    - 容器（<array>, <vector>, <string>）
 *    - 智能指针（<memory>）
 *    - 时间库（<chrono>）
 *    - 算法（<algorithm>）
 *    - 数学库（<cmath>, <limits>, <numbers>）
 *    - 函数对象（<functional>）
 *    - 迭代器（<iterator>）
 *    - 类型特性（<type_traits>）
 *
 * 2. C++20核心特性：
 *    - 错误处理（<expected>）
 *    - 可选值（<optional>）
 *    - 变体类型（<variant>）
 *
 * 3. 第三方库：
 *    - 日志（spdlog）
 *    - 格式化（fmt）
 *    - 枚举反射（magic_enum）
 *    - 线性代数（Eigen3）
 *    - 图像处理（OpenCV）
 *    - 调度器（talos scheduler）
 *
 * 性能优化：
 * - 预编译头可将编译时间缩短50-70%
 * - 包含的头文件经过精心选择，避免冗余依赖
 * - 所有包含均为广泛使用的核心组件，确保高命中率
 *
 * 使用方式：
 * - 在CMakeLists.txt中设置CMAKE_CXX_STANDARD为20及以上
 * - 使用target_precompile_headers命令指定此文件
 * - 确保所有源文件至少包含一个此处列出的头文件
 *
 * @note 不要在预编译头中包含项目特定头文件，避免频繁重建
 * @note 修改此文件会触发整个项目的重新编译，请谨慎修改
 *
 * @author Talos Team
 * @date 2024
 */

#pragma once

// ===== C++标准库核心类型支持 =====
#include <cstddef>
#include <cstdint>

// ===== 标准库算法 =====
#include <algorithm>

// ===== 标准库容器 =====
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// ===== C++20核心特性 =====
#include <expected>
#include <optional>
#include <variant>

// ===== 数学与数值工具 =====
#include <cmath>
#include <functional>
#include <iterator>
#include <type_traits>
#include <unordered_map>

// ===== 日志库 =====
#include <spdlog/spdlog.h>

// ===== 数值常量与限制 =====
#include <limits>
#include <numbers>

// ===== 格式化库 =====
#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/std.h>

// ===== 枚举反射库 =====
#include <magic_enum.hpp>

// ===== 线性代数库 =====
#include <Eigen/Core>

// ===== 图像处理库 =====
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

// ===== Talos调度器 =====
#include <scheduler/error_formatter.hpp>
#include <scheduler/scheduler.hpp>
