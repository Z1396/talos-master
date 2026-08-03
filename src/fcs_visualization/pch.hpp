#pragma once
// 防止头文件被多次重复包含，等价传统 #ifndef #define #endif 头文件保护
// 现代C++主流写法，简洁高效，主流编译器（GCC/Clang/MSVC）全部支持

#include <array>
// STL固定大小数组容器 std::array<T, N>
// 相比C原生数组安全、支持迭代器、可作为STL算法参数，相机标定固定数组（畸变系数5个）会用到

#include <cstdint>
// 固定宽度整数类型头文件：uint64_t、uint32_t、int64_t 等
// 跨平台统一整型宽度，避免不同系统long/int长度不一致，时间戳、帧序号强制用定长类型必备

#include <expected>
// C++23 标准模板 std::expected<T, E>
// 用来做「正常返回值 / 错误信息」二选一，替代错误码+全局errno、异常try-catch
// 你上一份cpp相机代码里 recv() -> std::expected<Frame, InputError> 就是在用它，无异常开销，嵌入式友好

#include <string>
// std::string 字符串，存储相机设备名称、错误描述文本

#include <variant>
// C++17 std::variant<Type1, Type2...> 类型联合体（安全版union）
// 本相机模块用途：InputMode = std::variant<IpcInput, HikInput>
// 实现无虚函数多态，不用基类+虚表，运行性能更高，Jetson嵌入式环境常用优化方案

#include <spdlog/common.h>
#include <spdlog/spdlog.h>
// spdlog 高性能日志库
// SPDLOG_INFO / SPDLOG_ERROR 打印相机断开、构造析构日志，比赛调试、赛后复盘排查相机掉线问题使用

#include <Eigen/Core>
#include <Eigen/Geometry>
// Eigen 线性代数库，机器人视觉必备
// Eigen/Core：矩阵、向量基础，相机内参矩阵 camera_matrix（3×3矩阵）
// Eigen/Geometry：旋转矩阵、四元数、欧拉角、位姿变换，PnP解算、手眼标定、云台坐标转换全部依赖它

#include <fmt/core.h>
#include <fmt/format.h>
// fmt 格式化库，C++20 std::format 的前身，比原生printf、stringstream安全好用
// 用途：拼接日志字符串、格式化打印相机内参、报错信息，和spdlog搭配使用

#include <magic_enum.hpp>
// 枚举反射开源库，无需宏即可获取枚举名字符串
// 例如相机触发模式枚举 TriggerMode::Soft，直接转字符串"Soft"打印日志，不用手写枚举映射表，简化调试

#include <nlohmann/json.hpp>
// json解析库，业界俗称json.hpp
// 读取相机配置JSON文件：曝光、增益、内参、相机模式(Ipc/Hik)配置持久化，不用手写toml/xml解析器

#include <opencv2/core/types.hpp>
// OpenCV 基础类型定义，不含imread、cvtColor等函数，只导入结构体
// cv::Size、cv::Rect、cv::Mat基础声明、cv::Point等，Frame结构体存放cv::Mat图像前置依赖