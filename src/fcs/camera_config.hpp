// 头文件保护宏，等价#ifndef + #define，防止头文件被多次重复包含
#pragma once

// Eigen线性代数库，提供矩阵、矩阵类型（相机内参矩阵、畸变系数）
#include <Eigen/Core>
// 固定宽度整数类型 uint32_t
#include <cstdint>
// fmt格式化库，用于日志打印、对象格式化输出
#include <fmt/core.h>
// std::optional：可选值类型，可以表达「参数存在 / 参数缺失」两种状态
#include <optional>
// 标准字符串
#include <string>

// magic_enum：编译期枚举反射库，可以不用宏，直接获取枚举名称字符串
#include <magic_enum.hpp>

// 海康相机驱动相关类型定义（相机SDK封装层）
#include "hik_camera.hpp"

// fcs框架顶层命名空间，所有组件收拢在此，避免全局命名冲突
namespace fcs {

// 从海康相机驱动命名空间导入旋转枚举类型，简化书写
using hikcamera::RotateType;

/**
 * @brief 相机工作参数配置（相机硬件属性、曝光、增益、触发模式）
 * 对应TOML配置里相机硬件控制相关参数
 */
struct CameraProfileConfig {
    // 相机是否开启硬触发模式：false 连续采集；true 外部信号触发拍照
    bool trigger_mode{false};
    // 是否图像镜像翻转
    bool invert_image{false};
    // 曝光时间，单位：微秒，默认3000us = 3ms
    uint32_t exposure_time_us{3000};
    // 模拟增益
    double gain{16.7};
    // 图像旋转方式，默认不旋转（RotateType::None）
    RotateType rotate_angle{RotateType::None};
    /**
     * 相机设备名称（可选）
     * std::optional含义：
     * TOML可以不填写此字段，代表自动搜索第一个可用相机；
     * 如果填写字符串，则根据名称精准打开指定相机
     */
    std::optional<std::string> device_name{};
};

/**
 * @brief 相机完整配置结构体
 * 分为两大块：标定参数（几何） + 相机硬件参数（采集配置）
 */
struct CameraConfig {
    /**
     * 相机内参矩阵 3×3
     * Eigen::RowMajor：行优先存储（和OpenCV数据布局对齐，方便直接转换）
     */
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix{};
    /**
     * 畸变系数：5参数模型 k1,k2,p1,p2,k3
     * 顺序与OpenCV标准distCoeff保持一致
     */
    Eigen::Matrix<double, 1, 5> distort_coefficient{};
    // 图像分辨率宽度 默认1440
    uint32_t width{1440};
    // 图像分辨率高度 默认1080
    uint32_t height{1080};
    // 子结构体：相机采集硬件参数
    CameraProfileConfig profile{};
};

} // namespace fcs

// ============================================================================
// fmt::formatter specializations 格式化特化
// 作用：让fmt库可以直接打印枚举 RotateType，不用手动转字符串
// ============================================================================
namespace fmt {

// 对枚举 RotateType 进行formatter模板特化
template <>
struct formatter<fcs::RotateType> : formatter<std::string_view> {
    // 格式化函数：把枚举值转换成名字字符串，输出到日志
    auto format(fcs::RotateType type, format_context& ctx) const {
        // magic_enum::enum_name：编译期获取枚举字面名称 "None" / "Rotate90"等
        return formatter<std::string_view>::format(magic_enum::enum_name(type), ctx);
    }
};

} // namespace fmt