// 引入变换校验函数声明头文件（包含 validate_transform / format_transform_values 模板函数原型）
#include "validation.hpp"
// 引入坐标系帧标识、变换矩阵 TransformMatrix 模板定义头文件
#include "frame.hpp"

// 快速坐标变换库顶层命名空间 fast_tf
namespace fast_tf {

// ============================================================================
// 注释说明：对项目中所有坐标系之间的刚性变换模板函数进行显式实例化
// C++ 模板默认仅在被调用处隐式实例化；此处手动强制实例化所有机器人用到的坐标变换组合
// 作用：1. 把模板代码编译进当前编译单元，避免跨链接缺失模板实现的 undefined reference 链接报错
//      2. 提前实例化全部固定坐标系变换，运行时无模板实例化开销，提升实时性
// ============================================================================

// ===================== world_fuxk_frame 世界坐标系 → odom_fuxk_frame 里程计底盘坐标系 =====================
// 显式实例化模板函数：校验世界到里程计变换矩阵合法性
template std::expected<void, std::string>
    validate_transform(const TransformMatrix<double, world_fuxk_frame, odom_fuxk_frame>&) noexcept;
// 显式实例化模板函数：格式化输出世界到里程计变换矩阵数值（平移+旋转）
template std::string format_transform_values(
    const TransformMatrix<double, world_fuxk_frame, odom_fuxk_frame>&) noexcept;

// ===================== odom_fuxk_frame 里程计底盘坐标系 → gimbal_yaw_fuxk_frame 云台偏航坐标系 =====================
template std::expected<void, std::string> validate_transform(
    const TransformMatrix<double, odom_fuxk_frame, gimbal_yaw_fuxk_frame>&) noexcept;
template std::string format_transform_values(
    const TransformMatrix<double, odom_fuxk_frame, gimbal_yaw_fuxk_frame>&) noexcept;

// ===================== gimbal_yaw_fuxk_frame 云台偏航坐标系 → gimbal_pitch_fuxk_frame 云台俯仰坐标系 =====================
template std::expected<void, std::string> validate_transform(
    const TransformMatrix<double, gimbal_yaw_fuxk_frame, gimbal_pitch_fuxk_frame>&) noexcept;
template std::string format_transform_values(
    const TransformMatrix<double, gimbal_yaw_fuxk_frame, gimbal_pitch_fuxk_frame>&) noexcept;

// ===================== gimbal_pitch_fuxk_frame 云台俯仰坐标系 → camera_link_fuxk_frame 相机机械安装坐标系 =====================
template std::expected<void, std::string> validate_transform(
    const TransformMatrix<double, gimbal_pitch_fuxk_frame, camera_link_fuxk_frame>&) noexcept;
template std::string format_transform_values(
    const TransformMatrix<double, gimbal_pitch_fuxk_frame, camera_link_fuxk_frame>&) noexcept;

// ===================== camera_link_fuxk_frame 相机机械坐标系 → camera_optical_fuxk_frame 相机光学像素坐标系 =====================
template std::expected<void, std::string> validate_transform(
    const TransformMatrix<double, camera_link_fuxk_frame, camera_optical_fuxk_frame>&) noexcept;
template std::string format_transform_values(
    const TransformMatrix<double, camera_link_fuxk_frame, camera_optical_fuxk_frame>&) noexcept;

// ===================== gimbal_pitch_fuxk_frame 云台俯仰坐标系 → muzzle_link_fuxk_frame 枪口坐标系 =====================
template std::expected<void, std::string> validate_transform(
    const TransformMatrix<double, gimbal_pitch_fuxk_frame, muzzle_link_fuxk_frame>&) noexcept;
template std::string format_transform_values(
    const TransformMatrix<double, gimbal_pitch_fuxk_frame, muzzle_link_fuxk_frame>&) noexcept;

} // namespace fast_tf