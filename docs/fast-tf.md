# fast-tf 快速参考

类型安全坐标变换 - `Fuck` types, 坐标系声明, `lookup`/`update`

## 30秒速查

| 我想... | 代码 |
|---------|------|
| **声明根坐标系** | `DECL_ROOT(map);` |
| **声明子坐标系** | `DECL(base_link, map);` |
| **创建变换 (四元数+平移)** | `TransformMatrix4d::from_quaternion_xyz(q, x, y, z)` |
| **创建变换 (欧拉角)** | `TransformMatrix4d::from_rpy(r, p, y, x, y, z)` |
| **创建变换 (纯平移)** | `TransformMatrix4d::from_translation(x, y, z)` |
| **创建变换 (旋转矩阵+平移)** | `TransformMatrix4d::from_rt(R, t)` |
| **创建变换 (PnP 结果)** | `TransformMatrix4d::from_pnp(rvec, tvec)` |
| **获取平移** | `tf->translation()` |
| **获取旋转** | `tf->rotation()` / `tf->quaternion()` |
| **求逆** | `tf.inverse()` |
| **变换乘法** | `tf1 * tf2` |
| **重父化** | `T_child.reparent_to(T_parent_new)` |
| **更新 TF** | `update(buffer, FuckedTransform<Frame>(...), time_ns);` |
| **查找 TF** | `lookup<ToFrame>(buffer, FuckFrom<From>(), time_ns);` |
| **查找 TF (超时不报错)** | `lookup_clamped<ToFrame>(buffer, FuckFrom<From>(), time_ns);` |

## 头文件

```cpp
#include "math/fast_tf/frame.hpp"    // 坐标系声明, Fuck 类型
#include "math/fast_tf/buffer.hpp"   // CircularTransformBuffer
#include "math/fast_tf/matrix.hpp"   // TransformMatrix4d
```

---

## 预定义坐标系

```cpp
// include/math/fast_tf/frame.hpp 已定义:
// world (root)
// └─ odom
//     └─ gimbal_link
//         ├─ camera_link
//         │   └─ camera_optical_frame
//         └─ muzzle_link
```

---

## 声明坐标系

```cpp
// 1. 声明根坐标系
DECL_ROOT(map);

// 2. 声明子坐标系 (父坐标系必须在之前声明)
DECL(base_link, map);
DECL(radar_link, base_link);
DECL(turret_link, base_link);

// 使用类型别名 (简化命名)
using base   = base_link_fuxk_frame;
using radar  = radar_link_fuxk_frame;
using turret = turret_link_fuxk_frame;
```

---

## TransformMatrix4d 完整示例

```cpp
#include "math/fast_tf/frame.hpp"
#include "math/fast_tf/matrix.hpp"
#include <opencv2/calib3d.hpp>

void example_transform_matrix() {
    using namespace fast_tf;

    // ============================================================================
    // 工厂方法: 创建变换
    // ============================================================================

    // 从四元数 + 平移创建
    Eigen::Quaterniond q(1, 0, 0, 0);  // (w, x, y, z)
    TransformMatrix4d tf1 = TransformMatrix4d::from_quaternion_xyz(q, 1.0, 2.0, 3.0);

    // 从四元数 (零平移)
    TransformMatrix4d tf2 = TransformMatrix4d::from_quaternion(q);
    TransformMatrix4d tf3 = TransformMatrix4d::from_quaternion(q, Eigen::Vector3d(1, 2, 3));

    // 从 RPY 欧拉角 (rad) - XYZ 内旋
    TransformMatrix4d tf4 = TransformMatrix4d::from_rpy(0.1, 0.2, 0.0);
    TransformMatrix4d tf5 = TransformMatrix4d::from_rpy(0.1, 0.2, 0.0, 1.0, 2.0, 3.0);

    // 从旋转矩阵 + 平移
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    TransformMatrix4d tf6 = TransformMatrix4d::from_rt(R, Eigen::Vector3d(0, 0, 0.1));

    // 纯平移 (单位旋转)
    TransformMatrix4d tf7 = TransformMatrix4d::from_translation(0.2, 0, 0);

    // 从 PnP 结果 (OpenCV)
    cv::Vec3d rvec(0.1, 0.2, 0.3);
    cv::Vec3d tvec(1.0, 2.0, 3.0);
    TransformMatrix4d tf8 = TransformMatrix4d::from_pnp(rvec, tvec);

    // ============================================================================
    // 访问器: 获取变换分量
    // ============================================================================

    TransformMatrix4d T = TransformMatrix4d::from_rpy(0.1, 0.2, 0.3, 1.0, 2.0, 3.0);

    // 获取平移向量 (3x1)
    Eigen::Vector3d t = T.translation();  // [1.0, 2.0, 3.0]

    // 获取旋转矩阵 (3x3)
    Eigen::Matrix3d R_mat = T.rotation();

    // 获取四元数
    Eigen::Quaterniond q_T = T.quaternion();

    // 获取欧拉角 (Ros2 convention)
    auto euler = T.euler_rot();
    double roll  = euler.roll;
    double pitch = euler.pitch;
    double yaw   = euler.yaw;

    // 获取完整 4x4 矩阵
    Eigen::Matrix4d mat = T.matrix();

    // 获取 SE(3) 李群
    const auto& se3 = T.se3();

    // ============================================================================
    // 运算符: 变换组合与求逆
    // ============================================================================

    TransformMatrix4d T_a = TransformMatrix4d::from_translation(1, 0, 0);
    TransformMatrix4d T_b = TransformMatrix4d::from_translation(0, 1, 0);

    // 变换组合 (矩阵乘法)
    TransformMatrix4d T_combined = T_a * T_b;

    // 重父化: 将子坐标系变换到新的父坐标系
    // 例如: 子坐标系原本在 parent_old 下，现在要变换到 parent_new 下
    TransformMatrix4d T_child_local    = TransformMatrix4d::from_translation(0.5, 0, 0);
    TransformMatrix4d T_parent_old     = TransformMatrix4d::from_translation(1, 0, 0);
    TransformMatrix4d T_parent_new     = TransformMatrix4d::from_translation(2, 0, 0);
    TransformMatrix4d T_child_in_new   = T_child_local.reparent_to(T_parent_new);

    // 求逆
    TransformMatrix4d T_inv = T_a.inverse();

    // ============================================================================
    // Lie 代数运算: 用于优化/插值
    // ============================================================================

    using Tangent = Eigen::Matrix<double, 6, 1>;  // [rx, ry, rz, vx, vy, vz]

    TransformMatrix4d T_ref = TransformMatrix4d::from_rpy(0.1, 0.2, 0.3, 1, 2, 3);

    // Log: SE(3) -> se(3) (李群 -> 李代数)
    Tangent xi = T_ref.log();

    // Exp: se(3) -> SE(3) (李代数 -> 李群)
    TransformMatrix4d T_from_xi = TransformMatrix4d::exp(xi);

    // Right-minus: 计算两个变换的误差 (用于优化目标函数)
    TransformMatrix4d T_estimated = TransformMatrix4d::from_rpy(0.11, 0.21, 0.31, 1.1, 2.1, 3.1);
    Tangent error = T_ref.rminus(T_estimated);  // = Log(T_ref^{-1} * T_estimated)

    // Right-plus: 在流形上加上增量 (用于优化更新)
    Tangent delta = Tangent::Zero();
    delta << 0.01, 0.02, 0.03, 0.1, 0.2, 0.3;
    TransformMatrix4d T_updated = T_ref.rplus(delta);  // = T_ref * Exp(delta)

    // ============================================================================
    // 插值: 两帧之间的变换
    // ============================================================================

    TransformMatrix4d T_start = TransformMatrix4d::from_rpy(0, 0, 0, 0, 0, 0);
    TransformMatrix4d T_end   = TransformMatrix4d::from_rpy(0.5, 0.5, 0.5, 1, 1, 1);

    // 线性插值 (位置线性, 四元数 slerp)
    TransformMatrix4d T_mid_lerp = TransformMatrix4d::lerp(T_start, T_end, 0.5);

    // SE(3) 测地线插值 (更准确, 适用于大旋转)
    TransformMatrix4d T_mid_se3 = TransformMatrix4d::lerp_se3(T_start, T_end, 0.5);

    // ============================================================================
    // 点变换: 将点从一个坐标系变换到另一个坐标系
    // ============================================================================

    Eigen::Vector3d point_camera(1.0, 2.0, 3.0);
    TransformMatrix4d T_odom_camera = TransformMatrix4d::from_rpy(0.1, 0.2, 0.3, 0.5, 1.0, 1.5);

    // 方式 1: 手动计算 (R * p + t)
    Eigen::Vector3d point_odom = T_odom_camera.rotation() * point_camera
                                  + T_odom_camera.translation();

    // 方式 2: 完整 4x4 矩阵 (需要齐次坐标)
    Eigen::Vector4d point_homo(point_camera.x(), point_camera.y(), point_camera.z(), 1.0);
    Eigen::Vector4d transformed = T_odom_camera.matrix() * point_homo;
    Eigen::Vector3d point_odom2 = transformed.head<3>();
}
```

---

## CoordinateSystem (坐标缓冲区)

```cpp
#include "math/fast_tf/frame.hpp"

// 创建坐标系统 (每个坐标系 1024 个历史样本)
using CoordinateSystem = fast_tf::CoordinateSystem<1024>;
auto tf_buffer = CoordinateSystem();
```

---

## 错误处理

```cpp
enum class TransformError : uint8_t {
    FrameNotFound,               // 坐标系未注册
    NoDataAvailable,             // 缓冲区为空
    PastExtrapolationRequired,   // 时间戳早于最早数据
    FutureExtrapolationRequired, // 时间戳晚于最新数据
    Timeout,                     // 查询超时
    PathNotFound,                // 坐标系之间无路径
};

// 错误转字符串
constexpr std::string_view to_string(TransformError e) noexcept;

// 使用
auto tf = lookup<odom>(...);
if (!tf) {
    spdlog::error("TF error: {}", to_string(tf.error()));
}
```

- **TransformMatrix4d** 封装 SE(3) 李群，支持多种构造方式和 Lie 代数运算
- **from_pnp** 自动处理 OpenCV 结果的正交化和数值稳定性
- **rminus/rplus** 用于流形上的优化（如卡尔曼滤波、Bundle Adjustment）
- **reparent_to** 是 TF 变换的核心语义，将子坐标系从一个父系变换到另一个父系
- **lerp_se3** 测地线插值比线性插值更准确，尤其适合大角度旋转
