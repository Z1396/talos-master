// ===========================================================================
// 阶段9：fast_tf 模块 —— 强类型坐标变换树（最难的一课）
//
// 文件对齐真实项目（crates/fast_tf/src/）：
//   - frame.hpp    帧树：DECL 宏 + frame concept + is_descendant_of 编译期校验
//   - matrix.hpp   TransformMatrix<T, A, B>：A/B 是帧类型（编译期防呆）
//   - buffer.hpp   时序环形缓冲：exact / nearest / interpolate / clamped
//   - validation   变换合法性校验：NaN/Inf/四元数归一化（容差 0.01，对齐 ROS tf2）
//   - euler.hpp    来自 crates/math 的 ROS2 Z-Y-X RPY 欧拉角
//
// 核心设计思想（对应 boot.cpp:112-167 init_coordinate_system 的实战用法）：
//   1. 每个坐标系是一个独立的 C++ 类型（帧标签），变换矩阵的类型
//      TransformMatrix<T, A, B> 携带源/目标帧信息——把 odom 系向量传给
//      camera 系函数直接编译报错，而不是运行时才发现数据串了坐标系；
//   2. 帧树每条边挂一个时序环形缓冲区（EphemeralBuffer），查询时按
//      时间戳插值，对齐 ROS2 TF2 的 lookup 语义；
//   3. lookup<Target, Source> 递归向上拼链：T<A,B> * T<B,C> = T<A,C>。
//
// 测试清单
// 测试1：帧树静态初始化（复刻 boot.cpp init_coordinate_system）+ 查询
// 测试2：编译期帧安全（is_descendant_of / 非法组合在注释中演示报错）
// 测试3：链式变换 vs 手算 Eigen 矩阵连乘（误差 < 1e-12）
// 测试4：时间戳插值（yaw 0 → 0.2rad，查中点 = 0.1）+ 四种查询模式
// 测试5：校验 fail-fast（NaN 报错、非归一化自愈、非法数据被缓冲区丢弃）
// 测试6：lookup_clamped 超界钳位 vs interpolate 超界报错（不外推）
// ===========================================================================

// fast_tf 帧树 + 变换 + 缓冲（真实项目头文件，依赖 Eigen/TBB/fmt/lieplusplus）
#include "frame.hpp"
// 变换合法性校验（真实项目头文件）
#include "validation.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <numbers>

namespace ft = fast_tf;

// ===========================================================================
// 轻量断言：失败打印位置并累计，main 末尾以非零退出码结束
// ===========================================================================
static int g_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::cerr << "  [CHECK 失败] " #cond "  (" << __FILE__ << ":"      \
                      << __LINE__ << ")\n";                                    \
            ++g_failures;                                                      \
        }                                                                      \
    } while (0)

/// 角度转弧度工具函数
static double deg2rad(double deg) { return deg * std::numbers::pi / 180.0; }

/// 打印 4x4 齐次矩阵（3 行有效内容，紧凑展示）
static void print_matrix(const Eigen::Matrix4d& m) {
    for (int i = 0; i < 3; ++i) {
        std::cout << "        ";
        for (int j = 0; j < 4; ++j) {
            std::cout << m(i, j) << (j == 3 ? "\n" : "  ");
        }
    }
}

// ===========================================================================
// 测试1：帧树静态初始化 —— 复刻 boot.cpp 的 init_coordinate_system
// 
// 构建的帧树结构：
//                         ┌─────────────────┐
//                         │     world       │  ← 根帧（全局锚点）
//                         └────────┬────────┘
//                                  │
//                         ┌────────▼────────┐
//                         │      odom       │  ← 里程计帧（机器人定位）
//                         └────────┬────────┘
//                                  │
//                         ┌────────▼────────┐
//                         │   gimbal_yaw    │  ← 云台偏航轴（绕 Z 旋转）
//                         └────────┬────────┘
//                                  │
//                         ┌────────▼────────┐
//                         │  gimbal_pitch   │  ← 云台俯仰轴（绕 Y 旋转）
//                         └────────┬────────┘
//                                  │
//                    ┌─────────────┴─────────────┐
//                    │                           │
//            ┌───────▼───────┐          ┌────────▼────────┐
//            │  camera_link  │          │   muzzle_link   │  ← 枪口（武器）
//            └───────┬───────┘          └─────────────────┘
//                    │
//            ┌───────▼───────┐
//            │camera_optical │  ← 相机光轴坐标系（ROS 标准）
//            └───────────────┘
//
// 每条边用 fast_tf::update<EdgeFrame>(system, 变换, 时间戳) 写入时序缓冲
// ===========================================================================
static void test_frame_tree_init() {
    std::cout << "=== 测试1：帧树静态初始化（复刻 boot.cpp）===\n";

    //翻译：定义一个编译期常量 pi，值是数学常数 π（3.141592653589793...）。
    constexpr double pi = std::numbers::pi;
    ft::CoordinateSystem sys;  // 创建坐标变换系统容器

    // ----- 1. world→odom、odom→gimbal_yaw：单位变换 -----
    // {} 即默认构造的 identity，表示两个坐标系完全重合
    // 实际项目中，odom 通常与 world 重合（或者有固定偏移）
    ft::update<ft::odom>(sys, {}, 0);
    ft::update<ft::gimbal>(sys, {}, 0);
    std::cout << "  1. world→odom、odom→gimbal_yaw：单位变换（重合）\n";

    // ----- 2. gimbal_yaw→gimbal_pitch：纯平移 -----
    // 云台俯仰轴相对偏航轴有机械偏移：Z 方向抬高 0.1 米
    // 实际含义：俯仰轴的旋转中心在偏航轴旋转中心上方 10cm
    ft::update<ft::gimbal_pitch>(
        sys, ft::EdgeTransform<ft::gimbal_pitch>::from_translation(0.0, 0.0, 0.1), 0);
    std::cout << "  2. gimbal_yaw→gimbal_pitch：平移 (0, 0, 0.1)\n";

    // ----- 3. gimbal_pitch→camera_link：平移 + 三轴 RPY -----
    // 相机安装在云台上的外参：
    //   - Roll（绕 X）：-10°  → 相机绕光轴旋转
    //   - Pitch（绕 Y）：5°   → 相机俯仰安装角
    //   - Yaw（绕 Z）：3°     → 相机偏航安装角
    //   - 平移：(0.05, 0.02, 0.01) 米 → 相机光心相对云台中心的偏移
    ft::update<ft::camera>(
        sys,
        ft::EdgeTransform<ft::camera>::from_rpy(
            deg2rad(-10.0),  // Roll (绕 X)
            deg2rad(5.0),    // Pitch (绕 Y)
            deg2rad(3.0),    // Yaw (绕 Z)
            0.05,            // X 方向偏移
            0.02,            // Y 方向偏移
            0.01),           // Z 方向偏移
        0);
    std::cout << "  3. gimbal_pitch→camera_link：RPY(-10°,5°,3°) + 平移(0.05,0.02,0.01)\n";

    // ----- 4. camera_link→camera_optical：ROS 标准转换 -----
    // ROS 相机坐标系约定：
    //   - camera_link：X 朝右，Y 朝下，Z 朝前（从镜头看）
    //   - camera_optical：X 朝右，Y 朝下，Z 朝前（光轴方向）
    // 但 ROS 标准中，camera_optical 的 Z 轴指向光轴前方，
    // 需要把 camera_link 的 Z 轴转到 camera_optical 的 X 轴方向
    // 
    // 具体转换：先绕 X 转 -90°，再绕 Z 转 -90°
    //   原 camera_link:   X→右, Y→下, Z→前
    //   绕 X 转 -90°:     X→右, Y→前, Z→上
    //   绕 Z 转 -90°:     X→前, Y→右, Z→上  ← 标准相机光学坐标系
    ft::update<ft::camera_optical>(
        sys, ft::EdgeTransform<ft::camera_optical>::from_rpy(-pi / 2.0, 0.0, -pi / 2.0), 0);
    std::cout << "  4. camera_link→camera_optical：ROS 标准 RPY(-90°,0°,-90°)\n";

    // ----- 5. gimbal_pitch→muzzle_link：枪口外参（纯平移）-----
    // 枪口相对云台俯仰轴的偏移：
    //   - X：0.2 米（枪口在云台前方 20cm）
    //   - Y：0 米（在正中间）
    //   - Z：-0.05 米（枪口在云台下方 5cm）
    ft::update<ft::muzzle>(
        sys, ft::EdgeTransform<ft::muzzle>::from_translation(0.2, 0.0, -0.05), 0);
    std::cout << "  5. gimbal_pitch→muzzle_link：平移(0.2,0,-0.05)\n";

    std::cout << "\n  ----- 查询验证 -----\n";

    // ----- 查询 T_world_camera：world 到 camera 的变换 -----
    // 路径：world → odom → gimbal_yaw → gimbal_pitch → camera_link → camera_optical
    // 前两帧是单位变换，所以平移 = 云台轴偏移 + 相机安装偏移
    // 手算：(0,0,0.1) + (0.05,0.02,0.01) = (0.05,0.02,0.11)
    const auto t_world_camera = ft::lookup<ft::world, ft::camera>(sys, 0);
    CHECK(t_world_camera.has_value());
    if (t_world_camera) {
        const Eigen::Vector3d t = t_world_camera->translation();
        std::cout << "  T_world_camera 平移 = (" << t.x() << ", " << t.y() << ", " << t.z()
                  << ")\n";
        CHECK(std::abs(t.x() - 0.05) < 1e-12);
        CHECK(std::abs(t.y() - 0.02) < 1e-12);
        CHECK(std::abs(t.z() - 0.11) < 1e-12);
    }

    // ----- 查询 T_world_muzzle：world 到枪口的变换 -----
    // 路径：world → odom → gimbal_yaw → gimbal_pitch → muzzle_link
    // 手算：枪口相对云台平移 (0.2,0,-0.05) + 云台偏移 (0,0,0.1) = (0.2,0,0.05)
    const auto t_world_muzzle = ft::lookup<ft::world, ft::muzzle>(sys, 0);
    CHECK(t_world_muzzle.has_value());
    if (t_world_muzzle) {
        const Eigen::Vector3d t = t_world_muzzle->translation();
        std::cout << "  T_world_muzzle 平移 = (" << t.x() << ", " << t.y() << ", " << t.z()
                  << ")\n";
        CHECK(std::abs(t.x() - 0.2) < 1e-12);
        CHECK(std::abs(t.z() - 0.05) < 1e-12);
    }

    std::cout << "测试1通过\n\n";
}

// ===========================================================================
// 测试2：编译期帧安全 —— 强类型的核心价值
//
// 这是 fast_tf 最难理解但最重要的设计：
//   每个帧标签（如 world、odom、camera）都是不同的 C++ 类型
//   变换矩阵的类型携带源/目标帧信息：TransformMatrix<T, A, B>
//   把 odom 系坐标传给 camera 系函数 → 编译报错
//   在编译期就拦截了"坐标系串了"的 bug
// ===========================================================================
static void test_compile_time_frame_safety() {
    std::cout << "=== 测试2：编译期帧安全 ===\n";

    // ----- is_descendant_of：编译期判断父子关系 -----
    // 在编译期递归遍历帧树，判断一个帧是否是另一个帧的后代
    // 这保证了 lookup 的合法性：lookup<Target, Source> 要求 Source 是 Target 的后代
    static_assert(ft::is_descendant_of<ft::camera, ft::odom>(),
                  "camera 必须是 odom 的后代");  // ✅ 成立
    static_assert(!ft::is_descendant_of<ft::odom, ft::camera>(),
                  "odom 不是 camera 的后代");  // ✅ 成立（方向反了）
    static_assert(ft::frame<ft::camera_optical>, "帧标签必须满足 frame concept");
    static_assert(ft::root_frame<ft::world>, "world 是根帧");
    std::cout << "  static_assert 全部成立：is_descendant_of<camera, odom> = "
              << ft::is_descendant_of<ft::camera, ft::odom>() << "\n";

    // ----- 运行时验证变换复合的类型代数 -----
    // 代数规则：T<A,B> * T<B,C> = T<A,C>
    // 中间帧必须一致，否则编译报错
    const auto t_odom_gyaw = ft::FrameTransform<ft::odom, ft::gimbal>::from_rpy(0, 0, 0.5);
    const auto t_gyaw_gpitch = ft::FrameTransform<ft::gimbal, ft::gimbal_pitch>::from_translation(
        0.0, 0.0, 0.1);
    
    // ✅ 正确复合：odom→gimbal × gimbal→gimbal_pitch = odom→gimbal_pitch
    // 中间帧都是 gimbal，类型匹配
    const auto t_odom_gpitch = t_odom_gyaw * t_gyaw_gpitch;
    CHECK(std::abs(t_odom_gpitch.translation().z() - 0.1) < 1e-12);
    std::cout << "  复合 T<odom,gimbal> * T<gimbal,gimbal_pitch> = T<odom,gimbal_pitch>，"
              << "平移 z = " << t_odom_gpitch.translation().z() << "\n";

    // ----- 求逆自动翻转帧类型 -----
    // inv(T<A,B>) = T<B,A>
    const auto t_gpitch_odom = t_odom_gpitch.inv();  // T<gimbal_pitch, odom>
    const auto round_trip = t_odom_gpitch * t_gpitch_odom;  // T<odom, odom> 单位变换
    CHECK((round_trip.matrix() - Eigen::Matrix4d::Identity()).norm() < 1e-12);
    std::cout << "  inv 复合回环 = 单位矩阵（误差 < 1e-12）\n";

    // ------------------------------------------------------------------
    // 【编译期错误演示】以下代码放开任何一行都直接编译报错——
    // 这就是强类型帧的全部意义：坐标错乱在编译期被拦截，而非运行时。
    // ------------------------------------------------------------------
    //
    // (a) 帧不匹配的复合：operator* 要求 T<A,B> * T<B,C>，中间帧必须一致
    //     下面的代码中，第一个变换是 odom→gimbal，第二个是 odom→gimbal_pitch
    //     中间帧一个是 gimbal，一个是 odom，不匹配 → 编译报错
    //     error: no match for 'operator*'
    //     ft::FrameTransform<ft::odom, ft::camera> bad =
    //         t_odom_gyaw * ft::FrameTransform<ft::odom, ft::gimbal_pitch>::from_translation(0,0,0);
    //
    // (b) lookup 方向非法：Source 必须是 Target 的后代
    //     lookup<camera, odom> 要求 odom 是 camera 的后代，但实际 camera 是 odom 的后代
    //     方向反了 → 编译报错
    //     error: static assertion failed: Source must be a descendant of Target
    //     auto bad = ft::lookup<ft::camera, ft::odom>(sys, 0);
    //
    // (c) 类型不能隐式互换：不同帧标签是完全不同的 C++ 类型
    //     下面试图把 T<odom,gimbal> 赋值给 T<odom,camera>，类型不匹配 → 编译报错
    //     error: conversion from 'TransformMatrix<double, odom, gimbal>' to
    //            'TransformMatrix<double, odom, camera>' not possible
    //     ft::FrameTransform<ft::odom, ft::camera> wrong = t_odom_gyaw;
    // ------------------------------------------------------------------

    std::cout << "测试2通过\n\n";
}

// ===========================================================================
// 测试3：链式变换 vs 手算 Eigen 矩阵连乘
//
// 构建一个简单的三帧链：
//   odom --yaw 0.5rad--> gimbal_yaw --平移(0,0,0.1)--> gimbal_pitch
//         --pitch 0.2rad + 平移(0.05,0,0)--> camera_link
//
// 验证两种方式结果一致：
//   方式A：fast_tf 的 lookup 自动链式查询
//   方式B：手工用 Eigen 矩阵连乘
// ===========================================================================
static void test_chain_lookup_hand_computed() {
    std::cout << "=== 测试3：链式变换 vs 手算矩阵连乘 ===\n";

    // ----- 构建帧树 -----
    ft::CoordinateSystem sys;
    // 边1：odom→gimbal_yaw：绕 Z 轴旋转 0.5rad（约 28.6°）
    ft::update<ft::gimbal>(sys, ft::EdgeTransform<ft::gimbal>::from_rpy(0.0, 0.0, 0.5), 0);
    // 边2：gimbal_yaw→gimbal_pitch：Z 方向平移 0.1 米（纯平移）
    ft::update<ft::gimbal_pitch>(
        sys, ft::EdgeTransform<ft::gimbal_pitch>::from_translation(0.0, 0.0, 0.1), 0);
    // 边3：gimbal_pitch→camera：绕 Y 轴旋转 0.2rad + X 方向平移 0.05 米
    ft::update<ft::camera>(
        sys, ft::EdgeTransform<ft::camera>::from_rpy(0.0, 0.2, 0.0, 0.05, 0.0, 0.0), 0);

    // ----- 方式A：fast_tf lookup -----
    const auto result = ft::lookup<ft::odom, ft::camera>(sys, 0);
    CHECK(result.has_value());
    if (!result) {
        std::cout << "  lookup 失败: " << result.error() << "\n";
        return;
    }

    // ----- 方式B：手算 Eigen 矩阵连乘 -----
    // 构造三个变换的旋转矩阵和平移向量
    const Eigen::Matrix3d R1 = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Matrix3d R2 = Eigen::Matrix3d::Identity();  // 纯平移，无旋转
    const Eigen::Matrix3d R3 = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();

    // 辅助函数：从旋转矩阵和平移向量构造 4x4 齐次变换矩阵
    auto make_homogeneous = [](const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        m.block<3, 3>(0, 0) = R;
        m.block<3, 1>(0, 3) = t;
        return m;
    };

    // 连乘：T_odom_camera = T_odom_gimbal * T_gimbal_gimbal_pitch * T_gimbal_pitch_camera
    const Eigen::Matrix4d expected =
        make_homogeneous(R1, Eigen::Vector3d::Zero())
        * make_homogeneous(R2, Eigen::Vector3d(0.0, 0.0, 0.1))
        * make_homogeneous(R3, Eigen::Vector3d(0.05, 0.0, 0.0));

    // ----- 比较两种方式的结果 -----
    const double matrix_err = (result->matrix() - expected).norm();
    std::cout << "  lookup 矩阵 vs 手算连乘，Frobenius 误差 = " << matrix_err << "\n";
    CHECK(matrix_err < 1e-12);

    std::cout << "  T_odom_camera =\n";
    print_matrix(result->matrix());

    // ----- 验证点变换的一致性 -----
    // 把 camera 系下的点 (1,2,3) 转换到 odom 系
    const Eigen::Vector3d p_camera(1.0, 2.0, 3.0);
    const Eigen::Vector4d p_hom(p_camera.x(), p_camera.y(), p_camera.z(), 1.0);

    // 途径A：用 lookup 结果的 4x4 矩阵左乘齐次坐标
    const Eigen::Vector4d p_odom_via_tf = result->matrix() * p_hom;

    // 途径B：手算逐层链式变换
    // p_odom = R1 * (R2 * (R3 * p_camera + t3) + t2) + t1
    const Eigen::Vector3d p_odom_by_hand =
        R1 * (R2 * (R3 * p_camera + Eigen::Vector3d(0.05, 0.0, 0.0))
              + Eigen::Vector3d(0.0, 0.0, 0.1))
        + Eigen::Vector3d::Zero();

    const double point_err = (p_odom_via_tf.head<3>() - p_odom_by_hand).norm();
    std::cout << "  点 (1,2,3)_camera → odom: (" << p_odom_by_hand.x() << ", "
              << p_odom_by_hand.y() << ", " << p_odom_by_hand.z() << ")\n";
    std::cout << "  tf 途径 vs 手算途径误差 = " << point_err << "\n";
    CHECK(point_err < 1e-9);

    std::cout << "测试3通过\n\n";
}

// ===========================================================================
// 测试4：时间戳插值 + 缓冲区四种查询模式
//
// 测试场景：gimbal_yaw 边随时间变化
//   t=0s:   yaw = 0.0 rad
//   t=1s:   yaw = 0.2 rad
//   查询 t=0.5s 应得 yaw = 0.1 rad（线性插值）
//
// 四种查询模式对比：
//   interpolate  → 时间线性插值
//   nearest      → 取最近时间戳的样本
//   exact        → 必须精确匹配时间戳
//   clamped      → 超界时钳位到端点
// ===========================================================================
static void test_time_interpolation() {
    std::cout << "=== 测试4：时间戳插值 + 四种查询模式 ===\n";

    // 时间戳定义（纳秒单位）
    constexpr std::uint64_t t0 = 0;                     // 0s
    constexpr std::uint64_t t1 = 1'000'000'000;         // 1s
    constexpr std::uint64_t tm = 500'000'000;           // 0.5s（中点）
    constexpr std::uint64_t tq = 600'000'000;           // 0.6s（非对称，避免 nearest 平局）

    ft::CoordinateSystem sys;

    // ----- 在 gimbal_yaw 边写入两个时间戳的样本 -----
    // t=0: yaw = 0
    ft::update<ft::gimbal>(sys, ft::EdgeTransform<ft::gimbal>::from_rpy(0.0, 0.0, 0.0), t0);
    // t=1: yaw = 0.2rad（约 11.5°）
    ft::update<ft::gimbal>(sys, ft::EdgeTransform<ft::gimbal>::from_rpy(0.0, 0.0, 0.2), t1);
    std::cout << "  写入样本：t=0 → yaw=0，t=1s → yaw=0.2rad\n";

    // ----- 在其他边上放置样本保证链式查询可用 -----
    // gimbal_pitch 边在两个端点各放一份相同样本，保证查询时刻全覆盖
    const auto pitch_edge = ft::EdgeTransform<ft::gimbal_pitch>::from_translation(0.0, 0.0, 0.1);
    ft::update<ft::gimbal_pitch>(sys, pitch_edge, t0);
    ft::update<ft::gimbal_pitch>(sys, pitch_edge, t1);

    // ---- 4a. 树级查询：lookup<odom, gimbal> 自动走 interpolate ----
    const auto t_odom_gimbal = ft::lookup<ft::odom, ft::gimbal>(sys, tm);
    CHECK(t_odom_gimbal.has_value());
    if (t_odom_gimbal) {
        const double yaw = t_odom_gimbal->euler_rot().yaw;
        std::cout << "  lookup<odom,gimbal_yaw>(0.5s)：yaw = " << yaw
                  << "（两端均值 0.1）\n";
        CHECK(std::abs(yaw - 0.1) < 1e-12);
    }

    // ---- 4b. 缓冲区级：直接操作单条边的 EphemeralBuffer ----
    const auto& buf = ft::buffer_of<ft::gimbal>(sys);
    std::cout << "  缓冲区样本数 = " << buf.size() << "，容量 = " << buf.capacity() << "\n";
    CHECK(buf.size() == 2);
    CHECK(buf.contains_time(tm));  // 查询时间在样本范围内

    // ----- interpolate：线性插值 -----
    // 0.5s 在 0s 和 1s 正中间 → ratio=0.5 → yaw=0.1
    const auto r_interp = buf.lookup(tm, ft::interpolate);
    CHECK(r_interp.has_value());
    if (r_interp) {
        std::cout << "  interpolate(0.5s)：yaw = " << r_interp->value.euler_rot().yaw << "\n";
        CHECK(std::abs(r_interp->value.euler_rot().yaw - 0.1) < 1e-12);
        CHECK(r_interp->timestamp == tm);
    }

    // ----- nearest：取最近时间戳的样本 -----
    // 0.6s 离 1s（距离 0.4s）比离 0s（距离 0.6s）更近 → 取 t=1s 的样本
    const auto r_nearest = buf.lookup(tq, ft::nearest);
    CHECK(r_nearest.has_value());
    if (r_nearest) {
        std::cout << "  nearest(0.6s)：yaw = " << r_nearest->value.euler_rot().yaw
                  << "（离 1s 样本更近，整样本返回）\n";
        CHECK(std::abs(r_nearest->value.euler_rot().yaw - 0.2) < 1e-12);
    }

    // ----- exact：必须精确匹配时间戳 -----
    // 0.5s 在 0s 和 1s 之间，没有精确匹配 → 报错
    const auto r_exact = buf.lookup(tm, ft::exact);
    CHECK(!r_exact.has_value());
    std::cout << "  exact(0.5s)：按预期报错 —— " << r_exact.error() << "\n";

    // ----- clamped：超界时钳位到端点样本（不外推）-----
    // 查询 2s，超出最新样本 1s → 钳位到 t=1s 的样本
    const auto r_clamped = buf.lookup(t1 * 2, ft::clamped);
    CHECK(r_clamped.has_value());
    if (r_clamped) {
        std::cout << "  clamped(2s)：钳位到最新样本，yaw = "
                  << r_clamped->value.euler_rot().yaw << "\n";
        CHECK(std::abs(r_clamped->value.euler_rot().yaw - 0.2) < 1e-12);
    }

    // ----- interpolate 超界：报错 -----
    // 工程上禁止外推，避免预测漂移 → 直接报错
    const auto r_extrap = buf.lookup(t1 * 2, ft::interpolate);
    CHECK(!r_extrap.has_value());
    std::cout << "  interpolate(2s)：按预期报错 —— " << r_extrap.error() << "\n";

    // ----- latest：最新写入的样本 -----
    const auto r_latest = buf.latest();
    CHECK(r_latest.has_value());
    if (r_latest) {
        CHECK(r_latest->timestamp == t1);
        CHECK(std::abs(r_latest->value.euler_rot().yaw - 0.2) < 1e-12);
    }

    std::cout << "测试4通过\n\n";
}

// ===========================================================================
// 测试5：校验 fail-fast
//
// 验证三层防御机制：
//   1. validate_transform()：检测 NaN/Inf/非归一化四元数
//   2. SO3 构造函数自愈：非归一化输入自动归一化
//   3. Buffer::push 门卫：非法变换被丢弃，不入队
// ===========================================================================
static void test_validation_fail_fast() {
    std::cout << "=== 测试5：校验 fail-fast ===\n";

    // ----- 合法变换：校验通过 -----
    const auto good = ft::EdgeTransform<ft::gimbal_pitch>::from_translation(0.0, 0.0, 0.1);
    CHECK(ft::validate_transform(good).has_value());
    std::cout << "  合法变换：validate_transform 通过\n";

    // ----- NaN 平移：校验失败 -----
    // 构造一个平移向量包含 NaN 的变换
    const auto nan_tf = ft::EdgeTransform<ft::gimbal_pitch>::from_translation(
        std::numeric_limits<double>::quiet_NaN(),  // X = NaN
        0.0,                                        // Y = 0
        0.1);                                       // Z = 0.1
    const auto nan_result = ft::validate_transform(nan_tf);
    CHECK(!nan_result.has_value());
    std::cout << "  NaN 变换：报错 —— " << nan_result.error() << "\n";
    CHECK(nan_result.error().find("NaN") != std::string::npos);

    // ----- 非归一化输入会被"自愈" -----
    // 这是 SO3 构造函数的特性：检测到非归一化输入会自动归一化
    // 源码行为：SO3(const MatrixType& R) 内部检查 isNormalized()
    //         不满足就强制归一化并重建 R_/q_
    //         from_quaternion() 也强制 normalize()
    // 
    // 下面构造一个 2 倍缩放的旋转矩阵（非归一化）
    // SO3 构造函数会把它归一化，所以 quaternion 模长恒为 1
    Eigen::Matrix4d scaled = 2.0 * Eigen::Matrix4d::Identity();  // 缩放 2 倍
    const ft::EdgeTransform<ft::gimbal_pitch> healed(scaled);
    CHECK(std::abs(healed.quaternion().norm() - 1.0) < 1e-12);
    CHECK(ft::validate_transform(healed).has_value());
    std::cout << "  自愈演示：2 倍缩放矩阵构造 → 四元数模长 = " << healed.quaternion().norm()
              << "（SO3 构造函数强制归一化），validate 通过\n";

    // ----- 真正会被拦截的是 NaN/Inf -----
    // 旋转矩阵混入 NaN → validate 报错
    Eigen::Matrix3d R_nan = Eigen::Matrix3d::Identity();
    R_nan(0, 0) = std::numeric_limits<double>::quiet_NaN();  // 在旋转矩阵中放 NaN
    const auto nan_rot = ft::EdgeTransform<ft::gimbal_pitch>::from_rt(
        R_nan, Eigen::Vector3d(0.0, 0.0, 0.1));
    const auto rot_result = ft::validate_transform(nan_rot);
    CHECK(!rot_result.has_value());
    std::cout << "  NaN 旋转：报错 —— " << rot_result.error() << "\n";
    CHECK(rot_result.error().find("NaN") != std::string::npos);

    // ----- Buffer::push 门卫：非法变换被静默丢弃 -----
    // update() 内部调用 Buffer::push()，push() 会先调用 validate_transform()
    // 非法变换不入队，后续 lookup 会报"buffer is empty"
    ft::CoordinateSystem sys;
    ft::update<ft::gimbal_pitch>(sys, nan_tf, 0);  // 尝试写入 NaN 变换
    const auto lookup_result = ft::lookup<ft::odom, ft::gimbal_pitch>(sys, 0);
    CHECK(!lookup_result.has_value());
    std::cout << "  push(NaN) 后 lookup：按预期报错 —— " << lookup_result.error() << "\n";
    CHECK(lookup_result.error().find("empty") != std::string::npos);

    std::cout << "测试5通过\n\n";
}

// ===========================================================================
// 测试6：lookup_clamped —— 树级钳位查询
//
// 对标 ROS2 TF2 的 TimePointZero 语义：
//   查询超界取边界样本，绝不做外推
//   - lookup (interpolate)：超界报错
//   - lookup_clamped：超界钳位到端点
// ===========================================================================
static void test_lookup_clamped() {
    std::cout << "=== 测试6：lookup_clamped 钳位查询 ===\n";

    constexpr std::uint64_t t0 = 0;
    constexpr std::uint64_t t1 = 1'000'000'000;
    constexpr std::uint64_t t_beyond = 2'000'000'000;  // 超出最新样本 1s

    ft::CoordinateSystem sys;
    // 写入两个样本：t=0 → yaw=0，t=1s → yaw=0.2rad
    ft::update<ft::gimbal>(sys, ft::EdgeTransform<ft::gimbal>::from_rpy(0.0, 0.0, 0.0), t0);
    ft::update<ft::gimbal>(sys, ft::EdgeTransform<ft::gimbal>::from_rpy(0.0, 0.0, 0.2), t1);

    // ----- lookup_clamped：超界钳位，不报错 -----
    // 查询 2s，超出最新样本 1s → 钳位到 yaw=0.2
    const auto clamped = ft::lookup_clamped<ft::odom, ft::gimbal>(sys, t_beyond);
    CHECK(clamped.has_value());
    if (clamped) {
        std::cout << "  lookup_clamped(2s)：yaw = " << clamped->euler_rot().yaw
                  << "（钳位到最新样本，不外推）\n";
        CHECK(std::abs(clamped->euler_rot().yaw - 0.2) < 1e-12);
    }

    // ----- lookup (interpolate)：超界报错 -----
    // 同一时刻，默认的 lookup 使用 interpolate 模式 → 拒绝外推
    const auto extrapolate = ft::lookup<ft::odom, ft::gimbal>(sys, t_beyond);
    CHECK(!extrapolate.has_value());
    std::cout << "  lookup(2s)：按预期报错 —— " << extrapolate.error() << "\n";

    // ----- 早于最早样本同理（past extrapolation）-----
    // 缓冲区只有 t=1s 的样本，查询 t=0s → 外推到过去，报错
    ft::CoordinateSystem sys2;
    ft::update<ft::gimbal>(sys2, ft::EdgeTransform<ft::gimbal>::from_rpy(0.0, 0.0, 0.1), t1);
    const auto past = ft::lookup<ft::odom, ft::gimbal>(sys2, t0);
    CHECK(!past.has_value());
    std::cout << "  lookup(0s, 缓冲区只有 1s)：按预期报错 —— " << past.error() << "\n";

    std::cout << "测试6通过\n\n";
}

// ===========================================================================
// main：依次跑全部测试，任一断言失败 → 非零退出码
// ===========================================================================
int main() {
    std::cout << "fast_tf 强类型坐标变换树 demo\n";
    std::cout << "========================================\n\n";

    test_frame_tree_init();          // 测试1：帧树初始化
    test_compile_time_frame_safety(); // 测试2：编译期帧安全
    test_chain_lookup_hand_computed(); // 测试3：链式查询 vs 手算
    test_time_interpolation();       // 测试4：时间戳插值
    test_validation_fail_fast();     // 测试5：校验 fail-fast
    test_lookup_clamped();           // 测试6：钳位查询

    if (g_failures == 0) {
        std::cout << "全部测试通过（PASS）\n";
        return 0;
    }
    std::cerr << "共 " << g_failures << " 个断言失败\n";
    return 1;
}