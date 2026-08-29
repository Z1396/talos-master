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
#include <string>

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
// world → odom → gimbal_yaw → gimbal_pitch → camera_link → camera_optical
//                                             └→ muzzle_link
// 每条边用 fast_tf::update<EdgeFrame>(system, 变换, 时间戳) 写入时序缓冲
// ===========================================================================
static void test_frame_tree_init() {
    std::cout << "=== 测试1：帧树静态初始化（复刻 boot.cpp）===\n";

    constexpr double pi = std::numbers::pi;
    ft::CoordinateSystem sys;

    // 1. world→odom、odom→gimbal_yaw：单位变换（{} 即默认构造的 identity）
    ft::update<ft::odom>(sys, {}, 0);
    ft::update<ft::gimbal>(sys, {}, 0);

    // 2. gimbal_yaw→gimbal_pitch：纯平移（云台俯仰轴机械偏移）
    ft::update<ft::gimbal_pitch>(
        sys, ft::EdgeTransform<ft::gimbal_pitch>::from_translation(0.0, 0.0, 0.1), 0);

    // 3. gimbal_pitch→camera_link：平移 + 三轴 RPY（相机安装外参）
    ft::update<ft::camera>(
        sys,
        ft::EdgeTransform<ft::camera>::from_rpy(
            deg2rad(-10.0), deg2rad(5.0), deg2rad(3.0), 0.05, 0.02, 0.01),
        0);

    // 4. camera_link→camera_optical：行业标准固定转换
    //    先绕 X 转 -90°，再绕 Z 转 -90°，使 Z 轴指向光轴前方、Y 轴向图像下方
    ft::update<ft::camera_optical>(
        sys, ft::EdgeTransform<ft::camera_optical>::from_rpy(-pi / 2.0, 0.0, -pi / 2.0), 0);

    // 5. gimbal_pitch→muzzle_link：枪口外参（纯平移）
    ft::update<ft::muzzle>(
        sys, ft::EdgeTransform<ft::muzzle>::from_translation(0.2, 0.0, -0.05), 0);

    // 查询 T_world_camera：前两帧单位变换，平移 = 云台轴偏移 + 相机安装偏移
    const auto t_world_camera = ft::lookup<ft::world, ft::camera>(sys, 0);
    CHECK(t_world_camera.has_value());
    if (t_world_camera) {
        const Eigen::Vector3d t = t_world_camera->translation();
        std::cout << "  T_world_camera 平移 = (" << t.x() << ", " << t.y() << ", " << t.z()
                  << ")\n";
        // 手算：(0,0,0.1) + (0.05,0.02,0.01) = (0.05,0.02,0.11)
        CHECK(std::abs(t.x() - 0.05) < 1e-12);
        CHECK(std::abs(t.y() - 0.02) < 1e-12);
        CHECK(std::abs(t.z() - 0.11) < 1e-12);
    }

    // 查询 T_world_muzzle：枪口在 world 下的位置
    const auto t_world_muzzle = ft::lookup<ft::world, ft::muzzle>(sys, 0);
    CHECK(t_world_muzzle.has_value());
    if (t_world_muzzle) {
        const Eigen::Vector3d t = t_world_muzzle->translation();
        std::cout << "  T_world_muzzle 平移 = (" << t.x() << ", " << t.y() << ", " << t.z()
                  << ")\n";
        // 手算：(0.2,0,-0.05) + (0,0,0.1) = (0.2,0,0.05)
        CHECK(std::abs(t.x() - 0.2) < 1e-12);
        CHECK(std::abs(t.z() - 0.05) < 1e-12);
    }

    std::cout << "测试1通过\n\n";
}

// ===========================================================================
// 测试2：编译期帧安全 —— 强类型的核心价值
// ===========================================================================
static void test_compile_time_frame_safety() {
    std::cout << "=== 测试2：编译期帧安全 ===\n";

    // is_descendant_of：编译期递归遍历 ancestor 父链，判断子树关系
    static_assert(ft::is_descendant_of<ft::camera, ft::odom>(),
                  "camera 必须是 odom 的后代");
    static_assert(!ft::is_descendant_of<ft::odom, ft::camera>(),
                  "odom 不是 camera 的后代");
    static_assert(ft::frame<ft::camera_optical>, "帧标签必须满足 frame concept");
    static_assert(ft::root_frame<ft::world>, "world 是根帧");
    std::cout << "  static_assert 全部成立：is_descendant_of<camera, odom> = "
              << ft::is_descendant_of<ft::camera, ft::odom>() << "\n";

    // 运行时验证变换复合的类型代数：T<A,B> * T<B,C> = T<A,C>
    const auto t_odom_gyaw    = ft::FrameTransform<ft::odom, ft::gimbal>::from_rpy(0, 0, 0.5);
    const auto t_gyaw_gpitch = ft::FrameTransform<ft::gimbal, ft::gimbal_pitch>::from_translation(
        0.0, 0.0, 0.1);
    // 复合结果的帧类型由编译器推导，中间帧不匹配直接编译失败
    const auto t_odom_gpitch = t_odom_gyaw * t_gyaw_gpitch;
    CHECK(std::abs(t_odom_gpitch.translation().z() - 0.1) < 1e-12);
    std::cout << "  复合 T<odom,gimbal> * T<gimbal,gimbal_pitch> = T<odom,gimbal_pitch>，"
              << "平移 z = " << t_odom_gpitch.translation().z() << "\n";

    // 求逆自动翻转帧类型：inv(T<A,B>) = T<B,A>
    const auto t_gpitch_odom = t_odom_gpitch.inv();
    const auto round_trip    = t_odom_gpitch * t_gpitch_odom; // T<A,A> 单位变换
    CHECK((round_trip.matrix() - Eigen::Matrix4d::Identity()).norm() < 1e-12);
    std::cout << "  inv 复合回环 = 单位矩阵（误差 < 1e-12）\n";

    // ------------------------------------------------------------------
    // 【编译期错误演示】以下代码放开任何一行都直接编译报错——
    // 这就是强类型帧的全部意义：坐标错乱在编译期被拦截，而非运行时。
    //
    // (a) 帧不匹配的复合：operator* 要求 T<A,B> * T<B,C>，中间帧必须一致
    //     error: no match for 'operator*'
    //     ft::FrameTransform<ft::odom, ft::camera> bad =
    //         t_odom_gyaw * ft::FrameTransform<ft::odom, ft::gimbal_pitch>::from_translation(0,0,0);
    //
    // (b) lookup 方向非法：Source 必须是 Target 的后代
    //     error: static assertion failed: Source must be a descendant of Target
    //     auto bad = ft::lookup<ft::camera, ft::odom>(sys, 0);
    //
    // (c) 类型不能隐式互换：不同帧标签是完全不同的 C++ 类型
    //     error: conversion from 'TransformMatrix<double, odom, gimbal>' to
    //            'TransformMatrix<double, odom, camera>' not possible
    //     ft::FrameTransform<ft::odom, ft::camera> wrong = t_odom_gyaw;
    // ------------------------------------------------------------------

    std::cout << "测试2通过\n\n";
}

// ===========================================================================
// 测试3：链式变换 vs 手算 Eigen 矩阵连乘
// 帧树：odom --yaw 0.5rad--> gimbal_yaw --平移(0,0,0.1)--> gimbal_pitch
//        --pitch 0.2rad + 平移(0.05,0,0)--> camera_link
// 验证 lookup<odom, camera_link> 与手工 4x4 矩阵连乘逐元素一致
// ===========================================================================
static void test_chain_lookup_hand_computed() {
    std::cout << "=== 测试3：链式变换 vs 手算矩阵连乘 ===\n";

    ft::CoordinateSystem sys;
    ft::update<ft::gimbal>(sys, ft::EdgeTransform<ft::gimbal>::from_rpy(0.0, 0.0, 0.5), 0);
    ft::update<ft::gimbal_pitch>(
        sys, ft::EdgeTransform<ft::gimbal_pitch>::from_translation(0.0, 0.0, 0.1), 0);
    ft::update<ft::camera>(
        sys, ft::EdgeTransform<ft::camera>::from_rpy(0.0, 0.2, 0.0, 0.05, 0.0, 0.0), 0);

    const auto result = ft::lookup<ft::odom, ft::camera>(sys, 0);
    CHECK(result.has_value());
    if (!result) {
        std::cout << "  lookup 失败: " << result.error() << "\n";
        return;
    }

    // ---- 手算：直接用 Eigen 原生类型构造三条边，再连乘 ----
    const Eigen::Matrix3d R1 =
        Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Matrix3d R2 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R3 =
        Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();

    auto make_homogeneous = [](const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
        Eigen::Matrix4d m   = Eigen::Matrix4d::Identity();
        m.block<3, 3>(0, 0) = R;
        m.block<3, 1>(0, 3) = t;
        return m;
    };

    const Eigen::Matrix4d expected =
        make_homogeneous(R1, Eigen::Vector3d::Zero())
        * make_homogeneous(R2, Eigen::Vector3d(0.0, 0.0, 0.1))
        * make_homogeneous(R3, Eigen::Vector3d(0.05, 0.0, 0.0));

    const double matrix_err = (result->matrix() - expected).norm();
    std::cout << "  lookup 矩阵 vs 手算连乘，Frobenius 误差 = " << matrix_err << "\n";
    CHECK(matrix_err < 1e-12);

    std::cout << "  T_odom_camera =\n";
    print_matrix(result->matrix());

    // ---- 点变换：camera 系点 (1,2,3) → odom 系 ----
    const Eigen::Vector3d p_camera(1.0, 2.0, 3.0);
    const Eigen::Vector4d p_hom(p_camera.x(), p_camera.y(), p_camera.z(), 1.0);

    // 途径A：用 lookup 结果的 4x4 矩阵左乘齐次坐标
    const Eigen::Vector4d p_odom_via_tf = result->matrix() * p_hom;

    // 途径B：手算逐层链式变换（不借助任何 fast_tf 代码）
    const Eigen::Vector3d p_odom_by_hand =
        R1 * (R2 * (R3 * p_camera + Eigen::Vector3d(0.05, 0.0, 0.0))
              + Eigen::Vector3d(0.0, 0.0, 0.1))
        + Eigen::Vector3d::Zero();

    const double point_err =
        (p_odom_via_tf.head<3>() - p_odom_by_hand).norm();
    std::cout << "  点 (1,2,3)_camera → odom: (" << p_odom_by_hand.x() << ", "
              << p_odom_by_hand.y() << ", " << p_odom_by_hand.z() << ")\n";
    std::cout << "  tf 途径 vs 手算途径误差 = " << point_err << "\n";
    CHECK(point_err < 1e-9);

    std::cout << "测试3通过\n\n";
}

// ===========================================================================
// 测试4：时间戳插值 + 缓冲区四种查询模式
// gimbal_yaw 边在 t=0 时 yaw=0，t=1s 时 yaw=0.2rad
// 查 t=0.5s 应得 yaw=0.1（四元数 slerp，单轴旋转下精确线性）
// ===========================================================================
static void test_time_interpolation() {
    std::cout << "=== 测试4：时间戳插值 + 四种查询模式 ===\n";

    constexpr std::uint64_t t0 = 0;
    constexpr std::uint64_t t1 = 1'000'000'000; // 1s（纳秒）
    constexpr std::uint64_t tm = 500'000'000;   // 0.5s
    constexpr std::uint64_t tq = 600'000'000;   // 0.6s（非对称，避免 nearest 平局）

    ft::CoordinateSystem sys;
    // gimbal_yaw 边：两个时刻，yaw 从 0 转到 0.2rad
    ft::update<ft::gimbal>(sys, ft::EdgeTransform<ft::gimbal>::from_rpy(0.0, 0.0, 0.0), t0);
    ft::update<ft::gimbal>(sys, ft::EdgeTransform<ft::gimbal>::from_rpy(0.0, 0.0, 0.2), t1);

    // 其余边在两个端点各放一份相同样本，保证链上查询时刻全覆盖
    const auto pitch_edge =
        ft::EdgeTransform<ft::gimbal_pitch>::from_translation(0.0, 0.0, 0.1);
    ft::update<ft::gimbal_pitch>(sys, pitch_edge, t0);
    ft::update<ft::gimbal_pitch>(sys, pitch_edge, t1);

    // ---- 4a. 树级查询：lookup<odom, gimbal_yaw> 自动走 interpolate ----
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
    CHECK(buf.contains_time(tm));

    // interpolate：按时间比例插值（0.5s → ratio 0.5 → yaw 0.1）
    const auto r_interp = buf.lookup(tm, ft::interpolate);
    CHECK(r_interp.has_value());
    if (r_interp) {
        std::cout << "  interpolate(0.5s)：yaw = " << r_interp->value.euler_rot().yaw << "\n";
        CHECK(std::abs(r_interp->value.euler_rot().yaw - 0.1) < 1e-12);
        CHECK(r_interp->timestamp == tm);
    }

    // nearest：就近取值（0.6s 离 1s 更近 → 取 yaw=0.2 的样本）
    const auto r_nearest = buf.lookup(tq, ft::nearest);
    CHECK(r_nearest.has_value());
    if (r_nearest) {
        std::cout << "  nearest(0.6s)：yaw = " << r_nearest->value.euler_rot().yaw
                  << "（离 1s 样本更近，整样本返回）\n";
        CHECK(std::abs(r_nearest->value.euler_rot().yaw - 0.2) < 1e-12);
    }

    // exact：必须完全匹配时间戳，0.5s 落在两样本之间 → 报错
    const auto r_exact = buf.lookup(tm, ft::exact);
    CHECK(!r_exact.has_value());
    std::cout << "  exact(0.5s)：按预期报错 —— " << r_exact.error() << "\n";

    // clamped：查询超界时钳位到端点样本（不外推）
    const auto r_clamped = buf.lookup(t1 * 2, ft::clamped);
    CHECK(r_clamped.has_value());
    if (r_clamped) {
        std::cout << "  clamped(2s)：钳位到最新样本，yaw = "
                  << r_clamped->value.euler_rot().yaw << "\n";
        CHECK(std::abs(r_clamped->value.euler_rot().yaw - 0.2) < 1e-12);
    }

    // interpolate 超界：报错（工程上禁止外推，避免预测漂移）
    const auto r_extrap = buf.lookup(t1 * 2, ft::interpolate);
    CHECK(!r_extrap.has_value());
    std::cout << "  interpolate(2s)：按预期报错 —— " << r_extrap.error() << "\n";

    // latest：最新写入的样本
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
//   - validate_transform：NaN/Inf 逐分量检测 + 四元数归一化检测（容差 0.01）
//   - SO3 构造函数自愈：非归一化输入在入口就被强制归一化（读源码确认的行为）
//   - Buffer::push 内置 is_valid 门卫：非法变换直接丢弃，绝不入队
// ===========================================================================
static void test_validation_fail_fast() {
    std::cout << "=== 测试5：校验 fail-fast ===\n";

    // 合法变换 → 校验通过
    const auto good = ft::EdgeTransform<ft::gimbal_pitch>::from_translation(0.0, 0.0, 0.1);
    CHECK(ft::validate_transform(good).has_value());
    std::cout << "  合法变换：validate_transform 通过\n";

    // NaN 平移 → 报错，错误串包含字段名与完整数值
    const auto nan_tf = ft::EdgeTransform<ft::gimbal_pitch>::from_translation(
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.1);
    const auto nan_result = ft::validate_transform(nan_tf);
    CHECK(!nan_result.has_value());
    std::cout << "  NaN 变换：报错 —— " << nan_result.error() << "\n";
    CHECK(nan_result.error().find("NaN") != std::string::npos);

    // 非归一化输入会被"自愈"：SO3 构造函数内置 normalize() 门卫
    // （读源码确认的真实行为：SO3(const MatrixType& R) 内部检查 isNormalized()，
    //  不满足就强制归一化并重建 R_/q_；from_quaternion 也强制 normalize()）
    // —— 2 倍缩放的旋转矩阵进来，四元数出来模长恒为 1，validate 通过。
    Eigen::Matrix4d scaled            = 2.0 * Eigen::Matrix4d::Identity();
    const ft::EdgeTransform<ft::gimbal_pitch> healed(scaled);
    CHECK(std::abs(healed.quaternion().norm() - 1.0) < 1e-12);
    CHECK(ft::validate_transform(healed).has_value());
    std::cout << "  自愈演示：2 倍缩放矩阵构造 → 四元数模长 = " << healed.quaternion().norm()
              << "（SO3 构造函数强制归一化），validate 通过\n";

    // 真正会被拦截的是 NaN/Inf：旋转矩阵混入 NaN → validate 报错
    Eigen::Matrix3d R_nan = Eigen::Matrix3d::Identity();
    R_nan(0, 0)           = std::numeric_limits<double>::quiet_NaN();
    const auto nan_rot    = ft::EdgeTransform<ft::gimbal_pitch>::from_rt(
        R_nan, Eigen::Vector3d(0.0, 0.0, 0.1));
    const auto rot_result = ft::validate_transform(nan_rot);
    CHECK(!rot_result.has_value());
    std::cout << "  NaN 旋转：报错 —— " << rot_result.error() << "\n";
    CHECK(rot_result.error().find("NaN") != std::string::npos);

    // Buffer::push 门卫：NaN 变换被静默丢弃，查询报"buffer is empty"
    ft::CoordinateSystem sys;
    ft::update<ft::gimbal_pitch>(sys, nan_tf, 0);
    const auto lookup_result = ft::lookup<ft::odom, ft::gimbal_pitch>(sys, 0);
    CHECK(!lookup_result.has_value());
    std::cout << "  push(NaN) 后 lookup：按预期报错 —— " << lookup_result.error() << "\n";
    CHECK(lookup_result.error().find("empty") != std::string::npos);

    std::cout << "测试5通过\n\n";
}

// ===========================================================================
// 测试6：lookup_clamped —— 树级钳位查询
// 对标 ROS2 TF2 的 TimePointZero 语义：查询超界取边界样本，绝不做外推
// ===========================================================================
static void test_lookup_clamped() {
    std::cout << "=== 测试6：lookup_clamped 钳位查询 ===\n";

    constexpr std::uint64_t t0 = 0;
    constexpr std::uint64_t t1 = 1'000'000'000;
    constexpr std::uint64_t t_beyond = 2'000'000'000;

    ft::CoordinateSystem sys;
    ft::update<ft::gimbal>(sys, ft::EdgeTransform<ft::gimbal>::from_rpy(0.0, 0.0, 0.0), t0);
    ft::update<ft::gimbal>(sys, ft::EdgeTransform<ft::gimbal>::from_rpy(0.0, 0.0, 0.2), t1);

    // clamped：超出最新样本 1s → 钳位到 yaw=0.2，不报错
    const auto clamped = ft::lookup_clamped<ft::odom, ft::gimbal>(sys, t_beyond);
    CHECK(clamped.has_value());
    if (clamped) {
        std::cout << "  lookup_clamped(2s)：yaw = " << clamped->euler_rot().yaw
                  << "（钳位到最新样本，不外推）\n";
        CHECK(std::abs(clamped->euler_rot().yaw - 0.2) < 1e-12);
    }

    // interpolate（lookup 默认）：同一时刻 → 报错，拒绝未来外推
    const auto extrapolate = ft::lookup<ft::odom, ft::gimbal>(sys, t_beyond);
    CHECK(!extrapolate.has_value());
    std::cout << "  lookup(2s)：按预期报错 —— " << extrapolate.error() << "\n";

    // 早于最早样本同理（past extrapolation）
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

    test_frame_tree_init();
    test_compile_time_frame_safety();
    test_chain_lookup_hand_computed();
    test_time_interpolation();
    test_validation_fail_fast();
    test_lookup_clamped();

    if (g_failures == 0) {
        std::cout << "全部测试通过（PASS）\n";
        return 0;
    }
    std::cerr << "共 " << g_failures << " 个断言失败\n";
    return 1;
}
