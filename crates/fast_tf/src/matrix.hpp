#pragma once
// 欧拉角转换工具头文件
#include "euler.hpp"

// Eigen线性代数基础库
#include <Eigen/Core>
#include <Eigen/Geometry>
// STL通用算法
#include <algorithm>
// 断言调试
#include <cassert>
// C++20概念约束，用于identity单位变换编译期校验
#include <concepts>
// 自研SE(3)/SO(3)李群实现
#include <groups/SEn3.hpp>
// OpenCV基础Vec类型，兼容PnP输出rvec/tvec
#include <opencv2/core/types.hpp>
// 类型萃取工具
#include <type_traits>

// 自研轻量坐标变换命名空间 fast_tf
namespace fast_tf {

/**
 * @brief 强类型SE(3)刚体变换类，采用【被动变换/基变换】约定
 *
 * 模板参数说明：
 * T: 浮点标量类型 float/double
 * A: ToTag 目标坐标系（变换后点所属帧）
 * B: FromTag 源坐标系（变换前点所属帧）
 *
 * 变换数学含义：
 * `TransformMatrix<T,A,B>` = T_AB
 * 作用：将 B 坐标系下任意点 P_B 转换为 A 坐标系下点 P_A
 * P_A = T_AB * P_B
 *
 * 代数运算规则（编译期类型校验，非法帧组合直接编译报错）
 * 1. 变换复合：T<A,B> * T<B,C> = T<A,C>  （连续坐标转换 B→C 叠加 A→B 得到 A→C）
 * 2. 变换逆元：inv(T<A,B>) = T<B,A>    反向变换 A→B
 * 3. 单位变换：I<A> = T<A,A>          同一坐标系下无变换
 */
template <typename T, typename A, typename B>
class TransformMatrix {
public:
    // 允许不同帧标签的TransformMatrix互为友元，访问私有SE3成员
    template <typename, typename, typename>
    friend class TransformMatrix;

    // 类型别名封装，统一对外接口，隔离底层李群实现
    using SE3     = group::SEn3<T, 1>;       // 底层SE(3)李群存储容器（旋转+平移）
    using SO3     = group::SO3<T>;           // SO(3)旋转子群
    using Matrix  = typename SE3::MatrixType;// 4x4齐次变换矩阵 Eigen::Matrix4<T>
    using Tangent = typename SE3::VectorType;// 李代数切空间 6维向量 [ωₓ,ωᵧ,ω_z,vₓ,vᵧ,v_z]
    using Scalar  = T;                       // 浮点标量别名
    using FromTag = B;                       // 源坐标系标签别名
    using ToTag   = A;                       // 目标坐标系标签别名

    // 默认构造：初始化为单位变换（内部SE3默认构造为单位元）
    TransformMatrix() = default;

    // 从SE3李群对象构造
    explicit TransformMatrix(const SE3& pose)
        : pose_(pose) {}

    // 移动语义构造，避免SE3拷贝开销
    explicit TransformMatrix(SE3&& pose)
        : pose_(std::move(pose)) {}

    // 直接从4x4齐次变换矩阵构造
    explicit TransformMatrix(const Matrix& t)
        : pose_(t) {}

    // ========================================================================
    // 静态工厂方法：多种输入格式快速生成SE(3)变换
    // ========================================================================

    /**
     * @brief 从旋转向量rvec+平移向量tvec生成变换（OpenCV PnP标准格式）
     * @param rvec 旋转向量，模长为旋转角θ，方向为旋转轴
     * @param tvec 三维平移向量
     * @return T<A,B>变换矩阵
     * @note 手动实现罗德里格斯旋转公式，无需引入opencv/calib3d.hpp，减少编译依赖
     */
    static TransformMatrix
        from_rvec_tvec(const Eigen::Matrix<T, 3, 1>& rvec, const Eigen::Matrix<T, 3, 1>& tvec) {
        const T theta = rvec.norm();
        Eigen::Matrix<T, 3, 3> R;

        // 旋转向量模长趋近0：极小旋转，直接单位矩阵避免除0
        if (theta < T(1e-8)) {
            R = Eigen::Matrix<T, 3, 3>::Identity();
        } else {
            // 单位旋转轴k = rvec / θ
            const Eigen::Matrix<T, 3, 1> k = rvec / theta;
            const T c                      = std::cos(theta);
            const T s                      = std::sin(theta);
            const T omc                    = T(1) - c; // 1-cosθ

            // 罗德里格斯旋转矩阵展开公式
            R(0, 0) = c + k(0) * k(0) * omc;
            R(0, 1) = k(0) * k(1) * omc - k(2) * s;
            R(0, 2) = k(0) * k(2) * omc + k(1) * s;
            R(1, 0) = k(1) * k(0) * omc + k(2) * s;
            R(1, 1) = c + k(1) * k(1) * omc;
            R(1, 2) = k(1) * k(2) * omc - k(0) * s;
            R(2, 0) = k(2) * k(0) * omc - k(1) * s;
            R(2, 1) = k(2) * k(1) * omc + k(0) * s;
            R(2, 2) = c + k(2) * k(2) * omc;
        }

        // 拼接旋转矩阵+平移向量生成变换
        return from_rt(R, tvec);
    }

    /**
     * @brief 兼容OpenCV原生cv::Vec rvec/tvec的包装接口
     */
    static TransformMatrix from_pnp(cv::Vec<T, 3> rvec, cv::Vec<T, 3> tvec) {
        return from_rvec_tvec(
            Eigen::Matrix<T, 3, 1>(rvec[0], rvec[1], rvec[2]),
            Eigen::Matrix<T, 3, 1>(tvec[0], tvec[1], tvec[2]));
    }

    /**
     * @brief 内旋XYZ欧拉角(Roll-Pitch-Yaw)生成变换
     * @param roll 绕X滚转 pitch绕Y俯仰 yaw绕Z偏航
     * @param x,y,z 三维平移分量，默认0
     */
    static TransformMatrix from_rpy(T roll, T pitch, T yaw, T x = T(0), T y = T(0), T z = T(0)) {
        // 欧拉角转四元数，再构造变换
        const auto q = math_fuxk::rpy<T>(roll, pitch, yaw).quat();
        return from_rt(q.toRotationMatrix(), Eigen::Matrix<T, 3, 1>(x, y, z));
    }

    /**
     * @brief 纯平移变换，旋转为单位矩阵
     */
    static TransformMatrix from_translation(T x, T y, T z) {
        return from_rt(Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 1>(x, y, z));
    }

    /**
     * @brief 四元数+平移向量构造变换，自动归一化四元数
     */
    static TransformMatrix
        from_quaternion(
            const Eigen::Quaternion<T>& q_in,
            const Eigen::Matrix<T, 3, 1>& t = Eigen::Matrix<T, 3, 1>::Zero()) {
        Eigen::Quaternion<T> q = q_in;
        q.normalize(); // 强制归一化，避免旋转矩阵退化
        return from_rt(q.toRotationMatrix(), t);
    }

    /**
     * @brief 四元数+分离x/y/z平移标量重载接口
     */
    static TransformMatrix
        from_quaternion_xyz(const Eigen::Quaternion<T>& q, T x = T(0), T y = T(0), T z = T(0)) {
        return from_quaternion(q, Eigen::Matrix<T, 3, 1>(x, y, z));
    }

    /**
     * @brief 底层通用构造接口：3x3旋转矩阵R + 3维平移t → 4x4齐次矩阵
     */
    static TransformMatrix
        from_rt(const Eigen::Matrix<T, 3, 3>& R, const Eigen::Matrix<T, 3, 1>& t) {
        Matrix tt                     = Matrix::Identity(); // 初始4x4单位矩阵
        tt.template block<3, 3>(0, 0) = R;                  // 左上角3x3填充旋转
        tt.template block<3, 1>(0, 3) = t;                   // 第四列前3行填充平移
        return TransformMatrix(tt);
    }

    /**
     * @brief 生成同一坐标系下单位变换 I<A> = T<A,A>
     * @requires A与B必须是同一帧标签，C++20 concept编译期约束，不满足直接编译报错
     */
    static TransformMatrix identity() requires(std::same_as<A, B>) { return TransformMatrix{}; }
    // 大小写别名，兼容不同代码书写习惯
    static TransformMatrix Identity() requires(std::same_as<A, B>) { return identity(); }

    // ========================================================================
    // 类型安全SE(3)代数运算：变换复合、逆变换
    // ========================================================================

    /**
     * @brief 变换乘法复合 T<A,B> * T<B,C> = T<A,C>
     * @param other T<B,C> B→C变换
     * @return T<A,C> A→C复合变换
     * @note 模板参数C自动推导，编译期校验中间帧B匹配，帧不匹配编译报错，杜绝坐标错乱
     */
    template <typename C>
    [[nodiscard]] TransformMatrix<T, A, C> operator*(const TransformMatrix<T, B, C>& other) const {
        return TransformMatrix<T, A, C>(pose_ * other.pose_);
    }

    /**
     * @brief 求逆变换 T<A,B> → T<B,A>
     */
    [[nodiscard]] TransformMatrix<T, B, A> inverse() const {
        return TransformMatrix<T, B, A>(pose_.inv());
    }
    // 简写别名 inv()
    [[nodiscard]] TransformMatrix<T, B, A> inv() const { return inverse(); }

    // ========================================================================
    // 坐标树父级重绑定工具函数
    // ========================================================================

    /**
     * @brief 重绑定变换父坐标系：已知 Parent→A，当前A→B，合成 Parent→B
     * @param parent_tf T<Parent,A> 父帧到A帧变换
     * @return T<Parent,B> 父帧直接到B帧变换
     * 数学等价：parent_tf * (*this)
     */
    template <typename Parent>
    [[nodiscard]] TransformMatrix<T, Parent, B>
        reparent_to(const TransformMatrix<T, Parent, A>& parent_tf) const {
        return parent_tf * *this;
    }

    // ========================================================================
    // 成员访问器：提取底层旋转/平移/矩阵/欧拉角
    // ========================================================================

    // 获取底层完整SE3李群对象
    [[nodiscard]] const SE3& se3() const { return pose_; }
    // 提取SO3旋转子群
    [[nodiscard]] SO3 so3() const { return SO3(pose_.R()); }
    // 获取完整4x4齐次变换矩阵
    [[nodiscard]] Matrix matrix() const { return pose_.T(); }
    // 获取三维平移向量 t
    [[nodiscard]] Eigen::Matrix<T, 3, 1> translation() const { return pose_.x(); }
    // 获取3x3旋转矩阵 R
    [[nodiscard]] Eigen::Matrix<T, 3, 3> rotation() const { return pose_.R(); }
    // 获取单位四元数 q
    [[nodiscard]] Eigen::Quaternion<T> quaternion() const { return pose_.q(); }

    // ========================================================================
    // SE(3)右不变李群运算（机器人SLAM标准右扰动约定）
    // ========================================================================

    /**
     * @brief 右减运算：δ = Log(T₁⁻¹ · T₂)
     * 含义：从当前变换this(T₁)运动到other(T₂)所需的切空间微小扰动（6维李代数）
     */
    [[nodiscard]] Tangent rminus(const TransformMatrix& other) const {
        return SE3::log(pose_.inv() * other.pose_);
    }

    /**
     * @brief 右加运算：T ⊕ δ = T · Exp(δ)
     * 在当前变换右侧叠加切空间微小扰动δ，生成新变换
     */
    [[nodiscard]] TransformMatrix rplus(const Tangent& delta) const {
        return TransformMatrix(pose_ * SE3::exp(delta));
    }

    // 对自身变换取对数映射 SE(3) → 6维李代数
    [[nodiscard]] Tangent log() const { return SE3::log(pose_); }
    // 静态指数映射：6维李代数 → SE(3)变换
    static TransformMatrix exp(const Tangent& xi) { return TransformMatrix(SE3::exp(xi)); }

    // 提取XYZ内旋RPY欧拉角封装对象
    [[nodiscard]] math_fuxk::Ros2EulerRot<T> euler_rot() const {
        return math_fuxk::rpy<T>(quaternion());
    }

    // ========================================================================
    // 插值工具：线性插值 / SE(3)流形测地线插值
    // ========================================================================

    /**
     * @brief 简单线性插值（分离旋转球面插值slerp + 平移线性插值）
     * @param a 起始变换
     * @param b 终止变换
     * @param t 插值系数 [0,1]
     * @note 分开插值旋转和平移，不是严格SE(3)流形插值，计算更快，适合云台平滑调试
     */
    static TransformMatrix lerp(const TransformMatrix& a, const TransformMatrix& b, T t) {
        if (t <= T(0)) {
            return a;
        }
        if (t >= T(1)) {
            return b;
        }
        // 平移直接线性插值
        const Eigen::Matrix<T, 3, 1> pos = a.translation() * (T(1) - t) + b.translation() * t;
        // 四元数球面插值平滑旋转
        const Eigen::Quaternion<T> q     = a.quaternion().slerp(t, b.quaternion());
        return from_quaternion(q, pos);
    }

    /**
     * @brief SE(3)流形测地线插值（严格右不变李群插值）
     * γ(t) = a · Exp( t · Log(a⁻¹ · b) )
     * 符合刚体运动流形几何，插值轨迹是最短测地线，SLAM/滤波预测专用
     */
    static TransformMatrix lerp_se3(const TransformMatrix& a, const TransformMatrix& b, T t) {
        if (t <= T(0)) {
            return a;
        }
        if (t >= T(1)) {
            return b;
        }
        // 计算a到b的相对变换 delta = a⁻¹ * b
        const SE3 delta  = a.pose_.inv() * b.pose_;
        // 映射到李代数
        const auto xi    = SE3::log(delta);
        // 缩放扰动后指数映射，叠加到起始变换a右侧
        const SE3 interp = a.pose_ * SE3::exp(xi * t);
        return TransformMatrix(interp);
    }

private:
    // 底层存储：SE(3)李群对象（封装4x4齐次矩阵、旋转、平移）
    SE3 pose_{};
};

// ========================================================================
// 全局工具函数、类型别名，简化业务代码书写
// ========================================================================

/**
 * @brief 生成指定坐标系F的单位变换 I<F>
 * 示例：I<odom>() 得到 TransformMatrixd<odom, odom>
 */
template <typename Frame, typename Scalar = double>
[[nodiscard]] TransformMatrix<Scalar, Frame, Frame> I() {
    return TransformMatrix<Scalar, Frame, Frame>::identity();
}

/**
 * @brief 全局逆变换简写函数，等价 tf.inv()
 */
template <typename T, typename A, typename B>
[[nodiscard]] TransformMatrix<T, B, A> inv(const TransformMatrix<T, A, B>& tf) {
    return tf.inv();
}

// 长模板别名简化
template <typename T, typename A, typename B>
using TypedTransformMatrix = TransformMatrix<T, A, B>;
// double精度常用别名
template <typename A, typename B>
using TransformMatrixd = TransformMatrix<double, A, B>;
// float精度轻量化别名
template <typename A, typename B>
using TransformMatrixf = TransformMatrix<float, A, B>;

/**
 * @brief 球面坐标结构体 Spherial = {yaw, pitch, distance}
 * 云台自瞄专用：以机体X轴为瞄准基线，球面极坐标描述瞄准点
 * yaw：水平偏航角 pitch：俯仰角 distance：直线距离
 */
template <std::floating_point T>
struct Spherial {
public:
    T yaw{};
    T pitch{};
    T distance{};

    using Vec3 = Eigen::Matrix<T, 3, 1>;

    /**
     * @brief 球面坐标平滑插值
     * 逻辑：旋转SO(3)球面插值 + 空间笛卡尔坐标线性插值距离
     */
    static Spherial<T> lerp(const Spherial<T>& a, const Spherial<T>& b, T t) {
        if (t <= T(0)) {
            return a;
        }
        if (t >= T(1)) {
            return b;
        }
        // 1. 构造a/b的旋转矩阵（仅俯仰+偏航，无滚转）
        const auto Ra = math_fuxk::rpy(T(0), a.pitch, a.yaw).so3();
        const auto Rb = math_fuxk::rpy(T(0), b.pitch, b.yaw).so3();
        // 2. SO(3)流形插值旋转矩阵
        const auto w  = decltype(Ra)::log(Ra.inv() * Rb);
        const auto Rt = Ra * decltype(Ra)::exp(w * t);

        // 机体X轴单位向量，生成瞄准方向向量
        const Vec3 unit_x = Vec3::UnitX();
        const Vec3 dir_a  = Ra * unit_x;
        const Vec3 dir_b  = Rb * unit_x;

        // 缩放距离得到三维空间点
        const Vec3 pos_a = a.distance * dir_a;
        const Vec3 pos_b = b.distance * dir_b;
        // 空间点线性插值
        const Vec3 pos  = (T(1) - t) * pos_a + t * pos_b;
        // 插值后的旋转矩阵转回RPY俯仰偏航
        auto [_r, p, y] = math_fuxk::rpy(Rt).rpy();
        // 插值后距离=插值点模长
        return {.yaw = y, .pitch = p, .distance = pos.norm()};
    }
};

// double/float精度球面坐标简写
using Spheriald = Spherial<double>;
using Spherialf = Spherial<float>;

} // namespace fast_tf