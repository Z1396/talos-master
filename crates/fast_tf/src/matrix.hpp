#pragma once

#include "euler.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cassert>
#include <concepts>
#include <groups/SEn3.hpp>
#include <opencv2/core/types.hpp>
#include <type_traits>

namespace fast_tf {

/// Strongly typed SE(3) transform (passive / change-of-basis convention).
///
/// `TransformMatrix<T, A, B>` represents the passive transform that
/// re-expresses a point from frame B into frame A.
///
/// Algebra:
/// - T<A, B> * T<B, C> :  T<A, C>   (composition)
/// - inv(T<A, B>)      :  T<B, A>    (inverse)
/// - I<A>              :  T<A, A>    (identity)
template <typename T, typename A, typename B>
class TransformMatrix {
public:
    template <typename, typename, typename>
    friend class TransformMatrix;

    using SE3     = group::SEn3<T, 1>;
    using SO3     = group::SO3<T>;
    using Matrix  = typename SE3::MatrixType;
    using Tangent = typename SE3::VectorType;
    using Scalar  = T;
    using FromTag = A;
    using ToTag   = B;

    TransformMatrix() = default;

    explicit TransformMatrix(const SE3& pose)
        : pose_(pose) {}

    explicit TransformMatrix(SE3&& pose)
        : pose_(std::move(pose)) {}

    explicit TransformMatrix(const Matrix& t)
        : pose_(t) {}

    // ========================================================================
    // Factory Methods
    // ========================================================================

    /// Create from rotation vector (rodrigues) + translation vector (OpenCV PnP convention)
    /// Uses a hand-rolled Rodrigues formula to avoid pulling calib3d.hpp into every TU.
    static TransformMatrix
        from_rvec_tvec(const Eigen::Matrix<T, 3, 1>& rvec, const Eigen::Matrix<T, 3, 1>& tvec) {
        const T theta = rvec.norm();
        Eigen::Matrix<T, 3, 3> R;

        if (theta < T(1e-8)) {
            R = Eigen::Matrix<T, 3, 3>::Identity();
        } else {
            const Eigen::Matrix<T, 3, 1> k = rvec / theta;
            const T c                      = std::cos(theta);
            const T s                      = std::sin(theta);
            const T omc                    = T(1) - c;

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

        return from_rt(R, tvec);
    }

    /// Create from OpenCV rvec/tvec. Thin wrapper around from_rvec_tvec.
    static TransformMatrix from_pnp(cv::Vec<T, 3> rvec, cv::Vec<T, 3> tvec) {
        return from_rvec_tvec(
            Eigen::Matrix<T, 3, 1>(rvec[0], rvec[1], rvec[2]),
            Eigen::Matrix<T, 3, 1>(tvec[0], tvec[1], tvec[2]));
    }

    /// Create EulerRot::XYZ intrinsic
    static TransformMatrix from_rpy(T roll, T pitch, T yaw, T x = T(0), T y = T(0), T z = T(0)) {
        const auto q = math_fuxk::rpy<T>(roll, pitch, yaw).quat();
        return from_rt(q.toRotationMatrix(), Eigen::Matrix<T, 3, 1>(x, y, z));
    }

    /// Create pure translation (identity rotation)
    static TransformMatrix from_translation(T x, T y, T z) {
        return from_rt(Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 1>(x, y, z));
    }

    static TransformMatrix from_quaternion(
        const Eigen::Quaternion<T>& q_in,
        const Eigen::Matrix<T, 3, 1>& t = Eigen::Matrix<T, 3, 1>::Zero()) {
        Eigen::Quaternion<T> q = q_in;
        q.normalize();
        return from_rt(q.toRotationMatrix(), t);
    }

    static TransformMatrix
        from_quaternion_xyz(const Eigen::Quaternion<T>& q, T x = T(0), T y = T(0), T z = T(0)) {
        return from_quaternion(q, Eigen::Matrix<T, 3, 1>(x, y, z));
    }

    static TransformMatrix
        from_rt(const Eigen::Matrix<T, 3, 3>& R, const Eigen::Matrix<T, 3, 1>& t) {
        Matrix tt                     = Matrix::Identity();
        tt.template block<3, 3>(0, 0) = R;
        tt.template block<3, 1>(0, 3) = t;
        return TransformMatrix(tt);
    }

    static TransformMatrix identity() requires(std::same_as<A, B>) { return TransformMatrix{}; }

    static TransformMatrix Identity() requires(std::same_as<A, B>) { return identity(); }

    // ========================================================================
    // Type-level Transform Algebra
    // ========================================================================

    /// Composition: T<A,B> * T<B,C> :  T<A,C>
    template <typename C>
    [[nodiscard]] TransformMatrix<T, A, C> operator*(const TransformMatrix<T, B, C>& other) const {
        return TransformMatrix<T, A, C>(pose_ * other.pose_);
    }

    [[nodiscard]] TransformMatrix<T, B, A> inverse() const {
        return TransformMatrix<T, B, A>(pose_.inv());
    }

    [[nodiscard]] TransformMatrix<T, B, A> inv() const { return inverse(); }

    // ========================================================================
    // Compatibility Helpers
    // ========================================================================

    /// reparent_to(T<Parent,A>, T<A,B>) :  T<Parent,B>
    template <typename Parent>
    [[nodiscard]] TransformMatrix<T, Parent, B>
        reparent_to(const TransformMatrix<T, Parent, A>& parent_tf) const {
        return parent_tf * *this;
    }

    // ========================================================================
    // Accessors
    // ========================================================================

    [[nodiscard]] const SE3& se3() const { return pose_; }
    [[nodiscard]] SO3 so3() const { return SO3(pose_.R()); }
    [[nodiscard]] Matrix matrix() const { return pose_.T(); }

    [[nodiscard]] Eigen::Matrix<T, 3, 1> translation() const { return pose_.x(); }

    [[nodiscard]] Eigen::Matrix<T, 3, 3> rotation() const { return pose_.R(); }

    [[nodiscard]] Eigen::Quaternion<T> quaternion() const { return pose_.q(); }

    // ========================================================================
    // Right-invariant Lie ops
    // ========================================================================

    /// Right-minus: δ = Log(this^{-1} · other)
    [[nodiscard]] Tangent rminus(const TransformMatrix& other) const {
        return SE3::log(pose_.inv() * other.pose_);
    }

    /// Right-plus: this ⊕ δ = this · Exp(δ)
    [[nodiscard]] TransformMatrix rplus(const Tangent& delta) const {
        return TransformMatrix(pose_ * SE3::exp(delta));
    }

    [[nodiscard]] Tangent log() const { return SE3::log(pose_); }
    static TransformMatrix exp(const Tangent& xi) { return TransformMatrix(SE3::exp(xi)); }

    [[nodiscard]] math_fuxk::Ros2EulerRot<T> euler_rot() const {
        return math_fuxk::rpy<T>(quaternion());
    }

    // ========================================================================
    // Interpolation
    // ========================================================================

    static TransformMatrix lerp(const TransformMatrix& a, const TransformMatrix& b, T t) {
        if (t <= T(0)) {
            return a;
        }
        if (t >= T(1)) {
            return b;
        }

        const Eigen::Matrix<T, 3, 1> pos = a.translation() * (T(1) - t) + b.translation() * t;
        const Eigen::Quaternion<T> q     = a.quaternion().slerp(t, b.quaternion());
        return from_quaternion(q, pos);
    }

    /// SE(3) geodesic interpolation (right-invariant):
    /// γ(t) = a · Exp(t · Log(a^{-1} · b))
    static TransformMatrix lerp_se3(const TransformMatrix& a, const TransformMatrix& b, T t) {
        if (t <= T(0)) {
            return a;
        }
        if (t >= T(1)) {
            return b;
        }

        const SE3 delta  = a.pose_.inv() * b.pose_;
        const auto xi    = SE3::log(delta);
        const SE3 interp = a.pose_ * SE3::exp(xi * t);
        return TransformMatrix(interp);
    }

private:
    SE3 pose_{};
};

/// Identity transform for frame F: I<F> :  TransformMatrix<Scalar, F, F>
template <typename Frame, typename Scalar = double>
[[nodiscard]] TransformMatrix<Scalar, Frame, Frame> I() {
    return TransformMatrix<Scalar, Frame, Frame>::identity();
}

template <typename T, typename A, typename B>
[[nodiscard]] TransformMatrix<T, B, A> inv(const TransformMatrix<T, A, B>& tf) {
    return tf.inv();
}

template <typename T, typename A, typename B>
using TypedTransformMatrix = TransformMatrix<T, A, B>;

template <typename A, typename B>
using TransformMatrixd = TransformMatrix<double, A, B>;

template <typename A, typename B>
using TransformMatrixf = TransformMatrix<float, A, B>;

template <std::floating_point T>
struct Spherial {
public:
    T yaw{};
    T pitch{};
    T distance{};

    using Vec3 = Eigen::Matrix<T, 3, 1>;

    static Spherial<T> lerp(const Spherial<T>& a, const Spherial<T>& b, T t) {
        if (t <= T(0)) {
            return a;
        }
        if (t >= T(1)) {
            return b;
        }

        const auto Ra = math_fuxk::rpy(T(0), a.pitch, a.yaw).so3();
        const auto Rb = math_fuxk::rpy(T(0), b.pitch, b.yaw).so3();
        const auto w  = decltype(Ra)::log(Ra.inv() * Rb);
        const auto Rt = Ra * decltype(Ra)::exp(w * t);

        const Vec3 unit_x = Vec3::UnitX();
        const Vec3 dir_a  = Ra * unit_x;
        const Vec3 dir_b  = Rb * unit_x;

        const Vec3 pos_a = a.distance * dir_a;
        const Vec3 pos_b = b.distance * dir_b;

        const Vec3 pos  = (T(1) - t) * pos_a + t * pos_b;
        auto [_r, p, y] = math_fuxk::rpy(Rt).rpy();
        return {.yaw = y, .pitch = p, .distance = pos.norm()};
    }
};

using Spheriald = Spherial<double>;
using Spherialf = Spherial<float>;

} // namespace fast_tf
