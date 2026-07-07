#pragma once
// 全局通用基础类型、PnP测量输出结构体 CameraArmorMeasurement
#include "core/types.hpp"
#include "core/types_pnp.hpp"

// C++标准数学/算法容器
#include <algorithm>
#include <cmath>
#include <expected>
#include <limits>
#include <numbers>
// OpenCV 固定尺寸矩阵
#include <opencv2/core/matx.hpp>
#include <optional>
#include <vector>

// Eigen 线性代数库：矩阵、向量、特征值、几何变换
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
// Ceres 非线性最小二乘优化库（BA光束平差）
#include <ceres/ceres.h>
// OpenCV PnP、畸变矫正、投影、旋转矩阵转换
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
// Eigen <-> OpenCV 类型互转
#include <opencv2/core/eigen.hpp>

// 相机内参、畸变系数配置结构体
#include "camera_config.hpp"
// 欧拉角转换工具函数
#include "euler.hpp"

namespace fcs::L2 {

// ============================================================================
// PnP 求解失败错误枚举 + 错误文本转换
// ============================================================================
/**
 * @brief PnP求解器错误类型枚举
 */
enum class PnPError {
    InvalidDetection, ///< 装甲检测无效（角点数量不足、非法坐标）
    SolveFailed,     ///< PnP求解迭代发散、数值NaN、无解
};

/**
 * @brief 将错误枚举转为可读字符串视图，日志打印专用
 * @param e PnP错误枚举
 * @return 静态字符串视图，无堆分配
 */
[[nodiscard]] constexpr std::string_view pnp_error_str(PnPError e) noexcept {
    switch (e) {
    case PnPError::InvalidDetection: return "Invalid detection";
    case PnPError::SolveFailed: return "Solve failed";
    }
    return "Unknown error";
}

// ============================================================================
// 约束重投影误差代价函数（Ceres残差块，4自由度优化）
// 物理模型：仅优化IMU系装甲yaw + 相机系三维平移(yaw/pitch/distance)
// 装甲俯仰角固定由装甲类型决定，不参与优化，降低求解自由度、提升收敛速度
// ============================================================================
/**
 * @brief Ceres残差代价结构体：约束重投影误差BA优化
 * 自由度说明：
 * 1. yaw：IMU世界坐标系装甲绕Z轴偏航角
 * 2. translation_ypd[0]：相机光心到装甲的方位角yaw
 * 3. translation_ypd[1]：相机光心到装甲的俯仰角pitch
 * 4. translation_ypd[2]：相机到装甲直线距离distance
 * 装甲俯仰R_pitch固定，不参与优化，减少优化变量
 */
struct ConstrainedReprojError {
    /**
     * @brief 构造代价函数，预存固定常量，避免每次迭代拷贝
     * @param Pw 装甲模型三维世界点（装甲坐标系）
     * @param uv 图像归一化平面二维观测点（去畸变后）
     * @param R_cam_imu IMU坐标系转相机坐标系旋转矩阵
     * @param R_pitch 装甲固定俯仰旋转矩阵（由装甲类型决定）
     */
    ConstrainedReprojError(
        const Eigen::Vector3d& Pw, const Eigen::Vector2d& uv, const Eigen::Matrix3d& R_cam_imu,
        const Eigen::Matrix3d& R_pitch)
        : Pw_(Pw)
        , uv_(uv)
        , R_cam_imu_(R_cam_imu)
        , R_pitch_(R_pitch) {}

    /**
     * @brief Ceres自动微分残差计算算子模板
     * T为ceres::Jet自动微分类型，同时计算函数值+雅可比矩阵
     * @param yaw 优化变量1：装甲IMU偏航角（1维）
     * @param translation_ypd 优化变量2/3/4：相机平移极坐标参数(yaw,pitch,distance)（3维）
     * @param residuals 输出2维残差：归一化平面x误差、y误差
     * @return false表示该观测无效（点在相机后方/过近，直接丢弃残差）
     */
    template <typename T>
    bool operator()(const T* const yaw, const T* const translation_ypd, T* residuals) const {
        // 1. 构造IMU系装甲yaw旋转矩阵 R_yaw
        const T cy = ceres::cos(yaw[0]);
        const T sy = ceres::sin(yaw[0]);
        Eigen::Matrix<T, 3, 3> R_yaw;
        R_yaw << cy, -sy, T(0), sy, cy, T(0), T(0), T(0), T(1);

        // 2. 转换常量旋转矩阵到Jet自动微分类型
        Eigen::Matrix<T, 3, 3> R_cam_imu = R_cam_imu_.template cast<T>();
        Eigen::Matrix<T, 3, 3> R_pitch   = R_pitch_.template cast<T>();
        // 总旋转：相机到装甲 = R_cam_imu * R_yaw(装甲偏航) * R_pitch(固定俯仰)
        Eigen::Matrix<T, 3, 3> R         = R_cam_imu * R_yaw * R_pitch;

        // 3. 装甲坐标系三维模型点，转为Jet类型
        Eigen::Matrix<T, 3, 1> Pw;
        Pw << T(Pw_.x()), T(Pw_.y()), T(Pw_.z());

        // 4. 极坐标(yaw/pitch/distance)还原相机三维平移向量t
        const T t_yaw      = translation_ypd[0];
        const T t_pitch    = translation_ypd[1];
        const T t_distance = translation_ypd[2];
        const T cp         = ceres::cos(t_pitch);
        Eigen::Matrix<T, 3, 1> t;
        t << t_distance * cp * ceres::sin(t_yaw),
            t_distance * ceres::sin(t_pitch),
            t_distance * cp * ceres::cos(t_yaw);

        // 5. 三维点变换到相机坐标系 Pc = R * Pw + t
        Eigen::Matrix<T, 3, 1> Pc = R * Pw + t;
        const T& Xc               = Pc(0);
        const T& Yc               = Pc(1);
        const T& Zc               = Pc(2);

        // 6. 深度合法性校验：点在相机后方/深度小于1mm，丢弃该残差
        constexpr double kMinDepth = 1e-3; // 1mm最小有效深度
        if (Zc < T(kMinDepth)) {
            return false;
        }

        // 7. 投影到归一化平面，计算残差 = 投影坐标 - 观测uv
        residuals[0] = Xc / Zc - T(uv_.x());
        residuals[1] = Yc / Zc - T(uv_.y());
        return true;
    }

    // ====================================================================
    // Ceres 内存所有权说明（RAII安全无泄漏）
    // Ceres::Problem::AddResidualBlock 会接管CostFunction指针所有权
    // Problem析构时自动delete，业务代码无需手动释放，无内存泄漏、无双重释放
    // ====================================================================
    /**
     * @brief 静态工厂：生成Ceres自动微分代价函数实例
     * 分配堆内存，所有权交给Ceres Problem管理
     * @return ceres::CostFunction* 自动微分残差块，2残差、1参数(yaw)、3参数(ypd)
     */
    static ceres::CostFunction* Create(
        const Eigen::Vector3d& Pw, const Eigen::Vector2d& uv, const Eigen::Matrix3d& R_cam_imu,
        const Eigen::Matrix3d& R_pitch) {
        return new ceres::AutoDiffCostFunction<ConstrainedReprojError, 2, 1, 3>(
            new ConstrainedReprojError(Pw, uv, R_cam_imu, R_pitch));
    }

private:
    Eigen::Vector3d Pw_;      ///< 装甲三维模型点（装甲局部坐标系）
    Eigen::Vector2d uv_;      ///< 归一化平面二维观测角点（去畸变后）
    Eigen::Matrix3d R_cam_imu_; ///< IMU -> 相机旋转矩阵（外参固定）
    Eigen::Matrix3d R_pitch_;  ///< 装甲固定俯仰旋转矩阵（不参与优化）
};

// ============================================================================
// 极坐标<->笛卡尔三维平移向量转换工具函数
// ============================================================================
/**
 * @brief 相机笛卡尔平移向量(x,y,z) 转为极坐标(yaw,pitch,distance)
 * yaw: 水平方位角 atan2(x,z)
 * pitch: 俯仰角 atan2(y, sqrt(x²+z²))
 * distance: 直线模长 ||t||
 * @param translation 相机到装甲三维平移笛卡尔向量
 * @return 三维向量 [yaw, pitch, distance]
 */
[[nodiscard]] inline Eigen::Vector3d
    camera_translation_to_ypd(const Eigen::Vector3d& translation) noexcept {
    const double horizontal = std::hypot(translation.x(), translation.z());
    return {
        std::atan2(translation.x(), translation.z()),
        std::atan2(translation.y(), horizontal),
        translation.norm(),
    };
}

/**
 * @brief 极坐标(yaw,pitch,distance)还原笛卡尔三维平移向量
 * @param translation_ypd [yaw,pitch,distance]极坐标
 * @return 相机到装甲笛卡尔三维平移向量
 */
[[nodiscard]] inline Eigen::Vector3d
    camera_ypd_to_translation(const Eigen::Vector3d& translation_ypd) noexcept {
    const double yaw      = translation_ypd.x();
    const double pitch    = translation_ypd.y();
    const double distance = translation_ypd.z();
    const double cp       = std::cos(pitch);

    return {
        distance * cp * std::sin(yaw),
        distance * std::sin(pitch),
        distance * cp * std::cos(yaw),
    };
}

// ============================================================================
// 装甲俯仰角、旋转矩阵生成工具
// ============================================================================
/**
 * @brief 根据装甲类型获取固定俯仰角弧度
 * 前哨装甲俯仰向下倾斜15°，普通装甲向上倾斜15°
 * @param name 装甲类型枚举
 * @return 俯仰角弧度
 */
[[nodiscard]] inline double armor_pitch_rad_for(ArmorName name) noexcept {
    constexpr double kTiltRad = 15.0 * std::numbers::pi / 180.0;
    return (name == ArmorName::Outpost) ? -kTiltRad : kTiltRad;
}

/**
 * @brief 生成绕Y轴装甲俯仰旋转矩阵
 * @param armor_pitch_rad 俯仰角弧度
 * @return 3×3旋转矩阵 R_pitch
 */
[[nodiscard]] inline Eigen::Matrix3d armor_pitch_rotation(double armor_pitch_rad) noexcept {
    const double cp = std::cos(armor_pitch_rad);
    const double sp = std::sin(armor_pitch_rad);
    Eigen::Matrix3d R_pitch;
    R_pitch << cp, 0.0, sp,
               0.0, 1.0, 0.0,
              -sp, 0.0, cp;
    return R_pitch;
}

/**
 * @brief 生成绕Z轴yaw偏航旋转矩阵
 * @param yaw 偏航角弧度
 * @return 3×3旋转矩阵 R_yaw
 */
[[nodiscard]] inline Eigen::Matrix3d yaw_rotation(double yaw) noexcept {
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    Eigen::Matrix3d R_yaw;
    R_yaw << cy, -sy, 0.0,
             sy, cy, 0.0,
            0.0, 0.0, 1.0;
    return R_yaw;
}

// ============================================================================
// 角点偏差去偏工具：消除灯条角点向内收缩系统误差
// ============================================================================
/**
 * @brief 检测器输出角点存在向内收缩系统偏差，向外径向偏移修正
 * 修正后消除尺度相关系统误差，提升PnP精度
 * @param img_points 原始4个装甲角点像素坐标
 * @return 修正后角点数组
 */
[[nodiscard]] inline std::vector<cv::Point2f>
    debias_correlated_corner_scale(std::vector<cv::Point2f> img_points) noexcept {
    // 仅4角装甲有效，非4点直接返回原数据
    if (img_points.size() != 4) {
        return img_points;
    }

    // 计算四个角点几何中心
    cv::Point2f center(0.0f, 0.0f);
    for (const auto& p : img_points) {
        center += p;
    }
    center *= 0.25f;

    // 固定0.65像素径向向外偏移，补偿检测器角点内缩误差
    constexpr float kCorrelatedScaleDebiasPx = 0.65f;
    for (auto& p : img_points) {
        const cv::Point2f radial = p - center;
        const float norm         = std::sqrt(radial.x * radial.x + radial.y * radial.y);
        // 模长合法时向外偏移
        if (std::isfinite(norm) && norm > 1e-6f) {
            p += (kCorrelatedScaleDebiasPx / norm) * radial;
        }
    }
    return img_points;
}

// ============================================================================
// 重投影雅可比矩阵计算：误差对4维优化变量的导数
// x = [armor_yaw, bearing_yaw, bearing_pitch, log_distance]
// 输出8×4矩阵：4个角点，每个点2维(u,v)残差，共8行；4个优化变量4列
// ============================================================================
[[nodiscard]] static Eigen::Matrix<double, 8, 4> compute_reprojection_jacobian_x(
    const std::vector<Eigen::Vector3d>& obj_pts, const Eigen::Matrix3d& R_cam_imu,
    const Eigen::Matrix3d& R_pitch, double armor_yaw, const Eigen::Vector3d& ypd) noexcept {
    Eigen::Matrix<double, 8, 4> J;
    J.setZero();
    // 仅支持4角装甲，点数量非法填充NaN
    if (obj_pts.size() != 4) {
        J.setConstant(std::numeric_limits<double>::quiet_NaN());
        return J;
    }

    const double r = armor_yaw;
    const double a = ypd.x();    // bearing yaw
    const double b = ypd.y();    // bearing pitch
    const double d = ypd.z();    // distance
    // 距离数值非法，填充NaN
    if (!(d > 1e-9) || !std::isfinite(d)) {
        J.setConstant(std::numeric_limits<double>::quiet_NaN());
        return J;
    }
    const double cr = std::cos(r);
    const double sr = std::sin(r);

    // 装甲yaw旋转矩阵 + 对yaw导数矩阵
    Eigen::Matrix3d R_yaw;
    R_yaw << cr, -sr, 0.0, sr, cr, 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix3d dR_yaw_dr;
    dR_yaw_dr << -sr, -cr, 0.0, cr, -sr, 0.0, 0.0, 0.0, 0.0;

    // 总旋转矩阵 + 对装甲yaw导数
    const Eigen::Matrix3d R        = R_cam_imu * R_yaw * R_pitch;
    const Eigen::Matrix3d dR_dr    = R_cam_imu * dR_yaw_dr * R_pitch;
    // 平移t对ypd三个参数的偏导
    const double sa                = std::sin(a);
    const double ca                = std::cos(a);
    const double sb                = std::sin(b);
    const double cb                = std::cos(b);
    const Eigen::Vector3d dt_da    = {d * cb * ca, 0.0, -d * cb * sa};
    const Eigen::Vector3d dt_db    = {-d * sb * sa, d * cb, -d * sb * ca};
    const Eigen::Vector3d dt_dlogd = {d * cb * sa, d * sb, d * cb * ca};

    // 逐点计算雅可比块，每个角点2行
    for (int i = 0; i < 4; ++i) {
        const Eigen::Vector3d& Pw = obj_pts[static_cast<size_t>(i)];
        const Eigen::Vector3d Pc  = R * Pw + Eigen::Vector3d{d * cb * sa, d * sb, d * cb * ca};
        const double X            = Pc.x();
        const double Y            = Pc.y();
        const double Z            = Pc.z();
        // 深度非法，整矩阵置NaN
        if (!(Z > 1e-9) || !Pc.allFinite()) {
            J.setConstant(std::numeric_limits<double>::quiet_NaN());
            return J;
        }

        // 投影函数雅可比：d(u,v)/d(Xc,Yc,Zc)
        Eigen::Matrix<double, 2, 3> J_proj;
        J_proj << 1.0 / Z, 0.0, -X / (Z * Z),
                  0.0 / Z, 1.0 / Z, -Y / (Z * Z);

        // 填充4列导数块：dyaw / dya / dyb / dlogd
        J.block<2, 1>(2 * i, 0) = J_proj * (dR_dr * Pw);
        J.block<2, 1>(2 * i, 1) = J_proj * dt_da;
        J.block<2, 1>(2 * i, 2) = J_proj * dt_db;
        J.block<2, 1>(2 * i, 3) = J_proj * dt_dlogd;
    }
    return J;
}

// ============================================================================
// PnP几何观测信息结构体：协方差矩阵、条件数（评估求解几何质量）
// ============================================================================
struct PnpGeometryInfo {
    // 4维优化变量[装甲yaw, bearing yaw, bearing pitch, log距离]协方差矩阵
    Eigen::Matrix4d cov_ypdr{Eigen::Matrix4d::Identity() * 1e6};
    // 海森矩阵条件数，越大几何退化越严重（四角共面/近距离）
    double condition_number{1e6};
};

// ============================================================================
// SPD半正定矩阵伪逆（特征值截断，过滤退化特征方向）
// ============================================================================
/**
 * @brief 对称半正定矩阵特征分解伪逆，截断过小特征值避免数值奇异
 * @tparam N 矩阵维度
 * @param H 输入N×N对称半正定海森矩阵
 * @param relative_threshold 特征值相对截断阈值
 * @return 稳定伪逆矩阵，退化特征值替换大值1e6
 */
template <int N>
[[nodiscard]] static Eigen::Matrix<double, N, N> pseudo_inverse_spd(
    const Eigen::Matrix<double, N, N>& H, double relative_threshold = 1e-9) noexcept {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N>> es(H);
    // 特征分解失败/矩阵含NaN，返回单位矩阵放大
    if (es.info() != Eigen::Success || !H.allFinite()) {
        return Eigen::Matrix<double, N, N>::Identity() * 1e6;
    }

    const auto evals        = es.eigenvalues();
    const auto evecs        = es.eigenvectors();
    const double lambda_max = std::max(evals.maxCoeff(), 1e-18);
    const double threshold  = relative_threshold * lambda_max;

    // 构造特征值逆对角矩阵，小于阈值置1e6
    Eigen::Matrix<double, N, N> D_inv = Eigen::Matrix<double, N, N>::Zero();
    for (int i = 0; i < N; ++i) {
        D_inv(i, i) = (evals(i) > threshold) ? 1.0 / evals(i) : 1e6;
    }

    // 特征分解逆公式：V * D_inv * V^T
    return evecs * D_inv * evecs.transpose();
}

// ============================================================================
// 距离参数化枚举：原始距离 / log距离（优化时使用log提升数值稳定性）
// ============================================================================
enum class RangeParam {
    Distance,    ///< 直接使用原始直线距离d
    LogDistance, ///< 使用log(d)作为优化变量，远距离数值更稳定
};

// ============================================================================
// 计算PnP优化后几何信息：协方差、条件数、融合角点尺度相关噪声
// ============================================================================
[[nodiscard]] static PnpGeometryInfo compute_pnp_geometry_info_ypdr(
    const std::vector<Eigen::Vector3d>& obj_pts, const Eigen::Matrix3d& R_cam_imu,
    const Eigen::Matrix3d& R_pitch, double armor_yaw, const Eigen::Vector3d& refined_ypd,
    const std::vector<Eigen::Vector2d>& img_pts, double residual_variance,
    double correlated_scale_variance,
    RangeParam output_range_param = RangeParam::Distance) noexcept {
    PnpGeometryInfo info;
    // 1. 计算重投影雅可比矩阵
    const Eigen::Matrix<double, 8, 4> J =
        compute_reprojection_jacobian_x(obj_pts, R_cam_imu, R_pitch, armor_yaw, refined_ypd);
    if (!J.allFinite()) {
        return info;
    }

    // 2. 海森矩阵 H = J^T J，求伪逆得到基础协方差
    const Eigen::Matrix4d H     = J.transpose() * J;
    const Eigen::Matrix4d H_inv = pseudo_inverse_spd<4>(H);

    // 基础协方差 = H_inv × 残差方差
    Eigen::Matrix4d R_ayplogd = H_inv * std::max(residual_variance, 1e-12);
    R_ayplogd                 = 0.5 * (R_ayplogd + R_ayplogd.transpose());

    // 3. 融合角点尺度相关噪声（径向缩放统一误差）
    if (img_pts.size() == 4 && std::isfinite(correlated_scale_variance)
        && correlated_scale_variance > 0.0) {
        // 计算图像点几何中心
        Eigen::Vector2d center = Eigen::Vector2d::Zero();
        for (const auto& p : img_pts) {
            center += p;
        }
        center *= 0.25;

        // 尺度扰动模式向量：沿径向向外缩放单位向量
        Eigen::Matrix<double, 8, 1> scale_mode;
        scale_mode.setZero();

        bool valid = true;
        for (int i = 0; i < 4; ++i) {
            const Eigen::Vector2d radial = img_pts[static_cast<size_t>(i)] - center;
            const double norm            = radial.norm();

            if (!std::isfinite(norm) || norm < 1e-12) {
                valid = false;
                break;
            }
            scale_mode.template segment<2>(2 * i) = radial / norm;
        }

        // 尺度噪声外加到协方差矩阵
        if (valid && scale_mode.allFinite()) {
            const Eigen::Vector4d dx = H_inv * J.transpose() * scale_mode;
            if (dx.allFinite()) {
                R_ayplogd += correlated_scale_variance * (dx * dx.transpose());
                R_ayplogd = 0.5 * (R_ayplogd + R_ayplogd.transpose());
            }
        }
    }

    // 4. 变量重排序映射矩阵，调整变量输出顺序
    Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
    P(0, 1)           = 1.0;
    P(1, 2)           = 1.0;
    P(2, 3)           = 1.0;
    P(3, 0)           = 1.0;
    Eigen::Matrix4d R = P * R_ayplogd * P.transpose();

    // 5. log距离变量还原为原始距离协方差
    if (output_range_param == RangeParam::Distance) {
        const double d = refined_ypd.z();
        if (std::isfinite(d) && d > 1e-9) {
            Eigen::Matrix4d S = Eigen::Matrix4d::Identity();
            S(2, 2)           = d; // logd → d 线性变换系数
            R                 = S * R * S.transpose();
        } else {
            return info;
        }
    }

    // 强制对称，消除数值不对称误差
    R = 0.5 * (R + R.transpose());

    // 6. 特征分解修正负特征值（保证协方差半正定）
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> psd(R);
    if (psd.info() == Eigen::Success && R.allFinite()) {
        Eigen::Vector4d evals = psd.eigenvalues();
        Eigen::Matrix4d evecs = psd.eigenvectors();
        // 截断极小负特征值为极小正数
        for (int i = 0; i < 4; ++i) {
            evals(i) = std::max(evals(i), 1e-15);
        }
        R = evecs * evals.asDiagonal() * evecs.transpose();
        R = 0.5 * (R + R.transpose());
    }

    info.cov_ypdr = R;

    // 7. 计算海森矩阵条件数，评估几何优劣
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(H);
    if (es.info() == Eigen::Success && H.allFinite()) {
        const auto evals        = es.eigenvalues();
        const double lambda_min = std::max(evals.minCoeff(), 1e-18);
        const double lambda_max = std::max(evals.maxCoeff(), lambda_min);
        info.condition_number   = std::clamp(lambda_max / lambda_min, 1.0, 1e6);
    }

    return info;
}

// ============================================================================
// 从相机-装甲旋转矩阵提取IMU系装甲yaw偏航角
// ============================================================================
[[nodiscard]] inline double extract_yaw_from_rotation(
    const Eigen::Matrix3d& R_cam_armor, const Eigen::Matrix3d& R_imu_cam) noexcept {
    // 总旋转：IMU -> 装甲 = IMU->相机 × 相机->装甲
    const Eigen::Matrix3d R_imu_armor = R_imu_cam * R_cam_armor;

    // 数值裁剪，过滤PnP旋转矩阵非正交微小噪声
    const double r01   = std::clamp(-R_imu_armor(0, 1), -1.0, 1.0);
    const double r11   = std::clamp(R_imu_armor(1, 1), -1.0, 1.0);
    const double yaw_s = std::asin(r01);
    const double yaw_c = std::acos(r11);

    // 象限判断，输出正确[-pi,pi]yaw
    if (std::abs(yaw_s) > 1e-5) {
        return (yaw_s > 0.0) ? yaw_c : -yaw_c;
    }
    return (r11 > 0.0) ? 0.0 : std::numbers::pi;
}

// ============================================================================
// PnP求解器顶层类：IPPE初解 + Ceres约束BA精修，输出带协方差的装甲位姿测量
// ============================================================================
class PnPSolver {
public:
    /**
     * @brief PnP求解输出结果：装甲位姿测量结构体，失败返回PnPError
     */
    using PoseResult = std::expected<CameraArmorMeasurement, PnPError>;

    /**
     * @brief 位姿先验结构体：时序上一帧PnP结果，用于迭代初值加速收敛
     */
    struct PosePrior {
        cv::Vec3d rvec{};       ///< 先验旋转向量
        cv::Vec3d tvec{};       ///< 先验平移向量
        double hint_cost{0.0};  ///< 先验代价权重
        int armor_id{-1};       ///< 对应装甲ID
    };

    /**
     * @brief 构造PnP求解器，传入相机内参畸变，预生成大小装甲三维模型点
     * @param config 相机标定配置（内参、畸变、分辨率）
     */
    explicit PnPSolver(const CameraConfig& config) {
        // Eigen内参/畸变矩阵转OpenCV Mat
        cv::eigen2cv(config.camera_matrix, camera_matrix_);
        cv::eigen2cv(config.distort_coefficient, dist_coeffs_);

        // 图像中心像素坐标，用于计算装甲中心距离画面中心
        image_center_ = cv::Point2f(
            config.width / 2.0,
            config.height / 2.0
        );

        // 预构建大小装甲三维模型坐标
        build_model_points();
    }

    /**
     * @brief 单装甲位姿求解：IPPE粗解 + Ceres 4自由度约束BA精修
     * @param detection 单帧装甲检测结果（4个角点）
     * @param R_imu_cam IMU坐标系转相机坐标系外参旋转矩阵
     * @param timestamp_ns 图像采集纳秒时间戳
     * @param pose_priors 时序先验位姿，加速收敛
     * @return 带协方差、条件数的装甲位姿测量结果 / 求解错误
     */
    [[nodiscard]] PoseResult solve_with_ba(
        const ArmorDetection& detection, const Eigen::Matrix3d& R_imu_cam, uint64_t timestamp_ns,
        const std::vector<PosePrior>& pose_priors = {}) const {
        // 1. 预处理检测角点：去偏、去畸变归一化，生成PnP输入
        auto input = make_pnp_input(detection);
        if (!input) {
            return std::unexpected(input.error());
        }

        // 2. 求解初始粗位姿（优先使用时序先验，无先验则IPPE）
        const auto initial_pose = solve_initial_pose(*input, pose_priors);
        if (!initial_pose.has_value()) {
            return std::unexpected(PnPError::SolveFailed);
        }
        cv::Mat rvec = initial_pose->rvec.clone();
        cv::Mat tvec = initial_pose->tvec.clone();

        // 3. OpenCV旋转向量转Eigen旋转矩阵（相机→装甲）
        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);
        Eigen::Matrix3d R_cam_armor;
        cv::cv2eigen(R_cv, R_cam_armor);
        Eigen::Vector3d t_cam_armor(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

        // 4. 粗解数值合法性校验，NaN/Inf直接判定求解失败
        {
            const double* rd = rvec.ptr<double>();
            const double* td = tvec.ptr<double>();
            for (int i = 0; i < 3; ++i) {
                if (!std::isfinite(rd[i]) || !std::isfinite(td[i])) {
                    return std::unexpected(PnPError::SolveFailed);
                }
            }
        }

        // 5. 构建约束优化固定常量
        const Eigen::Matrix3d R_cam_imu = R_imu_cam.transpose();
        const double armor_pitch_rad    = armor_pitch_rad_for(detection.name);
        const Eigen::Matrix3d R_pitch   = armor_pitch_rotation(armor_pitch_rad);
        // 从粗解提取初始装甲yaw
        double yaw                      = extract_yaw_from_rotation(R_cam_armor, R_imu_cam);
        // 粗解平移转为极坐标ypd作为优化初始值
        const Eigen::Vector3d initial_translation_ypd = camera_translation_to_ypd(t_cam_armor);
        double translation_ypd[3]                     = {
            initial_translation_ypd.x(),
            initial_translation_ypd.y(),
            initial_translation_ypd.z(),
        };

        // 6. 转换三维模型点、归一化二维观测点为Eigen格式，供Ceres使用
        std::vector<Eigen::Vector3d> obj_pts_eigen;
        obj_pts_eigen.reserve(input->obj_points.size());
        for (const auto& p : input->obj_points) {
            obj_pts_eigen.emplace_back(p.x, p.y, p.z);
        }
        std::vector<Eigen::Vector2d> img_pts_eigen;
        img_pts_eigen.reserve(input->norm_points.size());
        for (const auto& p : input->norm_points) {
            img_pts_eigen.emplace_back(p.x, p.y);
        }

        // 7. 构建Ceres最小二乘优化问题
        ceres::Problem problem;
        // 为4个角点逐个添加重投影残差块
        for (size_t i = 0; i < obj_pts_eigen.size(); ++i) {
            ceres::CostFunction* cost = ConstrainedReprojError::Create(
                obj_pts_eigen[i], img_pts_eigen[i], R_cam_imu, R_pitch);
            problem.AddResidualBlock(cost, nullptr, &yaw, translation_ypd);
        }

        // 8. 设置优化变量边界约束，防止数值发散
        // 装甲yaw [-pi, pi]
        problem.SetParameterLowerBound(&yaw, 0, -std::numbers::pi);
        problem.SetParameterUpperBound(&yaw, 0, std::numbers::pi);
        // 相机方位/俯仰角限制在前半球 ±90°
        constexpr double kBearingLimit = std::numbers::pi / 2.0 - 1e-6;
        problem.SetParameterLowerBound(translation_ypd, 0, -kBearingLimit);
        problem.SetParameterUpperBound(translation_ypd, 0, kBearingLimit);
        problem.SetParameterLowerBound(translation_ypd, 1, -kBearingLimit);
        problem.SetParameterUpperBound(translation_ypd, 1, kBearingLimit);

        // 9. Ceres求解器配置：稠密QR分解，最大20迭代，关闭打印
        ceres::Solver::Options options;
        options.linear_solver_type           = ceres::DENSE_QR;
        options.max_num_iterations           = 20;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // 10. 优化收敛后还原精修后的位姿
        const Eigen::Vector3d refined_translation = camera_ypd_to_translation(
            Eigen::Vector3d{translation_ypd[0], translation_ypd[1], translation_ypd[2]});
        const bool translation_finite =
            std::isfinite(translation_ypd[0]) && std::isfinite(translation_ypd[1])
            && std::isfinite(translation_ypd[2]) && refined_translation.allFinite()
            && refined_translation.z() > 1e-3;
        // 优化有效，覆盖rvec/tvec为精修位姿；失败保留原始粗解
        if (summary.IsSolutionUsable() && std::isfinite(yaw) && translation_finite) {
            const Eigen::Matrix3d R_cam_armor_refined = R_cam_imu * yaw_rotation(yaw) * R_pitch;
            cv::Mat R_refined_cv;
            cv::eigen2cv(R_cam_armor_refined, R_refined_cv);
            cv::Rodrigues(R_refined_cv, rvec);
            tvec =
                (cv::Mat_<double>(3, 1) << refined_translation.x(), refined_translation.y(),
                 refined_translation.z());
        }

        // 11. 生成基础位姿测量结构体
        auto measurement =
            make_camera_measurement(detection, rvec, tvec, timestamp_ns, input->dist_to_center);

        // 12. 优化收敛，计算PnP几何协方差、条件数存入测量结果
        if (summary.IsSolutionUsable() && std::isfinite(yaw) && translation_finite) {
            const Eigen::Vector3d refined_ypd{
                translation_ypd[0],
                translation_ypd[1],
                translation_ypd[2],
            };
            // 计算归一化平面重投影RMSE
            const double final_rmse    = normalized_reprojection_rmse(*input, rvec, tvec);
            const double point_count   = static_cast<double>(obj_pts_eigen.size());
            const double residual_dim  = 2.0 * point_count;
            constexpr double kParamDim = 4.0;
            const double dof           = std::max(1.0, residual_dim - kParamDim);
            // 由残差计算像素归一化方差
            const double sigma2_from_residual =
                std::isfinite(final_rmse) ? final_rmse * final_rmse * point_count / dof : 0.0;
            const double fx    = camera_matrix_.at<double>(0, 0);
            const double fy    = camera_matrix_.at<double>(1, 1);
            const double focal = std::sqrt(std::max(1e-9, fx * fy));
            // 独立角点噪声、尺度相关噪声系数（前哨装甲放大噪声权重）
            auto distance_factor = 0.25 / 5.0;
            if (detection.name == ArmorName::Outpost) {
                distance_factor = 4.0 / 5.0;
            }
            const double kIndependentCornerSigmaPxFloor =
                distance_factor * translation_ypd[2] + 0.5;
            const auto kCorrelatedScaleSigmaPx = distance_factor * translation_ypd[2] + 1.0;
            const double sigma_norm_floor      = kIndependentCornerSigmaPxFloor / focal;
            const double sigma_norm_scale      = kCorrelatedScaleSigmaPx / focal;
            const double residual_variance =
                std::max(sigma2_from_residual, sigma_norm_floor * sigma_norm_floor);
            // 计算几何信息（协方差、条件数）
            const PnpGeometryInfo pnp_geometry = compute_pnp_geometry_info_ypdr(
                obj_pts_eigen, R_cam_imu, R_pitch, yaw, refined_ypd, img_pts_eigen,
                residual_variance, sigma_norm_scale * sigma_norm_scale);
            measurement.pnp_cov_ypdr         = pnp_geometry.cov_ypdr;
            measurement.pnp_condition_number = pnp_geometry.condition_number;
        }
        return measurement;
    }

    /**
     * @brief 批量多装甲并行求解BA PnP
     * @param detections 一帧内多个装甲检测结果数组
     * @param R_imu_cam IMU→相机外参旋转矩阵
     * @param timestamp_ns 图像时间戳
     * @return 所有求解成功的装甲测量数组
     */
    [[nodiscard]] std::vector<CameraArmorMeasurement> solve_batch_with_ba(
        const std::vector<ArmorDetection>& detections, const Eigen::Matrix3d& R_imu_cam,
        uint64_t timestamp_ns) const {
        std::vector<CameraArmorMeasurement> measurements;
        measurements.reserve(detections.size());

        for (const auto& det : detections) {
            auto result = solve_with_ba(det, R_imu_cam, timestamp_ns);
            // 仅保留求解成功的装甲测量
            if (result) {
                measurements.push_back(std::move(*result));
            }
        }
        return measurements;
    }

private:
    /**
     * @brief 存储单次PnP求解结果结构体：旋转、平移、重投影误差、先验信息
     */
    struct SolvePose {
        cv::Mat rvec;
        cv::Mat tvec;
        double reprojection_rmse{std::numeric_limits<double>::infinity()};
        std::optional<double> prior_armor_yaw{};
        std::optional<double> prior_distance_m{};
    };

    /**
     * @brief PnP预处理输入结构体：模型三维点、去偏图像点、归一化平面点、中心距离
     */
    struct PnPInput {
        const std::vector<cv::Point3f>& obj_points;
        std::vector<cv::Point2f> img_points;
        std::vector<cv::Point2f> norm_points;
        float dist_to_center;
    };

    /**
     * @brief 生成3×3单位相机矩阵（归一化平面投影使用，无内参畸变）
     */
    [[nodiscard]] static cv::Mat identity_camera_matrix() { return cv::Mat::eye(3, 3, CV_64F); }

    /**
     * @brief 校验旋转/平移向量数值全部有限，无NaN/Inf
     */
    [[nodiscard]] static bool pose_is_finite(const cv::Mat& rvec, const cv::Mat& tvec) {
        const double* rd = rvec.ptr<double>();
        const double* td = tvec.ptr<double>();
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(rd[i]) || !std::isfinite(td[i])) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief 计算归一化平面重投影RMSE（无内参，仅用于评估PnP粗解质量）
     * @param input PnP预处理输入
     * @param rvec 旋转向量
     * @param tvec 平移向量
     * @return 均方根重投影误差
     */
    [[nodiscard]] static double normalized_reprojection_rmse(
        const PnPInput& input, const cv::Mat& rvec, const cv::Mat& tvec) {
        std::vector<cv::Point2f> projected;
        // 归一化平面投影，使用单位内参矩阵
        cv::projectPoints(
            input.obj_points, rvec, tvec, identity_camera_matrix(), cv::Mat{}, projected);
        if (projected.size() != input.norm_points.size()) {
            return std::numeric_limits<double>::infinity();
        }

        double sum_sq = 0.0;
        for (size_t i = 0; i < projected.size(); ++i) {
            const cv::Point2f d = projected[i] - input.norm_points[i];
            sum_sq += static_cast<double>(d.x) * static_cast<double>(d.x)
                    + static_cast<double>(d.y) * static_cast<double>(d.y);
        }
        return std::sqrt(sum_sq / static_cast<double>(projected.size()));
    }

    /**
     * @brief IPPE快速PnP求解，生成无先验初始粗位姿
     */
    [[nodiscard]] std::optional<SolvePose> solve_with_ippe(const PnPInput& input) const {
        cv::Mat rvec, tvec;
        // SOLVEPNP_IPPE 四点高速解析PnP
        const bool success = cv::solvePnP(
            input.obj_points, input.norm_points, identity_camera_matrix(), cv::Mat{}, rvec, tvec,
            false, cv::SOLVEPNP_IPPE);
        if (!success || !pose_is_finite(rvec, tvec)) {
            return std::nullopt;
        }
        const double reprojection_rmse = normalized_reprojection_rmse(input, rvec, tvec);
        return SolvePose{
            .rvec              = std::move(rvec),
            .tvec              = std::move(tvec),
            .reprojection_rmse = reprojection_rmse,
        };
    }

    /**
     * @brief 使用时序先验位姿迭代PnP，提升初解精度
     * @param input PnP预处理输入
     * @param pose_priors 时序历史位姿先验数组
     * @return 最优先验PnP结果（重投影误差最小）
     */
    [[nodiscard]] std::optional<SolvePose>
        solve_with_priors(const PnPInput& input, const std::vector<PosePrior>& pose_priors) const {
        if (pose_priors.empty()) {
            return std::nullopt;
        }
        // 先获取IPPE基础距离用于排序先验
        auto ippe = solve_with_ippe(input);
        if (!ippe) {
            return std::nullopt;
        }
        auto norm = cv::norm(ippe->tvec);

        // 按先验距离与当前粗解距离差值从小到大排序
        std::vector<PosePrior> sorted_priors = pose_priors;
        std::sort(
            sorted_priors.begin(), sorted_priors.end(),
            [norm](const PosePrior& a, const PosePrior& b) {
                return std::abs(norm - cv::norm(a.tvec)) < std::abs(norm - cv::norm(b.tvec));
            });

        std::optional<SolvePose> best;
        // 遍历排序后先验，作为迭代PnP初始值求解
        for (const auto& prior : sorted_priors) {
            const double prior_distance = std::sqrt(
                prior.tvec[0] * prior.tvec[0] + prior.tvec[1] * prior.tvec[1]
                + prior.tvec[2] * prior.tvec[2]);
            if (!std::isfinite(prior_distance) || prior_distance <= 1e-3) {
                continue;
            }

            cv::Mat rvec = (cv::Mat_<double>(3, 1) << prior.rvec[0], prior.rvec[1], prior.rvec[2]);
            cv::Mat tvec = (cv::Mat_<double>(3, 1) << prior.tvec[0], prior.tvec[1], prior.tvec[2]);
            // SOLVEPNP_ITERATIVE 带初值迭代优化
            const bool success = cv::solvePnP(
                input.obj_points, input.norm_points, identity_camera_matrix(), cv::Mat{}, rvec,
                tvec, true, cv::SOLVEPNP_ITERATIVE);
            if (!success || !pose_is_finite(rvec, tvec)) {
                continue;
            }

            const double reprojection_rmse = normalized_reprojection_rmse(input, rvec, tvec);
            if (!std::isfinite(reprojection_rmse)) {
                continue;
            }
            // 转换先验旋转矩阵提取yaw
            Eigen::Matrix3d x;
            cv::Mat xx;
            cv::Rodrigues(prior.rvec, xx);
            cv::cv2eigen(xx, x);
            // 保留误差最小的位姿作为最优解
            if (!best.has_value() || reprojection_rmse < best->reprojection_rmse) {
                best = SolvePose{
                    .rvec              = std::move(rvec),
                    .tvec              = std::move(tvec),
                    .reprojection_rmse = reprojection_rmse,
                    .prior_armor_yaw   = math_fuxk::rpy(x).yaw,
                    .prior_distance_m  = prior_distance,
                };
            }
        }
        return best;
    }

    /**
     * @brief 生成初始粗解：优先时序先验，无先验使用IPPE解析解
     */
    [[nodiscard]] std::optional<SolvePose>
        solve_initial_pose(const PnPInput& input, const std::vector<PosePrior>& pose_priors) const {
        const auto prior_pose = solve_with_priors(input, pose_priors);
        if (prior_pose.has_value()) {
            return prior_pose;
        }
        return solve_with_ippe(input);
    }

    /**
     * @brief 装甲检测原始数据预处理：角点去偏、去畸变归一化、匹配大小装甲模型点
     * @param detection 原始装甲检测结果
     * @return 合法PnP输入结构体 / InvalidDetection错误
     */
    [[nodiscard]] std::expected<PnPInput, PnPError>
        make_pnp_input(const ArmorDetection& detection) const {
        // 角点径向去偏修正系统误差
        auto img_points = debias_correlated_corner_scale(detection.image_points());
        // 必须4个角点，否则检测无效
        if (img_points.size() != 4) {
            return std::unexpected(PnPError::InvalidDetection);
        }

        // 畸变矫正，投影至归一化平面
        std::vector<cv::Point2f> norm_points;
        cv::undistortPoints(img_points, norm_points, camera_matrix_, dist_coeffs_);
        if (norm_points.size() != img_points.size()) {
            return std::unexpected(PnPError::InvalidDetection);
        }

        // 根据装甲类型选择对应三维模型点
        const auto& obj_points =
            (detection.type == ArmorType::Large) ? large_armor_points_ : small_armor_points_;

        return PnPInput{
            .obj_points     = obj_points,
            .img_points     = std::move(img_points),
            .norm_points    = std::move(norm_points),
            .dist_to_center = static_cast<float>(cv::norm(detection.center() - image_center_)),
        };
    }

    /**
     * @brief 预构建大小装甲三维模型坐标（单位米，装甲局部坐标系）
     */
    void build_model_points() {
        constexpr double SMALL_ARMOR_WIDTH  = 135.0 / 1000.0;
        constexpr double SMALL_ARMOR_HEIGHT = 55.0 / 1000.0;
        constexpr double LARGE_ARMOR_WIDTH  = 230.0 / 1000.0;
        constexpr double LARGE_ARMOR_HEIGHT = 55.0 / 1000.0;

        // Small armor: 135mm x 55mm
        const float sw      = static_cast<float>(SMALL_ARMOR_WIDTH / 2.0);
        const float sh      = static_cast<float>(SMALL_ARMOR_HEIGHT / 2.0);
        small_armor_points_ = {
            cv::Point3f(0, sw, sh),   // Top-left
            cv::Point3f(0, -sw, sh),  // Top-right
            cv::Point3f(0, -sw, -sh), // Bottom-right
            cv::Point3f(0, sw, -sh)   // Bottom-left
        };

        // Large armor: 230mm x 55mm
        const float lw      = static_cast<float>(LARGE_ARMOR_WIDTH / 2.0);
        const float lh      = static_cast<float>(LARGE_ARMOR_HEIGHT / 2.0);
        large_armor_points_ = {
            cv::Point3f(0, lw, lh),   // Top-left
            cv::Point3f(0, -lw, lh),  // Top-right
            cv::Point3f(0, -lw, -lh), // Bottom-right
            cv::Point3f(0, lw, -lh)   // Bottom-left
        };
    }

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Point2f image_center_;

    std::vector<cv::Point3f> small_armor_points_;
    std::vector<cv::Point3f> large_armor_points_;
};

} // namespace fcs::L2