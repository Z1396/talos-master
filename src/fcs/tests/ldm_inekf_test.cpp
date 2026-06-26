// Google测试框架头文件，提供TEST/EXPECT/ASSERT断言
#include <gtest/gtest.h>
// L3能量机关简易跟踪器顶层头文件（运动学模型+EKF+状态机封装）
#include "L3_estimation/ldm_naive/ldm_tracker.hpp"

// Eigen线性代数库：矩阵、旋转、向量运算
#include <Eigen/Core>
#include <Eigen/Geometry>

// 标准数学、常量、可选空值容器
#include <cmath>
#include <numbers>
#include <optional>

namespace {
// 别名简化长类型书写
using Model   = fcs::L3::ldm::LdmKinematic;       // LDM大符SE(3)运动学模型
using Nominal = Model::Nominal;                  // SE(3)名义状态：R(旋转)+t(平移/速度)
using CovXi   = Model::CovXi;                    // 李群切空间协方差矩阵

/**
 * @brief 构造SE(3)名义状态Nominal
 * @param R 世界->机体旋转矩阵
 * @param velocity 机体坐标系线速度
 * @param position 世界坐标系三维位置
 * @return Nominal 封装R、速度、位置的SE(3)状态
 */
Nominal make_nominal(
    const Eigen::Matrix3d& R, const Eigen::Vector3d& velocity, const Eigen::Vector3d& position) {
    // IsometriesType固定布局：[0]=机体速度，[1]=世界位置
    Nominal::IsometriesType t{};
    t[0] = velocity;
    t[1] = position;
    return Nominal(R, t);
}

/**
 * @brief 校验协方差矩阵严格对称（EKF协方差必须对称正定）
 * @param P 待校验协方差矩阵
 * @param tolerance 数值浮点容差，默认1e-10
 */
void expect_symmetric(const CovXi& P, double tolerance = 1e-10) {
    // P-P^T 矩阵所有元素范数小于容差则对称
    EXPECT_LE((P - P.transpose()).norm(), tolerance);
}

/**
 * @brief 计算SE(3)右不变误差范数（李群标准误差度量）
 * 公式：log(估计值逆 × 真值)，返回切空间向量二范数
 * @param estimate EKF估计SE3状态
 * @param truth 仿真真值SE3状态
 * @return 误差向量模长，用于判断滤波收敛
 */
double right_error_norm(const Nominal& estimate, const Nominal& truth) {
    return Nominal::log(estimate.inv() * truth).norm();
}

/**
 * @brief 从完整SE3名义状态提取位姿观测（仅R、p，丢弃速度）
 * @param X 完整Nominal状态
 * @return 位姿观测结构体PoseMeasurement
 */
Model::PoseMeasurement pose_measurement_from_nominal(const Nominal& X) {
    return Model::PoseMeasurement{.R_world_body = X.R(), .p_world_body = X.p()};
}

/**
 * @brief 快速构造位姿观测
 * @param R 世界-机体旋转矩阵
 * @param position 世界坐标系机体原点
 * @return 观测结构体
 */
Model::PoseMeasurement
    make_pose_measurement(const Eigen::Matrix3d& R, const Eigen::Vector3d& position) {
    return Model::PoseMeasurement{.R_world_body = R, .p_world_body = position};
}

} // 匿名工具命名空间结束

//=====================================================================================
// 一、底层运动学模型 LdmKinematic 纯数学单元测试
//=====================================================================================
/**
 * 测试1：单位旋转下预测步仅位置叠加速度×dt，速度、旋转保持不变
 * 场景：机体无旋转，世界坐标系速度直接叠加到位置
 */
TEST(LdmKinematic, PredictMovesPositionByVelocityAtIdentityRotation) {
    const Eigen::Vector3d velocity{1.0, -2.0, 0.5};
    const Eigen::Vector3d position{3.0, 4.0, 5.0};
    // 初始状态：单位旋转、给定速度、初始位置
    const Nominal x0 = make_nominal(Eigen::Matrix3d::Identity(), velocity, position);

    // 执行运动学预测 dt=0.2s
    const Nominal predicted = Model::predict_state(x0, 0.2);

    // 速度不变校验
    EXPECT_NEAR((predicted.v() - velocity).norm(), 0.0, 1e-12);
    // 位置 = pos + v*dt
    EXPECT_NEAR((predicted.p() - (position + 0.2 * velocity)).norm(), 0.0, 1e-12);
    // 旋转矩阵不变，仍是单位阵
    EXPECT_NEAR((predicted.R() - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12);
}

/**
 * 测试2：预测速度为机体坐标系矢量，需左乘旋转矩阵转换到世界坐标系叠加位移
 * 核心：v_body是机体局部速度，世界位移增量 = R × v_body × dt
 */
TEST(LdmKinematic, PredictTreatsVelocityAsBodyFrame) {
    // Z轴旋转90度旋转矩阵
    const Eigen::Matrix3d R =
        Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d velocity_body{2.0, 0.0, 0.0}; // 机体X向前速度
    const Eigen::Vector3d position{1.0, 2.0, 3.0};
    const Nominal x0 = make_nominal(R, velocity_body, position);

    // 预测步 dt=0.25s
    const Nominal predicted = Model::predict_state(x0, 0.25);

    // 机体坐标系速度存储不变
    EXPECT_NEAR((predicted.v() - velocity_body).norm(), 0.0, 1e-12);
    // 世界位置增量：R * v_body * dt
    EXPECT_NEAR((predicted.p() - (position + R * velocity_body * 0.25)).norm(), 0.0, 1e-12);
}

/**
 * 测试3：观测噪声矩阵R为对角正定矩阵，各维度噪声大于0
 * 维度定义：ROT_X/ROT_Y/ROT_Z 旋转噪声、BEARING_YAW方位角、BEARING_DISTANCE距离噪声
 */
TEST(LdmKinematic, NoiseMatricesHaveExpectedDiagonalShape) {
    Model model{};
    // 随便构造一条观测用于生成观测噪声矩阵
    const Model::PoseMeasurement z{
        .p_world_body = Eigen::Vector3d{5.0, 0.0, 0.0}
    };

    // 获取观测噪声协方差
    const auto R    = model.R(z);
    const auto diag = R.diagonal();

    // 校验矩阵维度匹配观测自由度NZ
    EXPECT_EQ(R.rows(), Model::NZ);
    EXPECT_EQ(R.cols(), Model::NZ);
    // 非对角元素全部为0，严格对角阵
    EXPECT_NEAR((R.diagonal() - diag).norm(), 0.0, 1e-12);
    // 各维度噪声方差>0，正定
    EXPECT_GT(diag[fcs::L3::ldm::BEARING_YAW], 0.0);
    EXPECT_GT(diag[fcs::L3::ldm::BEARING_DISTANCE], 0.0);
    EXPECT_GT(diag[fcs::L3::ldm::ROT_X], 0.0);
}

/**
 * 测试4：观测残差innovation逻辑
 * 1. 旋转残差使用SO(3)对数映射 log(R_est^T R_obs)
 * 2. 方位角BEARING_YAW自动角度环绕[-π,π]归一化
 */
TEST(LdmKinematic, PoseInnovationUsesSo3LogAndWrapsBearing) {
    // 预测状态：世界极坐标转换直角坐标，yaw接近π
    const Nominal predicted = make_nominal(
        Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
        fcs::L3::ypd2xyz(Eigen::Vector3d{std::numbers::pi - 1e-3, 0.0, 3.0}));

    // 观测旋转：仅X轴旋转0.01rad
    const Eigen::Matrix3d R_observed =
        Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX()).toRotationMatrix();
    // 观测方位角略小于-π，残差环绕后为3e-3
    const Model::PoseMeasurement observed{
        .R_world_body = R_observed,
        .p_world_body = fcs::L3::ypd2xyz(Eigen::Vector3d{-std::numbers::pi + 2e-3, 0.0, 4.0})};

    // 计算观测残差
    const auto innovation = Model::pose_innovation(predicted, observed);

    // X旋转残差严格0.01rad，Y/Z旋转无误差
    EXPECT_NEAR(innovation[fcs::L3::ldm::ROT_X], 0.01, 1e-12);
    EXPECT_NEAR(innovation[fcs::L3::ldm::ROT_Y], 0.0, 1e-12);
    EXPECT_NEAR(innovation[fcs::L3::ldm::ROT_Z], 0.0, 1e-12);
    // 方位角环绕后残差3e-3，距离残差4-3=1m
    EXPECT_NEAR(innovation[fcs::L3::ldm::BEARING_YAW], 3e-3, 1e-12);
    EXPECT_DOUBLE_EQ(innovation[fcs::L3::ldm::BEARING_DISTANCE], 1.0);
}

/**
 * 测试5：更新雅可比矩阵H分块校验
 * 1. 旋转观测对应雅可比单位块3×3
 * 2. 位置观测雅可比 = ypd2xyz对位置的导数 × 机体旋转矩阵R
 */
TEST(LdmKinematic, PoseUpdateJacobianMapsRightPositionPerturbationToYpd) {
    // 机体绕Z旋转0.4rad
    const Eigen::Matrix3d R = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d p{3.0, 1.0, -0.5};
    const Nominal predicted = make_nominal(R, Eigen::Vector3d::Zero(), p);

    // 获取EKF更新雅可比矩阵
    const auto H = Model::pose_update_H(predicted);

    // 旋转维度3×3雅可比为单位矩阵
    EXPECT_NEAR(
        (H.template block<3, 3>(fcs::L3::ldm::ROT_X, 0) - Eigen::Matrix3d::Identity()).norm(), 0.0,
        1e-12);
    // 位置观测分块雅可比 = ypd坐标转换导数 × R
    EXPECT_NEAR(
        (H.template block<3, 3>(fcs::L3::ldm::BEARING_YAW, 6) - fcs::L3::xyz2ypd_jacobian(p) * R)
            .norm(),
        0.0, 1e-12);
}

//=====================================================================================
// 二、LdmInEkf 李群不变卡尔曼滤波核心测试
//=====================================================================================
/**
 * 测试1：预测后协方差保持对称，且叠加过程噪声后迹增大（不确定性扩张）
 */
TEST(LdmInEkf, PredictKeepsCovarianceSymmetricAndAddsProcessNoise) {
    fcs::L3::ldm::LdmInEkfTracker target;
    // 初始状态：单位旋转、零速度、x=5m
    const Nominal x0 = make_nominal(
        Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d{5.0, 0.0, 0.0});
    // 初始协方差 0.01倍单位阵
    const CovXi P0 = CovXi::Identity() * 0.01;

    // EKF初始化
    target.initialize(Model::Params{}, x0, P0);
    // 预测前协方差迹
    const double trace_before = target.P().trace();
    // 执行预测步
    target.predict(0.1);

    // 校验协方差对称
    expect_symmetric(target.P());
    // 叠加过程噪声，迹变大
    EXPECT_GT(target.P().trace(), trace_before);
}

/**
 * 测试2：输入真值观测后，EKF更新减小右不变SE3误差（滤波收敛）
 */
TEST(LdmInEkf, UpdateReducesRightInvariantErrorForSyntheticMeasurement) {
    fcs::L3::ldm::LdmInEkfTracker target;
    // 初始估计状态，与真值存在偏差
    const Nominal x0 = make_nominal(
        Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d{5.0, 0.0, 0.0});

    // 构造真值SE3状态：微小旋转、位置偏移
    const Eigen::Matrix3d R_truth = (Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitZ())
                                     * Eigen::AngleAxisd(-0.02, Eigen::Vector3d::UnitY())
                                     * Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX()))
                                        .toRotationMatrix();
    const Nominal truth =
        make_nominal(R_truth, Eigen::Vector3d::Zero(), Eigen::Vector3d{5.1, 0.1, -0.05});

    // 初始协方差放大，允许快速收敛
    const CovXi P0 = CovXi::Identity() * 1.0;
    target.initialize(Model::Params{}, x0, P0);
    target.predict(0.01);

    // 更新前误差
    const double before = right_error_norm(target.nominal(), truth);
    // 输入真值观测更新
    target.update(pose_measurement_from_nominal(truth));
    // 更新后误差
    const double after = right_error_norm(target.nominal(), truth);

    expect_symmetric(target.P());
    // 更新后SE3右不变误差显著减小
    EXPECT_LT(after, before);
}

//=====================================================================================
// 三、顶层LdmTracker 状态机集成测试（Detecting/Tracking/TempLost/Idle）
//=====================================================================================
/**
 * 测试1：连续两帧有效观测，状态从Detecting检测切换为Tracking稳定跟踪
 * tracking_threshold=1：连续1帧观测达标即进入跟踪
 */
TEST(LdmTracker, ConfirmedMeasurementsPromoteToTracking) {
    fcs::L3::ldm::NaiveLdmConfig config;
    config.tracking_threshold = 1;
    fcs::L3::ldm::LdmTracker tracker(config);

    // 第一帧观测
    const auto z0 =
        make_pose_measurement(Eigen::Matrix3d::Identity(), Eigen::Vector3d{5.0, 0.0, 0.0});
    tracker.update(1'000'000, z0);
    // 仅一帧：检测状态，未跟踪
    EXPECT_EQ(tracker.status(), fcs::L3::TrackerStatus::Detecting);
    ASSERT_TRUE(tracker.get_output().has_value());
    EXPECT_FALSE(tracker.get_output()->is_tracking());

    // 第二帧有效观测
    const auto z1 =
        make_pose_measurement(Eigen::Matrix3d::Identity(), Eigen::Vector3d{5.1, 0.0, 0.0});
    tracker.update(2'000'000, z1);
    // 达标切换稳定跟踪
    EXPECT_EQ(tracker.status(), fcs::L3::TrackerStatus::Tracking);

    const auto output = tracker.get_output();
    ASSERT_TRUE(output.has_value());
    EXPECT_TRUE(output->is_tracking());
    EXPECT_TRUE(output->accurate);
    // 最后观测时间戳更新为第二帧
    EXPECT_EQ(output->last_observation_timestamp_ns, 2'000'000);
}

/**
 * 测试2：稳定跟踪后无观测进入临时丢失TempLost，超时彻底Idle清空跟踪器
 * lost_threshold=0.05s超时阈值
 */
TEST(LdmTracker, MissingMeasurementEntersTempLostAndTimesOut) {
    fcs::L3::ldm::NaiveLdmConfig config;
    config.tracking_threshold = 1;
    config.lost_threshold     = 0.05;
    fcs::L3::ldm::LdmTracker tracker(config);

    // 两帧观测进入跟踪
    const auto z =
        make_pose_measurement(Eigen::Matrix3d::Identity(), Eigen::Vector3d{5.0, 0.0, 0.0});
    tracker.update(1'000'000, z);
    tracker.update(2'000'000, z);
    ASSERT_EQ(tracker.status(), fcs::L3::TrackerStatus::Tracking);

    // 第三帧无观测：临时丢失，保留历史状态输出
    tracker.update(3'000'000, std::nullopt);
    EXPECT_EQ(tracker.status(), fcs::L3::TrackerStatus::TempLost);
    ASSERT_TRUE(tracker.get_output().has_value());
    EXPECT_TRUE(tracker.get_output()->is_tracking());
    EXPECT_FALSE(tracker.get_output()->accurate);

    // 超大时间间隔，超过丢失阈值，彻底Idle，无输出
    tracker.update(100'000'000, std::nullopt);
    EXPECT_EQ(tracker.status(), fcs::L3::TrackerStatus::Idle);
    EXPECT_FALSE(tracker.get_output().has_value());
}

/**
 * 测试3：检测阶段（Detecting）丢一帧观测直接退回Idle，不保留临时状态
 */
TEST(LdmTracker, DetectingDropsToIdleOnMissingMeasurement) {
    fcs::L3::ldm::NaiveLdmConfig config;
    fcs::L3::ldm::LdmTracker tracker(config);

    // 仅一帧观测：检测状态
    const auto z =
        make_pose_measurement(Eigen::Matrix3d::Identity(), Eigen::Vector3d{5.0, 0.0, 0.0});
    tracker.update(1'000'000, z);
    ASSERT_EQ(tracker.status(), fcs::L3::TrackerStatus::Detecting);

    // 下一帧无观测，直接清空进入空闲
    tracker.update(2'000'000, std::nullopt);
    EXPECT_EQ(tracker.status(), fcs::L3::TrackerStatus::Idle);
    EXPECT_FALSE(tracker.get_output().has_value());
}

/**
 * 测试4：输出世界坐标系速度 = 机体旋转矩阵 × 机体局部速度，坐标系转换校验
 */
TEST(LdmTracker, OutputVelocityWorldMatchesBodyVelocityThroughRotation) {
    fcs::L3::ldm::NaiveLdmConfig config;
    config.tracking_threshold = 1;
    fcs::L3::ldm::LdmTracker tracker(config);

    // 带旋转连续三帧观测，产生机体速度
    const Eigen::Matrix3d R = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto z0           = make_pose_measurement(R, Eigen::Vector3d{5.0, 0.0, 0.0});
    const auto z1           = make_pose_measurement(R, Eigen::Vector3d{5.0, 0.1, 0.0});
    const auto z2           = make_pose_measurement(R, Eigen::Vector3d{5.0, 0.2, 0.0});

    tracker.update(1'000'000, z0);
    tracker.update(11'000'000, z1);
    tracker.update(21'000'000, z2);

    const auto output = tracker.get_output();
    ASSERT_TRUE(output.has_value());
    // 世界速度 = R × 机体速度，误差趋近0
    EXPECT_NEAR(
        (output->velocity_world - output->R_world_body * output->velocity_body).norm(), 0.0, 1e-12);
}

/**
 * 附加底层EKF测试：临时丢失时缓存上一帧图像观测信息、时间戳不丢失
 */
TEST(LdmTracker, TempLostRetainsLastOpticalMeasurement) {
    fcs::L3::TrackerConfig config;
    config.robot.model              = fcs::L3::RobotEkfMotionModel::Params{};
    config.robot.lost_threshold     = 1.0;
    config.robot.tracking_threshold = 0;
    config.robot.matcher_gate       = 10.0;
    config.robot_inekf.radius0      = 0.2;
    config.robot_inekf.radius1      = 0.2;
    config.robot_inekf.height       = 0.0;

    fcs::L3::TrackerNew tracker(config);

    // 带图像像素距离的观测
    fcs::ArmorMeasurementBatch batch;
    batch.timestamp_ns = 100;
    batch.measurements.push_back(
        make_measurement(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0), fcs::ArmorName::Three, 42.0f));
    batch.measurements.front().timestamp_ns = 100;

    ASSERT_TRUE(tracker.first_meet(batch).has_value());
    tracker.predict(0.01);
    ASSERT_TRUE(tracker.update(batch));

    // 正常跟踪：缓存像素距离、观测时间戳
    const auto tracking_output = tracker.get_output();
    EXPECT_DOUBLE_EQ(tracking_output.last_image_center_distance_px, 42.0);
    EXPECT_EQ(tracking_output.last_observation_timestamp_ns, 100u);

    // 空观测更新，进入临时丢失
    fcs::ArmorMeasurementBatch empty_batch;
    empty_batch.timestamp_ns = 200;
    tracker.predict(0.01);
    ASSERT_FALSE(tracker.update(empty_batch));

    // 临时丢失仍保留上一轮有效观测信息，不刷新清空
    const auto temp_lost_output = tracker.get_output();
    EXPECT_EQ(temp_lost_output.status, fcs::L3::TrackerStatus::TempLost);
    EXPECT_DOUBLE_EQ(temp_lost_output.last_image_center_distance_px, 42.0);
    EXPECT_EQ(temp_lost_output.last_observation_timestamp_ns, 100u);
}