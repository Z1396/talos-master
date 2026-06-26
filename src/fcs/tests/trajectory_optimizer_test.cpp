// GoogleTest 单元测试框架头文件，提供 TEST、ASSERT、EXPECT 断言宏
#include <gtest/gtest.h>

// L5武器层：弹道优化器、武器控制系统核心实现头文件，被测主体
#include "L5_weapon/enhanced/trajectory_optimizer.hpp"
#include "L5_weapon/enhanced/weapon_systems.hpp"

// Eigen 线性代数库，用于存储弹道状态矩阵（横纵摆角、距离等时序状态）
#include <Eigen/Core>

// 业务顶层命名空间：fcs项目 L5武器层
namespace fcs::L5 {
// 匿名内部命名空间，隔离测试工具函数，防止全局符号冲突
namespace {

// 别名简化：核心弹道参考轨迹结构体，存储时序预测弹道
using ReferenceTrajectory = core::trajectory::ReferenceTrajectory;

/**
 * @brief 生成线性递增仿真参考弹道，用于基础时序窗口测试
 * @param horizon 弹道总预测步数（时域长度）
 * @return ReferenceTrajectory 填充线性距离、线性飞行时间的弹道对象
 * 规则：
 *  距离 distance[i] = i 米
 *  飞行时间 time_of_flights[i] = i * 0.01 秒
 *  4行全零状态矩阵：[yaw, pitch, yaw_dot, pitch_dot] 时序
 */
ReferenceTrajectory make_reference_trajectory(int horizon) {
    ReferenceTrajectory trajectory;
    // 4行N列状态矩阵：4维状态，horizon个时序步长，初始全0
    trajectory.state = ReferenceTrajectory::StateMatrix::Zero(4, horizon);
    // 预分配容器容量，避免多次内存扩容
    trajectory.distances.reserve(horizon);
    trajectory.time_of_flights.reserve(horizon);

    // 逐帧填充线性仿真数据
    for (int i = 0; i < horizon; ++i) {
        trajectory.distances.push_back(static_cast<double>(i));
        trajectory.time_of_flights.push_back(static_cast<double>(i) * 0.01);
    }

    return trajectory;
}

/**
 * @brief 基于参考弹道构造L4层跟踪指令包
 * @param reference 预测弹道（同时赋值给控制弹道、开火弹道）
 * @param timestamp_ns 指令纳秒时间戳
 * @return L4::TrackCommand 云台跟踪指令，包含控制/开火双弹道
 */
L4::TrackCommand make_track(const ReferenceTrajectory& reference, uint64_t timestamp_ns) {
    return L4::TrackCommand{
        .timestamp_ns       = timestamp_ns,
        // 云台跟踪控制使用的预测弹道
        .control_trajectory = reference,
        // 判定开火门限使用的专用弹道（远距离高TOF场景单独设计）
        .fire_trajectory    = reference,
    };
}

/**
 * @brief 生成固定值常量参考弹道，用于开火门限误差校验
 * @param horizon 总预测步长
 * @param yaw 弹道初始偏航基准角
 * @param pitch 全程固定俯仰角
 * @param distance 全程固定目标距离
 * @param tof 全程固定子弹飞行时间TOF
 * @return 所有时序步参数完全一致的平直弹道
 */
ReferenceTrajectory make_constant_reference_trajectory(
    int horizon, double yaw, double pitch, double distance, double tof) {
    ReferenceTrajectory trajectory;
    trajectory.state = ReferenceTrajectory::StateMatrix::Zero(4, horizon);
    trajectory.distances.reserve(horizon);
    trajectory.time_of_flights.reserve(horizon);
    // 弹道初始基准偏航角
    trajectory.yaw_origin = yaw;

    for (int i = 0; i < horizon; ++i) {
        // 第2行存储俯仰角，全程固定不变
        trajectory.state(2, i) = pitch;
        trajectory.distances.push_back(distance);
        trajectory.time_of_flights.push_back(tof);
    }

    return trajectory;
}

// ===================== 测试用例1 =====================
/**
 * @brief 测试：新鲜未过期弹道，优化器保留原始窗口中心采样点
 * 业务背景：
 * 参考弹道时间戳未超过老化阈值，判定为新鲜数据，优化窗口不滑动偏移
 * 窗口配置：历史回溯2帧 + 前瞻3帧，总长度 2+3+1=6 步，中心索引=2
 * 校验指标：输出距离、调试可视化中心索引、弹道序列长度、首帧距离
 */
TEST(TrajectoryOptimizerTest, FreshReferenceKeepsOriginalCenterSample) {
    // 武器控制器全局配置
    WeaponControllerConfig config;
    // 弹道老化判定阈值：1秒，超过则判定陈旧
    config.reference_age_threshold_s = 1.0;
    // 开启调试可视化数据输出，用于录制/上位机绘图
    config.enable_debug              = true;

    // 弹道时域窗口配置
    L4::ReferenceTrajectoryConfig trajectory_cfg;
    trajectory_cfg.horizon_back  = 2;  // 历史回溯帧数
    trajectory_cfg.horizon_ahead = 3;  // 未来预测前瞻帧数
    trajectory_cfg.dt            = 0.1; // 单步时间间隔 0.1s

    // 实例化轻量MPC弹道优化器
    TinyMpcTrajectoryOptimizer optimizer(config, trajectory_cfg);
    // 生成总长度6的完整线性弹道
    const auto reference =
        make_reference_trajectory(trajectory_cfg.horizon_back + trajectory_cfg.horizon_ahead + 1);
    // 构造跟踪指令，时间戳1s
    const auto track = make_track(reference, 1'000'000'000ULL);

    // 执行MPC弹道优化
    auto result = optimizer.optimize(track, track.timestamp_ns);
    // 优化必须成功，失败打印错误信息
    ASSERT_TRUE(result.has_value()) << result.error();

    const WeaponCommand& cmd = *result;
    // 新鲜弹道窗口不偏移，中心索引2对应距离2.0
    EXPECT_DOUBLE_EQ(cmd.distance, 2.0);
    // 必须输出调试可视化结构体
    ASSERT_TRUE(cmd.viz_debug.has_value());
    // 窗口中心索引固定为2
    EXPECT_EQ(cmd.viz_debug->center_index, 2);
    // 完整6步弹道全部保留
    EXPECT_EQ(cmd.viz_debug->reference_plan.size(), 6U);
    // 弹道首帧距离为0
    EXPECT_DOUBLE_EQ(cmd.viz_debug->reference_plan.front().distance, 0.0);
}

// ===================== 测试用例2 =====================
/**
 * @brief 测试：弹道陈旧超阈值，优化窗口向前滑动、调试窗口截断缩短
 * 场景：当前时间1.2s，弹道原始时间戳1.0s，时差0.2s？不对，逻辑是优化器内部计算弹道年龄超过阈值后，自动向前偏移窗口，丢弃过期历史帧
 * 校验：窗口起点后移2帧，可视弹道缩短为4帧，中心索引仍保持2，采样距离偏移至4.0
 */
TEST(TrajectoryOptimizerTest, ReferenceAgeShiftsWindowForwardAndShrinksDebugHorizon) {
    WeaponControllerConfig config;
    config.reference_age_threshold_s = 1.0;
    config.enable_debug              = true;

    L4::ReferenceTrajectoryConfig trajectory_cfg;
    trajectory_cfg.horizon_back  = 2;
    trajectory_cfg.horizon_ahead = 3;
    trajectory_cfg.dt            = 0.1;

    TinyMpcTrajectoryOptimizer optimizer(config, trajectory_cfg);
    const auto reference =
        make_reference_trajectory(trajectory_cfg.horizon_back + trajectory_cfg.horizon_ahead + 1);
    const auto track = make_track(reference, 1'000'000'000ULL);

    // 当前系统时间1.2s，弹道原始时间戳1.0s，触发老化偏移逻辑
    auto result = optimizer.optimize(track, 1'200'000'000ULL);
    ASSERT_TRUE(result.has_value()) << result.error();

    const WeaponCommand& cmd = *result;
    // 窗口向前滑动2帧，中心采样距离变为4.0，飞行时间0.04s
    EXPECT_DOUBLE_EQ(cmd.distance, 4.0);
    EXPECT_DOUBLE_EQ(cmd.tof, 0.04);
    ASSERT_TRUE(cmd.viz_debug.has_value());
    // 可视化窗口中心索引维持2（相对窗口内下标）
    EXPECT_EQ(cmd.viz_debug->center_index, 2);
    // 过期历史帧被截断，可视弹道只剩4步
    EXPECT_EQ(cmd.viz_debug->reference_plan.size(), 4U);
    // 可视窗口首帧对应原始第2步，距离=2
    EXPECT_DOUBLE_EQ(cmd.viz_debug->reference_plan.front().distance, 2.0);
    // 可视窗口第3帧对应原始第4步，距离=4
    EXPECT_DOUBLE_EQ(cmd.viz_debug->reference_plan[2].distance, 4.0);
}

// ===================== 测试用例3 =====================
/**
 * @brief 测试：长飞行时间(TOF)开火门限逻辑，使用fire_trajectory而非control_trajectory做开火判定
 * 业务场景：远距离子弹飞行时间长，云台跟踪用短距离控制弹道，开火判定需要专用补偿弹道
 * 两组弹道：
 * control：yaw基准0.0
 * fire：yaw基准0.08
 * 校验1：云台指令yaw=0.0，与fire弹道存在0.08偏差，不满足开火门限，禁止开火
 * 校验2：云台补偿yaw至0.08，与fire弹道对齐，误差小于阈值，允许开火
 */
TEST(TrajectoryOptimizerTest, HighTofTrackFireGateUsesFireTrajectoryInsteadOfControlTrajectory) {
    L4::ReferenceTrajectoryConfig trajectory_cfg;
    trajectory_cfg.horizon_back  = 2;
    trajectory_cfg.horizon_ahead = 2;
    trajectory_cfg.dt            = 0.1;
    const int horizon            = trajectory_cfg.horizon_back + trajectory_cfg.horizon_ahead + 1;

    // 云台控制弹道：yaw原点0，俯仰全程0，距离12m，飞行时间1.5s（远距离长TOF场景）
    const auto control = make_constant_reference_trajectory(horizon, 0.0, 0.0, 12.0, 1.5);
    // 开火判定专用补偿弹道：yaw原点偏移0.08
    const auto fire    = make_constant_reference_trajectory(horizon, 0.08, 0.0, 12.0, 1.5);
    const L4::TrackCommand track{
        .timestamp_ns       = 1'000'000'000ULL,
        .control_trajectory = control,
        .fire_trajectory    = fire,
    };

    // 开火判定阈值：角度误差小于0.001rad才允许开火
    FireDecisionConfig fire_cfg;
    fire_cfg.fire_thresh = 0.001;

    // 武器输出指令，默认使用control弹道的yaw基准0
    WeaponCommand cmd;
    cmd.timestamp_ns  = track.timestamp_ns;
    cmd.plan_yaw      = control.yaw_origin;
    cmd.plan_pitch    = 0.0;
    cmd.plan_distance = 12.0;

    // 场景1：云台yaw未补偿，和fire弹道存在0.08rad偏差，超出阈值，禁止开火
    auto center_aligned = apply_track_fire_gate(cmd, track, trajectory_cfg, fire_cfg, 0.0, 0.0);
    EXPECT_FALSE(center_aligned.fire);
    EXPECT_NEAR(center_aligned.yaw_error, 0.08, 1e-9);

    // 场景2：云台yaw补偿至0.08，和fire弹道对齐，误差接近0，允许开火
    auto fire_aligned = apply_track_fire_gate(cmd, track, trajectory_cfg, fire_cfg, 0.08, 0.0);
    EXPECT_TRUE(fire_aligned.fire);
    EXPECT_NEAR(fire_aligned.yaw_error, 0.0, 1e-9);
}

// ===================== 测试用例4 =====================
/**
 * @brief 测试：开火门限拒绝超过射击窗口的放大偏航偏差
 * 逻辑：定义yaw射击窗口宽度0.12rad，当云台yaw偏差0.012rad小于窗口，但内部放大后超出允许范围，禁止开火
 * 验证：偏差存在、开火标记false、有效射击窗口小于当前误差
 */
TEST(TrajectoryOptimizerTest, HighTofTrackFireGateRejectsAmplifiedYawBias) {
    L4::ReferenceTrajectoryConfig trajectory_cfg;
    trajectory_cfg.horizon_back  = 1;
    trajectory_cfg.horizon_ahead = 1;
    trajectory_cfg.dt            = 0.1;
    const int horizon            = trajectory_cfg.horizon_back + trajectory_cfg.horizon_ahead + 1;

    // 控制/开火弹道完全一致
    const auto fire = make_constant_reference_trajectory(horizon, 0.0, 0.0, 12.0, 1.5);
    const L4::TrackCommand track{
        .timestamp_ns       = 1'000'000'000ULL,
        .control_trajectory = fire,
        .fire_trajectory    = fire,
    };

    FireDecisionConfig fire_cfg;
    fire_cfg.fire_thresh            = 0.001;
    // 小装甲横向射击窗口宽度0.12rad
    fire_cfg.shooting_range_w_small = 0.12;
    fire_cfg.shooting_range_h       = 0.12;

    WeaponCommand cmd;
    cmd.timestamp_ns = track.timestamp_ns;

    // 传入0.012rad偏航偏差，内部放大后超出射击窗口，禁止开火
    auto biased = apply_track_fire_gate(cmd, track, trajectory_cfg, fire_cfg, 0.012, 0.0);
    EXPECT_FALSE(biased.fire);
    EXPECT_NEAR(biased.yaw_error, 0.012, 1e-9);
    // 有效射击窗口小于当前误差，判定脱靶
    EXPECT_LT(biased.shooting_range_yaw, biased.yaw_error);
}

} // 内部匿名命名空间结束
} // fcs::L5 武器层命名空间结束