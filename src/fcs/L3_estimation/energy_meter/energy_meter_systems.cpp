#include "L3_estimation/energy_meter/energy_meter_systems.hpp"

// L2感知输出的小符原始观测结构体
#include "L2_perception/rune/types.hpp"
// L3对外输出的能量机关状态结构体
#include "L3_estimation/energy_meter/types.hpp"
// EKF跟踪器实现、模型定义
#include "L3_estimation/energy_meter_solver/energy_meter_tracker.hpp"
#include "L3_estimation/energy_meter_solver/types.hpp"
// 大小符投票器
#include "L3_estimation/energy_meter_solver/voter.hpp"
// 全局消息通道Topic枚举
#include "core/channel_topics.hpp"
// 高精度时钟
#include "core/time.hpp"
// Talos DAG调度器
#include "scheduler/scheduler.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <variant>       // std::variant 变体类型，实现大小跟踪器共存

#include <spdlog/spdlog.h>

namespace fcs::energy_meter {

namespace {
// 匿名命名空间：仅本文件内部可见，避免全局命名冲突

/**
 * @brief 从任意类型的跟踪器（大符/小符）提取EKF状态，组装成统一对外结构体 EnergyMeterState
 * @tparam ModelT 模型类型 BigRuneModel / SmallRuneModel
 * @param tracker EKF跟踪器实例
 * @param timestamp_ns 当前帧时间戳(纳秒)
 * @param tracking_frames 连续成功跟踪帧数
 * @param is_big 是否判定为大能量机关
 * @param obs_valid 本帧感知观测是否有效
 * @return 统一格式的能量机关状态包
 * @note 两套模型状态向量布局完全一致：[XC=0, YC=1, ZC=2, YAW=3, THETA=4]
 *       大符额外扩展维度：τ(阻力系数)、A(振幅)、ω角速度
 */
template <typename ModelT>
EnergyMeterState build_state_from_tracker(
    const ::energy_meter::Tracker<ModelT>& tracker, uint64_t timestamp_ns, int tracking_frames,
    bool is_big, bool obs_valid)
{
    // 获取EKF滤波后的状态向量 x
    const auto& x = tracker.get_state();

    // 解析状态分量
    const double roll   = x(ModelT::THETA);    // 能量机关自身旋转角度
    const double yaw    = x(ModelT::YAW);      // 能量机关整体偏航角（摆放朝向）
    const double radius = tracker.getRadius(); // 旋转半径：中心到叶片的物理距离

    // 判断跟踪器状态：正常跟踪 / 临时丢失都算处于跟踪阶段
    const bool is_tracking =
        (tracker.state == ::energy_meter::Tracker<ModelT>::TRACKING
         || tracker.state == ::energy_meter::Tracker<ModelT>::TEMP_LOST);

    // 计算当前生效叶片的绝对旋转角度
    // 一共5片叶片，均分360°：每片间隔 2π/5 rad
    // tracker.getCurrentBladeId()：当前需要击打叶片编号
    const double blade_roll = roll + static_cast<double>(tracker.getCurrentBladeId()) * 2.0 * M_PI / 5.0;
    const double sr = std::sin(blade_roll);
    const double cr = std::cos(blade_roll);

    // 【叶片在能量机关自身坐标系下的坐标】
    // 旋转平面为 Y-Z 平面，X为转轴
    Eigen::Vector3d blade_local(0.0, -radius * sr, radius * cr);

    // 能量机关整体yaw朝向旋转矩阵，将叶片局部坐标转到世界坐标系
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const Eigen::Vector3d blade_offset(
        blade_local.x() * cy - blade_local.y() * sy,
        blade_local.x() * sy + blade_local.y() * cy,
        blade_local.z());

    // 能量机关旋转中心在odom里程计坐标系下的三维坐标(XC,YC,ZC)
    const Eigen::Vector3d r_center(x(ModelT::XC), x(ModelT::YC), x(ModelT::ZC));

    // 构造姿态四元数
    const Eigen::Quaterniond yaw_quat(std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2));
    const Eigen::Quaterniond roll_quat(std::cos(blade_roll / 2), std::sin(blade_roll / 2), 0.0, 0.0);

    EnergyMeterState out;
    out.timestamp_ns   = timestamp_ns;
    out.tracker_state  = static_cast<TrackerState>(tracker.state);
    out.tracking_valid = is_tracking;
    out.r_center_odom  = r_center;
    out.radius         = radius;
    out.yaw            = yaw;
    out.pitch          = 0.0;
    out.roll           = roll;
    out.blade_id       = tracker.getCurrentBladeId();
    out.direction      = tracker.getDirection(); // 顺时针/逆时针旋转方向
    out.is_big_rune    = is_big;
    // 最终待击打叶片在odom世界坐标系下的三维坐标（控制层用来解算云台角度）
    out.position       = r_center + blade_offset;
    // 叶片整体姿态
    out.quat           = yaw_quat * roll_quat;

    out.obs_valid = obs_valid;

    // ========= 区分大符、小符赋值动力学参数 =========
    if constexpr (std::is_same_v<ModelT, ::energy_meter::BigRuneModel>)
    {
        // 大符模型：带变速动力学（角加速度、阻力、振幅）
        out.t           = x(ModelT::TAU);   // 阻尼系数τ
        out.a           = x(ModelT::A);     // 摆动振幅
        out.omega       = x(ModelT::W);      // 实时角速度
        out.b           = ::energy_meter::to_b(out.a);
        // 大符需要连续跟踪5帧以上才认定模型有效
        out.model_valid = is_tracking && tracking_frames > 5;
    }
    else
    {
        // 小符：固定匀速旋转，无动力学模型
        out.t           = 0.0;
        out.a           = 0.0;
        out.omega       = ::energy_meter::FIXED_RUNE_SPEED; // 固定角速度常量
        out.b           = ::energy_meter::FIXED_RUNE_SPEED;
        out.model_valid = is_tracking;
    }

    return out;
}

/**
 * @brief 对接 std::variant 变体跟踪器，自动分发到大/小构建函数
 * @param tracker_var 变体 AnyTracker：monostate / BigTracker / SmallTracker
 * @param timestamp_ns 时间戳
 * @param tracking_frames 连续跟踪帧数
 * @param is_big 是否大符
 * @param obs_valid 观测是否有效
 * @return 统一EnergyMeterState数据包
 */
EnergyMeterState build_state(
    const ::energy_meter::AnyTracker& tracker_var, uint64_t timestamp_ns, int tracking_frames,
    bool is_big, bool obs_valid)
{
    // std::visit 访问variant内真实类型，编译期分支匹配
    return std::visit(
        [&](const auto& tk) -> EnergyMeterState
        {
            using T = std::decay_t<decltype(tk)>;
            if constexpr (std::is_same_v<T, std::monostate>)
            {
                // 空状态，返回空白结构体
                return EnergyMeterState{};
            }
            else
            {
                // 分派给模板函数组装状态
                return build_state_from_tracker(tk, timestamp_ns, tracking_frames, is_big, obs_valid);
            }
        },
        tracker_var);
}

} // namespace

/**
 * @brief 向Talos调度器注册【能量机关估计系统】，并入全局DAG计算图
 * @param scheduler DAG调度器实例
 * @param config L3能量机关配置参数
 */
void register_energy_meter_systems(talos::Scheduler& scheduler, EnergyMeterL3Config&& config)
{
    // 将配置存入全局资源池，系统各处可只读获取
    scheduler.world().insert_resource(std::move(config));

    // 在全局资源池存放变体跟踪器（初始为空monostate）
    scheduler.world().insert_resource(std::make_shared<::energy_meter::AnyTracker>());

    // 注册并行计算系统，线程池调度运行
    scheduler.add_system<talos::pool_compute>(
        "l3_energy_meter",
        // 捕获变量：上一帧时间戳、连续跟踪帧数、大小符投票器Voter
        [last_timestamp_ns = uint64_t{0}, tracking_frames = int{0},
         voter = ::energy_meter::Voter{}](
            // 只读配置资源
            talos::res<EnergyMeterL3Config> cfg,
            // 可变资源：全局跟踪器变体智能指针
            talos::res_mut<std::shared_ptr<::energy_meter::AnyTracker>> tracker_ptr,
            // 订阅L2感知下发的小符观测数据
            talos::subscribe<fcs::rune::RuneObservation, RuneObservationChannelTopic> obs_in,
            // 发布滤波后的能量机关状态给控制层
            talos::publish<EnergyMeterState, EnergyMeterStateChannelTopic> state_out) mutable
        {
            // 阻塞读取一帧感知观测
            auto obs = obs_in.read();

            // 判定本帧观测是否合法：观测存在+有效标记+叶片坐标/姿态非空
            const bool obs_valid = obs && obs->valid && !obs->target_positions_odom.empty()
                                && !obs->target_quats_odom.empty();

            // 取当前帧时间戳，无观测则取系统当前时钟
            const uint64_t frame_ns = obs ? obs->timestamp_ns : fcs::clock::now_ns();
            double dt = 0.0;
            // 计算帧间隔时间dt(s)，用于EKF时序预测
            if (last_timestamp_ns > 0 && frame_ns > last_timestamp_ns)
            {
                dt = static_cast<double>(frame_ns - last_timestamp_ns) * 1e-9;
            }

            // 拿到全局变体跟踪器引用
            auto& tracker_var = **tracker_ptr;

            // 判断跟踪器当前是否处于空闲IDLE状态
            bool is_idle = false;
            std::visit(
                [&](const auto& tk)
                {
                    using T = std::decay_t<decltype(tk)>;
                    if constexpr (std::is_same_v<T, std::monostate>)
                    {
                        is_idle = true;
                    }
                    else
                    {
                        is_idle = (tk.state == T::IDLE);
                    }
                },
                tracker_var);

            // 空闲状态 + 无有效观测，直接跳过本轮
            if (is_idle && !obs_valid)
            {
                return;
            }

            last_timestamp_ns = frame_ns;

            // ========= 帧间隔过大，判定目标丢失，重置整套跟踪器 =========
            if (dt > cfg->reset_vote_time)
            {
                **tracker_ptr     = ::energy_meter::AnyTracker{};
                last_timestamp_ns = 0;
                tracking_frames   = 0;
                voter.reset();
                SPDLOG_WARN("energy_meter: large gap {:.1f}s, resetting", dt);
                return;
            }

            // ========= 分支1：当前没有有效观测（感知丢帧、被遮挡） =========
            if (!obs_valid)
            {
                // 仅执行EKF时间预测predict，不做观测更新；送入零观测实现缓慢飘移衰减
                std::visit(
                    [&](auto& tk)
                    {
                        using T = std::decay_t<decltype(tk)>;
                        if constexpr (!std::is_same_v<T, std::monostate>)
                        {
                            if (dt > 0.0)
                            {
                                tk.predict(dt);
                                tk.update(Eigen::Vector3d::Zero(), {}, {});
                            }
                        }
                    },
                    tracker_var);

                // 已经投票确定大小符，才允许输出丢失状态
                if (voter.determined() && !std::holds_alternative<std::monostate>(tracker_var))
                {
                    const bool is_tracking = std::visit(
                        [&](const auto& tk) -> bool
                        {
                            using T = std::decay_t<decltype(tk)>;
                            if constexpr (std::is_same_v<T, std::monostate>)
                                return false;
                            else
                                return tk.state == T::TRACKING || tk.state == T::TEMP_LOST;
                        },
                        tracker_var);

                    tracking_frames = is_tracking ? tracking_frames + 1 : 0;
                    // 输出丢失状态给控制层
                    state_out.write(build_state(
                        tracker_var, frame_ns, tracking_frames, voter.is_big(), obs_valid));
                }
                return;
            }

            // ========= 分支2：观测有效，执行大小符投票判定 =========
            // 投票器尚未得出结论时，利用本帧叶片数量更新投票
            if (!voter.determined())
            {
                // 传入当前检测到的叶片数量，进行投票
                if (const auto decision = voter.update(obs->target_positions_odom.size()))
                {
                    SPDLOG_INFO(
                        "靶数投票决策: {}, big={} small={}, frames={}",
                        decision->is_big ? "大符" : "小符", decision->big_votes,
                        decision->small_votes, decision->frames);

                    // 根据投票结果构造对应跟踪器存入variant
                    if (decision->is_big)
                    {
                        tracker_var = ::energy_meter::BigTracker{};
                    }
                    else
                    {
                        tracker_var = ::energy_meter::SmallTracker{};
                    }
                }
            }

            // 跟踪器依然为空，直接退出
            if (std::holds_alternative<std::monostate>(tracker_var))
            {
                return;
            }

            // ========= 进入EKF跟踪迭代：配置参数 + 初始化/预测更新 =========
            std::visit(
                [&](auto& tk)
                {
                    using T = std::decay_t<decltype(tk)>;
                    if constexpr (!std::is_same_v<T, std::monostate>)
                    {
                        // 将配置参数灌入跟踪器
                        typename T::Params p;
                        p.lost_thres          = cfg->lost_thres;
                        p.tracking_thres      = cfg->tracking_thres;
                        p.matcher_gate        = cfg->matcher_gate;
                        p.blade_unlock_frames = cfg->blade_unlock_frames;
                        // 区分大小符填入各自模型噪声参数
                        if constexpr (std::is_same_v<T, ::energy_meter::BigTracker>)
                            p.model_params = cfg->big_model;
                        else
                            p.model_params = cfg->small_model;

                        tk.set_params(p);

                        // 首次观测到来，初始化EKF状态
                        if (!tk.has_ekf())
                        {
                            tk.first_meet_u(
                                obs->r_center_odom.translation(), obs->target_positions_odom[0],
                                obs->target_quats_odom[0]);
                        }

                        // 实测旋转半径 = 叶片坐标 - 中心坐标 的模长
                        const double observed_radius =
                            (obs->target_positions_odom[0] - obs->r_center_odom.translation()).norm();
                        tk.setRadius(observed_radius);

                        // 帧间隔正常：predict预测 + update观测更新完整EKF步进
                        if (dt > 0.0)
                        {
                            tk.step(
                                dt, obs->r_center_odom.translation(), obs->target_positions_odom,
                                obs->target_quats_odom);
                        }
                        else
                        {
                            // dt=0 无时间流逝，仅做观测更新
                            tk.update(
                                obs->r_center_odom.translation(), obs->target_positions_odom,
                                obs->target_quats_odom);
                        }
                    }
                },
                tracker_var);

            // ========= 统计跟踪状态计数，组装数据包发布 =========
            const bool is_tracking = std::visit(
                [&](const auto& tk) -> bool
                {
                    using T = std::decay_t<decltype(tk)>;
                    if constexpr (std::is_same_v<T, std::monostate>)
                        return false;
                    else
                        return tk.state == T::TRACKING || tk.state == T::TEMP_LOST;
                },
                tracker_var);

            tracking_frames = is_tracking ? tracking_frames + 1 : 0;
            // 统一封装状态并发布至总线，供云台控制模块订阅
            state_out.write(build_state(
                tracker_var, obs->timestamp_ns, tracking_frames, voter.is_big(), obs_valid));
        });
}

} // namespace fcs::energy_meter