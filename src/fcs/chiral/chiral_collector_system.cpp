// 手性采集系统注册入口头文件
#include "chiral/chiral_collector_system.hpp"
// L3跟踪器输出状态枚举定义（Idle/Detecting/Tracking等）
#include "L3_estimation/tracker/types.hpp"
// L4规划层：当前选中目标快照结构体，保存跟踪目标完整状态
#include "L4_planning/selected_target_snapshot.hpp"
// 手性通信双向端点定义，用于跨进程/左右手数据收发
#include "chiral/chiral_endpoint.hpp"
// 手性数据协议、向量/状态枚举、坐标系转换工具
#include "chiral/navigation.hpp"
// SPMC通信通道Topic常量：选中目标快照数据流标识
#include "core/channel_topics.hpp"
// fast_tf静态坐标系帧枚举（odom/gimbal/camera/muzzle）
#include "frame.hpp"
// Eigen矩阵/位姿变换工具封装
#include "matrix.hpp"
// ECS调度器主类，add_system注册并行任务
#include "scheduler/scheduler.hpp"

// 枚举转字符串打印工具，日志输出状态可读文本
#include <magic_enum.hpp>
// 全框架统一日志库，打印错误/警告信息
#include <spdlog/spdlog.h>
// 动态数组容器，存放批量转换数据
#include <vector>

/**
 * @namespace fcs::chiral
 * @brief 手性数据交互模块命名空间
 * 业务作用：适配左右手两套机械结构，将本端视觉火控跟踪数据做格式转换，
 * 通过专用通信端点同步给另一侧手性程序，实现左右手信息互通、协同瞄准。
 * 核心逻辑：
 * 1. 创建Talos侧双向通信端点（读远端指令、写本地跟踪状态）
 * 2. 注册线程池并行采集系统，实时读取最新选中目标快照
 * 3. 查询TF坐标系，获取云台/相机/枪口静态位姿
 * 4. 转换FCS内部数据结构 → chiral手性协议标准结构体
 * 5. 打包完整数据写入通信端点，同步给另一侧程序
 */
namespace fcs::chiral {
// 导入底层手性工具命名空间，简化API调用
using namespace talos::chiral;

/**
 * @namespace 匿名命名空间（文件私有）
 * 存放仅当前文件使用的类型转换工具函数，对外不可见，避免全局命名污染
 * 全部为无异常转换工具，实时系统安全
 */
namespace {

/**
 * @brief Eigen三维向量 转 chiral协议标准三维向量模板函数
 * @tparam From 源坐标系标签
 * @tparam To 目标坐标系标签，默认untyped无绑定坐标系
 * @param v Eigen::Vector3d 原三维坐标向量
 * @return navigation::Vector3d 手性协议向量结构体
 * @ noexcept 无抛出异常，嵌入式硬实时友好
 */
template <navigation::tag From, navigation::tag To = navigation::untyped>
navigation::Vector3d<From, To> to_chiral_vector3d(const Eigen::Vector3d& v) noexcept {
    // 直接赋值xyz分量，完成结构体转换
    return {v.x(), v.y(), v.z()};
}

/**
 * @brief FCS内部跟踪器状态枚举 → chiral协议跟踪状态枚举转换
 * @param status L3::TrackerStatus 本框架内部跟踪状态
 * @return navigation::TrackerStatus 手性通信标准状态
 */
navigation::TrackerStatus to_chiral_tracker_status(L3::TrackerStatus status) noexcept {
    switch (status) {
    case L3::TrackerStatus::Idle:
        return navigation::TrackerStatus::Idle;        // 空闲无目标
    case L3::TrackerStatus::Detecting:
        return navigation::TrackerStatus::Detecting;  // 正在检测目标
    case L3::TrackerStatus::Tracking:
        return navigation::TrackerStatus::Tracking;    // 稳定跟踪中
    case L3::TrackerStatus::TempLost:
        return navigation::TrackerStatus::TempLost;   // 临时丢失目标
    }
}

/**
 * @brief FCS装甲编号枚举 → chiral协议装甲编号转换
 * @param name ArmorName 内部装甲/目标类型
 * @return navigation::ArmorName 手性通信标准枚举
 */
navigation::ArmorName to_chiral_armor_name(ArmorName name) noexcept {
    switch (name) {
    case ArmorName::Sentry:
        return navigation::ArmorName::Sentry;    // 哨兵装甲
    case ArmorName::One:
        return navigation::ArmorName::One;      // 一号装甲
    case ArmorName::Two:
        return navigation::ArmorName::Two;      // 二号装甲
    case ArmorName::Three:
        return navigation::ArmorName::Three;    // 三号装甲
    case ArmorName::Four:
        return navigation::ArmorName::Four;     // 四号装甲
    case ArmorName::Five:
        return navigation::ArmorName::Five;     // 五号装甲
    case ArmorName::Outpost:
        return navigation::ArmorName::Outpost;  // 前哨站
    case ArmorName::Base:
        return navigation::ArmorName::Base;      // 基地
    case ArmorName::BaseLarge:
        return navigation::ArmorName::Invalid;  // 大型基地无效映射
    case ArmorName::Invalid:
        return navigation::ArmorName::Invalid;  // 无效目标
    }
}

/**
 * @brief FCS目标颜色枚举 → chiral协议颜色枚举
 * @ [[maybe_unused]] 标记可能未使用，消除编译告警
 * @param color ArmorColor 内部红蓝/中立/紫色目标颜色
 * @return navigation::ArmorColor 手性通信标准颜色
 */
[[maybe_unused]] navigation::ArmorColor to_chiral_armor_color(ArmorColor color) noexcept {
    switch (color) {
    case ArmorColor::Blue:
        return navigation::ArmorColor::Blue;    // 蓝色敌方
    case ArmorColor::Red:
        return navigation::ArmorColor::Red;     // 红色敌方
    case ArmorColor::Neutral:
        return navigation::ArmorColor::Neutral; // 中立目标
    case ArmorColor::Purple:
        return navigation::ArmorColor::Purple;  // 紫色能量机关
    }
}

} // 匿名命名空间结束

/**
 * @brief 注册手性数据采集ECS系统
 * @param scheduler ECS全局调度器实例
 * 功能说明：
 * 1. 创建左右手双向通信端点，失败打印错误直接退出
 * 2. 注册talos::pool_compute线程池并行任务，不阻塞视觉主线程
 * 3. 持久捕获通信端点（mutable允许lambda内部调用write发送数据）
 * 4. 每帧读取最新选中目标快照，无新数据直接短路返回减少计算
 * 5. 查询TF树获取云台、相机、枪口静态坐标变换，填充协议结构体
 * 6. 将机器人/前哨站目标位置、速度、角度转换至云台坐标系（统一左右手基准）
 * 7. 转换内部枚举、向量到手性标准格式
 * 8. 打包完整TalosData结构体写入通信端点，同步给另一侧手性程序
 */
void register_chiral_collector_system(talos::Scheduler& scheduler) {
    // 步骤1：创建Talos端双向通信端点
    // 双向通道：本端写入TalosData（本地跟踪状态）、读取远端IncomingData（另一侧指令/状态）
    auto endpoint = navigation::TalosEndpoint::create();
    // std::expected判断端点创建是否失败
    if (!endpoint) {
        // 打印错误枚举可读字符串，告知创建失败原因（共享内存/套接字初始化失败）
        SPDLOG_ERROR(
            "Failed to create ChiralEndpoint (TalosSide): {}",
            magic_enum::enum_name(endpoint.error()));
        // 创建失败直接返回，不注册采集系统
        return;
    }

    // 步骤2：在线程池注册并行采集系统
    scheduler.add_system<talos::pool_compute>(
        // 系统唯一名称，用于性能统计、日志定位
        "chiral_collector",
        // lambda捕获：持久持有通信端点（移动语义转移所有权）
        // mutable：捕获的ep_storage为非const，允许调用write修改内部缓冲区发送数据
        [ep_storage = std::move(endpoint.value())](
            // SPMC输入通道：最新选中目标跟踪快照（包含完整滤波输出）
            talos::spmc<L4::SelectedTargetSnapshot, SelectedTargetSnapshotChannelTopic>
                selected_target_in,
            // 全局只读资源：fast_tf坐标系树，查询各机械部件静态变换
            talos::res<fast_tf::CoordinateSystem> tf_buffer) mutable {
            // 前置判断：通道无新快照数据，直接跳过本轮，减少无效计算
            if (!selected_target_in.has_new()) {
                return;
            }

            // 读取当前最新一帧选中目标快照
            auto selected_target = selected_target_in.read();
            // 空数据快照直接返回
            if (!selected_target) {
                return;
            }

            // 初始化手性协议顶层数据包，填充默认空值
            navigation::TalosData talos_data{};
            talos_data.state_kind   = navigation::TargetStateKind::Robot;
            talos_data.state.status = navigation::TrackerStatus::Idle;
            talos_data.state.color  = navigation::ArmorColor::Neutral;
            talos_data.state.name   = navigation::ArmorName::Invalid;

            // 当前帧时间戳，用于TF插值查询对应时刻坐标变换
            const uint64_t current_ns = selected_target->timestamp_ns;
            using namespace fast_tf;

            // ========== 1. 查询云台偏航坐标系变换，用于后续坐标基准统一 ==========
            auto gimbal_yaw_tf =
                lookup_clamped<odom, fast_tf::gimbal_yaw_fuxk_frame>(*tf_buffer, current_ns);
            if (!gimbal_yaw_tf) {
                SPDLOG_WARN("Failed to lookup gimbal_yaw transform: {}", gimbal_yaw_tf.error());
                return;
            }
            // 求逆变换：odom系 → 云台偏航系，所有目标坐标统一转换到此基准，左右手对齐
            auto gimbal_yaw_fix = gimbal_yaw_tf->inverse();

            // ========== 2. 查询odom到云台俯仰完整变换，填充协议云台位姿 ==========
            auto gimbal_tf = lookup_clamped<odom, gimbal_pitch>(*tf_buffer, current_ns);
            if (gimbal_tf) {
                auto translation                     = gimbal_tf.value().translation();
                auto rotation                        = gimbal_tf.value().quaternion();
                // 平移xyz赋值到手性协议云台结构体
                talos_data.gimbal_link.translation.x = translation.x();
                talos_data.gimbal_link.translation.y = translation.y();
                talos_data.gimbal_link.translation.z = translation.z();
                // 四元数wxyz赋值
                talos_data.gimbal_link.rotation.x    = rotation.x();
                talos_data.gimbal_link.rotation.y    = rotation.y();
                talos_data.gimbal_link.rotation.z    = rotation.z();
                talos_data.gimbal_link.rotation.w    = rotation.w();
            } else {
                SPDLOG_WARN("Failed to lookup gimbal transform: {}", gimbal_tf.error());
                return;
            }

            // ========== 3. 查询云台到相机机械坐标系变换 ==========
            auto camera_tf = lookup_clamped<gimbal, camera>(*tf_buffer, current_ns);
            if (camera_tf) {
                auto translation                     = camera_tf.value().translation();
                auto rotation                        = camera_tf.value().quaternion();
                talos_data.camera_link.translation.x = translation.x();
                talos_data.camera_link.translation.y = translation.y();
                talos_data.camera_link.translation.z = translation.z();
                talos_data.camera_link.rotation.x    = rotation.x();
                talos_data.camera_link.rotation.y    = rotation.y();
                talos_data.camera_link.rotation.z    = rotation.z();
                talos_data.camera_link.rotation.w    = rotation.w();
            } else {
                SPDLOG_WARN("Failed to lookup camera transform: {}", camera_tf.error());
                return;
            }

            // ========== 4. 查询odom到枪口坐标系变换 ==========
            auto muzzle_tf = lookup_clamped<odom, muzzle>(*tf_buffer, current_ns);
            if (muzzle_tf) {
                auto translation                     = muzzle_tf.value().translation();
                auto rotation                        = muzzle_tf.value().quaternion();
                talos_data.muzzle_link.translation.x = translation.x();
                talos_data.muzzle_link.translation.y = translation.y();
                talos_data.muzzle_link.translation.z = translation.z();
                talos_data.muzzle_link.rotation.x    = rotation.x();
                talos_data.muzzle_link.rotation.y    = rotation.y();
                talos_data.muzzle_link.rotation.z    = rotation.z();
                talos_data.muzzle_link.rotation.w    = rotation.w();
            } else {
                SPDLOG_WARN("Failed to lookup muzzle transform: {}", muzzle_tf.error());
                return;
            }

            // 判断当前快照是否存在有效跟踪目标：有目标 + 跟踪状态正常
            const bool has_valid_armor_target =
                selected_target->has_target() && selected_target->tracker.is_tracking();
            // 存在有效目标则获取跟踪输出指针，否则置空
            const L3::TrackerOutput* tracker_output =
                has_valid_armor_target ? std::addressof(selected_target->tracker) : nullptr;

            // 存在跟踪目标：转换跟踪状态、目标颜色、装甲编号到手性协议枚举
            if (tracker_output) {
                talos_data.state.status = to_chiral_tracker_status(tracker_output->status);
                talos_data.state.color  = to_chiral_armor_color(tracker_output->target_color);
                talos_data.state.name   = to_chiral_armor_name(tracker_output->target_name);
            }

            // ========== 分支1：跟踪目标为敌方机器人（4块装甲） ==========
            if (tracker_output && tracker_output->is_robot()) {
                talos_data.state_kind   = navigation::TargetStateKind::Robot;
                const auto* robot_state = tracker_output->robot_state();
                if (robot_state) {
                    // 读取odom系目标位置、速度、偏航角速度
                    auto pos      = robot_state->position;
                    auto vel      = robot_state->velocity;
                    // 构造odom坐标系目标位姿变换，仅平移+偏航旋转
                    auto pose_fix = gimbal_yaw_fix
                                  * TransformMatrixd<odom, void>::from_rpy(
                                        0.0, 0.0, robot_state->yaw, pos.x(), pos.y(), pos.z());
                    // 速度变换矩阵（角速度转换）
                    auto vel_fix = gimbal_yaw_fix
                                 * TransformMatrixd<odom, void>::from_rpy(
                                       0.0, 0.0, robot_state->v_yaw, vel.x(), vel.y(), vel.z());

                    // 转换至云台偏航坐标系存入协议结构体
                    talos_data.state.robot.position =
                        to_chiral_vector3d<navigation::gimbal_yaw>(pose_fix.translation());
                    talos_data.state.robot.velocity =
                        to_chiral_vector3d<navigation::gimbal_yaw>(vel_fix.translation());
                    // 云台系下目标偏航角、角速度
                    talos_data.state.robot.yaw     = pose_fix.euler_rot().yaw;
                    talos_data.state.robot.v_yaw   = vel_fix.euler_rot().yaw;
                    // 机器人前后装甲半径、高度、装甲数量
                    talos_data.state.robot.radius0 = robot_state->radius0;
                    talos_data.state.robot.radius1 = robot_state->radius1;
                    talos_data.state.robot.z1      = robot_state->z1;
                    talos_data.state.robot.armor_num =
                        static_cast<uint32_t>(robot_state->armors_num);
                }
            }
            // ========== 分支2：跟踪目标为前哨站（环形3装甲） ==========
            else if (tracker_output && tracker_output->is_outpost()) {
                talos_data.state_kind     = navigation::TargetStateKind::Outpost;
                const auto* outpost_state = tracker_output->outpost_state();
                if (outpost_state) {
                    auto pos = outpost_state->position;
                    auto vel = outpost_state->velocity;
                    auto z   = outpost_state->z;
                    // 转换三块装甲高度到云台坐标系
                    for (auto& i : z) {
                        i = (gimbal_yaw_fix
                             * TransformMatrixd<odom, void>::from_rpy(
                                 0.0, 0.0, outpost_state->yaw, pos.x(), pos.y(), i))
                                .translation()
                                .z();
                    }
                    // 速度、位置统一转换云台坐标系
                    auto vel_fix = gimbal_yaw_fix
                                 * TransformMatrixd<odom, void>::from_rpy(
                                       0.0, 0.0, outpost_state->v_yaw, vel.x(), vel.y(), vel.z());
                    auto pos_fix =
                        (gimbal_yaw_fix
                         * TransformMatrixd<odom, void>::from_rpy(
                             0.0, 0.0, outpost_state->yaw, pos.x(), pos.y(), 0.0));
                    // 前哨站2D平面坐标xy，z置0
                    talos_data.state.outpost.position.x = pos_fix.translation().x();
                    talos_data.state.outpost.position.y = pos_fix.translation().y();
                    talos_data.state.outpost.position.z = 0.0;
                    // 云台系速度向量
                    talos_data.state.outpost.velocity =
                        to_chiral_vector3d<navigation::gimbal_yaw>(vel_fix.translation());
                    // 偏航角与角速度
                    talos_data.state.outpost.yaw   = pos_fix.euler_rot().yaw;
                    talos_data.state.outpost.v_yaw = vel_fix.euler_rot().yaw;
                    // 三块装甲高度数组
                    talos_data.state.outpost.z     = z;
                }
            }
            // 全部坐标、状态填充完成，写入通信端点，同步数据给另一侧手性程序
            ep_storage->write(talos_data);
        });
}

} // namespace fcs::chiral