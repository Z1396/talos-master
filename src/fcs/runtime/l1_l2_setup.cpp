// 头文件保护，防止多次包含
#include "runtime/l1_l2_setup.hpp"

// L1底层硬件抽象：相机输入、云台输出
#include "L1_sensor/camera_interface.hpp"
#include "L1_sensor/output_interface.hpp"
// L5武器发射控制指令
#include "L5_weapon/fire_control.hpp"
// Chiral分体云台驱动
#include "chiral/gimbal.hpp"
// 全局基础配置、通道话题定义、运行时时钟、弹道资源、基础类型、欧拉角、TF帧枚举
#include "config.hpp"
#include "core/channel_topics.hpp"
#include "core/runtime.hpp"
#include "core/time.hpp"
#include "core/trajectory/resource.hpp"
#include "core/types.hpp"
#include "euler.hpp"
// TF坐标系帧ID定义
#include "frame.hpp"
// 枚举转字符串打印工具
#include "magic_enum.hpp"
// 配置加载器、ECS调度器
#include "runtime/config_loader.hpp"
#include "scheduler/scheduler.hpp"
// 分体MCU串口/USB协议、数据包定义
#include "talos_gimbal/mcu_device.hpp"
#include "talos_gimbal/packet.hpp"
// STL基础工具
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
// 自制variant多分支分发工具
#include <primitive/overloaded.hpp>
#include <utility>

// 格式化打印、日志库
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <spdlog/spdlog.h>

namespace fcs::runtime {
namespace { // 匿名命名空间，内部工具仅当前编译单元可见

/**
 * @brief 截取两帧之间的TF变换快照，封装成控制快照结构体
 * @tparam From 源坐标系
 * @tparam To   目标坐标系
 * @param tf_buffer TF变换缓存池
 * @param timestamp_ns 采样时间戳(纳秒)
 * @return ControlTransformSnapshot 平移+四元数变换快照，不存在则present=false
 * @nodiscard 禁止丢弃返回值
 */
template <fast_tf::frame From, fast_tf::frame To>
[[nodiscard]] auto capture_control_transform_snapshot(
    const fast_tf::CoordinateSystem& tf_buffer, uint64_t timestamp_ns)
    -> core::ControlTransformSnapshot {
    // 初始化空快照，默认present=false（变换无效）
    core::ControlTransformSnapshot snapshot;
    // 时间钳位查找变换：找不到返回std::nullopt
    auto tf = fast_tf::lookup_clamped<From, To>(tf_buffer, timestamp_ns);
    if (!tf) {
        return snapshot;
    }

    // 标记变换有效
    snapshot.present       = true;
    // 提取平移向量 xyz
    const auto translation = tf->translation();
    snapshot.translation   = {translation.x(), translation.y(), translation.z()};
    // 提取旋转四元数 xyzw
    const auto quaternion = tf->quaternion();
    snapshot.quaternion   = {quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()};
    return snapshot;
}

/**
 * @brief 打包全部控制状态快照，写入spmc消息通道供上层自瞄、弹道模块读取
 * @param control_out 可写Spmc通道：输出控制完整状态
 * @param tf_buffer TF坐标变换缓存
 * @param imu_state 当前云台IMU姿态
 * @param detecting_color 敌方装甲颜色（红/蓝）
 * @param bullet_speed_raw MCU原始上报弹速
 * @param bullet_speed 限幅修正后有效弹速
 * @param sample_timestamp_ns 当前采样纳秒时间戳
 */
void publish_control_snapshot(
    talos::spmc_mut<core::ControlResourceSnapshot, RuntimeControlStateChannelTopic> control_out,
    const fast_tf::CoordinateSystem& tf_buffer, const core::ImuState& imu_state,
    ArmorColor detecting_color, double bullet_speed_raw, double bullet_speed,
    uint64_t sample_timestamp_ns) {
    // 聚合初始化完整控制快照，一次性写入通道
    control_out.write(
        core::ControlResourceSnapshot{
            .sample_timestamp_ns = sample_timestamp_ns, // 采样时间戳
            .imu                 = imu_state,            // IMU姿态、角速度
            .detecting_color     = detecting_color,      // 目标装甲颜色
            .bullet_speed_raw    = bullet_speed_raw,     // MCU原始弹速
            .bullet_speed        = bullet_speed,          // 限幅后可用弹速
            // 底盘odom -> 云台俯仰轴变换
            .odom_to_gimbal_pitch =
                capture_control_transform_snapshot<fast_tf::odom, fast_tf::gimbal_pitch_fuxk_frame>(
                    tf_buffer, sample_timestamp_ns),
            // 云台基座 -> 相机连杆变换
            .gimbal_to_camera_link =
                capture_control_transform_snapshot<fast_tf::gimbal, fast_tf::camera>(
                    tf_buffer, sample_timestamp_ns),
            // 底盘odom -> 相机光心坐标系
            .odom_to_camera_optical =
                capture_control_transform_snapshot<fast_tf::odom, fast_tf::camera_optical>(
                    tf_buffer, sample_timestamp_ns),
            // 底盘odom -> 枪口坐标系（弹道解算核心）
            .odom_to_muzzle =
                capture_control_transform_snapshot<fast_tf::odom, fast_tf::muzzle_link_fuxk_frame>(
                    tf_buffer, sample_timestamp_ns),
        });
}

/**
 * @brief Chiral分体云台MCU驱动系统
 * 适配独立Chiral云台控制器，串口/私有协议读取IMU、弹速、敌方颜色
 */
struct ChiralImu {
    McuConfig config; // MCU硬件配置（弹速上下限、是否由MCU做主数据源）
    // Chiral云台通信端点共享指针
    std::shared_ptr<talos::chiral::gimbal::TalosEndpoint> device;

    /**
     * @brief 1000Hz周期执行：读取云台IMU、更新TF、更新全局资源、推送控制快照
     * @param tf_buffer 可写TF坐标缓存
     * @param bullet_speed 可修改全局弹速资源
     * @param detecting_color 可修改敌方颜色资源
     * @param imu_state IMU姿态全局资源
     * @param control_out 控制状态输出通道
     * @param gt_out 真值数据通道（当前未使用，[[maybe_unused]]消除警告）
     * @noexcept 无异常抛出
     */
    void system(
        talos::res_mut<fast_tf::CoordinateSystem> tf_buffer,
        core::trajectory::bullet_speed_mut bullet_speed, core::detecting_color_mut detecting_color,
        core::imu_state_mut imu_state,
        talos::spmc_mut<core::ControlResourceSnapshot, RuntimeControlStateChannelTopic> control_out,
        [[maybe_unused]] talos::spmc_mut<ipc::GroundTruthBatch, GroundTruthBatchChannelTopic>
            gt_out) noexcept {
        // 读取一帧最新云台IMU数据包
        auto imu_data_result = device->read_new();
        // 无新数据包直接退出，[[unlikely]]编译器优化冷分支
        if (!imu_data_result.has_value()) [[unlikely]] {
            return;
        }
        auto imu_data = imu_data_result.value();

        // 获取当前系统纳秒时间戳
        const uint64_t timestamp_ns = fcs::clock::now_ns();
        // 更新云台Yaw偏航轴旋转TF（仅旋转，无平移）
        fast_tf::update_rotate_only<fast_tf::gimbal_yaw_fuxk_frame>(
            *tf_buffer,
            math_fuxk::rpy<double>(imu_data.roll, 0.0, std::numbers::pi * imu_data.yaw / 180.0),
            timestamp_ns);
        // 更新云台Pitch俯仰轴旋转TF
        fast_tf::update_rotate_only<fast_tf::gimbal_pitch_fuxk_frame>(
            *tf_buffer, math_fuxk::rpy<double>(0.0, std::numbers::pi * imu_data.pitch / 180.0, 0.0),
            timestamp_ns);

        // 配置开启MCU弹速权威模式：硬件上报弹速覆盖全局资源
        if (config.mcu_authoritative_bullet_speed && imu_data.bullet_speed > 0) {
            // 弹速限幅在配置min/max之间
            double new_speed = std::clamp(
                static_cast<double>(imu_data.bullet_speed), config.bullet_speed_min,
                config.bullet_speed_max);
            // 弹速发生变化则更新并打印日志
            if (new_speed != bullet_speed->bullet_speed) {
                SPDLOG_INFO(
                    "bullet speed changed: {} -> {}", bullet_speed->bullet_speed, new_speed);
                bullet_speed->bullet_speed = new_speed;
            }
        }

        // MCU控制敌方装甲颜色
        if (config.mcu_authoritative_self_color) {
            // MCU上报自身颜色，映射敌方对立颜色
            auto new_detecting_color = imu_data.self_color == talos::chiral::gimbal::Color::Blue
                                         ? ArmorColor::Red
                                         : ArmorColor::Blue;
            // 颜色变更打印日志
            if (new_detecting_color != *detecting_color) [[unlikely]] {
                SPDLOG_INFO(
                    "detecting color changed: {} -> {}", *detecting_color, new_detecting_color);
                *detecting_color = new_detecting_color;
            }
        }

        // 填充全局IMU姿态资源
        imu_state->timestamp_ns = timestamp_ns;
        imu_state->yaw          = imu_data.yaw;
        imu_state->pitch        = imu_data.pitch;
        imu_state->roll         = imu_data.roll;
        imu_state->yaw_vel      = imu_data.yaw_vel;
        imu_state->pitch_vel    = imu_data.pitch_vel;
        imu_state->roll_vel     = imu_data.roll_vel;

        // 打包所有状态写入消息通道
        publish_control_snapshot(
            control_out, *tf_buffer, *imu_state, *detecting_color, imu_data.bullet_speed,
            bullet_speed->bullet_speed, timestamp_ns);
    }
};

/**
 * @brief Daedalus一体化设备IMU系统
 * 基于共享内存IPC通信，直接读取底盘、云台、相机、枪口全部位姿真值
 * 无独立MCU，所有位姿由仿真/一体化硬件IPC输出
 */
struct DaedalusImu {
    // IPC共享内存客户端
    std::shared_ptr<ipc::ShmClient> ipc_device;
    // 上一帧真值时间戳，防止重复推送相同真值包
    uint64_t last_gt_timestamp_ns{0};

    void system(
        talos::res_mut<fast_tf::CoordinateSystem> tf_buffer,
        [[maybe_unused]] core::trajectory::bullet_speed_mut bullet_speed,
        [[maybe_unused]] core::detecting_color_mut detecting_color, core::following_mut following,
        [[maybe_unused]] core::imu_state_mut imu_state,
        talos::spmc_mut<core::ControlResourceSnapshot, RuntimeControlStateChannelTopic> control_out,
        talos::spmc_mut<ipc::GroundTruthBatch, GroundTruthBatchChannelTopic> gt_out) noexcept {
        bool control_updated           = false;
        uint64_t control_timestamp_ns  = 0;
        // 局部lambda：标记当前周期有位姿更新，记录最新时间戳
        const auto note_control_update = [&](uint64_t timestamp_ns) {
            control_updated      = true;
            control_timestamp_ns = std::max(control_timestamp_ns, timestamp_ns);
        };

        // 1. 读取云台全局位姿，更新云台TF
        if (const auto pose = ipc_device->recv_pose(ipc::POSE_GIMBAL)) {
            const Eigen::Quaterniond q(pose->qw, pose->qx, pose->qy, pose->qz);
            const auto euler              = math_fuxk::rpy(q);
            const auto [roll, pitch, yaw] = euler.rpy();
            // 更新云台基座完整平移+旋转TF
            fast_tf::update(
                *tf_buffer,
                fast_tf::EdgeTransform<fast_tf::gimbal>::from_quaternion_xyz(
                    q, pose->x, pose->y, pose->z),
                pose->timestamp_ns);
            // 俯仰轴单独TF（无旋转偏移）
            fast_tf::update(
                *tf_buffer,
                fast_tf::EdgeTransform<fast_tf::gimbal_pitch_fuxk_frame>::from_rpy(0, 0, 0),
                pose->timestamp_ns);
            // 填充IMU姿态
            imu_state->timestamp_ns = pose->timestamp_ns;
            imu_state->yaw          = yaw;
            imu_state->pitch        = pitch;
            imu_state->roll         = roll;
            // Daedalus无角速度数据，置0
            imu_state->yaw_vel      = 0.0;
            imu_state->pitch_vel    = 0.0;
            imu_state->roll_vel     = 0.0;
            note_control_update(pose->timestamp_ns);
        }

        // 2. 读取底盘odom平移TF
        if (const auto pose = ipc_device->recv_pose(ipc::POSE_ODOM)) {
            fast_tf::update(
                *tf_buffer,
                fast_tf::EdgeTransform<fast_tf::odom>::from_translation(pose->x, pose->y, pose->z),
                pose->timestamp_ns);
            note_control_update(pose->timestamp_ns);
        }

        // 3. 相机连杆位姿
        if (const auto pose = ipc_device->recv_pose(ipc::POSE_CAMERA)) {
            fast_tf::update(
                *tf_buffer,
                fast_tf::EdgeTransform<fast_tf::camera_link_fuxk_frame>::from_translation(
                    pose->x, pose->y, pose->z),
                pose->timestamp_ns);
            note_control_update(pose->timestamp_ns);
        }

        // 4. 枪口完整位姿（弹道解算核心）
        if (const auto pose = ipc_device->recv_pose(ipc::POSE_MUZZLE)) {
            const Eigen::Quaterniond q(pose->qw, pose->qx, pose->qy, pose->qz);
            fast_tf::update(
                *tf_buffer,
                fast_tf::EdgeTransform<fast_tf::muzzle_link_fuxk_frame>::from_quaternion_xyz(
                    q, pose->x, pose->y, pose->z),
                pose->timestamp_ns);
            note_control_update(pose->timestamp_ns);
        }

        // 底盘观测数据，预留扩展
        if (const auto obs = ipc_device->recv_chassis_observation()) {
            // TODO：底盘速度、碰撞检测扩展
        }

        // 读取运行时状态：自动跟随开关
        if (const auto state = ipc_device->recv_runtime_state()) {
            const auto next_following = state->following != 0U;
            // 原子变量更新跟随状态，变更打印日志
            if (next_following != following->load()) {
                following->store(next_following);
                SPDLOG_INFO("daedalus following changed: {}", next_following);
            }
        }

        // 真值数据包写入通道，重复时间戳跳过
        if (const auto gt = ipc_device->recv_ground_truth()) {
            if (gt->timestamp_ns != last_gt_timestamp_ns) {
                last_gt_timestamp_ns = gt->timestamp_ns;
                gt_out.write(*gt);
            }
        }

        // 本周期存在位姿更新，推送完整控制快照
        if (control_updated) {
            publish_control_snapshot(
                control_out, *tf_buffer, *imu_state, *detecting_color, bullet_speed->bullet_speed,
                bullet_speed->bullet_speed, control_timestamp_ns);
        }
    }
};

// 分体MCU设备句柄别名
using McuHandle = talos_gimbal::McuDeviceHandle;
// 全局静态缓冲区：串口解析IMU数据包、能力数据包
static talos_gimbal::ReceiveImuData g_imu_data{};
static talos_gimbal::ReceiveCapabilitiesData g_capabilities_data{};

/**
 * @brief 解析MCU上报设备能力，更新全局Capability资源
 * @param capabilities 设备能力可写资源
 * @param following 自动跟随开关原子资源
 * @param reported MCU上报原始能力数据包
 */
void apply_mcu_capabilities(
    core::capabilities_mut capabilities, core::following_mut following,
    const talos_gimbal::ReceiveCapabilitiesData& reported) noexcept {
    // 校验数据包帧头帧尾，非法包直接丢弃
    if (reported.header.sof != talos_gimbal::HeaderFrame::SoF()
        || reported.eof != talos_gimbal::HeaderFrame::EoF()) [[unlikely]] {
        return;
    }

    // 定义MCU可控制的能力掩码：能量机关、超级弹道
    constexpr auto mcu_controlled = static_cast<core::CapabilityMask>(
        core::capability_bit(core::Capability::Rune)
        | core::capability_bit(core::Capability::Quanta));
    // 更新跟随开关状态
    const auto next_following = reported.data.following != 0U;
    if (next_following != following->load()) {
        following->store(next_following);
        SPDLOG_INFO("MCU following changed: {}", next_following);
    }

    // 先清除MCU管控的能力位，再根据上报重新置位
    const auto previous = capabilities->load();
    auto next           = static_cast<core::CapabilityMask>(previous & ~mcu_controlled);
    if (reported.data.power_rune != 0U) {
        next =
            static_cast<core::CapabilityMask>(next | core::capability_bit(core::Capability::Rune));
    }
    if (reported.data.quanta != 0U) {
        next = static_cast<core::CapabilityMask>(
            next | core::capability_bit(core::Capability::Quanta));
    }
    // 能力无变化直接返回
    if (next == previous) {
        return;
    }
    // 更新原子资源并打印激活的能力列表
    capabilities->store(next);
    SPDLOG_INFO(
        "MCU capabilities changed: [{}]",
        fmt::join(core::active_capabilities(*capabilities), ", "));
}

/**
 * @brief 分体串口/USB MCU IMU系统
 * 解析STM32串口原始数据包，更新TF、弹速、设备能力、敌方颜色
 */
struct McuImu {
    McuConfig config;
    std::shared_ptr<McuHandle> device;
    // 指向全局静态IMU解析缓冲区
    talos_gimbal::ReceiveImuData* imu_data                         = nullptr;
    // USB模式才会解析能力包，串口置空
    const talos_gimbal::ReceiveCapabilitiesData* capabilities_data = nullptr;

    void system(
        talos::res_mut<fast_tf::CoordinateSystem> tf_buffer,
        core::trajectory::bullet_speed_mut bullet_speed, core::detecting_color_mut detecting_color,
        core::capabilities_mut capabilities, core::following_mut following,
        core::imu_state_mut imu_state,
        talos::spmc_mut<core::ControlResourceSnapshot, RuntimeControlStateChannelTopic> control_out,
        [[maybe_unused]] talos::spmc_mut<ipc::GroundTruthBatch, GroundTruthBatchChannelTopic>
            gt_out) noexcept {
        // 处理串口接收事件，底层填充g_imu_data/g_capabilities_data
        device->handle_events();
        // 存在能力包则更新全局设备能力
        if (capabilities_data != nullptr) {
            apply_mcu_capabilities(capabilities, following, *capabilities_data);
        }
        // 校验IMU数据包合法性
        if (imu_data->header.sof != talos_gimbal::HeaderFrame::SoF()
            || imu_data->eof != talos_gimbal::HeaderFrame::EoF()) [[unlikely]] {
            return;
        }
        const uint64_t timestamp_ns = fcs::clock::now_ns();
        // 更新Yaw、Pitch旋转TF
        fast_tf::update_rotate_only<fast_tf::gimbal_yaw_fuxk_frame>(
            *tf_buffer, math_fuxk::rpy<double>(imu_data->data.roll, 0.0, imu_data->data.yaw),
            timestamp_ns);
        fast_tf::update_rotate_only<fast_tf::gimbal_pitch_fuxk_frame>(
            *tf_buffer, math_fuxk::rpy<double>(0.0, imu_data->data.pitch, 0.0), timestamp_ns);

        // MCU权威弹速更新逻辑，与ChiralImu一致
        if (config.mcu_authoritative_bullet_speed && imu_data->data.bullet_speed > 0) {
            double new_speed = std::clamp(
                static_cast<double>(imu_data->data.bullet_speed), config.bullet_speed_min,
                config.bullet_speed_max);
            if (new_speed != bullet_speed->bullet_speed) {
                SPDLOG_INFO(
                    "bullet speed changed: {} -> {}", bullet_speed->bullet_speed, new_speed);
                bullet_speed->bullet_speed = new_speed;
            }
        }

        // MCU权威敌方颜色更新
        if (config.mcu_authoritative_self_color) {
            auto new_detecting_color = imu_data->data.self_color == talos_gimbal::Color::Blue
                                         ? ArmorColor::Red
                                         : ArmorColor::Blue;
            if (new_detecting_color != *detecting_color) [[unlikely]] {
                SPDLOG_INFO(
                    "detecting color changed: {} -> {}", *detecting_color, new_detecting_color);
                *detecting_color = new_detecting_color;
            }
        }

        // 填充全局IMU姿态资源
        imu_state->timestamp_ns = timestamp_ns;
        imu_state->yaw          = imu_data->data.yaw;
        imu_state->pitch        = imu_data->data.pitch;
        imu_state->roll         = imu_data->data.roll;
        imu_state->yaw_vel      = imu_data->data.yaw_vel;
        imu_state->pitch_vel    = imu_data->data.pitch_vel;
        imu_state->roll_vel     = imu_data->data.roll_vel;

        // 推送完整控制快照
        publish_control_snapshot(
            control_out, *tf_buffer, *imu_state, *detecting_color, imu_data->data.bullet_speed,
            bullet_speed->bullet_speed, timestamp_ns);
    }
};

/**
 * @brief IMU统一封装层
 * std::variant 存储三种硬件实现：DaedalusImu / McuImu / ChiralImu
 * 对外暴露统一system接口，通过std::visit+overloaded自动分发对应硬件逻辑
 */
struct Imu {
    std::variant<DaedalusImu, McuImu, ChiralImu> impl;

    /**
     * @brief 统一周期入口，自动分发到对应硬件实现的system函数
     */
    void system(
        talos::res_mut<fast_tf::CoordinateSystem> tf_buffer,
        core::trajectory::bullet_speed_mut bullet_speed, core::detecting_color_mut detecting_color,
        core::capabilities_mut capabilities, core::following_mut following,
        core::imu_state_mut imu_state,
        talos::spmc_mut<core::ControlResourceSnapshot, RuntimeControlStateChannelTopic> control_out,
        talos::spmc_mut<ipc::GroundTruthBatch, GroundTruthBatchChannelTopic> gt_out) noexcept {
        // std::visit + overloaded 多分支类型分发，匹配variant内部存储的硬件类型
        std::visit(
            overloaded{
                [&](McuImu& inner) {
                    inner.system(
                        tf_buffer, bullet_speed, detecting_color, capabilities, following,
                        imu_state, control_out, gt_out);
                },
                [&](DaedalusImu& inner) {
                    inner.system(
                        tf_buffer, bullet_speed, detecting_color, following, imu_state, control_out,
                        gt_out);
                },
                // 通用兜底分支：ChiralImu参数最少，直接转发
                [&](auto& inner) {
                    inner.system(
                        tf_buffer, bullet_speed, detecting_color, imu_state, control_out, gt_out);
                }},
            impl);
    }
};

} // 匿名命名空间结束

/**
 * @brief L1底层硬件统一初始化入口
 * 根据HardwareBackendConfig区分Daedalus一体化 / Direct分体MCU两种架构
 * 创建相机、云台输出、IMU硬件实例，向Scheduler注册固定频率系统
 * @param world ECS全局资源容器
 * @param scheduler Talos调度器，管理周期系统
 * @param cfg 顶层硬件variant配置 std::variant<DaedalusConfig, DirectConfig>
 * @return 成功返回相机配置；失败返回std::expected错误字符串
 */
std::expected<L1L2SetupResult, std::string> setup_l1(
    talos::World& world, talos::Scheduler& scheduler, hardware::HardwareBackendConfig cfg) {
    // 相机输入、武器输出接口句柄
    std::unique_ptr<fcs::L1::CameraInterface> camera;
    std::unique_ptr<fcs::L1::OutputInterface> output;
    // IMU统一封装实例
    auto imu       = std::make_shared<Imu>();
    bool imu_ready = false; // IMU硬件是否初始化完成标志
    // 预先插入全局IMU姿态资源
    world.insert_resource(core::ImuState{});

    // std::visit 分发顶层硬件配置分支：Daedalus / Direct分体MCU
    std::expected<void, std::string> result = std::visit(
        overloaded{
            // 分支1：Daedalus一体化IPC设备
            [&camera, &output, &imu,
             &imu_ready](hardware::DaedalusConfig) -> std::expected<void, std::string> {
                SPDLOG_INFO("connecting to daedalus ipc...");
                // 连接共享内存IPC
                auto ipc_ = ipc::ShmClient::connect();
                if (!ipc_) {
                    return std::unexpected(fmt::format("connect ipc: {}", ipc_.error()));
                }
                auto ipc_device = std::make_shared<ipc::ShmClient>(std::move(*ipc_));

                // 创建IPC相机输入接口
                SPDLOG_INFO("initializing daedalus input interface...");
                auto input_result = fcs::L1::CameraInterface::create_ipc(ipc_device);
                if (!input_result) {
                    return std::unexpected(
                        fmt::format("init daedalus input: {}", input_result.error()));
                }
                // 创建IPC武器输出接口
                SPDLOG_INFO("initializing daedalus output interface...");
                auto out_result = fcs::L1::OutputInterface::create_ipc(ipc_device);
                if (!out_result) {
                    return std::unexpected(
                        fmt::format("init daedalus output: {}", out_result.error()));
                }

                camera =
                    std::make_unique<fcs::L1::CameraInterface>(std::move(input_result.value()));
                output = std::make_unique<fcs::L1::OutputInterface>(std::move(out_result.value()));
                // variant存入DaedalusImu实现
                imu->impl = DaedalusImu{ipc_device};
                imu_ready = true;
                return {};
            },

            // 分支2：Direct分体硬件（Serial/USB MCU + 海康相机 / Chiral云台）
            [&camera, &output, &imu,
             &imu_ready](hardware::DirectConfig cfg) -> std::expected<void, std::string> {
                // 清空全局串口解析缓冲区
                g_imu_data                                     = {};
                g_capabilities_data                            = {};
                talos_gimbal::Stm32Parser::latest_imu          = &g_imu_data;
                talos_gimbal::Stm32Parser::latest_capabilities = nullptr;

                auto mcu_config = cfg.hardware.mcu;
                // 非仅相机模式：初始化MCU云台通信
                if (!cfg.camera_only) {
                    std::shared_ptr<McuHandle> mcu_device;
                    SPDLOG_INFO(
                        "MCU authoritative self_color={} bullet_speed={}",
                        mcu_config->mcu_authoritative_self_color,
                        mcu_config->mcu_authoritative_bullet_speed);
                    // 直连串口/USB MCU
                    if (cfg.transport == hardware::Transport::Direct) {
                        // 串口后端
                        if (mcu_config->mcu_backend == fcs::McuBackend::Serial) {
                            SPDLOG_INFO(
                                "connecting to mcu via serial with {} @ {} baud",
                                mcu_config->serial_device, mcu_config->serial_baud_rate);
                            auto device_result = McuHandle::create_serial(
                                mcu_config->serial_device, mcu_config->serial_baud_rate);
                            if (!device_result) {
                                return std::unexpected(device_result.error());
                            }
                            mcu_device =
                                std::make_shared<McuHandle>(std::move(device_result).value());
                        } else {
                            // USB HID后端，开启能力包解析
                            if (mcu_config->mcu_product_id) {
                                SPDLOG_INFO(
                                    "connecting to mcu via USB with {:#x}:{:#x}",
                                    mcu_config->mcu_vendor_id, mcu_config->mcu_product_id);
                            } else {
                                SPDLOG_INFO(
                                    "connecting to mcu via USB with {:#x} (product_id unspecified)",
                                    mcu_config->mcu_vendor_id);
                            }
                            auto device_result = McuHandle::create_usb(
                                mcu_config->mcu_vendor_id, mcu_config->mcu_product_id);
                            if (!device_result) {
                                return std::unexpected(device_result.error());
                            }
                            mcu_device =
                                std::make_shared<McuHandle>(std::move(device_result).value());
                            talos_gimbal::Stm32Parser::latest_capabilities = &g_capabilities_data;
                        }
                        SPDLOG_INFO("connected to mcu via {}", mcu_config->mcu_backend);
                        // 创建MCU武器输出接口
                        output = std::make_unique<fcs::L1::OutputInterface>(
                            fcs::L1::McuOutput(mcu_device));
                        // variant存入McuImu
                        imu->impl = McuImu{
                            mcu_config, mcu_device, &g_imu_data,
                            mcu_config->mcu_backend == fcs::McuBackend::Usb ? &g_capabilities_data
                                                                            : nullptr};
                        imu_ready = true;
                    } else {
                        // Chiral私有云台协议
                        SPDLOG_INFO("initializing chiral mcu...");
                        auto chiral_result = talos::chiral::gimbal::TalosEndpoint::create();
                        if (!chiral_result) {
                            SPDLOG_INFO(
                                "failed to create chiral mcu: {}",
                                magic_enum::enum_name(chiral_result.error()));
                            return std::unexpected(
                                std::string(magic_enum::enum_name(chiral_result.error())));
                        }
                        std::shared_ptr<talos::chiral::gimbal::TalosEndpoint> chiral =
                            std::move(chiral_result.value());

                        output = std::make_unique<fcs::L1::OutputInterface>(
                            fcs::L1::ChiralOutput(chiral));
                        imu->impl = ChiralImu{
                            mcu_config,
                            chiral,
                        };
                        imu_ready = true;
                    }
                }
                // 初始化海康工业相机
                SPDLOG_INFO("initializing hikrobot camera...");
                auto input_result = fcs::L1::CameraInterface::create_hik(cfg.hardware.camera);
                if (!input_result) {
                    return std::unexpected(
                        fmt::format("init hikrobot camera: {}", input_result.error()));
                }
                camera =
                    std::make_unique<fcs::L1::CameraInterface>(std::move(input_result.value()));
                return {};
            }},
        cfg);
    // 硬件初始化失败，向上返回错误信息
    if (!result) {
        return std::unexpected(result.error());
    }

    // 将相机内参存入全局ECS资源
    const auto& cam_info = camera->camera_info();
    world.insert_resource(cam_info);

    // IMU硬件正常，注册1000Hz周期IMU读取系统
    if (imu_ready) {
        scheduler.add_system<talos::fixed_rate<1000, 1>>(
            "imu_reader",
            [imu = std::move(imu)](
                talos::res_mut<fast_tf::CoordinateSystem> tf_buffer,
                core::trajectory::bullet_speed_mut bullet_speed,
                core::detecting_color_mut detecting_color, core::capabilities_mut capabilities,
                core::following_mut following, core::imu_state_mut imu_state,
                talos::spmc_mut<core::ControlResourceSnapshot, RuntimeControlStateChannelTopic>
                    control_out,
                talos::spmc_mut<ipc::GroundTruthBatch, GroundTruthBatchChannelTopic> gt_out) {
                imu->system(
                    tf_buffer, bullet_speed, detecting_color, capabilities, following, imu_state,
                    control_out, gt_out);
            });
    } else {
        SPDLOG_WARN("pretending mcu is connected, no mcu data will be available");
    }

    // 相机读取系统 250Hz，输出图像帧消息通道
    auto last_seq = std::make_shared<std::atomic<uint64_t>>(0);
    scheduler.add_system<talos::fixed_rate<250, 1>>(
        "camera_reader", [camera = std::move(camera),
                          last_seq](talos::spmc_mut<ImageFrame, ImageChannelTopic> cam_out) {
            using namespace std::chrono_literals;
            // 阻塞1秒等待图像帧
            const auto frame = camera->recv(1s);
            if (!frame) [[unlikely]] {
                SPDLOG_ERROR("read camera: {}", frame.error());
                return;
            }

            // 帧序号丢帧检测日志
            const auto seq  = frame->seq;
            const auto prev = last_seq->load(std::memory_order_relaxed);
            if (seq != prev + 1 && prev > 0) {
                SPDLOG_DEBUG("skip {} frame", seq - prev - 1);
            }
            last_seq->store(seq, std::memory_order_relaxed);

            // 图像帧写入IPC通道供上层识别使用
            auto img = frame->image;
            cam_out.write(fcs::ImageFrame{std::move(img), frame->timestamp_ns, seq});
        });

    // 武器输出设备存入全局资源
    scheduler.world().insert_resource(std::move(output));

    // IMU就绪时注册250Hz武器发射指令下发系统
    if (imu_ready) {
        scheduler.add_system<talos::fixed_rate<250, 1>>(
            "weapon_output",
            [](talos::res<std::unique_ptr<fcs::L1::OutputInterface>> output,
               talos::spmc<fcs::L5::WeaponCommand, fcs::WeaponCommandChannelTopic> cmd_in) {
                // 读取上层下发的发射指令
                auto cmd = cmd_in.read();
                if (!cmd) [[unlikely]] {
                    return;
                }
                // 下发到硬件MCU/IPC
                (*output)->send(*cmd);
            });
    }

    // 返回初始化结果，携带相机标定信息
    return L1L2SetupResult{.camera_config = cam_info};
}

} // namespace fcs::runtime