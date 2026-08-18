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
 * @brief Daedalus仿真器的IMU实现
 * 注意：这不是真实硬件IMU，是仿真后端；通过IPC共享内存从仿真器读取机器人各类连杆位姿，填充TF树、IMU状态、真值通道
 */
struct DaedalusImu {
    // IPC共享内存客户端：负责和Daedalus仿真进程通信，recv_xxx从共享内存读取仿真输出数据
    std::shared_ptr<ipc::ShmClient> ipc_device;

    // 记录上一次真值包时间戳，避免同一帧真值重复写入SPMC通道，防止重复消费
    uint64_t last_gt_timestamp_ns{0};

    /**
     * @brief Talos ECS系统入口函数，调度器固定周期调用
     * 所有带 _mut 后缀都是资源/通道访问器：res_mut<> 代表读写访问world资源；spmc_mut<>代表写SPMC通道
     * noexcept：函数承诺不会抛出C++异常
     *
     * @param tf_buffer 姿态变换树资源，读写，更新各个坐标系之间位姿
     * @param bullet_speed 子弹速度全局资源，[[maybe_unused]]标记参数暂时没使用，消除编译器“未使用参数”警告
     * @param detecting_color 识别颜色配置资源
     * @param following 自动跟随开关（原子布尔）
     * @param imu_state IMU状态全局资源，仿真中直接从仿真位姿解算roll/pitch/yaw填入
     * @param control_out SPMC写端点：输出控制快照快照通道
     * @param gt_out SPMC写端点：输出GroundTruth仿真真值数据包通道
     */
    void system(
        talos::res_mut<fast_tf::CoordinateSystem> tf_buffer,
        [[maybe_unused]] core::trajectory::bullet_speed_mut bullet_speed,
        [[maybe_unused]] core::detecting_color_mut detecting_color, core::following_mut following,
        [[maybe_unused]] core::imu_state_mut imu_state,
        talos::spmc_mut<core::ControlResourceSnapshot, RuntimeControlStateChannelTopic> control_out,
        talos::spmc_mut<ipc::GroundTruthBatch, GroundTruthBatchChannelTopic> gt_out) noexcept
    {
        // 标记：本调度周期是否读到有效的位姿更新
        bool control_updated           = false;
        // 记录本周期最新的位姿时间戳（纳秒）
        uint64_t control_timestamp_ns  = 0;

        /**
         * 局部捕获lambda：一旦读到任意一个连杆位姿，就调用这个lambda
         * 标记control_updated=true，同时保存最大时间戳；多个不同连杆可能时间戳不一样，取std::max最新时间
         * [&] 引用捕获：全部外部变量按引用访问
         */
        const auto note_control_update = [&](uint64_t timestamp_ns) {
            control_updated      = true;
            control_timestamp_ns = std::max(control_timestamp_ns, timestamp_ns);
        };

        // ==============================================
        // 1、读取云台(gimbal)仿真位姿，更新TF树 + 填充IMU欧拉角
        // ipc_device->recv_pose()：从共享内存读取指定类型位姿；返回std::optional<>，没有新数据就std::nullopt
        // if (auto val = opt) C++17结构化if初始化，只有有值才进入分支
        // ==============================================
        if (const auto pose = ipc_device->recv_pose(ipc::POSE_GIMBAL)) {
            // pose->qw/qx/qy/qz：仿真输出四元数，构造Eigen四元数对象
            const Eigen::Quaterniond q(pose->qw, pose->qx, pose->qy, pose->qz);
            // 四元数转RPY欧拉角
            const auto euler              = math_fuxk::rpy(q);
            // 结构化绑定：把euler.rpy()返回tuple解包给roll pitch yaw三个变量
            const auto [roll, pitch, yaw] = euler.rpy();

            // 更新【gimbal】坐标系变换：四元数 + xyz平移，时间戳来自仿真数据包
            fast_tf::update(
                *tf_buffer,
                fast_tf::EdgeTransform<fast_tf::gimbal>::from_quaternion_xyz(
                    q, pose->x, pose->y, pose->z),
                pose->timestamp_ns);

            // 更新gimbal_pitch_fuxk_frame俯仰轴坐标系，这里填0旋转，仿真预留帧
            fast_tf::update(
                *tf_buffer,
                fast_tf::EdgeTransform<fast_tf::gimbal_pitch_fuxk_frame>::from_rpy(0, 0, 0),
                pose->timestamp_ns);

            // 把仿真解算出来欧拉角写入全局ImuState资源
            imu_state->timestamp_ns = pose->timestamp_ns;
            imu_state->yaw          = yaw;
            imu_state->pitch        = pitch;
            imu_state->roll         = roll;

            // ⚠️ Daedalus仿真IMU模块**不输出角速度**，角速度直接置0
            imu_state->yaw_vel      = 0.0;
            imu_state->pitch_vel    = 0.0;
            imu_state->roll_vel     = 0.0;

            // 标记本周期有位姿更新，记录时间戳
            note_control_update(pose->timestamp_ns);
        }

        // ==============================================
        // 2、读取底盘odom坐标系位姿（只有平移，旋转由云台承担）更新TF树
        // ==============================================
        if (const auto pose = ipc_device->recv_pose(ipc::POSE_ODOM)) {
            fast_tf::update(
                *tf_buffer,
                fast_tf::EdgeTransform<fast_tf::odom>::from_translation(pose->x, pose->y, pose->z),
                pose->timestamp_ns);
            note_control_update(pose->timestamp_ns);
        }

        // ==============================================
        // 3、相机连杆位姿，更新TF树；视觉模块依赖这个坐标系做PnP、3D解算
        // ==============================================
        if (const auto pose = ipc_device->recv_pose(ipc::POSE_CAMERA)) {
            fast_tf::update(
                *tf_buffer,
                fast_tf::EdgeTransform<fast_tf::camera_link_fuxk_frame>::from_translation(
                    pose->x, pose->y, pose->z),
                pose->timestamp_ns);
            note_control_update(pose->timestamp_ns);
        }

        // ==============================================
        // 4、枪口muzzle连杆完整位姿，弹道解算核心输入！炮口的位置姿态
        // ==============================================
        if (const auto pose = ipc_device->recv_pose(ipc::POSE_MUZZLE)) {
            const Eigen::Quaterniond q(pose->qw, pose->qx, pose->qy, pose->qz);
            fast_tf::update(
                *tf_buffer,
                fast_tf::EdgeTransform<fast_tf::muzzle_link_fuxk_frame>::from_quaternion_xyz(
                    q, pose->x, pose->y, pose->z),
                pose->timestamp_ns);
            note_control_update(pose->timestamp_ns);
        }

        // 读取底盘观测数据包，当前仅预留接口，TODO后续扩展底盘速度、碰撞
        if (const auto obs = ipc_device->recv_chassis_observation()) {
            // TODO：底盘速度、碰撞检测扩展
        }

        // ==============================================
        // 读取仿真运行时状态：自动跟随开关 following
        // following 是原子资源，load()读、store()写，多线程安全
        // ==============================================
        if (const auto state = ipc_device->recv_runtime_state()) {
            // state->following非0代表开启自动跟随
            const auto next_following = state->following != 0U;

            // 如果开关状态发生变化，才执行store更新并打印日志，避免频繁写原子变量打日志
            if (next_following != following->load()) {
                following->store(next_following);
                SPDLOG_INFO("daedalus following changed: {}", next_following);
            }
        }

        // ==============================================
        // 接收仿真GroundTruth真值包，写入SPMC通道供录制/调试使用
        // last_gt_timestamp_ns做去重：同一时间戳数据包不重复写通道，防止重复消费
        // ==============================================
        if (const auto gt = ipc_device->recv_ground_truth()) {
            if (gt->timestamp_ns != last_gt_timestamp_ns) {
                last_gt_timestamp_ns = gt->timestamp_ns;
                gt_out.write(*gt); // spmc_mut写端点，把真值包写入SPMC通道
            }
        }

        // ==============================================
        // 如果本周期读到任意有效位姿更新：发布一份完整控制快照到SPMC通道
        // 下游瞄准、录制模块消费control_out通道的数据
        // ==============================================
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

// IMU统一抽象封装体
// 核心目标：对外暴露**唯一、不变的system()接口**，内部自动兼容3种IMU底层实现
// 不用写一堆if-else判断是仿真/串口MCU/Chiral云台，用std::variant+std::visit做静态分发
struct Imu {
    // variant：类型安全的联合体，同一时刻只会存下面三者中的**某一个实例**
    //  - DaedalusImu：Bevy仿真器IPC虚拟IMU
    //  - McuImu：实物STM32(串口/USB HID)上报的IMU
    //  - ChiralImu：Chiral私有云台协议的IMU
    std::variant<DaedalusImu, McuImu, ChiralImu> impl;

    /**
     * @brief 统一周期入口，自动分发到对应硬件实现的system函数
     * 这个函数会被调度器以1000Hz固定频率调用（就是上一段setup_l1里注册的imu_reader系统）
     */
    void system(
        talos::res_mut<fast_tf::CoordinateSystem> tf_buffer,      //【可写ECS资源】坐标变换缓存（fast_tf，维护坐标系树、插值查询）
        core::trajectory::bullet_speed_mut bullet_speed,          //【可写ECS资源】弹速配置，IMU/云台上报后更新
        core::detecting_color_mut detecting_color,                //【可写ECS资源】敌方识别颜色（红/蓝，云台MCU上报权威颜色）
        core::capabilities_mut capabilities,                      //【可写ECS资源】云台硬件能力包（底盘/云台限位、功率等，USB MCU才会上报）
        core::following_mut following,                        //【可写ECS资源】云台跟随状态
        core::imu_state_mut imu_state,                          //【可写ECS资源】IMU原始姿态/角速度输出，全框架共用的IMU状态结构体
        talos::spmc_mut<core::ControlResourceSnapshot, RuntimeControlStateChannelTopic> control_out, //【SPMC写端】控制快照通道，输出当前云台状态给上层、Foxglove可视化
        talos::spmc_mut<ipc::GroundTruthBatch, GroundTruthBatchChannelTopic> gt_out) noexcept   //【SPMC写端】真值数据通道，仿真时输出GroundTruth真值
    {
        // std::visit：访问variant里面当前存的那个类型实例
        // overloaded{}：语法糖，把多个lambda打包成一个可重载的访问器，编译期自动匹配variant内类型
        std::visit(
            overloaded{
                // 分支1：如果variant里存的是 McuImu（实物STM32串口/USB IMU）
                [&](McuImu& inner) {
                    // 转发调用McuImu自己实现的system，把全部参数原样传进去
                    inner.system(
                        tf_buffer, bullet_speed, detecting_color, capabilities, following,
                        imu_state, control_out, gt_out);
                },
                // 分支2：如果variant里存的是 DaedalusImu（仿真虚拟IMU）
                [&](DaedalusImu& inner) {
                    // 注意：Daedalus仿真IMU不需要capabilities参数，所以传参列表少一个
                    inner.system(
                        tf_buffer, bullet_speed, detecting_color, following, imu_state, control_out,
                        gt_out);
                },
                // 分支3【泛型兜底auto&】：剩下所有类型（这里就是ChiralImu）走这个lambda
                // auto& 是模板推导，编译期实例化，不用单独手写ChiralImu的lambda
                // ChiralImu的system参数更少，不需要following、capabilities
                [&](auto& inner) {
                    inner.system(
                        tf_buffer, bullet_speed, detecting_color, imu_state, control_out, gt_out);
                }
            },
            impl // 传入variant实例，std::visit会判断impl当前是哪一种，自动匹配上面lambda
        );
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
// 函数返回值：成功返回L1L2SetupResult结构体；失败返回string类型错误信息
// 作用：L1硬件层统一初始化入口，兼容两种硬件方案：Daedalus仿真IPC / Direct实物硬件（串口/USB/Chiral云台+海康相机）
std::expected<L1L2SetupResult, std::string> setup_l1(
    talos::World& world,                // ECS世界实例，存放全局资源(IMU状态、相机内参等)
    talos::Scheduler& scheduler,        // Talos拓扑调度器，用来注册固定周期运行的系统(IMU读取、相机读取、武器输出)
    hardware::HardwareBackendConfig cfg) // 顶层硬件配置variant：二选一 DaedalusConfig / DirectConfig
{
    // ===================== 局部句柄定义 =====================
    // 相机输入接口抽象（统一封装：仿真IPC相机 / 实物海康相机）
    std::unique_ptr<fcs::L1::CameraInterface> camera;
    // 武器输出接口抽象（统一封装：仿真IPC输出 / 实物MCU串口/USB/Chiral云台发射）
    std::unique_ptr<fcs::L1::OutputInterface> output;
    // IMU统一抽象封装（内部用variant包装不同实现：DaedalusImu / McuImu / ChiralImu）
    auto imu       = std::make_shared<Imu>();
    bool imu_ready = false; // 标记：IMU/MCU硬件是否初始化成功，后面用来决定要不要注册IMU、武器输出系统

    // 在ECS全局资源里预先插入IMU状态存储，给后面IMU读取系统写入姿态数据
    world.insert_resource(core::ImuState{});

    // ===================== Variant分发：根据硬件配置类型自动选择初始化分支 =====================
    // std::visit：访问variant（HardwareBackendConfig是std::variant<DaedalusConfig,DirectConfig>）
    // overloaded{}：语法糖，批量放多个lambda，自动匹配variant当前类型
    std::expected<void, std::string> result = std::visit(overloaded{
            // ========== 分支1：Daedalus一体化IPC（你的Bevy仿真器，共享内存IPC通信） ==========
            // 捕获&：引用外部所有局部变量camera/output/imu/imu_ready，在lambda内修改
            [&camera, &output, &imu,&imu_ready](hardware::DaedalusConfig) -> std::expected<void, std::string> {
                SPDLOG_INFO("connecting to daedalus ipc..."); // 日志：开始连接仿真共享内存IPC

                // 创建ShmClient共享内存IPC客户端，连接Daedalus仿真
                auto ipc_ = ipc::ShmClient::connect();
                // 判断IPC连接是否失败
                if (!ipc_) {
                    // 返回错误，包装字符串，向上抛出
                    return std::unexpected(fmt::format("connect ipc: {}", ipc_.error()));
                }
                // 把成功的IPC客户端move进shared_ptr，给相机、IMU、输出共用
                auto ipc_device = std::make_shared<ipc::ShmClient>(std::move(*ipc_));

                SPDLOG_INFO("initializing daedalus input interface...");
                // 创建仿真端相机输入接口（从共享内存拿仿真图像）
                auto input_result = fcs::L1::CameraInterface::create_ipc(ipc_device);
                if (!input_result) { // 相机IPC接口创建失败
                    return std::unexpected(
                        fmt::format("init daedalus input: {}", input_result.error()));
                }
                SPDLOG_INFO("initializing daedalus output interface...");
                // 创建仿真武器输出接口（发射指令写入共享内存给仿真器）
                auto out_result = fcs::L1::OutputInterface::create_ipc(ipc_device);
                if (!out_result) { // 输出IPC接口创建失败
                    return std::unexpected(
                        fmt::format("init daedalus output: {}", out_result.error()));
                }

                // 转移所有权，构造CameraInterface实例
                camera = std::make_unique<fcs::L1::CameraInterface>(std::move(input_result.value()));
                // 转移所有权，构造OutputInterface实例
                output = std::make_unique<fcs::L1::OutputInterface>(std::move(out_result.value()));
                // 给IMU抽象层绑定Daedalus仿真IMU实现，共用同一个IPC句柄
                imu->impl = DaedalusImu{ipc_device};
                imu_ready = true; // IMU就绪标记置true，后面会注册IMU、武器系统
                return {}; // 成功，返回空expected<void>
            },

            // ========== 分支2：Direct分体实物硬件（MCU+海康相机，支持串口/USB HID/Chiral私有云台） ==========
            [&camera, &output, &imu,&imu_ready](hardware::DirectConfig cfg) -> std::expected<void, std::string> {
                // 全局静态缓冲区清零：存放MCU上报IMU数据、云台能力数据
                g_imu_data                                     = {};
                g_capabilities_data                            = {};
                // 给STM32解析器挂全局IMU数据指针，解析回调直接写这块内存
                talos_gimbal::Stm32Parser::latest_imu          = &g_imu_data;
                // 默认先不启用能力包解析
                talos_gimbal::Stm32Parser::latest_capabilities = nullptr;

                // 取出MCU子配置
                auto mcu_config = cfg.hardware.mcu;
                // 判断：不是【仅相机模式】→ 需要初始化MCU云台、IMU、发射输出
                if (!cfg.camera_only) {
                    std::shared_ptr<McuHandle> mcu_device; // MCU通信句柄（串口/USB）
                    SPDLOG_INFO(
                        "MCU authoritative self_color={} bullet_speed={}",
                        mcu_config->mcu_authoritative_self_color,
                        mcu_config->mcu_authoritative_bullet_speed);

                    // 判断底层传输方式：Direct（串口/USB HID） 或者 Chiral私有云台协议
                    if (cfg.transport == hardware::Transport::Direct) {
                        // 子分支：MCU后端=Serial串口
                        if (mcu_config->mcu_backend == fcs::McuBackend::Serial) {
                            SPDLOG_INFO(
                                "connecting to mcu via serial with {} @ {} baud",
                                mcu_config->serial_device, mcu_config->serial_baud_rate);
                            // 创建串口MCU句柄：设备名+波特率
                            auto device_result = McuHandle::create_serial(mcu_config->serial_device, mcu_config->serial_baud_rate);
                            if (!device_result) { // 串口打开失败
                                return std::unexpected(device_result.error());
                            }
                            // 转移构造MCU共享句柄
                            mcu_device = std::make_shared<McuHandle>(std::move(device_result).value());
                        } else {
                            // 子分支：MCU后端=USB HID
                            if (mcu_config->mcu_product_id) {
                                SPDLOG_INFO(
                                    "connecting to mcu via USB with {:#x}:{:#x}",
                                    mcu_config->mcu_vendor_id, mcu_config->mcu_product_id);
                            } else {
                                SPDLOG_INFO(
                                    "connecting to mcu via USB with {:#x} (product_id unspecified)",
                                    mcu_config->mcu_vendor_id);
                            }
                            // 创建USB HID MCU句柄（VID/PID）
                            auto device_result = McuHandle::create_usb(mcu_config->mcu_vendor_id, mcu_config->mcu_product_id);
                            if (!device_result) { // USB打开失败
                                return std::unexpected(device_result.error());
                            }
                            // 转移构造MCU共享句柄
                            mcu_device = std::make_shared<McuHandle>(std::move(device_result).value());
                            std::make_shared<McuHandle>(std::move(device_result).value());
                            // USB模式启用能力包解析，挂载capabilities缓冲区
                            talos_gimbal::Stm32Parser::latest_capabilities = &g_capabilities_data;
                        }
                        SPDLOG_INFO("connected to mcu via {}", mcu_config->mcu_backend);
                        // 构造MCU武器输出接口（后续下发发射指令给STM32）
                        output = std::make_unique<fcs::L1::OutputInterface>(fcs::L1::McuOutput(mcu_device));
                        // IMU抽象绑定McuImu实现，挂载IMU缓冲区、能力缓冲区
                        imu->impl = McuImu{
                            mcu_config, mcu_device, &g_imu_data,
                            mcu_config->mcu_backend == fcs::McuBackend::Usb ? &g_capabilities_data
                                                                            : nullptr};
                        imu_ready = true; // IMU就绪标记打开
                    } else {
                        // 子分支：Chiral私有云台协议
                        SPDLOG_INFO("initializing chiral mcu...");
                        auto chiral_result = talos::chiral::gimbal::TalosEndpoint::create();
                        if (!chiral_result) { // Chiral云台创建失败
                            SPDLOG_INFO(
                                "failed to create chiral mcu: {}",
                                magic_enum::enum_name(chiral_result.error()));
                            return std::unexpected(
                                std::string(magic_enum::enum_name(chiral_result.error())));
                        }
                        std::shared_ptr<talos::chiral::gimbal::TalosEndpoint> chiral =
                            std::move(chiral_result.value());

                        // 构造Chiral武器输出接口
                        output = std::make_unique<fcs::L1::OutputInterface>(fcs::L1::ChiralOutput(chiral));
                        // IMU抽象绑定ChiralImu实现
                        imu->impl = ChiralImu{
                            mcu_config,
                            chiral,
                        };
                        imu_ready = true;
                    }
                }
                // ========== 海康相机初始化（Direct模式共用相机，不管MCU是否启用） ==========
                SPDLOG_INFO("initializing hikrobot camera...");
                auto input_result = fcs::L1::CameraInterface::create_hik(cfg.hardware.camera);
                if (!input_result) { // 海康相机打开失败
                    return std::unexpected(
                        fmt::format("init hikrobot camera: {}", input_result.error()));
                }
                // 构造相机接口，接管海康相机
                camera =
                    std::make_unique<fcs::L1::CameraInterface>(std::move(input_result.value()));
                return {}; // Direct硬件初始化成功
            }},
        cfg); // std::visit传入variant变量cfg
    // ==============================================
    // 上面硬件分支初始化失败 → 直接向上返回错误字符串
    if (!result) {
        return std::unexpected(result.error());
    }

    // 读取相机内参，存入ECS全局资源，给L2识别、PnP解算使用
    const auto& cam_info = camera->camera_info();
    world.insert_resource(cam_info);

    // ===================== IMU就绪：注册 1000Hz IMU读取系统 =====================
    if (imu_ready) {
        // fixed_rate<1000,1>：固定1000Hz周期执行
        scheduler.add_system<talos::fixed_rate<1000, 1>>(
            "imu_reader", // 系统名字（拓扑日志、调试用）
            // 捕获imu，move进lambda，所有权转移
            [imu = std::move(imu)](
                // res_mut：ECS可变资源引用，tf_buffer坐标变换缓存
                talos::res_mut<fast_tf::CoordinateSystem> tf_buffer,
                // 弹道相关资源：弹速、敌方颜色、云台能力、跟随状态、IMU状态
                core::trajectory::bullet_speed_mut bullet_speed,
                core::detecting_color_mut detecting_color, core::capabilities_mut capabilities,
                core::following_mut following, core::imu_state_mut imu_state,
                // spmc_mut：SPMC通道可变写入端 → 输出控制快照、真值数据（给Foxglove可视化/上层）
                talos::spmc_mut<core::ControlResourceSnapshot, RuntimeControlStateChannelTopic> control_out,
                talos::spmc_mut<ipc::GroundTruthBatch, GroundTruthBatchChannelTopic> gt_out) {
                // 调用IMU抽象层system函数：读取IMU、更新TF、写通道
                imu->system(
                    tf_buffer, bullet_speed, detecting_color, capabilities, following, imu_state,
                    control_out, gt_out);
            });
    } else {
        // IMU没就绪（仅相机模式），打印警告：没有MCU/IMU数据
        SPDLOG_WARN("pretending mcu is connected, no mcu data will be available");
    }

    // ===================== 注册 250Hz 相机读取系统 =====================
    // 原子变量记录帧序号，用来检测丢帧，shared_ptr是因为lambda捕获move后多轮循环持续持有
    auto last_seq = std::make_shared<std::atomic<uint64_t>>(0);
    scheduler.add_system<talos::fixed_rate<250, 1>>(
        "camera_reader",
        // 转移camera所有权、last_seq原子计数器到lambda
        [camera = std::move(camera),
          last_seq](talos::spmc_mut<ImageFrame, ImageChannelTopic> cam_out) {
            using namespace std::chrono_literals;
            // 阻塞读取图像帧，最长阻塞1s超时
            const auto frame = camera->recv(1s);
            // 读取失败（超时/断连），打日志直接return
            if (!frame) [[unlikely]] {
                SPDLOG_ERROR("read camera: {}", frame.error());
                return;
            }

            // 取出当前帧号
            const auto seq  = frame->seq;
            // 无锁原子加载上一帧序号，relaxed内存序（纯统计，不需要同步）
            const auto prev = last_seq->load(std::memory_order_relaxed);
            // 检测丢帧：当前seq不是上一帧+1，且不是第一帧 → 打印丢帧Debug日志
            if (seq != prev + 1 && prev > 0) {
                SPDLOG_DEBUG("skip {} frame", seq - prev - 1);
            }
            // 更新原子帧号
            last_seq->store(seq, std::memory_order_relaxed);

            // 提取图像，打包成ImageFrame写入SPMC通道，发给L2识别、Foxglove可视化
            auto img = frame->image;
            cam_out.write(fcs::ImageFrame{std::move(img), frame->timestamp_ns, seq});
        });

    // 把武器输出接口存入Scheduler内的ECS资源，给weapon_output系统读取
    scheduler.world().insert_resource(std::move(output));

    // ===================== IMU就绪：注册250Hz武器发射指令下发系统 =====================
    if (imu_ready) {
        scheduler.add_system<talos::fixed_rate<250, 1>>(
            "weapon_output",
            [](
                // res只读资源：拿到OutputInterface句柄
                talos::res<std::unique_ptr<fcs::L1::OutputInterface>> output,
                // spmc只读消费端：读取L5下发的WeaponCommand发射指令
                talos::spmc<fcs::L5::WeaponCommand, fcs::WeaponCommandChannelTopic> cmd_in) {
                // 从SPMC通道读取最新发射指令（三缓冲快照读）
                auto cmd = cmd_in.read();
                // 无新指令直接返回
                if (!cmd) [[unlikely]] {
                    return;
                }
                // 调用底层接口，下发发射指令到MCU/仿真IPC
                (*output)->send(*cmd);
            });
    }

    // 全部初始化成功，返回结果结构体（携带相机标定信息）
    return L1L2SetupResult{.camera_config = cam_info};
}

} // namespace fcs::runtime