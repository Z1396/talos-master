// 头文件保护，避免多次包含引发编译重定义错误
#pragma once

// 线性代数库 Eigen，用于三维向量、位姿、坐标变换
#include <Eigen/Core>
// 标准定长整型
#include <cstdint>
// fmt 高性能格式化库核心头文件
#include <fmt/core.h>
// 可选值类型，字段可空时使用
#include <optional>
// 标准字符串
#include <string>

// 各分层业务模块配置头文件（按机器人系统分层 L2~L5）
// L2 感知层：装甲板、激光雷达、能量机关
#include "L2_perception/armor/config.hpp"
#include "L2_perception/ldm/ldm_config.hpp"
#include "L2_perception/rune/rune_config.hpp"
// L3 估计/预测层：能量计、激光雷达预测、目标跟踪器
#include "L3_estimation/energy_meter/energy_meter_config.hpp"
#include "L3_estimation/ldm_naive/ldm_naive_config.hpp"
#include "L3_estimation/tracker/config.hpp"
// L4 规划决策层
#include "L4_planning/config.hpp"
// L5 武器控制层
#include "L5_weapon/config.hpp"

// 相机配置、机器人核心类型、运行时环境、弹道配置、欧拉角工具类
#include "camera_config.hpp"
#include "core/armor_types.hpp"
#include "core/runtime.hpp"
#include "core/trajectory/config.hpp"
#include "euler.hpp"

// 数据流编码参数配置
#include "quanta/stream_encoder.hpp"
// 枚举字符串反射库，快速将枚举值转为对应文本名称
#include <magic_enum.hpp>

// 项目顶层命名空间
namespace fcs {

/**
 * @brief 硬件后端类型枚举
 * 定义机器人整机硬件通信/驱动模式
 */
enum HardwareBackend {
    Direct,     // 直连模式：硬件与程序本地直连
    Daedalus,   // Daedalus 专用设备模式（发射/执行机构）
    Chiral,     // 数据转发/分流模式
    CameraOnly  // 仅启用相机，关闭其余硬件设备
};

/**
 * @brief 数据采集器配置
 * 控制图像、传感器数据录制相关参数
 */
struct CapturerConfig {
    bool enabled{false};                // 是否开启数据录制功能，默认关闭
    std::string output_dir{};           // 录制文件输出目录
    // 磁盘预留空间：5GB，剩余空间低于该值时停止录制，防止磁盘占满
    uint64_t reserved_free_bytes{5ULL * 1024ULL * 1024ULL * 1024ULL};
};

/**
 * @brief 类型别名：角度单位，统一使用浮点型 double 表示角度
 */
using degree = double;

/**
 * @brief 机器人外参配置
 * 描述云台、相机、炮口之间的三维坐标偏移与姿态旋转关系（标定参数）
 * 采用嵌套结构体逐级描述机械结构：偏航轴 -> 俯仰轴 -> 相机/炮口
 */
struct RobotExtrinsicConfig {
    /**
     * @brief 云台偏航轴（整体左右转动）
     */
    struct gimbal_yaw_t {
        /**
         * @brief 云台俯仰轴（整体上下转动）
         */
        struct gimbal_pitch_t {
            /**
             * @brief 相机连杆坐标系
             * 存储相机相对上级机构的平移、旋转姿态
             */
            struct camera_link_t {
                Eigen::Vector3d translation{};  // 三维平移向量 (x,y,z)
                degree roll{};                 // 横滚角
                degree pitch{};                // 俯仰角
                degree yaw{};                  // 偏航角

                /**
                 * @brief 将 RPY 欧拉角转为 ROS 标准旋转对象
                 * @return 欧拉角旋转结构体
                 * noexcept 保证无异常抛出
                 */
                math_fuxk::Ros2EulerRotd rotation() const noexcept {
                    return math_fuxk::rpy(roll, pitch, yaw);
                }
            };

            /**
             * @brief 炮口连杆坐标系
             * 描述发射口相对上级机构的位姿
             */
            struct muzzle_link_t {
                Eigen::Vector3d translation{};  // 三维平移向量
                degree roll{};                 // 横滚角
                degree pitch{};                // 俯仰角
                degree yaw{};                  // 偏航角

                /**
                 * @brief 转换为 ROS 标准欧拉旋转对象
                 */
                math_fuxk::Ros2EulerRotd rotation() const noexcept {
                    return math_fuxk::rpy(roll, pitch, yaw);
                }
            };

            Eigen::Vector3d translation{};   // 俯仰轴自身平移向量
            camera_link_t camera_link{};     // 挂载的相机连杆参数
            muzzle_link_t muzzle_link{};     // 挂载的炮口连杆参数
        };

        gimbal_pitch_t gimbal_pitch{};      // 偏航轴下的俯仰轴
    };

    gimbal_yaw_t gimbal_yaw{};             // 整机顶层：云台偏航轴
};

/**
 * @brief 下位机通信后端枚举
 * uint8_t 节省内存，限定取值范围
 */
enum class McuBackend : uint8_t {
    Usb,        // USB 虚拟串口通信
    Serial,     // 物理串口通信
};

/**
 * @brief 下位机(MCU)通信与控制配置
 * 机器人主控板通信、弹速、机体颜色等参数
 */
struct McuConfig {
    McuBackend mcu_backend{McuBackend::Usb};  // 通信方式，默认USB
    uint16_t mcu_vendor_id{0x0483};           // USB 厂商ID，默认ST设备
    std::optional<uint16_t> mcu_product_id{std::nullopt}; // USB 产品ID，可选（不填则模糊匹配）
    std::string serial_device{"/dev/ttyS4"};  // Linux 串口设备路径
    int serial_baud_rate{115200};              // 串口波特率

    bool mcu_authoritative_self_color{true};  // true：由下位机判定己方颜色
    bool mcu_authoritative_bullet_speed{true};// true：由下位机决定实际弹丸速度

    double bullet_speed_default{22.0};        // 默认弹速
    double bullet_speed_min{25.0};            // 弹速下限
    double bullet_speed_max{15.0};            // 弹速上限
};

/**
 * @brief 底层硬件总配置
 * 整合相机、下位机、机械外参三大核心硬件，均为**必填配置项**
 */
struct HardwareConfig {
    // toml_helper::required：TOML解析标记，缺失该字段直接解析报错
    toml_helper::required<CameraConfig> camera{};    // 相机配置（必填）
    toml_helper::required<McuConfig> mcu{};         // 下位机配置（必填）
    toml_helper::required<RobotExtrinsicConfig> extrinsic{}; // 机械外参（必填）

    bool chiral{false};  // 是否启用数据分流/转发模式
};

/**
 * @brief 视觉业务总配置
 * 整合 L2~L5 全分层算法、编码、弹道、功能能力等参数
 * toml_helper::flatten：TOML 解析时将子结构体字段扁平化，不生成嵌套节点
 */
struct VisionConfig {
    ArmorColor detect_color;                          // 需要识别的装甲板颜色

    // 机器人启用的功能能力列表（如识别、预测、打弹等），必填项
    toml_helper::required<std::vector<core::Capability>> capabilities;

    quanta::EncodeParams quanta{};                   // 数据流编码参数
    quanta::FilterParams quanta_filter{};            // 数据流滤波参数
    core::trajectory::TrajectoryConfig trajectory{}; // 弹道解算配置

    // L2 感知层各算法配置（扁平化解析）
    toml_helper::flatten<L2::ArmorDetectorConfig> armor{};  // 装甲板检测
    rune::RuneDetectorConfig rune_detector{};              // 能量机关检测
    L2::ldm::LdmDetectorConfig ldm{};                      // 激光雷达目标检测

    // L3 估计/预测层配置
    toml_helper::flatten<L3::L3Config> l3{};               // L3 总配置（扁平化）
    energy_meter::EnergyMeterL3Config energy_meter{};      // 能量计算法
    L3::ldm::NaiveLdmConfig naive_ldm{};                   // 激光雷达简易预测

    // L4 规划决策层配置
    toml_helper::flatten<L4::L4Config> l4{};

    // L5 武器控制层配置
    toml_helper::flatten<L5::L5Config> l5{};

    /**
     * @brief 只读获取跟踪器配置
     * @return L3 目标跟踪器配置引用
     * noexcept 无异常，const 保证不修改数据
     */
    [[nodiscard]] const L3::TrackerConfig& tracker() const noexcept { return l3->tracker; }

    /**
     * @brief 只读获取武器控制器配置
     * @return L5 武器MPC控制器配置引用
     */
    [[nodiscard]] const L5::WeaponControllerConfig& weapon() const noexcept {
        return l5->mpc_weapon;
    }

    /**
     * @brief 可写获取跟踪器配置，允许运行时修改参数
     */
    [[nodiscard]] L3::TrackerConfig& tracker() noexcept { return l3->tracker; }

    /**
     * @brief 可写获取武器控制器配置，允许运行时修改参数
     */
    [[nodiscard]] L5::WeaponControllerConfig& weapon() noexcept {
        return l5->mpc_weapon;
    }
};

} // namespace fcs

// ============================================================================
// fmt 库格式化特化：让枚举直接支持 fmt::print / 日志输出
// 按规范必须定义在 fmt 命名空间内
// ============================================================================
namespace fmt {

/**
 * @brief 下位机通信枚举格式化器
 * 借助 magic_enum 自动将枚举名转为字符串，用于日志打印
 */
template <>
struct formatter<fcs::McuBackend> : formatter<std::string_view> {
    auto format(fcs::McuBackend b, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(b), ctx);
    }
};

/**
 * @brief 硬件后端枚举格式化器
 */
template <>
struct formatter<fcs::HardwareBackend> : formatter<std::string_view> {
    auto format(fcs::HardwareBackend b, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(b), ctx);
    }
};

} // namespace fmt