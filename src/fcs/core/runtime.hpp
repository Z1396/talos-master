// 头文件保护，防止重复包含造成重定义编译报错
#pragma once

// 装甲相关枚举定义：ArmorColor 红蓝/中立/紫色目标颜色
#include "core/armor_types.hpp"
// ECS调度器资源组件基础：res<T> 只读资源、res_mut<T> 可写资源包装器
#include "scheduler/system/components.hpp"

// 标准算法库（遍历、查找等）
#include <algorithm>
// 定长数组，固定容量容器
#include <array>
// 原子变量，多线程无锁同步
#include <atomic>
// 固定宽度基础整数类型 uint64_t / uint8_t
#include <cstdint>
// fmt格式化库基础头文件，日志打印专用
#include <fmt/base.h>
// 连续内存只读视图，不持有内存，轻量化批量传参
#include <span>
// 动态变长数组容器
#include <vector>

/**
 * @namespace fcs::core
 * @brief 框架核心全局资源、状态结构体、能力掩码管理模块
 * 存放全框架共用的全局状态结构、多线程安全原子状态、ECS资源别名、功能开关掩码工具
 */
namespace fcs::core {

/**
 * @brief IMU惯性单元实时状态结构体
 * 存储云台IMU原始角速度、角度、时间戳，作为全局只读/可写资源供各系统读取
 */
struct ImuState {
    // 数据采样纳秒时间戳
    uint64_t timestamp_ns{0};
    // 欧拉角（单位 rad）
    double yaw{0.0};   // 偏航角 水平左右旋转
    double pitch{0.0}; // 俯仰角 上下俯仰
    double roll{0.0};  // 横滚角 云台侧倾
    // 三轴角速度 rad/s
    double yaw_vel{0.0};
    double pitch_vel{0.0};
    double roll_vel{0.0};
};

/**
 * @brief 坐标变换快照结构体
 * 存储两组数据：平移向量(xyz)、单位四元数(wxyz)
 * present标记该变换是否有效（硬件未初始化时置false）
 */
struct ControlTransformSnapshot {
    // 变换是否有效：true=数据可用；false=TF查询失败/未初始化
    bool present{false};
    // 平移 [x,y,z]
    std::array<double, 3> translation{};
    // 四元数 [x,y,z,w]
    std::array<double, 4> quaternion{};
};

/**
 * @brief 控制层全局资源完整快照
 * 打包所有机械、视觉、弹道、坐标系静态参数，启动时一次性填充为全局资源
 * 供感知、跟踪、火控系统统一读取
 */
struct ControlResourceSnapshot {
    // 当前快照采样时间戳
    uint64_t sample_timestamp_ns{0};
    // IMU三轴角度、角速度完整状态
    ImuState imu{};
    // 当前开启识别的敌方装甲颜色（红/蓝）
    ArmorColor detecting_color{ArmorColor::Blue};
    // MCU原始下发弹丸速度（未补偿）
    double bullet_speed_raw{0.0};
    // 经过弹道参数修正后的有效弹速（火控解算使用）
    double bullet_speed{0.0};

    // 四类固定静态坐标变换（fast_tf外参标定值）
    ControlTransformSnapshot odom_to_gimbal_pitch{};   // 里程计系 → 云台俯仰
    ControlTransformSnapshot gimbal_to_camera_link{};  // 云台 → 相机机械安装架
    ControlTransformSnapshot odom_to_camera_optical{};  // 里程计 → 相机光学成像坐标系
    ControlTransformSnapshot odom_to_muzzle{};          // 里程计 → 枪口发射坐标系
};

// 导入调度器资源组件命名空间，简化 res / res_mut 书写
using namespace talos::scheduler::system;

// ========== IMU状态 ECS资源别名 ==========
// res<T>：全局只读资源，多线程共享读取，不可修改
using imu_state     = res<ImuState>;
// res_mut<T>：全局可写资源，仅初始化/硬件采集系统拥有写入权限
using imu_state_mut = res_mut<ImuState>;

// ========== 识别目标颜色资源别名 ==========
using detecting_color     = res<ArmorColor>;
using detecting_color_mut = res_mut<ArmorColor>;

/**
 * @enum Capability
 * @brief 软件功能模块能力枚举，用于掩码开关控制各模块启用/关闭
 * 比赛场景可关闭无用模块降低CPU负载
 */
enum Capability {
    Armor  = 0,  // 装甲板识别跟踪模块
    Rune   = 1,  // 旋转能量机关Rune检测
    Ldm    = 2,  // 大符LDM能量机关光斑检测
    Quanta = 3,  // Foxglove高速视频流可视化推流
};

// 能力掩码底层存储类型：8位无符号整数，最多支持8个功能开关
using CapabilityMask = uint8_t;

/**
 * @brief 根据单个能力枚举生成对应掩码比特位
 * @param cap 功能枚举
 * @return 仅对应bit置1的掩码值
 * noexcept 无异常，实时系统安全
 * [[nodiscard]] 禁止丢弃返回值
 */
[[nodiscard]] constexpr auto capability_bit(const Capability cap) noexcept -> CapabilityMask {
    // 1U 左移对应位数，生成单bit掩码
    return static_cast<CapabilityMask>(1U << static_cast<unsigned>(cap));
}

/**
 * @brief 批量能力数组合并为单字节掩码
 * @param capabilities 批量启用的功能枚举只读视图
 * @return 按位或合并后的完整掩码
 */
[[nodiscard]] inline auto capability_mask(const std::span<const Capability> capabilities) noexcept
    -> CapabilityMask {
    CapabilityMask mask = 0;
    // 遍历所有功能，按位或叠加比特位
    for (const auto capability : capabilities) {
        mask = static_cast<CapabilityMask>(mask | capability_bit(capability));
    }
    return mask;
}

/**
 * @brief 全局功能能力状态管理类
 * 内部使用std::atomic实现多线程无锁读写，无需互斥锁
 * 不可拷贝、不可移动，全局单例资源
 */
struct CapabilityState {
    // 构造1：通过初始能力数组生成掩码
    explicit CapabilityState(const std::span<const Capability> initial) noexcept
        : mask_(capability_mask(initial)) {}

    // 构造2：直接传入预计算好的掩码
    explicit CapabilityState(const CapabilityMask initial) noexcept
        : mask_(initial) {}

    // 禁用拷贝构造、拷贝赋值、移动构造、移动赋值，全局唯一实例
    CapabilityState(const CapabilityState&)                    = delete;
    auto operator=(const CapabilityState&) -> CapabilityState& = delete;
    CapabilityState(CapabilityState&&)                         = delete;
    auto operator=(CapabilityState&&) -> CapabilityState&      = delete;

    /**
     * @brief 无锁读取当前能力掩码，acquire内存序保证数据可见性
     * @return 当前启用功能掩码
     */
    [[nodiscard]] auto load() const noexcept -> CapabilityMask {
        return mask_.load(std::memory_order_acquire);
    }

    /**
     * @brief 无锁写入新掩码，release内存序同步所有线程缓存
     * @param next 新功能掩码
     */
    void store(const CapabilityMask next) noexcept { mask_.store(next, std::memory_order_release); }

private:
    // 原子掩码存储，多线程安全读写
    std::atomic<CapabilityMask> mask_;
};

// ========== 能力开关全局ECS资源别名 ==========
using capabilities     = res<CapabilityState>;     // 只读访问
using capabilities_mut = res_mut<CapabilityState>; // 可写修改

/**
 * @brief 目标跟随模式状态（自瞄/能量机关优先）
 * 原子bool多线程无锁标记，true=优先跟踪能量机关，false=优先装甲自瞄
 */
struct FollowingState {
    // 读取当前跟随模式开关
    [[nodiscard]] auto load() const noexcept -> bool {
        return active_.load(std::memory_order_acquire);
    }

    // 修改跟随模式
    void store(const bool active) noexcept { active_.store(active, std::memory_order_release); }

private:
    std::atomic<bool> active_{false};
};

// ========== 跟随模式全局ECS资源别名 ==========
using following     = res<FollowingState>;
using following_mut = res_mut<FollowingState>;

// 全部能力枚举静态常量数组，遍历查询使用
inline constexpr std::array all_capabilities{
    Capability::Armor, Capability::Rune, Capability::Ldm, Capability::Quanta};

/**
 * @brief 从能力掩码提取当前所有启用的功能列表
 * @param state 全局能力状态原子实例
 * @return 动态数组，存放所有开启的Capability枚举
 */
[[nodiscard]] inline auto active_capabilities(const CapabilityState& state)
    -> std::vector<Capability> {
    std::vector<Capability> active;
    const auto mask = state.load();
    // 遍历全部功能，判断对应bit是否置1
    for (const auto capability : all_capabilities) {
        if ((mask & capability_bit(capability)) != 0U) {
            active.push_back(capability);
        }
    }
    return active;
}

/**
 * @brief 查询单个功能是否已启用
 * @param state 全局能力状态
 * @param cap 需要查询的功能
 * @return true=已开启；false=关闭
 */
[[nodiscard]] inline auto capable(const CapabilityState& state, const Capability cap) noexcept
    -> bool {
    // 掩码按位与，判断对应比特位
    return (state.load() & capability_bit(cap)) != 0U;
}

} // namespace fcs::core

/**
 * @namespace fmt
 * @brief fmt库自定义格式化特化，用于日志打印Capability枚举可读文本
 * 配合magic_enum将枚举转为字符串，打印日志时不再输出数字而是文字Armor/Rune等
 */
namespace fmt {
// 为 fcs::core::Capability 枚举实现fmt格式化器
template <>
struct formatter<fcs::core::Capability> : formatter<std::string_view> {
    auto format(const fcs::core::Capability c, format_context& ctx) const {
        // magic_enum自动将枚举转为字符串视图，复用string_view格式化逻辑
        return formatter<std::string_view>::format(magic_enum::enum_name(c), ctx);
    }
};
} // namespace fmt