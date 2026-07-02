// 头文件保护宏，防止该头文件被多次重复包含（替代传统 #ifndef / #define / #endif）
#pragma once

// 引入跨进程共享内存IPC通信底层端点实现
#include "chiral/chiral_endpoint.hpp"
// 标准容器：定长栈数组，用于装甲高度缓存
#include <array>
// C++20 类型约束concept，用于模板编译期类型校验
#include <concepts>
// 固定宽度整数类型：uint8_t / uint64_t 等无符号整数
#include <cstdint>

// 命名空间分层：talos框架 -> chiral共享内存IPC模块 -> 导航视觉解算子模块
namespace talos::chiral::navigation {

/**
 * @brief 编译期空Tag约束concept，仅用作模板类型标记，0运行时开销
 * @tparam T 任意完整类型
 * @note requires { sizeof(T) == 0; } 仅语法校验，不判断尺寸是否真为0
 *       只要T是完整类型，sizeof(T)语法合法，该concept就判定为true
 *       作用：给模板 Vector3d/Quateriond/Transform 绑定坐标系标签（odom/gimbal_yaw等）
 */
template <typename T>
/*concept（C++20）：模板类型约束工具，用来限制模板能传入什么类型，检查类型语法 / 接口是否达标；
constexpr（C++11）：编译期求值标记，标记变量 / 函数可以在编译阶段算出结果，消除运行时计算开销。*/
concept tag = requires { sizeof(T) == 0; };

// 类型别名：语义化区分浮点单位，底层都是double，避免单位传参混淆
using Radian       = double;        // 弧度（云台yaw/pitch角度单位）
using RadianPerSec = double;        // 弧度每秒（云台角速度单位）
using Meter        = double;        // 米（3D空间坐标、装甲半径高度单位）

// ========== 坐标系编译期Tag标签结构体（无成员，纯编译期标记，无运行时内存）==========
// 里程计世界坐标系
struct odom {};
// 云台yaw轴坐标系（云台水平旋转轴）
struct gimbal_yaw {};
// 云台pitch轴坐标系（云台俯仰旋转轴）
struct gimbal_pitch {};
// 相机光学坐标系
struct camera {};
// 枪口发射坐标系
struct muzzle {};
// 无坐标系，默认占位参数
struct untyped {};
// 编译期Tag定义结束

/**
 * @brief 带时间戳的数据包裹模板，给任意业务数据附加系统纳秒时间戳
 * @tparam T 被包裹的业务消息类型
 */
template <typename T>
struct timestamped {
    uint64_t timestamp_ns_system_clock; // 系统时钟纳秒级时间戳
    T data;                             // 实际业务载荷数据
};

/**
 * @brief 三维向量模板，带坐标系编译期Tag，用于空间平移/速度
 * @tparam From 源坐标系tag（必须满足tag概念）
 * @tparam To   目标坐标系tag，默认untyped无坐标系
 * @note 模板参数仅编译期校验标记，不占用运行时内存，规避不同坐标系向量混用bug
 *       固定内存布局x/y/z，保证跨进程共享内存ABI稳定
 */
 /*
 1. tag From
普通模板参数正常写法是 typename From，这里把 typename 替换成 concept 名字：
    tag：约束这个模板参数 From 必须满足 tag 概念；
    等价完整写法：template <typename From, typename To = untyped> requires tag<From> && tag<To>
    编译器强制校验：你传给 From 的类型，必须是 odom/gimbal_yaw 这类空 Tag 结构体，否则编译直接报错，杜绝乱传类型。

2. tag To
    第二个模板参数 To 同样受 tag 概念约束，只能传坐标系 Tag 类型。

3. = untyped
    模板参数默认实参：如果使用模板时只写第一个参数，第二个参数自动填充 untyped。*/
template <tag From, tag To = untyped>
struct Vector3d {
    double x, y, z;
};

/**
 * @brief 四元数模板，存储旋转姿态，带坐标系Tag
 * @tparam From 源坐标系tag
 * @tparam To   目标坐标系tag
 * 四元数标准顺序 x y z w
 */
template <tag From, tag To = untyped>
struct Quateriond {
    double x, y, z, w;
};

/**
 * @brief 坐标变换结构体：平移向量 + 旋转四元数，代表两个坐标系间位姿转换关系
 * @tparam From 源坐标系
 * @tparam To   目标坐标系
 */
template <tag From, tag To = untyped>
struct Transform {
    Vector3d<From, To> translation; // 平移分量
    Quateriond<From, To> rotation;  // 旋转分量
};

// 目标大类枚举：区分敌方机器人/前哨站两类目标
enum TargetStateKind : uint8_t {
    Robot = 0,    // 敌方英雄/步兵机器人
    Outpost = 1   // 前哨站/基地
};

// 跟踪器工作状态枚举
enum class TrackerStatus : uint8_t {
    Idle      = 0, // 空闲，无目标
    Detecting = 1, // 正在检测装甲灯条
    Tracking  = 2, // 稳定跟踪目标
    TempLost  = 3  // 临时丢失目标，短时间维持预测
};

// 装甲颜色枚举
enum class ArmorColor : uint8_t {
    Blue    = 0,
    Red     = 1,
    Neutral = 2,
    Purple  = 3
};

// 装甲编号枚举，区分不同位置装甲板
enum class ArmorName : uint8_t {
    Sentry = 0, // 哨兵机器人装甲
    One,        // 1号装甲
    Two,        // 2号装甲
    Three,      // 3号装甲
    Four,       // 4号装甲
    Five,       // 5号装甲
    Outpost,    // 前哨站装甲
    Base,       // 基地小装甲
    BaseLarge,  // 基地大装甲
    Invalid,    // 无效装甲
    MaxNum      // 枚举总数，用于数组长度定义
};

/**
 * @brief 前哨站目标状态结构体
 * 存储前哨站在gimbal_yaw坐标系下的位姿、速度、高度参数
 */
struct OutpostState {
    Vector3d<gimbal_yaw> position;  // 3D世界坐标
    Vector3d<gimbal_yaw> velocity;  // 三维速度
    Radian yaw;                     // 自身航向角
    RadianPerSec v_yaw;             // 航向角速度
    std::array<double, 3> z{0, 0, 0}; // z轴高度缓存 z0/z1/z2
};

/**
 * @brief 敌方机器人目标状态结构体
 * 存储敌方机器人底盘位姿、装甲尺寸、装甲数量
 */
struct RobotState {
    Vector3d<gimbal_yaw> position;  // 底盘中心三维坐标
    Vector3d<gimbal_yaw> velocity;  // 底盘三维速度
    Radian yaw;                     // 底盘航向角
    RadianPerSec v_yaw;             // 底盘旋转角速度
    Meter radius0; // 偶数号装甲(0,2)半径
    Meter radius1; // 奇数号装甲(1,3)半径
    Meter z1;      // 奇数装甲高度偏移 z0 + 装甲高度
    uint32_t armor_num; // 当前可见装甲数量
};

/**
 * @brief 完整目标状态包
 * 统一存储机器人/前哨站跟踪信息，由视觉PnP解算填充
 */
struct TargetState {
    TrackerStatus status; // 跟踪器状态
    ArmorColor color;     // 敌方装甲颜色
    ArmorName name;       // 当前主跟踪装甲编号
    RobotState robot;     // 机器人目标数据
    OutpostState outpost; // 前哨站目标数据
};

/**
 * @brief Talos视觉导航输出数据总包
 * 视觉解算模块输出，通过共享内存发给底盘/自瞄控制模块
 */
struct TalosData {
    TargetStateKind state_kind; // 当前目标类型：机器人/前哨
    TargetState state;          // 完整目标跟踪数据

    Transform<odom, gimbal_yaw> gimbal_link; // odom里程计 -> 云台yaw坐标系转换
    Transform<gimbal_yaw, muzzle> muzzle_link; // 云台 -> 枪口坐标系转换
    Transform<gimbal_yaw, camera> camera_link; // 云台 -> 相机坐标系转换
};

/**
 * @brief 导航控制下发数据
 * 底盘/裁判系统下发给视觉模块的控制参数
 */
struct NavigationData {
    int64_t timestamp_ns = 0;               // 消息时间戳纳秒
    uint64_t duration_ns = 1000000000ULL;   // 目标无敌时长默认1s
    // 各装甲无敌标记数组，MaxNum固定数组长度
    bool invincible[static_cast<size_t>(ArmorName::MaxNum)];

    /**
     * @brief 设置指定装甲无敌状态
     * @param name 装甲编号
     * @param invincible_ true=无敌不打击 false=正常可打击
     * noexcept 无异常抛出，实时系统保证稳定
     */
    void set_invincible(ArmorName name, bool invincible_) noexcept {
        invincible[static_cast<size_t>(name)] = invincible_;
    }

    /**
     * @brief 查询装甲是否无敌
     */
    bool is_invincible(ArmorName name) const noexcept {
        return invincible[static_cast<size_t>(name)];
    }

    /**
     * @brief 批量设置多个装甲无敌状态
     * @param names 装甲列表初始化列表
     * @param invincible_ 统一开关
     */
    void emplace_invincible(
        std::initializer_list<ArmorName> names, bool invincible_ = true) noexcept {
        for (auto name : names) {
            set_invincible(name, invincible_);
        }
    }

    /**
     * @brief 静态工厂函数，快速创建带当前系统时间的导航数据
     * @param duration_ns_ 无敌持续时间，默认1秒
     * @return 初始化完成的NavigationData对象
     */
    static NavigationData create(uint64_t duration_ns_ = 1000000000ULL) noexcept {
        // 获取当前系统时钟，转换为纳秒时间戳
        const auto now = std::chrono::system_clock::now();
        auto t =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        // 聚合初始化返回实例，invincible数组零初始化
        return {.timestamp_ns = t, .duration_ns = duration_ns_, .invincible = {}};
    }
};

// IPC共享内存通信端点别名
// TalosEndpoint：视觉模块发送TalosData，接收NavigationData
using TalosEndpoint      = ipc::ChiralEndpoint<TalosData, NavigationData>;
// NavigationEndpoint：底盘模块发送NavigationData，接收TalosData
using NavigationEndpoint = ipc::ChiralEndpoint<NavigationData, TalosData>;

} // namespace talos::chiral::navigation

// 进入IPC命名空间，特化共享内存文件名模板
namespace talos::chiral::ipc {

/**
 * @brief 模板特化：TalosData 对应的共享内存文件名
 * ShmName<T> 全局统一规范，不同消息类型绑定不同/dev/shm文件
 */
template <>
struct ShmName<navigation::TalosData> {
    static constexpr const char* value = "/chiral_nav_talos";
};

/**
 * @brief NavigationData 共享内存文件名
 */
template <>
struct ShmName<navigation::NavigationData> {
    static constexpr const char* value = "/chiral_nav_navigation";
};
} // namespace talos::chiral::ipc