// 头文件保护指令，替代传统 #ifndef / #define / #endif
// 作用：保证该头文件在整个工程中只会被包含一次，防止重复定义、编译冲突
#pragma once

// 引入标准固定宽度整数头文件
// uint8_t：无符号8位整型，占用1字节，机器人枚举/状态常用，节省内存、跨平台统一
#include <cstdint>

// 只读字符串视图，不持有内存、轻量，比 std::string 开销更低
#include <string_view>

// 高性能格式化日志/输出库 fmt（框架统一日志、打印依赖）
#include <fmt/core.h>

// magic_enum 第三方库：C++ 枚举反射库
// 功能：不用手写字符串映射，自动把 enum class 枚举值转成对应名字符串
#include <magic_enum.hpp>

// 整个文件归属 fcs 命名空间：Fire Control System 火控系统
// 隔离模块代码，避免全局命名冲突，工程模块化规范
namespace fcs {

// ============================================================================
// 装甲板相关枚举定义区（机器人视觉识别核心分类）
// ============================================================================

/**
 * @brief 装甲板尺寸类型枚举
 * @note 底层存储为 uint8_t 单字节，极致省内存，适合实时机器人系统
 */
enum class ArmorType : uint8_t {
    Small   = 0,    // 小装甲板
    Large   = 1,    // 大装甲板
    Invalid = 2,    // 无效/识别失败
};

/**
 * @brief 装甲板颜色枚举（对战阵营/标识色）
 */
enum class ArmorColor : uint8_t {
    Red     = 0,    // 红色装甲
    Blue    = 1,    // 蓝色装甲
    Neutral = 2,    // 中立色
    Purple  = 3,    // 紫色装甲
};

/**
 * @brief 装甲板具体名称/位置枚举
 * 对应机器人不同位置、不同单位的装甲编号
 */
enum class ArmorName : uint8_t {
    Sentry = 0,     // 哨兵机器人装甲
    One,            // 1号英雄机器人
    Two,            // 2号机器人
    Three,          // 3号机器人
    Four,           // 4号机器人
    Five,           // 5号机器人
    Outpost,        // 前哨站装甲
    Base,           // 基地小装甲
    BaseLarge,      // 基地大装甲
    Invalid         // 无效装甲
};

// ============================================================================
// 神经网络输出 -> 装甲类型 映射逻辑（历史兼容/后处理逻辑）
// 注释：AT Legacy cls/color Mapping (from NN output)
// 含义：对接深度学习模型(NN)推理结果，做类型转换
// ============================================================================

/**
 * @brief 根据装甲名称，推导对应的装甲尺寸类型
 * @param armor_name 识别到的装甲名称枚举
 * @return 最终装甲尺寸类型（大/小/无效）
 *
 * @attribute [[nodiscard]] 编译器属性
 * 作用：禁止调用者忽略函数返回值，防止漏处理转换结果，规避隐性BUG
 * @attribute noexcept 编译器属性
 * 作用：声明函数绝对不会抛出异常，符合框架「内部不抛异常」规范，提升性能、保证实时性
 */
[[nodiscard]] constexpr ArmorType cls_to_armor_type(ArmorName armor_name) noexcept {
    // 业务规则：英雄机器人(One)、基地大装甲(BaseLarge) 归类为【大装甲】
    // 其余所有装甲统一归类为【小装甲】
    return (armor_name == ArmorName::One || armor_name == ArmorName::BaseLarge) 
        ? ArmorType::Large 
        : ArmorType::Small;
}

} // namespace fcs 结束 fcs 火控系统命名空间

// ============================================================================
// fmt 库格式化特化：让枚举直接支持 fmt 打印输出
// ============================================================================
// 进入 fmt 库命名空间，对上面三个枚举做 formatter 特化
// 目的：可以直接使用 fmt::print("{}", 枚举变量) 打印出枚举字面名称，而非数字 0/1/2

namespace fmt {

/**
 * @brief 特化 ArmorType 枚举的 fmt 格式化器
 * 继承自 std::string_view 的格式化器，复用已有逻辑
 */
template <>
struct formatter<fcs::ArmorType> : formatter<std::string_view> {
    /**
     * @brief 格式化函数
     * @param c 待格式化的装甲类型枚举值
     * @param ctx fmt 格式化上下文
     * @return 格式化迭代器
     *
     * 逻辑：
     * magic_enum::enum_name(c) 自动将枚举值转为对应的名字符串（如 Small → "Small"）
     * 再调用父类 formatter 完成输出
     */
    auto format(const fcs::ArmorType c, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(c), ctx);
    }
};

/**
 * @brief 特化 ArmorColor 枚举的 fmt 格式化器
 * 功能同上：支持直接打印颜色枚举名称
 */
template <>
struct formatter<fcs::ArmorColor> : formatter<std::string_view> {
    auto format(const fcs::ArmorColor c, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(c), ctx);
    }
};

/**
 * @brief 特化 ArmorName 枚举的 fmt 格式化器
 * 功能同上：支持直接打印装甲位置/名称枚举
 */
template <>
struct formatter<fcs::ArmorName> : formatter<std::string_view> {
    auto format(const fcs::ArmorName c, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(c), ctx);
    }
};

} // namespace fmt 结束 fmt 命名空间