// 头文件保护，防止重复包含造成重定义编译报错
#pragma once

// 装甲相关枚举：ArmorName（装甲类型编号）、ArmorColor（目标红蓝/中立颜色）
#include "core/armor_types.hpp"

// 标准固定宽度整数：uint32_t
#include <cstdint>
// 标准size_t：哈希返回值类型
#include <cstddef>

/**
 * @namespace fcs::core
 * @brief 框架核心基础数据结构命名空间
 * TargetKey 是目标唯一标识键，用于哈希容器（unordered_map/unordered_set）区分不同敌方装甲目标
 */
namespace fcs::core {

/**
 * @brief 目标唯一标识键结构体
 * 二元组合：装甲编号 + 敌方颜色，二者唯一确定一个目标
 * 例：Red + One = 红方一号装甲；Blue + Outpost = 蓝方前哨站
 */
struct TargetKey {
    // 装甲/目标类型，默认无效
    ArmorName name{ArmorName::Invalid};
    // 目标阵营颜色，默认中立
    ArmorColor color{ArmorColor::Neutral};

    /**
     * @brief 相等比较运算符重载
     * @param other 对比另一个TargetKey
     * @return true 装甲编号、颜色完全一致则判定为同一个目标
     * @ noexcept 无异常，哈希容器实时查询安全
     * @ [[nodiscard]] 强制接收布尔结果，禁止丢弃比较返回值
     */
    [[nodiscard]] bool operator==(const TargetKey& other) const noexcept {
        return name == other.name && color == other.color;
    }
};

/**
 * @brief TargetKey 哈希函数仿函数
 * 适配 C++ std::unordered_map / std::unordered_set，提供哈希计算规则
 * 作用：把二元结构体(name+color)压缩为单个uint32_t，再生成哈希值
 */
struct TargetKeyHash {
    /**
     * @brief 哈希计算重载调用运算符
     * @param key 待哈希的目标标识键
     * @return std::size_t 哈希值，用于哈希桶索引
     * @ noexcept 无异常，容器高频查询无性能损耗
     */
    [[nodiscard]] auto operator()(const TargetKey& key) const noexcept -> std::size_t {
        // 打包规则：name占高24位，color占低8位
        // 1. name左移8位，留出低8位存储color
        // 2. 按位或拼接color，合并成单个32位无符号整数
        const auto packed =
            (static_cast<std::uint32_t>(key.name) << 8U) | static_cast<std::uint32_t>(key.color);
        // 调用标准uint32_t哈希生成最终哈希值
        return std::hash<std::uint32_t>{}(packed);
    }
};

} // namespace fcs::core