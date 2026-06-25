// 头文件保护，防止头文件被多次include重复定义
#pragma once

// Eigen基础矩阵运算头文件，Matrix/Vector/Array核心定义
#include <Eigen/Core>

// 存储解析后的一维数字数组临时容器
#include <vector>

// toml11 核心解析库头文件
#include "toml/core.hpp"

// 工具命名空间：所有TOML反序列化辅助逻辑统一放这里隔离污染
namespace toml_helper {

/**
 * @brief Eigen固定尺寸矩阵专用TOML反序列化模板特化
 * @tparam Mat Eigen矩阵类型（Matrix/Vector/Array）
 * @requires Mat必须是编译期固定行列：行数、列数都不能是Eigen::Dynamic
 *
 * 示例支持类型：
 * Eigen::Vector3d  R=3,C=1
 * Eigen::Matrix2f  R=2,C=2
 * Eigen::Array4i   R=4,C=1
 * 不支持：Eigen::MatrixXd(动态行列)、Eigen::VectorXf
 */
template <typename Mat>
requires(Mat::RowsAtCompileTime != Eigen::Dynamic && Mat::ColsAtCompileTime != Eigen::Dynamic)
struct Deserialize<Mat, void> {
private:
    /**
     * @brief 核心解析逻辑：把单个toml数组节点转为Eigen固定矩阵
     * @param node TOML节点（必须是array数组）
     * @param key 当前配置项键名，用于拼接错误日志
     * @return std::expected<Mat, std::string>
     *         成功：返回填充好数据的Eigen矩阵
     *         失败：返回unexpected，携带格式化错误字符串
     */
    static std::expected<Mat, std::string>
        parse_node(const toml::node& node, std::string_view key) {
        // 编译期计算矩阵总元素数量：行数 × 列数
        constexpr int expected_size = Mat::RowsAtCompileTime * Mat::ColsAtCompileTime;
        // 提取矩阵内部标量类型：double / float / int 等
        using Scalar                = typename Mat::Scalar;

        // 临时vector存放TOML数组读取出来的所有数字
        std::vector<Scalar> scalar_array;
        // 预分配内存，避免多次扩容，提升性能
        scalar_array.reserve(expected_size);

        // 尝试将当前toml节点转为数组指针
        const auto* arr = node.as_array();
        // 判断节点类型不是数组，直接返回错误
        if (!arr) {
            return std::unexpected(
                fmt::format(
                    "Invalid value for key '{}': expected array, got {}", key,
                    detail::node_type_name(node.type())));
        }

        // 遍历TOML数组内每一个元素
        for (const auto& value_node : *arr) {
            // 内部工具：安全将toml节点数值强制转换为Scalar类型（自动int→float等）
            // 返回std::optional<Scalar>，有值代表转换成功
            if (const auto scalar = detail::value_with_numeric_cast<Scalar>(value_node)) {
                scalar_array.push_back(*scalar);
                continue;
            }
            // 转换失败：数组内存在非数字元素，返回错误
            return std::unexpected(fmt::format("'{}' must be an array of numbers", key));
        }

        // 校验数组元素总数和矩阵所需元素数量完全匹配
        if (scalar_array.size() != static_cast<std::size_t>(expected_size)) {
            return std::unexpected(
                fmt::format("'{}' must have exactly '{}' elements", key, expected_size));
        }

        // Eigen::Map 零拷贝映射vector内存，直接构造Mat矩阵返回
        // Eigen::Map不拷贝数据，仅建立视图，vector生命周期在本函数内安全有效
        return Eigen::Map<const Mat>(scalar_array.data());
    }

public:
    /**
     * @brief 读取【必填】矩阵配置项
     * @param table TOML根表/子表
     * @param key 配置键名
     * @return expected<Mat, string>
     *         键缺失 / 类型错误 / 数组长度不匹配 均返回错误信息
     * @note [[nodiscard]] 强制接收返回值，防止忽略解析失败
     */
    [[nodiscard]] static std::expected<Mat, std::string>
        read(const toml::table& table, std::string_view key) {
        // 通用工具：读取必填节点，找不到key直接报错；找到后传入parse_node做矩阵解析
        return detail::read_required<Mat>(
            table, key, [&](const toml::node& node) { return parse_node(node, key); });
    }

    /**
     * @brief 读取【可选】矩阵配置项
     * @return expected<std::optional<Mat>, string>
     *         键不存在：返回std::nullopt（无矩阵）
     *         键存在但格式错误：返回错误字符串
     *         键存在且合法：返回std::optional包裹的矩阵
     */
    [[nodiscard]] static std::expected<std::optional<Mat>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        // 通用工具：处理可选键逻辑，不存在则返回空optional，存在则走parse_node解析
        return detail::read_optional_value<Mat>(
            table, key, [&](const toml::node& node) { return parse_node(node, key); });
    }

    /**
     * @brief 读取必填矩阵，解析成功后自动从table中删除该key
     * 适用场景：一次性消费配置，读完不再使用，清理table减少内存占用
     * @return 解析成功返回矩阵；键缺失/格式错误返回错误
     */
    [[nodiscard]] static std::expected<Mat, std::string>
        take(const toml::table& table, std::string_view key) {
        // 工具封装：先调用read，解析无错误则erase table内对应的key
        return detail::erase_on_success(read(table, key), table, key);
    }

    /**
     * @brief 读取可选矩阵，解析成功后自动删除table中的key
     * @return 键不存在：nullopt；存在且合法：optional<Mat>；格式错误：错误字符串
     */
    [[nodiscard]] static std::expected<std::optional<Mat>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        return detail::erase_on_success_optional(read_optional(table, key), table, key);
    }
};

} // namespace toml_helper