#pragma once
// 固定长度数组容器
#include <array>
// 有序键值映射 std::map
#include <map>
// 标准可选值 std::optional
#include <optional>
// 动态数组 std::vector
#include <vector>

// TOML底层反序列化核心实现、基础类型解析工具
#include "toml/detail/core_deserialize.hpp"

/**
 * @brief 内部实现命名空间：解析逻辑底层实现，不对外暴露
 */
namespace toml_helper::detail {

/**
 * @brief 顶层递归解析入口函数声明
 * 根据T类型分支匹配对应解析逻辑，返回expected<T, 错误字符串>
 * @tparam T 目标C++类型（标量/枚举/结构体/容器/required/optional等）
 * @param node TOML语法树节点
 * @param context 上下文路径字符串，用于报错定位配置路径（如robot.camera[0].fx）
 * @param ensure_all_tables_read 开启时校验table内所有键均被读取，存在未读key报错
 * @return 成功：T；失败：携带人类可读错误信息
 */
template <typename T>
[[nodiscard]] std::expected<T, std::string>
    parse_value(const toml::node& node, std::string_view context, bool ensure_all_tables_read);

/**
 * @brief std::optional<T> 专用解析函数
 * 处理可选字段：节点不存在/空节点返回std::nullopt；存在则解析内部T并包装optional
 * @tparam T optional内部存储的基础类型
 * @param node TOML节点
 * @param context 报错上下文路径
 * @param ensure_all_tables_read 全table键校验开关
 * @return expected<optional<T>>，解析失败返回错误文本
 */
template <typename T>
[[nodiscard]] std::expected<std::optional<T>, std::string> parse_optional_value(
    const toml::node& node, std::string_view context, bool ensure_all_tables_read) {
    // 节点为空节点，直接返回无值optional
    if (node.type() == toml::node_type::none) {
        return std::optional<T>{};
    }

    // 先解析内部真实类型T
    auto result = parse_value<T>(node, context, ensure_all_tables_read);
    // 内部解析失败，透传错误信息
    if (!result) {
        return std::unexpected(result.error());
    }
    // 解析成功，包装为optional返回
    return std::optional<T>{std::move(*result)};
}

/**
 * @brief std::vector<T> 动态数组解析函数
 * 校验节点为TOML array，遍历每个元素递归parse_value<T>，组装vector
 * @tparam T 数组元素类型
 * @param node TOML节点
 * @param key 当前字段名，用于拼接数组下标上下文key[i]
 * @param ensure_all_tables_read 全table键校验开关
 * @return expected<vector<T>>，类型不匹配/元素解析失败返回错误
 */
template <typename T>
[[nodiscard]] std::expected<std::vector<T>, std::string>
    parse_vector_value(const toml::node& node, std::string_view key, bool ensure_all_tables_read) {
    // 尝试转为数组指针，非array节点返回类型错误
    const toml::array* arr = node.as_array();
    if (!arr) {
        return std::unexpected(invalid_value_error(key, "array", node.type()));
    }

    // 预分配容器容量，避免多次扩容
    std::vector<T> result;
    result.reserve(arr->size());

    // 遍历数组每一个下标元素
    for (size_t i = 0; i < arr->size(); ++i) {
        const toml::node& elem = (*arr)[i];
        // 拼接上下文：数组key + 下标，如camera_ids[0]
        auto elem_context      = fmt::format("{}[{}]", key, i);

        // 递归解析当前数组元素
        auto elem_result = parse_value<T>(elem, elem_context, ensure_all_tables_read);
        // 单个元素解析失败，整体返回错误
        if (!elem_result) {
            return std::unexpected(elem_result.error());
        }

        // 移动构造存入vector，避免拷贝
        result.push_back(std::move(*elem_result));
    }

    return result;
}

/**
 * @brief std::array<T,N> 固定长度数组解析函数
 * 校验节点为TOML array，且数组长度严格等于N，逐个解析元素填充定长数组
 * @tparam T 数组元素类型
 * @tparam N 编译期固定长度
 * @param node TOML节点
 * @param key 字段名，用于拼接下标上下文
 * @param ensure_all_tables_read 全table键校验开关
 * @return expected<array<T,N>>，长度不匹配/类型错误/元素解析失败返回错误
 */
template <typename T, std::size_t N>
[[nodiscard]] std::expected<std::array<T, N>, std::string>
    parse_array_value(const toml::node& node, std::string_view key, bool ensure_all_tables_read) {
    const toml::array* arr = node.as_array();
    if (!arr) {
        return std::unexpected(invalid_value_error(key, "array", node.type()));
    }

    // 严格校验数组长度必须等于模板N，否则报错
    if (arr->size() != N) {
        return std::unexpected(
            fmt::format(
                "Invalid size for array '{}': expected {} elements, got {}", key, N, arr->size()));
    }

    // 默认零初始化定长数组
    std::array<T, N> result{};

    // 遍历下标填充每一位
    for (size_t i = 0; i < N; ++i) {
        const toml::node& elem = (*arr)[i];
        auto elem_context      = fmt::format("{}[{}]", key, i);

        auto elem_result = parse_value<T>(elem, elem_context, ensure_all_tables_read);
        if (!elem_result) {
            return std::unexpected(elem_result.error());
        }

        result[i] = std::move(*elem_result);
    }

    return result;
}

/**
 * @brief std::map<std::string,V> 映射表解析函数
 * 校验节点为TOML table，遍历table子键值对，key转std::string，value递归解析V
 * @tparam V map值类型
 * @param node TOML节点
 * @param key 当前table字段名，用于拼接子键上下文key.subkey
 * @param ensure_all_tables_read 全table键校验开关
 * @return expected<map<string,V>>，非table节点/子元素解析失败返回错误
 */
template <typename V>
[[nodiscard]] std::expected<std::map<std::string, V>, std::string>
    parse_map_value(const toml::node& node, std::string_view key, bool ensure_all_tables_read) {
    const auto* tbl = node.as_table();
    if (!tbl) {
        return std::unexpected(invalid_value_error(key, "table", node.type()));
    }

    std::map<std::string, V> result;

    // 遍历table内所有子键值
    for (const auto& [k, v] : *tbl) {
        // 拼接层级上下文，如robot.camera.fx
        const auto entry_context = fmt::format("{}.{}", key, std::string_view{k});
        auto entry_result        = parse_value<V>(v, entry_context, ensure_all_tables_read);
        if (!entry_result) {
            return std::unexpected(entry_result.error());
        }
        // 插入有序map，key转std::string存储
        result.emplace(std::string{k}, std::move(*entry_result));
    }

    return result;
}

/**
 * @brief 核心递归类型分发解析实现
 * 编译期if constexpr分支匹配目标类型T，转发对应专用解析逻辑
 * 分支优先级：标量 → 枚举 → 自定义结构体TableReadable → required → optional → vector → array → map
 * @tparam T 待解析目标类型
 * @param node TOML语法树节点
 * @param context 报错路径上下文
 * @param ensure_all_tables_read 开启后校验table无未读取冗余key
 * @return expected<T, 错误文本>
 */
template <typename T>
[[nodiscard]] std::expected<T, std::string>
    parse_value(const toml::node& node, std::string_view context, bool ensure_all_tables_read) {
    // 分支1：基础标量类型 int/float/bool/string，直接解析标量节点
    if constexpr (TomlScalarValue<T>) {
        return parse_scalar_node<T>(node, context);
    }
    // 分支2：C++枚举类型，使用magic_enum反射解析枚举字面量
    else if constexpr (std::is_enum_v<T>) {
        return parse_enum_node<T>(node, context);
    }
    // 分支3：自定义结构体（实现TableReadable trait，可从table完整反序列化）
    else if constexpr (TableReadable<T>) {
        auto parsed = parse_object_from_node<T>(node, context);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        // 开启全表键校验：检查table内是否存在未读取的冗余key
        if (ensure_all_tables_read) {
            if (auto unread = error_if_unread(*parsed->parsed_table, context); !unread) {
                return std::unexpected(unread.error());
            }
        }
        // 移动构造返回解析完成的结构体
        return std::move(parsed->value);
    }
    // 分支4：toml_helper::required<T> 必填包装类型，内部递归解析T并包装required
    else if constexpr (is_required_v<T>) {
        auto parsed = parse_value<required_value_t<T>>(node, context, ensure_all_tables_read);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        return T{std::move(*parsed)};
    }
    // 分支5：std::optional<T> 可选包装类型，转发parse_optional_value专用逻辑
    else if constexpr (is_std_optional_v<T>) {
        auto parsed =
            parse_optional_value<optional_value_t<T>>(node, context, ensure_all_tables_read);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        return T{std::move(*parsed)};
    }
    // 分支6：std::vector<T> 动态数组，转发parse_vector_value
    else if constexpr (is_std_vector_v<T>) {
        return parse_vector_value<vector_value_t<T>>(node, context, ensure_all_tables_read);
    }
    // 分支7：std::array<T,N> 定长数组，转发parse_array_value并传入编译期长度
    else if constexpr (is_std_array_v<T>) {
        return parse_array_value<array_value_t<T>, array_value<T>::size>(
            node, context, ensure_all_tables_read);
    }
    // 分支8：std::map<std::string,V> 有序映射表，转发parse_map_value
    else if constexpr (is_std_map_v<T>) {
        return parse_map_value<map_value_t<T>>(node, context, ensure_all_tables_read);
    }
    // 无匹配类型，编译期静态断言报错，提示无反序列化特化实现
    else {
        static_assert(
            always_false_v<T>,
            "Cannot parse element type: no toml_helper::Deserialize specialization available");
        return std::unexpected("Unsupported element type");
    }
}

} // namespace toml_helper::detail

// ============================================================================
// 对外暴露的 Deserialize 顶层特化：容器专用
// 提供 read / read_optional / take / take_optional 四种读取模式
// read：键必须存在，缺失报错；read_optional：键可选，缺失返回optional无值
// take：读取后从table删除该键；take_optional：可选读取并删除键
// ============================================================================
namespace toml_helper {

/**
 * @brief std::optional<T> 容器反序列化顶层特化
 * 提供四种读取策略：强制读、可选读、读取后删除、可选读取后删除
 */
template <typename T>
struct Deserialize<std::optional<T>, void> {
    /**
     * @brief 强制读取optional字段，键不存在直接返回缺失key错误
     * @param table TOML根表
     * @param key 配置字段名
     * @return expected<optional<T>>，key缺失失败
     */
    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        read(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        return detail::parse_optional_value<T>(*node, key, true);
    }

    /**
     * @brief 可选读取optional字段，key不存在返回包含std::nullopt的外层optional
     * @param table TOML表
     * @param key 字段名
     * @return expected<optional<optional<T>>>
     * 外层optional代表key是否存在，内层optional代表字段有无值
     */
    [[nodiscard]] static std::expected<std::optional<std::optional<T>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::optional<T>>{};
        }

        auto result = detail::parse_optional_value<T>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }
        return std::optional<std::optional<T>>{std::move(*result)};
    }

    /**
     * @brief 读取字段后从table中erase删除该key，强制key必须存在
     * @param table TOML表（const_cast可变删除）
     * @param key 字段名
     * @return expected<optional<T>>
     */
    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        take(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        auto result = detail::parse_optional_value<T>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        // 移除table内当前key，上层无法再次读取
        (void)const_cast<toml::table&>(table).erase(key);
        return std::move(*result);
    }

    /**
     * @brief 可选读取并删除key，key不存在返回外层无值optional
     */
    [[nodiscard]] static std::expected<std::optional<std::optional<T>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::optional<T>>{};
        }

        auto result = detail::parse_optional_value<T>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::optional<std::optional<T>>{std::move(*result)};
    }
};

/**
 * @brief std::vector<T> 动态数组容器反序列化顶层特化
 */
template <typename T>
struct Deserialize<std::vector<T>, void> {
    [[nodiscard]] static std::expected<std::vector<T>, std::string>
        read(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        return detail::parse_vector_value<T>(*node, key, true);
    }

    [[nodiscard]] static std::expected<std::optional<std::vector<T>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::vector<T>>{};
        }

        auto result = detail::parse_vector_value<T>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }
        return std::optional<std::vector<T>>{std::move(*result)};
    }

    [[nodiscard]] static std::expected<std::vector<T>, std::string>
        take(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        auto result = detail::parse_vector_value<T>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::move(*result);
    }

    [[nodiscard]] static std::expected<std::optional<std::vector<T>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::vector<T>>{};
        }

        auto result = detail::parse_vector_value<T>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::optional<std::vector<T>>{std::move(*result)};
    }
};

/**
 * @brief std::array<T,N> 固定长度数组容器反序列化顶层特化
 */
template <typename T, std::size_t N>
struct Deserialize<std::array<T, N>, void> {
    [[nodiscard]] static std::expected<std::array<T, N>, std::string>
        read(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        return detail::parse_array_value<T, N>(*node, key, true);
    }

    [[nodiscard]] static std::expected<std::optional<std::array<T, N>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::array<T, N>>{};
        }

        auto result = detail::parse_array_value<T, N>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }
        return std::optional<std::array<T, N>>{std::move(*result)};
    }

    [[nodiscard]] static std::expected<std::array<T, N>, std::string>
        take(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        auto result = detail::parse_array_value<T, N>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::move(*result);
    }

    [[nodiscard]] static std::expected<std::optional<std::array<T, N>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::array<T, N>>{};
        }

        auto result = detail::parse_array_value<T, N>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::optional<std::array<T, N>>{std::move(*result)};
    }
};

/**
 * @brief std::map<std::string,V> 有序映射表容器反序列化顶层特化
 */
template <typename V>
struct Deserialize<std::map<std::string, V>, void> {
    [[nodiscard]] static std::expected<std::map<std::string, V>, std::string>
        read(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        return detail::parse_map_value<V>(*node, key, true);
    }

    [[nodiscard]] static std::expected<std::optional<std::map<std::string, V>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::map<std::string, V>>{};
        }

        auto result = detail::parse_map_value<V>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }
        return std::optional<std::map<std::string, V>>{std::move(*result)};
    }

    [[nodiscard]] static std::expected<std::map<std::string, V>, std::string>
        take(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::unexpected(detail::missing_key_error(key));
        }

        auto result = detail::parse_map_value<V>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::move(*result);
    }

    [[nodiscard]] static std::expected<std::optional<std::map<std::string, V>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<std::map<std::string, V>>{};
        }

        auto result = detail::parse_map_value<V>(*node, key, true);
        if (!result) {
            return std::unexpected(result.error());
        }

        (void)const_cast<toml::table&>(table).erase(key);
        return std::optional<std::map<std::string, V>>{std::move(*result)};
    }
};

} // namespace toml_helper