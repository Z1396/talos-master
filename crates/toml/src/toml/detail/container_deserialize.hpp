#pragma once // 头文件保护：防止本头文件被重复包含
/*
 ============================================================================
 容器反序列化实现文件
 ============================================================================
 全局调用链中的位置：
   from_table<T> → 反射循环 → Deserialize<裸类型>::take → 本文件 parse_value
 本文件职责：
   1. parse_value        节点级类型分发器（if constexpr 编译期选分支）
   2. parse_xxx_value    四种容器的逐元素递归解析
   3. Deserialize<容器>  表级入口四件套（查key → 解析 → erase销账）
 双入口设计原因：
   Deserialize 输入 (table,key) —— 负责"定位节点+销账"；
   parse_value   输入 node      —— 负责"解析节点本身"；
   容器元素只有下标没有独立key，递归必须走节点级入口。
 ============================================================================
*/

#include <array>                            // std::array：定长数组容器支持
#include <map>                              // std::map：有序键值映射容器支持
#include <optional>                         // std::optional：可选值容器支持
#include <vector>                           // std::vector：动态数组容器支持

#include "toml/detail/core_deserialize.hpp" // 核心：标量/枚举解析函数、类型萃取器、always_false_v 陷阱

namespace toml_helper::detail {             // 内部实现命名空间：细节不对外暴露

/**
 * @brief parse_value 前置声明
 * 先声明后定义：下方四个容器解析函数会递归调用它，
 * C++ 要求"使用前可见"，故声明提前占位
 * @tparam T 目标类型（标量/枚举/结构体/容器/包装器均可）
 * @param node 待解析的 TOML 节点
 * @param context 报错路径（如 preset.host[0]），逐层拼接
 * @param ensure_all_tables_read true=解析后校验表内无未读冗余键
 * @return 成功返回 T，失败返回错误字符串
 */
template <typename T>
[[nodiscard]] std::expected<T, std::string> // nodiscard：返回值必须被使用，防止漏处理错误
    parse_value(const toml::node& node, std::string_view context, bool ensure_all_tables_read);

/**
 * @brief std::optional<T> 节点级解析
 * 处理"节点已找到"的情况：空节点→nullopt，否则解析内部T再包装
 * （key 本身不存在由上层表级 get 判空处理，两层各管一种"没有"）
 * @tparam T optional 内部的值类型
 */
template <typename T>
[[nodiscard]] std::expected<std::optional<T>, std::string> parse_optional_value(
    const toml::node& node, std::string_view context, bool ensure_all_tables_read) {
    // 空节点（TOML 里的空值）→ 返回无值 optional，属于合法情况不报错
    if (node.type() == toml::node_type::none) {
        return std::optional<T>{}; // 内层 nullopt：字段存在但无值
    }

    // 递归解析内部真实类型 T（int→标量分支 / struct→反射 / vector→容器...）
    auto result = parse_value<T>(node, context, ensure_all_tables_read);
    if (!result) {                               // 内部解析失败
        return std::unexpected(result.error());  // 透传错误（context 已带完整路径）
    }
    return std::optional<T>{std::move(*result)}; // 成功：move 包进 optional，避免拷贝
}

/**
 * @brief std::vector<T> 节点级解析
 * 对应 TOML 数组语法 key = [e1, e2, ...]，长度不限
 * @tparam T 元素类型
 */
template <typename T>
[[nodiscard]] std::expected<std::vector<T>, std::string>
    parse_vector_value(const toml::node& node, std::string_view key, bool ensure_all_tables_read) {
    // 向下转型：node 基类指针 → array 派生指针，失败返回 nullptr
    const toml::array* arr = node.as_array();
    if (!arr) { // 节点不是数组（如写成了字符串）
        return std::unexpected(invalid_value_error(key, "array", node.type())); // 报期望/实际类型
    }

    std::vector<T> result;                                                      // 结果容器
    result.reserve(arr->size());                           // 一次性预分配容量，避免逐个 push 扩容

    for (size_t i = 0; i < arr->size(); ++i) {             // 按下标遍历每个元素
        const toml::node& elem = (*arr)[i];                // 取当前元素节点
        auto elem_context = fmt::format("{}[{}]", key, i); // 拼下标路径 key[i]，报错可定位到第几个

        auto elem_result =
            parse_value<T>(elem, elem_context, ensure_all_tables_read); // 递归解析元素
        if (!elem_result) {                                             // 任一元素失败
            return std::unexpected(elem_result.error()); // fail-fast：立即中止，不部分填充
        }

        result.push_back(std::move(*elem_result));       // move 存入，避免元素拷贝
    }

    return result;                                       // 返回组装完成的 vector
}

/**
 * @brief std::array<T,N> 节点级解析
 * 与 vector 版的唯一区别：TOML 数组长度必须精确等于编译期 N
 * @tparam T 元素类型
 * @tparam N 编译期固定长度
 */
template <typename T, std::size_t N>
[[nodiscard]] std::expected<std::array<T, N>, std::string>
    parse_array_value(const toml::node& node, std::string_view key, bool ensure_all_tables_read) {
    const toml::array* arr = node.as_array(); // 确认节点是数组
    if (!arr) {                               // 不是数组 → 类型错误
        return std::unexpected(invalid_value_error(key, "array", node.type()));
    }

    // 定长校验：C++ 侧 std::array 编译期定长 → TOML 数据也必须恰好 N 个
    if (arr->size() != N) {
        return std::unexpected(             // 多写少写都算配置错误
            fmt::format(
                "Invalid size for array '{}': expected {} elements, got {}", key, N, arr->size()));
    }

    std::array<T, N> result{};              // 零初始化：解析失败也不会读到脏数据

    for (size_t i = 0; i < N; ++i) {        // 循环上界用 N（已与 size 校验相等）
        const toml::node& elem = (*arr)[i]; // 取第 i 个元素节点
        auto elem_context      = fmt::format("{}[{}]", key, i); // 下标路径 key[i]

        auto elem_result = parse_value<T>(elem, elem_context, ensure_all_tables_read); // 递归解析
        if (!elem_result) {                                                            // 元素失败
            return std::unexpected(elem_result.error());                               // 整体失败
        }

        result[i] = std::move(*elem_result); // 按下标赋值（定长无 push_back）
    }

    return result;                           // 返回填满的定长数组
}

/**
 * @brief std::map<std::string,V> 节点级解析
 * 对应 TOML 子表 [key.sub] 或内联表 {a=1}；
 * 唯一"C++ 键来自 TOML 键名"的容器（vector/array 下标是位置不是键）
 * @tparam V 值类型
 */
template <typename V>
[[nodiscard]] std::expected<std::map<std::string, V>, std::string>
    parse_map_value(const toml::node& node, std::string_view key, bool ensure_all_tables_read) {
    const auto* tbl = node.as_table(); // 确认节点是 table
    if (!tbl) {                        // 不是 table → 类型错误
        return std::unexpected(invalid_value_error(key, "table", node.type()));
    }

    std::map<std::string, V> result;   // 结果映射表

    for (const auto& [k, v] : *tbl) {  // 结构化绑定遍历：k=键名节点，v=值节点
        // 拼点号路径 key.sub，与 TOML 点号语法一致，报错可定位到具体子键
        const auto entry_context = fmt::format("{}.{}", key, std::string_view{k});
        auto entry_result = parse_value<V>(v, entry_context, ensure_all_tables_read); // 递归解析值
        if (!entry_result) {                                                          // 子值失败
            return std::unexpected(entry_result.error());                             // 整体失败
        }
        // emplace 原地插入：TOML 键(string_view)转拥有所有权的 std::string
        result.emplace(std::string{k}, std::move(*entry_result));
    }

    return result; // 返回组装完成的 map
}

/**
 * @brief 核心递归类型分发器（节点级，全库唯一定义）
 * if constexpr 编译期选分支：每个 T 实例化时仅生成命中的一个分支，零运行时开销
 * 优先级：标量 → 枚举 → 结构体 → required → optional → vector → array → map
 * 兜底：always_false_v 陷阱，分发落空的类型编译期报错
 */
template <typename T>
[[nodiscard]] std::expected<T, std::string>
    parse_value(const toml::node& node, std::string_view context, bool ensure_all_tables_read) {
    if constexpr (TomlScalarValue<T>) {             // 分支1：标量（int/float/bool/string/date...）
        return parse_scalar_node<T>(node, context); // 递归的底：直接读值，不再下钻
    } else if constexpr (std::is_enum_v<T>) {       // 分支2：枚举
        return parse_enum_node<T>(node, context);   // magic_enum 反射：支持字符串/数字写法
    } else if constexpr (TableReadable<T>) {        // 分支3：可解析结构体（ReadFrom 或反射聚合体）
        auto parsed = parse_object_from_node<T>(node, context); // 解析出值+来源表（供校验用）
        if (!parsed) {                                          // 结构体解析失败
            return std::unexpected(parsed.error());             // 透传错误
        }
        if (ensure_all_tables_read) {                           // 开启冗余键校验时
            // 检查该子表是否有已写入但没被任何字段读取的键
            if (auto unread = error_if_unread(*parsed->parsed_table, context); !unread) {
                return std::unexpected(unread.error()); // 有冗余键 → 报错
            }
        }
        return std::move(parsed->value);                // 返回解析完成的结构体
    } else if constexpr (is_required_v<T>) {            // 分支4：required<T> 必填包装
        // 拆壳：萃取内部裸类型 T，递归解析后重新包装（必填检查在表级完成）
        auto parsed = parse_value<required_value_t<T>>(node, context, ensure_all_tables_read);
        if (!parsed) {                              // 内部解析失败
            return std::unexpected(parsed.error()); // 透传
        }
        return T{std::move(*parsed)};               // 走 required 转换构造，装回包装器
    } else if constexpr (is_std_optional_v<T>) {    // 分支5：std::optional<T> 可选包装
        // 拆壳递归：空节点→nullopt 的处理在 parse_optional_value 内部
        auto parsed =
            parse_optional_value<optional_value_t<T>>(node, context, ensure_all_tables_read);
        if (!parsed) {                         // 失败透传
            return std::unexpected(parsed.error());
        }
        return T{std::move(*parsed)};          // 重新包装成 optional
    } else if constexpr (is_std_vector_v<T>) { // 分支6：动态数组
        // vector_value_t<T> 萃取元素类型，逐元素递归在 parse_vector_value 内
        return parse_vector_value<vector_value_t<T>>(node, context, ensure_all_tables_read);
    } else if constexpr (is_std_array_v<T>) { // 分支7：定长数组
        // 同时传元素类型和编译期长度 N
        return parse_array_value<array_value_t<T>, array_value<T>::size>(
            node, context, ensure_all_tables_read);
    } else if constexpr (is_std_map_v<T>) { // 分支8：有序映射
        // map_value_t<T> 萃取值类型
        return parse_map_value<map_value_t<T>>(node, context, ensure_all_tables_read);
    } else {                                                // 兜底：没有任何分支接得住
        static_assert( // always_false_v 依赖 T → 定义期不求值，实例化到此必编译报错
            always_false_v<T>,
            "Cannot parse element type: no toml_helper::Deserialize specialization available");
        return std::unexpected("Unsupported element type"); // 永不执行：仅满足签名完整性
    }
}

} // namespace toml_helper::detail

// ============================================================================
// Deserialize 容器顶层特化（表级入口）
// 四件套语义：read=必填 / read_optional=可缺省 / take=读+销账 / take_optional=可选+销账
// erase 销账的必要性：ensure_all_read 以"表内剩余键"判冗余，已读不删会被误报
// 三步骨架：get(key) → parse_xxx_value → (take系) const_cast+erase
// ============================================================================
namespace toml_helper {

/**
 * @brief std::optional<T> 顶层特化
 * 双层 optional：外层=key是否存在，内层=值是否有（空节点）
 */
template <typename T>
struct Deserialize<std::optional<T>, void> {
    /**
     * @brief 必填读：key 缺失报错
     */
    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        read(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                    // 查节点
        if (!node) {                                                // key 没写
            return std::unexpected(detail::missing_key_error(key)); // 必读语义：报错
        }
        return detail::parse_optional_value<T>(*node, key, true);   // 节点级解析（空值→nullopt）
    }

    /**
     * @brief 可选读：key 缺失返回空外层
     */
    [[nodiscard]] static std::expected<std::optional<std::optional<T>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);      // 查节点
        if (!node) {                                  // key 没写
            return std::optional<std::optional<T>>{}; // 合法：外层置空（区别于 read 报错）
        }
        auto result = detail::parse_optional_value<T>(*node, key, true); // 解析内层
        if (!result) {                                                   // 解析失败
            return std::unexpected(result.error());                      // 透传
        }
        return std::optional<std::optional<T>>{std::move(*result)};      // 外层装内层结果
    }

    /**
     * @brief 必填读+销账：反射解析必填字段的标准路径
     */
    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        take(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                         // 查节点
        if (!node) {                                                     // 缺失
            return std::unexpected(detail::missing_key_error(key));      // 报错
        }
        auto result = detail::parse_optional_value<T>(*node, key, true); // 解析
        if (!result) {                                                   // 失败
            return std::unexpected(result.error());                      // 透传
        }
        // 销账：const_cast 仅用于记账性删除，不修改其他数据
        (void)const_cast<toml::table&>(table).erase(key);
        return std::move(*result); // 返回解析值
    }

    /**
     * @brief 可选读+销账：反射解析可选字段的标准路径
     */
    [[nodiscard]] static std::expected<std::optional<std::optional<T>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);      // 查节点
        if (!node) {                                  // 缺失
            return std::optional<std::optional<T>>{}; // 合法（表里本无key，无需erase）
        }
        auto result = detail::parse_optional_value<T>(*node, key, true); // 解析
        if (!result) {                                                   // 失败
            return std::unexpected(result.error());                      // 透传
        }
        (void)const_cast<toml::table&>(table).erase(key);                // 有值才销账
        return std::optional<std::optional<T>>{std::move(*result)};      // 返回
    }
};

/**
 * @brief std::vector<T> 顶层特化
 * 对应 TOML 数组，长度自由；四件套骨架同上，解析换 parse_vector_value
 */
template <typename T>
struct Deserialize<std::vector<T>, void> {
    /**
     * @brief 必填读：key 缺失报错
     */
    [[nodiscard]] static std::expected<std::vector<T>, std::string>
        read(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                    // 查节点
        if (!node) {                                                // 缺失
            return std::unexpected(detail::missing_key_error(key)); // 报错
        }
        return detail::parse_vector_value<T>(*node, key, true);     // 节点级逐元素解析
    }

    /**
     * @brief 可选读：key 缺失返回空
     */
    [[nodiscard]] static std::expected<std::optional<std::vector<T>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                       // 查节点
        if (!node) {                                                   // 缺失
            return std::optional<std::vector<T>>{};                    // 合法置空
        }
        auto result = detail::parse_vector_value<T>(*node, key, true); // 解析
        if (!result) {                                                 // 失败
            return std::unexpected(result.error());                    // 透传
        }
        return std::optional<std::vector<T>>{std::move(*result)};      // 包装返回
    }

    /**
     * @brief 必填读+销账
     */
    [[nodiscard]] static std::expected<std::vector<T>, std::string>
        take(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                       // 查节点
        if (!node) {                                                   // 缺失
            return std::unexpected(detail::missing_key_error(key));    // 报错
        }
        auto result = detail::parse_vector_value<T>(*node, key, true); // 解析
        if (!result) {                                                 // 失败
            return std::unexpected(result.error());                    // 透传
        }
        (void)const_cast<toml::table&>(table).erase(key);              // 销账
        return std::move(*result);                                     // 返回
    }

    /**
     * @brief 可选读+销账
     */
    [[nodiscard]] static std::expected<std::optional<std::vector<T>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                       // 查节点
        if (!node) {                                                   // 缺失
            return std::optional<std::vector<T>>{};                    // 合法（无key可销）
        }
        auto result = detail::parse_vector_value<T>(*node, key, true); // 解析
        if (!result) {                                                 // 失败
            return std::unexpected(result.error());                    // 透传
        }
        (void)const_cast<toml::table&>(table).erase(key);              // 销账
        return std::optional<std::vector<T>>{std::move(*result)};      // 返回
    }
};

/**
 * @brief std::array<T,N> 顶层特化
 * 对应 TOML 数组，运行期强制长度==N（校验在 parse_array_value 内）
 */
template <typename T, std::size_t N>
struct Deserialize<std::array<T, N>, void> {
    /**
     * @brief 必填读：key 缺失报错，长度不匹配在解析层报错
     */
    [[nodiscard]] static std::expected<std::array<T, N>, std::string>
        read(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                    // 查节点
        if (!node) {                                                // 缺失
            return std::unexpected(detail::missing_key_error(key)); // 报错
        }
        return detail::parse_array_value<T, N>(*node, key, true);   // 解析（N 传入做长度校验）
    }

    /**
     * @brief 可选读：key 缺失返回空
     */
    [[nodiscard]] static std::expected<std::optional<std::array<T, N>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                         // 查节点
        if (!node) {                                                     // 缺失
            return std::optional<std::array<T, N>>{};                    // 合法置空
        }
        auto result = detail::parse_array_value<T, N>(*node, key, true); // 解析
        if (!result) {                                                   // 失败
            return std::unexpected(result.error());                      // 透传
        }
        return std::optional<std::array<T, N>>{std::move(*result)};      // 包装返回
    }

    /**
     * @brief 必填读+销账
     */
    [[nodiscard]] static std::expected<std::array<T, N>, std::string>
        take(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                         // 查节点
        if (!node) {                                                     // 缺失
            return std::unexpected(detail::missing_key_error(key));      // 报错
        }
        auto result = detail::parse_array_value<T, N>(*node, key, true); // 解析
        if (!result) {                                                   // 失败
            return std::unexpected(result.error());                      // 透传
        }
        (void)const_cast<toml::table&>(table).erase(key);                // 销账
        return std::move(*result);                                       // 返回
    }

    /**
     * @brief 可选读+销账
     */
    [[nodiscard]] static std::expected<std::optional<std::array<T, N>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                         // 查节点
        if (!node) {                                                     // 缺失
            return std::optional<std::array<T, N>>{};                    // 合法
        }
        auto result = detail::parse_array_value<T, N>(*node, key, true); // 解析
        if (!result) {                                                   // 失败
            return std::unexpected(result.error());                      // 透传
        }
        (void)const_cast<toml::table&>(table).erase(key);                // 销账
        return std::optional<std::array<T, N>>{std::move(*result)};      // 返回
    }
};

/**
 * @brief std::map<std::string,V> 顶层特化
 * TOML 键名直接成为 C++ map 的 string 键
 */
template <typename V>
struct Deserialize<std::map<std::string, V>, void> {
    /**
     * @brief 必填读：key 缺失报错
     */
    [[nodiscard]] static std::expected<std::map<std::string, V>, std::string>
        read(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                    // 查节点
        if (!node) {                                                // 缺失
            return std::unexpected(detail::missing_key_error(key)); // 报错
        }
        return detail::parse_map_value<V>(*node, key, true);        // 节点级遍历子键解析
    }

    /**
     * @brief 可选读：key 缺失返回空
     */
    [[nodiscard]] static std::expected<std::optional<std::map<std::string, V>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                            // 查节点
        if (!node) {                                                        // 缺失
            return std::optional<std::map<std::string, V>>{};               // 合法置空
        }
        auto result = detail::parse_map_value<V>(*node, key, true);         // 解析
        if (!result) {                                                      // 失败
            return std::unexpected(result.error());                         // 透传
        }
        return std::optional<std::map<std::string, V>>{std::move(*result)}; // 包装返回
    }

    /**
     * @brief 必填读+销账
     */
    [[nodiscard]] static std::expected<std::map<std::string, V>, std::string>
        take(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                    // 查节点
        if (!node) {                                                // 缺失
            return std::unexpected(detail::missing_key_error(key)); // 报错
        }
        auto result = detail::parse_map_value<V>(*node, key, true); // 解析
        if (!result) {                                              // 失败
            return std::unexpected(result.error());                 // 透传
        }
        (void)const_cast<toml::table&>(table).erase(key);           // 销账
        return std::move(*result);                                  // 返回
    }

    /**
     * @brief 可选读+销账
     */
    [[nodiscard]] static std::expected<std::optional<std::map<std::string, V>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);                            // 查节点
        if (!node) {                                                        // 缺失
            return std::optional<std::map<std::string, V>>{};               // 合法
        }
        auto result = detail::parse_map_value<V>(*node, key, true);         // 解析
        if (!result) {                                                      // 失败
            return std::unexpected(result.error());                         // 透传
        }
        (void)const_cast<toml::table&>(table).erase(key);                   // 销账
        return std::optional<std::map<std::string, V>>{std::move(*result)}; // 返回
    }
};

} // namespace toml_helper
