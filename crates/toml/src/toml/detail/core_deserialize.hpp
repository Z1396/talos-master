#pragma once
// 引入toml官方库底层公共工具
#include "toml/detail/common.hpp"

// 自研TOML反序列化辅助工具命名空间
namespace toml_helper {

/**
 * @brief 通用反序列化模板主入口，按类型特化分发解析逻辑
 * @tparam T 需要从toml读取的目标类型
 * @tparam Enable SFINAE特化开关，用于不同类型分支偏特化
 */
template <typename T, typename Enable = void>
struct Deserialize;

// 内部实现细节命名空间，对外隐藏
namespace detail {

/**
 * @concept ReflectDeserializeCandidate
 * 满足该约束的结构体：可通过编译期反射自动遍历字段解析，不需要手动写read_from
 * 层层过滤排除不适合自动反射解析的类型：
 * 1. Reflectable<T>：支持字段反射
 * 2. !ReadFrom<T>：没有自定义read_from方法（优先自定义解析，其次反射）
 * 3. !TomlScalarValue<T>：不是基础标量
 * 4. !std::is_enum_v<T>：不是枚举
 * 5. !is_flatten_v：不是flatten平铺包装器
 * 6. !is_required_v：不是required必填包装器
 * 7. !is_std_optional_v：不是std::optional
 * 8. !is_std_vector_v：不是vector容器
 * 9. !is_std_array_v：不是array数组
 * 10. !is_std_map_v：不是map键值容器
 * 仅普通自定义数据结构体符合，走自动反射解析分支
 */
template <typename T>
concept ReflectDeserializeCandidate =
    Reflectable<T> && !ReadFrom<T> && !TomlScalarValue<T> && !std::is_enum_v<T>
    && !is_flatten_v<std::remove_cvref_t<T>> && !is_required_v<std::remove_cvref_t<T>>
    && !is_std_optional_v<std::remove_cvref_t<T>> && !is_std_vector_v<std::remove_cvref_t<T>>
    && !is_std_array_v<std::remove_cvref_t<T>> && !is_std_map_v<std::remove_cvref_t<T>>;

/**
 * @concept TableReadable
 * 能直接解析toml table表格的类型二选一：
 * 1. ReadFrom<T>：手动实现read_from自定义解析
 * 2. ReflectDeserializeCandidate<T>：支持自动反射字段解析
 */
template <typename T>
concept TableReadable = ReadFrom<T> || ReflectDeserializeCandidate<T>;

/**
 * @brief 针对实现ReadFrom的类型，调用自定义read_from方法填充对象
 * @param table 当前toml子表
 * @param out 待填充对象引用
 * @return 成功空expected，失败携带错误字符串
 */
template <ReadFrom T>
[[nodiscard]] std::expected<void, std::string> read_object_into(const toml::table& table, T& out) {
    return out.read_from(table);
}

/**
 * @brief 解析单个反射字段的核心递归函数
 * 分支处理：flatten/required/std::optional/普通字段四类包装器
 * @param table 父toml表
 * @param field_name 字段名
 * @param field 结构体字段引用（cv/引用自动剥离处理）
 * @return 字段解析结果
 */
template <typename T>
[[nodiscard]] std::expected<void, std::string>
    parse_reflected_field(const toml::table& table, std::string_view field_name, T& field);

/**
 * @brief 使用反射遍历所有字段，自动填充结构体对象
 * @param table 当前toml子表
 * @param out 待填充结构体
 * @return 整体解析结果，遇到第一个字段错误立即停止并保存错误
 */
template <ReflectDeserializeCandidate T>
[[nodiscard]] std::expected<void, std::string>
    read_reflected_into(const toml::table& table, T& out) {
    // 初始化成功状态
    std::expected<void, std::string> result{};
    // 反射遍历结构体全部字段：回调传入字段名、字段引用
    field_reflection::for_each_field(out, [&](std::string_view field_name, auto&& field) {
        // 短路逻辑：已有字段解析失败，跳过后续所有字段
        if (!result) 
        {
            return;
        }
        // 解析单个字段
        /*##  为什么这样写？
        ### 优势：变量作用域限制在 if 语句内*/
        if (auto field_result = parse_reflected_field(table, field_name, field); !field_result) 
        {
            // 捕获第一个错误，覆盖result
            result = std::unexpected(field_result.error());
        }
    });
    return result;
}

/**
 * @brief 统一入口：反射类型走反射填充逻辑
 */
template <ReflectDeserializeCandidate T>
[[nodiscard]] std::expected<void, std::string>
    read_object_into(const toml::table& table, T& out) {
    return read_reflected_into(table, out);
}

/**
 * @brief 解析单个反射字段，四层if constexpr静态分支处理各类包装器
 * 分支优先级：flatten平铺 > required必填 > std::optional可选 > 普通字段
 */
template <typename T>
[[nodiscard]] std::expected<void, std::string>
    parse_reflected_field(const toml::table& table, std::string_view field_name, T& field) 
{
    // 剥离字段的const/引用修饰，获取原始包装/基础类型
    using field_type = std::remove_cvref_t<T>;

    // 分支1：flatten<T> 平铺子表，把子表的键全部提升到父表读取，无嵌套层级
    if constexpr (is_flatten_v<field_type>) 
    {
        using flattened_type = flatten_value_t<field_type>;
        // 编译期静态断言：flatten内部类型必须支持table解析（反射/ReadFrom）、
        /*static_assert = 编译期断言
        作用：在编译阶段执行条件判断；条件不成立 → 直接触发编译失败，打印提示信息。
        重点区分：
        assert()：运行期断言，程序跑起来才检查，头文件 <cassert>
        static_assert：编译期断言，程序还没生成就检查，不需要头文件
        第一个参数：必须是 constexpr 布尔常量表达式
        第二个参数：编译报错时输出的字符串字面量*/
        static_assert(
            TableReadable<flattened_type>,
            "toml_helper::flatten<T> requires T to be reflective or implement read_from");
        // 直接用父table填充flatten内部对象，不查找当前field_name键
        // 调用递归解析flatten内部对象
        /*## field.get() 得到的是什么？
        作用：得到 flatten<T> 内部包裹的 T 类型引用
        field 是什么？ flatten<PresetEntry> 类型的引用 field.get() 得到什么？ 
        PresetEntry& （内部值的引用） 为什么要用 get() ？ 
        从包装器中取出内部值 然后用它做什么？ 
        递归解析 PresetEntry 的字段*/
        auto result = read_object_into(table, field.get());
        if (!result) 
        {
            // 错误追加字段名前缀，方便定位配置错误
            return std::unexpected(prefixed_error(field_name, result.error()));
        }
        return {};
    }
    // 分支2：required<T> 强制必填字段，配置文件缺少该键直接报错
    else if constexpr (is_required_v<field_type>) 
    {
        using required_type = required_value_t<field_type>;
        // 禁止嵌套 required<flatten<T>>
        static_assert(
            !is_flatten_v<required_type>,
            "toml_helper::required<toml_helper::flatten<T>> is not supported");
        // 在table查找对应键
        const toml::node* node = table.get(field_name);
        if (!node) {
            // 缺失必填键，返回错误
            return std::unexpected(missing_key_error(field_name));
        }
        // 调用通用Deserialize读取基础类型
        auto result = Deserialize<required_type>::take(table, field_name);
        if (!result) {
            return std::unexpected(result.error());
        }
        // 移动赋值给字段
        field = std::move(*result);
        return {};
    }
    // 分支3：std::optional<T> 可选字段，不存在键则置空不报错
    else if constexpr (is_std_optional_v<field_type>) 
    {
        using optional_type = optional_value_t<field_type>;
        // 禁止 std::optional<flatten<T>>
        static_assert(
            !is_flatten_v<optional_type>,
            "std::optional<toml_helper::flatten<T>> is not supported");
        const toml::node* node = table.get(field_name);
        if (!node) {
            // 无键：重置optional为空
            field.reset();
            return {};
        }
        // 存在键，正常解析
        auto result = Deserialize<optional_type>::take(table, field_name);
        if (!result) {
            return std::unexpected(result.error());
        }
        field = std::move(*result);
        return {};
    }
    // 分支4：普通无包装字段，存在键才解析，不存在直接跳过
    else {
        const toml::node* node = table.get(field_name);
        if (!node) {
            return {};
        }
        auto result = Deserialize<field_type>::take(table, field_name);
        if (!result) {
            return std::unexpected(result.error());
        }
        field = std::move(*result);
        return {};
    }
}

/**
 * @brief 从toml节点（table类型）反射解析结构体，返回带原始table指针的包装对象
 * @param node 待解析toml节点，必须是table
 * @param key 父层键名，用于错误打印
 * @return parsed_table_value：包含解析后的对象 + 原始table指针，用于后续冗余键检查
 */
template <ReflectDeserializeCandidate T>
[[nodiscard]] std::expected<parsed_table_value<T>, std::string>
    parse_reflect_from_node(const toml::node& node, std::string_view key) {
    // 反射解析要求类型支持默认构造
    static_assert(
        std::default_initializable<T>,
        "Reflection-based TOML parsing requires a default-initializable type");
    // 校验节点是否为表格，数组/标量直接报错
    const auto* table = node.as_table();
    if (!table) {
        return std::unexpected(invalid_table_error(key));
    }
    // 默认构造对象，反射填充所有字段
    T value{};
    auto result = read_reflected_into(*table, value);
    if (!result) {
        return std::unexpected(prefixed_error(key, result.error()));
    }
    // 返回包装结构体，保存对象与原始table指针
    return parsed_table_value<T>{std::move(value), table};
}

/**
 * @brief 根据键名从父table取出子节点，再反射解析结构体
 */
template <ReflectDeserializeCandidate T>
[[nodiscard]] std::expected<parsed_table_value<T>, std::string>
    parse_reflect_from_key(const toml::table& table, std::string_view key) {
    const toml::node* node = table.get(key);
    if (!node) {
        return std::unexpected(missing_table_error(key));
    }
    return parse_reflect_from_node<T>(*node, key);
}

/**
 * @brief 统一分发入口：TableReadable类型二选一（ReadFrom自定义 / 反射自动解析）
 */
template <TableReadable T>
[[nodiscard]] std::expected<parsed_table_value<T>, std::string>
    parse_object_from_node(const toml::node& node, std::string_view key) {
    if constexpr (ReadFrom<T>) {
        return parse_read_from_node<T>(node, key);
    } else {
        return parse_reflect_from_node<T>(node, key);
    }
}

/**
 * @brief 根据键名解析TableReadable类型
 */
template <TableReadable T>
[[nodiscard]] std::expected<parsed_table_value<T>, std::string>
    parse_object_from_key(const toml::table& table, std::string_view key) {
    if constexpr (ReadFrom<T>) {
        return parse_read_from_key<T>(table, key);
    } else {
        return parse_reflect_from_key<T>(table, key);
    }
}

} // namespace detail

// ==================================================================================
// Deserialize 主模板全特化分支，覆盖所有业务类型
// 四套标准接口统一规范：
// 1. read：读取键，保留原table内该键，校验冗余键
// 2. read_optional：可选键，不存在返回空optional
// 3. take：读取成功后从table erase删除该键，上层可循环take全部配置不重复
// 4. take_optional：可选+读取后删除键
// ==================================================================================

/**
 * @brief 兜底默认模板：无对应特化直接编译报错，禁止未实现类型解析
 */
template <typename T, typename Enable>
struct Deserialize {
    /**
     * @brief 读取指定key对应的值，不删除table内键
     */
    [[nodiscard]] static std::expected<T, std::string> read(const toml::table&, std::string_view) {
        // 编译期断言，always_false_v<T> 保证任何未特化类型直接报错
        static_assert(
            detail::always_false_v<T>,
            "No toml_helper::Deserialize<T> specialization available for this type");
        return std::unexpected("No deserializer available");
    }

    /**
     * @brief 可选读取：键不存在返回空std::optional<T>
     */
    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<T>{};
        }
        // 存在键则调用read，封装为optional返回
        return detail::as_optional(read(table, key));
    }

    /**
     * @brief take读取：解析成功后从table移除该键，避免重复读取
     */
    [[nodiscard]] static std::expected<T, std::string>
        take(const toml::table& table, std::string_view key) {
        return detail::erase_on_success(read(table, key), table, key);
    }

    /**
     * @brief 可选take：存在则读取并删除键，不存在返回空optional
     */
    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        return detail::erase_on_success_optional(read_optional(table, key), table, key);
    }
};

/**
 * @brief 特化1：基础标量类型（int/float/bool/string等TomlScalarValue）
 */
template <TomlScalarValue T>
struct Deserialize<T, void> {
    [[nodiscard]] static std::expected<T, std::string>
        read(const toml::table& table, std::string_view key) {
        // 读取必填标量，回调执行标量节点解析
        return detail::read_required<T>(table, key, [&](const toml::node& node) {
            return detail::parse_scalar_node<T>(node, key);
        });
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        // 可选标量读取
        return detail::read_optional_value<T>(table, key, [&](const toml::node& node) {
            return detail::parse_scalar_node<T>(node, key);
        });
    }

    [[nodiscard]] static std::expected<T, std::string>
        take(const toml::table& table, std::string_view key) {
        return detail::erase_on_success(read(table, key), table, key);
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        return detail::erase_on_success_optional(read_optional(table, key), table, key);
    }
};

/**
 * @brief 特化2：枚举类型，自动将toml字符串/数字映射为枚举值
 */
template <typename T>
requires(std::is_enum_v<T>) struct Deserialize<T, void> {
    [[nodiscard]] static std::expected<T, std::string>
        read(const toml::table& table, std::string_view key) {
        return detail::read_required<T>(table, key, [&](const toml::node& node) {
            return detail::parse_enum_node<T>(node, key);
        });
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        return detail::read_optional_value<T>(table, key, [&](const toml::node& node) {
            return detail::parse_enum_node<T>(node, key);
        });
    }

    [[nodiscard]] static std::expected<T, std::string>
        take(const toml::table& table, std::string_view key) {
        return detail::erase_on_success(read(table, key), table, key);
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        return detail::erase_on_success_optional(read_optional(table, key), table, key);
    }
};

/**
 * @brief 特化3：required<T> 必填包装器，缺失键直接报错
 */
template <typename T>
struct Deserialize<required<T>, void> {
    [[nodiscard]] static std::expected<required<T>, std::string>
        read(const toml::table& table, std::string_view key) {
        // 底层读取原生T，再包装为required<T>
        auto parsed = Deserialize<T>::read(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        return required<T>{std::move(*parsed)};
    }

    [[nodiscard]] static std::expected<std::optional<required<T>>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        auto parsed = Deserialize<T>::read_optional(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (!*parsed) {
            return std::optional<required<T>>{};
        }
        return std::optional<required<T>>{required<T>{std::move(**parsed)}};
    }

    [[nodiscard]] static std::expected<required<T>, std::string>
        take(const toml::table& table, std::string_view key) {
        auto parsed = Deserialize<T>::take(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        return required<T>{std::move(*parsed)};
    }

    [[nodiscard]] static std::expected<std::optional<required<T>>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        auto parsed = Deserialize<T>::take_optional(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (!*parsed) {
            return std::optional<required<T>>{};
        }
        return std::optional<required<T>>{required<T>{std::move(**parsed)}};
    }
};

/**
 * @brief 特化4：自定义解析类型 ReadFrom<T>（手动实现read_from成员函数）
 * 额外特性：解析完成校验table内是否存在未读取的冗余键，多余配置项直接报错
 */
template <ReadFrom T>
struct Deserialize<T, void> {
    [[nodiscard]] static std::expected<T, std::string>
        read(const toml::table& table, std::string_view key) {
        auto parsed = detail::parse_read_from_key<T>(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        // 校验子table是否有未读取冗余键，防止配置写错无效参数
        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }
        return std::move(parsed->value);
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<T>{};
        }
        auto parsed = detail::parse_read_from_node<T>(*node, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }
        return std::optional<T>{std::move(parsed->value)};
    }

    [[nodiscard]] static std::expected<T, std::string>
        take(const toml::table& table, std::string_view key) {
        auto parsed = detail::parse_read_from_key<T>(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }
        // 常量table强制转换mutable，erase已读取键
        (void)const_cast<toml::table&>(table).erase(key);
        return std::move(parsed->value);
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<T>{};
        }
        auto parsed = detail::parse_read_from_node<T>(*node, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }
        (void)const_cast<toml::table&>(table).erase(key);
        return std::optional<T>{std::move(parsed->value)};
    }
};

/**
 * @brief 特化5：反射自动解析结构体 ReflectDeserializeCandidate<T>
 * 同ReadFrom分支，自带冗余键校验，自动遍历所有字段填充
 */
template <detail::ReflectDeserializeCandidate T>
struct Deserialize<T, void> {
    [[nodiscard]] static std::expected<T, std::string>
        read(const toml::table& table, std::string_view key) {
        auto parsed = detail::parse_reflect_from_key<T>(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }
        return std::move(parsed->value);
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        read_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<T>{};
        }
        auto parsed = detail::parse_reflect_from_node<T>(*node, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }
        return std::optional<T>{std::move(parsed->value)};
    }

    [[nodiscard]] static std::expected<T, std::string>
        take(const toml::table& table, std::string_view key) {
        auto parsed = detail::parse_reflect_from_key<T>(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }
        (void)const_cast<toml::table&>(table).erase(key);
        return std::move(parsed->value);
    }

    [[nodiscard]] static std::expected<std::optional<T>, std::string>
        take_optional(const toml::table& table, std::string_view key) {
        const toml::node* node = table.get(key);
        if (!node) {
            return std::optional<T>{};
        }
        auto parsed = detail::parse_reflect_from_node<T>(*node, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        if (auto unread = detail::error_if_unread(*parsed->parsed_table, key); !unread) {
            return std::unexpected(unread.error());
        }
        (void)const_cast<toml::table&>(table).erase(key);
        return std::optional<T>{std::move(parsed->value)};
    }
};

} // namespace toml_helper