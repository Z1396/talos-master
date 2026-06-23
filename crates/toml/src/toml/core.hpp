#pragma once
// 头文件保护，防止多次包含

// toml++ 基础公共定义、类型别名、错误结构体
#include "toml/detail/common.hpp"
// 核心反序列化底层实现：基础类型解析、嵌套对象读取逻辑
#include "toml/detail/core_deserialize.hpp"

// 项目TOML配置读写工具命名空间
namespace toml_helper {

// ============================================================================
// 基础单键读取接口（读取单个key，不消耗原表条目）
// ============================================================================
/// 读取必填基础类型，key缺失/类型不匹配返回错误
/// @tparam T 目标基础类型 int/double/bool/std::string
/// @param table TOML根表/子表
/// @param key 配置键名
/// @return 成功返回T，失败返回错误字符串
template <typename T>
[[nodiscard]] std::expected<T, std::string> read(const toml::table& table, std::string_view key) {
    return Deserialize<T>::read(table, key);
}

/// 读取可选基础类型，key不存在返回std::nullopt，不报错
template <typename T>
[[nodiscard]] std::expected<std::optional<T>, std::string>
    read_optional(const toml::table& table, std::string_view key) {
    return Deserialize<T>::read_optional(table, key);
}

/// 校验table内所有节点都已经被读取过，存在未使用配置项则报错
/// @param context 上下文标识，日志区分是哪个配置文件/结构体
[[nodiscard]] inline std::expected<void, std::string>
    ensure_all_read(const toml::table& table, std::string_view context = {}) noexcept {
    return detail::error_if_unread(table, context);
}

// ============================================================================
// take 系列：读取后标记节点为已消耗，配合 ensure_all_read 检测冗余配置
// ============================================================================
/// 读取必填项，同时标记该key已被读取，防止未读字段告警
template <typename T>
[[nodiscard]] std::expected<T, std::string> take(const toml::table& table, std::string_view key) {
    return Deserialize<T>::take(table, key);
}

/// 读取可选项，标记key已消耗，不存在返回nullopt
template <typename T>
[[nodiscard]] std::expected<std::optional<T>, std::string>
    take_optional(const toml::table& table, std::string_view key) {
    return Deserialize<T>::take_optional(table, key);
}

// ============================================================================
// 结构体整体反序列化：read_into / from_table 标准入口
// ============================================================================
/// 将table完整反序列化填充到已存在的结构体out
/// @tparam T 实现TableReadable约束的自定义配置结构体
/// @param table 待解析TOML表
/// @param out 输出结构体对象
/// @return 成功void，失败返回错误信息
template <detail::TableReadable T>
[[nodiscard]] std::expected<void, std::string> read_into(const toml::table& table, T& out) {
    // 临时构造空结构体，防止半填充污染原有数据
    T parsed{};
    // 递归解析table所有字段填入临时结构体
    auto result = detail::read_object_into(table, parsed);
    // 解析字段类型/语法错误直接返回
    if (!result) {
        return std::unexpected(result.error());
    }
    // 校验整张表无多余未读取配置键
    if (auto unread = detail::error_if_unread(table, {}); !unread) {
        return std::unexpected(unread.error());
    }
    // 移动语义转移解析结果，避免大结构体拷贝
    out = std::move(parsed);
    return {};
}

/// 一键从table生成完整配置结构体（对外最常用接口 from_table<T>）
/// 要求T支持默认构造，内部封装read_into
template <detail::TableReadable T>
requires(std::default_initializable<T>)
[[nodiscard]] std::expected<T, std::string> from_table(const toml::table& table) {
    T value{};
    auto result = read_into(table, value);
    if (!result) {
        return std::unexpected(result.error());
    }
    // 移动返回，不拷贝完整结构体
    return std::move(value);
}

// ============================================================================
// flatten<T> 平铺结构体专用读取，不校验子表冗余字段
// ============================================================================
/// 平铺模式解析：结构体字段直接读取顶层键，不进入独立子表
/// 不执行 ensure_all_read 全局未读检测，允许顶层其他无关key存在
template <detail::TableReadable T>
[[nodiscard]] std::expected<void, std::string>
    read_flatten_into(const toml::table& table, T& out, std::string_view context) noexcept {
    auto result = detail::read_object_into(table, out);
    if (result) {
        return {};
    }
    // 错误信息增加前缀标识当前平铺字段名
    return std::unexpected(detail::prefixed_error(context, result.error()));
}

// ============================================================================
// detail 内部底层工具，仅库内部使用，业务禁止直接调用
// ============================================================================
namespace detail {

/// 统一赋值可选字段：区分std::optional / required<T> / 普通字段，自动移动资源
/// @param field 结构体目标字段
/// @param parsed take_optional返回的std::optional<T>右值
template <typename Field, typename Parsed>
constexpr void assign_read_opt(Field& field, Parsed&& parsed) {
    // 如果字段是标准std::optional<T>
    if constexpr (is_std_optional_v<std::remove_cvref_t<Field>>) {
        // 解析无值则赋值nullopt，有值存入内部T
        field = std::forward<Parsed>(parsed).value_or(std::nullopt);
    } else if (parsed) {
        // 普通字段/required<T>，存在值则移动赋值，避免拷贝
        field = std::move(*parsed);
    }
}

/// 单个结构体字段通用读取逻辑，自动区分 required<T> 必填 / 普通可选字段
/// @param table 源TOML表
/// @param key 字段名
/// @param field 结构体成员引用
template <typename Field>
[[nodiscard]] std::expected<void, std::string>
    read_field(const toml::table& table, std::string_view key, Field& field) {
    using field_type = std::remove_cvref_t<Field>;

    // 分支1：字段是 required<T> 标记，必须存在，缺失直接报错
    if constexpr (is_required_v<field_type>) {
        auto parsed = Deserialize<required_value_t<field_type>>::take(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        // 移动赋值到required包装器内部
        field = std::move(*parsed);
        return {};
    } else {
        // 分支2：普通可选字段，key不存在保留结构体默认值
        auto parsed = Deserialize<field_type>::take_optional(table, key);
        if (!parsed) {
            return std::unexpected(parsed.error());
        }
        // 键存在才赋值，不存在跳过保留原有默认值
        if (*parsed) {
            field = std::move(**parsed);
        }
        return {};
    }
}

} // namespace detail

// ============================================================================
// 三个工具宏：简化结构体内部手动读取字段代码，消除重复模板代码
// do{}while(0) 标准宏封装，保证单行if/for也能正常执行多语句
// ============================================================================
/// 读取可选字段，自动处理std::optional/普通类型
/// #val 预编译取变量名字符串作为TOML键名
#define READ_OPT(tbl, val)                                               \
    do {                                                                 \
        auto res = toml_helper::take_optional<decltype(val)>(tbl, #val); \
        if (!res) {                                                      \
            return std::unexpected(res.error());                         \
        }                                                                \
        toml_helper::detail::assign_read_opt((val), std::move(*res));    \
    } while (0)

/// 读取必填字段，自动识别required<T>包装器，缺失直接返回错误
#define READ(tbl, val)                                                  \
    do {                                                                \
        auto res = toml_helper::detail::read_field((tbl), #val, (val)); \
        if (!res) {                                                     \
            return std::unexpected(res.error());                        \
        }                                                               \
    } while (0)

/// 读取flatten平铺子结构体，不校验全局多余key
#define READ_FLATTEN(tbl, val)                                         \
    do {                                                               \
        auto res = toml_helper::read_flatten_into((tbl), (val), #val); \
        if (!res) {                                                    \
            return std::unexpected(res.error());                       \
        }                                                              \
    } while (0)

// ============================================================================
// 配置合并工具：基础配置 + 覆盖配置，实现配置分层
// base：底层默认配置
// override_table：上层用户自定义配置，同名key覆盖底层
// 子表递归合并，基础值直接覆盖
// ============================================================================
[[nodiscard]] inline std::expected<toml::table, std::string>
    merge_configs(const toml::table& base, const toml::table& override_table) noexcept {
    // 拷贝基础配置作为结果容器
    toml::table result = base;

    // 遍历所有覆盖层配置项
    for (const auto& [key, override_node] : override_table) {
        // 查找底层配置中同名节点
        auto* base_node               = result.get(key);
        const auto* base_subtable     = base_node ? base_node->as_table() : nullptr;
        const auto* override_subtable = override_node.as_table();

        // 两边都是子表：递归合并子表，不直接覆盖
        if (base_subtable && override_subtable) {
            auto merged = merge_configs(*base_subtable, *override_subtable);
            if (!merged) {
                return std::unexpected(merged.error());
            }
            // 移动合并后的子表写入结果，避免拷贝
            result.insert_or_assign(key, std::move(*merged));
            continue;
        }

        // 普通值/一边是子表一边不是：上层配置直接覆盖底层
        result.insert_or_assign(key, override_node);
    }

    return result;
}

} // namespace toml_helper