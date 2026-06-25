// 头文件保护宏：保证该头文件只会被编译一次，防止多重包含导致重复定义报错
#pragma once

// C++23 标准库：std::expected 用于无异常式错误处理
#include <expected>
// 标准字符串，存放错误提示文本
#include <string>
// 只读字符串视图，零拷贝返回系统信息（用户名、主机名）
#include <string_view>

// 项目顶层命名空间 talos，primitive 代表底层基础原生工具模块
namespace talos::primitive {

/**
 * @brief 系统基础信息静态工具类
 * 仅提供静态接口获取操作系统用户名、主机名，不可实例化
 */
class SystemInfo {
public:
    /**
     * @brief 删除默认构造函数，禁止外部创建 SystemInfo 对象
     * 本类全部接口均为static静态函数，无需实例化，杜绝错误构造
     */
    SystemInfo() = delete;

    /**
     * @brief 自定义返回值类型别名，统一封装成功/失败两种返回结果
     * std::expected<T, E>：C++23 标准错误处理容器
     * 成功分支：存储 std::string_view，零拷贝返回字符串（指向静态缓冲区）
     * 失败分支：存储 std::string，存放可读的错误描述信息
     */
    using Result = std::expected<std::string_view, std::string>;

    /**
     * @brief 获取当前终端登录的系统用户名
     * @return Result
     *      成功：携带指向静态缓冲区的用户名 string_view
     *      失败：携带系统调用失败的错误文本
     * @noexcept 函数不会抛出C++异常，所有错误通过Result承载
     * @[[nodiscard]] 强制调用方接收返回值，忽略返回值会触发编译警告，防止遗漏错误
     */
    [[nodiscard]] static auto get_username() noexcept -> Result;

    /**
     * @brief 获取本机操作系统主机名(hostname)
     * @return Result
     *      成功：携带主机名字符串视图
     *      失败：携带系统调用失败错误信息
     * @noexcept 无C++异常抛出
     * @[[nodiscard]] 禁止丢弃返回结果，避免错误被静默忽略
     */
    [[nodiscard]] static auto get_hostname() noexcept -> Result;
};

} // namespace talos::primitive