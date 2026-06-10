// 头文件保护，避免重复包含
#pragma once

// 引入轻量字符串视图，存储编译期固定文本，无内存拷贝
#include <string_view>

// 项目顶层命名空间
namespace fcs {

/**
 * @brief 项目编译构建信息结构体
 * 存储程序编译时的环境、版本、Git 信息，用于日志、调试、版本溯源
 */
struct BuildInfo {
    std::string_view build_date;   // 编译日期
    std::string_view build_host;   // 编译机器主机名
    std::string_view git_commit;   // Git 提交哈希值（commit id）
    std::string_view git_branch;    // Git 分支名称
};

/**
 * @brief 获取当前程序的编译构建信息
 * @return BuildInfo 结构体，包含日期、主机、Git 版本等信息
 *
 * 修饰符说明：
 * - [[nodiscard]]：强制调用方接收返回值，防止无意丢弃版本信息
 * - noexcept：函数不会抛出异常
 */
[[nodiscard]] auto build_info() noexcept -> BuildInfo;

} // namespace fcs