#pragma once
// 类型名反混淆工具头文件
#include "scheduler/demangle.hpp"

// 字符判断 std::isalnum
#include <cctype>
// std::free 堆内存释放
#include <cstdlib>
// std::strlen 字符串长度查询
#include <cstring>
// GCC内置ABI类型反混淆函数 abi::__cxa_demangle
#include <cxxabi.h>

namespace talos::scheduler::detail {

/**
 * @brief 对typeid原始C++ mangled名称进行反混淆+轻量化清洗，输出可读简化类型名
 * @param name typeid(T).name() 返回的原始mangled字符串指针
 * @return 清洗完毕的可读类型std::string
 *
 * 完整流程：
 * 1. 空指针保护，返回<null>占位
 * 2. 调用GCC内置abi接口解析原始mangled名
 * 3. 移除所有命名空间前缀（namespace::）
 * 4. 删除标准库容器冗余模板参数（allocator、char_traits等）
 * 5. 模板内部压缩空格、逗号后多余空格，精简输出
 * 6. 首尾去空白，空字符串兜底占位
 */
std::string demangle(const char* name) noexcept 
{
    // 空指针兜底，直接返回占位字符串
    if (!name) {
        return "<null>";
    }

    // 反混淆执行状态码，0代表成功
    int status = -1;
    // 调用GCC内置ABI函数解析mangled名字，返回堆分配char*，需要手动free
    char* p    = abi::__cxa_demangle(name, nullptr, nullptr, &status);

    std::string s;
    if (status == 0 && p) {
        // 解析成功，拷贝堆字符串到std::string
        s = p;
        // 释放abi分配的堆内存，防止内存泄漏
        std::free(p);
    } else {
        // 解析失败，直接使用原始mangled字符串兜底
        s = name;
    }

    // ========== 步骤1：删除全部命名空间前缀 xxx:: ==========
    std::size_t pos;
    // 循环查找"::"命名空间分隔符
    while ((pos = s.find("::")) != std::string::npos) {
        std::size_t start = pos;
        // 向前回溯，找到命名空间起始边界（字母、数字、下划线）
        while (start > 0
               && (std::isalnum(static_cast<unsigned char>(s[start - 1])) || s[start - 1] == '_')) {
            --start;
        }
        // 擦除 [start, pos+2) 区间，删除整个namespace::前缀
        s.erase(start, pos - start + 2);
    }

    // ========== 步骤2：删除标准库冗余模板参数（allocator/char_traits/less等） ==========
    // 需要过滤的标准库冗余模板片段数组
    const char* patterns[] = {", std::allocator<", ", std::char_traits<", ", std::less<",
                              ", std::equal_to<",  ", std::hash<",        ", std::default_delete<"};
    for (const char* pat : patterns) {
        std::size_t p = 0;
        // 循环查找当前冗余片段
        while ((p = s.find(pat, p)) != std::string::npos) {
            // 从片段起点向后匹配对应闭合'>'
            std::size_t end = s.find('>', p + std::strlen(pat));
            if (end != std::string::npos) {
                // 擦除整个冗余模板参数块
                s.erase(p, end - p + 1);
            } else {
                // 找不到闭合>，终止本轮遍历，避免越界
                break;
            }
        }
    }

    // ========== 步骤3：模板内部压缩多余空格，精简输出 ==========
    std::string cleaned;
    bool in_template = false; // 标记当前字符是否处于模板<>内部
    for (std::size_t i = 0; i < s.size(); ++i) {
        char c = s[i];
        // 进入模板块，标记状态并保留'<'
        if (c == '<') {
            in_template = true;
            cleaned += c;
            continue;
        }
        // 退出模板块，标记状态并保留'>'
        if (c == '>') {
            in_template = false;
            cleaned += c;
            continue;
        }
        // 仅模板内部过滤空格、逗号后尾随空格
        if (in_template && (c == ' ' || (c == ',' && (i + 1 < s.size() && s[i + 1] == ' ')))) {
            // 逗号+空格组合，跳过逗号后空格
            if (c == ',' && i + 1 < s.size() && s[i + 1] == ' ') {
                ++i;
            }
            // 丢弃空格字符，不写入cleaned
            continue;
        }
        // 普通字符直接保留
        cleaned += c;
    }

    // ========== 步骤4：首尾去除空白字符（空格、制表符） ==========
    // 找到第一个非空白字符下标
    std::size_t start = cleaned.find_first_not_of(" \t");
    if (start == std::string::npos) {
        // 全空白字符串，返回占位
        return "<empty>";
    }
    // 找到最后一个非空白字符下标
    std::size_t end = cleaned.find_last_not_of(" \t");
    // 截取有效区间，裁剪首尾空白
    cleaned         = cleaned.substr(start, end - start + 1);

    // 兜底：清洗后空串返回<unknown>，否则返回清洗结果
    return cleaned.empty() ? "<unknown>" : cleaned;
}

} // namespace talos::scheduler::detail