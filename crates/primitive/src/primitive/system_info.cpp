// 引入头文件：包含 SystemInfo 类定义、Result 错误返回类型别名
#include "primitive/system_info.hpp"

// 标准库容器：栈/静态缓冲区用定长数组存储字符串
#include <array>
// errno 全局错误码、strerror 错误文本转换函数
#include <cerrno>
// strerror 字符串操作依赖
#include <cstring>

// Linux POSIX 系统调用头文件：getlogin_r / gethostname
#include <unistd.h>

// 项目顶层命名空间 talos，primitive 基础底层工具模块
namespace talos::primitive {

/**
 * @brief 获取当前登录操作系统的用户名（POSIX Linux 系统调用）
 * @return Result 兼容两种分支：
 *         成功：返回 std::string_view 指向静态缓冲区的用户名字符串
 *         失败：返回 std::unexpected<std::string> 携带错误描述
 * @noexcept 函数保证不会抛出C++异常，错误通过 Result 承载
 * @note 缓冲区 static 静态生命周期，进程全程有效，无需手动释放
 */
auto SystemInfo::get_username() noexcept -> Result {
    // 定义缓冲区固定大小 256 字节，足够容纳绝大多数系统用户名
    static constexpr std::size_t kBufSize = 256;
    // static 静态数组：只在第一次调用时初始化一次，全局生命周期，线程不安全！
    static std::array<char, kBufSize> buf{};

    // POSIX 可重入版获取登录用户名：getlogin_r(缓冲区指针, 缓冲区长度)
    // 返回值 0 = 成功；非0 = 失败，会设置全局 errno 错误码
    const int ret = getlogin_r(buf.data(), kBufSize);
    if (ret != 0) {
        // 调用失败：构造错误信息字符串，包装进 std::unexpected 返回错误分支
        // std::strerror(errno) 将数字错误码转为人类可读文字（如 "No such file or directory"）
        return std::unexpected(
            std::string("get_username: getlogin_r failed: ") + std::strerror(errno));
    }
    // 成功：返回 string_view 视图，不拷贝内存，直接引用静态buf内字符串
    return std::string_view{buf.data()};
}

/**
 * @brief 获取本机操作系统主机名（设备 hostname）
 * @return Result 成功返回主机名字符串视图；失败返回错误描述
 * @noexcept 无C++异常抛出
 */
auto SystemInfo::get_hostname() noexcept -> Result {
    // 主机名缓冲区固定256字节
    static constexpr std::size_t kBufSize = 256;
    // 静态缓冲区，全局只初始化一次，复用内存
    static std::array<char, kBufSize> buf{};

    // POSIX 系统调用 gethostname：读取本机主机名写入缓冲区
    // 返回0成功，非0失败并设置errno
    const int ret = gethostname(buf.data(), kBufSize);
    if (ret != 0) {
        // 构造错误信息，返回错误分支
        return std::unexpected(
            std::string("get_hostname: gethostname failed: ") + std::strerror(errno));
    }
    // 成功返回字符串视图，指向静态缓冲区
    return std::string_view{buf.data()};
}

} // namespace talos::primitive