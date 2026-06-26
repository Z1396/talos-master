#pragma once
// 头文件保护宏，防止该头文件被多次重复include，标准C++头文件防护写法

// 命名空间：fcs业务工程下的可视化工具模块，存放Foxglove可视化配套工具函数
namespace fcs::visualization {

/**
 * @brief 将纳秒级uint64时间戳，转换为Foxglove协议标准Timestamp时间结构体
 * @param ns 输入：全局统一纳秒时间戳（系统clock::now_ns()输出格式）
 * @return foxglove::schemas::Timestamp 协议标准时分纳秒拆分结构体
 * @noexcept 函数无异常抛出
 * [[nodiscard]] 标记返回值不可丢弃，调用必须接收返回对象，避免误用忽略时间戳
 */
[[nodiscard]] inline ::foxglove::schemas::Timestamp timestamp_from_ns(uint64_t ns) noexcept {
    // 实例化Foxglove定义的标准时间戳结构体
    ::foxglove::schemas::Timestamp t;
    // 1秒 = 1e9纳秒，整除得到总秒数，强转uint32存入sec字段
    t.sec  = static_cast<uint32_t>(ns / 1'000'000'000ULL);
    // 取模得到剩余不足1秒的纳秒部分，存入nsec字段
    t.nsec = static_cast<uint32_t>(ns % 1'000'000'000ULL);
    return t;
}

/**
 * @brief JSON字符串转为uint8二进制字节数组，用于Foxglove消息payload填充
 * @param json_str 输入：完整JSON序列化字符串
 * @return std::vector<uint8_t> 二进制字节容器，存储字符串原始字节
 */
[[nodiscard]] inline std::vector<uint8_t> json_to_bytes(const std::string& json_str) {
    // vector初始化：传入起始、结束uint8指针，直接拷贝字符串底层内存字节
    return {
        // string底层char* 强制转无符号8位字节指针（协议传输统一uint8）
        reinterpret_cast<const uint8_t*>(json_str.data()),
        // 字符串末尾指针，左闭右开区间 [begin, end)
        reinterpret_cast<const uint8_t*>(json_str.data() + json_str.size())};
}

} // namespace fcs::visualization