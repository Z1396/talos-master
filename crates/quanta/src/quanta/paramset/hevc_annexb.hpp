#pragma once
// 头文件保护宏，防止同一头文件被多次重复包含，避免重定义编译错误

// 上层流编码器定义，包含 EncodedPacket 编码包结构体
#include "quanta/stream_encoder.hpp"

// C++标准基础类型头文件
#include <cstddef>    // size_t 标准无符号长度类型
#include <cstdint>    // 固定宽度整数 uint8_t 等，跨平台统一字节类型
#include <expected>   // C++23 预期返回类型，统一承载执行结果+错误信息
#include <string>     // 存储错误描述字符串

namespace quanta {

/**
 * @brief HEVC VPS/SPS/PPS 参数集导出状态标记结构体
 * 用于全局控制参数集仅导出一次，避免重复写入本地文件
 */
struct HevcParameterSetExportState {
    // 标记是否已经导出过VPS/SPS/PPS参数集，默认false未导出
    bool exported = false;
};

/**
 * @brief 判断HEVC NALU单元类型是否为IDR关键帧
 * @param type NALU头部5位类型字段（提取后纯数值）
 * @return true 是IDR图像NALU；false 非IDR
 * noexcept 函数不会抛出C++异常，纯无副作用判断
 * [[nodiscard]] 强制调用方接收返回值，防止丢弃判断结果
 */
[[nodiscard]] bool is_hevc_idr_nalu(uint8_t type) noexcept;

/**
 * @brief 判断HEVC NALU单元是否为参数集（VPS/SPS/PPS）
 * @param type NALU头部5位类型字段
 * @return true 是VPS/SPS/PPS其中一类；false 普通图像/SEI等NALU
 */
[[nodiscard]] bool is_hevc_parameter_set_nalu(uint8_t type) noexcept;

/**
 * @brief 剥离编码包内所有HEVC参数集NALU（VPS/SPS/PPS），仅保留图像数据
 * @param packet 待处理编码包，会原地修改包内裸流数据
 * @return expected<bool, string>
 *        成功：返回true代表包内存在并剥离了参数集；false代表无参数集
 *        失败：携带错误描述字符串，包数据保持不变
 */
[[nodiscard]] std::expected<bool, std::string>
    strip_hevc_parameter_sets(EncodedPacket& packet) noexcept;

/**
 * @brief 一次性导出HEVC VPS/SPS/PPS参数集到本地文件，仅首次遇到IDR帧执行导出
 * 结合 HevcParameterSetExportState 状态标记，全局只导出一次，重复调用无重复写入
 * @param packet 编码包，必须是IDR关键帧包才包含完整参数集
 * @param state 导出状态上下文，记录是否已完成导出
 * @return expected<void, string> 成功空返回；失败携带错误文本
 */
[[nodiscard]] std::expected<void, std::string> export_hevc_parameter_sets_once(
    const EncodedPacket& packet, HevcParameterSetExportState& state) noexcept;

} // namespace quanta