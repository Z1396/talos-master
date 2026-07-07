#include "hevc_annexb.hpp"
// 引入头文件，包含结构体、函数声明：HevcParameterSetExportState、EncodedPacket、NALU判断接口等

// C++标准库头文件
#include <algorithm>        // std::min、std::copy_n 内存拷贝、数值取最小
#include <exception>        // 标准异常基类捕获，统一异常包装为错误字符串
#include <filesystem>       // C++17 文件系统，创建输出目录、拼接文件路径
#include <fstream>          // 二进制文件写入，导出VPS/SPS/PPS到本地hevc裸流文件
#include <memory>           // std::unique_ptr 动态内存管理剥离后的码流缓存

namespace quanta {
namespace { // 匿名命名空间，内部工具函数/常量仅当前编译单元可见，不对外暴露

// ===================== HEVC NALU 类型常量定义 =====================
// HEVC AnnexB 标准NALU头部5bit类型字段
/**
 * @brief IDR_W_RADL 带前置参考帧的IDR图像 NALU类型值
 */
constexpr uint8_t kHevcNaluIdrWRadl = 19;
/**
 * @brief IDR_N_LP 无前置参考帧的IDR图像 NALU类型值
 */
constexpr uint8_t kHevcNaluIdrNLp   = 20;
/**
 * @brief VPS 视频参数集 NALU类型
 */
constexpr uint8_t kHevcNaluVps      = 32;
/**
 * @brief SPS 序列参数集 NALU类型
 */
constexpr uint8_t kHevcNaluSps      = 33;
/**
 * @brief PPS 图像参数集 NALU类型
 */
constexpr uint8_t kHevcNaluPps      = 34;

/**
 * @brief HEVC AnnexB 标准4字节起始码 0x00 00 00 01，分割各个NALU单元
 */
constexpr uint8_t kStartCode4[] = {0x00, 0x00, 0x00, 0x01};

/**
 * @brief 封装完整HEVC三套参数集缓存容器
 * 存储从IDR帧中提取的VPS、SPS、PPS原始NALU二进制数据
 */
struct HevcParameterSets {
    std::vector<uint8_t> vps; // 视频参数集原始NALU数据
    std::vector<uint8_t> sps; // 序列参数集原始NALU数据
    std::vector<uint8_t> pps; // 图像参数集原始NALU数据

    /**
     * @brief 判断三套参数集是否全部完整提取（均非空）
     * @return true 同时存在VPS/SPS/PPS；false 缺失任意一套
     * noexcept 无异常抛出，[[nodiscard]]强制接收返回值
     */
    [[nodiscard]] bool complete() const noexcept {
        return !vps.empty() && !sps.empty() && !pps.empty();
    }
};

/**
 * @brief 从编码包中截取指定NALU的二进制数据，拷贝到目标参数集容器
 * @param dst 目标vector，存放VPS/SPS/PPS原始数据
 * @param packet 完整编码包裸流缓冲区
 * @param nalu 当前待拷贝的NALU元信息（偏移、长度、类型）
 */
void assign_parameter_set(
    std::vector<uint8_t>& dst, const EncodedPacket& packet, const EncodedPacketNalu& nalu) {
    // 从NALU偏移到包末尾剩余字节长度
    const size_t remaining   = packet.size - nalu.packet_offset;
    // 实际有效NALU长度：取NALU声明长度、剩余字节两者最小值，防止越界
    const size_t packet_size = std::min(nalu.packet_size, remaining);
    // NALU起始内存指针
    const uint8_t* data      = packet.data.get() + nalu.packet_offset;
    // 覆盖写入目标容器，完整拷贝一段NALU二进制
    dst.assign(data, data + packet_size);
}

/**
 * @brief 遍历编码包内所有NALU，提取VPS/SPS/PPS三套参数集
 * @param packet 携带完整参数集的IDR关键帧编码包
 * @return HevcParameterSets 填充好VPS/SPS/PPS的参数集结构体
 */
[[nodiscard]] HevcParameterSets collect_parameter_sets(const EncodedPacket& packet) {
    HevcParameterSets sets;
    // 遍历包内所有NALU元数据
    for (const auto& nalu : packet.nalus) {
        // 跳过空长度、偏移越界的非法NALU
        if (nalu.packet_size == 0 || nalu.packet_offset >= packet.size) {
            continue;
        }

        // 根据NALU类型分流拷贝对应参数集
        switch (nalu.type) {
        case kHevcNaluVps: assign_parameter_set(sets.vps, packet, nalu); break;
        case kHevcNaluSps: assign_parameter_set(sets.sps, packet, nalu); break;
        case kHevcNaluPps: assign_parameter_set(sets.pps, packet, nalu); break;
        default: break; // 非参数集NALU直接跳过
        }
    }
    return sets;
}

/**
 * @brief 计算三套参数集拼接AnnexB裸流总字节大小（含每个NALU前置4字节起始码）
 * @param sets 完整VPS/SPS/PPS参数集
 * @return 拼接后总字节长度，用于预分配vector内存减少扩容开销
 * noexcept 无异常抛出
 */
[[nodiscard]] size_t annexb_parameter_sets_size(const HevcParameterSets& sets) noexcept {
    // 每个NALU = 4字节起始码 + NALU数据长度，三套相加
    return (sizeof(kStartCode4) + sets.vps.size()) + (sizeof(kStartCode4) + sets.sps.size())
         + (sizeof(kStartCode4) + sets.pps.size());
}

/**
 * @brief 将单套参数集NALU追加到目标二进制缓冲区，自动前置4字节起始码
 * @param nalu 单套参数集原始二进制（VPS/SPS/PPS其一）
 * @param dst 拼接输出缓冲区
 */
void append_parameter_set(const std::vector<uint8_t>& nalu, std::vector<uint8_t>& dst) {
    // 先写入4字节起始码
    dst.insert(dst.end(), kStartCode4, kStartCode4 + sizeof(kStartCode4));
    // 再写入NALU原始数据
    dst.insert(dst.end(), nalu.begin(), nalu.end());
}

/**
 * @brief 将完整VPS/SPS/PPS导出为本地hevc裸流文件 output/quanta_vps_sps_pps.hevc
 * @param sets 完整三套参数集
 * @return std::expected 成功返回文件路径字符串；失败返回错误描述
 * noexcept 函数内部捕获所有标准异常，对外不抛异常
 */
[[nodiscard]] std::expected<std::string, std::string>
    export_hevc_parameter_sets(const HevcParameterSets& sets) noexcept {
    // 校验三套参数集是否齐全，缺失直接返回错误
    if (!sets.complete()) {
        return std::unexpected("HEVC parameter sets are incomplete");
    }

    try {
        // 拼接输出目录：当前程序运行目录下 output 文件夹
        const auto output_dir = std::filesystem::current_path() / "output";
        // 不存在则递归创建output目录
        std::filesystem::create_directories(output_dir);

        // 完整输出文件路径
        const auto path = output_dir / "quanta_vps_sps_pps.hevc";
        std::vector<uint8_t> bytes;
        // 预分配足够内存，避免多次扩容拷贝
        bytes.reserve(annexb_parameter_sets_size(sets));
        // 依次拼接 VPS、SPS、PPS，每段前置起始码
        append_parameter_set(sets.vps, bytes);
        append_parameter_set(sets.sps, bytes);
        append_parameter_set(sets.pps, bytes);

        // 二进制覆盖模式打开文件，截断原有内容
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        // 文件打开失败返回错误
        if (!out) {
            return std::unexpected("open HEVC parameter set output file failed");
        }
        // 批量写入二进制裸流
        out.write(
            reinterpret_cast<const char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
        // 写入过程IO失败、磁盘满等情况返回错误
        if (!out) {
            return std::unexpected("write HEVC parameter set output file failed");
        }

        // 成功返回文件完整路径字符串
        return path.string();
    } catch (const std::exception& e) {
        // 捕获文件系统、内存分配等所有标准异常，包装为错误信息返回
        return std::unexpected(std::string("export HEVC parameter sets: ") + e.what());
    }
}

} // 匿名命名空间内部工具结束

// ===================== 对外暴露接口实现 =====================
/**
 * @brief 判断NALU类型是否为HEVC IDR关键帧
 * @param type NALU 5bit类型值
 * @return true IDR帧；false 普通P/B帧
 * noexcept 无异常抛出
 */
bool is_hevc_idr_nalu(uint8_t type) noexcept {
    return type == kHevcNaluIdrWRadl || type == kHevcNaluIdrNLp;
}

/**
 * @brief 判断NALU是否为VPS/SPS/PPS参数集
 * @param type NALU类型值
 * @return true 是参数集NALU；false 图像/SEI等数据NALU
 * noexcept 无异常抛出
 */
bool is_hevc_parameter_set_nalu(uint8_t type) noexcept {
    return type == kHevcNaluVps || type == kHevcNaluSps || type == kHevcNaluPps;
}

/**
 * @brief 原地剥离编码包内所有VPS/SPS/PPS参数集NALU，仅保留图像裸流
 * 适用场景：RTMP/RTSP实时推流，去除重复参数集降低带宽占用
 * @param packet 待处理编码包，会修改内部data裸流缓冲区、清空nalus元数据
 * @return std::expected<bool, std::string>
 *        成功：true=剥离过参数集；false=包内无参数集无需修改
 *        失败：携带错误字符串，原始包数据保持不变
 * noexcept 内部捕获所有标准异常，不对外抛出
 */
std::expected<bool, std::string> strip_hevc_parameter_sets(EncodedPacket& packet) noexcept {
    try {
        // 空包、空指针、无NALU元数据直接返回false，无需处理
        if (packet.size == 0 || !packet.data || packet.nalus.empty()) {
            return false;
        }

        size_t removed_bytes = 0; // 统计所有参数集总字节长度

        // 第一轮遍历：统计需要删除的参数集总大小
        for (const auto& nalu : packet.nalus) {
            // 跳过非法NALU
            if (nalu.packet_size == 0 || nalu.packet_offset >= packet.size) {
                continue;
            }

            const size_t remaining   = packet.size - nalu.packet_offset;
            const size_t packet_size = std::min(nalu.packet_size, remaining);
            // 识别参数集NALU，累加待删除字节
            if (is_hevc_parameter_set_nalu(nalu.type)) {
                removed_bytes += packet_size;
            }
        }
        // 无参数集，直接返回false，不修改包
        if (removed_bytes == 0) {
            return false;
        }

        // 计算剥离后新码流总长度
        const size_t new_size = packet.size - removed_bytes;
        // 剥离后码流为空，直接清空包内存
        if (new_size == 0) {
            packet.data.reset();
            packet.size     = 0;
            packet.keyframe = false;
            return true;
        }

        // 分配新缓冲区存储剥离后的纯图像码流
        auto stripped       = std::make_unique<uint8_t[]>(new_size);
        size_t read_offset  = 0;  // 原始包读取偏移指针
        size_t write_offset = 0;  // 新缓冲区写入偏移指针

        // 第二轮遍历：拷贝非参数集NALU，跳过VPS/SPS/PPS
        for (const auto& nalu : packet.nalus) {
            // 跳过非法NALU
            if (nalu.packet_size == 0 || nalu.packet_offset >= packet.size) {
                continue;
            }
            // 当前NALU是参数集，直接跳过拷贝
            if (!is_hevc_parameter_set_nalu(nalu.type)) {
                continue;
            }

            const size_t remaining   = packet.size - nalu.packet_offset;
            const size_t packet_size = std::min(nalu.packet_size, remaining);
            // 参数集NALU前存在有效图像数据，拷贝到新缓冲区
            if (read_offset < nalu.packet_offset) {
                const size_t keep_bytes = nalu.packet_offset - read_offset;
                std::copy_n(
                    packet.data.get() + read_offset, keep_bytes, stripped.get() + write_offset);
                write_offset += keep_bytes;
            }
            // 读取指针跳过整段参数集，下一次从参数集末尾继续读取
            read_offset = nalu.packet_offset + packet_size;
        }

        // 拷贝包末尾剩余图像数据（最后一段非参数集码流）
        if (read_offset < packet.size) {
            const size_t keep_bytes = packet.size - read_offset;
            std::copy_n(packet.data.get() + read_offset, keep_bytes, stripped.get() + write_offset);
            write_offset += keep_bytes;
        }

        // 替换包内部裸流缓冲区，更新有效长度，清空NALU元数据（已失效）
        packet.data = std::move(stripped);
        packet.size = write_offset;
        packet.nalus.clear();
        // 成功剥离参数集，返回true
        return true;
    } catch (const std::exception& e) {
        // 捕获内存分配、拷贝等异常，包装错误信息返回
        return std::unexpected(std::string("strip HEVC VPS/SPS/PPS: ") + e.what());
    }
}

/**
 * @brief 仅首次遇到完整参数集IDR帧，导出VPS/SPS/PPS到本地文件，重复调用不重复写入
 * @param packet 编码包，必须是携带完整三套参数集的IDR关键帧
 * @param state 导出状态上下文，标记是否已完成导出
 * @return std::expected<void, string> 成功无返回值；失败携带错误描述
 * noexcept 内部捕获所有标准异常，不对外抛出
 */
std::expected<void, std::string> export_hevc_parameter_sets_once(
    const EncodedPacket& packet, HevcParameterSetExportState& state) noexcept {
    try {
        // 已导出过、空包、无NALU元数据直接返回，不执行导出逻辑
        if (state.exported || packet.size == 0 || !packet.data || packet.nalus.empty()) {
            return {};
        }

        // 提取包内所有参数集
        auto sets = collect_parameter_sets(packet);
        // 缺少任意一套参数集，不导出直接返回
        if (!sets.complete()) {
            return {};
        }

        // 执行本地文件导出
        auto export_result = export_hevc_parameter_sets(sets);
        // 导出文件失败，透传错误信息
        if (!export_result) {
            return std::unexpected(std::move(export_result.error()));
        }
        // 标记已导出，后续所有调用直接跳过导出逻辑
        state.exported = true;
        return {};
    } catch (const std::exception& e) {
        // 捕获内存、文件系统异常，包装错误返回
        return std::unexpected(std::string("export HEVC parameter sets: ") + e.what());
    }
}

} // namespace quanta