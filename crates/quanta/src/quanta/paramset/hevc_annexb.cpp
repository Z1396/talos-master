#include "hevc_annexb.hpp"

#include <algorithm>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>

namespace quanta {
namespace {

constexpr uint8_t kHevcNaluIdrWRadl = 19;
constexpr uint8_t kHevcNaluIdrNLp   = 20;
constexpr uint8_t kHevcNaluVps      = 32;
constexpr uint8_t kHevcNaluSps      = 33;
constexpr uint8_t kHevcNaluPps      = 34;

constexpr uint8_t kStartCode4[] = {0x00, 0x00, 0x00, 0x01};

struct HevcParameterSets {
    std::vector<uint8_t> vps;
    std::vector<uint8_t> sps;
    std::vector<uint8_t> pps;

    [[nodiscard]] bool complete() const noexcept {
        return !vps.empty() && !sps.empty() && !pps.empty();
    }
};

void assign_parameter_set(
    std::vector<uint8_t>& dst, const EncodedPacket& packet, const EncodedPacketNalu& nalu) {
    const size_t remaining   = packet.size - nalu.packet_offset;
    const size_t packet_size = std::min(nalu.packet_size, remaining);
    const uint8_t* data      = packet.data.get() + nalu.packet_offset;
    dst.assign(data, data + packet_size);
}

[[nodiscard]] HevcParameterSets collect_parameter_sets(const EncodedPacket& packet) {
    HevcParameterSets sets;
    for (const auto& nalu : packet.nalus) {
        if (nalu.packet_size == 0 || nalu.packet_offset >= packet.size) {
            continue;
        }

        switch (nalu.type) {
        case kHevcNaluVps: assign_parameter_set(sets.vps, packet, nalu); break;
        case kHevcNaluSps: assign_parameter_set(sets.sps, packet, nalu); break;
        case kHevcNaluPps: assign_parameter_set(sets.pps, packet, nalu); break;
        default: break;
        }
    }
    return sets;
}

[[nodiscard]] size_t annexb_parameter_sets_size(const HevcParameterSets& sets) noexcept {
    return (sizeof(kStartCode4) + sets.vps.size()) + (sizeof(kStartCode4) + sets.sps.size())
         + (sizeof(kStartCode4) + sets.pps.size());
}

void append_parameter_set(const std::vector<uint8_t>& nalu, std::vector<uint8_t>& dst) {
    dst.insert(dst.end(), kStartCode4, kStartCode4 + sizeof(kStartCode4));
    dst.insert(dst.end(), nalu.begin(), nalu.end());
}

[[nodiscard]] std::expected<std::string, std::string>
    export_hevc_parameter_sets(const HevcParameterSets& sets) noexcept {
    if (!sets.complete()) {
        return std::unexpected("HEVC parameter sets are incomplete");
    }

    try {
        const auto output_dir = std::filesystem::current_path() / "output";
        std::filesystem::create_directories(output_dir);

        const auto path = output_dir / "quanta_vps_sps_pps.hevc";
        std::vector<uint8_t> bytes;
        bytes.reserve(annexb_parameter_sets_size(sets));
        append_parameter_set(sets.vps, bytes);
        append_parameter_set(sets.sps, bytes);
        append_parameter_set(sets.pps, bytes);

        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out) {
            return std::unexpected("open HEVC parameter set output file failed");
        }
        out.write(
            reinterpret_cast<const char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
        if (!out) {
            return std::unexpected("write HEVC parameter set output file failed");
        }

        return path.string();
    } catch (const std::exception& e) {
        return std::unexpected(std::string("export HEVC parameter sets: ") + e.what());
    }
}

} // namespace

bool is_hevc_idr_nalu(uint8_t type) noexcept {
    return type == kHevcNaluIdrWRadl || type == kHevcNaluIdrNLp;
}

bool is_hevc_parameter_set_nalu(uint8_t type) noexcept {
    return type == kHevcNaluVps || type == kHevcNaluSps || type == kHevcNaluPps;
}

std::expected<bool, std::string> strip_hevc_parameter_sets(EncodedPacket& packet) noexcept {
    try {
        if (packet.size == 0 || !packet.data || packet.nalus.empty()) {
            return false;
        }

        size_t removed_bytes = 0;

        for (const auto& nalu : packet.nalus) {
            if (nalu.packet_size == 0 || nalu.packet_offset >= packet.size) {
                continue;
            }

            const size_t remaining   = packet.size - nalu.packet_offset;
            const size_t packet_size = std::min(nalu.packet_size, remaining);
            if (is_hevc_parameter_set_nalu(nalu.type)) {
                removed_bytes += packet_size;
            }
        }
        if (removed_bytes == 0) {
            return false;
        }

        const size_t new_size = packet.size - removed_bytes;
        if (new_size == 0) {
            packet.data.reset();
            packet.size     = 0;
            packet.keyframe = false;
            return true;
        }

        auto stripped       = std::make_unique<uint8_t[]>(new_size);
        size_t read_offset  = 0;
        size_t write_offset = 0;

        for (const auto& nalu : packet.nalus) {
            if (nalu.packet_size == 0 || nalu.packet_offset >= packet.size) {
                continue;
            }
            if (!is_hevc_parameter_set_nalu(nalu.type)) {
                continue;
            }

            const size_t remaining   = packet.size - nalu.packet_offset;
            const size_t packet_size = std::min(nalu.packet_size, remaining);
            if (read_offset < nalu.packet_offset) {
                const size_t keep_bytes = nalu.packet_offset - read_offset;
                std::copy_n(
                    packet.data.get() + read_offset, keep_bytes, stripped.get() + write_offset);
                write_offset += keep_bytes;
            }
            read_offset = nalu.packet_offset + packet_size;
        }

        if (read_offset < packet.size) {
            const size_t keep_bytes = packet.size - read_offset;
            std::copy_n(packet.data.get() + read_offset, keep_bytes, stripped.get() + write_offset);
            write_offset += keep_bytes;
        }

        packet.data = std::move(stripped);
        packet.size = write_offset;
        packet.nalus.clear();
        return true;
    } catch (const std::exception& e) {
        return std::unexpected(std::string("strip HEVC VPS/SPS/PPS: ") + e.what());
    }
}

std::expected<void, std::string> export_hevc_parameter_sets_once(
    const EncodedPacket& packet, HevcParameterSetExportState& state) noexcept {
    try {
        if (state.exported || packet.size == 0 || !packet.data || packet.nalus.empty()) {
            return {};
        }

        auto sets = collect_parameter_sets(packet);
        if (!sets.complete()) {
            return {};
        }

        auto export_result = export_hevc_parameter_sets(sets);
        if (!export_result) {
            return std::unexpected(std::move(export_result.error()));
        }
        state.exported = true;
        return {};
    } catch (const std::exception& e) {
        return std::unexpected(std::string("export HEVC parameter sets: ") + e.what());
    }
}

} // namespace quanta
