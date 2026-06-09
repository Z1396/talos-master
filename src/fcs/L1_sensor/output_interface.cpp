#include "L1_sensor/output_interface.hpp"

#include "chiral/gimbal.hpp"
#include "quanta/stream_transport.hpp"
#include "shm_client.hpp"
#include "talos_gimbal/mcu_device.hpp"
#include "talos_gimbal/packet.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cstring>

namespace fcs::L1 {

namespace {
constexpr double kRadToDeg = 180.0 / M_PI;
} // namespace

// ============================================================================
// IpcOutput 实现
// ============================================================================

IpcOutput::IpcOutput(std::shared_ptr<ipc::ShmClient> client) noexcept
    : client_(std::move(client)) {}

void IpcOutput::send(const L5::WeaponCommand& cmd) const noexcept {
    if (!client_) [[unlikely]] {
        SPDLOG_ERROR("IpcOutput::send: client is null, dropping weapon command");
        return;
    }

    const auto yaw_deg    = static_cast<float>(cmd.yaw * kRadToDeg);
    const auto pitch_deg  = -static_cast<float>(cmd.pitch * kRadToDeg);
    const auto distance_m = static_cast<float>(cmd.distance);

    client_->send_gimbal_cmd(yaw_deg, pitch_deg, distance_m, cmd.fire);
}
void IpcOutput::send_quanta(const quanta::QuantaPacket& pakcet) const noexcept {
    /// No-op
}

// ============================================================================
// McuOutput 实现
// ============================================================================

static_assert(std::is_trivially_copyable_v<talos_gimbal::SendSimpleVisionData>);
static_assert(std::is_trivially_copyable_v<talos_gimbal::SendVisionData>);
static_assert(std::is_trivially_copyable_v<talos_gimbal::SendQuantaData>);

McuOutput::McuOutput(std::shared_ptr<talos_gimbal::McuDeviceHandle> device) noexcept
    : device_(std::move(device)) {}

void McuOutput::send(const L5::WeaponCommand& cmd) const noexcept {
    if (!device_) [[unlikely]] {
        SPDLOG_ERROR("McuOutput::send: device is null, dropping weapon command");
        return;
    }

    if (!device_->is_connected()) [[unlikely]] {
        SPDLOG_ERROR("McuOutput::send: device disconnected, dropping weapon command");
        return;
    }

    if (device_->is_serial()) {
        talos_gimbal::SendSimpleVisionData packet{
            .header =
                {
                         .sof = talos_gimbal::HeaderFrame::SoF(),
                         .len = sizeof(talos_gimbal::SendSimpleVisionData::data),
                         .id  = 0x04,
                         },
            .data =
                {
                         .target_yaw = static_cast<float>(cmd.yaw * kRadToDeg),
                         },
            .eof = talos_gimbal::HeaderFrame::EoF(),
        };
        if (auto result =
                device_->send_sync(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
            !result) {
            SPDLOG_WARN("send simple vision data: {}", result.error());
        }
    } else {
        talos_gimbal::SendVisionData packet{
            .header =
                {
                         .sof = talos_gimbal::HeaderFrame::SoF(),
                         .len = sizeof(talos_gimbal::SendVisionData::data),
                         .id  = 0x02,
                         },
            .data =
                {
                         .fire_advice  = cmd.fire,
                         .target_yaw   = static_cast<float>(cmd.yaw * kRadToDeg),
                         .target_pitch = -static_cast<float>(cmd.pitch * kRadToDeg),
                         .ref_yaw_v    = static_cast<float>(cmd.yaw_vel * kRadToDeg),
                         .ref_pitch_v  = -static_cast<float>(cmd.pitch_vel * kRadToDeg),
                         .ref_yaw_a    = static_cast<float>(cmd.yaw_accel * kRadToDeg),
                         .ref_pitch_a  = -static_cast<float>(cmd.pitch_accel * kRadToDeg),
                         .distance     = static_cast<float>(cmd.distance),
                         },
            .eof = talos_gimbal::HeaderFrame::EoF(),
        };
        if (auto result =
                device_->send_sync(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
            !result) {
            SPDLOG_WARN("send vision data: {}", result.error());
        }
    }
}

void McuOutput::send_quanta(const quanta::QuantaPacket& packet) const noexcept {
    if (!device_) [[unlikely]] {
        return;
    }
    if (!device_->is_connected()) [[unlikely]] {
        return;
    }
    if (device_->is_serial()) {
        return;
    }
    // evil hack for extended data length
    constexpr auto data_length_raw = sizeof(talos_gimbal::SendQuantaData::data);
    // maximum extended data length is stored as u12
    static_assert(data_length_raw < 4096);
    // lower bits
    // higher bits, 4bits available
    constexpr uint8_t data_length_hi = data_length_raw >> 8;
    // double check for only 4bits
    static_assert((data_length_hi & 0b1111) == data_length_hi);
    constexpr uint8_t data_length_protocol = data_length_raw & 0b11111111;
    // actual id is 0x04
    constexpr uint8_t data_id_protocol = (data_length_hi << 4) | 0x04;

    constexpr std::size_t kMaxCustomBlockBytes =
        sizeof(std::declval<talos_gimbal::SendQuantaData>().data.custom_byte_block);

    const auto bytes = packet.bytes();
    if (bytes.empty() || bytes.size() > kMaxCustomBlockBytes) {
        return;
    }

    static uint32_t s = 0;
    talos_gimbal::SendQuantaData out{
        .header =
            {
                     .sof = talos_gimbal::HeaderFrame::SoF(),
                     .len = data_length_protocol,
                     .id  = data_id_protocol,
                     },
        .time_stamp = static_cast<uint32_t>(s++),
        .data =
            {
                     .custom_byte_block_len = static_cast<uint16_t>(bytes.size()),
                     .custom_byte_block     = {},
                     },
        .eof = talos_gimbal::HeaderFrame::EoF(),
    };

    std::memcpy(out.data.custom_byte_block, bytes.data(), bytes.size());
    SPDLOG_DEBUG("send_quanta: bytes.size() = {}, seq = {}", bytes.size(), packet.seq);
    device_->send_sync(reinterpret_cast<uint8_t*>(&out), sizeof(out));
}

// ============================================================================
// ChiralOutput 实现
// ============================================================================

ChiralOutput::ChiralOutput(std::shared_ptr<talos::chiral::gimbal::TalosEndpoint> device) noexcept
    : device_(std::move(device)) {}

void ChiralOutput::send(const L5::WeaponCommand& cmd) const noexcept {
    if (!device_) [[unlikely]] {
        SPDLOG_ERROR("ChiralOutput::send: device is null, dropping weapon command");
        return;
    }

    device_->write(
        talos::chiral::gimbal::McuRequest{
            .timestamp_ns_system_clock = static_cast<int64_t>(cmd.timestamp_ns),
            .fire_advice               = cmd.fire,
            .target_yaw                = static_cast<float>(cmd.yaw * kRadToDeg),
            .target_pitch              = -static_cast<float>(cmd.pitch * kRadToDeg),
            .ref_yaw_v                 = static_cast<float>(cmd.yaw_vel * kRadToDeg),
            .ref_pitch_v               = -static_cast<float>(cmd.pitch_vel * kRadToDeg),
            .ref_yaw_a                 = static_cast<float>(cmd.yaw_accel * kRadToDeg),
            .ref_pitch_a               = -static_cast<float>(cmd.pitch_accel * kRadToDeg),
            .distance                  = static_cast<float>(cmd.distance),
        });
}

void ChiralOutput::send_quanta(const quanta::QuantaPacket& packet) const noexcept {
    (void)packet;
    /// No-op
}

// ============================================================================
// OutputInterface 实现
// ============================================================================

OutputInterface::OutputInterface(OutputMode mode) noexcept
    : mode_(std::move(mode)) {}

void OutputInterface::send(const L5::WeaponCommand& cmd) noexcept {
    std::visit([&cmd](auto& output) { output.send(cmd); }, mode_);
}

// ============================================================================
// OutputInterface 工厂方法
// ============================================================================

auto OutputInterface::create_ipc(std::shared_ptr<ipc::ShmClient> client) noexcept
    -> std::expected<OutputInterface, std::string> {
    return OutputInterface(IpcOutput(std::move(client)));
}

auto OutputInterface::create_chiral(
    std::shared_ptr<talos::chiral::gimbal::TalosEndpoint> client) noexcept
    -> std::expected<OutputInterface, std::string> {
    return OutputInterface(ChiralOutput(std::move(client)));
}

auto OutputInterface::create_usb(
    const uint16_t vendor_id, const std::optional<uint16_t> product_id) noexcept
    -> std::expected<OutputInterface, std::string> {
    auto device_result = talos_gimbal::McuDeviceHandle::create_usb(vendor_id, product_id);
    if (!device_result) {
        return std::unexpected(std::move(device_result).error());
    }
    return OutputInterface(McuOutput(
        std::make_shared<talos_gimbal::McuDeviceHandle>(std::move(device_result).value())));
}

auto OutputInterface::create_serial(const std::string& device_path, int baud_rate) noexcept
    -> std::expected<OutputInterface, std::string> {
    auto device_result = talos_gimbal::McuDeviceHandle::create_serial(device_path, baud_rate);
    if (!device_result) {
        return std::unexpected(std::move(device_result).error());
    }
    return OutputInterface(McuOutput(
        std::make_shared<talos_gimbal::McuDeviceHandle>(std::move(device_result).value())));
}

} // namespace fcs::L1
