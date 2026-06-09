#pragma once

#include "export.hpp"

// ---------------------------------------------------------------------------
// RAII wrappers for Axera VENC (Video Encoder) hardware module
// ---------------------------------------------------------------------------
// AxSysModule: shared, reference-counted AX_SYS_Init / AX_SYS_Deinit.
//   This is the process-wide AX system lifetime token used by Quanta AX modules
//   and FCS Axera backends. The refcount state lives in libquanta so all users
//   share one SYS lifetime.
//
// VencModule: shared, reference-counted AX_VENC_Init / AX_VENC_Deinit.
//   The hardware module is a global resource — multiple encoder instances
//   (channels) share one module. The last channel to drop its module handle
//   triggers deinit automatically.
//
// VencChannel: exclusive owner of one VENC_CHN. CreateChn on construction,
//   DestroyChn on destruction. RAII guarantees the channel is always
//   cleaned up, even if encoding fails.
// ---------------------------------------------------------------------------

#include <atomic>
#include <cstdint>
#include <expected>
#include <memory>
#include <string>

extern "C" {
#include "ax_base_type.h"
#include "ax_ive_api.h"
#include "ax_ivps_api.h"
#include "ax_venc_api.h"
}

namespace quanta {

// ---------------------------------------------------------------------------
// RAII AX SYS module handle (reference-counted across libquanta users)
// ---------------------------------------------------------------------------
class QUANTA_API AxSysModule {
public:
    /// Acquire a shared reference to AX SYS.
    /// Initializes SYS on first call; subsequent calls bump the refcount.
    [[nodiscard]] static std::expected<std::shared_ptr<AxSysModule>, std::string>
        acquire() noexcept;

    ~AxSysModule() noexcept;

    AxSysModule(const AxSysModule&)            = delete;
    AxSysModule& operator=(const AxSysModule&) = delete;
    AxSysModule(AxSysModule&&)                 = delete;
    AxSysModule& operator=(AxSysModule&&)      = delete;

private:
    AxSysModule() noexcept = default;
};

// ---------------------------------------------------------------------------
// RAII VENC module handle (reference-counted)
// ---------------------------------------------------------------------------
class VencModule {
public:
    /// Acquire a shared reference to the VENC module.
    /// Initializes the module on first call; subsequent calls bump the refcount.
    [[nodiscard]] static std::expected<std::shared_ptr<VencModule>, std::string>
        acquire(AX_VENC_ENCODER_TYPE_E encoder_type = AX_VENC_VIDEO_ENCODER) noexcept;

    ~VencModule();

    VencModule(const VencModule&)            = delete;
    VencModule& operator=(const VencModule&) = delete;
    VencModule(VencModule&&)                 = delete;
    VencModule& operator=(VencModule&&)      = delete;

    [[nodiscard]] AX_VENC_ENCODER_TYPE_E encoder_type() const noexcept { return encoder_type_; }

private:
    explicit VencModule(
        std::shared_ptr<AxSysModule> ax_sys_module, AX_VENC_ENCODER_TYPE_E encoder_type) noexcept;

    std::shared_ptr<AxSysModule> ax_sys_module_;
    AX_VENC_ENCODER_TYPE_E encoder_type_;
    static std::atomic<int> refcount_;
};

// ---------------------------------------------------------------------------
// RAII IVPS module handle (reference-counted)
// ---------------------------------------------------------------------------
class IvpsModule {
public:
    [[nodiscard]] static std::expected<std::shared_ptr<IvpsModule>, std::string> acquire() noexcept;

    ~IvpsModule();

    IvpsModule(const IvpsModule&)            = delete;
    IvpsModule& operator=(const IvpsModule&) = delete;
    IvpsModule(IvpsModule&&)                 = delete;
    IvpsModule& operator=(IvpsModule&&)      = delete;

private:
    explicit IvpsModule(std::shared_ptr<AxSysModule> ax_sys_module) noexcept;

    std::shared_ptr<AxSysModule> ax_sys_module_;
    static std::atomic<int> refcount_;
};

// ---------------------------------------------------------------------------
// RAII IVE module handle (reference-counted)
// ---------------------------------------------------------------------------
class IveModule {
public:
    [[nodiscard]] static std::expected<std::shared_ptr<IveModule>, std::string> acquire() noexcept;

    ~IveModule();

    IveModule(const IveModule&)            = delete;
    IveModule& operator=(const IveModule&) = delete;
    IveModule(IveModule&&)                 = delete;
    IveModule& operator=(IveModule&&)      = delete;

private:
    explicit IveModule(std::shared_ptr<AxSysModule> ax_sys_module) noexcept;

    std::shared_ptr<AxSysModule> ax_sys_module_;
    static std::atomic<int> refcount_;
};

// ---------------------------------------------------------------------------
// RAII VENC channel (exclusive ownership)
// ---------------------------------------------------------------------------
class VencChannel {
public:
    /// Create a VENC channel for HEVC encoding.
    /// The module handle keeps the VENC hardware initialized for the channel's lifetime.
    [[nodiscard]] static std::expected<VencChannel, std::string>
        create(std::shared_ptr<VencModule> module, const AX_VENC_CHN_ATTR_T& chn_attr) noexcept;

    ~VencChannel();

    VencChannel(const VencChannel&)            = delete;
    VencChannel& operator=(const VencChannel&) = delete;

    VencChannel(VencChannel&& other) noexcept;
    VencChannel& operator=(VencChannel&& other) noexcept;

    [[nodiscard]] VENC_CHN id() const noexcept { return chn_; }

    /// Send a frame to the encoder. Returns 0 on success, AX error code on failure.
    [[nodiscard]] AX_S32 send_frame(const AX_VIDEO_FRAME_INFO_T& frame, AX_S32 timeout_ms) noexcept;

    /// Send a frame with per-frame user RC metadata. Returns 0 on success, AX error code on
    /// failure.
    [[nodiscard]] AX_S32
        send_frame_ex(const AX_USER_FRAME_INFO_T& frame, AX_S32 timeout_ms) noexcept;

    /// Get encoded stream. Returns 0 on success, AX error code on timeout/failure.
    [[nodiscard]] AX_S32 get_stream(AX_VENC_STREAM_T& stream, AX_S32 timeout_ms) noexcept;

    /// Release a previously acquired stream buffer.
    [[nodiscard]] AX_S32 release_stream(const AX_VENC_STREAM_T& stream) noexcept;

    /// Start receiving frames (must be called before send_frame).
    [[nodiscard]] AX_S32 start_recv_frame(const AX_VENC_RECV_PIC_PARAM_T& param) noexcept;

    /// Stop receiving frames.
    [[nodiscard]] AX_S32 stop_recv_frame() noexcept;

    /// Request an IDR (instantaneous decoder refresh) frame.
    [[nodiscard]] AX_S32 request_idr(bool instant = true) noexcept;

    /// Get RC (rate control) parameters.
    std::expected<AX_VENC_RC_PARAM_T, std::string> get_rc_param() noexcept;
    /// Set RC (rate control) parameters dynamically.
    [[nodiscard]] AX_S32 set_rc_param(const AX_VENC_RC_PARAM_T& rc_param) noexcept;

    /// Set VUI parameters.
    [[nodiscard]] AX_S32 set_vui_param(const AX_VENC_VUI_PARAM_T& vui_param) noexcept;

    /// Set intra refresh configuration.
    [[nodiscard]] AX_S32 set_intra_refresh(const AX_VENC_INTRA_REFRESH_T& cfg) noexcept;

    /// Set ROI attributes.
    [[nodiscard]] AX_S32 set_roi_attr(const AX_VENC_ROI_ATTR_T& roi) noexcept;

    /// Query channel status (pending frames, buffer occupancy).
    [[nodiscard]] AX_S32 query_status(AX_VENC_CHN_STATUS_T& status) noexcept;

private:
    explicit VencChannel(std::shared_ptr<VencModule> module, VENC_CHN chn);

    void destroy() noexcept;

    std::shared_ptr<VencModule> module_;
    VENC_CHN chn_ = -1;
    bool active_  = false;
};

[[nodiscard]] AX_IVPS_ASPECT_RATIO_T make_stretch_aspect_ratio() noexcept;
[[nodiscard]] AX_S32 ivps_crop_resize(
    const AX_VIDEO_FRAME_T* src, AX_VIDEO_FRAME_T* dst,
    const AX_IVPS_ASPECT_RATIO_T* aspect_ratio) noexcept;
[[nodiscard]] AX_S32 ivps_csc(const AX_VIDEO_FRAME_T* src, AX_VIDEO_FRAME_T* dst) noexcept;

} // namespace quanta
