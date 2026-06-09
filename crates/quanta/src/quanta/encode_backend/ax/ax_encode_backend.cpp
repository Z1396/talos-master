#include "quanta/encode_backend/ax/ax_encode_backend.hpp"
#include "quanta/encode_backend/ax/ax_venc_raii.hpp"
#include "quanta/encode_backend/encoder_backend.hpp"
#include "quanta/paramset/hevc_annexb.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <ax_base_type.h>
#include <ax_venc_api.h>
#include <ax_venc_comm.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <exception>
#include <expected>
#include <format>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <spdlog/spdlog.h>

extern "C" {
#include "ax_global_type.h"
#include "ax_ive_api.h"
#include "ax_ive_type.h"
#include "ax_ivps_api.h"
#include "ax_ivps_type.h"
#include "ax_venc_rc.h"
}

namespace quanta {

// ===========================================================================
// Axera error code decoder
// ===========================================================================

namespace {
constexpr double kMaxSharpenAmount = 6.0;

[[nodiscard]] const char* ax_module_name(int id) noexcept {
    switch (id) {
    case 0x01: return "AX_ID_ISP";
    case 0x02: return "AX_ID_CE";
    case 0x03: return "AX_ID_VO";
    case 0x04: return "AX_ID_VDSP";
    case 0x06: return "AX_ID_NPU";
    case 0x07: return "AX_ID_VENC";
    case 0x08: return "AX_ID_VDEC";
    case 0x09: return "AX_ID_JENC";
    case 0x0A: return "AX_ID_JDEC";
    case 0x0B: return "AX_ID_SYS";
    case 0x0D: return "AX_ID_IVPS";
    case 0x12: return "AX_ID_USER";
    case 0x15: return "AX_ID_IVE";
    default: return "AX_ID_?";
    }
}

[[nodiscard]] const char* ax_err_name(int id) noexcept {
    switch (id) {
    case 0x01: return "AX_ERR_INVALID_MODID";
    case 0x02: return "AX_ERR_INVALID_DEVID";
    case 0x03: return "AX_ERR_INVALID_GRPID";
    case 0x04: return "AX_ERR_INVALID_CHNID";
    case 0x05: return "AX_ERR_INVALID_PIPEID";
    case 0x0A: return "AX_ERR_ILLEGAL_PARAM";
    case 0x0B: return "AX_ERR_NULL_PTR";
    case 0x0C: return "AX_ERR_BAD_ADDR";
    case 0x10: return "AX_ERR_SYS_NOTREADY";
    case 0x11: return "AX_ERR_BUSY";
    case 0x12: return "AX_ERR_NOT_INIT";
    case 0x13: return "AX_ERR_NOT_CONFIG";
    case 0x14: return "AX_ERR_NOT_SUPPORT";
    case 0x15: return "AX_ERR_NOT_PERM";
    case 0x16: return "AX_ERR_EXIST";
    case 0x17: return "AX_ERR_UNEXIST";
    case 0x18: return "AX_ERR_NOMEM";
    case 0x19: return "AX_ERR_NOBUF";
    case 0x1A: return "AX_ERR_NOT_MATCH";
    case 0x20: return "AX_ERR_BUF_EMPTY";
    case 0x21: return "AX_ERR_BUF_FULL";
    case 0x22: return "AX_ERR_QUEUE_EMPTY";
    case 0x23: return "AX_ERR_QUEUE_FULL";
    case 0x27: return "AX_ERR_TIMED_OUT";
    case 0x28: return "AX_ERR_FLOW_END";
    case 0x29: return "AX_ERR_UNKNOWN";
    case 0x50: return "AX_ERR_IVE_OPEN_FAILED";
    case 0x51: return "AX_ERR_IVE_INIT_FAILED";
    case 0x52: return "AX_ERR_IVE_NOT_INIT";
    case 0x53: return "AX_ERR_IVE_SYS_TIMEOUT";
    case 0x54: return "AX_ERR_IVE_QUERY_TIMEOUT";
    case 0x55: return "AX_ERR_IVE_BUS_ERR";
    default: return "AX_ERR_?";
    }
}

} // namespace

std::string ax_error_string(AX_S32 err) noexcept {
    if (err >= 0)
        return std::format("OK(0x{:X})", static_cast<unsigned>(err));

    const unsigned code       = static_cast<unsigned>(err);
    const unsigned module     = (code >> 16) & 0xFF;
    const unsigned sub_module = (code >> 8) & 0xFF;
    const unsigned err_id     = code & 0xFF;

    return std::format(
        "{}/{} {}(0x{:02X}) [raw=0x{:08X}]", ax_module_name(static_cast<int>(module)), sub_module,
        ax_err_name(static_cast<int>(err_id)), err_id, code);
}

// ===========================================================================
// AxSysModule RAII
// ===========================================================================

namespace {
std::mutex g_ax_sys_mutex;
int g_ax_sys_refcount = 0;
} // namespace

std::expected<std::shared_ptr<AxSysModule>, std::string> AxSysModule::acquire() noexcept {
    std::scoped_lock lock(g_ax_sys_mutex);
    if (g_ax_sys_refcount == 0) {
        const AX_S32 ret = AX_SYS_Init();
        if (ret != AX_SUCCESS) {
            return std::unexpected(std::format("AX_SYS_Init failed: {}", ax_error_string(ret)));
        }
    }

    ++g_ax_sys_refcount;
    return std::shared_ptr<AxSysModule>(new AxSysModule(), [](AxSysModule* p) { delete p; });
}

AxSysModule::~AxSysModule() noexcept {
    std::scoped_lock lock(g_ax_sys_mutex);
    if (g_ax_sys_refcount <= 0)
        return;

    --g_ax_sys_refcount;
    if (g_ax_sys_refcount == 0) {
        AX_SYS_Deinit();
    }
}

// ===========================================================================
// VencModule RAII
// ===========================================================================

std::atomic<int> VencModule::refcount_{0};

VencModule::VencModule(
    std::shared_ptr<AxSysModule> ax_sys_module, AX_VENC_ENCODER_TYPE_E encoder_type) noexcept
    : ax_sys_module_(std::move(ax_sys_module))
    , encoder_type_(encoder_type) {}

std::expected<std::shared_ptr<VencModule>, std::string>
    VencModule::acquire(AX_VENC_ENCODER_TYPE_E encoder_type) noexcept {
    auto ax_sys_module = AxSysModule::acquire();
    if (!ax_sys_module)
        return std::unexpected(std::move(ax_sys_module.error()));

    int prev = refcount_.fetch_add(1);
    if (prev == 0) {
        AX_VENC_MOD_ATTR_T mod_attr{};
        mod_attr.enVencType                     = encoder_type;
        mod_attr.stModThdAttr.bExplicitSched    = AX_FALSE;
        mod_attr.stModThdAttr.u32TotalThreadNum = 1;

        AX_S32 ret = AX_VENC_Init(&mod_attr);
        if (ret != AX_SUCCESS) {
            refcount_.fetch_sub(1);
            return std::unexpected(std::format("AX_VENC_Init failed: {}", ax_error_string(ret)));
        }
    }
    return std::shared_ptr<VencModule>(
        new VencModule(std::move(*ax_sys_module), encoder_type), [](VencModule* p) { delete p; });
}

VencModule::~VencModule() {
    int prev = refcount_.fetch_sub(1);
    if (prev == 1) {
        AX_VENC_Deinit();
    }
}

// ===========================================================================
// IvpsModule RAII and IVPS filter dispatch
// ===========================================================================

std::atomic<int> IvpsModule::refcount_{0};

IvpsModule::IvpsModule(std::shared_ptr<AxSysModule> ax_sys_module) noexcept
    : ax_sys_module_(std::move(ax_sys_module)) {}

std::expected<std::shared_ptr<IvpsModule>, std::string> IvpsModule::acquire() noexcept {
    auto ax_sys_module = AxSysModule::acquire();
    if (!ax_sys_module)
        return std::unexpected(std::move(ax_sys_module.error()));

    int prev = refcount_.fetch_add(1);
    if (prev == 0) {
        AX_S32 ret = AX_IVPS_Init();
        if (ret != AX_SUCCESS) {
            refcount_.fetch_sub(1);
            return std::unexpected(std::format("AX_IVPS_Init failed: {}", ax_error_string(ret)));
        }
    }

    return std::shared_ptr<IvpsModule>(
        new IvpsModule(std::move(*ax_sys_module)), [](IvpsModule* p) { delete p; });
}

IvpsModule::~IvpsModule() {
    int prev = refcount_.fetch_sub(1);
    if (prev == 1) {
        AX_IVPS_Deinit();
    }
}

// ===========================================================================
// IveModule RAII
// ===========================================================================

std::atomic<int> IveModule::refcount_{0};

IveModule::IveModule(std::shared_ptr<AxSysModule> ax_sys_module) noexcept
    : ax_sys_module_(std::move(ax_sys_module)) {}

std::expected<std::shared_ptr<IveModule>, std::string> IveModule::acquire() noexcept {
    auto ax_sys_module = AxSysModule::acquire();
    if (!ax_sys_module)
        return std::unexpected(std::move(ax_sys_module.error()));

    int prev = refcount_.fetch_add(1);
    if (prev == 0) {
        AX_S32 ret = AX_IVE_Init();
        if (ret != AX_SUCCESS) {
            refcount_.fetch_sub(1);
            return std::unexpected(std::format("AX_IVE_Init failed: {}", ax_error_string(ret)));
        }
    }

    return std::shared_ptr<IveModule>(
        new IveModule(std::move(*ax_sys_module)), [](IveModule* p) { delete p; });
}

IveModule::~IveModule() {
    int prev = refcount_.fetch_sub(1);
    if (prev == 1) {
        AX_IVE_Exit();
    }
}

AX_IVPS_ASPECT_RATIO_T make_stretch_aspect_ratio() noexcept {
    AX_IVPS_ASPECT_RATIO_T aspect_ratio{};
    aspect_ratio.eMode      = AX_IVPS_ASPECT_RATIO_STRETCH;
    aspect_ratio.eAligns[0] = AX_IVPS_ASPECT_RATIO_HORIZONTAL_CENTER;
    aspect_ratio.eAligns[1] = AX_IVPS_ASPECT_RATIO_VERTICAL_CENTER;
    aspect_ratio.nBgColor   = 0;
    return aspect_ratio;
}

AX_S32 ivps_crop_resize(
    const AX_VIDEO_FRAME_T* src, AX_VIDEO_FRAME_T* dst,
    const AX_IVPS_ASPECT_RATIO_T* aspect_ratio) noexcept {
    AX_S32 ret = AX_IVPS_CropResizeVpp(src, dst, aspect_ratio);
    if (ret == AX_SUCCESS)
        return ret;
    ret = AX_IVPS_CropResizeVgp(src, dst, aspect_ratio);
    if (ret == AX_SUCCESS)
        return ret;
    return AX_IVPS_CropResizeTdp(src, dst, aspect_ratio);
}

AX_S32 ivps_csc(const AX_VIDEO_FRAME_T* src, AX_VIDEO_FRAME_T* dst) noexcept {
    AX_S32 ret = AX_IVPS_CscVpp(src, dst);
    if (ret == AX_SUCCESS)
        return ret;
    ret = AX_IVPS_CscVgp(src, dst);
    if (ret == AX_SUCCESS)
        return ret;
    return AX_IVPS_CscTdp(src, dst);
}

namespace {

[[nodiscard]] AX_U32 align_up_u32(AX_U32 value, AX_U32 alignment) noexcept {
    if (alignment == 0U)
        return value;
    return ((value + alignment - 1U) / alignment) * alignment;
}

[[nodiscard]] AX_U32 bgr888_stride_aligned(int width) noexcept {
    // Same convention as ax-video-sdk: RGB/BGR24 uses 16-pixel alignment,
    // i.e. 16 * 3 = 48 bytes, so the packed stride remains divisible by 3.
    constexpr AX_U32 kBgrAlign = 48U;
    return align_up_u32(static_cast<AX_U32>(std::max(width, 0)) * 3U, kBgrAlign);
}

[[nodiscard]] bool filter_value_enabled(double value) noexcept {
    constexpr double kEpsilon = 1e-6;
    return std::abs(value) > kEpsilon;
}

[[nodiscard]] bool positive_filter_enabled(double value) noexcept {
    constexpr double kEpsilon = 1e-6;
    return value > kEpsilon;
}

[[nodiscard]] bool denoise_luma_enabled(const FilterParams& filter) noexcept {
    return filter.enable && filter.enable_denoise_luma && filter.denoise_luma.kernel_size > 1;
}

[[nodiscard]] bool denoise_chroma_enabled(const FilterParams& filter) noexcept {
    return filter.enable && filter.enable_denoise_chroma && filter.denoise_chroma.kernel_size > 1;
}

[[nodiscard]] double luma_sharpen_amount(const FilterParams& filter) noexcept {
    if (!filter.enable)
        return 0.0;
    return (filter.enable_sharpen1_luma ? std::max(0.0, filter.sharpen1_luma) : 0.0)
         + (filter.enable_sharpen2_luma ? std::max(0.0, filter.sharpen2_luma) : 0.0)
         + (filter.enable_sharpen3_luma ? std::max(0.0, filter.sharpen3_luma) : 0.0);
}

[[nodiscard]] double chroma_sharpen_amount(const FilterParams& filter) noexcept {
    if (!filter.enable)
        return 0.0;
    return (filter.enable_sharpen1_chroma ? std::max(0.0, filter.sharpen1_chroma) : 0.0)
         + (filter.enable_sharpen2_chroma ? std::max(0.0, filter.sharpen2_chroma) : 0.0)
         + (filter.enable_sharpen3_chroma ? std::max(0.0, filter.sharpen3_chroma) : 0.0);
}

[[nodiscard]] bool luma_quantization_enabled(const FilterParams& filter) noexcept {
    return filter.enable && filter.enable_luma_quantization && filter.luma_levels > 0;
}

[[nodiscard]] bool chroma_quantization_enabled(const FilterParams& filter) noexcept {
    return filter.enable && filter.enable_chroma_quantization && filter.chroma_levels > 0;
}

[[nodiscard]] bool contour_enabled(const FilterParams& filter) noexcept {
    return filter.enable && filter.enable_contour
        && positive_filter_enabled(filter.contour_strength);
}

[[nodiscard]] bool qp_delta_map_enabled(const FilterParams& filter) noexcept {
    return filter.enable && filter.qp_delta_map;
}

[[nodiscard]] bool has_ive_image_filters(const FilterParams& filter) noexcept {
    return denoise_luma_enabled(filter) || denoise_chroma_enabled(filter)
        || filter_value_enabled(luma_sharpen_amount(filter)) || luma_quantization_enabled(filter)
        || contour_enabled(filter) || qp_delta_map_enabled(filter);
}

[[nodiscard]] bool has_chroma_plane_filters(const FilterParams& filter) noexcept {
    return filter_value_enabled(chroma_sharpen_amount(filter))
        || chroma_quantization_enabled(filter);
}

[[nodiscard]] int clamped_luma_levels(int levels) noexcept {
    if (levels <= 0)
        return 0;
    return std::clamp(levels, 2, 16);
}

[[nodiscard]] int clamped_chroma_levels(int levels) noexcept {
    if (levels <= 0)
        return 0;
    return std::clamp(levels, 2, 16);
}

[[nodiscard]] uint8_t clamp_u8(int value) noexcept {
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

[[nodiscard]] uint8_t clamp_u8(double value) noexcept {
    return clamp_u8(static_cast<int>(std::lround(value)));
}

[[nodiscard]] int align_down_int(int value, int alignment) noexcept {
    if (alignment <= 0)
        return value;
    return (value / alignment) * alignment;
}

[[nodiscard]] std::expected<std::pair<int, int>, std::string>
    ive_filter_dimensions(int width, int height) noexcept {
    // AX IVE Filter is documented as 64x64..1920x1024 with 16-pixel-aligned
    // stride. Deployed AX650 firmware also times out on non-16-aligned effective
    // dimensions for small NV12 frames such as 200x150, so make the geometry
    // explicit before IVPS writes the frame.
    constexpr int kIveFilterAlignment = 16;
    constexpr int kIveFilterMinSize   = 64;
    constexpr int kIveFilterMaxWidth  = 1920;
    constexpr int kIveFilterMaxHeight = 1024;

    const int aligned_width =
        align_down_int(std::min(width, kIveFilterMaxWidth), kIveFilterAlignment);
    const int aligned_height =
        align_down_int(std::min(height, kIveFilterMaxHeight), kIveFilterAlignment);

    if (aligned_width < kIveFilterMinSize || aligned_height < kIveFilterMinSize) {
        return std::unexpected(
            std::format(
                "AX IVE filters require output dimensions at least {}x{} after {}-pixel "
                "alignment, got {}x{} -> {}x{}",
                kIveFilterMinSize, kIveFilterMinSize, kIveFilterAlignment, width, height,
                aligned_width, aligned_height));
    }

    return std::pair{aligned_width, aligned_height};
}

[[nodiscard]] int edge_threshold(const FilterParams& filter) noexcept {
    const int low  = std::clamp(filter.contour_low_thresh, 1, 254);
    const int high = std::clamp(filter.contour_high_thresh, low + 1, 255);
    return std::clamp((low + high) / 2, 1, 254);
}

[[nodiscard]] std::mutex& ive_submit_mutex() noexcept {
    static std::mutex mutex;
    return mutex;
}

[[nodiscard]] std::expected<void, std::string> check_ive(AX_S32 ret, std::string_view op) noexcept {
    if (ret != AX_SUCCESS) {
        return std::unexpected(std::format("{} failed: {}", op, ax_error_string(ret)));
    }

    return {};
}

[[nodiscard]] int normalize_gaussian_kernel_size(int kernel_size, int max_kernel_size) noexcept {
    const int max_odd = (max_kernel_size % 2 == 0) ? (max_kernel_size - 1) : max_kernel_size;
    int clamped       = std::clamp(kernel_size, 1, std::max(1, max_odd));
    if (clamped % 2 == 0) {
        clamped = std::min(clamped + 1, std::max(1, max_odd));
        if (clamped % 2 == 0)
            --clamped;
    }
    return std::max(1, clamped);
}

[[nodiscard]] double opencv_default_gaussian_sigma(int kernel_size) noexcept {
    if (kernel_size <= 1)
        return 1.0;
    return 0.3 * ((static_cast<double>(kernel_size) - 1.0) * 0.5 - 1.0) + 0.8;
}

[[nodiscard]] double
    gaussian_sigma_x(const GaussianDenoiseParams& params, int kernel_size) noexcept {
    return positive_filter_enabled(params.sigma_x) ? params.sigma_x
                                                   : opencv_default_gaussian_sigma(kernel_size);
}

[[nodiscard]] double gaussian_sigma_y(
    const GaussianDenoiseParams& params, int kernel_size, double sigma_x) noexcept {
    return positive_filter_enabled(params.sigma_y)
             ? params.sigma_y
             : (positive_filter_enabled(params.sigma_x)
                    ? sigma_x
                    : opencv_default_gaussian_sigma(kernel_size));
}

[[nodiscard]] std::array<double, 5> gaussian_1d_weights(int kernel_size, double sigma) noexcept {
    std::array<double, 5> weights{};
    const int radius = kernel_size / 2;
    if (radius == 0) {
        weights[2] = 1.0;
        return weights;
    }

    double sum         = 0.0;
    const double denom = 2.0 * sigma * sigma;
    for (int offset = -radius; offset <= radius; ++offset) {
        const double value  = std::exp(-(static_cast<double>(offset * offset)) / denom);
        weights[2 + offset] = value;
        sum += value;
    }

    if (sum <= 0.0) {
        weights[2] = 1.0;
        return weights;
    }

    for (auto& value : weights)
        value /= sum;
    return weights;
}

void rebalance_q10_filter_sum(AX_IVE_FILTER_CTRL_T& ctrl) noexcept {
    int sum = 0;
    for (const auto coeff : ctrl.as6q10Mask)
        sum += coeff;
    ctrl.as6q10Mask[12] = static_cast<AX_S6Q10>(ctrl.as6q10Mask[12] + (1024 - sum));
}

[[nodiscard]] AX_IVE_FILTER_CTRL_T
    make_luma_gaussian_filter(const GaussianDenoiseParams& params) noexcept {
    AX_IVE_FILTER_CTRL_T ctrl{};
    const int kernel_size = normalize_gaussian_kernel_size(params.kernel_size, 5);
    const double sigma_x  = gaussian_sigma_x(params, kernel_size);
    const double sigma_y  = gaussian_sigma_y(params, kernel_size, sigma_x);
    const auto x_weights  = gaussian_1d_weights(kernel_size, sigma_x);
    const auto y_weights  = gaussian_1d_weights(kernel_size, sigma_y);
    const int radius      = kernel_size / 2;

    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
            const int index        = (2 + y) * 5 + (2 + x);
            const double weight    = y_weights[2 + y] * x_weights[2 + x];
            ctrl.as6q10Mask[index] = static_cast<AX_S6Q10>(std::lround(weight * 1024.0));
        }
    }
    rebalance_q10_filter_sum(ctrl);
    return ctrl;
}

[[nodiscard]] AX_IVE_FILTER_CTRL_T
    make_chroma_gaussian_filter(const GaussianDenoiseParams& params) noexcept {
    AX_IVE_FILTER_CTRL_T ctrl{};
    // NV12 UV is interleaved. A 3x3 chroma-space kernel maps to byte columns
    // -2, 0, +2 in IVE's 5x5 U8C1 stencil and keeps U/V from bleeding together.
    const int kernel_size = normalize_gaussian_kernel_size(params.kernel_size, 3);
    const double sigma_x  = gaussian_sigma_x(params, kernel_size);
    const double sigma_y  = gaussian_sigma_y(params, kernel_size, sigma_x);
    const auto x_weights  = gaussian_1d_weights(kernel_size, sigma_x);
    const auto y_weights  = gaussian_1d_weights(kernel_size, sigma_y);
    const int radius      = kernel_size / 2;

    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
            const int index        = (2 + y) * 5 + (2 + x * 2);
            const double weight    = y_weights[2 + y] * x_weights[2 + x];
            ctrl.as6q10Mask[index] = static_cast<AX_S6Q10>(std::lround(weight * 1024.0));
        }
    }
    rebalance_q10_filter_sum(ctrl);
    return ctrl;
}

[[nodiscard]] AX_IVE_FILTER_CTRL_T make_high_boost_5x5(double amount) noexcept {
    AX_IVE_FILTER_CTRL_T ctrl{};
    const int clamped =
        static_cast<int>(std::lround(std::clamp(amount, 0.0, kMaxSharpenAmount) * 1024.0));
    const int surround = -clamped / 24;
    for (auto& coeff : ctrl.as6q10Mask) {
        coeff = static_cast<AX_S6Q10>(surround);
    }
    ctrl.as6q10Mask[12] = static_cast<AX_S6Q10>(1024 + clamped);
    return ctrl;
}

[[nodiscard]] AX_IVE_DILATE_CTRL_T make_dilate_5x5() noexcept {
    AX_IVE_DILATE_CTRL_T ctrl{};
    for (auto& value : ctrl.au8Mask) {
        value = 255;
    }
    return ctrl;
}

struct IvpsInputFrame {
    AX_U64 phy_addr   = 0;
    void* vir_addr    = nullptr;
    AX_U32 alloc_size = 0;
    AX_U32 stride     = 0;
    int width         = 0;
    int height        = 0;

    IvpsInputFrame()                                 = default;
    IvpsInputFrame(const IvpsInputFrame&)            = delete;
    IvpsInputFrame& operator=(const IvpsInputFrame&) = delete;

    IvpsInputFrame(IvpsInputFrame&& other) noexcept
        : phy_addr(other.phy_addr)
        , vir_addr(other.vir_addr)
        , alloc_size(other.alloc_size)
        , stride(other.stride)
        , width(other.width)
        , height(other.height) {
        other.phy_addr   = 0;
        other.vir_addr   = nullptr;
        other.alloc_size = 0;
        other.stride     = 0;
        other.width      = 0;
        other.height     = 0;
    }

    IvpsInputFrame& operator=(IvpsInputFrame&& other) noexcept {
        if (this != &other) {
            destroy();
            phy_addr         = other.phy_addr;
            vir_addr         = other.vir_addr;
            alloc_size       = other.alloc_size;
            stride           = other.stride;
            width            = other.width;
            height           = other.height;
            other.phy_addr   = 0;
            other.vir_addr   = nullptr;
            other.alloc_size = 0;
            other.stride     = 0;
            other.width      = 0;
            other.height     = 0;
        }
        return *this;
    }

    ~IvpsInputFrame() { destroy(); }

    void destroy() noexcept {
        if (vir_addr != nullptr && phy_addr != 0) {
            AX_SYS_MemFree(phy_addr, vir_addr);
        }
        phy_addr   = 0;
        vir_addr   = nullptr;
        alloc_size = 0;
        stride     = 0;
        width      = 0;
        height     = 0;
    }

    [[nodiscard]] std::expected<void, std::string> allocate(int w, int h) noexcept {
        if (w <= 0 || h <= 0) {
            return std::unexpected(std::format("invalid IVPS input size: {}x{}", w, h));
        }

        const AX_U32 new_stride = bgr888_stride_aligned(w);
        const AX_U32 new_size   = new_stride * static_cast<AX_U32>(h);
        if (vir_addr != nullptr && width == w && height == h && stride == new_stride
            && alloc_size >= new_size) {
            return {};
        }

        destroy();

        width  = w;
        height = h;
        stride = new_stride;

        constexpr AX_U32 kBgrAlign = 48U;
        AX_S32 ret                 = AX_SYS_MemAlloc(
            &phy_addr, &vir_addr, new_size, kBgrAlign,
            const_cast<AX_S8*>(reinterpret_cast<const AX_S8*>("QuantaIVPSIn")));
        if (ret != AX_SUCCESS || vir_addr == nullptr) {
            destroy();
            return std::unexpected(
                std::format(
                    "AX_SYS_MemAlloc failed for IVPS input {}x{}: {}", w, h, ax_error_string(ret)));
        }

        alloc_size = new_size;
        return {};
    }
};

struct QpDeltaMap {
    static constexpr int kBlockSize = 16;

    AX_U64 phy_addr   = 0;
    void* vir_addr    = nullptr;
    AX_U32 alloc_size = 0;
    AX_U32 map_size   = 0;
    int block_cols    = 0;
    int block_rows    = 0;

    QpDeltaMap()                             = default;
    QpDeltaMap(const QpDeltaMap&)            = delete;
    QpDeltaMap& operator=(const QpDeltaMap&) = delete;

    QpDeltaMap(QpDeltaMap&& other) noexcept
        : phy_addr(other.phy_addr)
        , vir_addr(other.vir_addr)
        , alloc_size(other.alloc_size)
        , map_size(other.map_size)
        , block_cols(other.block_cols)
        , block_rows(other.block_rows) {
        other.phy_addr   = 0;
        other.vir_addr   = nullptr;
        other.alloc_size = 0;
        other.map_size   = 0;
        other.block_cols = 0;
        other.block_rows = 0;
    }

    QpDeltaMap& operator=(QpDeltaMap&& other) noexcept {
        if (this != &other) {
            destroy();
            phy_addr         = other.phy_addr;
            vir_addr         = other.vir_addr;
            alloc_size       = other.alloc_size;
            map_size         = other.map_size;
            block_cols       = other.block_cols;
            block_rows       = other.block_rows;
            other.phy_addr   = 0;
            other.vir_addr   = nullptr;
            other.alloc_size = 0;
            other.map_size   = 0;
            other.block_cols = 0;
            other.block_rows = 0;
        }
        return *this;
    }

    ~QpDeltaMap() { destroy(); }

    void destroy() noexcept {
        if (vir_addr != nullptr && phy_addr != 0) {
            AX_SYS_MemFree(phy_addr, vir_addr);
        }
        phy_addr   = 0;
        vir_addr   = nullptr;
        alloc_size = 0;
        map_size   = 0;
        block_cols = 0;
        block_rows = 0;
    }

    [[nodiscard]] std::expected<void, std::string> allocate(int width, int height) noexcept {
        if (width <= 0 || height <= 0) {
            return std::unexpected(std::format("invalid QP delta map size: {}x{}", width, height));
        }

        destroy();

        block_cols = (width + kBlockSize - 1) / kBlockSize;
        block_rows = (height + kBlockSize - 1) / kBlockSize;
        map_size   = static_cast<AX_U32>(block_cols * block_rows);
        alloc_size = align_up_u32(map_size, kCmmAlignment);

        AX_S32 ret = AX_SYS_MemAllocCached(
            &phy_addr, &vir_addr, alloc_size, kCmmAlignment,
            const_cast<AX_S8*>(reinterpret_cast<const AX_S8*>("QuantaQpMap")));
        if (ret != AX_SUCCESS || vir_addr == nullptr) {
            destroy();
            return std::unexpected(
                std::format(
                    "AX_SYS_MemAllocCached failed for QP delta map {}x{} blocks: {}", block_cols,
                    block_rows, ax_error_string(ret)));
        }

        std::memset(vir_addr, 0, alloc_size);
        return {};
    }

    [[nodiscard]] AX_S8* data() noexcept { return static_cast<AX_S8*>(vir_addr); }
    [[nodiscard]] const AX_S8* data() const noexcept { return static_cast<const AX_S8*>(vir_addr); }

    void flush_cache() noexcept {
        if (phy_addr != 0 && vir_addr != nullptr && alloc_size != 0) {
            AX_SYS_MflushCache(phy_addr, vir_addr, alloc_size);
        }
    }
};

struct IveImage {
    AX_U64 phy_addr   = 0;
    void* vir_addr    = nullptr;
    AX_U32 alloc_size = 0;
    AX_U32 stride     = 0;
    int width         = 0;
    int height        = 0;

    IveImage()                           = default;
    IveImage(const IveImage&)            = delete;
    IveImage& operator=(const IveImage&) = delete;

    IveImage(IveImage&& other) noexcept
        : phy_addr(other.phy_addr)
        , vir_addr(other.vir_addr)
        , alloc_size(other.alloc_size)
        , stride(other.stride)
        , width(other.width)
        , height(other.height) {
        other.phy_addr   = 0;
        other.vir_addr   = nullptr;
        other.alloc_size = 0;
        other.stride     = 0;
        other.width      = 0;
        other.height     = 0;
    }

    IveImage& operator=(IveImage&& other) noexcept {
        if (this != &other) {
            destroy();
            phy_addr         = other.phy_addr;
            vir_addr         = other.vir_addr;
            alloc_size       = other.alloc_size;
            stride           = other.stride;
            width            = other.width;
            height           = other.height;
            other.phy_addr   = 0;
            other.vir_addr   = nullptr;
            other.alloc_size = 0;
            other.stride     = 0;
            other.width      = 0;
            other.height     = 0;
        }
        return *this;
    }

    ~IveImage() { destroy(); }

    void destroy() noexcept {
        if (vir_addr != nullptr && phy_addr != 0) {
            AX_SYS_MemFree(phy_addr, vir_addr);
        }
        phy_addr   = 0;
        vir_addr   = nullptr;
        alloc_size = 0;
        stride     = 0;
        width      = 0;
        height     = 0;
    }

    [[nodiscard]] std::expected<void, std::string>
        allocate_u8(int w, int h, const char* name) noexcept {
        if (w <= 0 || h <= 0) {
            return std::unexpected(std::format("invalid IVE image size: {}x{}", w, h));
        }

        const AX_U32 new_stride = align_up_u32(static_cast<AX_U32>(w), kCmmAlignment);
        const AX_U32 new_size   = new_stride * static_cast<AX_U32>(h);
        if (vir_addr != nullptr && width == w && height == h && stride == new_stride
            && alloc_size >= new_size) {
            return {};
        }

        destroy();

        width  = w;
        height = h;
        stride = new_stride;

        AX_S32 ret = AX_SYS_MemAllocCached(
            &phy_addr, &vir_addr, new_size, kCmmAlignment,
            const_cast<AX_S8*>(reinterpret_cast<const AX_S8*>(name)));
        if (ret != AX_SUCCESS || vir_addr == nullptr) {
            destroy();
            return std::unexpected(
                std::format(
                    "AX_SYS_MemAllocCached failed for IVE image {}x{}: {}", w, h,
                    ax_error_string(ret)));
        }

        alloc_size = new_size;
        std::memset(vir_addr, 0, alloc_size);
        flush_cache();
        return {};
    }

    [[nodiscard]] uint8_t* data() noexcept { return static_cast<uint8_t*>(vir_addr); }
    [[nodiscard]] const uint8_t* data() const noexcept {
        return static_cast<const uint8_t*>(vir_addr);
    }

    void clear(uint8_t value) noexcept {
        if (vir_addr == nullptr || alloc_size == 0)
            return;
        std::memset(vir_addr, value, alloc_size);
        flush_cache();
    }

    void flush_cache() noexcept {
        if (phy_addr != 0 && vir_addr != nullptr && alloc_size != 0) {
            AX_SYS_MflushCache(phy_addr, vir_addr, alloc_size);
        }
    }

    [[nodiscard]] AX_S32 invalidate_cache() noexcept {
        if (phy_addr != 0 && vir_addr != nullptr && alloc_size != 0) {
            return AX_SYS_MinvalidateCache(phy_addr, vir_addr, alloc_size);
        }
        return AX_SUCCESS;
    }
};

[[nodiscard]] AX_VIDEO_FRAME_T make_ivps_src_frame(const IvpsInputFrame& src) noexcept {
    AX_VIDEO_FRAME_T frame{};
    frame.u32Width        = static_cast<AX_U32>(src.width);
    frame.u32Height       = static_cast<AX_U32>(src.height);
    frame.u32PicStride[0] = src.stride;
    // Axera names this RGB888, but the SDK layout comment is BGRBGR...
    // Quanta's public input contract is packed BGR24 (OpenCV cv::Mat).
    frame.enImgFormat                   = AX_FORMAT_RGB888;
    frame.u64PhyAddr[0]                 = src.phy_addr;
    frame.u64VirAddr[0]                 = reinterpret_cast<AX_U64>(src.vir_addr);
    frame.u32FrameSize                  = src.alloc_size;
    frame.stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
    return frame;
}

[[nodiscard]] AX_VIDEO_FRAME_T make_ivps_dst_frame(const Nv12Frame& dst) noexcept {
    AX_VIDEO_FRAME_T frame{};
    frame.u32Width                      = static_cast<AX_U32>(dst.width);
    frame.u32Height                     = static_cast<AX_U32>(dst.height);
    frame.u32PicStride[0]               = static_cast<AX_U32>(dst.y_stride);
    frame.u32PicStride[1]               = static_cast<AX_U32>(dst.uv_stride);
    frame.enImgFormat                   = AX_FORMAT_YUV420_SEMIPLANAR;
    frame.u64PhyAddr[0]                 = dst.y_phy_addr();
    frame.u64VirAddr[0]                 = reinterpret_cast<AX_U64>(dst.y_data());
    frame.u64PhyAddr[1]                 = dst.uv_phy_addr();
    frame.u64VirAddr[1]                 = reinterpret_cast<AX_U64>(dst.uv_data());
    frame.u32FrameSize                  = static_cast<AX_U32>(dst.y_size() + dst.uv_size());
    frame.stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
    return frame;
}

[[nodiscard]] AX_IVE_IMAGE_T make_ive_u8_image(const IveImage& image) noexcept {
    AX_IVE_IMAGE_T out{};
    out.enType         = AX_IVE_IMAGE_TYPE_U8C1;
    out.u32Width       = static_cast<AX_U32>(image.width);
    out.u32Height      = static_cast<AX_U32>(image.height);
    out.au32Stride[0]  = image.stride;
    out.au64PhyAddr[0] = image.phy_addr;
    out.au64VirAddr[0] = reinterpret_cast<AX_U64>(image.vir_addr);
    return out;
}

[[nodiscard]] AX_IVE_IMAGE_T make_ive_y_image(const Nv12Frame& frame) noexcept {
    AX_IVE_IMAGE_T out{};
    out.enType         = AX_IVE_IMAGE_TYPE_U8C1;
    out.u32Width       = static_cast<AX_U32>(frame.width);
    out.u32Height      = static_cast<AX_U32>(frame.height);
    out.au32Stride[0]  = static_cast<AX_U32>(frame.y_stride);
    out.au64PhyAddr[0] = frame.y_phy_addr();
    out.au64VirAddr[0] = reinterpret_cast<AX_U64>(frame.y_data());
    return out;
}

[[nodiscard]] AX_IVE_IMAGE_T make_ive_uv_image(const Nv12Frame& frame) noexcept {
    AX_IVE_IMAGE_T out{};
    out.enType         = AX_IVE_IMAGE_TYPE_U8C1;
    out.u32Width       = static_cast<AX_U32>(frame.width);
    out.u32Height      = static_cast<AX_U32>(frame.height / 2);
    out.au32Stride[0]  = static_cast<AX_U32>(frame.uv_stride);
    out.au64PhyAddr[0] = frame.uv_phy_addr();
    out.au64VirAddr[0] = reinterpret_cast<AX_U64>(frame.uv_data());
    return out;
}

[[nodiscard]] AX_IVE_IMAGE_T
    make_ive_u8_view(const IveImage& image, int width, int height) noexcept {
    AX_IVE_IMAGE_T out = make_ive_u8_image(image);
    out.u32Width       = static_cast<AX_U32>(std::max(0, std::min(width, image.width)));
    out.u32Height      = static_cast<AX_U32>(std::max(0, std::min(height, image.height)));
    return out;
}

[[nodiscard]] AX_IVE_DATA_T make_ive_data(
    AX_U64 phy_addr, void* vir_addr, AX_U32 stride, AX_U32 width, AX_U32 height) noexcept {
    AX_IVE_DATA_T data{};
    data.u64PhyAddr = phy_addr;
    data.u64VirAddr = reinterpret_cast<AX_U64>(vir_addr);
    data.u32Stride  = stride;
    data.u32Width   = width;
    data.u32Height  = height;
    return data;
}

[[nodiscard]] AX_U32 ive_u8_image_size(const AX_IVE_IMAGE_T& image) noexcept {
    return image.au32Stride[0] * image.u32Height;
}

[[nodiscard]] AX_S32 invalidate_ive_u8_image(const AX_IVE_IMAGE_T& image) noexcept {
    if (image.au64PhyAddr[0] == 0 || image.au64VirAddr[0] == 0 || image.au32Stride[0] == 0
        || image.u32Height == 0) {
        return AX_SUCCESS;
    }
    return AX_SYS_MinvalidateCache(
        image.au64PhyAddr[0], reinterpret_cast<void*>(image.au64VirAddr[0]),
        ive_u8_image_size(image));
}

void flush_ive_u8_image(const AX_IVE_IMAGE_T& image) noexcept {
    if (image.au64PhyAddr[0] == 0 || image.au64VirAddr[0] == 0 || image.au32Stride[0] == 0
        || image.u32Height == 0) {
        return;
    }
    AX_SYS_MflushCache(
        image.au64PhyAddr[0], reinterpret_cast<void*>(image.au64VirAddr[0]),
        ive_u8_image_size(image));
}

[[nodiscard]] std::expected<void, std::string>
    split_nv12_uv_to_planes(Nv12Frame& src, IveImage& u_plane, IveImage& v_plane) noexcept {
    const int chroma_width  = src.width / 2;
    const int chroma_height = src.height / 2;
    if (chroma_width <= 0 || chroma_height <= 0) {
        return std::unexpected(
            std::format("invalid chroma split dimensions: {}x{}", chroma_width, chroma_height));
    }
    if (u_plane.data() == nullptr || v_plane.data() == nullptr || u_plane.width < chroma_width
        || v_plane.width < chroma_width || u_plane.height < chroma_height
        || v_plane.height < chroma_height) {
        return std::unexpected("chroma split planes are not allocated");
    }

    const AX_S32 invalidate_ret = src.invalidate_uv_cache();
    if (invalidate_ret != AX_SUCCESS) {
        return std::unexpected(
            std::format(
                "AX_SYS_MinvalidateCache NV12 UV failed: {}", ax_error_string(invalidate_ret)));
    }

    const auto* uv_base = src.uv_data();
    for (int y = 0; y < chroma_height; ++y) {
        const auto* uv_row = uv_base + static_cast<size_t>(y) * src.uv_stride;
        auto* u_row        = u_plane.data() + static_cast<size_t>(y) * u_plane.stride;
        auto* v_row        = v_plane.data() + static_cast<size_t>(y) * v_plane.stride;
        for (int x = 0; x < chroma_width; ++x) {
            u_row[x] = uv_row[x * 2];
            v_row[x] = uv_row[x * 2 + 1];
        }
    }

    return {};
}

[[nodiscard]] uint8_t
    sample_u8_plane(const IveImage& image, int width, int height, int x, int y) noexcept {
    const int sx = std::clamp(x, 0, width - 1);
    const int sy = std::clamp(y, 0, height - 1);
    return image.data()[static_cast<size_t>(sy) * image.stride + sx];
}

[[nodiscard]] std::expected<void, std::string> high_boost_u8_plane(
    const IveImage& src, IveImage& dst, int width, int height, double amount,
    std::string_view op) noexcept {
    if (src.data() == nullptr || dst.data() == nullptr) {
        return std::unexpected(std::format("{}: source or destination plane is not allocated", op));
    }
    if (width <= 0 || height <= 0 || src.width < width || dst.width < width || src.height < height
        || dst.height < height) {
        return std::unexpected(
            std::format("{}: invalid plane dimensions {}x{}", op, width, height));
    }

    const double clamped_amount = std::clamp(amount, 0.0, kMaxSharpenAmount);
    for (int y = 0; y < height; ++y) {
        auto* dst_row = dst.data() + static_cast<size_t>(y) * dst.stride;
        for (int x = 0; x < width; ++x) {
            int surround_sum = 0;
            for (int ky = -2; ky <= 2; ++ky) {
                for (int kx = -2; kx <= 2; ++kx) {
                    if (kx == 0 && ky == 0) {
                        continue;
                    }
                    surround_sum += sample_u8_plane(src, width, height, x + kx, y + ky);
                }
            }

            const int center = sample_u8_plane(src, width, height, x, y);
            const double boosted =
                static_cast<double>(center)
                + clamped_amount
                      * (static_cast<double>(center) - static_cast<double>(surround_sum) / 24.0);
            dst_row[x] = clamp_u8(boosted);
        }
    }

    return {};
}

[[nodiscard]] std::expected<void, std::string> quantize_u8_plane(
    const IveImage& src, IveImage& dst, int width, int height, int levels,
    std::string_view op) noexcept {
    const int clamped_levels = clamped_chroma_levels(levels);
    if (clamped_levels <= 0) {
        return {};
    }
    if (src.data() == nullptr || dst.data() == nullptr) {
        return std::unexpected(std::format("{}: source or destination plane is not allocated", op));
    }
    if (width <= 0 || height <= 0 || src.width < width || dst.width < width || src.height < height
        || dst.height < height) {
        return std::unexpected(
            std::format("{}: invalid plane dimensions {}x{}", op, width, height));
    }

    const int step = std::max(1, static_cast<int>(std::lround(255.0 / (clamped_levels - 1))));
    for (int y = 0; y < height; ++y) {
        const auto* src_row = src.data() + static_cast<size_t>(y) * src.stride;
        auto* dst_row       = dst.data() + static_cast<size_t>(y) * dst.stride;
        for (int x = 0; x < width; ++x) {
            const int bucket =
                std::min((static_cast<int>(src_row[x]) * clamped_levels) / 256, clamped_levels - 1);
            dst_row[x] = clamp_u8(bucket * step);
        }
    }

    return {};
}

[[nodiscard]] std::expected<void, std::string> merge_chroma_planes_to_nv12_uv(
    const IveImage& u_plane, const IveImage& v_plane, Nv12Frame& dst) noexcept {
    const int chroma_width  = dst.width / 2;
    const int chroma_height = dst.height / 2;
    if (u_plane.data() == nullptr || v_plane.data() == nullptr || u_plane.width < chroma_width
        || v_plane.width < chroma_width || u_plane.height < chroma_height
        || v_plane.height < chroma_height) {
        return std::unexpected("chroma merge planes are not allocated");
    }

    auto* uv_base = dst.uv_data();
    for (int y = 0; y < chroma_height; ++y) {
        const auto* u_row = u_plane.data() + static_cast<size_t>(y) * u_plane.stride;
        const auto* v_row = v_plane.data() + static_cast<size_t>(y) * v_plane.stride;
        auto* uv_row      = uv_base + static_cast<size_t>(y) * dst.uv_stride;
        for (int x = 0; x < chroma_width; ++x) {
            uv_row[x * 2]     = u_row[x];
            uv_row[x * 2 + 1] = v_row[x];
        }
    }

    dst.flush_uv_cache();
    return {};
}

[[nodiscard]] std::expected<void, std::string> ive_dma_copy(
    AX_U64 src_phy, void* src_vir, AX_U32 src_stride, AX_U64 dst_phy, void* dst_vir,
    AX_U32 dst_stride, AX_U32 width_bytes, AX_U32 height, std::string_view op) noexcept {
    AX_IVE_SRC_DATA_T src = make_ive_data(src_phy, src_vir, src_stride, width_bytes, height);
    AX_IVE_DST_DATA_T dst = make_ive_data(dst_phy, dst_vir, dst_stride, width_bytes, height);

    AX_IVE_DMA_CTRL_T ctrl{};
    ctrl.enMode = AX_IVE_DMA_MODE_DIRECT_COPY;

    std::scoped_lock lock(ive_submit_mutex());
    AX_IVE_HANDLE handle = 0;
    const AX_S32 ret     = AX_IVE_DMA(&handle, &src, &dst, &ctrl, AX_TRUE);
    return check_ive(ret, op);
}

[[nodiscard]] std::expected<void, std::string> ive_filter(
    AX_IVE_IMAGE_T& src, AX_IVE_IMAGE_T& dst, AX_IVE_FILTER_CTRL_T& ctrl,
    std::string_view op) noexcept {
    std::scoped_lock lock(ive_submit_mutex());
    AX_IVE_HANDLE handle = 0;
    const AX_S32 ret     = AX_IVE_Filter(&handle, &src, &dst, &ctrl, AX_TRUE);
    return check_ive(ret, op);
}

[[nodiscard]] std::expected<void, std::string> ive_thresh_binary(
    AX_IVE_IMAGE_T& src, AX_IVE_IMAGE_T& dst, int threshold, uint8_t max_value,
    std::string_view op) noexcept {
    const auto low = static_cast<AX_U8>(std::clamp(threshold, 0, 255));

    AX_IVE_THRESH_CTRL_T ctrl{};
    // Deployed AX650 runtimes validate the full threshold range even when
    // BINARY mode ignores HighThr/MidVal according to the public header.
    ctrl.enMode    = AX_IVE_THRESH_MODE_BINARY;
    ctrl.u8LowThr  = low;
    ctrl.u8HighThr = 255;
    ctrl.u8MinVal  = 0;
    ctrl.u8MidVal  = max_value;
    ctrl.u8MaxVal  = max_value;

    std::scoped_lock lock(ive_submit_mutex());
    AX_IVE_HANDLE handle = 0;
    const AX_S32 ret     = AX_IVE_Thresh(&handle, &src, &dst, &ctrl, AX_TRUE);
    return check_ive(ret, op);
}

[[nodiscard]] std::expected<void, std::string> ive_sub_abs(
    AX_IVE_IMAGE_T& lhs, AX_IVE_IMAGE_T& rhs, AX_IVE_IMAGE_T& dst, std::string_view op) noexcept {
    AX_IVE_SUB_CTRL_T ctrl{};
    ctrl.enMode = AX_IVE_SUB_MODE_ABS;

    std::scoped_lock lock(ive_submit_mutex());
    AX_IVE_HANDLE handle = 0;
    const AX_S32 ret     = AX_IVE_Sub(&handle, &lhs, &rhs, &dst, &ctrl, AX_TRUE);
    return check_ive(ret, op);
}

[[nodiscard]] std::expected<void, std::string>
    ive_dilate(AX_IVE_IMAGE_T& src, AX_IVE_IMAGE_T& dst, std::string_view op) noexcept {
    AX_IVE_DILATE_CTRL_T ctrl = make_dilate_5x5();
    std::scoped_lock lock(ive_submit_mutex());
    AX_IVE_HANDLE handle = 0;
    const AX_S32 ret     = AX_IVE_Dilate(&handle, &src, &dst, &ctrl, AX_TRUE);
    return check_ive(ret, op);
}

[[nodiscard]] std::expected<void, std::string> ive_xor(
    AX_IVE_IMAGE_T& lhs, AX_IVE_IMAGE_T& rhs, AX_IVE_IMAGE_T& dst, std::string_view op) noexcept {
    std::scoped_lock lock(ive_submit_mutex());
    AX_IVE_HANDLE handle = 0;
    const AX_S32 ret     = AX_IVE_Xor(&handle, &lhs, &rhs, &dst, AX_TRUE);
    return check_ive(ret, op);
}

[[nodiscard]] std::expected<void, std::string> ive_and(
    AX_IVE_IMAGE_T& lhs, AX_IVE_IMAGE_T& rhs, AX_IVE_IMAGE_T& dst, std::string_view op) noexcept {
    std::scoped_lock lock(ive_submit_mutex());
    AX_IVE_HANDLE handle = 0;
    const AX_S32 ret     = AX_IVE_And(&handle, &lhs, &rhs, &dst, AX_TRUE);
    return check_ive(ret, op);
}

void fill_qp_delta_map(
    QpDeltaMap& qp_map, IveImage& edge_mask, bool edge_mask_valid, int width, int height,
    const FilterParams& filter) noexcept {
    if (qp_map.data() == nullptr || qp_map.map_size == 0)
        return;

    const auto edge_delta     = static_cast<AX_S8>(std::clamp(filter.qp_edge_delta, -31, 32));
    const auto interior_delta = static_cast<AX_S8>(std::clamp(filter.qp_interior_delta, -31, 32));

    auto* map = qp_map.data();
    std::fill_n(map, qp_map.map_size, interior_delta);

    if (!edge_mask_valid || edge_mask.data() == nullptr)
        return;

    const AX_S32 invalidate_ret = edge_mask.invalidate_cache();
    if (invalidate_ret != AX_SUCCESS)
        return;

    for (int by = 0; by < qp_map.block_rows; ++by) {
        const int y0 = by * QpDeltaMap::kBlockSize;
        const int bh = std::min(QpDeltaMap::kBlockSize, height - y0);
        if (bh <= 0)
            continue;

        for (int bx = 0; bx < qp_map.block_cols; ++bx) {
            const int x0 = bx * QpDeltaMap::kBlockSize;
            const int bw = std::min(QpDeltaMap::kBlockSize, width - x0);
            if (bw <= 0)
                continue;

            bool has_edge = false;
            for (int y = 0; y < bh && !has_edge; ++y) {
                const auto* row = edge_mask.data() + static_cast<size_t>(y0 + y) * edge_mask.stride;
                for (int x = 0; x < bw; ++x) {
                    if (row[x0 + x] != 0) {
                        has_edge = true;
                        break;
                    }
                }
            }

            if (has_edge) {
                map[by * qp_map.block_cols + bx] = edge_delta;
            }
        }
    }
}

[[nodiscard]] AX_USER_FRAME_INFO_T
    make_user_frame_info(const AX_VIDEO_FRAME_INFO_T& frame_info, QpDeltaMap& qp_map) noexcept {
    AX_USER_FRAME_INFO_T user_frame{};
    user_frame.stUserFrame = frame_info;

    auto& rc              = user_frame.stUserRcInfo;
    rc.bQpMapValid        = AX_TRUE;
    rc.bIPCMMapValid      = AX_FALSE;
    rc.u32BlkStartQp      = 46;
    rc.u64QpMapPhyAddr    = qp_map.phy_addr;
    rc.pQpMapVirAddr      = qp_map.data();
    rc.u64IpcmMapPhyAddr  = 0;
    rc.enFrameType        = AX_FRAME_TYPE_AUTO;
    rc.u32RoiMapDeltaSize = qp_map.map_size;

    return user_frame;
}

} // namespace

std::expected<AxFilterChain, std::string> build_ax_filter_chain(
    const FilterParams& filter, int src_width, int src_height, int dst_width,
    int dst_height) noexcept {
    auto ivps_module_result = IvpsModule::acquire();
    if (!ivps_module_result)
        return std::unexpected(std::move(ivps_module_result.error()));

    const bool has_geometry_change = src_width != dst_width || src_height != dst_height;
    const bool use_split_chroma    = has_chroma_plane_filters(filter);
    const bool use_ive_filters     = has_ive_image_filters(filter);

    std::shared_ptr<IveModule> ive_module;
    if (use_ive_filters) {
        auto ive_module_result = IveModule::acquire();
        if (!ive_module_result)
            return std::unexpected(std::move(ive_module_result.error()));
        ive_module = std::move(*ive_module_result);
    }

    return AxFilterChain{
        .ivps_module           = std::move(*ivps_module_result),
        .ive_module            = std::move(ive_module),
        .aspect_ratio          = make_stretch_aspect_ratio(),
        .stage_resize_then_csc = has_geometry_change,
        .use_split_chroma      = use_split_chroma,
        .use_ive_filters       = use_ive_filters,
    };
}

// ===========================================================================
// VencChannel RAII
// ===========================================================================

namespace {
std::atomic<int> g_next_chn{0}; // round-robin channel allocation
constexpr int kMaxVencChn = 64;

// Some deployed AX650 runtime images ship libax_venc.so that memcpy()s
// more bytes for AX_VENC_CHN_ATTR_T than older headers define (e.g. 224 vs 208).
// Use a zero-padded staging buffer (with extra headroom) to keep compatibility
// across minor SDK drifts.
constexpr std::size_t kCompatVencChnAttrBytes = 256;
static_assert(std::is_trivially_copyable_v<AX_VENC_CHN_ATTR_T>);

// The deployed runtime and the build-time headers can drift across minor SDK
// revisions. Keep a generous zero-padded staging area for VENC config structs
// that the runtime may over-read or over-write.
constexpr std::size_t kCompatVencConfigBytes = 512;

template <typename T, std::size_t CompatBytes, typename F>
AX_S32 ax_venc_call_with_compat_in(const T& value, F&& fn) noexcept {
    static_assert(std::is_trivially_copyable_v<T>);
    if constexpr (sizeof(T) >= CompatBytes) {
        return fn(&value);
    } else {
        alignas(T) std::array<std::byte, CompatBytes> raw{};
        std::memcpy(raw.data(), &value, sizeof(T));
        return fn(reinterpret_cast<const T*>(raw.data()));
    }
}

template <typename T, std::size_t CompatBytes, typename F>
AX_S32 ax_venc_call_with_compat_out(T& value, F&& fn) noexcept {
    static_assert(std::is_trivially_copyable_v<T>);
    if constexpr (sizeof(T) >= CompatBytes) {
        return fn(&value);
    } else {
        alignas(T) std::array<std::byte, CompatBytes> raw{};
        const AX_S32 ret = fn(reinterpret_cast<T*>(raw.data()));
        if (ret == AX_SUCCESS) {
            std::memcpy(&value, raw.data(), sizeof(T));
        }
        return ret;
    }
}

AX_S32 ax_venc_create_chn_compat(VENC_CHN chn, const AX_VENC_CHN_ATTR_T& chn_attr) noexcept {
    if constexpr (sizeof(AX_VENC_CHN_ATTR_T) >= kCompatVencChnAttrBytes) {

        return AX_VENC_CreateChn(chn, &chn_attr);
    } else {
        alignas(AX_VENC_CHN_ATTR_T) std::array<std::byte, kCompatVencChnAttrBytes> attr_raw{};
        std::memcpy(attr_raw.data(), &chn_attr, sizeof(AX_VENC_CHN_ATTR_T));
        return AX_VENC_CreateChn(chn, reinterpret_cast<const AX_VENC_CHN_ATTR_T*>(attr_raw.data()));
    }
}
} // namespace

VencChannel::VencChannel(std::shared_ptr<VencModule> module, VENC_CHN chn)
    : module_(std::move(module))
    , chn_(chn)
    , active_(true) {}

std::expected<VencChannel, std::string> VencChannel::create(
    std::shared_ptr<VencModule> module, const AX_VENC_CHN_ATTR_T& chn_attr) noexcept {
    // Round-robin channel allocation.  AX_VENC_CreateChn will fail with an
    // appropriate error code if the channel is already in use — the caller
    // should retry with a different index.  In practice a single encoder
    // instance uses at most one channel.
    VENC_CHN chn = g_next_chn.fetch_add(1) % kMaxVencChn;

    AX_S32 ret = ax_venc_create_chn_compat(chn, chn_attr);

    if (ret != AX_SUCCESS) {
        // If the preferred channel is taken, try the next few indices
        for (int attempt = 1; attempt < kMaxVencChn; ++attempt) {
            chn = (chn + 1) % kMaxVencChn;
            ret = ax_venc_create_chn_compat(chn, chn_attr);
            if (ret == AX_SUCCESS)
                return VencChannel(std::move(module), chn);
        }
        return std::unexpected(
            std::format(
                "AX_VENC_CreateChn failed: all {} channels occupied, "
                "last {}",
                kMaxVencChn, ax_error_string(ret)));
    }

    return VencChannel(std::move(module), chn);
}

VencChannel::~VencChannel() { destroy(); }

VencChannel::VencChannel(VencChannel&& other) noexcept
    : module_(std::move(other.module_))
    , chn_(other.chn_)
    , active_(other.active_) {
    other.active_ = false;
    other.chn_    = -1;
}

VencChannel& VencChannel::operator=(VencChannel&& other) noexcept {
    if (this != &other) {
        destroy();
        module_       = std::move(other.module_);
        chn_          = other.chn_;
        active_       = other.active_;
        other.active_ = false;
        other.chn_    = -1;
    }
    return *this;
}

void VencChannel::destroy() noexcept {
    if (active_) {
        AX_VENC_StopRecvFrame(chn_);
        AX_VENC_DestroyChn(chn_);
        active_ = false;
    }
}

AX_S32 VencChannel::send_frame(const AX_VIDEO_FRAME_INFO_T& frame, AX_S32 timeout_ms) noexcept {
    return AX_VENC_SendFrame(chn_, &frame, timeout_ms);
}

AX_S32 VencChannel::send_frame_ex(const AX_USER_FRAME_INFO_T& frame, AX_S32 timeout_ms) noexcept {
    return AX_VENC_SendFrameEx(chn_, &frame, timeout_ms);
}

AX_S32 VencChannel::get_stream(AX_VENC_STREAM_T& stream, AX_S32 timeout_ms) noexcept {
    return AX_VENC_GetStream(chn_, &stream, timeout_ms);
}

AX_S32 VencChannel::release_stream(const AX_VENC_STREAM_T& stream) noexcept {
    return AX_VENC_ReleaseStream(chn_, &stream);
}

AX_S32 VencChannel::start_recv_frame(const AX_VENC_RECV_PIC_PARAM_T& param) noexcept {
    return AX_VENC_StartRecvFrame(chn_, &param);
}

AX_S32 VencChannel::stop_recv_frame() noexcept { return AX_VENC_StopRecvFrame(chn_); }

AX_S32 VencChannel::request_idr(bool instant) noexcept {
    return AX_VENC_RequestIDR(chn_, instant ? AX_TRUE : AX_FALSE);
}

std::expected<AX_VENC_RC_PARAM_T, std::string> VencChannel::get_rc_param() noexcept {
    AX_VENC_RC_PARAM_T rc_param{};
    AX_S32 ret = ax_venc_call_with_compat_out<AX_VENC_RC_PARAM_T, kCompatVencConfigBytes>(
        rc_param, [&](AX_VENC_RC_PARAM_T* raw) noexcept { return AX_VENC_GetRcParam(id(), raw); });
    if (ret != AX_SUCCESS) {
        return std::unexpected(std::format(" AX_VENC_GetRcParam failed: {}", ax_error_string(ret)));
    }
    return rc_param;
}

AX_S32 VencChannel::set_rc_param(const AX_VENC_RC_PARAM_T& rc_param) noexcept {
    return ax_venc_call_with_compat_in<AX_VENC_RC_PARAM_T, kCompatVencConfigBytes>(
        rc_param,
        [&](const AX_VENC_RC_PARAM_T* raw) noexcept { return AX_VENC_SetRcParam(chn_, raw); });
}

AX_S32 VencChannel::set_vui_param(const AX_VENC_VUI_PARAM_T& vui_param) noexcept {
    return ax_venc_call_with_compat_in<AX_VENC_VUI_PARAM_T, kCompatVencConfigBytes>(
        vui_param,
        [&](const AX_VENC_VUI_PARAM_T* raw) noexcept { return AX_VENC_SetVuiParam(chn_, raw); });
}

AX_S32 VencChannel::set_intra_refresh(const AX_VENC_INTRA_REFRESH_T& cfg) noexcept {
    return ax_venc_call_with_compat_in<AX_VENC_INTRA_REFRESH_T, kCompatVencConfigBytes>(
        cfg, [&](const AX_VENC_INTRA_REFRESH_T* raw) noexcept {
            return AX_VENC_SetIntraRefresh(chn_, raw);
        });
}

AX_S32 VencChannel::set_roi_attr(const AX_VENC_ROI_ATTR_T& roi) noexcept {
    return ax_venc_call_with_compat_in<AX_VENC_ROI_ATTR_T, kCompatVencConfigBytes>(
        roi, [&](const AX_VENC_ROI_ATTR_T* raw) noexcept { return AX_VENC_SetRoiAttr(chn_, raw); });
}

AX_S32 VencChannel::query_status(AX_VENC_CHN_STATUS_T& status) noexcept {
    return ax_venc_call_with_compat_out<AX_VENC_CHN_STATUS_T, kCompatVencConfigBytes>(
        status, [&](AX_VENC_CHN_STATUS_T* raw) noexcept { return AX_VENC_QueryStatus(chn_, raw); });
}

// ===========================================================================
// Channel attribute construction
// ===========================================================================

[[nodiscard]] int64_t ax_rate_control_bitrate_bps(const EncodeParams& params) noexcept {
    const int selected_bps = params.enVBR ? params.max_bit_rate : params.target_bitrate;
    return std::clamp<int64_t>(selected_bps, VENC_MIN_BITRATE * 1000LL, VENC_MAX_BITRATE * 1000LL);
}

[[nodiscard]] AX_U32 ax_rate_control_bitrate_kbps(const EncodeParams& params) noexcept {
    const auto selected_bps = ax_rate_control_bitrate_bps(params);
    return static_cast<AX_U32>(std::clamp<int64_t>(
        static_cast<int64_t>(std::lround(static_cast<double>(selected_bps) / 1000.0)),
        VENC_MIN_BITRATE, VENC_MAX_BITRATE));
}

[[nodiscard]] const char* ax_rc_mode_name(const EncodeParams& params) noexcept {
    return params.enVBR ? "H265VBR" : "H265CBR";
}

std::expected<AX_VENC_CHN_ATTR_T, std::string>
    build_hevc_channel_attr(const EncodeParams& params, int width, int height) noexcept {
    if (width < MIN_VENC_PIC_WIDTH || width > MAX_VENC_PIC_WIDTH)
        return std::unexpected(
            std::format(
                "Width {} out of range [{}, {}]", width, MIN_VENC_PIC_WIDTH, MAX_VENC_PIC_WIDTH));
    if (height < MIN_VENC_PIC_HEIGHT || height > MAX_VENC_PIC_HEIGHT)
        return std::unexpected(
            std::format(
                "Height {} out of range [{}, {}]", height, MIN_VENC_PIC_HEIGHT,
                MAX_VENC_PIC_HEIGHT));
    if (width % 2 != 0 || height % 2 != 0)
        return std::unexpected(std::format("Dimensions {}x{} must be even", width, height));

    const auto bitrate_bps = ax_rate_control_bitrate_bps(params);

    // Estimate stream buffer size: ~3 frames worth at target bitrate (in bytes)
    const int framerate   = std::max(1, params.framerate);
    const AX_U32 buf_size = std::max<AX_U32>(
        256 * 1024, // minimum 256KB
        static_cast<AX_U32>((bitrate_bps / framerate / 8) * 3));

    AX_VENC_CHN_ATTR_T attr{};

    // --- stVencAttr ---
    attr.stVencAttr.enType          = PT_H265;
    attr.stVencAttr.u32MaxPicWidth  = static_cast<AX_U32>(std::max(width, params.max_width));
    attr.stVencAttr.u32MaxPicHeight = static_cast<AX_U32>(std::max(height, params.max_height));
    attr.stVencAttr.enMemSource     = AX_MEMORY_SOURCE_CMM;
    attr.stVencAttr.u32BufSize      = buf_size;
    attr.stVencAttr.enProfile       = AX_VENC_HEVC_MAIN_PROFILE;
    attr.stVencAttr.enLevel         = AX_VENC_HEVC_LEVEL_5;
    attr.stVencAttr.enTier          = AX_VENC_HEVC_MAIN_TIER;
    attr.stVencAttr.enStrmBitDepth  = AX_VENC_STREAM_BIT_8;
    attr.stVencAttr.u32PicWidthSrc  = static_cast<AX_U32>(width);
    attr.stVencAttr.u32PicHeightSrc = static_cast<AX_U32>(height);
    attr.stVencAttr.enLinkMode      = AX_VENC_UNLINK_MODE;
    attr.stVencAttr.s32StopWaitTime = -1; // wait for output queue to drain on destroy
    attr.stVencAttr.u8InFifoDepth   = 3;
    attr.stVencAttr.u8OutFifoDepth  = 3;
    // HEADER_ATTACH_TO_PB must stay disabled. If AX VENC still emits
    // VPS/SPS/PPS, strip those NALUs before packets enter the transport queue.

    // RC Attr
    attr.stRcAttr = build_rc_attr(params, framerate);
    // Gop Attr
    attr.stGopAttr.enGopMode =
        params.bframe == 0 ? AX_VENC_GOPMODE_NORMALP : AX_VENC_GOPMODE_ONELTR;

    return attr;
}
// RC Settings
AX_VENC_RC_ATTR_T build_rc_attr(const EncodeParams& params, int framerate) noexcept {
    AX_VENC_RC_ATTR_T rc{};
    rc.enRcMode                  = params.enVBR ? AX_VENC_RC_MODE_H265VBR : AX_VENC_RC_MODE_H265CBR;
    rc.s32FirstFrameStartQp      = -1; // let encoder decide
    rc.stFrameRate.fSrcFrameRate = static_cast<AX_F32>(framerate);
    rc.stFrameRate.fDstFrameRate = static_cast<AX_F32>(framerate);

    const auto bitrate_kbps = ax_rate_control_bitrate_kbps(params);
    const auto gop          = std::max<AX_U32>(1, static_cast<AX_U32>(params.gop_size));
    const auto intra_delta =
        params.chroma_qp_offset > 0 ? std::clamp(-params.chroma_qp_offset, -51, 51) : 0;

    auto apply_qpmap = [&params](AX_VENC_QPMAP_META_T& qpmap) noexcept {
        qpmap.enCtbRcMode = params.enAqMode ? AX_VENC_RC_CTBRC_QUALITY_RATE : AX_VENC_RC_CTBRC_RATE;
        if (qp_delta_map_enabled(params.filter)) {
            qpmap.enQpmapQpType    = AX_VENC_QPMAP_QP_DELTA;
            qpmap.enQpmapBlockType = AX_VENC_QPMAP_BLOCK_DISABLE;
            qpmap.enQpmapBlockUnit = AX_VENC_QPMAP_BLOCK_UNIT_16x16;
        }
    };

    if (params.enVBR) {
        auto& vbr           = rc.stH265Vbr;
        vbr.u32MaxQp        = 51;
        vbr.u32MinQp        = 10;
        vbr.u32MaxIQp       = 51;
        vbr.u32MinIQp       = 10;
        vbr.u32Gop          = gop;
        vbr.u32StatTime     = 1;       // 1-second rate statistic window
        vbr.u32MaxBitRate   = bitrate_kbps;
        vbr.s32IntraQpDelta = intra_delta;
        vbr.u32ChangePos    = VENC_DEF_CHANGE_POS;
        apply_qpmap(vbr.stQpmapInfo);
    } else {
        // Tight QP to hold the wire rate close to target.
        // Match the old x265 strict CBR behavior.
        auto& cbr           = rc.stH265Cbr;
        cbr.u32MaxQp        = 51;
        cbr.u32MinQp        = 10;
        cbr.u32MaxIQp       = 51;
        cbr.u32MinIQp       = 10;
        cbr.u32Gop          = gop;
        cbr.u32StatTime     = 1; // 1-second rate statistic window
        cbr.u32BitRate      = bitrate_kbps;
        cbr.u32MaxIprop     = 100;
        cbr.u32MinIprop     = 1;
        cbr.s32IntraQpDelta = intra_delta;
        apply_qpmap(cbr.stQpmapInfo);
    }

    return rc;
}

AX_VENC_VUI_PARAM_T build_vui_param() noexcept {
    AX_VENC_VUI_PARAM_T vui{};

    // Video signal: BT.709, limited range
    vui.stVuiVideoSignal.video_signal_type_present_flag  = 1;
    vui.stVuiVideoSignal.video_format                    = 5; // Unspecified
    vui.stVuiVideoSignal.video_full_range_flag           = 0; // Limited range
    vui.stVuiVideoSignal.colour_description_present_flag = 1;
    vui.stVuiVideoSignal.colour_primaries                = 1; // BT.709
    vui.stVuiVideoSignal.transfer_characteristics        = 1; // BT.709
    vui.stVuiVideoSignal.matrix_coefficients             = 1; // BT.709

    return vui;
}

AX_VENC_INTRA_REFRESH_T build_intra_refresh(const EncodeParams& params) noexcept {
    AX_VENC_INTRA_REFRESH_T ir{};
    ir.bRefresh           = params.intra_refresh ? AX_TRUE : AX_FALSE;
    ir.u32RefreshNum      = params.refresh_num;
    ir.u32ReqIQp          = 40;
    ir.enIntraRefreshMode = AX_VENC_INTRA_REFRESH_ROW;
    return ir;
}

// ===========================================================================
// Frame submission helpers
// ===========================================================================

AX_VIDEO_FRAME_INFO_T
make_frame_info(const Nv12Frame& nv12, int64_t pts, uint64_t seq_num) noexcept {
    AX_VIDEO_FRAME_INFO_T info{};
    info.enModId = AX_ID_USER;

    auto& frame          = info.stVFrame;
    frame.u32Width       = static_cast<AX_U32>(nv12.width);
    frame.u32Height      = static_cast<AX_U32>(nv12.height);
    frame.enImgFormat    = AX_FORMAT_YUV420_SEMIPLANAR; // NV12
    frame.enVscanFormat  = AX_VSCAN_FORMAT_RASTER;
    frame.stDynamicRange = AX_DYNAMIC_RANGE_SDR8;
    frame.stColorGamut   = AX_COLOR_GAMUT_BT709;

    // Y plane (aligned stride, CMM physical address)
    frame.u32PicStride[0] = static_cast<AX_U32>(nv12.y_stride);
    frame.u64PhyAddr[0]   = nv12.y_phy_addr();
    frame.u64VirAddr[0]   = reinterpret_cast<AX_U64>(nv12.y_data());

    // UV plane (aligned stride, same as Y for NV12, CMM physical address)
    frame.u32PicStride[1] = static_cast<AX_U32>(nv12.uv_stride);
    frame.u64PhyAddr[1]   = nv12.uv_phy_addr();
    frame.u64VirAddr[1]   = reinterpret_cast<AX_U64>(nv12.uv_data());

    // Total frame size in bytes (Y + UV, with aligned strides)
    frame.u32FrameSize = static_cast<AX_U32>(nv12.y_size() + nv12.uv_size());

    frame.u64PTS       = static_cast<AX_U64>(pts);
    frame.u64SeqNum    = seq_num;
    frame.u32FrameFlag = 0;

    return info;
}

std::expected<EncodedPacket, std::string>
    extract_packet(VencChannel& chn, AX_VENC_STREAM_T& stream) noexcept {
    const auto& pack = stream.stPack;

    EncodedPacket pkt;
    pkt.pts = static_cast<int64_t>(pack.u64PTS);
    pkt.nalus.reserve(pack.u32NaluNum);

    if (pack.u32Len > 0 && pack.pu8Addr != nullptr) {
        pkt.data = std::unique_ptr<uint8_t[]>(new uint8_t[pack.u32Len]);
        std::copy_n(pack.pu8Addr, pack.u32Len, pkt.data.get());
        pkt.size = pack.u32Len;
    } else {
        pkt.size = 0;
    }

    if (pkt.size > 0 && pkt.data != nullptr) {
        for (AX_U32 i = 0; i < pack.u32NaluNum; ++i) {
            const auto& nalu = pack.stNaluInfo[i];
            if (nalu.u32NaluLength == 0 || nalu.u32NaluOffset >= pkt.size) {
                continue;
            }

            const size_t packet_offset = static_cast<size_t>(nalu.u32NaluOffset);
            const size_t packet_size   = static_cast<size_t>(nalu.u32NaluLength);
            pkt.nalus.push_back(
                {packet_offset, packet_size, static_cast<uint8_t>(nalu.unNaluType.enH265EType)});
        }
        pkt.keyframe = (pack.enCodingType == AX_VENC_VIRTUAL_INTRA_FRAME)
                    || (pack.enCodingType == AX_VENC_INTRA_FRAME);
        if (pkt.keyframe)
            SPDLOG_DEBUG("I Frame Generated, seq = {}", pack.u64SeqNum);
    }

    // Release the stream buffer back to the hardware
    AX_S32 ret = chn.release_stream(stream);
    if (ret != AX_SUCCESS) {
        return std::unexpected(
            std::format("AX_VENC_ReleaseStream failed: {}", ax_error_string(ret)));
    }

    return pkt;
}

// ===========================================================================
// AxBackend — AX VENC hardware encoder backend
// ===========================================================================

class AxBackend final : public EncoderBackend {
public:
    static constexpr AX_S32 kSendTimeoutMs   = 30;
    static constexpr AX_S32 kStreamTimeoutMs = 5;
    // Match the official ax-video-sdk mitigation for AX650: keep a small
    // inflight window of CMM-backed input frames alive after SendFrame
    // succeeds, otherwise libax_venc may dereference released memory later.
    static constexpr std::size_t kReusableInflightDepth = 4;

    [[nodiscard]] static std::expected<std::unique_ptr<AxBackend>, std::string>
        create(EncodeParams params, int src_width, int src_height, int framerate) noexcept {
        auto backend = std::unique_ptr<AxBackend>(new AxBackend(std::move(params)));
        auto result  = backend->setup(src_width, src_height, framerate);
        if (!result)
            return std::unexpected(std::move(result.error()));
        return backend;
    }

    ~AxBackend() override {
        if (channel_) {
            drain_encoder();
        }
    }

    AxBackend(const AxBackend&)            = delete;
    AxBackend& operator=(const AxBackend&) = delete;
    AxBackend(AxBackend&&)                 = default;
    AxBackend& operator=(AxBackend&&)      = default;

    std::expected<void, std::string>
        push_frame(const uint8_t* data, int linesize, int64_t pts) noexcept override {
        Nv12Frame nv12;
        auto alloc_result = nv12.allocate(out_width_, out_height_);
        if (!alloc_result) {
            return std::unexpected(std::move(alloc_result.error()));
        }

        auto preprocess_result = preprocess_frame_ivps(data, linesize, nv12);
        if (!preprocess_result)
            return std::unexpected(std::move(preprocess_result.error()));

        // --- Submit to hardware encoder ---
        const bool force_keyframe = std::exchange(force_keyframe_requested_, false);
        if (force_keyframe) {
            (void)channel_->request_idr(true);
        }

        AX_VIDEO_FRAME_INFO_T frame_info = make_frame_info(nv12, pts, frame_seq_++);

        std::optional<QpDeltaMap> qp_delta_map;
        if (qp_delta_map_enabled(params_.filter)) {
            qp_delta_map.emplace();
            auto qp_alloc_result = qp_delta_map->allocate(out_width_, out_height_);
            if (!qp_alloc_result) {
                return std::unexpected(std::move(qp_alloc_result.error()));
            }
            fill_qp_delta_map(
                *qp_delta_map, edge_mask_, edge_mask_valid_, out_width_, out_height_,
                params_.filter);
            qp_delta_map->flush_cache();
        }

        auto send_current_frame = [&]() noexcept -> AX_S32 {
            if (qp_delta_map) {
                AX_USER_FRAME_INFO_T user_frame = make_user_frame_info(frame_info, *qp_delta_map);
                return channel_->send_frame_ex(user_frame, kSendTimeoutMs);
            }
            return channel_->send_frame(frame_info, kSendTimeoutMs);
        };

        AX_S32 ret = send_current_frame();

        if (ret == AX_ERR_VENC_QUEUE_FULL || ret == AX_ERR_VENC_BUF_FULL) {
            auto drain_result = drain_packets();
            if (!drain_result) {
                return std::unexpected(std::move(drain_result.error()));
            }
            ret = send_current_frame();
        }
        if (ret != AX_SUCCESS) {
            return std::unexpected(
                std::format("AX_VENC_SendFrame failed: {}", ax_error_string(ret)));
        }

        retain_inflight_frame(std::move(nv12), std::move(qp_delta_map));

        // --- Pull encoded packets ---
        return drain_packets();
    }

    void request_keyframe() noexcept override { force_keyframe_requested_ = true; }

    std::optional<EncodedPacket> poll_packet() noexcept override {
        // Try to drain more packets from the encoder
        auto drain_result = drain_packets();
        if (!drain_result) {
            std::fprintf(
                stderr, "[quanta::stream][ax] drain error: %s\n", drain_result.error().c_str());
        }

        if (packet_queue_.empty())
            return std::nullopt;
        auto pkt = std::move(packet_queue_.front());
        packet_queue_.pop_front();
        return pkt;
    }

    std::expected<void, std::string> flush() noexcept override {
        drain_encoder();
        return {};
    }

    const EncodeParams& params() const noexcept override { return params_; }
    std::pair<int, int> dimensions() const noexcept override { return {out_width_, out_height_}; }

private:
    explicit AxBackend(EncodeParams params)
        : params_(std::move(params)) {}

    [[nodiscard]] std::expected<void, std::string>
        copy_ive_image_to_y_plane(AX_IVE_IMAGE_T& src, Nv12Frame& dst) noexcept {
        return ive_dma_copy(
            src.au64PhyAddr[0], reinterpret_cast<void*>(src.au64VirAddr[0]), src.au32Stride[0],
            dst.y_phy_addr(), dst.y_data(), static_cast<AX_U32>(dst.y_stride),
            static_cast<AX_U32>(dst.width), static_cast<AX_U32>(dst.height),
            "AX_IVE_DMA copy luma");
    }

    [[nodiscard]] std::expected<void, std::string>
        copy_ive_image_to_uv_plane(AX_IVE_IMAGE_T& src, Nv12Frame& dst) noexcept {
        return ive_dma_copy(
            src.au64PhyAddr[0], reinterpret_cast<void*>(src.au64VirAddr[0]), src.au32Stride[0],
            dst.uv_phy_addr(), dst.uv_data(), static_cast<AX_U32>(dst.uv_stride),
            static_cast<AX_U32>(dst.width), static_cast<AX_U32>(dst.height / 2),
            "AX_IVE_DMA copy chroma");
    }

    [[nodiscard]] std::expected<void, std::string> quantize_luma(
        AX_IVE_IMAGE_T& src, AX_IVE_IMAGE_T& acc, AX_IVE_IMAGE_T&, AX_IVE_IMAGE_T&,
        IveImage& acc_owner, int levels, AX_IVE_IMAGE_T*& quantized) noexcept {
        const int clamped_levels = clamped_luma_levels(levels);
        if (clamped_levels <= 0) {
            quantized = &src;
            return {};
        }

        const int step = std::max(1, static_cast<int>(std::lround(255.0 / (clamped_levels - 1))));
        const AX_S32 invalidate_ret = invalidate_ive_u8_image(src);
        if (invalidate_ret != AX_SUCCESS) {
            return std::unexpected(
                std::format(
                    "AX_SYS_MinvalidateCache luma quant source failed: {}",
                    ax_error_string(invalidate_ret)));
        }

        auto* src_base = reinterpret_cast<const uint8_t*>(src.au64VirAddr[0]);
        auto* dst_base = acc_owner.data();
        if (src_base == nullptr || dst_base == nullptr) {
            return std::unexpected("luma quant image has null virtual address");
        }

        std::memset(dst_base, 0, acc_owner.alloc_size);
        for (AX_U32 y = 0; y < src.u32Height; ++y) {
            const auto* src_row = src_base + static_cast<size_t>(y) * src.au32Stride[0];
            auto* dst_row       = dst_base + static_cast<size_t>(y) * acc_owner.stride;
            for (AX_U32 x = 0; x < src.u32Width; ++x) {
                const int bucket = std::min(
                    (static_cast<int>(src_row[x]) * clamped_levels) / 256, clamped_levels - 1);
                dst_row[x] = static_cast<uint8_t>(std::min(bucket * step, 255));
            }
        }
        flush_ive_u8_image(acc);

        quantized = &acc;
        return {};
    }

    [[nodiscard]] std::expected<void, std::string> build_ive_edge_mask(
        AX_IVE_IMAGE_T& src, AX_IVE_IMAGE_T& lowpass, AX_IVE_IMAGE_T& diff, AX_IVE_IMAGE_T& edge,
        AX_IVE_IMAGE_T& edge_tmp) noexcept {
        AX_IVE_FILTER_CTRL_T low_pass = make_luma_gaussian_filter(
            GaussianDenoiseParams{.kernel_size = 3, .sigma_x = 0.0, .sigma_y = 0.0});
        auto filter_result = ive_filter(src, lowpass, low_pass, "AX_IVE_Filter edge low-pass");
        if (!filter_result)
            return filter_result;

        auto sub_result = ive_sub_abs(src, lowpass, diff, "AX_IVE_Sub edge magnitude");
        if (!sub_result)
            return sub_result;

        const auto& filter = params_.filter;
        auto threshold_result =
            ive_thresh_binary(diff, edge, edge_threshold(filter), 255, "AX_IVE_Thresh edge mask");
        if (!threshold_result)
            return threshold_result;

        const int width             = std::clamp(filter.contour_width, 1, 9);
        const int dilate_iterations = std::clamp((width + 1) / 4, 0, 2);
        bool final_is_edge          = true;
        for (int i = 0; i < dilate_iterations; ++i) {
            AX_IVE_IMAGE_T& dilate_src = final_is_edge ? edge : edge_tmp;
            AX_IVE_IMAGE_T& dilate_dst = final_is_edge ? edge_tmp : edge;
            auto dilate_result = ive_dilate(dilate_src, dilate_dst, "AX_IVE_Dilate edge mask");
            if (!dilate_result)
                return dilate_result;
            final_is_edge = !final_is_edge;
        }

        if (!final_is_edge) {
            auto copy_result = ive_dma_copy(
                edge_tmp.au64PhyAddr[0], reinterpret_cast<void*>(edge_tmp.au64VirAddr[0]),
                edge_tmp.au32Stride[0], edge.au64PhyAddr[0],
                reinterpret_cast<void*>(edge.au64VirAddr[0]), edge.au32Stride[0], edge.u32Width,
                edge.u32Height, "AX_IVE_DMA copy edge mask");
            if (!copy_result)
                return copy_result;
        }

        edge_mask_valid_ = true;
        return {};
    }

    [[nodiscard]] std::expected<void, std::string>
        darken_luma_edges(AX_IVE_IMAGE_T& src, AX_IVE_IMAGE_T& dst) noexcept {
        edge_full_mask_.clear(255);

        AX_IVE_IMAGE_T edge      = make_ive_u8_image(edge_mask_);
        AX_IVE_IMAGE_T full_mask = make_ive_u8_image(edge_full_mask_);
        AX_IVE_IMAGE_T inverse   = make_ive_u8_image(edge_inverse_);

        auto inverse_result = ive_xor(edge, full_mask, inverse, "AX_IVE_Xor inverse edge mask");
        if (!inverse_result)
            return inverse_result;

        return ive_and(src, inverse, dst, "AX_IVE_And darken luma edges");
    }

    [[nodiscard]] std::expected<void, std::string>
        apply_ive_luma_denoise(AX_IVE_IMAGE_T& src, AX_IVE_IMAGE_T& dst) noexcept {
        const auto& filter = params_.filter;
        if (!denoise_luma_enabled(filter))
            return {};

        AX_IVE_FILTER_CTRL_T gaussian = make_luma_gaussian_filter(filter.denoise_luma);
        return ive_filter(src, dst, gaussian, "AX_IVE_Filter luma Gaussian denoise");
    }

    [[nodiscard]] std::expected<void, std::string>
        apply_ive_chroma_denoise(Nv12Frame& dst) noexcept {
        const auto& filter = params_.filter;
        if (!denoise_chroma_enabled(filter))
            return {};

        if ((dst.height / 2) < 64) {
            return std::unexpected(
                std::format(
                    "AX IVE chroma denoise requires UV height >= 64, got {}", dst.height / 2));
        }

        AX_IVE_IMAGE_T uv             = make_ive_uv_image(dst);
        AX_IVE_IMAGE_T scratch_a      = make_ive_u8_view(ive_y_a_, dst.width, dst.height / 2);
        AX_IVE_FILTER_CTRL_T low_pass = make_chroma_gaussian_filter(filter.denoise_chroma);
        auto filter_result =
            ive_filter(uv, scratch_a, low_pass, "AX_IVE_Filter chroma Gaussian denoise");
        if (!filter_result)
            return filter_result;

        return copy_ive_image_to_uv_plane(scratch_a, dst);
    }

    [[nodiscard]] std::expected<void, std::string>
        apply_split_chroma_filters(Nv12Frame& dst) noexcept {
        const auto& filter = params_.filter;
        if (!has_chroma_plane_filters(filter))
            return {};

        const int chroma_width  = dst.width / 2;
        const int chroma_height = dst.height / 2;
        auto split_result       = split_nv12_uv_to_planes(dst, chroma_u_, chroma_v_);
        if (!split_result)
            return std::unexpected(std::move(split_result.error()));

        const IveImage* current_u = &chroma_u_;
        const IveImage* current_v = &chroma_v_;

        const double sharpen_amount = chroma_sharpen_amount(filter);
        if (positive_filter_enabled(sharpen_amount)) {
            auto sharpen_u_result = high_boost_u8_plane(
                *current_u, chroma_u_tmp_, chroma_width, chroma_height, sharpen_amount,
                "chroma U high-boost");
            if (!sharpen_u_result)
                return std::unexpected(std::move(sharpen_u_result.error()));

            auto sharpen_v_result = high_boost_u8_plane(
                *current_v, chroma_v_tmp_, chroma_width, chroma_height, sharpen_amount,
                "chroma V high-boost");
            if (!sharpen_v_result)
                return std::unexpected(std::move(sharpen_v_result.error()));

            current_u = &chroma_u_tmp_;
            current_v = &chroma_v_tmp_;
        }

        const int chroma_levels = chroma_quantization_enabled(filter) ? filter.chroma_levels : 0;
        if (chroma_levels > 0) {
            IveImage& quant_u_dst = (current_u == &chroma_u_) ? chroma_u_tmp_ : chroma_u_;
            IveImage& quant_v_dst = (current_v == &chroma_v_) ? chroma_v_tmp_ : chroma_v_;

            auto quant_u_result = quantize_u8_plane(
                *current_u, quant_u_dst, chroma_width, chroma_height, chroma_levels,
                "chroma U quantization");
            if (!quant_u_result)
                return std::unexpected(std::move(quant_u_result.error()));

            auto quant_v_result = quantize_u8_plane(
                *current_v, quant_v_dst, chroma_width, chroma_height, chroma_levels,
                "chroma V quantization");
            if (!quant_v_result)
                return std::unexpected(std::move(quant_v_result.error()));

            current_u = &quant_u_dst;
            current_v = &quant_v_dst;
        }

        return merge_chroma_planes_to_nv12_uv(*current_u, *current_v, dst);
    }

    [[nodiscard]] std::expected<void, std::string> apply_ive_filters(Nv12Frame& dst) noexcept {
        edge_mask_valid_ = false;
        if (!filter_chain_.use_ive_filters)
            return {};

        const auto& filter = params_.filter;

        AX_IVE_IMAGE_T dst_y    = make_ive_y_image(dst);
        AX_IVE_IMAGE_T y_a      = make_ive_u8_image(ive_y_a_);
        AX_IVE_IMAGE_T y_b      = make_ive_u8_image(ive_y_b_);
        AX_IVE_IMAGE_T y_c      = make_ive_u8_image(ive_y_c_);
        AX_IVE_IMAGE_T edge     = make_ive_u8_image(edge_mask_);
        AX_IVE_IMAGE_T edge_tmp = make_ive_u8_image(edge_tmp_);

        AX_IVE_IMAGE_T* current_y = &dst_y;

        const double sharpen_amount = luma_sharpen_amount(filter);

        if (denoise_luma_enabled(filter)) {
            auto denoise_result = apply_ive_luma_denoise(dst_y, y_a);
            if (!denoise_result)
                return denoise_result;
            current_y = &y_a;
        }

        if (positive_filter_enabled(sharpen_amount)) {
            AX_IVE_FILTER_CTRL_T high_boost = make_high_boost_5x5(sharpen_amount);
            AX_IVE_IMAGE_T& sharpen_dst     = (current_y == &y_a) ? y_b : y_a;
            auto sharpen_result =
                ive_filter(*current_y, sharpen_dst, high_boost, "AX_IVE_Filter luma high-boost");
            if (!sharpen_result)
                return sharpen_result;
            if (&sharpen_dst == &y_b) {
                auto copy_result = ive_dma_copy(
                    y_b.au64PhyAddr[0], reinterpret_cast<void*>(y_b.au64VirAddr[0]),
                    y_b.au32Stride[0], y_a.au64PhyAddr[0],
                    reinterpret_cast<void*>(y_a.au64VirAddr[0]), y_a.au32Stride[0], y_a.u32Width,
                    y_a.u32Height, "AX_IVE_DMA stage sharpened luma");
                if (!copy_result)
                    return copy_result;
            }
            current_y = &y_a;
        }

        const int luma_levels = luma_quantization_enabled(filter) ? filter.luma_levels : 0;
        if (luma_levels > 0 && current_y == &y_b) {
            auto copy_result = ive_dma_copy(
                y_b.au64PhyAddr[0], reinterpret_cast<void*>(y_b.au64VirAddr[0]), y_b.au32Stride[0],
                y_a.au64PhyAddr[0], reinterpret_cast<void*>(y_a.au64VirAddr[0]), y_a.au32Stride[0],
                y_a.u32Width, y_a.u32Height, "AX_IVE_DMA stage denoised luma for quantization");
            if (!copy_result)
                return copy_result;
            current_y = &y_a;
        }

        AX_IVE_IMAGE_T* quantized_y = current_y;
        auto quant_result =
            quantize_luma(*current_y, y_b, y_c, edge_tmp, ive_y_b_, luma_levels, quantized_y);
        if (!quant_result)
            return quant_result;
        current_y = quantized_y;

        if (contour_enabled(filter) || qp_delta_map_enabled(filter)) {
            AX_IVE_IMAGE_T& edge_lowpass = (current_y == &y_b) ? y_a : y_b;
            AX_IVE_IMAGE_T& edge_diff    = (current_y == &y_c) ? y_a : y_c;
            auto edge_result =
                build_ive_edge_mask(*current_y, edge_lowpass, edge_diff, edge, edge_tmp);
            if (!edge_result)
                return edge_result;
        }

        if (edge_mask_valid_ && contour_enabled(filter)) {
            AX_IVE_IMAGE_T& contour_dst = (current_y == &y_c) ? y_a : y_c;
            auto contour_result         = darken_luma_edges(*current_y, contour_dst);
            if (!contour_result)
                return contour_result;
            current_y = &contour_dst;
        }

        if (current_y != &dst_y) {
            auto copy_result = copy_ive_image_to_y_plane(*current_y, dst);
            if (!copy_result)
                return copy_result;
        }
        return {};
    }

    std::expected<void, std::string>
        preprocess_frame_ivps(const uint8_t* data, int linesize, Nv12Frame& dst) noexcept {
        if (data == nullptr)
            return std::unexpected("push_frame input is null");

        const int src_linesize = (linesize > 0) ? linesize : (src_width_ * 3);
        const size_t row_bytes = static_cast<size_t>(ivps_src_width_) * 3U;
        if (src_linesize < static_cast<int>(row_bytes)) {
            return std::unexpected(
                std::format("input linesize {} smaller than required {}", src_linesize, row_bytes));
        }

        auto* src_buffer = static_cast<uint8_t*>(ivps_input_.vir_addr);
        for (int y = 0; y < ivps_src_height_; ++y) {
            std::memcpy(
                src_buffer + static_cast<size_t>(y) * ivps_input_.stride,
                data + static_cast<size_t>(y) * src_linesize, row_bytes);
        }

        AX_VIDEO_FRAME_T src_frame = make_ivps_src_frame(ivps_input_);
        AX_VIDEO_FRAME_T dst_frame = make_ivps_dst_frame(dst);
        dst.flush_cache();

        AX_S32 ret = AX_SUCCESS;
        if (filter_chain_.stage_resize_then_csc) {
            AX_VIDEO_FRAME_T resized_bgr = make_ivps_src_frame(ivps_intermediate_);
            ret = ivps_crop_resize(&src_frame, &resized_bgr, &filter_chain_.aspect_ratio);
            if (ret == AX_SUCCESS) {
                ret = ivps_csc(&resized_bgr, &dst_frame);
            }
        } else {
            ret = ivps_csc(&src_frame, &dst_frame);
        }
        if (ret != AX_SUCCESS) {
            return std::unexpected(
                std::format(
                    "AX IVPS filter chain failed: {}, src={}x{} stride={}, dst={}x{} "
                    "strideY={} strideUV={}, staged={}",
                    ax_error_string(ret), src_frame.u32Width, src_frame.u32Height,
                    src_frame.u32PicStride[0], dst_frame.u32Width, dst_frame.u32Height,
                    dst_frame.u32PicStride[0], dst_frame.u32PicStride[1],
                    filter_chain_.stage_resize_then_csc ? 1 : 0));
        }

        auto ive_result = apply_ive_filters(dst);
        if (!ive_result)
            return std::unexpected(std::move(ive_result.error()));

        auto chroma_denoise_result = apply_ive_chroma_denoise(dst);
        if (!chroma_denoise_result)
            return std::unexpected(std::move(chroma_denoise_result.error()));

        auto split_chroma_result = apply_split_chroma_filters(dst);
        if (!split_chroma_result)
            return std::unexpected(std::move(split_chroma_result.error()));

        return {};
    }

    std::expected<void, std::string> setup(int src_width, int src_height, int framerate) noexcept {
        src_width_  = src_width;
        src_height_ = src_height;
        framerate_  = framerate;

        int out_w{}, out_h{};
        fit_dimensions(src_width, src_height, params_.max_width, params_.max_height, out_w, out_h);
        const bool ive_filters_requested = has_ive_image_filters(params_.filter);
        if (ive_filters_requested) {
            auto ive_dimensions = ive_filter_dimensions(out_w, out_h);
            if (!ive_dimensions)
                return std::unexpected(std::move(ive_dimensions.error()));

            const auto [ive_w, ive_h] = *ive_dimensions;
            if (ive_w != out_w || ive_h != out_h) {
                std::fprintf(
                    stderr,
                    "[quanta::stream][ax] adjusted IVE filter output geometry from %dx%d to "
                    "%dx%d for 16-pixel hardware alignment\n",
                    out_w, out_h, ive_w, ive_h);
                out_w = ive_w;
                out_h = ive_h;
            }
        }
        out_width_  = out_w;
        out_height_ = out_h;

        auto module_result = VencModule::acquire(AX_VENC_VIDEO_ENCODER);
        if (!module_result)
            return std::unexpected(std::move(module_result.error()));
        module_ = std::move(*module_result);

        auto filter_chain_result =
            build_ax_filter_chain(params_.filter, src_width, src_height, out_w, out_h);
        if (!filter_chain_result)
            return std::unexpected(std::move(filter_chain_result.error()));
        filter_chain_ = std::move(*filter_chain_result);

        ivps_src_width_  = src_width;
        ivps_src_height_ = src_height;
        if (ivps_src_width_ <= 0 || ivps_src_height_ <= 0) {
            return std::unexpected(
                std::format("invalid IVPS source dimensions: {}x{}", src_width, src_height));
        }
        auto ivps_alloc_result = ivps_input_.allocate(ivps_src_width_, ivps_src_height_);
        if (!ivps_alloc_result)
            return std::unexpected(std::move(ivps_alloc_result.error()));

        if (filter_chain_.stage_resize_then_csc) {
            auto intermediate_result = ivps_intermediate_.allocate(out_w, out_h);
            if (!intermediate_result)
                return std::unexpected(std::move(intermediate_result.error()));
        }

        if (filter_chain_.use_ive_filters) {
            auto y_a_result = ive_y_a_.allocate_u8(out_w, out_h, "QuantaIveYA");
            if (!y_a_result)
                return std::unexpected(std::move(y_a_result.error()));
            auto y_b_result = ive_y_b_.allocate_u8(out_w, out_h, "QuantaIveYB");
            if (!y_b_result)
                return std::unexpected(std::move(y_b_result.error()));
            auto y_c_result = ive_y_c_.allocate_u8(out_w, out_h, "QuantaIveYC");
            if (!y_c_result)
                return std::unexpected(std::move(y_c_result.error()));
            auto edge_result = edge_mask_.allocate_u8(out_w, out_h, "QuantaIveEdge");
            if (!edge_result)
                return std::unexpected(std::move(edge_result.error()));
            auto edge_tmp_result = edge_tmp_.allocate_u8(out_w, out_h, "QuantaIveEdgeTmp");
            if (!edge_tmp_result)
                return std::unexpected(std::move(edge_tmp_result.error()));
            auto full_mask_result = edge_full_mask_.allocate_u8(out_w, out_h, "QuantaIveFullMask");
            if (!full_mask_result)
                return std::unexpected(std::move(full_mask_result.error()));
            auto inverse_result = edge_inverse_.allocate_u8(out_w, out_h, "QuantaIveEdgeInv");
            if (!inverse_result)
                return std::unexpected(std::move(inverse_result.error()));
        }

        if (filter_chain_.use_split_chroma) {
            const int chroma_w = out_w / 2;
            const int chroma_h = out_h / 2;
            auto u_result      = chroma_u_.allocate_u8(chroma_w, chroma_h, "QuantaChromaU");
            if (!u_result)
                return std::unexpected(std::move(u_result.error()));
            auto v_result = chroma_v_.allocate_u8(chroma_w, chroma_h, "QuantaChromaV");
            if (!v_result)
                return std::unexpected(std::move(v_result.error()));
            auto u_tmp_result = chroma_u_tmp_.allocate_u8(chroma_w, chroma_h, "QuantaChromaUTmp");
            if (!u_tmp_result)
                return std::unexpected(std::move(u_tmp_result.error()));
            auto v_tmp_result = chroma_v_tmp_.allocate_u8(chroma_w, chroma_h, "QuantaChromaVTmp");
            if (!v_tmp_result)
                return std::unexpected(std::move(v_tmp_result.error()));
        }

        auto chn_attr_result = build_hevc_channel_attr(params_, out_w, out_h);
        if (!chn_attr_result)
            return std::unexpected(std::move(chn_attr_result.error()));
        AX_VENC_CHN_ATTR_T chn_attr = std::move(*chn_attr_result);

        auto channel_result = VencChannel::create(module_, chn_attr);
        if (!channel_result)
            return std::unexpected(std::move(channel_result.error()));
        channel_                = std::move(*channel_result);
        AX_VENC_VUI_PARAM_T vui = build_vui_param();
        AX_S32 ret              = channel_->set_vui_param(vui);
        if (ret != AX_SUCCESS) {
            return std::unexpected(
                std::format("AX_VENC_SetVuiParam failed: {}", ax_error_string(ret)));
        }
        std::expected<AX_VENC_RC_PARAM_T, std::string> rc_param_result = channel_->get_rc_param();
        if (!rc_param_result)
            return std::unexpected(std::move(rc_param_result.error()));
        AX_VENC_RC_PARAM_T rc_param                     = std::move(*rc_param_result);
        rc_param.stSceneChangeDetect.bDetectSceneChange = params_.enScenecut ? AX_TRUE : AX_FALSE;
        rc_param.stSceneChangeDetect.bAdaptiveInsertIDRFrame =
            params_.enScenecut ? AX_TRUE : AX_FALSE;
        ret = channel_->set_rc_param(rc_param);
        if (ret != AX_SUCCESS) {
            return std::unexpected(
                std::format("AX_VENC_SetRcParam failed: {}", ax_error_string(ret)));
        }

        if (params_.intra_refresh) {
            AX_VENC_INTRA_REFRESH_T ir = build_intra_refresh(params_);
            ret                        = channel_->set_intra_refresh(ir);
            if (ret != AX_SUCCESS) {
                std::fprintf(
                    stderr,
                    "[quanta::stream][ax] AX_VENC_SetIntraRefresh rejected; continuing without "
                    "intra refresh: %s\n",
                    ax_error_string(ret).c_str());
                intra_refresh_enabled_ = false;
            } else {
                intra_refresh_enabled_ = true;
            }
        }

        AX_VENC_RECV_PIC_PARAM_T recv_param{};
        recv_param.s32RecvPicNum = -1;
        ret                      = channel_->start_recv_frame(recv_param);
        if (ret != AX_SUCCESS) {
            return std::unexpected(
                std::format("AX_VENC_StartRecvFrame failed: {}", ax_error_string(ret)));
        }

        const auto rc_bitrate_bps = ax_rate_control_bitrate_bps(params_);
        std::fprintf(
            stderr,
            "[quanta::stream][ax] %dx%d @ %dfps -> %dx%d HEVC, rc=%s, bitrate=%lld bps, "
            "target=%d bps, max=%d bps, ch=%d, intra_refresh=%d, ivps_src=%dx%d stride=%u, "
            "staged_ivps=%d, "
            "ive_filters=%d, chroma_filters=%d, luma_levels=%d, chroma_levels=%d\n",
            src_width, src_height, framerate, out_w, out_h, ax_rc_mode_name(params_),
            static_cast<long long>(rc_bitrate_bps), params_.target_bitrate, params_.max_bit_rate,
            static_cast<int>(channel_->id()), intra_refresh_enabled_ ? 1 : 0, ivps_src_width_,
            ivps_src_height_, ivps_input_.stride, filter_chain_.stage_resize_then_csc ? 1 : 0,
            filter_chain_.use_ive_filters ? 1 : 0, has_chroma_plane_filters(params_.filter) ? 1 : 0,
            clamped_luma_levels(
                luma_quantization_enabled(params_.filter) ? params_.filter.luma_levels : 0),
            chroma_quantization_enabled(params_.filter) ? params_.filter.chroma_levels : 0);

        return {};
    }

    std::expected<void, std::string> drain_packets() noexcept {
        while (true) {
            AX_VENC_STREAM_T stream{};
            AX_S32 ret = channel_->get_stream(stream, kStreamTimeoutMs);
            if (ret != AX_SUCCESS)
                break;

            auto pkt_result = extract_packet(*channel_, stream);
            if (!pkt_result)
                return std::unexpected(std::move(pkt_result.error()));
            export_parameter_sets_once(*pkt_result);
            auto strip_result = strip_parameter_sets(*pkt_result);
            if (!strip_result) {
                return std::unexpected(std::move(strip_result.error()));
            }
            if (pkt_result->size > 0) {
                packet_queue_.push_back(std::move(*pkt_result));
            }
        }
        return {};
    }

    void drain_encoder() noexcept {
        if (!channel_.has_value())
            return;

        (void)channel_->stop_recv_frame();

        constexpr int kDrainTimeoutMs   = 200;
        constexpr int kMaxDrainAttempts = 10;
        for (int i = 0; i < kMaxDrainAttempts; ++i) {
            AX_VENC_STREAM_T stream{};
            AX_S32 ret = channel_->get_stream(stream, kDrainTimeoutMs);
            if (ret != AX_SUCCESS)
                break;

            auto pkt_result = extract_packet(*channel_, stream);
            if (pkt_result) {
                export_parameter_sets_once(*pkt_result);
                auto strip_result = strip_parameter_sets(*pkt_result);
                if (!strip_result) {
                    SPDLOG_WARN("strip VPS/SPS/PPS during drain: {}", strip_result.error());
                    continue;
                }
                if (pkt_result->size > 0) {
                    packet_queue_.push_back(std::move(*pkt_result));
                }
            }
        }

        inflight_frames_.clear();
    }

    struct InflightFrame {
        Nv12Frame nv12;
        std::optional<QpDeltaMap> qp_delta_map;
    };

    void retain_inflight_frame(
        Nv12Frame&& frame, std::optional<QpDeltaMap>&& qp_delta_map) noexcept {
        inflight_frames_.push_back(
            InflightFrame{.nv12 = std::move(frame), .qp_delta_map = std::move(qp_delta_map)});
        while (inflight_frames_.size() > kReusableInflightDepth) {
            inflight_frames_.pop_front();
        }
    }

    [[nodiscard]] std::expected<void, std::string>
        strip_parameter_sets(EncodedPacket& pkt) noexcept {
        const auto result = strip_hevc_parameter_sets(pkt);
        if (!result) {
            return std::unexpected(std::move(result.error()));
        }
        if (*result) {}
        return {};
    }

    void export_parameter_sets_once(const EncodedPacket& pkt) noexcept {
        const bool was_exported = parameter_set_state_.exported;
        const auto result       = export_hevc_parameter_sets_once(pkt, parameter_set_state_);
        if (!result) {
            SPDLOG_WARN("export VPS/SPS/PPS: {}", result.error());
            return;
        }
        if (!was_exported && parameter_set_state_.exported)
            SPDLOG_INFO("VPS/SPS/PPS exported to output/quanta_vps_sps_pps.hevc");
    }

    EncodeParams params_;

    int src_width_  = 0;
    int src_height_ = 0;
    int out_width_  = 0;
    int out_height_ = 0;
    int framerate_  = 0;

    std::shared_ptr<VencModule> module_;
    std::optional<VencChannel> channel_;
    AxFilterChain filter_chain_;
    IvpsInputFrame ivps_input_;
    IvpsInputFrame ivps_intermediate_;
    IveImage ive_y_a_;
    IveImage ive_y_b_;
    IveImage ive_y_c_;
    IveImage edge_mask_;
    IveImage edge_tmp_;
    IveImage edge_full_mask_;
    IveImage edge_inverse_;
    IveImage chroma_u_;
    IveImage chroma_v_;
    IveImage chroma_u_tmp_;
    IveImage chroma_v_tmp_;
    int ivps_src_width_  = 0;
    int ivps_src_height_ = 0;

    std::deque<EncodedPacket> packet_queue_;
    std::deque<InflightFrame> inflight_frames_;
    uint64_t frame_seq_            = 0;
    bool force_keyframe_requested_ = false;
    bool intra_refresh_enabled_    = false;
    bool edge_mask_valid_          = false;

    HevcParameterSetExportState parameter_set_state_;
};

// ===========================================================================
// Factory
// ===========================================================================

std::expected<std::unique_ptr<EncoderBackend>, std::string>
    create_ax_backend(EncodeParams params, int src_width, int src_height, int framerate) noexcept {
    auto backend = AxBackend::create(std::move(params), src_width, src_height, framerate);
    if (!backend)
        return std::unexpected(std::move(backend.error()));
    auto derived = std::move(*backend);
    return std::unique_ptr<EncoderBackend>(derived.release());
}

} // namespace quanta
