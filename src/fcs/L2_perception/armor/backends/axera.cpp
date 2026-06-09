#include "L2_perception/armor/backends/axera.hpp"

#include "L2_perception/armor/config.hpp"
#include "core/armor_types.hpp"
#include "core/types.hpp"
#include "quanta/encode_backend/ax/ax_venc_raii.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace fcs::L2 {

namespace {

namespace {
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

    return fmt::format(
        "{}/{} {}(0x{:02X}) [raw=0x{:08X}]", ax_module_name(static_cast<int>(module)), sub_module,
        ax_err_name(static_cast<int>(err_id)), err_id, code);
}

// 快速 sigmoid（直接用 std::exp，但提前过滤减少调用次数）
inline float fast_sigmoid(float x) noexcept {
    if (x > 6.0f)
        return 1.0f;
    if (x < -6.0f)
        return 0.0f;
    return 1.0f / (1.0f + std::exp(-x));
}

constexpr int AX_CMM_ALIGN_SIZE           = 128;
constexpr const char* AX_CMM_SESSION_NAME = "ax-samples-cmm";
constexpr float GRID_OFFSET               = 0.5f;
constexpr float FOUR_THIRDS               = 4.0f / 3.0f;
constexpr float FOUR_THIRDS_TOLERANCE     = 0.01f;
constexpr std::array<int, 4> KPT_ORDER    = {0, 3, 2, 1};

enum class AX_ENGINE_ALLOC_BUFFER_STRATEGY_T {
    DEFAULT = 0,
    CACHED  = 1,
};

static constexpr std::array<ArmorName, 12> PAIR_TO_ARMOR{
    ArmorName::Sentry,   // 0: Gs
    ArmorName::Two,      // 1: 2s
    ArmorName::Three,    // 2: 3s
    ArmorName::Four,     // 3: 4s
    ArmorName::Five,     // 4: 5s
    ArmorName::Outpost,  // 5: Os
    ArmorName::Sentry,   // 6: Gb
    ArmorName::One,      // 7: 1b
    ArmorName::Three,    // 8: 3b
    ArmorName::Four,     // 9: 4b
    ArmorName::Five,     // 10: 5b
    ArmorName::BaseLarge // 11: Bb
};

// ====================================================================
// AXERA NPU BACKEND: MANUAL MEMORY MANAGEMENT (C API INTEROP)
// ====================================================================
//
// PROVENANCE: Hardware NPU inference on AX650 chip
//
// CONSTRAINT: Axera SDK C API requires C arrays (cannot use std::vector)
//
//   typedef struct AX_ENGINE_IO_T {
//       AX_ENGINE_IO_BUFFER_T* pInputs;   // ← C array required
//       uint32_t nInputSize;
//       AX_ENGINE_IO_BUFFER_T* pOutputs;  // ← C array required
//       uint32_t nOutputSize;
//   } AX_ENGINE_IO_T;
//
// WHY NOT std::vector?
//   - C structs cannot contain C++ objects with non-trivial constructors
//   - std::vector has different memory layout than C array
//   - Passing std::vector to C API is UB (undefined behavior)
//
// OWNERSHIP MODEL:
//   - AxeraBackend class owns io_data_ (RAII)
//   - Destructor calls free_io() which calls delete[]
//   - Move-only semantics prevent double-delete
//   - Error paths clean up partial allocations
//
// EXCEPTION SAFETY:
//   - ✅ Most error paths properly clean up
//   - ⚠️ Rare case: if second new[] throws, first array leaks (only on OOM)
//
// VERDICT: Manual new[] is ACCEPTABLE here due to C API constraint
//
// ====================================================================
// Cleanup helper: Free hardware memory and C arrays
void free_io_index(AX_ENGINE_IO_BUFFER_T* io_buf, size_t index) noexcept {
    for (size_t i = 0; i < index; ++i) {
        AX_ENGINE_IO_BUFFER_T* p_buf = io_buf + i;
        AX_SYS_MemFree(p_buf->phyAddr, p_buf->pVirAddr);
    }
}

void free_io(AX_ENGINE_IO_T* io) noexcept {
    if (io->pInputs != nullptr) {
        for (size_t j = 0; j < io->nInputSize; ++j) {
            AX_ENGINE_IO_BUFFER_T* p_buf = io->pInputs + j;
            AX_SYS_MemFree(p_buf->phyAddr, p_buf->pVirAddr);
        }
        delete[] io->pInputs;  // ← Delete C array (not vector!)
    }
    if (io->pOutputs != nullptr) {
        for (size_t j = 0; j < io->nOutputSize; ++j) {
            AX_ENGINE_IO_BUFFER_T* p_buf = io->pOutputs + j;
            AX_SYS_MemFree(p_buf->phyAddr, p_buf->pVirAddr);
        }
        delete[] io->pOutputs; // ← Delete C array (not vector!)
    }
    *io = {};
}

// ====================================================================
// ALLOCATION STRATEGY: new[] for C API interop
// See provenance documentation in free_io() above (lines 98-124)
// ====================================================================
std::expected<void, std::string> prepare_io(
    AX_ENGINE_IO_INFO_T* info, AX_ENGINE_IO_T* io_data,
    [[maybe_unused]] std::pair<AX_ENGINE_ALLOC_BUFFER_STRATEGY_T, AX_ENGINE_ALLOC_BUFFER_STRATEGY_T>
        strategy) noexcept {
    *io_data         = {};
    io_data->pInputs = new AX_ENGINE_IO_BUFFER_T[info->nInputSize];
    memset(io_data->pInputs, 0, sizeof(AX_ENGINE_IO_BUFFER_T) * info->nInputSize);
    io_data->nInputSize = info->nInputSize;

    for (int i = 0; i < static_cast<int>(info->nInputSize); ++i) {
        auto meta     = info->pInputs[i];
        auto* buffer  = &io_data->pInputs[i];
        buffer->nSize = meta.nSize;

        // Input buffer: non-cached to eliminate AX_SYS_MflushCache syscall jitter.
        // NPU reads via DMA (physical address), IVPS TDP writes via DMA — both bypass cache.
        // CPU fallback (cv::resize) writes directly to DDR; slight throughput loss but no
        // unpredictable syscall latency.  p50 may rise ~0.5ms but p99 drops 2-3ms.
        const auto ret = AX_SYS_MemAlloc(
            reinterpret_cast<AX_U64*>(&buffer->phyAddr), &buffer->pVirAddr, meta.nSize,
            AX_CMM_ALIGN_SIZE, reinterpret_cast<const AX_S8*>(AX_CMM_SESSION_NAME));
        if (ret != 0) {
            free_io_index(io_data->pInputs, i);
            delete[] io_data->pInputs;
            io_data->pInputs = nullptr;
            return std::unexpected(
                "Axera NPU input buffer allocation failed for index " + std::to_string(i));
        }
    }

    io_data->pOutputs = new AX_ENGINE_IO_BUFFER_T[info->nOutputSize];
    memset(io_data->pOutputs, 0, sizeof(AX_ENGINE_IO_BUFFER_T) * info->nOutputSize);
    io_data->nOutputSize = info->nOutputSize;

    for (int i = 0; i < static_cast<int>(info->nOutputSize); ++i) {
        auto meta     = info->pOutputs[i];
        auto* buffer  = &io_data->pOutputs[i];
        buffer->nSize = meta.nSize;

        const auto ret = AX_SYS_MemAllocCached(
            reinterpret_cast<AX_U64*>(&buffer->phyAddr), &buffer->pVirAddr, meta.nSize,
            AX_CMM_ALIGN_SIZE, reinterpret_cast<const AX_S8*>(AX_CMM_SESSION_NAME));
        if (ret != 0) {
            free_io_index(io_data->pInputs, io_data->nInputSize);
            free_io_index(io_data->pOutputs, i);
            delete[] io_data->pInputs;
            delete[] io_data->pOutputs;
            io_data->pInputs  = nullptr;
            io_data->pOutputs = nullptr;
            return std::unexpected(
                "Axera NPU output buffer allocation failed for index " + std::to_string(i));
        }
    }

    // Enable NPU parallel run: overlap DMA input transfer with compute on previous sub-op.
    // May reduce effective inference latency; ignored by firmware versions that don't support it.
    io_data->nParallelRun = 1;

    return {};
}

bool read_file(const std::string& path, std::vector<char>& data) {
    std::fstream fs(path, std::ios::in | std::ios::binary);
    if (!fs.is_open()) {
        return false;
    }

    fs.seekg(std::ios::end);
    const auto fs_end = fs.tellg();
    fs.seekg(std::ios::beg);
    const auto fs_beg = fs.tellg();

    const auto file_size = static_cast<size_t>(fs_end - fs_beg);
    data.reserve(file_size);
    data.insert(data.end(), std::istreambuf_iterator<char>(fs), std::istreambuf_iterator<char>());
    return true;
}

constexpr float clampf(float value, float lo, float hi) noexcept {
    return value < lo ? lo : (value > hi ? hi : value);
}

bool is_four_three(int width, int height) noexcept {
    if (width <= 0 || height <= 0) {
        return false;
    }
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    return std::fabs(aspect - FOUR_THIRDS) <= FOUR_THIRDS_TOLERANCE;
}

// O(1) bounding rect from 4 corner points — avoids cv::boundingRect allocation
cv::Rect2f bounding_rect_4pt(const std::array<cv::Point2f, 4>& pts) noexcept {
    const float x_min = std::min({pts[0].x, pts[1].x, pts[2].x, pts[3].x});
    const float x_max = std::max({pts[0].x, pts[1].x, pts[2].x, pts[3].x});
    const float y_min = std::min({pts[0].y, pts[1].y, pts[2].y, pts[3].y});
    const float y_max = std::max({pts[0].y, pts[1].y, pts[2].y, pts[3].y});
    return {x_min, y_min, x_max - x_min, y_max - y_min};
}

float pts_iou(const std::array<cv::Point2f, 4>& a, const std::array<cv::Point2f, 4>& b) noexcept {
    const cv::Rect2f ra = bounding_rect_4pt(a);
    const cv::Rect2f rb = bounding_rect_4pt(b);
    const float inter   = (ra & rb).area();
    if (inter <= 0.0f) {
        return 0.0f;
    }
    const float union_area = ra.area() + rb.area() - inter;
    return union_area > 0.0f ? inter / union_area : 0.0f;
}

int clamp_int(int value, int lo, int hi) noexcept {
    return value < lo ? lo : (value > hi ? hi : value);
}

AX_U32 align_up_u32(AX_U32 value, AX_U32 alignment) noexcept {
    if (alignment == 0U) {
        return value;
    }
    return ((value + alignment - 1U) / alignment) * alignment;
}

AX_U32 bgr888_stride_aligned(int width) noexcept {
    // Same convention as ax-video-sdk: RGB/BGR24 uses 16-pixel alignment,
    // i.e. 16 * 3 = 48 bytes, so stride remains divisible by 3.
    constexpr AX_U32 ALIGN = 48U;
    return align_up_u32(static_cast<AX_U32>(std::max(width, 0)) * 3U, ALIGN);
}

// Drop-in cpp-local IVPS destination buffer. This avoids changing axera.hpp while
// keeping the IVPS image-frame buffer separate from the NPU tensor buffer.
// Assumption: one AxeraBackend instance is active, which matches the current backend usage.
AX_U64 g_ivps_dst_phy_addr    = 0;
void* g_ivps_dst_vir_addr     = nullptr;
AX_U32 g_ivps_dst_buffer_size = 0;
AX_U32 g_ivps_dst_stride      = 0;
int g_ivps_dst_width          = 0;
int g_ivps_dst_height         = 0;

void free_ivps_dst_buffer() noexcept {
    if (g_ivps_dst_vir_addr != nullptr && g_ivps_dst_phy_addr != 0) {
        AX_SYS_MemFree(g_ivps_dst_phy_addr, g_ivps_dst_vir_addr);
    }
    g_ivps_dst_phy_addr    = 0;
    g_ivps_dst_vir_addr    = nullptr;
    g_ivps_dst_buffer_size = 0;
    g_ivps_dst_stride      = 0;
    g_ivps_dst_width       = 0;
    g_ivps_dst_height      = 0;
}

std::expected<void, std::string> allocate_ivps_dst_buffer(int width, int height) noexcept {
    if (width <= 0 || height <= 0) {
        return std::unexpected(fmt::format("invalid IVPS dst size: {}x{}", width, height));
    }

    const AX_U32 stride = bgr888_stride_aligned(width);
    const AX_U32 size   = stride * static_cast<AX_U32>(height);

    if (g_ivps_dst_vir_addr != nullptr && g_ivps_dst_width == width && g_ivps_dst_height == height
        && g_ivps_dst_stride == stride && g_ivps_dst_buffer_size >= size) {
        return {};
    }

    free_ivps_dst_buffer();

    constexpr AX_U32 ALIGN = 0x1000;
    const auto ret         = AX_SYS_MemAlloc(
        &g_ivps_dst_phy_addr, &g_ivps_dst_vir_addr, size, ALIGN,
        reinterpret_cast<const AX_S8*>("ivps-dst"));

    if (ret != AX_SUCCESS || g_ivps_dst_vir_addr == nullptr) {
        g_ivps_dst_phy_addr    = 0;
        g_ivps_dst_vir_addr    = nullptr;
        g_ivps_dst_buffer_size = 0;
        g_ivps_dst_stride      = 0;
        g_ivps_dst_width       = 0;
        g_ivps_dst_height      = 0;
        return std::unexpected(fmt::format("AX_SYS_MemAlloc ivps-dst: {}", ax_error_string(ret)));
    }

    g_ivps_dst_buffer_size = size;
    g_ivps_dst_stride      = stride;
    g_ivps_dst_width       = width;
    g_ivps_dst_height      = height;

    SPDLOG_INFO("IVPS dst buffer allocated: {}x{} stride={} size={}", width, height, stride, size);
    return {};
}

} // namespace

AxeraBackend::AxeraBackend(Config config) noexcept
    : config_(std::move(config)) {}

AxeraBackend::~AxeraBackend() {
    if (handle_ != nullptr) {
        free_io(&io_data_);
        AX_ENGINE_DestroyHandle(handle_);
        handle_ = nullptr;
        AX_ENGINE_Deinit();
    }
    deinit_ivps();
    ax_sys_module_.reset();
    initialized_ = false;
}

AxeraBackend::AxeraBackend(AxeraBackend&& other) noexcept
    : cached_grid_(std::move(other.cached_grid_))
    , postproc_ranked_buf_(std::move(other.postproc_ranked_buf_))
    , postproc_candidates_buf_(std::move(other.postproc_candidates_buf_))
    , nms_removed_buf_(std::move(other.nms_removed_buf_))
    , nms_tile_bins_buf_(std::move(other.nms_tile_bins_buf_))
    , config_(std::move(other.config_))
    , initialized_(other.initialized_)
    , warned_non_four_three_input_(other.warned_non_four_three_input_)
    , ax_sys_module_(std::move(other.ax_sys_module_))
    , handle_(other.handle_)
    , io_data_(other.io_data_)
    , model_buffer_(std::move(other.model_buffer_))
    , ivps_initialized_(other.ivps_initialized_)
    , ivps_src_phy_addr_(other.ivps_src_phy_addr_)
    , ivps_src_vir_addr_(other.ivps_src_vir_addr_)
    , ivps_src_buffer_size_(other.ivps_src_buffer_size_)
    , ivps_src_stride_(other.ivps_src_stride_)
    , ivps_max_src_width_(other.ivps_max_src_width_)
    , ivps_max_src_height_(other.ivps_max_src_height_)
    , ivps_aspect_ratio_(other.ivps_aspect_ratio_) {
    other.initialized_                 = false;
    other.warned_non_four_three_input_ = false;
    other.handle_                      = nullptr;
    other.io_data_                     = {};
    other.model_buffer_.clear();
    other.ivps_initialized_     = false;
    other.ivps_src_phy_addr_    = 0;
    other.ivps_src_vir_addr_    = nullptr;
    other.ivps_src_buffer_size_ = 0;
    other.ivps_src_stride_      = 0;
}

AxeraBackend& AxeraBackend::operator=(AxeraBackend&& other) noexcept {
    if (this == &other) {
        return *this;
    }

    if (handle_ != nullptr) {
        free_io(&io_data_);
        AX_ENGINE_DestroyHandle(handle_);
        AX_ENGINE_Deinit();
    }
    deinit_ivps();
    ax_sys_module_.reset();
    initialized_ = false;

    cached_grid_                 = std::move(other.cached_grid_);
    postproc_ranked_buf_         = std::move(other.postproc_ranked_buf_);
    postproc_candidates_buf_     = std::move(other.postproc_candidates_buf_);
    nms_removed_buf_             = std::move(other.nms_removed_buf_);
    nms_tile_bins_buf_           = std::move(other.nms_tile_bins_buf_);
    config_                      = std::move(other.config_);
    initialized_                 = other.initialized_;
    warned_non_four_three_input_ = other.warned_non_four_three_input_;
    ax_sys_module_               = std::move(other.ax_sys_module_);
    handle_                      = other.handle_;
    io_data_                     = other.io_data_;
    model_buffer_                = std::move(other.model_buffer_);
    ivps_initialized_            = other.ivps_initialized_;
    ivps_src_phy_addr_           = other.ivps_src_phy_addr_;
    ivps_src_vir_addr_           = other.ivps_src_vir_addr_;
    ivps_src_buffer_size_        = other.ivps_src_buffer_size_;
    ivps_src_stride_             = other.ivps_src_stride_;
    ivps_max_src_width_          = other.ivps_max_src_width_;
    ivps_max_src_height_         = other.ivps_max_src_height_;
    ivps_aspect_ratio_           = other.ivps_aspect_ratio_;

    other.initialized_                 = false;
    other.warned_non_four_three_input_ = false;
    other.handle_                      = nullptr;
    other.io_data_                     = {};
    other.model_buffer_.clear();
    other.ivps_initialized_     = false;
    other.ivps_src_phy_addr_    = 0;
    other.ivps_src_vir_addr_    = nullptr;
    other.ivps_src_buffer_size_ = 0;
    other.ivps_src_stride_      = 0;
    return *this;
}

// ============================================================================
// Factory — Construction IS Initialization
// ============================================================================

std::expected<AxeraBackend, std::string> AxeraBackend::create(Config config) noexcept {
    AxeraBackend backend(std::move(config));
    const auto& cfg = backend.get_config();
    if (!std::filesystem::exists(cfg.model_path)) {
        return std::unexpected(fmt::format("axmodel not found: {}", cfg.model_path));
    }

    auto init_result = backend.init();
    if (!init_result) {
        return std::unexpected(init_result.error());
    }

    return backend;
}

std::expected<void, std::string> AxeraBackend::init() noexcept {
    if (initialized_) {
        return {};
    }

    const auto& cfg = get_config();
    if (!std::filesystem::exists(cfg.model_path)) {
        return std::unexpected(fmt::format("axmodel not found: {}", cfg.model_path));
    }

    bool engine_initialized  = false;
    auto cleanup_failed_init = [&]() {
        if (io_data_.pInputs != nullptr || io_data_.pOutputs != nullptr) {
            free_io(&io_data_);
            io_data_ = {};
        }
        if (handle_ != nullptr) {
            AX_ENGINE_DestroyHandle(handle_);
            handle_ = nullptr;
        }
        if (engine_initialized) {
            AX_ENGINE_Deinit();
            engine_initialized = false;
        }
        deinit_ivps();
        ax_sys_module_.reset();
        initialized_ = false;
    };

    auto ax_sys_module = quanta::AxSysModule::acquire();
    if (!ax_sys_module) {
        return std::unexpected(std::move(ax_sys_module.error()));
    }
    ax_sys_module_ = std::move(*ax_sys_module);

    auto ivps = init_ivps();
    if (!ivps) {
        SPDLOG_WARN("init_ivps: {}, falling back to OpenCV preprocessing", ivps.error());
    } else {
        SPDLOG_INFO("IVPS initialized for hardware-accelerated resize");
    }

    AX_ENGINE_NPU_ATTR_T npu_attr{};
    npu_attr.eHardMode = AX_ENGINE_VIRTUAL_NPU_DISABLE;
    auto ret           = AX_ENGINE_Init(&npu_attr);
    if (ret != 0) {
        cleanup_failed_init();
        return std::unexpected(fmt::format("AX_ENGINE_Init: {}", ax_error_string(ret)));
    }
    engine_initialized = true;

    if (!read_file(cfg.model_path, model_buffer_)) {
        cleanup_failed_init();
        return std::unexpected(fmt::format("read {}: {}", cfg.model_path, ax_error_string(ret)));
    }

    ret = AX_ENGINE_CreateHandle(&handle_, model_buffer_.data(), model_buffer_.size());
    if (ret != 0) {
        cleanup_failed_init();
        return std::unexpected(fmt::format("AX_ENGINE_CreateHandle: {}", ax_error_string(ret)));
    }

    ret = AX_ENGINE_CreateContext(handle_);
    if (ret != 0) {
        cleanup_failed_init();
        return std::unexpected(fmt::format("AX_ENGINE_CreateContext: {}", ax_error_string(ret)));
    }

    AX_ENGINE_IO_INFO_T* io_info = nullptr;
    ret                          = AX_ENGINE_GetIOInfo(handle_, &io_info);
    if (ret != 0) {
        cleanup_failed_init();
        return std::unexpected(fmt::format("AX_ENGINE_GetIOInfo: {}", ax_error_string(ret)));
    }

    SPDLOG_INFO("axmodel: {}", cfg.model_path);
    const int predecode_output_dim = 4 + cfg.num_colors + cfg.num_pairs + cfg.num_kpts * 2;
    int expected_anchors           = 0;
    for (const int s : cfg.strides) {
        expected_anchors += (cfg.input_width / s) * (cfg.input_height / s);
    }
    SPDLOG_INFO(
        "expecting concat-predecode model: input={}x{}, output_dim={}, "
        "anchors={}",
        cfg.input_width, cfg.input_height, predecode_output_dim, expected_anchors);
    for (int i = 0; i < static_cast<int>(io_info->nInputSize); ++i) {
        const auto& in = io_info->pInputs[i];
        SPDLOG_INFO("  input[{}]: name={}, size={} bytes", i, in.pName, in.nSize);
    }
    for (int i = 0; i < static_cast<int>(io_info->nOutputSize); ++i) {
        const auto& out = io_info->pOutputs[i];
        SPDLOG_INFO("  output[{}]: name={}, size={} bytes", i, out.pName, out.nSize);
    }

    auto io_result = prepare_io(
        io_info, &io_data_,
        std::make_pair(
            AX_ENGINE_ALLOC_BUFFER_STRATEGY_T::CACHED, AX_ENGINE_ALLOC_BUFFER_STRATEGY_T::CACHED));
    if (!io_result) {
        cleanup_failed_init();
        return std::unexpected(io_result.error());
    }

    cached_grid_ = build_decode_grid();
    SPDLOG_INFO("cached decode grid with {} anchors", cached_grid_.grid_x.size());

    if (io_data_.nInputSize > 0 && io_data_.pInputs[0].pVirAddr != nullptr) {
        auto* warmup_input = static_cast<uint8_t*>(io_data_.pInputs[0].pVirAddr);
        std::memset(warmup_input, 114, static_cast<size_t>(io_data_.pInputs[0].nSize));
        // Non-cached buffer: no flush needed.
    }
    ret = AX_ENGINE_RunSync(handle_, &io_data_);
    if (ret != AX_SUCCESS) {
        cleanup_failed_init();
        return std::unexpected(fmt::format("warmup AX_ENGINE_RunSync: {}", ax_error_string(ret)));
    }

    initialized_ = true;
    return {};
}

AxeraBackend::PreprocContext
    AxeraBackend::preprocess(const cv::Mat& image, ArmorColor color) const noexcept {
    PreprocContext ctx;
    ctx.orig_w        = image.cols;
    ctx.orig_h        = image.rows;
    ctx.scale_x       = static_cast<float>(image.cols) / static_cast<float>(config_.input_width);
    ctx.scale_y       = static_cast<float>(image.rows) / static_cast<float>(config_.input_height);
    ctx.is_four_three = is_four_three(image.cols, image.rows);
    ctx.detect_color  = color;
    return ctx;
}

void AxeraBackend::preprocess_image(const cv::Mat& image) noexcept {
    auto* npu_input = static_cast<uint8_t*>(io_data_.pInputs[0].pVirAddr);

    cv::Mat npu_input_mat(config_.input_height, config_.input_width, CV_8UC3, npu_input);
    cv::resize(
        image, npu_input_mat, cv::Size(config_.input_width, config_.input_height), 0, 0,
        cv::INTER_LINEAR);

    // Input buffer is non-cached; no flush needed — CPU writes land directly in DDR.
}

std::expected<void, std::string> AxeraBackend::init_ivps() noexcept {
    const auto ret = AX_IVPS_Init();
    if (ret != AX_SUCCESS) {
        ivps_initialized_ = false;
        return std::unexpected(fmt::format("AX_IVPS_Init: {}", ax_error_string(ret)));
    }

    // 初始化 aspect ratio 配置（STRETCH 模式，类似测试代码）
    ivps_aspect_ratio_.eMode      = AX_IVPS_ASPECT_RATIO_STRETCH;
    ivps_aspect_ratio_.eAligns[0] = AX_IVPS_ASPECT_RATIO_HORIZONTAL_CENTER;
    ivps_aspect_ratio_.eAligns[1] = AX_IVPS_ASPECT_RATIO_VERTICAL_CENTER;
    ivps_aspect_ratio_.nBgColor   = 0xFF00FF; // 紫色背景（STRETCH 模式下不可见）

    ivps_initialized_ = true;
    return {};
}

void AxeraBackend::deinit_ivps() noexcept {
    if (!ivps_initialized_) {
        return;
    }
    free_ivps_buffers();
    // 不清理 AX_POOL，可能是共享资源
    AX_IVPS_Deinit();
    ivps_initialized_ = false;
}

std::expected<void, std::string>
    AxeraBackend::allocate_ivps_buffers(int max_src_width, int max_src_height) noexcept {
    free_ivps_buffers();

    ivps_max_src_width_  = max_src_width;
    ivps_max_src_height_ = max_src_height;

    // BGR888 stride must be a multiple of 3; align to 48 bytes (16 pixels × 3 bytes/pixel)
    // to satisfy both the 3-byte pixel packing constraint and hardware alignment requirements.
    // Reference: ax-video-sdk src/common/ax_image_processor_ax650.cpp MakeAlignedDescriptor()
    constexpr int ALIGN         = 48;
    const AX_U32 aligned_stride = bgr888_stride_aligned(max_src_width);
    ivps_src_stride_            = aligned_stride;
    ivps_src_buffer_size_       = aligned_stride * static_cast<AX_U32>(max_src_height);

    // Use non-cached allocation: CPU writes once, IVPS DMA reads once.
    // Cached buffer would require AX_SYS_MflushCache syscall per frame (~0.1-0.2ms)
    // with no cache reuse benefit.  Non-cached writes are ~10-20% slower per byte
    // but eliminate syscall jitter entirely — same tradeoff as NPU input buffer (line 232).
    const auto ret = AX_SYS_MemAlloc(
        &ivps_src_phy_addr_, &ivps_src_vir_addr_, ivps_src_buffer_size_, ALIGN,
        reinterpret_cast<const AX_S8*>("ivps-src"));

    if (ret != AX_SUCCESS || ivps_src_vir_addr_ == nullptr) {
        ivps_src_phy_addr_    = 0;
        ivps_src_vir_addr_    = nullptr;
        ivps_src_buffer_size_ = 0;
        ivps_src_stride_      = 0;
        return std::unexpected(fmt::format("AX_SYS_MemAlloc: {}", ax_error_string(ret)));
    }

    SPDLOG_INFO(
        "IVPS buffers allocated: {}x{} (stride={}) -> {}x{}", max_src_width, max_src_height,
        aligned_stride, config_.input_width, config_.input_height);
    return {};
}

void AxeraBackend::free_ivps_buffers() noexcept {
    if (ivps_src_vir_addr_ != nullptr && ivps_src_phy_addr_ != 0) {
        AX_SYS_MemFree(ivps_src_phy_addr_, ivps_src_vir_addr_);
    }
    ivps_src_phy_addr_    = 0;
    ivps_src_vir_addr_    = nullptr;
    ivps_src_buffer_size_ = 0;
    ivps_src_stride_      = 0;

    free_ivps_dst_buffer();
}

void AxeraBackend::preprocess_image_ivps(const cv::Mat& image) noexcept {
    if (!ivps_initialized_) {
        preprocess_image(image);
        return;
    }

    if (image.empty() || image.type() != CV_8UC3) {
        SPDLOG_WARN(
            "preprocess_image_ivps expects non-empty CV_8UC3 BGR image, got type={} "
            "size={}x{}; using opencv",
            image.type(), image.cols, image.rows);
        preprocess_image(image);
        return;
    }

    // VPP is stricter than cv::resize: several MSP builds reject odd source geometry
    // with AX_ERR_ILLEGAL_PARAM even for BGR888. Crop off the last row/column for
    // the hardware path only; losing one border pixel is negligible and avoids CPU fallback.
    const int hw_src_w = image.cols & ~1;
    const int hw_src_h = image.rows & ~1;
    if (hw_src_w <= 0 || hw_src_h <= 0) {
        SPDLOG_WARN(
            "IVPS source size becomes invalid after even alignment: original={}x{}, hw={}x{}; "
            "using opencv",
            image.cols, image.rows, hw_src_w, hw_src_h);
        preprocess_image(image);
        return;
    }

    // Check whether the reusable source image-frame buffer must grow.
    // Allocate against the effective even geometry, not the original odd camera width.
    if (hw_src_w > ivps_max_src_width_ || hw_src_h > ivps_max_src_height_
        || ivps_src_vir_addr_ == nullptr) {
        auto buffer_result = allocate_ivps_buffers(hw_src_w, hw_src_h);
        if (!buffer_result) {
            static bool warned = false;
            if (!warned) {
                SPDLOG_WARN("allocate_ivps_buffers: {}, using opencv", buffer_result.error());
                warned = true;
            }
            preprocess_image(image);
            return;
        }
    }

    const size_t packed_row  = static_cast<size_t>(config_.input_width) * 3U;
    const size_t packed_size = packed_row * static_cast<size_t>(config_.input_height);
    if (io_data_.pInputs == nullptr || io_data_.pInputs[0].pVirAddr == nullptr
        || static_cast<size_t>(io_data_.pInputs[0].nSize) < packed_size) {
        SPDLOG_WARN(
            "NPU input buffer invalid/small for direct IVPS dst: nSize={}, required={}; using "
            "opencv",
            io_data_.pInputs != nullptr ? io_data_.pInputs[0].nSize : 0, packed_size);
        preprocess_image(image);
        return;
    }

    AX_VIDEO_FRAME_T src_frame{};
    src_frame.u32Width                      = static_cast<AX_U32>(hw_src_w);
    src_frame.u32Height                     = static_cast<AX_U32>(hw_src_h);
    src_frame.u32PicStride[0]               = ivps_src_stride_;
    src_frame.enImgFormat                   = AX_FORMAT_BGR888;
    src_frame.u64PhyAddr[0]                 = ivps_src_phy_addr_;
    src_frame.u64VirAddr[0]                 = reinterpret_cast<AX_U64>(ivps_src_vir_addr_);
    src_frame.u32FrameSize                  = ivps_src_buffer_size_;
    src_frame.stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;

    auto* src_buffer       = static_cast<uint8_t*>(ivps_src_vir_addr_);
    const size_t src_bytes = static_cast<size_t>(hw_src_w) * 3U;
    for (int y = 0; y < hw_src_h; ++y) {
        std::memcpy(
            src_buffer + static_cast<size_t>(y) * ivps_src_stride_, image.ptr(y), src_bytes);
    }

    // Fast path: direct IVPS -> NPU input.
    // This is safe for your current model size because 768 * 3 = 2304, which already satisfies
    // the BGR888 48-byte stride convention used by the Axera sample code. It removes the extra
    // staging-buffer memcpy that doubled latency on your board.
    AX_VIDEO_FRAME_T dst_frame{};
    dst_frame.u32Width        = config_.input_width;
    dst_frame.u32Height       = config_.input_height;
    dst_frame.u32PicStride[0] = static_cast<AX_U32>(packed_row);
    dst_frame.enImgFormat     = AX_FORMAT_BGR888;
    dst_frame.u64PhyAddr[0]   = io_data_.pInputs[0].phyAddr;
    dst_frame.u64VirAddr[0]   = reinterpret_cast<AX_U64>(io_data_.pInputs[0].pVirAddr);
    dst_frame.u32FrameSize    = static_cast<AX_U32>(packed_size);
    dst_frame.stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;

    const auto ret = AX_IVPS_CropResizeVpp(&src_frame, &dst_frame, &ivps_aspect_ratio_);
    if (ret != AX_SUCCESS) {
        SPDLOG_WARN(
            "AX_IVPS_CropResizeVpp direct-dst failed: {}, src={}x{} stride={} frameSize={}, "
            "dst={}x{} stride={} frameSize={} npuSize={}; using opencv",
            ax_error_string(ret), src_frame.u32Width, src_frame.u32Height,
            src_frame.u32PicStride[0], src_frame.u32FrameSize, dst_frame.u32Width,
            dst_frame.u32Height, dst_frame.u32PicStride[0], dst_frame.u32FrameSize,
            io_data_.pInputs[0].nSize);
        preprocess_image(image);
        return;
    }

    static bool logged_ivps_success = false;
    if (!logged_ivps_success) {
        SPDLOG_INFO(
            "IVPS hardware resize working direct to NPU input! ({}x{} effective from {}x{} -> "
            "{}x{}, src_stride={}, dst_stride={})",
            hw_src_w, hw_src_h, image.cols, image.rows, config_.input_width, config_.input_height,
            ivps_src_stride_, dst_frame.u32PicStride[0]);
        logged_ivps_success = true;
    }

    // NPU input buffer is allocated non-cached in prepare_io(); no flush required.
}

bool AxeraBackend::infer() noexcept {
    if (handle_ == nullptr) {
        return false;
    }

    const auto ret = AX_ENGINE_RunSync(handle_, &io_data_);
    if (ret != 0) {
        SPDLOG_ERROR("infer: AX_ENGINE_RunSync: {}", ax_error_string(ret));
        return false;
    }
    return true;
}

AxeraBackend::DecodeGrid AxeraBackend::build_decode_grid() const noexcept {
    int total_anchors = 0;
    for (const int stride : config_.strides) {
        total_anchors += (config_.input_width / stride) * (config_.input_height / stride);
    }

    DecodeGrid grid;
    grid.grid_x.reserve(total_anchors);
    grid.grid_y.reserve(total_anchors);
    grid.strides.reserve(total_anchors);

    for (const int stride : config_.strides) {
        const int feat_h = config_.input_height / stride;
        const int feat_w = config_.input_width / stride;
        for (int y = 0; y < feat_h; ++y) {
            for (int x = 0; x < feat_w; ++x) {
                grid.grid_x.push_back(static_cast<float>(x) + GRID_OFFSET);
                grid.grid_y.push_back(static_cast<float>(y) + GRID_OFFSET);
                grid.strides.push_back(static_cast<float>(stride));
            }
        }
    }

    return grid;
}

std::vector<ArmorDetection>
    AxeraBackend::postprocess_concat_predecode(const PreprocContext& ctx) const noexcept {
    const int predecode_box_dim  = 4;
    const int predecode_obj_dim  = config_.num_pairs;
    const int predecode_kpts_dim = config_.num_kpts * 2;
    const int predecode_output_dim =
        predecode_box_dim + config_.num_colors + predecode_obj_dim + predecode_kpts_dim;

    const AX_ENGINE_IO_BUFFER_T* output_buf = nullptr;
    const size_t min_output_bytes = static_cast<size_t>(predecode_output_dim) * sizeof(float);

    for (int i = 0; i < static_cast<int>(io_data_.nOutputSize); ++i) {
        const AX_ENGINE_IO_BUFFER_T& buf = io_data_.pOutputs[i];
        if (buf.pVirAddr == nullptr || static_cast<size_t>(buf.nSize) < min_output_bytes) {
            continue;
        }
        if (static_cast<size_t>(buf.nSize) % (sizeof(float) * predecode_output_dim) != 0U) {
            continue;
        }
        if (output_buf == nullptr || buf.nSize > output_buf->nSize) {
            output_buf = &buf;
        }
    }

    if (output_buf == nullptr) {
        SPDLOG_ERROR("No valid concat-predecode output tensor found");
        return {};
    }

    const int num_anchors = static_cast<int>(
        static_cast<size_t>(output_buf->nSize) / (sizeof(float) * predecode_output_dim));
    if (num_anchors != static_cast<int>(cached_grid_.grid_x.size())) {
        SPDLOG_ERROR(
            "Anchor count mismatch: output={} grid={}", num_anchors, cached_grid_.grid_x.size());
        return {};
    }

    const float* output = static_cast<const float*>(output_buf->pVirAddr);
    // Skip raw_boxes (first predecode_box_dim * num_anchors floats)
    const float* color_logits = output + predecode_box_dim * num_anchors;
    const float* obj_logits   = color_logits + config_.num_colors * num_anchors;
    const float* raw_kpts     = obj_logits + predecode_obj_dim * num_anchors;

    const float confidence_threshold = static_cast<float>(config_.confidence_threshold);
    const float logit_threshold = std::log(confidence_threshold / (1.0f - confidence_threshold));

    static bool logged_threshold = false;
    if (!logged_threshold) {
        SPDLOG_INFO(
            "confidence_threshold={}, logit_threshold={:.2f}", confidence_threshold,
            logit_threshold);
        logged_threshold = true;
    }

    const size_t pre_nms_top_k =
        static_cast<size_t>(std::min(config_.top_k, std::max(num_anchors, 1)));
    auto min_heap_comp = [](const RankedAnchor& lhs, const RankedAnchor& rhs) {
        return lhs.best_obj_logit > rhs.best_obj_logit;
    };

    postproc_ranked_buf_.clear();
    if (postproc_ranked_buf_.capacity() < pre_nms_top_k) {
        postproc_ranked_buf_.reserve(pre_nms_top_k);
    }
    auto& ranked = postproc_ranked_buf_;

    for (int anchor_idx = 0; anchor_idx < num_anchors; ++anchor_idx) {
        float best_obj_logit = obj_logits[anchor_idx];
        int pair_id          = 0;
        for (int cls = 1; cls < config_.num_pairs; ++cls) {
            const float score = obj_logits[cls * num_anchors + anchor_idx];
            if (score > best_obj_logit) {
                best_obj_logit = score;
                pair_id        = cls;
            }
        }

        if (best_obj_logit < logit_threshold) {
            continue;
        }

        const RankedAnchor item{best_obj_logit, pair_id, anchor_idx};
        if (ranked.size() < pre_nms_top_k) {
            ranked.push_back(item);
            std::push_heap(ranked.begin(), ranked.end(), min_heap_comp);
            continue;
        }
        if (!ranked.empty() && item.best_obj_logit > ranked.front().best_obj_logit) {
            std::pop_heap(ranked.begin(), ranked.end(), min_heap_comp);
            ranked.back() = item;
            std::push_heap(ranked.begin(), ranked.end(), min_heap_comp);
        }
    }

    std::sort(ranked.begin(), ranked.end(), [](const RankedAnchor& lhs, const RankedAnchor& rhs) {
        return lhs.best_obj_logit > rhs.best_obj_logit;
    });

    const float max_x   = static_cast<float>(std::max(ctx.orig_w - 1, 0));
    const float max_y   = static_cast<float>(std::max(ctx.orig_h - 1, 0));
    const float scale_x = ctx.scale_x;
    const float scale_y = ctx.scale_y;

    postproc_candidates_buf_.clear();
    if (postproc_candidates_buf_.capacity() < ranked.size()) {
        postproc_candidates_buf_.reserve(ranked.size());
    }
    auto& candidates = postproc_candidates_buf_;

    for (const RankedAnchor& ranked_anchor : ranked) {
        // Ranked anchors are sorted descending by logit; all passed logit_threshold in
        // the heap phase, and sigmoid is monotonic, so no second threshold check needed.
        const float confidence = fast_sigmoid(ranked_anchor.best_obj_logit);

        const int anchor_idx = ranked_anchor.anchor_idx;
        const float grid_x   = cached_grid_.grid_x[anchor_idx];
        const float grid_y   = cached_grid_.grid_y[anchor_idx];
        const float stride   = cached_grid_.strides[anchor_idx];

        // Color classification
        int best_color_id   = 0;
        float best_color_sc = color_logits[anchor_idx];
        for (int color_idx = 1; color_idx < config_.num_colors; ++color_idx) {
            const float score = color_logits[color_idx * num_anchors + anchor_idx];
            if (score > best_color_sc) {
                best_color_sc = score;
                best_color_id = color_idx;
            }
        }

        // Early-out: skip detections whose color doesn't match the target
        if (static_cast<ArmorColor>(best_color_id) != ctx.detect_color) {
            continue;
        }

        // Decode pts (keypoints) only — skip bbox
        std::array<cv::Point2f, 4> model_corners{};
        for (int k = 0; k < config_.num_kpts; ++k) {
            const float kx = raw_kpts[(k * 2 + 0) * num_anchors + anchor_idx];
            const float ky = raw_kpts[(k * 2 + 1) * num_anchors + anchor_idx];

            model_corners[k].x = clampf((kx + grid_x) * stride * scale_x, 0.0f, max_x);
            model_corners[k].y = clampf((ky + grid_y) * stride * scale_y, 0.0f, max_y);
        }

        // Reorder corners via KPT_ORDER
        std::array<cv::Point2f, 4> ordered_corners{};
        for (int k = 0; k < config_.num_kpts; ++k) {
            ordered_corners[k] = model_corners[KPT_ORDER[k]];
        }

        // Validate: reject degenerate corners (zero area)
        const cv::Rect2f br = bounding_rect_4pt(ordered_corners);
        if (br.width <= 0.0f || br.height <= 0.0f) [[unlikely]] {
            continue;
        }

        DecodedCandidate candidate;
        candidate.corners    = ordered_corners;
        candidate.name       = PAIR_TO_ARMOR[ranked_anchor.pair_id];
        candidate.color      = static_cast<ArmorColor>(best_color_id);
        candidate.confidence = confidence;
        candidate.pair_id    = ranked_anchor.pair_id;
        candidates.push_back(std::move(candidate));
    }

    return nms(candidates, ctx.orig_w, ctx.orig_h);
}

std::vector<ArmorDetection> AxeraBackend::nms(
    std::vector<DecodedCandidate>& detections, int orig_w, int orig_h) const noexcept {
    if (detections.empty()) {
        return {};
    }

    // Assign each detection to a tile based on corner center
    const float tile_w = static_cast<float>(orig_w) / TILE_GRID_SIZE;
    const float tile_h = static_cast<float>(orig_h) / TILE_GRID_SIZE;

    for (auto& bin : nms_tile_bins_buf_) {
        bin.clear();
    }
    auto& tile_bins = nms_tile_bins_buf_;

    for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
        const auto& det = detections[i];
        const cv::Point2f center =
            (det.corners[0] + det.corners[1] + det.corners[2] + det.corners[3]) * 0.25f;
        const int tx = clamp_int(static_cast<int>(center.x / tile_w), 0, TILE_GRID_SIZE - 1);
        const int ty = clamp_int(static_cast<int>(center.y / tile_h), 0, TILE_GRID_SIZE - 1);
        tile_bins[ty * TILE_GRID_SIZE + tx].push_back(i);
    }

    const float nms_thresh = static_cast<float>(config_.nms_threshold);

    nms_removed_buf_.assign(detections.size(), 0);
    auto& removed = nms_removed_buf_;

    // Reuse pre-allocated NMS result buffer to avoid per-frame allocation jitter
    nms_result_buf_.clear();
    nms_result_buf_.reserve(std::min(detections.size(), static_cast<size_t>(config_.top_k)));

    // Process detections in confidence order
    for (size_t i = 0;
         i < detections.size() && nms_result_buf_.size() < static_cast<size_t>(config_.top_k);
         ++i) {
        if (removed[i] != 0) {
            continue;
        }

        ArmorDetection det(
            detections[i].corners, detections[i].name, detections[i].color,
            detections[i].confidence);
        if (det.area <= 0) {
            continue;
        }
        nms_result_buf_.push_back(std::move(det));

        // Find which tile this detection belongs to
        const auto& corners_i = detections[i].corners;
        const cv::Point2f center_i =
            (corners_i[0] + corners_i[1] + corners_i[2] + corners_i[3]) * 0.25f;
        const int tx = clamp_int(static_cast<int>(center_i.x / tile_w), 0, TILE_GRID_SIZE - 1);
        const int ty = clamp_int(static_cast<int>(center_i.y / tile_h), 0, TILE_GRID_SIZE - 1);

        // Only check neighboring tiles (3x3 window centered on this tile)
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const int nx = tx + dx;
                const int ny = ty + dy;
                if (nx < 0 || nx >= TILE_GRID_SIZE || ny < 0 || ny >= TILE_GRID_SIZE) {
                    continue;
                }
                const int neighbor_tile = ny * TILE_GRID_SIZE + nx;
                for (const int j : tile_bins[neighbor_tile]) {
                    if (j <= static_cast<int>(i) || removed[j] != 0
                        || detections[i].pair_id != detections[j].pair_id) {
                        continue;
                    }
                    if (pts_iou(detections[i].corners, detections[j].corners) > nms_thresh) {
                        removed[j] = 1;
                    }
                }
            }
        }
    }

    return std::move(nms_result_buf_);
}

AxeraBackend::DetectionResult
    AxeraBackend::detect_impl(const cv::Mat& image, ArmorColor color) noexcept {
    const auto ctx = preprocess(image, color);
    if (!ctx.is_four_three && !warned_non_four_three_input_) {
        SPDLOG_WARN(
            "Input {}x{} is not 4:3; direct resize to {}x{} will be used "
            "without letterbox",
            image.cols, image.rows, config_.input_width, config_.input_height);
        warned_non_four_three_input_ = true;
    }

    if (ivps_initialized_) {
        preprocess_image_ivps(image);
    } else {
        preprocess_image(image);
    }

    if (!infer()) {
        return std::unexpected("Axera NPU inference failed (AX_ENGINE_RunSync)");
    }

    return postprocess_concat_predecode(ctx);
}

} // namespace fcs::L2
