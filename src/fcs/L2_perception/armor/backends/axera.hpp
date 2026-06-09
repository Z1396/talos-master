#pragma once

#include "../config.hpp"
#include "base.hpp"
#include "core/armor_types.hpp"

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

// AX650 NPU API
#include "ax_engine_api.h"
#include "ax_sys_api.h"

// AX650 IVPS (Image Video Processing System) API for hardware-accelerated preprocessing
#include "ax_ivps_api.h"
#include "ax_ivps_type.h"

namespace quanta {
class AxSysModule;
}

namespace fcs::L2 {

// ============================================================================
// AT Legacy Axera Backend Implementation
// ============================================================================

/// Neural network detection backend using AX650 NPU
/// Implements RAII for AX engine resources
/// Construction IS initialization — no separate init() needed.
class AxeraBackend : public DetectorBackendBase<AxeraBackend> {
public:
    using DetectionResult = std::expected<std::vector<ArmorDetection>, std::string>;
    using Config          = ArmorAxeraConfig;

    /// Factory: construct a fully-initialized backend (load model onto NPU).
    /// Construction IS initialization — no separate init() needed.
    [[nodiscard]] static std::expected<AxeraBackend, std::string> create(Config config) noexcept;

    ~AxeraBackend();

    // Move-only (AX engine resources are non-copyable)
    AxeraBackend(AxeraBackend&&) noexcept;
    AxeraBackend& operator=(AxeraBackend&&) noexcept;
    AxeraBackend(const AxeraBackend&)            = delete;
    AxeraBackend& operator=(const AxeraBackend&) = delete;

    /// Detect armors in image (synchronous)
    [[nodiscard]] DetectionResult detect_impl(const cv::Mat& image, ArmorColor color) noexcept;

    /// Get current config
    [[nodiscard]] const Config& get_config() const noexcept { return config_; }

private:
    /// Private constructor — use create() factory
    explicit AxeraBackend(Config config) noexcept;
    [[nodiscard]] std::expected<void, std::string> init() noexcept;

    /// Direct-resize preprocessing context.
    struct PreprocContext {
        ArmorColor detect_color;
        float scale_x      = 1.0f;
        float scale_y      = 1.0f;
        int orig_w         = 0;
        int orig_h         = 0;
        bool is_four_three = false;
    };

    /// Build direct-resize transform for current frame.
    [[nodiscard]] PreprocContext preprocess(const cv::Mat& image, ArmorColor color) const noexcept;

    /// Preprocess image into NPU buffer (resize + BGR2RGB + cache flush).
    void preprocess_image(const cv::Mat& image) noexcept;

    /// Run pure NPU inference (assumes input buffer is ready)
    [[nodiscard]] bool infer() noexcept;

    /// Cached anchor grid used to decode the predecode-concat output on CPU.
    struct DecodeGrid {
        std::vector<float> grid_x;
        std::vector<float> grid_y;
        std::vector<float> strides;
    };

    struct DecodedCandidate {
        std::array<cv::Point2f, 4> corners{};
        ArmorName name   = ArmorName::Invalid;
        ArmorColor color = ArmorColor::Neutral;
        float confidence = 0.0f;
        int pair_id      = -1;
    };

    struct RankedAnchor {
        float best_obj_logit = -std::numeric_limits<float>::infinity();
        int pair_id          = -1;
        int anchor_idx       = -1;
    };

    static constexpr int TILE_GRID_SIZE = 2;

    /// Build decode grid for anchor-based decoding (called once at init).
    [[nodiscard]] DecodeGrid build_decode_grid() const noexcept;

    /// Cached decode grid (built once at initialization).
    DecodeGrid cached_grid_;

    // Reusable postprocess/NMS buffers (mutable: internal detail, logical const-ness unaffected)
    mutable std::vector<RankedAnchor> postproc_ranked_buf_;
    mutable std::vector<DecodedCandidate> postproc_candidates_buf_;
    mutable std::vector<uint8_t> nms_removed_buf_;
    mutable std::array<std::vector<int>, TILE_GRID_SIZE * TILE_GRID_SIZE> nms_tile_bins_buf_;
    mutable std::vector<ArmorDetection> nms_result_buf_;

    /// Decode and postprocess single-output concat-predecode tensor.
    [[nodiscard]] std::vector<ArmorDetection>
        postprocess_concat_predecode(const PreprocContext& ctx) const noexcept;

    /// Tile-based NMS on pts corner points.
    [[nodiscard]] std::vector<ArmorDetection>
        nms(std::vector<DecodedCandidate>& detections, int orig_w, int orig_h) const noexcept;

private:
    Config config_;
    bool initialized_                 = false;
    bool warned_non_four_three_input_ = false;
    std::shared_ptr<quanta::AxSysModule> ax_sys_module_;

    // AX650 NPU resources
    AX_ENGINE_HANDLE handle_ = nullptr;
    AX_ENGINE_IO_T io_data_{};
    std::vector<char> model_buffer_;

    // IVPS (Image Video Processing System) resources for hardware-accelerated preprocessing
    bool ivps_initialized_       = false;
    AX_U64 ivps_src_phy_addr_    = 0;            // Physical address for source buffer
    AX_VOID* ivps_src_vir_addr_  = nullptr;      // Virtual address for source buffer
    AX_U32 ivps_src_buffer_size_ = 0;            // Source buffer size (bytes)
    AX_U32 ivps_src_stride_      = 0;            // Source stride (aligned)
    int ivps_max_src_width_      = 0;
    int ivps_max_src_height_     = 0;
    AX_IVPS_ASPECT_RATIO_T ivps_aspect_ratio_{}; // Cached aspect ratio config

    // IVPS helper methods
    [[nodiscard]] std::expected<void, std::string> init_ivps() noexcept;
    void deinit_ivps() noexcept;
    [[nodiscard]] std::expected<void, std::string>
        allocate_ivps_buffers(int max_src_width, int max_src_height) noexcept;
    void free_ivps_buffers() noexcept;
    void preprocess_image_ivps(const cv::Mat& image) noexcept;
};

} // namespace fcs::L2
