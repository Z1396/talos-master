#pragma once

#include "../config.hpp"
#include "base.hpp"
#include "core/armor_types.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

namespace cv {
class Mat;
}

namespace fcs::L2 {

class TrtBackend : public DetectorBackendBase<TrtBackend> {
public:
    static constexpr int INPUT_W     = 640;
    static constexpr int INPUT_H     = 640;
    static constexpr int NUM_COLORS  = 4;
    static constexpr int NUM_SIZES   = 2;
    static constexpr int NUM_CLASSES = 64;
    static constexpr int NUM_KPTS    = 4;
    static constexpr int OUTPUT_DIM  = 14;

    using DetectionResult = std::expected<std::vector<fcs::ArmorDetection>, std::string>;
    using Config          = ArmorTensorRtConfig;

    struct PreprocContext {
        float scale_x;
        float scale_y;
        int pad_x;
        int pad_y;
        int orig_width;
        int orig_height;
    };

    /// Factory: construct a fully-initialized backend (load TensorRT engine).
    /// Construction IS initialization — no separate init() needed.
    [[nodiscard]] static std::expected<TrtBackend, std::string> create(Config config) noexcept;

    ~TrtBackend();

    TrtBackend(TrtBackend&&) noexcept;
    TrtBackend& operator=(TrtBackend&&) noexcept;
    TrtBackend(const TrtBackend&)            = delete;
    TrtBackend& operator=(const TrtBackend&) = delete;

    DetectionResult detect_impl(const cv::Mat& image, ArmorColor color) noexcept;

private:
    /// Private constructor — use create() factory
    explicit TrtBackend(Config config) noexcept;

    struct Impl;
    std::unique_ptr<Impl> impl_;

    Config config_;
};

} // namespace fcs::L2
