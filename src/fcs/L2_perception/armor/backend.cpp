#include "L2_perception/armor/backends/ort.hpp"
#include "L2_perception/armor/backends/traditional.hpp"
#include "L2_perception/armor/config.hpp"
#include "core/armor_types.hpp"
#if TALOS_HAS_TENSORRT
# include "L2_perception/armor/backends/tensor_rt.hpp"
#endif
#if TALOS_HAS_AXERA
# include "L2_perception/armor/backends/axera.hpp"
#endif

#include <type_traits>

namespace fcs::L2 {

// ============================================================================
// DetectorBackend Implementation
// ============================================================================

DetectorBackend::DetectorBackend()  = default;
DetectorBackend::~DetectorBackend() = default;

DetectorBackend::DetectorBackend(DetectorBackend&&) noexcept            = default;
DetectorBackend& DetectorBackend::operator=(DetectorBackend&&) noexcept = default;

DetectorBackend::DetectorBackend(std::unique_ptr<TraditionalBackend> backend)
    : backend_(std::move(backend)) {}

DetectorBackend::DetectorBackend(std::unique_ptr<OrtBackend> backend)
    : backend_(std::move(backend)) {}

#if TALOS_HAS_TENSORRT
DetectorBackend::DetectorBackend(std::unique_ptr<TrtBackend> backend)
    : backend_(std::move(backend)) {}
#endif

#if TALOS_HAS_AXERA
DetectorBackend::DetectorBackend(std::unique_ptr<AxeraBackend> backend)
    : backend_(std::move(backend)) {}
#endif

DetectorBackend::DetectionResult
    DetectorBackend::detect(const cv::Mat& image, ArmorColor color) noexcept {
    return std::visit(
        [&image, color](auto& backend) -> DetectionResult {
            if (!backend) {
                return std::unexpected("No detector backend is active");
            }
            return backend->detect(image, color);
        },
        backend_);
}

void DetectorBackend::set_backend(std::unique_ptr<TraditionalBackend> backend) {
    backend_ = std::move(backend);
}

void DetectorBackend::set_backend(std::unique_ptr<OrtBackend> backend) {
    backend_ = std::move(backend);
}

#if TALOS_HAS_TENSORRT
void DetectorBackend::set_backend(std::unique_ptr<TrtBackend> backend) {
    backend_ = std::move(backend);
}
#endif

#if TALOS_HAS_AXERA
void DetectorBackend::set_backend(std::unique_ptr<AxeraBackend> backend) {
    backend_ = std::move(backend);
}
#endif

BackendInputResolution DetectorBackend::input_resolution() const noexcept {
    return std::visit(
        [](const auto& backend) -> BackendInputResolution {
            using BackendPtr = std::decay_t<decltype(backend)>;
            using Backend    = typename BackendPtr::element_type;
            if (!backend) {
                return {};
            }
            if constexpr (std::is_same_v<Backend, TraditionalBackend>) {
                return {.width = 480, .height = 270};
#if TALOS_HAS_AXERA
            } else if constexpr (std::is_same_v<Backend, AxeraBackend>) {
                return {
                    .width  = backend->get_config().input_width,
                    .height = backend->get_config().input_height};
#endif
            } else {
                return {.width = Backend::INPUT_W, .height = Backend::INPUT_H};
            }
        },
        backend_);
}

// ============================================================================
// Factory Function
// ============================================================================

[[nodiscard]] std::expected<DetectorBackend, std::string>
    create_detector_backend(const ArmorDetectorConfig& config) noexcept {
    switch (config.backend_type) {
    case ArmorBackendType::Traditional: {
        auto result = TraditionalBackend::create(config.traditional);
        if (!result) {
            return std::unexpected(result.error());
        }
        return DetectorBackend(std::make_unique<TraditionalBackend>(std::move(*result)));
    }
    case ArmorBackendType::OnnxRuntime: {
        auto result = OrtBackend::create(config.onnx_runtime);
        if (!result) {
            return std::unexpected(result.error());
        }
        return DetectorBackend(std::make_unique<OrtBackend>(std::move(*result)));
    }
#if TALOS_HAS_TENSORRT
    case ArmorBackendType::TensorRT: {
        auto result = TrtBackend::create(config.tensor_rt);
        if (!result) {
            return std::unexpected(result.error());
        }
        return DetectorBackend(std::make_unique<TrtBackend>(std::move(*result)));
    }
#endif
#if TALOS_HAS_AXERA
    case ArmorBackendType::Axera: {
        auto result = AxeraBackend::create(config.axera);
        if (!result) {
            return std::unexpected(result.error());
        }
        return DetectorBackend(std::make_unique<AxeraBackend>(std::move(*result)));
    }
#endif
    }
    std::unreachable();
}

} // namespace fcs::L2
