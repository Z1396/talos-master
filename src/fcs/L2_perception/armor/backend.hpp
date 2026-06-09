#pragma once

#include "L2_perception/armor/backends/ort.hpp"
#include "L2_perception/armor/readback_roi.hpp"
#include "core/armor_types.hpp"
#include "core/types.hpp"

#include <expected>
#include <memory>
#include <variant>
#include <vector>

// Forward declare OpenCV types to avoid heavy header pollution
namespace cv {
class Mat;
}

#include "backends/base.hpp"

namespace fcs::L2 {

// ============================================================================
// Forward Declarations for Concrete Backends
// ============================================================================

class TraditionalBackend;
class OrtBackend;
#if TALOS_HAS_TENSORRT
class TrtBackend;
#endif
#if TALOS_HAS_AXERA
class AxeraBackend;
#endif

// Forward declare config types (defined in config.hpp)
struct ArmorDetectorConfig;
#if TALOS_HAS_TENSORRT
struct TrtConfig;
#endif
#if TALOS_HAS_AXERA
struct AxeraConfig;
#endif
struct ArmorTraditionalConfig;
struct ArmorOrtConfig;

// ============================================================================
// Runtime Polymorphic Backend Wrapper (using std::variant)
// ============================================================================

/// Backend variant type for runtime switching
using DetectorBackendVariant = std::variant<
#if TALOS_HAS_AXERA
    std::unique_ptr<AxeraBackend>,
#endif
#if TALOS_HAS_TENSORRT
    std::unique_ptr<TrtBackend>,
#endif
    std::unique_ptr<TraditionalBackend>, std::unique_ptr<OrtBackend>>;

/// Runtime-configurable detector backend wrapper
/// Implementation methods are in detector_backend.cpp to avoid
/// incomplete type issues with std::visit
class DetectorBackend {
public:
    using DetectionResult = std::expected<std::vector<ArmorDetection>, std::string>;

    DetectorBackend();
    ~DetectorBackend();

    // Move operations
    DetectorBackend(DetectorBackend&&) noexcept;
    DetectorBackend& operator=(DetectorBackend&&) noexcept;

    // No copy
    DetectorBackend(const DetectorBackend&)            = delete;
    DetectorBackend& operator=(const DetectorBackend&) = delete;

    /// Construct with a specific backend
    explicit DetectorBackend(std::unique_ptr<TraditionalBackend> backend);
    explicit DetectorBackend(std::unique_ptr<OrtBackend> backend);
#if TALOS_HAS_TENSORRT
    explicit DetectorBackend(std::unique_ptr<TrtBackend> backend);
#endif
#if TALOS_HAS_AXERA
    explicit DetectorBackend(std::unique_ptr<AxeraBackend> backend);
#endif

    /// Detect armors in image (dispatches to active backend) — noexcept boundary
    [[nodiscard]] DetectionResult detect(const cv::Mat& image, ArmorColor color) noexcept;

    /// Input resolution used to derive ROI aspect ratio and minimum crop size.
    [[nodiscard]] BackendInputResolution input_resolution() const noexcept;

    /// Switch to a different backend
    void set_backend(std::unique_ptr<TraditionalBackend> backend);
    void set_backend(std::unique_ptr<OrtBackend> backend);
#if TALOS_HAS_TENSORRT
    void set_backend(std::unique_ptr<TrtBackend> backend);
#endif
#if TALOS_HAS_AXERA
    void set_backend(std::unique_ptr<AxeraBackend> backend);
#endif

    /// Check if backend variant holds a specific type
    template <typename Backend>
    [[nodiscard]] bool holds() const noexcept {
        return std::holds_alternative<std::unique_ptr<Backend>>(backend_);
    }

    /// Get reference to specific backend
    template <typename Backend>
    [[nodiscard]] Backend& get() {
        return *std::get<std::unique_ptr<Backend>>(backend_);
    }

    template <typename Backend>
    [[nodiscard]] const Backend& get() const {
        return *std::get<std::unique_ptr<Backend>>(backend_);
    }

private:
    DetectorBackendVariant backend_;
};

// ============================================================================
// Backend Factory
// ============================================================================

/// Create detector backend from unified config
/// Implementation in detector_backend.cpp
[[nodiscard]] std::expected<DetectorBackend, std::string>
    create_detector_backend(const ArmorDetectorConfig& config) noexcept;

} // namespace fcs::L2
