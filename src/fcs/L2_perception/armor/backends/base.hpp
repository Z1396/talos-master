#pragma once

#include "core/armor_types.hpp"

#include <concepts>
#include <expected>
#include <string>
#include <utility>
#include <vector>

// Forward declare OpenCV types to avoid heavy header pollution
namespace cv {
class Mat;
}

namespace fcs {

// ============================================================================
// Detector Backend Concept — Compile-Time Interface Enforcement
// ============================================================================
//
// Every backend MUST satisfy this contract. Violations are hard compile errors.
//
// Invariants enforced by concept:
//   - detect_impl()  noexcept — handles all errors internally via std::expected
//   - create()       noexcept — factory handles all errors via std::expected
//   - get_config() — noexcept
//
// Design rule: exceptions are system boundary concerns only.
// Internal code MUST NOT throw. All errors propagate via std::expected
// with full context (backend name, operation, error detail).

template <typename T>
concept DetectorBackend = requires(T& t, const T& ct, const cv::Mat& image, ArmorColor color) {
    // Type aliases
    typename T::DetectionResult;
    typename T::Config;

    // Core detection — noexcept, errors via std::expected with full context
    { t.detect_impl(image, color) } noexcept -> std::same_as<typename T::DetectionResult>;

    // Config access — noexcept guaranteed
    { t.get_config() } noexcept -> std::same_as<const typename T::Config&>;
    { ct.get_config() } noexcept -> std::same_as<const typename T::Config&>;

    // Factory — static, noexcept, all errors via std::expected
    {
        T::create(std::declval<typename T::Config>())
    } noexcept -> std::same_as<std::expected<T, std::string>>;
};

// ============================================================================
// CRTP Base — Zero-Cost Abstraction
// ============================================================================

/// CRTP base for detector backends.
/// Pure noexcept forwarding — detect_impl() is responsible for all error handling.
/// No try/catch here. detect_impl() is noexcept (enforced by concept),
/// so this entire call chain is exception-free.
template <typename Derived>
class DetectorBackendBase {
public:
    using DetectionResult = std::expected<std::vector<ArmorDetection>, std::string>;

    /// Detect armors in image — pure noexcept forwarding.
    [[nodiscard]] DetectionResult detect(const cv::Mat& image, ArmorColor color) noexcept {
        return static_cast<Derived*>(this)->detect_impl(image, color);
    }

protected:
    DetectorBackendBase()  = default;
    ~DetectorBackendBase() = default;
};

} // namespace fcs
