#pragma once

#include <cstdint>

namespace toml::inline v3 {
class table;
}

namespace fcs::core::trajectory::model {

enum class ModelType {
    Ideal,
    LinearDrag,
};

struct ModelConfig {
    ModelType type{ModelType::LinearDrag};

    /// Number of iterations for pitch solving / compensation loop.
    uint32_t iteration_times{20};

    /// Gravity acceleration (m/s^2)
    double gravity{9.8};

    /// Linearized air resistance coefficient (used by ResistanceModel only).
    double resistance{0.001};

    /// Air density rho (kg/m^3), used by RK4Model only.
    double rho{1.225};

    /// Bullet diameter (meters), used by RK4Model only.
    double diameter{0.017};

    /// Bullet mass (kg), used by RK4Model only.
    double mass{0.0032};
};

} // namespace fcs::core::trajectory::model
