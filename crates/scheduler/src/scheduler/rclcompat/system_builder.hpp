#pragma once

#include "../system/execution_policy.hpp"
#include "../system/system_meta.hpp"
#include <string>
#include <typeindex>
#include <utility>

namespace talos::scheduler::rclcompat {

// ============================================================================
// SystemMetaBuilder: Runtime construction of SystemMeta
// ============================================================================

/**
 * @brief Builder for runtime construction of SystemMeta
 *
 * Used by rclcompat systems that need to construct metadata at runtime
 * rather than extracting it from function signatures.
 *
 * ## Example
 *
 * ```cpp
 * auto meta = SystemMetaBuilder("camera_pub")
 *     .policy(make_policy_info<pool_compute>())
 *     .add_spmc_writer(typeid(ImageFrame), typeid(CameraTag))
 *     .build();
 * ```
 */
class SystemMetaBuilder {
public:
    /**
     * @brief Construct a builder with the system name
     *
     * ## Parameters
     *
     * - `name`: system name for identification
     */
    explicit SystemMetaBuilder(std::string name) noexcept { meta_.name = std::move(name); }

    /**
     * @brief Set the execution policy
     *
     * ## Parameters
     *
     * - `p`: policy info
     *
     * ## Returns
     *
     * Reference to this builder for chaining
     */
    SystemMetaBuilder& policy(system::PolicyInfo p) noexcept {
        meta_.policy = p;
        return *this;
    }

    /**
     * @brief Add a channel to the metadata (unified helper)
     *
     * ## Template parameters
     *
     * - `Kind`: channel kind enum value
     *
     * ## Parameters
     *
     * - `type`: data type index
     * - `topic`: topic type index
     *
     * ## Returns
     *
     * Reference to this builder for chaining
     */
    template <system::channel_kind Kind>
    SystemMetaBuilder&
        add_channel(const std::type_index type, const std::type_index topic) noexcept {
        if constexpr (
            Kind == system::channel_kind::spmc_writer
            || Kind == system::channel_kind::spmc_reader) {
            meta_.spmc_channels.push_back(
                system::ChannelMeta{.type = type, .topic = topic, .kind = Kind});
        } else {
            meta_.spsc_channels.push_back(
                system::ChannelMeta{.type = type, .topic = topic, .kind = Kind});
        }
        return *this;
    }

    // Convenience wrappers using the unified template
    SystemMetaBuilder&
        add_spmc_writer(const std::type_index type, const std::type_index topic) noexcept {
        return add_channel<system::channel_kind::spmc_writer>(type, topic);
    }

    SystemMetaBuilder&
        add_spmc_reader(const std::type_index type, const std::type_index topic) noexcept {
        return add_channel<system::channel_kind::spmc_reader>(type, topic);
    }

    SystemMetaBuilder&
        add_spsc_writer(const std::type_index type, const std::type_index topic) noexcept {
        return add_channel<system::channel_kind::spsc_writer>(type, topic);
    }

    SystemMetaBuilder&
        add_spsc_reader(const std::type_index type, const std::type_index topic) noexcept {
        return add_channel<system::channel_kind::spsc_reader>(type, topic);
    }

    /**
     * @brief Build the SystemMeta (rvalue version)
     *
     * ## Returns
     *
     * The constructed SystemMeta
     */
    [[nodiscard]] system::SystemMeta build() && noexcept { return std::move(meta_); }

    /**
     * @brief Build the SystemMeta (lvalue version)
     *
     * ## Returns
     *
     * Copy of the constructed SystemMeta
     */
    [[nodiscard]] system::SystemMeta build() const& noexcept { return meta_; }

private:
    system::SystemMeta meta_;
};

} // namespace talos::scheduler::rclcompat
