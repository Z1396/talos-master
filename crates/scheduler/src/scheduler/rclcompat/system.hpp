#pragma once

#include "../demangle.hpp"
#include "../error.hpp"
#include "../system/system.hpp"
#include "registry.hpp"
#include "system_builder.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace talos::scheduler::rclcompat {

using system::ChannelKey;
using system::DefaultTopic;
using system::ExternalComputeSource;
using system::fixed_rate;
using system::pool_compute;
using system::res;
using system::res_mut;
using system::spmc;
using system::spmc_mut;
using system::SystemBase;
using system::SystemMeta;

// Forward declaration
class RclSubSystemBase;

/**
 * @brief Thread-local context for tracking the currently executing callback
 *
 * This tracks which callback first claims a Publisher handle,
 * preventing multiple callbacks from sharing the same Publisher instance.
 *
 * ## Architecture
 *
 * ```
 * RclSubSystem::run()
 *   ├─ CallbackContextScope scope(this)  // Set g_callback_context.current_system = this
 *   ├─ callback_(*msg)  ─────────────────┐
 *   │   └─ publisher.publish(msg)        │
 *   │       ├─ First use: claim ownership
 *   │       └─ Subsequent: verify same owner ───┘
 *   └─ ~CallbackContextScope()  // Restore previous context
 * ```
 */
struct CallbackContext {
    RclSubSystemBase* current_system = nullptr; ///< Pointer to the currently executing RclSubSystem
};

/// Global thread-local callback context
extern thread_local CallbackContext g_callback_context;

/**
 * @brief RAII scope guard for setting/restoring callback context
 *
 * Automatically restores the previous context when destroyed,
 * ensuring exception safety.
 */
struct CallbackContextScope {
    CallbackContext* ctx_;
    RclSubSystemBase* prev_system_;

    explicit CallbackContextScope(RclSubSystemBase* current) noexcept;

    ~CallbackContextScope() noexcept;

    // Non-copyable, non-movable
    CallbackContextScope(const CallbackContextScope&)            = delete;
    CallbackContextScope& operator=(const CallbackContextScope&) = delete;
};

/**
 * @brief Base class for RclSubSystem
 *
 * Provides SystemMeta storage and callback ownership tracking.
 */
class RclSubSystemBase : public SystemBase {
public:
    ~RclSubSystemBase() override = default;

    const SystemMeta& meta() const noexcept override;

protected:
    /**
     * @brief Build system metadata with policy and channel configuration
     *
     * Unified helper for RclPubSystem and RclSubSystem constructors.
     * Eliminates duplicate SystemMetaBuilder pattern.
     *
     * ## Template parameters
     *
     * - `Policy`: execution policy type
     *
     * ## Parameters
     *
     * - `name`: system name
     * - `config_fn`: function to configure channels (e.g., add_spmc_writer)
     *
     * ## Returns
     *
     * Configured SystemMeta
     */
    template <typename Policy = pool_compute>
    [[nodiscard]] static auto build_meta(std::string name, auto&& config_fn) noexcept
        -> SystemMeta {
        SystemMetaBuilder builder(std::move(name));
        builder.policy(system::make_policy_info<Policy>());
        config_fn(builder);
        return builder.build();
    }

    SystemMeta meta_;
};

// ============================================================================
// PubSlot: Publisher state (owned by Node)
// ============================================================================

/**
 * @brief Publisher slot - holds channel writer and write state
 *
 * Owned by the Node, shared with Publisher handles and RclPubSystem.
 *
 * ## Template parameters
 *
 * - `T`: message type
 * - `Topic`: topic tag type
 */
template <typename T, typename Topic = DefaultTopic>
struct PubSlot {
    std::optional<spmc_mut<T, Topic>> writer;
    std::atomic<bool> bound{false};         ///< Has bind() been called?
    std::atomic<bool> pending_write{false}; ///< Is there a pending write for run() to consume?
    std::atomic<std::uint64_t>* ready_systems = nullptr;
    std::uint64_t ready_bit                   = 0;

    // Callback ownership tracking only; scheduling still flows through RclPubSystem.
    std::atomic<RclSubSystemBase*> owner_callback{
        nullptr};                        ///< The callback that owns this publisher
    std::atomic<bool> auto_bound{false}; ///< Whether auto-binding has occurred

    /**
     * @brief Bind this slot to the world
     *
     * Called by RclPubSystem::bind()
     *
     * ## Parameters
     *
     * - `world`: the world
     */
    void bind(World& world) noexcept {
        world.open_channel_binding();
        writer = world.get<spmc_mut<T, Topic>>();
        world.close_channel_binding();
        bound.store(true, std::memory_order_release);
    }

    void bind_ready_target(
        std::atomic<std::uint64_t>* ready_mask, const std::size_t system_index) noexcept {
        ready_systems = ready_mask;
        ready_bit     = 1ULL << system_index;
    }

    /**
     * @brief Publish a message
     *
     * Called by Publisher::publish()
     *
     * ## Parameters
     *
     * - `msg`: message to publish (moved)
     */
    void publish(T msg) noexcept {
        if (bound.load(std::memory_order_acquire)) [[likely]] {
            writer->write(std::move(msg));
            pending_write.store(true, std::memory_order_release);
            if (ready_systems != nullptr) [[likely]] {
                ready_systems->fetch_or(ready_bit, std::memory_order_release);
            }
        }
    }

    /**
     * @brief Check if the slot is ready to publish
     *
     * ## Returns
     *
     * `true` if bound to the world
     */
    [[nodiscard]] bool ready() const noexcept { return bound.load(std::memory_order_acquire); }
};

// ============================================================================
// Publisher: Move-only handle to PubSlot
// ============================================================================

/**
 * @brief Move-only publisher handle
 *
 * Lightweight handle that references a PubSlot owned by a Node.
 * Move-only to enforce single-ownership at compile time. The slot itself is
 * shared with scheduler-owned systems, so the handle may outlive the Node.
 *
 * ## Template parameters
 *
 * - `T`: message type
 * - `Topic`: topic tag type
 *
 * ## Example
 *
 * ```cpp
 * auto pub = node.create_publisher<ImageFrame, CameraTag>();
 * pub.publish(ImageFrame{...});
 *
 * // Move to another scope
 * auto pub2 = std::move(pub);
 * pub2.publish(ImageFrame{...});
 *
 * // pub is now invalid
 * ```
 */
template <typename T, typename Topic = DefaultTopic>
class Publisher {
public:
    Publisher() noexcept = default;

    /**
     * @brief Construct a publisher handle
     *
     * ## Parameters
     *
     * - `slot`: shared ownership of the PubSlot
     * - `registry`: shared ownership of the OwnershipRegistry state
     * - `key`: channel key for this publisher
     */
    Publisher(
        std::shared_ptr<PubSlot<T, Topic>> slot, std::shared_ptr<OwnershipRegistry> registry,
        const ChannelKey key) noexcept
        : slot_(slot)
        , registry_(registry)
        , key_(key) {}

    // Move-only
    Publisher(const Publisher&)            = delete;
    Publisher& operator=(const Publisher&) = delete;

    Publisher(Publisher&& other) noexcept
        : slot_(std::move(other.slot_))
        , registry_(std::move(other.registry_))
        , key_(other.key_) {
        other.slot_.reset();
        other.registry_.reset();
    }

    Publisher& operator=(Publisher&& other) noexcept {
        if (this != &other) {
            // Release current claim if any
            if (slot_ && registry_) {
                registry_->release_claim(key_);
            }
            slot_     = std::move(other.slot_);
            registry_ = std::move(other.registry_);
            key_      = other.key_;
            other.slot_.reset();
            other.registry_.reset();
        }
        return *this;
    }

    ~Publisher() noexcept {
        if (slot_ && registry_) {
            registry_->release_claim(key_);
        }
    }

    /**
     * @brief Publish a message
     *
     * ## Parameters
     *
     * - `msg`: message to publish (moved)
     *
     * ## Panics
     *
     * Aborts if the publisher is invalid (moved-from) or if used from a different
     * callback than the one that first claimed ownership.
     */
    void publish(T msg) noexcept {
        if (!slot_) [[unlikely]] {
            panic("Publisher::publish() called on invalid (moved-from) publisher");
        }

        // Automatic callback ownership check. This is not part of graph construction.
        auto* current_callback = g_callback_context.current_system;

        // Helper lambda: log ownership error
        auto abort_ownership_error = [&](RclSubSystemBase* owner, RclSubSystemBase* used_by) {
            panic(
                "Publisher {}@{} owned by callback '{}', used by '{}'",
                scheduler::detail::demangle(typeid(T).name()),
                scheduler::detail::demangle(typeid(Topic).name()),
                owner ? owner->meta().name.c_str() : "<unknown>",
                used_by ? used_by->meta().name.c_str() : "<unknown>");
        };

        if (current_callback && !slot_->auto_bound.load(std::memory_order_acquire)) {
            // First use: attempt to claim ownership for the current callback
            RclSubSystemBase* expected = nullptr;
            if (slot_->owner_callback.compare_exchange_strong(
                    expected, current_callback, std::memory_order_release,
                    std::memory_order_acquire)) {
                // Successfully bound to this callback
                slot_->auto_bound.store(true, std::memory_order_release);
            } else if (expected != current_callback) {
                // Another callback already owns this publisher
                abort_ownership_error(expected, current_callback);
            }
        } else if (current_callback) {
            // Already bound: verify the current callback is the owner
            auto* owner = slot_->owner_callback.load(std::memory_order_acquire);
            if (owner && owner != current_callback) {
                abort_ownership_error(owner, current_callback);
            }
        }

        slot_->publish(std::move(msg));
    }

    /**
     * @brief Check if the publisher is valid
     *
     * ## Returns
     *
     * `true` if this publisher handle is valid
     */
    [[nodiscard]] bool valid() const noexcept { return slot_ != nullptr; }

    /**
     * @brief Check if the publisher is ready to publish
     *
     * ## Returns
     *
     * `true` if the underlying slot is bound
     */
    [[nodiscard]] bool ready() const noexcept { return slot_ && slot_->ready(); }

private:
    std::shared_ptr<PubSlot<T, Topic>> slot_{};
    std::shared_ptr<OwnershipRegistry> registry_{};
    ChannelKey key_{typeid(void), typeid(void)};
};

/**
 * @brief System that manages a publisher channel
 *
 * This system is the single graph-visible writer for a publisher channel.
 * Publisher::publish() writes into the slot and wakes this compute source,
 * then run() consumes the pending_write flag to propagate data downstream.
 *
 * ## Template parameters
 *
 * - `T`: message type
 * - `Topic`: topic tag type
 * - `Policy`: execution policy (default: pool_compute)
 */
template <typename T, typename Topic = DefaultTopic, typename Policy = pool_compute>
class RclPubSystem
    : public RclSubSystemBase
    , public ExternalComputeSource {
public:
    /**
     * @brief Construct a publisher system
     *
     * ## Parameters
     *
     * - `name`: system name
     * - `slot`: shared ownership of the PubSlot to manage
     */
    explicit RclPubSystem(std::string name, std::shared_ptr<PubSlot<T, Topic>> slot) noexcept
        : slot_(slot) {
        meta_ = build_meta<Policy>(std::move(name), [](auto& builder) {
            builder.add_spmc_writer(typeid(T), typeid(Topic));
        });
    }

    void bind(World& world) noexcept override { slot_->bind(world); }

    /// RTTI-free dispatch: return this as ExternalComputeSource
    ExternalComputeSource* as_external_compute() noexcept override { return this; }

    void bind_external_ready_slot(
        std::atomic<std::uint64_t>* ready_systems,
        const std::size_t system_index) noexcept override {
        slot_->bind_ready_target(ready_systems, system_index);
    }

    bool run([[maybe_unused]] World& world) noexcept override {
        // Consume the pending_write flag atomically
        return slot_->pending_write.exchange(false, std::memory_order_acq_rel);
    }

private:
    std::shared_ptr<PubSlot<T, Topic>> slot_;
};

/**
 * @brief System that manages a subscription
 *
 * Reads from an SPMC channel and invokes a callback.
 *
 * ## Template parameters
 *
 * - `T`: message type
 * - `Topic`: topic tag type
 * - `Policy`: execution policy (default: pool_compute)
 */
template <typename T, typename Topic = DefaultTopic, typename Policy = pool_compute>
class RclSubSystem : public RclSubSystemBase {
public:
    using Callback = std::function<void(const T&)>;

    explicit RclSubSystem(std::string name, Callback callback) noexcept
        : callback_(std::move(callback)) {
        meta_ = build_meta<Policy>(std::move(name), [](auto& builder) {
            builder.add_spmc_reader(typeid(T), typeid(Topic));
        });
    }

    void bind(World& world) noexcept override { reader_ = world.get<spmc<T, Topic>>(); }

    bool run([[maybe_unused]] World& world) noexcept override {
        if (!reader_.has_new()) {
            return false;
        }

        auto msg = reader_.read();
        if (msg) {
            // Set thread-local context so Publisher ownership checks can identify the callback.
            CallbackContextScope scope(this);
            callback_(*msg);
        }

        return false;
    }

private:
    Callback callback_;
    spmc<T, Topic> reader_;
};

// ============================================================================
// ResourceAccessor: Proxy for accessing World resources
// ============================================================================

/**
 * @brief Proxy for accessing resources from the World
 *
 * Similar to Publisher, this is a lightweight handle that provides access
 * to resources stored in the World. Can be stored as a class member and
 * used from callbacks. Access is rejected after the owning World dies.
 *
 * ## Template parameters
 *
 * - `T`: resource type
 *
 * ## Example
 *
 * ```cpp
 * class Detector {
 *     ResourceAccessor<TfBuffer> tf_buffer_;
 *     Publisher<Detection> det_pub_;
 *
 *     void init(Node& node) {
 *         tf_buffer_ = node.create_resource<TfBuffer>();
 *         det_pub_ = node.create_publisher<Detection>();
 *
 *         node.create_subscription<Image>([this](const Image& img) {
 *             auto tf = tf_buffer_.get();  // Access resource
 *             det_pub_.publish(process(img, tf));
 *         });
 *     }
 * };
 * ```
 */
template <typename T>
class ResourceAccessor {
public:
    ResourceAccessor() noexcept = default;

    /**
     * @brief Construct a resource accessor
     *
     * ## Parameters
     *
     * - `world`: pointer to the world (non-owning, checked by lifetime token)
     * - `lifetime`: token used to detect a dead World/Scheduler owner
     */
    explicit ResourceAccessor(World* world, std::weak_ptr<WorldLifetimeToken> lifetime) noexcept
        : world_(world)
        , lifetime_(std::move(lifetime)) {}

    /**
     * @brief Get immutable access to the resource
     *
     * ## Returns
     *
     * `res<T>` handle for reading the resource
     *
     * ## Thread safety
     *
     * Multiple threads can call get() concurrently.
     * Resource identity is expected to remain stable after the first successful build.
     */
    [[nodiscard]] res<T> get() const noexcept { return validate_world()->template get_res<T>(); }

    /**
     * @brief Get mutable access to the resource
     *
     * ## Returns
     *
     * `res_mut<T>` handle for reading/writing the resource
     *
     * ## Thread safety
     *
     * Multiple threads can call get_mut(), but concurrent writes must be
     * synchronized externally by the resource owner.
     * Resource identity is expected to remain stable after the first successful build.
     */
    [[nodiscard]] res_mut<T> get_mut() const noexcept {
        return validate_world()->template get_res_mut<T>();
    }

    /**
     * @brief Check if a resource accessor is valid
     *
     * ## Returns
     *
     * `true` if this accessor is bound to a World
     */
    [[nodiscard]] bool valid() const noexcept { return world_ != nullptr && !lifetime_.expired(); }

    // Copyable (multiple accessors can reference the same World)
    ResourceAccessor(const ResourceAccessor&)            = default;
    ResourceAccessor& operator=(const ResourceAccessor&) = default;

private:
    World* world_ = nullptr;
    std::weak_ptr<WorldLifetimeToken> lifetime_;

    [[nodiscard]] World* validate_world() const noexcept {
        if (!world_ || lifetime_.expired()) [[unlikely]] {
            panic("ResourceAccessor::get() called on invalid accessor");
        }
        return world_;
    }
};

} // namespace talos::scheduler::rclcompat
