#pragma once

#include "../demangle.hpp"
#include "../error.hpp"
#include "../scheduler.hpp"
#include "../system/components.hpp"
#include "../system/system.hpp"
#include "registry.hpp"
#include "system.hpp"
#include "timer_constants.hpp"

#include <any>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace talos::scheduler::rclcompat {
using namespace system;
using namespace talos::scheduler::detail;
// ============================================================================
// Node: Container for publishers and subscriptions
// ============================================================================

enum class Qos {
    Volatile, ///< No history, latest value only
};

/**
 * @brief ROS2-style node container
 *
 * A Node owns publishers and subscriptions and manages their lifecycle.
 * Call finalize() after creating all pub/sub to register systems with the scheduler.
 *
 * ## Thread safety
 *
 * - create_publisher() / create_subscription() must be called from a single thread
 * - finalize() must be called before scheduler.run()
 * - unsafe_finalize() keeps the experimental running hot-add path for callers that explicitly opt
 *   into it
 * - Publisher::publish() is thread-safe after finalize() and wakes the internal publisher system
 *
 * ## Lifetime
 *
 * Publisher slots are shared with scheduler-owned systems, so Publisher handles may outlive
 * the Node object itself. `unsafe_finalize()` is not transactional across multiple pending
 * systems.
 */
class Node {
public:
    /**
     * @brief Construct a node
     *
     * ## Parameters
     *
     * - `name`: node name (used as prefix for system names)
     * - `scheduler`: reference to the scheduler to register systems with
     */
    explicit Node(std::string name, Scheduler& scheduler) noexcept;

    // Non-copyable, non-movable (owns references to internal data)
    Node(const Node&)            = delete;
    Node& operator=(const Node&) = delete;
    Node(Node&&)                 = delete;
    Node& operator=(Node&&)      = delete;

    /**
     * @brief Create a publisher
     *
     * Returns a move-only Publisher handle. The underlying PubSlot is owned by this Node.
     *
     * ## Template parameters
     *
     * - `T`: message type
     * - `Topic`: topic tag type (default: DefaultTopic)
     *
     * ## Parameters
     *
     * - `qos`: quality of service settings (currently unused)
     *
     * ## Returns
     *
     * Move-only Publisher handle
     *
     * ## Panics
     *
     * Aborts if a publisher for this channel already exists
     */
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] Publisher<T, Topic> create_publisher([[maybe_unused]] Qos qos = Qos::Volatile) {
        const ChannelKey key{typeid(T), typeid(Topic)};
        const std::string system_name = make_system_name<T, Topic>("pub");

        // Check if this channel is already owned
        if (pub_slots_.contains(key)) {
            using namespace talos::scheduler;
            panic(
                "Node '{}': Publisher for channel {}@{} already exists", name_,
                demangle(key.type.name()), demangle(key.topic.name()));
        }

        // Create PubSlot using shared_ptr (std::any requires copy-constructibility)
        auto slot       = std::make_shared<PubSlot<T, Topic>>();
        pub_slots_[key] = slot;

        // Register ownership
        registry_->register_owner(key, system_name);

        // Claim the publisher handle
        if (!registry_->try_claim(key)) {
            using namespace talos::scheduler;
            panic(
                "Node '{}': Failed to claim publisher for channel {}@{}", name_,
                demangle(key.type.name()), demangle(key.topic.name()));
        }

        // Create the system (deferred registration until finalize())
        pending_systems_.push_back(std::make_unique<RclPubSystem<T, Topic>>(system_name, slot));

        return Publisher<T, Topic>(slot, registry_, key);
    }

    /**
     * @brief Create a subscription
     *
     * Registers a callback to be invoked when messages are received.
     *
     * ## Template parameters
     *
     * - `T`: message type
     * - `Topic`: topic tag type (default: DefaultTopic)
     *
     * ## Parameters
     *
     * - `callback`: function to call when a message is received
     * - `qos`: quality of service settings (currently unused)
     */
    template <typename T, typename Topic = DefaultTopic>
    void create_subscription(
        std::function<void(const T&)> callback, [[maybe_unused]] Qos qos = Qos::Volatile) {
        const std::string system_name = make_system_name<T, Topic>("sub");

        pending_systems_.push_back(
            std::make_unique<RclSubSystem<T, Topic>>(system_name, std::move(callback)));
    }

    /**
     * @brief Create a wall timer
     *
     * Creates a timer that invokes a callback at a fixed frequency.
     * The callback runs on a dedicated fixed_rate thread.
     *
     * ## Parameters
     *
     * - `frequency`: Timer frequency from the Frequency enum (Hz_1 to Hz_1000)
     * - `callback`: function to call at each timer tick (taken by value to enable move semantics)
     *
     * ## Example
     *
     * ```cpp
     * // Simple timer at 10Hz
     * node.create_wall_timer(Frequency::Hz_10, []() {
     *     // Executes at 10Hz
     * });
     *
     * // Timer with Publisher at 30Hz
     * auto pub = node.create_publisher<ImageFrame>();
     * node.create_wall_timer(Frequency::Hz_30, [&pub]() {
     *     pub.publish(...);
     * });
     *
     * // Timer with ResourceAccessor at 20Hz
     * auto tf = node.create_resource<TfBuffer>();
     * node.create_wall_timer(Frequency::Hz_20, [&tf]() {
     *     auto transform = tf.get();
     * });
     * ```
     *
     * ## Note
     *
     * The callback is taken by value (not const reference) to enable efficient
     * move semantics when forwarding to the underlying timer system. For large
     * capture lists, consider capturing by reference in the lambda itself.
     */
    void create_wall_timer(Frequency frequency, std::function<void()> callback);

    /**
     * @brief Create a resource accessor
     *
     * Returns a copyable ResourceAccessor handle that can be used to access
     * resources from the World. Can be stored as a class member and used from
     * callbacks.
     *
     * ## Template parameters
     *
     * - `T`: resource type
     *
     * ## Returns
     *
     * ResourceAccessor handle for accessing the resource
     *
     * ## Contract
     *
     * Resource values may be mutated after build/finalize, but safe-path structural changes
     * after the first successful build are rejected.
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
    [[nodiscard]] ResourceAccessor<T> create_resource() noexcept {
        auto& world = scheduler_.world();
        return ResourceAccessor<T>(&world, world.lifetime_token());
    }

    /**
     * @brief Check if a resource exists in the World
     *
     * ## Template parameters
     *
     * - `T`: resource type
     *
     * ## Returns
     *
     * `true` if the resource exists
     */
    template <typename T>
    [[nodiscard]] bool has_resource() const noexcept {
        return scheduler_.world().has_resource<T>();
    }

    /**
     * @brief Insert a resource into the World
     *
     * ## Template parameters
     *
     * - `T`: resource type
     *
     * ## Parameters
     *
     * - `resource`: the resource to insert (moved)
     *
     * ## Contract
     *
     * Safe-path structural resource mutation after the first successful scheduler build/finalize
     * is rejected because bound systems may cache raw resource pointers.
     */
    template <typename T>
    void insert_resource(T&& resource) {
        scheduler_.world().insert_resource(std::forward<T>(resource));
    }

    /**
     * @brief Experimental resource-structure escape hatch
     *
     * Bypasses the safe-path post-build resource freeze. Callers are responsible for any
     * synchronization and pointer-stability consequences.
     */
    template <typename T>
    void unsafe_insert_resource(T&& resource) {
        scheduler_.world().unsafe_insert_resource(std::forward<T>(resource));
    }

    /**
     * @brief Get the node name
     *
     * ## Returns
     *
     * Node name as string_view
     */
    [[nodiscard]] std::string_view name() const noexcept;

    /**
     * @brief Finalize and register all systems with the scheduler
     *
     * Must be called after creating all publishers and subscriptions and before `scheduler.run()`.
     *
     * ## Returns
     *
     * BuildResult indicating success or failure
     *
     * ## Contract
     *
     * This operation is not transactional. On failure, already-consumed systems stay registered
     * in the scheduler, while the remaining pending systems stay on the Node.
     *
     * ## Thread safety
     *
     * Must be called from a single thread. Calling this while the scheduler is running aborts
     * and points callers at `unsafe_finalize()`.
     */
    [[nodiscard]] BuildResult finalize();

    /**
     * @brief Experimental finalize escape hatch for runtime hot-add
     *
     * Preserves the legacy running finalize path by routing through
     * `Scheduler::unsafe_hot_add_system()`.
     */
    [[nodiscard]] BuildResult unsafe_finalize();

    /**
     * @brief Get the ownership registry
     *
     * ## Returns
     *
     * Reference to the ownership registry
     */
    [[nodiscard]] OwnershipRegistry& registry() noexcept;

    /**
     * @brief Get a raw pointer to a PubSlot (for advanced use)
     *
     * ## Template parameters
     *
     * - `T`: message type
     * - `Topic`: topic tag type
     *
     * ## Returns
     *
     * Pointer to the PubSlot, or nullptr if not found
     */
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] PubSlot<T, Topic>* get_pub_slot() noexcept {
        const ChannelKey key{typeid(T), typeid(Topic)};
        const auto it = pub_slots_.find(key);
        if (it == pub_slots_.end()) {
            return nullptr;
        }
        return std::any_cast<std::shared_ptr<PubSlot<T, Topic>>>(it->second).get();
    }

private:
    std::string name_;
    Scheduler& scheduler_;
    std::shared_ptr<OwnershipRegistry> registry_ = std::make_shared<OwnershipRegistry>();

    // Type-erased storage for PubSlots using shared_ptr for ownership
    std::unordered_map<ChannelKey, std::any, ChannelKeyHash> pub_slots_;

    std::vector<std::unique_ptr<SystemBase>> pending_systems_;

    [[nodiscard]] BuildResult finalize_impl(bool allow_running_finalize);

    /// Helper: build system name from type, topic, and kind (eliminates duplicate string
    /// construction)
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] std::string make_system_name(std::string_view kind) const {
        return name_ + "/" + std::string(kind) + "/" + demangle(typeid(Topic).name()) + "/"
             + demangle(typeid(T).name());
    }
};

} // namespace talos::scheduler::rclcompat

namespace talos {
namespace rclcompat = scheduler::rclcompat;
} // namespace talos
