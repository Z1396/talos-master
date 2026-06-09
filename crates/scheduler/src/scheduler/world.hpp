#pragma once

#include "demangle.hpp"
#include "error.hpp"
#include "system/components.hpp"
#include "system/system_meta.hpp"

#include <atomic>
#include <concepts>
#include <cstdint>
#include <memory>
#include <spdlog/spdlog.h>
#include <type_traits>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

namespace talos::scheduler {

namespace rclcompat {
template <typename T, typename Topic>
struct PubSlot;
} // namespace rclcompat

// ============================================================================
// Move-only type erasure
// ============================================================================
// Similar in spirit to std::any, but intentionally move-only.
// This is suitable for storing unique_ptr<T>, move-only channel storage, and
// non-copyable resources without forcing the underlying storage to inherit from
// a virtual base.

class UniqueAny {
    struct Concept {
        virtual ~Concept()                                  = default;
        virtual const std::type_info& type() const noexcept = 0;
    };

    template <typename T>
    struct Model final : Concept {
        T value;

        template <typename... Args>
        explicit Model(std::in_place_t, Args&&... args) noexcept(
            std::is_nothrow_constructible_v<T, Args&&...>)
            : value(std::forward<Args>(args)...) {}

        const std::type_info& type() const noexcept override { return typeid(T); }
    };

    std::unique_ptr<Concept> ptr_;

public:
    UniqueAny()  = default;
    ~UniqueAny() = default;

    UniqueAny(UniqueAny&&) noexcept            = default;
    UniqueAny& operator=(UniqueAny&&) noexcept = default;

    UniqueAny(const UniqueAny&)            = delete;
    UniqueAny& operator=(const UniqueAny&) = delete;

    template <typename T, typename... Args>
    static UniqueAny make(Args&&... args) {
        UniqueAny out;
        out.emplace<T>(std::forward<Args>(args)...);
        return out;
    }

    template <typename T, typename... Args>
    T& emplace(Args&&... args) {
        ptr_ = std::make_unique<Model<T>>(std::in_place, std::forward<Args>(args)...);
        return static_cast<Model<T>*>(ptr_.get())->value;
    }

    [[nodiscard]] bool has_value() const noexcept { return static_cast<bool>(ptr_); }

    [[nodiscard]] const std::type_info& type() const noexcept {
        return ptr_ ? ptr_->type() : typeid(void);
    }

    template <typename T>
    [[nodiscard]] T* get() noexcept {
        if (!ptr_ || ptr_->type() != typeid(T)) {
            return nullptr;
        }
        return &static_cast<Model<T>*>(ptr_.get())->value;
    }

    template <typename T>
    [[nodiscard]] const T* get() const noexcept {
        if (!ptr_ || ptr_->type() != typeid(T)) {
            return nullptr;
        }
        return &static_cast<const Model<T>*>(ptr_.get())->value;
    }

    template <typename T>
    [[nodiscard]] T& as() noexcept {
        auto* p = get<T>();
        if (!p) {
            panic(
                "UniqueAny bad cast: requested {} but stored {}",
                detail::demangle(typeid(T).name()), detail::demangle(type().name()));
        }
        return *p;
    }

    template <typename T>
    [[nodiscard]] const T& as() const noexcept {
        auto* p = get<T>();
        if (!p) {
            panic(
                "UniqueAny bad cast: requested {} but stored {}",
                detail::demangle(typeid(T).name()), detail::demangle(type().name()));
        }
        return *p;
    }
};

// ============================================================================
// Resource Wrapper
// ============================================================================

template <typename T>
struct Resource {
    T value;

    Resource() noexcept(std::is_nothrow_default_constructible_v<T>)
        requires std::default_initializable<T>
        = default;

    template <typename... Args>
    explicit Resource(std::in_place_t, Args&&... args) noexcept(
        std::is_nothrow_constructible_v<T, Args&&...>)
        : value(std::forward<Args>(args)...) {}

    template <typename U>
    explicit Resource(U&& val) noexcept(std::is_nothrow_constructible_v<T, U&&>)
        : value(std::forward<U>(val)) {}
};

struct WorldLifetimeToken {};

// ============================================================================
// Design Note: Type Erasure for Heterogeneous Storage
// ============================================================================
// World stores arbitrary resource and channel storage types behind UniqueAny.
// UniqueAny is intentionally move-only, unlike std::any. This allows the world
// to store move-only resources and unique_ptr-backed channel storage without
// forcing the actual storage types into a virtual inheritance hierarchy.
//
// Public APIs remain typed. All casts happen at the boundary where the world
// already knows the exact T from the template call site.
// ============================================================================

// ============================================================================
// SPSC Storage: holds split writer and reader
// ============================================================================

template <typename T>
struct SpscStorage {
    using Channel    = primitive::SpscChannel<T>;
    using Writer     = Channel::Writer;
    using Reader     = Channel::Reader;
    using value_type = T;

    std::unique_ptr<Writer> writer;
    std::unique_ptr<Reader> reader;
    bool writer_claimed = false;
    bool reader_claimed = false;

    static std::unique_ptr<SpscStorage> create() {
        auto ch         = primitive::make_spsc_channel<T>();
        auto [w, r]     = ch.split();
        auto storage    = std::make_unique<SpscStorage>();
        storage->writer = std::make_unique<Writer>(std::move(w));
        storage->reader = std::make_unique<Reader>(std::move(r));
        return storage;
    }
};

// ============================================================================
// SPMC Storage: holds channel and reader clones
// ============================================================================

template <typename T>
struct SpmcStorage {
    using Channel    = primitive::SpmcChannel<T>;
    using Reader     = Channel::Reader;
    using value_type = T;

    std::unique_ptr<Channel> channel;
    std::vector<std::unique_ptr<Reader>> readers;

    static std::unique_ptr<SpmcStorage> create() {
        auto storage     = std::make_unique<SpmcStorage>();
        storage->channel = std::make_unique<Channel>(primitive::make_spmc_channel<T>());
        return storage;
    }

    Reader* add_reader() {
        auto reader = std::make_unique<Reader>(channel->clone_reader());
        auto* raw   = reader.get();
        readers.push_back(std::move(reader));
        return raw;
    }
};

// ============================================================================
// ResourceStore
// ============================================================================

class ResourceStore {
public:
    template <typename T, typename... Args>
    [[nodiscard]] T& emplace(Args&&... args) {
        using U = std::remove_cvref_t<T>;

        auto [it, inserted] = resources_.try_emplace(typeid(U));

        if (!inserted) [[unlikely]] {
            panic("Resource already exists: {}", detail::demangle(typeid(U).name()));
        }

        auto& storage =
            it->second.template emplace<Resource<U>>(std::in_place, std::forward<Args>(args)...);

        return storage.value;
    }

    template <typename T>
    [[nodiscard]] bool contains() const noexcept {
        return resources_.contains(typeid(T));
    }

    template <typename T>
    [[nodiscard]] const Resource<T>* get_storage() const noexcept {
        const auto it = resources_.find(typeid(T));
        if (it == resources_.end()) [[unlikely]] {
            return nullptr;
        }
        return it->second.template get<Resource<T>>();
    }

    template <typename T>
    [[nodiscard]] Resource<T>* get_storage_mut() noexcept {
        const auto it = resources_.find(typeid(T));
        if (it == resources_.end()) [[unlikely]] {
            return nullptr;
        }
        return it->second.template get<Resource<T>>();
    }

    template <typename T>
    [[nodiscard]] const T& get() const noexcept {
        auto* storage = get_storage<T>();
        if (!storage) [[unlikely]] {
            panic("Resource not found: {}", detail::demangle(typeid(T).name()));
        }
        return storage->value;
    }

    template <typename T>
    [[nodiscard]] T& get_mut() noexcept {
        auto* storage = get_storage_mut<T>();
        if (!storage) [[unlikely]] {
            panic("Resource not found: {}", detail::demangle(typeid(T).name()));
        }
        return storage->value;
    }

private:
    std::unordered_map<std::type_index, UniqueAny> resources_;
};

// ============================================================================
// ChannelStore
// ============================================================================

class ChannelStore {
public:
    void open_binding() noexcept { channel_binding_depth_.fetch_add(1, std::memory_order_acq_rel); }

    void close_binding() noexcept {
        channel_binding_depth_.fetch_sub(1, std::memory_order_acq_rel);
    }

    [[nodiscard]] bool binding_open() const noexcept {
        return channel_binding_depth_.load(std::memory_order_acquire) != 0;
    }

    template <typename Storage, typename Topic = system::DefaultTopic>
    [[nodiscard]] Storage* get_storage() noexcept {
        ensure_binding_open();

        using T = typename Storage::value_type;
        const system::ChannelKey key{typeid(T), typeid(Topic)};

        auto& storage_map   = select_storage_map<Storage>();
        auto [it, inserted] = storage_map.try_emplace(key);

        if (inserted) {
            it->second.template emplace<std::unique_ptr<Storage>>(Storage::create());
        }

        auto* ptr = it->second.template get<std::unique_ptr<Storage>>();

        if (!ptr || !*ptr) [[unlikely]] {
            panic(
                "Channel storage type mismatch: {}@{}", detail::demangle(typeid(T).name()),
                detail::demangle(typeid(Topic).name()));
        }

        return ptr->get();
    }

private:
    std::unordered_map<system::ChannelKey, UniqueAny, system::ChannelKeyHash> spsc_storage_;
    std::unordered_map<system::ChannelKey, UniqueAny, system::ChannelKeyHash> spmc_storage_;
    std::atomic<std::uint32_t> channel_binding_depth_{0};

    void ensure_binding_open() const noexcept {
        if (!binding_open()) [[unlikely]] {
            panic("Channel endpoints can only be claimed during scheduler-controlled bind phases");
        }
    }

    template <typename Storage>
    [[nodiscard]] auto& select_storage_map() noexcept {
        using T = typename Storage::value_type;

        if constexpr (std::is_same_v<Storage, SpscStorage<T>>) {
            return spsc_storage_;
        } else {
            return spmc_storage_;
        }
    }
};

// ============================================================================
// World: Resource + Channel Container
// ============================================================================

using namespace system;
using namespace talos::scheduler::detail;

class World {
public:
    // ==================== Resource API ====================

    template <typename T>
    void insert_resource(T&& resource) {
        using U = std::remove_cvref_t<T>;
        SPDLOG_DEBUG("register {}", demangle(typeid(T).name()));
        ensure_resource_structure_mutable<U>();
        static_cast<void>(resources_.template emplace<U>(std::forward<T>(resource)));
    }

    template <typename T, typename... Args>
    [[nodiscard]] T& emplace_resource(Args&&... args) {
        using U = std::remove_cvref_t<T>;
        ensure_resource_structure_mutable<U>();
        return resources_.template emplace<U>(std::forward<Args>(args)...);
    }

    template <typename T>
    void unsafe_insert_resource(T&& resource) {
        using U = std::remove_cvref_t<T>;
        static_cast<void>(resources_.template emplace<U>(std::forward<T>(resource)));
    }

    template <typename T, typename... Args>
    [[nodiscard]] T& unsafe_emplace_resource(Args&&... args) {
        using U = std::remove_cvref_t<T>;
        return resources_.template emplace<U>(std::forward<Args>(args)...);
    }

    void freeze_resource_structure() noexcept;

    [[nodiscard]] bool resource_structure_frozen() const noexcept;

    void freeze_resource_identity() noexcept;

    [[nodiscard]] bool resource_identity_frozen() const noexcept;

    template <typename T>
    [[nodiscard]] bool has_resource() const noexcept {
        return resources_.template contains<T>();
    }

    /// Get read-only resource. Panics if resource not found.
    template <typename T>
    [[nodiscard]] res<T> get_res() const noexcept {
        return res<T>{&resources_.template get<T>()};
    }

    /// Get mutable resource. Panics if resource not found.
    template <typename T>
    [[nodiscard]] res_mut<T> get_res_mut() noexcept {
        return res_mut<T>{&resources_.template get_mut<T>()};
    }

    // ==================== Type-Dispatched Access ====================

    template <typename ComponentT>
    [[nodiscard]] auto get() noexcept {
        return get_impl(ComponentT{});
    }

    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_spsc_reader() noexcept {
        return get<spsc<T, Topic>>();
    }

    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_spsc_writer() noexcept {
        return get<spsc_mut<T, Topic>>();
    }

    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_spmc_reader() noexcept {
        return get<spmc<T, Topic>>();
    }

    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_spmc_writer() noexcept {
        return get<spmc_mut<T, Topic>>();
    }

private:
    friend class Scheduler;
    template <typename T, typename Topic>
    friend struct rclcompat::PubSlot;

    void open_channel_binding() noexcept { channels_.open_binding(); }

    void close_channel_binding() noexcept { channels_.close_binding(); }

    [[nodiscard]] bool channel_binding_open() const noexcept { return channels_.binding_open(); }

    // Unified channel accessor: one template for member-pointer-based access.
    template <typename Ret, typename Topic, typename Storage, auto MemberPtr>
    [[nodiscard]] auto get_channel_member() noexcept {
        auto* s = get_channel_storage<Storage, Topic>();
        return Ret{(s->*MemberPtr).get()};
    }

    // Channel implementations.
    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_impl(spsc<T, Topic>) noexcept {
        auto* s = get_channel_storage<SpscStorage<T>, Topic>();
        if (s->reader_claimed) [[unlikely]] {
            panic(
                "SPSC reader already bound: {}@{}", demangle(typeid(T).name()),
                demangle(typeid(Topic).name()));
        }
        s->reader_claimed = true;
        return spsc<T, Topic>{s->reader.get()};
    }

    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_impl(spsc_mut<T, Topic>) noexcept {
        auto* s = get_channel_storage<SpscStorage<T>, Topic>();
        if (s->writer_claimed) [[unlikely]] {
            panic(
                "SPSC writer already bound: {}@{}", demangle(typeid(T).name()),
                demangle(typeid(Topic).name()));
        }
        s->writer_claimed = true;
        return spsc_mut<T, Topic>{s->writer.get()};
    }

    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_impl(spmc<T, Topic>) noexcept {
        auto* s = get_channel_storage<SpmcStorage<T>, Topic>();
        return spmc<T, Topic>{s->add_reader()};
    }

    template <typename T, typename Topic = DefaultTopic>
    [[nodiscard]] auto get_impl(spmc_mut<T, Topic>) noexcept {
        return get_channel_member<
            spmc_mut<T, Topic>, Topic, SpmcStorage<T>, &SpmcStorage<T>::channel>();
    }

    template <typename T>
    [[nodiscard]] auto get_impl(res<T>) noexcept {
        return get_res<T>();
    }

    template <typename T>
    [[nodiscard]] auto get_impl(res_mut<T>) noexcept {
        return get_res_mut<T>();
    }

    template <typename Storage, typename Topic = DefaultTopic>
    [[nodiscard]] auto* get_channel_storage() noexcept {
        return channels_.template get_storage<Storage, Topic>();
    }

private:
    ResourceStore resources_;
    ChannelStore channels_;
    std::atomic<bool> resource_structure_frozen_{false};
    std::shared_ptr<WorldLifetimeToken> lifetime_token_ = std::make_shared<WorldLifetimeToken>();

    template <typename T>
    void ensure_resource_structure_mutable() const noexcept {
        if (resource_structure_frozen_.load(std::memory_order_acquire)) [[unlikely]] {
            panic(
                "Mutating resource structure for '{}' after build() breaks scheduler invariants; "
                "use unsafe_insert_resource()/unsafe_emplace_resource() only if you need the "
                "explicit escape hatch",
                demangle(typeid(T).name()));
        }
    }

public:
    [[nodiscard]] std::weak_ptr<WorldLifetimeToken> lifetime_token() const noexcept;
};

template <typename T, typename Topic = DefaultTopic>
using publish = spmc_mut<T, Topic>;

template <typename T, typename Topic = DefaultTopic>
using subscribe = spmc<T, Topic>;

} // namespace talos::scheduler

// ============================================================================
// Convenience aliases for talos::
// ============================================================================

namespace talos {

template <typename T, typename Topic = scheduler::DefaultTopic>
using subscribe = scheduler::spmc<T, Topic>;

template <typename T, typename Topic = scheduler::DefaultTopic>
using publish = scheduler::spmc_mut<T, Topic>;

} // namespace talos
