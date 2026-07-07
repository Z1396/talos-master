#pragma once

// 共享内存布局头文件：定义ShmHeader、TripleBufferLayout三缓冲结构
#include "shm_layout.hpp"
// 三缓冲操作封装头文件：TripleBufferOps读写逻辑
#include "shm_triple_buffer.hpp"
// 原子操作：用于无锁缓冲区序列号标记
#include <atomic>
// 内存操作：memset、memcpy
#include <cstring>
// C++23 预期错误处理，替代异常返回错误码
#include <expected>
// Linux 文件打开模式宏 O_RDWR/O_CREAT/O_EXCL
#include <fcntl.h>
// 智能指针管理共享内存句柄
#include <memory>
// 可选容器，延迟打开读通道
#include <optional>
// 共享内存路径字符串存储
#include <string>
// Linux 文件状态结构体 stat
#include <sys/stat.h>
// Linux 共享内存映射、释放系统调用 mmap/munmap
#include <sys/mman.h>
// 系统调用：close、shm_unlink
#include <unistd.h>
// 移动语义转发、交换
#include <utility>

// 项目IPC命名空间：talos机器人框架 手性双向通道IPC
namespace talos::chiral::ipc {

// ============ 共享内存操作错误枚举类型 ============
enum class ShmError : uint8_t {
    OpenFailed      = 0,  // 共享内存文件打开失败
    TruncateFailed  = 1,  // 共享内存扩容ftruncate失败
    MapFailed       = 2,  // 虚拟地址映射mmap失败
    NotFound        = 3,  // 目标共享内存不存在
    InvalidMagic    = 4,  // 魔数校验不匹配，非本框架SHM
    VersionMismatch = 5,  // SHM版本号不兼容，无法互通
};

// ============ 全局常量定义 ============
// 共享内存魔数标识：ASCII "TLDT" 十六进制 0x544C4454，校验是否为本框架创建的SHM
inline constexpr uint32_t TALOS_SHM_MAGIC   = 0x544C4454;
// SHM布局版本 v2：采用三缓冲无锁架构
inline constexpr uint32_t TALOS_SHM_VERSION = 2;

// ============ 模板前置声明：共享内存命名规则 ============
// 外部需特化该模板，为每种消息类型定义唯一SHM文件名
template <typename T>
struct ShmName;

// ============ 共享内存头部布局结构体 ============
// 64字节对齐，保证缓存行隔离，避免伪共享
struct alignas(64) ShmHeader {
    uint32_t magic;        // 魔数标识，校验合法性
    uint32_t version;      // 版本号，兼容判断
    uint8_t _pad[56];      // 填充占位，补足64字节对齐
};
// 静态编译期断言：头部严格占用64字节，对齐无误
static_assert(sizeof(ShmHeader) == 64);

// ============ 完整共享内存模板布局 ============
// T 存储的消息数据类型
template <typename T>
struct ShmLayout {
    ShmHeader header;               // 头部校验信息
    TripleBufferLayout<T> buffer;   // 三缓冲无锁环形数据区
};

// ============ RAII 共享内存区域管理类 ============
// 封装shm_open/mmap/munmap/shm_unlink全部系统调用，自动资源释放
class ShmRegion {
public:
    // 析构：自动释放全部共享内存资源
    ~ShmRegion() noexcept {
        // 如果是创建者拥有所有权，销毁时删除共享内存文件
        if (owner_ && !path_.empty()) {
            (void)::shm_unlink(path_.c_str());
        }

        // 解除虚拟地址映射
        if (data_ != nullptr && data_ != MAP_FAILED) {
            munmap(data_, size_);
        }
        // 关闭文件描述符
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    // 移动构造：转移共享内存资源所有权，原对象置空
    ShmRegion(ShmRegion&& other) noexcept
        : data_(std::exchange(other.data_, nullptr))
        , size_(std::exchange(other.size_, 0))
        , fd_(std::exchange(other.fd_, -1))
        , owner_(std::exchange(other.owner_, false))
        , path_(std::move(other.path_)) {}

    // 移动赋值：转移资源，先清理当前持有资源
    ShmRegion& operator=(ShmRegion&& other) noexcept {
        if (this != &other) {
            cleanup(); // 释放自身原有资源
            // 交换转移对方资源
            data_  = std::exchange(other.data_, nullptr);
            size_  = std::exchange(other.size_, 0);
            fd_    = std::exchange(other.fd_, -1);
            owner_ = std::exchange(other.owner_, false);
            path_  = std::move(other.path_);
        }
        return *this;
    }

    // 禁用拷贝构造，共享内存资源不可复制
    ShmRegion(const ShmRegion&)            = delete;
    // 禁用拷贝赋值
    ShmRegion& operator=(const ShmRegion&) = delete;

    /**
     * @brief 创建全新共享内存，若存在旧SHM先尝试复用，损坏则重建
     * @param name 共享内存文件名
     * @param size 总占用字节大小
     * @return 成功返回ShmRegion，失败返回ShmError错误码
     */
    [[nodiscard]] static std::expected<ShmRegion, ShmError> create(const char* name, size_t size) {
        // 崩溃恢复逻辑：先尝试打开已有共享内存
        int fd = ::shm_open(name, O_RDWR, 0);
        if (fd >= 0) {
            struct stat st{};
            // 获取文件大小，若容量足够直接复用映射
            if (fstat(fd, &st) == 0 && static_cast<size_t>(st.st_size) >= size) {
                void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
                if (ptr != MAP_FAILED) {
                    // 复用已有SHM，当前进程不拥有所有权，不执行unlink
                    return ShmRegion(ptr, size, fd, false, name);
                }
            }
            // 映射失败，关闭fd并删除损坏的旧共享内存
            close(fd);
            shm_unlink(name);
        }

        // 创建全新共享内存：读写、创建、独占不存在则报错
        fd = ::shm_open(name, O_RDWR | O_CREAT | O_EXCL, 0644);
        if (fd < 0) {
            return std::unexpected(ShmError::OpenFailed);
        }

        // 扩容至指定size字节，共享内存初始长度为0
        if (ftruncate(fd, static_cast<off_t>(size)) < 0) {
            close(fd);
            shm_unlink(name);
            return std::unexpected(ShmError::TruncateFailed);
        }

        // 虚拟地址映射：读写、共享映射
        void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (ptr == MAP_FAILED) {
            close(fd);
            shm_unlink(name);
            return std::unexpected(ShmError::MapFailed);
        }

        // 内存全部清零初始化
        std::memset(ptr, 0, size);
        // 新建SHM，当前进程拥有所有权，析构自动删除文件
        return ShmRegion(ptr, size, fd, true, name);
    }

    /**
     * @brief 打开已存在的共享内存，仅读取映射，不拥有所有权
     * @param name 共享内存文件名
     * @param size 预期占用字节大小
     * @return 共享内存区域对象
     */
    [[nodiscard]] static std::expected<ShmRegion, ShmError> open(const char* name, size_t size) {
        const int fd = ::shm_open(name, O_RDWR, 0);
        if (fd < 0) {
            return std::unexpected(ShmError::NotFound);
        }

        void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (ptr == MAP_FAILED) {
            close(fd);
            return std::unexpected(ShmError::MapFailed);
        }

        return ShmRegion(ptr, size, fd, false, name);
    }

    /**
     * @brief 类型转换：将映射内存指针强转为指定布局结构体
     * @tparam T 目标结构体类型
     * @return 可修改结构体指针
     */
    template <typename T>
    [[nodiscard]] T* as() noexcept {
        return static_cast<T*>(data_);
    }

    /**
     * @brief 只读版本类型转换
     */
    template <typename T>
    [[nodiscard]] const T* as() const noexcept {
        return static_cast<const T*>(data_);
    }

    /**
     * @brief 判断当前映射的共享内存是否为目标名称对应的文件
     * 用于检测对端重启后共享内存重建，旧映射失效
     * @param name 共享内存文件名
     * @return 匹配返回true，失效/重建返回false
     */
    [[nodiscard]] bool is_current_mapping(const char* name) const noexcept {
        if (fd_ < 0) {
            return false;
        }

        struct stat current{};
        // 获取当前fd对应文件的设备、inode标识
        if (fstat(fd_, &current) < 0) {
            return false;
        }

        // 重新打开目标名称SHM获取stat
        const int named_fd = ::shm_open(name, O_RDWR, 0);
        if (named_fd < 0) {
            return false;
        }

        struct stat named{};
        // 设备号+inode完全一致代表是同一个文件，映射有效
        const bool ok = fstat(named_fd, &named) == 0 && current.st_dev == named.st_dev
                     && current.st_ino == named.st_ino;
        close(named_fd);
        return ok;
    }

private:
    // 私有构造：仅静态create/open函数调用
    ShmRegion(void* data, size_t size, int fd, bool owner, const char* name) noexcept
        : data_(data)
        , size_(size)
        , fd_(fd)
        , owner_(owner)
        , path_(name) {}

    // 统一资源清理逻辑
    void cleanup() noexcept {
        if (data_ != nullptr && data_ != MAP_FAILED) {
            munmap(data_, size_);
        }
        if (fd_ >= 0) {
            close(fd_);
        }
        if (owner_ && !path_.empty()) {
            (void)::shm_unlink(path_.c_str());
        }
    }

    void* data_{nullptr};   // 共享内存虚拟映射起始地址
    size_t size_{0};        // 共享内存总字节大小
    int fd_{-1};            // shm_open返回的文件描述符
    bool owner_{false};     // 是否拥有SHM所有权（创建者为true）
    std::string path_;      // 共享内存文件名路径
};

// ============ 单向写通道：生产者Writer ============
// T：写入消息类型，绑定一套独立SHM三缓冲
template <typename T>
class ChannelWriter {
public:
    // 析构：收尾未完成的写入序列号更新
    ~ChannelWriter() noexcept { finalize_write_if_needed(); }

    // 移动构造：转移共享内存区域所有权
    ChannelWriter(ChannelWriter&& other) noexcept
        : region_(std::move(other.region_))
        , buffer_(&region_.template as<ShmLayout<T>>()->buffer)
        , write_in_progress_(std::exchange(other.write_in_progress_, false))
        , write_slot_(std::exchange(other.write_slot_, 0)) {}

    // 移动赋值：转移资源，清理原有写入状态
    ChannelWriter& operator=(ChannelWriter&& other) noexcept {
        if (this != &other) {
            finalize_write_if_needed();
            region_ = std::move(other.region_);
            buffer_ = TripleBufferOps<TripleBufferLayout<T>, T>(
                &region_.template as<ShmLayout<T>>()->buffer);
            write_in_progress_ = std::exchange(other.write_in_progress_, false);
            write_slot_        = std::exchange(other.write_slot_, 0);
        }
        return *this;
    }

    // 禁用拷贝，通道不可复制
    ChannelWriter(const ChannelWriter&)            = delete;
    ChannelWriter& operator=(const ChannelWriter&) = delete;

    /**
     * @brief 创建写入端，新建对应类型的共享内存，初始化头部魔数版本
     * @return 写入通道对象
     */
    [[nodiscard]] static std::expected<ChannelWriter, ShmError> create() {
        // 计算当前消息类型完整SHM布局大小
        constexpr size_t shm_size = sizeof(ShmLayout<T>);
        auto region               = ShmRegion::create(ShmName<T>::value, shm_size);
        if (!region) {
            return std::unexpected(region.error());
        }

        // 初始化共享内存头部校验字段
        auto* shm           = region->template as<ShmLayout<T>>();
        shm->header.magic   = TALOS_SHM_MAGIC;
        shm->header.version = TALOS_SHM_VERSION;

        return ChannelWriter(std::move(*region));
    }

    /**
     * @brief 写入一帧数据，自动完成无锁三缓冲发布
     * @param data 待写入消息对象
     */
    void write(const T& data) noexcept {
        begin_write_if_needed();
        // 写入当前写槽位
        auto* shm                      = region_.template as<ShmLayout<T>>();
        shm->buffer.slots[write_slot_] = data;
        // 完成写入，更新序列号
        finalize_write_if_needed();
        // 发布缓冲区，切换可读帧
        buffer_.publish();
    }

private:
    // 开启写入事务：占用当前写槽，序列号奇数标记写入中
    void begin_write_if_needed() noexcept {
        if (write_in_progress_) {
            return;
        }
        auto* shm   = region_.template as<ShmLayout<T>>();
        write_slot_ = shm->buffer.write_idx;
        // 序列号+1变为奇数，表示缓冲区正在被修改，读者不可读取
        shm->buffer.slot_seq[write_slot_].fetch_add(1, std::memory_order_seq_cst);
        write_in_progress_ = true;
    }

    // 结束写入事务：序列号再次+1变回偶数，缓冲区完整可读
    void finalize_write_if_needed() noexcept {
        if (!write_in_progress_) {
            return;
        }
        auto* shm = region_.template as<ShmLayout<T>>();
        shm->buffer.slot_seq[write_slot_].fetch_add(1, std::memory_order_seq_cst);
        write_in_progress_ = false;
    }

    // 私有构造：仅create静态函数调用
    explicit ChannelWriter(ShmRegion&& region) noexcept
        : region_(std::move(region))
        , buffer_(&region_.template as<ShmLayout<T>>()->buffer) {}

    ShmRegion region_;                              // 绑定的共享内存区域
    TripleBufferOps<TripleBufferLayout<T>, T> buffer_; // 三缓冲操作封装
    bool write_in_progress_{false};                 // 是否存在未完成写入事务
    uint8_t write_slot_{0};                         // 当前占用的写缓冲槽位 0/1/2
};

// ============ 单向读通道：消费者Reader ============
template <typename T>
class ChannelReader {
public:
    // 析构默认，资源由ShmRegion RAII自动释放
    ~ChannelReader() = default;

    // 支持移动语义
    ChannelReader(ChannelReader&&)                 = default;
    ChannelReader& operator=(ChannelReader&&)      = default;
    // 禁用拷贝
    ChannelReader(const ChannelReader&)            = delete;
    ChannelReader& operator=(const ChannelReader&) = delete;

    /**
     * @brief 打开已存在的消息类型共享内存读通道，校验魔数版本
     * @return 读通道对象
     */
    [[nodiscard]] static std::expected<ChannelReader, ShmError> open() {
        constexpr size_t shm_size = sizeof(ShmLayout<T>);
        auto region               = ShmRegion::open(ShmName<T>::value, shm_size);
        if (!region) {
            return std::unexpected(region.error());
        }

        auto* shm = region->template as<ShmLayout<T>>();
        // 校验魔数，判断是否为本框架SHM
        if (shm->header.magic != TALOS_SHM_MAGIC) {
            return std::unexpected(ShmError::InvalidMagic);
        }
        // 校验版本号，跨版本不兼容直接报错
        if (shm->header.version != TALOS_SHM_VERSION) {
            return std::unexpected(ShmError::VersionMismatch);
        }

        return ChannelReader(std::move(*region));
    }

    /**
     * @brief 读取最新未消费的新帧，无新帧返回std::nullopt
     * @return 存在新消息返回T，无则空
     */
    [[nodiscard]] std::optional<T> read_new() noexcept {
        auto result = buffer_.borrow();
        if (!result) {
            return std::nullopt;
        }

        auto* shm   = region_.template as<ShmLayout<T>>();
        auto* slots = &shm->buffer.slots[0];
        // 计算返回指针对应的槽索引 0/1/2
        auto diff   = *result - slots;
        if (diff < 0 || diff >= 3) {
            return std::nullopt;
        }

        // 安全拷贝完整快照返回
        return copy_consistent_slot(static_cast<uint8_t>(diff));
    }

    /**
     * @brief 直接读取当前最新一帧，不区分是否已消费
     * @return 最新帧快照
     */
    [[nodiscard]] T read_latest() const noexcept {
        auto* shm = region_.template as<ShmLayout<T>>();
        return copy_consistent_slot(shm->buffer.read_idx);
    }

    /**
     * @brief 检测当前共享内存映射是否有效（对端重启后失效）
     * @return 有效true，重建后false
     */
    [[nodiscard]] bool is_current_mapping() const noexcept {
        return region_.is_current_mapping(ShmName<T>::value);
    }

private:
    /**
     * @brief 无锁安全拷贝槽位快照：双序列号校验，保证读取时写入不中断
     * 偶数序列号代表缓冲区完整无写入，奇数代表正在写入，循环等待
     * @param slot_idx 缓冲槽位0/1/2
     * @return 完整稳定消息副本
     */
    [[nodiscard]] T copy_consistent_slot(uint8_t slot_idx) const noexcept {
        auto* shm = region_.template as<ShmLayout<T>>();
        T snapshot{};
        while (true) {
            // 读取起始序列号
            const uint64_t begin = shm->buffer.slot_seq[slot_idx].load(std::memory_order_acquire);
            // 序列号奇数：写入进行中，等待
            if (begin & 1ULL) {
                continue;
            }

            // 拷贝缓冲区数据
            snapshot = shm->buffer.slots[slot_idx];

            // 内存屏障，防止读写重排
            std::atomic_thread_fence(std::memory_order_acquire);
            // 读取结束序列号
            const uint64_t end = shm->buffer.slot_seq[slot_idx].load(std::memory_order_acquire);
            // 起始、结束序列号完全相等且为偶数：拷贝期间无写入，数据稳定
            if (begin == end && !(end & 1ULL)) {
                return snapshot;
            }
        }
    }

    // 私有构造，仅open静态函数调用
    explicit ChannelReader(ShmRegion&& region) noexcept
        : region_(std::move(region))
        , buffer_(&region_.template as<ShmLayout<T>>()->buffer) {}

    ShmRegion region_;                              // 绑定共享内存区域
    TripleBufferOps<TripleBufferLayout<T>, T> buffer_; // 三缓冲读操作封装
};

// ============ 双向IPC通信端点 ChiralEndpoint ============
/**
 * @brief 通用双向IPC端点，实现类型安全的跨进程双向通信
 * 模板参数：
 * Outgoing：本端向外发送的消息类型（生产者）
 * Incoming：本端从对端接收的消息类型（消费者）
 *
 * 设计说明：
 * 1. 两套独立共享内存：Outgoing写SHM、Incoming读SHM，完全单向隔离无竞争
 * 2. 写通道创建时直接新建SHM；读通道延迟懒加载，读取时自动打开，对端重启自动重连
 * 3. 强类型模板约束，编译期阻止收发消息类型混淆
 *
 * 使用示例：
 * // Talos机器人端：发送TalosData，接收外部IncomingData
 * using TalosSide  = ChiralEndpoint<TalosData, IncomingData>;
 * // 外部上位机端：发送IncomingData，接收TalosData
 * using RemoteSide = ChiralEndpoint<IncomingData, TalosData>;
 */
template <typename Outgoing, typename Incoming>
class ChiralEndpoint {
public:
    // 资源全部RAII自动释放，默认析构
    ~ChiralEndpoint() = default;

    // 支持移动语义，端点可转移所有权
    ChiralEndpoint(ChiralEndpoint&&)                 = default;
    ChiralEndpoint& operator=(ChiralEndpoint&&)      = default;
    // 禁用拷贝，通信端点不可复制
    ChiralEndpoint(const ChiralEndpoint&)            = delete;
    ChiralEndpoint& operator=(const ChiralEndpoint&) = delete;

    /**
     * @brief 创建双向通信端点，初始化本端发送共享内存
     * 接收通道延迟打开，第一次read时自动加载
     * @return 智能指针封装端点，失败返回错误码
     */
    [[nodiscard]] static std::expected<std::unique_ptr<ChiralEndpoint>, ShmError>
        create() noexcept {
        auto writer = ChannelWriter<Outgoing>::create();
        if (!writer) {
            return std::unexpected(writer.error());
        }

        return std::make_unique<ChiralEndpoint>(std::move(*writer));
    }

    /**
     * @brief 发送消息，仅支持Outgoing类型，编译期类型校验
     * @param data 待发送消息
     */
    void write(const Outgoing& data) noexcept { writer_.write(data); }

    /**
     * @brief 读取对端新推送消息，自动处理共享内存失效重连
     * 1. 检测读通道映射失效则销毁重建
     * 2. 无新消息返回std::nullopt
     * @return 新消息或空
     */
    [[nodiscard]] std::optional<Incoming> read_new() noexcept {
        auto* reader = lazy_reader();
        if (reader == nullptr) {
            return std::nullopt;
        }

        auto data = reader->read_new();
        // 读取成功或映射有效直接返回
        if (data || reader->is_current_mapping()) {
            return data;
        }

        // 映射失效，销毁旧读通道重新打开
        reader_.reset();
        reader = lazy_reader();
        if (reader == nullptr) {
            return std::nullopt;
        }
        return reader->read_new();
    }

    /**
     * @brief 读取当前最新帧，不区分是否已消费
     * @return 最新消息，无通道返回空
     */
    [[nodiscard]] std::optional<Incoming> read_latest() const noexcept {
        auto* reader = lazy_reader();
        if (reader == nullptr) {
            return std::nullopt;
        }
        return reader->read_latest();
    }

    // 私有构造：接收已创建的写通道对象
    explicit ChiralEndpoint(ChannelWriter<Outgoing>&& writer) noexcept
        : writer_(std::move(writer))
        , reader_(std::nullopt) {}

private:
    /**
     * @brief 懒加载读通道，检测映射失效自动重建
     * 可变修饰mutable：const读取接口内允许修改reader_
     * @return 有效读通道指针，打开失败返回nullptr
     */
    [[nodiscard]] ChannelReader<Incoming>* lazy_reader() const noexcept {
        // 已有读通道但映射失效，销毁重建
        if (reader_ && !reader_->is_current_mapping()) {
            reader_.reset();
        }
        // 通道有效直接返回
        if (reader_) {
            return &*reader_;
        }

        // 首次打开读共享内存
        auto reader = ChannelReader<Incoming>::open();
        if (!reader) {
            return nullptr;
        }

        reader_.emplace(std::move(*reader));
        return &*reader_;
    }

    ChannelWriter<Outgoing> writer_;                          // 本端发送写通道，生命周期永久有效
    mutable std::optional<ChannelReader<Incoming>> reader_;   // 延迟加载读通道，可变允许const接口修改
};

} // namespace talos::chiral::ipc