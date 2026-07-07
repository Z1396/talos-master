#pragma once
// 头文件保护宏，防止同一头文件被多次重复包含，替代传统 #ifndef / #define / #endif

// 标准固定宽度整数类型，提供 u8/i32/u64 等跨平台定长整型
#include <cstdint>
// C++23 标准错误返回类型 std::expected<T, E>
// 替代错误码+输出参数、异常两种方案，无异常开销，返回值天然携带成功/失败信息
#include <expected>
// Linux 文件控制相关系统调用宏与函数：open、O_RDWR/O_CREAT/O_TRUNC 等标志
#include <fcntl.h>
// C++17 文件系统库，路径拼接、文件存在性判断、删除文件等跨平台文件操作
#include <filesystem>
// 只读字符串视图，不持有内存，轻量化字符串参数传递，无拷贝开销
#include <string_view>
// Linux mmap 内存映射、munmap 解除映射、msync 刷写映射缓存系统调用
#include <sys/mman.h>
// 文件状态结构体 stat，fstat 获取文件大小、权限等元信息
#include <sys/stat.h>
// Linux 文件IO基础系统调用：close、ftruncate、read/write 等
#include <unistd.h>

// 高性能格式化打印库，替代std::cout/printf，支持自定义格式化器
#include <fmt/core.h>
// 编译期枚举反射库，无需手写字符串映射，自动将枚举值转为对应名字字符串
#include <magic_enum.hpp>

// IPC 进程间通信命名空间，隔离共享内存相关代码，避免全局命名污染
namespace ipc {

/**
 * @brief 共享内存操作全部错误枚举
 * 所有 ShmRegion 静态工厂函数失败时，std::expected 会返回该错误码
 */
enum class ShmError {
    OpenFailed,        // open() 系统调用打开文件失败（权限/不存在/路径非法）
    TruncateFailed,    // ftruncate() 设置文件大小失败（磁盘满/权限不足）
    MapFailed,         // mmap() 创建内存映射失败（地址空间不足/文件非法）
    AlreadyExists,     // 预留：主动创建时文件已存在（当前create逻辑直接覆盖未使用）
    NotFound,          // open() 打开模式：目标共享内存文件不存在
    PermissionDenied,  // open() 打开模式：文件存在但无读写权限(EACCES)
    InvalidSize,       // open() 校验失败：磁盘文件实际尺寸小于用户传入映射尺寸
};

/**
 * @brief 拼接共享内存文件的完整本地路径
 * @param name 用户传入共享内存名称（允许带开头/，函数内部自动剔除）
 * @return std::filesystem::path 拼接后的绝对路径 /tmp/[清理后的名称]
 * @note 选用 /tmp 临时目录存储mmap文件，兼容Rust memmap2库的文件式共享内存实现
 *       不使用POSIX shm_open(shmget)，跨语言互通更简单
 */
inline std::filesystem::path shm_path(std::string_view name) {
    // 清理路径前缀，统一格式：用户传入 "/shm01" → "shm01"
    if (!name.empty() && name[0] == '/') {
        name = name.substr(1);
    }
    // 路径拼接：/tmp + 清理后的名称，返回平台无关路径对象
    return std::filesystem::path("/tmp") / name;
}

/**
 * @brief RAII 自动管理生命周期的内存映射共享内存区域
 * 核心特性：
 * 1. Move-only 仅允许移动，禁止拷贝（映射句柄、文件描述符不可多份持有）
 * 2. 析构自动释放mmap内存、关闭fd、所有者自动删除临时共享文件
 * 3. 兼容Rust memmap2 文件映射实现，C++ ↔ Rust 进程可互通共享内存
 * 4. 提供 create(新建) / open(打开已有) / create_or_open(智能创建或打开) 三种模式
 */
class ShmRegion {
public:
    /**
     * @brief RAII 析构函数，自动释放所有资源，杜绝内存/文件句柄泄漏
     * 执行流程：
     * 1. 若存在有效mmap映射，调用munmap解除虚拟内存映射，归还地址空间
     * 2. 若持有合法文件fd，close关闭文件描述符，释放内核句柄
     * 3. 若当前实例是共享内存所有者(创建者)，删除/tmp下对应的共享内存文件
     */
    ~ShmRegion() {
        // 判断：映射指针非空且不是失败标记MAP_FAILED，执行解除映射
        if (data_ != nullptr && data_ != MAP_FAILED) {
            munmap(data_, size_);
        }
        // 合法文件描述符(>=0)则关闭fd
        if (fd_ >= 0) {
            close(fd_);
        }
        // 仅创建者owner_有权删除文件，防止消费者打开后析构误删共享文件
        if (owner_ && !path_.empty()) {
            std::filesystem::remove(path_);
        }
    }

    /**
     * @brief 移动构造函数，转移共享内存全部资源所有权
     * noexcept 保证不会抛出异常，满足std::move容器存储要求
     * @param other 待转移资源的旧ShmRegion实例
     * 逻辑：复制资源字段，清空旧实例，防止旧实例析构重复释放资源
     */
    ShmRegion(ShmRegion&& other) noexcept
        : data_(other.data_)        // 接管mmap内存指针
        , size_(other.size_)        // 接管映射总字节大小
        , fd_(other.fd_)            // 接管文件描述符
        , owner_(other.owner_)      // 接管所有者标记
        , path_(std::move(other.path_)) // 移动文件路径字符串，无拷贝开销
    {
        // 清空原对象所有资源标记，原对象析构时不会执行释放操作
        other.data_  = nullptr;
        other.fd_    = -1;
        other.owner_ = false;
    }

    /**
     * @brief 移动赋值运算符，转移资源，先释放当前自身资源再接管新资源
     * @param other 待转移资源的右值实例
     * @return 当前对象引用，支持链式赋值
     */
    ShmRegion& operator=(ShmRegion&& other) noexcept {
        // 自赋值保护：防止自己移动给自己导致资源全部清空
        if (this != &other) {
            // 第一步：释放自身当前持有的全部资源（同析构逻辑）
            if (data_ != nullptr && data_ != MAP_FAILED) {
                munmap(data_, size_);
            }
            if (fd_ >= 0) {
                close(fd_);
            }
            if (owner_ && !path_.empty()) {
                std::filesystem::remove(path_);
            }

            // 第二步：接管传入实例的全部资源
            data_        = other.data_;
            size_        = other.size_;
            fd_          = other.fd_;
            owner_       = other.owner_;
            path_        = std::move(other.path_);

            // 第三步：清空旧实例资源，避免双重释放
            other.data_  = nullptr;
            other.fd_    = -1;
            other.owner_ = false;
        }
        return *this;
    }

    // 删除拷贝构造：mmap映射、文件fd无法共享，禁止复制实例
    ShmRegion(const ShmRegion&)            = delete;
    // 删除拷贝赋值：同上，不允许复制资源句柄
    ShmRegion& operator=(const ShmRegion&) = delete;

    /**
     * @brief 【生产者专用】创建全新共享内存映射文件
     * @param name 共享内存标识名称
     * @param size 需要分配的共享内存总字节大小
     * @return std::expected<ShmRegion, ShmError>
     *         成功：返回持有全部资源的ShmRegion实例，标记为owner所有者
     *         失败：返回对应ShmError错误码
     * 流程：创建空文件 → 扩展到指定size → mmap读写映射 → 内存清零
     */
    [[nodiscard]] static std::expected<ShmRegion, ShmError>
        create(const std::string_view name, const size_t size) {
        // 生成/tmp下完整文件路径
        const auto path = shm_path(name);

        // open标志解析：
        // O_RDWR：读写打开
        // O_CREAT：文件不存在则新建
        // O_TRUNC：文件已存在直接截断清空覆盖
        // 0644 文件权限：所有者读写，组/其他只读
        const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
        if (fd < 0) {
            // open系统调用返回-1，打开失败
            return std::unexpected(ShmError::OpenFailed);
        }

        // ftruncate 将文件磁盘占用扩展到size字节，mmap必须依赖文件实际空间
        if (ftruncate(fd, static_cast<off_t>(size)) < 0) {
            // 扩容失败，先关闭fd、删除残留空文件，再返回错误
            close(fd);
            std::filesystem::remove(path);
            return std::unexpected(ShmError::TruncateFailed);
        }

        // mmap 创建内存映射：
        // nullptr：由内核自动分配虚拟地址
        // size：映射字节长度
        // PROT_READ | PROT_WRITE：内存可读可写
        // MAP_SHARED：修改同步写入磁盘文件，多进程共享可见
        // fd：映射绑定的文件描述符
        // 0：文件偏移量，从头映射
        void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (ptr == MAP_FAILED) {
            // 映射失败，清理资源后返回错误
            close(fd);
            std::filesystem::remove(path);
            return std::unexpected(ShmError::MapFailed);
        }

        // 新建共享内存默认全部清零，避免残留脏数据
        std::memset(ptr, 0, size);

        // 调用私有构造函数构造实例，owner=true 标记为创建所有者
        return ShmRegion(ptr, size, fd, true, path);
    }

    /**
     * @brief 【消费者专用】打开已存在的共享内存文件
     * @param name 共享内存标识名称
     * @param size 预期映射字节大小，会校验磁盘文件实际尺寸是否足够
     * @return std::expected<ShmRegion, ShmError>
     *         成功：返回非所有者实例，析构不会删除共享文件
     *         失败：NotFound/InvalidSize/PermissionDenied/OpenFailed/MapFailed
     */
    [[nodiscard]] static std::expected<ShmRegion, ShmError>
        open(const std::string_view name, const size_t size) {
        const auto path = shm_path(name);

        // 第一步：判断文件是否存在，不存在直接返回NotFound
        if (!std::filesystem::exists(path)) {
            return std::unexpected(ShmError::NotFound);
        }

        // 仅读写打开，不创建、不截断
        const int fd = ::open(path.c_str(), O_RDWR, 0);
        if (fd < 0) {
            // 区分权限不足错误码EACCES，单独返回PermissionDenied
            if (errno == EACCES) {
                return std::unexpected(ShmError::PermissionDenied);
            }
            // 其他打开错误统一归类OpenFailed
            return std::unexpected(ShmError::OpenFailed);
        }

        // fstat 获取文件元信息，校验磁盘文件实际大小
        struct stat st{};
        // 文件读取失败 或 文件真实尺寸小于用户传入size → 尺寸非法
        if (fstat(fd, &st) < 0 || static_cast<size_t>(st.st_size) < size) {
            close(fd);
            return std::unexpected(ShmError::InvalidSize);
        }

        // 执行内存映射，参数同create函数
        void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (ptr == MAP_FAILED) {
            close(fd);
            return std::unexpected(ShmError::MapFailed);
        }

        // owner=false 消费者，析构不会删除共享文件
        return ShmRegion(ptr, size, fd, false, path);
    }

    /**
     * @brief 智能接口：优先打开已有共享内存，不存在则新建
     * @param name 共享内存名称
     * @param size 映射总字节大小
     * @return 打开成功返回现有实例；打开失败(文件不存在)则新建一份
     */
    [[nodiscard]] static std::expected<ShmRegion, ShmError>
        create_or_open(const std::string_view name, const size_t size) {
        // 先尝试打开
        if (auto result = open(name, size)) {
            // open成功，直接返回打开实例
            return result;
        }
        // open失败（大概率文件不存在），调用create新建
        return create(name, size);
    }

    // -------------------------- 只读访问接口 --------------------------
    /**
     * @brief 获取共享内存裸指针（读写）
     * @return void* 映射内存起始地址
     * noexcept 不会抛出异常
     */
    [[nodiscard]] void* data() noexcept { return data_; }

    /**
     * @brief 获取共享内存const裸指针（只读访问）
     */
    [[nodiscard]] const void* data() const noexcept { return data_; }

    /**
     * @brief 获取当前共享内存总字节长度
     */
    [[nodiscard]] size_t size() const noexcept { return size_; }

    /**
     * @brief 判断当前实例是否为共享内存创建所有者
     * true：析构会删除/tmp共享文件；false：仅消费者，不删除文件
     */
    [[nodiscard]] bool is_owner() const noexcept { return owner_; }

    /**
     * @brief 获取共享内存磁盘文件完整路径
     */
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }

    /**
     * @brief 类型转换模板：将共享内存指针强转为自定义结构体/类型T（读写）
     * @tparam T 目标数据类型（自定义IPC结构体、数组等）
     * @return T* 类型安全指针
     * 示例：auto buf = region.as<SharedMsg>(); buf->msg = 123;
     */
    template <typename T>
    [[nodiscard]] T* as() noexcept {
        return static_cast<T*>(data_);
    }

    /**
     * @brief 只读版本类型转换模板，禁止通过指针修改共享内存
     * 示例：const auto buf = region.as<const SharedMsg>();
     */
    template <typename T>
    [[nodiscard]] const T* as() const noexcept {
        return static_cast<const T*>(data_);
    }

    /**
     * @brief msync强制刷新内存映射缓存同步到磁盘文件
     * MS_SYNC：同步阻塞调用，直到全部修改落盘才返回
     * 多进程场景下，可主动调用保证数据对其他进程立即可见
     */
    void flush() const {
        if (data_ != nullptr && data_ != MAP_FAILED) {
            msync(data_, size_, MS_SYNC);
        }
    }

private:
    /**
     * @brief 私有构造函数，禁止外部直接构造实例，统一通过静态工厂函数创建
     * @param data mmap返回的内存映射指针
     * @param size 映射字节总数
     * @param fd 共享内存文件描述符
     * @param owner 是否为创建所有者标记
     * @param path 共享内存磁盘文件路径，移动语义接收避免拷贝
     */
    ShmRegion(
        void* data, const size_t size, const int fd, const bool owner, std::filesystem::path path)
        : data_(data)
        , size_(size)
        , fd_(fd)
        , owner_(owner)
        , path_(std::move(path)) {}

    void* data_{nullptr};                  // mmap映射内存起始地址
    size_t size_{0};                       // 映射总字节大小
    int fd_{-1};                           // 共享内存文件描述符，-1代表无效
    bool owner_{false};                    // 所有者标记：true=创建者，析构删文件
    std::filesystem::path path_;           // 共享内存磁盘文件绝对路径
};

} // namespace ipc

// ============================================================================
// fmt 库自定义格式化器：支持直接打印 ShmError 枚举
// 无需手动switch转字符串，fmt::print("错误：{}", err) 自动输出枚举名称
// ============================================================================
namespace fmt {

template <>
// 特化fmt格式化器，继承std::string_view基础格式化逻辑
struct formatter<ipc::ShmError> : formatter<std::string_view> {
    /**
     * @brief 格式化枚举值为可读字符串
     * @param e 共享内存错误枚举
     * @param ctx fmt输出上下文
     * @return 格式化迭代器
     * magic_enum::enum_name(e) 编译期生成枚举对应的字符串名，零运行时开销
     */
    auto format(const ipc::ShmError e, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(e), ctx);
    }
};

} // namespace fmt