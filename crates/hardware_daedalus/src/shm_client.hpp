#pragma once
// 头文件保护，防止重复包含导致重定义编译错误

// 时间工具：steady_clock计时、system_clock纳秒时间戳
#include <chrono>
// C++23 标准错误返回类型，用于connect/create返回成功/错误
#include <expected>
// OpenCV核心矩阵，用于图像帧存储、零拷贝映射共享内存图像
#include <opencv2/core.hpp>
// 可选值，无数据时返回std::nullopt，区分“无新帧”和“有效帧”
#include <optional>
// 线程休眠，用于等待生产者就绪轮询
#include <thread>

// 共享内存全局布局结构体定义（魔数、版本、三缓冲通道、相机内参等）
#include "shm_layout.hpp"
// 单块共享内存区域封装类，负责shmget/shmat/shmdt生命周期管理
#include "shm_region.hpp"
// 无锁三缓冲数据结构，实现生产者-消费者无锁并发
#include "shm_triple_buffer.hpp"

namespace ipc {

/**
 * @brief 共享内存 IPC 客户端
 * 通信双方：Rust模拟器(生产者) ↔ C++视觉/云台程序(消费者)
 * 两种工作模式：
 * 1. 消费者模式 connect()：连接Rust已创建的共享内存，只读订阅图像、位姿、真值；可下发云台控制指令
 * 2. 生产者模式 create()：新建共享内存，仅单元测试使用，模拟Rust发布图像/位姿
 *
 * 核心能力：
 * - 图像流订阅：零拷贝cv::Mat直接映射共享内存图像池
 * - 多通道位姿订阅：云台、底盘、枪口、相机位姿
 * - 底盘观测、仿真真值、仿真运行状态读取
 * - 下发云台目标角度、距离、开火建议指令
 * - 心跳保活、版本/魔数校验、生产者就绪等待
 *
 * 使用示例:
 * ```cpp
 * auto client = ShmClient::connect();
 * if (!client) {
 *     std::cerr << "连接失败: " << to_string(client.error()) << std::endl;
 *     return;
 * }
 *
 * // 非阻塞读取最新图像帧
 * if (auto frame = client->recv_image()) {
 *     cv::Mat img = frame->image;
 *     // 业务处理图像...
 * }
 *
 * // 下发云台控制指令：偏航15°，俯仰-8°，距离3.5m，允许开火
 * client->send_gimbal_cmd(15.0f, -8.0f, 3.5f, true);
 * ```
 */
class ShmClient {
public:
    /**
     * @brief 从共享内存读取到的图像帧封装结构体
     */
    struct ImageFrame {
        // 零拷贝OpenCV矩阵，内存直接指向共享内存图像池，不复制像素
        // 风险：下次调用recv_image()后，内存可能被生产者覆盖，矩阵失效
        cv::Mat image;
        // 图像帧全局递增序列号，用于匹配对应位姿数据
        uint64_t seq;
        // 帧生成纳秒时间戳，与Rust端SystemTime对齐
        uint64_t timestamp_ns;
    };

    /**
     * @brief 位姿数据结构体（位置+四元数）
     */
    struct Pose {
        // 三维坐标，单位米
        double x, y, z;
        // 单位四元数 qw + qx qy qz
        double qw, qx, qy, qz;
        // 关联对应的图像帧序列号，实现图像-位姿时间对齐
        uint64_t frame_seq;
        // 位姿生成纳秒时间戳
        uint64_t timestamp_ns;
    };

    // 默认析构：成员ShmRegion自动析构，执行shmdt脱离共享内存，释放资源
    ~ShmClient() = default;

    // 仅允许移动语义，禁止拷贝
    // 共享内存句柄是独占资源，拷贝会导致重复shmdt、内存野指针崩溃
    ShmClient(ShmClient&&)                 = default;
    ShmClient& operator=(ShmClient&&)      = default;
    ShmClient(const ShmClient&)            = delete;
    ShmClient& operator=(const ShmClient&) = delete;

    /**
     * @brief 消费者模式：连接Rust预先创建好的共享内存
     * @return std::expected<ShmClient, ShmError> 成功返回客户端实例，失败返回错误码
     * 校验逻辑：打开元数据区+图像池 → 校验魔数、版本号，防止旧版本内存不兼容
     */
    [[nodiscard]] static std::expected<ShmClient, ShmError> connect() {
        // 1. 打开全局元数据共享内存：存储三缓冲、相机内参、心跳、真值等控制信息
        auto meta_result = ShmRegion::open(SHM_NAME_META, sizeof(ShmMetaRegion));
        if (!meta_result) {
            // 打开失败（共享内存不存在/权限不足）直接返回错误
            return std::unexpected(meta_result.error());
        }

        // 2. 打开图像像素池共享内存：存放所有图像原始像素数据
        auto pool_result = ShmRegion::open(SHM_NAME_IMAGE_POOL, IMAGE_POOL_SIZE);
        if (!pool_result) {
            return std::unexpected(pool_result.error());
        }

        // 3. 内存映射完成，获取元数据头部指针
        const auto* meta = meta_result->as<ShmMetaRegion>();

        // 魔数校验：区分合法共享内存与残留垃圾shm文件
        if (meta->header.magic != SHM_MAGIC) {
            return std::unexpected(ShmError::InvalidSize); // 注释：实际应新增InvalidMagic错误枚举
        }
        // 版本校验：防止C++与Rust两端shm_layout结构体定义不一致
        if (meta->header.version != SHM_VERSION) {
            return std::unexpected(ShmError::InvalidSize); // 注释：实际应新增VersionMismatch错误枚举
        }

        // 构造客户端实例，移交两块共享内存所有权
        return ShmClient(std::move(*meta_result), std::move(*pool_result));
    }

    /**
     * @brief 生产者模式：全新创建共享内存，仅单元测试使用
     * 业务场景仅Rust作为生产者，C++正常业务不调用此接口
     * @return 初始化完成的ShmClient，包含初始化好的三缓冲通道
     */
    [[nodiscard]] static std::expected<ShmClient, ShmError> create() {
        // 创建元数据共享内存段
        auto meta_result = ShmRegion::create(SHM_NAME_META, sizeof(ShmMetaRegion));
        if (!meta_result) {
            return std::unexpected(meta_result.error());
        }

        // 创建图像像素池共享内存段
        auto pool_result = ShmRegion::create(SHM_NAME_IMAGE_POOL, IMAGE_POOL_SIZE);
        if (!pool_result) {
            return std::unexpected(pool_result.error());
        }

        // 获取元数据裸指针，初始化头部信息
        auto* meta = meta_result->as<ShmMetaRegion>();

        // 填充共享内存头部标识
        meta->header.magic   = SHM_MAGIC;
        meta->header.version = SHM_VERSION;
        // 记录共享内存创建时间戳
        meta->header.created_ns =
            static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
        // 写入全局图像分辨率配置
        meta->header.image_width  = IMAGE_WIDTH;
        meta->header.image_height = IMAGE_HEIGHT;

        // 关键：手动初始化全部TripleBuffer三缓冲
        // 底层ShmRegion::create会调用memset全内存清零，会覆盖三缓冲合法初始状态
        // 标准初始状态规范：state=1，write_idx=0，read_idx=2
        init_triple_buffer(meta->image);
        // 批量初始化5路位姿三缓冲
        for (auto& pose : meta->poses) {
            init_triple_buffer(pose);
        }
        // 初始化云台下发指令三缓冲
        init_triple_buffer(meta->gimbal_cmd);

        return ShmClient(std::move(*meta_result), std::move(*pool_result));
    }

    // ====================== 图像订阅接口（消费者） ======================
    /**
     * @brief 非阻塞读取最新图像帧
     * @return std::nullopt 无新图像；返回ImageFrame 存在未消费新帧
     * 零拷贝机制：cv::Mat直接绑定共享内存像素地址，无内存拷贝开销
     * 警告：下次recv_image调用后，图像内存可能被生产者覆盖，不要长期持有ImageFrame
     */
    [[nodiscard]] std::optional<ImageFrame> recv_image() const {
        // 图像三缓冲操作封装，绑定元数据内图像通道
        ImageOps ops(&meta_->image);
        // borrow：无锁借用最新就绪帧，无新数据返回空
        const auto slot = ops.borrow();
        if (!slot) {
            return std::nullopt;
        }

        // 取出帧元数据（序号、分辨率、buffer索引、格式）
        const auto& img_meta = **slot;

        // 根据buffer_id计算像素数据在图像池内的内存偏移
        uint8_t* img_data = image_pool_ + img_meta.buffer_id * IMAGE_SIZE;

        // 根据图像格式映射OpenCV矩阵类型
        int cv_type = CV_8UC3; // 默认RGB8
        if (img_meta.format == 1)
            cv_type = CV_8UC3; // BGR8格式
        else if (img_meta.format == 2)
            cv_type = CV_8UC1; // 单通道灰度图

        // 构造零拷贝cv::Mat，仅记录尺寸、类型、内存地址，不分配像素缓存
        const cv::Mat image(
            static_cast<int>(img_meta.height), static_cast<int>(img_meta.width), cv_type, img_data);

        // 封装帧信息返回
        return ImageFrame{
            .image        = image,
            .seq          = img_meta.seq,
            .timestamp_ns = img_meta.timestamp_ns,
        };
    }

    /**
     * @brief 快速判断是否存在未读取的新图像，不取出帧
     */
    [[nodiscard]] bool has_new_image() const {
        const ImageOps ops(&meta_->image);
        return ops.has_new_data();
    }

    // ====================== 位姿订阅接口（消费者） ======================
    /**
     * @brief 读取指定类型位姿数据
     * @param index 位姿枚举索引：POSE_GIMBAL/POSE_ODOM/POSE_MUZZLE/POSE_CAMERA/POSE_CHASSIS_OBSERVATION
     * @return 存在新位姿返回Pose，无数据返回nullopt
     */
    [[nodiscard]] std::optional<Pose> recv_pose(const PoseIndex index) const {
        // 索引越界直接返回空
        if (index > 4)
            return std::nullopt;

        PoseOps ops(&meta_->poses[index]);
        const auto slot = ops.borrow();
        if (!slot) {
            return std::nullopt;
        }

        // 解析位姿结构体到对外Pose类型
        const auto& pose = **slot;
        return Pose{
            .x            = pose.position[0],
            .y            = pose.position[1],
            .z            = pose.position[2],
            .qw           = pose.quaternion[0],
            .qx           = pose.quaternion[1],
            .qy           = pose.quaternion[2],
            .qz           = pose.quaternion[3],
            .frame_seq    = pose.frame_seq,
            .timestamp_ns = pose.timestamp_ns,
        };
    }

    /**
     * @brief 读取底盘观测独立通道数据
     * 区别于poses数组：单独全局结构体，不使用三缓冲，生产者直接覆盖写入
     * @return timestamp_ns=0代表无有效数据，返回nullopt
     */
    [[nodiscard]] std::optional<ChassisObservation> recv_chassis_observation() const {
        const auto observation = meta_->chassis_observation;
        if (observation.timestamp_ns == 0) {
            return std::nullopt;
        }
        return observation;
    }

    // ====================== 真值/仿真状态订阅 ======================
    /**
     * @brief 读取仿真全局真值批量数据（目标真值、障碍物真值等）
     */
    [[nodiscard]] std::optional<GroundTruthBatch> recv_ground_truth() const {
        const auto& gt = meta_->ground_truth;
        if (gt.timestamp_ns == 0) {
            return std::nullopt;
        }
        return gt;
    }

    /**
     * @brief 读取仿真运行状态（是否跟随目标等标识）
     * 数据存储在元数据尾部预留空间，旧版本Rust生产者无此字段，timestamp=0表示无效
     */
    [[nodiscard]] std::optional<RuntimeState> recv_runtime_state() const {
        const auto state = meta_->runtime_state;
        if (state.timestamp_ns == 0) {
            return std::nullopt;
        }
        return state;
    }

    // ====================== 云台指令下发（消费者写通道） ======================
    /**
     * @brief 向Rust模拟器下发云台控制目标
     * @param yaw_deg 目标偏航角 单位度
     * @param pitch_deg 目标俯仰角 单位度
     * @param distance_m 目标预测距离，-1代表无效
     * @param fire_advice 是否允许开火建议
     */
    void send_gimbal_cmd(
        const float yaw_deg, const float pitch_deg, const float distance_m,
        const bool fire_advice) const {
        // 获取云台指令三缓冲写入操作器
        GimbalOps ops(&meta_->gimbal_cmd);
        // borrow_mut 获取可修改的写入槽位
        auto& cmd = ops.borrow_mut();

        // 填充当前纳秒时间戳
        cmd.timestamp_ns =
            static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
        cmd.yaw_deg     = yaw_deg;
        cmd.pitch_deg   = pitch_deg;
        cmd.distance_m  = distance_m;
        cmd.fire_advice = fire_advice ? 1 : 0;

        // publish 发布新指令，切换缓冲槽，消费者可见
        ops.publish();
    }

    // ====================== 生产者专属发布接口（仅测试create模式） ======================
    /**
     * @brief 生产者：写入位姿数据到指定通道
     */
    void publish_pose(const PoseIndex index, const Pose& pose) const {
        if (index > 4) {
            return;
        }

        PoseOps ops(&meta_->poses[index]);
        auto& meta         = ops.borrow_mut();
        meta.frame_seq     = pose.frame_seq;
        meta.position[0]   = static_cast<float>(pose.x);
        meta.position[1]   = static_cast<float>(pose.y);
        meta.position[2]   = static_cast<float>(pose.z);
        meta.quaternion[0] = static_cast<float>(pose.qw);
        meta.quaternion[1] = static_cast<float>(pose.qx);
        meta.quaternion[2] = static_cast<float>(pose.qy);
        meta.quaternion[3] = static_cast<float>(pose.qz);
        meta.timestamp_ns  = pose.timestamp_ns;
        ops.publish();
    }

    /**
     * @brief 生产者：全局写入相机内参（单变量覆盖，无缓冲）
     */
    void publish_camera_info(const CameraInfo& info) const { meta_->camera_info = info; }

    /**
     * @brief 生产者：更新仿真运行状态标识
     */
    void publish_runtime_state(const bool following, const uint64_t timestamp_ns) const {
        meta_->runtime_state.timestamp_ns = timestamp_ns;
        meta_->runtime_state.following    = following ? 1U : 0U;
    }

    // ====================== 诊断/保活工具接口 ======================
    /**
     * @brief 获取共享内存全局头部（魔数、版本、分辨率、心跳）
     */
    [[nodiscard]] const ShmHeader& header() const { return meta_->header; }

    /**
     * @brief 生产者：更新心跳时间戳，标记进程存活
     */
    void update_heartbeat() const { meta_->header.heartbeat_ns = now_ns(); }

    /**
     * @brief 消费者：判断生产者是否存活
     * @param timeout_ns 心跳超时阈值，默认1秒无更新判定离线
     */
    [[nodiscard]] bool is_producer_alive(const uint64_t timeout_ns = 1'000'000'000) const {
        // 当前时间 - 上次心跳 < 超时阈值 = 存活
        return now_ns() - meta_->header.heartbeat_ns < timeout_ns;
    }

    /**
     * @brief 阻塞等待生产者上线，超时自动退出
     * @param timeout 最大等待时长，默认5秒
     * @return true 生产者正常上线；false 等待超时
     * 作用：避免连接到残留旧共享内存，等待Rust启动并刷新心跳
     */
    [[nodiscard]] bool
        wait_for_producer(const std::chrono::milliseconds timeout = std::chrono::seconds(5)) const {
        // 计算截止时间点
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        // 循环轮询心跳，每50ms检测一次
        while (std::chrono::steady_clock::now() < deadline) {
            if (is_producer_alive()) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        // 超时未检测到活跃生产者
        return false;
    }

    /**
     * @brief 生产者测试接口：将OpenCV图像写入共享内存图像池并发布
     * @param image 待发布图像矩阵
     * @param seq 帧序列号
     * @param timestamp_ns 帧时间戳
     */
    void
        publish_image(const cv::Mat& image, const uint64_t seq, const uint64_t timestamp_ns) const {
        ImageOps ops(&meta_->image);
        auto& meta = ops.borrow_mut();

        // 轮换图像缓冲ID 0/1/2 三缓冲循环
        uint8_t buffer_id = meta.buffer_id;
        buffer_id         = (buffer_id + 1) % 3;

        // 计算目标像素内存地址
        uint8_t* dst = image_pool_ + buffer_id * IMAGE_SIZE;
        // 区分连续/非连续Mat，安全拷贝像素到共享内存
        if (image.isContinuous()) {
            // 内存连续，一次性memcpy
            std::memcpy(dst, image.data, IMAGE_SIZE);
        } else {
            // 行不连续，逐行拷贝
            for (int row = 0; row < image.rows; ++row) {
                std::memcpy(
                    dst + row * image.cols * image.channels(), image.ptr(row),
                    image.cols * image.channels());
            }
        }

        // 更新图像元数据
        meta.seq          = seq;
        meta.timestamp_ns = timestamp_ns;
        meta.width        = static_cast<uint32_t>(image.cols);
        meta.height       = static_cast<uint32_t>(image.rows);
        meta.buffer_id    = buffer_id;
        // 图像格式映射：灰度=2，BGR=1
        meta.format       = image.type() == CV_8UC1 ? 2 : 1;

        // 发布新帧，消费者可读取
        ops.publish();
    }

private:
    /**
     * @brief 静态工具：重置三缓冲到合法初始状态
     * 问题根源：创建共享内存时memset清零，会覆盖TripleBuffer原子变量与索引
     * 标准初始化状态约定：
     * state=1：无新数据标记
     * write_idx=0：生产者写入槽位
     * read_idx=2：消费者上次读取槽位
     */
    template <typename TripleBuffer>
    static void init_triple_buffer(TripleBuffer& buf) {
        buf.state.store(1, std::memory_order_relaxed);
        buf.write_idx = 0;
        buf.read_idx  = 2;
    }

    /**
     * @brief 获取系统时钟纳秒时间戳，与Rust SystemTime对齐
     * steady_clock仅用于程序内计时，system_clock用于跨进程时间同步
     */
    [[nodiscard]] static uint64_t now_ns() {
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::system_clock::now().time_since_epoch())
                                         .count());
    }

    /**
     * @brief 私有构造函数，仅connect/create静态方法调用
     * 移动传入两块共享内存区域，并缓存内存映射指针
     */
    ShmClient(ShmRegion meta_region, ShmRegion pool_region)
        : meta_region_(std::move(meta_region))
        , pool_region_(std::move(pool_region))
        // 缓存元数据结构体指针，避免重复调用as<>
        , meta_(meta_region_.as<ShmMetaRegion>())
        // 缓存图像池像素内存起始地址
        , image_pool_(static_cast<uint8_t*>(pool_region_.data())) {}

    // 元数据共享内存管理对象（三缓冲、相机内参、心跳、真值）
    ShmRegion meta_region_;
    // 图像像素池共享内存管理对象（所有图像原始像素）
    ShmRegion pool_region_;
    // meta_region_映射后的结构体裸指针，缓存优化
    ShmMetaRegion* meta_;
    // pool_region_映射后的像素内存起始指针，缓存优化
    uint8_t* image_pool_;
};

} // namespace ipc