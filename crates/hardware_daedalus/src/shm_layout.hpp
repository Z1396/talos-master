// 头文件保护，防止重复包含
#pragma once

// 无锁三缓冲用到原子变量 std::atomic
#include <atomic>
// 标准固定宽度整数、size_t
#include <cstddef>
#include <cstdint>

namespace ipc {

// ====================== 全局常量定义 ======================
// CPU 缓存行标准大小 64 字节，用于避免伪共享（false sharing）
static constexpr size_t CACHE_LINE_SIZE = 64;
// 共享内存魔数，校验内存是否为本程序合法共享内存，防止读脏内存
static constexpr uint32_t SHM_MAGIC     = 0x54414C05; // ASCII "TAL05"
// 共享内存协议版本，升级结构时用来做兼容性判断
static constexpr uint32_t SHM_VERSION   = 2;

// 相机图像固定分辨率 1440×1080 RGB8
static constexpr uint32_t IMAGE_WIDTH    = 1440;
static constexpr uint32_t IMAGE_HEIGHT   = 1080;
static constexpr uint32_t IMAGE_CHANNELS = 3;                                     // RGB8 三通道
// 单张图像字节大小：1440*1080*3 = 4665600 Byte
static constexpr size_t IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS;
// 图像三缓冲总内存：3 帧图像缓冲区
static constexpr size_t IMAGE_POOL_SIZE = IMAGE_SIZE * 3;

// 两块独立共享内存文件，存放在 /tmp 临时目录
// 1. 元数据共享内存：存放所有结构体、三缓冲控制原子、状态
static constexpr auto SHM_NAME_META       = "talos_ipc_meta";
// 2. 图像大池共享内存：单独存放超大图像原始像素，避免元内存区域过大
static constexpr auto SHM_NAME_IMAGE_POOL = "talos_ipc_image_pool";

// ====================== 三缓冲原子状态标记位定义 ======================
// state 原子变量 uint8_t 位域划分
static constexpr uint8_t FLAG_NEW   = 0x80; // 第7位：置1代表有新数据可读取
static constexpr uint8_t INDEX_MASK = 0x03; // 低2位掩码：提取就绪缓冲下标 0/1/2

// ====================== 消息数据类型枚举 ======================
enum class MessageType : uint8_t {
    Image      = 0, // 图像帧
    Pose       = 1, // 位姿变换(里程计/云台/相机/枪口)
    GimbalCmd  = 2, // 云台自瞄控制指令
    CameraInfo = 3, // 相机内参畸变参数
};

// ====================== 通信结构体（严格C ABI兼容，跨C++/Rust） ======================
/**
 * @brief 图像元数据结构体，描述一张图像帧信息，不存像素
 * 总大小强制固定32字节，alignas(32)保证内存对齐
 */
struct alignas(32) ImageMeta {
    uint64_t seq;          // 全局帧序号，同步多进程数据
    uint64_t timestamp_ns; // 图像采集纳秒时间戳
    uint32_t width;        // 实际图像宽
    uint32_t height;       // 实际图像高
    uint8_t buffer_id;     // 图像像素存放在图像池的几号缓冲槽 0/1/2
    uint8_t format;        // 像素格式：0=RGB8,1=BGR8,2=GRAY8
    uint8_t _pad[6];       // 手动填充字节，强制整体凑齐32字节，防止编译器自动填充导致跨语言布局错乱
};
// 编译期静态断言，结构体尺寸严格32字节，编译不通过说明布局修改出错
static_assert(sizeof(ImageMeta) == 32, "ImageMeta must be 32 bytes");

/**
 * @brief 位姿数据结构体，存储三维平移+四元数旋转
 * 64字节对齐、固定64字节，适配浮点矩阵运算对齐需求
 */
struct alignas(64) PoseMeta {
    uint64_t frame_seq;  // 绑定对应图像帧序号，用于时序对齐
    float position[3];   // 三维平移 x,y,z 单位米
    float quaternion[4]; // 单位四元数 w,x,y,z 旋转
    uint64_t timestamp_ns;
    uint8_t _pad[16];    // 填充补齐至64字节
};
static_assert(sizeof(PoseMeta) == 64, "PoseMeta must be 64 bytes");

/**
 * @brief 云台自瞄控制指令
 * 下发给云台电机的目标角度、距离、开火建议
 */
struct alignas(32) GimbalCmd {
    uint64_t timestamp_ns;
    float yaw_deg;       // 目标云台偏航角 度
    float pitch_deg;     // 目标云台俯仰角 度
    float distance_m;    // 目标直线距离，-1代表无有效目标
    uint8_t fire_advice; // 1=允许开火，0=禁止开火
    uint8_t _pad[11];    // 填充补齐32字节
};
static_assert(sizeof(GimbalCmd) == 32, "GimbalCmd must be 32 bytes");

/**
 * @brief 相机内参与畸变系数
 * 相机标定原始参数，128字节固定长度
 */
struct alignas(64) CameraInfo {
    uint64_t timestamp_ns;
    double fx, fy;        // 相机焦距
    double cx, cy;        // 图像主点(光心)
    double distortion[5]; // 径向切向畸变 k1 k2 p1 p2 k3
    uint32_t width;
    uint32_t height;
    uint8_t _pad[24];     // 填充补齐128字节
};
static_assert(sizeof(CameraInfo) == 128, "CameraInfo must be 128 bytes");

/**
 * @brief 底盘IMU/轮速观测数据
 * 与Rust端结构体内存布局完全一致，跨语言共享内存互通
 */
struct alignas(64) ChassisObservation {
    uint64_t frame_seq;
    uint64_t timestamp_ns;
    float dt_s;                      // 帧间隔时间
    float v_body[2];              // 底盘坐标系xy线速度
    float wz_radps;               // 底盘自转角速度
    float wheel_linear_mps[4];    // 四轮线速度 [左前、右前、左后、右后]
    float wheel_angular_radps[4]; // 四轮轮角速度
    float a_body[2];              // 底盘xy加速度
    float alpha_z_radps2;         // z角加速度
    float rpy_rad[3];             // 底盘滚转、俯仰、偏航欧拉角
    float gyro_xyz_radps[3];      // IMU陀螺仪原始三轴角速度
    float accel_xyz_mps2[3];      // IMU加速度计三轴
    uint8_t _pad[16];             // 填充补齐128字节
};
static_assert(sizeof(ChassisObservation) == 128, "ChassisObservation must be 128 bytes");

// ====================== 仿真/模拟器真值数据结构 ======================
// 真值最大敌方机器人数量
static constexpr size_t GROUND_TRUTH_MAX_TARGETS = 16;
// 最大能量机关数量
static constexpr size_t GROUND_TRUTH_MAX_RUNES   = 4;

/**
 * @brief 单个敌方机器人/前哨站真值
 */
struct alignas(32) GroundTruthTarget {
    uint64_t frame_seq;
    uint64_t timestamp_ns;
    uint8_t team;        // 0红方 1蓝方
    uint8_t armor_label; // 装甲类型标识
    uint8_t is_outpost;  // 1=前哨站，0=普通机器人
    uint8_t _pad1;
    float position[3];   // 里程计坐标系三维坐标
    float vyaw;          // 机器人偏航角速度 rad/s
    float yaw;           // 当前机器人朝向偏航角
    uint8_t _pad[24];    // 填充至64字节
};
static_assert(sizeof(GroundTruthTarget) == 64, "GroundTruthTarget must be 64 bytes");

/**
 * @brief 能量机关旋转真值完整模型
 */
struct alignas(64) GroundTruthRune {
    uint64_t frame_seq;
    uint64_t timestamp_ns;
    uint8_t team;            // 所属队伍
    uint8_t rune_mode;       // 0小能量机关 1大能量机关
    uint8_t mechanism_state; // 闲置/激活中/已激活/故障
    uint8_t _pad1;
    float r_center_odom[3];  // 能量机关中心里程计坐标
    float radius;            // 旋转半径
    float current_angle;     // 当前叶片角度
    float v_roll;            // 瞬时角速度
    int32_t direction;       // 旋转方向 +1顺时针 -1逆时针
    float sin_amplitude;     // 变速正弦模型振幅a
    float sin_omega;         // 正弦角速度ω
    float sin_phase;         // 正弦相位φ
    float sin_offset;        // 正弦偏移量b
    float relative_time;     // 变速启动后经过时间
    int32_t blade_id;        // 当前待激活叶片，-1无
    uint8_t target_activations[5]; // 各叶片激活计数
    uint8_t _pad[20];        // 填充补齐128字节
};
static_assert(sizeof(GroundTruthRune) == 128, "GroundTruthRune must be 128 bytes");

/**
 * @brief 每一帧所有真值批量包
 * 包含所有敌方机器人+能量机关列表
 */
struct alignas(64) GroundTruthBatch {
    uint64_t frame_seq;
    uint64_t timestamp_ns;
    uint32_t target_count; // 有效机器人数量
    uint32_t rune_count;   // 有效能量机关数量
    GroundTruthTarget targets[GROUND_TRUTH_MAX_TARGETS]; // 机器人数组
    GroundTruthRune runes[GROUND_TRUTH_MAX_RUNES];       // 能量机关数组
    uint8_t _pad[64];      // 尾部填充对齐
};
static_assert(sizeof(GroundTruthBatch) == 1664, "GroundTruthBatch must be 1664 bytes");

/**
 * @brief 运行时自瞄控制状态
 * 存放自动瞄准开关全局状态
 */
struct alignas(64) RuntimeState {
    uint64_t timestamp_ns;
    uint8_t following; // 1开启自动跟踪自瞄，0关闭
    uint8_t _pad[55];  // 填充至64字节
};
static_assert(sizeof(RuntimeState) == 64, "RuntimeState must be 64 bytes");

// ====================== 无锁三缓冲 TripleBuffer 核心结构 ======================
/**
 * @brief 图像三缓冲控制结构
 * 严格按 CacheLine 64字节对齐，解决CPU伪共享
 * 三缓冲模型：生产者写缓冲、消费者读缓冲、中间就绪缓冲，无锁并发
 */
struct alignas(CACHE_LINE_SIZE) ImageTripleBuffer {
    // 原子状态单独占一整条缓存行，避免读写时CPU缓存互相失效
    alignas(CACHE_LINE_SIZE) std::atomic<uint8_t> state{1}; // 初始就绪下标1，无新数据
    uint8_t write_idx{0};                                   // 生产者私有：当前可写入缓冲下标
    uint8_t read_idx{2};                                    // 消费者私有：当前可读缓冲下标
    uint8_t _pad1[61];                                      // 填充填满64字节缓存行隔离原子变量

    // 三块图像元数据缓冲槽位
    ImageMeta slots[3]{};
};
static_assert(sizeof(ImageTripleBuffer) == 192, "ImageTripleBuffer size mismatch");

/**
 * @brief 位姿三缓冲结构，单通道位姿数据无锁传输
 */
struct alignas(CACHE_LINE_SIZE) PoseTripleBuffer {
    alignas(CACHE_LINE_SIZE) std::atomic<uint8_t> state{1};
    uint8_t write_idx{0};
    uint8_t read_idx{2};
    uint8_t _pad1[61];

    PoseMeta slots[3]{};
};
static_assert(sizeof(PoseTripleBuffer) == 256, "PoseTripleBuffer size mismatch");

/**
 * @brief 云台指令三缓冲
 */
struct alignas(CACHE_LINE_SIZE) GimbalTripleBuffer {
    alignas(CACHE_LINE_SIZE) std::atomic<uint8_t> state{1};
    uint8_t write_idx{0};
    uint8_t read_idx{2};
    uint8_t _pad1[61];

    GimbalCmd slots[3]{};
};
static_assert(sizeof(GimbalTripleBuffer) == 192, "GimbalTripleBuffer size mismatch");

// ====================== 共享内存元区域头部 ======================
/**
 * @brief 共享内存头部校验信息，全局标识这块共享内存
 */
struct alignas(CACHE_LINE_SIZE) ShmHeader {
    uint32_t magic;        // 魔数校验
    uint32_t version;      // 协议版本
    uint64_t created_ns;   // 共享内存创建时间戳
    uint64_t heartbeat_ns; // 生产者心跳，消费者判断进程是否存活
    uint32_t image_width;  // 全局图像分辨率
    uint32_t image_height;
    uint8_t _pad[32];      // 填充至64字节缓存行
};
static_assert(sizeof(ShmHeader) == 64, "ShmHeader must be 64 bytes");

/**
 * @brief 元共享内存完整布局结构体
 * 所有控制结构、三缓冲、全局状态全部存放在这块共享内存
 * 独立一块大图像像素分离到另一块 shm_image_pool，防止元内存过大
 * 下方 static_assert 强制校验每个成员内存偏移，一旦结构修改偏移错乱直接编译报错
 */
struct ShmMetaRegion {
    ShmHeader header;                    // 偏移0 头部校验
    ImageTripleBuffer image;             // 偏移64 图像三缓冲
    PoseTripleBuffer poses[5];           // 偏移256 五路位姿通道
    GimbalTripleBuffer gimbal_cmd;       // 偏移1536 云台指令三缓冲
    CameraInfo camera_info;              // 偏移1728 相机内参
    ChassisObservation chassis_observation; // 偏移1856 底盘观测
    GroundTruthBatch ground_truth;       // 偏移1984 仿真真值批量数据
    RuntimeState runtime_state;           // 偏移3648 自瞄运行状态
};
// 强制校验总大小、各成员内存偏移，防止结构调整后跨进程读取错位
static_assert(sizeof(ShmMetaRegion) == 3712, "ShmMetaRegion must be 3712 bytes");
static_assert(offsetof(ShmMetaRegion, camera_info) == 1728, "camera_info offset mismatch");
static_assert(
    offsetof(ShmMetaRegion, chassis_observation) == 1856, "chassis_observation offset mismatch");
static_assert(offsetof(ShmMetaRegion, ground_truth) == 1984, "ground_truth offset mismatch");
static_assert(offsetof(ShmMetaRegion, runtime_state) == 3648, "runtime_state offset mismatch");

// 五路位姿数组下标枚举，方便代码索引
enum PoseIndex : uint8_t {
    POSE_GIMBAL = 0,       // 云台位姿
    POSE_ODOM   = 1,       // 里程计底盘位姿
    POSE_MUZZLE = 2,       // 枪口三维位姿
    POSE_CAMERA = 3,       // 相机光学坐标系位姿
    POSE_CHASSIS_OBSERVATION = 4, // 兼容旧版底盘观测通道（优先使用独立 chassis_observation）
};

} // namespace ipc