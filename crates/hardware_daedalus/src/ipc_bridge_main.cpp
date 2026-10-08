// ============================================================================
// talos_ipc_bridge：跨机仿真网络桥（Windows Rust 仿真 ↔ 本机共享内存）
//
// 场景：仿真跑在 Windows、Talos 跑在 Linux 小电脑，两机用网线组局域网。
// 共享内存(mmap文件)只能同机互通，本桥负责网络包 → 共享内存的落地转换：
//
//   UDP(control_port)  收：位姿×5 / 底盘观测 / 真值 / 相机内参 / 运行状态 / 心跳
//   TCP(image_port)    收：JPEG 图像流（长度前缀分帧）
//   UDP(→仿真源地址)   发：云台指令（从共享内存三缓冲轮询取出后回传）
//
// 落地方式：以生产者模式 ShmClient::create() 创建 /tmp 共享内存，
// 把收到的数据 publish 进去；Talos 主程序零改动，仍 ShmClient::connect() 消费。
//
// 时域统一：所有数据（含图像）的 timestamp_ns 在接收时改写为 NUC 本机时钟，
// 避免跨机时钟不同步污染 fast_tf 时序缓冲（无需 NTP 对时）。
//
// 启动顺序：先启动本桥（创建共享内存），再启动 talos 主程序。
//
// 用法：talos_ipc_bridge [控制端口=9571] [图像端口=9572]
// ============================================================================

// 项目头文件：共享内存客户端（create/publish 接口）与网络线协议定义
#include "shm_client.hpp"
#include "net_protocol.hpp"

// Linux 网络套接字：UDP/TCP 收发
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
// close 系统调用
#include <unistd.h>

// 原子变量：仿真回传地址有效性、帧计数
#include <atomic>
// 时间：心跳、时间戳改写、主循环节拍
#include <chrono>
// atoi 端口参数解析
#include <cstdlib>
// printf 打印状态
#include <cstdio>
// memcpy/memset
#include <cstring>
// 互斥锁：保护仿真回传地址（UDP收包线程写、云台回传线程读）
#include <mutex>
// 线程：UDP循环、TCP循环与主循环分离
#include <thread>
// std::ref 线程传参、std::vector 动态缓冲
#include <functional>
#include <vector>

// OpenCV：JPEG 解码为 cv::Mat
#include <opencv2/imgcodecs.hpp>

using namespace ipc;
using namespace ipc::net;

namespace {

// 发送序号计数器（诊断丢包用，每发一包+1）
std::atomic<uint64_t> g_seq{1};

// 仿真回传地址缓存：UDP 收包线程持续更新，云台回传线程读取
std::mutex g_sim_addr_mutex;
// 仿真回传地址缓存：UDP 收包线程持续更新，云台回传线程读取
sockaddr_in g_sim_addr{};
// 仿真回传地址有效性标志：UDP 收包线程更新，云台回传线程读取
bool g_sim_addr_valid = false;

// 获取本机 system_clock 纳秒时间戳（NUC 时钟域，桥负责统一时域）
uint64_t bridge_now_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
}

// TCP 精确接收 n 字节（处理内核部分送达），对端断开返回 false
bool recv_exact(int fd, void* buf, size_t n) {
    auto* p = static_cast<uint8_t*>(buf);
    size_t got = 0;
    while (got < n) {
        /*- TCP 收数据。`ssize_t`是有符号整数（可以返回负数代表错误）
        - 参数
        - `sockfd`：连接 fd
        - `buf`：接收缓冲区
        - `len`：想要读取的字节数
        - `flags=0`：默认阻塞模式
        - 返回值
        > 0：读到多少字节
        - =0：**对端 TCP 连接正常关闭**
        - <0：发生错误
        > 你封装的`recv_exact`就是为了解决 TCP**流式无边界**问题：一次 recv 不一定读完你想要的全部字节，循环读直到凑够 n 字节。*/
        const ssize_t r = ::recv(fd, p + got, n - got, 0);
        if (r <= 0) {
            return false;
        }
        got += static_cast<size_t>(r);
    }
    return true;
}

// ====================== UDP 控制包接收线程 ======================
/**
 * @brief 循环接收仿真发来的小数据包，解析后落地到共享内存
 * 每收到一包都记录发送方地址（云台指令回传的目标）
 */
void udp_recv_loop(ShmClient& client, int udp_fd) {
    // 接收缓冲区：包头 + 最大载荷（真值批量1664B），2048字节富余
    uint8_t buf[sizeof(PacketHeader) + NET_MAX_PAYLOAD];
    sockaddr_in src{};
    socklen_t src_len = sizeof(src);

    while (true) {
        // recvfrom: 从指定地址接收数据包，阻塞等待新数据,会时刻更新 src 地址信息
        // 接收缓冲区：包头 + 最大载荷（真值批量1664B），2048字节富余
        const ssize_t r = ::recvfrom(udp_fd, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&src), &src_len);
        if (r < static_cast<ssize_t>(sizeof(PacketHeader))) {
            continue; // 过短包直接丢弃
        }

        // 解析包头并校验魔数/版本
        PacketHeader header{};
        // memcpy: 复制buf到本地结构体header
        std::memcpy(&header, buf, sizeof(header));
        if (header.magic != NET_MAGIC || header.version != NET_VERSION) {
            continue;
        }

        // 记录仿真地址：云台指令回传用（仿真从固定socket发包，源地址稳定）
        {
            std::lock_guard lock(g_sim_addr_mutex);
            //g_sim_addr: 仿真回传地址缓存，UDP 收包线程持续更新，云台回传线程读取
            g_sim_addr      = src;
            // g_sim_addr_valid: 仿真回传地址有效性标志，UDP 收包线程更新，云台回传线程读取
            g_sim_addr_valid = true;
        }

        // 载荷起始指针与实际长度校验
        const uint8_t* payload = buf + sizeof(header);
        // got_len: 实际载荷长度，与header.payload_len对比
        const size_t got_len   = static_cast<size_t>(r) - sizeof(header);
        if (got_len != header.payload_len) {
            continue; // 长度不匹配（粘包/截断）丢弃
        }

        // 按类型分发落地；时间戳统一改写为 NUC 本机时钟
        switch (static_cast<PacketType>(header.type)) {
        case PacketType::Pose: {
            if (header.index > 4 || got_len != sizeof(PoseMeta)) {
                break;
            }
            PoseMeta pm{};
            std::memcpy(&pm, payload, sizeof(pm));
            const ShmClient::Pose pose{
                .x            = pm.position[0],
                .y            = pm.position[1],
                .z            = pm.position[2],
                .qw           = pm.quaternion[0],
                .qx           = pm.quaternion[1],
                .qy           = pm.quaternion[2],
                .qz           = pm.quaternion[3],
                .frame_seq    = pm.frame_seq,
                .timestamp_ns = bridge_now_ns(),
            };
            client.publish_pose(static_cast<PoseIndex>(header.index), pose);
            break;
        }
        case PacketType::ChassisObserv: {
            if (got_len != sizeof(ChassisObservation)) {
                break;
            }
            ChassisObservation obs{};
            std::memcpy(&obs, payload, sizeof(obs));
            obs.timestamp_ns = bridge_now_ns();
            client.publish_chassis_observation(obs);
            break;
        }
        case PacketType::GroundTruth: {
            if (got_len != sizeof(GroundTruthBatch)) {
                break;
            }
            GroundTruthBatch batch{};
            std::memcpy(&batch, payload, sizeof(batch));
            batch.timestamp_ns = bridge_now_ns();
            client.publish_ground_truth(batch);
            break;
        }
        case PacketType::CameraInfo: {
            if (got_len != sizeof(CameraInfo)) {
                break;
            }
            CameraInfo info{};
            std::memcpy(&info, payload, sizeof(info));
            info.timestamp_ns = bridge_now_ns();
            client.publish_camera_info(info);
            break;
        }
        case PacketType::RuntimeState: {
            if (got_len != sizeof(RuntimeState)) {
                break;
            }
            RuntimeState state{};
            std::memcpy(&state, payload, sizeof(state));
            client.publish_runtime_state(state.following != 0, bridge_now_ns());
            break;
        }
        case PacketType::Heartbeat: {
            // 心跳仅用于记录仿真存活，桥自身作为生产者持续刷自己的心跳
            client.update_heartbeat();
            break;
        }
        default:
            break; // 未知类型丢弃，保证前向兼容
        }
    }
}

// ====================== TCP JPEG 图像流接收线程 ======================
/**
 * @brief 循环接受仿真图像连接，按"40字节帧头+JPEG数据"分帧接收，
 *        解码后写入共享内存图像池（分辨率必须等于 IMAGE_WIDTH×IMAGE_HEIGHT）
 */
void tcp_image_loop(ShmClient& client, uint16_t image_port) {
    // 创建 TCP 监听套接字
    // AF_INET: IPv4 协议族
    // SOCK_STREAM: 流式套接字，用于可靠的连接
    // 0: 选择默认协议（TCP）
    const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        std::perror("[bridge] image socket");
        return;
    }
    int opt = 1;
    //setsockopt:api函数，用于设置套接字选项
    // SOL_SOCKET: 套接字选项
    // SO_REUSEADDR: 程序重启后端口立即可绑定，避免 TIME_WAIT 卡住,选项值，1表示开启，0表示关闭
    // &opt: 选项值指针，指向 opt 变量,1表示开启，0表示关闭
    // sizeof(opt): 选项值大小，这里为 int 类型，大小为 4 字节
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // sockaddr_in: IPv4 地址结构体其他的字段，如sin_addr.s_addr 表示IP地址，sin_port 表示端口号
    sockaddr_in addr{};
    // AF_INET: IPv4 协议族
    addr.sin_family      = AF_INET;
    // INADDR_ANY: 任意IP(0,0,0,0)地址，监听所有接口
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(image_port);
    // bind: 绑定套接字到指定地址，监听端口tcp需要绑定到具体地址，udp不需要绑定直接监听
    if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("[bridge] image bind");
        ::close(listen_fd);
        return;
    }
    /*- **TCP 独有，UDP 不能调用 listen**
    - 作用：标记 socket 为被动监听套接字，准备接受客户端连接
    - 参数`backlog=1`：未完成连接队列最大长度，这里只允许**1 个仿真图像客户端**接入*/
    ::listen(listen_fd, 1);
    std::printf("[bridge] image TCP listening on :%u\n", image_port);

    uint64_t frame_count = 0;
    uint64_t drop_count  = 0;

    while (true) {
        /*- **TCP 独有，阻塞函数！
        - 作用：阻塞等待客户端 TCP 连接；客户端连上之后，返回全新 conn_fd(新套接字作用是和客户端通信) 专门用来和这个客户端通信
        - 第二个、第三个参数填`nullptr`：不需要获取客户端地址
        - 注意：`listen_fd`仍然继续监听下一个连接；`conn_fd`才是收发图像的 fd*/
        const int conn_fd = ::accept(listen_fd, nullptr, nullptr);
        if (conn_fd < 0) {
            continue;
        }
        std::printf("[bridge] image sender connected\n");

        while (true) {
            // 1. 精确接收 40 字节帧头
            ImageFrameHeader fh{};
            // 1.0 接收帧头是否成功
            if (!recv_exact(conn_fd, &fh, sizeof(fh))) {
                break; // 对端断开
            }
            // 1.1 校验魔数和类型是否匹配
            if (fh.magic != NET_MAGIC || fh.type != static_cast<uint8_t>(PacketType::ImageJpeg)) {
                break; // 协议错乱，断开重连
            }
            // 2. JPEG 长度合法性校验
            if (fh.jpeg_len < JPEG_MIN_LEN || fh.jpeg_len > JPEG_MAX_LEN) {
                break;
            }

            // 3. 接收 JPEG 数据
            static std::vector<uint8_t> jpeg_buf;
            // 3.0 重置缓冲区大小
            jpeg_buf.resize(fh.jpeg_len);
            // jpeg_buf.data()表示jpeg_buf的第一个元素的地址;
            // fh.jpeg_len表示要接收的字节数，recv_exact表示阻塞接收
            /*recv_exact就是为了解决recv函数一次可能只接收部分数据的问题，确保每次接收完整数据*/
            if (!recv_exact(conn_fd, jpeg_buf.data(), fh.jpeg_len)) {
                break;
            }

            // 4. 解码 JPEG → BGR Mat
            /*参数解释：
            - 1：单通道（JPEG）
            - static_cast<int>(fh.jpeg_len)：图像高度（行数）
            - CV_8U：8 位无符号整数（每个像素 1 字节）
            - jpeg_buf.data()：图像数据指针
            `cv::Mat raw(1, len, CV_8U, ptr)`不是图像！不是图片矩阵！
            它只是**把一段连续的 JPEG 二进制内存，包装成一个单行的一维矩阵，专门给`imdecode`当输入**。
            > JPEG 是**压缩后的二进制字节流**，不是像素矩阵，不能直接当成图像。`imdecode`才负责把压缩 JPEG 字节 → 解压成 BGR 像素图。*/
            const cv::Mat raw(1, static_cast<int>(fh.jpeg_len), CV_8U, jpeg_buf.data());
            /* `imdecode`作用
            输入：压缩图像的二进制字节缓冲区（jpg/png 等）**
            输出：解压之后的像素矩阵
            - `cv::IMREAD_COLOR`：解压为 3 通道 BGR（OpenCV 默认通道顺序，不是 RGB！）
            - 解压完成后，`mat`才是真正的图像矩阵：
            - `mat.rows`：图像高度
            - `mat.cols`：图像宽度
            - `mat.type()`：`CV_8UC3`，3 通道 8 位 BGR 像素*/
            const cv::Mat mat = cv::imdecode(raw, cv::IMREAD_COLOR);
            if (mat.empty()) {
                ++drop_count;
                continue;
            }
            // 5. 分辨率守卫：图像池按固定 1440×1080 布局，不符直接丢弃
            if (mat.cols != static_cast<int>(IMAGE_WIDTH) || mat.rows != static_cast<int>(IMAGE_HEIGHT)) {
                ++drop_count;
                if (drop_count % 100 == 1) {
                    std::printf(
                        "[bridge] image size mismatch %dx%d, expect %ux%u (dropped %llu)\n",
                        mat.cols, mat.rows, IMAGE_WIDTH, IMAGE_HEIGHT,
                        static_cast<unsigned long long>(drop_count));
                }
                continue;
            }

            // 6. 写入共享内存图像池，时间戳改写为 NUC 时钟
            client.publish_image(mat, fh.seq, bridge_now_ns());
            ++frame_count;
            // 7. 打印统计信息
            // 每 300 帧打印一次，避免输出过快
            // 打印格式：[bridge] images: 123456 (dropped 7890)
            if (frame_count % 300 == 0) {
                std::printf(
                    "[bridge] images: %llu (dropped %llu)\n",
                    static_cast<unsigned long long>(frame_count),
                    static_cast<unsigned long long>(drop_count));
            }
        }
        std::printf("[bridge] image sender disconnected\n");
        ::close(conn_fd);
    }
}

// ====================== 云台指令回传 ======================
/**
 * @brief 从共享内存三缓冲取出 Talos 写入的云台指令，UDP 回传给仿真
 * @return true 成功发出；false 无指令/仿真地址未知
 */
bool forward_gimbal_cmd(ShmClient& client, int udp_fd) {
    // 未收到过任何仿真包，不知道往哪回
    // 仿真回传地址有效性标志：UDP 收包线程更新，云台回传线程读取
    sockaddr_in dst{};
    {
        std::lock_guard lock(g_sim_addr_mutex);
        if (!g_sim_addr_valid) {
            return false;
        }
        dst = g_sim_addr;
    }

    // borrow 语义取出新指令（桥是唯一消费者，取出即消费）
    const auto cmd_opt = client.recv_gimbal_cmd();
    if (!cmd_opt) {
        return false;
    }

    // 组包：32字节包头 + GimbalCmd 载荷
    std::vector<uint8_t> packet(sizeof(PacketHeader) + sizeof(GimbalCmd));
    PacketHeader header{};
    header.magic        = NET_MAGIC;
    header.version      = NET_VERSION;
    header.payload_len  = sizeof(GimbalCmd);
    header.type         = static_cast<uint8_t>(PacketType::GimbalCmd);
    header.index        = 0;
    header.seq          = g_seq.fetch_add(1, std::memory_order_relaxed);
    header.timestamp_ns = bridge_now_ns();
    std::memcpy(packet.data(), &header, sizeof(header));
    std::memcpy(packet.data() + sizeof(header), &*cmd_opt, sizeof(GimbalCmd));

    ::sendto(
        udp_fd, packet.data(), packet.size(), 0, reinterpret_cast<const sockaddr*>(&dst),
        sizeof(dst));
    return true;
}

} // namespace

// ====================== 主入口 ======================
int main(int argc, char* argv[]) {
    // 启动参数：控制端口、图像端口（可选，不输入argc就用默认 9571/9572）
    const uint16_t control_port = argc > 1 ? static_cast<uint16_t>(std::atoi(argv[1])) : DEFAULT_CONTROL_PORT;
    const uint16_t image_port   = argc > 2 ? static_cast<uint16_t>(std::atoi(argv[2])) : DEFAULT_IMAGE_PORT;
    // 打印端口号，方便调试和确认
    std::printf(
        "[bridge] talos_ipc_bridge start (control UDP :%u, image TCP :%u)\n", control_port,
        image_port);

    // 1. 生产者模式创建共享内存（O_TRUNC 覆盖残留文件，初始化全部三缓冲）
    auto client_result = ShmClient::create();
    if (!client_result) {
        std::printf("[bridge] shm create failed: %s\n", fmt::format("{}", client_result.error()).c_str());
        return 1;
    }
    ShmClient& client = *client_result;
    // 打印创建的共享内存文件名，方便调试
    std::printf("[bridge] shm created: %s + %s\n", SHM_NAME_META, SHM_NAME_IMAGE_POOL);

    // 2. 创建 UDP 控制套接字并绑定端口
    // socket: 创建套接字返回文件描述符，失败返回 -1 并设置 errno
    // AF_INET: IPv4 地址族，SOCK_DGRAM: UDP 套接字类型
    const int udp_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0) {
        std::perror("[bridge] udp socket");
        return 1;
    }
    //
    //sockaddr_in: IPv4 地址结构体，包含地址族、IP地址、端口号9571等
    sockaddr_in udp_addr{};
    udp_addr.sin_family      = AF_INET;
    // udp_addr.sin_addr.s_addr 是 IP 地址，htonl: 将主机字节序转换为网络字节序 INADDR_ANY: 任意 IP 地址，0.0.0.0 表示所有接口
    udp_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    //udp_addr.sin_port 是控制端口号为9571，htons: 将主机字节序转换为网络字节序
    udp_addr.sin_port        = htons(control_port);
    // bind: 绑定套接字到指定端口，失败返回 -1 并设置 errno（将udp套接字绑定到9571端口）
    /*各个参数说明*/
    // udp_fd: udp套接字文件描述符
    // reinterpret_cast<sockaddr*>(&udp_addr): 将 sockaddr_in 结构体转换为 sockaddr 指针
    // sizeof(udp_addr): 地址结构体大小
    if (::bind(udp_fd, reinterpret_cast<sockaddr*>(&udp_addr), sizeof(udp_addr)) < 0) {
        std::perror("[bridge] udp bind");
        return 1;
    }
    std::printf("[bridge] control UDP listening on :%u\n", control_port);

    // 3. 启动两条接收线程：UDP 小包 + TCP 图像流
    /* 参数 2：`std::ref(client)`
    `std::ref` 来自 `<functional>`，**包装成引用包装器**。
    - `std::thread` 默认会**拷贝 / 移动**所有传入的参数！
    - 如果直接写 `client`，不写`std::ref`：`client`会被**拷贝一份**传到子线程。子线程操作的是副本，主线程的 client 完全不受影响，这基本不是你想要的。
    - `std::ref(client)`：告诉 thread，**不要拷贝，传引用。子线程里面拿到的是`client`的引用，读写的是主线程原来那个对象。
    `std::ref` 会把一个变量**包装成一个引用包装器 `std::reference_wrapper<T>`**。
    > ⚠️ 重大风险：生命周期！
    > 子线程还在跑的时候，如果主线程`client`对象被销毁，子线程拿着悬空引用，直接崩溃。
    > 机器人代码里一般保证 client 生命周期长于 udp_thread。
    
    参数 3：`udp_fd`
    int 类型的 socket 文件描述符，**值传递**。
    这里会拷贝一份 int 的值给到子线程。
    fd 本质只是一个数字编号，拷贝数字完全没问题，不需要`std::ref`。
    子线程内部用这个数字调用`recvfrom(udp_fd, ...)`，访问同一个内核 socket 对象。
    > 重点：fd 只是一个数字，进程内所有线程共用同一个 socket，多个线程使用同一个 fd 是允许的（注意并发读写 socket 的竞争问题）。*/
    // udp_thread: UDP 接收线程，负责处理控制端口的 UDP 数据包，然后将数据写入共享内存
    //client: 共享引用 client 实例，避免复制构造函数,由create()函数创建一个路径然后打开共享内存文件然后设置好文件大小映射到进程内存空间
    std::thread udp_thread(udp_recv_loop, std::ref(client), udp_fd);
    // tcp_thread: TCP 接收线程，负责处理图像端口的 TCP 数据流，然后将数据写入共享内存
    std::thread tcp_thread(tcp_image_loop, std::ref(client), image_port);

    // 4. 主循环 200Hz：心跳保活 + 云台指令轮询回传
    while (true) {
        client.update_heartbeat();
        // 5. 云台指令轮询回传
        // forward_gimbal_cmd: 云台指令回传函数，负责将云台指令发送到仿真回传地址
        forward_gimbal_cmd(client, udp_fd);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 不可达：桥以 Ctrl+C/SIGTERM 终止，套接字与线程由内核回收
    ::close(udp_fd);
    udp_thread.join();
    tcp_thread.join();
    return 0;
}
