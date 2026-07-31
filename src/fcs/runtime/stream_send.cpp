#include "runtime/stream_send.hpp"

// L1层底层硬件/通信输出接口（机器人底层总线/以太网发送封装）
#include "L1_sensor/output_interface.hpp"
// runtime基础、全局能力管理
#include "core/runtime.hpp"
// Quanta传输协议定义：数据包结构、队列类型
#include "quanta/stream_transport.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>
#include <utility> // std::move

namespace fcs::runtime {

/**
 * @brief 注册Quanta视频发送ECS系统
 * 调用时机：register_quanta_stream_systems()内部调用，和编码系统成对注册
 * @param scheduler Talos ECS调度器
 * 运行策略：固定50Hz周期执行
 */
void register_quanta_stream_send_system(talos::Scheduler& scheduler) {
    // 注册【固定频率系统】talos::fixed_rate<目标频率, 最大帧补偿>
    // fixed_rate<50,3>：目标运行频率50Hz（20ms一次）；最多允许累积3帧延迟防止雪崩
    scheduler.add_system<talos::fixed_rate<50, 3>>(
        "stream_send", // 系统名称，用于调试、日志、性能监控
        // System回调函数，Talos调度器自动注入所需全局资源
        [](
            // res_mut：可变资源引用；全局数据包队列，编码系统写、发送系统读
            talos::res_mut<quanta::QuantaPacketQueue> packet_queue,
            // res：只读资源；底层通信输出接口（unique_ptr持有）
            talos::res<std::unique_ptr<fcs::L1::OutputInterface>> output,
            // 运行时能力标志位，全局功能开关
            core::capabilities cap)
        {
            // ===== 前置条件判断，不满足直接跳过本轮发送 =====
            // 1. 全局未开启Quanta流媒体能力
            // 2. 底层通信接口未初始化（空unique_ptr）
            if (!core::capable(*cap, core::Capability::Quanta) || !*output) {
                return;
            }

            // 从队列取出一条待发送分片包（原子并发安全弹出）
            // std::optional包裹，队列为空返回std::nullopt
            auto dequeued = packet_queue->pop_packet();
            if (!dequeued) {
                SPDLOG_DEBUG("Packet queue is empty");
                return;
            }

            // 调用L1底层接口发送Quanta协议数据包
            // std::move：转移数据包内存所有权，避免大包拷贝，零复制发送
            (*output)->send_quanta(std::move(dequeued->packet));

            // 打印调试日志：分片序号、总分片数、队列剩余统计
            // fragment_index 从0开始，+1转为人类可读序号
            SPDLOG_DEBUG(
                "Quanta packet sent: frag={}/{}, packets_left={}, frames_left={}",
                static_cast<unsigned int>(dequeued->packet.fragment_index) + 1U,
                static_cast<unsigned int>(dequeued->packet.fragment_count),
                dequeued->remaining_packets,
                dequeued->remaining_frames);
        });
}

} // namespace fcs::runtime