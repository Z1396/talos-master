#pragma once
// 固定宽度整数类型 uint64_t 序列号、纳秒时间戳
#include <cstdint>
// OpenCV图像容器 cv::Mat
#include <opencv2/core.hpp>

namespace fcs::L1 {

// ============================================================================
// Frame 图像帧数据结构体
// 数据流方向：相机/IPC共享内存外部采集端 → 任务机MissionComputer内部处理
// ============================================================================

/**
 * @brief 单帧图像完整数据包结构体
 * 承载一帧图像的时序标识、图像像素数据，L1层图像输入统一载体
 */
struct Frame {
    /// 图像帧自增序列号，用于丢帧检测、时序对齐、帧匹配
    uint64_t seq{0};
    /// 图像采集时刻纳秒级时间戳，用于多传感器时间同步、提前量计算
    uint64_t timestamp_ns{0};
    /// OpenCV图像矩阵，存储BGR/灰度原始像素数据，使用移动语义避免拷贝
    cv::Mat image;

    /**
     * @brief 静态工厂构造方法，快速创建Frame对象，使用移动语义转移cv::Mat所有权
     * @param seq 帧序列号
     * @param ts 采集纳秒时间戳
     * @param mat 原始图像矩阵，std::move转移，无深拷贝开销
     * @return 构造完成的Frame实例
     */
    static Frame from_mat(const uint64_t seq, const uint64_t ts, cv::Mat mat) {
        return Frame{seq, ts, std::move(mat)};
    }
};

} // namespace fcs::L1