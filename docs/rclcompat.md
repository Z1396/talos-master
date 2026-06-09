# rclcompat 快速参考

ROS2 风格兼容层 API - `Node`, `Publisher`, `Subscription`, `Timer`, `ResourceAccessor`

## 30秒速查

| 我想...                 | 代码                                                                    |
| ----------------------- | ----------------------------------------------------------------------- |
| **创建 Node**           | `talos::rclcompat::Node node("name", scheduler);`                       |
| **创建 Publisher**      | `auto pub = node.create_publisher Msg>();`                              |
| **发布消息**            | `pub.publish(Message{...});`                                            |
| **订阅消息**            | `node.create_subscription Msg>([](const Msg& m) {});`                   |
| **创建定时器 (30Hz)**   | `node.create_wall_timer(Frequency::Hz_30, [=]() {});`                   |
| **定时器回调类成员**    | `node.create_wall_timer(Frequency::Hz_30, [this]() { this->tick(); });` |
| **访问共享资源 (只读)** | `auto r = accessor.get();`                                              |
| **访问共享资源 (可写)** | `auto r = accessor.get_mut();`                                          |
| **插入资源到 World**    | `node.insert_resource(std::move(resource));`                            |
| **完成注册**            | `node.finalize();`                                                      |
| **运行调度器**          | `scheduler.run();`                                                      |

## 头文件

```cpp
#include "scheduler/rclcompat/node.hpp"
```

---

## 可用频率

```cpp
namespace talos::rclcompat {
enum class Frequency : uint32_t {
    Hz_1 = 1, Hz_2 = 2, Hz_5 = 5,
    Hz_10 = 10, Hz_20 = 20, Hz_27 = 27, Hz_30 = 30,
    Hz_50 = 50, Hz_60 = 60, Hz_100 = 100,
    Hz_120 = 120, Hz_150 = 150, Hz_200 = 200,
    Hz_250 = 250, Hz_500 = 500, Hz_1000 = 1000
};
}
```

---

## 完整系统示例

```cpp
#include "scheduler/rclcompat/node.hpp"
#include "scheduler/scheduler/scheduler.hpp"

class ArmorDetectorNode : public talos::rclcompat::Node {
public:
    explicit ArmorDetectorNode(std::string name, talos::Scheduler& scheduler) noexcept
        : Node(std::move(name), scheduler) {

        detections_pub_ = this->create_publisher<DetectionBatch>();
        debug_image_pub_ = this->create_publisher<DebugImage>();
        status_pub_ = this->create_publisher<NodeStatus>();

        this->create_subscription<ImageFrame>(
            [this](const ImageFrame& img) {
                this->on_image_received(img);
            }
        );

        // 访问共享 TF 资源 (只读)
        if (this->has_resource<TfBuffer>()) {
            tf_accessor_ = this->create_resource<TfBuffer>();
        }

        // 创建定时器: 1Hz 心跳
        this->create_wall_timer(talos::rclcompat::Frequency::Hz_1, [this]() {
            this->publish_heartbeat();
        });

        // 创建定时器: 10Hz 状态统计
        this->create_wall_timer(talos::rclcompat::Frequency::Hz_10, [this]() {
            this->update_statistics();
        });
    }

private:
    talos::rclcompat::Publisher<DetectionBatch> detections_pub_;
    talos::rclcompat::Publisher<DebugImage> debug_image_pub_;
    talos::rclcompat::Publisher<NodeStatus> status_pub_;

    talos::rclcompat::ResourceAccessor<TfBuffer> tf_accessor_;

    std::atomic<int> frame_count_{0};
    std::atomic<int> detection_count_{0};
    std::atomic<uint64_t> last_publish_time_ns_{0};

    void on_image_received(const ImageFrame& img) {
        frame_count_.fetch_add(1);

        // .........

        // 发布检测结果
        detections_pub_.publish(DetectionBatch{
            .detections = detections,
            .timestamp_ns = img.timestamp_ns
        });
    }

    // 回调函数: 定时器心跳
    void publish_heartbeat() {
        uint64_t now = clock::now_ns();

        status_pub_.publish(NodeStatus{
            .node_name = std::string(this->name()),
            .status = NodeStatus::RUNNING,
            .frames_processed = frame_count_.load(),
            .detections_found = detection_count_.load(),
            .timestamp_ns = now
        });

        last_publish_time_ns_.store(now);
    }

    // 回调函数: 状态统计
    void update_statistics() {
        int frames = frame_count_.load();
        int detections = detection_count_.load();

        double detection_rate = frames > 0
            ? static_cast<double>(detections) / frames
            : 0.0;

        spdlog::debug("Detection rate: {:.2%}", detection_rate);
    }

    // 辅助函数: 检测装甲板
    std::vector<ArmorDetection> detect_armor(const cv::Mat& img) {
        // 实际检测逻辑...
        return {};
    }

    DebugImage create_debug_image(
        const cv::Mat& img,
        const std::vector<ArmorDetection>& detections
    ) {
        // 创建调试图像...
        return DebugImage{};
    }
};

int main() {
    // 创建自定义 Node
    ArmorDetectorNode detector("armor_detector", scheduler);

    // 注册所有系统 (必须在创建 pub/sub 后调用)
    auto result = detector.finalize();
    if (!result) {
        spdlog::error("Failed to finalize node: {}", result.error());
        return 1;
    }

    return 0;
}
```

- **Lambda 捕获 `this`** 比 `std::bind` 更安全，编译期类型推导更准确
- **Publisher** 是 move-only 类型，确保单写入者语义，必须存储为类成员
- **ResourceAccessor** 可拷贝，多个类成员可持有同一资源访问器
- **定时器运行在独立线程** (fixed_rate)，适合周期性任务
- **finalize()** 必须在创建所有 pub/sub 后调用，触发系统注册
