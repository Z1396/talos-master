# 计划：补全装甲检测调试图像的 Foxglove 可视化

## Context（背景）

项目中一批 Foxglove 调试图话题（`/debug/binary_img`、`/debug/number_img`）只有通道定义、没有任何发布者——用户调 `binary_threshold`、曝光、看灯条形态时无图可看。目标：把传统后端检测流程中**用完即丢的中间图**（二值化掩码、数字分类器裁剪图）发布到 Foxglove，复用**已预留**的话题定义，零新增话题注册成本。

采用项目内**现成的 RuneDebugFrame 模式**（能量机关调试图的完整先例）：L2 侧编码 JPEG → 专用调试通道 → 可视化系统低频发布。

## 数据流（目标状态）

```
traditional.cpp detect_impl()          armor/systems.cpp detector系统        l2_perception_systems.cpp
binary / number_img(局部量)  ──节流编码──►  take_debug_frame()  ──write──►  spmc<ArmorDebugFrame>  ──fixed_rate发布──►
                                        (新增虚函数,默认空)                        /debug/binary_img + /debug/number_img
```

## 改动清单（6 个文件）

### 1. `src/fcs/L2_perception/armor/types.hpp`（或同构位置，与 rune/types.hpp 对齐）
- 新增 `ArmorDebugFrame` 结构（仿 `RuneDebugFrame`，rune/types.hpp L22-43）：
  - `uint64_t timestamp_ns`、`std::string frame_id`
  - `std::vector<uint8_t> binary_jpeg`（二值掩码 JPEG）
  - `std::vector<uint8_t> number_jpeg`（数字分类裁剪图 JPEG，20×28 放大 4 倍后编码）
- 编码工具函数 `encode_jpeg()` 直接仿 rune_systems.cpp L42-54

### 2. `src/fcs/core/channel_topics.hpp`
- 新增 `ArmorDebugFrameChannelTopic`（仿 `RuneDebugFrameChannelTopic`，L34 附近有 DetectionChannelTopic 先例）

### 3. `src/fcs/L2_perception/armor/backends/base.hpp`
- `BackendBase` 加虚函数（默认空实现，其他后端零改动）：
  ```cpp
  virtual std::optional<ArmorDebugFrame> take_debug_frame() { return std::nullopt; }
  ```

### 4. `src/fcs/L2_perception/armor/backends/traditional.hpp/.cpp`
- `TraditionalBackend` 覆写 `take_debug_frame()`：内部持 mutex + `std::optional<ArmorDebugFrame> pending_debug_`
- `detect_impl()` 末尾：**时间节流**（距上次编码 <50ms 直接跳过，~20Hz，避免 200Hz 全速编码拖累检测）后：
  - binary 掩码 JPEG 编码（复用局部变量，quality 50）
  - 数字裁剪图：取本次**置信度最高**装甲的 `extract_number()` 结果（20×28 单通道），resize ×4 + 转 BGR 再编码
  - 填入 `pending_debug_`
- `take_debug_frame()` 加锁取出并清空

### 5. `src/fcs/L2_perception/armor/systems.cpp`（armor_detector 系统，L570 附近）
- lambda 参数追加 `talos::spmc_mut<ArmorDebugFrame, ArmorDebugFrameChannelTopic> debug_out`
- `detect()` 调用后：`if (auto dbg = backend->get()->take_debug_frame()) debug_out.write(std::move(*dbg));`

### 6. `src/fcs_visualization/systems/l2_perception_systems.cpp`
- 新增系统 `foxglove_l2_debug_images`（仿 rune_systems.cpp L39-86）：
  - 输入 `talos::spmc<ArmorDebugFrame, ArmorDebugFrameChannelTopic>`
  - 有新帧时分别填充 `BinaryImageMessage`（→`/debug/binary_img`）与 `PatternImageMessage`（→`/debug/number_img`）
  - 消息格式照抄 `publish_jpeg_image()`（l1_sensor_systems.cpp L389-410）：`CompressedImage{timestamp, frame_id, format="jpeg", data}`
- **不需要改 foxglove_types.hpp**：binary_img_def/pattern_img_def/通道/消息别名/ChannelRegistry 全部已存在

## 不做的事（避免过度设计）

- 不给 `ArmorDetectionBatch` 塞 cv::Mat（它贯穿 PnP/L3 主链路，Mat 随拷贝传播代价大）
- 不加 toml 开关：节流已把开销控制在 ~20Hz 编码，正式比赛想彻底关闭只需注释一处 write
- ONNX/TRT/Axera 后端不产生二值图（无此概念），默认虚函数返回空，零改动

## 验证

1. `./build.sh --release --no-run` 编译通过
2. `./build/bin/talos` 启动（Traditional 后端 + Armor capability）
3. Foxglove 连 8765 → 加两个 **Image 面板**：
   - `/debug/binary_img` → 出现二值化掩码（调节 `binary_threshold` 应实时看到亮区变化）
   - `/debug/number_img` → 出现放大后的数字裁剪图（对准装甲板时应显示数字形状）
4. `/perf_stats` 确认 `armor_detection` 耗时无显著上升（节流生效）
5. 回归：PnP 距离验证流程不受影响（`/solver/measurement` 的 distance 照常）
