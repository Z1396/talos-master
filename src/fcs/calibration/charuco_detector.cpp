// 棋盘格标定板检测器实现头文件
#include "calibration/charuco_detector.hpp"
// ChArUco复合标定板检测器实现头文件
#include "calibration/chessboard_detector.hpp"

// 编译开关：存在OpenCV Aruco模块才引入相关头文件
#ifdef HAVE_OPENCV_ARUCO
// Aruco基础标记字典、检测器API
# include <opencv2/aruco.hpp>
// ChArUco棋盘+标记复合标定板类
# include <opencv2/aruco/charuco.hpp>
// 图像灰度转换、基础图像处理函数
# include <opencv2/imgproc.hpp>
#endif

namespace fcs::calibration {

// ===================== ChArUco检测器实现，仅Aruco模块开启时编译 =====================
#ifdef HAVE_OPENCV_ARUCO

/**
 * @brief ChArUcoDetector 构造函数
 * @param width 棋盘格内角点列数
 * @param height 棋盘格内角点行数
 * @param square_size 棋盘方格物理边长（米）
 * @param marker_size 方格内ArUco标记边长（米）
 * @param dictionary ArUco标记字典枚举
 * @ noexcept 无抛异常，构造失败逻辑统一通过detect返回错误
 */
CharucoDetector::CharucoDetector(
    uint32_t width, uint32_t height, double square_size, double marker_size,
    ArucoDictionary dictionary) noexcept
    // 转换为int尺寸存入成员
    : board_size_(static_cast<int>(width), static_cast<int>(height))
    , square_size_(square_size)
    , marker_size_(marker_size)
    // 转换自定义枚举为OpenCV内置标记字典
    , dictionary_(cv::aruco::getPredefinedDictionary(to_opencv_dict(dictionary)))
    // 实例化OpenCV ChArUco标定板对象
    , charuco_board_(
          cv::aruco::CharucoBoard(
              cv::Size(static_cast<int>(width), static_cast<int>(height)),
              static_cast<float>(square_size), static_cast<float>(marker_size), dictionary_))
    // 使用默认标记检测参数
    , detector_params_()
    , object_points_() {
    // 预加载标定板全部三维世界坐标点，后续detect直接索引复用
    object_points_ = charuco_board_.getChessboardCorners();
}

/**
 * @brief 执行单帧图像ChArUco角点检测，实现CalibrationBoard虚接口
 * @param image 输入原图（BGR彩色/灰度单通道）
 * @param ts 图像采集纳秒时间戳，绑定检测结果用于时序对齐
 * @return std::expected<CornerDetection, string> 检测结果/错误文本
 */
std::expected<CornerDetection, std::string>
    CharucoDetector::detect(const cv::Mat& image, timestamp_ns_t ts) noexcept {
    // 初始化空检测结果结构体
    CornerDetection result;
    result.timestamp_ns = ts;
    // 拷贝原图存入结果，供可视化绘制使用
    result.image        = image.clone();

    // 图像预处理：彩色图转灰度，灰度图直接复用
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // 使用当前构造绑定的ChArUco板创建检测器实例
    cv::aruco::CharucoDetector detector(charuco_board_);
    // 加载构造时保存的标记检测参数
    detector.setDetectorParameters(detector_params_);

    // 输出容器：检测到的插值棋盘角点、对应标记ID
    std::vector<cv::Point2f> charuco_corners;
    std::vector<int> charuco_ids;

    // 核心检测：识别ArUco标记并插值生成棋盘格内角点
    detector.detectBoard(gray, charuco_corners, charuco_ids);

    // 获取有效角点数量
    const int num_charuco = static_cast<int>(charuco_ids.size());
    // 有效角点不足6个，判定检测失败（标定最低有效点数阈值）
    if (num_charuco < 6) {
        result.success = false;
        return std::unexpected("Not enough ChArUco corners detected");
    }

    // 清空结果点位容器，预分配内存避免扩容
    result.object_points.clear();
    result.image_points.clear();
    result.object_points.reserve(static_cast<size_t>(num_charuco));
    result.image_points.reserve(static_cast<size_t>(num_charuco));

    // 从标定板预存数据读取全部三维标准点位
    std::vector<cv::Point3f> all_object_points = charuco_board_.getChessboardCorners();

    // 遍历每个检测到的角点ID，匹配对应三维坐标
    for (int i = 0; i < num_charuco; ++i) {
        const int id = charuco_ids[i];
        // 存入图像二维像素角点
        result.image_points.emplace_back(charuco_corners[static_cast<size_t>(i)]);
        // ID越界非法，直接检测失败
        if (id < 0 || id >= static_cast<int>(all_object_points.size())) {
            result.success = false;
            return std::unexpected("Invalid ChArUco corner id returned by OpenCV");
        }
        // 根据ID匹配对应标定板三维世界坐标
        result.object_points.emplace_back(all_object_points[static_cast<size_t>(id)]);
    }

    // 全部校验通过，标记检测成功
    result.success = true;
    return result;
}

/**
 * @brief 获取标定板标准三维坐标点数组，实现CalibrationBoard虚接口
 * @return 只读引用，预加载的棋盘格三维点位
 */
const std::vector<cv::Point3f>& CharucoDetector::object_points() const noexcept {
    return object_points_;
}

/**
 * @brief 在原图绘制检测到的ChArUco角点与ID数字，可视化调试接口
 * @param image 原始输入图像
 * @param detection detect输出的角点检测结果
 * @return 叠加绿色圆点+蓝色ID文字的新图像，不修改原图
 */
cv::Mat CharucoDetector::draw_corners(
    const cv::Mat& image, const CornerDetection& detection) const noexcept {
    // 拷贝原图用于绘制，不污染输入图像
    cv::Mat output = image.clone();
    // 灰度图转为BGR彩色，保证彩色绘制可见
    if (output.channels() == 1) {
        cv::cvtColor(output, output, cv::COLOR_GRAY2BGR);
    }

    // 检测成功且存在角点才执行绘制
    if (detection.success && !detection.image_points.empty()) {
        // 遍历所有角点
        for (size_t i = 0; i < detection.image_points.size(); ++i) {
            // 绿色实心圆绘制角点，半径5，线条宽度2
            cv::circle(output, detection.image_points[i], 5, cv::Scalar(0, 255, 0), 2);
            // 角点ID文字偏移右上角，蓝色小字
            cv::putText(
                output, std::to_string(i), detection.image_points[i] + cv::Point2f(5, -5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
        }
    }

    return output;
}

#endif // HAVE_OPENCV_ARUCO

// ===================== 标定板工厂函数实现 =====================
/**
 * @brief 工厂创建CalibrationBoard多态检测器实例
 * 根据BoardConfig.type自动匹配对应子类：棋盘格/ChArUco/圆点网格
 * @param board_config 通用标定板尺寸配置
 * @param charuco_config ChArUco专用标记配置（仅ChArUco类型生效）
 * @return expected<unique_ptr<CalibrationBoard>, 错误字符串>
 *         成功：多态检测器独占智能指针；失败：类型不支持/无Aruco模块报错
 */
std::expected<std::unique_ptr<CalibrationBoard>, std::string>
    create_board(const BoardConfig& board_config, const CharucoConfig& charuco_config) noexcept {
    switch (board_config.type) {
    // 分支1：棋盘格检测器，无额外依赖，直接构造
    case BoardType::Chessboard:
        return std::make_unique<ChessboardDetector>(
            board_config.width, board_config.height, board_config.square_size);

    // 分支2：ChArUco检测器，判断是否开启Aruco编译开关
    case BoardType::ChArUco:
#ifdef HAVE_OPENCV_ARUCO
        return std::make_unique<CharucoDetector>(
            board_config.width, board_config.height, board_config.square_size,
            charuco_config.marker_size, charuco_config.dictionary);
#else
        // 编译未启用Aruco模块，返回业务错误
        return std::unexpected("ChArUco support not available (OpenCV aruco module not found)");
#endif

    // 分支3：圆点网格标定板暂未实现，返回错误
    case BoardType::CirclesGrid: return std::unexpected("CirclesGrid not yet implemented");
    }

    // 未知枚举类型兜底错误
    return std::unexpected("Unknown board type");
}

} // namespace fcs::calibration