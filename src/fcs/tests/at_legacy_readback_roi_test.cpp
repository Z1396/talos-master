// Google Test 单元测试框架，提供 TEST 测试宏、ASSERT/EXPECT 断言
#include <gtest/gtest.h>

// 引入 ROI 裁剪框计算工具头文件：实现ROI扩充、边界修正、3D框投影到图像等业务函数
#include "L2_perception/armor/readback_roi.hpp"

// 匿名命名空间：隔离测试代码内部符号，避免多测试文件链接冲突
namespace {

/**
 * @brief 测试1：ROI解析时，强制维持正方形长宽比 + 满足最小输入分辨率
 * 业务背景：装甲推理网络输入固定正方形尺寸，必须保证裁剪ROI是正方形且边长≥网络输入尺寸；同时加边距扩充、限制不越出原图边界
 */
TEST(ReadbackRoi, ResolveMaintainsAspectAndMinimumSize) {
    // 原始图像帧分辨率：宽1440，高1080
    const cv::Size frame_size(1440, 1080);
    // 检测得到的原始装甲矩形框浮点数坐标 (x,y,w,h)
    const cv::Rect2f raw_roi(100.0f, 120.0f, 80.0f, 40.0f);
    // ROI扩充配置结构体
    fcs::L2::ArmorReadbackRoiConfig config;
    // X方向左右扩充比例：基于原始宽度向外扩10%
    config.margin_ratio_x = 0.10;
    // Y方向上下扩充比例：基于原始高度向外扩10%
    config.margin_ratio_y = 0.10;

    // 解析合法ROI
    // 参数说明：原图尺寸、原始检测框、扩充边距配置、推理网络输入尺寸640×640
    const auto resolved =
        fcs::L2::resolve_readback_roi(frame_size, raw_roi, config, {.width = 640, .height = 640});
    // 断言：必须生成有效ROI，不能返回空
    ASSERT_TRUE(resolved.has_value());

    // ROI宽高不能小于网络输入640
    EXPECT_GE(resolved->width, 640);
    EXPECT_GE(resolved->height, 640);
    // 强制正方形，宽高相等
    EXPECT_EQ(resolved->width, resolved->height);
    // ROI左上角坐标不能小于0（不超出图像左、上边界）
    EXPECT_GE(resolved->x, 0);
    EXPECT_GE(resolved->y, 0);
    // ROI右下角不能超出图像右、下边界
    EXPECT_LE(resolved->x + resolved->width, frame_size.width);
    EXPECT_LE(resolved->y + resolved->height, frame_size.height);
}

/**
 * @brief 测试2：原始ROI贴近图像右/下边界时，自动向内偏移修正，保证裁剪框完全落在画面内
 * 场景：装甲目标靠近画面右下角，扩充后的框会超出原图，算法自动左移/上移对齐图像边缘
 */
TEST(ReadbackRoi, ResolveShiftsOutOfBoundsRoiBackIntoFrame) {
    const cv::Size frame_size(1440, 1080);
    // 原始框位置极度靠右、靠下，极易越界
    const cv::Rect2f raw_roi(1300.0f, 900.0f, 120.0f, 120.0f);
    fcs::L2::ArmorReadbackRoiConfig config;
    // 不额外扩充边距，仅测试边界修正逻辑
    config.margin_ratio_x = 0.0;
    config.margin_ratio_y = 0.0;

    // 网络输入尺寸416×416
    const auto resolved =
        fcs::L2::resolve_readback_roi(frame_size, raw_roi, config, {.width = 416, .height = 416});
    ASSERT_TRUE(resolved.has_value());

    // 修正后ROI右边缘刚好贴合图像最右侧
    EXPECT_EQ(resolved->x + resolved->width, frame_size.width);
    // 修正后ROI下边缘刚好贴合图像最底部
    EXPECT_EQ(resolved->y + resolved->height, frame_size.height);
}

/**
 * @brief 测试3：当要求的最小网络输入尺寸大于原图分辨率时，返回std::nullopt（无合法ROI）
 * 场景：原图分辨率过小，不足以放下推理所需最小正方形输入，直接放弃裁剪推理
 */
TEST(ReadbackRoi, ResolveReturnsNulloptWhenMinInputCannotFitFrame) {
    // 原图仅640×480，高度不足以容纳896×672的输入框
    const cv::Size frame_size(640, 480);
    const cv::Rect2f raw_roi(10.0f, 10.0f, 50.0f, 50.0f);
    const auto resolved = fcs::L2::resolve_readback_roi(
        frame_size, raw_roi, fcs::L2::ArmorReadbackRoiConfig{}, {.width = 896, .height = 672});
    // 断言：返回空，无法生成合法ROI
    EXPECT_FALSE(resolved.has_value());
}

/**
 * @brief 测试4：X/Y轴使用独立扩充比例，验证非对称边距扩充计算正确
 * 业务需求：上下扩充幅度和左右扩充幅度可以分开配置，适配装甲狭长外形
 */
TEST(ReadbackRoi, ExpandUsesIndependentMarginsForXAndY) {
    // 原始检测框 x=100 y=200 w=50 h=80
    const cv::Rect2f raw_roi(100.0f, 200.0f, 50.0f, 80.0f);
    fcs::L2::ArmorReadbackRoiConfig config;
    // X左右向外扩原始宽度的20%
    config.margin_ratio_x = 0.20;
    // Y上下向外扩原始高度的50%
    config.margin_ratio_y = 0.50;

    // 执行ROI非对称扩充
    const auto expanded = fcs::L2::expand_raw_roi(raw_roi, config);
    // 校验扩充后坐标与尺寸
    // x = 100 - 50*0.2 = 90
    EXPECT_FLOAT_EQ(expanded.x, 90.0f);
    // y = 200 - 80*0.5 = 160
    EXPECT_FLOAT_EQ(expanded.y, 160.0f);
    // w = 50 + 50*0.2*2 = 70
    EXPECT_FLOAT_EQ(expanded.width, 70.0f);
    // h = 80 + 80*0.5*2 = 160
    EXPECT_FLOAT_EQ(expanded.height, 160.0f);
}

/**
 * @brief 测试5：3D装甲包围盒投影到图像，输出合法、包含目标中心点的2D ROI
 * 流程：世界坐标系3D装甲尺寸 + TF位姿变换 + 相机内参 → 投影得到图像2D矩形框
 */
TEST(ReadbackRoi, ProjectBoxToImageProducesFiniteRoi) {
    fcs::CameraConfig camera_config;
    // 简易相机内参矩阵 fx=100, fy=100, cx=50, cy=60
    camera_config.camera_matrix << 100.0, 0.0, 50.0,
                                   0.0, 100.0, 60.0,
                                   0.0, 0.0, 1.0;

    // 相机光学坐标系相对odom坐标系无平移旋转（单位变换）
    const auto T_camera_odom =
        fast_tf::TransformMatrixd<fast_tf::camera_optical, fast_tf::odom>::from_translation(
            0.0, 0.0, 0.0);
    // 参数：装甲3D中心坐标(0,0,10)、装甲绕Z轴旋转0、相机TF变换、相机标定、装甲半长宽高(0.8,0.8,0.6)
    const auto roi = fcs::L2::project_box_to_image(
        Eigen::Vector3d(0.0, 0.0, 10.0), 0.0, T_camera_odom, camera_config, {0.8, 0.8, 0.6});

    ASSERT_TRUE(roi.has_value());
    // 投影框宽高必须大于0，有效框
    EXPECT_GT(roi->width, 0.0f);
    EXPECT_GT(roi->height, 0.0f);
    // 目标中心点cx=50在ROI水平范围内
    EXPECT_LT(roi->x, 50.0f);
    EXPECT_GT(roi->x + roi->width, 50.0f);
    // 目标中心点cy=60在ROI垂直范围内
    EXPECT_LT(roi->y, 60.0f);
    EXPECT_GT(roi->y + roi->height, 60.0f);
}

} // namespace