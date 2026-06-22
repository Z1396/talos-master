// GoogleTest 单元测试框架头文件，提供 ASSERT_* / EXPECT_* 系列断言宏
#include <gtest/gtest.h>

// C++标准库容器：定长数组，用于存放Eigen矩阵预期数值
#include <array>
// C++23 标准预期类型 std::expected<T, E>，用于函数返回成功值/错误字符串
#include <expected>
// C++17 文件系统库，处理配置文件路径、文件读取
#include <filesystem>
// fmt 格式化库，替代std::stringstream，高性能字符串格式化
#include <fmt/format.h>
// 标准字符串，存储错误信息、文件路径文本
#include <string>

// 项目全局配置头文件，定义编译期宏 TALOS_SOURCE_DIR
#include "config.hpp"
// 弹道求解器核心配置、模型、求解器工厂相关定义
#include "core/trajectory/resource.hpp"
// 运行时L1/L2硬件初始化函数声明（MCU、相机、世界实例初始化逻辑）
#include "runtime/l1_l2_setup.hpp"
// 任务调度器：全局资源、机器人任务时序管理类
#include "scheduler/scheduler.hpp"
// TOML配置解析工具封装，提供toml表转业务配置结构体的通用方法
#include "toml_helper.hpp"

// 匿名命名空间：隔离测试内部工具函数，防止全局符号冲突
namespace {

/**
 * @brief 获取项目源码根目录路径
 * @return std::filesystem::path 源码根目录绝对路径
 * @note TALOS_SOURCE_DIR 为CMake编译时注入的宏，指向项目根目录
 */
std::filesystem::path source_root() {
    return std::filesystem::path(TALOS_SOURCE_DIR);
}

/**
 * @brief 通用TOML配置文件解析模板函数
 * @tparam T 目标业务配置结构体类型（如fcs::HardwareConfig / fcs::VisionConfig）
 * @param path TOML配置文件磁盘路径
 * @return std::expected<T, std::string>
 *         - 成功：包含解析完成的T类型配置对象
 *         - 失败：unexpected携带带文件路径的错误描述字符串
 * @details 分两步解析：
 *          1. toml::parse_file 读取磁盘文件并语法校验
 *          2. toml_helper::from_table 将原始toml表映射为业务结构体
 */
template <typename T>
std::expected<T, std::string> parse_config_file(const std::filesystem::path& path) {
    // 1. 读取并解析TOML文件，返回toml::parse_result（包含table或error）
    auto parsed = toml::parse_file(path.string());
    // 文件读取/语法解析失败判断
    if (!parsed) {
        // 拼接错误信息：文件路径 + toml库原生错误描述，返回错误分支
        return std::unexpected(path.string() + ": " + std::string(parsed.error().description()));
    }

    // 2. 将原始toml根table映射为目标业务配置结构体T
    auto config = toml_helper::from_table<T>(parsed.table());
    // 结构体字段映射失败（缺字段、类型不匹配）
    if (!config) {
        // 返回带文件路径的映射错误
        return std::unexpected(path.string() + ": " + config.error());
    }
    // 解析全部成功，返回配置对象
    return *config;
}

/**
 * @brief Eigen矩阵数值校验通用工具函数
 * @tparam Derived Eigen矩阵表达式派生类（兼容Matrix/Vector）
 * @tparam N 预期数值数组长度（编译期定长）
 * @param actual 待校验的Eigen矩阵/向量
 * @param expected 预期double数值定长数组，按内存展开顺序排列
 * @details
 *  1. 先强断言元素总数必须相等，不等直接终止当前TEST
 *  2. 逐元素浮点等值校验，误差0，失败打印数组下标辅助定位
 *  EXPECT_DOUBLE_EQ：浮点相等校验，轻微浮点误差会判定失败
 */
template <typename Derived, std::size_t N>
void expect_matrix_values(
    const Eigen::MatrixBase<Derived>& actual, const std::array<double, N>& expected) {
    // 强断言：矩阵总元素数量必须和预期数组长度一致，失败直接中断测试用例
    ASSERT_EQ(static_cast<std::size_t>(actual.size()), expected.size());
    // 遍历矩阵所有元素（Eigen按列优先内存排布）
    for (Eigen::Index i = 0; i < actual.size(); ++i) {
        // 逐值对比，失败附加下标日志，方便定位哪个像素/标定参数出错
        EXPECT_DOUBLE_EQ(actual(i), expected[static_cast<std::size_t>(i)]) << "index=" << i;
    }
}

/**
 * @brief 测试用例：Hero英雄机器人硬件TOML配置完整语义与数值校验
 *  作用：校验 config/robot/hero.toml 所有硬件标定参数是否和标定文档一致
 *  校验范围：相机内参、畸变系数、相机采集参数、手眼外参、MCU电控参数
 */
TEST(ConfigParse, HeroRobotTomlMatchesExpectedSemanticsAndValues) {
    // 解析英雄机器人硬件配置文件
    auto result = parse_config_file<fcs::HardwareConfig>(source_root() / "config/robot/hero.toml");
    // 强断言文件解析不能失败，失败打印完整错误信息并终止该用例
    ASSERT_TRUE(result.has_value()) << result.error();
    // 提取解析成功的硬件配置常量引用
    const auto& config = *result;

    // ========== 相机内参矩阵3x3校验（9个参数，按列展开） ==========
    expect_matrix_values(
        config.camera->camera_matrix,
        std::array<double, 9>{
            1784.40785518, 0.0, 709.26908080,
            0.0, 1784.39799730, 556.61031728,
            0.0, 0.0, 1.0});
    // ========== 相机5阶畸变系数校验 k1,k2,p1,p2,k3 ==========
    expect_matrix_values(
        config.camera->distort_coefficient,
        std::array<double, 5>{-0.05413741, 0.13077699, -0.00008913, 0.00029836, -0.05802791});
    // 图像分辨率宽高
    EXPECT_EQ(config.camera->width, 1440u);
    EXPECT_EQ(config.camera->height, 1080u);

    // ========== 相机采集参数profile校验 ==========
    EXPECT_FALSE(config.camera->profile.trigger_mode);    // 不使用硬件外触发
    EXPECT_FALSE(config.camera->profile.invert_image);    // 图像不做镜像翻转
    EXPECT_EQ(config.camera->profile.exposure_time_us, 5000u); // 曝光5000微秒
    EXPECT_DOUBLE_EQ(config.camera->profile.gain, 16.7);   // 模拟增益16.7倍
    EXPECT_EQ(config.camera->profile.rotate_angle, fcs::RotateType::None); // 图像无旋转

    // ========== 云台手眼外参平移向量校验 ==========
    // 云台yaw坐标系到pitch坐标系平移量
    expect_matrix_values(
        config.extrinsic->gimbal_yaw.gimbal_pitch.translation,
        std::array<double, 3>{-0.06492, 0.0, 0.164});
    // pitch坐标系到相机link平移
    expect_matrix_values(
        config.extrinsic->gimbal_yaw.gimbal_pitch.camera_link.translation,
        std::array<double, 3>{0.2403, 0.0, -0.0547});
    // pitch坐标系到炮管muzzle枪口link平移
    expect_matrix_values(
        config.extrinsic->gimbal_yaw.gimbal_pitch.muzzle_link.translation,
        std::array<double, 3>{0.1598, 0.0, 0.0});
    // 枪口link rpy三轴旋转角度
    EXPECT_DOUBLE_EQ(config.extrinsic->gimbal_yaw.gimbal_pitch.muzzle_link.roll, 0.0);
    EXPECT_DOUBLE_EQ(config.extrinsic->gimbal_yaw.gimbal_pitch.muzzle_link.pitch, 0.8);
    EXPECT_DOUBLE_EQ(config.extrinsic->gimbal_yaw.gimbal_pitch.muzzle_link.yaw, -0.3);

    // ========== MCU电控USB与弹速参数校验 ==========
    EXPECT_EQ(config.mcu->mcu_vendor_id, 0x0483); // STM32 USB厂商VID
    EXPECT_FALSE(config.mcu->mcu_product_id.has_value()); // PID配置项未填写（std::optional无值）
    EXPECT_TRUE(config.mcu->mcu_authoritative_self_color); // MCU作为己方颜色权威源
    EXPECT_TRUE(config.mcu->mcu_authoritative_bullet_speed); // MCU下发弹速为权威值
    EXPECT_DOUBLE_EQ(config.mcu->bullet_speed_default, 11.0); // 默认弹速11m/s
    EXPECT_DOUBLE_EQ(config.mcu->bullet_speed_min, 10.0);    // 最小弹速下限
    EXPECT_DOUBLE_EQ(config.mcu->bullet_speed_max, 20.0);    // 最大弹速上限
}

/**
 * @brief 测试用例：配置内置辅助函数、fmt格式化输出、追踪器/滤波/开火决策工具校验
 *  校验：枚举格式化、旋转矩阵RPY解算、追踪器指针转发、图像量化滤波、开火判定函数
 */
TEST(ConfigParse, InlineHelpersAndFormattersBehaveAsExpected) {
    // 读取英雄硬件配置
    auto hardware_result =
        parse_config_file<fcs::HardwareConfig>(source_root() / "config/robot/hero.toml");
    ASSERT_TRUE(hardware_result.has_value()) << hardware_result.error();
    const auto& hardware = *hardware_result;

    // 读取视觉基础公共配置 vision_base.toml（所有机器人共用视觉参数）
    auto vision_result =
        parse_config_file<fcs::VisionConfig>(source_root() / "config/vision_base.toml");
    ASSERT_TRUE(vision_result.has_value()) << vision_result.error();
    const auto& vision = *vision_result;

    // 校验RotateType枚举自定义fmt格式化输出，None打印字符串"None"
    EXPECT_EQ(fmt::format("{}", hardware.camera->profile.rotate_angle), "None");

    // 调用muzzle_link内置rotation()生成旋转对象，解算RPY欧拉角
    const auto muzzle_rotation = hardware.extrinsic->gimbal_yaw.gimbal_pitch.muzzle_link.rotation();
    // 结构化绑定提取roll/pitch/yaw三轴角度
    const auto [roll, pitch, yaw] = muzzle_rotation.rpy();
    EXPECT_DOUBLE_EQ(roll, 0.0);
    EXPECT_DOUBLE_EQ(pitch, 0.8);
    EXPECT_DOUBLE_EQ(yaw, -0.3);

    // l3层级追踪器指针转发接口校验 tracker_ptr()
    const auto tracker_ptr = vision.l3->tracker_ptr();
    ASSERT_TRUE(tracker_ptr); // 指针不能为空，强断言失败终止用例
    // 校验指针内匹配阈值与顶层vision.tracker()原值一致（包装转发逻辑校验）
    EXPECT_DOUBLE_EQ(tracker_ptr->robot.matcher_gate, vision.tracker().robot.matcher_gate);
    // 前哨目标卡尔曼滤波yaw日志系数校验
    EXPECT_DOUBLE_EQ(tracker_ptr->outpost.model.yaw_log_k, 0.005);

    // ========== 图像量化降噪滤波模块完整参数校验 ==========
    EXPECT_TRUE(vision.quanta_filter.enable_denoise_luma); // 开启亮度降噪
    EXPECT_EQ(vision.quanta_filter.denoise_luma.kernel_size, 5); // 5x5高斯核
    EXPECT_DOUBLE_EQ(vision.quanta_filter.denoise_luma.sigma_x, 1.0);
    EXPECT_DOUBLE_EQ(vision.quanta_filter.denoise_luma.sigma_y, 1.0);
    EXPECT_TRUE(vision.quanta_filter.enable_denoise_chroma); // 开启色度降噪
    EXPECT_EQ(vision.quanta_filter.denoise_chroma.kernel_size, 3); // 3x3色度核
    EXPECT_DOUBLE_EQ(vision.quanta_filter.denoise_chroma.sigma_x, 1.0);
    EXPECT_DOUBLE_EQ(vision.quanta_filter.denoise_chroma.sigma_y, 1.0);
    EXPECT_TRUE(vision.quanta_filter.enable_luma_quantization); // 亮度量化压缩开启
    EXPECT_EQ(vision.quanta_filter.luma_levels, 16); // 亮度量化16阶

    // ========== 视频码率控制参数校验 ==========
    EXPECT_FALSE(vision.quanta.enVBR); // 关闭可变码率VBR，使用CBR恒定码率
    EXPECT_EQ(vision.quanta.target_bitrate, 45000); // 目标码率45000 kbps
    EXPECT_EQ(vision.quanta.min_bit_rate, 0);       // 码率下限无限制
    EXPECT_EQ(vision.quanta.max_bit_rate, 50000);   // 码率上限50000 kbps

    // ========== L5层级开火判定工具函数校验 is_on_target ==========
    // 误差极小，满足开火条件 fire=true
    EXPECT_TRUE(
        fcs::L5::is_on_target(vision.l5->fire_decision, 0.0, 0.0, 0.001, 0.001, 100.0).fire);
    // 误差超过阈值，禁止开火 fire=false
    EXPECT_FALSE(fcs::L5::is_on_target(vision.l5->fire_decision, 0.0, 0.0, 0.2, 0.2, 100.0).fire);
}

/**
 * @brief 测试用例：配置可变访问器、弹道求解器工厂创建逻辑校验
 *  校验：
 *  1. 多层级mutable可变引用转发（顶层修改同步到底层l3/l5包装对象）
 *  2. 两种弹道模型（理想质点/线性空气阻力）求解器工厂正常实例化
 *  3. Scheduler调度器无参构造合法性
 */
TEST(ConfigParse, MutableAccessorsAndTrajectoryHelpersUseWrappedConfigFields) {
    // 读取视觉公共配置
    auto vision_result =
        parse_config_file<fcs::VisionConfig>(source_root() / "config/vision_base.toml");
    ASSERT_TRUE(vision_result.has_value()) << vision_result.error();
    // 非const可变配置对象，用于测试mutable修改转发
    auto vision = *vision_result;

    // ========== 可变访问器转发校验 ==========
    // 通过顶层tracker()可变引用修改匹配阈值
    vision.tracker().robot.matcher_gate = 12.0;
    // 底层l3包装tracker同步被修改，证明mutable包装转发逻辑正确
    EXPECT_DOUBLE_EQ(vision.l3->tracker.robot.matcher_gate, 12.0);

    // 顶层weapon可变引用修改调试开关
    vision.weapon().enable_debug = false;
    // l5层级mpc武器模块同步变更
    EXPECT_FALSE(vision.l5->mpc_weapon.enable_debug);

    // ========== 弹道求解器工厂测试：理想无空气阻力模型 ==========
    fcs::core::trajectory::TrajectoryConfig ideal{};
    ideal.model->type    = fcs::core::trajectory::model::ModelType::Ideal; // 理想质点模型
    ideal.model->gravity = 9.81;                                         // 重力加速度9.81
    // 工厂函数创建对应弹道求解器实例
    auto ideal_solver    = fcs::core::trajectory::solver::create_solver(ideal);
    ASSERT_NE(ideal_solver, nullptr); // 求解器指针不能为空，工厂创建成功

    // ========== 弹道求解器工厂测试：线性空气阻力模型 ==========
    fcs::core::trajectory::TrajectoryConfig linear{};
    linear.model->type       = fcs::core::trajectory::model::ModelType::LinearDrag; // 线性阻力模型
    linear.model->gravity    = 9.79;    // 本地重力修正值
    linear.model->resistance = 0.001;  // 空气阻力系数
    auto linear_solver       = fcs::core::trajectory::solver::create_solver(linear);
    ASSERT_NE(linear_solver, nullptr); // 阻力模型求解器正常构造

    // 测试调度器无参构造函数编译运行合法性
    talos::Scheduler scheduler;
}

/**
 * @brief 测试用例：L1底层硬件初始化setup_l1函数分支覆盖测试
 *  目标：覆盖MCU USB设备两种分支（PID为空 / PID赋值），在无真实硬件时验证错误分支
 *  逻辑：无真实MCU设备时，无论PID是否配置，setup_l1均返回MCU连接失败错误，保证分支全量覆盖
 */
TEST(ConfigParse, SetupL1CoversUsbMcuWrapperBranchesBeforeStubCameraFails) {
    // 加载英雄机器人硬件配置
    auto hardware_result =
        parse_config_file<fcs::HardwareConfig>(source_root() / "config/robot/hero.toml");
    ASSERT_TRUE(hardware_result.has_value()) << hardware_result.error();
    // 可变硬件配置，用于修改PID字段测试不同分支
    auto hardware = *hardware_result;

    // 分支1：原始配置 mcu_product_id 无值(std::optional空)
    {
        // 构造全局世界资源容器World
        talos::World world;
        // World绑定调度器实例
        talos::Scheduler scheduler(world);
        // 执行L1底层硬件初始化：无硬件仿真环境，预期返回错误
        auto result = fcs::runtime::setup_l1(world, scheduler, false, false, &hardware);
        // 强断言初始化一定失败
        ASSERT_FALSE(result.has_value());
        // 错误信息必须包含MCU连接失败关键字，证明进入MCU连接失败分支
        EXPECT_NE(result.error().find("failed connecting to mcu"), std::string::npos);
    }

    // 分支2：手动给mcu_product_id赋值，覆盖PID存在的USB匹配分支
    hardware.mcu->mcu_product_id = 0x1234;
    {
        talos::World world;
        talos::Scheduler scheduler(world);
        auto result = fcs::runtime::setup_l1(world, scheduler, false, false, &hardware);
        ASSERT_FALSE(result.has_value());
        // 赋值PID后依旧报MCU连接失败，覆盖USB设备匹配全部分支
        EXPECT_NE(result.error().find("failed connecting to mcu"), std::string::npos);
    }
}

} // namespace 匿名命名空间结束