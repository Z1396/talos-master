// GoogleTest 单元测试框架，提供 TEST、ASSERT、EXPECT 断言
#include <gtest/gtest.h>

// 标准库：高精度计时、文件系统、文件读写、字符串
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

// 被测核心模块：运行时配置加载器，实现多层TOML配置合并、校验、解析逻辑
#include "runtime/config_loader.hpp"

// 匿名命名空间：隔离测试工具函数，避免全局符号冲突
namespace {

/**
 * @brief 获取项目源码根目录路径，由编译宏 TALOS_SOURCE_DIR 在编译期注入
 * @return std::filesystem::path 源码根路径
 */
std::filesystem::path source_root() { return std::filesystem::path(TALOS_SOURCE_DIR); }

/**
 * @brief RAII 自动切换工作目录工具类
 * 构造：切换到指定工作目录
 * 析构：自动还原程序原始工作目录
 * 作用：每个测试独立切换临时目录，互不干扰，测试结束自动恢复环境
 */
class ScopedCurrentPath {
public:
    explicit ScopedCurrentPath(const std::filesystem::path& path)
        // 保存进入作用域前的原始工作目录
        : old_path_(std::filesystem::current_path()) {
        std::filesystem::current_path(path);
    }

    ~ScopedCurrentPath() {
        // 离开作用域恢复原始目录，防止污染其他测试
        std::filesystem::current_path(old_path_);
    }

private:
    std::filesystem::path old_path_;
};

/**
 * @brief 写入文本文件，自动递归创建父目录
 * @param path 文件完整路径
 * @param contents 文件TOML文本内容
 */
void write_file(const std::filesystem::path& path, std::string_view contents) {
    // 自动创建多级父文件夹
    std::filesystem::create_directories(path.parent_path());
    // 覆盖写入文件
    std::ofstream(path) << contents;
}

/**
 * @brief 从源码仓库复制配置模板到临时测试目录
 * @param from 源码内相对路径（如 config/vision_base.toml）
 * @param to 临时目录目标路径
 * 逻辑：覆盖写入，自动创建目标父文件夹
 */
void copy_file_from_repo(const std::filesystem::path& from, const std::filesystem::path& to) {
    std::filesystem::create_directories(to.parent_path());
    // 从源码根目录拼接源文件路径，覆盖拷贝
    std::filesystem::copy_file(
        source_root() / from, to, std::filesystem::copy_options::overwrite_existing);
}

/**
 * @brief 创建全局唯一临时测试目录，防止多测试并发目录冲突
 * @param name 测试场景标识字符串
 * @return 临时目录完整路径
 * 原理：使用steady_clock高精度时间戳拼接目录名，保证唯一性
 */
std::filesystem::path make_temp_dir(std::string_view name) {
    const auto unique = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto path   = std::filesystem::temp_directory_path()
                    / ("talos-read-config-" + std::string(name) + "-" + std::to_string(unique));
    std::filesystem::create_directories(path);
    return path;
}

// ===================== 测试用例1：仓库原生启动配置正常加载、多层配置合并覆盖 =====================
/**
 * @brief 场景：直接在项目源码根目录加载标准入口配置 at_vision.toml
 * 业务逻辑：
 * 1. 入口at_vision.toml顶层配置
 * 2. 自动合并 vision_base.toml 基础视觉配置
 * 3. 自动合并对应robot机器人配置、vision细分视觉配置
 * 4. 上层配置键覆盖底层基础配置，下层缺失字段使用默认值
 * 校验点：Daedalus仿真开关、硬件弹道初速、Foxglove可视化全量参数、录制器、弹道模型、传统视觉参数
 */
TEST(ReadConfig, RepositoryLaunchConfigLoadsAndMergesOverrideSemantics) {
    // RAII切换工作目录到源码根目录
    const ScopedCurrentPath cwd(source_root());

    // 加载顶层入口配置文件
    auto result = fcs::load_config("at_vision.toml");
    // 加载不能报错
    ASSERT_TRUE(result.has_value()) << result.error();

    // 移动配置结果，避免拷贝开销
    auto config = std::move(*result);
    // 源码默认开启Daedalus仿真模式
    EXPECT_TRUE(config.launch.daedalus);
    // 硬件配置必须存在
    ASSERT_TRUE(config.hardware.has_value());
    // MCU默认子弹初速24m/s
    EXPECT_DOUBLE_EQ(config.hardware->mcu->bullet_speed_default, 24.0);

    // Foxglove可视化Websocket通道启用校验
    EXPECT_TRUE(config.foxglove.enabled);
    EXPECT_EQ(config.foxglove.transport, fcs::FoxgloveTransport::WebSocket);
    EXPECT_EQ(config.foxglove.host, "0.0.0.0");
    EXPECT_EQ(config.foxglove.port, 8765);
    // MCAP录制路径非空
    EXPECT_FALSE(config.foxglove.mcap_path.empty());
    // 视频编码码率参数
    EXPECT_EQ(config.foxglove.quanta.target_bitrate, 100500);
    EXPECT_FALSE(config.foxglove.quanta.enVBR);

    // x264编码预设、调参、帧内刷新配置校验
    EXPECT_EQ(config.foxglove.quanta.preset, "fast");
    EXPECT_EQ(config.foxglove.quanta.tune, "ssim");
    EXPECT_TRUE(config.foxglove.quanta.intra_refresh);

    // 录制器默认关闭，输出目录默认record
    EXPECT_FALSE(config.capturer.enabled);
    EXPECT_EQ(config.capturer.output_dir, "record");

    // 弹道模型：仿真模式使用理想无阻力模型，重力9.81
    EXPECT_EQ(config.vision.trajectory.model->type, fcs::core::trajectory::model::ModelType::Ideal);
    EXPECT_DOUBLE_EQ(config.vision.trajectory.model->gravity, 9.81);
    // 传统装甲检测启用高级二值化，默认识别蓝色装甲
    EXPECT_TRUE(config.vision.at_legacy->traditional.advanced_binary);
    EXPECT_EQ(config.vision.at_legacy->default_detect_color, fcs::ArmorColor::Blue);
}

// ===================== 测试用例2：入口配置缺失必填robot字段，返回明确错误 =====================
/**
 * @brief 异常场景：at_vision.toml 缺少 robot 关键字，配置加载校验失败
 * 业务规则：入口配置必须指定robot字段，用于匹配对应机器人硬件配置文件
 */
TEST(ReadConfig, MissingRequiredLaunchFieldReturnsEntryConfigError) {
    // 创建临时测试目录
    const auto temp_dir = make_temp_dir("missing-launch-field");
    {
        // 切换到临时目录
        const ScopedCurrentPath cwd(temp_dir);
        // 写入残缺入口配置，无robot键
        std::ofstream(temp_dir / "at_vision.toml") << "daedalus = false\nvision = 'std'\n";

        auto result = fcs::load_config("at_vision.toml");
        // 加载失败
        ASSERT_FALSE(result.has_value());
        // 校验错误提示精准提示缺失robot
        EXPECT_EQ(result.error(), "entry config: Missing key 'robot'");
    }
    // 销毁临时目录，释放磁盘
    std::filesystem::remove_all(temp_dir);
}

// ===================== 测试用例3：非仿真(真实机器人)模式，成功合并硬件+视觉分层配置 =====================
/**
 * @brief 正常业务场景：daedalus=false 实体机器人模式
 * 加载链路：入口at_vision.toml → vision_base.toml → vision/std.toml → robot/hero.toml
 * 校验：关闭仿真、硬件配置正常加载、弹道模型切换为空气阻力线性模型
 */
TEST(ReadConfig, NonDaedalusLaunchReturnsHardwareAndMergedBaseVision) {
    const auto temp_dir = make_temp_dir("non-daedalus-success");
    // 从源码拷贝三层基础配置模板
    copy_file_from_repo("config/vision_base.toml", temp_dir / "config/vision_base.toml");
    copy_file_from_repo("config/robot/hero.toml", temp_dir / "config/robot/hero.toml");
    // 空细分视觉配置文件
    write_file(temp_dir / "config/vision/std.toml", "");
    // 写入真实机器人启动配置，指定hero机器人、std视觉
    write_file(temp_dir / "at_vision.toml", "daedalus = false\nrobot = 'hero'\nvision = 'std'\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_TRUE(result.has_value()) << result.error();

        auto config = std::move(*result);
        // 非仿真模式
        EXPECT_FALSE(config.launch.daedalus);
        // 硬件配置正常加载
        ASSERT_TRUE(config.hardware.has_value());
        // Foxglove默认开启WebSocket
        EXPECT_TRUE(config.foxglove.enabled);
        EXPECT_EQ(config.foxglove.transport, fcs::FoxgloveTransport::WebSocket);
        // 实体机器人使用带空气阻力线性弹道模型，和仿真Ideal模型区分
        EXPECT_EQ(
            config.vision.trajectory.model->type,
            fcs::core::trajectory::model::ModelType::LinearDrag);
    }

    std::filesystem::remove_all(temp_dir);
}

// ===================== 测试用例4：录制器开启但未配置输出目录，校验报错 =====================
/**
 * @brief 业务约束：capturer.enabled=true 时 output_dir 必须提供，防止录制无存储路径
 */
TEST(ReadConfig, CapturerRequiresOutputDirWhenEnabled) {
    const auto temp_dir = make_temp_dir("capturer-missing-output-dir");
    copy_file_from_repo("config/vision_base.toml", temp_dir / "config/vision_base.toml");
    copy_file_from_repo("config/robot/hero.toml", temp_dir / "config/robot/hero.toml");
    write_file(temp_dir / "config/vision/std.toml", "");
    // 开启录制器，但不写output_dir字段
    write_file(
        temp_dir / "at_vision.toml",
        "daedalus = false\nrobot = 'hero'\nvision = 'std'\n[capturer]\nenabled = true\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_FALSE(result.has_value());
        // 错误提示明确告知缺少输出目录
        EXPECT_EQ(
            result.error(),
            "entry config: capturer.output_dir is required when capturer.enabled=true");
    }

    std::filesystem::remove_all(temp_dir);
}

// ===================== 测试用例5：Foxglove传输模式设为Mcap文件录制，缺失mcap_path报错 =====================
/**
 * @brief 业务约束：transport="Mcap" 文件录制模式必须指定存储文件路径
 */
TEST(ReadConfig, McapTransportRequiresPath) {
    const auto temp_dir = make_temp_dir("mcap-missing-path");
    copy_file_from_repo("config/vision_base.toml", temp_dir / "config/vision_base.toml");
    copy_file_from_repo("config/robot/hero.toml", temp_dir / "config/robot/hero.toml");
    write_file(temp_dir / "config/vision/std.toml", "");
    // 指定Mcap传输，但不配置mcap_path
    write_file(
        temp_dir / "at_vision.toml",
        "daedalus = false\nrobot = 'hero'\nvision = 'std'\n[foxglove]\ntransport = 'Mcap'\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(
            result.error(),
            "entry config: foxglove.mcap_path is required when foxglove.transport=\"Mcap\"");
    }

    std::filesystem::remove_all(temp_dir);
}

// ===================== 测试用例6：Mcap模式完整配置路径，正常解析加载 =====================
/**
 * @brief 正常场景：transport=Mcap 且配置mcap_path，所有Foxglove参数正常合并
 */
TEST(ReadConfig, McapTransportParsesWhenPathProvided) {
    const auto temp_dir = make_temp_dir("mcap-success");
    copy_file_from_repo("config/vision_base.toml", temp_dir / "config/vision_base.toml");
    copy_file_from_repo("config/robot/hero.toml", temp_dir / "config/robot/hero.toml");
    write_file(temp_dir / "config/vision/std.toml", "");
    // 完整Mcap配置，提供输出路径
    write_file(
        temp_dir / "at_vision.toml", "daedalus = false\n"
                                     "robot = 'hero'\n"
                                     "vision = 'std'\n"
                                     "[foxglove]\n"
                                     "transport = 'Mcap'\n"
                                     "mcap_path = 'logs/out.mcap'\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_TRUE(result.has_value()) << result.error();

        auto config = std::move(*result);
        // 传输模式正确识别为Mcap文件录制
        EXPECT_EQ(config.foxglove.transport, fcs::FoxgloveTransport::Mcap);
        EXPECT_EQ(config.foxglove.mcap_path, "logs/out.mcap");
        // WebSocket端口、主机继承基础配置默认值
        EXPECT_EQ(config.foxglove.port, 8765);
        EXPECT_EQ(config.foxglove.host, "0.0.0.0");
    }

    std::filesystem::remove_all(temp_dir);
}

// ===================== 测试用例7：自定义Quanta视频编码参数，上层配置覆盖基础默认值 =====================
/**
 * @brief 验证配置覆盖语义：入口文件中 [foxglove.quanta] 自定义参数覆盖vision_base默认编码参数
 * 校验全部视频编码自定义字段：码率、VBR、预设、lookahead等
 */
TEST(ReadConfig, QuantaOverridesParseWhenProvided) {
    const auto temp_dir = make_temp_dir("quanta-success");
    copy_file_from_repo("config/vision_base.toml", temp_dir / "config/vision_base.toml");
    copy_file_from_repo("config/robot/hero.toml", temp_dir / "config/robot/hero.toml");
    write_file(temp_dir / "config/vision/std.toml", "");
    // 自定义完整x264编码参数
    write_file(
        temp_dir / "at_vision.toml", "daedalus = false\n"
                                     "robot = 'hero'\n"
                                     "vision = 'std'\n"
                                     "[foxglove]\n"
                                     "transport = 'WebSocket'\n"
                                     "[foxglove.quanta]\n"
                                     "target_bitrate = 64000\n"
                                     "enVBR = true\n"
                                     "min_bit_rate = 32000\n"
                                     "max_bit_rate = 96000\n"
                                     "preset = 'medium'\n"
                                     "tune = 'fastdecode'\n"
                                     "intra_refresh = true\n"
                                     "lookahead = 8\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_TRUE(result.has_value()) << result.error();

        auto config = std::move(*result);
        // 全部自定义编码参数生效，覆盖基础配置默认值
        EXPECT_EQ(config.foxglove.quanta.target_bitrate, 64000);
        EXPECT_TRUE(config.foxglove.quanta.enVBR);
        EXPECT_EQ(config.foxglove.quanta.min_bit_rate, 32000);
        EXPECT_EQ(config.foxglove.quanta.max_bit_rate, 96000);
        EXPECT_EQ(config.foxglove.quanta.preset, "medium");
        EXPECT_EQ(config.foxglove.quanta.tune, "fastdecode");
        EXPECT_TRUE(config.foxglove.quanta.intra_refresh);
        EXPECT_EQ(config.foxglove.quanta.lookahead, 8);
    }

    std::filesystem::remove_all(temp_dir);
}

// ===================== 测试用例8：基础视觉配置存在非法值，加载抛出视觉配置错误 =====================
/**
 * @brief 异常校验：vision_base.toml 字段类型非法（数组赋值给标量backend_type）
 * 错误提示携带"vision config:"前缀，定位视觉分层配置错误
 */
TEST(ReadConfig, InvalidMergedVisionConfigIsReported) {
    const auto temp_dir = make_temp_dir("invalid-vision-config");
    // 写入非法视觉基础配置，backend_type赋值数组[1]，类型不匹配
    write_file(temp_dir / "config/vision_base.toml", "backend_type = [1]\n");
    write_file(temp_dir / "config/vision/std.toml", "");
    write_file(temp_dir / "at_vision.toml", "daedalus = true\nrobot = 'hero'\nvision = 'std'\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_FALSE(result.has_value());
        const auto& error = result.error();
        // 错误信息包含分层标识、非法字段名
        EXPECT_NE(error.find("vision config:"), std::string::npos);
        EXPECT_NE(error.find("backend_type"), std::string::npos);
    }

    std::filesystem::remove_all(temp_dir);
}

// ===================== 测试用例9：机器人硬件配置文件缺失必填camera键，报错 =====================
/**
 * @brief 异常场景：robot/hero.toml 空文件，缺少camera硬件标定配置
 * 错误提示携带 hardware config 分层标识，精准定位硬件配置缺失字段
 */
TEST(ReadConfig, InvalidHardwareConfigIsReported) {
    const auto temp_dir = make_temp_dir("invalid-hardware-config");
    copy_file_from_repo("config/vision_base.toml", temp_dir / "config/vision_base.toml");
    write_file(temp_dir / "config/vision/std.toml", "");
    // 空机器人硬件配置文件，无camera字段
    write_file(temp_dir / "config/robot/hero.toml", "");
    write_file(temp_dir / "at_vision.toml", "daedalus = false\nrobot = 'hero'\nvision = 'std'\n");

    {
        const ScopedCurrentPath cwd(temp_dir);
        auto result = fcs::load_config("at_vision.toml");
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error(), "hardware config: Missing key 'camera'");
    }

    std::filesystem::remove_all(temp_dir);
}

} // 匿名命名空间结束