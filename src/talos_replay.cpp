// 编译时构建信息头：获取Git分支、提交哈希、编译时间、编译主机等版本信息
#include "runtime/build_info.hpp"
// 离线回放核心逻辑：命令行参数解析、Mcap文件回放执行入口
#include "runtime/replay.hpp"
// spdlog日志全局初始化、标准输出重定向钩子
#include "spdlog_hook.hpp"

// C++标准控制台IO，用于打印错误、帮助文本到stderr
#include <iostream>

/**
 * @brief talos-replay 离线数据回放工具入口主函数
 * 功能：加载录制好的Mcap离线数据包，复现机器人全流程图像、跟踪、调试数据，用于离线复现bug、算法调参
 * @param argc 命令行参数个数
 * @param argv 命令行参数字符串数组
 * @return int 程序退出码：0正常退出，1参数错误/回放失败
 */
int main(int argc, char** argv) {
    // 1. 初始化全局spdlog日志系统（日志级别、输出文件/控制台、格式统一配置）
    init_logger();
    // 钩子绑定：将printf、std::cout/cout底层标准流重定向到spdlog日志输出
    hook_cstream();

    // 2. 获取编译时固化的项目构建版本信息
    const auto build = fcs::build_info();
    // 打印构建信息日志，方便定位版本问题
    SPDLOG_INFO(
        "talos-replay build version={}@{} git={} host={}",
        build.git_branch,    // Git分支名，如main/dev
        build.build_date,    // 编译构建日期时间
        build.git_commit,    // Git完整commit哈希值，精准定位代码快照
        build.build_host);   // 编译机器主机名，区分不同编译环境

    // 3. 解析命令行传入的回放启动参数（回放文件路径、倍速、是否开启可视化、指定话题过滤等）
    const auto options = fcs::runtime::parse_replay_options(argc, argv);
    // 判断参数解析是否失败（std::expected 失败分支）
    if (!options) {
        // 打印错误信息到标准错误流stderr
        std::cerr << options.error() << '\n';
        // 特殊分支：错误文本以usage:开头，代表用户输入-h/--help打印帮助，直接正常退出0
        if (options.error().rfind("usage:", 0) == 0) {
            return 0;
        }
        // 普通参数错误：打印完整使用帮助文档
        std::cerr << fcs::runtime::replay_usage(argc > 0 ? argv[0] : "talos-replay") << '\n';
        // 返回异常退出码1
        return 1;
    }

    // 4. 传入解析完成的参数结构体，启动离线回放主逻辑
    const auto replay = fcs::runtime::run_replay(*options);
    // 回放运行出现内部故障（文件损坏、话题缺失、IO异常等）
    if (!replay) {
        // 严重错误日志打印，终止程序
        SPDLOG_CRITICAL("{}", replay.error());
        return 1;
    }

    // 回放完整正常执行完毕，无报错，返回0正常退出
    return 0;
}