// ===========================================================================
// 练习5：综合演示 - concept + CRTP + std::expected
// 目标：实现一个数值处理流水线，综合运用三大现代特性
// 模拟 Talos 项目的检测器后端抽象设计
// ===========================================================================

#include <expected>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>
#include <concepts>

// ===========================================================================
// 一、Concept：约束数值类型
// ===========================================================================

// 数值概念：整数或浮点
template <typename T>
concept Numeric = std::integral<T> || std::floating_point<T>;

// 可序列化概念：必须有 to_string 成员
template <typename T>
concept Stringifiable = requires(const T& t) {
    { t.to_string() } -> std::convertible_to<std::string>;
};

// ===========================================================================
// 二、std::expected：错误处理
// ===========================================================================

enum class PipelineError {
    EmptyInput,
    NoMatch,
    InvalidConfig,
};

std::string_view to_string_view(PipelineError e) {
    switch (e) {
        case PipelineError::EmptyInput:   return "empty input";
        case PipelineError::NoMatch:      return "no match";
        case PipelineError::InvalidConfig:return "invalid config";
    }
    return "unknown";
}

// ===========================================================================
// 三、CRTP：检测器后端抽象（模拟 Talos armor/backends/base.hpp）
// ===========================================================================

// 检测结果
struct Detection {
    float confidence;
    std::string label;

    std::string to_string() const {
        return label + " (" + std::to_string(confidence) + ")";
    }
};

// CRTP 基类：定义检测器统一接口
template <typename Derived>
class DetectorBase {
public:
    // 统一检测接口：转发到派生类 detect_impl
    [[nodiscard]] std::expected<std::vector<Detection>, PipelineError>
    detect(const std::vector<float>& input) const noexcept {
        if (input.empty()) {
            return std::unexpected(PipelineError::EmptyInput);
        }
        const auto& derived = static_cast<const Derived&>(*this);
        return derived.detect_impl(input);
    }

    // 模板方法：后处理（去低置信度）
    [[nodiscard]] std::expected<std::vector<Detection>, PipelineError>
    detect_filtered(const std::vector<float>& input, float threshold) const noexcept {
        auto result = detect(input);
        if (!result) {
            return std::unexpected(result.error());
        }
        std::vector<Detection> filtered;
        for (const auto& d : *result) {
            if (d.confidence >= threshold) {
                filtered.push_back(d);
            }
        }
        return filtered;
    }
};

// ---------------------------------------------------------------------------
// 派生后端1：阈值检测器（模拟 Traditional 后端）
// ---------------------------------------------------------------------------
class ThresholdDetector : public DetectorBase<ThresholdDetector> {
public:
    explicit ThresholdDetector(float threshold) : threshold_(threshold) {}

    friend class DetectorBase<ThresholdDetector>;

    [[nodiscard]] std::expected<std::vector<Detection>, PipelineError>
    detect_impl(const std::vector<float>& input) const noexcept {
        std::vector<Detection> results;
        for (std::size_t i = 0; i < input.size(); ++i) {
            if (input[i] > threshold_) {
                results.push_back({
                    input[i],
                    "object_" + std::to_string(i)
                });
            }
        }
        if (results.empty()) {
            return std::unexpected(PipelineError::NoMatch);
        }
        return results;
    }

private:
    float threshold_;
};

// ---------------------------------------------------------------------------
// 派生后端2：峰值检测器（模拟 ONNX 后端）
// ---------------------------------------------------------------------------
class PeakDetector : public DetectorBase<PeakDetector> {
public:
    friend class DetectorBase<PeakDetector>;

    [[nodiscard]] std::expected<std::vector<Detection>, PipelineError>
    detect_impl(const std::vector<float>& input) const noexcept {
        std::vector<Detection> results;
        // 比左右邻居都大的点为峰值
        for (std::size_t i = 1; i + 1 < input.size(); ++i) {
            if (input[i] > input[i - 1] && input[i] > input[i + 1]) {
                results.push_back({input[i], "peak_" + std::to_string(i)});
            }
        }
        if (results.empty()) {
            return std::unexpected(PipelineError::NoMatch);
        }
        return results;
    }
};

// ===========================================================================
// 四、Concept 约束通用函数：Stringifiable 类型的打印
// ===========================================================================
template <Stringifiable T>
void print_results(const std::string& title, const std::vector<T>& items) {
    std::cout << title << ":\n";
    for (const auto& item : items) {
        std::cout << "  " << item.to_string() << "\n";
    }
}

// ===========================================================================
// 五、主程序：模拟检测流水线
// ===========================================================================
int main() {
    // 输入信号
    std::vector<float> signal = {0.1f, 0.8f, 0.3f, 0.9f, 0.2f, 0.7f, 0.4f};

    // 后端1：阈值检测
    ThresholdDetector thresh(0.5f);
    auto result1 = thresh.detect_filtered(signal, 0.6f);
    if (result1) {
        print_results("ThresholdDetector 结果", *result1);
    } else {
        std::cout << "ThresholdDetector 失败: " << to_string_view(result1.error()) << "\n";
    }

    std::cout << "\n";

    // 后端2：峰值检测
    PeakDetector peak;
    auto result2 = peak.detect(signal);
    if (result2) {
        print_results("PeakDetector 结果", *result2);
    } else {
        std::cout << "PeakDetector 失败: " << to_string_view(result2.error()) << "\n";
    }

    // 错误传播演示：空输入
    std::cout << "\n";
    std::vector<float> empty;
    auto result3 = thresh.detect(empty);
    if (result3) {
        print_results("空输入结果", *result3);
    } else {
        std::cout << "空输入失败（预期）: " << to_string_view(result3.error()) << "\n";
    }

    std::cout << "\n综合演示完成\n";
    return 0;
}
