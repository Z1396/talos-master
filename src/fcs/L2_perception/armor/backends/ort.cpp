#include "L2_perception/armor/backends/ort.hpp"

#include "L2_perception/armor/config.hpp"
#include "L2_perception/common/geometry.hpp"
#include "core/types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <numeric>
#include <onnxruntime_c_api.h>
#include <spdlog/common.h>
#include <spdlog/spdlog-inl.h>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#ifdef __APPLE__
# include <Availability.h>
# include <coreml_provider_factory.h>
#endif

#include <onnxruntime_cxx_api.h>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace fcs::L2 {

// ============================================================================
// Class-to-Color/Number Mapping (64 classes)
// ============================================================================
// 64 classes = 4 colors × 2 sizes × 8 numbers
// Color encoding: 0=Red, 1=Blue, 2=None (Gray), 3=Purple
// Size encoding: 0-7=Small, 8-15=Large, ...
// Number encoding: mod 8 (0=Sentry, 1-5=Numbers, 6=Outpost, 7=Base)

namespace {

struct ClassMapping {
    ArmorColor color;
    ArmorName name;
};

#if defined(__linux__) && defined(__x86_64__)
[[nodiscard]] bool provider_available(
    const std::vector<std::string>& providers, std::string_view provider_name) noexcept {
    return std::any_of(providers.begin(), providers.end(), [&](const std::string& provider) {
        return provider == provider_name;
    });
}

[[nodiscard]] std::string pick_openvino_device_type() noexcept {
    // Prefer FP16 on Intel Xe-class iGPU. Fall back to FP32 if session creation rejects it.
    return "GPU_FP16";
}
#endif

constexpr std::array<ClassMapping, 64> g_class_mappings = {
    {
     // Red, Small (0-7)
        {ArmorColor::Red, ArmorName::Sentry},
     {ArmorColor::Red, ArmorName::One},
     {ArmorColor::Red, ArmorName::Two},
     {ArmorColor::Red, ArmorName::Three},
     {ArmorColor::Red, ArmorName::Four},
     {ArmorColor::Red, ArmorName::Five},
     {ArmorColor::Red, ArmorName::Outpost},
     {ArmorColor::Red, ArmorName::Base},
     // Red, Large (8-15)
        {ArmorColor::Red, ArmorName::Sentry},
     {ArmorColor::Red, ArmorName::One},
     {ArmorColor::Red, ArmorName::Two},
     {ArmorColor::Red, ArmorName::Three},
     {ArmorColor::Red, ArmorName::Four},
     {ArmorColor::Red, ArmorName::Five},
     {ArmorColor::Red, ArmorName::Outpost},
     {ArmorColor::Red, ArmorName::Base},
     // Blue, Small (16-23)
        {ArmorColor::Blue, ArmorName::Sentry},
     {ArmorColor::Blue, ArmorName::One},
     {ArmorColor::Blue, ArmorName::Two},
     {ArmorColor::Blue, ArmorName::Three},
     {ArmorColor::Blue, ArmorName::Four},
     {ArmorColor::Blue, ArmorName::Five},
     {ArmorColor::Blue, ArmorName::Outpost},
     {ArmorColor::Blue, ArmorName::Base},
     // Blue, Large (24-31)
        {ArmorColor::Blue, ArmorName::Sentry},
     {ArmorColor::Blue, ArmorName::One},
     {ArmorColor::Blue, ArmorName::Two},
     {ArmorColor::Blue, ArmorName::Three},
     {ArmorColor::Blue, ArmorName::Four},
     {ArmorColor::Blue, ArmorName::Five},
     {ArmorColor::Blue, ArmorName::Outpost},
     {ArmorColor::Blue, ArmorName::Base},
     // None (Gray), Small (32-39)
        {ArmorColor::Neutral, ArmorName::Sentry},
     {ArmorColor::Neutral, ArmorName::One},
     {ArmorColor::Neutral, ArmorName::Two},
     {ArmorColor::Neutral, ArmorName::Three},
     {ArmorColor::Neutral, ArmorName::Four},
     {ArmorColor::Neutral, ArmorName::Five},
     {ArmorColor::Neutral, ArmorName::Outpost},
     {ArmorColor::Neutral, ArmorName::Base},
     // None (Gray), Large (40-47)
        {ArmorColor::Neutral, ArmorName::Sentry},
     {ArmorColor::Neutral, ArmorName::One},
     {ArmorColor::Neutral, ArmorName::Two},
     {ArmorColor::Neutral, ArmorName::Three},
     {ArmorColor::Neutral, ArmorName::Four},
     {ArmorColor::Neutral, ArmorName::Five},
     {ArmorColor::Neutral, ArmorName::Outpost},
     {ArmorColor::Neutral, ArmorName::Base},
     // Purple, Small (48-55)
        {ArmorColor::Purple, ArmorName::Sentry},
     {ArmorColor::Purple, ArmorName::One},
     {ArmorColor::Purple, ArmorName::Two},
     {ArmorColor::Purple, ArmorName::Three},
     {ArmorColor::Purple, ArmorName::Four},
     {ArmorColor::Purple, ArmorName::Five},
     {ArmorColor::Purple, ArmorName::Outpost},
     {ArmorColor::Purple, ArmorName::Base},
     // Purple, Large (56-63)
        {ArmorColor::Purple, ArmorName::Sentry},
     {ArmorColor::Purple, ArmorName::One},
     {ArmorColor::Purple, ArmorName::Two},
     {ArmorColor::Purple, ArmorName::Three},
     {ArmorColor::Purple, ArmorName::Four},
     {ArmorColor::Purple, ArmorName::Five},
     {ArmorColor::Purple, ArmorName::Outpost},
     {ArmorColor::Purple, ArmorName::Base},
     }
};

} // anonymous namespace

void spdlog_ort_logger(
    void* param, OrtLoggingLevel severity, const char* category, const char* logid,
    const char* code_location, const char* message) {
    spdlog::source_loc sloc{};

    std::istringstream iss(code_location);

    std::string file_line;
    std::string funcname;
    iss >> file_line;
    iss >> funcname;
    sloc.funcname = funcname.c_str();

    auto colon_pos = file_line.rfind(':');
    auto filename  = file_line.substr(0, colon_pos);

    if (colon_pos != std::string::npos) {
        sloc.filename = filename.c_str();
        sloc.line     = std::stoi(file_line.substr(colon_pos + 1));
    } else {
        sloc.filename = file_line.c_str();
    }

    // Map ORT severity to spdlog severity
    switch (severity) {
    case ORT_LOGGING_LEVEL_VERBOSE:
        spdlog::default_logger()->log(sloc, spdlog::level::debug, "[{}] {}", category, message);
        break;
    case ORT_LOGGING_LEVEL_INFO:
        spdlog::default_logger()->log(sloc, spdlog::level::info, "[{}] {}", category, message);
        break;
    case ORT_LOGGING_LEVEL_WARNING:
        spdlog::default_logger()->log(sloc, spdlog::level::warn, "[{}] {}", category, message);
        break;
    case ORT_LOGGING_LEVEL_ERROR:
        spdlog::default_logger()->log(sloc, spdlog::level::err, "[{}] {}", category, message);
        break;
    case ORT_LOGGING_LEVEL_FATAL:
        spdlog::default_logger()->log(sloc, spdlog::level::critical, "[{}] {}", category, message);
        break;
    }
}
struct OrtBackend::Impl {
    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;

    Ort::AllocatorWithDefaultOptions allocator;

    std::string input_name;
    std::string output_name;

    std::array<const char*, 1> input_names{};
    std::array<const char*, 1> output_names{};

    std::vector<int64_t> input_shape{1, 3, INPUT_H, INPUT_W};

    std::vector<float> input_buffer;
    std::vector<Ort::Value> output_tensors;

    Impl()
        : env(ORT_LOGGING_LEVEL_WARNING, "", spdlog_ort_logger, nullptr) {}
};

// ============================================================================
// Constructor / Destructor
// ============================================================================

OrtBackend::OrtBackend(Config config) noexcept
    : config_{std::move(config)}
    , impl_(std::make_unique<Impl>()) {}

OrtBackend::~OrtBackend() = default;

OrtBackend::OrtBackend(OrtBackend&& other) noexcept
    : config_{std::move(other.config_)}
    , impl_(std::move(other.impl_)) {}

OrtBackend& OrtBackend::operator=(OrtBackend&& other) noexcept {
    if (this != &other) {
        config_ = std::move(other.config_);
        impl_   = std::move(other.impl_);
    }
    return *this;
}

// ============================================================================
// Factory — Construction IS Initialization
// ============================================================================
std::expected<OrtBackend, std::string> OrtBackend::create(Config config) noexcept {
    try {
        OrtBackend backend(std::move(config));
        const auto& cfg = backend.get_config();

        auto& impl = *backend.impl_;
        std::string selected_provider{"CPUExecutionProvider"};
        const auto available_providers = Ort::GetAvailableProviders();

        impl.session_options.SetIntraOpNumThreads(2);
        impl.session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        impl.session_options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        impl.session_options.SetDeterministicCompute(true);
#if defined(__linux__) && defined(__x86_64__)
        if (provider_available(available_providers, "OpenVINOExecutionProvider")
            || provider_available(available_providers, "OpenVINO")) {
            const auto openvino_device_type = pick_openvino_device_type();

            try {
                OrtOpenVINOProviderOptions openvino_options{};
                openvino_options.device_type = openvino_device_type.c_str();
                openvino_options.cache_dir   = cfg.cache_dir.empty() ? "" : cfg.cache_dir.c_str();

                impl.session_options.AppendExecutionProvider_OpenVINO(openvino_options);
                selected_provider = "OpenVINOExecutionProvider(" + openvino_device_type + ")";
            } catch (const std::exception& gpu_fp16_error) {
                SPDLOG_WARN(
                    "[OrtBackend] Failed to enable OpenVINO {}: {}. Retrying with GPU_FP32.",
                    openvino_device_type, gpu_fp16_error.what());

                try {
                    OrtOpenVINOProviderOptions openvino_options{};
                    static constexpr char kFallbackDeviceType[] = "GPU_FP32";
                    openvino_options.device_type                = kFallbackDeviceType;
                    openvino_options.cache_dir = cfg.cache_dir.empty() ? "" : cfg.cache_dir.c_str();

                    impl.session_options.AppendExecutionProvider_OpenVINO(openvino_options);
                    selected_provider = "OpenVINOExecutionProvider(GPU_FP32)";
                } catch (const std::exception& gpu_fp32_error) {
                    SPDLOG_WARN(
                        "[OrtBackend] Failed to enable OpenVINO GPU execution: {}. Falling back "
                        "to CPUExecutionProvider.",
                        gpu_fp32_error.what());
                }
            }
        } else {
            SPDLOG_INFO(
                "[OrtBackend] OpenVINOExecutionProvider is not available in this ONNX Runtime "
                "build. Falling back to CPUExecutionProvider.");
        }
#endif
#ifdef __APPLE__
        std::unordered_map<std::string, std::string> coreml_options;

        coreml_options["ModelFormat"]                        = "MLProgram";
        coreml_options["MLComputeUnits"]                     = "CPUAndNeuralEngine";
        coreml_options["RequireStaticInputShapes"]           = "1";
        coreml_options["SpecializationStrategy"]             = "FastPrediction";
        coreml_options["AllowLowPrecisionAccumulationOnGPU"] = "1";
        coreml_options["ModelCacheDirectory"]                = cfg.cache_dir;

        impl.session_options.AppendExecutionProvider("CoreML", coreml_options);
        selected_provider = "CoreMLExecutionProvider";
#endif
        impl.session =
            std::make_unique<Ort::Session>(impl.env, cfg.model_path.c_str(), impl.session_options);

        auto input_name  = impl.session->GetInputNameAllocated(0, impl.allocator);
        auto output_name = impl.session->GetOutputNameAllocated(0, impl.allocator);

        impl.input_name  = input_name.get();
        impl.output_name = output_name.get();

        impl.input_names  = {impl.input_name.c_str()};
        impl.output_names = {impl.output_name.c_str()};

        auto input_type_info   = impl.session->GetInputTypeInfo(0);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        auto input_shape       = input_tensor_info.GetShape();

        if (input_shape.size() != 4) {
            return std::unexpected("[OrtBackend] Expected input shape [1,3,H,W]");
        }

        // 支持动态维度：-1 就用你的固定 INPUT_H / INPUT_W。
        if (input_shape[0] <= 0)
            input_shape[0] = 1;
        if (input_shape[1] <= 0)
            input_shape[1] = 3;
        if (input_shape[2] <= 0)
            input_shape[2] = INPUT_H;
        if (input_shape[3] <= 0)
            input_shape[3] = INPUT_W;

        if (input_shape[0] != 1 || input_shape[1] != 3) {
            return std::unexpected("[OrtBackend] Only NCHW input [1,3,H,W] is supported");
        }

        impl.input_shape = std::move(input_shape);

        SPDLOG_INFO(
            "[OrtBackend] ONNXRuntime initialized. provider={}, input={}, output={}, shape=[{}, "
            "{}, {}, {}]",
            selected_provider, impl.input_name, impl.output_name, impl.input_shape[0],
            impl.input_shape[1], impl.input_shape[2], impl.input_shape[3]);

        return backend;
    } catch (const std::exception& e) {
        return std::unexpected(
            std::string("[OrtBackend] Failed to create ONNXRuntime backend: ") + e.what());
    }
}

// ============================================================================
// Preprocessing
// ============================================================================
OrtBackend::PreprocContext
    OrtBackend::preprocess(const cv::Mat& image, const ArmorOrtConfig& /*cfg*/) const noexcept {
    PreprocContext ctx;
    ctx.orig_w = image.cols;
    ctx.orig_h = image.rows;

    if (!impl_ || !impl_->session) {
        SPDLOG_ERROR("[OrtBackend] Session is not initialized");
        return ctx;
    }

    const auto& shp = impl_->input_shape; // [1, 3, H, W]

    if (shp.size() != 4 || shp[0] != 1 || shp[1] != 3) {
        SPDLOG_ERROR(
            "[OrtBackend] Input shape mismatch, got [{}, {}, {}, {}]", shp.size() > 0 ? shp[0] : -1,
            shp.size() > 1 ? shp[1] : -1, shp.size() > 2 ? shp[2] : -1,
            shp.size() > 3 ? shp[3] : -1);
        return ctx;
    }

    const int input_h = static_cast<int>(shp[2]);
    const int input_w = static_cast<int>(shp[3]);

    const float r = std::min(
        static_cast<float>(input_w) / static_cast<float>(image.cols),
        static_cast<float>(input_h) / static_cast<float>(image.rows));

    const int unpad_w = static_cast<int>(std::round(r * static_cast<float>(image.cols)));
    const int unpad_h = static_cast<int>(std::round(r * static_cast<float>(image.rows)));

    const int pad_w = input_w - unpad_w;
    const int pad_h = input_h - unpad_h;

    const int left   = pad_w / 2;
    const int right  = pad_w - left;
    const int top    = pad_h / 2;
    const int bottom = pad_h - top;

    ctx.scale_x = 1.0f / r;
    ctx.scale_y = 1.0f / r;
    ctx.pad_x   = static_cast<float>(left);
    ctx.pad_y   = static_cast<float>(top);

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(unpad_w, unpad_h));

    cv::copyMakeBorder(
        resized, resized, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    resized.convertTo(resized, CV_32F, 1.0f / 255.0f);
    ctx.preprocessed = resized;

    return ctx;
}

bool OrtBackend::infer(const PreprocContext& ctx) noexcept {
    if (ctx.preprocessed.empty()) {
        return false;
    }

    if (!impl_ || !impl_->session) {
        SPDLOG_ERROR("[OrtBackend] Session is not initialized");
        return false;
    }

    try {
        const auto& shape = impl_->input_shape;

        const int64_t n = shape[0];
        const int64_t c = shape[1];
        const int64_t h = shape[2];
        const int64_t w = shape[3];

        if (n != 1 || c != 3) {
            SPDLOG_ERROR("[OrtBackend] Only [1,3,H,W] input is supported");
            return false;
        }

        const size_t plane      = static_cast<size_t>(h * w);
        const size_t input_size = static_cast<size_t>(n * c * h * w);

        impl_->input_buffer.resize(input_size);

        cv::Mat ch[3];
        cv::split(ctx.preprocessed, ch);

        if (ch[0].rows != h || ch[0].cols != w) {
            SPDLOG_ERROR(
                "[OrtBackend] Preprocessed image shape mismatch, got {}x{}, expected {}x{}",
                ch[0].cols, ch[0].rows, w, h);
            return false;
        }

        std::memcpy(impl_->input_buffer.data() + plane * 0, ch[0].data, plane * sizeof(float));
        std::memcpy(impl_->input_buffer.data() + plane * 1, ch[1].data, plane * sizeof(float));
        std::memcpy(impl_->input_buffer.data() + plane * 2, ch[2].data, plane * sizeof(float));

        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPUOutput);

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, impl_->input_buffer.data(), impl_->input_buffer.size(),
            impl_->input_shape.data(), impl_->input_shape.size());

        impl_->output_tensors = impl_->session->Run(
            Ort::RunOptions{nullptr}, impl_->input_names.data(), &input_tensor, 1,
            impl_->output_names.data(), 1);

        return !impl_->output_tensors.empty();
    } catch (const std::exception& e) {
        SPDLOG_ERROR("[OrtBackend] Inference failed: {}", e.what());
        return false;
    }
}

// ============================================================================
// Postprocessing
// ============================================================================

std::vector<ArmorDetection>
    OrtBackend::postprocess(const PreprocContext& ctx, const ArmorOrtConfig& cfg) const noexcept {
    std::vector<ArmorDetection> detections;

    if (!impl_ || impl_->output_tensors.empty()) {
        return detections;
    }

    const Ort::Value& out = impl_->output_tensors[0];

    if (!out.IsTensor()) {
        SPDLOG_WARN("[OrtBackend] Output is not a tensor");
        return detections;
    }

    const float* p = out.GetTensorData<float>();
    if (!p) {
        return detections;
    }

    auto tensor_info = out.GetTensorTypeAndShapeInfo();
    const auto shape = tensor_info.GetShape();

    // Output: [1, max_det, 14]
    // [0-3]: bbox x, y, w, h
    // [4]: confidence
    // [5]: class_id
    // [6-13]: 4 keypoints
    if (shape.size() != 3 || shape[0] != 1) {
        SPDLOG_WARN("[OrtBackend] Unexpected output shape");
        return detections;
    }

    const int max_det    = static_cast<int>(shape[1]);
    const int output_dim = static_cast<int>(shape[2]);

    if (output_dim < 14) {
        SPDLOG_WARN("[OrtBackend] Unexpected output dim: {}", output_dim);
        return detections;
    }

    static constexpr std::array<int, 4> keypoint_map = {0, 3, 2, 1};

    detections.reserve(128);

    const float conf_thresh = static_cast<float>(cfg.confidence_threshold);

    const float scale_x = 1.0f / ctx.scale_x;
    const float scale_y = 1.0f / ctx.scale_y;

    for (int i = 0; i < max_det; ++i) {
        const float* row = p + i * output_dim;

        const float conf = row[4];
        if (!std::isfinite(conf) || conf < conf_thresh) {
            continue;
        }

        const float x = row[0];
        const float y = row[1];
        const float w = row[2];
        const float h = row[3];

        if (!std::isfinite(x) || !std::isfinite(y) || w <= 0.0f || h <= 0.0f) {
            continue;
        }

        const int cls_id = static_cast<int>(row[5]);
        if (cls_id < 0 || cls_id >= static_cast<int>(g_class_mappings.size())) {
            continue;
        }

        const auto class_info = g_class_mappings[cls_id];

        std::array<cv::Point2f, 4> pts;
        for (int k = 0; k < 4; ++k) {
            const float kx       = row[6 + 2 * k];
            const float ky       = row[6 + 2 * k + 1];
            pts[keypoint_map[k]] = cv::Point2f(kx, ky);
        }

        for (auto& pt : pts) {
            pt.x = (pt.x - ctx.pad_x) / scale_x;
            pt.y = (pt.y - ctx.pad_y) / scale_y;
        }

        ArmorDetection det(pts, class_info.name, class_info.color, conf);
        if (det.area > 0) {
            detections.push_back(det);
        }
    }

    return detections;
}

// ============================================================================
// Coordinate Transform
// ============================================================================

void OrtBackend::transform_coordinates(
    std::vector<ArmorDetection>& detections, const PreprocContext& ctx) const noexcept {
    // Already done in postprocess via map_point
    (void)detections;
    (void)ctx;
}

// ============================================================================
// NMS
// ============================================================================

std::vector<ArmorDetection> OrtBackend::nms(
    std::vector<ArmorDetection>& detections, const ArmorOrtConfig& cfg) const noexcept {
    if (detections.empty()) {
        return {};
    }

    // Sort by confidence (descending)
    std::sort(
        detections.begin(), detections.end(), [](const ArmorDetection& a, const ArmorDetection& b) {
            return a.confidence > b.confidence;
        });

    // NMS
    const float nms_thresh = static_cast<float>(cfg.nms_threshold);
    std::vector<bool> suppressed(detections.size(), false);
    std::vector<ArmorDetection> result;
    result.reserve(detections.size());

    for (size_t i = 0; i < detections.size(); ++i) {
        if (suppressed[i])
            continue;

        result.push_back(detections[i]);

        for (size_t j = i + 1; j < detections.size(); ++j) {
            if (suppressed[j])
                continue;

            if (at_legacy::detail::iou(detections[i].rect, detections[j].rect) > nms_thresh) {
                suppressed[j] = true;
            }
        }
    }

    // Apply top_k after NMS (matching tensor_rt order)
    if (static_cast<int>(result.size()) > cfg.top_k) {
        result.resize(cfg.top_k);
    }

    return result;
}

// ============================================================================
// Detection Entry Point
// ============================================================================

OrtBackend::DetectionResult
    OrtBackend::detect_impl(const cv::Mat& image, ArmorColor color) noexcept {
    if (image.empty()) {
        return std::unexpected("Empty image passed to NN backend");
    }

    const auto& cfg = get_config();

    // Preprocess
    auto ctx = preprocess(image, cfg);
    if (ctx.preprocessed.empty()) {
        return std::unexpected("NN backend preprocessing failed");
    }

    // Infer
    if (!infer(ctx)) {
        return std::unexpected("NN backend inference failed");
    }

    // Postprocess
    auto detections = postprocess(ctx, cfg);

    // NMS
    auto result = nms(detections, cfg);

    // Debug: print detection results
    // SPDLOG_INFO("[OrtBackend] ===== Detection Results =====");
    // SPDLOG_INFO("[OrtBackend] Total detections: {}", result.size());
    // for (size_t i = 0; i < result.size(); ++i) {
    //     const auto& det = result[i];
    //     const auto center = det.center();

    //     SPDLOG_INFO("[OrtBackend] [{}] {} {} | conf={:.3f} | area={}",
    //         i,
    //         to_string(det.name),
    //         to_string(det.color),
    //         det.confidence,
    //         det.area);

    //     SPDLOG_INFO("[OrtBackend]     Center: ({:.1f}, {:.1f})", center.x, center.y);

    //     SPDLOG_INFO("[OrtBackend]     Corners: TL=({:.1f},{:.1f}) TR=({:.1f},{:.1f})
    //     BR=({:.1f},{:.1f}) BL=({:.1f},{:.1f})",
    //         det.corners[0].x, det.corners[0].y,
    //         det.corners[1].x, det.corners[1].y,
    //         det.corners[2].x, det.corners[2].y,
    //         det.corners[3].x, det.corners[3].y);

    //     SPDLOG_INFO("[OrtBackend]     BBox: x={:.1f} y={:.1f} w={:.1f} h={:.1f}",
    //         det.rect.x, det.rect.y, det.rect.width, det.rect.height);
    // }
    // SPDLOG_INFO("[OrtBackend] =================================");

    return result;
}

// ============================================================================
// Utility
// ============================================================================

std::vector<std::string> OrtBackend::execution_devices() const noexcept {
    try {
        return Ort::GetAvailableProviders();
    } catch (const std::exception& e) {
        SPDLOG_WARN("[OrtBackend] Failed to get ONNXRuntime providers: {}", e.what());
        return {};
    }
}

} // namespace fcs::L2
// mod
