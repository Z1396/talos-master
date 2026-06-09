#include "L2_perception/armor/backends/tensor_rt.hpp"
#include "L2_perception/armor/backends/letterbox.cuh"
#include "L2_perception/armor/config.hpp"
#include "core/types.hpp"

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <thread>
#include <unordered_map>
#include <vector>

#include <cuda_runtime.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>

namespace fcs::L2 {

namespace fs = std::filesystem;

// ============================================================================
// Logger for TensorRT
// ============================================================================

class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            spdlog::default_logger()->log(
                spdlog::level::from_str(severity_to_str(severity)), "[TensorRT] {}", msg);
        }
    }

    static const char* severity_to_str(Severity severity) noexcept {
        switch (severity) {
        case Severity::kINTERNAL_ERROR: return "critical";
        case Severity::kERROR: return "err";
        case Severity::kWARNING: return "warn";
        case Severity::kINFO: return "info";
        case Severity::kVERBOSE: return "debug";
        default: return "trace";
        }
    }
};

static TrtLogger g_trt_logger;

// ============================================================================
// Class-to-Color/Number Mapping (64 classes)
// ============================================================================
// 64 classes = 4 colors × 2 sizes × 8 numbers
// Color encoding: 0=Red, 1=Blue, 2=None (Gray), 3=Purple
// Size encoding: 0-7=Small, 8-15=Small, ...
// Number encoding: mod 8 (0=Sentry, 1-5=Numbers, 6=Outpost, 7=Base)

struct ClassMapping {
    ArmorColor color;
    ArmorName name;
};

static constexpr std::array<ClassMapping, 64> g_class_mappings = {
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

// ============================================================================
// PIMPL Implementation
// ============================================================================

struct TrtBackend::Impl {
    // TensorRT resources
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::vector<std::unique_ptr<nvinfer1::IExecutionContext>> contexts_;

    // CUDA resources
    std::vector<cudaStream_t> streams_;
    void* device_input_{nullptr};
    void* device_output_{nullptr};
    std::vector<float> host_output_;
    std::vector<unsigned char> host_input_; // For pinned memory upload

    // Context pool mutex for thread safety
    std::mutex context_mutex_;
    std::atomic<uint32_t> next_context_{0};

    // Model dimensions
    std::string input_name_;
    std::string output_name_;
    size_t input_size_{0};
    size_t output_size_{0};
    int max_detections_{0};

    // Keypoint mapping: YOLO outputs TL, BL, BR, TR; we want TL, TR, BR, BL
    static constexpr std::array<int, 4> keypoint_map = {0, 3, 2, 1};

    Impl() noexcept = default;

    ~Impl() noexcept { release(); }

    void release() noexcept {
        // Synchronize and destroy streams
        for (auto& stream : streams_) {
            if (stream) {
                cudaStreamSynchronize(stream);
                cudaStreamDestroy(stream);
            }
        }
        streams_.clear();

        // Free device memory
        if (device_input_) {
            cudaFree(device_input_);
            device_input_ = nullptr;
        }
        if (device_output_) {
            cudaFree(device_output_);
            device_output_ = nullptr;
        }

        contexts_.clear();
        engine_.reset();
        runtime_.reset();
    }
};

// ============================================================================
// Helper Functions
// ============================================================================

namespace {

/// Load engine from file
[[nodiscard]] std::expected<std::unique_ptr<nvinfer1::ICudaEngine>, std::string>
    load_engine(nvinfer1::IRuntime* runtime, const fs::path& engine_path) noexcept {
    try {
        std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
            SPDLOG_ERROR("Failed to open engine file: {}", engine_path.string());
            return std::unexpected("TensorRT: failed to open engine file " + engine_path.string());
        }

        std::streamsize file_size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::vector<char> engine_data(file_size);
        if (!file.read(engine_data.data(), file_size)) {
            SPDLOG_ERROR("Failed to read engine file: {}", engine_path.string());
            return std::unexpected("TensorRT: failed to read engine file " + engine_path.string());
        }

        auto* engine = runtime->deserializeCudaEngine(engine_data.data(), file_size);
        if (!engine) {
            SPDLOG_ERROR("Failed to deserialize CUDA engine");
            return std::unexpected("TensorRT: failed to deserialize CUDA engine");
        }

        return std::unique_ptr<nvinfer1::ICudaEngine>(engine);
    } catch (const std::exception& e) {
        SPDLOG_ERROR("[TrtBackend] Unexpected exception in load_engine: {}", e.what());
        return std::unexpected(
            "[TrtBackend::load_engine] Unhandled exception: " + std::string(e.what()));
    }
}

/// Build engine from ONNX model
[[nodiscard]] std::expected<std::unique_ptr<nvinfer1::ICudaEngine>, std::string>
    build_engine_from_onnx(
        nvinfer1::IRuntime* runtime, const fs::path& onnx_path, const fs::path& cache_dir,
        const std::string& compute_capability, bool enable_fp16, bool enable_dla,
        int dla_core) noexcept {
    auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(g_trt_logger));
    if (!builder) {
        SPDLOG_ERROR("Failed to create TensorRT builder");
        return std::unexpected("TensorRT: failed to create builder");
    }

    const auto explicit_batch =
        1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network =
        std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicit_batch));
    if (!network) {
        SPDLOG_ERROR("Failed to create network definition");
        return std::unexpected("TensorRT: failed to create network definition");
    }

    auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
    if (!config) {
        SPDLOG_ERROR("Failed to create builder config");
        return std::unexpected("TensorRT: failed to create builder config");
    }

    // Set FP16 mode
    if (enable_fp16 && builder->platformHasFastFp16()) {
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
        SPDLOG_INFO("TensorRT: FP16 mode enabled");
    }

    // Set DLA core
    if (enable_dla && builder->getNbDLACores() > 0) {
        config->setDLACore(dla_core);
        SPDLOG_INFO("TensorRT: DLA core {} enabled", dla_core);
    }

    // Set memory limits
    size_t free_mem, total_mem;
    cudaMemGetInfo(&free_mem, &total_mem);
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, free_mem * 0.9);

    // Parse ONNX
    auto parser =
        std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, g_trt_logger));
    if (!parser) {
        SPDLOG_ERROR("Failed to create ONNX parser");
        return std::unexpected("TensorRT: failed to create ONNX parser");
    }

    if (!parser->parseFromFile(
            onnx_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kINFO))) {
        SPDLOG_ERROR("Failed to parse ONNX model: {}", onnx_path.string());
        return std::unexpected("TensorRT: failed to parse ONNX model " + onnx_path.string());
    }

    // Build serialized network
    auto plan =
        std::unique_ptr<nvinfer1::IHostMemory>(builder->buildSerializedNetwork(*network, *config));
    if (!plan) {
        SPDLOG_ERROR("Failed to build serialized network");
        return std::unexpected("TensorRT: failed to build serialized network");
    }

    // Cache engine
    if (!cache_dir.empty()) {
        fs::create_directories(cache_dir);
        std::string cache_name = onnx_path.stem().string();
        cache_name += "_sm";
        cache_name += compute_capability;
        if (enable_fp16)
            cache_name += "_fp16";
        cache_name += ".engine";

        fs::path cache_path = cache_dir / cache_name;

        std::ofstream cache_file(cache_path, std::ios::binary);
        if (cache_file.is_open()) {
            cache_file.write(static_cast<const char*>(plan->data()), plan->size());
            SPDLOG_INFO("Cached TensorRT engine to: {}", cache_path.string());
        }
    }

    auto* engine = runtime->deserializeCudaEngine(plan->data(), plan->size());
    if (!engine) {
        SPDLOG_ERROR("Failed to deserialize CUDA engine from ONNX");
        return std::unexpected("TensorRT: failed to deserialize CUDA engine from ONNX build");
    }

    return std::unique_ptr<nvinfer1::ICudaEngine>(engine);
}

/// Calculate IoU for NMS
[[nodiscard]] inline float calculate_iou(const cv::Rect2f& a, const cv::Rect2f& b) noexcept {
    const cv::Rect2f inter = a & b;
    const float inter_area = inter.area();
    const float union_area = a.area() + b.area() - inter_area;
    if (union_area <= 0.0f || std::isnan(union_area)) {
        return 0.0f;
    }
    return inter_area / union_area;
}

/// NMS with merging for overlapping detections
[[nodiscard]] std::vector<int> nms_merge_sorted_bboxes(
    const std::vector<cv::Rect2f>& boxes, const std::vector<float>& confidences,
    float nms_threshold) {
    const size_t n = boxes.size();
    std::vector<int> keep;
    std::vector<bool> suppressed(n, false);

    for (size_t i = 0; i < n; ++i) {
        if (suppressed[i])
            continue;

        keep.push_back(static_cast<int>(i));

        for (size_t j = i + 1; j < n; ++j) {
            if (suppressed[j])
                continue;

            float iou = calculate_iou(boxes[i], boxes[j]);
            if (iou > nms_threshold) {
                suppressed[j] = true;
            }
        }
    }

    return keep;
}

} // anonymous namespace

// ============================================================================
// Constructor/Destructor
// ============================================================================

TrtBackend::TrtBackend(Config config) noexcept
    : config_(std::move(config)) {
    impl_ = std::make_unique<Impl>();
}

TrtBackend::~TrtBackend() noexcept = default;

TrtBackend::TrtBackend(TrtBackend&&) noexcept = default;

TrtBackend& TrtBackend::operator=(TrtBackend&&) noexcept = default;

// ============================================================================
// Factory — Construction IS Initialization
// ============================================================================

std::expected<TrtBackend, std::string> TrtBackend::create(Config config) noexcept {
    TrtBackend backend(std::move(config));

    // Set CUDA device
    cudaError_t cuda_err = cudaSetDevice(backend.config_.device_id);
    if (cuda_err != cudaSuccess) {
        SPDLOG_ERROR(
            "Failed to set CUDA device {}: {}", backend.config_.device_id,
            cudaGetErrorString(cuda_err));
        return std::unexpected(
            "TensorRT: failed to set CUDA device " + std::to_string(backend.config_.device_id)
            + ": " + cudaGetErrorString(cuda_err));
    }

    // Create TensorRT runtime
    backend.impl_->runtime_ =
        std::unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(g_trt_logger));
    if (!backend.impl_->runtime_) {
        SPDLOG_ERROR("Failed to create TensorRT runtime");
        return std::unexpected(
            "TensorRT: failed to set CUDA device " + std::to_string(backend.config_.device_id)
            + ": " + cudaGetErrorString(cuda_err));
    }

    // Load or build engine
    fs::path engine_path(backend.config_.engine_path);
    fs::path model_path(backend.config_.model_path);
    fs::path cache_dir(backend.config_.engine_cache_dir);

    std::unique_ptr<nvinfer1::ICudaEngine> engine;

    // Try to load existing engine file
    if (!engine_path.empty() && fs::exists(engine_path)) {
        SPDLOG_INFO("Loading TensorRT engine from: {}", engine_path.string());
        auto result = load_engine(backend.impl_->runtime_.get(), engine_path);
        if (!result) {
            SPDLOG_ERROR("Failed to load engine, will try building from ONNX if available");
        } else {
            engine = std::move(result.value());
        }
    }

    // Build from ONNX if engine loading failed
    if (!engine && !model_path.empty() && fs::exists(model_path)) {
        SPDLOG_INFO("Building TensorRT engine from ONNX: {}", model_path.string());
        auto result = build_engine_from_onnx(
            backend.impl_->runtime_.get(), model_path, cache_dir,
            backend.config_.compute_capability, backend.config_.enable_fp16,
            backend.config_.enable_dla, backend.config_.dla_core);
        if (!result) {
            SPDLOG_ERROR("Failed to build engine from ONNX");
            return std::unexpected("TensorRT: failed to build engine from ONNX");
        }
        engine = std::move(result.value());
    }

    if (!engine) {
        SPDLOG_ERROR("No valid engine or model found");
        return std::unexpected("TensorRT: no valid engine or model file found");
    }

    backend.impl_->engine_ = std::move(engine);

    // Get I/O tensors
    const int num_tensors = backend.impl_->engine_->getNbIOTensors();
    for (int i = 0; i < num_tensors; ++i) {
        const char* name = backend.impl_->engine_->getIOTensorName(i);
        auto mode        = backend.impl_->engine_->getTensorIOMode(name);
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            backend.impl_->input_name_ = name;
            auto dims                  = backend.impl_->engine_->getTensorShape(name);
            backend.impl_->input_size_ = INPUT_W * INPUT_H * 3 * sizeof(float);
            SPDLOG_INFO("Input: {} dims=[{}]", name, dims.d[0]);
        } else {
            backend.impl_->output_name_ = name;
            auto dims                   = backend.impl_->engine_->getTensorShape(name);
            // Output shape: [1, max_det, 14]
            backend.impl_->max_detections_ = dims.d[1];
            backend.impl_->output_size_ =
                backend.impl_->max_detections_ * OUTPUT_DIM * sizeof(float);
            SPDLOG_INFO("Output: {} dims=[1, {}, {}]", name, dims.d[1], dims.d[2]);
        }
    }

    // Allocate device memory
    cuda_err = cudaMalloc(&backend.impl_->device_input_, backend.impl_->input_size_);
    if (cuda_err != cudaSuccess) {
        SPDLOG_ERROR("Failed to allocate device input buffer: {}", cudaGetErrorString(cuda_err));
        return std::unexpected(
            "TensorRT: failed to set CUDA device " + std::to_string(backend.config_.device_id)
            + ": " + cudaGetErrorString(cuda_err));
    }

    cuda_err = cudaMalloc(&backend.impl_->device_output_, backend.impl_->output_size_);
    if (cuda_err != cudaSuccess) {
        SPDLOG_ERROR("Failed to allocate device output buffer: {}", cudaGetErrorString(cuda_err));
        return std::unexpected(
            "TensorRT: failed to set CUDA device " + std::to_string(backend.config_.device_id)
            + ": " + cudaGetErrorString(cuda_err));
    }

    // Resize host buffers
    backend.impl_->host_output_.resize(backend.impl_->max_detections_ * OUTPUT_DIM);
    backend.impl_->host_input_.resize(INPUT_W * INPUT_H * 3);

    // Create execution contexts and streams
    int num_streams = backend.config_.num_streams;
    backend.impl_->contexts_.reserve(num_streams);
    backend.impl_->streams_.reserve(num_streams);

    for (int i = 0; i < num_streams; ++i) {
        auto* ctx = backend.impl_->engine_->createExecutionContext();
        if (!ctx) {
            SPDLOG_ERROR("Failed to create execution context {}", i);
            return std::unexpected(
                "TensorRT: failed to set CUDA device " + std::to_string(backend.config_.device_id)
                + ": " + cudaGetErrorString(cuda_err));
        }
        backend.impl_->contexts_.emplace_back(ctx);

        cudaStream_t stream;
        cuda_err = cudaStreamCreate(&stream);
        if (cuda_err != cudaSuccess) {
            SPDLOG_ERROR("Failed to create CUDA stream {}: {}", i, cudaGetErrorString(cuda_err));
            return std::unexpected(
                "TensorRT: failed to set CUDA device " + std::to_string(backend.config_.device_id)
                + ": " + cudaGetErrorString(cuda_err));
        }
        backend.impl_->streams_.push_back(stream);
    }

    SPDLOG_INFO("TensorRT backend initialized with {} streams", num_streams);
    return backend;
}

// ============================================================================
// Detection
// ============================================================================

TrtBackend::DetectionResult
    TrtBackend::detect_impl(const cv::Mat& image, ArmorColor color) noexcept {
    if (!impl_) {
        return std::unexpected("TensorRT backend not initialized");
    }

    if (image.empty()) {
        return std::unexpected("TensorRT backend received empty image");
    }

    // Get context index (round-robin)
    uint32_t ctx_idx    = impl_->next_context_.fetch_add(1) % impl_->contexts_.size();
    auto* ctx           = impl_->contexts_[ctx_idx].get();
    cudaStream_t stream = impl_->streams_[ctx_idx];

    // Preprocess: letterbox resize
    const int src_w = image.cols;
    const int src_h = image.rows;

    // Calculate letterbox parameters
    float scale =
        std::min(static_cast<float>(INPUT_W) / src_w, static_cast<float>(INPUT_H) / src_h);
    int rw    = static_cast<int>(src_w * scale);
    int rh    = static_cast<int>(src_h * scale);
    int pad_l = (INPUT_W - rw) / 2;
    int pad_t = (INPUT_H - rh) / 2;

    // Convert BGR to packed RGB if needed (model expects RGB)
    // AT model uses BGR input (swap_rb=false)
    cv::Mat rgb_image;
    if (image.channels() == 3) {
        // Keep BGR format for AT model
        rgb_image = image;
    } else if (image.channels() == 1) {
        cv::cvtColor(image, rgb_image, cv::COLOR_GRAY2BGR);
    } else {
        return std::unexpected("TensorRT backend received image with unsupported channel count");
    }

#if TALOS_HAS_CUDA_RUNTIME
    // Use CUDA preprocessing when available
    // Check if image is contiguous
    bool is_contiguous = (rgb_image.step == rgb_image.cols * rgb_image.elemSize());

    if (is_contiguous) {
        // Upload contiguous image data
        cudaMemcpyAsync(
            impl_->device_input_, rgb_image.ptr<unsigned char>(),
            rgb_image.total() * rgb_image.elemSize(), cudaMemcpyHostToDevice, stream);

        // Launch letterbox kernel
        constexpr float NORM   = 1.0f / 255.0f;
        constexpr bool SWAP_RB = false; // AT model expects BGR
        launch_letterbox_shared(
            static_cast<unsigned char*>(impl_->device_input_), src_w, src_h,
            static_cast<float*>(impl_->device_input_), INPUT_W, INPUT_H, scale, pad_t, pad_l, NORM,
            SWAP_RB, stream);
    } else {
        // For non-contiguous (ROI) data, upload with pitch
        size_t pitch = rgb_image.step;
        cudaMemcpyAsync(
            impl_->device_input_, rgb_image.ptr<unsigned char>(), rgb_image.rows * pitch,
            cudaMemcpyHostToDevice, stream);

        constexpr float NORM   = 1.0f / 255.0f;
        constexpr bool SWAP_RB = false;
        launch_letterbox_pitched(
            static_cast<unsigned char*>(impl_->device_input_), pitch, src_w, src_h,
            static_cast<float*>(impl_->device_input_), INPUT_W, INPUT_H, scale, pad_t, pad_l, NORM,
            SWAP_RB, stream);
    }
#else
    // CPU preprocessing fallback (using OpenCV)
    cv::Mat letterboxed;
    cv::Size resized_size(rw, rh);

    // Resize the image
    cv::resize(rgb_image, letterboxed, resized_size);

    // Create the final image with padding
    cv::Mat final_image(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(114, 114, 114));

    // Copy the resized image to the center
    cv::Rect roi_rect(pad_l, pad_t, rw, rh);
    letterboxed.copyTo(final_image(roi_rect));

    // Convert to float and normalize (NCHW format)
    std::vector<cv::Mat> channels(3);
    cv::split(final_image, channels);

    std::vector<float> host_input(INPUT_W * INPUT_H * 3);
    constexpr float NORM = 1.0f / 255.0f;
    for (int c = 0; c < 3; ++c) {
        for (int i = 0; i < INPUT_W * INPUT_H; ++i) {
            host_input[c * INPUT_W * INPUT_H + i] = channels[c].data[i] * NORM;
        }
    }

    // Copy to GPU
    cudaMemcpyAsync(
        impl_->device_input_, host_input.data(), INPUT_W * INPUT_H * 3 * sizeof(float),
        cudaMemcpyHostToDevice, stream);
#endif

    // Run inference with new TensorRT API (8.5+)
    ctx->setTensorAddress(impl_->input_name_.c_str(), impl_->device_input_);
    ctx->setTensorAddress(impl_->output_name_.c_str(), impl_->device_output_);
    bool success = ctx->enqueueV3(stream);
    if (!success) {
        SPDLOG_ERROR("TensorRT inference failed");
        return std::unexpected("TensorRT inference failed (enqueueV3)");
    }

    // Copy output to host
    cudaMemcpyAsync(
        impl_->host_output_.data(), impl_->device_output_, impl_->output_size_,
        cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    // Postprocess output
    std::vector<ArmorDetection> detections;

    // AT model output: [1, max_det, 14]
    // [0-3]: bbox (x, y, w, h) center format
    // [4]: confidence
    // [5]: class_id (0-63)
    // [6-13]: 4 keypoints (x, y) × 4
    const float* output = impl_->host_output_.data();

    std::vector<cv::Rect2f> boxes;
    std::vector<float> confidences;
    std::vector<std::array<cv::Point2f, 4>> corners;
    std::vector<ClassMapping> class_infos;

    for (int i = 0; i < impl_->max_detections_; ++i) {
        const float* row = output + i * OUTPUT_DIM;

        float conf = row[4];
        if (!std::isfinite(conf) || conf < config_.confidence_threshold) {
            continue;
        }

        float x = row[0];
        float y = row[1];
        float w = row[2];
        float h = row[3];

        if (!std::isfinite(x) || !std::isfinite(y) || w <= 0.0f || h <= 0.0f) {
            continue;
        }

        int cls_id = static_cast<int>(row[5]);
        if (cls_id < 0 || cls_id >= 64) {
            continue;
        }

        // NOTE: Color filtering removed - detect all colors (RBGP)
        // Colors are still detected and preserved in the output for downstream use
        auto class_info = g_class_mappings[cls_id];
        // No color filtering - all colors allowed

        // Extract and reorder keypoints
        // YOLO outputs: TL, BL, BR, TR
        // We want: TL, TR, BR, BL
        std::array<cv::Point2f, 4> pts;
        for (int k = 0; k < 4; ++k) {
            float kx                   = row[6 + 2 * k];
            float ky                   = row[6 + 2 * k + 1];
            pts[Impl::keypoint_map[k]] = cv::Point2f(kx, ky);
        }

        // Transform coordinates back to original image
        for (auto& pt : pts) {
            pt.x = (pt.x - pad_l) / scale;
            pt.y = (pt.y - pad_t) / scale;
        }

        cv::Rect2f box(x - w / 2, y - h / 2, w, h);
        box.x = (box.x - pad_l) / scale;
        box.y = (box.y - pad_t) / scale;
        box.width /= scale;
        box.height /= scale;

        boxes.push_back(box);
        confidences.push_back(conf);
        corners.push_back(pts);
        class_infos.push_back(class_info);
    }

    // Apply NMS
    if (!boxes.empty()) {
        // Sort by confidence
        std::vector<int> indices(boxes.size());
        std::ranges::iota(indices.begin(), indices.end(), 0);
        std::ranges::sort(indices.begin(), indices.end(), [&confidences](int a, int b) {
            return confidences[a] > confidences[b];
        });

        // Reorder by confidence
        std::vector<cv::Rect2f> sorted_boxes(boxes.size());
        std::vector<std::array<cv::Point2f, 4>> sorted_corners(boxes.size());
        std::vector<ClassMapping> sorted_classes(boxes.size());
        std::vector<float> sorted_confidences(boxes.size());

        for (size_t i = 0; i < indices.size(); ++i) {
            sorted_boxes[i]       = boxes[indices[i]];
            sorted_corners[i]     = corners[indices[i]];
            sorted_classes[i]     = class_infos[indices[i]];
            sorted_confidences[i] = confidences[indices[i]];
        }

        // Apply NMS
        std::vector<int> keep =
            nms_merge_sorted_bboxes(sorted_boxes, sorted_confidences, config_.nms_threshold);

        // Create detections
        detections.reserve(keep.size());
        for (int idx : keep) {
            detections.emplace_back(
                sorted_corners[idx], sorted_classes[idx].name, sorted_classes[idx].color,
                sorted_confidences[idx]);
        }

        // Apply top_k
        if (static_cast<int>(detections.size()) > config_.top_k) {
            detections.resize(config_.top_k);
        }
    }

    return detections;
}

} // namespace fcs::L2
