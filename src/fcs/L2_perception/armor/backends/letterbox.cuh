#pragma once

#include <cuda_runtime.h>

/// Letterbox preprocessing constants
constexpr int TILE_W = 32;
constexpr int TILE_H = 32;

/// CUDA letterbox kernel for contiguous memory
/// @param input_bgr Input BGR image data (row-major, packed)
/// @param in_w Input image width
/// @param in_h Input image height
/// @param output_nchw Output NCHW tensor (3 * out_w * out_h floats)
/// @param out_w Output width (model input size)
/// @param out_h Output height (model input size)
/// @param scale Scale factor for letterbox
/// @param pad_t Top padding
/// @param pad_l Left padding
/// @param norm Normalization factor (typically 1.0f / 255.0f)
/// @param swap_rb Swap R and B channels (true for RGB output)
extern "C" void letterbox_kernel_shared(
    const unsigned char* __restrict__ input_bgr,
    int in_w,
    int in_h,
    float* __restrict__ output_nchw,
    int out_w,
    int out_h,
    float scale,
    int pad_t,
    int pad_l,
    float norm,
    bool swap_rb
);

/// CUDA letterbox kernel for pitched (non-contiguous) memory
/// Use this when copying from cv::Mat with ROI or non-contiguous data
/// @param d_input_bgr Input BGR image data on GPU (pitched)
/// @param pitch Pitch of input data in bytes
/// @param src_w Source width
/// @param src_h Source height
/// @param d_nchw Output NCHW tensor on GPU
/// @param OUT_W Output width
/// @param OUT_H Output height
/// @param scale Scale factor for letterbox
/// @param pad_t Top padding
/// @param pad_l Left padding
/// @param norm Normalization factor
/// @param swap_rb Swap R and B channels
extern "C" void letterbox_kernel_pitched(
    const unsigned char* __restrict__ d_input_bgr,
    size_t pitch,
    int src_w,
    int src_h,
    float* __restrict__ d_nchw,
    int OUT_W,
    int OUT_H,
    float scale,
    int pad_t,
    int pad_l,
    float norm,
    bool swap_rb
);

/// Launch letterbox preprocessing kernel (contiguous memory version)
/// Uses cudaLaunchKernel API instead of <<<>>> syntax for clang++ compatibility
/// @param d_input Input image on GPU
/// @param in_w Input width
/// @param in_h Input height
/// @param d_output Output tensor on GPU
/// @param out_w Output width
/// @param out_h Output height
/// @param scale Letterbox scale
/// @param pad_t Top padding
/// @param pad_l Left padding
/// @param norm Normalization factor
/// @param swap_rb Swap channels
/// @param stream CUDA stream
inline void launch_letterbox_shared(
    const unsigned char* d_input,
    int in_w,
    int in_h,
    float* d_output,
    int out_w,
    int out_h,
    float scale,
    int pad_t,
    int pad_l,
    float norm,
    bool swap_rb,
    cudaStream_t stream
) {
    void* args[] = {
        const_cast<unsigned char**>(&d_input),
        &in_w, &in_h,
        &d_output,
        &out_w, &out_h,
        &scale, &pad_t, &pad_l,
        &norm, &swap_rb
    };
    dim3 block(TILE_W, TILE_H);
    dim3 grid((out_w + TILE_W - 1) / TILE_W, (out_h + TILE_H - 1) / TILE_H);
    cudaLaunchKernel(
        reinterpret_cast<const void*>(letterbox_kernel_shared),
        grid, block, args, 0, stream
    );
}

/// Launch letterbox preprocessing kernel (pitched memory version)
/// Uses cudaLaunchKernel API instead of <<<>>> syntax for clang++ compatibility
inline void launch_letterbox_pitched(
    const unsigned char* d_input,
    size_t pitch,
    int src_w,
    int src_h,
    float* d_output,
    int out_w,
    int out_h,
    float scale,
    int pad_t,
    int pad_l,
    float norm,
    bool swap_rb,
    cudaStream_t stream
) {
    void* args[] = {
        const_cast<unsigned char**>(&d_input),
        &pitch,
        &src_w, &src_h,
        &d_output,
        &out_w, &out_h,
        &scale, &pad_t, &pad_l,
        &norm, &swap_rb
    };
    dim3 block(16, 16);
    dim3 grid((out_w + 15) / 16, (out_h + 15) / 16);
    cudaLaunchKernel(
        reinterpret_cast<const void*>(letterbox_kernel_pitched),
        grid, block, args, 0, stream
    );
}
