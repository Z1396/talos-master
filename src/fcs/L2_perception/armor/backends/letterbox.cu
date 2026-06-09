#include "L2_perception/armor/backends/letterbox.cuh"

/// Letterbox preprocessing kernel with shared memory optimization
/// Uses shared memory to cache input tiles for better memory coalescing
__global__ void letterbox_kernel_shared(
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
) {
    int x = blockIdx.x * TILE_W + threadIdx.x;
    int y = blockIdx.y * TILE_H + threadIdx.y;
    if (x >= out_w || y >= out_h)
        return;

    // Shared memory for tile caching (TILE_H+1 x TILE_W+1 for bilinear interpolation)
    __shared__ unsigned char smem[TILE_H + 1][TILE_W + 1][3];

    int tid = threadIdx.y * blockDim.x + threadIdx.x;
    int total_smem = (TILE_W + 1) * (TILE_H + 1);
    int iter = (total_smem + blockDim.x * blockDim.y - 1) / (blockDim.x * blockDim.y);

    float inv_scale = 1.0f / scale;
    float block_start_x = (blockIdx.x * TILE_W - pad_l) * inv_scale;
    float block_start_y = (blockIdx.y * TILE_H - pad_t) * inv_scale;

    // Load input data into shared memory
    for (int i = 0; i < iter; i++) {
        int idx = tid + i * (blockDim.x * blockDim.y);
        if (idx < total_smem) {
            int sx = idx % (TILE_W + 1);
            int sy = idx / (TILE_W + 1);

            float in_x = block_start_x + sx * inv_scale;
            float in_y = block_start_y + sy * inv_scale;
            int ix = static_cast<int>(in_x); // floorf for positive values
            int iy = static_cast<int>(in_y);

            // Default padding color (114, 114, 114) - YOLO standard
            unsigned char b = 114, g = 114, r = 114;
            if (ix >= 0 && iy >= 0 && ix < in_w && iy < in_h) {
                int offset = (iy * in_w + ix) * 3;
                b = input_bgr[offset];
                g = input_bgr[offset + 1];
                r = input_bgr[offset + 2];
            }
            smem[sy][sx][0] = b;
            smem[sy][sx][1] = g;
            smem[sy][sx][2] = r;
        }
    }
    __syncthreads();

    // Bilinear interpolation
    float in_x = (x - pad_l) * inv_scale;
    float in_y = (y - pad_t) * inv_scale;
    float dx = in_x - floorf(in_x);
    float dy = in_y - floorf(in_y);
    float dx1 = 1.0f - dx;
    float dy1 = 1.0f - dy;

    // Fetch 4 neighboring pixels from shared memory
    unsigned char* p00 = smem[threadIdx.y][threadIdx.x];
    unsigned char* p01 = smem[threadIdx.y][threadIdx.x + 1];
    unsigned char* p10 = smem[threadIdx.y + 1][threadIdx.x];
    unsigned char* p11 = smem[threadIdx.y + 1][threadIdx.x + 1];

    // Interpolate each channel
    float out_r = dx1 * dy1 * p00[2] + dx * dy1 * p01[2] + dx1 * dy * p10[2] + dx * dy * p11[2];
    float out_g = dx1 * dy1 * p00[1] + dx * dy1 * p01[1] + dx1 * dy * p10[1] + dx * dy * p11[1];
    float out_b = dx1 * dy1 * p00[0] + dx * dy1 * p01[0] + dx1 * dy * p10[0] + dx * dy * p11[0];

    // Write output in NCHW format
    int out_idx = y * out_w + x;
    int plane_size = out_w * out_h;
    if (swap_rb) {
        // RGB output
        output_nchw[out_idx] = out_r * norm;
        output_nchw[out_idx + plane_size] = out_g * norm;
        output_nchw[out_idx + 2 * plane_size] = out_b * norm;
    } else {
        // BGR output
        output_nchw[out_idx] = out_b * norm;
        output_nchw[out_idx + plane_size] = out_g * norm;
        output_nchw[out_idx + 2 * plane_size] = out_r * norm;
    }
}

/// Letterbox preprocessing kernel for pitched (non-contiguous) memory
/// Used when input comes from cv::Mat ROI or has padding
extern "C" __global__ void letterbox_kernel_pitched(
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
) {
    int ox = blockIdx.x * blockDim.x + threadIdx.x;
    int oy = blockIdx.y * blockDim.y + threadIdx.y;
    if (ox >= OUT_W || oy >= OUT_H)
        return;

    // Inverse mapping: output -> input
    float fx = (ox - pad_l) / scale;
    float fy = (oy - pad_t) / scale;

    int out_idx = oy * OUT_W + ox;
    int plane = OUT_W * OUT_H;

    // Default padding color
    float r = 114.0f, g = 114.0f, b = 114.0f;

    if (fx >= 0.0f && fy >= 0.0f && fx < src_w - 1 && fy < src_h - 1) {
        int x0 = static_cast<int>(fx);
        int y0 = static_cast<int>(fy);
        int x1 = x0 + 1;
        int y1 = y0 + 1;

        float dx = fx - x0;
        float dy = fy - y0;
        float dx1 = 1.0f - dx;
        float dy1 = 1.0f - dy;

        // Get row pointers using pitch (stride in bytes)
        const unsigned char* row0 = d_input_bgr + y0 * pitch;
        const unsigned char* row1 = d_input_bgr + y1 * pitch;

        int i00 = x0 * 3;
        int i01 = x1 * 3;

        // Load 4 pixels (BGR)
        float b00 = row0[i00 + 0], g00 = row0[i00 + 1], r00 = row0[i00 + 2];
        float b01 = row0[i01 + 0], g01 = row0[i01 + 1], r01 = row0[i01 + 2];
        float b10 = row1[i00 + 0], g10 = row1[i00 + 1], r10 = row1[i00 + 2];
        float b11 = row1[i01 + 0], g11 = row1[i01 + 1], r11 = row1[i01 + 2];

        // Bilinear interpolation weights
        float w00 = dx1 * dy1;
        float w01 = dx * dy1;
        float w10 = dx1 * dy;
        float w11 = dx * dy;

        r = r00 * w00 + r01 * w01 + r10 * w10 + r11 * w11;
        g = g00 * w00 + g01 * w01 + g10 * w10 + g11 * w11;
        b = b00 * w00 + b01 * w01 + b10 * w10 + b11 * w11;
    }

    // Write output in NCHW format
    if (swap_rb) {
        d_nchw[out_idx] = r * norm;
        d_nchw[out_idx + plane] = g * norm;
        d_nchw[out_idx + 2 * plane] = b * norm;
    } else {
        d_nchw[out_idx] = b * norm;
        d_nchw[out_idx + plane] = g * norm;
        d_nchw[out_idx + 2 * plane] = r * norm;
    }
}
