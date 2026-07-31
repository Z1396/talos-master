#pragma once

/**
 * @file letterbox.cuh
 * @brief CUDA Letterbox 预处理头文件
 *
 * 本文件定义了用于图像预处理的 CUDA Letterbox 操作接口。
 * Letterbox 是目标检测模型（如 YOLO）中常用的图像缩放和填充技术，
 * 用于将不同尺寸的输入图像统一缩放到模型所需的固定尺寸。
 *
 * 【Letterbox 算法几何原理】
 * Letterbox 的核心思想是在保持图像纵横比不变的前提下，将图像缩放到目标尺寸：
 * 1. 计算缩放比例 scale = min(target_w / src_w, target_h / src_h)
 * 2. 按比例缩放图像：scaled_w = src_w * scale, scaled_h = src_h * scale
 * 3. 在缩放后的图像周围填充灰色边框，使其达到目标尺寸
 * 4. 填充颜色通常为 (114, 114, 114)，这是 YOLO 的标准填充色
 *
 * 【坐标映射公式】
 * 输出坐标 (ox, oy) -> 输入坐标 (ix, iy)：
 *   ix = (ox - pad_l) / scale
 *   iy = (oy - pad_t) / scale
 * 如果 ix, iy 超出输入图像范围，则使用填充色。
 *
 * 【两种 kernel 的区别】
 * 1. letterbox_kernel_shared: 用于连续内存布局的输入图像
 *    - 适用于通过 cudaMalloc 直接分配的连续内存
 *    - 使用共享内存优化，性能更好
 *    - 适合批量处理场景
 *
 * 2. letterbox_kernel_pitched: 用于非连续内存布局（pitch内存）
 *    - 适用于 cv::Mat ROI 或有行对齐填充的内存
 *    - pitch 表示每行的字节步长（可能大于 width * channels）
 *    - 不使用共享内存优化，但支持非连续内存访问
 *
 * 【NCHW 格式说明】
 * 输出采用 NCHW（Batch-Channel-Height-Width）内存布局：
 * - 内存排列：[C, H, W] 展平为线性内存
 * - Channel 0 (R/B): [0, H*W)
 * - Channel 1 (G):    [H*W, 2*H*W)
 * - Channel 2 (B/R):  [2*H*W, 3*H*W)
 * - 这种布局适合卷积神经网络（CNN）的内存访问模式
 *
 * 【CUDA 优化要点】
 * 1. 共享内存缓存：将输入图像块缓存到共享内存，减少全局内存访问
 * 2. 双线性插值：在 GPU 上高效实现图像缩放的重采样
 * 3. 内存合并访问：通过 tile 策略优化内存访问模式
 *
 * @author Talos Team
 * @date 2024
 */

#include <cuda_runtime.h>

/// ============================================================================
/// 常量定义
/// ============================================================================

/**
 * @brief Tile 宽度（用于共享内存优化）
 *
 * 每个 CUDA block 处理 32x32 个输出像素。
 * 32x32 是经过测试的性能最优值，原因如下：
 * 1. 每个 block 有 32x32=1024 个线程，接近 GPU 最大线程数限制（1024）
 * 2. 32 是 CUDA warp 大小，有利于线程束级别的优化
 * 3. 共享内存大小：(32+1)*(32+1)*3 ≈ 3KB，适合大多数 GPU 的共享内存容量
 */
constexpr int TILE_W = 32;

/**
 * @brief Tile 高度（用于共享内存优化）
 *
 * 与 TILE_W 配合使用，定义 block 的处理区域。
 */
constexpr int TILE_H = 32;

/// ============================================================================
/// Kernel 函数声明
/// ============================================================================

/**
 * @brief 连续内存版本的 Letterbox 预处理 kernel
 *
 * 【功能说明】
 * 对连续内存布局的输入图像执行 letterbox 变换，包括：
 * 1. 坐标映射（输出 -> 输入）
 * 2. 双线性插值重采样
 * 3. RGB/BGR 通道交换（可选）
 * 4. 归一化处理（像素值 0-255 -> 浮点 0.0-1.0）
 *
 * 【CUDA 优化原理】
 * 1. 共享内存缓存：
 *    - 将输入图像的一个 tile 加载到共享内存
 *    - 共享内存大小：(TILE_H+1) x (TILE_W+1) x 3
 *    - +1 是为了双线性插值需要访问相邻像素
 *    - 减少全局内存访问次数：从 O(output_size) 降低到 O(input_tile_size)
 *
 * 2. 双线性插值优化：
 *    - 在共享内存上直接进行插值计算，避免重复访问全局内存
 *    - 插值权重计算：w00 = (1-dx)*(1-dy), w01 = dx*(1-dy), etc.
 *    - 最终值 = w00*p00 + w01*p01 + w10*p10 + w11*p11
 *
 * 3. 内存合并访问：
 *    - 线程块内按行优先顺序访问，提高缓存命中率
 *    - 使用 __restrict__ 指针修饰符，告诉编译器没有指针别名
 *
 * 【参数说明】
 * @param input_bgr   输入图像指针（BGR 格式，uint8，行优先，连续内存）
 *                    - 内存布局：[H, W, C]，每像素 3 字节
 *                    - 必须通过 cudaMalloc 分配，确保内存连续
 * @param in_w        输入图像宽度（像素数）
 * @param in_h        输入图像高度（像素数）
 * @param output_nchw 输出张量指针（NCHW 格式，float32）
 *                    - 内存布局：[C, H, W]，Channel 0/1/2 对应 R/G/B 或 B/G/R
 *                    - 总大小：3 * out_w * out_h * sizeof(float)
 * @param out_w       输出宽度（模型输入尺寸，如 640）
 * @param out_h       输出高度（模型输入尺寸，如 640）
 * @param scale       缩放因子（letterbox 计算得出）
 *                    - 计算公式：scale = min(out_w/in_w, out_h/in_h)
 * @param pad_t       顶部填充像素数
 *                    - 计算公式：pad_t = (out_h - in_h * scale) / 2
 * @param pad_l       左侧填充像素数
 *                    - 计算公式：pad_l = (out_w - in_w * scale) / 2
 * @param norm        归一化因子（通常为 1.0f / 255.0f）
 *                    - 将像素值从 [0, 255] 归一化到 [0.0, 1.0]
 *                    - 数值稳定性：使用 float 避免整数除法精度损失
 * @param swap_rb     是否交换 R 和 B 通道
 *                    - true:  输出 RGB 格式（Channel 0=R, 1=G, 2=B）
 *                    - false: 输出 BGR 格式（Channel 0=B, 1=G, 2=R）
 *
 * 【返回值】
 * 无返回值（CUDA kernel）
 *
 * 【潜在副作用】
 * - 修改 output_nchw 指向的 GPU 内存
 * - 需要在 kernel 执行前确保输入数据已传输到 GPU
 *
 * 【与其他模块的交互】
 * - 被 launch_letterbox_shared() 调用
 * - 输入数据通常来自相机驱动或图像解码模块
 * - 输出数据供神经网络推理引擎使用（如 TensorRT、ONNX Runtime）
 */
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

/**
 * @brief Pitch 内存版本的 Letterbox 预处理 kernel
 *
 * 【功能说明】
 * 针对非连续内存布局（pitch 内存）的 Letterbox 实现。
 * 主要用于处理 cv::Mat ROI 或带有行对齐填充的图像数据。
 *
 * 【Pitch 内存概念】
 * Pitch（步长）是指图像每行在内存中占用的字节数。
 * 在某些情况下，pitch > width * channels，原因包括：
 * 1. 内存对齐：GPU 内存通常要求行首地址对齐到特定字节边界（如 256 字节）
 * 2. ROI 副本：cv::Mat 的 ROI 是原矩阵的子区域，行之间不连续
 * 3. 性能优化：对齐的内存访问更快
 *
 * 示例：
 *   图像尺寸：640x480，3 通道
 *   实际行字节数：640 * 3 = 1920 字节
 *   Pitch（对齐到 256）：1920 -> 2048 字节（padding 128 字节）
 *
 * 【与连续内存版本的区别】
 * 1. 不使用共享内存优化（因为 pitch 内存不适合连续的 tile 加载）
 * 2. 直接从全局内存读取，通过 pitch 计算行起始地址
 * 3. Block 大小为 16x16（而非 32x32），适应更小的 tile
 *
 * 【参数说明】
 * @param d_input_bgr 输入图像指针（GPU 内存，pitch 布局）
 * @param pitch       每行的字节步长（包括填充字节）
 *                    - 注意：单位是字节（byte），不是像素
 * @param src_w       源图像宽度（像素数）
 * @param src_h       源图像高度（像素数）
 * @param d_nchw      输出张量指针（GPU 内存，NCHW 格式）
 * @param OUT_W       输出宽度
 * @param OUT_H       输出高度
 * @param scale       缩放因子
 * @param pad_t       顶部填充
 * @param pad_l       左侧填充
 * @param norm        归一化因子
 * @param swap_rb     是否交换 R/B 通道
 *
 * 【实现注意事项】
 * - 访问像素时使用：row_ptr = base + y * pitch + x * 3
 * - pitch 是字节数，乘以 y 得到行偏移
 * - x * 3 是因为每个像素 3 字节（BGR）
 */
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

/// ============================================================================
/// Kernel 启动函数
/// ============================================================================

/**
 * @brief 启动连续内存版本的 Letterbox kernel
 *
 * 【功能说明】
 * 封装 CUDA kernel 启动逻辑，使用 cudaLaunchKernel API 而非 <<<>>> 语法。
 * 这种方式对 clang++ 编译器更友好，避免某些编译器兼容性问题。
 *
 * 【CUDA Grid/Block 配置】
 * - Block 大小：(TILE_W, TILE_H) = (32, 32) 个线程
 * - Grid 大小：((out_w + TILE_W - 1) / TILE_W, (out_h + TILE_H - 1) / TILE_H)
 * - 计算：向上取整，确保覆盖所有输出像素
 * - 示例：out_w=640, TILE_W=32 -> grid.x = 20 个 block
 *
 * 【参数说明】
 * @param d_input  输入图像 GPU 指针
 * @param in_w     输入宽度
 * @param in_h     输入高度
 * @param d_output 输出张量 GPU 指针
 * @param out_w    输出宽度
 * @param out_h    输出高度
 * @param scale    缩放因子
 * @param pad_t    顶部填充
 * @param pad_l    左侧填充
 * @param norm     归一化因子
 * @param swap_rb  是否交换通道
 * @param stream   CUDA 流（用于异步执行和流同步）
 *
 * 【使用示例】
 * @code
 * cudaStream_t stream;
 * cudaStreamCreate(&stream);
 * launch_letterbox_shared(d_input, 1920, 1080, d_output, 640, 640,
 *                          scale, pad_t, pad_l, 1.0f/255.0f, true, stream);
 * cudaStreamSynchronize(stream);
 * @endcode
 *
 * 【潜在副作用】
 * - 在指定的 CUDA 流上异步执行 kernel
 * - 调用方需要通过 cudaStreamSynchronize 或事件同步等待完成
 */
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
    // 构造 kernel 参数数组（cudaLaunchKernel 要求 void** 类型）
    void* args[] = {
        const_cast<unsigned char**>(&d_input),  // 输入指针（需要 const_cast）
        &in_w, &in_h,                            // 输入尺寸
        &d_output,                               // 输出指针
        &out_w, &out_h,                          // 输出尺寸
        &scale, &pad_t, &pad_l,                  // Letterbox 参数
        &norm, &swap_rb                          // 预处理参数
    };

    // 配置 CUDA 执行配置
    dim3 block(TILE_W, TILE_H);  // 每个 block 32x32 个线程
    dim3 grid((out_w + TILE_W - 1) / TILE_W, (out_h + TILE_H - 1) / TILE_H);  // Grid 大小

    // 使用 cudaLaunchKernel API 启动 kernel
    // 优势：编译器兼容性好，参数传递更灵活
    // 参数：kernel 函数指针，grid 大小，block 大小，参数数组，共享内存大小，CUDA 流
    cudaLaunchKernel(
        reinterpret_cast<const void*>(letterbox_kernel_shared),
        grid, block, args, 0, stream
    );
}

/**
 * @brief 启动 Pitch 内存版本的 Letterbox kernel
 *
 * 【功能说明】
 * 针对非连续内存布局的 kernel 启动封装。
 * Block 大小为 16x16（比连续版本小），原因：
 * 1. Pitch 内存访问不如连续内存高效，减少每个 block 的工作量
 * 2. 16x16 = 256 个线程，适合中等粒度的并行任务
 *
 * 【参数说明】
 * @param d_input  输入图像 GPU 指针（pitch 布局）
 * @param pitch    每行字节步长
 * @param src_w    源宽度
 * @param src_h    源高度
 * @param d_output 输出张量 GPU 指针
 * @param out_w    输出宽度
 * @param out_h    输出高度
 * @param scale    缩放因子
 * @param pad_t    顶部填充
 * @param pad_l    左侧填充
 * @param norm     归一化因子
 * @param swap_rb  是否交换通道
 * @param stream   CUDA 流
 *
 * 【使用场景】
 * - 处理 cv::Mat 的 ROI（Region of Interest）
 * - 处理 cudaMallocPitch 分配的内存
 * - 处理有内存对齐的图像数据
 */
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
    // 构造 kernel 参数数组
    void* args[] = {
        const_cast<unsigned char**>(&d_input),
        &pitch,                                   // Pitch 参数（字节步长）
        &src_w, &src_h,
        &d_output,
        &out_w, &out_h,
        &scale, &pad_t, &pad_l,
        &norm, &swap_rb
    };

    // 配置 CUDA 执行配置（16x16 block）
    dim3 block(16, 16);  // 每个 block 16x16 个线程
    dim3 grid((out_w + 15) / 16, (out_h + 15) / 16);  // Grid 大小

    // 启动 kernel
    cudaLaunchKernel(
        reinterpret_cast<const void*>(letterbox_kernel_pitched),
        grid, block, args, 0, stream
    );
}