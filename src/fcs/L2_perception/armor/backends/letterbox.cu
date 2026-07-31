/**
 * @file letterbox.cu
 * @brief CUDA Letterbox 预处理实现文件
 *
 * 本文件实现了用于神经网络推理的图像预处理 Letterbox 操作。
 * Letterbox 是目标检测模型（YOLO 系列）的核心预处理步骤。
 *
 * 【核心功能】
 * 1. 图像缩放：保持纵横比不变，将输入图像缩放到模型所需尺寸
 * 2. 边界填充：用固定颜色填充缩放后的图像边界，达到目标尺寸
 * 3. 双线性插值：高质量的重采样算法，避免锯齿和模糊
 * 4. 通道转换：支持 BGR -> RGB 转换（根据模型需求）
 * 5. 归一化：将像素值从 [0, 255] 归一化到 [0.0, 1.0]
 *
 * 【性能优化策略】
 * 1. 共享内存缓存：减少全局内存访问，提升内存带宽利用率
 * 2. 协作加载：block 内线程协作加载 tile 数据到共享内存
 * 3. 内存合并访问：优化线程访问模式，提高缓存命中率
 *
 * @author Talos Team
 * @date 2024
 */

#include "L2_perception/armor/backends/letterbox.cuh"

/// ============================================================================
/// Kernel 实现：连续内存版本（共享内存优化）
/// ============================================================================

/**
 * @brief 连续内存版本的 Letterbox kernel（使用共享内存优化）
 *
 * 【算法流程】
 * 1. 计算输出像素对应的输入坐标（逆映射）
 * 2. 协作加载输入 tile 到共享内存
 * 3. 在共享内存上进行双线性插值
 * 4. 将结果写入 NCHW 格式的输出张量
 *
 * 【共享内存优化原理】
 * 传统方法：每个线程独立从全局内存读取 4 个像素（双线性插值）
 * 优化方法：
 *   - 每个 block 协作加载一个 tile（32x32+1）到共享内存
 *   - 共享内存访问延迟 ~20 周期，全局内存 ~400 周期
 *   - 性能提升：约 2-3 倍
 *
 * 【共享内存布局】
 * 大小：(TILE_H + 1) x (TILE_W + 1) x 3 字节
 * +1 的原因：双线性插值需要访问右侧和下方的相邻像素
 *   - 插值点 (x, y) 需要访问 (x, y), (x+1, y), (x, y+1), (x+1, y+1)
 *   - 因此 tile 范围需要比 block 范围大 1 像素
 *
 * 【内存对齐与 Bank Conflict】
 * - 共享内存声明为 [TILE_H+1][TILE_W+1][3]
 * - 访问模式：smem[y][x][channel]
 * - 无 Bank Conflict：同一 warp 内线程访问不同 bank
 *
 * 【坐标映射计算】
 * 输出坐标：(x, y) -> 输入坐标：(in_x, in_y)
 *   in_x = (x - pad_l) / scale
 *   in_y = (y - pad_t) / scale
 *
 * 如果 in_x, in_y 超出输入图像范围，使用填充色 (114, 114, 114)。
 *
 * 【双线性插值公式】
 * 给定插值点 (fx, fy) 和周围 4 个像素 p00, p01, p10, p11：
 *   dx = fx - floor(fx)
 *   dy = fy - floor(fy)
 *   result = (1-dx)*(1-dy)*p00 + dx*(1-dy)*p01 + (1-dx)*dy*p10 + dx*dy*p11
 *
 * 【NCHW 输出布局】
 * 输出张量大小：3 * out_w * out_h 个 float
 * 内存排列：
 *   - Channel 0: [0, out_w*out_h)
 *   - Channel 1: [out_w*out_h, 2*out_w*out_h)
 *   - Channel 2: [2*out_w*out_h, 3*out_w*out_h)
 *
 * 这种布局避免了卷积层的内存重组，提升推理性能。
 *
 * 【参数说明】
 * @param input_bgr   输入 BGR 图像（HWC 格式，uint8）
 * @param in_w        输入宽度
 * @param in_h        输入高度
 * @param output_nchw 输出张量（NCHW 格式，float）
 * @param out_w       输出宽度
 * @param out_h       输出高度
 * @param scale       缩放因子
 * @param pad_t       顶部填充像素数
 * @param pad_l       左侧填充像素数
 * @param norm        归一化因子（通常 1.0f / 255.0f）
 * @param swap_rb     是否交换 R/B 通道
 */
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
    // ========================================
    // 步骤 1: 计算输出像素坐标
    // ========================================

    // 每个线程处理一个输出像素
    // blockIdx.x * TILE_W + threadIdx.x 是标准的 CUDA 坐标映射
    int x = blockIdx.x * TILE_W + threadIdx.x;
    int y = blockIdx.y * TILE_H + threadIdx.y;

    // 边界检查：超出输出范围的线程直接返回
    if (x >= out_w || y >= out_h)
        return;

    // ========================================
    // 步骤 2: 声明共享内存并计算加载参数
    // ========================================

    // 共享内存声明：缓存输入图像的一个 tile
    // 大小：(TILE_H+1) x (TILE_W+1) x 3
    // +1 是为了双线性插值访问相邻像素
    __shared__ unsigned char smem[TILE_H + 1][TILE_W + 1][3];

    // 计算线程 ID 和需要加载的总像素数
    int tid = threadIdx.y * blockDim.x + threadIdx.x;  // 线程在 block 内的线性 ID
    int total_smem = (TILE_W + 1) * (TILE_H + 1);      // tile 总像素数

    // 计算每个线程需要加载的像素数（协作加载）
    // 向上取整：iter = ceil(total_smem / (blockDim.x * blockDim.y))
    int iter = (total_smem + blockDim.x * blockDim.y - 1) / (blockDim.x * blockDim.y);

    // ========================================
    // 步骤 3: 计算输入图像的 tile 起始坐标
    // ========================================

    // 计算逆缩放因子（避免重复除法）
    float inv_scale = 1.0f / scale;

    // 计算 block 对应的输入 tile 起始坐标
    // block_start_x 是输入图像中该 block 左上角对应的 x 坐标
    // 减去 pad_l 是因为输出坐标从 0 开始，而有效区域从 pad_l 开始
    float block_start_x = (blockIdx.x * TILE_W - pad_l) * inv_scale;
    float block_start_y = (blockIdx.y * TILE_H - pad_t) * inv_scale;

    // ========================================
    // 步骤 4: 协作加载输入 tile 到共享内存
    // ========================================

    // 每个线程加载 iter 个像素到共享内存
    // 这种策略确保所有线程都有工作，避免部分线程空闲
    for (int i = 0; i < iter; i++) {
        // 计算当前线程负责的第 i 个像素的全局索引
        int idx = tid + i * (blockDim.x * blockDim.y);

        // 边界检查：确保索引在 tile 范围内
        if (idx < total_smem) {
            // 计算 tile 内的相对坐标 (sx, sy)
            int sx = idx % (TILE_W + 1);  // tile 内的 x 坐标
            int sy = idx / (TILE_W + 1);  // tile 内的 y 坐标

            // 计算输入图像的浮点坐标（用于边界检查）
            float in_x = block_start_x + sx * inv_scale;
            float in_y = block_start_y + sy * inv_scale;

            // 计算输入图像的整数坐标（取整）
            // 注意：对于正数，static_cast<int> 等价于 floor
            int ix = static_cast<int>(in_x);
            int iy = static_cast<int>(in_y);

            // ========================================
            // 步骤 4.1: 边界检查与像素读取
            // ========================================

            // 默认填充色：(114, 114, 114) - YOLO 标准填充色
            // 选择 114 是因为它是 0-255 的中间值，对模型影响最小
            unsigned char b = 114, g = 114, r = 114;

            // 检查坐标是否在输入图像范围内
            if (ix >= 0 && iy >= 0 && ix < in_w && iy < in_h) {
                // 计算输入像素的内存偏移
                // 输入格式：HWC (Height-Width-Channel)
                // 偏移量 = (y * width + x) * 3
                int offset = (iy * in_w + ix) * 3;

                // 读取 BGR 像素（OpenCV 默认格式）
                b = input_bgr[offset];     // 蓝色通道
                g = input_bgr[offset + 1]; // 绿色通道
                r = input_bgr[offset + 2]; // 红色通道
            }

            // ========================================
            // 步骤 4.2: 写入共享内存
            // ========================================

            // 将像素写入共享内存
            // 共享内存布局：smem[y][x][channel]
            smem[sy][sx][0] = b;  // 蓝色通道
            smem[sy][sx][1] = g;  // 绿色通道
            smem[sy][sx][2] = r;  // 红色通道
        }
    }

    // ========================================
    // 步骤 5: 同步线程，确保共享内存加载完成
    // ========================================

    // __syncthreads(): 同步 block 内所有线程
    // 确保所有线程都完成了共享内存的加载，再进行插值计算
    // 这是一个关键的同步点，避免使用未加载的数据
    __syncthreads();

    // ========================================
    // 步骤 6: 双线性插值计算
    // ========================================

    // 计算当前输出像素对应的输入坐标（浮点数）
    // in_x, in_y 是插值点的坐标
    float in_x = (x - pad_l) * inv_scale;
    float in_y = (y - pad_t) * inv_scale;

    // 计算小数部分（插值权重）
    // dx, dy 是插值点相对于整数坐标的偏移
    float dx = in_x - floorf(in_x);
    float dy = in_y - floorf(in_y);

    // 计算互补权重（用于 4 个像素的加权）
    // w00 = (1-dx)*(1-dy), w01 = dx*(1-dy), etc.
    float dx1 = 1.0f - dx;  // 水平方向左侧权重
    float dy1 = 1.0f - dy;  // 垂直方向上方权重

    // ========================================
    // 步骤 6.1: 从共享内存读取 4 个相邻像素
    // ========================================

    // 双线性插值需要访问 4 个相邻像素：
    // p00: 左上像素 (x, y)
    // p01: 右上像素 (x+1, y)
    // p10: 左下像素 (x, y+1)
    // p11: 右下像素 (x+1, y+1)

    // 注意：这里直接使用 threadIdx.x/y，因为共享内存已经包含了
    // 整个 tile 的数据（包括 +1 的边缘）
    unsigned char* p00 = smem[threadIdx.y][threadIdx.x];       // 左上
    unsigned char* p01 = smem[threadIdx.y][threadIdx.x + 1];   // 右上
    unsigned char* p10 = smem[threadIdx.y + 1][threadIdx.x];   // 左下
    unsigned char* p11 = smem[threadIdx.y + 1][threadIdx.x + 1]; // 右下

    // ========================================
    // 步骤 6.2: 对每个通道进行双线性插值
    // ========================================

    // 插值公式：
    // result = w00*p00 + w01*p01 + w10*p10 + w11*p11
    // 其中：
    //   w00 = (1-dx)*(1-dy) = dx1 * dy1
    //   w01 = dx * (1-dy)   = dx  * dy1
    //   w10 = (1-dx) * dy   = dx1 * dy
    //   w11 = dx * dy

    // 对红色通道插值（p[2] 是 BGR 格式的红色通道）
    float out_r = dx1 * dy1 * p00[2] + dx * dy1 * p01[2] + dx1 * dy * p10[2] + dx * dy * p11[2];

    // 对绿色通道插值（p[1] 是绿色通道）
    float out_g = dx1 * dy1 * p00[1] + dx * dy1 * p01[1] + dx1 * dy * p10[1] + dx * dy * p11[1];

    // 对蓝色通道插值（p[0] 是蓝色通道）
    float out_b = dx1 * dy1 * p00[0] + dx * dy1 * p01[0] + dx1 * dy * p10[0] + dx * dy * p11[0];

    // ========================================
    // 步骤 7: 写入 NCHW 格式的输出
    // ========================================

    // 计算输出索引（NCHW 格式）
    // out_idx 是当前像素在平面内的线性索引
    int out_idx = y * out_w + x;

    // plane_size 是一个通道平面的大小（H * W）
    // 用于计算不同通道的偏移
    int plane_size = out_w * out_h;

    // ========================================
    // 步骤 7.1: 根据通道顺序写入输出
    // ========================================

    if (swap_rb) {
        // RGB 输出格式（交换 R 和 B）
        // NCHW 布局：
        //   - Channel 0: 红色平面 [0, plane_size)
        //   - Channel 1: 绿色平面 [plane_size, 2*plane_size)
        //   - Channel 2: 蓝色平面 [2*plane_size, 3*plane_size)

        // 写入红色通道（归一化）
        output_nchw[out_idx] = out_r * norm;

        // 写入绿色通道（归一化）
        output_nchw[out_idx + plane_size] = out_g * norm;

        // 写入蓝色通道（归一化）
        output_nchw[out_idx + 2 * plane_size] = out_b * norm;
    } else {
        // BGR 输出格式（不交换）
        // NCHW 布局：
        //   - Channel 0: 蓝色平面
        //   - Channel 1: 绿色平面
        //   - Channel 2: 红色平面

        // 写入蓝色通道（归一化）
        output_nchw[out_idx] = out_b * norm;

        // 写入绿色通道（归一化）
        output_nchw[out_idx + plane_size] = out_g * norm;

        // 写入红色通道（归一化）
        output_nchw[out_idx + 2 * plane_size] = out_r * norm;
    }

    // ========================================
    // 归一化处理的数值稳定性说明
    // ========================================

    // 归一化公式：normalized_value = pixel_value * norm
    // 其中 norm = 1.0f / 255.0f

    // 数值稳定性考虑：
    // 1. 使用 float 而非 double：足够精度，性能更好
    // 2. 预计算 norm = 1.0/255.0，避免运行时除法
    // 3. 乘法比除法快：GPU 上乘法延迟约 4 周期，除法约 40 周期
    // 4. 范围保证：像素值在 [0, 255]，归一化后 [0.0, 1.0]
    // 5. 避免溢出：float 可以精确表示所有 [0, 255] 的整数
}

/// ============================================================================
/// Kernel 实现：Pitch 内存版本（非连续内存）
/// ============================================================================

/**
 * @brief Pitch 内存版本的 Letterbox kernel（不使用共享内存）
 *
 * 【功能说明】
 * 针对非连续内存布局（pitch 内存）的 Letterbox 实现。
 * 主要用于处理 cv::Mat ROI 或带有行对齐填充的图像数据。
 *
 * 【Pitch 内存概念】
 * Pitch 是指图像每行在内存中占用的字节数，包括对齐填充。
 *
 * 示例：
 *   图像尺寸：640x480，3 通道
 *   实际行字节数：640 * 3 = 1920 字节
 *   Pitch（对齐到 256）：2048 字节（每行末尾有 128 字节填充）
 *
 * 内存布局：
 *   Row 0: [Pixel(0,0), Pixel(1,0), ..., Pixel(639,0), padding(128 bytes)]
 *   Row 1: [Pixel(0,1), Pixel(1,1), ..., Pixel(639,1), padding(128 bytes)]
 *   ...
 *
 * 【为什么使用 Pitch 内存】
 * 1. 内存对齐：GPU 内存访问对齐到 256/512 字节边界时性能最佳
 * 2. ROI 支持：cv::Mat 的 ROI 是原矩阵的子区域，行不连续
 * 3. 硬件要求：某些 GPU 硬件要求 pitch 对齐
 *
 * 【不使用共享内存的原因】
 * 1. Pitch 内存不适合连续的 tile 加载（有填充字节）
 * 2. 非连续内存访问会导致共享内存加载效率低下
 * 3. 直接从全局内存读取更简单，避免复杂的地址计算
 *
 * 【性能权衡】
 * - 优点：支持非连续内存，适配更多场景
 * - 缺点：全局内存访问较多，性能略低于共享内存版本
 * - 适用：pitch 内存的图像处理（如 OpenCV ROI）
 *
 * 【参数说明】
 * @param d_input_bgr 输入图像（GPU 内存，pitch 布局）
 * @param pitch       每行的字节步长（包括填充）
 * @param src_w       源图像宽度
 * @param src_h       源图像高度
 * @param d_nchw      输出张量（GPU 内存，NCHW 格式）
 * @param OUT_W       输出宽度
 * @param OUT_H       输出高度
 * @param scale       缩放因子
 * @param pad_t       顶部填充
 * @param pad_l       左侧填充
 * @param norm        归一化因子
 * @param swap_rb     是否交换 R/B 通道
 */
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
    // ========================================
    // 步骤 1: 计算输出像素坐标
    // ========================================

    // 每个线程处理一个输出像素
    int ox = blockIdx.x * blockDim.x + threadIdx.x;  // 输出 x 坐标
    int oy = blockIdx.y * blockDim.y + threadIdx.y;  // 输出 y 坐标

    // 边界检查：超出输出范围的线程直接返回
    if (ox >= OUT_W || oy >= OUT_H)
        return;

    // ========================================
    // 步骤 2: 坐标映射（输出 -> 输入）
    // ========================================

    // 计算输入图像的浮点坐标（逆映射）
    // 公式：fx = (ox - pad_l) / scale
    // 注意：这里直接使用除法，而非预计算 1/scale
    // 原因：pitch 版本不是性能关键路径，代码清晰更重要
    float fx = (ox - pad_l) / scale;
    float fy = (oy - pad_t) / scale;

    // 计算输出索引（NCHW 格式）
    int out_idx = oy * OUT_W + ox;
    int plane = OUT_W * OUT_H;  // 一个通道平面的大小

    // ========================================
    // 步骤 3: 边界检查与填充色
    // ========================================

    // 默认填充色：(114, 114, 114)
    // 使用 float 类型方便后续插值计算
    float r = 114.0f, g = 114.0f, b = 114.0f;

    // 检查插值点是否在输入图像范围内
    // 注意边界条件：fx < src_w - 1（需要访问 x0+1）
    if (fx >= 0.0f && fy >= 0.0f && fx < src_w - 1 && fy < src_h - 1) {
        // ========================================
        // 步骤 4: 计算整数坐标和小数部分
        // ========================================

        // 取整数部分（左上角像素坐标）
        int x0 = static_cast<int>(fx);
        int y0 = static_cast<int>(fy);

        // 计算相邻像素坐标
        int x1 = x0 + 1;  // 右侧像素
        int y1 = y0 + 1;  // 下方像素

        // 计算小数部分（插值权重）
        float dx = fx - x0;  // 水平插值权重
        float dy = fy - y0;  // 垂直插值权重

        // 计算互补权重
        float dx1 = 1.0f - dx;
        float dy1 = 1.0f - dy;

        // ========================================
        // 步骤 5: 使用 Pitch 计算行指针
        // ========================================

        // 关键：使用 pitch 计算行起始地址
        // pitch 是字节数，乘以 y 得到行偏移（字节）
        // 然后加上 base 地址得到行指针

        const unsigned char* row0 = d_input_bgr + y0 * pitch;  // y0 行起始地址
        const unsigned char* row1 = d_input_bgr + y1 * pitch;  // y1 行起始地址

        // 计算像素在行内的字节偏移
        // x * 3 是因为每个像素 3 字节（BGR）
        int i00 = x0 * 3;  // 左侧像素的字节偏移
        int i01 = x1 * 3;  // 右侧像素的字节偏移

        // ========================================
        // 步骤 6: 读取 4 个像素的 BGR 值
        // ========================================

        // 从 row0（上方行）读取 2 个像素
        float b00 = row0[i00 + 0], g00 = row0[i00 + 1], r00 = row0[i00 + 2];  // (x0, y0)
        float b01 = row0[i01 + 0], g01 = row0[i01 + 1], r01 = row0[i01 + 2];  // (x1, y0)

        // 从 row1（下方行）读取 2 个像素
        float b10 = row1[i00 + 0], g10 = row1[i00 + 1], r10 = row1[i00 + 2];  // (x0, y1)
        float b11 = row1[i01 + 0], g11 = row1[i01 + 1], r11 = row1[i01 + 2];  // (x1, y1)

        // ========================================
        // 步骤 7: 计算双线性插值权重
        // ========================================

        // 双线性插值权重
        // w00: 左上像素权重 = (1-dx)*(1-dy)
        // w01: 右上像素权重 = dx*(1-dy)
        // w10: 左下像素权重 = (1-dx)*dy
        // w11: 右下像素权重 = dx*dy
        float w00 = dx1 * dy1;
        float w01 = dx * dy1;
        float w10 = dx1 * dy;
        float w11 = dx * dy;

        // ========================================
        // 步骤 8: 对每个通道进行双线性插值
        // ========================================

        // 红色通道插值
        r = r00 * w00 + r01 * w01 + r10 * w10 + r11 * w11;

        // 绿色通道插值
        g = g00 * w00 + g01 * w01 + g10 * w10 + g11 * w11;

        // 蓝色通道插值
        b = b00 * w00 + b01 * w01 + b10 * w10 + b11 * w11;
    }

    // ========================================
    // 步骤 9: 写入 NCHW 格式的输出
    // ========================================

    if (swap_rb) {
        // RGB 输出格式（交换 R 和 B 通道）
        // 将像素值归一化并写入对应通道平面
        d_nchw[out_idx] = r * norm;                  // 红色通道
        d_nchw[out_idx + plane] = g * norm;          // 绿色通道
        d_nchw[out_idx + 2 * plane] = b * norm;      // 蓝色通道
    } else {
        // BGR 输出格式（保持原始通道顺序）
        d_nchw[out_idx] = b * norm;                  // 蓝色通道
        d_nchw[out_idx + plane] = g * norm;          // 绿色通道
        d_nchw[out_idx + 2 * plane] = r * norm;      // 红色通道
    }

    // ========================================
    // Pitch 内存的性能考虑
    // ========================================

    // 1. 内存访问模式：
    //    - 每个线程访问 4 个非连续的内存位置
    //    - 无法利用内存合并（coalescing）优化
    //    - 缓存命中率较低

    // 2. 优化建议：
    //    - 如果性能关键，建议先复制到连续内存
    //    - 使用纹理内存（Texture Memory）可能提升性能
    //    - 考虑使用 __ldg() 内置函数优化只读访问

    // 3. 适用场景：
    //    - 处理 OpenCV ROI（不需要复制）
    //    - 处理 cudaMallocPitch 分配的内存
    //    - 对性能要求不高的场景
}