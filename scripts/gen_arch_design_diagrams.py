#!/usr/bin/env python3
"""Talos 核心架构设计可视化 — 三视角架构图生成器

生成三张专业架构图到 docs/architecture/：
1. talos_arch_overview.png     — 整体架构图（分层视图：硬件→原语→内核→流水线→应用 + 横向支撑）
2. talos_arch_components.png   — 组件关系图（Scheduler / System / World / DAG 内部组件关系）
3. talos_arch_dataflow.png     — 数据流程图（5 级流水线数据流 + SPMC 三缓冲机制 + 可视化解耦订阅）

采用行业标准架构图符号：
- 圆角矩形 = 组件 / 模块
- 圆柱体   = 数据存储 / 通道
- 菱形     = 决策 / 分支
- 实线箭头 = 数据流 / 调用
- 虚线箭头 = 依赖 / 订阅

内容数据已逐条对照源码核实。
"""

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon, Ellipse, Rectangle
from matplotlib import font_manager

# ---------- 字体 ----------
for f in font_manager.fontManager.ttflist:
    if "NotoSansCJK" in f.name or "Noto Sans CJK" in f.name:
        matplotlib.rcParams["font.family"] = f.name
        break
matplotlib.rcParams["axes.unicode_minus"] = False

OUTDIR = "/home/pldx/Desktop/talos-master/docs/architecture"
os.makedirs(OUTDIR, exist_ok=True)

# ---------- 配色（专业架构图色板） ----------
C = {
    "bg": "#FFFFFF",
    "text": "#1F2937",
    "sub": "#6B7280",
    "edge": "#9CA3AF",
    # 层级背景
    "app_bg": "#FEF3C7",        # 应用层 - 暖黄
    "fcs_bg": "#DBEAFE",        # FCS流水线 - 浅蓝
    "kernel_bg": "#EDE9FE",     # 调度内核 - 浅紫
    "comm_bg": "#CFFAFE",       # 通信原语 - 浅青
    "hw_bg": "#E2E8F0",         # 硬件层 - 浅灰
    # 组件填色
    "app_fill": "#FCD34D",
    "fcs_fill": "#60A5FA",
    "kernel_fill": "#A78BFA",
    "comm_fill": "#22D3EE",
    "hw_fill": "#94A3B8",
    "viz_fill": "#FB923C",
    "channel_fill": "#F8FAFC",
    # 强调
    "accent_blue": "#1D4ED8",
    "accent_green": "#059669",
    "accent_purple": "#7C3AED",
    "accent_orange": "#EA580C",
    "accent_red": "#DC2626",
    "accent_cyan": "#0891B2",
}


# ============================================================================
# 辅助绘图函数（行业标准符号）
# ============================================================================
def box(ax, x, y, w, h, text, fill="#F8FAFC", edge="#475569", lw=1.4, fs=8.5,
        fc="#1F2937", bold=True, style="round,pad=0.01", ha="center", va="center",
        italic=False, zorder=2):
    """圆角矩形组件 + 文本（行业标准：组件/模块符号）"""
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=style, facecolor=fill,
                                edgecolor=edge, linewidth=lw, zorder=zorder))
    ax.text(x + w / 2 if ha == "center" else x + 0.01, y + h / 2 if va == "center" else y + h - 0.02,
            text, ha=ha, va=va, fontsize=fs, color=fc,
            fontweight="bold" if bold else "normal", style="italic" if italic else "normal",
            zorder=zorder + 1)


def cylinder(ax, cx, cy, w, h, text, fill="#F8FAFC", edge="#475569", lw=1.4, fs=7.5,
             fc="#1F2937", zorder=2):
    """圆柱体 = 数据存储 / 通道（行业标准：数据存储符号）"""
    eh = w * 0.18
    # 主体矩形
    ax.add_patch(Rectangle((cx - w / 2, cy - h / 2 + eh / 2), w, h - eh,
                           facecolor=fill, edgecolor=edge, linewidth=lw, zorder=zorder))
    # 底部椭圆
    ax.add_patch(Ellipse((cx, cy - h / 2 + eh / 2), w, eh, facecolor=fill,
                         edgecolor=edge, linewidth=lw, zorder=zorder))
    # 顶部椭圆（实线 + 浅填充）
    ax.add_patch(Ellipse((cx, cy + h / 2 - eh / 2), w, eh, facecolor=fill,
                         edgecolor=edge, linewidth=lw, zorder=zorder + 1))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fs, color=fc,
            fontweight="bold", zorder=zorder + 2)


def diamond(ax, cx, cy, w, h, text, fill="#FEF3C7", edge="#D97706", lw=1.4, fs=7.5,
            fc="#1F2937", zorder=2):
    """菱形 = 决策 / 分支（行业标准：决策符号）"""
    pts = [(cx, cy + h / 2), (cx + w / 2, cy), (cx, cy - h / 2), (cx - w / 2, cy)]
    ax.add_patch(Polygon(pts, closed=True, facecolor=fill, edgecolor=edge,
                         linewidth=lw, zorder=zorder))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fs, color=fc,
            fontweight="bold", zorder=zorder + 1)


def arrow(ax, x1, y1, x2, y2, color="#475569", lw=1.6, ls="-", label=None,
          lx=None, ly=None, lcolor=None, fs=7.0, above=True, zorder=3, ms=12):
    """实线/虚线箭头 = 数据流 / 依赖"""
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                 mutation_scale=ms, color=color, linewidth=lw,
                                 linestyle=ls, zorder=zorder))
    if label:
        if lx is None:
            lx = (x1 + x2) / 2
        if ly is None:
            ly = (y1 + y2) / 2 + (0.015 if above else -0.015)
        ax.text(lx, ly, label, ha="center", va="bottom" if above else "top",
                fontsize=fs, color=lcolor or color,
                bbox=dict(boxstyle="round,pad=0.18", fc="white", ec="none", alpha=0.92),
                zorder=zorder + 1)


def layer_bg(ax, x, y, w, h, title, fill, ec="#9CA3AF", title_color=None, fs=10):
    """层级背景条 + 标题"""
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.008",
                                facecolor=fill, edgecolor=ec, linewidth=1.0,
                                alpha=0.35, zorder=0))
    ax.text(x + 0.012, y + h - 0.025, title, ha="left", va="top", fontsize=fs,
            fontweight="bold", color=title_color or ec, zorder=1)


def verify(fig, name):
    """重叠 / 越界检测"""
    rend = fig.canvas.get_renderer()
    fw, fh = fig.get_size_inches() * fig.dpi
    n_over = n_oob = 0
    for ax in fig.axes:
        texts = [t for t in ax.texts if t.get_text().strip()]
        boxes = [t.get_window_extent(rend) for t in texts]
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                b1, b2 = boxes[i], boxes[j]
                if b1.overlaps(b2):
                    area = max(0, min(b1.x1, b2.x1) - max(b1.x0, b2.x0)) * \
                           max(0, min(b1.y1, b2.y1) - max(b1.y0, b2.y0))
                    if area > 80:
                        n_over += 1
                        print(f"  [{name}] OVERLAP: '{texts[i].get_text()[:20]}' x "
                              f"'{texts[j].get_text()[:20]}' area={area:.0f}")
            bb = boxes[i]
            if bb.x0 < 0 or bb.y0 < 0 or bb.x1 > fw or bb.y1 > fh:
                n_oob += 1
                print(f"  [{name}] OOB: '{texts[i].get_text()[:20]}'")
    print(f"[{name}] texts={len([t for ax in fig.axes for t in ax.texts])} "
          f"overlap>{80}px²: {n_over}  oob: {n_oob}")


# ============================================================================
# 图 1：整体架构图（分层视图）
# ============================================================================
def draw_overview(out_name, figsize=(22, 16)):
    fig = plt.figure(figsize=figsize, facecolor=C["bg"])
    ax = fig.add_axes([0.02, 0.02, 0.96, 0.96])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    ax.text(0.5, 0.985, "Talos 整体架构图 — 分层视图", ha="center", va="top",
            fontsize=17, fontweight="bold", color=C["text"])
    ax.text(0.5, 0.955, "五层垂直堆叠（硬件 → 原语 → 内核 → 流水线 → 应用）+ 右侧横向支撑维度",
            ha="center", va="top", fontsize=9.5, color=C["sub"])

    LX, LW = 0.02, 0.74      # 主分层区
    SX, SW = 0.78, 0.20      # 右侧支撑区

    # ---------- 主分层（自上而下） ----------
    # L5 应用层
    layer_bg(ax, LX, 0.86, LW, 0.085, "① 应用层 / 主程序入口", C["app_bg"], C["app_fill"])
    box(ax, LX + 0.02, 0.865, 0.18, 0.045, "main.cpp\nsignal_handler + init_logger",
        C["app_fill"], C["app_fill"], fs=7.5, fc="#1F2937")
    box(ax, LX + 0.22, 0.865, 0.16, 0.045, "fcs::boot\nmove(config)",
        C["app_fill"], C["app_fill"], fs=7.5, fc="#1F2937")
    box(ax, LX + 0.40, 0.865, 0.16, 0.045, "scheduler.build()\n拓扑 + bind + 冻结",
        C["app_fill"], C["app_fill"], fs=7.5, fc="#1F2937")
    box(ax, LX + 0.58, 0.865, 0.14, 0.045, "scheduler.run()\n定频线程启动",
        C["app_fill"], C["app_fill"], fs=7.5, fc="#1F2937")
    arrow(ax, LX + 0.20, 0.887, LX + 0.22, 0.887, C["app_fill"], lw=1.4, ms=10)
    arrow(ax, LX + 0.38, 0.887, LX + 0.40, 0.887, C["app_fill"], lw=1.4, ms=10)
    arrow(ax, LX + 0.56, 0.887, LX + 0.58, 0.887, C["app_fill"], lw=1.4, ms=10)

    # L4 FCS 五级流水线
    layer_bg(ax, LX, 0.68, LW, 0.16, "② FCS 五级流水线（业务架构）", C["fcs_bg"], C["fcs_fill"])
    fcs_layers = [
        ("L1 采集", "camera_reader\nfixed_rate<250Hz>", "#D1FAE5", "#059669"),
        ("L2 感知", "armor / rune / ldm\n200Hz×2 (det+solver)", "#DBEAFE", "#2563EB"),
        ("L3 估计", "tracker / EKF\nfixed_rate<250Hz>", "#E0E7FF", "#4F46E5"),
        ("L4 规划", "aimer / MPC / 弹道\nfixed_rate<250Hz>", "#FEF3C7", "#B45309"),
        ("L5 武器", "fire_ctrl / 开火门\nfixed_rate<250Hz>", "#FEE2E2", "#BE123C"),
        ("执行器", "weapon_output\n云台 / 摩擦轮", "#EDE9FE", "#6D28D9"),
    ]
    fw_w, fw_gap = 0.105, 0.014
    fx = LX + 0.02
    for i, (lbl, sub, fill, ec) in enumerate(fcs_layers):
        box(ax, fx, 0.71, fw_w, 0.11, f"{lbl}\n{sub}", fill, ec, lw=1.5, fs=7.2, fc="#1F2937")
        if i < len(fcs_layers) - 1:
            arrow(ax, fx + fw_w + 0.001, 0.765, fx + fw_w + fw_gap - 0.001, 0.765,
                  ec, lw=1.8, ms=11)
        fx += fw_w + fw_gap

    # L3 调度内核
    layer_bg(ax, LX, 0.46, LW, 0.20, "③ 调度内核层（ECS 式数据流调度）", C["kernel_bg"], C["kernel_fill"])
    box(ax, LX + 0.02, 0.50, 0.17, 0.13,
        "World\n资源 / 通道容器\nResourceStore\n+ ChannelStore",
        "#EDE9FE", C["kernel_fill"], lw=1.6, fs=7.5, fc="#1F2937")
    box(ax, LX + 0.21, 0.50, 0.17, 0.13,
        "System\nlambda + 组件\nspsc / spmc\nres / local",
        "#EDE9FE", C["kernel_fill"], lw=1.6, fs=7.5, fc="#1F2937")
    box(ax, LX + 0.40, 0.50, 0.17, 0.13,
        "Scheduler\n执行策略引擎\nboot → build\n→ run",
        "#EDE9FE", C["kernel_fill"], lw=1.6, fs=7.5, fc="#1F2937")
    box(ax, LX + 0.59, 0.50, 0.17, 0.13,
        "DAG 依赖分析\n拓扑 / 分层\n唤醒掩码\n(上限 64)",
        "#EDE9FE", C["kernel_fill"], lw=1.6, fs=7.5, fc="#1F2937")
    arrow(ax, LX + 0.19, 0.565, LX + 0.21, 0.565, C["kernel_fill"], lw=1.5, ms=10)
    arrow(ax, LX + 0.38, 0.565, LX + 0.40, 0.565, C["kernel_fill"], lw=1.5, ms=10)
    arrow(ax, LX + 0.57, 0.565, LX + 0.59, 0.565, C["kernel_fill"], lw=1.5, ms=10)

    # L2 通信与原语
    layer_bg(ax, LX, 0.28, LW, 0.16, "④ 通信与原语层", C["comm_bg"], C["comm_fill"])
    primitives = [
        ("SPMC 三缓冲\nshared_ptr 快照\n+ generation", "#CFFAFE", C["comm_fill"]),
        ("SPSC 三缓冲\n点对点通信\n(保留 / 未用)", "#CFFAFE", C["comm_fill"]),
        ("RAII Owner\n资源生命周期\n禁止裸指针", "#CFFAFE", C["comm_fill"]),
        ("性能探针 / spin\n线程亲和性 / lazy\noverloaded", "#CFFAFE", C["comm_fill"]),
    ]
    pw = (LW - 0.04 - 0.03 * 3) / 4
    px = LX + 0.02
    for lbl, fill, ec in primitives:
        box(ax, px, 0.31, pw, 0.11, lbl, fill, ec, lw=1.4, fs=7.3, fc="#1F2937")
        px += pw + 0.01

    # L1 硬件层
    layer_bg(ax, LX, 0.10, LW, 0.16, "⑤ 硬件抽象与驱动层", C["hw_bg"], C["hw_fill"])
    box(ax, LX + 0.02, 0.13, 0.17, 0.11, "HIK 相机驱动\nhik_camera_driver\n(Linux x86_64)",
        "#E2E8F0", C["hw_fill"], lw=1.4, fs=7.0, fc="#1F2937")
    box(ax, LX + 0.21, 0.13, 0.17, 0.11, "云台控制\nat_gimbal\n(串口 / termios)",
        "#E2E8F0", C["hw_fill"], lw=1.4, fs=7.0, fc="#1F2937")
    box(ax, LX + 0.40, 0.13, 0.17, 0.11, "hardware_daedalus\n共享内存抽象层\n(进程间 IPC)",
        "#E2E8F0", C["hw_fill"], lw=1.4, fs=7.0, fc="#1F2937")
    box(ax, LX + 0.59, 0.13, 0.17, 0.11, "摩擦轮 / 发射\n数字 IO / 编码器\n状态回读",
        "#E2E8F0", C["hw_fill"], lw=1.4, fs=7.0, fc="#1F2937")

    # ---------- 层间数据流（左侧粗箭头） ----------
    for y1, y2 in [(0.86, 0.84), (0.68, 0.66), (0.46, 0.44), (0.28, 0.26)]:
        arrow(ax, LX + 0.005, y1, LX + 0.005, y2, C["edge"], lw=2.2, ms=14)
    # 回流（运行时数据上行）
    arrow(ax, LX + LW - 0.005, 0.26, LX + LW - 0.005, 0.84, C["accent_green"],
          lw=1.8, ls="--", ms=12, label="运行时数据上行\n(采集 → 武器执行)",
          lx=LX + LW + 0.012, ly=0.55, lcolor=C["accent_green"], fs=7.5)

    # ---------- 右侧横向支撑维度 ----------
    ax.add_patch(FancyBboxPatch((SX, 0.10), SW, 0.84, boxstyle="round,pad=0.008",
                                facecolor="#F8FAFC", edgecolor=C["edge"], linewidth=1.2,
                                alpha=0.6, zorder=0))
    ax.text(SX + SW / 2, 0.925, "横向支撑维度", ha="center", va="top",
            fontsize=11, fontweight="bold", color=C["text"])

    # 支撑 1: fast_tf
    box(ax, SX + 0.01, 0.83, SW - 0.02, 0.075,
        "fast_tf 类型安全坐标变换\nworld / odom / gimbal /\ncamera_optical / muzzle\n(编译期防误用)",
        "#E0F2FE", C["accent_cyan"], lw=1.5, fs=7.3, fc="#1F2937")

    # 支撑 2: ADT 编程范式
    box(ax, SX + 0.01, 0.735, SW - 0.02, 0.075,
        "ADT 编程范式\nstruct = product type\nvariant = sum type\nexpected = 可恢复失败",
        "#EDE9FE", C["accent_purple"], lw=1.5, fs=7.3, fc="#1F2937")

    # 支撑 3: Parse Don't Validate
    box(ax, SX + 0.01, 0.64, SW - 0.02, 0.075,
        "Parse, Don't Validate\n边界数据先解析为强类型\nTOML / 相机参数 / PnP\n核心域拒绝半合法数据",
        "#FEE2E2", C["accent_red"], lw=1.5, fs=7.3, fc="#1F2937")

    # 支撑 4: 可视化解耦
    box(ax, SX + 0.01, 0.545, SW - 0.02, 0.075,
        "Foxglove 可视化解耦\npool_compute 数据事件触发\n并行订阅主链路通道\n→ WebSocket / MCAP",
        "#FED7AA", C["accent_orange"], lw=1.5, fs=7.3, fc="#1F2937")

    # 支撑 5: 标定
    box(ax, SX + 0.01, 0.45, SW - 0.02, 0.075,
        "标定模块（离线）\n相机内参 / ChArUco\n棋盘格 / 手眼标定\n(独立于运行时)",
        "#D1FAE5", C["accent_green"], lw=1.5, fs=7.3, fc="#1F2937")

    # 支撑 6: Chiral 数据采集
    box(ax, SX + 0.01, 0.355, SW - 0.02, 0.075,
        "Chiral 数据采集 / 记录\n离线数据集构建\n回放 / 调试 / 回归测试\n(独立于运行时)",
        "#FEF3C7", "#D97706", lw=1.5, fs=7.3, fc="#1F2937")

    # 支撑 7: 配置
    box(ax, SX + 0.01, 0.26, SW - 0.02, 0.075,
        "TOML 配置系统\nfcs / camera / foxglove\n各模块独立加载\nfail-fast 缺失即退出",
        "#F1F5F9", C["hw_fill"], lw=1.5, fs=7.3, fc="#1F2937")

    # 支撑 8: 日志
    box(ax, SX + 0.01, 0.165, SW - 0.02, 0.075,
        "spdlog 日志系统\n分级输出 INFO/WARN/ERROR\n结构化日志 / 异步刷新\n(全局共享)",
        "#F1F5F9", C["hw_fill"], lw=1.5, fs=7.3, fc="#1F2937")

    # 图例
    legend = [
        (C["app_fill"], "应用层"),
        (C["fcs_fill"], "FCS 流水线"),
        (C["kernel_fill"], "调度内核"),
        (C["comm_fill"], "通信原语"),
        (C["hw_fill"], "硬件层"),
    ]
    handles = [mpatches.Rectangle((0, 0), 1, 1, color=c, label=lbl) for c, lbl in legend]
    fig.legend(handles=handles, loc="lower center", ncol=5, fontsize=9,
               bbox_to_anchor=(0.5, 0.005), fancybox=True, frameon=False)

    out = os.path.join(OUTDIR, out_name)
    fig.canvas.draw()
    verify(fig, out_name)
    plt.savefig(out, dpi=130, bbox_inches="tight", facecolor=C["bg"])
    plt.close(fig)
    print(f"saved: {out}")


# ============================================================================
# 图 2：组件关系图（调度器内部组件）
# ============================================================================
def draw_components(out_name, figsize=(22, 16)):
    fig = plt.figure(figsize=figsize, facecolor=C["bg"])
    ax = fig.add_axes([0.02, 0.04, 0.96, 0.92])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    ax.text(0.5, 0.985, "Talos 组件关系图 — 调度内核内部结构", ha="center", va="top",
            fontsize=17, fontweight="bold", color=C["text"])
    ax.text(0.5, 0.958, "Scheduler ↔ System ↔ World ↔ DAG 的组件关系与依赖流向",
            ha="center", va="top", fontsize=9.5, color=C["sub"])

    # ---------- 中心：Scheduler ----------
    box(ax, 0.38, 0.82, 0.24, 0.10,
        "Scheduler\n执行策略引擎 · 调度核心",
        "#EDE9FE", C["accent_purple"], lw=2.0, fs=10, fc="#1F2937")

    # ---------- 上方：执行策略（3 种） ----------
    ax.text(0.5, 0.78, "执行策略（线程模型）", ha="center", fontsize=10,
            fontweight="bold", color=C["accent_purple"])
    policies = [
        ("fixed_rate<Freq,CPU,Prio>", "独占线程定频触发\n通知下游\n200/250Hz 主链路",
         "#D1FAE5", C["accent_green"], 0.10),
        ("fixed_rate_silent", "独占线程定频\n不通知下游\n高频静默 (IMU)",
         "#DBEAFE", C["accent_blue"], 0.30),
        ("pool_compute", "TBB 线程池\n位掩码依赖触发\n辅助系统 (可视化)",
         "#FED7AA", C["accent_orange"], 0.50),
        ("pool_visualization", "可视化专用线程池\n(预留)",
         "#FEE2E2", C["accent_red"], 0.66),
    ]
    for title, sub, fill, ec, cx in policies:
        box(ax, cx - 0.08, 0.68, 0.16, 0.085, f"{title}\n{sub}", fill, ec,
            lw=1.5, fs=6.8, fc="#1F2937")
        arrow(ax, 0.50, 0.82, cx, 0.765, C["accent_purple"], lw=1.4, ls="--", ms=10)

    # ---------- 中部：System（lambda + 组件） ----------
    box(ax, 0.38, 0.52, 0.24, 0.10,
        "System\nlambda 函数 + 组件访问\nrun(World&) 执行逻辑",
        "#DBEAFE", C["accent_blue"], lw=2.0, fs=9, fc="#1F2937")
    arrow(ax, 0.40, 0.82, 0.40, 0.62, C["accent_purple"], lw=2.0, ms=14,
          label="调度执行", lx=0.40, ly=0.805, lcolor=C["accent_purple"], fs=7.5)

    # System 的四类组件
    ax.text(0.5, 0.495, "System 组件类型（编译期声明依赖）", ha="center", fontsize=10,
            fontweight="bold", color=C["accent_blue"])
    comps = [
        ("通道组件", "spsc<T,Tag>  SPSC 只读\nspsc_mut<T,Tag>  SPSC 只写\nspmc<T,Tag>  SPMC 只读\nspmc_mut<T,Tag>  SPMC 只写",
         "#CFFAFE", C["accent_cyan"], 0.10),
        ("资源组件", "res<T>  只读资源\nres_mut<T>  可写资源\n(版本追踪 / 自增)\nlocal<T>  系统本地变量",
         "#E0F2FE", "#0E7490", 0.30),
        ("访问语义", "bind()  预创建通道\nrun()   执行系统逻辑\n只读 / 只写分离\n(数据流单向)",
         "#F1F5F9", C["hw_fill"], 0.50),
        ("依赖分析", "组件类型即依赖\nspsc/spsc_mut 配对\nres_mut 写后版本自增\n触发下游重算",
         "#FEE2E2", C["accent_red"], 0.66),
    ]
    for title, sub, fill, ec, cx in comps:
        box(ax, cx - 0.08, 0.36, 0.16, 0.115, f"{title}\n{sub}", fill, ec,
            lw=1.4, fs=6.5, fc="#1F2937")
        arrow(ax, 0.50, 0.52, cx, 0.475, C["accent_blue"], lw=1.2, ls="--", ms=9)

    # ---------- 下方：World（容器） ----------
    box(ax, 0.34, 0.20, 0.32, 0.10,
        "World\n资源 + 通道的根容器\nUniqueAny 类型擦除存储",
        "#EDE9FE", C["accent_purple"], lw=2.0, fs=9, fc="#1F2937")
    arrow(ax, 0.40, 0.52, 0.40, 0.30, C["accent_blue"], lw=2.0, ms=14,
          label="组件访问 World", lx=0.40, ly=0.505, lcolor=C["accent_blue"], fs=7.5)

    # World 内部两大数据存储
    ax.text(0.5, 0.175, "World 内部存储", ha="center", fontsize=10,
            fontweight="bold", color=C["accent_purple"])
    # ChannelStore（圆柱体 = 数据存储）
    cylinder(ax, 0.22, 0.115, 0.18, 0.10,
             "ChannelStore\nSpscStorage<T>\nSpmcStorage<T>\n(通道实例容器)",
             fill="#CFFAFE", edge=C["accent_cyan"], lw=1.5, fs=6.8, fc="#1F2937")
    # ResourceStore（圆柱体）
    cylinder(ax, 0.50, 0.115, 0.18, 0.10,
             "ResourceStore\nResource<T>\n版本号追踪\nemplace_resource / get<>",
             fill="#E0F2FE", edge="#0E7490", lw=1.5, fs=6.8, fc="#1F2937")
    # Registry（圆柱体）
    cylinder(ax, 0.78, 0.115, 0.18, 0.10,
             "Registry\n系统注册表\nadd_system<T>(...)\n(系统工厂容器)",
             fill="#F1F5F9", edge=C["hw_fill"], lw=1.5, fs=6.8, fc="#1F2937")
    arrow(ax, 0.40, 0.20, 0.28, 0.165, C["accent_purple"], lw=1.2, ls=":", ms=9)
    arrow(ax, 0.50, 0.20, 0.50, 0.165, C["accent_purple"], lw=1.2, ls=":", ms=9)
    arrow(ax, 0.60, 0.20, 0.72, 0.165, C["accent_purple"], lw=1.2, ls=":", ms=9)

    # ---------- 三阶段生命周期（右侧） ----------
    ax.text(0.86, 0.925, "三阶段生命周期", ha="center", fontsize=10,
            fontweight="bold", color=C["accent_red"])
    stages = [
        ("boot()", "注册系统\nadd_system × N\n注入资源", 0.86, 0.82, "#D1FAE5", C["accent_green"]),
        ("build()", "拓扑校验 + bind\n分层 + 唤醒掩码\n冻结拓扑（不可变）", 0.86, 0.68, "#DBEAFE", C["accent_blue"]),
        ("run()", "启动定频线程\npool_compute 就绪\nshutdown_watcher", 0.86, 0.54, "#FEE2E2", C["accent_red"]),
    ]
    for title, sub, cx, cy, fill, ec in stages:
        box(ax, cx - 0.10, cy, 0.20, 0.10, f"{title}\n{sub}", fill, ec,
            lw=1.5, fs=7.0, fc="#1F2937")
    arrow(ax, 0.86, 0.82, 0.86, 0.78, C["accent_red"], lw=1.6, ms=11)
    arrow(ax, 0.86, 0.68, 0.86, 0.64, C["accent_red"], lw=1.6, ms=11)
    # run → Scheduler 回流
    arrow(ax, 0.76, 0.87, 0.62, 0.87, C["accent_red"], lw=1.4, ls="--", ms=10,
          label="驱动", lcolor=C["accent_red"], fs=7)

    # 图例
    legend = [
        (C["accent_purple"], "调度内核组件"),
        (C["accent_blue"], "System / 数据流"),
        (C["accent_cyan"], "通道存储"),
        ("#0E7490", "资源存储"),
        (C["accent_red"], "生命周期阶段"),
    ]
    handles = [mpatches.Rectangle((0, 0), 1, 1, color=c, label=lbl) for c, lbl in legend]
    fig.legend(handles=handles, loc="lower center", ncol=5, fontsize=9,
               bbox_to_anchor=(0.5, 0.005), fancybox=True, frameon=False)

    out = os.path.join(OUTDIR, out_name)
    fig.canvas.draw()
    verify(fig, out_name)
    plt.savefig(out, dpi=130, bbox_inches="tight", facecolor=C["bg"])
    plt.close(fig)
    print(f"saved: {out}")


# ============================================================================
# 图 3：数据流程图（运行时数据流 + 三缓冲机制 + 可视化解耦）
# ============================================================================
def draw_dataflow(out_name, figsize=(24, 16)):
    fig = plt.figure(figsize=figsize, facecolor=C["bg"])
    ax = fig.add_axes([0.02, 0.02, 0.96, 0.96])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    ax.text(0.5, 0.985, "Talos 数据流程图 — 运行时数据流与通信机制", ha="center", va="top",
            fontsize=17, fontweight="bold", color=C["text"])
    ax.text(0.5, 0.958, "上：五级 FCS 流水线数据流（5 × SPMC 通道）  |  下：三缓冲机制 + 可视化解耦订阅",
            ha="center", va="top", fontsize=9.5, color=C["sub"])

    # ========== 上区：五级流水线数据流 ==========
    layer_bg(ax, 0.01, 0.66, 0.98, 0.28, "运行时五级 FCS 流水线（fixed_rate 定频驱动 · 数据流左→右）",
             C["fcs_bg"], C["fcs_fill"])

    # 6 个系统盒子
    sys_boxes = [
        ("camera_reader", "fixed_rate<250Hz>", "① recv(1s) 取图\n写入 Image 通道",
         "#D1FAE5", C["accent_green"], 0.025),
        ("L2 armor", "200Hz × 2", "② detector: NN infer\n→ solver: PnP+BA\n★空检测 → 空 batch",
         "#DBEAFE", C["accent_blue"], 0.205),
        ("L3 tracker", "fixed_rate<250Hz>", "③ meas_in.read()\nEKF 预测-更新\n+ 数据关联",
         "#E0E7FF", "#4F46E5", 0.385),
        ("L4 aimer", "fixed_rate<250Hz>", "④ trk_in.read() + aim\n目标选择 + MPC\n+ 弹道 ★无目标→hold",
         "#FEF3C7", "#B45309", 0.565),
        ("L5 fire_ctrl", "fixed_rate<250Hz>", "⑤ intent_in.read()\nMPC + 开火门\n★超差 → 不开火",
         "#FEE2E2", C["accent_red"], 0.745),
        ("执行器", "weapon_output 250Hz", "⑥ 读 WeaponCommand\n云台/摩擦轮\n+ 状态回读",
         "#EDE9FE", "#6D28D9", 0.908),
    ]
    box_w = 0.085
    for name, freq, action, fill, ec, cx in sys_boxes:
        box(ax, cx - box_w / 2, 0.72, box_w, 0.18, f"{name}\n{freq}\n\n{action}",
            fill, ec, lw=1.6, fs=6.8, fc="#1F2937")

    # 5 个 SPMC 通道节点（圆柱体 = 数据存储）
    channels = [
        (0.115, "ImageTopic\nImageFrame", "图像帧\n相机 → L2"),
        (0.295, "Measurement\nArmorMeas", "装甲测量\nL2 → L3"),
        (0.475, "TrackerOut\nTrackerOutputs", "追踪输出\nL3 → L4"),
        (0.655, "CtrlIntent\nControlIntent", "控制意图\nL4 → L5"),
        (0.835, "WeaponCmd\nWeaponCommand", "武器指令\nL5 → 执行器"),
    ]
    for cx, label, sub in channels:
        cylinder(ax, cx, 0.81, 0.07, 0.10, label, fill=C["channel_fill"],
                 edge=C["accent_cyan"], lw=1.3, fs=5.8, fc="#1F2937")
        ax.text(cx, 0.745, sub, ha="center", va="center", fontsize=5.5,
                color=C["sub"], style="italic", zorder=3)

    # 系统间数据流箭头（穿过通道节点）
    sys_centers = [0.025, 0.205, 0.385, 0.565, 0.745, 0.908]
    for i in range(5):
        sx = sys_centers[i] + box_w / 2
        ex = sys_centers[i + 1] - box_w / 2
        cy = 0.81
        # writer → channel
        arrow(ax, sx + 0.001, cy, channels[i][0] - 0.035 - 0.001, cy,
              sys_boxes[i][4], lw=1.8, ms=11)
        # channel → reader
        arrow(ax, channels[i][0] + 0.035 + 0.001, cy, ex - 0.001, cy,
              sys_boxes[i + 1][4], lw=1.8, ms=11)

    # 执行器输出回流（到硬件）
    arrow(ax, 0.908 + box_w / 2, 0.81, 0.985, 0.81, "#6D28D9", lw=2.0, ms=12,
          label="→ 云台/摩擦轮", lx=0.97, ly=0.835, lcolor="#6D28D9", fs=7)

    # ========== 下区左：SPMC 三缓冲机制详图 ==========
    layer_bg(ax, 0.01, 0.30, 0.55, 0.32, "SPMC 三缓冲通信机制（无锁多读 · shared_ptr 快照）",
             C["comm_bg"], C["comm_fill"])

    # Writer
    box(ax, 0.03, 0.50, 0.10, 0.06, "Writer\n(生产者)", "#D1FAE5",
        C["accent_green"], lw=1.5, fs=7.5, fc="#1F2937")

    # 三个缓冲区（圆柱体）
    buf_labels = ["Buf 0", "Buf 1", "Buf 2"]
    buf_cols = ["#FEE2E2", "#FEF3C7", "#D1FAE5"]
    for i, (lbl, col) in enumerate(zip(buf_labels, buf_cols)):
        bx = 0.17 + i * 0.075
        cylinder(ax, bx, 0.53, 0.06, 0.08, lbl, fill=col, edge=C["comm_fill"],
                 lw=1.3, fs=6.5, fc="#1F2937")
        # writer → buffers
        arrow(ax, 0.13, 0.53, bx - 0.03 - 0.001, 0.53, C["accent_green"],
              lw=1.4, ms=10)

    # generation 版本号标签
    box(ax, 0.17, 0.42, 0.225, 0.05,
        "generation 版本号 · 原子递增 · 标识最新写入",
        "#F1F5F9", C["hw_fill"], lw=1.2, fs=7, fc="#1F2937")
    arrow(ax, 0.28, 0.49, 0.28, 0.47, C["sub"], lw=1.0, ls=":", ms=8)

    # shared_ptr 快照层
    box(ax, 0.17, 0.355, 0.225, 0.05,
        "shared_ptr 快照 · 引用计数 · 无拷贝传递",
        "#E0F2FE", C["accent_cyan"], lw=1.3, fs=7, fc="#1F2937")
    arrow(ax, 0.28, 0.42, 0.28, 0.405, C["sub"], lw=1.0, ls=":", ms=8)

    # Readers（多读）
    readers = [
        ("Reader 1\n(L3 tracker)", "#E0E7FF", "#4F46E5", 0.43),
        ("Reader 2\n(可视化)", "#FED7AA", C["accent_orange"], 0.505),
        ("Reader 3\n(其他订阅)", "#F1F5F9", C["hw_fill"], 0.58),
    ]
    for lbl, fill, ec, cx in readers:
        box(ax, cx - 0.045, 0.32, 0.09, 0.05, lbl, fill, ec, lw=1.3, fs=6.3, fc="#1F2937")
        arrow(ax, 0.28, 0.355, cx, 0.37, C["accent_cyan"], lw=1.2, ls="--", ms=9)

    # 关键特性标注
    box(ax, 0.42, 0.50, 0.13, 0.085,
        "关键特性\n• 无锁多读\n• 取最新帧\n• 消费端天然跳帧\n• 生产消费解耦",
        "#F8FAFC", C["comm_fill"], lw=1.4, fs=6.8, fc="#1F2937")

    # ========== 下区右：可视化解耦订阅 ==========
    layer_bg(ax, 0.58, 0.30, 0.41, 0.32, "Foxglove 可视化解耦订阅（pool_compute 数据事件触发）",
             "#FFF7ED", C["accent_orange"])

    # 5 个主链路通道（订阅源）
    src_channels = ["Image", "Detection", "Measurement", "Tracker", "Intent"]
    src_colors = [C["accent_green"], C["accent_blue"], "#4F46E5", "#B45309", C["accent_red"]]
    for i, (lbl, col) in enumerate(zip(src_channels, src_colors)):
        cx = 0.61 + i * 0.075
        cylinder(ax, cx, 0.575, 0.055, 0.07, lbl, fill=C["channel_fill"],
                 edge=col, lw=1.2, fs=5.8, fc="#1F2937")

    # 中间：pool_compute 可视化系统池
    box(ax, 0.69, 0.45, 0.20, 0.06,
        "pool_compute 可视化系统池\nfoxglove_* 系列 · TBB 线程池 · 位掩码触发",
        "#FED7AA", C["accent_orange"], lw=1.6, fs=7, fc="#1F2937")

    # 通道 → 可视化池（解耦订阅虚线）
    for i in range(5):
        cx = 0.61 + i * 0.075
        arrow(ax, cx, 0.54, 0.79, 0.51, C["accent_orange"], lw=1.0, ls=":", ms=8)
    ax.text(0.66, 0.535, "解耦订阅", ha="center", fontsize=6.5, color=C["accent_orange"],
            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.92), zorder=4)

    # 下层：场景构建 + 编码器
    box(ax, 0.61, 0.36, 0.16, 0.055,
        "SceneBuilder\n场景构建\n(3D / 文本 / 图像)",
        "#FED7AA", C["accent_orange"], lw=1.3, fs=6.5, fc="#1F2937")
    box(ax, 0.79, 0.36, 0.16, 0.055,
        "FoxgloveServer\nWebSocket / MCAP\n编码推送",
        "#FED7AA", C["accent_orange"], lw=1.3, fs=6.5, fc="#1F2937")
    arrow(ax, 0.79, 0.45, 0.69, 0.415, C["accent_orange"], lw=1.3, ms=10)
    arrow(ax, 0.77, 0.39, 0.79, 0.39, C["accent_orange"], lw=1.3, ms=10)

    # 终端
    box(ax, 0.69, 0.315, 0.20, 0.035,
        "Foxglove Studio · MCAP 文件 · 实时调试",
        "#F1F5F9", C["hw_fill"], lw=1.2, fs=6.5, fc="#1F2937")
    arrow(ax, 0.87, 0.36, 0.87, 0.35, C["accent_orange"], lw=1.2, ms=9)

    # ========== 底部：调度机制对照 ==========
    layer_bg(ax, 0.01, 0.04, 0.98, 0.22, "调度机制对照（主链路 vs 辅助系统）",
             "#F8FAFC", C["edge"])

    # 主链路
    box(ax, 0.03, 0.10, 0.44, 0.11,
        "主链路：fixed_rate 定频驱动\n\n"
        "• 独占线程 · sleep_until 定时触发\n"
        "• 250Hz: 相机 / L3 / L4 / L5 / 执行器   200Hz: L2 (det + solver)\n"
        "• read() 取最新帧 · 天然跳帧 · 低延迟\n"
        "• 写后通知下游 · 唤醒掩码（上限 64）",
        "#DBEAFE", C["accent_blue"], lw=1.5, fs=7.5, fc="#1F2937", ha="left")

    # 辅助系统
    box(ax, 0.53, 0.10, 0.44, 0.11,
        "辅助系统：pool_compute 数据事件触发\n\n"
        "• TBB 线程池 · 位掩码依赖触发\n"
        "• 可视化 foxglove_* 系列并行订阅主链路通道\n"
        "• 不参与定频主链路 · 不影响实时性\n"
        "• 解耦设计 · 故障隔离 · 独立扩缩",
        "#FED7AA", C["accent_orange"], lw=1.5, fs=7.5, fc="#1F2937", ha="left")

    out = os.path.join(OUTDIR, out_name)
    fig.canvas.draw()
    verify(fig, out_name)
    plt.savefig(out, dpi=130, bbox_inches="tight", facecolor=C["bg"])
    plt.close(fig)
    print(f"saved: {out}")


# ============================================================================
# 主入口
# ============================================================================
if __name__ == "__main__":
    draw_overview("talos_arch_overview.png")
    draw_components("talos_arch_components.png")
    draw_dataflow("talos_arch_dataflow.png")
