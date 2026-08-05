#!/usr/bin/env python3
"""Talos 全景架构图（深度细化版）生成器

单张总图五区布局，覆盖全部模块（含遗漏补充）：
- 顶部    ：启动流程 4 大阶段（配置 → 框架 → boot 初始化 → 构建运行）
- 左区    ：调度内核 + 原语层（Scheduler / System / World / DAG / 错误系统 / primitive / 三缓冲实现）
- 中区    ：主链路（L1→L5 逐系统深度展开：推理后端多态 / PnP+BA / EKF / MPC / 开火门）
             + 副链路（能量机关 rune→energy_meter / LDM → 汇入 L4）
- 右区    ：子系统带（Quanta 图传 / Chiral 采集回放 / 标定 4 件套 / 可视化 / fast_tf / ADT / 配置日志）
- 底部    ：通信机制（fixed_rate / SPMC 三缓冲 / shm 跨进程三缓冲 / pool_compute）

内容已逐条对照源码核实（含 backends 多态 / energy_meter / ldm / dual MPC / replay 等）。
"""

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Ellipse, Rectangle
from matplotlib import font_manager
from matplotlib.transforms import Bbox

for f in font_manager.fontManager.ttflist:
    if "NotoSansCJK" in f.name or "Noto Sans CJK" in f.name:
        matplotlib.rcParams["font.family"] = f.name
        break
matplotlib.rcParams["axes.unicode_minus"] = False

OUTDIR = "/home/pldx/Desktop/talos-master/docs/architecture"
os.makedirs(OUTDIR, exist_ok=True)

C = {
    "text": "#1F2937", "sub": "#6B7280", "edge": "#9CA3AF",
    "app": "#FCD34D", "fcs": "#60A5FA", "kernel": "#A78BFA",
    "comm": "#22D3EE", "hw": "#94A3B8", "viz": "#FB923C",
    "green": "#059669", "blue": "#1D4ED8", "purple": "#7C3AED",
    "yellow": "#D97706",
    "orange": "#EA580C", "red": "#DC2626", "cyan": "#0891B2",
}


def box(ax, x, y, w, h, text, fill="#F8FAFC", edge="#475569", lw=2.6, fs=18.8,
        fc="#1F2937", bold=True, ha="center", va="center", italic=False, zorder=2):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.006",
                                facecolor=fill, edgecolor=edge, linewidth=lw, zorder=zorder))
    tx = x + w / 2 if ha == "center" else x + 0.012
    ty = y + h / 2 if va == "center" else y + h - 0.018
    ax.text(tx, ty, text, ha=ha, va=va, fontsize=fs, color=fc,
            fontweight="bold" if bold else "normal",
            style="italic" if italic else "normal", zorder=zorder + 1)


def cylinder(ax, cx, cy, w, h, text, fill="#F8FAFC", edge="#475569", lw=2.6, fs=16.2,
             fc="#1F2937", zorder=2):
    eh = w * 0.18
    ax.add_patch(Rectangle((cx - w / 2, cy - h / 2 + eh / 2), w, h - eh,
                           facecolor=fill, edgecolor=edge, linewidth=lw, zorder=zorder))
    ax.add_patch(Ellipse((cx, cy - h / 2 + eh / 2), w, eh, facecolor=fill,
                         edgecolor=edge, linewidth=lw, zorder=zorder))
    ax.add_patch(Ellipse((cx, cy + h / 2 - eh / 2), w, eh, facecolor=fill,
                         edgecolor=edge, linewidth=lw, zorder=zorder + 1))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fs, color=fc,
            fontweight="bold", zorder=zorder + 2)


def arrow(ax, x1, y1, x2, y2, color="#475569", lw=3.0, ls="-", label=None,
          lx=None, ly=None, lcolor=None, fs=17.0, above=True, zorder=3, ms=22.0):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                 mutation_scale=ms, color=color, linewidth=lw,
                                 linestyle=ls, zorder=zorder))
    if label:
        if lx is None:
            lx = (x1 + x2) / 2
        if ly is None:
            ly = (y1 + y2) / 2 + (0.012 if above else -0.012)
        ax.text(lx, ly, label, ha="center", va="bottom" if above else "top",
                fontsize=fs, color=lcolor or color,
                bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.92),
                zorder=zorder + 1)


def _ov_area(b1, b2):
    return max(0, min(b1.x1, b2.x1) - max(b1.x0, b2.x0)) * \
           max(0, min(b1.y1, b2.y1) - max(b1.y0, b2.y0))


def verify(fig, name):
    rend = fig.canvas.get_renderer()
    fw, fh = fig.get_size_inches() * fig.dpi
    n_over = n_oob = n_bb = n_tbox = n_tout = 0
    for ax in fig.axes:
        texts = [t for t in ax.texts if t.get_text().strip()]
        boxes = [t.get_window_extent(rend) for t in texts]
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                b1, b2 = boxes[i], boxes[j]
                if b1.overlaps(b2):
                    area = _ov_area(b1, b2)
                    if area > 80:
                        n_over += 1
                        print(f"  [{name}] OVERLAP: '{texts[i].get_text()[:16]}' x "
                              f"'{texts[j].get_text()[:16]}' area={area:.0f}")
            bb = boxes[i]
            if bb.x0 < 0 or bb.y0 < 0 or bb.x1 > fw or bb.y1 > fh:
                n_oob += 1
                print(f"  [{name}] OOB: '{texts[i].get_text()[:16]}'")
        # 盒-盒重叠检测（用原始数据坐标，避开 pad 扩展；嵌套盒也会报出，人工甄别）
        patches = [p for p in ax.patches if isinstance(p, FancyBboxPatch)]
        pboxes = [ax.transData.transform_bbox(
                      Bbox([[p.get_x(), p.get_y()],
                            [p.get_x() + p.get_width(), p.get_y() + p.get_height()]]))
                  for p in patches]
        pboxes_r = [p.get_window_extent(rend) for p in patches]
        for i in range(len(pboxes)):
            for j in range(i + 1, len(pboxes)):
                b1, b2 = pboxes[i], pboxes[j]
                # 跳过完全嵌套（卡内子盒正常）
                inside = (b1.x0 <= b2.x0 and b1.y0 <= b2.y0 and b1.x1 >= b2.x1
                          and b1.y1 >= b2.y1) or (b2.x0 <= b1.x0 and b2.y0 <= b1.y0
                                                  and b2.x1 >= b1.x1 and b2.y1 >= b1.y1)
                if b1.overlaps(b2) and not inside:
                    area = _ov_area(b1, b2)
                    if area > 1500:
                        n_bb += 1
                        print(f"  [{name}] BOX-BOX area={area:.0f} "
                              f"A=({patches[i].get_x():.3f},{patches[i].get_y():.3f},"
                              f"{patches[i].get_width():.2f},{patches[i].get_height():.2f}) "
                              f"B=({patches[j].get_x():.3f},{patches[j].get_y():.3f},"
                              f"{patches[j].get_width():.2f},{patches[j].get_height():.2f})")
        # 文本-盒子遮蔽检测：文本中心在某盒子内却超出盒子边界，或文本压在非父盒子上
        for i, tb in enumerate(boxes):
            tc = ((tb.x0 + tb.x1) / 2, (tb.y0 + tb.y1) / 2)
            owner = None
            for j, pb in enumerate(pboxes_r):
                if pb.x0 <= tc[0] <= pb.x1 and pb.y0 <= tc[1] <= pb.y1:
                    owner = j
                    break
            if owner is not None:
                pb = pboxes_r[owner]
                # 文本超出所属盒子边界（每边容忍 18px）
                if (tb.x0 < pb.x0 - 18 or tb.y0 < pb.y0 - 18
                        or tb.x1 > pb.x1 + 18 or tb.y1 > pb.y1 + 18):
                    n_tout += 1
                    print(f"  [{name}] TEXT-OUT: '{texts[i].get_text()[:20]}' "
                          f"box=({patches[owner].get_x():.3f},{patches[owner].get_y():.3f},"
                          f"{patches[owner].get_width():.2f},{patches[owner].get_height():.2f})")
            else:
                # 中心不在任何盒内：检查是否压在非父盒上
                for j, pb in enumerate(pboxes_r):
                    if tb.overlaps(pb) and _ov_area(tb, pb) > 150:
                        n_tbox += 1
                        print(f"  [{name}] TEXT-BOX: '{texts[i].get_text()[:20]}' "
                              f"on box=({patches[j].get_x():.3f},{patches[j].get_y():.3f},"
                              f"{patches[j].get_width():.2f},{patches[j].get_height():.2f})")
    print(f"[{name}] texts={len([t for ax in fig.axes for t in ax.texts])} "
          f"overlap>{80}px²: {n_over}  oob: {n_oob}  box-box>{1500}px²: {n_bb}  "
          f"text-out: {n_tout}  text-box: {n_tbox}")


# ============================================================================
# 深度细化全景总图
# ============================================================================
def draw_master(out_name, figsize=(42, 36)):
    fig = plt.figure(figsize=figsize, facecolor="white")

    fig.text(0.5, 0.985, "Talos 全景架构总图（深度细化版）", ha="center", va="top",
             fontsize=62.5, fontweight="bold", color=C["text"])
    fig.text(0.5, 0.960, "主链路 × 副链路 × 调度内核 × 子系统带 × 通信机制 × 通道关系 — 全模块覆盖",
             ha="center", va="top", fontsize=27.5, color=C["sub"])

    # 调整布局：删除独立通道表，更多空间给下方机制卡
    ax_top = fig.add_axes([0.010, 0.870, 0.980, 0.080])
    ax_left = fig.add_axes([0.010, 0.340, 0.235, 0.515])
    ax_mid = fig.add_axes([0.260, 0.340, 0.480, 0.515])
    ax_right = fig.add_axes([0.755, 0.340, 0.235, 0.515])
    ax_bot = fig.add_axes([0.010, 0.020, 0.980, 0.310])

    for ax in (ax_top, ax_left, ax_mid, ax_right, ax_bot):
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.axis("off")

    # ==================================================================
    # ① 顶部：启动流程 4 大阶段
    # ==================================================================
    ax_top.text(0.5, 0.93, "启动流程 — 4 大阶段（main → boot → world）", ha="center",
                va="center", fontsize=21.2, fontweight="bold", color="#1F2937")
    stages = [
        ("M1 配置", "#059669", ["① signal + init_logger", "② load_config", "③ RuntimeConfig"],
         "★解析失败→退出"),
        ("M2 框架注册", "#7C3AED", ["⑤ Scheduler(cfg)", "⑥⑦ 注册 Foxglove", "⑧ systems"],
         "★WS/MCAP·失败降级"),
        ("M3 boot 初始化", "#1D4ED8", ["⑨ fcs::boot(move)", "⑩⑪ 后端+配置×5",
                                        "⑫⑬ 相机+内参", "⑭⑮ 资源+系统×8"],
         "★失败→return 1"),
        ("M4 构建运行", "#DC2626", ["⑯ build 冻结", "⑰ shutdown_watcher", "⑱ run()"],
         "★Ctrl+C→stop"),
    ]
    scx = [0.135, 0.385, 0.635, 0.885]
    sw = 0.215
    # ===== 配置链路：M1 左侧加 TOML 配置文件入口 =====
    # 1) TOML 文件 → M1 load_config（缩小并左移，避免与 M1 重叠）
    toml_x = 0.010
    cylinder(ax_top, toml_x + 0.025, 0.43, 0.050, 0.22, "TOML\n配置",
             fill="#F1F5F9", edge="#6B7280", lw=2.0, fs=13.0, fc="#374151")
    arrow(ax_top, toml_x + 0.050, 0.43, scx[0] - sw / 2 - 0.008, 0.43,
          C["hw"], lw=2.4, ms=16.0)
    # M1 旁加 load_config 代码锚点（放在 M1 盒下方，紧贴 ax_top 底部）
    ax_top.text(scx[0], 0.025,
                'fcs::load_config("at_vision.toml")  @ main.cpp:90',
                ha="center", va="center", fontsize=8.5, color="#1F2937",
                family="monospace",
                bbox=dict(boxstyle="round,pad=0.05", fc="#F3F4F6", ec="#D1D5DB", lw=0.8))
    for i, (title, ac, items, branch) in enumerate(stages):
        cx = scx[i]
        box(ax_top, cx - sw / 2, 0.06, sw, 0.72, "", "#FAFAFA", ac, lw=3.0, zorder=1)
        box(ax_top, cx - sw / 2, 0.60, sw, 0.18, title, ac, ac, lw=0.0, fs=20.0, zorder=2)
        # 3 条 items 横向 3 栏并排，占满框宽
        n_items = len(items)
        gap = 0.010
        item_x0 = cx - sw / 2 + 0.012
        item_w = (sw - 0.024 - gap * (n_items - 1)) / n_items
        item_h = 0.30
        item_y = 0.28
        for j, it in enumerate(items):
            ix = item_x0 + j * (item_w + gap)
            box(ax_top, ix, item_y, item_w, item_h, it, "#FFFFFF", ac,
                lw=1.8, fs=13.1, fc="#1F2937")
        # 底部退出/分支条件
        ax_top.text(cx, 0.15, branch, ha="center", va="center", fontsize=14.5,
                    color=C["red"], fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.13", fc="#FEE2E2", ec="none"))
        if i < 3:
            arrow(ax_top, cx + sw / 2 + 0.006, 0.43, scx[i + 1] - sw / 2 - 0.006, 0.43,
                  C["edge"], lw=2.8, ms=20.0)

    # ==================================================================
    # ② 左区：调度内核 + 原语层
    # ==================================================================
    ax_left.text(0.5, 0.975, "调度内核 + 原语层", ha="center", va="center",
                 fontsize=23.8, fontweight="bold", color=C["purple"])

    # Scheduler
    box(ax_left, 0.04, 0.895, 0.92, 0.06, "Scheduler 执行策略引擎",
        "#EDE9FE", C["purple"], lw=3.6, fs=18.8)
    pol = [
        ("fixed_rate<F,P,C>", "独占线程定频·通知", "#D1FAE5", C["green"], 0.02, 0.83),
        ("fixed_rate_silent", "定频·不通知 (IMU)", "#DBEAFE", C["blue"], 0.54, 0.83),
        ("pool_compute", "TBB 池·位掩码触发", "#FED7AA", C["orange"], 0.02, 0.755),
        ("pool_visualization", "可视化专用池", "#FEE2E2", C["red"], 0.54, 0.755),
    ]
    for title, sub, fill, ec, px, py in pol:
        box(ax_left, px, py, 0.44, 0.065, f"{title}\n{sub}", fill, ec,
            lw=2.4, fs=13.0, fc="#1F2937")
        arrow(ax_left, 0.50, 0.895, px + 0.22, py + 0.065, C["purple"],
              lw=1.8, ls="--", ms=14.0)

    # System
    box(ax_left, 0.04, 0.695, 0.92, 0.055,
        "System = lambda(组件参数) · 数据流节点\n"
        "bind() 预建通道/资源 · run(world) 执行业务",
        "#DBEAFE", C["blue"], lw=3.4, fs=14.5)
    arrow(ax_left, 0.50, 0.895, 0.50, 0.75, C["purple"], lw=3.2, ms=22.0,
          label="调度执行", lx=0.50, ly=0.87, fs=14.0)

    # components
    comps = [
        ("spsc / spsc_mut", "#CFFAFE", C["cyan"]),
        ("spmc / spmc_mut", "#CFFAFE", C["cyan"]),
        ("res / res_mut", "#E0F2FE", "#0E7490"),
        ("local", "#F1F5F9", C["hw"]),
    ]
    for i, (lbl, fill, ec) in enumerate(comps):
        px = 0.04 + i * 0.2632
        box(ax_left, px, 0.61, 0.22, 0.05, lbl, fill, ec, lw=2.2, fs=14.0, fc="#1F2937")
        arrow(ax_left, 0.50, 0.695, px + 0.11, 0.66, C["blue"], lw=1.8, ls=":", ms=14.0)
    ax_left.text(0.5, 0.672, "Topic 空标签类型(sizeof=0) · component_kind 编译期校验 · writer 带 written_flag_ 唤醒标记",
                 ha="center", va="center", fontsize=11.5, color=C["sub"])

    # DAG
    box(ax_left, 0.04, 0.55, 0.92, 0.055,
        "DAG 7 阶段：≤64 位掩码 · 通道采集/合法性 · 邻接掩码\n"
        "Kahn 拓扑分层 · 环检测 · 唤醒掩码 compute_affects_",
        "#F1F5F9", C["hw"], lw=2.8, fs=13.5, fc="#1F2937")
    arrow(ax_left, 0.50, 0.61, 0.50, 0.605, C["blue"], lw=2.4, ms=18.0)

    # World
    box(ax_left, 0.04, 0.49, 0.92, 0.05,
        "World：UniqueAny 类型擦除 · 组件访问根\n"
        "ChannelStore 通道 / ResourceStore 版本追踪资源",
        "#EDE9FE", C["purple"], lw=3.4, fs=13.5)
    arrow(ax_left, 0.50, 0.55, 0.50, 0.54, C["blue"], lw=2.8, ms=20.0)

    stores = [
        (0.165, "ChannelStore\nSpsc/Spmc", "#CFFAFE", C["cyan"]),
        (0.50, "ResourceStore\n版本追踪", "#E0F2FE", "#0E7490"),
        (0.835, "Registry\nadd_system", "#F1F5F9", C["hw"]),
    ]
    for cx, lbl, fill, ec in stores:
        cylinder(ax_left, cx, 0.4425, 0.19, 0.075, lbl, fill=fill, edge=ec,
                 lw=2.2, fs=13.0, fc="#1F2937")
        arrow(ax_left, 0.50, 0.49, cx, 0.48, C["purple"], lw=1.6, ls=":", ms=12.0)

    # ===== 配置链路：World 分发配置到各 System =====
    # 从 ResourceStore 向右画虚线箭头到中区系统（在左区内用延伸箭头示意）
    arrow(ax_left, 0.96, 0.4425, 0.995, 0.4425, "#0E7490", lw=2.6, ls="--", ms=16.0,
          label="res<Config> 分发", lx=0.98, ly=0.418, fs=10.0, lcolor="#0E7490")
    # ResourceStore 内的配置类型（上移避免与代码锚点重叠）
    ax_left.text(0.50, 0.418,
                 "FcsConfig · CameraConfig\nFoxgloveConfig · GimbalConfig",
                 ha="center", va="center", fontsize=9.0, color="#0369A1", style="italic")

    # 错误系统
    box(ax_left, 0.04, 0.34, 0.92, 0.05,
        "错误系统：error / error_formatter / demangle",
        "#FEE2E2", C["red"], lw=2.4, fs=15.5, fc="#1F2937")

    # ECS 数据流（组件写→掩码→唤醒→执行）
    box(ax_left, 0.04, 0.265, 0.92, 0.065,
        "ECS 数据流：write() → written_flag_ 置位 → notify(idx)\n"
        "→ ready_systems_.fetch_or → run_compute_loop 轮询\n"
        "→ run_compute_selective 分层并行 → 级联下游",
        "#FEF9C3", "#CA8A04", lw=2.6, fs=13.5, fc="#1F2937")

    # primitive
    box(ax_left, 0.04, 0.19, 0.92, 0.07,
        "primitive 原语：\nspin 自旋 · performance_probe 探针\nthread_affinity 亲和 · lazy · system_info",
        "#F1F5F9", C["hw"], lw=2.4, fs=14.0, fc="#1F2937")

    # 生命周期
    box(ax_left, 0.04, 0.115, 0.92, 0.07,
        "生命周期：boot 注册 → build 冻结拓扑 → run 启动\nfixed_rate 独占线程 + pool_compute 线程池",
        "#EDE9FE", C["purple"], lw=2.4, fs=14.0, fc="#1F2937")

    # ===== 代码锚点：左区底部"代码索引"区（集中展示，避免与盒内文字重叠）=====
    idx_y = 0.085
    box(ax_left, 0.04, 0.015, 0.92, 0.085, "", "#F9FAFB", "#9CA3AF", lw=1.8, zorder=1)
    ax_left.text(0.50, idx_y, "代码索引 Code Anchors", ha="center", va="center",
                 fontsize=11.0, fontweight="bold", color="#374151")
    idx_items = [
        ('class Scheduler {', "scheduler.hpp:128"),
        ('TopologySnapshot{ levels; affects; }', "scheduler.hpp:413"),
        ('class World {  // Store', "world.hpp:623"),
        ('spmc_mut<T, Topic>', "channel_topics.hpp"),
    ]
    for i, (code, path) in enumerate(idx_items):
        col = i % 2
        row = i // 2
        ix = 0.08 + col * 0.46
        iy = idx_y - 0.022 - row * 0.022
        ax_left.text(ix, iy, code, ha="left", va="center",
                     fontsize=7.5, color="#1F2937", family="monospace")
        ax_left.text(ix + 0.32, iy, f"@ {path}", ha="left", va="center",
                     fontsize=6.8, color="#6B7280", style="italic")

    # ==================================================================
    # ③ 中区：主链路（深度细化）+ 副链路
    # ==================================================================
    ax_mid.text(0.5, 0.975, "主链路 — 五级 FCS 流水线（深度细化）", ha="center",
                va="center", fontsize=23.8, fontweight="bold", color=C["blue"])
    ax_mid.text(0.5, 0.955, "fixed_rate 定频驱动 · SPMC 通道数据流 · pool_compute 分层并行",
                ha="center", va="center", fontsize=15.5, color=C["sub"])

    # ===== 配置链路：中区系统从 World 读取 res<Config> =====
    # 在主链路系统盒上方画一条从左进入的虚线箭头，示意所有 System 读配置
    arrow(ax_mid, 0.005, 0.785, 0.055, 0.785, "#0E7490", lw=2.6, ls="--", ms=16.0,
          label="res<Config> · 每 System 只读", lx=0.260, ly=0.938, fs=10.5, lcolor="#0E7490")

    # 层级徽标
    chips = [("L1", "#D1FAE5"), ("L2", "#DBEAFE"), ("L3", "#E0E7FF"),
             ("L4", "#FEF3C7"), ("L5", "#FEE2E2"), ("执行", "#EDE9FE")]
    for i, (lbl, col) in enumerate(chips):
        cx = 0.075 + i * 0.1904
        box(ax_mid, cx - 0.065, 0.90, 0.13, 0.035, lbl, col, C["edge"], lw=1.4, fs=15.0)

    # 系统盒（6） —— 内容校准：L2 双通道 / L4 FSM / L5 variant
    sys_boxes = [
        ("camera_reader", "250Hz", "#D1FAE5", C["green"],
         ["HIK 取图", "→ ImageFrame"], None),
        ("L2 armor", "200Hz×2", "#DBEAFE", C["blue"],
         ["img_in.read()", "检测→2D Det", "PnP+BA→3D Meas"],
         "★空检测→空 batch"),
        ("L3 tracker", "250Hz", "#E0E7FF", "#4F46E5",
         ["多实例 EKF", "数据关联", "预测→odom"], None),
        ("L4 aimer", "250Hz", "#FEF3C7", "#B45309",
         ["FSM 状态机", "目标+MPC", "→ ControlIntent variant"],
         "★无目标→hold"),
        ("L5 fire_ctrl", "250Hz", "#FEE2E2", C["red"],
         ["开火门控", "dual MPC", "→ WeaponCommand"],
         "★超差→不开火"),
        ("执行器", "250Hz", "#EDE9FE", "#6D28D9",
         ["gimbal 串口", "daedalus 共享内存", "状态回读"], None),
    ]
    bxs = [0.025, 0.195, 0.365, 0.535, 0.705, 0.875]
    bw = 0.14
    # ===== 通道数据流：详细 Topic 名称 + 生产者 + 消费者 =====
    # 每个系统之间展示：通道名(Topic) + 生产者 → 消费者
    channel_info = [
        # camera_reader → L2 systems
        {
            "topic": "ImageChannelTopic",
            "data": "ImageFrame",
            "producer": "camera_reader",
            "consumers": ["armor_detector", "ldm_detector", "rune_detector", "capturer", "stream_encoder"],
            "color": C["green"],
        },
        # L2 armor → L3 tracker
        {
            "topic": "DetectionChannelTopic",
            "data": "ArmorDetectionBatch (2D)",
            "producer": "armor_detector",
            "consumers": ["armor_solver"],
            "color": C["blue"],
        },
        {
            "topic": "MeasurementChannelTopic",
            "data": "ArmorMeasurementBatch (3D)",
            "producer": "armor_solver",
            "consumers": ["tracker"],
            "color": "#4F46E5",
        },
        # L3 tracker → L4 aimer
        {
            "topic": "TrackerOutputChannelTopic",
            "data": "TrackerOutputs",
            "producer": "tracker",
            "consumers": ["aimer"],
            "color": "#B45309",
        },
        # L4 aimer → L5 fire_control
        {
            "topic": "ControlIntentChannelTopic",
            "data": "ControlIntent (variant)",
            "producer": "aimer",
            "consumers": ["fire_control"],
            "color": C["red"],
        },
    ]

    for i, (name, freq, fill, ec, lines, branch) in enumerate(sys_boxes):
        cx = bxs[i] + bw / 2
        box(ax_mid, bxs[i], 0.58, bw, 0.30, "", fill, ec, lw=3.2, zorder=1)
        ax_mid.text(cx, 0.855, name, ha="center", va="center", fontsize=17.5,
                    fontweight="bold", color="#1F2937")
        ax_mid.text(cx, 0.828, freq, ha="center", va="center", fontsize=13.0,
                    color=C["sub"])
        ty = 0.795
        for ln in lines:
            ax_mid.text(cx, ty, ln, ha="center", va="center", fontsize=14.5, color="#374151")
            ty -= 0.045
        if branch:
            ax_mid.text(cx, 0.602, branch, ha="center", va="center", fontsize=13.5,
                        color=C["red"], fontweight="bold",
                        bbox=dict(boxstyle="round,pad=0.10", fc="#FEE2E2", ec="none"))
        
        # 系统间通道：圆柱体 + 上下双通道标注
        if i < 5:
            ccx = bxs[i] + bw + 0.017
            # 通道圆柱体（更大更明显）
            cylinder(ax_mid, ccx, 0.73, 0.034, 0.08, "", fill="#F8FAFC",
                     edge=C["cyan"], lw=2.8, fs=11.2, fc="#475569")
            
            ch = channel_info[i]
            # ===== 通道上标签：Topic 结构体名（在圆柱体上方）=====
            ax_mid.text(ccx, 0.805, ch["topic"], ha="center", va="center",
                        fontsize=7.2, color="#0E7490", family="monospace",
                        fontweight="bold",
                        bbox=dict(boxstyle="round,pad=0.05", fc="#ECFEFF", ec="#22D3EE", lw=1.2, zorder=4))
            
            # ===== 通道下标签：数据类型 + 消费者流向（在圆柱体下方）=====
            consumers_txt = " → " + ch["consumers"][0]
            if len(ch["consumers"]) > 1:
                consumers_txt += f" +{len(ch['consumers'])-1}消费者"
            ax_mid.text(ccx, 0.650, ch["data"], ha="center", va="center",
                        fontsize=8.2, color=ch["color"], fontweight="bold", zorder=4)
            ax_mid.text(ccx, 0.635, consumers_txt, ha="center", va="center",
                        fontsize=6.5, color="#475569", style="italic", zorder=4)
            
            # 数据流箭头（带颜色区分通道类型）
            ax_mid.add_patch(FancyArrowPatch((bxs[i] + bw, 0.73), (ccx - 0.017, 0.73),
                                             arrowstyle="-|>", mutation_scale=20, color=ch["color"],
                                             linewidth=3.4, zorder=3))
            ax_mid.add_patch(FancyArrowPatch((ccx + 0.017, 0.73),
                                             (bxs[i + 1] - 0.003, 0.73),
                                             arrowstyle="-|>", mutation_scale=20,
                                             color=sys_boxes[i + 1][3], linewidth=3.4, zorder=3))
    
    # ===== 执行器输出通道 =====
    ax_mid.add_patch(FancyArrowPatch((0.875 + bw, 0.73), (0.99, 0.73),
                                     arrowstyle="-|>", mutation_scale=26, color="#6D28D9", linewidth=4.8, zorder=3))
    # 在箭头上方（靠左，避开执行器盒内容）标注通道
    ax_mid.text(0.930, 0.812, "WeaponCommandChannelTopic", ha="center", va="center",
                fontsize=6.5, color="#6D28D9", family="monospace", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.05", fc="#FAF5FF", ec="#A78BFA", lw=1.2, zorder=4))
    ax_mid.text(0.930, 0.795, "WeaponCommand", ha="center", va="center",
                fontsize=7.2, color="#6D28D9", fontweight="bold")
    ax_mid.text(0.930, 0.778, "→ gimbal串口 / daedalus SHM", ha="center", va="center",
                fontsize=6.0, color="#475569", style="italic")

    # ===== 代码锚点：中区各系统盒底部（真实代码语句 + 文件路径）=====
    # 每个系统盒下方放一条最能代表该模块的代码语句 + 路径
    code_anchors_mid = [
        # (bx, code_line, file_path)
        (bxs[0],
         'add_system<fixed_rate<250>>("camera_reader")',
         "runtime/l1_l2_setup.cpp:687"),
        (bxs[1],
         'add_system<fixed_rate<200>>("armor_detector")',
         "L2_perception/armor/systems.cpp:570"),
        (bxs[2],
         'add_system<fixed_rate<250>>("armor_tracker")',
         "L3_estimation/tracker_systems.cpp:66"),
        (bxs[3],
         'advance_armor_aim_phase(cfg, v_yaw, ...)',
         "L4_planning/aimer/fsm.hpp:52"),
        (bxs[4],
         'add_system<fixed_rate<250>>("enhanced_weapon")',
         "L5_weapon/enhanced/weapon_systems.cpp:244"),
        (bxs[5],
         'std::visit(overloaded{...}, intent)',
         "L5_weapon/enhanced/weapon_systems.cpp:259"),
    ]
    for bx, code, path in code_anchors_mid:
        cx = bx + bw / 2
        # 代码语句（深色等宽风格）
        ax_mid.text(cx, 0.585, code, ha="center", va="center",
                    fontsize=7.2, color="#1F2937", family="monospace",
                    bbox=dict(boxstyle="round,pad=0.06", fc="#F3F4F6", ec="#D1D5DB", lw=0.8))
        # 文件路径（灰色小字）
        ax_mid.text(cx, 0.568, f"@ {path}", ha="center", va="center",
                    fontsize=6.8, color="#6B7280", style="italic")

    # L4 与 ADT 范式的关联标注（缩小字号放在 chips 与 system box 间隙）
    l4_cx = bxs[3] + bw / 2  # 0.605
    ax_mid.annotate(
        "FSM+variant→ADT",
        xy=(l4_cx, 0.880), xytext=(l4_cx, 0.872),
        ha="center", va="center", fontsize=8.5, color=C["purple"],
        fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.08", fc="#F5F3FF", ec=C["purple"], lw=1.0),
        arrowprops=dict(arrowstyle="-", color=C["purple"], lw=0.8,
                        linestyle=(0, (3, 3))))

    # 错误传播链 + World/ECS 连接
    err_y = 0.545
    box(ax_mid, 0.025, err_y - 0.015, 0.95, 0.028,
        "错误：空检测→短路 → 无目标→hold → 超差→不开火  |  System↔World: res读·res_mut写",
        "#FEF2F2", C["red"], lw=2.0, fs=10.5, fc="#1F2937")

    # 副链路区（下移以腾出空间）
    ax_mid.text(0.5, 0.490, "副链路（并行感知，汇入 L4 目标选择）", ha="center",
                va="center", fontsize=17.5, fontweight="bold", color=C["orange"])

    # 能量机关链 + 通道标注
    energy_chain = [
        ("rune_detector\n(L2 检测)", "#FEE2E2", C["red"]),
        ("energy_meter\nvoter+motion\n(L3 预测)", "#FED7AA", C["orange"]),
    ]
    exs = [0.115, 0.295]
    for (lbl, fill, ec), cx in zip(energy_chain, exs):
        box(ax_mid, cx - 0.075, 0.375, 0.15, 0.085, lbl, fill, ec, lw=2.6, fs=14.0,
            fc="#1F2937")
    # 能量机关链路通道
    arrow(ax_mid, 0.19, 0.418, 0.22, 0.418, C["red"], lw=3.2, ms=22.0)
    # 通道名标注
    ax_mid.text(0.205, 0.448, "RuneObservationChannelTopic", ha="center", va="center",
                fontsize=6.5, color=C["red"], family="monospace",
                bbox=dict(boxstyle="round,pad=0.04", fc="#FEF2F2", ec=C["red"], lw=1.0))
    ax_mid.text(0.205, 0.432, "RuneObservation → ", ha="center", va="center",
                fontsize=7.0, color="#1F2937", fontweight="bold")
    # 能量机关状态通道（汇入L4）
    ax_mid.text(0.295, 0.478, "EnergyMeterStateChannelTopic", ha="center", va="center",
                fontsize=6.0, color=C["orange"], family="monospace",
                bbox=dict(boxstyle="round,pad=0.03", fc="#FFF7ED", ec=C["orange"], lw=0.8))
    ax_mid.text(0.295, 0.464, "EnergyMeterState", ha="center", va="center",
                fontsize=6.5, color=C["orange"], fontweight="bold")

    arrow(ax_mid, 0.37, 0.418, 0.565, 0.560, C["orange"], lw=3.0, ls="--", ms=22.0,
          label="汇入 L4", lx=0.49, ly=0.500, fs=13.0)

    # LDM 链 + 通道标注
    ldm_chain = [
        ("ldm_detector\n(L2 几何检测)", "#E0E7FF", "#4F46E5"),
        ("ldm_naive\n(L3 简易跟踪)", "#E0E7FF", "#4F46E5"),
    ]
    lxs = [0.475, 0.655]
    for (lbl, fill, ec), cx in zip(ldm_chain, lxs):
        box(ax_mid, cx - 0.075, 0.375, 0.15, 0.085, lbl, fill, ec, lw=2.6, fs=14.0,
            fc="#1F2937")
    # LDM链路通道
    arrow(ax_mid, 0.55, 0.418, 0.58, 0.418, "#4F46E5", lw=3.2, ms=22.0)
    ax_mid.text(0.565, 0.448, "LdmDetectionChannelTopic", ha="center", va="center",
                fontsize=6.0, color="#4F46E5", family="monospace",
                bbox=dict(boxstyle="round,pad=0.03", fc="#EEF2FF", ec="#4F46E5", lw=0.8))
    ax_mid.text(0.565, 0.432, "LdmDetection", ha="center", va="center",
                fontsize=7.0, color="#1F2937", fontweight="bold")
    # LDM测量通道
    ax_mid.text(0.655, 0.478, "LdmMeasurementChannelTopic", ha="center", va="center",
                fontsize=5.8, color="#4F46E5", family="monospace",
                bbox=dict(boxstyle="round,pad=0.03", fc="#EEF2FF", ec="#4F46E5", lw=0.8))
    ax_mid.text(0.655, 0.464, "LdmMeasurement", ha="center", va="center",
                fontsize=6.5, color="#4F46E5", fontweight="bold")

    arrow(ax_mid, 0.73, 0.418, 0.855, 0.560, "#4F46E5", lw=3.0, ls="--", ms=22.0)

    # 副链路来源（L2 相机）
    ax_mid.text(0.015, 0.405, "L2 相机\n帧分路", ha="center", va="center", fontsize=12.0,
                color=C["sub"], style="italic")
    ax_mid.text(0.015, 0.378, "ImageChannelTopic\n→ rune / ldm / capturer", ha="center", va="center",
                fontsize=6.0, color=C["green"], family="monospace",
                bbox=dict(boxstyle="round,pad=0.03", fc="#F0FDF4", ec=C["green"], lw=0.8))
    arrow(ax_mid, 0.095, 0.73, 0.045, 0.428, C["edge"], lw=2.2, ls=":", ms=18.0)

    # 底部注：可视化解耦 + 通道分路说明
    box(ax_mid, 0.02, 0.300, 0.44, 0.045,
        "Foxglove 订阅所有通道：Image / Det / Meas / TrkOut / Ctrl / WpCmd / Rune / Calib",
        "#FED7AA", C["orange"], lw=2.2, fs=10.0, fc="#1F2937")
    box(ax_mid, 0.48, 0.300, 0.50, 0.045,
        "Capturer 录制 15+ 通道：SPMC 多消费者·三缓冲保活·自然跳帧",
        "#CFFAFE", C["cyan"], lw=2.2, fs=10.0, fc="#1F2937")

    # ==================================================================
    # ④ 右区：子系统带
    # ==================================================================
    ax_right.text(0.5, 0.975, "子系统带", ha="center", va="center",
                  fontsize=23.8, fontweight="bold", color=C["orange"])

    subs = [
        ("Quanta 图传系统", "#DBEAFE", C["blue"],
         ["stream_encoder / transport", "FFmpeg 8.x · Axera venc", "HEVC AnnexB 编码",
          "运行时 stream_send / _encode"]),
        ("Chiral 采集 / 回放", "#D1FAE5", C["green"],
         ["chiral_collector 采集", "endpoint 共享内存", "replay 离线回放",
          "navigation 导航同步"]),
        ("标定 4 件套（离线）", "#FEF3C7", "#D97706",
         ["intrinsic 内参标定", "handeye 手眼标定", "chessboard 棋盘格",
          "charuco 检测板"]),
        ("Foxglove 可视化", "#FED7AA", C["orange"],
         ["foxglove_server (WS)", "foxglove_sink 消息汇", "scene_builder 场景+配色",
          "各层 systems 并行订阅"]),
        ("fast_tf 坐标变换", "#E0F2FE", C["cyan"],
         ["frame ×5: world/odom/", "gimbal/camera/muzzle", "buffer 变换缓冲",
          "validation 校验 · 编译期防误用"]),
        ("ADT 编程范式", "#EDE9FE", C["purple"],
         ["struct product / variant sum", "expected<T,string> · RAII owner",
          "using ControlIntent = variant<Track,Shot,Hold>",
          "@ control_intent.hpp:120"]),
        ("配置 + 日志", "#F1F5F9", C["hw"],
         ["config/: fcs.toml · camera.toml", "foxglove.toml · 标定参数",
          "spdlog 分级日志 + hook", "TOML→World.res 只读"]),
    ]
    # 双列左右排列：左列 4 个（0-3），右列 3 个（4-6）
    col_w = 0.465
    col_xs = [0.015, 0.520]
    col_ytops = [0.935, 0.715, 0.495, 0.275]
    box_h = 0.195  # 0.205→0.195，为底部图例腾出 0.010
    for col_idx, col_subs in enumerate([subs[:4], subs[4:]]):
        for i, (title, fill, ec, lines) in enumerate(col_subs):
            ytop = col_ytops[i]
            x = col_xs[col_idx]
            box(ax_right, x, ytop - box_h, col_w, box_h, "", fill, ec, lw=2.6, zorder=1)
            ax_right.text(x + 0.014, ytop - 0.028, title, ha="left", va="center",
                          fontsize=16.5, fontweight="bold", color="#1F2937")
            ty = ytop - 0.062
            for ln in lines:
                # 代码行（以 using/struct/class 开头）用等宽字体 + 灰底
                is_code = ln.startswith(("using ", "struct ", "class ", "auto ", "void "))
                is_path = ln.startswith("@ ")
                if is_code:
                    ax_right.text(x + 0.014, ty, ln, ha="left", va="center",
                                  fontsize=9.5, color="#1F2937", family="monospace",
                                  bbox=dict(boxstyle="round,pad=0.03", fc="#F3F4F6",
                                            ec="#D1D5DB", lw=0.6))
                elif is_path:
                    ax_right.text(x + 0.014, ty, ln, ha="left", va="center",
                                  fontsize=9.0, color="#6B7280", style="italic")
                else:
                    ax_right.text(x + 0.014, ty, ln, ha="left", va="center", fontsize=12.0,
                                  color="#374151")
                ty -= 0.031

    # ===== 配置链路：右区"配置+日志"盒向上连接到顶部 M1 阶段 =====
    # 从配置盒顶部向上画箭头，示意"→ 顶部 M1 载入"
    cfg_cx = 0.520 + 0.465 / 2  # 0.7525
    arrow(ax_right, cfg_cx, 0.495, cfg_cx, 0.515, C["hw"], lw=2.0, ls=":", ms=12.0,
          label="→ M1 load_config", lx=cfg_cx + 0.100, ly=0.512, fs=10.0, lcolor=C["hw"])

    # ===== 图例：右区底部（利用配置盒下方空白）=====
    lg_x, lg_y = 0.015, 0.062
    lg_w, lg_h = 0.970, 0.062
    box(ax_right, lg_x, lg_y - lg_h, lg_w, lg_h, "", "#FAFAFA", "#6B7280", lw=2.2, zorder=1)
    ax_right.text(lg_x + 0.012, lg_y - 0.018, "图例 Legend", ha="left", va="center",
                  fontsize=13.0, fontweight="bold", color="#374151")
    legend_lines = [
        ("数据流", C["blue"], "-"),
        ("控制流", C["purple"], "-"),
        ("DAG 层依赖", C["orange"], "-"),
        ("错误/唤醒", C["red"], "-"),
        ("副路汇入", "#4F46E5", "--"),
        ("订阅级联", C["purple"], ":"),
    ]
    lx0 = lg_x + 0.155  # 减小起始偏移
    ldx = 0.130  # 0.138→0.130，缩小间距
    for i, (lbl, col, ls) in enumerate(legend_lines):
        xp = lx0 + i * ldx
        yp = lg_y - 0.032
        ax_right.annotate("", xy=(xp + 0.042, yp), xytext=(xp, yp),
                          arrowprops=dict(arrowstyle="-|>", color=col, lw=1.8,
                                          linestyle=ls, mutation_scale=9.0))
        ax_right.text(xp + 0.045, yp, lbl, ha="left", va="center",
                      fontsize=10.2, color="#374151")

    # ==================================================================
    # ⑤ 底部：调度线程模型函数链 + 通信机制
    # ==================================================================
    ax_bot.text(0.5, 0.985,
                "调度通信机制 — 支撑上方主链路 · fixed_rate 驱动 · SPMC 传输 · pool_compute 并行",
                ha="center", va="center", fontsize=28.0, fontweight="bold",
                color="#1F2937")

    # 调度线程模型：四条关键函数链横条
    chains = [
        ("fixed_rate 线程 ×N（独占）", "#D1FAE5", C["green"], 0.015, 0.21,
         ["run_fixed_rate_thread → run(world) → sleep_until",
          "notify(idx) → fetch_or 置位下游掩码"]),
        ("Channel 通道（数据驱动）", "#FEF3C7", C["yellow"], 0.24, 0.10,
         ["写者 → written_flag_=true",
          "→ 唤醒所有订阅者"]),
        ("compute 主线程 ×1（调度器）", "#FED7AA", C["orange"], 0.355, 0.25,
         ["run_compute_loop → ready_systems_.exchange(0)",
          "run_compute_selective → task_group + 级联"]),
        ("停机流程", "#F1F5F9", C["hw"], 0.675, 0.15,
         ["stop() → lifecycle=Stopped → join() 回收",
          "shutdown_hooks 顺序执行资源释放"]),
    ]
    for title, fill, ec, cx, cw, lines in chains:
        box(ax_bot, cx, 0.885, cw, 0.068, "", fill, ec, lw=2.4, zorder=1)
        ax_bot.text(cx + cw / 2, 0.932, title, ha="center", va="center",
                    fontsize=15.0, fontweight="bold", color=ec)
        for i, ln in enumerate(lines):
            ax_bot.text(cx + 0.012, 0.905 - i * 0.024, ln, ha="left", va="center",
                        fontsize=12.0, color="#1F2937")

    # fixed_rate → Channel（写方向）
    arrow(ax_bot, 0.22, 0.925, 0.24, 0.925, C["red"], lw=3.2, ms=18.0,
          label="写", lx=0.228, ly=0.895, fs=10.5, lcolor=C["red"])
    # Channel → compute（唤醒方向）
    arrow(ax_bot, 0.34, 0.925, 0.355, 0.925, C["red"], lw=3.2, ms=18.0,
          label="唤醒", lx=0.347, ly=0.895, fs=10.5, lcolor=C["red"])
    # compute → Channel（反向：pool_compute 也写通道给下游消费者）
    arrow(ax_bot, 0.355, 0.905, 0.34, 0.905, C["orange"], lw=2.6, ls="--", ms=14.0,
          label="写", lx=0.347, ly=0.948, fs=10.0, lcolor=C["orange"])
    # fixed_rate 也能被唤醒（silent / 数据源订阅者场景）
    arrow(ax_bot, 0.24, 0.905, 0.22, 0.905, C["green"], lw=2.6, ls="--", ms=14.0,
          label="订阅", lx=0.228, ly=0.948, fs=10.0, lcolor=C["green"])
    # compute 自级联（pool_compute 之间通过通道互相唤醒）
    arrow(ax_bot, 0.605, 0.885, 0.625, 0.852, C["orange"], lw=2.8, ls="--", ms=16.0,
          label="compute_affects_ 级联",
          lx=0.64, ly=0.855, fs=10.0, lcolor=C["orange"])

    # 卡 1：fixed_rate
    box(ax_bot, 0.015, 0.22, 0.25, 0.60, "", "#DBEAFE", C["blue"], lw=3.2, zorder=1)
    ax_bot.text(0.14, 0.795, "① fixed_rate 独占线程 · 数据源驱动", ha="center", va="center",
                fontsize=24.0, fontweight="bold", color=C["blue"])
    fr_lines = [
        ("【线程初始化】", True),
        ("• affinity ≥0 → pin_to_core 绑核", False),
        ("• priority >0 → SCHED_FIFO 实时", False),
        ("【run_fixed_rate_thread 定频循环】", True),
        ("• freq>0: period=1s/freq", False),
        ("   next_tick+=period · 定时休眠", False),
        ("• freq=0: 全速无限循环", False),
        ("【驱动主链路】", True),
        ("• 250Hz: 相机/L3/L4/L5/执行器", False),
        ("• 200Hz×2: L2 det + solver", False),
        ("【通知下游】", True),
        ("• written && notifies → notify(idx)", False),
        ("• ready_systems_ 位掩码置位", False),
    ]
    for i, (ln, is_head) in enumerate(fr_lines):
        ax_bot.text(0.03, 0.750 - i * 0.0365, ln, ha="left", va="center",
                    fontsize=17.0 if is_head else 16.0,
                    fontweight="bold" if is_head else "normal",
                    color=C["blue"] if is_head else "#1F2937")

    # 卡 2：SPMC 三缓冲（细节逻辑 + 真实通道示例）
    box(ax_bot, 0.285, 0.22, 0.36, 0.60, "", "#CFFAFE", C["cyan"], lw=3.2, zorder=1)
    ax_bot.text(0.465, 0.795, "② SPMC 三缓冲 · 主链路 5 核心通道 + 12 辅助通道", ha="center", va="center",
                fontsize=22.0, fontweight="bold", color=C["cyan"])
    ax_bot.text(0.465, 0.760, "单写者·多读者 · current 原子替换 + 快照引用保活 · 所有通道均此模型",
                ha="center", va="center", fontsize=14.5, color="#1F2937")

    # 左：Writer 盒 + 通道示例
    box(ax_bot, 0.290, 0.52, 0.095, 0.21, "", "#D1FAE5", C["green"], lw=2.6, zorder=1)
    ax_bot.text(0.3375, 0.708, "Writer 写句柄", ha="center", va="center", fontsize=16.0,
                fontweight="bold", color=C["green"])
    for i, ln in enumerate([
            "write(data):",
            "① make_shared 新数据",
            "② write_lock 置 WRITER_BIT",
            "③ 自旋等读者计数→0",
            "④ current 原子替换",
            "⑤ gen++ → unlock"]):
        ax_bot.text(0.297, 0.682 - i * 0.027, ln, ha="left", va="center",
                    fontsize=13.5, color="#1F2937")

    # 中：State 盒
    box(ax_bot, 0.400, 0.52, 0.115, 0.21, "", "#E0F2FE", "#0E7490", lw=2.6, zorder=1)
    ax_bot.text(0.4575, 0.708, "State alignas(64)", ha="center", va="center",
                fontsize=15.5, fontweight="bold", color="#0E7490")
    for i, ln in enumerate([
            "RWSpinLock u32",
            "· Bit31 WRITER_BIT",
            "· Bit0~30 读者计数",
            "current: shptr<const T>",
            "generation: u64 自增"]):
        ax_bot.text(0.407, 0.678 - i * 0.029, ln, ha="left", va="center",
                    fontsize=13.5, color="#1F2937")

    # 右：Readers 盒 + 真实通道消费者示例
    box(ax_bot, 0.530, 0.52, 0.110, 0.21, "", "#E0E7FF", "#4F46E5", lw=2.6, zorder=1)
    ax_bot.text(0.585, 0.708, "Readers ×N 多消费者", ha="center", va="center",
                fontsize=15.0, fontweight="bold", color="#4F46E5")
    for i, ln in enumerate([
            "Read① tracker / foxglove",
            "Read② aimer / chiral",
            "Read③ fire_ctrl / visual",
            "clone() 继承版本·保活"]):
        ax_bot.text(0.537, 0.678 - i * 0.029, ln, ha="left", va="center",
                    fontsize=12.5, color="#1F2937")

    # 写锁箭头 Writer→State，快照箭头 State→Readers
    arrow(ax_bot, 0.385, 0.625, 0.40, 0.625, C["green"], lw=2.6, ms=16.0, label="写锁",
          lx=0.3925, ly=0.638, fs=12.0)
    for ry in (0.68, 0.65, 0.62):
        arrow(ax_bot, 0.515, ry, 0.530, ry, "#4F46E5", lw=2.0, ls="--", ms=14.0)
    ax_bot.text(0.5225, 0.562, "current 指针拷贝\n(引用计数+1)", ha="center", va="center",
                fontsize=11.5, color="#4F46E5")

    # 读路径流程条 + 通道映射
    box(ax_bot, 0.290, 0.385, 0.350, 0.11, "", "#F8FAFC", C["cyan"], lw=2.2, zorder=1)
    # 通道映射表（左半）
    ax_bot.text(0.295, 0.478, "主链路通道映射", ha="left", va="center",
                fontsize=11.5, fontweight="bold", color=C["cyan"])
    ch_map_lines = [
        "→ ImageChannelTopic        (CameraFrameParcel  250Hz)",
        "→ DetectionChannelTopic    (ArmorDetection     200Hz)",
        "→ MeasurementChannelTopic  (ArmorMeasurement   250Hz)",
        "→ TrackerOutputTopic      (TrackOutput        250Hz)",
        "→ WeaponCommandChannelTopic(WeaponCommand      250Hz)",
    ]
    for i, ln in enumerate(ch_map_lines):
        ax_bot.text(0.295, 0.462 - i * 0.016, ln, ha="left", va="center",
                    fontsize=10.8, color="#1F2937", family="monospace")

    # 要点：辅助通道
    for i, ln in enumerate([
            "• 辅助: RuneObservation / EnergyMeterState / LdmDetection / LdmMeasurement / Calibration / DebugParcel",
            "• 写 50~100ns · 读 20~40ns · 读者互不阻塞 · last_gen_ 过滤天然跳帧",
            "• 旧数据由读者持有时引用计数保活 → 三份生命周期防 ABA"]):
        ax_bot.text(0.297, 0.368 - i * 0.029, ln, ha="left", va="center",
                    fontsize=13.0, color="#1F2937")
    # SPMC 代码锚点：用真实的 write() / read() 关键语句
    ax_bot.text(0.465, 0.242,
                "write(): make_shared → write_lock → current=ptr → gen++",
                ha="center", va="center", fontsize=8.5, color="#1F2937",
                family="monospace",
                bbox=dict(boxstyle="round,pad=0.05", fc="#F3F4F6", ec="#D1D5DB", lw=0.7))
    ax_bot.text(0.465, 0.225, "@ spmc_triple_buffer.hpp:224  消费者 8+ : tracker / aimer / fire_ctrl / foxglove / chiral",
                ha="center", va="center", fontsize=7.8, color="#6B7280", style="italic")

    # 卡 3：shm 三缓冲（跨进程）
    # —— 扩展宽度 0.17→0.215，展示三槽/状态位/生产者消费者流程 ——
    card3_x, card3_w = 0.665, 0.180
    card3_cx = card3_x + card3_w / 2
    box(ax_bot, card3_x, 0.22, card3_w, 0.60, "", "#E0F2FE", "#0E7490", lw=3.2, zorder=1)

    # 标题
    ax_bot.text(card3_cx, 0.795, "③ shm 三缓冲", ha="center", va="center",
                fontsize=24.0, fontweight="bold", color="#0E7490")
    ax_bot.text(card3_cx, 0.760, "共享内存跨进程 · 无锁", ha="center", va="center",
                fontsize=16.5, color="#1F2937")

    # ---- 三槽可视化 (y=0.700) ----
    slot_y, slot_h, slot_w = 0.700, 0.038, 0.048
    slot_gap = 0.006
    total_slot_w = slot_w * 3 + slot_gap * 2
    slot_start_x = card3_cx - total_slot_w / 2
    slot_labels = ["slot[0]", "slot[1]", "slot[2]"]
    for i in range(3):
        sx = slot_start_x + i * (slot_w + slot_gap)
        box(ax_bot, sx, slot_y, slot_w, slot_h, slot_labels[i], "#FFFFFF", "#0E7490",
            lw=1.8, fs=12.0, fc="#0E7490")
    ax_bot.text(card3_cx, slot_y - 0.010, "三槽轮转 · write_idx 写 / read_idx 读",
                ha="center", va="center", fontsize=11.0, color="#0369A1",
                fontstyle="italic")

    # ---- state uint8_t 位分解 (y=0.632, h=0.045) ----
    state_y, state_h = 0.632, 0.045
    box(ax_bot, card3_x + 0.010, state_y, card3_w - 0.020, state_h,
        "", "#F0F9FF", "#0284C7", lw=1.8, zorder=1)
    ax_bot.text(card3_x + 0.014, state_y + state_h - 0.012, "state uint8_t 位分解",
                ha="left", va="center", fontsize=12.5, fontweight="bold", color="#0284C7")
    ax_bot.text(card3_x + 0.014, state_y + state_h - 0.030,
                "Bit7 FLAG_NEW | Bit0-1 ready_idx(0~2)",
                ha="left", va="center", fontsize=10.0, color="#1F2937")

    # ---- 生产者 publish() (y=0.552, h=0.065) ----
    pub_y, pub_h = 0.552, 0.065
    box(ax_bot, card3_x + 0.010, pub_y, card3_w - 0.020, pub_h,
        "", "#D1FAE5", C["green"], lw=1.8, zorder=1)
    ax_bot.text(card3_x + 0.014, pub_y + pub_h - 0.012, "生产者 publish()",
                ha="left", va="center", fontsize=11.5, fontweight="bold", color=C["green"])
    pub_lines = [
        "borrow_mut(): &slots[write_idx]",
        "state.exchange(write|FLAG_NEW)",
        "write_idx = old & INDEX_MASK",
    ]
    for i, ln in enumerate(pub_lines):
        ax_bot.text(card3_x + 0.014, pub_y + pub_h - 0.030 - i * 0.014, ln,
                    ha="left", va="center", fontsize=9.5, color="#1F2937")

    # ---- 消费者 borrow() (y=0.478, h=0.070) ----
    bor_y, bor_h = 0.478, 0.070
    box(ax_bot, card3_x + 0.010, bor_y, card3_w - 0.020, bor_h,
        "", "#E0E7FF", "#4F46E5", lw=1.8, zorder=1)
    ax_bot.text(card3_x + 0.014, bor_y + bor_h - 0.012, "消费者 borrow()",
                ha="left", va="center", fontsize=11.5, fontweight="bold", color="#4F46E5")
    bor_lines = [
        "state.load() → FLAG_NEW?",
        "CAS: state ↔ read_idx",
        "buf.read_idx = ready_idx",
        "return &slots[ready_idx]",
    ]
    for i, ln in enumerate(bor_lines):
        ax_bot.text(card3_x + 0.014, bor_y + bor_h - 0.028 - i * 0.014, ln,
                    ha="left", va="center", fontsize=9.5, color="#1F2937")

    # ---- TripleBufferOps 类型 (y=0.408, h=0.065) ----
    type_y, type_h = 0.408, 0.065
    box(ax_bot, card3_x + 0.010, type_y, card3_w - 0.020, type_h,
        "", "#FEF3C7", "#D97706", lw=1.8, zorder=1)
    ax_bot.text(card3_x + 0.014, type_y + type_h - 0.012, "TripleBufferOps<Buffer,Slot>",
                ha="left", va="center", fontsize=10.8, fontweight="bold", color="#D97706")
    type_lines = [
        "ImageTripleBuffer  192B",
        "PoseTripleBuffer×5  256B",
        "GimbalTripleBuffer  192B",
    ]
    for i, ln in enumerate(type_lines):
        ax_bot.text(card3_x + 0.014, type_y + type_h - 0.027 - i * 0.014, ln,
                    ha="left", va="center", fontsize=9.5, color="#1F2937")

    # ---- SHM 布局 (y=0.358) ----
    ax_bot.text(card3_cx, 0.358, "SHM 布局", ha="center", va="center",
                fontsize=11.5, fontweight="bold", color="#0E7490")
    shm_lines = [
        "ShmHeader magic=0x54414C05",
        "heartbeat · created_ns",
        "meta shm 3712B+image_pool",
    ]
    for i, ln in enumerate(shm_lines):
        ax_bot.text(card3_cx, 0.342 - i * 0.014, ln, ha="center", va="center",
                    fontsize=9.5, color="#1F2937")

    # ---- 进程间示意 (y=0.225, h=0.020) ----
    pbox_y, pbox_h = 0.225, 0.020
    pgap = 0.002
    left_w = card3_w * 0.45
    pw = (left_w - pgap * 2) / 3
    pbox_x0 = card3_x + 0.010
    box(ax_bot, pbox_x0, pbox_y, pw, pbox_h, "进程A(写)", "#D1FAE5", C["green"],
        lw=1.5, fs=9.0, fc="#1F2937")
    box(ax_bot, pbox_x0 + pw + pgap, pbox_y, pw, pbox_h, "共享内存", "#F0F9FF", "#0284C7",
        lw=1.5, fs=9.0, fc="#0284C7")
    box(ax_bot, pbox_x0 + 2 * (pw + pgap), pbox_y, pw, pbox_h, "进程B(读)", "#E0E7FF", "#4F46E5",
        lw=1.5, fs=9.0, fc="#1F2937")
    # SHM 代码锚点：用真实的 publish() / borrow() 关键语句
    ax_bot.text(card3_cx, 0.205,
                "publish(): state.exchange(write|FLAG_NEW)",
                ha="center", va="center", fontsize=7.8, color="#1F2937",
                family="monospace",
                bbox=dict(boxstyle="round,pad=0.03", fc="#F3F4F6", ec="#D1D5DB", lw=0.6))
    ax_bot.text(card3_cx, 0.190, "@ hardware_daedalus/shm_triple_buffer.hpp:36",
                ha="center", va="center", fontsize=7.0, color="#6B7280", style="italic")

    # 卡 4：pool_compute — DAG 分层可视化（加宽以容纳可视化）
    card4_x, card4_w = 0.855, 0.145
    card4_cx = card4_x + card4_w / 2
    box(ax_bot, card4_x, 0.22, card4_w, 0.60, "", "#FED7AA", C["orange"], lw=3.2, zorder=1)
    ax_bot.text(card4_cx, 0.800, "④ pool_compute", ha="center", va="center",
                fontsize=19.0, fontweight="bold", color=C["orange"])
    ax_bot.text(card4_cx, 0.772, "DAG 分层 · 数据驱动", ha="center", va="center",
                fontsize=13.0, color="#1F2937")

    # DAG 分层可视化 — 实际主链路拓扑
    dag_layers = [
        ("L0", "camera_reader", "#D1FAE5", C["green"]),
        ("L1", "detector + solver", "#DBEAFE", C["blue"]),
        ("L2", "tracker", "#E0E7FF", "#4F46E5"),
        ("L3", "aimer + fire_ctrl", "#FEF3C7", "#B45309"),
        ("L4", "执行器输出", "#FEE2E2", C["red"]),
    ]
    layer_h = 0.048
    layer_gap = 0.004
    total_h = len(dag_layers) * layer_h + (len(dag_layers) - 1) * layer_gap
    start_y = 0.725
    for i, (lvl, name, fill, ec) in enumerate(dag_layers):
        y = start_y - i * (layer_h + layer_gap)
        box(ax_bot, card4_x + 0.008, y, card4_w - 0.016, layer_h,
            f"{lvl} {name}", fill, ec, lw=2.0, fs=10.8, fc="#1F2937")
        if i < len(dag_layers) - 1:
            arrow(ax_bot, card4_cx, y, card4_cx, y - layer_gap,
                  C["orange"], lw=2.2, ms=14.0)

    # 规则 + 触发说明
    rule_y = start_y - total_h - 0.015
    ax_bot.text(card4_cx, rule_y, "同层并行 · 跨层等待", ha="center", va="center",
                fontsize=11.5, fontweight="bold", color=C["orange"],
                bbox=dict(boxstyle="round,pad=0.10", fc="#FFF7ED", ec=C["orange"], lw=1.2))
    for i, ln in enumerate([
            "TBB task_group 层内并行",
            "ready_systems_ 位掩码",
            "自旋→yield→10µs 退避"]):
        ax_bot.text(card4_cx, rule_y - 0.022 - i * 0.018, ln, ha="center", va="center",
                    fontsize=10.5, color="#374151")
    # fixed_rate 代码锚点：真实策略定义
    ax_bot.text(0.140, 0.238,
                "using fixed_rate = fixed_rate_base<F,A,P,NotifyingTopic>",
                ha="center", va="center", fontsize=8.0, color="#1F2937",
                family="monospace",
                bbox=dict(boxstyle="round,pad=0.04", fc="#F3F4F6", ec="#D1D5DB", lw=0.6))
    ax_bot.text(0.140, 0.222, "@ execution_policy.hpp:81",
                ha="center", va="center", fontsize=7.5, color="#6B7280", style="italic")
    # pool_compute 代码锚点：真实调度核心
    ax_bot.text(card4_cx, 0.233,
                "run_compute_selective(ready_mask)",
                ha="center", va="center", fontsize=8.0, color="#1F2937",
                family="monospace",
                bbox=dict(boxstyle="round,pad=0.04", fc="#F3F4F6", ec="#D1D5DB", lw=0.6))
    ax_bot.text(card4_cx, 0.218, "@ scheduler.hpp:438",
                ha="center", va="center", fontsize=7.2, color="#6B7280", style="italic")

    out = os.path.join(OUTDIR, out_name)
    fig.canvas.draw()
    verify(fig, out_name)
    plt.savefig(out, dpi=130, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"saved: {out}")


if __name__ == "__main__":
    draw_master("talos_arch_master.png")
