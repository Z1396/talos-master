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
def draw_master(out_name, figsize=(40, 32)):
    fig = plt.figure(figsize=figsize, facecolor="white")

    fig.text(0.5, 0.977, "Talos 全景架构总图（深度细化版）", ha="center", va="top",
             fontsize=62.5, fontweight="bold", color=C["text"])
    fig.text(0.5, 0.952, "主链路 × 副链路 × 调度内核 × 子系统带 × 通信机制 —— 全模块覆盖，一图看懂",
             ha="center", va="top", fontsize=27.5, color=C["sub"])

    ax_top = fig.add_axes([0.010, 0.862, 0.980, 0.080])
    ax_left = fig.add_axes([0.010, 0.335, 0.235, 0.512])
    ax_mid = fig.add_axes([0.260, 0.335, 0.480, 0.512])
    ax_right = fig.add_axes([0.755, 0.335, 0.235, 0.512])
    ax_bot = fig.add_axes([0.010, 0.030, 0.980, 0.300])

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

    # ==================================================================
    # ③ 中区：主链路（深度细化）+ 副链路
    # ==================================================================
    ax_mid.text(0.5, 0.975, "主链路 — 五级 FCS 流水线（深度细化）", ha="center",
                va="center", fontsize=23.8, fontweight="bold", color=C["blue"])
    ax_mid.text(0.5, 0.955, "fixed_rate 定频 · SPMC 通道 read() 取最新帧 · ★ 为分支",
                ha="center", va="center", fontsize=15.5, color=C["sub"])

    # 层级徽标
    chips = [("L1", "#D1FAE5"), ("L2", "#DBEAFE"), ("L3", "#E0E7FF"),
             ("L4", "#FEF3C7"), ("L5", "#FEE2E2"), ("执行", "#EDE9FE")]
    for i, (lbl, col) in enumerate(chips):
        cx = 0.075 + i * 0.1904
        box(ax_mid, cx - 0.065, 0.90, 0.13, 0.035, lbl, col, C["edge"], lw=1.4, fs=15.0)

    # 系统盒（6）
    sys_boxes = [
        ("camera_reader", "250Hz", "#D1FAE5", C["green"],
         ["HIK recv(1s) 取图", "→ ImageFrame 写入"], None),
        ("L2 armor", "200Hz×2", "#DBEAFE", C["blue"],
         ["detector: img_in.read()", "backend 多态: Axera", "/ ORT / TRT / 传统",
          "→ [Det] → solver: PnP+BA", "先验位姿 · readback ROI"],
         "★空检测→空 batch"),
        ("L3 tracker", "250Hz", "#E0E7FF", "#4F46E5",
         ["TrackerManager 多实例", "EKF / InvariantEKF", "data_associator",
          "motion model 预测", "协方差→odom"], None),
        ("L4 aimer", "250Hz", "#FEF3C7", "#B45309",
         ["target decider 选目标", "aimer FSM 状态机", "trajectory_builder",
          "TOF 补偿 · gimbal MPC", "→ ControlIntent"],
         "★无目标→hold"),
        ("L5 fire_ctrl", "250Hz", "#FEE2E2", C["red"],
         ["fire_decision 开火门", "dual MPC: OSQP 求解", "TinyMpc 轨迹优化",
          "visit: Track/Shot/Hold", "→ WeaponCommand"],
         "★超差→不开火"),
        ("执行器", "250Hz", "#EDE9FE", "#6D28D9",
         ["weapon_output", "at_gimbal 串口", "daedalus 共享内存", "状态回读"], None),
    ]
    bxs = [0.025, 0.195, 0.365, 0.535, 0.705, 0.875]
    bw = 0.14
    chans = ["ImageFrame", "ArmorMeas", "TrackerOut", "CtrlIntent", "WeaponCmd"]
    for i, (name, freq, fill, ec, lines, branch) in enumerate(sys_boxes):
        cx = bxs[i] + bw / 2
        box(ax_mid, bxs[i], 0.58, bw, 0.30, "", fill, ec, lw=3.2, zorder=1)
        ax_mid.text(cx, 0.855, name, ha="center", va="center", fontsize=17.5,
                    fontweight="bold", color="#1F2937")
        ax_mid.text(cx, 0.828, freq, ha="center", va="center", fontsize=13.0,
                    color=C["sub"])
        ty = 0.795
        for ln in lines:
            ax_mid.text(cx, ty, ln, ha="center", va="center", fontsize=14.0, color="#374151")
            ty -= 0.0414
        if branch:
            ax_mid.text(cx, 0.602, branch, ha="center", va="center", fontsize=13.5,
                        color=C["red"], fontweight="bold",
                        bbox=dict(boxstyle="round,pad=0.10", fc="#FEE2E2", ec="none"))
        if i < 5:
            ccx = bxs[i] + bw + 0.017
            cylinder(ax_mid, ccx, 0.73, 0.028, 0.06, "", fill="#F8FAFC",
                     edge=C["cyan"], lw=2.4, fs=11.2, fc="#475569")
            ax_mid.text(ccx, 0.54, chans[i], ha="center", va="center", fontsize=10.8,
                        color="#475569", fontweight="bold")
            ax_mid.add_patch(FancyArrowPatch((bxs[i] + bw, 0.73), (ccx - 0.014, 0.73),
                                             arrowstyle="-|>", mutation_scale=8, color=ec,
                                             linewidth=1.3, zorder=3))
            ax_mid.add_patch(FancyArrowPatch((ccx + 0.014, 0.73),
                                             (bxs[i + 1] - 0.003, 0.73),
                                             arrowstyle="-|>", mutation_scale=8,
                                             color=sys_boxes[i + 1][3], linewidth=1.3, zorder=3))
    # 执行器输出
    arrow(ax_mid, 0.875 + bw, 0.73, 0.99, 0.73, "#6D28D9", lw=3.6, ms=22.0)

    # 副链路区
    ax_mid.text(0.5, 0.51, "副链路（并行感知，汇入 L4 目标选择）", ha="center",
                va="center", fontsize=18.8, fontweight="bold", color=C["orange"])

    # 能量机关链
    energy_chain = [
        ("rune_detector\n(L2 检测)", "#FEE2E2", C["red"]),
        ("energy_meter\nvoter+motion\n(L3 预测)", "#FED7AA", C["orange"]),
    ]
    exs = [0.115, 0.295]
    for (lbl, fill, ec), cx in zip(energy_chain, exs):
        box(ax_mid, cx - 0.075, 0.39, 0.15, 0.09, lbl, fill, ec, lw=2.6, fs=14.0,
            fc="#1F2937")
    arrow(ax_mid, 0.19, 0.435, 0.22, 0.435, C["red"], lw=2.8, ms=20.0)
    arrow(ax_mid, 0.37, 0.435, 0.565, 0.58, C["orange"], lw=2.4, ls="--", ms=18.0,
          label="汇入 L4", lx=0.49, ly=0.518, fs=13.5)

    # LDM 链
    ldm_chain = [
        ("ldm_detector\n(L2 几何检测)", "#E0E7FF", "#4F46E5"),
        ("ldm_naive\n(L3 简易跟踪)", "#E0E7FF", "#4F46E5"),
    ]
    lxs = [0.475, 0.655]
    for (lbl, fill, ec), cx in zip(ldm_chain, lxs):
        box(ax_mid, cx - 0.075, 0.39, 0.15, 0.09, lbl, fill, ec, lw=2.6, fs=14.0,
            fc="#1F2937")
    arrow(ax_mid, 0.55, 0.435, 0.58, 0.435, "#4F46E5", lw=2.8, ms=20.0)
    arrow(ax_mid, 0.73, 0.435, 0.855, 0.58, "#4F46E5", lw=2.4, ls="--", ms=18.0)

    # 副链路来源（L2 相机）与可视化注
    ax_mid.text(0.015, 0.42, "L2 相机\n帧分路", ha="center", va="center", fontsize=11.5,
                color=C["sub"], style="italic")
    arrow(ax_mid, 0.095, 0.73, 0.045, 0.445, C["edge"], lw=1.8, ls=":", ms=16.0)

    # 底部注：可视化解耦
    box(ax_mid, 0.02, 0.315, 0.44, 0.05,
        "可视化 foxglove_*（pool_compute）并行订阅各通道 → WebSocket / MCAP",
        "#FED7AA", C["orange"], lw=2.2, fs=13.0, fc="#1F2937")
    box(ax_mid, 0.48, 0.315, 0.50, 0.05,
        "5 条通道均 SPMC 三缓冲 · 多消费者并行 · 消费端天然跳帧",
        "#CFFAFE", C["cyan"], lw=2.2, fs=13.0, fc="#1F2937")

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
         ["struct = product type", "variant = sum type",
          "expected<T,string> 失败", "RAII owner 资源"]),
        ("配置 + 日志", "#F1F5F9", C["hw"],
         ["TOML 配置 fail-fast", "fcs/camera/foxglove 分载", "spdlog 分级日志",
          "spdlog_hook 钩子"]),
    ]
    # 双列左右排列：左列 4 个（0-3），右列 3 个（4-6）
    col_w = 0.465
    col_xs = [0.015, 0.520]
    col_ytops = [0.935, 0.715, 0.495, 0.275]
    box_h = 0.205
    for col_idx, col_subs in enumerate([subs[:4], subs[4:]]):
        for i, (title, fill, ec, lines) in enumerate(col_subs):
            ytop = col_ytops[i]
            x = col_xs[col_idx]
            box(ax_right, x, ytop - box_h, col_w, box_h, "", fill, ec, lw=2.6, zorder=1)
            ax_right.text(x + 0.014, ytop - 0.028, title, ha="left", va="center",
                          fontsize=16.5, fontweight="bold", color="#1F2937")
            ty = ytop - 0.065
            for ln in lines:
                ax_right.text(x + 0.014, ty, ln, ha="left", va="center", fontsize=12.2,
                              color="#374151")
                ty -= 0.0336

    # ==================================================================
    # ⑤ 底部：调度线程模型函数链 + 通信机制
    # ==================================================================
    ax_bot.text(0.5, 0.985,
                "调度线程模型 — fixed_rate / pool_compute 为平级 System（Channel 数据驱动互联）",
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
    ax_bot.text(0.14, 0.795, "① fixed_rate 独占线程", ha="center", va="center",
                fontsize=24.0, fontweight="bold", color=C["blue"])
    fr_lines = [
        ("【线程初始化】", True),
        ("• affinity ≥0 → pin_to_core 绑核", False),
        ("• priority >0 → SCHED_FIFO 实时", False),
        ("【run_fixed_rate_thread 定频循环】", True),
        ("• freq>0: period=1s/freq", False),
        ("   next_tick+=period · 定时休眠", False),
        ("• freq=0: 全速无限循环", False),
        ("• LatencyProbe + 延迟直方图", False),
        ("【数据源角色】", True),
        ("• 250Hz: 相机/L3/L4/L5/执行器", False),
        ("• 200Hz: L2 det + solver", False),
        ("【通知下游】", True),
        ("• written && notifies → notify(idx)", False),
        ("• ready_systems_ 位掩码置位 ≤64", False),
    ]
    for i, (ln, is_head) in enumerate(fr_lines):
        ax_bot.text(0.03, 0.750 - i * 0.0365, ln, ha="left", va="center",
                    fontsize=17.0 if is_head else 16.0,
                    fontweight="bold" if is_head else "normal",
                    color=C["blue"] if is_head else "#1F2937")

    # 卡 2：SPMC 三缓冲（细节逻辑）
    box(ax_bot, 0.285, 0.22, 0.36, 0.60, "", "#CFFAFE", C["cyan"], lw=3.2, zorder=1)
    ax_bot.text(0.465, 0.795, "② SPMC 三缓冲 — 单写者 · 多读者", ha="center", va="center",
                fontsize=24.0, fontweight="bold", color=C["cyan"])
    ax_bot.text(0.465, 0.760, "非槽位轮换：current 原子替换 + 快照引用保活",
                ha="center", va="center", fontsize=16.0, color="#1F2937")

    # 左：Writer 盒
    box(ax_bot, 0.290, 0.52, 0.095, 0.21, "", "#D1FAE5", C["green"], lw=2.6, zorder=1)
    ax_bot.text(0.3375, 0.708, "Writer 写句柄", ha="center", va="center", fontsize=17.0,
                fontweight="bold", color=C["green"])
    for i, ln in enumerate([
            "write(data):",
            "① make_shared 新数据",
            "② write_lock 置 WRITER_BIT",
            "③ 自旋等读者计数→0",
            "④ current 原子替换",
            "⑤ gen++ → unlock"]):
        ax_bot.text(0.297, 0.682 - i * 0.027, ln, ha="left", va="center",
                    fontsize=14.5, color="#1F2937")

    # 中：State 盒
    box(ax_bot, 0.400, 0.52, 0.115, 0.21, "", "#E0F2FE", "#0E7490", lw=2.6, zorder=1)
    ax_bot.text(0.4575, 0.708, "State alignas(64)", ha="center", va="center",
                fontsize=16.5, fontweight="bold", color="#0E7490")
    for i, ln in enumerate([
            "RWSpinLock u32",
            "· Bit31 WRITER_BIT",
            "· Bit0~30 读者计数",
            "current: shptr<const T>",
            "generation: u64 自增"]):
        ax_bot.text(0.407, 0.678 - i * 0.029, ln, ha="left", va="center",
                    fontsize=14.5, color="#1F2937")

    # 右：Readers 盒
    box(ax_bot, 0.530, 0.52, 0.110, 0.21, "", "#E0E7FF", "#4F46E5", lw=2.6, zorder=1)
    ax_bot.text(0.585, 0.708, "Readers ×N 多消费者", ha="center", va="center",
                fontsize=16.5, fontweight="bold", color="#4F46E5")
    for i, ln in enumerate([
            "Read① last_gen_ 过滤",
            "Read② last_gen_ 过滤",
            "Read③ last_gen_ 过滤",
            "clone() 继承版本"]):
        ax_bot.text(0.537, 0.678 - i * 0.029, ln, ha="left", va="center",
                    fontsize=14.5, color="#1F2937")

    # 写锁箭头 Writer→State，快照箭头 State→Readers
    arrow(ax_bot, 0.385, 0.625, 0.40, 0.625, C["green"], lw=2.6, ms=16.0, label="写锁",
          lx=0.3925, ly=0.638, fs=13.0)
    for ry in (0.68, 0.65, 0.62):
        arrow(ax_bot, 0.515, ry, 0.530, ry, "#4F46E5", lw=2.0, ls="--", ms=14.0)
    ax_bot.text(0.5225, 0.562, "current 指针拷贝\n(引用计数+1)", ha="center", va="center",
                fontsize=12.5, color="#4F46E5")

    # 读路径流程条
    box(ax_bot, 0.290, 0.385, 0.350, 0.11, "", "#F8FAFC", C["cyan"], lw=2.2, zorder=1)
    for i, ln in enumerate([
            "read() 两段式读路径：",
            "① 无锁快路径: gen ≤ last_gen_ → nullopt",
            "② 慢路径 read_lock(读者+1) → 拷 current 指针",
            "   → 锁内读 gen → unlock → 校验 → last_gen_=gen",
            "③ read_current() 强制最新 · clone() 继承版本"]):
        ax_bot.text(0.297, 0.482 - i * 0.023, ln, ha="left", va="center",
                    fontsize=13.8, color="#1F2937")

    # 要点
    for i, ln in enumerate([
            "• 写 50~100ns · 读 20~40ns · 读者互不阻塞",
            "• 旧数据由读者持有时引用计数保活 → 三份生命周期防 ABA",
            "• 5 条主链路通道 + 可视化均为此模型"]):
        ax_bot.text(0.297, 0.355 - i * 0.033, ln, ha="left", va="center",
                    fontsize=16.0, color="#1F2937")

    # 卡 3：shm 三缓冲（跨进程）
    box(ax_bot, 0.665, 0.22, 0.17, 0.60, "", "#E0F2FE", "#0E7490", lw=3.2, zorder=1)
    ax_bot.text(0.75, 0.795, "③ shm 三缓冲", ha="center", va="center",
                fontsize=24.0, fontweight="bold", color="#0E7490")
    ax_bot.text(0.75, 0.760, "共享内存跨进程", ha="center", va="center",
                fontsize=17.0, color="#1F2937")
    box(ax_bot, 0.675, 0.59, 0.04, 0.08, "进程A\n(硬件)", "#D1FAE5", C["green"],
        lw=2.2, fs=14.5, fc="#1F2937")
    cylinder(ax_bot, 0.75, 0.63, 0.05, 0.10, "shm\nregion", fill="#E0F2FE",
             edge="#0E7490", lw=2.0, fs=14.0, fc="#1F2937")
    box(ax_bot, 0.795, 0.59, 0.04, 0.08, "进程B\n(应用)", "#F1F5F9", C["hw"],
        lw=2.2, fs=14.5, fc="#1F2937")
    arrow(ax_bot, 0.715, 0.63, 0.725, 0.63, C["green"], lw=2.0, ms=14.0)
    arrow(ax_bot, 0.775, 0.63, 0.795, 0.63, C["blue"], lw=2.0, ms=14.0)
    ax_bot.text(0.75, 0.52, "hardware_daedalus shm_triple_buffer\nChiral endpoint 同用",
                ha="center", va="center", fontsize=16.0, color="#1F2937")
    ax_bot.text(0.75, 0.39, "共享内存双缓冲三槽\n无锁跨进程数据交换",
                ha="center", va="center", fontsize=17.0, color="#1F2937")

    # 卡 4：pool_compute
    box(ax_bot, 0.855, 0.22, 0.13, 0.60, "", "#FED7AA", C["orange"], lw=3.2, zorder=1)
    ax_bot.text(0.92, 0.795, "④ pool_compute", ha="center", va="center",
                fontsize=24.0, fontweight="bold", color=C["orange"])
    ax_bot.text(0.92, 0.760, "数据驱动计算池", ha="center", va="center",
                fontsize=17.0, color="#1F2937")
    for i, ln in enumerate([
            "• TBB task_arena 并行池",
            "  (compute_concurrency 核)",
            "• run_compute_loop 轮询掩码",
            "  ready_systems_.exchange(0)",
            "• run_compute_selective 分层",
            "• 单层并行 task_group+wait",
            "• 级联 compute_affects_",
            "• 退避: 自旋→yield→10µs",
            "• 暂停/恢复 条件变量",
            "• foxglove_* · 5s 统计"]):
        ax_bot.text(0.868, 0.715 - i * 0.042, ln, ha="left", va="center",
                    fontsize=16.0, color="#1F2937")

    out = os.path.join(OUTDIR, out_name)
    fig.canvas.draw()
    verify(fig, out_name)
    plt.savefig(out, dpi=130, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"saved: {out}")


if __name__ == "__main__":
    draw_master("talos_arch_master.png")
