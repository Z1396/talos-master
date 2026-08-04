#!/usr/bin/env python3
"""Talos 架构时序图（增强版）生成器

生成 docs/variants/talos_v4_horizontal_enhanced.png：
- 上区：启动流程 6 个里程碑卡片（main → boot → world，含失败分支与关键产物）
- 下区：运行时五级 FCS 流水线（L1 采集 → L5 武器 + 执行器），含层级徽标、
  5 个 SPMC 通道节点、频率总览、调度机制条与可视化解耦订阅

内容数据已逐条对照源码核实（main.cpp / boot.cpp / L1-L5 systems / channel_topics）：
- 主链路全部为 fixed_rate 定频（相机 250Hz / L2 200Hz×2 / L3-L5 与执行器 250Hz）
- SPMC 通道 read() 取最新帧；仅 pool_compute 辅助系统（可视化）按数据事件触发
- PnP+BA 在 L2 armor_solver；L2→L3 之间是 MeasurementChannelTopic
"""

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from matplotlib import font_manager
import matplotlib.gridspec as gridspec

for f in font_manager.fontManager.ttflist:
    if "NotoSansCJK" in f.name or "Noto Sans CJK" in f.name:
        matplotlib.rcParams["font.family"] = f.name
        break
matplotlib.rcParams["axes.unicode_minus"] = False

OUTDIR = "/home/pldx/Desktop/talos-master/docs/variants"
os.makedirs(OUTDIR, exist_ok=True)

# ============================================================================
# 内容数据（已核实）
# ============================================================================
B_CHANNELS = ["ImageTopic\nImageFrame", "Measurement\nArmorMeas", "TrackerOut\nTrackerOutputs",
              "CtrlIntent\nControlIntent", "WeaponCmd\nWeaponCommand"]

MILESTONE_COLORS = {"main": "#059669", "boot": "#7C3AED", "world": "#1D4ED8"}

MILESTONES_ENH = [
    # (title, actor, items[(text, is_branch)], footer)
    ("M1 程序启动", "main",
     [("① signal_handler + init_logger", False),
      ("hook_cstream → spdlog", False),
      ("打印 build_info", False),
      ("④ foxglove_cfg 提前拷贝", False)],
     "产出：日志 / 信号钩子就绪"),
    ("M2 加载配置", "boot",
     [("② load_config(\"at_vision.toml\")", False),
      ("③ 返回 RuntimeConfig", False),
      ("★解析失败 → 直接退出", True)],
     "产出：RuntimeConfig"),
    ("M3 可视化注册", "main",
     [("⑤ Scheduler scheduler(cfg)", False),
      ("⑥ create_foxglove_server", False),
      ("⑦ attach_sink + insert_resource", False),
      ("⑧ register_foxglove_systems", False),
      ("★WS/MCAP 传输 · 失败→降级", True)],
     "产出：Foxglove server + sink 资源"),
    ("M4 boot 初始化", "boot",
     [("⑨ fcs::boot(move(config))", False),
      ("⑩ emplace + visit(backend)", False),
      ("⑪ 注入视觉配置×5", False),
      ("⑫ setup_l1: 打开 HIK 相机", False),
      ("⑬ 相机句柄 + 内参", False),
      ("⑭ 推理后端 + PnP + 弹道", False),
      ("⑮ 注册 L2-L5 ×8", False),
      ("★Direct/Daedalus · 失败→return 1", True)],
     "产出：全局资源 + 系统注册"),
    ("M5 构建", "world",
     [("⑯ build(): 校验 + bind + 冻结", False),
      ("⑰ ok（拓扑 / 分层 / 掩码）", False)],
     "产出：拓扑 / 唤醒掩码（64 上限）"),
    ("M6 运行", "world",
     [("⑱ run() → 定频线程 + compute", False),
      ("shutdown_watcher（run 前启动）", False),
      ("★Ctrl+C → 优雅 stop", True)],
     "产出：运行态（直至 stop）"),
]

PIPE_ENH = [
    # (name, freq, actions[(text, is_branch)], fill, edge)
    ("camera_reader", "fixed_rate<250Hz>",
     [("① recv(1s) 取图", False), ("写入 Image 通道", False)], "#ECFDF5", "#059669"),
    ("L2 armor", "200Hz × 2",
     [("② detector: img_in.read() + NN infer", False),
      ("→ [Det 通道] → solver: PnP+BA", False),
      ("★空检测 → 写空 batch", True)], "#D1FAE5", "#2563EB"),
    ("L3 tracker", "fixed_rate<250Hz>",
     [("③ meas_in.read() + update_all()", False),
      ("EKF 预测-更新 + 数据关联", False)], "#DBEAFE", "#1D4ED8"),
    ("L4 aimer", "fixed_rate<250Hz>",
     [("④ trk_in.read() + aim()", False),
      ("目标选择 + MPC + 弹道", False),
      ("★无目标 → hold 输出", True)], "#DBEAFE", "#B45309"),
    ("L5 fire_ctrl", "fixed_rate<250Hz>",
     [("⑤ intent_in.read() + decide()", False),
      ("MPC 优化 + 开火门(角度/命中)", False),
      ("★超差 → 不开火", True)], "#DBEAFE", "#BE123C"),
    ("执行器", "weapon_output 250Hz",
     [("⑥ 读 WeaponCommand", False),
      ("云台/摩擦轮 + 状态回读", False)], "#EDE9FE", "#6D28D9"),
]

LAYER_CHIPS = [("L1 采集", "#D1FAE5"), ("L2 感知", "#DBEAFE"), ("L3 估计", "#E0E7FF"),
               ("L4 规划", "#FEF3C7"), ("L5 武器", "#FEE2E2"), ("执行", "#EDE9FE")]

MECH_CHIPS = [
    ("fixed_rate 线程", "独占线程 sleep_until 定频\n200Hz(L2) / 250Hz(其余)"),
    ("SPMC 三缓冲", "shared_ptr 快照 + generation\n版本号，无锁多读"),
    ("read() 最新帧", "各层定时 read() 取最新帧\n消费端天然跳帧"),
    ("pool_compute", "辅助系统按数据事件触发\nTBB 线程池 + 位掩码"),
    ("可视化订阅", "foxglove_* 并行订阅\n→ WebSocket / MCAP"),
]


# ============================================================================
# 重叠 / 越界检测
# ============================================================================
def verify(fig, name):
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
                        print(f"  [{name}] OVERLAP: '{texts[i].get_text()[:20]}' x '{texts[j].get_text()[:20]}' area={area:.0f}")
            bb = boxes[i]
            if bb.x0 < 0 or bb.y0 < 0 or bb.x1 > fw or bb.y1 > fh:
                n_oob += 1
                print(f"  [{name}] OOB: '{texts[i].get_text()[:20]}'")
    print(f"[{name}] texts={len([t for ax in fig.axes for t in ax.texts])} "
          f"overlap>{80}px²: {n_over}  oob: {n_oob}")


# ============================================================================
# v4 增强版：横向流水线（里程碑卡片 + 五级流水线）
# ============================================================================
def draw_horizontal_enhanced(out_name, figsize=(26, 17)):
    fig = plt.figure(figsize=figsize, facecolor="white")
    gs = gridspec.GridSpec(2, 1, height_ratios=[0.44, 0.56], hspace=0.16)
    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1])
    for ax in (ax1, ax2):
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1.02)
        ax.axis("off")

    # ---------------- 场景 A：里程碑卡片（架构级细节） ----------------
    ax1.text(0.5, 0.985, "启动流程 — 6 个里程碑（main → boot → world）", ha="center",
             fontsize=14, fontweight="bold", color="#1F2937")
    ax1.text(0.5, 0.935, "18 步启动逻辑归并为里程碑；红字 ★ 为失败/分支路径；卡片底部为关键产物",
             ha="center", fontsize=8.5, color="#6B7280")

    centers = [0.089, 0.2533, 0.4176, 0.5819, 0.7462, 0.9105]
    bw = 0.077
    card_y0, card_h = 0.10, 0.72
    for i, (title, actor, items, footer) in enumerate(MILESTONES_ENH):
        cx = centers[i]
        ac = MILESTONE_COLORS[actor]
        ax1.add_patch(FancyBboxPatch((cx - bw, card_y0), 2 * bw, card_h,
                                     boxstyle="round,pad=0.007", facecolor="#FAFAFA",
                                     edgecolor=ac, linewidth=1.6, zorder=1))
        ax1.add_patch(FancyBboxPatch((cx - bw, card_y0 + card_h - 0.075), 2 * bw, 0.075,
                                     boxstyle="round,pad=0.007", facecolor=ac,
                                     edgecolor="none", linewidth=0, zorder=1))
        ax1.text(cx, card_y0 + card_h - 0.037, title, ha="center", va="center",
                 fontsize=8.5, fontweight="bold", color="white", zorder=2)
        ty = card_y0 + card_h - 0.115
        for text, is_b in items:
            ax1.text(cx, ty, text, ha="center", va="center",
                     fontsize=6.6 if not is_b else 6.3,
                     color="#DC2626" if is_b else "#374151",
                     fontweight="bold" if is_b else "normal", zorder=2)
            ty -= 0.052
        ax1.text(cx, card_y0 + 0.022, footer, ha="center", va="center",
                 fontsize=5.8, color="#6B7280", style="italic", zorder=2)
        if i < 5:
            xa = cx + bw + 0.003
            xb = centers[i + 1] - bw - 0.003
            ax1.add_patch(FancyArrowPatch((xa, 0.46), (xb, 0.46), arrowstyle="-|>",
                                          mutation_scale=13, color="#9CA3AF",
                                          linewidth=1.5, zorder=2))
            ax1.text((xa + xb) / 2, 0.478, "→", ha="center", fontsize=9, color="#6B7280")

    # ---------------- 场景 B：五级流水线（架构级） ----------------
    ax2.text(0.5, 0.985, "运行时 — 五级 FCS 流水线（L1 采集 → L5 武器）", ha="center",
             fontsize=14, fontweight="bold", color="#1F2937")
    ax2.text(0.5, 0.935, "全部 fixed_rate 定频（200/250Hz）；SPMC 通道 read() 取最新帧；pool_compute 辅助系统（可视化）数据事件触发",
             ha="center", fontsize=8.5, color="#6B7280")

    # 层级徽标
    bcenters = [0.048, 0.200, 0.352, 0.504, 0.656, 0.808]
    for (lbl, col), cx in zip(LAYER_CHIPS, bcenters):
        ax2.add_patch(FancyBboxPatch((cx - 0.040, 0.855), 0.080, 0.045,
                                     boxstyle="round,pad=0.005", facecolor=col,
                                     edgecolor="#9CA3AF", linewidth=0.8, zorder=1))
        ax2.text(cx, 0.8775, lbl, ha="center", va="center", fontsize=7.0,
                 fontweight="bold", color="#374151", zorder=2)

    # 流水线盒子
    pipe_y0, pipe_h = 0.44, 0.30
    bxs = [0.006, 0.158, 0.310, 0.462, 0.614, 0.766]      # 盒子左边界
    cxs = [0.100, 0.252, 0.404, 0.556, 0.708]             # 通道中心
    boxw = 0.084
    for i, (name, freq, actions, fill, ec) in enumerate(PIPE_ENH):
        x0 = bxs[i]
        cx = x0 + boxw / 2
        ax2.add_patch(FancyBboxPatch((x0, pipe_y0), boxw, pipe_h,
                                     boxstyle="round,pad=0.005", facecolor=fill,
                                     edgecolor=ec, linewidth=1.5, zorder=1))
        ax2.text(cx, pipe_y0 + pipe_h - 0.032, name, ha="center", va="center",
                 fontsize=7.5, fontweight="bold", color="#1F2937")
        ax2.text(cx, pipe_y0 + pipe_h - 0.068, freq, ha="center", va="center",
                 fontsize=6.2, color="#6B7280")
        ty = pipe_y0 + pipe_h - 0.112
        for text, is_b in actions:
            if "[Det" in text:
                # Det 通道片段整行高亮（蓝色斜体），保持单行不越界
                ax2.text(cx, ty, text.replace("[", "").replace("]", ""),
                         ha="center", va="center", fontsize=6.0,
                         color="#2563EB", style="italic")
            else:
                ax2.text(cx, ty, text, ha="center", va="center",
                         fontsize=6.2 if not is_b else 6.0,
                         color="#DC2626" if is_b else "#374151",
                         fontweight="bold" if is_b else "normal")
            ty -= 0.034
        # 左右链路箭头
        if i < 5:
            ccx = cxs[i]
            ax2.add_patch(FancyArrowPatch((x0 + boxw + 0.003, pipe_y0 + 0.15),
                                          (ccx - 0.023 - 0.003, pipe_y0 + 0.15),
                                          arrowstyle="-|>", mutation_scale=11, color=ec,
                                          linewidth=1.6, zorder=2))
            ax2.add_patch(FancyArrowPatch((ccx + 0.023 + 0.003, pipe_y0 + 0.15),
                                          (bxs[i + 1] - 0.003, pipe_y0 + 0.15),
                                          arrowstyle="-|>", mutation_scale=11,
                                          color=PIPE_ENH[i + 1][4], linewidth=1.6, zorder=2))
            # 通道节点
            ax2.add_patch(FancyBboxPatch((ccx - 0.023, pipe_y0 + 0.03), 0.046, 0.24,
                                         boxstyle="round,pad=0.004", facecolor="white",
                                         edgecolor="#64748B", linewidth=1.0,
                                         linestyle="--", zorder=1))
            ax2.text(ccx, pipe_y0 + 0.15, B_CHANNELS[i], ha="center", va="center",
                     fontsize=5.0, color="#475569", fontweight="bold")

    # 频率总览卡（右侧）
    ax2.add_patch(FancyBboxPatch((0.862, pipe_y0), 0.132, pipe_h,
                                 boxstyle="round,pad=0.006", facecolor="#F8FAFC",
                                 edgecolor="#D1D5DB", linewidth=1.0, zorder=1))
    ax2.text(0.928, pipe_y0 + pipe_h - 0.032, "频率总览", ha="center", va="center",
             fontsize=6.6, fontweight="bold", color="#1F2937")
    lines = ["250Hz: 相机 / L3 / L4", "/ L5 / 执行器",
             "200Hz: L2 det · solver", "触发: pool_compute",
             "通道: 5 × SPMC 三缓冲"]
    ty = pipe_y0 + pipe_h - 0.072
    for ln in lines:
        ax2.text(0.928, ty, ln, ha="center", va="center", fontsize=5.4, color="#374151")
        ty -= 0.048

    # 调度机制条
    ax2.add_patch(FancyBboxPatch((0.006, 0.06), 0.988, 0.235,
                                 boxstyle="round,pad=0.006", facecolor="#F8FAFC",
                                 edgecolor="#D1D5DB", linewidth=1.0, zorder=1))
    ax2.text(0.018, 0.265, "调度机制", ha="left", va="center", fontsize=9,
             fontweight="bold", color="#1F2937")
    mcx = [0.1095, 0.3045, 0.4995, 0.6945, 0.8895]
    mw = 0.185
    for (mtitle, mdesc), cx in zip(MECH_CHIPS, mcx):
        ax2.add_patch(FancyBboxPatch((cx - mw / 2, 0.075), mw, 0.15,
                                     boxstyle="round,pad=0.005", facecolor="white",
                                     edgecolor="#CBD5E1", linewidth=0.9, zorder=1))
        ax2.text(cx, 0.205, mtitle, ha="center", va="center", fontsize=6.8,
                 fontweight="bold", color="#1F2937")
        d1, d2 = mdesc.split("\n")
        ax2.text(cx, 0.165, d1, ha="center", va="center", fontsize=5.6, color="#374151")
        ax2.text(cx, 0.125, d2, ha="center", va="center", fontsize=5.6, color="#374151")

    # 可视化解耦订阅（通道 → 可视化机制片）
    for ccx in cxs[:4]:
        ax2.add_patch(FancyArrowPatch((ccx, pipe_y0 - 0.012), (0.85, 0.155),
                                      arrowstyle="-|>", mutation_scale=9, color="#E67E22",
                                      linewidth=1.1, linestyle=":", alpha=0.8, zorder=2))
    ax2.text(0.72, 0.335, "解耦订阅", ha="center", fontsize=6.4, color="#E67E22",
             bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.92))

    out = os.path.join(OUTDIR, out_name)
    fig.canvas.draw()
    verify(fig, out_name)
    plt.savefig(out, dpi=130, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"saved: {out}")


if __name__ == "__main__":
    draw_horizontal_enhanced("talos_v4_horizontal_enhanced.png")
