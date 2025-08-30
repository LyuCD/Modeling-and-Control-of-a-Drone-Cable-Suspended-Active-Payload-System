import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle
from matplotlib.patheffects import withStroke

# ---------- helpers ----------
def add_box(ax, xy, w, h, text, fc="#ffffff", ec="#111111", lw=1.8, r=0.15, fs=11,
            text_color="black", alpha=1.0, z=3):
    x, y = xy
    box = FancyBboxPatch((x, y), w, h,
                         boxstyle=f"round,pad=0.03,rounding_size={r}",
                         linewidth=lw, edgecolor=ec, facecolor=fc, alpha=alpha, zorder=z)
    ax.add_patch(box)
    ax.text(x + w/2, y + h/2, text,
            ha="center", va="center", fontsize=fs, color=text_color, zorder=z+1)
    return (x, y, w, h)

def arrow(ax, p0, p1, text=None, color="#111111", lw=1.8, ls="solid",
          dy=0.3, fs=10, ms=14, z=4, glow=False):
    a = FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=ms,
                        linewidth=lw, color=color, linestyle=ls, zorder=z)
    if glow:
        a.set_path_effects([withStroke(linewidth=lw+3.5, foreground="white", alpha=0.9)])
    ax.add_patch(a)
    if text:
        mx, my = (p0[0]+p1[0])/2, (p0[1]+p1[1])/2
        ax.text(mx, my+dy, text, ha="center", va="bottom",
                fontsize=fs, color=color, zorder=z+1)

def add_shadow_box(ax, xy, w, h, text, fc="#fff4e6", ec="#f0ad4e",
                   lw=2.2, r=0.18, fs=11, z=6):
    x, y = xy
    shadow = FancyBboxPatch((x+0.18, y-0.18), w, h,
                            boxstyle=f"round,pad=0.03,rounding_size={r}",
                            linewidth=0, edgecolor="none",
                            facecolor="#000000", alpha=0.12, zorder=z-1)
    ax.add_patch(shadow)
    return add_box(ax, (x, y), w, h, text, fc=fc, ec=ec,
                   lw=lw, r=r, fs=fs, z=z)

# ---------- canvas ----------
FIG_W, FIG_H = 14, 7
fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
ax.set_xlim(0, 30)
ax.set_ylim(0, 16)
ax.axis("off")

# Backgrounds
ctrl_bg = Rectangle((1, 1), 19, 14, facecolor="#f0ad4e",
                    alpha=0.08, edgecolor="#f0ad4e", zorder=1)
plant_bg = Rectangle((22, 1), 7, 14, facecolor="#d9f2e3",
                     alpha=0.35, edgecolor="#cfe9da", zorder=1)
ax.add_patch(ctrl_bg); ax.add_patch(plant_bg)
ax.text(1.4, 15.3, "Control Module",
        color="#9b6a08", fontsize=12, fontweight="bold", zorder=2)
ax.text(22.4, 15.3, "Drone Cable-Suspended Active\n Payload System \n(Derived Dynamics Model)",
        color="#2f6a31", fontsize=12, fontweight="bold", zorder=2)

# ---------- control blocks ----------
dim_ec = "#555555"
dim_fc = "#ffffff"

inp   = add_box(ax, (2.0, 12.5), 5.5, 2.0,
                "Input:\nDrone Target Position",
                fs=12, ec=dim_ec, fc=dim_fc, alpha=0.95)

pos   = add_box(ax, (2.3, 9.7),  4.8, 2.0,
                "Position Control",
                fs=11, ec=dim_ec, fc=dim_fc, alpha=0.95)
invrt = add_box(ax, (8.3, 9.7),  4.8, 2.0,
                "Attitude Inversion",
                fs=11, ec=dim_ec, fc=dim_fc, alpha=0.95)
atti  = add_box(ax, (14.3, 9.7), 4.8, 2.0,
                "Attitude Control",
                fs=11, ec=dim_ec, fc=dim_fc, alpha=0.95)

zref  = add_box(ax, (2.3, 7.2),  4.8, 1.9,
                "Altitude Ref",
                fs=11, ec=dim_ec, fc=dim_fc, alpha=0.95)
alt   = add_box(ax, (14.3, 6.2), 4.8, 2.0,
                "Altitude Control",
                fs=11, ec=dim_ec, fc=dim_fc, alpha=0.95)

p_in  = add_box(ax, (4.8, 2.4),  4.6, 2.2,
                "Payload Inputs\n(xp, yp, zp, vxp, vyp, vzp,\nDrone target pos,\nDrone current pos)",
                fs=10, ec="#a46a1a", fc="#fff8ee", alpha=1.0, z=6)

pctrl = add_shadow_box(ax, (10.0, 2.35),  8.2, 2.35,
                       "Payload Controller\n(MPC / PID)",
                       fc="#fff1df", ec="#f0ad4e", fs=11, z=7)

# ---------- plant 主体 ----------
plant = add_box(ax, (22.8, 3.0), 6.2, 10.8, "",
                fs=11, ec="#2f6a31", fc="#ffffff", alpha=1.0)

# 在 plant 内部添加三个子模块（不写公式）
add_box(ax, (23.2, 11.0), 5.4, 2.4, "Drone Dynamics",
        fs=10.5, ec="#2f6a31", fc="#f5fff8", alpha=1.0, z=5)
add_box(ax, (23.2, 7.6), 5.4, 2.4, "Payload Dynamics",
        fs=10.5, ec="#2f6a31", fc="#f5fff8", alpha=1.0, z=5)
add_box(ax, (23.2, 4.6), 5.4, 2.2, "Cable Coupling",
        fs=10.5, ec="#2f6a31", fc="#f5fff8", alpha=1.0, z=5)

# ---------- wiring ----------
arrow(ax, (inp[0]+inp[2]/2, inp[1]),
      (pos[0]+pos[2]/2, pos[1]+pos[3]), "", z=3)
arrow(ax, (pos[0]+pos[2], pos[1]+pos[3]/2),
      (invrt[0], invrt[1]+invrt[3]/2), "Fx, Fy", z=3)
arrow(ax, (invrt[0]+invrt[2], invrt[1]+invrt[3]/2),
      (atti[0],  atti[1]+atti[3]/2), "", z=3)
arrow(ax, (zref[0]+zref[2], zref[1]+zref[3]/2),
      (alt[0], alt[1]+alt[3]/2), "zd", z=3)

arrow(ax, (atti[0]+atti[2], atti[1]+atti[3]/2),
      (plant[0], plant[1]+plant[3]*0.72),
      "Attitude Torque", z=3)
arrow(ax, (alt[0]+alt[2],   alt[1]+alt[3]/2),
      (plant[0], plant[1]+plant[3]*0.5),
      "Total Thrust", z=3)

arrow(ax, (p_in[0]+p_in[2], p_in[1]+p_in[3]/2),
      (pctrl[0], pctrl[1]+pctrl[3]/2), "", z=7)

arrow(ax, (pctrl[0]+pctrl[2], pctrl[1]+pctrl[3]/2),
      (plant[0], plant[1]+plant[3]*0.2),
      "fp_x, fp_y", color="#d9534f", lw=2.8,
      fs=11, z=8, glow=True)

# ---------- “Payload 控制” 贡献虚线分组框 ----------
grp_x, grp_y = 4.5, 2.0
grp_w, grp_h = 14.2, 3.2
group = FancyBboxPatch((grp_x, grp_y), grp_w, grp_h,
                       boxstyle="round,pad=0.02,rounding_size=0.15",
                       linewidth=2.2, edgecolor="#f0ad4e", facecolor="none",
                       linestyle="--", zorder=5)
ax.add_patch(group)
ax.text(grp_x+grp_w-0.1, grp_y+grp_h+0.25,
        "Payload Controller Contribution (New)", ha="right", va="bottom",
        fontsize=11, color="#a35e00", fontweight="bold", zorder=6)

# ---------- “Derived Dynamics” 绿色虚线分组（包住整个 plant） ----------
model_grp = FancyBboxPatch((22.55, 2.7), 6.7, 11.4,
                           boxstyle="round,pad=0.02,rounding_size=0.15",
                           linewidth=2.2, edgecolor="#2f6a31",
                           facecolor="none", linestyle="--", zorder=6)
ax.add_patch(model_grp)
ax.text(29.1, 14.2, "Derived Dynamics Contribution", ha="right", va="bottom",
        fontsize=11, color="#2f6a31", fontweight="bold", zorder=7)

# ---------- Feedback闭环（底部绕行到最左侧再分叉） ----------
feedback_color = "#117a65"
feedback_lw = 2.2
feedback_fs = 9.5
z_fb = 6   # 保证在背景之上

# Plant 底部中心
plant_bottom = (plant[0] + plant[2]/2, plant[1])
y_bottom = 1.0        # 底部高度
x_far_left = 1.0      # 绕到白框左边的 x 坐标（更靠左）

# ========== 1. 从 Plant 到最左侧（无箭头） ==========
ax.plot([plant_bottom[0], plant_bottom[0]], [plant_bottom[1], y_bottom],
        color=feedback_color, linewidth=feedback_lw, zorder=z_fb)  # 竖直段
ax.plot([plant_bottom[0], x_far_left], [y_bottom, y_bottom],
        color=feedback_color, linewidth=feedback_lw, zorder=z_fb)  # 底部水平段

# 分叉点（小圆点）
ax.plot(x_far_left, y_bottom, marker="o", markersize=5,
        color=feedback_color, zorder=z_fb+1)

# ========== 2. 分叉一路 → Payload Inputs ==========
p_in_left = (p_in[0], p_in[1] + p_in[3]/2)
ax.plot([x_far_left, x_far_left], [y_bottom, p_in_left[1]],
        color=feedback_color, linewidth=feedback_lw, zorder=z_fb)  # 竖直段
arrow(ax, (x_far_left, p_in_left[1]), p_in_left,
      "Feedback: \npayload/drone states", color=feedback_color,
      lw=feedback_lw, fs=feedback_fs, z=z_fb)

# ========== 3. 分叉另一路 → Position Control ==========
pos_left = (pos[0], pos[1] + pos[3]/2)
ax.plot([x_far_left, x_far_left], [y_bottom, pos_left[1]],
        color=feedback_color, linewidth=feedback_lw, zorder=z_fb)  # 竖直段
arrow(ax, (x_far_left, pos_left[1]), pos_left,
      "Feedback: \nstate estimate", color=feedback_color,
      lw=feedback_lw, fs=feedback_fs, z=z_fb)

# ========== 4. 分叉第三路 → Altitude Control ==========
alt_left = (alt[0], alt[1] + alt[3]/6)
ax.plot([x_far_left, x_far_left], [y_bottom, alt_left[1]],
        color=feedback_color, linewidth=feedback_lw, zorder=z_fb)  # 竖直段
arrow(ax, (x_far_left, alt_left[1]), alt_left,
      "Feedback: altitude z", color=feedback_color,
      lw=feedback_lw, fs=feedback_fs, z=z_fb)


plt.tight_layout()
plt.show()
