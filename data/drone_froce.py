import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

# =================== 论文版显示风格 ===================
# 矢量字体设置（兼容 AI/LaTeX）
plt.rcParams.update({
    'pdf.fonttype': 42,
    'ps.fonttype': 42,
    'font.size': 12,              # 基础字号
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'legend.fontsize': 11,
})
LABEL_FONT = dict(fontsize=12, fontweight='bold')
# 主体线宽/箭头粗细
LW_MAIN = 2.4
LW_AUX  = 1.2
QV_LW   = 2.2
ARROW_RATIO_SMALL = 0.08
ARROW_RATIO_FORCE = 0.18

# =================== 参数区 ===================
L = 0.6                          # 绳长 (m)
B = np.array([0.0, 0.0, 1.0])    # 无人机质心/挂点
dx, dy = 0.3, 0.3                # 负载相对 B 的水平偏移
rotor_offsets = [(0.3, 0.3), (-0.3, 0.3), (-0.3, -0.3), (0.3, -0.3)]
r_rotor = 0.1

# 欧拉角示意大小（右手定则）
phi   = np.deg2rad(180)  # roll  绕 x_B
theta = np.deg2rad(180)  # pitch 绕 y_B
psi   = np.deg2rad(180)  # yaw   绕 z_B

# 绳自旋（γ）小圆弧的参数
gamma_offset_along_rope = 0.15   # 相对负载沿绳向上挪动的距离（m）
gamma_arc_radius = 0.07          # 小圆弧半径
gamma_span = (-0.6*np.pi, 0.4*np.pi)

# ========== 由参数自动求负载位置，确保 |BP| = L ==========
rho2 = dx**2 + dy**2
assert rho2 < L**2, "水平偏移超过绳长，无法找到可行的 z_p！"
z_b = B[2]
z_p = z_b - np.sqrt(L**2 - rho2)
P = np.array([B[0] + dx, B[1] + dy, z_p])

# 单位向量与角度
v_BP = P - B
u_BP = v_BP / np.linalg.norm(v_BP)               # 绳方向（B->P）
alpha = np.arccos(np.clip(np.dot(u_BP, np.array([0,0,-1])), -1, 1))   # 与 -z 的夹角
beta = np.arctan2(dy, dx)                        # 水平投影相对 +x 的角度

# =================== 工具函数 ===================
def draw_arc_between_vectors(ax, center, v_start, angle, axis, r=0.2, n=100, add_arrow=False, arrow_scale=0.08, **kwargs):
    """围绕给定 axis，从 v_start 旋转 angle 画圆弧（罗德里格旋转公式），返回末端点与切向量。"""
    v_start = v_start / (np.linalg.norm(v_start) + 1e-12)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    ts = np.linspace(0, angle, n)
    pts = []
    for t in ts:
        vt = (v_start*np.cos(t) +
              np.cross(axis, v_start)*np.sin(t) +
              axis*np.dot(axis, v_start)*(1-np.cos(t)))
        pts.append(center + r * vt)
    pts = np.array(pts)
    ax.plot(pts[:,0], pts[:,1], pts[:,2], **kwargs)
    v_end = (v_start*np.cos(angle) +
             np.cross(axis, v_start)*np.sin(angle) +
             axis*np.dot(axis, v_start)*(1-np.cos(angle)))
    tangent = np.cross(axis, v_end)
    tangent = tangent / (np.linalg.norm(tangent) + 1e-12)
    end_pt = center + r * v_end
    if add_arrow:
        ax.quiver(end_pt[0], end_pt[1], end_pt[2],
                  tangent[0]*arrow_scale, tangent[1]*arrow_scale, tangent[2]*arrow_scale,
                  color=kwargs.get('color', 'k'), arrow_length_ratio=ARROW_RATIO_SMALL, linewidth=QV_LW*0.6)
    return pts, end_pt, tangent

def perp_basis_from_axis(axis):
    """给定轴向量，构造与之正交的单位基 (e1,e2)。"""
    a = axis / (np.linalg.norm(axis) + 1e-12)
    ref = np.array([1,0,0]) if abs(a[0]) < 0.9 else np.array([0,1,0])
    e1 = np.cross(a, ref); e1 = e1 / (np.linalg.norm(e1) + 1e-12)
    e2 = np.cross(a, e1); e2 = e2 / (np.linalg.norm(e2) + 1e-12)
    return e1, e2

def draw_circle_arc_around_axis(ax, center, axis, r, t0, t1, n=100, add_arrow=False, arrow_scale=0.08, **kwargs):
    """画绕给定 axis 的圆弧（圆面法向即 axis）。"""
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    e1, e2 = perp_basis_from_axis(axis)
    ts = np.linspace(t0, t1, n)
    xs = center[0] + r*np.cos(ts)*e1[0] + r*np.sin(ts)*e2[0]
    ys = center[1] + r*np.cos(ts)*e1[1] + r*np.sin(ts)*e2[1]
    zs = center[2] + r*np.cos(ts)*e1[2] + r*np.sin(ts)*e2[2]
    ax.plot(xs, ys, zs, **kwargs)
    t_end = ts[-1]
    tangent = -np.sin(t_end)*e1 + np.cos(t_end)*e2
    tangent = tangent / (np.linalg.norm(tangent) + 1e-12)
    end_pt = np.array([xs[-1], ys[-1], zs[-1]])
    if add_arrow:
        ax.quiver(end_pt[0], end_pt[1], end_pt[2],
                  tangent[0]*arrow_scale, tangent[1]*arrow_scale, tangent[2]*arrow_scale,
                  color=kwargs.get('color', 'k'), arrow_length_ratio=ARROW_RATIO_SMALL, linewidth=QV_LW*0.6)
    return end_pt, tangent

# =================== 画布 ===================
fig = plt.figure(figsize=(8.5, 7.2))  # 稍紧凑，便于论文排版
ax = fig.add_subplot(111, projection='3d')

# 背景与边框弱化，主体更突出
fig.patch.set_facecolor('white')
ax.set_facecolor('white')
ax.xaxis.pane.set_edgecolor('white')
ax.yaxis.pane.set_edgecolor('white')
ax.zaxis.pane.set_edgecolor('white')
ax.grid(False)

ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([0, 1.5])
ax.set_xlabel("x", fontweight='bold'); ax.set_ylabel("y", fontweight='bold'); ax.set_zlabel("z", fontweight='bold')
ax.set_box_aspect([1,1,1])

# 世界坐标系 {O}（更短箭头，避免喧宾夺主）
world_axis_len = 0.35
ax.quiver(0,0,0, world_axis_len,0,0, color='k', arrow_length_ratio=ARROW_RATIO_SMALL, linewidth=LW_MAIN*0.8)
ax.text(world_axis_len+0.03,0,0, '$x_o$', color='k', **LABEL_FONT)
ax.quiver(0,0,0, 0,world_axis_len,0, color='k', arrow_length_ratio=ARROW_RATIO_SMALL, linewidth=LW_MAIN*0.8)
ax.text(0,world_axis_len+0.03,0, '$y_o$', color='k', **LABEL_FONT)
ax.quiver(0,0,0, 0,0,world_axis_len, color='k', arrow_length_ratio=ARROW_RATIO_SMALL, linewidth=LW_MAIN*0.8)
ax.text(0,0,world_axis_len+0.03, '$z_o$', color='k', **LABEL_FONT)

# 地面与水平投影面（极弱）
Xg, Yg = np.meshgrid(np.linspace(-1,1,6), np.linspace(-1,1,6))
Z0 = np.zeros_like(Xg)
Zp = np.full_like(Xg, z_p)
ax.plot_surface(Xg, Yg, Z0, alpha=0.02, linewidth=0, antialiased=False, color='gray')
ax.plot_surface(Xg, Yg, Zp, alpha=0.03, linewidth=0, antialiased=False, color='gray')

# 无人机中心 B 与机体系 {B}
ax.scatter(*B, c='black', s=18)
ax.text(B[0], B[1], B[2]+0.05, 'B', fontweight='bold')
ax.quiver(*B, 0.48,0,0, color='black', arrow_length_ratio=ARROW_RATIO_SMALL, linewidth=LW_MAIN); ax.text(B[0]+0.52,B[1],B[2], '$x_B$', **LABEL_FONT)
ax.quiver(*B, 0,0.48,0, color='black', arrow_length_ratio=ARROW_RATIO_SMALL, linewidth=LW_MAIN); ax.text(B[0],B[1]+0.52,B[2], '$y_B$', **LABEL_FONT)
ax.quiver(*B, 0,0,0.48, color='black', arrow_length_ratio=ARROW_RATIO_SMALL, linewidth=LW_MAIN); ax.text(B[0],B[1],B[2]+0.52, '$z_B$', **LABEL_FONT)

# ======== 无人机三轴旋转示意（小圆弧 + 箭头） ========
ex = np.array([1,0,0])   # x_B
ey = np.array([0,1,0])   # y_B
ez = np.array([0,0,1])   # z_B
offset_on_axis = 0.22
r_euler = 0.06

center_roll  = B + offset_on_axis * ex
center_pitch = B + offset_on_axis * ey
center_yaw   = B + offset_on_axis * ez

# Roll φ：绕 x_B（绿色）
end_phi, _ = draw_circle_arc_around_axis(
    ax, center=center_roll, axis=ex,
    r=r_euler, t0=-0.6*np.pi, t1=-0.6*np.pi + phi,
    n=140, add_arrow=True, arrow_scale=0.06, color='g', linestyle='-', linewidth=LW_MAIN*0.9)
ax.text(center_roll[0]+r_euler*0.9, center_roll[1], center_roll[2]+0.02, r'$\phi$', color='g', **LABEL_FONT)

# Pitch θ：绕 y_B（青色）
end_theta, _ = draw_circle_arc_around_axis(
    ax, center=center_pitch, axis=ey,
    r=r_euler, t0=0.15*np.pi, t1=0.15*np.pi + theta,
    n=120, add_arrow=True, arrow_scale=0.06, color='c', linestyle='-', linewidth=LW_MAIN*0.9)
ax.text(center_pitch[0], center_pitch[1]+r_euler*0.9, center_pitch[2]+0.02, r'$\theta$', color='c', **LABEL_FONT)

# Yaw ψ：绕 z_B（洋红）
end_psi, _ = draw_circle_arc_around_axis(
    ax, center=center_yaw, axis=ez,
    r=r_euler, t0=-0.2*np.pi, t1=-0.2*np.pi + psi,
    n=120, add_arrow=True, arrow_scale=0.06, color='m', linestyle='-', linewidth=LW_MAIN*0.9)
ax.text(center_yaw[0], center_yaw[1], center_yaw[2]+r_euler*1.05, r'$\psi$', color='m', **LABEL_FONT)

# 架臂与旋翼圆 + 推力箭头命名
for i, (rx, ry) in enumerate(rotor_offsets, start=1):
    ax.plot([B[0], B[0]+rx], [B[1], B[1]+ry], [B[2], B[2]],
            color='dimgray', linewidth=LW_MAIN)
    t_c = np.linspace(0, 2*np.pi, 160)
    xc = B[0]+rx + r_rotor*np.cos(t_c)
    yc = B[1]+ry + r_rotor*np.sin(t_c)
    zc = np.full_like(xc, B[2])
    ax.plot(xc, yc, zc, color='navy', linewidth=LW_MAIN*0.9)

    # 推力箭头（向上）
    ax.quiver(B[0]+rx, B[1]+ry, B[2], 0,0,0.3,
              color='navy', arrow_length_ratio=ARROW_RATIO_FORCE, linewidth=QV_LW)

    # 给箭头加标签 T_1, T_2, T_3, T_4
    ax.text(B[0]+rx, B[1]+ry, B[2]+0.35, f"$T_{i}$", color='navy', **LABEL_FONT)

# 绳索与负载
ax.plot([B[0],P[0]],[B[1],P[1]],[B[2],P[2]], 'k--', linewidth=LW_MAIN, solid_capstyle='round')
ax.scatter(*P, c='gray', s=150, edgecolors='black', linewidths=0.8)
ax.text(P[0]+0.05, P[1]+0.05, P[2]-0.05, r'$(x_p, y_p, z_p)$', **LABEL_FONT)

# 投影辅助（到 z=z_p 的垂线）
ax.plot([B[0],B[0]], [B[1],B[1]], [B[2],z_p], 'k:', linewidth=LW_AUX)

# ======== β 的“直角三角形”虚线平面 + 半透明面 ========
B_proj = np.array([B[0], B[1], z_p])
X_proj = np.array([B[0] + dx, B[1], z_p])   # (x_p, y_b)
P_proj = np.array([P[0], P[1], z_p])        # (x_p, y_p)

ax.plot([B_proj[0], X_proj[0]], [B_proj[1], X_proj[1]], [z_p, z_p], 'k:', linewidth=LW_AUX)
ax.plot([X_proj[0], P_proj[0]], [X_proj[1], P_proj[1]], [z_p, z_p], 'k:', linewidth=LW_AUX)
ax.plot([B_proj[0], P_proj[0]], [B_proj[1], P_proj[1]], [z_p, z_p], 'k:', linewidth=LW_AUX)

tri = Poly3DCollection([[B_proj, X_proj, P_proj]], alpha=0.06, facecolor='tab:blue', edgecolor='none')
ax.add_collection3d(tri)

# β 弧（以 B_proj 为圆心，在 z=z_p 平面，从 +x 旋到 (dx,dy)）
r_beta = 0.22
t_beta = np.linspace(0, beta, 160)
arc_x = B_proj[0] + r_beta*np.cos(t_beta)
arc_y = B_proj[1] + r_beta*np.sin(t_beta)
arc_z = np.full_like(arc_x, z_p)
ax.plot(arc_x, arc_y, arc_z, 'b--', linewidth=LW_MAIN*0.9)
ax.text(arc_x[-1]+0.02, arc_y[-1], arc_z[-1]+0.02, r'$\beta$', color='b', **LABEL_FONT)

# ======== α 角：从 -z 旋到绳方向（B 点附近，虚线） ========
u_minus_z = np.array([0,0,-1])
axis_alpha = np.cross(u_minus_z, u_BP)
if np.linalg.norm(axis_alpha) < 1e-9:
    axis_alpha = np.array([1,0,0])
_, end_alpha, _ = draw_arc_between_vectors(
    ax, center=B, v_start=u_minus_z, angle=alpha, axis=axis_alpha,
    r=0.22, n=160, add_arrow=False, color='b', linestyle='--', linewidth=LW_MAIN*0.9)
ax.text(end_alpha[0], end_alpha[1], end_alpha[2]+0.02, r'$\alpha$', color='b', **LABEL_FONT)

# 主动控制力（负载平面内）
ax.quiver(*P, 0.22,0,0, color='orange', arrow_length_ratio=ARROW_RATIO_FORCE, linewidth=QV_LW)
ax.text(P[0]+0.26, P[1]-0.04, P[2]+0.02, '$f_{px}$', color='orange', **LABEL_FONT)
ax.quiver(*P, 0,0.22,0, color='orange', arrow_length_ratio=ARROW_RATIO_FORCE, linewidth=QV_LW)
ax.text(P[0]-0.06, P[1]+0.26, P[2]+0.02, '$f_{py}$', color='orange', **LABEL_FONT)

# 重力（UAV 与 Payload）
ax.quiver(*B, 0,0,-0.5, color='red', arrow_length_ratio=ARROW_RATIO_FORCE, linewidth=QV_LW)
ax.text(B[0],B[1],B[2]-0.54, '$Mg$', color='red', **LABEL_FONT)
ax.quiver(*P, 0,0,-0.32, color='red', arrow_length_ratio=ARROW_RATIO_FORCE, linewidth=QV_LW)
ax.text(P[0],P[1],P[2]-0.36, '$mg$', color='red', **LABEL_FONT)

# 绳张力方向（沿绳反向）
ax.quiver(*P, *(-0.27*u_BP), color='purple', arrow_length_ratio=ARROW_RATIO_FORCE, linewidth=QV_LW)
ax.text(P[0]-0.28*u_BP[0], P[1]-0.28*u_BP[1], P[2]-0.28*u_BP[2], '$\\mathbf{T}$', color='purple', **LABEL_FONT)

# ======== 绳的“自旋”示意（γ） ========
center_gamma = P - gamma_offset_along_rope * u_BP
end_pt_gamma, _ = draw_circle_arc_around_axis(
    ax, center=center_gamma, axis=u_BP,
    r=gamma_arc_radius, t0=gamma_span[0], t1=gamma_span[1],
    n=140, add_arrow=True, arrow_scale=0.06, color='brown', linestyle='-', linewidth=LW_MAIN*0.9)
ax.text(end_pt_gamma[0]+0.02, end_pt_gamma[1], end_pt_gamma[2]+0.02, r'$\gamma$', color='brown', **LABEL_FONT)

# 绳长注释
mid = (B + P) / 2.0
ax.text(mid[0]+0.05, mid[1], mid[2]+0.05, r'$l=0.6\,\mathrm{m}$', color='black', **LABEL_FONT)

# 视角
ax.view_init(elev=28, azim=132)
plt.tight_layout()

# 高分辨导出（论文用）
plt.savefig("uav_payload_diagram.pdf", dpi=400, bbox_inches='tight')
plt.savefig("uav_payload_diagram.png", dpi=400, bbox_inches='tight')
plt.show()
