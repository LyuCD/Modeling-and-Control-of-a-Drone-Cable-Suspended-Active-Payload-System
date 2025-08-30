import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

# ======== 配置 ========
CSV_NO  = "error_log_20250808_065243.csv"
CSV_PID = "error_log_20250808_071708.csv"
CSV_MPC = "error_log_20250808_064332.csv"

OUT_DIR = "figs_all"
os.makedirs(OUT_DIR, exist_ok=True)

# 收敛判据与显示
SETTLING_MODE = "angle"   # "angle" 或 "error"
THETA_EPS_DEG = 2.0       # 角度阈值
ERR_EPS = 0.05            # 误差阈值 (m)
HOLD_SEC = 1.0            # 持续时间
# 无 z 时的备选绳长
L_BACKUP = 0.6

# ======== 工具函数 ========
def load_one(path):
    # 读 CSV；把 "--" 记为 NaN；过滤 SUMMARY
    df = pd.read_csv(path, na_values=["--"], comment="#")
    df.columns = [c.strip() for c in df.columns]
    time_num = pd.to_numeric(df['time'], errors='coerce')
    df = df[time_num.notna()].copy()

    # 全列转数值
    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors='coerce')
    df.dropna(axis=1, how='all', inplace=True)

    # 相对时间
    t0 = df['time'].iloc[0]
    df['t'] = df['time'] - t0

    # 只保留前 10 秒
    df = df[df['t'] <= 10].reset_index(drop=True)
    return df

def theta_from_xyz(df):
    dx = df['payload_x'] - df['drone_x']
    dy = df['payload_y'] - df['drone_y']
    if 'drone_z' in df.columns and 'payload_z' in df.columns:
        dz = df['drone_z'] - df['payload_z']
        th = np.degrees(np.arctan2(np.sqrt(dx**2 + dy**2), np.abs(dz)))
    else:
        th = np.degrees(np.arctan2(np.sqrt(dx**2 + dy**2), L_BACKUP))
    return th

def err_norm_from_xy(df):
    ex = df['payload_x'] - df['drone_x']
    ey = df['payload_y'] - df['drone_y']
    return np.sqrt(ex**2 + ey**2)

def settling_time(t, y, eps, hold=HOLD_SEC):
    if len(t) < 2:
        return np.nan
    dt = float(np.mean(np.diff(t)))
    win = max(1, int(round(hold/dt)))
    within = (np.abs(y) <= eps).astype(int)
    if len(within) >= win:
        csum = np.convolve(within, np.ones(win, dtype=int), mode='valid')
        idxs = np.where(csum == win)[0]
        if len(idxs) > 0:
            return float(t.iloc[idxs[0]])
    return np.nan

def metrics_from_df(df):
    t = df['t']
    theta = df['theta_deg'] if 'theta_deg' in df.columns else theta_from_xyz(df)
    errn  = df['err_norm']  if 'err_norm'  in df.columns else err_norm_from_xy(df)
    peak  = float(np.nanmax(np.abs(theta)))
    peak_t = float(t.iloc[int(np.nanargmax(np.abs(theta.values)))])
    if SETTLING_MODE == "angle":
        ts = settling_time(t, theta, THETA_EPS_DEG)
    else:
        ts = settling_time(t, errn, ERR_EPS)
    # IAE / ISE
    iae = np.trapz(errn.fillna(0), t)
    ise = np.trapz((errn.fillna(0))**2, t)
    # 控制努力
    fmag = df['fmag'] if 'fmag' in df.columns else np.hypot(df.get('fx',0), df.get('fy',0))
    effort_l1 = np.trapz(fmag.fillna(0), t)
    effort_l2 = np.trapz((fmag.fillna(0))**2, t)
    # 饱和/松绳
    sat_ratio = float(np.nanmean(df.get('saturated', pd.Series([0]*len(df)))))
    slack_ratio = float(np.nanmean(df.get('slack', pd.Series([0]*len(df)))))
    return dict(PeakTiltDeg=peak, PeakTime=peak_t, SettlingTime=ts,
                IAE=iae, ISE=ise, Effort_L1=effort_l1, Effort_L2=effort_l2,
                SatRatio=sat_ratio, SlackRatio=slack_ratio)

def pad(lo, hi, r=0.05):
    span = (hi - lo) if (hi - lo) > 0 else 1.0
    p = span * r
    return lo - p, hi + p

# ======== 读取数据 ========
cases = [(CSV_NO,"No Control"), (CSV_PID,"PID"), (CSV_MPC,"MPC")]
dfs = [load_one(p) for p,_ in cases]

# ======== 统一轴范围 ========
# X/Y 位置
xmins = [min(df['drone_x'].min(), df['payload_x'].min()) for df in dfs]
xmaxs = [max(df['drone_x'].max(), df['payload_x'].max()) for df in dfs]
ymins = [min(df['drone_y'].min(), df['payload_y'].min()) for df in dfs]
ymaxs = [max(df['drone_y'].max(), df['payload_y'].max()) for df in dfs]
ylims_pos = (pad(min(xmins), max(xmaxs)), pad(min(ymins), max(ymaxs)))

# Tilt
thmins, thmaxs = [], []
for df in dfs:
    th = df['theta_deg'] if 'theta_deg' in df.columns else theta_from_xyz(df)
    thmins.append(np.nanmin(th)); thmaxs.append(np.nanmax(th))
ylim_tilt = pad(min(thmins), max(thmaxs))

# |e|
enmins, enmaxs = [], []
for df in dfs:
    en = df['err_norm'] if 'err_norm' in df.columns else err_norm_from_xy(df)
    enmins.append(np.nanmin(en)); enmaxs.append(np.nanmax(en))
ylim_err = pad(min(enmins), max(enmaxs))

# |F|
fmins, fmaxs = [], []
for df in dfs:
    fm = df['fmag'] if 'fmag' in df.columns else np.hypot(df.get('fx',0), df.get('fy',0))
    fmins.append(np.nanmin(fm)); fmaxs.append(np.nanmax(fm))
ylim_f = pad(0.0, max(fmaxs))

# 估计 Fmax（若 saturated 列存在，用饱和样本中位数）
FMAX = None
for df in dfs:
    if 'saturated' in df.columns and 'fmag' in df.columns:
        sat_vals = df.loc[df['saturated'] == 1, 'fmag']
        if len(sat_vals) > 5:
            FMAX = float(np.nanmedian(sat_vals))
            break
if FMAX is None:
    FMAX = float(np.nanmax(fmaxs))

# ======== Panel A：主面板（只包含曲线） ========
fig = plt.figure(figsize=(13, 10))
gs = fig.add_gridspec(5, 3)  # X / Y / Tilt / |e| / |F|

metrics_rows = []
for i, ((path, name), df) in enumerate(zip(cases, dfs)):
    t = df['t']
    uav_x, uav_y = df['drone_x'], df['drone_y']
    pl_x, pl_y   = df['payload_x'], df['payload_y']
    theta = df['theta_deg'] if 'theta_deg' in df.columns else theta_from_xyz(df)
    errn  = df['err_norm']  if 'err_norm'  in df.columns else err_norm_from_xy(df)
    fmag  = df['fmag'] if 'fmag' in df.columns else np.hypot(df.get('fx',0), df.get('fy',0))
    sat   = df.get('saturated', pd.Series([0]*len(df)))

    # 指标
    m = metrics_from_df(df); m['Mode'] = name; metrics_rows.append(m)

    # X
    ax = fig.add_subplot(gs[0, i])
    ax.plot(t, uav_x, label='UAV X')
    ax.plot(t, pl_x, '--', label='Payload X')
    ax.set_title(f"{name} - X position")
    ax.set_ylabel("X (m)")
    ax.grid(True)
    ax.legend(fontsize=8)
    ax.set_ylim(*ylims_pos[0])

    # Y
    ax = fig.add_subplot(gs[1, i])
    ax.plot(t, uav_y, label='UAV Y')
    ax.plot(t, pl_y, '--', label='Payload Y')
    ax.set_title(f"{name} - Y position")
    ax.set_ylabel("Y (m)")
    ax.grid(True)
    ax.legend(fontsize=8)
    ax.set_ylim(*ylims_pos[1])

    # Tilt
    ax = fig.add_subplot(gs[2, i])
    ax.plot(t, theta, label='Tilt (deg)')
    if SETTLING_MODE == "angle":
        ax.axhline(THETA_EPS_DEG, ls=':', color='gray')
        ax.axhline(-THETA_EPS_DEG, ls=':', color='gray')
    ax.set_title(f"{name} - Tilt")
    ax.set_ylabel("Tilt (°)")
    ax.grid(True)
    ax.set_ylim(*ylim_tilt)
# 标注峰值（用相对像素偏移，避免重叠）
    ax.axvline(m['PeakTime'], ls='--', color='red')
    ax.annotate(f"Peak {m['PeakTiltDeg']:.2f}°@{m['PeakTime']:.2f}s",
                xy=(m['PeakTime'], m['PeakTiltDeg']),
                xytext=(10, -5),              # 向右上偏移 20 像素
                textcoords='offset points',
                arrowprops=dict(arrowstyle='->', color='red'),
                fontsize=8)

    # 标注收敛时间（同样用相对像素偏移）
    if not np.isnan(m['SettlingTime']):
        ax.axvline(m['SettlingTime'], ls='--', color='green')
        ax.annotate(f"Settle {m['SettlingTime']:.2f}s",
                    xy=(m['SettlingTime'], 0),
                    xytext=(20, -30),         # 向右下偏移，避开横轴
                    textcoords='offset points',
                    arrowprops=dict(arrowstyle='->', color='green'),
                    fontsize=8)
    # |e|
    ax = fig.add_subplot(gs[3, i])
    ax.plot(t, errn, label='|e| (m)')
    if SETTLING_MODE == "error":
        ax.axhline(ERR_EPS, ls=':', color='gray', label='error band')
    ax.set_title(f"{name} - Error norm")
    ax.set_ylabel("|e| (m)")
    ax.grid(True)
    ax.set_ylim(*ylim_err)

    # |F|
    ax = fig.add_subplot(gs[4, i])
    ax.plot(t, fmag, label='|F|')
    ax.axhline(FMAX, ls=':', color='gray', label='Fmax')
    try:
        ax.fill_between(t, 0, fmag, where=(sat==1), alpha=0.2, step='mid', label='saturated')
    except Exception:
        pass
    ax.set_title(f"{name} - Control effort")
    ax.set_ylabel("|F|")
    ax.set_xlabel("Time (s)")
    ax.grid(True)
    ax.set_ylim(*ylim_f)
    if i == 0:
        ax.legend(fontsize=8)

plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, "panel_A_main.png"), dpi=300)
plt.close(fig)

# ======== Panel A 表格（单独成图：上下两张子表） ========
fig = plt.figure(figsize=(10, 3.8))
gs = fig.add_gridspec(2, 1, height_ratios=[1, 1])

# 上半：Method + Max Tilt / Settling / IAE / ISE
ax_table_top = fig.add_subplot(gs[0, 0])
ax_table_top.axis('off')
cols_top = ["Control Method","Max Tilt (°)","Settling Time (s)","IAE(|e|)","ISE(|e|)"]
rows_top = []
for m in metrics_rows:
    rows_top.append([
        m['Mode'],
        f"{m['PeakTiltDeg']:.2f}",
        "--" if np.isnan(m['SettlingTime']) else f"{m['SettlingTime']:.2f}",
        f"{m['IAE']:.3f}",
        f"{m['ISE']:.3f}",
    ])
tbl_top = ax_table_top.table(cellText=rows_top, colLabels=cols_top,
                             loc='center', cellLoc='center')
tbl_top.auto_set_font_size(False); tbl_top.set_fontsize(9); tbl_top.scale(1.1, 1.25)

# 下半：Method + Effort L1 / L2 / Sat / Slack
ax_table_bot = fig.add_subplot(gs[1, 0])
ax_table_bot.axis('off')
cols_bot = ["Control Method","Effort L1","Effort L2","Sat Ratio","Slack Ratio"]
rows_bot = []
for m in metrics_rows:
    rows_bot.append([
        m['Mode'],
        f"{m['Effort_L1']:.3f}",
        f"{m['Effort_L2']:.3f}",
        f"{100*m['SatRatio']:.1f}%",
        f"{100*m['SlackRatio']:.1f}%",
    ])
tbl_bot = ax_table_bot.table(cellText=rows_bot, colLabels=cols_bot,
                             loc='center', cellLoc='center')
tbl_bot.auto_set_font_size(False); tbl_bot.set_fontsize(9); tbl_bot.scale(1.1, 1.25)

plt.tight_layout()
plt.savefig(os.path.join(OUT_DIR, "panel_A_tables.png"), dpi=300)
plt.close(fig)

# ======== Panel B：误差分量 (ex/ey) ========
fig = plt.figure(figsize=(13, 7))
gs = fig.add_gridspec(2, 3)
for i, ((path,name), df) in enumerate(zip(cases, dfs)):
    t = df['t']; ex = df['err_x']; ey = df['err_y']
    ax = fig.add_subplot(gs[0,i]); ax.plot(t, ex); ax.grid(True); ax.set_title(f"{name} - e_x"); ax.set_ylabel("e_x (m)")
    ax.axhline(ERR_EPS, ls=':', color='gray'); ax.axhline(-ERR_EPS, ls=':', color='gray')
    ax = fig.add_subplot(gs[1,i]); ax.plot(t, ey); ax.grid(True); ax.set_title(f"{name} - e_y"); ax.set_ylabel("e_y (m)"); ax.set_xlabel("Time (s)")
    ax.axhline(ERR_EPS, ls=':', color='gray'); ax.axhline(-ERR_EPS, ls=':', color='gray')
plt.tight_layout(); plt.savefig(os.path.join(OUT_DIR,"panel_B_error_components.png"), dpi=300); plt.close(fig)

# ======== Panel C：θ 分量 (theta_x/theta_y) ========
fig = plt.figure(figsize=(13, 7))
gs = fig.add_gridspec(2, 3)
for i, ((path,name), df) in enumerate(zip(cases, dfs)):
    t = df['t']
    tx = df['theta_x_deg'] if 'theta_x_deg' in df.columns else np.degrees(np.arctan2(df['payload_x']-df['drone_x'], df['drone_z']-df['payload_z']))
    ty = df['theta_y_deg'] if 'theta_y_deg' in df.columns else np.degrees(np.arctan2(df['payload_y']-df['drone_y'], df['drone_z']-df['payload_z']))
    ax = fig.add_subplot(gs[0,i]); ax.plot(t, tx); ax.grid(True); ax.set_title(f"{name} - θx"); ax.set_ylabel("deg")
    ax = fig.add_subplot(gs[1,i]); ax.plot(t, ty); ax.grid(True); ax.set_title(f"{name} - θy"); ax.set_ylabel("deg"); ax.set_xlabel("Time (s)")
plt.tight_layout(); plt.savefig(os.path.join(OUT_DIR,"panel_C_theta_components.png"), dpi=300); plt.close(fig)

# ======== Panel D：|F| 直方图 ========
fig, axes = plt.subplots(1,3, figsize=(13,4), sharey=True)
for ax, ((path,name), df) in zip(axes, zip(cases, dfs)):
    fmag = df['fmag'] if 'fmag' in df.columns else np.hypot(df.get('fx',0), df.get('fy',0))
    ax.hist(fmag.dropna(), bins=30)
    ax.axvline(FMAX, ls=':', color='gray')
    ax.set_title(f"{name} - |F| histogram"); ax.set_xlabel("|F|"); ax.grid(True)
axes[0].set_ylabel("count")
plt.tight_layout(); plt.savefig(os.path.join(OUT_DIR,"panel_D_force_hist.png"), dpi=300); plt.close(fig)

# ======== 导出指标表 ========
metrics_df = pd.DataFrame(metrics_rows)[["Mode","PeakTiltDeg","SettlingTime","IAE","ISE","Effort_L1","Effort_L2","SatRatio","SlackRatio"]]
metrics_df.to_csv(os.path.join(OUT_DIR,"metrics_summary.csv"), index=False)

# 独立英文汇总表（9 列，论文备用）
fig, ax = plt.subplots(figsize=(8,1.8))
ax.axis('off')
cell = []
for _,r in metrics_df.iterrows():
    cell.append([r["Mode"],
                 f"{r['PeakTiltDeg']:.2f}",
                 "--" if np.isnan(r['SettlingTime']) else f"{r['SettlingTime']:.2f}",
                 f"{r['IAE']:.3f}", f"{r['ISE']:.3f}",
                 f"{r['Effort_L1']:.3f}", f"{r['Effort_L2']:.3f}",
                 f"{100*r['SatRatio']:.1f}%", f"{100*r['SlackRatio']:.1f}%"])
table = ax.table(cellText=cell,
                 colLabels=["Method","Max Tilt (°)","Settling (s)","IAE(|e|)","ISE(|e|)","Effort L1","Effort L2","Sat (%)","Slack (%)"],
                 loc='center', cellLoc='center')
table.auto_set_font_size(False); table.set_fontsize(9); table.scale(1.1,1.3)
plt.tight_layout(); plt.savefig(os.path.join(OUT_DIR,"metrics_table.png"), dpi=300); plt.close(fig)

print("✅ Done. Saved to:", OUT_DIR)
