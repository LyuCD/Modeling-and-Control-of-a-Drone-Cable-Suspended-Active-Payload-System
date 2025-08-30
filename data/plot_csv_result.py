import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

# ===== 配置 =====
CSV_NO  = "error_log_20250720_132918.csv"   # 无控制
CSV_PID = "error_log_20250720_135632.csv"   # PID
CSV_MPC = "error_log_20250806_232459.csv"   # MPC

OUT_FILE = "comparison_panel_xy.png"
L = 0.6                 # 绳长 (m)，无 z 数据时使用
THETA_EPS_DEG = 2.0     # 收敛阈值 (deg)
HOLD_SEC = 1.0          # 收敛持续时间 (s)

def load_one(path):
    df = pd.read_csv(path)
    t0 = df['time'].iloc[0]
    df['t'] = df['time'] - t0
    return df

def compute_tilt(df):
    dx = df['payload_x'] - df['drone_x']
    dy = df['payload_y'] - df['drone_y']
    if 'drone_z' in df.columns and 'payload_z' in df.columns:
        dz = df['drone_z'] - df['payload_z']
        theta = np.arctan2(np.sqrt(dx**2 + dy**2), np.abs(dz))
    else:
        theta = np.arctan2(np.sqrt(dx**2 + dy**2), L)
    return np.degrees(theta)

def settling_time(t, theta_deg, eps_deg=THETA_EPS_DEG, hold_sec=HOLD_SEC):
    dt = np.mean(np.diff(t))
    win = max(1, int(round(hold_sec / dt)))
    within = (np.abs(theta_deg) <= eps_deg).astype(int)
    if len(within) >= win:
        csum = np.convolve(within, np.ones(win, dtype=int), mode='valid')
        idxs = np.where(csum == win)[0]
        if len(idxs) > 0:
            return float(t.iloc[idxs[0]])
    return np.nan

def plot_case(axs, df, title):
    t = df['t']
    uav_x, uav_y = df['drone_x'], df['drone_y']
    pl_x, pl_y   = df['payload_x'], df['payload_y']
    theta_deg = compute_tilt(df)

    # 最大倾角
    peak_idx = int(np.argmax(np.abs(theta_deg)))
    peak_val = float(theta_deg[peak_idx])
    peak_time = float(t.iloc[peak_idx])
    t_settle = settling_time(t, theta_deg)

    # X 位置
    ax_x, ax_y, ax_tilt = axs
    ax_x.plot(t, uav_x, label='UAV X')
    ax_x.plot(t, pl_x, '--', label='PL X')
    ax_x.set_title(f"{title} - X pos")
    ax_x.set_ylabel("X (m)")
    ax_x.grid(True)
    ax_x.legend(fontsize=8)

    # Y 位置
    ax_y.plot(t, uav_y, label='UAV Y')
    ax_y.plot(t, pl_y, '--', label='PL Y')
    ax_y.set_title(f"{title} - Y pos")
    ax_y.set_ylabel("Y (m)")
    ax_y.grid(True)
    ax_y.legend(fontsize=8)

    # 倾角
    ax_tilt.plot(t, theta_deg, label='Tilt (deg)')
    ax_tilt.axhline(THETA_EPS_DEG, linestyle=':', color='gray')
    ax_tilt.axhline(-THETA_EPS_DEG, linestyle=':', color='gray')
    ax_tilt.axvline(peak_time, linestyle='--', color='red')
    ax_tilt.annotate(f"Peak {peak_val:.2f}°\n@{peak_time:.2f}s",
                     xy=(peak_time, peak_val),
                     xytext=(peak_time, peak_val+5),
                     arrowprops=dict(arrowstyle='->', color='red'),
                     fontsize=8)
    if not np.isnan(t_settle):
        ax_tilt.axvline(t_settle, linestyle='--', color='green')
        ax_tilt.annotate(f"Settle {t_settle:.2f}s",
                         xy=(t_settle, 0),
                         xytext=(t_settle, max(theta_deg)*0.6),
                         arrowprops=dict(arrowstyle='->', color='green'),
                         fontsize=8)
    ax_tilt.set_ylabel("Tilt (°)")
    ax_tilt.set_xlabel("Time (s)")
    ax_tilt.grid(True)

    return dict(Mode=title, PeakTiltDeg=abs(peak_val), SettlingTime=t_settle)

# ===== 主流程 =====
cases = [
    (CSV_NO,  "No Control"),
    (CSV_PID, "PID"),
    (CSV_MPC, "MPC"),
]

fig = plt.figure(figsize=(12, 10))
gs = fig.add_gridspec(5, 3)  # 3列对应三种方法，5行：X / Y / Tilt / 表格

metrics_rows = []
for i, (path, name) in enumerate(cases):
    df = load_one(path)
    # 三行子图：X / Y / Tilt
    ax_x    = fig.add_subplot(gs[0, i])
    ax_y    = fig.add_subplot(gs[1, i])
    ax_tilt = fig.add_subplot(gs[2, i])
    metrics = plot_case((ax_x, ax_y, ax_tilt), df, name)
    metrics_rows.append(metrics)

# 表格占据第4、5行
ax_table = fig.add_subplot(gs[3:, :])
ax_table.axis('off')
col_labels = ["Control Method", "Max Tilt (°)", "Settling Time (s)"]
table_data = [
    [m['Mode'],
     f"{m['PeakTiltDeg']:.2f}",
     "--" if pd.isna(m['SettlingTime']) else f"{m['SettlingTime']:.2f}"]
    for m in metrics_rows
]
table = ax_table.table(cellText=table_data, colLabels=col_labels, loc='center', cellLoc='center')
table.auto_set_font_size(False)
table.set_fontsize(10)
table.scale(1.2, 1.5)

plt.tight_layout()
plt.savefig(OUT_FILE, dpi=300)
plt.show()

# 保存表格数据到 CSV
pd.DataFrame(metrics_rows).to_csv("tilt_summary.csv", index=False)
