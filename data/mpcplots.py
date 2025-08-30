#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

# === 输出文件夹 ===
OUT_DIR = "mpc_figs"
os.makedirs(OUT_DIR, exist_ok=True)

# === 输入 CSV 映射（放当前目录或写成相对路径即可） ===
FILES = {
    0.1:  "error_log_20250808_074905.csv",
    0.2:  "error_log_20250808_075110.csv",
    0.4:  "error_log_20250808_075223.csv",
    0.6:  "error_log_20250808_075352.csv",
    0.8:  "error_log_20250808_075539.csv",
    1.0:  "error_log_20250808_064332.csv",
    1.5:  "error_log_20250808_075721.csv",
    2.0:  "error_log_20250808_075844.csv",
    3.0:  "error_log_20250808_080014.csv",
    5.0:  "error_log_20250808_080216.csv",
    10.0: "error_log_20250808_080345.csv",
    "Unlimited": "error_log_20250808_080548.csv",
}

# === 参数 ===
THETA_EPS_DEG = 2.0       # 收敛阈值（度）
HOLD_SEC = 1.0            # 需连续满足阈值的时间
ONLY_FIRST_10S = True     # 只分析前10秒

def load_df(path):
    """读CSV、过滤非数值行、转相对时间t，可选裁剪到前10s；确保有theta_deg。"""
    df = pd.read_csv(path, na_values=["--"], comment="#")
    df.columns = [c.strip() for c in df.columns]
    time_num = pd.to_numeric(df["time"], errors="coerce")
    df = df[time_num.notna()].copy()
    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    t0 = df["time"].iloc[0]
    df["t"] = df["time"] - t0
    if ONLY_FIRST_10S:
        df = df[df["t"] <= 10.0].reset_index(drop=True)
    if "theta_deg" not in df.columns:
        raise KeyError(f"{path} 缺少 theta_deg 列")
    return df

def settling_time_from_series(t, y_deg, eps_deg=THETA_EPS_DEG, hold_sec=HOLD_SEC):
    """首次进入±eps并连续保持hold_sec的时间；否则NaN。"""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y_deg, dtype=float)
    if t.size < 2:
        return np.nan
    dt_mean = float(np.mean(np.diff(t)))
    if dt_mean <= 0 or not np.isfinite(dt_mean):
        return np.nan
    win = max(1, int(round(hold_sec / dt_mean)))
    within = (np.abs(y) <= eps_deg).astype(int)
    if within.size < win:
        return np.nan
    csum = np.convolve(within, np.ones(win, dtype=int), mode="valid")
    idx = np.where(csum == win)[0]
    return float(t[idx[0]]) if idx.size > 0 else np.nan

def compute_metrics(df):
    """
    返回:
      MaxTilt(deg), SettlingTime(s), MeanTilt(deg),
      Effort_L1(∫|F|dt), Effort_L2(∫F^2dt), SatRatio
    """
    th = df["theta_deg"].abs()
    max_tilt  = float(np.nanmax(th))
    mean_tilt = float(np.nanmean(th))
    t_settle  = settling_time_from_series(df["t"], th)

    # 努力：优先用 fmag，否则由 fx, fy 合成
    if "fmag" in df.columns:
        fmag = df["fmag"].fillna(0).to_numpy(dtype=float)
    else:
        fx = df.get("fx", pd.Series(0, index=df.index)).fillna(0).to_numpy(dtype=float)
        fy = df.get("fy", pd.Series(0, index=df.index)).fillna(0).to_numpy(dtype=float)
        fmag = np.hypot(fx, fy)

    t = df["t"].to_numpy(dtype=float)
    effort_l1 = float(np.trapz(fmag, t))
    effort_l2 = float(np.trapz(fmag**2, t))

    sat_ratio = float(np.nanmean(df["saturated"])) if "saturated" in df.columns else np.nan

    return max_tilt, t_settle, mean_tilt, effort_l1, effort_l2, sat_ratio

# === 读取、计算 ===
metrics = {}
dfs = {}
for limit, csv_file in FILES.items():
    df = load_df(csv_file)
    dfs[limit] = df
    metrics[limit] = compute_metrics(df)

# === 数值型 Fmax 排序 ===
num_limits = sorted([float(k) for k in metrics.keys() if k != "Unlimited"])
x = np.array(num_limits)

max_tilts   = np.array([metrics[l][0] for l in num_limits])
settles     = np.array([metrics[l][1] for l in num_limits])
mean_tilts  = np.array([metrics[l][2] for l in num_limits])
effort_l1   = np.array([metrics[l][3] for l in num_limits])
effort_l2   = np.array([metrics[l][4] for l in num_limits])
sat_ratio   = np.array([metrics[l][5] for l in num_limits])

has_unl = ("Unlimited" in metrics)
unl_vals = metrics["Unlimited"] if has_unl else None

def plot_metric(ax, y, title, ylabel):
    ax.semilogx(x, y, marker='o', linewidth=2)
    ax.grid(True, which='both', ls=':')
    ax.set_title(title)
    ax.set_xlabel("Fmax (N) [log]")
    ax.set_ylabel(ylabel)
    # 标注最优点
    if np.isfinite(y).any():
        i_min = int(np.nanargmin(y))
        ax.scatter([x[i_min]], [y[i_min]], s=80, facecolors='none',
                   edgecolors='red', linewidths=2, zorder=5)
        ax.annotate(f"best @ {x[i_min]:g}N\n{y[i_min]:.2f}",
                    xy=(x[i_min], y[i_min]),
                    xytext=(20, 20),               # 向右上偏移 30px, 20px
                    textcoords='offset points',
                    arrowprops=dict(arrowstyle='->'),
                    fontsize=9)
    # Unlimited 点
    if has_unl and unl_vals is not None:
        x_unl = x.max() * 1.3
        if ylabel.startswith("Max"):
            y_unl = unl_vals[0]
        elif ylabel.startswith("Settling"):
            y_unl = unl_vals[1]
        else:
            y_unl = unl_vals[2]
        ax.scatter([x_unl], [y_unl], marker='^', s=90, facecolors='none',
                   edgecolors='purple', linewidths=2, zorder=6)
        ax.annotate("Unlimited", xy=(x_unl, y_unl),
                    xytext=(x_unl*1.03, y_unl),
                    arrowprops=dict(arrowstyle='->', color='purple'),
                    color='purple', fontsize=9)

# === 图1：性能趋势（Max/Settling/Mean Tilt） ===
fig, axes = plt.subplots(3, 1, figsize=(10, 11), sharex=True)
plot_metric(axes[0], max_tilts,  "Max Tilt vs Fmax",      "Max Tilt (deg)")
plot_metric(axes[1], settles,    f"Settling Time (≤±{THETA_EPS_DEG}°, {HOLD_SEC}s) vs Fmax", "Settling Time (s)")
plot_metric(axes[2], mean_tilts, "Mean Tilt vs Fmax",     "Mean Tilt (deg)")
plt.tight_layout()
fig_path1 = os.path.join(OUT_DIR, "mpc_metrics_trends.png")
plt.savefig(fig_path1, dpi=300)
plt.close(fig)
print(f"✅ 连续趋势图已保存到：{fig_path1}")

# === 图2：控制代价 & 饱和率（Effort L1/L2 & Sat%） ===
fig, ax1 = plt.subplots(figsize=(10, 4.8), constrained_layout=True)
ax1.semilogx(x, effort_l1, marker='o', label='Effort L1 (∫|F|dt)')
ax1.semilogx(x, effort_l2, marker='s', label='Effort L2 (∫F²dt)')
ax1.set_xlabel("Fmax (N) [log]")
ax1.set_ylabel("Control Effort")
ax1.grid(True, which='both', ls=':')

# Unlimited Effort 点
if has_unl and unl_vals is not None:
    x_unl = x.max() * 1.3
    ax1.scatter([x_unl], [unl_vals[3]], marker='^', s=80, facecolors='none',
                edgecolors='purple', label='Unlimited L1')
    ax1.scatter([x_unl], [unl_vals[4]], marker='v', s=80, facecolors='none',
                edgecolors='purple', label='Unlimited L2')

# 饱和率右轴
ax2 = ax1.twinx()
ax2.semilogx(x, 100 * sat_ratio, marker='d', color='tab:orange', label='Sat (%)')
if has_unl and unl_vals is not None and np.isfinite(unl_vals[5]):
    ax2.scatter([x_unl], [100 * unl_vals[5]], marker='D', s=80, facecolors='none',
                edgecolors='tab:orange', label='Unlimited Sat')
ax2.set_ylabel("Saturation (%)")

# 合并图例
lines = ax1.get_lines() + ax2.get_lines()
labels = [l.get_label() for l in lines]
ax1.legend(lines, labels, loc='upper left')

fig_path2 = os.path.join(OUT_DIR, "mpc_effort_sat_trends.png")
plt.savefig(fig_path2, dpi=300)
plt.close(fig)
print(f"✅ Effort/Sat 趋势图已保存到：{fig_path2}")

# === 导出汇总 CSV（含 Effort/Sat） ===
rows = []
for k, v in metrics.items():
    rows.append({
        "Fmax": k,
        "MaxTilt_deg": v[0],
        "Settling_s": v[1],
        "MeanTilt_deg": v[2],
        "Effort_L1": v[3],
        "Effort_L2": v[4],
        "Sat_%": None if np.isnan(v[5]) else 100 * v[5],
    })
csv_path = os.path.join(OUT_DIR, "mpc_fmax_sweep_summary.csv")
pd.DataFrame(rows).to_csv(csv_path, index=False)
print(f"✅ 汇总表已保存到：{csv_path}")
