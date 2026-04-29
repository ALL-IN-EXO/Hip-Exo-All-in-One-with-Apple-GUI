#!/usr/bin/env python3
"""
Windowed 5s comparison for PF-IMU:
- emphasize deployment difficulty vs simulation
- visualize phase lag / amplitude mismatch / window-to-window variability
"""

from __future__ import annotations

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[2]
TRACE_CSV = (
    ROOT
    / "Algorithm Reference"
    / "Zhemin Reference"
    / "pf_imu_compare_results_50_150s"
    / "pf_imu_zhemin_vs_local_traces_50_150s.csv"
)
OUT_DIR = ROOT / "Algorithm Reference" / "Zhemin Reference" / "pf_imu_compare_results_50_150s"


def metric(x: np.ndarray, y: np.ndarray):
    err = y - x
    return {
        "corr": float(np.corrcoef(x, y)[0, 1]),
        "rmse": float(np.sqrt(np.mean(np.square(err)))),
        "mae": float(np.mean(np.abs(err))),
        "bias": float(np.mean(err)),
    }


def best_lag_metric(x: np.ndarray, y: np.ndarray, max_lag_frames: int = 8):
    best = None
    for lag in range(-max_lag_frames, max_lag_frames + 1):
        if lag > 0:
            xx, yy = x[lag:], y[:-lag]
        elif lag < 0:
            xx, yy = x[:lag], y[-lag:]
        else:
            xx, yy = x, y
        if len(xx) < 20:
            continue
        c = float(np.corrcoef(xx, yy)[0, 1])
        e = float(np.sqrt(np.mean(np.square(yy - xx))))
        if (best is None) or (c > best["corr"]):
            best = {"lag_frames": lag, "lag_ms": lag * 10.0, "corr": c, "rmse": e}
    return best


def shift_series(y: np.ndarray, lag_frames: int):
    # advance y for negative lag (y delayed), delay y for positive lag
    out = np.full_like(y, np.nan, dtype=float)
    if lag_frames < 0:
        k = -lag_frames
        out[:-k] = y[k:]
    elif lag_frames > 0:
        k = lag_frames
        out[k:] = y[:-k]
    else:
        out[:] = y
    return out


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    df = pd.read_csv(TRACE_CSV)

    t = df["t_s"].to_numpy(float)
    windows = []
    t0 = float(np.floor(t.min()))
    t1 = float(np.ceil(t.max()))
    cur = 50.0 if t0 < 50 else t0
    while cur + 5.0 <= 150.0 + 1e-9:
        windows.append((cur, cur + 5.0))
        cur += 5.0

    rows = []
    for side in ["L", "R"]:
        mat = df[f"mat_tau_{side}"].to_numpy(float)
        ref = df[f"local_ref_mcmean_{side}"].to_numpy(float)
        dep = df[f"local_deploy_mcmean_{side}"].to_numpy(float)
        for ws, we in windows:
            m = (t >= ws) & (t < we)
            if m.sum() < 20:
                continue
            x = mat[m]
            y_ref = ref[m]
            y_dep = dep[m]
            m_ref = metric(x, y_ref)
            m_dep = metric(x, y_dep)
            b_ref = best_lag_metric(x, y_ref, max_lag_frames=8)
            b_dep = best_lag_metric(x, y_dep, max_lag_frames=8)
            rows.append(
                {
                    "side": side,
                    "window_start_s": ws,
                    "window_end_s": we,
                    "pair": "matlab_vs_ref_mc",
                    "corr_0lag": m_ref["corr"],
                    "rmse_0lag": m_ref["rmse"],
                    "mae_0lag": m_ref["mae"],
                    "bias_0lag": m_ref["bias"],
                    "best_lag_ms": b_ref["lag_ms"],
                    "best_lag_corr": b_ref["corr"],
                    "best_lag_rmse": b_ref["rmse"],
                }
            )
            rows.append(
                {
                    "side": side,
                    "window_start_s": ws,
                    "window_end_s": we,
                    "pair": "matlab_vs_deploy_mc",
                    "corr_0lag": m_dep["corr"],
                    "rmse_0lag": m_dep["rmse"],
                    "mae_0lag": m_dep["mae"],
                    "bias_0lag": m_dep["bias"],
                    "best_lag_ms": b_dep["lag_ms"],
                    "best_lag_corr": b_dep["corr"],
                    "best_lag_rmse": b_dep["rmse"],
                }
            )

    met = pd.DataFrame(rows)
    met_path = OUT_DIR / "pf_imu_windowed_5s_metrics.csv"
    met.to_csv(met_path, index=False)

    # use global deploy lag from whole-sequence metrics if available; else -20ms default
    try:
        whole = pd.read_csv(OUT_DIR / "pf_imu_zhemin_vs_local_metrics_50_150s.csv")
        lagL = int(round(float(whole[(whole.side == "L") & (whole.pair == "matlab vs local_deploy_mcmean")]["best_lag_frames"].iloc[0])))
        lagR = int(round(float(whole[(whole.side == "R") & (whole.pair == "matlab vs local_deploy_mcmean")]["best_lag_frames"].iloc[0])))
    except Exception:
        lagL = lagR = -2

    for side, lag in [("L", lagL), ("R", lagR)]:
        mat = df[f"mat_tau_{side}"].to_numpy(float)
        ref = df[f"local_ref_mcmean_{side}"].to_numpy(float)
        dep = df[f"local_deploy_mcmean_{side}"].to_numpy(float)
        dep_aligned = shift_series(dep, lag)

        fig, axes = plt.subplots(4, 5, figsize=(23, 13), sharey=True)
        axes = axes.reshape(-1)
        for i, (ws, we) in enumerate(windows):
            ax = axes[i]
            m = (t >= ws) & (t < we)
            tw = t[m] - ws
            ax.plot(tw, mat[m], lw=1.5, label="MATLAB", color="#111111")
            ax.plot(tw, ref[m], lw=1.1, label="Local ref MC", color="#1f77b4")
            ax.plot(tw, dep[m], lw=1.0, label="Deploy MC (raw)", color="#d62728")
            ax.plot(tw, dep_aligned[m], lw=1.0, ls="--", label=f"Deploy MC (lag-comp {lag*10}ms)", color="#2ca02c")
            ax.set_title(f"{ws:.0f}-{we:.0f}s")
            ax.grid(alpha=0.25)
            if i % 5 == 0:
                ax.set_ylabel("Torque (Nm)")
            if i >= 15:
                ax.set_xlabel("Window time (s)")
        handles, labels = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc="upper center", ncol=4, frameon=False)
        fig.suptitle(f"PF-IMU {side}-leg torque comparison in 5s windows (50-150s)", y=0.98, fontsize=14)
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        out = OUT_DIR / f"pf_imu_windowed_5s_{side}_grid.png"
        fig.savefig(out, dpi=170)
        plt.close(fig)

    # Summary chart for windowed metrics.
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    for r, side in enumerate(["L", "R"]):
        sub_ref = met[(met.side == side) & (met.pair == "matlab_vs_ref_mc")].sort_values("window_start_s")
        sub_dep = met[(met.side == side) & (met.pair == "matlab_vs_deploy_mc")].sort_values("window_start_s")
        x = sub_ref["window_start_s"].to_numpy(float)

        axes[r, 0].plot(x, sub_ref["corr_0lag"], marker="o", lw=1.2, label="ref 0-lag")
        axes[r, 0].plot(x, sub_dep["corr_0lag"], marker="o", lw=1.2, label="deploy 0-lag")
        axes[r, 0].plot(x, sub_dep["best_lag_corr"], marker="o", lw=1.2, label="deploy best-lag")
        axes[r, 0].set_title(f"{side} corr by 5s window")
        axes[r, 0].grid(alpha=0.3)
        axes[r, 0].set_ylabel("corr")
        axes[r, 0].legend()

        axes[r, 1].plot(x, sub_ref["rmse_0lag"], marker="o", lw=1.2, label="ref 0-lag")
        axes[r, 1].plot(x, sub_dep["rmse_0lag"], marker="o", lw=1.2, label="deploy 0-lag")
        axes[r, 1].plot(x, sub_dep["best_lag_rmse"], marker="o", lw=1.2, label="deploy best-lag")
        axes[r, 1].set_title(f"{side} RMSE by 5s window")
        axes[r, 1].grid(alpha=0.3)
        axes[r, 1].set_ylabel("RMSE (Nm)")
        axes[r, 1].legend()
    axes[1, 0].set_xlabel("window start (s)")
    axes[1, 1].set_xlabel("window start (s)")
    fig.tight_layout()
    out_summary = OUT_DIR / "pf_imu_windowed_5s_metric_trends.png"
    fig.savefig(out_summary, dpi=170)
    plt.close(fig)

    print(f"[saved] {met_path}")
    print(f"[saved] {OUT_DIR / 'pf_imu_windowed_5s_L_grid.png'}")
    print(f"[saved] {OUT_DIR / 'pf_imu_windowed_5s_R_grid.png'}")
    print(f"[saved] {out_summary}")


if __name__ == "__main__":
    main()

