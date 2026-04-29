#!/usr/bin/env python3
"""
Compare Zhemin MATLAB PF-IMU torque output vs local PF-IMU implementation.

Inputs:
  1) MATLAB reference torque CSV (tau_exo_L/R)
  2) Local input CSV (imu_LTx/imu_RTx/imu_Lvel/imu_Rvel)

Local paths compared:
  - "local_sim": run_reference_leg (MATLAB-style reference logic in consistency tool)
  - "local_deploy": run_pfimu_both (deployed PFIMUController path)

Outputs are written under:
  Algorithm Reference/Zhemin Reference/pf_imu_compare_results_50_150s/
"""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy.io as sio


ROOT = Path(__file__).resolve().parents[2]
OUT_DIR_DEFAULT = ROOT / "Algorithm Reference" / "Zhemin Reference" / "pf_imu_compare_results_50_150s"


def load_consistency_module():
    mod_path = ROOT / "tools" / "pf_imu_consistency_eval.py"
    spec = importlib.util.spec_from_file_location("pf_eval_mod", mod_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load module: {mod_path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def metrics_aligned(x: np.ndarray, y: np.ndarray) -> dict[str, float]:
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    err = y - x
    return {
        "corr": float(np.corrcoef(x, y)[0, 1]),
        "rmse_nm": float(np.sqrt(np.mean(np.square(err)))),
        "mae_nm": float(np.mean(np.abs(err))),
        "bias_nm": float(np.mean(err)),
        "std_x": float(np.std(x)),
        "std_y": float(np.std(y)),
        "amp_ratio_std": float(np.std(y) / (np.std(x) + 1e-12)),
        "max_abs_err_nm": float(np.max(np.abs(err))),
    }


def add_pair_metrics(rows: list[dict], side: str, pair_name: str, x: np.ndarray, y: np.ndarray, lag_fn):
    m0 = metrics_aligned(x, y)
    ml = lag_fn(x, y, max_lag=80)
    rows.append(
        {
            "side": side,
            "pair": pair_name,
            **m0,
            "best_lag_ms": float(ml["lag_ms"]),
            "best_lag_frames": float(ml["lag_frames"]),
            "best_lag_corr": float(ml["corr"]),
            "best_lag_rmse_nm": float(ml["rmse_nm"]),
            "best_lag_mae_nm": float(ml["mae_nm"]),
        }
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--input-csv",
        type=Path,
        default=ROOT / "Algorithm Reference" / "NJIT Reference" / "PI5_lstm_pd-20260424-221451.csv",
    )
    ap.add_argument(
        "--matlab-csv",
        type=Path,
        default=ROOT
        / "Algorithm Reference"
        / "Zhemin Reference"
        / "50_150_210_280"
        / "50_150s"
        / "csv"
        / "control_torque_50_150s.csv",
    )
    ap.add_argument("--t0", type=float, default=50.0)
    ap.add_argument("--t1", type=float, default=150.0)
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument(
        "--matlab-mat",
        type=Path,
        default=ROOT
        / "Algorithm Reference"
        / "Zhemin Reference"
        / "50_150_210_280"
        / "50_150s"
        / "mat"
        / "PF_IMU_v12_50_150s_result.mat",
    )
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR_DEFAULT)
    args = ap.parse_args()

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    mod = load_consistency_module()
    opt = mod.Opt()

    # Input signal used by all local runs.
    raw = pd.read_csv(args.input_csv, usecols=["Time_ms", "imu_LTx", "imu_RTx", "imu_Lvel", "imu_Rvel"])
    raw = raw[(raw["Time_ms"] >= args.t0) & (raw["Time_ms"] <= args.t1)].dropna().reset_index(drop=True)
    t_s = raw["Time_ms"].to_numpy(float)

    # MATLAB reference output from collaborator.
    mat = pd.read_csv(args.matlab_csv)
    mat = mat[(mat["time_abs_s"] >= args.t0) & (mat["time_abs_s"] <= args.t1)].reset_index(drop=True)

    # Ensure exact sample/time alignment.
    n = min(len(raw), len(mat))
    if len(raw) != len(mat):
        print(f"[warn] length mismatch raw={len(raw)} matlab={len(mat)}; truncating to {n}")
    raw = raw.iloc[:n].reset_index(drop=True)
    mat = mat.iloc[:n].reset_index(drop=True)
    t_s = raw["Time_ms"].to_numpy(float)
    dt = np.max(np.abs(mat["time_abs_s"].to_numpy(float) - t_s))
    print(f"[info] samples={n}, max |time_abs_s - Time_ms|={dt:.6g}s")

    # Single-seed comparison.
    l_ref_s = mod.run_reference_leg(raw["imu_LTx"].to_numpy(float), raw["imu_Lvel"].to_numpy(float), t_s, opt, seed=1)
    r_ref_s = mod.run_reference_leg(raw["imu_RTx"].to_numpy(float), raw["imu_Rvel"].to_numpy(float), t_s, opt, seed=1001)
    l_dep_s, r_dep_s = mod.run_pfimu_both(raw, opt, seed=1)

    # MATLAB-v12-parameter profile (diagnostic run).
    opt_mat = mod.Opt()
    try:
        mat_opt = sio.loadmat(args.matlab_mat, simplify_cells=True)["result"]["opt"]
        opt_mat.num_particles = int(mat_opt.get("numParticles", opt_mat.num_particles))
        opt_mat.smooth_angle_window = int(mat_opt.get("smoothAngleWindow", opt_mat.smooth_angle_window))
        opt_mat.smooth_velocity_window = int(mat_opt.get("smoothVelocityWindow", opt_mat.smooth_velocity_window))
        opt_mat.sigma_a = float(mat_opt.get("sigmaA", opt_mat.sigma_a))
        opt_mat.sigma_qstar = float(mat_opt.get("sigmaQstar", opt_mat.sigma_qstar))
        opt_mat.p_switch = float(mat_opt.get("pSwitch", opt_mat.p_switch))
    except Exception as e:
        print(f"[warn] failed loading matlab .mat options ({args.matlab_mat}): {e}; skip matcfg run.")
        mat_opt = None

    if mat_opt is not None:
        l_ref_matcfg = mod.run_reference_leg(
            raw["imu_LTx"].to_numpy(float), raw["imu_Lvel"].to_numpy(float), t_s, opt_mat, seed=1
        )
        r_ref_matcfg = mod.run_reference_leg(
            raw["imu_RTx"].to_numpy(float), raw["imu_Rvel"].to_numpy(float), t_s, opt_mat, seed=1001
        )
        l_dep_matcfg, r_dep_matcfg = mod.run_pfimu_both(raw, opt_mat, seed=1)
    else:
        l_ref_matcfg = np.full_like(l_ref_s, np.nan)
        r_ref_matcfg = np.full_like(r_ref_s, np.nan)
        l_dep_matcfg = np.full_like(l_dep_s, np.nan)
        r_dep_matcfg = np.full_like(r_dep_s, np.nan)

    # MC-mean comparison.
    l_ref_mc, r_ref_mc, l_dep_mc, r_dep_mc = [], [], [], []
    for s in range(1, args.seeds + 1):
        l_ref_mc.append(
            mod.run_reference_leg(raw["imu_LTx"].to_numpy(float), raw["imu_Lvel"].to_numpy(float), t_s, opt, seed=s)
        )
        r_ref_mc.append(
            mod.run_reference_leg(raw["imu_RTx"].to_numpy(float), raw["imu_Rvel"].to_numpy(float), t_s, opt, seed=1000 + s)
        )
        ll, rr = mod.run_pfimu_both(raw, opt, seed=s)
        l_dep_mc.append(ll)
        r_dep_mc.append(rr)
    l_ref_m = np.mean(np.stack(l_ref_mc, axis=0), axis=0)
    r_ref_m = np.mean(np.stack(r_ref_mc, axis=0), axis=0)
    l_dep_m = np.mean(np.stack(l_dep_mc, axis=0), axis=0)
    r_dep_m = np.mean(np.stack(r_dep_mc, axis=0), axis=0)

    traces = pd.DataFrame(
        {
            "t_s": t_s,
            "mat_tau_L": mat["tau_exo_L"].to_numpy(float),
            "mat_tau_R": mat["tau_exo_R"].to_numpy(float),
            "local_ref_single_L": l_ref_s,
            "local_ref_single_R": r_ref_s,
            "local_deploy_single_L": l_dep_s,
            "local_deploy_single_R": r_dep_s,
            "local_ref_mcmean_L": l_ref_m,
            "local_ref_mcmean_R": r_ref_m,
            "local_deploy_mcmean_L": l_dep_m,
            "local_deploy_mcmean_R": r_dep_m,
            "local_ref_matcfg_L": l_ref_matcfg,
            "local_ref_matcfg_R": r_ref_matcfg,
            "local_deploy_matcfg_L": l_dep_matcfg,
            "local_deploy_matcfg_R": r_dep_matcfg,
        }
    )
    traces_path = out_dir / "pf_imu_zhemin_vs_local_traces_50_150s.csv"
    traces.to_csv(traces_path, index=False)

    rows: list[dict] = []
    add_pair_metrics(rows, "L", "matlab vs local_ref_single", traces["mat_tau_L"], traces["local_ref_single_L"], mod.best_lag_corr_rmse)
    add_pair_metrics(rows, "L", "matlab vs local_deploy_single", traces["mat_tau_L"], traces["local_deploy_single_L"], mod.best_lag_corr_rmse)
    add_pair_metrics(rows, "L", "matlab vs local_ref_mcmean", traces["mat_tau_L"], traces["local_ref_mcmean_L"], mod.best_lag_corr_rmse)
    add_pair_metrics(rows, "L", "matlab vs local_deploy_mcmean", traces["mat_tau_L"], traces["local_deploy_mcmean_L"], mod.best_lag_corr_rmse)
    add_pair_metrics(rows, "L", "matlab vs local_ref_matcfg", traces["mat_tau_L"], traces["local_ref_matcfg_L"], mod.best_lag_corr_rmse)
    add_pair_metrics(rows, "L", "matlab vs local_deploy_matcfg", traces["mat_tau_L"], traces["local_deploy_matcfg_L"], mod.best_lag_corr_rmse)
    add_pair_metrics(rows, "L", "local_ref_mcmean vs local_deploy_mcmean", traces["local_ref_mcmean_L"], traces["local_deploy_mcmean_L"], mod.best_lag_corr_rmse)

    add_pair_metrics(rows, "R", "matlab vs local_ref_single", traces["mat_tau_R"], traces["local_ref_single_R"], mod.best_lag_corr_rmse)
    add_pair_metrics(rows, "R", "matlab vs local_deploy_single", traces["mat_tau_R"], traces["local_deploy_single_R"], mod.best_lag_corr_rmse)
    add_pair_metrics(rows, "R", "matlab vs local_ref_mcmean", traces["mat_tau_R"], traces["local_ref_mcmean_R"], mod.best_lag_corr_rmse)
    add_pair_metrics(rows, "R", "matlab vs local_deploy_mcmean", traces["mat_tau_R"], traces["local_deploy_mcmean_R"], mod.best_lag_corr_rmse)
    add_pair_metrics(rows, "R", "matlab vs local_ref_matcfg", traces["mat_tau_R"], traces["local_ref_matcfg_R"], mod.best_lag_corr_rmse)
    add_pair_metrics(rows, "R", "matlab vs local_deploy_matcfg", traces["mat_tau_R"], traces["local_deploy_matcfg_R"], mod.best_lag_corr_rmse)
    add_pair_metrics(rows, "R", "local_ref_mcmean vs local_deploy_mcmean", traces["local_ref_mcmean_R"], traces["local_deploy_mcmean_R"], mod.best_lag_corr_rmse)

    metrics_df = pd.DataFrame(rows)
    metrics_path = out_dir / "pf_imu_zhemin_vs_local_metrics_50_150s.csv"
    metrics_df.to_csv(metrics_path, index=False)

    # Plot overlays for easier visual inspection.
    for side in ["L", "R"]:
        fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
        t = traces["t_s"].to_numpy(float)
        mat_tau = traces[f"mat_tau_{side}"].to_numpy(float)
        dep = traces[f"local_deploy_mcmean_{side}"].to_numpy(float)
        ref = traces[f"local_ref_mcmean_{side}"].to_numpy(float)
        dep_s = traces[f"local_deploy_single_{side}"].to_numpy(float)

        axes[0].plot(t, mat_tau, lw=1.8, label="MATLAB tau_exo")
        axes[0].plot(t, dep, lw=1.2, label="Local deploy (MC mean)")
        axes[0].plot(t, ref, lw=1.0, label="Local sim/ref (MC mean)")
        axes[0].set_ylabel("Torque (Nm)")
        axes[0].set_title(f"PF-IMU torque overlay ({side} leg, 50-150s)")
        axes[0].legend(loc="upper right")
        axes[0].grid(alpha=0.3)

        axes[1].plot(t, dep - mat_tau, lw=1.0, label="deploy_mcmean - matlab")
        axes[1].plot(t, ref - mat_tau, lw=1.0, label="ref_mcmean - matlab")
        axes[1].plot(t, dep_s - mat_tau, lw=0.8, alpha=0.7, label="deploy_single - matlab")
        axes[1].axhline(0.0, color="k", lw=0.8)
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("Error (Nm)")
        axes[1].grid(alpha=0.3)
        axes[1].legend(loc="upper right")

        fig.tight_layout()
        fig.savefig(out_dir / f"pf_imu_zhemin_compare_{side}_overlay_error.png", dpi=160)
        plt.close(fig)

    # Zoom plot on first 10s window for phase detail.
    t = traces["t_s"].to_numpy(float)
    z0, z1 = args.t0, min(args.t0 + 10.0, args.t1)
    sel = (t >= z0) & (t <= z1)
    fig, axes = plt.subplots(1, 2, figsize=(14, 4), sharex=False, sharey=False)
    for i, side in enumerate(["L", "R"]):
        axes[i].plot(t[sel], traces[f"mat_tau_{side}"].to_numpy(float)[sel], lw=1.8, label="MATLAB")
        axes[i].plot(t[sel], traces[f"local_deploy_mcmean_{side}"].to_numpy(float)[sel], lw=1.2, label="Deploy MC")
        axes[i].plot(t[sel], traces[f"local_ref_mcmean_{side}"].to_numpy(float)[sel], lw=1.0, label="Ref MC")
        axes[i].set_title(f"{side} leg zoom ({z0:.0f}-{z1:.0f}s)")
        axes[i].set_xlabel("Time (s)")
        axes[i].set_ylabel("Torque (Nm)")
        axes[i].grid(alpha=0.3)
        axes[i].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_dir / "pf_imu_zhemin_compare_zoom_50_60s.png", dpi=170)
    plt.close(fig)

    print(f"[saved] {traces_path}")
    print(f"[saved] {metrics_path}")
    print(f"[saved] {out_dir / 'pf_imu_zhemin_compare_L_overlay_error.png'}")
    print(f"[saved] {out_dir / 'pf_imu_zhemin_compare_R_overlay_error.png'}")
    print(f"[saved] {out_dir / 'pf_imu_zhemin_compare_zoom_50_60s.png'}")


if __name__ == "__main__":
    main()
