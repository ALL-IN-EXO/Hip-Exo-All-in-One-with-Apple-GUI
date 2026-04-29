#!/usr/bin/env python3
"""
PF-IMU consistency evaluation:
compare MATLAB-reference logic vs deployed RPi PF-IMU implementation.

Usage example:
  python tools/pf_imu_consistency_eval.py \
    --csv "Algorithm Reference/NJIT Reference/PI5_lstm_pd-20260424-221451.csv" \
    --t0 50 --t1 150 --seeds 10
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "RPi_Unified"))
from networks.pf_imu import PFIMUController  # noqa: E402


@dataclass
class Opt:
    # preprocessing parity with v17 raw-online runtime
    smooth_angle_window: int = 1
    smooth_velocity_window: int = 1
    use_measured_velocity: bool = True
    use_auto_qstar_prior: bool = False
    prior_margin_ratio: float = 0.10

    # prior (fixed, same as deployed defaults)
    q_low_range_deg: tuple[float, float] = (-65.0, -20.0)
    q_high_range_deg: tuple[float, float] = (-20.0, 15.0)

    # PF core
    num_particles: int = 900
    amin: float = -35.0
    amax: float = -0.1
    sigma_a: float = 0.20
    sigma_qstar: float = 0.003
    p_switch: float = 0.003
    switch_eps_deg: float = 2.5
    switch_vel_thresh_deg: float = 10.0
    eta_v: float = 2.0
    eta_sign: float = 2.0
    use_robust_likelihood: bool = True
    vel_err_clip_deg_s: float = 120.0
    use_direction_side_penalty: bool = True
    eta_side: float = 1.5
    dq_guide_deadzone_deg_s: float = 8.0
    use_guided_injection: bool = True
    guided_frac: float = 0.03
    qstar_guide_sep_deg: float = 2.0

    # confidence + torque synthesis
    d_conf: float = 1.5
    conf_window_sec: float = 0.5
    b_max: float = 2.0
    d_max: float = 1.0
    b_gamma: float = 1.0
    d_gamma: float = 1.0
    tau_max: float = 25.0
    use_assistive_only_damping: bool = False
    dq_deadzone_deg_s: float = 5.0
    use_torque_rate_limit: bool = True
    tau_rate_max: float = 80.0


def smooth_movmean(x: np.ndarray, win: int) -> np.ndarray:
    win = max(1, int(round(win)))
    if win <= 1:
        return x.copy()
    if win % 2 == 0:
        win += 1
    return pd.Series(x).rolling(window=win, center=True, min_periods=1).mean().to_numpy()


def percentile_local(x: np.ndarray, p: float) -> float:
    xx = np.asarray(x, dtype=float)
    xx = xx[np.isfinite(xx)]
    if xx.size == 0:
        return float("nan")
    xx = np.sort(xx)
    p = float(np.clip(p, 0.0, 100.0))
    pos = 1.0 + (float(xx.size) - 1.0) * p / 100.0
    lo = int(np.floor(pos))
    hi = int(np.ceil(pos))
    if lo == hi:
        return float(xx[lo - 1])
    return float(xx[lo - 1] + (xx[hi - 1] - xx[lo - 1]) * (pos - float(lo)))


def build_qstar_prior_ranges(q_deg: np.ndarray, opt: Opt):
    if not bool(opt.use_auto_qstar_prior):
        q_low = np.radians(np.array(opt.q_low_range_deg, dtype=float))
        q_high = np.radians(np.array(opt.q_high_range_deg, dtype=float))
        return np.sort(q_low), np.sort(q_high)

    p01 = percentile_local(q_deg, 1.0)
    p45 = percentile_local(q_deg, 45.0)
    p55 = percentile_local(q_deg, 55.0)
    p99 = percentile_local(q_deg, 99.0)
    amp = max(p99 - p01, 1.0)
    margin = float(opt.prior_margin_ratio) * amp
    q_low_deg = [p01 - margin, p45]
    q_high_deg = [p55, p99 + margin]
    med = percentile_local(q_deg, 50.0)
    qmin = float(np.min(q_deg))
    qmax = float(np.max(q_deg))
    if q_low_deg[0] >= q_low_deg[1]:
        q_low_deg = [qmin - margin, med]
    if q_high_deg[0] >= q_high_deg[1]:
        q_high_deg = [med, qmax + margin]
    q_low = np.sort(np.radians(np.array(q_low_deg, dtype=float)))
    q_high = np.sort(np.radians(np.array(q_high_deg, dtype=float)))
    return q_low, q_high


def normalize_weights(w: np.ndarray) -> np.ndarray:
    w = np.asarray(w, dtype=float)
    w[~np.isfinite(w)] = 0.0
    w[w < 0.0] = 0.0
    s = float(np.sum(w))
    if (not np.isfinite(s)) or s <= 1e-12:
        return np.ones_like(w) / float(len(w))
    return w / s


def systematic_resample(weights: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    n = len(weights)
    positions = (np.arange(n, dtype=float) + float(rng.random())) / float(n)
    cdf = np.cumsum(weights, dtype=float)
    cdf[-1] = 1.0
    return np.searchsorted(cdf, positions, side="left")


def sign_nonzero(x):
    s = np.sign(x)
    if np.isscalar(s):
        return 1.0 if s == 0 else s
    s[s == 0] = 1
    return s


def run_reference_leg(
    angle_deg: np.ndarray,
    vel_deg_s: np.ndarray,
    t_s: np.ndarray,
    opt: Opt,
    seed: int,
) -> np.ndarray:
    # MATLAB v17 guided raw-online reference logic, with deployed-parity
    # confidence O(1) update in fixed 100Hz control.
    q = np.radians(smooth_movmean(angle_deg, opt.smooth_angle_window))
    if opt.use_measured_velocity:
        dq = np.radians(smooth_movmean(vel_deg_s, opt.smooth_velocity_window))
    else:
        qd = smooth_movmean(q, max(opt.smooth_angle_window, 5))
        dq = smooth_movmean(np.gradient(qd, t_s), opt.smooth_velocity_window)

    q_low, q_high = build_qstar_prior_ranges(angle_deg, opt)

    rng = np.random.default_rng(seed)
    p = opt.num_particles

    mode = np.ones(p, dtype=np.int8)
    prob_high0 = 0.75 if dq[0] >= 0.0 else 0.25
    mode[rng.random(p) > prob_high0] = -1

    a = opt.amin + (opt.amax - opt.amin) * rng.random(p)
    qstar = np.empty(p, dtype=float)
    hi = mode == 1
    lo = ~hi
    qstar[hi] = q_high[0] + (q_high[1] - q_high[0]) * rng.random(np.count_nonzero(hi))
    qstar[lo] = q_low[0] + (q_low[1] - q_low[0]) * rng.random(np.count_nonzero(lo))

    weights = np.ones(p, dtype=float) / float(p)
    dq_int_hat = np.zeros(len(q), dtype=float)
    tau = np.zeros(len(q), dtype=float)
    tau_last = 0.0
    tau_first = True

    switch_eps = np.radians(opt.switch_eps_deg)
    switch_vel = np.radians(opt.switch_vel_thresh_deg)
    vel_err_clip = np.radians(opt.vel_err_clip_deg_s)
    dq_guide_deadzone = np.radians(opt.dq_guide_deadzone_deg_s)
    qstar_guide_sep = np.radians(opt.qstar_guide_sep_deg)
    dq_deadzone = np.radians(opt.dq_deadzone_deg_s)
    dt = 0.01

    conf_len = max(5, int(round(opt.conf_window_sec * 100.0)))
    conf_hist = np.zeros(conf_len, dtype=float)
    conf_sum = 0.0
    conf_head = 0
    conf_fill = 0

    for k in range(len(q)):
        if k > 0:
            a = np.clip(a + opt.sigma_a * rng.standard_normal(p), opt.amin, opt.amax)

            hi = mode == 1
            lo = ~hi
            n_hi = int(np.count_nonzero(hi))
            n_lo = int(np.count_nonzero(lo))
            if n_hi:
                qstar[hi] = np.clip(
                    qstar[hi] + opt.sigma_qstar * rng.standard_normal(n_hi),
                    q_high[0],
                    q_high[1],
                )
            if n_lo:
                qstar[lo] = np.clip(
                    qstar[lo] + opt.sigma_qstar * rng.standard_normal(n_lo),
                    q_low[0],
                    q_low[1],
                )

            sw_random = rng.random(p) < opt.p_switch
            hi = mode == 1
            lo = ~hi
            sw_down = hi & ((q[k] >= (qstar - switch_eps)) | (dq[k] < -switch_vel))
            sw_up = lo & ((q[k] <= (qstar + switch_eps)) | (dq[k] > switch_vel))
            sw = sw_random | sw_down | sw_up
            if np.any(sw):
                mode[sw] = -mode[sw]
                new_hi = sw & (mode == 1)
                new_lo = sw & (mode == -1)
                n_new_hi = int(np.count_nonzero(new_hi))
                n_new_lo = int(np.count_nonzero(new_lo))
                if n_new_hi:
                    qstar[new_hi] = q_high[0] + (q_high[1] - q_high[0]) * rng.random(n_new_hi)
                if n_new_lo:
                    qstar[new_lo] = q_low[0] + (q_low[1] - q_low[0]) * rng.random(n_new_lo)

            if bool(opt.use_guided_injection) and abs(dq[k]) > dq_guide_deadzone:
                n_guide = int(round(opt.guided_frac * p))
                if n_guide > 0:
                    n_guide = int(np.clip(n_guide, 1, p))
                    guide_idx = rng.choice(p, size=n_guide, replace=False)
                    if dq[k] > 0:
                        mode[guide_idx] = 1
                        q_min = max(float(q_high[0]), float(q[k] + qstar_guide_sep))
                        q_max = float(q_high[1])
                        if q_min >= q_max:
                            q_min = float(q_high[0])
                            q_max = float(q_high[1])
                        qstar[guide_idx] = q_min + (q_max - q_min) * rng.random(n_guide)
                    elif dq[k] < 0:
                        mode[guide_idx] = -1
                        q_min = float(q_low[0])
                        q_max = min(float(q_low[1]), float(q[k] - qstar_guide_sep))
                        if q_min >= q_max:
                            q_min = float(q_low[0])
                            q_max = float(q_low[1])
                        qstar[guide_idx] = q_min + (q_max - q_min) * rng.random(n_guide)
                    a[guide_idx] = opt.amin + (opt.amax - opt.amin) * rng.random(n_guide)
                    a[guide_idx] = np.clip(a[guide_idx], opt.amin, opt.amax)

        dq_pred = a * (q[k] - qstar)
        wrong_mode = ((mode == 1) & (dq_pred < 0.0)) | ((mode == -1) & (dq_pred > 0.0))
        s_obs = 1 if dq[k] >= 0.0 else -1
        s_pred = np.where(dq_pred >= 0.0, 1, -1)
        wrong_sign = s_obs != s_pred

        vel_err = dq[k] - dq_pred
        if bool(opt.use_robust_likelihood):
            vel_err_used = np.clip(vel_err, -vel_err_clip, vel_err_clip)
        else:
            vel_err_used = vel_err

        if bool(opt.use_direction_side_penalty) and abs(dq[k]) > dq_guide_deadzone:
            wrong_side = sign_nonzero(dq[k]) * (qstar - q[k]) < 0
        else:
            wrong_side = np.zeros_like(qstar, dtype=bool)

        logw = (
            -opt.eta_v * np.square(vel_err_used)
            -opt.eta_sign * wrong_mode.astype(float)
            -0.5 * opt.eta_sign * wrong_sign.astype(float)
            -opt.eta_side * wrong_side.astype(float)
        )
        logw = logw - np.max(logw)
        weights *= np.exp(logw)
        weights = normalize_weights(weights)

        dqh = float(np.sum(weights * dq_pred))
        dq_int_hat[k] = dqh

        ess = 1.0 / float(np.sum(np.square(weights)))
        if ess < (0.5 * p):
            idx = systematic_resample(weights, rng)
            a = a[idx]
            qstar = qstar[idx]
            mode = mode[idx]
            weights = np.ones(p, dtype=float) / float(p)

        # deployed-parity confidence (online O(1))
        contrib = opt.d_conf - abs(dqh - float(dq[k]))
        if conf_fill < conf_len:
            conf_hist[conf_fill] = contrib
            conf_sum += contrib
            conf_fill += 1
        else:
            conf_sum -= conf_hist[conf_head]
            conf_hist[conf_head] = contrib
            conf_sum += contrib
            conf_head = (conf_head + 1) % conf_len

        conf = float(np.clip(conf_sum * dt, 0.0, 1.0))
        b = opt.b_max * (conf ** opt.b_gamma)
        d = opt.d_max * (conf ** opt.d_gamma)
        tau_ff = b * dqh
        tau_damp_raw = d * (dqh - float(dq[k]))

        if bool(opt.use_assistive_only_damping):
            is_assistive = (abs(float(dq[k])) > dq_deadzone) and ((tau_damp_raw * float(dq[k])) > 0.0)
            tau_damp = float(tau_damp_raw) if is_assistive else 0.0
        else:
            tau_damp = float(tau_damp_raw)

        tau_sat = float(np.clip(tau_ff + tau_damp, -opt.tau_max, opt.tau_max))
        if bool(opt.use_torque_rate_limit):
            if tau_first:
                tau_k = tau_sat
                tau_first = False
            else:
                max_step = float(opt.tau_rate_max) * dt
                delta = float(np.clip(tau_sat - tau_last, -max_step, max_step))
                tau_k = float(np.clip(tau_last + delta, -opt.tau_max, opt.tau_max))
        else:
            tau_k = tau_sat
        tau[k] = tau_k
        tau_last = tau_k

    return tau


def run_pfimu_both(df: pd.DataFrame, opt: Opt, seed: int) -> tuple[np.ndarray, np.ndarray]:
    q_low_L, q_high_L = build_qstar_prior_ranges(df["imu_LTx"].to_numpy(float), opt)
    q_low_R, q_high_R = build_qstar_prior_ranges(df["imu_RTx"].to_numpy(float), opt)
    q_low = (
        float(min(q_low_L[0], q_low_R[0]) * 180.0 / np.pi),
        float(min(q_low_L[1], q_low_R[1]) * 180.0 / np.pi),
    )
    q_high = (
        float(max(q_high_L[0], q_high_R[0]) * 180.0 / np.pi),
        float(max(q_high_L[1], q_high_R[1]) * 180.0 / np.pi),
    )

    pf = PFIMUController(
        sample_rate=100.0,
        num_particles=opt.num_particles,
        tau_max=opt.tau_max,
        q_low_range_deg=q_low,
        q_high_range_deg=q_high,
        Amin=opt.amin,
        Amax=opt.amax,
        sigmaA=opt.sigma_a,
        sigmaQstar=opt.sigma_qstar,
        pSwitch=opt.p_switch,
        switchEpsDeg=opt.switch_eps_deg,
        switchVelThreshDeg=opt.switch_vel_thresh_deg,
        etaV=opt.eta_v,
        etaSign=opt.eta_sign,
        use_robust_likelihood=opt.use_robust_likelihood,
        vel_err_clip_deg_s=opt.vel_err_clip_deg_s,
        use_direction_side_penalty=opt.use_direction_side_penalty,
        etaSide=opt.eta_side,
        dqGuideDeadzoneDegS=opt.dq_guide_deadzone_deg_s,
        use_guided_injection=opt.use_guided_injection,
        guidedFrac=opt.guided_frac,
        qstarGuideSepDeg=opt.qstar_guide_sep_deg,
        dConf=opt.d_conf,
        confWindowSec=opt.conf_window_sec,
        bMax=opt.b_max,
        dMax=opt.d_max,
        bGamma=opt.b_gamma,
        dGamma=opt.d_gamma,
        use_assistive_only_damping=opt.use_assistive_only_damping,
        dqDeadzoneDegS=opt.dq_deadzone_deg_s,
        use_torque_rate_limit=opt.use_torque_rate_limit,
        tauRateMax=opt.tau_rate_max,
        smooth_angle_window=opt.smooth_angle_window,
        smooth_velocity_window=opt.smooth_velocity_window,
        use_auto_qstar_prior=opt.use_auto_qstar_prior,
        prior_margin_ratio=opt.prior_margin_ratio,
        auto_prior_min_samples=200,
        auto_prior_update_interval_sec=0.50,
        auto_prior_hist_sec=120.0,
        rng_seed=seed,
    )

    l_tau = np.zeros(len(df), dtype=float)
    r_tau = np.zeros(len(df), dtype=float)
    for i, row in enumerate(df.itertuples(index=False)):
        out = pf.generate_assistance(row.imu_LTx, row.imu_RTx, row.imu_Lvel, row.imu_Rvel)
        l_tau[i] = float(out[0])
        r_tau[i] = float(out[1])
    return l_tau, r_tau


def best_lag_corr_rmse(x: np.ndarray, y: np.ndarray, max_lag: int = 80) -> dict[str, float]:
    best_corr = -9.0
    best_lag = 0
    best_x = x
    best_y = y
    for lag in range(-max_lag, max_lag + 1):
        if lag > 0:
            xx = x[lag:]
            yy = y[:-lag]
        elif lag < 0:
            xx = x[:lag]
            yy = y[-lag:]
        else:
            xx = x
            yy = y
        if len(xx) < 10:
            continue
        c = float(np.corrcoef(xx, yy)[0, 1])
        if np.isfinite(c) and c > best_corr:
            best_corr = c
            best_lag = lag
            best_x = xx
            best_y = yy

    rmse = float(np.sqrt(np.mean(np.square(best_x - best_y))))
    mae = float(np.mean(np.abs(best_x - best_y)))
    amp_ratio = float(np.std(best_y) / (np.std(best_x) + 1e-12))
    return {
        "corr": best_corr,
        "lag_frames": float(best_lag),
        "lag_ms": float(best_lag * 10),
        "rmse_nm": rmse,
        "mae_nm": mae,
        "amp_ratio": amp_ratio,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--csv",
        type=Path,
        default=ROOT / "Algorithm Reference" / "NJIT Reference" / "PI5_lstm_pd-20260424-221451.csv",
    )
    ap.add_argument("--t0", type=float, default=50.0)
    ap.add_argument("--t1", type=float, default=150.0)
    ap.add_argument("--seeds", type=int, default=10, help="Number of seeds for MC mean consistency.")
    ap.add_argument(
        "--out-prefix",
        type=Path,
        default=ROOT / "Docs" / "pf_imu_consistency_50_150s",
    )
    args = ap.parse_args()

    opt = Opt()

    df = pd.read_csv(
        args.csv,
        usecols=["Time_ms", "imu_LTx", "imu_RTx", "imu_Lvel", "imu_Rvel"],
    )
    df = df[(df["Time_ms"] >= args.t0) & (df["Time_ms"] <= args.t1)].dropna().reset_index(drop=True)

    print(f"[info] samples={len(df)}, t=[{df['Time_ms'].iloc[0]:.4f}, {df['Time_ms'].iloc[-1]:.4f}] s")
    print("[info] profile=v17_guided_raw_online, delay=0 (no runtime delay).")

    t_s = df["Time_ms"].to_numpy(dtype=float)

    # Single-seed comparison (shows stochastic path-level gap).
    l_ref_s = run_reference_leg(df["imu_LTx"].to_numpy(float), df["imu_Lvel"].to_numpy(float), t_s, opt, seed=1)
    r_ref_s = run_reference_leg(df["imu_RTx"].to_numpy(float), df["imu_Rvel"].to_numpy(float), t_s, opt, seed=1001)
    l_pf_s, r_pf_s = run_pfimu_both(df, opt, seed=1)

    m_l_single = best_lag_corr_rmse(l_ref_s, l_pf_s)
    m_r_single = best_lag_corr_rmse(r_ref_s, r_pf_s)
    print("[single-seed] L:", m_l_single)
    print("[single-seed] R:", m_r_single)

    # Monte-Carlo mean comparison (robust consistency under stochastic PF).
    l_ref_mc, r_ref_mc, l_pf_mc, r_pf_mc = [], [], [], []
    for s in range(1, args.seeds + 1):
        l_ref_mc.append(
            run_reference_leg(df["imu_LTx"].to_numpy(float), df["imu_Lvel"].to_numpy(float), t_s, opt, seed=s)
        )
        r_ref_mc.append(
            run_reference_leg(df["imu_RTx"].to_numpy(float), df["imu_Rvel"].to_numpy(float), t_s, opt, seed=1000 + s)
        )
        ll, rr = run_pfimu_both(df, opt, seed=s)
        l_pf_mc.append(ll)
        r_pf_mc.append(rr)

    l_ref_mean = np.mean(np.stack(l_ref_mc, axis=0), axis=0)
    r_ref_mean = np.mean(np.stack(r_ref_mc, axis=0), axis=0)
    l_pf_mean = np.mean(np.stack(l_pf_mc, axis=0), axis=0)
    r_pf_mean = np.mean(np.stack(r_pf_mc, axis=0), axis=0)

    m_l_mc = best_lag_corr_rmse(l_ref_mean, l_pf_mean)
    m_r_mc = best_lag_corr_rmse(r_ref_mean, r_pf_mean)
    print("[mc-mean] L:", m_l_mc)
    print("[mc-mean] R:", m_r_mc)

    out_single = pd.DataFrame(
        {
            "t_s": t_s,
            "imu_LTx": df["imu_LTx"].to_numpy(float),
            "imu_RTx": df["imu_RTx"].to_numpy(float),
            "imu_Lvel": df["imu_Lvel"].to_numpy(float),
            "imu_Rvel": df["imu_Rvel"].to_numpy(float),
            "L_tau_ref": l_ref_s,
            "R_tau_ref": r_ref_s,
            "L_tau_pfimu": l_pf_s,
            "R_tau_pfimu": r_pf_s,
        }
    )
    out_mc = pd.DataFrame(
        {
            "t_s": t_s,
            "imu_LTx": df["imu_LTx"].to_numpy(float),
            "imu_RTx": df["imu_RTx"].to_numpy(float),
            "imu_Lvel": df["imu_Lvel"].to_numpy(float),
            "imu_Rvel": df["imu_Rvel"].to_numpy(float),
            "L_tau_ref_mean": l_ref_mean,
            "R_tau_ref_mean": r_ref_mean,
            "L_tau_pfimu_mean": l_pf_mean,
            "R_tau_pfimu_mean": r_pf_mean,
        }
    )

    out_single_path = Path(f"{args.out_prefix}_single.csv")
    out_mc_path = Path(f"{args.out_prefix}_mcmean.csv")
    out_single.to_csv(out_single_path, index=False)
    out_mc.to_csv(out_mc_path, index=False)
    print(f"[saved] {out_single_path}")
    print(f"[saved] {out_mc_path}")


if __name__ == "__main__":
    main()
