#!/usr/bin/env python3
"""
MyoAssist consistency evaluation (NJIT reference logic vs RPi migrated logic).

Compares torque outputs on the same IMU input window.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import io
from pathlib import Path
import sys
import zipfile

import numpy as np
import pandas as pd
import torch
import torch.nn as nn


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "RPi_Unified"))
from networks.myoassist import MyoAssistController  # noqa: E402


SWAP_INDICES = [1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 17, 16]


class ExoActorMyoAssistRef(nn.Module):
    """Reference actor copied from HipExoCode_Apr20.py semantics."""

    def __init__(self, symmetric=True):
        super().__init__()
        self.symmetric = bool(symmetric)
        self.register_buffer("_swap_idx", torch.tensor(SWAP_INDICES, dtype=torch.long))
        self.net = nn.Sequential(
            nn.Linear(18, 128), nn.Tanh(),
            nn.Linear(128, 64), nn.Tanh(),
            nn.Linear(64, 2), nn.Tanh(),
        )

    def forward(self, obs):
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)
        if self.symmetric:
            batch = torch.cat([obs, obs.index_select(1, self._swap_idx)], dim=0)
            out = self.net(batch)
            return torch.cat([out[0:1, 0:1], out[1:2, 0:1]], dim=-1)
        return self.net(obs)

    def load_zip_policy(self, zip_path: Path):
        with zipfile.ZipFile(str(zip_path), "r") as zf:
            with zf.open("policy.pth") as fp:
                full_sd = torch.load(io.BytesIO(fp.read()), map_location=torch.device("cpu"), weights_only=False)
        prefix = "policy_network.exo_policy_net."
        exo_sd = {
            k[len(prefix):]: v
            for k, v in full_sd.items()
            if k.startswith(prefix)
        }
        if not exo_sd:
            raise RuntimeError(f"No exo policy weights found in {zip_path}")
        self.net.load_state_dict(exo_sd)
        self.eval()


@dataclass
class RefState:
    input_hist: list[np.ndarray]
    output_hist: list[np.ndarray]


class MyoAssistReferenceRunner:
    """Reference runner matching NN_mode() MyoAssist branch in HipExoCode_Apr20.py."""

    def __init__(self, model_zip: Path, max_torque_nm: float = 15.0):
        self.actor = ExoActorMyoAssistRef(symmetric=True)
        self.actor.load_zip_policy(model_zip)
        self.max_torque_nm = float(max_torque_nm)
        self.state = RefState(
            input_hist=[np.zeros(4, dtype=np.float32) for _ in range(2)],
            output_hist=[np.zeros(2, dtype=np.float32) for _ in range(3)],
        )

    def step(self, ltx_deg: float, rtx_deg: float, lvel_dps: float, rvel_dps: float) -> tuple[float, float]:
        left_angle_rad = float(ltx_deg) * (np.pi / 180.0)
        right_angle_rad = float(rtx_deg) * (np.pi / 180.0)
        left_vel_rad = float(lvel_dps) * (np.pi / 180.0)
        right_vel_rad = float(rvel_dps) * (np.pi / 180.0)

        # Exact NJIT order for MyoAssist:
        # [right_angle, left_angle, right_vel, left_vel]
        current_input = np.array([
            right_angle_rad,
            left_angle_rad,
            right_vel_rad,
            left_vel_rad,
        ], dtype=np.float32)

        parts = self.state.input_hist + [current_input] + self.state.output_hist
        obs = np.concatenate(parts).astype(np.float32, copy=False)
        obs_t = torch.from_numpy(obs).unsqueeze(0)
        with torch.no_grad():
            pred = self.actor(obs_t).cpu().numpy().reshape(-1).astype(np.float32)

        tau_r = float(pred[0]) * self.max_torque_nm
        tau_l = float(pred[1]) * self.max_torque_nm

        # history update order matches NJIT code
        self.state.input_hist.pop(0)
        self.state.input_hist.append(current_input.copy())
        self.state.output_hist.pop(0)
        self.state.output_hist.append(pred.copy())

        return tau_l, tau_r


def _best_lag_corr(a: np.ndarray, b: np.ndarray, max_lag: int = 50) -> tuple[int, float]:
    best_lag = 0
    best_corr = -2.0
    for lag in range(-max_lag, max_lag + 1):
        if lag > 0:
            aa, bb = a[lag:], b[:-lag]
        elif lag < 0:
            aa, bb = a[:lag], b[-lag:]
        else:
            aa, bb = a, b
        if len(aa) < 10:
            continue
        c = np.corrcoef(aa, bb)[0, 1]
        if np.isfinite(c) and c > best_corr:
            best_corr = float(c)
            best_lag = int(lag)
    return best_lag, best_corr


def _metrics(ref: np.ndarray, new: np.ndarray, fs_hz: float = 100.0) -> dict:
    err = new - ref
    rmse = float(np.sqrt(np.mean(np.square(err))))
    mae = float(np.mean(np.abs(err)))
    corr = float(np.corrcoef(ref, new)[0, 1]) if len(ref) > 1 else float("nan")
    lag_f, lag_corr = _best_lag_corr(ref, new, max_lag=80)
    amp_ratio = float(np.std(new) / max(1e-9, np.std(ref)))
    return {
        "corr": corr,
        "rmse_nm": rmse,
        "mae_nm": mae,
        "best_lag_frames": int(lag_f),
        "best_lag_ms": float(lag_f * (1000.0 / fs_hz)),
        "best_lag_corr": float(lag_corr),
        "amp_ratio_std": amp_ratio,
    }


def evaluate_one(csv_path: Path, model_zip: Path, t0: float, t1: float, max_torque_nm: float, out_dir: Path):
    df = pd.read_csv(csv_path)
    req = ["Time_ms", "imu_LTx", "imu_RTx", "imu_Lvel", "imu_Rvel"]
    miss = [c for c in req if c not in df.columns]
    if miss:
        raise ValueError(f"Missing required columns: {miss}")

    t = pd.to_numeric(df["Time_ms"], errors="coerce").to_numpy(dtype=np.float64)
    mask = np.isfinite(t) & (t >= float(t0)) & (t <= float(t1))
    sub = df.loc[mask, req].copy()
    if len(sub) < 100:
        raise ValueError(f"Selected window [{t0}, {t1}] too short: {len(sub)} rows")

    ref_runner = MyoAssistReferenceRunner(model_zip=model_zip, max_torque_nm=max_torque_nm)
    new_runner = MyoAssistController(model_path=str(model_zip), max_torque_nm=max_torque_nm, symmetric=True)

    ref_l = np.zeros(len(sub), dtype=np.float64)
    ref_r = np.zeros(len(sub), dtype=np.float64)
    new_l = np.zeros(len(sub), dtype=np.float64)
    new_r = np.zeros(len(sub), dtype=np.float64)

    for i, row in enumerate(sub.itertuples(index=False)):
        tt, ltx, rtx, lvel, rvel = row
        rl, rr = ref_runner.step(ltx, rtx, lvel, rvel)
        _ = new_runner.generate_assistance(ltx, rtx, lvel, rvel)
        nl = float(new_runner.hip_torque_L)
        nr = float(new_runner.hip_torque_R)

        ref_l[i] = rl
        ref_r[i] = rr
        new_l[i] = nl
        new_r[i] = nr

    m_l = _metrics(ref_l, new_l, fs_hz=100.0)
    m_r = _metrics(ref_r, new_r, fs_hz=100.0)

    out_name = f"myoassist_consistency_{model_zip.stem}_{int(t0)}_{int(t1)}s.csv"
    out_path = out_dir / out_name
    out_df = pd.DataFrame({
        "Time_s": sub["Time_ms"].to_numpy(dtype=np.float64),
        "ref_tau_L": ref_l,
        "new_tau_L": new_l,
        "ref_tau_R": ref_r,
        "new_tau_R": new_r,
        "err_L": new_l - ref_l,
        "err_R": new_r - ref_r,
    })
    out_df.to_csv(out_path, index=False)

    return {
        "model": model_zip.name,
        "rows": int(len(sub)),
        "t0": float(sub["Time_ms"].iloc[0]),
        "t1": float(sub["Time_ms"].iloc[-1]),
        "left": m_l,
        "right": m_r,
        "csv": str(out_path),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--t0", type=float, default=50.0)
    ap.add_argument("--t1", type=float, default=150.0)
    ap.add_argument("--max-torque", type=float, default=15.0)
    ap.add_argument("--out-dir", default="Docs")
    ap.add_argument("--models", nargs="*", default=["model_1966080.zip", "model_2293760.zip"])
    args = ap.parse_args()

    csv_path = Path(args.csv).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    model_dir = ROOT / "Algorithm Reference" / "NJIT Reference"
    summaries = []
    for m in args.models:
        model_path = Path(m)
        if not model_path.is_absolute():
            model_path = model_dir / m
        model_path = model_path.resolve()
        if not model_path.exists():
            raise FileNotFoundError(f"Model not found: {model_path}")

        s = evaluate_one(
            csv_path=csv_path,
            model_zip=model_path,
            t0=args.t0,
            t1=args.t1,
            max_torque_nm=args.max_torque,
            out_dir=out_dir,
        )
        summaries.append(s)

    print("=== MyoAssist Consistency Summary ===")
    for s in summaries:
        ml = s["left"]
        mr = s["right"]
        print(
            f"{s['model']}: rows={s['rows']} t=[{s['t0']:.3f},{s['t1']:.3f}]s\n"
            f"  Left : corr={ml['corr']:.6f} rmse={ml['rmse_nm']:.6f}Nm "
            f"lag={ml['best_lag_ms']:.1f}ms amp={ml['amp_ratio_std']:.6f}\n"
            f"  Right: corr={mr['corr']:.6f} rmse={mr['rmse_nm']:.6f}Nm "
            f"lag={mr['best_lag_ms']:.1f}ms amp={mr['amp_ratio_std']:.6f}\n"
            f"  csv={s['csv']}"
        )


if __name__ == "__main__":
    main()
