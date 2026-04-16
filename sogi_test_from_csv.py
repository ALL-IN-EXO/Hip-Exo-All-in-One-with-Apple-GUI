#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线验证脚本：从 Data/6subjects/S1_Bowen_walkrun.csv 读取髋角速度，
用 SOGI (Second-Order Generalized Integrator) 实时估计瞬时相位，
输出一路与速度同相的单位正弦 sin(phi)，用来驱动外骨骼力矩。

评价指标：
  1. 符号一致率 sign_match = mean( sign(sin_phi) == sign(omega) )
  2. 归一化相位对齐 corr = <omega, sin_phi> / (||omega|| * ||sin_phi||)
     （这是 cos(平均相位差)，越接近 1 越同相）
  3. 平均正功代理 <max(sin_phi, 0) * omega> / <|omega|>

用法：
  python sogi_test_from_csv.py
  python sogi_test_from_csv.py --f0 0.9 --phi_lead_deg 0 --start 10 --dur 15
"""

import argparse
import math
import os

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# SOGI-FLL (Rodriguez et al. 2006)：
#   带通 + 正交发生器 + 频率锁相
#     ε      = ω - x1
#     dx1/dt = k·wn·ε - wn·x2        (稳态 x1 ≈  A·sin(wn·t)  与 ω 同相)
#     dx2/dt = wn·x1                  (稳态 x2 ≈ -A·cos(wn·t)  90° 滞后)
#     dwn/dt = -γ · ε · x2 / (x1² + x2² + ε0)   (频率锁相)
# 瞬时相位 φ = atan2(x1, -x2)  →  sin(φ) 与 ω 同相
# 瞬时幅值   = hypot(x1, x2)
# ============================================================
class SOGI_FLL:
    def __init__(self, f0_hz: float, fs: float, k: float = 1.41,
                 gamma: float = 40.0,
                 f_min_hz: float = 0.3, f_max_hz: float = 3.5):
        self.wn = 2.0 * math.pi * f0_hz
        self.k = k
        self.gamma = gamma
        self.dt = 1.0 / fs
        self.w_min = 2.0 * math.pi * f_min_hz
        self.w_max = 2.0 * math.pi * f_max_hz
        self.x1 = 0.0
        self.x2 = 0.0

    def step(self, omega: float):
        err = omega - self.x1
        dx1 = self.k * self.wn * err - self.wn * self.x2
        dx2 = self.wn * self.x1
        self.x1 += dx1 * self.dt
        self.x2 += dx2 * self.dt

        # FLL: 把频率朝减小 (ε * x2) 的方向推
        denom = self.x1 * self.x1 + self.x2 * self.x2 + 1e-6
        dwn = -self.gamma * err * self.x2 / denom
        self.wn += dwn * self.dt
        # 频率范围夹紧，防止发散
        if self.wn < self.w_min:
            self.wn = self.w_min
        elif self.wn > self.w_max:
            self.wn = self.w_max

        phi = math.atan2(self.x1, -self.x2)   # 使 sin(phi) 与 ω 同相
        amp = math.hypot(self.x1, self.x2)
        return math.sin(phi), phi, amp, self.wn


def run_sogi(omega: np.ndarray, fs: float, f0_hz: float,
             k: float = 1.41, gamma: float = 40.0):
    """对整段 omega 跑 SOGI-FLL，返回 (sin_phi, phi, amp, f_est_hz)。"""
    sogi = SOGI_FLL(f0_hz=f0_hz, fs=fs, k=k, gamma=gamma)
    n = len(omega)
    sin_phi = np.zeros(n)
    phi = np.zeros(n)
    amp = np.zeros(n)
    f_est = np.zeros(n)
    for i, w in enumerate(omega):
        s, p, a, wn = sogi.step(float(w))
        sin_phi[i] = s
        phi[i] = p
        amp[i] = a
        f_est[i] = wn / (2.0 * math.pi)
    return sin_phi, phi, amp, f_est


def evaluate(omega: np.ndarray, sin_phi: np.ndarray, amp: np.ndarray,
             amp_min: float, warmup_sec: float, fs: float):
    """跳过启动瞬态和静止段，算对齐指标。"""
    warmup_n = int(warmup_sec * fs)
    mask = np.zeros_like(omega, dtype=bool)
    mask[warmup_n:] = True
    mask &= (amp > amp_min)

    if mask.sum() < 10:
        return dict(sign_match=np.nan, corr=np.nan, pos_work_ratio=np.nan,
                    valid_frac=0.0)

    w = omega[mask]
    s = sin_phi[mask]

    sign_match = float(np.mean(np.sign(w) == np.sign(s)))

    denom = (np.linalg.norm(w) * np.linalg.norm(s)) + 1e-12
    corr = float(np.dot(w, s) / denom)

    # 正功比例：sin_phi 只取正时的贡献 / 全体 |omega| 平均
    pos_work = float(np.mean(np.maximum(s, 0.0) * w) / (np.mean(np.abs(w)) + 1e-12))

    return dict(sign_match=sign_match,
                corr=corr,
                pos_work_ratio=pos_work,
                valid_frac=float(mask.mean()))


def plot_side(ax_raw, ax_sin, ax_amp, t, omega, sin_phi, amp, side_label, amp_min):
    """一个髋侧画三张子图：原始 ω、sin(φ)叠加归一化 ω、幅值 + 门限"""
    # --- ω ---
    ax_raw.plot(t, omega, color='tab:blue', lw=0.9)
    ax_raw.axhline(0, color='k', lw=0.5, alpha=0.3)
    ax_raw.set_ylabel(f'{side_label} ω (deg/s)')
    ax_raw.grid(alpha=0.3)

    # --- sin(φ) 与归一化 ω 叠加 ---
    omega_norm = omega / (np.max(np.abs(omega)) + 1e-9)
    ax_sin.plot(t, omega_norm, color='tab:blue', lw=0.9, label='ω (norm)', alpha=0.7)
    ax_sin.plot(t, sin_phi, color='tab:red', lw=1.1, label='sin(φ)')
    ax_sin.axhline(0, color='k', lw=0.5, alpha=0.3)
    ax_sin.set_ylabel(f'{side_label} 同相正弦')
    ax_sin.set_ylim(-1.2, 1.2)
    ax_sin.legend(loc='upper right', fontsize=8)
    ax_sin.grid(alpha=0.3)

    # --- 幅值与门限 ---
    ax_amp.plot(t, amp, color='tab:green', lw=0.9)
    ax_amp.axhline(amp_min, color='r', lw=0.8, ls='--', label=f'amp_min={amp_min:g}')
    ax_amp.set_ylabel(f'{side_label} amp')
    ax_amp.set_xlabel('t (s)')
    ax_amp.legend(loc='upper right', fontsize=8)
    ax_amp.grid(alpha=0.3)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--input', default='Data/6subjects/S1_Bowen_walkrun.csv')
    p.add_argument('--f0', type=float, default=1.0,
                   help='SOGI 初始中心频率 Hz，FLL 会自适应')
    p.add_argument('--k', type=float, default=1.41,
                   help='SOGI 阻尼，1.41 ≈ 临界阻尼附近')
    p.add_argument('--gamma', type=float, default=40.0,
                   help='FLL 学习率，越大锁频越快但越抖')
    p.add_argument('--phi_lead_deg', type=float, default=0.0,
                   help='相位超前补偿（度），正值让输出提前')
    p.add_argument('--amp_min', type=float, default=20.0,
                   help='幅值看门狗阈值（与 ω 同单位 deg/s）')
    p.add_argument('--warmup', type=float, default=2.0,
                   help='评估时跳过的前 N 秒')
    p.add_argument('--start', type=float, default=0.0,
                   help='绘图起始时间 s')
    p.add_argument('--dur', type=float, default=15.0,
                   help='绘图时长 s')
    p.add_argument('--outdir', default='output')
    return p.parse_args()


def main():
    args = parse_args()

    if not os.path.exists(args.input):
        raise FileNotFoundError(args.input)

    df = pd.read_csv(args.input)
    print(f'[INFO] 读取 {len(df)} 行，列：{list(df.columns)[:8]}...')

    # Time_ms 列实际单位是秒（从 0.010 开始，差分 ≈ 0.010）
    t_raw = df['Time_ms'].to_numpy(dtype=float)
    dt = float(np.median(np.diff(t_raw)))
    fs = 1.0 / dt
    print(f'[INFO] 检测采样：dt={dt*1000:.2f} ms, fs={fs:.1f} Hz')

    Lvel = df['imu_Lvel'].to_numpy(dtype=float)
    Rvel = df['imu_Rvel'].to_numpy(dtype=float)

    # 跑 SOGI-FLL
    sinL, phiL, ampL, fL = run_sogi(Lvel, fs, args.f0, args.k, args.gamma)
    sinR, phiR, ampR, fR = run_sogi(Rvel, fs, args.f0, args.k, args.gamma)

    # 相位超前补偿（只影响输出波形，不影响原始 phi/amp）
    lead = math.radians(args.phi_lead_deg)
    if lead != 0.0:
        sinL = np.sin(phiL + lead)
        sinR = np.sin(phiR + lead)

    # --- 定量评估 ---
    evalL = evaluate(Lvel, sinL, ampL, args.amp_min, args.warmup, fs)
    evalR = evaluate(Rvel, sinR, ampR, args.amp_min, args.warmup, fs)

    print('\n==== 整体对齐评估（跳过前 {:.1f}s 和 amp<{:g} 的段） ===='.format(
        args.warmup, args.amp_min))
    for name, e in [('Left', evalL), ('Right', evalR)]:
        print(f'[{name:5s}] sign_match={e["sign_match"]:.3f}   '
              f'corr(cosΔφ)={e["corr"]:+.3f}   '
              f'pos_work_ratio={e["pos_work_ratio"]:+.3f}   '
              f'valid_frac={e["valid_frac"]:.2f}')

    # --- 按 tag 分段评估 ---
    if 'tag' in df.columns:
        tags = df['tag'].fillna('').astype(str).to_numpy()
        unique_tags = [t for t in pd.unique(tags) if t not in ('', 'stop')]
        print('\n==== 按 tag 分段（仅 Left，已锁定区间） ====')
        print(f'{"tag":<12} {"f_est(Hz)":>10} {"sign":>7} {"corr":>7} {"posW":>7}')
        for tg in unique_tags:
            m = (tags == tg)
            if m.sum() < 50:
                continue
            w = Lvel[m]; s = sinL[m]; a = ampL[m]; fe = fL[m]
            valid = a > args.amp_min
            if valid.sum() < 20:
                continue
            w = w[valid]; s = s[valid]; fe = fe[valid]
            sign_m = float(np.mean(np.sign(w) == np.sign(s)))
            corr = float(np.dot(w, s) / (np.linalg.norm(w)*np.linalg.norm(s)+1e-12))
            pw = float(np.mean(np.maximum(s, 0)*w) / (np.mean(np.abs(w))+1e-12))
            print(f'{tg:<12} {np.median(fe):>10.3f} '
                  f'{sign_m:>7.3f} {corr:>+7.3f} {pw:>+7.3f}')

    # --- 绘图（只画一段，避免太挤） ---
    i0 = int(args.start * fs)
    i1 = min(len(t_raw), int((args.start + args.dur) * fs))
    sl = slice(i0, i1)
    t = t_raw[sl] - t_raw[i0]

    fig, axes = plt.subplots(3, 2, figsize=(13, 8), sharex=True)
    plot_side(axes[0, 0], axes[1, 0], axes[2, 0],
              t, Lvel[sl], sinL[sl], ampL[sl], 'L', args.amp_min)
    plot_side(axes[0, 1], axes[1, 1], axes[2, 1],
              t, Rvel[sl], sinR[sl], ampR[sl], 'R', args.amp_min)
    fig.suptitle(
        f'SOGI phase tracking  |  file={os.path.basename(args.input)}  '
        f'f0={args.f0} Hz  k={args.k}  lead={args.phi_lead_deg}°',
        fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    os.makedirs(args.outdir, exist_ok=True)
    fig_path = os.path.join(args.outdir, 'sogi_S1_phase_tracking.png')
    fig.savefig(fig_path, dpi=130)
    print(f'\n[INFO] 相位跟踪图已保存: {fig_path}')

    # --- Lissajous: ω vs sin(φ) 散点，看相位对齐 ---
    fig2, ax2 = plt.subplots(1, 2, figsize=(10, 5))
    warm = int(args.warmup * fs)
    for ax, w, s, amp, lbl in [
        (ax2[0], Lvel, sinL, ampL, 'Left'),
        (ax2[1], Rvel, sinR, ampR, 'Right'),
    ]:
        m = (amp > args.amp_min)
        m[:warm] = False
        w_n = w[m] / (np.max(np.abs(w[m])) + 1e-9)
        ax.scatter(w_n, s[m], s=2, alpha=0.3)
        ax.plot([-1, 1], [-1, 1], 'r--', lw=0.8, label='perfect sync')
        ax.set_xlabel('ω (norm)')
        ax.set_ylabel('sin(φ)')
        ax.set_title(lbl)
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig2.suptitle('ω vs sin(φ)  —  同相时点应落在 y=x 附近')
    fig2.tight_layout(rect=(0, 0, 1, 0.94))
    fig2_path = os.path.join(args.outdir, 'sogi_S1_lissajous.png')
    fig2.savefig(fig2_path, dpi=130)
    print(f'[INFO] 相位散点图已保存: {fig2_path}')

    # --- 全程 FLL 估计频率 + 幅值 overview，验证 FLL 是否在每段都锁上 ---
    fig3, ax3 = plt.subplots(2, 1, figsize=(13, 5), sharex=True)
    ax3[0].plot(t_raw, fL, color='tab:blue', lw=0.8, label='f_est L')
    ax3[0].plot(t_raw, fR, color='tab:orange', lw=0.8, label='f_est R', alpha=0.7)
    ax3[0].set_ylabel('f_est (Hz)')
    ax3[0].set_ylim(0.2, 3.5)
    ax3[0].grid(alpha=0.3)
    ax3[0].legend(fontsize=8, loc='upper right')

    ax3[1].plot(t_raw, ampL, color='tab:blue', lw=0.8, label='amp L')
    ax3[1].plot(t_raw, ampR, color='tab:orange', lw=0.8, label='amp R', alpha=0.7)
    ax3[1].axhline(args.amp_min, color='r', lw=0.6, ls='--',
                   label=f'amp_min={args.amp_min:g}')
    ax3[1].set_ylabel('amp')
    ax3[1].set_xlabel('t (s)')
    ax3[1].grid(alpha=0.3)
    ax3[1].legend(fontsize=8, loc='upper right')

    # 叠加 tag 分段阴影 + 文字
    if 'tag' in df.columns:
        tags = df['tag'].fillna('').astype(str).to_numpy()
        prev = tags[0]; start = t_raw[0]
        for i in range(1, len(tags)):
            if tags[i] != prev:
                if prev and prev != 'stop':
                    ax3[0].axvspan(start, t_raw[i], alpha=0.06, color='green')
                    ax3[0].text((start + t_raw[i]) / 2, 3.2, prev,
                                ha='center', fontsize=7, alpha=0.7)
                prev = tags[i]; start = t_raw[i]

    fig3.suptitle('SOGI-FLL  频率自适应 & 幅值全程视图')
    fig3.tight_layout(rect=(0, 0, 1, 0.95))
    fig3_path = os.path.join(args.outdir, 'sogi_S1_fll_overview.png')
    fig3.savefig(fig3_path, dpi=130)
    print(f'[INFO] FLL overview 已保存: {fig3_path}')


if __name__ == '__main__':
    main()
