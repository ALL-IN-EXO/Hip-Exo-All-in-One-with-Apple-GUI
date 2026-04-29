"""
PF-IMU controller (runtime version for RPi Unified).

v17 guided raw-online profile (MATLAB parity oriented):
  - RAW IMU angle/velocity inputs
  - robust clipped velocity likelihood
  - direction-side penalty
  - direction-guided particle injection
  - confidence-gated torque synthesis
  - optional torque rate limit

The external interface is kept compatible with existing NN controllers.
"""

from collections import deque
import math
import time
import numpy as np


class IdentityFilter:
    def filter(self, x):
        return x


class PFIMUController:
    def __init__(
        self,
        sample_rate=100.0,
        num_particles=900,
        tau_max=25.0,
        q_low_range_deg=(-65.0, -20.0),
        q_high_range_deg=(-20.0, 15.0),
        Amin=-35.0,
        Amax=-0.1,
        sigmaA=0.20,
        sigmaQstar=0.003,
        pSwitch=0.003,
        switchEpsDeg=2.5,
        switchVelThreshDeg=10.0,
        etaV=2.0,
        etaSign=2.0,
        use_robust_likelihood=True,
        vel_err_clip_deg_s=120.0,
        use_direction_side_penalty=True,
        etaSide=1.5,
        dqGuideDeadzoneDegS=8.0,
        use_guided_injection=True,
        guidedFrac=0.03,
        qstarGuideSepDeg=2.0,
        dConf=1.5,
        confWindowSec=0.50,
        bMax=2.0,
        dMax=1.0,
        bGamma=1.0,
        dGamma=1.0,
        use_assistive_only_damping=False,
        dqDeadzoneDegS=5.0,
        use_torque_rate_limit=True,
        tauRateMax=80.0,
        smooth_angle_window=1,
        smooth_velocity_window=1,
        use_auto_qstar_prior=False,
        prior_margin_ratio=0.10,
        auto_prior_min_samples=200,
        auto_prior_update_interval_sec=1.0,
        auto_prior_hist_sec=120.0,
        compute_warn_ms=3.0,
        rng_seed=1,
    ):
        self.fs = float(sample_rate)
        self.dt = 1.0 / max(1e-6, self.fs)
        self.P = int(max(20, num_particles))

        self.tau_max = float(max(1.0, tau_max))
        self.Amin = float(Amin)
        self.Amax = float(Amax)
        self.sigmaA = float(max(0.0, sigmaA))
        self.sigmaQstar = float(max(0.0, sigmaQstar))
        self.pSwitch = float(np.clip(pSwitch, 0.0, 1.0))
        self.switch_eps = math.radians(float(max(0.0, switchEpsDeg)))
        self.switch_vel_thresh = math.radians(float(max(0.0, switchVelThreshDeg)))

        self.etaV = float(max(0.0, etaV))
        self.etaSign = float(max(0.0, etaSign))
        self.use_robust_likelihood = bool(use_robust_likelihood)
        self.vel_err_clip = math.radians(float(max(1e-6, vel_err_clip_deg_s)))

        self.use_direction_side_penalty = bool(use_direction_side_penalty)
        self.etaSide = float(max(0.0, etaSide))
        self.dq_guide_deadzone = math.radians(float(max(0.0, dqGuideDeadzoneDegS)))

        self.use_guided_injection = bool(use_guided_injection)
        self.guided_frac = float(np.clip(guidedFrac, 0.0, 0.5))
        self.qstar_guide_sep = math.radians(float(max(0.0, qstarGuideSepDeg)))

        self.dConf = float(max(0.0, dConf))
        self.conf_window_frames = int(max(5, round(float(confWindowSec) * self.fs)))
        self.bMax = float(max(0.0, bMax))
        self.dMax = float(max(0.0, dMax))
        self.bGamma = float(max(1e-6, bGamma))
        self.dGamma = float(max(1e-6, dGamma))

        self.use_assistive_only_damping = bool(use_assistive_only_damping)
        self.dq_deadzone = math.radians(float(max(0.0, dqDeadzoneDegS)))

        self.use_torque_rate_limit = bool(use_torque_rate_limit)
        self.tau_rate_max = float(max(1e-6, tauRateMax))

        self.smooth_angle_window = int(max(1, round(float(smooth_angle_window))))
        if self.smooth_angle_window % 2 == 0:
            self.smooth_angle_window += 1
        self.smooth_velocity_window = int(max(1, round(float(smooth_velocity_window))))
        if self.smooth_velocity_window % 2 == 0:
            self.smooth_velocity_window += 1

        self.use_auto_qstar_prior = bool(use_auto_qstar_prior)
        self.prior_margin_ratio = float(max(0.0, prior_margin_ratio))
        self.auto_prior_min_samples = int(max(20, round(float(auto_prior_min_samples))))
        self.auto_prior_update_interval_frames = int(max(1, round(float(auto_prior_update_interval_sec) * self.fs)))
        self.auto_prior_hist_frames = int(max(self.auto_prior_min_samples, round(float(auto_prior_hist_sec) * self.fs)))
        self._frame_idx = 0
        self._last_prior_update_frame = -10**9

        self.q_low_range = np.array(
            [math.radians(float(q_low_range_deg[0])), math.radians(float(q_low_range_deg[1]))],
            dtype=np.float32,
        )
        self.q_high_range = np.array(
            [math.radians(float(q_high_range_deg[0])), math.radians(float(q_high_range_deg[1]))],
            dtype=np.float32,
        )
        if self.q_low_range[0] > self.q_low_range[1]:
            self.q_low_range = self.q_low_range[::-1]
        if self.q_high_range[0] > self.q_high_range[1]:
            self.q_high_range = self.q_high_range[::-1]

        self._rng = np.random.default_rng(int(rng_seed))
        self._state_L = self._init_leg_state()
        self._state_R = self._init_leg_state()

        self._smooth_q_L = deque(maxlen=self.smooth_angle_window)
        self._smooth_q_R = deque(maxlen=self.smooth_angle_window)
        self._smooth_dq_L = deque(maxlen=self.smooth_velocity_window)
        self._smooth_dq_R = deque(maxlen=self.smooth_velocity_window)
        self._prior_hist_qdeg_L = deque(maxlen=self.auto_prior_hist_frames)
        self._prior_hist_qdeg_R = deque(maxlen=self.auto_prior_hist_frames)

        # Keep fields compatible with existing NN classes.
        self.left_exo_filter = IdentityFilter()
        self.right_exo_filter = IdentityFilter()

        # Runtime-configurable input filters from RL GUI (Vel+Ref).
        self.left_input_pos_filter = IdentityFilter()
        self.right_input_pos_filter = IdentityFilter()
        self.left_input_vel_filter = IdentityFilter()
        self.right_input_vel_filter = IdentityFilter()

        self.b = np.array([1.0], dtype=np.float32)
        self.a = np.array([1.0], dtype=np.float32)

        self.hip_torque_L = 0.0
        self.hip_torque_R = 0.0
        self.filtered_hip_torque_L = 0.0
        self.filtered_hip_torque_R = 0.0
        self.L_p = 0.0
        self.L_d = 0.0
        self.R_p = 0.0
        self.R_d = 0.0

        # Diagnostics.
        self.pf_dq_int_hat_L = 0.0
        self.pf_dq_int_hat_R = 0.0
        self.pf_conf_L = 0.0
        self.pf_conf_R = 0.0
        self.pf_ess_L = float(self.P)
        self.pf_ess_R = float(self.P)
        self.pf_mode_prob_high_L = 0.5
        self.pf_mode_prob_high_R = 0.5
        self.pf_compute_ms = 0.0
        self.pf_compute_p95_ms = 0.0
        self.pf_overrun_count = 0
        self.pf_exception_count = 0

        # Keep legacy fields for logging compatibility.
        self.pf_prior_ready_L = int(not self.use_auto_qstar_prior)
        self.pf_prior_ready_R = int(not self.use_auto_qstar_prior)
        self.pf_prior_updates = 0
        self.pf_prior_q_low_L_deg = float(np.degrees(self.q_low_range[0]))
        self.pf_prior_q_high_L_deg = float(np.degrees(self.q_high_range[1]))
        self.pf_prior_q_low_R_deg = float(np.degrees(self.q_low_range[0]))
        self.pf_prior_q_high_R_deg = float(np.degrees(self.q_high_range[1]))

        self._compute_warn_ms = float(max(0.1, compute_warn_ms))
        self._compute_hist = deque(maxlen=400)

    def _init_leg_state(self):
        return {
            "initialized": False,
            "step": 0,
            "A": np.zeros(self.P, dtype=np.float32),
            "qstar": np.zeros(self.P, dtype=np.float32),
            "mode": np.ones(self.P, dtype=np.int8),
            "w": np.ones(self.P, dtype=np.float32) / float(self.P),
            "conf_hist": deque(maxlen=self.conf_window_frames),
            "conf_sum": 0.0,
            "last_tau": 0.0,
            "first_tau": True,
            "q_low_range": self.q_low_range.copy(),
            "q_high_range": self.q_high_range.copy(),
        }

    @staticmethod
    def _clip(x, lo, hi):
        return np.minimum(np.maximum(x, lo), hi)

    @staticmethod
    def _sign_nonzero(x):
        s = np.sign(x)
        if np.isscalar(s):
            return 1.0 if s == 0 else s
        s[s == 0] = 1
        return s

    @staticmethod
    def _percentile_local(x: np.ndarray, p: float) -> float:
        xx = np.asarray(x, dtype=float)
        xx = xx[np.isfinite(xx)]
        if xx.size == 0:
            return float("nan")
        xx = np.sort(xx)
        pp = float(np.clip(p, 0.0, 100.0))
        pos = 1.0 + (float(xx.size) - 1.0) * pp / 100.0
        lo = int(math.floor(pos))
        hi = int(math.ceil(pos))
        if lo == hi:
            return float(xx[lo - 1])
        vlo = float(xx[lo - 1])
        vhi = float(xx[hi - 1])
        return vlo + (vhi - vlo) * (pos - float(lo))

    @staticmethod
    def _causal_movmean_push(buf: deque, x: float) -> float:
        buf.append(float(x))
        return float(np.mean(np.asarray(buf, dtype=np.float32), dtype=np.float32))

    def _build_auto_prior_ranges(self, q_hist_deg: deque):
        if len(q_hist_deg) < self.auto_prior_min_samples:
            return None
        qd = np.asarray(q_hist_deg, dtype=np.float32)
        p01 = self._percentile_local(qd, 1.0)
        p45 = self._percentile_local(qd, 45.0)
        p55 = self._percentile_local(qd, 55.0)
        p99 = self._percentile_local(qd, 99.0)
        amp = max(p99 - p01, 1.0)
        margin = self.prior_margin_ratio * amp
        q_low_deg = [p01 - margin, p45]
        q_high_deg = [p55, p99 + margin]
        med = self._percentile_local(qd, 50.0)
        qmin = float(np.min(qd))
        qmax = float(np.max(qd))
        if q_low_deg[0] >= q_low_deg[1]:
            q_low_deg = [qmin - margin, med]
        if q_high_deg[0] >= q_high_deg[1]:
            q_high_deg = [med, qmax + margin]
        q_low = np.sort(np.radians(np.asarray(q_low_deg, dtype=np.float32)))
        q_high = np.sort(np.radians(np.asarray(q_high_deg, dtype=np.float32)))
        return q_low.astype(np.float32), q_high.astype(np.float32)

    def _apply_leg_prior(self, st, q_low, q_high):
        st["q_low_range"] = np.asarray(q_low, dtype=np.float32).copy()
        st["q_high_range"] = np.asarray(q_high, dtype=np.float32).copy()
        st["qstar"] = self._clip(
            st["qstar"],
            np.where(st["mode"] == 1, st["q_high_range"][0], st["q_low_range"][0]),
            np.where(st["mode"] == 1, st["q_high_range"][1], st["q_low_range"][1]),
        )

    def _maybe_update_auto_prior(self):
        if not self.use_auto_qstar_prior:
            return
        if (self._frame_idx - self._last_prior_update_frame) < self.auto_prior_update_interval_frames:
            return
        self._last_prior_update_frame = self._frame_idx

        prior_L = self._build_auto_prior_ranges(self._prior_hist_qdeg_L)
        if prior_L is not None:
            q_low_L, q_high_L = prior_L
            self._apply_leg_prior(self._state_L, q_low_L, q_high_L)
            self.pf_prior_ready_L = 1
            self.pf_prior_q_low_L_deg = float(np.degrees(q_low_L[0]))
            self.pf_prior_q_high_L_deg = float(np.degrees(q_high_L[1]))
            self.pf_prior_updates += 1

        prior_R = self._build_auto_prior_ranges(self._prior_hist_qdeg_R)
        if prior_R is not None:
            q_low_R, q_high_R = prior_R
            self._apply_leg_prior(self._state_R, q_low_R, q_high_R)
            self.pf_prior_ready_R = 1
            self.pf_prior_q_low_R_deg = float(np.degrees(q_low_R[0]))
            self.pf_prior_q_high_R_deg = float(np.degrees(q_high_R[1]))
            self.pf_prior_updates += 1

    def _reinit_if_needed(self, st, dq):
        if st["initialized"]:
            return
        prob_high0 = 0.75 if dq >= 0.0 else 0.25
        mode = np.ones(self.P, dtype=np.int8)
        mode[self._rng.random(self.P) > prob_high0] = -1
        st["mode"] = mode

        st["A"] = self.Amin + (self.Amax - self.Amin) * self._rng.random(self.P, dtype=np.float32)

        qstar = np.empty(self.P, dtype=np.float32)
        hi = mode == 1
        lo = ~hi
        n_hi = int(np.count_nonzero(hi))
        n_lo = self.P - n_hi
        if n_hi > 0:
            qstar[hi] = st["q_high_range"][0] + (st["q_high_range"][1] - st["q_high_range"][0]) * self._rng.random(
                n_hi, dtype=np.float32
            )
        if n_lo > 0:
            qstar[lo] = st["q_low_range"][0] + (st["q_low_range"][1] - st["q_low_range"][0]) * self._rng.random(
                n_lo, dtype=np.float32
            )

        st["qstar"] = qstar
        st["w"].fill(1.0 / float(self.P))
        st["conf_hist"].clear()
        st["conf_sum"] = 0.0
        st["last_tau"] = 0.0
        st["first_tau"] = True
        st["step"] = 0
        st["initialized"] = True

    def _systematic_resample(self, w):
        n = w.size
        positions = (np.arange(n, dtype=np.float32) + float(self._rng.random())) / float(n)
        cdf = np.cumsum(w, dtype=np.float64)
        cdf[-1] = 1.0
        return np.searchsorted(cdf, positions, side="left")

    def _guided_injection(self, st, q_now, dq_now):
        if not self.use_guided_injection:
            return
        n_guide = int(round(self.guided_frac * self.P))
        if n_guide <= 0:
            return
        n_guide = int(np.clip(n_guide, 1, self.P))
        idx = self._rng.choice(self.P, size=n_guide, replace=False)

        qstar = st["qstar"]
        mode = st["mode"]
        A = st["A"]

        if dq_now > 0.0:
            mode[idx] = 1
            q_min = max(float(st["q_high_range"][0]), float(q_now + self.qstar_guide_sep))
            q_max = float(st["q_high_range"][1])
            if q_min >= q_max:
                q_min = float(st["q_high_range"][0])
                q_max = float(st["q_high_range"][1])
            qstar[idx] = q_min + (q_max - q_min) * self._rng.random(n_guide, dtype=np.float32)
        elif dq_now < 0.0:
            mode[idx] = -1
            q_min = float(st["q_low_range"][0])
            q_max = min(float(st["q_low_range"][1]), float(q_now - self.qstar_guide_sep))
            if q_min >= q_max:
                q_min = float(st["q_low_range"][0])
                q_max = float(st["q_low_range"][1])
            qstar[idx] = q_min + (q_max - q_min) * self._rng.random(n_guide, dtype=np.float32)
        else:
            return

        A[idx] = self.Amin + (self.Amax - self.Amin) * self._rng.random(n_guide, dtype=np.float32)
        A[idx] = self._clip(A[idx], self.Amin, self.Amax)

    def _update_leg(self, st, q, dq):
        self._reinit_if_needed(st, dq)

        A = st["A"]
        qstar = st["qstar"]
        mode = st["mode"]
        w = st["w"]

        # MATLAB: process + switching start from k>1.
        if st["step"] > 0:
            A[:] = self._clip(A + self.sigmaA * self._rng.standard_normal(self.P, dtype=np.float32), self.Amin, self.Amax)

            hi = mode == 1
            lo = ~hi
            n_hi = int(np.count_nonzero(hi))
            n_lo = self.P - n_hi
            if n_hi > 0:
                qstar[hi] = self._clip(
                    qstar[hi] + self.sigmaQstar * self._rng.standard_normal(n_hi, dtype=np.float32),
                    st["q_high_range"][0],
                    st["q_high_range"][1],
                )
            if n_lo > 0:
                qstar[lo] = self._clip(
                    qstar[lo] + self.sigmaQstar * self._rng.standard_normal(n_lo, dtype=np.float32),
                    st["q_low_range"][0],
                    st["q_low_range"][1],
                )

            sw_random = self._rng.random(self.P) < self.pSwitch
            hi = mode == 1
            lo = ~hi
            sw_down = hi & ((q >= (qstar - self.switch_eps)) | (dq < -self.switch_vel_thresh))
            sw_up = lo & ((q <= (qstar + self.switch_eps)) | (dq > self.switch_vel_thresh))
            sw = sw_random | sw_down | sw_up
            if np.any(sw):
                mode[sw] = -mode[sw]
                new_hi = sw & (mode == 1)
                new_lo = sw & (mode == -1)
                n_new_hi = int(np.count_nonzero(new_hi))
                n_new_lo = int(np.count_nonzero(new_lo))
                if n_new_hi > 0:
                    qstar[new_hi] = st["q_high_range"][0] + (
                        st["q_high_range"][1] - st["q_high_range"][0]
                    ) * self._rng.random(n_new_hi, dtype=np.float32)
                if n_new_lo > 0:
                    qstar[new_lo] = st["q_low_range"][0] + (
                        st["q_low_range"][1] - st["q_low_range"][0]
                    ) * self._rng.random(n_new_lo, dtype=np.float32)

        if st["step"] > 0 and abs(dq) > self.dq_guide_deadzone:
            self._guided_injection(st, q, dq)

        dq_pred = A * (q - qstar)

        wrong_mode = ((mode == 1) & (dq_pred < 0.0)) | ((mode == -1) & (dq_pred > 0.0))
        s_obs = self._sign_nonzero(dq)
        s_pred = self._sign_nonzero(dq_pred)
        wrong_sign_obs = s_obs != s_pred

        vel_err = dq - dq_pred
        if self.use_robust_likelihood:
            vel_err_used = self._clip(vel_err, -self.vel_err_clip, self.vel_err_clip)
        else:
            vel_err_used = vel_err

        if self.use_direction_side_penalty and abs(dq) > self.dq_guide_deadzone:
            wrong_side = (self._sign_nonzero(dq) * (qstar - q)) < 0.0
        else:
            wrong_side = np.zeros_like(qstar, dtype=bool)

        logw = (
            -self.etaV * np.square(vel_err_used, dtype=np.float32)
            -self.etaSign * wrong_mode.astype(np.float32)
            -0.5 * self.etaSign * wrong_sign_obs.astype(np.float32)
            -self.etaSide * wrong_side.astype(np.float32)
        )
        logw = logw - np.max(logw)
        w *= np.exp(logw).astype(np.float32)
        w_sum = float(np.sum(w))
        if (not math.isfinite(w_sum)) or w_sum <= 1e-12:
            w.fill(1.0 / float(self.P))
        else:
            w /= w_sum

        hi = mode == 1
        p_high = float(np.sum(w[hi])) if np.any(hi) else 0.0
        dq_int_hat = float(np.sum(w * dq_pred))

        ess = 1.0 / max(1e-9, float(np.sum(np.square(w, dtype=np.float32))))
        if ess < (0.5 * self.P):
            idx = self._systematic_resample(w)
            st["A"] = A[idx]
            st["qstar"] = qstar[idx]
            st["mode"] = mode[idx]
            st["w"] = np.ones(self.P, dtype=np.float32) / float(self.P)
            A = st["A"]
            qstar = st["qstar"]
            mode = st["mode"]
            w = st["w"]

        vel_err_abs = abs(dq_int_hat - float(dq))
        contrib = self.dConf - vel_err_abs
        qh = st["conf_hist"]
        if len(qh) == qh.maxlen:
            st["conf_sum"] -= float(qh[0])
        qh.append(float(contrib))
        st["conf_sum"] += float(contrib)
        conf = float(np.clip(st["conf_sum"] * self.dt, 0.0, 1.0))

        B = self.bMax * (conf ** self.bGamma)
        D = self.dMax * (conf ** self.dGamma)
        tau_ff = B * dq_int_hat
        tau_damp_raw = D * (dq_int_hat - float(dq))

        if self.use_assistive_only_damping:
            is_assistive = (abs(float(dq)) > self.dq_deadzone) and ((tau_damp_raw * float(dq)) > 0.0)
            tau_damp = float(tau_damp_raw) if is_assistive else 0.0
        else:
            tau_damp = float(tau_damp_raw)

        tau_cmd = float(tau_ff + tau_damp)
        tau_sat = float(np.clip(tau_cmd, -self.tau_max, self.tau_max))

        if self.use_torque_rate_limit:
            if st["first_tau"]:
                tau_out = tau_sat
                st["first_tau"] = False
            else:
                max_step = self.tau_rate_max * self.dt
                delta = tau_sat - float(st["last_tau"])
                delta = float(np.clip(delta, -max_step, max_step))
                tau_out = float(st["last_tau"] + delta)
                tau_out = float(np.clip(tau_out, -self.tau_max, self.tau_max))
        else:
            tau_out = tau_sat

        st["last_tau"] = tau_out
        st["step"] += 1

        return {
            "tau": tau_out,
            "tau_ff": float(tau_ff),
            "tau_damp": float(tau_damp),
            "dq_int_hat": dq_int_hat,
            "conf": conf,
            "ess": float(ess),
            "mode_prob_high": p_high,
        }

    def generate_assistance(self, L_IMU_angle, R_IMU_angle, L_IMU_Vel, R_IMU_Vel):
        t0 = time.perf_counter()
        try:
            L_IMU_angle = float(self.left_input_pos_filter.filter(float(L_IMU_angle)))
            R_IMU_angle = float(self.right_input_pos_filter.filter(float(R_IMU_angle)))
            L_IMU_Vel = float(self.left_input_vel_filter.filter(float(L_IMU_Vel)))
            R_IMU_Vel = float(self.right_input_vel_filter.filter(float(R_IMU_Vel)))

            # v17 raw-online default is no extra smoothing (window=1),
            # but keep runtime option for controlled A/B.
            L_IMU_angle = self._causal_movmean_push(self._smooth_q_L, L_IMU_angle)
            R_IMU_angle = self._causal_movmean_push(self._smooth_q_R, R_IMU_angle)
            L_IMU_Vel = self._causal_movmean_push(self._smooth_dq_L, L_IMU_Vel)
            R_IMU_Vel = self._causal_movmean_push(self._smooth_dq_R, R_IMU_Vel)

            if self.use_auto_qstar_prior:
                self._prior_hist_qdeg_L.append(L_IMU_angle)
                self._prior_hist_qdeg_R.append(R_IMU_angle)
                self._maybe_update_auto_prior()

            qL = math.radians(L_IMU_angle)
            qR = math.radians(R_IMU_angle)
            dqL = math.radians(L_IMU_Vel)
            dqR = math.radians(R_IMU_Vel)

            stL = self._update_leg(self._state_L, qL, dqL)
            stR = self._update_leg(self._state_R, qR, dqR)

            self.hip_torque_L = float(stL["tau"])
            self.hip_torque_R = float(stR["tau"])
            self.filtered_hip_torque_L = self.hip_torque_L
            self.filtered_hip_torque_R = self.hip_torque_R

            self.L_p = float(stL["tau_ff"])
            self.L_d = float(stL["tau_damp"])
            self.R_p = float(stR["tau_ff"])
            self.R_d = float(stR["tau_damp"])

            self.pf_dq_int_hat_L = float(stL["dq_int_hat"])
            self.pf_dq_int_hat_R = float(stR["dq_int_hat"])
            self.pf_conf_L = float(stL["conf"])
            self.pf_conf_R = float(stR["conf"])
            self.pf_ess_L = float(stL["ess"])
            self.pf_ess_R = float(stR["ess"])
            self.pf_mode_prob_high_L = float(stL["mode_prob_high"])
            self.pf_mode_prob_high_R = float(stR["mode_prob_high"])

        except Exception:
            self.pf_exception_count += 1
            self.hip_torque_L = 0.0
            self.hip_torque_R = 0.0
            self.filtered_hip_torque_L = 0.0
            self.filtered_hip_torque_R = 0.0
            self.L_p = 0.0
            self.L_d = 0.0
            self.R_p = 0.0
            self.R_d = 0.0

        self._frame_idx += 1

        dt_ms = (time.perf_counter() - t0) * 1000.0
        self.pf_compute_ms = float(dt_ms)
        self._compute_hist.append(self.pf_compute_ms)
        if len(self._compute_hist) >= 20:
            self.pf_compute_p95_ms = float(np.percentile(np.asarray(self._compute_hist, dtype=np.float32), 95.0))
        else:
            self.pf_compute_p95_ms = self.pf_compute_ms
        if self.pf_compute_ms > self._compute_warn_ms:
            self.pf_overrun_count += 1

        return np.array([self.hip_torque_L, self.hip_torque_R], dtype=np.float32)
