"""
MyoAssist controller wrapper for RPi_Unified.

Ported from:
  Algorithm Reference/NJIT Reference/HipExoCode_Apr20.py

Key behavior kept identical to NJIT path:
  - 18-dim observation: [input_hist(2x4), current_input(4), output_hist(3x2)]
  - current_input order: [right_angle, left_angle, right_vel, left_vel] (radians)
  - symmetric actor forward with swapped observation indices
  - torque mapping: tau_R=out[0]*max_torque, tau_L=out[1]*max_torque
"""

from __future__ import annotations

from collections import deque
import io
import zipfile

import numpy as np
import torch
import torch.nn as nn


class IdentityFilter:
    def filter(self, x):
        return x


class ExoActorMyoAssist(nn.Module):
    SWAP_INDICES = [1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 17, 16]

    def __init__(self, symmetric=True):
        super().__init__()
        self.symmetric = bool(symmetric)
        self.register_buffer("_swap_idx", torch.tensor(self.SWAP_INDICES, dtype=torch.long))
        self.net = nn.Sequential(
            nn.Linear(18, 128), nn.Tanh(),
            nn.Linear(128, 64), nn.Tanh(),
            nn.Linear(64, 2), nn.Tanh(),
        )

    def forward(self, obs):
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)
        if self.symmetric:
            swapped = obs.index_select(1, self._swap_idx)
            batch = torch.cat([obs, swapped], dim=0)
            out = self.net(batch)
            # Match NJIT mapping: [orig[0], swapped[0]] -> [tau_R basis, tau_L basis]
            return torch.cat([out[0:1, 0:1], out[1:2, 0:1]], dim=-1)
        return self.net(obs)


class MyoAssistController:
    INPUT_HISTORY_FRAMES = 2
    OUTPUT_HISTORY_FRAMES = 3

    def __init__(self, model_path: str, max_torque_nm: float = 15.0, symmetric: bool = True):
        self.model_path = str(model_path)
        self.max_torque_nm = float(max_torque_nm)

        self.actor = ExoActorMyoAssist(symmetric=symmetric)
        self._load_from_zip(self.model_path)
        self.actor.eval()

        self.input_history = deque(maxlen=self.INPUT_HISTORY_FRAMES)
        for _ in range(self.INPUT_HISTORY_FRAMES):
            self.input_history.append(np.zeros(4, dtype=np.float32))
        self.output_history = deque(maxlen=self.OUTPUT_HISTORY_FRAMES)
        for _ in range(self.OUTPUT_HISTORY_FRAMES):
            self.output_history.append(np.zeros(2, dtype=np.float32))

        # Keep fields compatible with existing NN classes.
        self.left_exo_filter = IdentityFilter()
        self.right_exo_filter = IdentityFilter()
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

    def _load_from_zip(self, zip_path: str):
        with zipfile.ZipFile(zip_path, "r") as zf:
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
        self.actor.net.load_state_dict(exo_sd)

    def _build_obs(self, L_IMU_angle, R_IMU_angle, L_IMU_Vel, R_IMU_Vel):
        left_angle_rad = float(L_IMU_angle) * (np.pi / 180.0)
        right_angle_rad = float(R_IMU_angle) * (np.pi / 180.0)
        left_vel_rad = float(L_IMU_Vel) * (np.pi / 180.0)
        right_vel_rad = float(R_IMU_Vel) * (np.pi / 180.0)

        current_input = np.array([
            right_angle_rad,
            left_angle_rad,
            right_vel_rad,
            left_vel_rad,
        ], dtype=np.float32)

        parts = list(self.input_history) + [current_input] + list(self.output_history)
        obs = np.concatenate(parts).astype(np.float32, copy=False)
        return obs, current_input

    def generate_assistance(self, L_IMU_angle, R_IMU_angle, L_IMU_Vel, R_IMU_Vel):
        obs, current_input = self._build_obs(L_IMU_angle, R_IMU_angle, L_IMU_Vel, R_IMU_Vel)
        obs_t = torch.from_numpy(obs).unsqueeze(0)
        with torch.no_grad():
            pred = self.actor(obs_t).cpu().numpy().reshape(-1).astype(np.float32)

        torque_R = float(pred[0]) * self.max_torque_nm
        torque_L = float(pred[1]) * self.max_torque_nm

        self.hip_torque_L = torque_L
        self.hip_torque_R = torque_R
        # No internal torque filter for MyoAssist (training path used raw output).
        self.filtered_hip_torque_L = torque_L
        self.filtered_hip_torque_R = torque_R

        # For protocol compatibility only.
        self.L_p = torque_L
        self.R_p = torque_R
        self.L_d = 0.0
        self.R_d = 0.0

        self.input_history.append(current_input.copy())
        self.output_history.append(pred.copy())
        return pred
