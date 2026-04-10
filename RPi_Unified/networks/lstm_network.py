"""
LSTMNetwork - LSTM神经网络控制器
输入: 4维 (左右角度 + 左右速度)
输出: 2维 (左/右腿扭矩)
滤波器: 只需torque前滤波 (exo_filter)
"""

import numpy as np
import torch
import torch.nn as nn

from filter_library import IIRFilter


class LSTMNetwork(nn.Module):
    def __init__(self,
                 # ====== kp, kd 定义在最前面，方便修改 ======
                 kp=50.0,
                 kd=1.0,
                 # ====== 网络结构 ======
                 n_input=4, n_layer_1=256, num_layers=2, n_output=2,
                 # ====== torque前滤波器系数 ======
                 b=np.array([0.06745527, 0.13491055, 0.06745527]),
                 a=np.array([1., -1.1429805, 0.4128016])
                 ):
        super(LSTMNetwork, self).__init__()

        self.p_lstm1 = nn.LSTM(n_input, n_layer_1, num_layers, batch_first=True)
        self.p_fc3 = nn.Linear(n_layer_1, n_output)

        self.qTd_L = 10
        self.qTd_R = 10
        self.dqTd_L = 0
        self.dqTd_R = 0

        self.qHr_L = 0.0
        self.qHr_R = 0.0

        self.kp = kp
        self.kd = kd

        self.b = b
        self.a = a

        # 只需要 exo_filter (torque前滤波)
        self.left_exo_filter = IIRFilter(b=self.b, a=self.a)
        self.right_exo_filter = IIRFilter(b=self.b, a=self.a)

        self.hip_torque_L = 0
        self.hip_torque_R = 0
        self.filtered_hip_torque_L = 0
        self.filtered_hip_torque_R = 0
        self.L_p = 0
        self.L_d = 0
        self.R_p = 0
        self.R_d = 0

    def forward(self, x):
        p_out, _ = self.p_lstm1(x)
        if p_out.dim() == 2:
            p_out = p_out.unsqueeze(1)
        p_out = p_out[:, -1, :]
        p_out = torch.relu(p_out)
        p_out = self.p_fc3(p_out)
        return p_out.detach().numpy().reshape(-1)  # always 1-d, guards against 0-d scalar

    def get_predicted_action(self, L_IMU_angle, R_IMU_angle, L_IMU_Vel, R_IMU_Vel):
        state = np.array([L_IMU_angle, R_IMU_angle, L_IMU_Vel, R_IMU_Vel])
        state_tensor = torch.tensor(state[np.newaxis, :], dtype=torch.float32)
        action = self.forward(state_tensor)
        # Guard: if model has only 1 output (e.g. legdcp weights loaded into LSTMNetwork),
        # use the same value for both legs
        if action.size < 2:
            v = float(action[0])
            return v, v
        return action[0], action[1]

    def generate_assistance(self, L_IMU_angle, R_IMU_angle, L_IMU_Vel, R_IMU_Vel):
        self.qTd_L = L_IMU_angle * np.pi / 180.0
        self.qTd_R = R_IMU_angle * np.pi / 180.0
        self.dqTd_L = L_IMU_Vel * np.pi / 180.0
        self.dqTd_R = R_IMU_Vel * np.pi / 180.0

        action = self.get_predicted_action(self.qTd_L, self.qTd_R, self.dqTd_L, self.dqTd_R)

        self.hip_torque_L = 0.1 * action[0] * self.kp + self.dqTd_L * self.kd
        self.hip_torque_R = 0.1 * action[1] * self.kp + self.dqTd_R * self.kd
        self.filtered_hip_torque_L = self.left_exo_filter.filter(self.hip_torque_L)
        self.filtered_hip_torque_R = self.right_exo_filter.filter(self.hip_torque_R)

        # D control
        self.L_d = self.dqTd_L * self.kd
        self.R_d = self.dqTd_R * self.kd

        # P control
        self.L_p = 0.1 * action[0] * self.kp
        self.R_p = 0.1 * action[1] * self.kp

        return action

    def load_saved_policy(self, state_dict):
        self.p_lstm1.weight_ih_l0.data = state_dict['p_lstm1.weight_ih_l0']
        self.p_lstm1.weight_hh_l0.data = state_dict['p_lstm1.weight_hh_l0']
        self.p_lstm1.bias_ih_l0.data = state_dict['p_lstm1.bias_ih_l0']
        self.p_lstm1.bias_hh_l0.data = state_dict['p_lstm1.bias_hh_l0']
        self.p_lstm1.weight_ih_l1.data = state_dict['p_lstm1.weight_ih_l1']
        self.p_lstm1.weight_hh_l1.data = state_dict['p_lstm1.weight_hh_l1']
        self.p_lstm1.bias_ih_l1.data = state_dict['p_lstm1.bias_ih_l1']
        self.p_lstm1.bias_hh_l1.data = state_dict['p_lstm1.bias_hh_l1']
        self.p_fc3.weight.data = state_dict['p_fc3.weight']
        self.p_fc3.bias.data = state_dict['p_fc3.bias']
