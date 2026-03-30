"""
DNN (Dense Neural Network) - 前馈神经网络控制器
输入: 18维 (历史2帧角度+速度 + 当前角度+速度 + 历史3帧NN输出)
输出: 2维 (左/右腿参考角度)
滤波器: 5个接口 (input_pos, input_vel, vel, ref, torque)
"""

import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

import numpy as np
import torch
from .base_network import Network

import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from filter_library import create_filter, RECOMMENDED_FILTERS


class DNN:
    def __init__(self,
                 # ====== kp, kd 定义在最前面，方便修改 ======
                 kp=50.0,
                 kd=14.142,
                 # ====== 网络结构 ======
                 n_input=18, n_layer_1=128, n_layer_2=64, n_output=2,
                 saved_policy_path='./models/dnn/Trained_model3.pt',
                 # ====== 速度计算方式 ======
                 use_velocity_from_derivative=False,
                 derivative_dt=0.01,
                 derivative_smooth_filter_type=None,
                 derivative_smooth_filter_b=None,
                 derivative_smooth_filter_a=None,
                 # ====== 输入信号滤波器 (滤波原始IMU数据) ======
                 input_pos_filter_type=None,
                 input_pos_filter_b=None,
                 input_pos_filter_a=None,
                 input_vel_filter_type=None,
                 input_vel_filter_b=None,
                 input_vel_filter_a=None,
                 # ====== 速度滤波器 (PD控制的D项) ======
                 vel_filter_type='butter_12hz_2nd',
                 vel_filter_b=None,
                 vel_filter_a=None,
                 # ====== NN输出滤波器 (滤波qHr) ======
                 ref_filter_type='butter_12hz_2nd',
                 ref_filter_b=None,
                 ref_filter_a=None,
                 # ====== Torque滤波器 (最终扭矩输出) ======
                 torque_filter_type=None,
                 torque_filter_b=None,
                 torque_filter_a=None,
                 torque_filter_enable=False,
                 # ====== Zero Mean ======
                 enable_zero_mean=False,
                 zero_mean_buffer_size=200,
                 zero_mean_warmup=100
                 ):

        self.n_input = n_input
        self.n_layer_1 = n_layer_1
        self.n_layer_2 = n_layer_2
        self.n_output = n_output

        self.saved_policy_path = saved_policy_path
        self.network = Network(self.n_input, self.n_layer_1, self.n_layer_2, self.n_output)
        self.network.load_saved_policy(torch.load(self.saved_policy_path, map_location=torch.device('cpu')))

        # ========= 速度计算方式配置 =========
        self.use_velocity_from_derivative = use_velocity_from_derivative
        self.derivative_dt = derivative_dt

        self.prev_pos_L_deg = None
        self.prev_pos_R_deg = None

        self.pos_smoothed_L_deg = 0.0
        self.pos_smoothed_R_deg = 0.0
        self.vel_from_derivative_L_deg = 0.0
        self.vel_from_derivative_R_deg = 0.0

        # ========= 求导前位置平滑滤波器 =========
        if derivative_smooth_filter_b is not None and derivative_smooth_filter_a is not None:
            self.left_derivative_smooth_filter = create_filter(b=derivative_smooth_filter_b, a=derivative_smooth_filter_a)
            self.right_derivative_smooth_filter = create_filter(b=derivative_smooth_filter_b, a=derivative_smooth_filter_a)
            print(f"[求导平滑滤波器] 使用自定义系数: b={derivative_smooth_filter_b}, a={derivative_smooth_filter_a}")
        elif derivative_smooth_filter_type is not None:
            self.left_derivative_smooth_filter = create_filter(filter_name=derivative_smooth_filter_type)
            self.right_derivative_smooth_filter = create_filter(filter_name=derivative_smooth_filter_type)
            print(f"[求导平滑滤波器] 使用预定义滤波器: {derivative_smooth_filter_type}")
        else:
            self.left_derivative_smooth_filter = None
            self.right_derivative_smooth_filter = None

        if self.use_velocity_from_derivative:
            print(f"[速度计算] 使用位置求导, dt={derivative_dt*1000:.2f}ms")
        else:
            print(f"[速度计算] 使用IMU直接测量")

        # ========= 输入位置滤波器 =========
        if input_pos_filter_b is not None and input_pos_filter_a is not None:
            self.left_input_pos_filter = create_filter(b=input_pos_filter_b, a=input_pos_filter_a)
            self.right_input_pos_filter = create_filter(b=input_pos_filter_b, a=input_pos_filter_a)
            print(f"[输入位置滤波器] 使用自定义系数")
        elif input_pos_filter_type is not None:
            self.left_input_pos_filter = create_filter(filter_name=input_pos_filter_type)
            self.right_input_pos_filter = create_filter(filter_name=input_pos_filter_type)
            print(f"[输入位置滤波器] 使用预定义滤波器: {input_pos_filter_type}")
        else:
            self.left_input_pos_filter = None
            self.right_input_pos_filter = None

        # ========= 输入速度滤波器 =========
        if input_vel_filter_b is not None and input_vel_filter_a is not None:
            self.left_input_vel_filter = create_filter(b=input_vel_filter_b, a=input_vel_filter_a)
            self.right_input_vel_filter = create_filter(b=input_vel_filter_b, a=input_vel_filter_a)
            print(f"[输入速度滤波器] 使用自定义系数")
        elif input_vel_filter_type is not None:
            self.left_input_vel_filter = create_filter(filter_name=input_vel_filter_type)
            self.right_input_vel_filter = create_filter(filter_name=input_vel_filter_type)
            print(f"[输入速度滤波器] 使用预定义滤波器: {input_vel_filter_type}")
        else:
            self.left_input_vel_filter = None
            self.right_input_vel_filter = None

        # ========= 速度滤波器 (PD控制D项) =========
        if vel_filter_b is not None and vel_filter_a is not None:
            self.left_vel_filter = create_filter(b=vel_filter_b, a=vel_filter_a)
            self.right_vel_filter = create_filter(b=vel_filter_b, a=vel_filter_a)
            print(f"[VelFilter] 使用自定义系数")
        elif vel_filter_type is not None:
            self.left_vel_filter = create_filter(vel_filter_type)
            self.right_vel_filter = create_filter(vel_filter_type)
            print(f"[VelFilter] 使用预定义: {vel_filter_type}")
        else:
            raise ValueError("必须指定 vel_filter_type 或 vel_filter_b/a")

        # ========= NN输出滤波器 (qHr) =========
        if ref_filter_b is not None and ref_filter_a is not None:
            self.left_ref_filter = create_filter(b=ref_filter_b, a=ref_filter_a)
            self.right_ref_filter = create_filter(b=ref_filter_b, a=ref_filter_a)
            print(f"[RefFilter] 使用自定义系数")
        elif ref_filter_type is not None:
            self.left_ref_filter = create_filter(ref_filter_type)
            self.right_ref_filter = create_filter(ref_filter_type)
            print(f"[RefFilter] 使用预定义: {ref_filter_type}")
        else:
            raise ValueError("必须指定 ref_filter_type 或 ref_filter_b/a")

        # ========= Torque滤波器 =========
        self.torque_filter_enable = torque_filter_enable
        if torque_filter_enable:
            if torque_filter_b is not None and torque_filter_a is not None:
                self.left_torque_filter = create_filter(b=torque_filter_b, a=torque_filter_a)
                self.right_torque_filter = create_filter(b=torque_filter_b, a=torque_filter_a)
                print(f"[TorqueFilter] 使用自定义系数")
            elif torque_filter_type is not None:
                self.left_torque_filter = create_filter(torque_filter_type)
                self.right_torque_filter = create_filter(torque_filter_type)
                print(f"[TorqueFilter] 使用预定义: {torque_filter_type}")
            else:
                print("[TorqueFilter] 警告: 启用了torque滤波但未指定滤波器，将禁用")
                self.torque_filter_enable = False
                self.left_torque_filter = None
                self.right_torque_filter = None
        else:
            self.left_torque_filter = None
            self.right_torque_filter = None
            print(f"[TorqueFilter] 未启用")

        # Zero Mean 配置
        self.enable_zero_mean = enable_zero_mean
        self.zero_mean_buffer_size = zero_mean_buffer_size
        self.zero_mean_warmup = zero_mean_warmup

        self.in_1 = np.ones(4)
        self.in_2 = np.ones(4)
        self.out_3 = np.ones(2)
        self.out_2 = np.ones(2)
        self.out_1 = np.ones(2)
        self.input_data = np.zeros(self.n_input)

        self.qTd_L = 10
        self.qTd_R = 10
        self.dqTd_L = 0
        self.dqTd_R = 0

        self.qHr_L = 0
        self.qHr_R = 0

        self.kp2 = kp
        self.kp3 = kp
        self.kd2 = kd
        self.kd3 = kd

        self.dqTd_history_L = np.zeros(3)
        self.dqTd_filtered_history_L = np.zeros(3)
        self.dqTd_filtered_L = 0
        self.dqTd_history_R = np.zeros(3)
        self.dqTd_filtered_history_R = np.zeros(3)
        self.dqTd_filtered_R = 0

        self.hip_torque_L = 0
        self.hip_torque_R = 0

        # Zero mean tracking
        self.angle_buffer_L = []
        self.angle_buffer_R = []
        self.buffer_size = zero_mean_buffer_size
        self.mean_L = 0.0
        self.mean_R = 0.0

    def generate_assistance(self, LTx, RTx, LTAVx, RTAVx):
        # ========= 输入信号滤波 (可选) =========
        if self.left_input_pos_filter is not None:
            LTx = self.left_input_pos_filter.filter(LTx)
            RTx = self.right_input_pos_filter.filter(RTx)

        # ========= 速度计算 =========
        if self.use_velocity_from_derivative:
            if self.left_derivative_smooth_filter is not None:
                pos_smoothed_L = self.left_derivative_smooth_filter.filter(LTx)
                pos_smoothed_R = self.right_derivative_smooth_filter.filter(RTx)
            else:
                pos_smoothed_L = LTx
                pos_smoothed_R = RTx

            self.pos_smoothed_L_deg = pos_smoothed_L
            self.pos_smoothed_R_deg = pos_smoothed_R

            if self.prev_pos_L_deg is not None:
                LTAVx = (pos_smoothed_L - self.prev_pos_L_deg) / self.derivative_dt
                RTAVx = (pos_smoothed_R - self.prev_pos_R_deg) / self.derivative_dt
            else:
                LTAVx = LTAVx if LTAVx is not None else 0.0
                RTAVx = RTAVx if RTAVx is not None else 0.0

            self.vel_from_derivative_L_deg = LTAVx
            self.vel_from_derivative_R_deg = RTAVx

            self.prev_pos_L_deg = pos_smoothed_L
            self.prev_pos_R_deg = pos_smoothed_R
        else:
            if self.left_input_vel_filter is not None:
                LTAVx = self.left_input_vel_filter.filter(LTAVx)
                RTAVx = self.right_input_vel_filter.filter(RTAVx)

            self.pos_smoothed_L_deg = LTx
            self.pos_smoothed_R_deg = RTx
            self.vel_from_derivative_L_deg = 0.0
            self.vel_from_derivative_R_deg = 0.0

        # Convert to radians
        self.qTd_L = LTx * np.pi / 180.0
        self.qTd_R = RTx * np.pi / 180.0
        self.dqTd_L = LTAVx * np.pi / 180.0
        self.dqTd_R = RTAVx * np.pi / 180.0

        # Zero mean processing
        if self.enable_zero_mean:
            self.angle_buffer_L.append(self.qTd_L)
            self.angle_buffer_R.append(self.qTd_R)
            if len(self.angle_buffer_L) > self.buffer_size:
                self.angle_buffer_L.pop(0)
                self.angle_buffer_R.pop(0)

            if len(self.angle_buffer_L) >= self.zero_mean_warmup:
                self.mean_L = np.mean(self.angle_buffer_L)
                self.mean_R = np.mean(self.angle_buffer_R)
                self.qTd_L = self.qTd_L - self.mean_L
                self.qTd_R = self.qTd_R - self.mean_R
        else:
            self.mean_L = 0.0
            self.mean_R = 0.0

        # Velocity filter (for PD D-term)
        self.dqTd_filtered_L = self.left_vel_filter.filter(self.dqTd_L)
        self.dqTd_filtered_R = self.right_vel_filter.filter(self.dqTd_R)

        # NN input (uses raw velocity, not filtered)
        self.input_data = np.concatenate((
            self.in_2, self.in_1,
            self.qTd_L, self.qTd_R, self.dqTd_L, self.dqTd_R,
            self.out_3, self.out_2, self.out_1
        ), axis=None)
        self.in_2 = np.copy(self.in_1)
        self.in_1 = np.array([self.qTd_L, self.qTd_R, self.dqTd_L, self.dqTd_R])
        self.out_3 = np.copy(self.out_2)
        self.out_2 = np.copy(self.out_1)

        input_data_tensor = torch.tensor(self.input_data, dtype=torch.float32)
        output_tensor = self.network(input_data_tensor)
        output_data = output_tensor.detach().numpy()

        # NN raw output
        self.qHr_L_ori, self.qHr_R_ori = output_data
        self.out_1 = np.copy(output_data)

        # Filter NN output
        self.qHr_L = self.left_ref_filter.filter(self.qHr_L_ori)
        self.qHr_R = self.right_ref_filter.filter(self.qHr_R_ori)

        # Scale by 0.1
        self.qHr_L_ori = self.qHr_L_ori * 0.1
        self.qHr_R_ori = self.qHr_R_ori * 0.1
        self.qHr_L = self.qHr_L * 0.1
        self.qHr_R = self.qHr_R * 0.1

        # Hip torque (degree domain)
        qHr_L_deg = self.qHr_L * 180.0 / np.pi
        qHr_R_deg = self.qHr_R * 180.0 / np.pi
        qTd_L_deg = self.qTd_L * 180.0 / np.pi
        qTd_R_deg = self.qTd_R * 180.0 / np.pi
        dqTd_filtered_L_deg = self.dqTd_filtered_L * 180.0 / np.pi
        dqTd_filtered_R_deg = self.dqTd_filtered_R * 180.0 / np.pi

        hip_torque_L_raw = ((qHr_L_deg - qTd_L_deg) * 50.0 - dqTd_filtered_L_deg * 14.142) * 0.008
        hip_torque_R_raw = ((qHr_R_deg - qTd_R_deg) * 50.0 - dqTd_filtered_R_deg * 14.142) * 0.008

        # Optional torque filter
        if self.torque_filter_enable and self.left_torque_filter is not None:
            self.hip_torque_L = self.left_torque_filter.filter(hip_torque_L_raw)
            self.hip_torque_R = self.right_torque_filter.filter(hip_torque_R_raw)
        else:
            self.hip_torque_L = hip_torque_L_raw
            self.hip_torque_R = hip_torque_R_raw

        self.hip_torque_L_raw = hip_torque_L_raw
        self.hip_torque_R_raw = hip_torque_R_raw

        # P and D components for logging
        self.L_p = (qHr_L_deg - qTd_L_deg) * 50.0 * 0.008
        self.L_d = -dqTd_filtered_L_deg * 14.142 * 0.008
        self.R_p = (qHr_R_deg - qTd_R_deg) * 50.0 * 0.008
        self.R_d = -dqTd_filtered_R_deg * 14.142 * 0.008

        self.input_angle_L_deg = LTx
        self.input_angle_R_deg = RTx
        self.input_vel_L_deg_s = LTAVx
        self.input_vel_R_deg_s = RTAVx

        self.power_L = self.hip_torque_L * self.dqTd_filtered_L
        self.power_R = self.hip_torque_R * self.dqTd_filtered_R

        return self.hip_torque_L, self.hip_torque_R, self.L_p, self.L_d, self.R_p, self.R_d
