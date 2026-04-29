PF+DS IMU Variable-Speed Cycle Analysis v12
==========================================

This is based on v11.

New in v12:
1. Runs two analysis windows in one command:
   - 50_150s
   - 210_280s

2. Saves computed control torques into CSV files:
   - control_torque_50_150s.csv
   - control_torque_210_280s.csv

Default input:
  C:\Users\12617\Documents\MATLAB\PF_DS\PF_VARY SPEED\PI5_lstm_pd-20260424-221451.csv

Default output root:
  C:\Users\12617\Documents\MATLAB\PF_DS\PF_VARY SPEED\PF_IMU_v12_outputs

Output structure:
  PF_IMU_v12_outputs/
    50_150s/
      figures/
        PF_IMU_v12_50_150s_pf_v8_style_analysis_L.png
        PF_IMU_v12_50_150s_pf_v8_style_analysis_R.png
        PF_IMU_v12_50_150s_cycle_mean_std_LR_overlay_fixed.png
      csv/
        control_torque_50_150s.csv
        summary_metrics_50_150s.csv
        cycle_mean_std_L_50_150s.csv
        cycle_mean_std_R_50_150s.csv
        window_info_50_150s.csv
      mat/
        PF_IMU_v12_50_150s_result.mat

    210_280s/
      figures/
        PF_IMU_v12_210_280s_pf_v8_style_analysis_L.png
        PF_IMU_v12_210_280s_pf_v8_style_analysis_R.png
        PF_IMU_v12_210_280s_cycle_mean_std_LR_overlay_fixed.png
      csv/
        control_torque_210_280s.csv
        summary_metrics_210_280s.csv
        cycle_mean_std_L_210_280s.csv
        cycle_mean_std_R_210_280s.csv
        window_info_210_280s.csv
      mat/
        PF_IMU_v12_210_280s_result.mat

Control torque CSV columns include:
  time_abs_s
  time_window_s
  q_L_deg, dq_L_deg_s, dq_int_L_deg_s
  confidence_L, B_L, D_L
  tau_ff_L, tau_damp_L, tau_exo_L, tau_measured_L
  q_R_deg, dq_R_deg_s, dq_int_R_deg_s
  confidence_R, B_R, D_R
  tau_ff_R, tau_damp_R, tau_exo_R, tau_measured_R

Run:
  demo_run_pf_imu_varspeed_cycle_v12
