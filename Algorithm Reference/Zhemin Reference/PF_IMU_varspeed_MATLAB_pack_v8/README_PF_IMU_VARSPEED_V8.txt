PF+DS IMU Variable-Speed MATLAB Pack v8
======================================

This pack is for the new CSV dataset containing:
  Time_ms
  imu_LTx, imu_RTx
  imu_Lvel, imu_Rvel

Default data path in the demo:
  C:\Users\12617\Documents\MATLAB\PF_DS\PF_VARY SPEED\PI5_lstm_pd-20260424-221451.csv

Default tested segment:
  210 s to 280 s

This segment is treated as variable-speed walking:
  1.25 -> 1.75 -> 0.75

Main files:
  demo_run_pf_imu_varspeed_v8.m
  run_pf_imu_varspeed_demo_v8.m

What this script does:
  1. Loads L/R hip angle and velocity from IMU columns.
  2. Converts degrees and deg/s to radians and rad/s.
  3. Runs the hybrid-attractor PF+DS intent estimator:
       dq_int = A(q-qstar), A<0
  4. Uses velocity-only particle weights, because no acceleration is available:
       etaA = 0
  5. Computes:
       velocity RMSE
       velocity sign agreement
       one-step prediction q_{k+1}=q_k+dt*dq_int_hat
       paper-style confidence
  6. Tests the proposed human-side torque policy offline:
       tau_exo^h = tau_gc + B(c)*dq_int_hat + D(c)*(dq_int_hat-dq)

Important:
  Gravity compensation is disabled by default because hardware mass/COM
  parameters are unknown. You can enable it by setting:
       'useGravityComp', true
       'alphaG', ...
       'gSin', ...
       'gCos', ...
       'gBias', ...

Recommended first run:
  demo_run_pf_imu_varspeed_v8

Main tuning:
  etaV             velocity likelihood weight
  etaSign          direction/mode consistency penalty
  dConf            confidence ascent threshold
  confWindowSec    confidence integration window
  bMax             max velocity-feedforward assistance gain
  dMax             max velocity-error correction gain

Acceleration:
  This dataset does not include acceleration. The particle acceleration
  likelihood is intentionally disabled with etaA = 0.
