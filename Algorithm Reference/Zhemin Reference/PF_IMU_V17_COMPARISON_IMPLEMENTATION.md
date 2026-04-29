# PF-IMU v17 对比实现说明（过程与结果）

## 1. 目标

把对方提供的 `PF_IMU_varspeed_cycle_MATLAB_pack_v17_guided_raw_online_interactive_html_fixed` 逻辑尽量一致移植到 Pi 侧，并验证两件事：

1. **本地参考实现** 与 **本地部署实现** 是否一致（证明移植正确性）。
2. **MATLAB 参考扭矩** 与 **本地输出** 差异有多大（证明差异主要来源于口径/流程，而不只是代码 bug）。

---

## 2. 输入与参考

- 输入数据：
  - `/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/Algorithm Reference/NJIT Reference/PI5_lstm_pd-20260424-221451.csv`
- 时间区间：
  - `50s ~ 150s`
- 使用列：
  - `imu_LTx, imu_RTx, imu_Lvel, imu_Rvel`
- MATLAB 参考扭矩（ground truth reference）：
  - `/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/Algorithm Reference/Zhemin Reference/50_150_210_280/50_150s/csv/control_torque_50_150s.csv`

---

## 3. 实现路径

### 3.1 Pi 运行实现（部署路径）

- 文件：
  - `/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/RPi_Unified/networks/pf_imu.py`
  - `/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/RPi_Unified/RL_controller_torch.py`

### 3.2 一致性评估脚本（离线复现实验）

- 文件：
  - `/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/tools/pf_imu_consistency_eval.py`
  - `/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/Algorithm Reference/Zhemin Reference/pf_imu_matlab_local_compare.py`

---

## 4. 验证方法

### Step A：本地 ref vs 本地 deploy（一致性）

```bash
python3 tools/pf_imu_consistency_eval.py \
  --csv "Algorithm Reference/NJIT Reference/PI5_lstm_pd-20260424-221451.csv" \
  --t0 50 --t1 150 --seeds 10 \
  --out-prefix "Algorithm Reference/Zhemin Reference/pf_imu_consistency_50_150s_v17"
```

输出：
- `pf_imu_consistency_50_150s_v17_single.csv`
- `pf_imu_consistency_50_150s_v17_mcmean.csv`

说明：
- `single` = 单 seed 结果（路径级比较）
- `mcmean` = 多 seed 均值（统计一致性）

### Step B：MATLAB reference vs 本地（差异定位）

```bash
python3 "Algorithm Reference/Zhemin Reference/pf_imu_matlab_local_compare.py" \
  --input-csv "Algorithm Reference/NJIT Reference/PI5_lstm_pd-20260424-221451.csv" \
  --matlab-csv "Algorithm Reference/Zhemin Reference/50_150_210_280/50_150s/csv/control_torque_50_150s.csv" \
  --matlab-mat "Algorithm Reference/Zhemin Reference/50_150_210_280/50_150s/mat/PF_IMU_v12_50_150s_result.mat" \
  --t0 50 --t1 150 --seeds 10
```

输出目录：
- `/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/Algorithm Reference/Zhemin Reference/pf_imu_compare_results_50_150s`

---

## 5. 关键结果

### 5.1 本地 ref 与本地 deploy（是否一致）

- `single`:
  - L: corr=0.9920, RMSE=0.2413 Nm
  - R: corr=0.9922, RMSE=0.2711 Nm
- `mcmean`:
  - L: corr=0.999126, RMSE=0.0797 Nm, lag=0 ms
  - R: corr=0.999249, RMSE=0.0837 Nm, lag=0 ms

结论：**移植实现内部一致性非常高**，部署路径和参考路径在同一输入下几乎重合。

### 5.2 MATLAB 参考 vs 本地输出

来自 `pf_imu_zhemin_vs_local_metrics_50_150s.csv`：

- `matlab vs local_deploy_mcmean`
  - L: corr=0.5391, RMSE=1.6095 Nm, best_lag=-60 ms, best_lag_corr=0.6401
  - R: corr=0.5400, RMSE=1.8175 Nm, best_lag=-60 ms, best_lag_corr=0.5996
- `matlab vs local_ref_mcmean`
  - L: corr=0.5391, RMSE=1.6133 Nm
  - R: corr=0.5403, RMSE=1.8176 Nm

结论：MATLAB 参考与本地两条路径都存在明显差距，且最佳对齐在约 `-60ms`。  
这更像是**模型口径/流程差异**，而不是“本地部署代码实现错误”。

---

## 6. 可视化产物

- 交互 HTML（可缩放、平移、区间拖拽、Torque Only/All Panels）：
  - `/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/Algorithm Reference/Zhemin Reference/pf_imu_compare_results_50_150s/pf_imu_interactive_viewer.html`
- 轨迹对比与误差图：
  - `pf_imu_zhemin_compare_L_overlay_error.png`
  - `pf_imu_zhemin_compare_R_overlay_error.png`
  - `pf_imu_zhemin_compare_zoom_50_60s.png`

---

## 7. 现阶段判断

1. 代码移植 correctness：通过（ref/deploy 高一致）。
2. 与对方 MATLAB 结果完全重合：未通过（约 0.54 相关，-60ms 最佳滞后）。
3. 后续重点：应优先排查 MATLAB 链路与当前链路在数据预处理、参数口径、实时因果性上的差异，而不是继续怀疑 Pi 主体实现写错。

