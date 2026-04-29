# PF-IMU MATLAB vs Local Comparison (50-150s)

## Scope
目标：在同一输入数据上，对比以下三条链路输出的左右扭矩，并确认“移植实现是否一致”。

1. MATLAB 参考扭矩（Zhemin 提供）
2. Local Ref（`tools/pf_imu_consistency_eval.py` 中 `run_reference_leg`）
3. Local Deploy（`RPi_Unified/networks/pf_imu.py` 中 `PFIMUController`，即 Pi 真正部署路径）

## Inputs

- 输入数据（50s~150s）  
  `/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/Algorithm Reference/NJIT Reference/PI5_lstm_pd-20260424-221451.csv`
- 使用列  
  `imu_LTx, imu_RTx, imu_Lvel, imu_Rvel`
- MATLAB 参考扭矩（ground truth reference）  
  `/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/Algorithm Reference/Zhemin Reference/50_150_210_280/50_150s/csv/control_torque_50_150s.csv`

结论先说：本对比里的 MATLAB 参考扭矩**完全来自上面这份 `control_torque_50_150s.csv`**。

## Reproduce

```bash
python3 tools/pf_imu_consistency_eval.py \
  --csv "Algorithm Reference/NJIT Reference/PI5_lstm_pd-20260424-221451.csv" \
  --t0 50 --t1 150 --seeds 10 \
  --out-prefix "Algorithm Reference/Zhemin Reference/pf_imu_consistency_50_150s_v17"

python3 "Algorithm Reference/Zhemin Reference/pf_imu_matlab_local_compare.py" \
  --input-csv "Algorithm Reference/NJIT Reference/PI5_lstm_pd-20260424-221451.csv" \
  --matlab-csv "Algorithm Reference/Zhemin Reference/50_150_210_280/50_150s/csv/control_torque_50_150s.csv" \
  --matlab-mat "Algorithm Reference/Zhemin Reference/50_150_210_280/50_150s/mat/PF_IMU_v12_50_150s_result.mat" \
  --t0 50 --t1 150 --seeds 10
```

## Outputs

- 一致性（local ref vs local deploy，v17）
  - `Algorithm Reference/Zhemin Reference/pf_imu_consistency_50_150s_v17_single.csv`
  - `Algorithm Reference/Zhemin Reference/pf_imu_consistency_50_150s_v17_mcmean.csv`
- MATLAB 对比输出目录
  - `Algorithm Reference/Zhemin Reference/pf_imu_compare_results_50_150s/`
  - 核心指标文件：`pf_imu_zhemin_vs_local_metrics_50_150s.csv`
  - 轨迹文件：`pf_imu_zhemin_vs_local_traces_50_150s.csv`

## Key Results (2026-04-29, v17 guided-raw profile)

### A) 本地“仿真路径”与“部署路径”是否一致？

是，一致性非常高：

- `local_ref_mcmean vs local_deploy_mcmean`
  - L: `corr=0.999126`, `RMSE=0.0797 Nm`, `lag=0 ms`
  - R: `corr=0.999249`, `RMSE=0.0837 Nm`, `lag=0 ms`

这说明：**Pi 上的 PF-IMU 实现和本地参考逻辑在同一输入下基本一致**，移植路径本身没有明显错误。

### B) MATLAB 参考扭矩 vs 本地（v17）

- `matlab vs local_deploy_mcmean`
  - L: `corr=0.539`, `RMSE=1.610 Nm`, best lag=`-60 ms`，best-lag corr=`0.640`
  - R: `corr=0.540`, `RMSE=1.817 Nm`, best lag=`-60 ms`，best-lag corr=`0.600`

- `matlab vs local_ref_mcmean`
  - L: `corr=0.539`, `RMSE=1.613 Nm`
  - R: `corr=0.540`, `RMSE=1.818 Nm`

解释：MATLAB 参考与本地两条链路都差不多远（且都在约 `-60 ms` 处出现最佳对齐），这更像是**算法/参数/预处理口径差异**，而不是“部署实现写错”。

## Data Note

你提到的 `pf_imu_consistency_50_150s_single.csv`（旧文件）只有 torque 列，不含 `imu_*` 列。  
本次已生成含输入列的新文件：

- `pf_imu_consistency_50_150s_v17_single.csv`
- `pf_imu_consistency_50_150s_v17_mcmean.csv`

其中已包含：`imu_LTx, imu_RTx, imu_Lvel, imu_Rvel`，可直接用于后续对齐分析。

## Why this matters

这个结果说明两件事：

1. 本地实现内部是自洽的（ref 与 deploy 几乎重合）。
2. 真实部署难点不在“把代码搬过来”，而在“与外部 MATLAB 参考的完整口径一致化”（前处理、参数档位、随机过程细节、平滑/因果实现、单位与符号约定等）。

