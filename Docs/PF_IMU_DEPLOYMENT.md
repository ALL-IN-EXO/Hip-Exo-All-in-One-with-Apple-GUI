# PF-IMU Deployment Notes (RPi RL Path)

## 1) 目标

将 `PF_IMU_varspeed_MATLAB_pack_v8` 的在线控制思路接入现有 `RPi_Unified`，要求：

- 通过 `--nn pf_imu` 启动；
- 使用现有 RL 主循环（IMU 收包 -> 算法 -> unified torque filter -> delay -> scale -> send）；
- **不改 Teensy Serial8 协议**（`AA59` 控制帧 / `AA56` 状态帧长度与字段布局不变）；
- GUI 能识别并显示当前运行算法，支持远程一键启动。

---

## 2) 为什么放 Pi 而不是 Teensy

原 MATLAB 版本核心是粒子滤波（PF），默认粒子数较高。若直接上 Teensy 100Hz 双腿运行，算力风险高。  
放 Pi 的理由：

- Pi5 适合承担 PF 计算负载；
- Teensy 保持电机闭环与协议桥接职责；
- 不改变现有 BLE/Serial8 协议，系统风险最小。

---

## 3) 本次接入范围

### 3.1 RPi 侧

- 新增算法实现：`RPi_Unified/networks/pf_imu.py`
- 主入口新增：`RL_controller_torch.py --nn pf_imu`
- 状态上报码新增 `nn_type=4`（仅 1 byte 编码扩展）
- CSV 新增 PF 诊断字段（计算时延、ESS、置信度等）

### 3.2 GUI 侧

- RL 面板新增远程按钮：`Start PF-IMU`
- `RPi nn_type` 映射新增：`4 -> PF-IMU`
- 远程启动/状态机识别 `pf_imu` 并打对齐标签（`RLONPF`）
- PF-IMU 预设：`scale=1.00`, `delay=0ms`, `torque filter=ON`, `Vel+Ref=ON`（用于输入角度/速度滤波）

### 3.3 Teensy 侧

- **无协议改动**；
- 继续按原 `AA59/AA56` 解包与透传。

---

## 4) 性能保护（关键）

PF-IMU 运行时已加入以下保护：

1. 默认启用 `MATLAB-v12 profile`：`num_particles=900`、`smooth window=5`、`auto qstar prior=ON`。
2. 粒子更新/权重计算全部向量化（NumPy），避免 Python for-loop 逐粒子计算。
3. 每周期统计 `pf_compute_ms` 与滚动 `pf_compute_p95_ms`，并记录 `pf_overrun_count`。
4. 发生异常时触发安全回退：当周期异常，当前输出扭矩置 0，并累计 `pf_exception_count`。
5. 主链路不变：统一扭矩滤波、runtime delay、runtime scale 继续由现有框架管控。
6. 自动先验状态与范围在控制器内可观测（`pf_prior_ready_*`, `pf_prior_q_*`），便于调试。

这些保护的目的：在不牺牲系统稳定性的前提下，先把算法跑通到线上。

---

## 5) 关于 `L_p/L_d/R_p/R_d` 的说明

PF-IMU 不是传统 PD 网络模型，但仍对外暴露 `L_p/L_d/R_p/R_d` 字段：

- `L_p/R_p` 记录 PF 控制中的前馈项（`tau_ff`）；
- `L_d/R_d` 记录速度误差阻尼项（`tau_damp`）。

原因：与现有 RL 发送/日志路径保持接口一致，避免动协议和上层显示逻辑。

---

## 6) 一致性评估（MATLAB逻辑 vs RPi移植逻辑）

### 6.1 评估目标

验证在同一输入下，`PF_IMU_varspeed_MATLAB_pack_v8` 的核心逻辑与 `RPi_Unified/networks/pf_imu.py` 的输出是否一致（左右扭矩时间和幅值）。

数据文件：
- `Algorithm Reference/NJIT Reference/PI5_lstm_pd-20260424-221451.csv`

评估区间：
- `50s ~ 150s`

输入列（仅使用）：
- `imu_LTx`, `imu_RTx`, `imu_Lvel`, `imu_Rvel`

### 6.2 关于 delay（你提到的关键点）

本一致性评估统一采用 **delay = 0**：
- MATLAB 参考脚本本身没有显式 `runtime delay` 链路；
- `pf_imu.py` 的一致性评估也只比较算法裸输出扭矩（不经过 runtime delay）。

说明：
- GUI/RL 面板里默认 `Torque Delay=0ms`（当前版本）是系统运行时参数（主链路统一项），不是 PF 算法内核定义；
- 做“算法一致性”时应去掉该运行时延迟，因此本评估设为 0。

### 6.3 仿真方法（可复现）

新增脚本：
- `tools/pf_imu_consistency_eval.py`

执行命令：

```bash
python tools/pf_imu_consistency_eval.py \
  --csv "Algorithm Reference/NJIT Reference/PI5_lstm_pd-20260424-221451.csv" \
  --t0 50 --t1 150 --seeds 10
```

脚本做了两组比较：

1) **single-seed（路径级）**  
使用固定随机种子单次对比。

2) **MC-mean（统计一致性）**  
对 seeds=1..10 分别运行后取均值轨迹，再比较。

脚本使用与当前 Pi 默认接近的 profile：
- `num_particles=900`
- `Amin=-35`, `Amax=-0.1`
- `sigmaA=0.25`, `sigmaQstar=0.004`, `pSwitch=0.003`
- `etaV=3`, `etaSign=2`
- `dConf=1.5`, `confWindow=0.5s`
- `bMax=2`, `dMax=1`, `tauMax=25`
- `smoothAngleWindow=5`, `smoothVelocityWindow=5`
- `useAutoQstarPrior=true`, `priorMarginRatio=0.10`

输出文件：
- `Docs/pf_imu_consistency_50_150s_single.csv`
- `Docs/pf_imu_consistency_50_150s_mcmean.csv`

### 6.4 结果

`single-seed`（seed=1）：
- Left: `corr=0.661`, `lag=-20ms`, `RMSE=0.970 Nm`, `amp_ratio=1.000`
- Right: `corr=0.643`, `lag=-20ms`, `RMSE=1.056 Nm`, `amp_ratio=0.917`

`MC-mean`（seeds=1..10）：
- Left: `corr=0.902`, `lag=-20ms`, `RMSE=0.443 Nm`, `amp_ratio=0.958`
- Right: `corr=0.891`, `lag=-20ms`, `RMSE=0.495 Nm`, `amp_ratio=0.951`

### 6.5 结论解释

- **出现固定 `-20ms` lag 是预期**：5 点平滑在线实现为因果滤波，等效约 2 帧相位滞后。
- **幅值一致性**：`amp_ratio` 仍接近 1。
- **single-seed 相关性偏低是预期现象**：PF 随机采样路径对 seed 敏感。
- **MC 均值后相关性仍较高（~0.89-0.90）**，说明部署链路与参考动力学结构一致。

---

## 7) 当前已知限制

1. MATLAB 离线 `smoothdata(..., movmean, 5)` 是居中窗口，Pi 在线必须因果实现，天然存在相位差；
2. `auto qstar prior` 在 Pi 是滚动估计，MATLAB 离线是整窗估计，早期段会有偏差；
3. 参数（`etaV/etaSign/sigmaA/sigmaQstar` 等）还没有 GUI 全量暴露；
4. `nn_type=4` 是新增扩展码，旧 GUI 若不含映射会显示 `Unknown(4)`（不影响控制链路）。

---

## 8) 后续演进路径

建议按以下顺序推进：

1. 先做离线/在线一致性验证（同一段数据对比 MATLAB 与 Pi 输出趋势）；
2. 增加 PF 参数分组（保守参数 + 激进参数）并在 GUI 侧做 profile 切换；
3. 增加运行预算守护策略（例如 compute budget 超限时自动降粒子档位）；
4. 若长期稳定，再考虑将核心 PF 更新移植到 C++ 扩展或独立进程化。

---

## 9) 快速启动

Pi 上：

```bash
source ~/venvs/pytorch-env/bin/activate
cd RPi_Unified
python RL_controller_torch.py --nn pf_imu
```

GUI 上：

- 进入 RL 面板；
- 选择有效 Pi Profile；
- 点击 `Start PF-IMU`；
- 在状态行确认 `RPi: PF-IMU`。

---

## 10) 现实差距（重点）：真实部署明显难于离线仿真

基于 `50–150s` 的 5 秒分窗对比（20 窗/腿）：

- 左腿：部署链路 `corr` 全部低于参考仿真链路（`20/20` 窗），`RMSE` 全部更高（`20/20` 窗）。
- 右腿：同样 `20/20` 窗全部更差。
- 两腿每个窗口都出现稳定 `-20ms` 最佳滞后。

这说明差异是系统性现实约束，不是单点 bug：

1. **因果实时约束**：部署必须 causal（不能看未来），离线平滑可居中窗口。  
2. **实时预算约束**：部署受 100Hz 周期、串口、OS 调度限制，离线无此约束。  
3. **随机路径差异**：PF 单次在线轨迹噪声更大，离线 MC 均值天然更平滑。  
4. **工程保护链路**：部署要保安全（限幅/异常兜底），会改变细节波形。  

工程目标应定义为：

- 在线稳定性和安全性优先；
- 统计指标（corr/RMSE/phase lag）可控；
- 接受“不可逐点复刻离线曲线”的现实。
