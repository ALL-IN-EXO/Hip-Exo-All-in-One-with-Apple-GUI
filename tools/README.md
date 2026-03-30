# tools 使用说明

本目录用于本地和树莓派代码/数据同步。

## 脚本职责

- `deploy_code.sh`  
  本地执行。把本地 `RPi_Unified` 代码下发到树莓派  
  `aboutberlin@192.168.31.34:/home/aboutberlin/Desktop/RPi_Unified`

- `pull_data_watch.sh`  
  本地执行。把树莓派 `output/` 数据拉回本地  
  `/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/data_pi`

- `rpi_sync.py`  
  底层同步脚本（通常不直接调用）。

## 典型用法

在本机终端执行：

```bash
cd "/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI"
```

### 1) 手动下发代码（一次）

```bash
./tools/deploy_code.sh
```

### 2) 单次拉取数据（一次）

```bash
./tools/pull_data_watch.sh 0
```

### 3) 持续拉取数据（默认每 2 秒）

```bash
./tools/pull_data_watch.sh
```

自定义间隔（例如 1 秒）：

```bash
./tools/pull_data_watch.sh 1
```

## 数据保存位置

拉回的数据和记录都在：

`/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/data_pi`

目录结构：

- `data_pi/pull_history/<时间戳>/pull_report.json`  
  每次拉取的详细报告（拉了哪些文件）
- `data_pi/pull_history/<时间戳>/summary.txt`  
  每次拉取的文本摘要
- `data_pi/pull_history/<时间戳>/pulled_files.txt`  
  每次拉取的文件名列表（最直观）
- `data_pi/pull_history/_mirror_output/`  
  当前镜像的数据文件（CSV）
- `data_pi/pull_history/history.csv`  
  所有拉取历史总表（每次一行，最新在前；含 rsync 返回码）
- `data_pi/pull_history/LATEST`  
  最近一次拉取的时间戳
- `data_pi/latest_pull.txt`  
  最近一次拉取汇总

## 如何查看“这次到底拉了什么”

查看最近一次：

```bash
cat "/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/data_pi/latest_pull.txt"
```

更直接看最近一次拉了哪些文件：

```bash
latest=$(cat "/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/data_pi/pull_history/LATEST")
cat "/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/data_pi/pull_history/${latest}/pulled_files.txt"
```

看完整历史（每次一行）：

```bash
cat "/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI/data_pi/pull_history/history.csv"
```

## 常见说明

- `deploy_code.sh` 只推代码，不拉数据。  
- `pull_data_watch.sh` 只拉数据，不推代码。  
- 已配置 `sshpass` 密码登录（脚本内置）。  
- 如果你用 `Ctrl+Z` 暂停了拉取进程，可用 `fg` 恢复，或 `kill %1` 结束。
