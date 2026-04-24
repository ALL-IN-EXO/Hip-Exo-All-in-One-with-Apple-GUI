# 新树莓派首次配置与注册（Pi4B / Pi5）

适用仓库：Hip Exo Controller All in one with Apple GUI  
目标：从 0 到可用，完成新树莓派基础环境、SSH、串口、同步部署、GUI 注册。

---

## 0) 先说结论：IP 什么时候必须有？

- 仅在 Pi 本机装环境时：不必须先拿 IP。
- 但以下操作前必须有 `host`（IP 或主机名）：
  - `./tools/deploy_code.sh`
  - `./tools/pull_data_watch.sh`
  - GUI `Configure Pi...` + `Start LegDcp / Start LSTM-PD`

建议：基础环境完成后立刻记录 IP。

---

## 1) 基础依赖（Pi 上执行）

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip tmux rsync
```

说明：
- `tmux`：GUI 远程启动 RL 必需
- `rsync`：本机同步脚本依赖

---

## 2) 串口配置（通用）

项目默认串口：`SER_DEV='/dev/ttyAMA0'`（见 `RPi_Unified/RL_controller_torch.py`）。

先执行：

```bash
sudo raspi-config
```

进入 `Interface Options -> Serial Port`：
- `Login shell over serial?` -> `No`
- `Serial hardware enabled?` -> `Yes`

重启：

```bash
sudo reboot
```

---

## 2.1) Pi4B / Pi5 差异（精简版）

先确认机型：

```bash
tr -d '\0' < /proc/device-tree/model; echo
```

### Pi4B（目标：GPIO14/15 使用 `/dev/ttyAMA0`）

若 `ttyAMA0` 不存在，执行：

```bash
sudo bash -c 'grep -q "^enable_uart=1$" /boot/firmware/config.txt || echo "enable_uart=1" >> /boot/firmware/config.txt'
sudo bash -c 'grep -q "^dtoverlay=disable-bt$" /boot/firmware/config.txt || echo "dtoverlay=disable-bt" >> /boot/firmware/config.txt'
sudo systemctl disable hciuart || true
sudo systemctl stop hciuart || true
sudo reboot
```

重启后检查：

```bash
ls -l /dev/ttyAMA0 /dev/serial0 /dev/serial1
```

期望：`/dev/ttyAMA0` 存在，`/dev/serial0` 通常指向它。  
补充：部分 Pi4B 没有 `/dev/serial1`，可忽略。

### Pi5（目标：确认 40-pin GPIO14/15 对应设备）

检查：

```bash
ls -l /dev/ttyAMA0 /dev/serial0 /dev/serial1 /dev/ttyAMA10
```

按本项目约定：
- 若走 40-pin 8/10 脚（GPIO14/15），优先用 `/dev/ttyAMA0`
- `serial0` 若指向 `ttyAMA10`（调试口），不要在本项目里用 `serial0`

---

## 3) Python 运行环境（Pi 上执行）

标准库（`os/struct/time/csv/datetime/math/argparse/collections`）无需安装。  
项目必需三方包：
- `numpy`
- `pyserial`
- `scipy`（`filter_library.py` 用到 `scipy.signal`）
- `torch`

安装：

```bash
python3 -m venv ~/venvs/pytorch-env
source ~/venvs/pytorch-env/bin/activate
python -m pip install --upgrade pip setuptools wheel
pip install numpy pyserial scipy torch
```

快速自检：

```bash
source ~/venvs/pytorch-env/bin/activate
python - <<'PY'
import torch, serial, numpy, scipy
print("torch:", torch.__version__)
print("pyserial:", serial.__version__)
print("numpy:", numpy.__version__)
print("scipy:", scipy.__version__)
PY
tmux -V
ls -l /dev/ttyAMA0
```

若报 `ModuleNotFoundError`（例如 `numpy`/`scipy`）：

```bash
source ~/venvs/pytorch-env/bin/activate
python -m pip install --upgrade pip setuptools wheel
python -m pip install numpy pyserial scipy
python -m pip install torch
```

---

## 4) SSH 先打通（Pi 上执行）

```bash
sudo systemctl enable ssh
sudo systemctl start ssh
sudo systemctl status ssh --no-pager
whoami
hostname -I
```

核对：
- `status` 显示 `active (running)`
- 记下 `whoami`（用户名）
- 记下 `hostname -I`（IP）

示例：
- `whoami = klangkarussell`
- `host = 192.168.31.218`

---

## 5) 获取并记录连接信息（Pi 上可补充检查）

```bash
hostname -I
ip -4 addr show
```

后续要填的四项：
- `host`
- `user`
- `password`（若用密码）
- `remote_dir`

---

## 5.1) macOS 免密 SSH + 别名（推荐）

### A. 在 Mac 生成 key（已有可跳过）

```bash
ls -l ~/.ssh/id_ed25519 ~/.ssh/id_ed25519.pub
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -C "macbook-hipexo"
```

### B. 下发公钥到 Pi

```bash
cat ~/.ssh/id_ed25519.pub | ssh klangkarussell@192.168.31.218 \
"mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
```

### C. 配置 SSH 别名（Mac）

编辑 `~/.ssh/config`：

```sshconfig
Host pi4b-hipexo
  HostName 192.168.31.218
  User klangkarussell
  IdentityFile ~/.ssh/id_ed25519
  IdentitiesOnly yes
  ServerAliveInterval 30
```

权限：

```bash
chmod 700 ~/.ssh
chmod 600 ~/.ssh/config
```

测试：

```bash
ssh pi4b-hipexo
```

---

## 6) 准备 Pi 上代码目录（Pi 上执行）

```bash
mkdir -p ~/Desktop/RPi_Unified/output
```

若用户名不是 `pi`，目录应是：
- `/home/<你的用户名>/Desktop/RPi_Unified`

---

## 7) Mac 首次同步部署

在项目根目录：

```bash
cd "/Volumes/X10 Pro/Engineering For Lifelong Use/Hip Exo Controller All in one with Apple GUI"
cp -n tools/rpi_profiles.conf.example tools/rpi_profiles.conf
```

编辑 `tools/rpi_profiles.conf` 示例（直接 IP + 密码）：

```ini
[active]
profile = pi4b-218

[pi4b-218]
host       = 192.168.31.218
user       = klangkarussell
remote_dir = /home/klangkarussell/Desktop/RPi_Unified
password   = your_password
```

若用 SSH key，可留空密码：

```ini
[active]
profile = klangkarussell

[klangkarussell]
host       = klangkarussell
user       = klangkarussell
remote_dir = /home/klangkarussell/Desktop/RPi_Unified
password   =
```

首次验证：

```bash
./tools/deploy_code.sh
./tools/pull_data_watch.sh 0
```

---

## 8) Pi 本机手动试跑 RL

```bash
source ~/venvs/pytorch-env/bin/activate
cd ~/Desktop/RPi_Unified
python RL_controller_torch.py --nn lstm_leg_dcp
# 或
python RL_controller_torch.py --nn lstm_pd
```

---

## 9) GUI 注册 Pi Profile

GUI profile 存放于用户目录（不是仓库内 `tools/rpi_profiles.conf`）：
- macOS: `~/Library/Application Support/HipExoController/rpi_profiles.conf`

GUI 操作：
- RL 面板 -> `Configure Pi...`
- 新建或编辑 profile
- 填写 `host/user/port/remote_dir` 和认证
- 设为 active

提示：
- `Remote Dir` 现在支持 `~/...`（会在远端展开为 `$HOME`）

---

## 常见故障

- `WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!`
  - Pi 本机先核对：
    ```bash
    ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
    ```
  - Mac 清理旧 host key 并重连：
    ```bash
    ssh-keygen -R 192.168.31.218
    ssh -o StrictHostKeyChecking=accept-new klangkarussell@192.168.31.218
    ```

- `Pi RL Remote: __ERR__:remote_dir missing`
  - 检查 GUI `Remote Dir` 是否正确（建议绝对路径）
  - 若用 `~/...`，当前 GUI 已支持自动展开

- `Pi RL Remote: tmux not found`
  - Pi 上执行：`sudo apt install -y tmux`

- `venv activate missing`
  - Pi 上确认 `~/venvs/pytorch-env/bin/activate` 存在

- `ModuleNotFoundError: No module named scipy`（或 `numpy`）
  - 重新安装：
    ```bash
    source ~/venvs/pytorch-env/bin/activate
    python -m pip install numpy pyserial scipy torch
    ```

- `sshpass not found`（Mac 且使用密码登录）
  - 安装：`brew install hudochenkov/sshpass/sshpass`
  - 或改用 SSH key 登录
