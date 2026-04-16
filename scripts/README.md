# scripts/ — Git 与打包脚本

这里的脚本用于两类任务：
- Git 协作工作流
- GUI 打包（macOS `.app` / Windows `.exe`）

和 `tools/`（RPi 同步）完全分开。
# scripts/ — 辅助脚本

这里的脚本分两类，和 `tools/`（RPi 同步）完全分开：

1. **Git 协作工作流脚本**（bash，日常开发用）
2. **PyInstaller 打包脚本**（Windows PowerShell + macOS bash，给非开发者分发 GUI 用）

---

## 一、Git 工作流脚本

| 脚本 | 功能 | 用法 |
|------|------|------|
| `new_feature.sh` | 拉取最新 main → 建分支 → 提交 → 推送（一条龙） | `./scripts/new_feature.sh feature/xxx "commit信息"` |
| `push_current.sh` | 提交当前分支所有改动并推送 | `./scripts/push_current.sh "commit信息"` |
| `cleanup_branch.sh` | PR merge 后清理（切 main、拉最新、删本地+远端分支） | `./scripts/cleanup_branch.sh [branch-name]` |
| `status.sh` | 一键查看 Git 状态全貌 | `./scripts/status.sh` |
| `build_mac.sh` | 打包 Hip GUI 为 macOS `.app` | `./scripts/build_mac.sh` |
| `build_win.ps1` | 打包 Hip GUI 为 Windows `.exe` | `powershell -ExecutionPolicy Bypass -File .\\scripts\\build_win.ps1` |

## 打包脚本默认行为（已默认开启）

- 默认启用自定义图标（来源：`scripts/assets/app_icon.jpeg`）
- 默认启用版本号标注（从 GUI 入口文件中的 `vX.Y` 自动提取）
- 输出 zip 文件名包含版本号（例如 `..._v3.3.0_...zip`）

可选覆盖参数：
- mac: `APP_VERSION=3.3.1 ICON_SOURCE=/path/icon.jpeg ICON_ENABLED=0 VERSION_ENABLED=0 ./scripts/build_mac.sh`
- windows: `-AppVersion 3.3.1 -DisableIcon -DisableVersion`

详细 Git 协作教程见 [GIT_GUIDE_CN.md](GIT_GUIDE_CN.md)。

---

## 二、PyInstaller 打包脚本

把 `GUI_RL_update/GUI.py` 打成可分发的可执行文件，**无需目标机器安装 Python**。

| 脚本 | 平台 | 产物 |
|------|------|------|
| `build_win.ps1` | Windows 10/11 | `.exe` 目录 + `.zip` |
| `build_mac.sh` | macOS (Intel / Apple Silicon) | `.app` bundle + `.zip` |

### 2.1 依赖

两个脚本首次运行时会自动创建独立 venv 并安装以下包（不污染系统 Python）：

```
pyinstaller pyqt5 pyqtgraph pyserial numpy
```

Windows venv 位置：`GUI_RL_update/.venv-build-win/`
macOS venv 位置：`GUI_RL_update/.venv-build-mac/`

这两个 venv 目录已加入 `.gitignore`，可复用（下次加 `-SkipDepInstall` / `--skip-deps` 跳过 pip 安装）。

### 2.2 Windows 打包（build_win.ps1）

在**仓库根目录**下执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\build_win.ps1
```

常用选项：

```powershell
# 指定 Python 解释器（py 启动器找不到时）
powershell -ExecutionPolicy Bypass -File .\scripts\build_win.ps1 -PythonExe "C:\Python311\python.exe"

# FULL 模式：用 --collect-all，更稳但体积更大，调试首次打包时建议开
powershell -ExecutionPolicy Bypass -File .\scripts\build_win.ps1 -FullBuild

# 复用已有 venv，跳过 pip install（省几分钟）
powershell -ExecutionPolicy Bypass -File .\scripts\build_win.ps1 -SkipDepInstall

# 自定义应用名
powershell -ExecutionPolicy Bypass -File .\scripts\build_win.ps1 -AppName "MyHipExoGUI"
```

完整参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-PythonExe` | `py` | 用于创建 venv 的 Python 解释器 |
| `-AppName` | `HipExoRLGUI` | 生成的 exe 名字 |
| `-Entry` | `GUI.py` | 入口文件（相对 `SourceDir`） |
| `-SourceDir` | `GUI_RL_update` | 源代码目录（相对仓库根） |
| `-SkipDepInstall` | （关） | 跳过 pip install |
| `-FullBuild` | （关） | 启用 `--collect-all PyQt5 pyqtgraph` |

### 2.3 macOS 打包（build_mac.sh）

```bash
chmod +x ./scripts/build_mac.sh   # 只需第一次
./scripts/build_mac.sh
```

常用选项：

```bash
# 指定 Python 解释器
./scripts/build_mac.sh --python python3.11

# FULL 模式
./scripts/build_mac.sh --full

# 复用已有 venv
./scripts/build_mac.sh --skip-deps

# 自定义应用名
./scripts/build_mac.sh --name MyHipExoGUI
```

完整参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--python <path>` | `python3` | Python 解释器 |
| `--name <AppName>` | `HipExoRLGUI` | 生成的 .app 名字 |
| `--entry <file>` | `GUI.py` | 入口文件 |
| `--source <dir>` | `GUI_RL_update` | 源代码目录 |
| `--skip-deps` | （关） | 跳过 pip install |
| `--full` | （关） | 启用 `--collect-all PyQt5 pyqtgraph` |

### 2.4 产物位置与分发

每次打包都会在以下路径生成**带时间戳 + git 短哈希**的目录（支持历史版本回溯）：

```
GUI_RL_update/release/windows/<时间戳>_<gitsha>/HipExoRLGUI/           ← 可执行目录
GUI_RL_update/release/windows/<时间戳>_<gitsha>/HipExoRLGUI_win_....zip ← 用于分发

GUI_RL_update/release/macos/<时间戳>_<gitsha>/HipExoRLGUI.app           ← .app bundle
GUI_RL_update/release/macos/<时间戳>_<gitsha>/HipExoRLGUI_mac_....zip   ← 用于分发
```

**发给别人：**只发 `.zip` 文件。

**Windows 用户收到 zip 后：**

1. 右键 → **全部解压**（不要在压缩包里双击 exe，会缺依赖）
2. 进入 `HipExoRLGUI/` 文件夹，双击 `HipExoRLGUI.exe`
3. 若被 SmartScreen 拦截："更多信息" → "仍要运行"（未签名 exe 的通病）

**macOS 用户收到 zip 后：**

1. 双击 zip 解压出 `HipExoRLGUI.app`
2. 首次打开：**右键 → 打开**（绕过 Gatekeeper），不要双击
3. 或终端执行：`xattr -dr com.apple.quarantine HipExoRLGUI.app`

### 2.5 SLIM vs FULL 模式

| 模式 | 触发方式 | 体积 | 稳定性 | 何时用 |
|------|---------|------|--------|--------|
| SLIM（默认） | 不加参数 | 较小 | 已针对 pyqtgraph 调优；保留 QtSvg/QtPrintSupport/QtOpenGL/QtSerialPort | 验证通过后的生产打包 |
| FULL | `-FullBuild` / `--full` | 较大 | 用 `--collect-all PyQt5 pyqtgraph` 全量打入 | 第一次打包调试、或 SLIM 出问题时 |

### 2.6 常见问题排查

1. **PowerShell "禁止运行脚本"** → 用 `powershell -ExecutionPolicy Bypass -File .\scripts\build_win.ps1`
2. **`py` / `python3` 不是内部命令** → 用 `-PythonExe "C:\Python311\python.exe"` 或 `--python python3.11`
3. **exe 双击闪退** → 先去掉 `--windowed` 在命令行手动跑一次看 traceback，或查看 `GUI_RL_update/build/HipExoRLGUI/warn-HipExoRLGUI.txt`
4. **.app 打不开 "已损坏"** → 打包产物未签名，Gatekeeper 会隔离。执行 `xattr -dr com.apple.quarantine <App>.app` 即可
5. **zip 里的 exe 跑不动** → 必须先解压到磁盘再运行，不能在 zip 内部直接双击
6. **版本累积占硬盘** → `release/` 每次打包都会新增时间戳目录；旧版本手动删即可（zip 就是完整产物，删文件夹只留 zip 也没问题）

### 2.7 清理打包产物

打完包后可以安全删除以下目录来释放磁盘（venv 建议保留）：

```bash
# macOS / Linux
rm -rf GUI_RL_update/build GUI_RL_update/dist GUI_RL_update/*.spec

# Windows PowerShell
Remove-Item -Recurse -Force GUI_RL_update\build, GUI_RL_update\dist
Remove-Item -Force GUI_RL_update\*.spec
```

保留 `release/` 里的 `.zip` 就够了，里面已经包含完整产物。
