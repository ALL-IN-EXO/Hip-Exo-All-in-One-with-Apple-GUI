# scripts/ — Git 与打包脚本

这里的脚本用于两类任务：
- Git 协作工作流
- GUI 打包（macOS `.app` / Windows `.exe`）

和 `tools/`（RPi 同步）完全分开。

## 脚本列表

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

## Git 协作教程（中文）

详见 [GIT_GUIDE_CN.md](GIT_GUIDE_CN.md)
