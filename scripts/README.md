# scripts/ — Git 工作流脚本

这里的脚本专门用于 **Git 协作工作流**，和 `tools/`（RPi 同步）完全分开。

## 脚本列表

| 脚本 | 功能 | 用法 |
|------|------|------|
| `new_feature.sh` | 拉取最新 main → 建分支 → 提交 → 推送（一条龙） | `./scripts/new_feature.sh feature/xxx "commit信息"` |
| `push_current.sh` | 提交当前分支所有改动并推送 | `./scripts/push_current.sh "commit信息"` |
| `cleanup_branch.sh` | PR merge 后清理（切 main、拉最新、删本地+远端分支） | `./scripts/cleanup_branch.sh [branch-name]` |
| `status.sh` | 一键查看 Git 状态全貌 | `./scripts/status.sh` |

## Git 协作教程（中文）

详见 [GIT_GUIDE_CN.md](GIT_GUIDE_CN.md)
