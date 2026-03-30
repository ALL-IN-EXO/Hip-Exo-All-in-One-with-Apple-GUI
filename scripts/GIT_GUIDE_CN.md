# Git 协作扫盲指南

给从来没用过 branch / PR / merge 的队友看的。

---

## 0. 基本概念

```
你的电脑 (local)                    GitHub (remote/origin)
┌─────────────┐                    ┌─────────────┐
│  工作目录     │  ── git push ──>  │  远程仓库     │
│  (你改代码)   │  <── git pull ──  │  (大家共享)   │
└─────────────┘                    └─────────────┘
```

- **local**: 你电脑上的代码
- **origin**: GitHub 上的代码（`git remote -v` 可以查看）
- **branch**: 分支，就是一条独立的修改线路，互不干扰
- **main**: 主分支，始终保持可用的稳定版本
- **commit**: 一次保存点，记录了你改了什么
- **PR (Pull Request)**: 在 GitHub 上请求把你的 branch 合并到 main

---

## 1. 日常工作流程（每次改代码都走这个）

### 最快方式：用脚本

```bash
# 开新功能（自动拉 main → 建分支 → 提交 → 推送）
./scripts/new_feature.sh feature/add-emg-support "add: EMG data loading support"

# 在已有分支上继续提交
./scripts/push_current.sh "fix: BLE frame parsing off-by-one"

# PR merge 后清理
./scripts/cleanup_branch.sh
```

### 手动方式（理解原理用）

**Step 1: 确保本地 main 是最新的**
```bash
git checkout main
git pull origin main
```

**Step 2: 创建新分支**
```bash
# 命名规则: feature/功能名 或 fix/修复名
git checkout -b feature/add-emg-support
```

**Step 3: 改代码、写代码，然后提交**
```bash
git status                   # 看改了什么
git add 文件1 文件2           # 加入暂存区（推荐指定文件，而不是 git add .）
git commit -m "add: EMG data loading and plotting"
```

**Step 4: 推送到 GitHub**
```bash
git push -u origin feature/add-emg-support   # 第一次
git push                                       # 后续同一分支
```

**Step 5: 在 GitHub 上开 Pull Request**
1. 打开 https://github.com/ALL-IN-EXO/Hip-Exo-All-in-One-with-Apple-
2. 会看到黄色提示条 "Compare & pull request" — 点它
3. 填写标题和描述（改了什么、为什么改、测试了什么）
4. 点 "Create pull request"
5. 通知队友来 review

**Step 6: Review 通过后 Merge**
1. 队友看完代码，点 "Approve"
2. 点 "Squash and merge"（把所有 commit 压成一个）
3. 删除远程分支（GitHub 会提示，勾选即可）

**Step 7: 本地清理**
```bash
./scripts/cleanup_branch.sh
# 或手动:
git checkout main
git pull origin main
git branch -d feature/add-emg-support
```

---

## 2. 常用命令速查表

| 你想干什么 | 命令 |
|-----------|------|
| 看当前在哪个分支 | `git branch` |
| 看所有分支（含远程） | `git branch -a` |
| 切换到某个分支 | `git checkout 分支名` |
| 看改了哪些文件 | `git status` |
| 看具体改了什么内容 | `git diff` |
| 看提交历史 | `git log --oneline -10` |
| 拉取远程最新代码 | `git pull origin main` |
| 撤销还没 commit 的修改 | `git checkout -- 文件名` |
| 撤销已 add 但没 commit 的 | `git reset HEAD 文件名` |

---

## 3. 分支命名规范

| 前缀 | 用途 | 例子 |
|------|------|------|
| `feature/` | 新功能 | `feature/add-emg-tab` |
| `fix/` | 修 bug | `fix/ble-reconnect-crash` |
| `docs/` | 改文档 | `docs/update-protocol-spec` |
| `refactor/` | 重构代码 | `refactor/consolidate-motor-driver` |

---

## 4. Issues 怎么用

Issues 是 GitHub 上的"待办事项"系统。

### 什么时候开 Issue
- 发现了 bug："加载某个 CSV 会崩溃"
- 想要新功能："支持 EMG 数据"
- 有个想法要讨论："要不要把 LSTM 和 DNN 合并入口？"

### 怎么开
1. GitHub 仓库页面 → Issues → New Issue
2. 标题写清楚问题/需求
3. 描述里写细节、截图、复现步骤
4. 可以打标签（bug, enhancement, question）

### Issue + PR 联动
在 PR 描述里写 `Closes #3`，merge 后会自动关闭对应的 Issue #3。

---

## 5. 遇到冲突怎么办

当你和队友改了同一个文件的同一个地方，merge 时会冲突：

```
<<<<<<< HEAD
你的代码
=======
队友的代码
>>>>>>> main
```

解决方法：
1. 手动编辑文件，保留正确的代码，删掉 `<<<` `===` `>>>` 标记
2. `git add 冲突的文件`
3. `git commit -m "resolve merge conflict in xxx"`

**避免冲突的最佳方式**：经常 `git pull origin main` 保持同步，不要在一个分支上憋太久。

---

## 6. 完整示例：我要给 RPi 加一个新网络类型

```bash
# 1. 先同步
git checkout main
git pull origin main

# 2. 开分支
git checkout -b feature/add-transformer-network

# 3. 写代码... 在 RPi_Unified/networks/ 新建文件...
# 4. 同时更新 Docs/CHANGELOG.md

# 5. 提交（只 add 你改的文件）
git add RPi_Unified/networks/transformer_network.py
git add RPi_Unified/RL_controller_torch.py
git add Docs/CHANGELOG.md
git commit -m "add: Transformer network type for RPi RL controller"

# 6. 推送
git push -u origin feature/add-transformer-network

# 7. 去 GitHub 开 PR，等队友 review

# 8. merge 后清理
./scripts/cleanup_branch.sh
```

就这些。反复走几次这个流程就熟了。
