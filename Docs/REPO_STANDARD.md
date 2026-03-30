# Repo Creation Standard

Version: v0.1 (2026-03-20)
Status: Draft — 随项目实践持续迭代

本文档定义在 `Engineering For Lifelong Use` 下创建和维护任何新 repo 的规范。
适用于所有协作项目（含硬件、固件、数据分析、部署工具等）。

---

## 1. 项目结构

所有 repo 必须遵循以下骨架。按项目类型选择性启用（标 `*` 为必须）。

```
project-name/
├── README.md *              # 项目入口，一句话说清楚这是什么
├── CLAUDE.md *              # AI 助手指引（Claude Code 自动加载）
├── .gitignore *             # 排除规则
├── environment.yaml         # conda 环境（Python 项目）
│
├── src/ *                   # 所有源代码（或按子系统命名，如 RPi_Unified/、GUI_RL_update/ 等）
│   ├── utils.py             # 共享工具函数
│   └── pages/               # GUI 项目: 一个文件一个功能页
│
├── Docs/ *                  # 所有文档（大写 D 与已有项目保持一致）
│   ├── CHANGELOG.md *       # 版本记录（Keep a Changelog 格式）
│   ├── CONTRIBUTING.md *    # 协作规范（分支、PR、代码风格）
│   └── ...                  # 按需: SYSTEM_ARCHITECTURE, DATA_FORMAT, PLOTTING_GUIDE 等
│
├── scripts/                 # Git 工作流自动化脚本
│   ├── new_feature.sh       # 一键: 拉取 main → 建分支 → 提交 → 推送
│   ├── push_current.sh      # 一键: 提交当前分支 → 推送
│   ├── cleanup_branch.sh    # 一键: 切回 main → 拉取 → 删本地+远程分支
│   └── status.sh            # 一键: 显示当前分支/改动/远程状态
│
├── tools/                   # 设备同步工具（非 Git，如 RPi 代码下发 / 数据回拉）
│
├── data/ 或 data_pi/        # 数据目录（大文件 .gitignore 排除）
│   └── output/
│
└── tests/                   # 测试（如适用）
```

### 关键原则

- **不要把数据文件提交到 Git**（除极小 sample）。大文件、模型权重、日志用 `.gitignore` 排除
- **不要在根目录散落文档**。除 `README.md` 和 `CLAUDE.md` 外，所有文档放 `Docs/`
- **`tools/` 和 `scripts/` 职责分离**：`scripts/` 只做 Git 工作流；`tools/` 只做设备同步

---

## 2. 必备文件模板

### 2.1 README.md

```markdown
# 项目名

一句话描述。

**Current Version: vX.Y** (YYYY-MM-DD) | [Full Changelog](Docs/CHANGELOG.md)

## System Overview / Project Structure
（系统图或目录树，带注释）

## Quick Start
（安装 + 运行命令，每个子系统一节）

## Documentation
（链接到 Docs/ 下的各文件）
```

### 2.2 CLAUDE.md

```markdown
# CLAUDE.md - AI Assistant Guide

## Project Overview
一句话。主入口文件。

## Project Structure
（简洁目录树）

## Key Documentation
（表格: 什么场景读什么文件）

## Development Rules
1. Never push directly to main
2. One branch per task
3. 改代码后更新 CHANGELOG
4. ...（项目特定规则，特别是跨子系统协调规则）

## Common Commands
（运行、安装、部署命令）

## Code Conventions
（语言、命名、框架约定）
```

### 2.3 CHANGELOG.md

遵循 [Keep a Changelog](https://keepachangelog.com/) 格式：

```markdown
# Changelog

## [Unreleased]
（开发中的变更暂时写在这里）

## [vX.Y] - YYYY-MM-DD

### Added
### Fixed
### Changed
### Removed
```

### 2.4 .gitignore（Python + 嵌入式项目基准）

```gitignore
__pycache__/
*.pyc
*.pyo
.DS_Store
._*
*.egg-info/
dist/
build/
.env
*.log

# 模型权重（大文件）
*.pt
*.pth

# 运行时数据/日志（本地）
data_pi/
*/output/
```

---

## 3. Git 工作流

### 3.1 分支命名

| 类型 | 格式 | 示例 |
|------|------|------|
| 新功能 | `feature/<name>` | `feature/add-emg-support` |
| Bug 修复 | `fix/<name>` | `fix/ble-reconnect-crash` |
| 文档 | `docs/<name>` | `docs/update-protocol-spec` |
| 重构 | `refactor/<name>` | `refactor/consolidate-motor-driver` |

### 3.2 Commit Message 格式

```
<type>: <描述>

可选正文（解释 why，不解释 what）

Co-Authored-By: xxx <xxx@xxx.com>
```

type 取值：`add` / `fix` / `update` / `refactor` / `docs` / `remove`

### 3.3 完整流程

```
1. git checkout main && git pull
2. git checkout -b feature/xxx
3. 改代码 + 改文档（CHANGELOG 必须同步）
4. git add <具体文件> && git commit
5. git push -u origin feature/xxx
6. GitHub 上开 PR（写清 Summary + Test plan）
7. Review → Squash merge（勾选 "Delete branch"）
8. 本地跑 scripts/cleanup_branch.sh
```

### 3.4 PR 规范

```markdown
## Summary
- 做了什么（3-5 个要点）

## Changed files
| File | Change |
|------|--------|
| ... | ... |

## Test plan
- [ ] 测试项 1
- [ ] 测试项 2
```

### 3.5 分支清理（重要）

merge 后必须删除分支，否则会堆积：
- **远程**：PR merge 时勾选 "Delete branch"，或 `git push origin --delete <branch>`
- **本地**：`git branch -d <branch>`，或用 `scripts/cleanup_branch.sh`

---

## 4. 版本号规则

采用简化的语义版本：**vX.Y**

| 场景 | 版本变化 | 示例 |
|------|----------|------|
| 新功能 / 大改动 | Y + 1 | v3.0 → v3.1 |
| 重大架构变更 / 不兼容 | X + 1 | v3.x → v4.0 |
| Bug 修复 / 小调整 | 随下次功能版本一起发 | 不单独 bump |

版本号出现在：
- `README.md` 的 "Current Version" 行
- `Docs/CHANGELOG.md` 的版本标题

---

## 5. 文档更新 Checklist

每次 PR 前对照检查：

- [ ] `Docs/CHANGELOG.md` — 添加了本次变更记录
- [ ] `README.md` — 版本号、项目结构、Quick Start 是否需要更新
- [ ] `CLAUDE.md` — 项目结构、规则是否需要同步
- [ ] `Docs/CONTRIBUTING.md` — 代码组织、测试步骤是否需要更新
- [ ] 专项文档（SYSTEM_ARCHITECTURE / DATA_FORMAT / PLOTTING_GUIDE）— 如果改了相关内容

---

## 6. 脚本规范

### scripts/（Git 工作流）

所有 Git 自动化脚本放 `scripts/`，必须满足：

1. 开头有 `#!/bin/bash` 和 `set -e`（出错即停）
2. 有用法注释
3. 保护 `main` 分支（不允许直接在 main 上 commit/push）
4. `cleanup_branch.sh` 必须同时删除本地和远程分支

### tools/（设备同步）

设备同步脚本放 `tools/`，使用 `${BASH_SOURCE[0]}` 相对路径，不硬编码绝对路径。

---

## 7. 新 Repo 创建步骤

```bash
# 1. GitHub 上创建 repo（加 .gitignore 和 LICENSE）
# 2. clone 到本地
git clone git@github.com:ORG/project-name.git
cd project-name

# 3. 创建目录骨架
mkdir -p Docs scripts

# 4. 创建必备文件（按上面的模板）
# - README.md
# - CLAUDE.md
# - .gitignore
# - Docs/CHANGELOG.md
# - Docs/CONTRIBUTING.md

# 5. 复制标准脚本
cp /path/to/template/scripts/*.sh scripts/
chmod +x scripts/*.sh

# 6. 初始提交
git add README.md CLAUDE.md .gitignore Docs/ scripts/
git commit -m "add: initial project structure"
git push

# 7. GitHub 上设置 branch protection（可选但推荐）
# Settings → Branches → Add rule → main → Require PR
```

---

## 版本历史（本文档）

| 版本 | 日期 | 变更 |
|------|------|------|
| v0.1 | 2026-03-20 | 初版。基于 Exo-Data-Analysis-GUI 实践提炼，Hip-Exo-All-in-One 补充多子系统规范 |

---

## 待改进（下次迭代）

- [ ] GitHub branch protection 自动化配置
- [ ] PR template 文件（`.github/pull_request_template.md`）
- [ ] CI/CD 基础模板（GitHub Actions: lint + build check）
- [ ] 硬件/固件项目特定测试流程
- [ ] 多人 code review 流程细化
