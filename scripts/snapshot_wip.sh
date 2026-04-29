#!/bin/bash
# snapshot_wip.sh — 将当前工作区改动“搬运”到新分支并推送远端备份
#
# 场景：
# - 你在本地 main（或任意分支）改了一堆，想先备份到远端
# - 不想 checkout main / pull / rebase（避免冲突）
#
# 用法：
#   ./scripts/snapshot_wip.sh <branch-name> "<commit message>" [--keep-main-dirty]
#
# 示例：
#   ./scripts/snapshot_wip.sh feature/jz-wip-2026-04-29 "WIP snapshot before cleanup"
#   ./scripts/snapshot_wip.sh feature/jz-wip-2026-04-29 "WIP snapshot before cleanup" --keep-main-dirty
#
# --keep-main-dirty:
#   备份完成后回到原分支，并恢复“同样的未提交改动”。
#   适合你想继续在本地 main 保持当前工作态继续改的场景。

set -euo pipefail

REMOTE="origin"

if [ $# -lt 2 ]; then
  echo "用法: $0 <branch-name> \"<commit message>\" [--keep-main-dirty]"
  echo "示例: $0 feature/jz-wip-2026-04-29 \"WIP snapshot before cleanup\" --keep-main-dirty"
  exit 1
fi

BRANCH="$1"
MESSAGE="$2"
KEEP_DIRTY="${3:-}"
CURRENT="$(git rev-parse --abbrev-ref HEAD)"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "错误：当前目录不是 git 仓库。"
  exit 1
fi

if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
  echo "错误：本地分支已存在：$BRANCH"
  echo "可先改名，或先删除：git branch -D $BRANCH"
  exit 1
fi

if git ls-remote --exit-code --heads "$REMOTE" "$BRANCH" >/dev/null 2>&1; then
  echo "错误：远端分支已存在：$REMOTE/$BRANCH"
  echo "请换一个分支名。"
  exit 1
fi

echo ""
echo ">>> 当前分支: $CURRENT"
echo ">>> 创建并切换到备份分支: $BRANCH"
git checkout -b "$BRANCH"

echo ""
echo ">>> 暂存所有改动"
git add -A

if git diff --cached --quiet; then
  echo "没有可提交改动（工作区干净或仅忽略文件）。"
  echo "当前仍在分支: $BRANCH"
  echo "可手动回主分支: git checkout $CURRENT"
  exit 0
fi

echo ""
echo ">>> 提交"
git commit -m "$MESSAGE"

echo ""
echo ">>> 推送到远端: $REMOTE/$BRANCH"
git push -u "$REMOTE" "$BRANCH"

echo ""
echo "完成：WIP 已备份到远端分支 $BRANCH"

if [ "$KEEP_DIRTY" = "--keep-main-dirty" ] || [ "$KEEP_DIRTY" = "--keep-dirty" ]; then
  echo ""
  echo ">>> --keep-main-dirty: 回到原分支并恢复同样未提交改动"
  git checkout "$CURRENT"
  # 把刚才这次快照提交临时拣回当前分支，再回退为未提交改动
  git cherry-pick "$BRANCH"
  git reset --mixed HEAD~1
  echo "已恢复未提交改动到分支: $CURRENT"
else
  echo "如需回到原分支继续开发："
  echo "  git checkout $CURRENT"
fi
