#!/bin/bash
# cleanup_branch.sh — PR merge 后清理：切回 main → 拉取最新 → 删除本地+远程分支
#
# 用法:
#   ./scripts/cleanup_branch.sh [branch-name]
#
# 如果不传参数，默认清理当前所在分支。
# 示例:
#   ./scripts/cleanup_branch.sh feature/add-emg-support
#   ./scripts/cleanup_branch.sh   （当前在 feature/xxx 时直接运行）

set -e

REMOTE="origin"
MAIN_BRANCH="main"

# ── 确定要删除的分支 ──────────────────────────────────────
if [ $# -ge 1 ]; then
  BRANCH="$1"
else
  BRANCH=$(git rev-parse --abbrev-ref HEAD)
fi

# ── 安全检查：不允许删除 main ──────────────────────────────
if [ "$BRANCH" = "$MAIN_BRANCH" ]; then
  echo "[错误] 不能删除 $MAIN_BRANCH 分支。"
  exit 1
fi

echo "准备清理分支: $BRANCH"
echo ""

# ── 切回 main 并拉取最新 ──────────────────────────────────
echo ">>> 切换到 $MAIN_BRANCH 并拉取最新 ..."
git checkout "$MAIN_BRANCH"
git pull "$REMOTE" "$MAIN_BRANCH"

# ── 删除本地分支 ──────────────────────────────────────────
echo ""
echo ">>> 删除本地分支 $BRANCH ..."
git branch -d "$BRANCH" || {
  echo "[警告] 本地分支 $BRANCH 不存在或未完全 merge，跳过本地删除。"
}

# ── 删除远端分支 ──────────────────────────────────────────
echo ""
echo ">>> 删除远端分支 $REMOTE/$BRANCH ..."
git push "$REMOTE" --delete "$BRANCH" || {
  echo "[警告] 远端分支 $BRANCH 不存在或已删除，跳过。"
}

echo ""
echo "清理完成。当前在 $MAIN_BRANCH，本地和远端 $BRANCH 均已移除。"
