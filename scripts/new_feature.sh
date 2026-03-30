#!/bin/bash
# new_feature.sh — 一键：拉取最新 main → 建新分支 → 提交 → 推送
#
# 用法:
#   ./scripts/new_feature.sh <branch-name> "<commit message>"
#
# 示例:
#   ./scripts/new_feature.sh feature/add-emg-support "add: EMG data loading support"
#   ./scripts/new_feature.sh fix/ble-crash "fix: BLE reconnect crash on timeout"

set -e

REMOTE="origin"
MAIN_BRANCH="main"
REPO_URL="https://github.com/ALL-IN-EXO/Hip-Exo-All-in-One-with-Apple-GUI"

# ── 参数检查 ──────────────────────────────────────────────
if [ $# -lt 2 ]; then
  echo "用法: $0 <branch-name> \"<commit message>\""
  echo "示例: $0 feature/add-emg-support \"add: EMG data loading support\""
  exit 1
fi

BRANCH="$1"
MESSAGE="$2"

# ── 安全检查：不允许在 main 上直接操作 ────────────────────
CURRENT=$(git rev-parse --abbrev-ref HEAD)
if [ "$CURRENT" = "$MAIN_BRANCH" ]; then
  echo "[保护] 当前在 $MAIN_BRANCH 上，将创建新分支 $BRANCH"
fi

# ── 拉取最新 main ──────────────────────────────────────────
echo ""
echo ">>> 拉取最新 $MAIN_BRANCH ..."
git checkout "$MAIN_BRANCH"
git pull "$REMOTE" "$MAIN_BRANCH"

# ── 创建并切换到新分支 ─────────────────────────────────────
echo ""
echo ">>> 创建分支 $BRANCH ..."
git checkout -b "$BRANCH"

# ── 暂存所有改动并提交 ─────────────────────────────────────
echo ""
echo ">>> 暂存并提交 ..."
git add -A
git commit -m "$MESSAGE"

# ── 推送到远端 ────────────────────────────────────────────
echo ""
echo ">>> 推送 $BRANCH 到 $REMOTE ..."
git push -u "$REMOTE" "$BRANCH"

echo ""
echo "完成！接下来在 GitHub 上开 PR："
echo "  $REPO_URL/compare/$BRANCH"
