#!/bin/bash
# push_current.sh — 一键：提交当前分支所有改动并推送
#
# 用法:
#   ./scripts/push_current.sh "<commit message>"
#
# 示例:
#   ./scripts/push_current.sh "update: increase Serial8 baud rate to 230400"

set -e

REMOTE="origin"
MAIN_BRANCH="main"

# ── 参数检查 ──────────────────────────────────────────────
if [ $# -lt 1 ]; then
  echo "用法: $0 \"<commit message>\""
  echo "示例: $0 \"fix: GUI hang when Teensy disconnects\""
  exit 1
fi

MESSAGE="$1"

# ── 安全检查：不允许在 main 上直接 commit/push ─────────────
CURRENT=$(git rev-parse --abbrev-ref HEAD)
if [ "$CURRENT" = "$MAIN_BRANCH" ]; then
  echo "[错误] 当前在 $MAIN_BRANCH 分支上，不允许直接推送。"
  echo "请先创建 feature/fix 分支: git checkout -b feature/xxx"
  exit 1
fi

echo "当前分支: $CURRENT"

# ── 暂存所有改动并提交 ─────────────────────────────────────
echo ""
echo ">>> 暂存并提交 ..."
git add -A
git commit -m "$MESSAGE"

# ── 推送 ─────────────────────────────────────────────────
echo ""
echo ">>> 推送 $CURRENT 到 $REMOTE ..."
git push -u "$REMOTE" "$CURRENT"

echo ""
echo "推送完成。"
