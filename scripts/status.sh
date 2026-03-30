#!/bin/bash
# status.sh — 一键查看 Git 状态全貌
#
# 用法:
#   ./scripts/status.sh

set -e

REMOTE="origin"
MAIN_BRANCH="main"

echo "============================================"
echo "  Git 状态总览"
echo "============================================"

# ── 当前分支 ──────────────────────────────────────────────
CURRENT=$(git rev-parse --abbrev-ref HEAD)
echo ""
echo "当前分支: $CURRENT"

# ── 与远端对比 ────────────────────────────────────────────
echo ""
echo ">>> 与远端对比 (fetch) ..."
git fetch "$REMOTE" --quiet 2>/dev/null || echo "  [无法连接远端，跳过 fetch]"

AHEAD=$(git rev-list --count "$REMOTE/$CURRENT".."$CURRENT" 2>/dev/null || echo "?")
BEHIND=$(git rev-list --count "$CURRENT".."$REMOTE/$CURRENT" 2>/dev/null || echo "?")
echo "  本地领先远端: $AHEAD commit(s)"
echo "  本地落后远端: $BEHIND commit(s)"

# ── 文件改动 ──────────────────────────────────────────────
echo ""
echo ">>> 文件改动 (git status):"
git status --short

# ── 最近 5 条提交 ─────────────────────────────────────────
echo ""
echo ">>> 最近 5 条提交:"
git log --oneline -5

# ── 所有本地分支 ──────────────────────────────────────────
echo ""
echo ">>> 本地所有分支:"
git branch -v

echo ""
echo "============================================"
