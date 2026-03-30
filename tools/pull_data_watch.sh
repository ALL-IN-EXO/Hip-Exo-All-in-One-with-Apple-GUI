#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOCAL_DIR="${ROOT_DIR}/RPi_Unified"
DATA_ROOT="${ROOT_DIR}/data_pi"

WATCH_SECONDS="2"
if [[ $# -gt 0 && "${1}" != -* ]]; then
  WATCH_SECONDS="${1}"
  shift
fi

exec python3 "${SCRIPT_DIR}/rpi_sync.py" \
  --local-dir "${LOCAL_DIR}" \
  --data-root "${DATA_ROOT}" \
  --direction pull \
  --watch "${WATCH_SECONDS}" \
  "$@"
