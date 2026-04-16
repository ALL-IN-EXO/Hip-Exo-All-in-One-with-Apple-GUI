#!/usr/bin/env bash
set -euo pipefail

# Build macOS .app for:
#   GUI_RL_update/GUI.py
#
# Usage:
#   ./scripts/build_mac.sh
#
# Optional env vars:
#   PYTHON_BIN=/usr/local/bin/python3
#   APP_NAME=HipExoControllerGUI
#   APP_VERSION=3.3.0
#   ICON_SOURCE=/path/to/icon.jpeg
#   ICON_ENABLED=1      # default ON; set 0 to disable
#   VERSION_ENABLED=1   # default ON; set 0 to disable plist version stamping
#   SKIP_DEP_INSTALL=1
#   FULL_BUILD=1
#   VENV_DIR=/custom/path/.venv-build
#   WORK_ROOT=/custom/local/build-root

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
APP_DIR="${REPO_ROOT}/GUI_RL_update"
ENTRY="${APP_DIR}/GUI.py"

if [[ ! -f "${ENTRY}" ]]; then
  echo "Entry not found: ${ENTRY}"
  exit 1
fi

APP_NAME="${APP_NAME:-HipExoControllerGUI}"
ICON_ENABLED="${ICON_ENABLED:-1}"
VERSION_ENABLED="${VERSION_ENABLED:-1}"
ICON_SOURCE="${ICON_SOURCE:-${REPO_ROOT}/scripts/assets/app_icon.jpeg}"
if [[ ! -f "${ICON_SOURCE}" && -f "/Users/junchengzhou/Downloads/images.jpeg" ]]; then
  ICON_SOURCE="/Users/junchengzhou/Downloads/images.jpeg"
fi

if [[ -n "${PYTHON_BIN:-}" ]]; then
  PY="${PYTHON_BIN}"
elif [[ -x "/usr/local/bin/python3" ]]; then
  PY="/usr/local/bin/python3"
else
  PY="$(command -v python3 || true)"
fi

if [[ -z "${PY}" ]]; then
  echo "python3 not found. Install Python first."
  exit 1
fi

extract_app_version() {
  local entry_file="$1"
  local v
  v="$(grep -Eo 'v[0-9]+([.][0-9]+)+' "${entry_file}" | head -n1 | sed 's/^v//' || true)"
  if [[ -n "${v}" ]]; then
    echo "${v}"
  else
    echo "1.0.0"
  fi
}

APP_VERSION="${APP_VERSION:-$(extract_app_version "${ENTRY}")}"

# Keep build env outside external drives to avoid AppleDouble metadata issues.
VENV_DIR="${VENV_DIR:-${HOME}/.exo_gui_build/hip/venv_mac_py39}"
WORK_ROOT="${WORK_ROOT:-${HOME}/.exo_gui_build/hip/work_mac}"
DIST_DIR="${WORK_ROOT}/dist"
BUILD_DIR="${WORK_ROOT}/build"
SPEC_DIR="${WORK_ROOT}/spec"
ICON_ICNS="${WORK_ROOT}/app_icon.icns"

GIT_SHA="$(git -C "${REPO_ROOT}" rev-parse --short HEAD 2>/dev/null || echo 'nogit')"
TS="$(date '+%Y%m%d_%H%M%S')"
BUILD_TAG="${TS}_${GIT_SHA}"
RELEASE_DIR="${APP_DIR}/release/mac/${BUILD_TAG}"

echo "==> Repo: ${REPO_ROOT}"
echo "==> App dir: ${APP_DIR}"
echo "==> Python: ${PY}"
echo "==> App version: ${APP_VERSION}"
echo "==> Build venv: ${VENV_DIR}"
echo "==> Work root: ${WORK_ROOT}"
echo "==> Build tag: ${BUILD_TAG}"
echo "==> Icon enabled: ${ICON_ENABLED}"

if [[ ! -d "${VENV_DIR}" ]]; then
  echo "==> Creating build venv: ${VENV_DIR}"
  "${PY}" -m venv "${VENV_DIR}"
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

cleanup_appledouble() {
  find "${VENV_DIR}" -type f -name '._*' -delete 2>/dev/null || true
}

cleanup_appledouble_tree() {
  local target="$1"
  if [[ -d "${target}" || -f "${target}" ]]; then
    find "${target}" -type f -name '._*' -delete 2>/dev/null || true
  fi
}

set_plist_value() {
  local plist_file="$1"
  local key="$2"
  local type="$3"
  local value="$4"
  /usr/libexec/PlistBuddy -c "Set :${key} ${value}" "${plist_file}" >/dev/null 2>&1 \
    || /usr/libexec/PlistBuddy -c "Add :${key} ${type} ${value}" "${plist_file}" >/dev/null 2>&1 \
    || true
}

prepare_icon_icns() {
  local src="$1"
  local out="$2"
  local iconset_dir="${WORK_ROOT}/app_icon.iconset"

  rm -rf "${iconset_dir}" "${out}"
  mkdir -p "${iconset_dir}"

  python - <<PY
from PIL import Image

src = r"""${src}"""
iconset_dir = r"""${iconset_dir}"""
img = Image.open(src).convert("RGBA")
targets = [
    ("icon_16x16.png", 16),
    ("icon_16x16@2x.png", 32),
    ("icon_32x32.png", 32),
    ("icon_32x32@2x.png", 64),
    ("icon_128x128.png", 128),
    ("icon_128x128@2x.png", 256),
    ("icon_256x256.png", 256),
    ("icon_256x256@2x.png", 512),
    ("icon_512x512.png", 512),
    ("icon_512x512@2x.png", 1024),
]
for name, size in targets:
    out = img.resize((size, size), Image.LANCZOS)
    out.save(f"{iconset_dir}/{name}", format="PNG")
PY

  iconutil -c icns "${iconset_dir}" -o "${out}"
}

export COPYFILE_DISABLE=1
cleanup_appledouble

if [[ "${SKIP_DEP_INSTALL:-0}" != "1" ]]; then
  echo "==> Installing build/runtime dependencies into venv"
  cleanup_appledouble
  python -m pip install --upgrade pip setuptools wheel
  cleanup_appledouble
  python -m pip install --upgrade pyinstaller pyqt5 pyqtgraph pyserial numpy pillow
  cleanup_appledouble
fi

echo "==> Cleaning previous build folders"
mkdir -p "${WORK_ROOT}"
rm -rf "${DIST_DIR}" "${BUILD_DIR}" "${SPEC_DIR}"
mkdir -p "${SPEC_DIR}"

ICON_FLAG_ARGS=()
if [[ "${ICON_ENABLED}" != "0" ]]; then
  if [[ -f "${ICON_SOURCE}" ]]; then
    echo "==> Preparing macOS icon (.icns) from: ${ICON_SOURCE}"
    if prepare_icon_icns "${ICON_SOURCE}" "${ICON_ICNS}"; then
      ICON_FLAG_ARGS+=(--icon "${ICON_ICNS}")
    else
      echo "[warn] Failed to prepare .icns icon, continue without custom icon"
    fi
  else
    echo "[warn] ICON_SOURCE not found: ${ICON_SOURCE} (continue without custom icon)"
  fi
fi

FULL_BUILD="${FULL_BUILD:-0}"
export QT_API=pyqt5

PYI_ARGS=(
  --noconfirm
  --clean
  --windowed
  --name "${APP_NAME}"
  --distpath "${DIST_DIR}"
  --workpath "${BUILD_DIR}"
  --specpath "${SPEC_DIR}"
)
if [[ ${#ICON_FLAG_ARGS[@]} -gt 0 ]]; then
  PYI_ARGS+=("${ICON_FLAG_ARGS[@]}")
fi

if [[ "${FULL_BUILD}" == "1" ]]; then
  echo "==> Build mode: FULL (collect-all)"
  PYI_ARGS+=(--collect-all PyQt5 --collect-all pyqtgraph)
else
  echo "==> Build mode: SLIM (default)"
  PYI_ARGS+=(
    --hidden-import serial.tools.list_ports
    --hidden-import pyqtgraph
    --exclude-module tkinter
    --exclude-module _tkinter
    --exclude-module PyQt5.Qt3DAnimation
    --exclude-module PyQt5.Qt3DCore
    --exclude-module PyQt5.Qt3DExtras
    --exclude-module PyQt5.Qt3DInput
    --exclude-module PyQt5.Qt3DLogic
    --exclude-module PyQt5.Qt3DRender
    --exclude-module PyQt5.QtBluetooth
    --exclude-module PyQt5.QtChart
    --exclude-module PyQt5.QtDataVisualization
    --exclude-module PyQt5.QtDesigner
    --exclude-module PyQt5.QtHelp
    --exclude-module PyQt5.QtLocation
    --exclude-module PyQt5.QtMultimedia
    --exclude-module PyQt5.QtMultimediaWidgets
    --exclude-module PyQt5.QtNetworkAuth
    --exclude-module PyQt5.QtNfc
    --exclude-module PyQt5.QtOpenGL
    --exclude-module PyQt5.QtPositioning
    --exclude-module PyQt5.QtPrintSupport
    --exclude-module PyQt5.QtQml
    --exclude-module PyQt5.QtQuick
    --exclude-module PyQt5.QtQuick3D
    --exclude-module PyQt5.QtQuickWidgets
    --exclude-module PyQt5.QtRemoteObjects
    --exclude-module PyQt5.QtSensors
    --exclude-module PyQt5.QtSerialPort
    --exclude-module PyQt5.QtSql
    --exclude-module PyQt5.QtSvg
    --exclude-module PyQt5.QtTest
    --exclude-module PyQt5.QtTextToSpeech
    --exclude-module PyQt5.QtWebChannel
    --exclude-module PyQt5.QtWebSockets
    --exclude-module PyQt5.QtXml
    --exclude-module PyQt5.QtXmlPatterns
    --exclude-module PyQt5.QtWebEngine
    --exclude-module PyQt5.QtWebEngineCore
    --exclude-module PyQt5.QtWebEngineWidgets
  )
fi

echo "==> Running PyInstaller"
python -m PyInstaller "${PYI_ARGS[@]}" "${ENTRY}"

APP_BUNDLE="${DIST_DIR}/${APP_NAME}.app"
if [[ ! -d "${APP_BUNDLE}" ]]; then
  echo "Build failed: app bundle not found at ${APP_BUNDLE}"
  exit 1
fi

if [[ "${VERSION_ENABLED}" != "0" ]]; then
  INFO_PLIST="${APP_BUNDLE}/Contents/Info.plist"
  if [[ -f "${INFO_PLIST}" ]]; then
    echo "==> Stamping app version in Info.plist"
    set_plist_value "${INFO_PLIST}" "CFBundleShortVersionString" "string" "${APP_VERSION}"
    set_plist_value "${INFO_PLIST}" "CFBundleVersion" "string" "${BUILD_TAG}"
  fi
fi

echo "==> Cleaning AppleDouble metadata in app bundle"
cleanup_appledouble_tree "${APP_BUNDLE}"

echo "==> Ad-hoc signing app bundle (best-effort)"
codesign --force --deep --sign - "${APP_BUNDLE}" >/dev/null 2>&1 || true

mkdir -p "${RELEASE_DIR}"
echo "==> Copying app bundle to release folder"
rsync -a "${APP_BUNDLE}" "${RELEASE_DIR}/"
cleanup_appledouble_tree "${RELEASE_DIR}/${APP_NAME}.app"

ZIP_NAME="${APP_NAME}_v${APP_VERSION}_mac_${BUILD_TAG}.zip"
echo "==> Creating zip: ${ZIP_NAME}"
(
  cd "${RELEASE_DIR}"
  ditto -c -k --keepParent "${APP_NAME}.app" "${ZIP_NAME}"
)

echo
echo "Build complete."
echo "App bundle: ${RELEASE_DIR}/${APP_NAME}.app"
echo "Zip file:   ${RELEASE_DIR}/${ZIP_NAME}"
echo
