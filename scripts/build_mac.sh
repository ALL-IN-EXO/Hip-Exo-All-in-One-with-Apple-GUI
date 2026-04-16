#!/usr/bin/env bash
set -euo pipefail

# Build macOS .app + .zip for Hip-Exo GUI
#   <SourceDir>/<Entry>   (default: GUI_RL_update/GUI.py)
#
# Usage:
#   ./scripts/build_mac.sh
#   ./scripts/build_mac.sh --python python3.11
#   ./scripts/build_mac.sh --name HipExoRLGUI --full
#   ./scripts/build_mac.sh --skip-deps
#
# Options:
#   --python <path>      Python interpreter (default: python3)
#   --name <AppName>     App name (default: HipExoRLGUI)
#   --entry <file>       Entry .py (default: GUI.py)
#   --source <dir>       Source dir relative to repo root (default: GUI_RL_update)
#   --skip-deps          Skip pip install (reuse existing venv)
#   --full               Use --collect-all (slower/larger, but safer)

PYTHON_EXE="python3"
APP_NAME="HipExoRLGUI"
ENTRY="GUI.py"
SOURCE_DIR="GUI_RL_update"
SKIP_DEPS=0
FULL_BUILD=0

while [ $# -gt 0 ]; do
  case "$1" in
    --python)    PYTHON_EXE="$2"; shift 2 ;;
    --name)      APP_NAME="$2";   shift 2 ;;
    --entry)     ENTRY="$2";      shift 2 ;;
    --source)    SOURCE_DIR="$2"; shift 2 ;;
    --skip-deps) SKIP_DEPS=1;     shift ;;
    --full)      FULL_BUILD=1;    shift ;;
    -h|--help)
      sed -n '3,22p' "$0" | sed 's/^# \{0,1\}//'
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
APP_DIR="$REPO_ROOT/$SOURCE_DIR"
ENTRY_PATH="$APP_DIR/$ENTRY"

if [ ! -f "$ENTRY_PATH" ]; then
  echo "Entry not found: $ENTRY_PATH" >&2
  exit 1
fi

GIT_SHA="$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo nogit)"
TS="$(date +%Y%m%d_%H%M%S)"
BUILD_TAG="${TS}_${GIT_SHA}"

VENV_DIR="$APP_DIR/.venv-build-mac"
DIST_DIR="$APP_DIR/dist"
BUILD_DIR="$APP_DIR/build"
SPEC_PATH="$APP_DIR/${APP_NAME}.spec"
RELEASE_DIR="$APP_DIR/release/macos/$BUILD_TAG"

echo "==> Repo:      $REPO_ROOT"
echo "==> App dir:   $APP_DIR"
echo "==> Entry:     $ENTRY_PATH"
echo "==> App name:  $APP_NAME"
echo "==> Build tag: $BUILD_TAG"

if [ ! -d "$VENV_DIR" ]; then
  echo "==> Creating build venv: $VENV_DIR"
  "$PYTHON_EXE" -m venv "$VENV_DIR"
fi

VENV_PY="$VENV_DIR/bin/python"
if [ ! -x "$VENV_PY" ]; then
  echo "venv python not found: $VENV_PY" >&2
  exit 1
fi

if [ "$SKIP_DEPS" -eq 0 ]; then
  echo "==> Installing build/runtime dependencies into venv"
  "$VENV_PY" -m pip install --upgrade pip setuptools wheel
  "$VENV_PY" -m pip install --upgrade pyinstaller pyqt5 pyqtgraph pyserial numpy
fi

echo "==> Cleaning previous build folders"
rm -rf "$DIST_DIR" "$BUILD_DIR"
rm -f "$SPEC_PATH"

export QT_API=pyqt5

PY_ARGS=(
  --noconfirm
  --clean
  --windowed
  --name "$APP_NAME"
  --distpath "$DIST_DIR"
  --workpath "$BUILD_DIR"
  --specpath "$APP_DIR"
  --paths "$APP_DIR"
)

if [ "$FULL_BUILD" -eq 1 ]; then
  echo "==> Build mode: FULL (collect-all)"
  PY_ARGS+=( --collect-all PyQt5 --collect-all pyqtgraph )
else
  echo "==> Build mode: SLIM (default)"
  PY_ARGS+=(
    --hidden-import pyqtgraph
    --hidden-import serial
    --hidden-import serial.tools.list_ports
    --collect-submodules pyqtgraph
    --exclude-module tkinter
    --exclude-module _tkinter
    --exclude-module PIL.ImageTk
    --exclude-module matplotlib
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
    --exclude-module PyQt5.QtPositioning
    --exclude-module PyQt5.QtQml
    --exclude-module PyQt5.QtQuick
    --exclude-module PyQt5.QtQuick3D
    --exclude-module PyQt5.QtQuickWidgets
    --exclude-module PyQt5.QtRemoteObjects
    --exclude-module PyQt5.QtSensors
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
"$VENV_PY" -m PyInstaller "${PY_ARGS[@]}" "$ENTRY_PATH"

BUILT_APP="$DIST_DIR/${APP_NAME}.app"
if [ ! -d "$BUILT_APP" ]; then
  echo "Build failed: .app not found at $BUILT_APP" >&2
  exit 1
fi

mkdir -p "$RELEASE_DIR"
echo "==> Copying .app to release folder"
cp -R "$BUILT_APP" "$RELEASE_DIR/"

ZIP_PATH="$RELEASE_DIR/${APP_NAME}_mac_${BUILD_TAG}.zip"
echo "==> Creating zip: $ZIP_PATH"
# Use ditto to preserve .app bundle symlinks / resource forks; regular zip breaks them.
( cd "$RELEASE_DIR" && /usr/bin/ditto -c -k --keepParent "${APP_NAME}.app" "$ZIP_PATH" )

echo ""
echo "Build complete."
echo "App bundle: $RELEASE_DIR/${APP_NAME}.app"
echo "Zip file:   $ZIP_PATH"
echo ""
echo "NOTE: The .app is not codesigned / notarized. On other Macs, users may need to:"
echo "  1) Right-click the .app -> Open (first time only), OR"
echo "  2) In Terminal: xattr -dr com.apple.quarantine \"${APP_NAME}.app\""
echo ""
echo "For updates: pull latest code and run this script again; it creates a new timestamped release folder each run."
