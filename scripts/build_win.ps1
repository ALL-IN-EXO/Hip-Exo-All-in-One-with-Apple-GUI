Param(
  [string]$PythonExe = "py",
  [string]$AppName = "HipExoRLGUI",
  [string]$Entry = "GUI.py",
  [string]$SourceDir = "GUI_RL_update",
  [switch]$SkipDepInstall,
  [switch]$FullBuild
)

$ErrorActionPreference = "Stop"

# Build Windows .exe for Hip-Exo GUI
#   <SourceDir>/<Entry>   (default: GUI_RL_update/GUI.py)
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File .\scripts\build_win.ps1
#   powershell -ExecutionPolicy Bypass -File .\scripts\build_win.ps1 -PythonExe "C:\Python311\python.exe"
#   powershell -ExecutionPolicy Bypass -File .\scripts\build_win.ps1 -FullBuild
#   powershell -ExecutionPolicy Bypass -File .\scripts\build_win.ps1 -SkipDepInstall

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot  = Resolve-Path (Join-Path $ScriptDir "..")
$AppDir    = Resolve-Path (Join-Path $RepoRoot $SourceDir)
$EntryPath = Join-Path $AppDir $Entry

if (!(Test-Path $EntryPath)) {
  throw "Entry not found: $EntryPath"
}

try {
  $GitSha = (git -C $RepoRoot rev-parse --short HEAD).Trim()
} catch {
  $GitSha = "nogit"
}

$Ts       = Get-Date -Format "yyyyMMdd_HHmmss"
$BuildTag = "${Ts}_${GitSha}"

$VenvDir    = Join-Path $AppDir ".venv-build-win"
$DistDir    = Join-Path $AppDir "dist"
$BuildDir   = Join-Path $AppDir "build"
$SpecPath   = Join-Path $AppDir "$AppName.spec"
$ReleaseDir = Join-Path $AppDir "release\windows\$BuildTag"

Write-Host "==> Repo:      $RepoRoot"
Write-Host "==> App dir:   $AppDir"
Write-Host "==> Entry:     $EntryPath"
Write-Host "==> App name:  $AppName"
Write-Host "==> Build tag: $BuildTag"

if (!(Test-Path $VenvDir)) {
  Write-Host "==> Creating build venv: $VenvDir"
  & $PythonExe -m venv $VenvDir
}

$VenvPy = Join-Path $VenvDir "Scripts\python.exe"
if (!(Test-Path $VenvPy)) {
  throw "venv python not found: $VenvPy"
}

if (!$SkipDepInstall) {
  Write-Host "==> Installing build/runtime dependencies into venv"
  & $VenvPy -m pip install --upgrade pip setuptools wheel
  & $VenvPy -m pip install --upgrade pyinstaller pyqt5 pyqtgraph pyserial numpy
}

Write-Host "==> Cleaning previous build folders"
if (Test-Path $DistDir)  { Remove-Item -Recurse -Force $DistDir }
if (Test-Path $BuildDir) { Remove-Item -Recurse -Force $BuildDir }
if (Test-Path $SpecPath) { Remove-Item -Force $SpecPath }

$env:QT_API = "pyqt5"

$PyArgs = @(
  "--noconfirm",
  "--clean",
  "--windowed",
  "--name", $AppName,
  "--distpath", $DistDir,
  "--workpath", $BuildDir,
  "--specpath", $AppDir,
  "--paths", $AppDir
)

if ($FullBuild) {
  Write-Host "==> Build mode: FULL (collect-all)"
  $PyArgs += @(
    "--collect-all", "PyQt5",
    "--collect-all", "pyqtgraph"
  )
}
else {
  Write-Host "==> Build mode: SLIM (default)"
  $PyArgs += @(
    "--hidden-import", "pyqtgraph",
    "--hidden-import", "serial",
    "--hidden-import", "serial.tools.list_ports",
    "--collect-submodules", "pyqtgraph",
    "--exclude-module", "tkinter",
    "--exclude-module", "_tkinter",
    "--exclude-module", "PIL.ImageTk",
    "--exclude-module", "matplotlib",
    "--exclude-module", "PyQt5.Qt3DAnimation",
    "--exclude-module", "PyQt5.Qt3DCore",
    "--exclude-module", "PyQt5.Qt3DExtras",
    "--exclude-module", "PyQt5.Qt3DInput",
    "--exclude-module", "PyQt5.Qt3DLogic",
    "--exclude-module", "PyQt5.Qt3DRender",
    "--exclude-module", "PyQt5.QtBluetooth",
    "--exclude-module", "PyQt5.QtChart",
    "--exclude-module", "PyQt5.QtDataVisualization",
    "--exclude-module", "PyQt5.QtDesigner",
    "--exclude-module", "PyQt5.QtHelp",
    "--exclude-module", "PyQt5.QtLocation",
    "--exclude-module", "PyQt5.QtMultimedia",
    "--exclude-module", "PyQt5.QtMultimediaWidgets",
    "--exclude-module", "PyQt5.QtNetworkAuth",
    "--exclude-module", "PyQt5.QtNfc",
    "--exclude-module", "PyQt5.QtPositioning",
    "--exclude-module", "PyQt5.QtQml",
    "--exclude-module", "PyQt5.QtQuick",
    "--exclude-module", "PyQt5.QtQuick3D",
    "--exclude-module", "PyQt5.QtQuickWidgets",
    "--exclude-module", "PyQt5.QtRemoteObjects",
    "--exclude-module", "PyQt5.QtSensors",
    "--exclude-module", "PyQt5.QtTextToSpeech",
    "--exclude-module", "PyQt5.QtWebChannel",
    "--exclude-module", "PyQt5.QtWebSockets",
    "--exclude-module", "PyQt5.QtXml",
    "--exclude-module", "PyQt5.QtXmlPatterns",
    "--exclude-module", "PyQt5.QtWebEngine",
    "--exclude-module", "PyQt5.QtWebEngineCore",
    "--exclude-module", "PyQt5.QtWebEngineWidgets"
  )
}

Write-Host "==> Running PyInstaller"
& $VenvPy -m PyInstaller @PyArgs $EntryPath

$BuiltExeDir = Join-Path $DistDir $AppName
$BuiltExe    = Join-Path $BuiltExeDir "$AppName.exe"
if (!(Test-Path $BuiltExe)) {
  throw "Build failed: exe not found at $BuiltExe"
}

New-Item -ItemType Directory -Force -Path $ReleaseDir | Out-Null

Write-Host "==> Copying build output to release folder"
Copy-Item -Recurse -Force $BuiltExeDir $ReleaseDir

$ZipPath = Join-Path $ReleaseDir "${AppName}_win_${BuildTag}.zip"
Write-Host "==> Creating zip: $ZipPath"
Compress-Archive -Path (Join-Path $ReleaseDir $AppName) -DestinationPath $ZipPath -Force

Write-Host ""
Write-Host "Build complete."
Write-Host "Executable folder: $(Join-Path $ReleaseDir $AppName)"
Write-Host "Zip file:          $ZipPath"
Write-Host ""
Write-Host "For updates: pull latest code and run this script again; it creates a new timestamped release folder each run."
