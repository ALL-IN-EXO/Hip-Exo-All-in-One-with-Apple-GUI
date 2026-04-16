Param(
  [string]$PythonExe = "py",
  [string]$AppName = "HipExoControllerGUI",
  [string]$AppVersion = "",
  [switch]$SkipDepInstall,
  [switch]$FullBuild,
  [switch]$DisableIcon,
  [switch]$DisableVersion
)

$ErrorActionPreference = "Stop"

# Build Windows .exe for:
#   GUI_RL_update/GUI.py
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File .\scripts\build_win.ps1
#   powershell -ExecutionPolicy Bypass -File .\scripts\build_win.ps1 -PythonExe "C:\Python39\python.exe"
#
# Notes:
# - Icon and version stamping are ON by default.
# - Disable with -DisableIcon / -DisableVersion.

function Get-AppVersionFromEntry([string]$EntryPath) {
  if (!(Test-Path $EntryPath)) { return "1.0.0" }
  $raw = Get-Content -Raw -Path $EntryPath
  $m = [regex]::Match($raw, 'v(\d+(?:\.\d+)+)')
  if ($m.Success) { return $m.Groups[1].Value }
  return "1.0.0"
}

function Get-VersionTuple4([string]$VersionText) {
  $parts = @()
  foreach ($p in ($VersionText -split '\.')) {
    if ($p -match '^\d+$') { $parts += [int]$p }
  }
  while ($parts.Count -lt 4) { $parts += 0 }
  if ($parts.Count -gt 4) { $parts = $parts[0..3] }
  return $parts
}

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Resolve-Path (Join-Path $ScriptDir "..")
$AppDir = Resolve-Path (Join-Path $RepoRoot "GUI_RL_update")
$Entry = Join-Path $AppDir "GUI.py"

if (!(Test-Path $Entry)) {
  throw "Entry not found: $Entry"
}

$IconEnabled = (-not $DisableIcon)
$VersionEnabled = (-not $DisableVersion)
if ($env:ICON_ENABLED -eq "0") { $IconEnabled = $false }
if ($env:VERSION_ENABLED -eq "0") { $VersionEnabled = $false }

$IconSource = if ($env:ICON_SOURCE) { $env:ICON_SOURCE } else { Join-Path $RepoRoot "scripts\assets\app_icon.jpeg" }

if ([string]::IsNullOrWhiteSpace($AppVersion)) {
  if ($env:APP_VERSION) {
    $AppVersion = $env:APP_VERSION
  } else {
    $AppVersion = Get-AppVersionFromEntry $Entry
  }
}

try {
  $GitSha = (git -C $RepoRoot rev-parse --short HEAD).Trim()
} catch {
  $GitSha = "nogit"
}

$Ts = Get-Date -Format "yyyyMMdd_HHmmss"
$BuildTag = "${Ts}_${GitSha}"

$VenvDir = Join-Path $AppDir ".venv-build-win"
$DistDir = Join-Path $AppDir "dist"
$BuildDir = Join-Path $AppDir "build"
$SpecPath = Join-Path $AppDir "$AppName.spec"
$ReleaseDir = Join-Path $AppDir "release\windows\$BuildTag"
$VersionFile = Join-Path $BuildDir "version_info.txt"
$IconIco = Join-Path $BuildDir "app_icon.ico"

Write-Host "==> Repo: $RepoRoot"
Write-Host "==> App dir: $AppDir"
Write-Host "==> Build tag: $BuildTag"
Write-Host "==> App version: $AppVersion"
Write-Host "==> Icon enabled: $IconEnabled"
Write-Host "==> Version enabled: $VersionEnabled"

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
  & $VenvPy -m pip install --upgrade pyinstaller pyqt5 pyqtgraph pyserial numpy pillow
}

Write-Host "==> Cleaning previous build folders"
if (Test-Path $DistDir) { Remove-Item -Recurse -Force $DistDir }
if (Test-Path $BuildDir) { Remove-Item -Recurse -Force $BuildDir }
if (Test-Path $SpecPath) { Remove-Item -Force $SpecPath }
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null

$env:QT_API = "pyqt5"

$PyArgs = @(
  "--noconfirm",
  "--clean",
  "--windowed",
  "--name", $AppName,
  "--distpath", $DistDir,
  "--workpath", $BuildDir,
  "--specpath", $AppDir
)

if ($IconEnabled -and (Test-Path $IconSource)) {
  Write-Host "==> Preparing Windows icon (.ico) from: $IconSource"
  $IconPy = Join-Path $BuildDir "_make_icon.py"
  @"
from PIL import Image
img = Image.open(r'''$IconSource''').convert('RGBA')
img.save(r'''$IconIco''', sizes=[(16,16),(24,24),(32,32),(48,48),(64,64),(128,128),(256,256)])
"@ | Set-Content -Path $IconPy -Encoding UTF8
  & $VenvPy $IconPy
  if (Test-Path $IconIco) {
    $PyArgs += @("--icon", $IconIco)
  } else {
    Write-Host "[warn] icon conversion failed; continue without custom icon"
  }
} elseif ($IconEnabled) {
  Write-Host "[warn] icon source not found: $IconSource"
}

if ($VersionEnabled) {
  $tuple = Get-VersionTuple4 $AppVersion
  $tupleStr = "$($tuple[0]), $($tuple[1]), $($tuple[2]), $($tuple[3])"
  $versionNorm = "$($tuple[0]).$($tuple[1]).$($tuple[2]).$($tuple[3])"
  $VersionText = @"
VSVersionInfo(
  ffi=FixedFileInfo(
    filevers=($tupleStr),
    prodvers=($tupleStr),
    mask=0x3f,
    flags=0x0,
    OS=0x40004,
    fileType=0x1,
    subtype=0x0,
    date=(0, 0)
    ),
  kids=[
    StringFileInfo(
      [
      StringTable(
        '040904B0',
        [StringStruct('CompanyName', 'ALL-IN-EXO'),
        StringStruct('FileDescription', '$AppName'),
        StringStruct('FileVersion', '$versionNorm'),
        StringStruct('InternalName', '$AppName'),
        StringStruct('OriginalFilename', '$AppName.exe'),
        StringStruct('ProductName', '$AppName'),
        StringStruct('ProductVersion', '$versionNorm')])
      ]),
    VarFileInfo([VarStruct('Translation', [1033, 1200])])
  ]
)
"@
  Set-Content -Path $VersionFile -Value $VersionText -Encoding UTF8
  $PyArgs += @("--version-file", $VersionFile)
}

if ($FullBuild) {
  Write-Host "==> Build mode: FULL (collect-all)"
  $PyArgs += @("--collect-all", "PyQt5", "--collect-all", "pyqtgraph")
}
else {
  Write-Host "==> Build mode: SLIM (default)"
  $PyArgs += @(
    "--hidden-import", "serial.tools.list_ports",
    "--hidden-import", "pyqtgraph",
    "--exclude-module", "tkinter",
    "--exclude-module", "_tkinter",
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
    "--exclude-module", "PyQt5.QtOpenGL",
    "--exclude-module", "PyQt5.QtPositioning",
    "--exclude-module", "PyQt5.QtPrintSupport",
    "--exclude-module", "PyQt5.QtQml",
    "--exclude-module", "PyQt5.QtQuick",
    "--exclude-module", "PyQt5.QtQuick3D",
    "--exclude-module", "PyQt5.QtQuickWidgets",
    "--exclude-module", "PyQt5.QtRemoteObjects",
    "--exclude-module", "PyQt5.QtSensors",
    "--exclude-module", "PyQt5.QtSerialPort",
    "--exclude-module", "PyQt5.QtSql",
    "--exclude-module", "PyQt5.QtSvg",
    "--exclude-module", "PyQt5.QtTest",
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
& $VenvPy -m PyInstaller @PyArgs $Entry

$BuiltExeDir = Join-Path $DistDir $AppName
$BuiltExe = Join-Path $BuiltExeDir "$AppName.exe"
if (!(Test-Path $BuiltExe)) {
  throw "Build failed: exe not found at $BuiltExe"
}

New-Item -ItemType Directory -Force -Path $ReleaseDir | Out-Null

Write-Host "==> Copying build output to release folder"
Copy-Item -Recurse -Force $BuiltExeDir $ReleaseDir

$ZipPath = Join-Path $ReleaseDir "${AppName}_v${AppVersion}_win_${BuildTag}.zip"
Write-Host "==> Creating zip: $ZipPath"
Compress-Archive -Path (Join-Path $ReleaseDir $AppName) -DestinationPath $ZipPath -Force

Write-Host ""
Write-Host "Build complete."
Write-Host "Executable folder: $(Join-Path $ReleaseDir $AppName)"
Write-Host "Zip file:          $ZipPath"
Write-Host ""
