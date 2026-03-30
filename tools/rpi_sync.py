#!/usr/bin/env python3
"""
Sync local RPi_Unified code with Raspberry Pi deployment folder.

Default behavior:
1) Run once: push local code to Pi (exclude output/)
2) Watch mode: pull Pi output/ back to local output/

Examples:
  python tools/rpi_sync.py
  python tools/rpi_sync.py --watch 2
  python tools/rpi_sync.py --direction pull --watch 1
  python tools/rpi_sync.py --direction both
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import pathlib
import re
import shlex
import shutil
import subprocess
import sys
import time


DEFAULT_HOST = "192.168.31.34"
DEFAULT_USER = "aboutberlin"
DEFAULT_REMOTE_DIR = "/home/aboutberlin/Desktop/RPi_Unified"
DEFAULT_PASSWORD = "a1231111"


def _detect_default_local_dir() -> pathlib.Path:
    """
    Support both layouts:
    1) .../RPi_Unified/tools/rpi_sync.py
    2) .../Deployment Code/tools/rpi_sync.py
    """
    script_dir = pathlib.Path(__file__).resolve().parent
    parent = script_dir.parent

    if parent.name == "RPi_Unified":
        return parent

    candidate = parent / "RPi_Unified"
    if candidate.is_dir():
        return candidate

    return parent


DEFAULT_LOCAL_DIR = _detect_default_local_dir()


def _detect_default_data_root() -> pathlib.Path:
    return DEFAULT_LOCAL_DIR.parent / "data_pi"


DEFAULT_DATA_ROOT = _detect_default_data_root()

DEFAULT_EXCLUDES = [
    ".git/",
    "__pycache__/",
    "*.pyc",
    ".DS_Store",
    "output/",
]


def now_str() -> str:
    return dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def build_ssh_transport(port: int, identity_file: str | None) -> str:
    parts = [
        "ssh",
        "-p",
        str(port),
        "-o",
        "StrictHostKeyChecking=accept-new",
        "-o",
        "LogLevel=ERROR",
    ]
    if identity_file:
        parts.extend(["-i", identity_file])
    return shlex.join(parts)


def with_auth_prefix(cmd: list[str], password: str | None) -> list[str]:
    if password:
        return ["sshpass", "-p", password, *cmd]
    return cmd


def format_cmd_for_log(cmd: list[str]) -> str:
    masked = list(cmd)
    for i in range(len(masked) - 1):
        if masked[i] == "-p" and i > 0 and masked[i - 1] == "sshpass":
            masked[i + 1] = "******"
    return " ".join(shlex.quote(x) for x in masked)


def run_cmd(cmd: list[str], dry_run: bool) -> bool:
    print(f"[{now_str()}] $ {format_cmd_for_log(cmd)}")
    if dry_run:
        return True

    env = dict(**os.environ, COPYFILE_DISABLE="1")
    proc = subprocess.run(cmd, check=False, env=env)
    return proc.returncode == 0


def push_code(args: argparse.Namespace, ssh_transport: str) -> bool:
    local_dir = pathlib.Path(args.local_dir).resolve()
    remote = f"{args.user}@{args.host}:{args.remote_dir.rstrip('/')}/"

    cmd = [
        "rsync",
        "-az",
        "--partial",
        "--omit-dir-times",
        "-e",
        ssh_transport,
    ]
    if args.delete_remote:
        cmd.append("--delete")

    for pattern in args.exclude:
        cmd.append(f"--exclude={pattern}")

    cmd.extend([f"{str(local_dir)}/", remote])
    cmd = with_auth_prefix(cmd, args.password)
    return run_cmd(cmd, dry_run=args.dry_run)


def pull_output(args: argparse.Namespace, ssh_transport: str) -> bool:
    data_root = pathlib.Path(args.data_root).resolve()
    history_root = data_root / "pull_history"
    mirror_output = history_root / "_mirror_output"
    mirror_output.mkdir(parents=True, exist_ok=True)
    history_root.mkdir(parents=True, exist_ok=True)
    _migrate_legacy_mirror(data_root, mirror_output)

    remote_output = (
        f"{args.user}@{args.host}:{args.remote_dir.rstrip('/')}/output/"
    )

    cmd = [
        "rsync",
        "-az",
        "--no-perms",
        "--no-owner",
        "--no-group",
        "--partial",
        "--omit-dir-times",
        "--itemize-changes",
        "--stats",
        "-e",
        ssh_transport,
        remote_output,
        f"{str(mirror_output)}/",
    ]
    cmd = with_auth_prefix(cmd, args.password)
    print(f"[{now_str()}] $ {format_cmd_for_log(cmd)}")
    if args.dry_run:
        return True

    proc = subprocess.run(
        cmd,
        check=False,
        text=True,
        capture_output=True,
        env=dict(**os.environ, COPYFILE_DISABLE="1"),
    )
    if proc.stdout:
        print(proc.stdout.strip())
    if proc.stderr:
        print(proc.stderr.strip())

    changed_items = parse_rsync_changes(proc.stdout or "")
    pulled_files = [
        item["path"]
        for item in changed_items
        if item["code"].startswith(">f")
    ]

    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir = history_root / stamp
    run_dir.mkdir(parents=True, exist_ok=True)
    report_path = run_dir / "pull_report.json"
    summary_path = run_dir / "summary.txt"
    pulled_list_path = run_dir / "pulled_files.txt"

    report = {
        "timestamp": stamp,
        "remote_output": remote_output,
        "mirror_output": str(mirror_output),
        "changed_count": len(changed_items),
        "pulled_file_count": len(pulled_files),
        "pulled_files": pulled_files,
        "rsync_return_code": proc.returncode,
    }
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    summary_path.write_text(
        (
            f"time={stamp}\n"
            f"changed_count={len(changed_items)}\n"
            f"pulled_file_count={len(pulled_files)}\n"
            f"report={report_path}\n"
        ),
        encoding="utf-8",
    )
    pulled_list_path.write_text(
        ("\n".join(pulled_files) + ("\n" if pulled_files else "")),
        encoding="utf-8",
    )

    latest_summary = data_root / "latest_pull.txt"
    latest_summary.write_text(
        (
            f"time={stamp}\n"
            f"changed_count={len(changed_items)}\n"
            f"pulled_file_count={len(pulled_files)}\n"
            f"last_report={report_path}\n"
            f"last_files={pulled_list_path}\n"
        ),
        encoding="utf-8",
    )
    _append_history_index(
        history_root=history_root,
        timestamp=stamp,
        changed_count=len(changed_items),
        pulled_count=len(pulled_files),
        report_path=report_path,
    )
    (history_root / "LATEST").write_text(stamp + "\n", encoding="utf-8")

    print(
        f"[{now_str()}] Pull summary: changed={len(changed_items)}, "
        f"pulled={len(pulled_files)}, report={report_path}"
    )
    return proc.returncode == 0


def _migrate_legacy_mirror(data_root: pathlib.Path, mirror_output: pathlib.Path) -> None:
    legacy_mirror = data_root / "mirror_output"
    if legacy_mirror.exists() and legacy_mirror.is_dir() and not any(mirror_output.iterdir()):
        for item in legacy_mirror.iterdir():
            target = mirror_output / item.name
            if target.exists():
                continue
            item.rename(target)
        try:
            legacy_mirror.rmdir()
        except OSError:
            pass


def _append_history_index(
    history_root: pathlib.Path,
    timestamp: str,
    changed_count: int,
    pulled_count: int,
    report_path: pathlib.Path,
) -> None:
    _ = (timestamp, changed_count, pulled_count, report_path)
    _ensure_history_index(history_root)


def _ensure_history_index(history_root: pathlib.Path) -> None:
    history_csv = history_root / "history.csv"
    lines = ["timestamp,changed_count,pulled_file_count,rsync_return_code,report_path\n"]
    run_dirs = []
    for run_dir in history_root.iterdir():
        if not run_dir.is_dir():
            continue
        if not re.match(r"^\d{8}-\d{6}$", run_dir.name):
            continue
        run_dirs.append(run_dir)

    # Newest first for quick inspection in Finder/editor.
    for run_dir in sorted(run_dirs, key=lambda p: p.name, reverse=True):
        report = run_dir / "pull_report.json"
        changed, pulled, ret = "0", "0", "NA"
        if report.exists():
            try:
                obj = json.loads(report.read_text(encoding="utf-8"))
                changed = str(obj.get("changed_count", 0))
                pulled = str(obj.get("pulled_file_count", 0))
                ret = str(obj.get("rsync_return_code", "NA"))
            except Exception:
                pass
        else:
            summary = _read_summary(run_dir / "summary.txt")
            changed = summary.get("changed_count", "0")
            pulled = summary.get("pulled_file_count", "0")
        lines.append(f"{run_dir.name},{changed},{pulled},{ret},{report}\n")

    history_csv.write_text("".join(lines), encoding="utf-8")


def _read_summary(path: pathlib.Path) -> dict[str, str]:
    if not path.exists():
        return {}
    out: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def parse_rsync_changes(stdout: str) -> list[dict[str, str]]:
    items: list[dict[str, str]] = []
    for line in stdout.splitlines():
        line = line.strip()
        m = re.match(r"^([<>ch\.\*][^\s]*)\s+(.+)$", line)
        if not m:
            continue
        code, path = m.group(1), m.group(2)
        if code.startswith("."):
            continue
        items.append({"code": code, "path": path})
    return items


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Local <-> Raspberry Pi sync helper for RPi_Unified",
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help=f"Pi host/IP (default: {DEFAULT_HOST})",
    )
    parser.add_argument(
        "--user",
        default=DEFAULT_USER,
        help=f"Pi user (default: {DEFAULT_USER})",
    )
    parser.add_argument(
        "--remote-dir",
        default=DEFAULT_REMOTE_DIR,
        help=f"Pi deployment dir (default: {DEFAULT_REMOTE_DIR})",
    )
    parser.add_argument(
        "--local-dir",
        default=str(DEFAULT_LOCAL_DIR),
        help=f"Local RPi_Unified dir (default: {DEFAULT_LOCAL_DIR})",
    )
    parser.add_argument(
        "--data-root",
        default=str(DEFAULT_DATA_ROOT),
        help=f"Local data root for pulled output (default: {DEFAULT_DATA_ROOT})",
    )
    parser.add_argument(
        "--direction",
        choices=["both", "push", "pull"],
        default=None,
        help="Sync direction (default: push for run-once, pull for watch mode)",
    )
    parser.add_argument(
        "--watch",
        type=float,
        default=0.0,
        help="Loop interval in seconds. 0 = run once (default: 0)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=22,
        help="SSH port (default: 22)",
    )
    parser.add_argument(
        "--identity-file",
        default=None,
        help="SSH private key path, optional",
    )
    parser.add_argument(
        "--password",
        default=DEFAULT_PASSWORD,
        help="SSH password for sshpass (default: built-in)",
    )
    parser.add_argument(
        "--delete-remote",
        action="store_true",
        help="Mirror local code to remote by deleting remote-only files",
    )
    parser.add_argument(
        "--exclude",
        action="append",
        default=list(DEFAULT_EXCLUDES),
        help="Additional rsync exclude pattern. Can repeat.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands without executing",
    )
    return parser.parse_args()


def run_once(args: argparse.Namespace, ssh_transport: str) -> bool:
    ok = True

    if args.direction in {"both", "push"}:
        print(f"[{now_str()}] Push local code -> Pi")
        ok = push_code(args, ssh_transport) and ok

    if args.direction in {"both", "pull"}:
        print(f"[{now_str()}] Pull Pi output -> local")
        ok = pull_output(args, ssh_transport) and ok

    return ok


def main() -> int:
    args = parse_args()

    if shutil.which("rsync") is None:
        print("Error: rsync not found in PATH.")
        return 2
    if args.password and shutil.which("sshpass") is None:
        print("Warning: sshpass not found; fallback to normal ssh password prompt.")
        print("To enable built-in password mode: brew install hudochenkov/sshpass/sshpass")
        args.password = None

    if args.direction is None:
        args.direction = "pull" if args.watch > 0 else "push"

    if args.watch > 0 and args.direction != "pull":
        print(
            "Error: watch mode only supports --direction pull "
            "(to avoid repeated code pushes)."
        )
        print("Run code deploy once with: python tools/rpi_sync.py")
        return 2

    ssh_transport = build_ssh_transport(args.port, args.identity_file)

    if args.watch <= 0:
        return 0 if run_once(args, ssh_transport) else 1

    print(
        f"[{now_str()}] Start watch mode: interval={args.watch}s, "
        f"direction={args.direction}"
    )
    try:
        while True:
            run_once(args, ssh_transport)
            time.sleep(args.watch)
    except KeyboardInterrupt:
        print(f"\n[{now_str()}] Stop watch mode.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
