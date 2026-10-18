#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import json
import os
import re
import signal
import subprocess
import sys
import time
from urllib.request import Request, urlopen
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT_DIR = Path(__file__).resolve().parent
JOBS_DIR = ROOT_DIR / "runtime" / "cabinet_inspection_jobs"
SCANNER = ROOT_DIR / "ros2_tools" / "cabinet_apriltag_inspection.py"
TERMINAL_STATES = {"completed", "partial", "failed", "canceled", "timeout"}
SAFE_ID = re.compile(r"^[A-Za-z0-9_.-]{1,120}$")
VISION_CONTROL_URL = os.environ.get("CAR_VISION_STREAM_CONTROL_URL", "http://127.0.0.1:8767").rstrip("/")


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def stop_idle_preview() -> None:
    request = Request(
        f"{VISION_CONTROL_URL}/test/stop",
        data=b"{}",
        method="POST",
        headers={
            "Content-Type": "application/json",
            "X-Control-Token": os.environ.get("CAR_CLOUD_UPLOAD_TOKEN", "car-cloud-upload"),
        },
    )
    try:
        with urlopen(request, timeout=4.0) as response:
            response.read(256)
    except Exception:
        pass


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except Exception:
        return {}


def safe_inspection_id(value: str) -> str:
    result = str(value or "").strip()
    if not SAFE_ID.fullmatch(result):
        raise RuntimeError("inspection_id must contain only letters, numbers, dot, dash or underscore")
    return result


def paths_for(inspection_id: str) -> dict[str, Path]:
    root = JOBS_DIR / safe_inspection_id(inspection_id)
    return {
        "root": root,
        "pid": root / "pid",
        "status": root / "status.json",
        "result": root / "result.json",
        "scanner_status": root / "scanner_status.json",
        "stdout": root / "scanner_stdout.log",
        "stderr": root / "scanner_stderr.log",
        "supervisor": root / "supervisor.log",
    }


def read_pid(path: Path) -> int | None:
    try:
        value = int(path.read_text(encoding="utf-8").strip())
        return value if value > 0 else None
    except Exception:
        return None


def pid_alive(pid: int | None) -> bool:
    if not pid:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def job_snapshot(inspection_id: str) -> dict[str, Any]:
    paths = paths_for(inspection_id)
    status = read_json(paths["status"])
    result = read_json(paths["result"])
    pid = read_pid(paths["pid"])
    state = str(status.get("state") or "")
    if state not in TERMINAL_STATES and pid_alive(pid):
        state = state or "running"
    elif state not in TERMINAL_STATES and pid and not pid_alive(pid):
        state = "failed"
    return {
        **status,
        "inspection_id": inspection_id,
        "state": state or "not_started",
        "pid": pid or 0,
        "process_live": pid_alive(pid),
        "result": result or status.get("result", {}),
        "job_dir": str(paths["root"]),
    }


def scanner_result(paths: dict[str, Path], returncode: int) -> dict[str, Any]:
    scanner_status = read_json(paths["scanner_status"])
    result = scanner_status.get("result")
    if isinstance(result, dict) and result:
        return result
    try:
        value = json.loads(paths["stdout"].read_text(encoding="utf-8"))
        if isinstance(value, dict):
            return value
    except Exception:
        pass
    return {
        "status": "completed" if returncode == 0 else "failed",
        "message": "scanner exited without a structured result",
        "returncode": int(returncode),
    }


def command_start(args: argparse.Namespace) -> int:
    inspection_id = safe_inspection_id(args.inspection_id)
    if int(args.expected_tag_id) not in {10, 11, 12, 13}:
        raise RuntimeError("expected tag ID must be 10-13")
    snapshot = job_snapshot(inspection_id)
    if snapshot["state"] in TERMINAL_STATES or snapshot["process_live"]:
        snapshot["idempotent"] = True
        print(json.dumps(snapshot, ensure_ascii=False))
        return 0

    paths = paths_for(inspection_id)
    paths["root"].mkdir(parents=True, exist_ok=True)
    status = {
        "inspection_id": inspection_id,
        "state": "starting",
        "pid": 0,
        "process_live": False,
        "expected_apriltag_id": int(args.expected_tag_id),
        "navigation_session_id": str(args.navigation_session_id or ""),
        "waypoint_index": int(args.waypoint_index),
        "waypoint_label": str(args.waypoint_label or ""),
        "tag_search_skipped": True,
        "started_at": utc_now_iso(),
        "updated_at": utc_now_iso(),
    }
    atomic_write_json(paths["status"], status)
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "run",
        "--inspection-id",
        inspection_id,
        "--expected-tag-id",
        str(args.expected_tag_id),
        "--navigation-session-id",
        str(args.navigation_session_id or ""),
        "--waypoint-index",
        str(args.waypoint_index),
        "--waypoint-label",
        str(args.waypoint_label or ""),
        "--timeout-s",
        str(args.timeout_s),
    ]
    with paths["supervisor"].open("a", encoding="utf-8") as log_handle:
        process = subprocess.Popen(
            command,
            cwd=str(ROOT_DIR),
            stdout=log_handle,
            stderr=log_handle,
            start_new_session=True,
        )
    paths["pid"].write_text(str(process.pid), encoding="utf-8")
    current = read_json(paths["status"])
    if str(current.get("state") or "") == "starting":
        status.update({"pid": process.pid, "process_live": True, "updated_at": utc_now_iso()})
        atomic_write_json(paths["status"], status)
    else:
        status = current
    print(json.dumps({**status, "job_dir": str(paths["root"]), "idempotent": False}, ensure_ascii=False))
    return 0


def command_run(args: argparse.Namespace) -> int:
    inspection_id = safe_inspection_id(args.inspection_id)
    paths = paths_for(inspection_id)
    paths["root"].mkdir(parents=True, exist_ok=True)
    paths["pid"].write_text(str(os.getpid()), encoding="utf-8")
    child: subprocess.Popen[str] | None = None
    cancel_requested = {"value": False}

    def handle_stop(_signum, _frame) -> None:
        cancel_requested["value"] = True
        if child is not None and child.poll() is None:
            with contextlib.suppress(OSError):
                child.send_signal(signal.SIGTERM)

    signal.signal(signal.SIGTERM, handle_stop)
    signal.signal(signal.SIGINT, handle_stop)
    metadata = {
        "inspection_id": inspection_id,
        "expected_apriltag_id": int(args.expected_tag_id),
        "navigation_session_id": str(args.navigation_session_id or ""),
        "waypoint_index": int(args.waypoint_index),
        "waypoint_label": str(args.waypoint_label or ""),
        "tag_search_skipped": True,
    }
    stop_idle_preview()
    atomic_write_json(
        paths["status"],
        {**metadata, "state": "running", "pid": os.getpid(), "process_live": True, "started_at": utc_now_iso(), "updated_at": utc_now_iso()},
    )
    command = [
        "python3",
        str(SCANNER),
        "--search-config",
        str(ROOT_DIR / "config" / "arm_apriltag_s0_search.yaml"),
        "--profile",
        str(ROOT_DIR / "cabinet_inspection_profiles" / "fixed_scan.yaml"),
        "--execute",
        "--upload",
        "--skip-tag-search",
        "--inspection-id",
        inspection_id,
        "--expected-tag-id",
        str(args.expected_tag_id),
        "--navigation-session-id",
        str(args.navigation_session_id or ""),
        "--waypoint-index",
        str(args.waypoint_index),
        "--waypoint-label",
        str(args.waypoint_label or ""),
        "--status-file",
        str(paths["scanner_status"]),
    ]
    timed_out = False
    returncode = 1
    try:
        with paths["stdout"].open("w", encoding="utf-8") as stdout_handle, paths["stderr"].open("w", encoding="utf-8") as stderr_handle:
            child = subprocess.Popen(command, cwd=str(ROOT_DIR), stdout=stdout_handle, stderr=stderr_handle, text=True)
            deadline = time.monotonic() + max(60, min(1800, int(args.timeout_s)))
            while child.poll() is None:
                if cancel_requested["value"]:
                    child.send_signal(signal.SIGTERM)
                    break
                if time.monotonic() >= deadline:
                    timed_out = True
                    child.send_signal(signal.SIGTERM)
                    break
                time.sleep(0.2)
            try:
                returncode = child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                child.kill()
                returncode = child.wait(timeout=5)
        result = scanner_result(paths, returncode)
        state = str(result.get("status") or "failed").lower()
        if cancel_requested["value"]:
            state = "canceled"
            result = {**result, "status": "canceled", "message": "inspection canceled"}
        elif timed_out:
            state = "timeout"
            result = {**result, "status": "failed", "message": "inspection timed out"}
        elif state not in {"completed", "partial"}:
            state = "failed"
        result.update({key: value for key, value in metadata.items() if key not in result})
        atomic_write_json(paths["result"], result)
        atomic_write_json(
            paths["status"],
            {**metadata, "state": state, "pid": 0, "process_live": False, "returncode": returncode, "result": result, "updated_at": utc_now_iso()},
        )
        return 0 if state in {"completed", "partial"} else 1
    finally:
        with contextlib.suppress(OSError):
            paths["pid"].unlink()


def command_status(args: argparse.Namespace) -> int:
    print(json.dumps(job_snapshot(args.inspection_id), ensure_ascii=False))
    return 0


def command_stop(args: argparse.Namespace) -> int:
    inspection_id = safe_inspection_id(args.inspection_id)
    paths = paths_for(inspection_id)
    pid = read_pid(paths["pid"])
    if pid_alive(pid):
        os.kill(pid, signal.SIGTERM)
        deadline = time.monotonic() + 15.0
        while pid_alive(pid) and time.monotonic() < deadline:
            time.sleep(0.2)
        if pid_alive(pid):
            with contextlib.suppress(OSError):
                os.killpg(pid, signal.SIGKILL)
    snapshot = job_snapshot(inspection_id)
    if snapshot["state"] not in TERMINAL_STATES:
        snapshot.update({"state": "canceled", "process_live": False, "pid": 0, "updated_at": utc_now_iso()})
        atomic_write_json(paths["status"], snapshot)
    print(json.dumps(job_snapshot(inspection_id), ensure_ascii=False))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Asynchronous cabinet inspection job manager")
    subparsers = parser.add_subparsers(dest="command", required=True)
    for name, handler in (("start", command_start), ("run", command_run)):
        command = subparsers.add_parser(name)
        command.add_argument("--inspection-id", required=True)
        command.add_argument("--expected-tag-id", type=int, required=True)
        command.add_argument("--navigation-session-id", default="")
        command.add_argument("--waypoint-index", type=int, default=-1)
        command.add_argument("--waypoint-label", default="")
        command.add_argument("--timeout-s", type=int, default=900)
        command.set_defaults(func=handler)
    status = subparsers.add_parser("status")
    status.add_argument("--inspection-id", required=True)
    status.set_defaults(func=command_status)
    stop = subparsers.add_parser("stop")
    stop.add_argument("--inspection-id", required=True)
    stop.set_defaults(func=command_stop)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        return int(args.func(args))
    except Exception as exc:
        print(json.dumps({"status": "failed", "message": str(exc)}, ensure_ascii=False))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
