from __future__ import annotations

import ast
import copy
import json
import mimetypes
import os
from pathlib import Path
import struct
import threading
from datetime import datetime, timezone
from socketserver import ThreadingMixIn
from typing import Any
from urllib.parse import quote
from uuid import uuid4
from wsgiref.simple_server import WSGIRequestHandler, WSGIServer, make_server

from flask import Flask, jsonify, render_template, request, send_file

try:
    from .teleop_shared import (
        TELEOP_SPEED_PRESETS,
        empty_teleop_state,
        normalize_controller_id,
        normalize_pressed_keys,
        normalize_speed_level,
        sanitize_teleop_state,
        utc_now_iso,
    )
    from .alarm_evaluator import evaluate_cabinet_data, event_type as assessment_event_type
except Exception:
    from teleop_shared import (
        TELEOP_SPEED_PRESETS,
        empty_teleop_state,
        normalize_controller_id,
        normalize_pressed_keys,
        normalize_speed_level,
        sanitize_teleop_state,
        utc_now_iso,
    )
    from alarm_evaluator import evaluate_cabinet_data, event_type as assessment_event_type

ROOT_DIR = Path(__file__).resolve().parent
RUNTIME_DIR = ROOT_DIR / "runtime"
FRAME_DIR = RUNTIME_DIR / "frames"
RECOGNITION_DIR = RUNTIME_DIR / "recognition"
STATE_PATH = RUNTIME_DIR / "latest_state.json"
ALARM_EVENTS_PATH = RUNTIME_DIR / "alarm_events.jsonl"
MAP_LIBRARY_DIRS = [ROOT_DIR / "lidar_maps", ROOT_DIR.parent / "lidar_maps"]
ALLOWED_FRAME_KINDS = {"map", "scan", "camera", "nav_overlay"}
ALLOWED_COMMAND_ACTIONS = {
    "start_mapping",
    "stop_mapping",
    "save_map",
    "start_navigation",
    "stop_navigation",
    "set_initial_pose",
    "start_cruise",
    "stop_cruise",
    "capture_recognition",
}
DEFAULT_UPLOAD_TOKEN = "car-cloud-upload"
COMMAND_HISTORY_LIMIT = 40
COMMAND_CLAIM_TIMEOUT_S = 120.0
RECOGNITION_HISTORY_LIMIT = 80

CABINET_DATA_TEMPLATE = {
    "voltage": {"value": None, "unit": "V", "min": 0, "max": 450, "status": "unknown"},
    "current": {"value": None, "unit": "A", "min": 0, "max": 1.0, "status": "unknown"},
    "temperature_controller": {"pv": None, "sv": None, "unit": "℃", "status": "unknown"},
    "motor_1": {"start": False, "stop": False, "mode": "关闭"},
    "motor_2": {"start": False, "stop": False, "mode": "关闭"},
    "warning": {"high_voltage": "normal"},
    "door": {"state": "unknown"},
    "environment": {
        "smoke": {"value": None, "unit": "ppm", "status": "unknown"},
        "hydrogen": {"value": None, "unit": "ppm", "status": "unknown"},
        "carbon_monoxide": {"value": None, "unit": "ppm", "status": "unknown"},
        "temperature": {"value": None, "unit": "℃", "status": "unknown"},
        "humidity": {"value": None, "unit": "%RH", "status": "unknown"},
        "infrared_temperature": {"value": None, "unit": "℃", "status": "unknown"},
        "sound_level": {"value": None, "unit": "dB", "status": "unknown"},
    },
    "updated_at": "",
}

RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
FRAME_DIR.mkdir(parents=True, exist_ok=True)
RECOGNITION_DIR.mkdir(parents=True, exist_ok=True)

app = Flask(__name__, template_folder="templates", static_folder="static")


def deep_merge(base: dict[str, Any], update: dict[str, Any]) -> dict[str, Any]:
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def merged_cabinet_data(update: Any) -> dict[str, Any]:
    data = copy.deepcopy(CABINET_DATA_TEMPLATE)
    if isinstance(update, dict):
        deep_merge(data, update)
    return data


def cabinet_data_from_meter_readings(readings: Any) -> dict[str, Any]:
    if not isinstance(readings, list):
        return {}
    payload: dict[str, Any] = {}
    for item in readings:
        if not isinstance(item, dict):
            continue
        meter_type = str(item.get("meter_type") or "").strip()
        if meter_type not in {"voltage", "current"}:
            continue
        metric = item.get("cabinet_metric")
        if not isinstance(metric, dict):
            value = item.get("value")
            try:
                value = None if value is None else float(value)
            except (TypeError, ValueError):
                value = None
            if value is None:
                continue
            metric = {
                "value": value,
                "unit": "V" if meter_type == "voltage" else "A",
                "status": str(item.get("status") or "ok"),
                "min": 0,
                "max": 450 if meter_type == "voltage" else 1.0,
            }
        payload[meter_type] = metric
    if payload:
        payload["updated_at"] = utc_now_iso()
    return payload


LAMP_LABELS = {"red_on", "red_off", "green_on", "green_off"}
LAMP_OPPOSITES = {
    "red_on": "red_off",
    "red_off": "red_on",
    "green_on": "green_off",
    "green_off": "green_on",
}


def detection_label(item: Any) -> str:
    if not isinstance(item, dict):
        return ""
    raw = str(item.get("class_name") or item.get("label") or item.get("display_name") or "").strip().lower()
    if ":" in raw:
        raw = raw.rsplit(":", 1)[-1].strip()
    return raw


def detection_box_xyxy(item: Any) -> list[float] | None:
    if not isinstance(item, dict):
        return None
    box = item.get("box_xyxy") or item.get("bbox") or item.get("box")
    if not isinstance(box, list) and not isinstance(box, tuple):
        return None
    if len(box) != 4:
        return None
    try:
        x1, y1, x2, y2 = [float(value) for value in box]
    except (TypeError, ValueError):
        return None
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    return [x1, y1, x2, y2]


def detection_center_y(item: Any) -> float | None:
    box = detection_box_xyxy(item)
    if box is None:
        return None
    return (box[1] + box[3]) * 0.5


def detection_height(item: Any) -> float:
    box = detection_box_xyxy(item)
    if box is None:
        return 1.0
    return max(1.0, box[3] - box[1])


def sanitized_lamp_detection(item: dict[str, Any], label: str, item_id: str, uploaded_at: str) -> dict[str, Any]:
    box = detection_box_xyxy(item) or []
    return {
        "label": label,
        "confidence": item.get("confidence"),
        "box_xyxy": [round(float(value), 3) for value in box],
        "center_y": None if detection_center_y(item) is None else round(float(detection_center_y(item)), 3),
        "item_id": item_id,
        "updated_at": uploaded_at,
    }


def group_lamp_detections_by_row(detections: Any) -> list[dict[str, Any]]:
    if not isinstance(detections, list):
        return []
    candidates: list[dict[str, Any]] = []
    for item in detections:
        if not isinstance(item, dict):
            continue
        label = detection_label(item)
        center_y = detection_center_y(item)
        if label not in LAMP_LABELS or center_y is None:
            continue
        candidates.append({"label": label, "center_y": center_y, "height": detection_height(item), "item": item})
    if not candidates:
        return []

    candidates.sort(key=lambda value: value["center_y"])
    heights = sorted(float(value["height"]) for value in candidates)
    median_height = heights[len(heights) // 2] if heights else 24.0
    tolerance = max(24.0, median_height * 0.75)

    rows: list[dict[str, Any]] = []
    for candidate in candidates:
        target = None
        for row in rows:
            if abs(float(candidate["center_y"]) - float(row["center_y"])) <= tolerance:
                target = row
                break
        if target is None:
            target = {"center_y": float(candidate["center_y"]), "items": []}
            rows.append(target)
        target["items"].append(candidate)
        target["center_y"] = sum(float(item["center_y"]) for item in target["items"]) / len(target["items"])

    return sorted(rows, key=lambda row: float(row["center_y"]))


def assign_lamp_rows(rows: list[dict[str, Any]], image_size: tuple[int, int] | None) -> dict[str, dict[str, Any]]:
    assigned: dict[str, dict[str, Any]] = {}
    if not rows:
        return assigned

    if len(rows) == 1:
        row_key = "motor_1"
        if image_size is not None:
            _, height = image_size
            if height > 0 and float(rows[0]["center_y"]) >= height * 0.52:
                row_key = "motor_2"
        assigned[row_key] = rows[0]
        return assigned

    for index, row in enumerate(rows[:2], start=1):
        assigned[f"motor_{index}"] = row
    return assigned


def update_lamp_detection_memory(
    memory: Any,
    detections: Any,
    *,
    image_size: tuple[int, int] | None,
    item_id: str,
    uploaded_at: str,
) -> dict[str, Any]:
    if not isinstance(memory, dict):
        memory = {}
    rows = assign_lamp_rows(group_lamp_detections_by_row(detections), image_size)
    for motor_key, row in rows.items():
        motor_memory = memory.setdefault(motor_key, {})
        if not isinstance(motor_memory, dict):
            motor_memory = {}
            memory[motor_key] = motor_memory
        best_by_label: dict[str, dict[str, Any]] = {}
        for row_item in row.get("items", []):
            label = str(row_item.get("label") or "")
            item = row_item.get("item")
            if label not in LAMP_LABELS or not isinstance(item, dict):
                continue
            previous = best_by_label.get(label)
            if previous is None or float(item.get("confidence") or 0.0) >= float(previous.get("confidence") or 0.0):
                best_by_label[label] = item
        for label, item in best_by_label.items():
            opposite = LAMP_OPPOSITES.get(label)
            if opposite:
                motor_memory.pop(opposite, None)
            motor_memory[label] = sanitized_lamp_detection(item, label, item_id, uploaded_at)
    return memory


def cabinet_data_from_lamp_memory(memory: Any) -> dict[str, Any]:
    if not isinstance(memory, dict):
        return {}
    payload: dict[str, Any] = {}
    for motor_key in ("motor_1", "motor_2"):
        labels = memory.get(motor_key)
        if not isinstance(labels, dict):
            continue
        motor_update: dict[str, Any] = {}
        if "green_on" in labels:
            motor_update["start"] = True
        elif "green_off" in labels:
            motor_update["start"] = False
        if "red_on" in labels:
            motor_update["stop"] = True
        elif "red_off" in labels:
            motor_update["stop"] = False
        if "red_on" in labels and "green_off" in labels:
            motor_update["mode"] = "关闭"
        elif "red_off" in labels and "green_on" in labels:
            motor_update["mode"] = "手动"
        if motor_update:
            payload[motor_key] = motor_update
    if payload:
        payload["updated_at"] = utc_now_iso()
    return payload


def read_image_size(image_path: Path) -> tuple[int, int] | None:
    if not image_path.exists():
        return None
    suffix = image_path.suffix.lower()
    try:
        with image_path.open("rb") as handle:
            if suffix == ".png":
                header = handle.read(24)
                if header.startswith(b"\x89PNG\r\n\x1a\n") and len(header) >= 24:
                    width, height = struct.unpack(">II", header[16:24])
                    if width > 0 and height > 0:
                        return int(width), int(height)
            elif suffix in {".jpg", ".jpeg"}:
                if handle.read(2) != b"\xff\xd8":
                    return None
                while True:
                    marker_prefix = handle.read(1)
                    if not marker_prefix:
                        break
                    if marker_prefix != b"\xff":
                        continue
                    marker = handle.read(1)
                    while marker == b"\xff":
                        marker = handle.read(1)
                    if not marker:
                        break
                    if marker in {b"\xc0", b"\xc1", b"\xc2", b"\xc3", b"\xc5", b"\xc6", b"\xc7", b"\xc9", b"\xca", b"\xcb", b"\xcd", b"\xce", b"\xcf"}:
                        length_data = handle.read(2)
                        if len(length_data) != 2:
                            break
                        segment_length = struct.unpack(">H", length_data)[0]
                        segment = handle.read(max(0, segment_length - 2))
                        if len(segment) >= 5:
                            height, width = struct.unpack(">HH", segment[1:5])
                            if width > 0 and height > 0:
                                return int(width), int(height)
                        break
                    if marker in {b"\xd8", b"\xd9"}:
                        continue
                    length_data = handle.read(2)
                    if len(length_data) != 2:
                        break
                    segment_length = struct.unpack(">H", length_data)[0]
                    if segment_length < 2:
                        break
                    handle.seek(segment_length - 2, os.SEEK_CUR)
            elif suffix == ".pgm":
                tokens: list[bytes] = []
                while len(tokens) < 3:
                    line = handle.readline()
                    if not line:
                        break
                    line = line.split(b"#", 1)[0]
                    tokens.extend(line.split())
                if len(tokens) >= 3 and tokens[0] in {b"P2", b"P5"}:
                    width = int(tokens[1])
                    height = int(tokens[2])
                    if width > 0 and height > 0:
                        return width, height
    except Exception:
        return None
    return None


def load_saved_map_meta(map_name: str) -> dict[str, Any]:
    clean_name = str(map_name or "").strip()
    if not clean_name:
        return {}
    yaml_name = clean_name if clean_name.endswith(".yaml") else f"{clean_name}.yaml"
    yaml_path: Path | None = None
    map_dir: Path | None = None
    for candidate_dir in MAP_LIBRARY_DIRS:
        candidate_path = candidate_dir / yaml_name
        if candidate_path.exists():
            yaml_path = candidate_path
            map_dir = candidate_dir
            break
    if yaml_path is None or map_dir is None:
        return {}

    meta: dict[str, Any] = {}
    image_path: Path | None = None
    try:
        text = yaml_path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        try:
            text = yaml_path.read_text(encoding="utf-8-sig")
        except Exception:
            return {}
    except Exception:
        return {}

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, raw_value = line.split(":", 1)
        key = key.strip()
        value = raw_value.strip().strip('"').strip("'")
        if key == "resolution":
            try:
                meta["resolution"] = float(value)
            except Exception:
                pass
        elif key == "origin":
            try:
                origin = ast.literal_eval(value)
            except Exception:
                origin = None
            if isinstance(origin, (list, tuple)) and len(origin) >= 2:
                try:
                    meta["origin_x"] = float(origin[0])
                    meta["origin_y"] = float(origin[1])
                except Exception:
                    pass
        elif key == "image" and value:
            image_path = (yaml_path.parent / value).resolve()

    if image_path is None:
        stem = yaml_path.stem
        for candidate in (yaml_path.with_suffix(".pgm"), yaml_path.with_suffix(".png"), map_dir / f"{stem}.pgm", map_dir / f"{stem}.png"):
            if candidate.exists():
                image_path = candidate
                break

    if image_path is not None:
        image_size = read_image_size(image_path)
        if image_size is not None:
            meta["width"], meta["height"] = image_size

    required_keys = {"width", "height", "resolution", "origin_x", "origin_y"}
    if not required_keys.issubset(meta):
        return {}
    return meta


def command_token_value() -> str:
    return str(os.environ.get("CAR_CLOUD_COMMAND_TOKEN", os.environ.get("CAR_CLOUD_UPLOAD_TOKEN", DEFAULT_UPLOAD_TOKEN)))


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def iso_is_stale(iso_value: str, timeout_s: float) -> bool:
    try:
        claimed_at = datetime.fromisoformat(iso_value)
    except Exception:
        return True
    return (utc_now() - claimed_at).total_seconds() > timeout_s


def normalize_command_payload(value: Any, *, max_string: int = 4000) -> Any:
    if isinstance(value, dict):
        normalized: dict[str, Any] = {}
        for key, item in list(value.items())[:20]:
            normalized[str(key)] = normalize_command_payload(item, max_string=max_string)
        return normalized
    if isinstance(value, list):
        return [normalize_command_payload(item, max_string=max_string) for item in value[:20]]
    if isinstance(value, str):
        return value[:max_string]
    if isinstance(value, (int, float, bool)) or value is None:
        return value
    return str(value)[:max_string]


class CloudStore:
    def __init__(self, state_path: Path, frame_dir: Path):
        self._state_path = state_path
        self._frame_dir = frame_dir
        self._lock = threading.Lock()
        self._state = self._load_state()

    def _empty_state(self) -> dict[str, Any]:
        now = utc_now_iso()
        return {
            "server": {
                "name": "car-cloud-platform",
                "version": "2026.04.20",
                "started_at": now,
                "updated_at": now,
            },
            "board": {
                "board_id": "",
                "label": "",
                "last_seen_at": "",
                "status": {},
                "ros_status": {},
                "nav_status": {},
                "telemetry": {},
                "map_meta": {},
                "cabinet_data": merged_cabinet_data({}),
                "assessment": evaluate_cabinet_data(merged_cabinet_data({})),
                "saved_maps": [],
                "nav_path": {"topic": "", "points": []},
                "robot_pose": {},
                "goal_pose": {},
                "cruise_status": {},
                "logs": [],
            },
            "frames": {},
            "saved_map_previews": {},
            "ingest": {
                "state_upload_count": 0,
                "frame_upload_count": 0,
                "last_state_upload_at": "",
                "last_frame_upload_at": "",
            },
            "commands": {
                "next_seq": 1,
                "queue": [],
                "history": [],
                "last_poll_at": "",
                "last_poll_board_id": "",
            },
            "teleop": empty_teleop_state(),
            "recognition": {
                "latest": {},
                "history": [],
                "lamp_detection_memory": {},
                "upload_count": 0,
                "last_upload_at": "",
            },
        }

    def _load_state(self) -> dict[str, Any]:
        if not self._state_path.exists():
            return self._empty_state()
        try:
            payload = json.loads(self._state_path.read_text(encoding="utf-8"))
        except Exception:
            return self._empty_state()
        merged = self._empty_state()
        deep_merge(merged, payload if isinstance(payload, dict) else {})
        board = merged.setdefault("board", {})
        board["cabinet_data"] = merged_cabinet_data(board.get("cabinet_data", {}))
        board["assessment"] = evaluate_cabinet_data(
            board.get("cabinet_data", {}),
            last_seen_at=str(board.get("last_seen_at") or ""),
            previous_assessment=board.get("assessment", {}) if isinstance(board.get("assessment"), dict) else {},
        )
        return merged

    def _persist_locked(self) -> None:
        self._state["server"]["updated_at"] = utc_now_iso()
        tmp_path = self._state_path.with_suffix(".tmp")
        tmp_path.write_text(json.dumps(self._state, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp_path.replace(self._state_path)

    def _append_alarm_event_locked(self, previous: Any, current: dict[str, Any]) -> None:
        kind = assessment_event_type(previous if isinstance(previous, dict) else {}, current)
        if not kind:
            return
        event = {
            "type": kind,
            "level": current.get("level", 0),
            "label": current.get("label", "无警告"),
            "created_at": utc_now_iso(),
            "assessment": copy.deepcopy(current),
        }
        if isinstance(previous, dict):
            event["previous_level"] = previous.get("level", 0)
            event["previous_label"] = previous.get("label", "无警告")
        with ALARM_EVENTS_PATH.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(event, ensure_ascii=False) + "\n")

    def _refresh_assessment_locked(self) -> dict[str, Any]:
        board = self._state.setdefault("board", {})
        previous = copy.deepcopy(board.get("assessment", {}))
        assessment = evaluate_cabinet_data(
            merged_cabinet_data(board.get("cabinet_data", {})),
            last_seen_at=str(board.get("last_seen_at") or ""),
            previous_assessment=previous if isinstance(previous, dict) else {},
        )
        board["assessment"] = assessment
        self._append_alarm_event_locked(previous, assessment)
        return assessment

    def alarm_events(self, limit: int = 80) -> list[dict[str, Any]]:
        if not ALARM_EVENTS_PATH.exists():
            return []
        try:
            lines = ALARM_EVENTS_PATH.read_text(encoding="utf-8", errors="replace").splitlines()
        except Exception:
            return []
        events: list[dict[str, Any]] = []
        for line in reversed(lines[-max(1, min(limit, 500)):]):
            try:
                item = json.loads(line)
            except Exception:
                continue
            if isinstance(item, dict):
                events.append(item)
        return events

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            payload = copy.deepcopy(self._state)
            saved_maps = payload.get("board", {}).get("saved_maps")
            if isinstance(saved_maps, list):
                payload["board"]["saved_maps"] = self._augment_saved_maps_locked(saved_maps)
            payload["teleop"] = sanitize_teleop_state(payload.get("teleop", {}), board_id=payload.get("board", {}).get("board_id", ""))
            payload["teleop_presets"] = copy.deepcopy(TELEOP_SPEED_PRESETS)
        payload["server_time"] = utc_now_iso()
        return payload

    def _update_history_locked(self, command: dict[str, Any]) -> None:
        history = self._state.setdefault("commands", {}).setdefault("history", [])
        for index, item in enumerate(history):
            if item.get("id") == command.get("id"):
                history[index] = copy.deepcopy(command)
                break
        else:
            history.insert(0, copy.deepcopy(command))
        del history[COMMAND_HISTORY_LIMIT:]

    def enqueue_command(self, *, action: str, params: dict[str, Any], target_board_id: str) -> dict[str, Any]:
        if action not in ALLOWED_COMMAND_ACTIONS:
            raise RuntimeError(f"unsupported command action: {action}")
        now = utc_now_iso()
        with self._lock:
            commands = self._state.setdefault("commands", {})
            next_seq = int(commands.get("next_seq", 1))
            command = {
                "id": f"cmd-{next_seq:06d}",
                "action": action,
                "params": normalize_command_payload(params if isinstance(params, dict) else {}),
                "target_board_id": str(target_board_id or "").strip(),
                "status": "pending",
                "created_at": now,
                "updated_at": now,
                "claimed_at": "",
                "claimed_by": "",
                "result": {},
            }
            commands["next_seq"] = next_seq + 1
            commands.setdefault("queue", []).append(command)
            self._update_history_locked(command)
            self._persist_locked()
            return copy.deepcopy(command)

    def claim_next_command(self, *, board_id: str) -> dict[str, Any] | None:
        now = utc_now_iso()
        with self._lock:
            commands = self._state.setdefault("commands", {})
            commands["last_poll_at"] = now
            commands["last_poll_board_id"] = board_id
            for command in commands.setdefault("queue", []):
                target = str(command.get("target_board_id") or "").strip()
                if target and target != board_id:
                    continue
                status = str(command.get("status") or "")
                claimed_by = str(command.get("claimed_by") or "")
                claimed_at = str(command.get("claimed_at") or "")
                updated_at = str(command.get("updated_at") or "") or claimed_at
                stale_claim = status == "claimed" and iso_is_stale(claimed_at, COMMAND_CLAIM_TIMEOUT_S)
                stale_running = status == "running" and iso_is_stale(updated_at, COMMAND_CLAIM_TIMEOUT_S)
                if (
                    status == "pending"
                    or (status == "claimed" and (claimed_by == board_id or stale_claim))
                    or stale_running
                ):
                    command["status"] = "claimed"
                    command["claimed_by"] = board_id
                    command["claimed_at"] = now
                    command["updated_at"] = now
                    self._update_history_locked(command)
                    self._persist_locked()
                    return copy.deepcopy(command)
            self._persist_locked()
            return None

    def update_command_result(self, *, command_id: str, board_id: str, status: str, result: Any) -> dict[str, Any] | None:
        if status not in {"claimed", "running", "completed", "failed", "rejected"}:
            raise RuntimeError(f"unsupported command status: {status}")
        now = utc_now_iso()
        with self._lock:
            commands = self._state.setdefault("commands", {})
            queue = commands.setdefault("queue", [])
            target_command = None
            for command in queue:
                if command.get("id") == command_id:
                    target_command = command
                    break
            if target_command is None:
                for command in commands.setdefault("history", []):
                    if command.get("id") == command_id:
                        target_command = command
                        break
            if target_command is None:
                return None
            claimed_by = str(target_command.get("claimed_by") or "")
            if claimed_by and board_id and claimed_by != board_id:
                raise RuntimeError(f"command {command_id} is claimed by another board")

            target_command["status"] = status
            target_command["updated_at"] = now
            if board_id:
                target_command["claimed_by"] = board_id
            target_command["result"] = normalize_command_payload(result)
            self._update_history_locked(target_command)
            if status in {"completed", "failed", "rejected"}:
                commands["queue"] = [item for item in queue if item.get("id") != command_id]
            self._persist_locked()
            return copy.deepcopy(target_command)

    def update_teleop(
        self,
        *,
        board_id: str,
        controller_id: str,
        page_mode: str,
        enabled: bool,
        pressed_keys: list[Any],
        speed_level: Any,
        seq: Any,
        force: bool = False,
    ) -> dict[str, Any]:
        clean_board_id = str(board_id or "").strip()
        clean_controller_id = normalize_controller_id(controller_id)
        if not clean_board_id:
            raise RuntimeError("missing board_id")
        if not clean_controller_id:
            raise RuntimeError("missing controller_id")

        now = utc_now_iso()
        cleaned_keys = normalize_pressed_keys(pressed_keys)
        cleaned_speed = normalize_speed_level(speed_level)
        clean_page_mode = str(page_mode or "mapping").strip() or "mapping"
        try:
            seq_value = int(seq or 0)
        except Exception:
            seq_value = 0

        with self._lock:
            current = sanitize_teleop_state(self._state.get("teleop", {}), board_id=clean_board_id)
            owned_by_other = (
                current.get("live")
                and current.get("board_id") == clean_board_id
                and current.get("controller_id")
                and current.get("controller_id") != clean_controller_id
            )
            if owned_by_other and not force:
                raise RuntimeError("teleop is controlled by another page")

            if enabled:
                updated = sanitize_teleop_state(
                    {
                        "board_id": clean_board_id,
                        "controller_id": clean_controller_id,
                        "page_mode": clean_page_mode,
                        "enabled": True,
                        "pressed_keys": cleaned_keys,
                        "speed_level": cleaned_speed,
                        "seq": seq_value,
                        "updated_at": now,
                        "claimed_at": current.get("claimed_at") if current.get("controller_id") == clean_controller_id else now,
                        "released_at": "",
                        "message": "teleop active",
                    },
                    board_id=clean_board_id,
                )
            else:
                updated = empty_teleop_state()
                updated.update(
                    {
                        "board_id": clean_board_id,
                        "controller_id": clean_controller_id,
                        "page_mode": clean_page_mode,
                        "updated_at": now,
                        "released_at": now,
                        "seq": seq_value,
                        "message": "teleop released",
                    }
                )
                updated = sanitize_teleop_state(updated, board_id=clean_board_id)

            self._state["teleop"] = updated
            self._persist_locked()
            return copy.deepcopy(updated)

    def teleop_for_board(self, *, board_id: str) -> dict[str, Any]:
        clean_board_id = str(board_id or "").strip()
        with self._lock:
            teleop = sanitize_teleop_state(self._state.get("teleop", {}), board_id=clean_board_id)
            return copy.deepcopy(teleop)

    def update_state(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise RuntimeError("state payload must be a JSON object")
        board_id = str(payload.get("board_id") or payload.get("board", {}).get("board_id") or "").strip()
        board_label = str(payload.get("board_label") or payload.get("board", {}).get("label") or "").strip()

        update_payload = {
            "board": {
                "board_id": board_id,
                "label": board_label,
                "last_seen_at": utc_now_iso(),
            }
        }
        board_payload = payload.get("board", {}) if isinstance(payload.get("board"), dict) else {}
        deep_merge(update_payload, {"board": board_payload})
        if isinstance(payload.get("status"), dict):
            update_payload["board"]["status"] = payload["status"]
        if isinstance(payload.get("ros_status"), dict):
            update_payload["board"]["ros_status"] = payload["ros_status"]
        if isinstance(payload.get("nav_status"), dict):
            update_payload["board"]["nav_status"] = payload["nav_status"]
        if isinstance(payload.get("telemetry"), dict):
            update_payload["board"]["telemetry"] = payload["telemetry"]
        if isinstance(payload.get("map_meta"), dict):
            update_payload["board"]["map_meta"] = payload["map_meta"]
        if isinstance(board_payload.get("cabinet_data"), dict):
            update_payload["board"]["cabinet_data"] = merged_cabinet_data(board_payload["cabinet_data"])
        if isinstance(payload.get("cabinet_data"), dict):
            update_payload["board"]["cabinet_data"] = merged_cabinet_data(payload["cabinet_data"])
        telemetry = payload.get("telemetry")
        if isinstance(telemetry, dict) and isinstance(telemetry.get("cabinet_data"), dict):
            update_payload["board"]["cabinet_data"] = merged_cabinet_data(telemetry["cabinet_data"])
        if isinstance(payload.get("nav_path"), dict):
            update_payload["board"]["nav_path"] = payload["nav_path"]
        if isinstance(payload.get("robot_pose"), dict):
            update_payload["board"]["robot_pose"] = payload["robot_pose"]
        if isinstance(payload.get("goal_pose"), dict):
            update_payload["board"]["goal_pose"] = payload["goal_pose"]
        if isinstance(payload.get("cruise_status"), dict):
            update_payload["board"]["cruise_status"] = payload["cruise_status"]
        if isinstance(payload.get("logs"), list):
            update_payload["board"]["logs"] = payload["logs"][-200:]

        with self._lock:
            saved_maps = update_payload.get("board", {}).get("saved_maps")
            if isinstance(saved_maps, list):
                update_payload["board"]["saved_maps"] = self._augment_saved_maps_locked(saved_maps)
            deep_merge(self._state, update_payload)
            self._refresh_assessment_locked()
            self._state["ingest"]["state_upload_count"] = int(self._state["ingest"].get("state_upload_count", 0)) + 1
            self._state["ingest"]["last_state_upload_at"] = utc_now_iso()
            self._persist_locked()
            return copy.deepcopy(self._state)

    def _augment_saved_maps_locked(self, items: list[Any]) -> list[dict[str, Any]]:
        previews = self._state.get("saved_map_previews", {})
        result: list[dict[str, Any]] = []
        for item in items:
            if not isinstance(item, dict):
                continue
            name = str(item.get("name") or "").strip()
            yaml_name = str(item.get("yaml") or "").strip()
            merged = copy.deepcopy(item)
            preview = previews.get(name, {}) if name else {}
            if preview:
                merged["preview_url"] = preview.get("url", "")
                merged["preview_uploaded_at"] = preview.get("uploaded_at", "")
            if not isinstance(merged.get("preview_meta"), dict) or not merged.get("preview_meta"):
                preview_meta = load_saved_map_meta(name or yaml_name)
                if preview_meta:
                    merged["preview_meta"] = preview_meta
            result.append(merged)
        return result

    def save_frame(self, kind: str, upload) -> dict[str, Any]:
        if kind not in ALLOWED_FRAME_KINDS:
            raise RuntimeError(f"unsupported frame kind: {kind}")
        if upload is None or not getattr(upload, "filename", ""):
            raise RuntimeError("missing upload file")
        suffix = Path(upload.filename).suffix.lower() or ".bin"
        if suffix not in {".jpg", ".jpeg", ".png", ".webp", ".bmp"}:
            content_type = str(getattr(upload, "mimetype", "") or "")
            guessed = mimetypes.guess_extension(content_type) or ".bin"
            suffix = guessed.lower()
        filename = f"{kind}_{uuid4().hex[:12]}{suffix}"
        path = self._frame_dir / filename
        upload.save(path)
        guessed_type = mimetypes.guess_type(str(path))[0] or getattr(upload, "mimetype", "") or "application/octet-stream"
        frame_info = {
            "kind": kind,
            "filename": filename,
            "url": f"/api/cloud/frame/{kind}",
            "uploaded_at": utc_now_iso(),
            "content_type": guessed_type,
            "size_bytes": path.stat().st_size,
        }
        with self._lock:
            stale = self._state["frames"].get(kind, {}).get("filename")
            self._state["frames"][kind] = frame_info
            self._state["ingest"]["frame_upload_count"] = int(self._state["ingest"].get("frame_upload_count", 0)) + 1
            self._state["ingest"]["last_frame_upload_at"] = frame_info["uploaded_at"]
            self._persist_locked()
        if stale and stale != filename:
            stale_path = self._frame_dir / stale
            if stale_path.exists():
                try:
                    stale_path.unlink()
                except OSError:
                    pass
        return frame_info

    def frame_path(self, kind: str) -> Path | None:
        with self._lock:
            filename = self._state.get("frames", {}).get(kind, {}).get("filename")
        if not filename:
            return None
        path = self._frame_dir / filename
        return path if path.exists() else None

    def save_saved_map_preview(self, name: str, upload) -> dict[str, Any]:
        clean_name = str(name or "").strip()
        if not clean_name:
            raise RuntimeError("missing saved map name")
        if upload is None or not getattr(upload, "filename", ""):
            raise RuntimeError("missing preview file")

        suffix = Path(upload.filename).suffix.lower() or ".png"
        if suffix not in {".jpg", ".jpeg", ".png", ".webp", ".bmp"}:
            suffix = ".png"

        safe_name = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in clean_name)
        filename = f"saved_map_{safe_name}_{uuid4().hex[:10]}{suffix}"
        path = self._frame_dir / filename
        upload.save(path)
        preview_info = {
            "name": clean_name,
            "filename": filename,
            "url": f"/api/cloud/saved-map-preview/{quote(clean_name, safe='')}",
            "uploaded_at": utc_now_iso(),
            "content_type": mimetypes.guess_type(str(path))[0] or getattr(upload, "mimetype", "") or "image/png",
            "size_bytes": path.stat().st_size,
        }
        with self._lock:
            stale = self._state.setdefault("saved_map_previews", {}).get(clean_name, {}).get("filename")
            self._state["saved_map_previews"][clean_name] = preview_info
            self._persist_locked()
        if stale and stale != filename:
            stale_path = self._frame_dir / stale
            if stale_path.exists():
                try:
                    stale_path.unlink()
                except OSError:
                    pass
        return preview_info

    def saved_map_preview_path(self, name: str) -> Path | None:
        with self._lock:
            filename = self._state.get("saved_map_previews", {}).get(name, {}).get("filename")
        if not filename:
            return None
        path = self._frame_dir / filename
        return path if path.exists() else None

    def save_recognition(self, payload: dict[str, Any], raw_upload, annotated_upload) -> dict[str, Any]:
        if raw_upload is None or not getattr(raw_upload, "filename", ""):
            raise RuntimeError("missing raw image")
        if annotated_upload is None or not getattr(annotated_upload, "filename", ""):
            raise RuntimeError("missing annotated image")

        now = utc_now_iso()
        item_id = f"rec_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}_{uuid4().hex[:8]}"
        item_dir = RECOGNITION_DIR / item_id
        item_dir.mkdir(parents=True, exist_ok=True)

        def save_image(upload, stem: str) -> dict[str, Any]:
            suffix = Path(upload.filename).suffix.lower() or ".jpg"
            if suffix not in {".jpg", ".jpeg", ".png", ".webp", ".bmp"}:
                suffix = ".jpg"
            filename = f"{item_id}_{stem}{suffix}"
            path = item_dir / filename
            upload.save(path)
            return {
                "filename": f"{item_id}/{filename}",
                "url": f"/api/cloud/recognition/image/{quote(item_id, safe='')}/{quote(filename, safe='')}",
                "content_type": mimetypes.guess_type(str(path))[0] or getattr(upload, "mimetype", "") or "image/jpeg",
                "size_bytes": path.stat().st_size,
            }

        raw_info = save_image(raw_upload, "raw")
        annotated_info = save_image(annotated_upload, "det")
        raw_image_size = read_image_size(RECOGNITION_DIR / raw_info["filename"])
        clean_payload = normalize_command_payload(payload if isinstance(payload, dict) else {}, max_string=8000)
        detections = clean_payload.get("detections", []) if isinstance(clean_payload, dict) else []
        if not isinstance(detections, list):
            detections = []
        meter_readings = clean_payload.get("meter_readings", []) if isinstance(clean_payload, dict) else []
        if not isinstance(meter_readings, list):
            meter_readings = []
        digit_detections = clean_payload.get("digit_detections", []) if isinstance(clean_payload, dict) else []
        if not isinstance(digit_detections, list):
            digit_detections = []
        digit_readings = clean_payload.get("digit_readings", {}) if isinstance(clean_payload, dict) else {}
        if not isinstance(digit_readings, dict):
            digit_readings = {}
        cabinet_update: dict[str, Any] = {}
        payload_cabinet_data = clean_payload.get("cabinet_data", {}) if isinstance(clean_payload, dict) else {}
        if isinstance(payload_cabinet_data, dict):
            deep_merge(cabinet_update, copy.deepcopy(payload_cabinet_data))
        deep_merge(cabinet_update, cabinet_data_from_meter_readings(meter_readings))
        item = {
            "id": item_id,
            "uploaded_at": now,
            "board_id": str(clean_payload.get("board_id") or "").strip() if isinstance(clean_payload, dict) else "",
            "board_label": str(clean_payload.get("board_label") or "").strip() if isinstance(clean_payload, dict) else "",
            "backend": str(clean_payload.get("backend") or "") if isinstance(clean_payload, dict) else "",
            "inference_ms": clean_payload.get("inference_ms", "") if isinstance(clean_payload, dict) else "",
            "classes": clean_payload.get("classes", []) if isinstance(clean_payload, dict) else [],
            "detections": detections,
            "detection_count": len(detections),
            "meter_readings": meter_readings,
            "digit_detections": digit_detections,
            "digit_readings": digit_readings,
            "image_size": {"width": raw_image_size[0], "height": raw_image_size[1]} if raw_image_size else {},
            "cabinet_update": copy.deepcopy(cabinet_update),
            "raw_image": raw_info,
            "annotated_image": annotated_info,
            "source": clean_payload,
        }
        metadata_path = item_dir / f"{item_id}.json"
        metadata_path.write_text(json.dumps(item, ensure_ascii=False, indent=2), encoding="utf-8")

        with self._lock:
            recognition = self._state.setdefault("recognition", {})
            history = recognition.setdefault("history", [])
            history.insert(0, copy.deepcopy(item))
            stale_items = history[RECOGNITION_HISTORY_LIMIT:]
            del history[RECOGNITION_HISTORY_LIMIT:]
            recognition["latest"] = copy.deepcopy(item)
            recognition["upload_count"] = int(recognition.get("upload_count", 0) or 0) + 1
            recognition["last_upload_at"] = now
            lamp_memory = update_lamp_detection_memory(
                recognition.get("lamp_detection_memory", {}),
                detections,
                image_size=raw_image_size,
                item_id=item_id,
                uploaded_at=now,
            )
            recognition["lamp_detection_memory"] = copy.deepcopy(lamp_memory)
            lamp_update = cabinet_data_from_lamp_memory(lamp_memory)
            if lamp_update:
                deep_merge(cabinet_update, lamp_update)
                recognition["cabinet_lamp_state"] = copy.deepcopy(lamp_update)
            if cabinet_update:
                current_data = merged_cabinet_data(self._state.get("board", {}).get("cabinet_data", {}))
                deep_merge(current_data, cabinet_update)
                self._state.setdefault("board", {})["cabinet_data"] = merged_cabinet_data(current_data)
                self._refresh_assessment_locked()
            self._persist_locked()

        for stale in stale_items:
            stale_id = str(stale.get("id") or "").strip()
            if not stale_id:
                continue
            stale_dir = RECOGNITION_DIR / stale_id
            try:
                for child in stale_dir.glob("*"):
                    child.unlink()
                stale_dir.rmdir()
            except OSError:
                pass
        return copy.deepcopy(item)

    def recognition_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._state.get("recognition", {}))

    def recognition_image_path(self, item_id: str, filename: str) -> Path | None:
        clean_id = Path(str(item_id or "").strip()).name
        clean_name = Path(str(filename or "").strip()).name
        if not clean_id or not clean_name:
            return None
        path = RECOGNITION_DIR / clean_id / clean_name
        return path if path.exists() else None


store = CloudStore(STATE_PATH, FRAME_DIR)


def json_ok(**kwargs):
    payload = {"ok": True}
    payload.update(kwargs)
    return jsonify(payload)


def json_error(message: str, status_code: int = 400, **kwargs):
    payload = {"ok": False, "message": message}
    payload.update(kwargs)
    response = jsonify(payload)
    response.status_code = status_code
    return response


def upload_authorized() -> bool:
    configured = str(os.environ.get("CAR_CLOUD_UPLOAD_TOKEN", DEFAULT_UPLOAD_TOKEN))
    supplied = request.headers.get("X-Upload-Token", "")
    return bool(supplied) and supplied == configured


def command_authorized() -> bool:
    supplied = request.headers.get("X-Command-Token", "")
    return bool(supplied) and supplied == command_token_value()


def disabled_teleop_state(board_id: str = "") -> dict[str, Any]:
    teleop = empty_teleop_state()
    teleop.update(
        {
            "board_id": str(board_id or "").strip(),
            "updated_at": utc_now_iso(),
            "message": "cloud teleop disabled",
        }
    )
    return teleop


@app.route("/ping")
def ping():
    return "pong", 200


@app.route("/")
def index():
    return render_template("menu.html")


@app.route("/mapping")
def mapping():
    return render_template("dashboard.html", page_mode="mapping", command_token=command_token_value())


@app.route("/navigation")
def navigation():
    return render_template("dashboard.html", page_mode="navigation", command_token=command_token_value())


@app.route("/recognition")
def recognition():
    return render_template("recognition.html", command_token=command_token_value())


@app.route("/sandbox/recognition")
def sandbox_recognition():
    return render_template("recognition.html", command_token=command_token_value(), sandbox=True)


@app.route("/data")
def data_display():
    return render_template("data_display.html")


@app.route("/sandbox/data")
def sandbox_data_display():
    return render_template("data_display.html", sandbox=True)


@app.route("/api/cloud/state")
def api_cloud_state():
    return jsonify(store.snapshot())


@app.route("/api/cloud/cabinet-data")
def api_cloud_cabinet_data():
    payload = store.snapshot()
    board = payload.get("board", {}) if isinstance(payload, dict) else {}
    cabinet_data = merged_cabinet_data(board.get("cabinet_data", {}))
    return jsonify(
        {
            "ok": True,
            "board_id": board.get("board_id", ""),
            "board_label": board.get("label", ""),
            "last_seen_at": board.get("last_seen_at", ""),
            "server_time": payload.get("server_time", ""),
            "data": cabinet_data,
            "assessment": board.get("assessment", {}),
        }
    )


@app.route("/api/cloud/alarm-events")
def api_cloud_alarm_events():
    try:
        limit = int(request.args.get("limit", 80) or 80)
    except Exception:
        limit = 80
    return jsonify({"ok": True, "events": store.alarm_events(limit)})


@app.route("/api/cloud/recognition/history")
def api_cloud_recognition_history():
    recognition_payload = store.recognition_snapshot()
    history = recognition_payload.get("history", [])
    if not isinstance(history, list):
        history = []
    try:
        limit = max(1, min(200, int(request.args.get("limit", "80"))))
    except Exception:
        limit = 80
    recognition_payload["history"] = history[:limit]
    return jsonify(recognition_payload)


@app.route("/api/cloud/recognition/image/<path:item_id>/<path:filename>")
def api_cloud_recognition_image(item_id: str, filename: str):
    path = store.recognition_image_path(item_id, filename)
    if path is None:
        return json_error("recognition image not found", 404)
    mimetype = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
    return send_file(path, mimetype=mimetype, download_name=path.name)


@app.route("/api/cloud/commands", methods=["POST"])
def api_cloud_commands():
    if not command_authorized():
        return json_error("unauthorized", 401)
    payload = request.get_json(force=True, silent=True)
    if payload is None or not isinstance(payload, dict):
        return json_error("invalid JSON payload")
    action = str(payload.get("action") or "").strip()
    if action not in ALLOWED_COMMAND_ACTIONS:
        return json_error("unsupported command action")
    params = payload.get("params", {})
    if params is None:
        params = {}
    if not isinstance(params, dict):
        return json_error("params must be an object")
    target_board_id = str(payload.get("target_board_id") or "").strip()
    try:
        command = store.enqueue_command(action=action, params=params, target_board_id=target_board_id)
    except RuntimeError as exc:
        return json_error(str(exc))
    return json_ok(message="command queued", command=command)


@app.route("/api/cloud/teleop", methods=["POST"])
def api_cloud_teleop():
    if not command_authorized():
        return json_error("unauthorized", 401)
    payload = request.get_json(force=True, silent=True)
    if payload is None or not isinstance(payload, dict):
        return json_error("invalid JSON payload")
    board_id = str(payload.get("board_id") or "").strip()
    return json_ok(message="cloud teleop disabled", teleop=disabled_teleop_state(board_id))


@app.route("/api/board/commands/next")
def api_board_commands_next():
    if not upload_authorized():
        return json_error("unauthorized", 401)
    board_id = str(request.args.get("board_id") or "").strip()
    if not board_id:
        return json_error("missing board_id")
    command = store.claim_next_command(board_id=board_id)
    return json_ok(command=command)


@app.route("/api/board/teleop")
def api_board_teleop():
    if not upload_authorized():
        return json_error("unauthorized", 401)
    board_id = str(request.args.get("board_id") or "").strip()
    if not board_id:
        return json_error("missing board_id")
    return json_ok(teleop=disabled_teleop_state(board_id))


@app.route("/api/board/commands/<command_id>/result", methods=["POST"])
def api_board_command_result(command_id: str):
    if not upload_authorized():
        return json_error("unauthorized", 401)
    payload = request.get_json(force=True, silent=True)
    if payload is None or not isinstance(payload, dict):
        return json_error("invalid JSON payload")
    board_id = str(payload.get("board_id") or "").strip()
    status = str(payload.get("status") or "").strip()
    result = payload.get("result", {})
    try:
        command = store.update_command_result(command_id=command_id, board_id=board_id, status=status, result=result)
    except RuntimeError as exc:
        return json_error(str(exc))
    if command is None:
        return json_error("command not found", 404)
    return json_ok(message="command updated", command=command)


@app.route("/api/cloud/frame/<kind>")
def api_cloud_frame(kind: str):
    path = store.frame_path(kind)
    if path is None:
        return json_error(f"frame not found: {kind}", 404)
    mimetype = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
    return send_file(path, mimetype=mimetype, download_name=path.name)


@app.route("/api/cloud/saved-map-preview/<path:name>")
def api_cloud_saved_map_preview(name: str):
    path = store.saved_map_preview_path(name)
    if path is None:
        return json_error(f"saved map preview not found: {name}", 404)
    mimetype = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
    return send_file(path, mimetype=mimetype, download_name=path.name)


@app.route("/api/upload/state", methods=["POST"])
def api_upload_state():
    if not upload_authorized():
        return json_error("unauthorized", 401)
    payload = request.get_json(force=True, silent=True)
    if payload is None:
        return json_error("invalid JSON payload")
    state = store.update_state(payload)
    return json_ok(
        message="state uploaded",
        board_id=state["board"].get("board_id", ""),
        assessment=state["board"].get("assessment", {}),
    )


@app.route("/api/upload/frame/<kind>", methods=["POST"])
def api_upload_frame(kind: str):
    if not upload_authorized():
        return json_error("unauthorized", 401)
    upload = request.files.get("file")
    if upload is None:
        return json_error("missing file")
    frame_info = store.save_frame(kind, upload)
    return json_ok(message=f"{kind} frame uploaded", frame=frame_info)


@app.route("/api/upload/saved-map-preview/<path:name>", methods=["POST"])
def api_upload_saved_map_preview(name: str):
    if not upload_authorized():
        return json_error("unauthorized", 401)
    upload = request.files.get("file")
    if upload is None:
        return json_error("missing file")
    preview = store.save_saved_map_preview(name, upload)
    return json_ok(message=f"saved map preview uploaded: {name}", preview=preview)


@app.route("/api/upload/snapshot", methods=["POST"])
def api_upload_snapshot():
    if not upload_authorized():
        return json_error("unauthorized", 401)
    payload_raw = request.form.get("payload", "").strip()
    payload: dict[str, Any] = {}
    if payload_raw:
        try:
            payload = json.loads(payload_raw)
        except json.JSONDecodeError as exc:
            return json_error(f"invalid payload JSON: {exc}")
    frame_results: dict[str, Any] = {}
    for kind in ALLOWED_FRAME_KINDS:
        upload = request.files.get(f"{kind}_frame")
        if upload is not None and getattr(upload, "filename", ""):
            frame_results[kind] = store.save_frame(kind, upload)
    state = store.update_state(payload)
    return json_ok(
        message="snapshot uploaded",
        board_id=state["board"].get("board_id", ""),
        uploaded_frames=frame_results,
    )


@app.route("/api/upload/recognition", methods=["POST"])
def api_upload_recognition():
    if not upload_authorized():
        return json_error("unauthorized", 401)
    payload_raw = request.form.get("payload", "").strip()
    payload: dict[str, Any] = {}
    if payload_raw:
        try:
            loaded = json.loads(payload_raw)
            payload = loaded if isinstance(loaded, dict) else {}
        except json.JSONDecodeError as exc:
            return json_error(f"invalid payload JSON: {exc}")
    try:
        item = store.save_recognition(
            payload,
            request.files.get("raw_image"),
            request.files.get("annotated_image"),
        )
    except RuntimeError as exc:
        return json_error(str(exc))
    return json_ok(message="recognition uploaded", recognition=item)


class ThreadingWSGIServer(ThreadingMixIn, WSGIServer):
    daemon_threads = True


if __name__ == "__main__":
    host = os.environ.get("CAR_CLOUD_HOST", "0.0.0.0")
    port = int(os.environ.get("CAR_CLOUD_PORT", "8765"))
    with make_server(host, port, app, server_class=ThreadingWSGIServer, handler_class=WSGIRequestHandler) as httpd:
        httpd.serve_forever()
