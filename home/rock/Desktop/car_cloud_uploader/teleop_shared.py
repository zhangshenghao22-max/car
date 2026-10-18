from __future__ import annotations

from datetime import datetime, timedelta, timezone
from typing import Any

TELEOP_ALLOWED_KEYS = ("w", "a", "s", "d", "q", "e", "x", " ")
TELEOP_DEFAULT_SPEED_LEVEL = 2
TELEOP_SESSION_TIMEOUT_S = 1.0
TELEOP_SPEED_PRESETS: dict[int, dict[str, float]] = {
    1: {"linear": 0.15, "strafe": 0.15, "angular": 0.60},
    2: {"linear": 0.30, "strafe": 0.30, "angular": 0.90},
    3: {"linear": 0.45, "strafe": 0.45, "angular": 1.20},
}


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def utc_now_iso() -> str:
    return utc_now().replace(microsecond=0).isoformat()


def parse_iso_datetime(value: Any) -> datetime | None:
    text = str(value or "").strip()
    if not text:
        return None
    try:
        parsed = datetime.fromisoformat(text)
    except Exception:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed


def normalize_controller_id(value: Any) -> str:
    text = str(value or "").strip()
    return text[:80]


def normalize_speed_level(value: Any) -> int:
    try:
        level = int(value)
    except Exception:
        level = TELEOP_DEFAULT_SPEED_LEVEL
    return level if level in TELEOP_SPEED_PRESETS else TELEOP_DEFAULT_SPEED_LEVEL


def normalize_pressed_keys(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    normalized: list[str] = []
    seen: set[str] = set()
    for item in value:
        key = str(item or "").lower()[:1] if str(item or "") != " " else " "
        if item == " ":
            key = " "
        elif not key:
            continue
        if key not in TELEOP_ALLOWED_KEYS or key in seen:
            continue
        seen.add(key)
        normalized.append(key)
    return normalized


def zero_twist_dict() -> dict[str, float]:
    return {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}


def teleop_twist_from_pressed_keys(pressed_keys: Any, speed_level: Any) -> dict[str, float]:
    keys = set(normalize_pressed_keys(pressed_keys))
    if "x" in keys or " " in keys:
        return zero_twist_dict()

    preset = TELEOP_SPEED_PRESETS[normalize_speed_level(speed_level)]
    forward = 1 if "w" in keys else 0
    backward = 1 if "s" in keys else 0
    left = 1 if "a" in keys else 0
    right = 1 if "d" in keys else 0
    rotate_left = 1 if "q" in keys else 0
    rotate_right = 1 if "e" in keys else 0

    return {
        "linear_x": float((forward - backward) * preset["linear"]),
        "linear_y": float((left - right) * preset["strafe"]),
        "angular_z": float((rotate_left - rotate_right) * preset["angular"]),
    }


def empty_teleop_state() -> dict[str, Any]:
    return {
        "board_id": "",
        "controller_id": "",
        "page_mode": "mapping",
        "enabled": False,
        "status": "idle",
        "pressed_keys": [],
        "speed_level": TELEOP_DEFAULT_SPEED_LEVEL,
        "seq": 0,
        "updated_at": "",
        "claimed_at": "",
        "released_at": "",
        "expires_at": "",
        "live": False,
        "message": "",
        "twist": zero_twist_dict(),
    }


def teleop_session_is_live(session: dict[str, Any] | None, *, now: datetime | None = None) -> bool:
    if not isinstance(session, dict):
        return False
    if not bool(session.get("enabled")):
        return False
    updated_at = parse_iso_datetime(session.get("updated_at"))
    if updated_at is None:
        return False
    current = now or utc_now()
    return (current - updated_at).total_seconds() <= TELEOP_SESSION_TIMEOUT_S


def sanitize_teleop_state(session: dict[str, Any] | None, *, board_id: str = "") -> dict[str, Any]:
    current = utc_now()
    clean = empty_teleop_state()
    if isinstance(session, dict):
        clean.update(session)
    clean["board_id"] = str(clean.get("board_id") or "").strip()
    clean["controller_id"] = normalize_controller_id(clean.get("controller_id"))
    clean["page_mode"] = str(clean.get("page_mode") or "mapping").strip() or "mapping"
    clean["pressed_keys"] = normalize_pressed_keys(clean.get("pressed_keys"))
    clean["speed_level"] = normalize_speed_level(clean.get("speed_level"))
    clean["twist"] = teleop_twist_from_pressed_keys(clean.get("pressed_keys"), clean.get("speed_level"))
    clean["live"] = teleop_session_is_live(clean, now=current)
    clean["expires_at"] = (
        (parse_iso_datetime(clean.get("updated_at")) or current) + timedelta(seconds=TELEOP_SESSION_TIMEOUT_S)
    ).replace(microsecond=0).isoformat() if clean.get("updated_at") else ""

    board_match = not board_id or clean["board_id"] == str(board_id or "").strip()
    if not clean["live"] or not board_match:
        released_at = clean.get("released_at") or clean.get("updated_at") or ""
        message = clean.get("message") or ("teleop session expired" if clean.get("enabled") else "")
        clean.update(
            {
                "enabled": False,
                "status": "idle",
                "pressed_keys": [],
                "released_at": released_at,
                "live": False,
                "message": message,
                "twist": zero_twist_dict(),
            }
        )
    elif any(abs(float(clean["twist"][axis])) > 1e-9 for axis in ("linear_x", "linear_y", "angular_z")):
        clean["status"] = "driving"
    else:
        clean["status"] = "armed"
    return clean
