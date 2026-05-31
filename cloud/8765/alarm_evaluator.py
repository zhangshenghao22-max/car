from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

LEVEL_NONE = 0
LEVEL_THREE = 3
LEVEL_TWO = 2
LEVEL_ONE = 1
LEVEL_PRIORITY = {LEVEL_NONE: 0, LEVEL_THREE: 1, LEVEL_TWO: 2, LEVEL_ONE: 3}
LEVEL_LABELS = {
    LEVEL_NONE: "\u65e0\u8b66\u544a",
    LEVEL_THREE: "\u4e09\u7ea7\u8b66\u544a",
    LEVEL_TWO: "\u4e8c\u7ea7\u8b66\u544a",
    LEVEL_ONE: "\u4e00\u7ea7\u8b66\u544a",
}

TEMP_THRESHOLDS = [(LEVEL_ONE, 60.0), (LEVEL_TWO, 45.0), (LEVEL_THREE, 35.0)]
GAS_THRESHOLDS = [(LEVEL_ONE, 200.0), (LEVEL_TWO, 100.0), (LEVEL_THREE, 50.0)]
HUMIDITY_THRESHOLDS = [(LEVEL_ONE, 95.0), (LEVEL_TWO, 85.0), (LEVEL_THREE, 75.0)]
SOUND_THRESHOLDS = [(LEVEL_ONE, 95.0), (LEVEL_TWO, 85.0), (LEVEL_THREE, 70.0)]
CURRENT_THRESHOLDS = [(LEVEL_ONE, 1.0), (LEVEL_TWO, 0.9), (LEVEL_THREE, 0.8)]
VOLTAGE_RANGES = [(LEVEL_ONE, 170.0, 270.0), (LEVEL_TWO, 180.0, 260.0), (LEVEL_THREE, 190.0, 250.0)]
STALE_THRESHOLDS = [(LEVEL_ONE, 300.0), (LEVEL_TWO, 120.0), (LEVEL_THREE, 30.0)]


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_iso(value: Any) -> datetime | None:
    text = str(value or "").strip()
    if not text:
        return None
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    try:
        parsed = datetime.fromisoformat(text)
    except Exception:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed


def age_seconds(iso_value: Any, *, now: datetime) -> float | None:
    parsed = parse_iso(iso_value)
    if parsed is None:
        return None
    return max(0.0, (now - parsed).total_seconds())


def finite_number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if number != number or number in {float("inf"), float("-inf")}:
        return None
    return number


def metric_value(metric: Any, *aliases: str) -> float | None:
    if not isinstance(metric, dict):
        return None
    for key in ("value", *aliases):
        number = finite_number(metric.get(key))
        if number is not None:
            return number
    return None


def evaluate_min(value: float | None, thresholds: list[tuple[int, float]]) -> tuple[int, float | None]:
    if value is None:
        return LEVEL_NONE, None
    for level, threshold in thresholds:
        if value >= threshold:
            return level, threshold
    return LEVEL_NONE, thresholds[-1][1]


def evaluate_voltage(value: float | None) -> tuple[int, str]:
    if value is None:
        return LEVEL_NONE, ""
    for level, low, high in VOLTAGE_RANGES:
        if value <= low or value >= high:
            return level, f"\u5b89\u5168\u8303\u56f4 {low:g}-{high:g} V"
    return LEVEL_NONE, "\u4e09\u7ea7\u9608\u503c <=190 V \u6216 >=250 V"


def level_label(level: int) -> str:
    return LEVEL_LABELS.get(level, LEVEL_LABELS[LEVEL_NONE])


def better_level(left: int, right: int) -> int:
    return left if LEVEL_PRIORITY.get(left, 0) >= LEVEL_PRIORITY.get(right, 0) else right


def unit_from(metric: Any, fallback: str) -> str:
    if isinstance(metric, dict):
        return str(metric.get("unit") or fallback)
    return fallback


def add_threshold_item(
    items: list[dict[str, Any]],
    *,
    key: str,
    name: str,
    value: Any,
    unit: str,
    level: int,
    threshold_text: str,
    digits: int = 1,
) -> None:
    if level <= LEVEL_NONE:
        return
    if isinstance(value, (int, float)):
        value_text = f"{float(value):.{digits}f} {unit}".strip()
    elif value is None:
        value_text = "\u672a\u4e0a\u62a5"
    else:
        value_text = str(value)
    items.append(
        {
            "key": key,
            "name": name,
            "value": value,
            "unit": unit,
            "level": level,
            "label": level_label(level),
            "threshold": threshold_text,
            "message": f"{name}\u8fbe\u5230 {value_text}\uff0c{threshold_text}",
        }
    )


def add_min_item(
    items: list[dict[str, Any]],
    *,
    key: str,
    name: str,
    value: float | None,
    unit: str,
    thresholds: list[tuple[int, float]],
    digits: int = 1,
) -> None:
    level, threshold = evaluate_min(value, thresholds)
    if level <= LEVEL_NONE:
        return
    add_threshold_item(
        items,
        key=key,
        name=name,
        value=value,
        unit=unit,
        level=level,
        threshold_text=f"\u8d85\u8fc7{level_label(level)}\u9608\u503c {threshold:g} {unit}".strip(),
        digits=digits,
    )


def truthy_state(value: Any) -> bool:
    if value is True:
        return True
    raw = str(value or "").strip().lower()
    return raw in {"1", "true", "on", "yes", "active", "open", "opened", "\u6709\u7535", "\u544a\u8b66", "\u62a5\u8b66", "\u6253\u5f00", "\u5f00\u542f"}


def evaluate_cabinet_data(cabinet_data: Any, *, last_seen_at: str = "", previous_assessment: dict[str, Any] | None = None, now_iso: str | None = None) -> dict[str, Any]:
    now = parse_iso(now_iso) or datetime.now(timezone.utc)
    now_text = now.isoformat()
    data = cabinet_data if isinstance(cabinet_data, dict) else {}
    env = data.get("environment") if isinstance(data.get("environment"), dict) else {}
    items: list[dict[str, Any]] = []

    voltage = data.get("voltage") if isinstance(data.get("voltage"), dict) else {}
    voltage_value = metric_value(voltage)
    voltage_level, voltage_threshold = evaluate_voltage(voltage_value)
    add_threshold_item(items, key="voltage", name="\u7535\u538b\u8868", value=voltage_value, unit=unit_from(voltage, "V"), level=voltage_level, threshold_text=voltage_threshold, digits=1)

    current = data.get("current") if isinstance(data.get("current"), dict) else {}
    add_min_item(items, key="current", name="\u7535\u6d41\u8868", value=metric_value(current), unit=unit_from(current, "A"), thresholds=CURRENT_THRESHOLDS, digits=2)

    temp_ctrl = data.get("temperature_controller") if isinstance(data.get("temperature_controller"), dict) else {}
    add_min_item(items, key="temperature_controller.pv", name="\u6e29\u63a7\u8868 PV", value=metric_value(temp_ctrl, "pv"), unit=unit_from(temp_ctrl, "\u2103"), thresholds=TEMP_THRESHOLDS, digits=1)

    add_min_item(items, key="environment.temperature", name="\u67dc\u5185\u6e29\u5ea6", value=metric_value(env.get("temperature")), unit=unit_from(env.get("temperature"), "\u2103"), thresholds=TEMP_THRESHOLDS, digits=1)
    add_min_item(items, key="environment.infrared_temperature", name="\u7ea2\u5916\u6d4b\u6e29\u6a21\u5757", value=metric_value(env.get("infrared_temperature"), "temperature", "target_temperature", "object_temperature", "surface_temperature"), unit=unit_from(env.get("infrared_temperature"), "\u2103"), thresholds=TEMP_THRESHOLDS, digits=1)
    add_min_item(items, key="environment.smoke", name="\u67dc\u5185\u70df\u96fe\u6d53\u5ea6", value=metric_value(env.get("smoke")), unit=unit_from(env.get("smoke"), "ppm"), thresholds=GAS_THRESHOLDS, digits=1)
    add_min_item(items, key="environment.hydrogen", name="\u6c22\u6c14\u6d53\u5ea6", value=metric_value(env.get("hydrogen")), unit=unit_from(env.get("hydrogen"), "ppm"), thresholds=GAS_THRESHOLDS, digits=1)
    add_min_item(items, key="environment.carbon_monoxide", name="\u4e00\u6c27\u5316\u78b3\u6d53\u5ea6", value=metric_value(env.get("carbon_monoxide")), unit=unit_from(env.get("carbon_monoxide"), "ppm"), thresholds=GAS_THRESHOLDS, digits=1)
    add_min_item(items, key="environment.humidity", name="\u6e7f\u5ea6", value=metric_value(env.get("humidity")), unit=unit_from(env.get("humidity"), "%RH"), thresholds=HUMIDITY_THRESHOLDS, digits=1)
    add_min_item(items, key="environment.sound_level", name="\u58f0\u97f3\u76d1\u6d4b\u6a21\u5757", value=metric_value(env.get("sound_level"), "db", "level", "relative_db", "dbfs", "rms_dbfs"), unit=unit_from(env.get("sound_level"), "dB"), thresholds=SOUND_THRESHOLDS, digits=1)

    warning = data.get("warning") if isinstance(data.get("warning"), dict) else {}
    if truthy_state(warning.get("high_voltage")):
        items.append({"key": "warning.high_voltage", "name": "\u5c0f\u5fc3\u6709\u7535", "value": warning.get("high_voltage"), "unit": "", "level": LEVEL_TWO, "label": level_label(LEVEL_TWO), "threshold": "\u73b0\u573a\u9ad8\u538b\u8b66\u793a\u89e6\u53d1", "message": "\u5c0f\u5fc3\u6709\u7535\u8b66\u793a\u89e6\u53d1\uff0c\u9700\u8981\u590d\u6838\u67dc\u9762\u72b6\u6001"})

    door = data.get("door") if isinstance(data.get("door"), dict) else {}
    if truthy_state(door.get("state")):
        items.append({"key": "door.state", "name": "\u67dc\u95e8\u72b6\u6001", "value": door.get("state"), "unit": "", "level": LEVEL_THREE, "label": level_label(LEVEL_THREE), "threshold": "\u67dc\u95e8\u6253\u5f00", "message": "\u67dc\u95e8\u5904\u4e8e\u6253\u5f00\u72b6\u6001\uff0c\u9700\u8981\u786e\u8ba4\u73b0\u573a\u5b89\u5168"})

    freshness_source = data.get("updated_at") or last_seen_at
    stale_age = age_seconds(freshness_source, now=now)
    stale_level, stale_threshold = evaluate_min(stale_age, STALE_THRESHOLDS)
    if stale_level > LEVEL_NONE:
        items.append({"key": "data.stale", "name": "\u6570\u636e\u4e0a\u62a5\u72b6\u6001", "value": round(stale_age or 0.0, 1), "unit": "s", "level": stale_level, "label": level_label(stale_level), "threshold": f"\u8d85\u8fc7{level_label(stale_level)}\u79bb\u7ebf\u9608\u503c {stale_threshold:g} \u79d2", "message": f"\u6570\u636e\u5df2 {int(stale_age or 0)} \u79d2\u672a\u66f4\u65b0\uff0c\u9700\u8981\u68c0\u67e5\u677f\u7aef\u4e0a\u4f20\u94fe\u8def"})

    overall = LEVEL_NONE
    for item in items:
        overall = better_level(overall, int(item.get("level") or LEVEL_NONE))

    previous = previous_assessment if isinstance(previous_assessment, dict) else {}
    previous_level = int(previous.get("level") or LEVEL_NONE)
    previous_started_at = str(previous.get("started_at") or "")
    started_at = previous_started_at if overall > LEVEL_NONE and previous_level > LEVEL_NONE and previous_started_at else (now_text if overall > LEVEL_NONE else "")
    summary = "\u65e0\u5f02\u5e38\uff0c\u6240\u6709\u4e0a\u4f20\u6570\u636e\u5904\u4e8e\u8bc4\u4f30\u9608\u503c\u8303\u56f4\u5185\u3002"
    if items:
        summary = f"{len(items)} \u4e2a\u6a21\u5757\u5f02\u5e38\uff0c\u6700\u9ad8\u7b49\u7ea7\u4e3a{level_label(overall)}\u3002"

    return {
        "level": overall,
        "label": level_label(overall),
        "summary": summary,
        "started_at": started_at,
        "updated_at": now_text,
        "source_updated_at": str(freshness_source or ""),
        "items": sorted(items, key=lambda item: (-LEVEL_PRIORITY.get(int(item.get("level") or 0), 0), str(item.get("name") or ""))),
    }


def event_type(previous: dict[str, Any] | None, current: dict[str, Any]) -> str:
    previous_level = int((previous or {}).get("level") or LEVEL_NONE) if isinstance(previous, dict) else LEVEL_NONE
    current_level = int(current.get("level") or LEVEL_NONE)
    if previous_level <= LEVEL_NONE and current_level > LEVEL_NONE:
        return "start"
    if previous_level > LEVEL_NONE and current_level <= LEVEL_NONE:
        return "resolved"
    if previous_level > LEVEL_NONE and current_level > LEVEL_NONE and previous_level != current_level:
        return "level_changed"
    return ""
