#!/usr/bin/env python3
"""Classical seven-segment reader for the cabinet temperature controller."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import cv2
import numpy as np


SEGMENT_PATTERNS: dict[str, tuple[int, ...]] = {
    "0": (1, 1, 1, 1, 1, 1, 0),
    "1": (0, 1, 1, 0, 0, 0, 0),
    "2": (1, 1, 0, 1, 1, 0, 1),
    "3": (1, 1, 1, 1, 0, 0, 1),
    "4": (0, 1, 1, 0, 0, 1, 1),
    "5": (1, 0, 1, 1, 0, 1, 1),
    "6": (1, 0, 1, 1, 1, 1, 1),
    "7": (1, 1, 1, 0, 0, 0, 0),
    "8": (1, 1, 1, 1, 1, 1, 1),
    "9": (1, 1, 1, 1, 0, 1, 1),
}

# a, b, c, d, e, f, g. Narrow probes avoid the overexposed joins between
# adjacent segments on the XMT803 display.
SEGMENT_REGIONS: tuple[tuple[float, float, float, float], ...] = (
    (0.20, 0.00, 0.80, 0.20),
    (0.70, 0.10, 1.00, 0.48),
    (0.70, 0.52, 1.00, 0.90),
    (0.20, 0.80, 0.80, 1.00),
    (0.00, 0.52, 0.30, 0.90),
    (0.00, 0.10, 0.30, 0.48),
    (0.20, 0.40, 0.80, 0.60),
)


@dataclass(frozen=True)
class DigitComponent:
    label: int
    box: tuple[int, int, int, int]
    area: int
    fill_ratio: float
    warm_ratio: float

    @property
    def center(self) -> tuple[float, float]:
        x, y, width, height = self.box
        return (x + width * 0.5, y + height * 0.5)


def _warm_halo_ratio(hsv: np.ndarray, box: Sequence[int]) -> float:
    x, y, width, height = [int(value) for value in box]
    padding = max(2, int(round(max(width, height) * 0.25)))
    x1 = max(0, x - padding)
    y1 = max(0, y - padding)
    x2 = min(hsv.shape[1], x + width + padding)
    y2 = min(hsv.shape[0], y + height + padding)
    roi = hsv[y1:y2, x1:x2]
    if roi.size == 0:
        return 0.0
    hue = roi[:, :, 0]
    saturation = roi[:, :, 1]
    value = roi[:, :, 2]
    warm = ((hue <= 35) | (hue >= 170)) & (saturation >= 60) & (value >= 100)
    return float(np.mean(warm))


def _digit_components(image: np.ndarray) -> tuple[np.ndarray, np.ndarray, list[DigitComponent]]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # The segment cores clip toward white while their immediate halo stays
    # orange/red. This separates neighboring digits better than a red mask.
    core_mask = ((hsv[:, :, 2] >= 230) & (hsv[:, :, 1] <= 140)).astype(np.uint8)
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(core_mask, connectivity=8)
    image_height, image_width = image.shape[:2]
    min_height = max(24, int(round(image_height * 0.015)))
    max_height = max(min_height + 1, int(round(image_height * 0.12)))
    components: list[DigitComponent] = []
    for label in range(1, count):
        x, y, width, height, area = [int(value) for value in stats[label]]
        if height < min_height or height > max_height or width <= 0:
            continue
        aspect = float(width) / float(height)
        fill_ratio = float(area) / float(width * height)
        center_y = y + height * 0.5
        if not 0.10 <= aspect <= 0.90:
            continue
        if fill_ratio < 0.10 or center_y < image_height * 0.08 or center_y > image_height * 0.78:
            continue
        warm_ratio = _warm_halo_ratio(hsv, (x, y, width, height))
        if warm_ratio < 0.08:
            continue
        components.append(
            DigitComponent(
                label=label,
                box=(x, y, width, height),
                area=area,
                fill_ratio=fill_ratio,
                warm_ratio=warm_ratio,
            )
        )
    return core_mask, labels, components


def _row_groups(components: Sequence[DigitComponent], image_shape: Sequence[int]) -> list[list[DigitComponent]]:
    groups: list[list[DigitComponent]] = []
    ordered = sorted(components, key=lambda item: (item.center[1], item.center[0]))
    for seed in ordered:
        seed_height = float(seed.box[3])
        row = [
            item
            for item in ordered
            if 0.60 <= float(item.box[3]) / seed_height <= 1.55
            and abs(item.center[1] - seed.center[1]) <= max(seed_height, float(item.box[3])) * 0.38
        ]
        row.sort(key=lambda item: item.box[0])
        contiguous: list[DigitComponent] = []
        for item in row:
            if contiguous:
                previous = contiguous[-1]
                gap = item.box[0] - (previous.box[0] + previous.box[2])
                max_gap = max(previous.box[3], item.box[3]) * 1.20
                if gap > max_gap:
                    if len(contiguous) >= 2:
                        groups.append(contiguous)
                    contiguous = []
            contiguous.append(item)
        if len(contiguous) >= 2:
            groups.append(contiguous)

    unique: dict[tuple[int, ...], list[DigitComponent]] = {}
    for group in groups:
        key = tuple(item.label for item in group)
        unique[key] = group
    return list(unique.values())


def _group_score(group: Sequence[DigitComponent], image_shape: Sequence[int]) -> float:
    image_height, image_width = int(image_shape[0]), int(image_shape[1])
    mean_height = float(np.mean([item.box[3] for item in group]))
    vertical_spread = float(np.ptp([item.center[1] for item in group])) / max(1.0, mean_height)
    center_x = float(np.mean([item.center[0] for item in group]))
    center_y = float(np.mean([item.center[1] for item in group]))
    center_penalty = abs(center_x / image_width - 0.5) * 0.15 + abs(center_y / image_height - 0.38) * 0.10
    return (
        float(np.mean([item.warm_ratio for item in group])) * 2.0
        + float(np.mean([item.fill_ratio for item in group]))
        + min(4, len(group)) * 0.08
        - vertical_spread
        - center_penalty
    )


def _decode_component(component: DigitComponent, labels: np.ndarray) -> dict[str, Any]:
    x, y, width, height = component.box
    digit_mask = labels[y : y + height, x : x + width] == component.label
    occupancies: list[float] = []
    for x1, y1, x2, y2 in SEGMENT_REGIONS:
        left = int(round(x1 * width))
        top = int(round(y1 * height))
        right = max(left + 1, int(round(x2 * width)))
        bottom = max(top + 1, int(round(y2 * height)))
        occupancies.append(float(np.mean(digit_mask[top:bottom, left:right])))

    active = tuple(value >= 0.30 for value in occupancies)
    ranked: list[tuple[int, float, str]] = []
    for digit, pattern in SEGMENT_PATTERNS.items():
        hamming = sum(bool(active[index]) != bool(pattern[index]) for index in range(7))
        continuous = sum(
            (1.0 - occupancies[index]) if pattern[index] else occupancies[index]
            for index in range(7)
        )
        ranked.append((hamming, continuous, digit))
    hamming, continuous, digit = min(ranked)
    return {
        "digit": digit,
        "box_xyxy": [x, y, x + width, y + height],
        "segment_occupancy": [round(value, 4) for value in occupancies],
        "active_segments": [int(value) for value in active],
        "hamming_distance": int(hamming),
        "match_distance": round(float(hamming) / 7.0 + continuous / 49.0, 6),
        "warm_ratio": round(component.warm_ratio, 6),
    }


def read_temperature_display(image: np.ndarray) -> dict[str, Any]:
    if image is None or image.size == 0:
        return {"status": "no_image", "upper_text": "", "rows": [], "digits": [], "valid": False}

    _core_mask, labels, components = _digit_components(image)
    groups = _row_groups(components, image.shape)
    if not groups:
        return {
            "status": "no_upper_row",
            "upper_text": "",
            "rows": [],
            "digits": [],
            "valid": False,
            "method": "seven_segment_cv",
        }

    group = max(groups, key=lambda value: _group_score(value, image.shape))
    decoded = [_decode_component(component, labels) for component in group]
    text = "".join(item["digit"] for item in decoded)
    mean_distance = float(np.mean([item["match_distance"] for item in decoded]))
    exact = all(int(item["hamming_distance"]) == 0 for item in decoded)
    return {
        "status": "ok" if exact else "nearest",
        "upper_text": text,
        "rows": [text],
        "digits": decoded,
        "valid": len(text) >= 2,
        "reliable": bool(exact and len(text) >= 2),
        "method": "seven_segment_cv",
        "match_distance": round(mean_distance, 6),
        "quality_score": round(_group_score(group, image.shape) - mean_distance, 6),
    }


def draw_temperature_reading(image: np.ndarray, reading: dict[str, Any]) -> np.ndarray:
    annotated = image.copy()
    color = (0, 220, 0) if reading.get("reliable") else (0, 180, 255)
    for digit in reading.get("digits", []):
        box = digit.get("box_xyxy", [])
        if not isinstance(box, list) or len(box) != 4:
            continue
        x1, y1, x2, y2 = [int(value) for value in box]
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 3)
        cv2.putText(
            annotated,
            str(digit.get("digit") or "?"),
            (x1, max(24, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            color,
            2,
            cv2.LINE_AA,
        )
    text = str(reading.get("upper_text") or "")
    if text:
        cv2.putText(
            annotated,
            f"temperature upper={text}",
            (24, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            color,
            3,
            cv2.LINE_AA,
        )
    return annotated
