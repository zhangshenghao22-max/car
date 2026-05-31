from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
import yaml


IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


@dataclass(frozen=True)
class MeterConfig:
    min_value: float
    max_value: float
    center: tuple[float, float]
    zero_angle: float
    full_angle: float
    canonical_size: int
    rectify_mode: str = "auto"


@dataclass(frozen=True)
class PointerCandidate:
    tip: tuple[int, int]
    angle: float
    score: float
    method: str


@dataclass
class MeterReadResult:
    value: float | None
    det_conf: float | None
    status: str
    bbox: tuple[int, int, int, int] | None = None
    canonical_image: np.ndarray | None = None
    debug_image: np.ndarray | None = None
    pointer_tip: tuple[int, int] | None = None
    pointer_angle: float | None = None
    pointer_method: str | None = None


def load_meter_config(path: Path) -> MeterConfig:
    with path.open("r", encoding="utf-8") as handle:
        payload = yaml.safe_load(handle) or {}

    required_fields = {
        "min_value",
        "max_value",
        "center",
        "zero_angle",
        "full_angle",
        "canonical_size",
    }
    missing = required_fields.difference(payload)
    if missing:
        missing_list = ", ".join(sorted(missing))
        raise ValueError(f"Missing meter config fields: {missing_list}")

    center = payload["center"]
    if not isinstance(center, (list, tuple)) or len(center) != 2:
        raise ValueError("meter_config.yaml field 'center' must contain two numeric values.")

    config = MeterConfig(
        min_value=float(payload["min_value"]),
        max_value=float(payload["max_value"]),
        center=(float(center[0]), float(center[1])),
        zero_angle=float(payload["zero_angle"]),
        full_angle=float(payload["full_angle"]),
        canonical_size=int(payload["canonical_size"]),
        rectify_mode=str(payload.get("rectify_mode", "auto")),
    )

    if config.canonical_size <= 0:
        raise ValueError("canonical_size must be positive.")
    if config.rectify_mode not in {"auto", "resize"}:
        raise ValueError("rectify_mode must be 'auto' or 'resize'.")
    if abs(signed_delta_deg(config.zero_angle, config.full_angle)) < 1e-6:
        raise ValueError(f"{path} is not calibrated yet. Run calibrate.py first.")
    return config


def save_meter_config(path: Path, config: MeterConfig) -> None:
    payload = {
        "min_value": float(config.min_value),
        "max_value": float(config.max_value),
        "center": [float(config.center[0]), float(config.center[1])],
        "zero_angle": float(normalize_angle(config.zero_angle)),
        "full_angle": float(normalize_angle(config.full_angle)),
        "canonical_size": int(config.canonical_size),
        "rectify_mode": config.rectify_mode,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False)


def load_image(path: Path) -> np.ndarray | None:
    if not path.exists():
        return None
    buffer = np.fromfile(str(path), dtype=np.uint8)
    if buffer.size == 0:
        return None
    return cv2.imdecode(buffer, cv2.IMREAD_COLOR)


def save_image(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix or ".png"
    success, encoded = cv2.imencode(suffix, image)
    if not success:
        raise RuntimeError(f"Unable to encode image for {path}")
    encoded.tofile(str(path))


def list_image_files(source: Path) -> list[Path]:
    if source.is_file():
        return [source]
    files = [path for path in source.rglob("*") if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES]
    return sorted(files)


def normalize_angle(angle: float) -> float:
    return angle % 360.0


def signed_delta_deg(start_angle: float, end_angle: float) -> float:
    return (end_angle - start_angle + 180.0) % 360.0 - 180.0


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def angle_from_center(center: tuple[float, float], point: tuple[float, float]) -> float:
    dx = float(point[0]) - float(center[0])
    dy = float(center[1]) - float(point[1])
    return normalize_angle(math.degrees(math.atan2(dy, dx)))


def angle_in_sector(angle: float, start_angle: float, end_angle: float, margin: float = 0.0) -> bool:
    span = signed_delta_deg(start_angle, end_angle)
    delta = signed_delta_deg(start_angle, angle)
    if span >= 0:
        return -margin <= delta <= span + margin
    return span - margin <= delta <= margin


def build_angle_samples(start_angle: float, end_angle: float, step_deg: float = 0.5) -> list[float]:
    span = signed_delta_deg(start_angle, end_angle)
    steps = max(2, int(abs(span) / step_deg) + 1)
    return [normalize_angle(start_angle + span * index / (steps - 1)) for index in range(steps)]


def order_quad_points(points: np.ndarray) -> np.ndarray:
    sums = points.sum(axis=1)
    diffs = np.diff(points, axis=1).reshape(-1)
    ordered = np.zeros((4, 2), dtype=np.float32)
    ordered[0] = points[np.argmin(sums)]
    ordered[2] = points[np.argmax(sums)]
    ordered[1] = points[np.argmin(diffs)]
    ordered[3] = points[np.argmax(diffs)]
    return ordered


def find_outer_quad(image: np.ndarray, min_area_ratio: float = 0.20) -> np.ndarray | None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    edges = cv2.dilate(edges, np.ones((3, 3), dtype=np.uint8), iterations=1)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    image_area = float(image.shape[0] * image.shape[1])
    min_area = image_area * min_area_ratio
    best_quad = None
    best_area = 0.0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
        if len(approx) != 4 or not cv2.isContourConvex(approx):
            continue

        quad = approx.reshape(4, 2).astype(np.float32)
        quad_area = cv2.contourArea(quad)
        if quad_area > best_area:
            best_quad = quad
            best_area = quad_area

    if best_quad is not None:
        return order_quad_points(best_quad)

    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    rect = cv2.boxPoints(cv2.minAreaRect(largest)).astype(np.float32)
    if cv2.contourArea(rect) < min_area:
        return None
    return order_quad_points(rect)


def warp_square(image: np.ndarray, quad: np.ndarray, size: int) -> np.ndarray:
    destination = np.array(
        [[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]],
        dtype=np.float32,
    )
    transform = cv2.getPerspectiveTransform(order_quad_points(quad), destination)
    return cv2.warpPerspective(image, transform, (size, size))


def resize_square(image: np.ndarray, size: int) -> np.ndarray:
    return cv2.resize(image, (size, size), interpolation=cv2.INTER_AREA)


def expand_box(box: Iterable[float], image_shape: tuple[int, int, int], margin_ratio: float = 0.12) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = [int(round(value)) for value in box]
    width = max(1, x2 - x1)
    height = max(1, y2 - y1)
    margin = int(round(max(width, height) * margin_ratio))
    max_y, max_x = image_shape[:2]
    return (
        max(0, x1 - margin),
        max(0, y1 - margin),
        min(max_x, x2 + margin),
        min(max_y, y2 + margin),
    )


def crop_box(image: np.ndarray, box: tuple[int, int, int, int]) -> np.ndarray:
    x1, y1, x2, y2 = box
    return image[y1:y2, x1:x2].copy()


def prepare_pointer_mask(image: np.ndarray, config: MeterConfig) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
    binary = cv2.adaptiveThreshold(
        blurred,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        31,
        7,
    )
    binary = cv2.medianBlur(binary, 3)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8), iterations=1)

    mask = np.zeros_like(binary)
    center = (int(round(config.center[0])), int(round(config.center[1])))
    max_radius = int(round(config.canonical_size * 0.65))
    cv2.circle(mask, center, max_radius, 255, thickness=-1)
    return cv2.bitwise_and(binary, mask)


def longest_true_run(values: Iterable[bool]) -> int:
    longest = 0
    current = 0
    for value in values:
        if value:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest


def sample_ray_points(
    center: tuple[float, float],
    angle_deg: float,
    start_radius: float,
    end_radius: float,
    sample_count: int,
) -> np.ndarray:
    angle_rad = math.radians(angle_deg)
    radii = np.linspace(start_radius, end_radius, sample_count, dtype=np.float32)
    direction = np.array([math.cos(angle_rad), -math.sin(angle_rad)], dtype=np.float32)
    points = np.array(center, dtype=np.float32) + radii[:, None] * direction[None, :]
    return np.round(points).astype(np.int32)


def find_pointer_by_hough(pointer_mask: np.ndarray, config: MeterConfig) -> PointerCandidate | None:
    lines = cv2.HoughLinesP(
        pointer_mask,
        rho=1,
        theta=np.pi / 180.0,
        threshold=25,
        minLineLength=max(20, int(config.canonical_size * 0.15)),
        maxLineGap=12,
    )
    if lines is None:
        return None

    center = np.array(config.center, dtype=np.float32)
    best_candidate = None
    best_score = -1.0

    for raw_line in lines:
        x1, y1, x2, y2 = raw_line[0]
        point_a = np.array([x1, y1], dtype=np.float32)
        point_b = np.array([x2, y2], dtype=np.float32)
        dist_a = float(np.linalg.norm(point_a - center))
        dist_b = float(np.linalg.norm(point_b - center))
        near_point, far_point = (point_a, point_b) if dist_a <= dist_b else (point_b, point_a)
        near_dist = min(dist_a, dist_b)
        far_dist = max(dist_a, dist_b)
        line_length = float(np.linalg.norm(point_a - point_b))

        if near_dist > config.canonical_size * 0.14:
            continue
        if far_dist < config.canonical_size * 0.14 or far_dist > config.canonical_size * 0.70:
            continue

        angle = angle_from_center(config.center, (float(far_point[0]), float(far_point[1])))
        if not angle_in_sector(angle, config.zero_angle, config.full_angle, margin=12.0):
            continue

        score = line_length + far_dist - near_dist * 2.0
        if score <= best_score:
            continue

        best_candidate = PointerCandidate(
            tip=(int(round(far_point[0])), int(round(far_point[1]))),
            angle=angle,
            score=score,
            method="hough",
        )
        best_score = score

    return best_candidate


def find_pointer_by_ray(pointer_mask: np.ndarray, config: MeterConfig) -> PointerCandidate | None:
    height, width = pointer_mask.shape[:2]
    start_radius = config.canonical_size * 0.05
    end_radius = config.canonical_size * 0.65
    sample_count = max(80, int(config.canonical_size * 0.55))
    best_candidate = None
    best_score = -1.0

    for angle in build_angle_samples(config.zero_angle, config.full_angle, step_deg=0.5):
        ray_points = sample_ray_points(config.center, angle, start_radius, end_radius, sample_count)
        xs = ray_points[:, 0]
        ys = ray_points[:, 1]
        valid = (xs >= 0) & (xs < width) & (ys >= 0) & (ys < height)
        if not np.any(valid):
            continue

        xs = xs[valid]
        ys = ys[valid]
        occupied = pointer_mask[ys, xs] > 0
        if not np.any(occupied):
            continue

        run_length = longest_true_run(occupied.tolist())
        total_hits = int(np.count_nonzero(occupied))
        score = float(run_length * 4 + total_hits)

        if run_length < int(sample_count * 0.10):
            continue
        if score <= best_score:
            continue

        farthest_index = int(np.flatnonzero(occupied)[-1])
        best_candidate = PointerCandidate(
            tip=(int(xs[farthest_index]), int(ys[farthest_index])),
            angle=angle,
            score=score,
            method="ray",
        )
        best_score = score

    return best_candidate


def choose_pointer_candidate(
    hough_candidate: PointerCandidate | None,
    ray_candidate: PointerCandidate | None,
) -> PointerCandidate | None:
    if hough_candidate and ray_candidate:
        delta = abs(signed_delta_deg(hough_candidate.angle, ray_candidate.angle))
        if delta <= 6.0:
            return ray_candidate
        return hough_candidate if hough_candidate.score >= ray_candidate.score else ray_candidate
    return hough_candidate or ray_candidate


def select_best_detection(result) -> tuple[np.ndarray, float] | None:
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return None

    confidences = boxes.conf.detach().cpu().numpy()
    best_index = int(np.argmax(confidences))
    box = boxes.xyxy[best_index].detach().cpu().numpy()
    confidence = float(confidences[best_index])
    return box, confidence


def read_meter_from_crop(
    crop: np.ndarray,
    config: MeterConfig,
    det_conf: float | None = None,
    bbox: tuple[int, int, int, int] | None = None,
) -> MeterReadResult:
    if crop.size == 0:
        return MeterReadResult(value=None, det_conf=det_conf, status="crop_failed", bbox=bbox)

    if config.rectify_mode == "resize":
        canonical_image = resize_square(crop, config.canonical_size)
    else:
        quad = find_outer_quad(crop)
        if quad is None:
            return MeterReadResult(
                value=None,
                det_conf=det_conf,
                status="rectify_failed",
                bbox=bbox,
                debug_image=crop,
            )
        canonical_image = warp_square(crop, quad, config.canonical_size)

    pointer_mask = prepare_pointer_mask(canonical_image, config)
    hough_candidate = find_pointer_by_hough(pointer_mask, config)
    ray_candidate = find_pointer_by_ray(pointer_mask, config)
    candidate = choose_pointer_candidate(hough_candidate, ray_candidate)
    if candidate is None:
        return MeterReadResult(
            value=None,
            det_conf=det_conf,
            status="pointer_failed",
            bbox=bbox,
            canonical_image=canonical_image,
            debug_image=build_debug_image(canonical_image, config, None, "pointer_failed", None),
        )

    value = meter_value_from_angle(candidate.angle, config)
    debug_image = build_debug_image(canonical_image, config, candidate, "ok", value)
    return MeterReadResult(
        value=value,
        det_conf=det_conf,
        status="ok",
        bbox=bbox,
        canonical_image=canonical_image,
        debug_image=debug_image,
        pointer_tip=candidate.tip,
        pointer_angle=candidate.angle,
        pointer_method=candidate.method,
    )


def read_meter_from_box(
    image: np.ndarray,
    box: Iterable[float],
    config: MeterConfig,
    det_conf: float | None = None,
    margin_ratio: float = 0.12,
) -> MeterReadResult:
    expanded_box = expand_box(box, image.shape, margin_ratio=margin_ratio)
    crop = crop_box(image, expanded_box)
    return read_meter_from_crop(crop, config, det_conf=det_conf, bbox=expanded_box)


def meter_value_from_angle(angle: float, config: MeterConfig) -> float:
    span = signed_delta_deg(config.zero_angle, config.full_angle)
    progress = signed_delta_deg(config.zero_angle, angle)
    ratio = clamp(progress / span, 0.0, 1.0)
    return config.min_value + ratio * (config.max_value - config.min_value)


def build_debug_image(
    canonical_image: np.ndarray,
    config: MeterConfig,
    candidate: PointerCandidate | None,
    status: str,
    value: float | None,
) -> np.ndarray:
    debug = canonical_image.copy()
    center = (int(round(config.center[0])), int(round(config.center[1])))
    cv2.circle(debug, center, 6, (0, 255, 0), thickness=-1)

    ray_length = int(round(config.canonical_size * 0.55))
    for angle, color in (
        (config.zero_angle, (255, 0, 0)),
        (config.full_angle, (0, 0, 255)),
    ):
        angle_rad = math.radians(angle)
        endpoint = (
            int(round(center[0] + ray_length * math.cos(angle_rad))),
            int(round(center[1] - ray_length * math.sin(angle_rad))),
        )
        cv2.line(debug, center, endpoint, color, thickness=2)

    if candidate is not None:
        cv2.line(debug, center, candidate.tip, (0, 255, 255), thickness=2)
        cv2.circle(debug, candidate.tip, 5, (0, 255, 255), thickness=-1)
        cv2.putText(
            debug,
            f"{candidate.method}:{candidate.angle:.1f}",
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            lineType=cv2.LINE_AA,
        )

    summary = status if value is None else f"{status} value={value:.2f}"
    cv2.putText(
        debug,
        summary,
        (10, debug.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0) if status == "ok" else (0, 0, 255),
        2,
        lineType=cv2.LINE_AA,
    )
    return debug


def read_meter_from_image(
    image: np.ndarray,
    detector,
    config: MeterConfig,
    conf_threshold: float = 0.25,
    device: str | None = None,
) -> MeterReadResult:
    predict_args = {
        "source": image,
        "conf": conf_threshold,
        "verbose": False,
    }
    if device:
        predict_args["device"] = device
    prediction = detector.predict(**predict_args)[0]
    detection = select_best_detection(prediction)
    if detection is None:
        return MeterReadResult(value=None, det_conf=None, status="no_detection")

    raw_box, det_conf = detection
    return read_meter_from_box(image, raw_box, config, det_conf=det_conf, margin_ratio=0.12)
