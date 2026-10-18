from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pathlib import Path
import time
from typing import Iterable

import cv2
import numpy as np
import yaml


IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


class MeterProcessingTimeout(RuntimeError):
    """Raised when one meter frame exceeds its CPU processing budget."""


def check_deadline(deadline: float | None) -> None:
    if deadline is not None and time.monotonic() >= float(deadline):
        raise MeterProcessingTimeout("meter processing deadline exceeded")


@dataclass(frozen=True)
class MeterConfig:
    min_value: float
    max_value: float
    center: tuple[float, float]
    zero_angle: float
    full_angle: float
    canonical_size: int
    rectify_mode: str = "auto"
    template_path: str | None = None


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
    rectify_method: str | None = None
    pointer_candidates: list[dict] | None = None
    pointer_consensus_count: int = 0
    rectify_quality: dict | None = None
    quality_score: float = 0.0


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

    raw_template_path = payload.get("template_path")
    template_path = None
    if raw_template_path:
        candidate_path = Path(str(raw_template_path))
        if not candidate_path.is_absolute():
            candidate_path = path.parent / candidate_path
        template_path = str(candidate_path)

    config = MeterConfig(
        min_value=float(payload["min_value"]),
        max_value=float(payload["max_value"]),
        center=(float(center[0]), float(center[1])),
        zero_angle=float(payload["zero_angle"]),
        full_angle=float(payload["full_angle"]),
        canonical_size=int(payload["canonical_size"]),
        rectify_mode=str(payload.get("rectify_mode", "auto")),
        template_path=template_path,
    )

    if config.canonical_size <= 0:
        raise ValueError("canonical_size must be positive.")
    if config.rectify_mode not in {"auto", "resize", "template"}:
        raise ValueError("rectify_mode must be 'auto', 'resize', or 'template'.")
    if config.rectify_mode == "template" and not config.template_path:
        raise ValueError("template mode requires template_path.")
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
    if config.template_path:
        payload["template_path"] = config.template_path
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


def find_closed_edge_outer_quad(image: np.ndarray, min_area_ratio: float = 0.40) -> np.ndarray | None:
    """Recover a complete meter housing when glare splits its outer contour."""
    if image.size == 0:
        return None

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    height, width = image.shape[:2]
    image_area = float(height * width)
    edge_margin = max(4.0, min(height, width) * 0.02)
    candidates: list[tuple[float, np.ndarray]] = []

    for low, high in ((30, 100), (40, 120), (50, 150)):
        edges = cv2.Canny(blurred, low, high)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((15, 15), dtype=np.uint8))
        edges = cv2.dilate(edges, np.ones((3, 3), dtype=np.uint8), iterations=1)
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            contour_area_ratio = float(cv2.contourArea(contour)) / image_area
            if contour_area_ratio < min_area_ratio * 0.20:
                continue
            quad = order_quad_points(cv2.boxPoints(cv2.minAreaRect(contour)).astype(np.float32))
            quality = validate_outer_quad(image, quad, min_area_ratio=min_area_ratio)
            if quality is None:
                continue
            min_x, min_y = np.min(quad, axis=0)
            max_x, max_y = np.max(quad, axis=0)
            if (
                min_x <= edge_margin
                or min_y <= edge_margin
                or max_x >= width - 1 - edge_margin
                or max_y >= height - 1 - edge_margin
            ):
                continue
            score = contour_area_ratio + float(quality["area_ratio"])
            candidates.append((score, quad))

    if not candidates:
        return None
    return max(candidates, key=lambda item: item[0])[1]


def validate_outer_quad(
    image: np.ndarray,
    quad: np.ndarray,
    min_area_ratio: float = 0.40,
) -> dict | None:
    if quad is None or np.asarray(quad).shape != (4, 2):
        return None
    ordered = order_quad_points(np.asarray(quad, dtype=np.float32))
    if not np.all(np.isfinite(ordered)) or not cv2.isContourConvex(ordered.astype(np.int32)):
        return None

    image_height, image_width = image.shape[:2]
    image_area = float(max(1, image_height * image_width))
    area_ratio = float(abs(cv2.contourArea(ordered))) / image_area
    sides = [
        float(np.linalg.norm(ordered[(index + 1) % 4] - ordered[index]))
        for index in range(4)
    ]
    if area_ratio < min_area_ratio or min(sides) < min(image_height, image_width) * 0.30:
        return None
    opposite_ratios = (
        max(sides[0], sides[2]) / max(1.0, min(sides[0], sides[2])),
        max(sides[1], sides[3]) / max(1.0, min(sides[1], sides[3])),
    )
    if max(opposite_ratios) > 2.2:
        return None

    min_x, min_y = np.min(ordered, axis=0)
    max_x, max_y = np.max(ordered, axis=0)
    coverage_x = float(max_x - min_x) / max(1.0, float(image_width))
    coverage_y = float(max_y - min_y) / max(1.0, float(image_height))
    center = np.mean(ordered, axis=0)
    if coverage_x < 0.55 or coverage_y < 0.55:
        return None
    if not (image_width * 0.25 <= center[0] <= image_width * 0.75):
        return None
    if not (image_height * 0.25 <= center[1] <= image_height * 0.75):
        return None
    return {
        "area_ratio": round(area_ratio, 6),
        "coverage_x": round(coverage_x, 6),
        "coverage_y": round(coverage_y, 6),
        "side_lengths": [round(value, 3) for value in sides],
        "opposite_side_ratio": round(max(opposite_ratios), 6),
    }


def warp_square(image: np.ndarray, quad: np.ndarray, size: int) -> np.ndarray:
    destination = np.array(
        [[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]],
        dtype=np.float32,
    )
    transform = cv2.getPerspectiveTransform(order_quad_points(quad), destination)
    return cv2.warpPerspective(image, transform, (size, size))


def resize_square(image: np.ndarray, size: int) -> np.ndarray:
    return cv2.resize(image, (size, size), interpolation=cv2.INTER_AREA)


_TEMPLATE_CACHE: dict[str, np.ndarray] = {}


def load_template_image(config: MeterConfig) -> np.ndarray | None:
    if not config.template_path:
        return None
    path = str(config.template_path)
    cached = _TEMPLATE_CACHE.get(path)
    if cached is not None:
        return cached
    template = load_image(Path(path))
    if template is None:
        return None
    if template.shape[0] != config.canonical_size or template.shape[1] != config.canonical_size:
        template = resize_square(template, config.canonical_size)
    _TEMPLATE_CACHE[path] = template
    return template


def registration_mask(config: MeterConfig) -> np.ndarray:
    size = int(config.canonical_size)
    mask = np.zeros((size, size), dtype=np.uint8)
    center = (int(round(config.center[0])), int(round(config.center[1])))
    outer_radius = int(round(size * 0.62))
    inner_radius = int(round(size * 0.30))
    cv2.circle(mask, center, outer_radius, 255, thickness=-1)
    cv2.circle(mask, center, inner_radius, 0, thickness=-1)
    mask[int(round(config.center[1] - size * 0.04)) :, :] = 0
    mask[:, : int(round(size * 0.04))] = 0
    mask[:, int(round(size * 0.96)) :] = 0
    return mask


def gray_for_registration(image: np.ndarray, config: MeterConfig) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    color_mask = prepare_color_pointer_mask(image, config)
    static_mask = registration_mask(config)
    fill_pixels = gray[(static_mask > 0) & (color_mask == 0)]
    fill_value = int(np.median(fill_pixels)) if fill_pixels.size else int(np.median(gray))
    gray[color_mask > 0] = fill_value
    return gray


def align_by_ecc(
    image: np.ndarray,
    template: np.ndarray,
    config: MeterConfig,
    min_score: float = 0.45,
) -> tuple[np.ndarray, float] | None:
    size = int(config.canonical_size)
    template_gray = gray_for_registration(template, config)
    image_gray = gray_for_registration(image, config)
    mask = registration_mask(config)
    warp_matrix = np.eye(2, 3, dtype=np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 80, 1e-5)
    try:
        cc, warp_matrix = cv2.findTransformECC(
            template_gray,
            image_gray,
            warp_matrix,
            cv2.MOTION_AFFINE,
            criteria,
            inputMask=mask,
            gaussFiltSize=5,
        )
    except cv2.error:
        return None
    if not np.isfinite(cc) or cc < float(min_score):
        return None
    aligned = cv2.warpAffine(
        image,
        warp_matrix,
        (size, size),
        flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
        borderMode=cv2.BORDER_REPLICATE,
    )
    return aligned, float(cc)


def align_by_orb_detailed(
    image: np.ndarray,
    template: np.ndarray,
    config: MeterConfig,
    *,
    min_inliers: int = 10,
    max_reprojection_error: float = 5.0,
    validate_projection: bool = True,
) -> tuple[np.ndarray, dict] | None:
    size = int(config.canonical_size)
    template_gray = gray_for_registration(template, config)
    image_gray = gray_for_registration(image, config)
    mask = registration_mask(config)
    orb = cv2.ORB_create(nfeatures=1000, edgeThreshold=15, patchSize=31)
    template_kp, template_desc = orb.detectAndCompute(template_gray, mask)
    image_kp, image_desc = orb.detectAndCompute(image_gray, mask)
    if template_desc is None or image_desc is None or len(template_kp) < 12 or len(image_kp) < 12:
        return None

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    raw_matches = matcher.knnMatch(image_desc, template_desc, k=2)
    good_matches = []
    for match_pair in raw_matches:
        if len(match_pair) != 2:
            continue
        first, second = match_pair
        if first.distance < 0.75 * second.distance:
            good_matches.append(first)
    if len(good_matches) < max(12, int(min_inliers)):
        return None

    source_points = np.float32([image_kp[match.queryIdx].pt for match in good_matches]).reshape(-1, 1, 2)
    target_points = np.float32([template_kp[match.trainIdx].pt for match in good_matches]).reshape(-1, 1, 2)
    transform, inliers = cv2.findHomography(source_points, target_points, cv2.RANSAC, 5.0)
    if transform is None or inliers is None:
        return None
    inlier_count = int(np.count_nonzero(inliers))
    if inlier_count < int(min_inliers):
        return None

    inlier_mask = inliers.reshape(-1).astype(bool)
    projected = cv2.perspectiveTransform(source_points[inlier_mask], transform)
    target_inliers = target_points[inlier_mask]
    reprojection_error = float(np.mean(np.linalg.norm(projected - target_inliers, axis=2)))
    if not math.isfinite(reprojection_error) or reprojection_error > float(max_reprojection_error):
        return None

    corners = np.float32(
        [[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]]
    ).reshape(-1, 1, 2)
    projected_corners = cv2.perspectiveTransform(corners, transform).reshape(4, 2)
    projected_area = float(abs(cv2.contourArea(projected_corners))) / float(size * size)
    if validate_projection and (
        not np.all(np.isfinite(projected_corners)) or not 0.35 <= projected_area <= 2.5
    ):
        return None

    aligned = cv2.warpPerspective(
        image,
        transform,
        (size, size),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE,
    )
    return aligned, {
        "inliers": inlier_count,
        "reprojection_error": round(reprojection_error, 6),
        "projected_area_ratio": round(projected_area, 6),
    }


def align_by_orb(image: np.ndarray, template: np.ndarray, config: MeterConfig) -> tuple[np.ndarray, int] | None:
    result = align_by_orb_detailed(
        image,
        template,
        config,
        max_reprojection_error=float("inf"),
        validate_projection=False,
    )
    if result is None:
        return None
    aligned, quality = result
    return aligned, int(quality["inliers"])


def resize_fallback_quality(
    crop: np.ndarray,
    *,
    template_available: bool,
    reason: str,
) -> dict:
    """Describe a non-geometric fallback for downstream quality decisions."""
    height, width = crop.shape[:2]
    return {
        "method": "geometry_fallback",
        "geometry_fallback": True,
        "legacy_mode": True,
        "reason": reason,
        "template_available": bool(template_available),
        "crop_size": [int(width), int(height)],
    }


def template_align_after_geometry_failure(
    crop: np.ndarray,
    config: MeterConfig,
    template: np.ndarray | None,
) -> tuple[np.ndarray, str, dict]:
    """Try template registration, then preserve the detector crop as a marked fallback."""
    base = resize_square(crop, config.canonical_size)
    if template is not None:
        ecc_result = align_by_ecc(base, template, config)
        if ecc_result is not None:
            aligned, score = ecc_result
            quality = resize_fallback_quality(
                crop,
                template_available=True,
                reason="outer_quad_failed",
            )
            quality["ecc_score"] = round(float(score), 6)
            return aligned, f"template-ecc:{score:.3f}:geometry-fallback", quality

    return (
        base,
        "resize_fallback:geometry_failed",
        resize_fallback_quality(
            crop,
            template_available=template is not None,
            reason="outer_quad_and_template_failed",
        ),
    )


def rectify_quad_crop(
    crop: np.ndarray,
    config: MeterConfig,
    quad: np.ndarray,
    quad_quality: dict,
    method: str,
) -> tuple[np.ndarray, str, dict]:
    canonical = warp_square(crop, quad, config.canonical_size)
    quality = {"method": method, **quad_quality}
    template = load_template_image(config)
    if template is not None:
        ecc_result = align_by_ecc(canonical, template, config, min_score=0.65)
        if ecc_result is not None:
            canonical, ecc_score = ecc_result
            quality["ecc_score"] = round(float(ecc_score), 6)
            return canonical, f"{method}+ecc:{ecc_score:.3f}", quality
    return canonical, method, quality


def rectify_meter_crop_detailed(
    crop: np.ndarray,
    config: MeterConfig,
    *,
    allow_relaxed_quad: bool = False,
    fast_alignment: bool = False,
    deadline: float | None = None,
) -> tuple[np.ndarray | None, str, dict]:
    check_deadline(deadline)
    if crop.size == 0:
        return None, "crop_failed", {}

    if fast_alignment:
        minimum_area = 0.40
        quad = find_outer_quad(crop, min_area_ratio=minimum_area)
        quad_quality = (
            None
            if quad is None
            else validate_outer_quad(crop, quad, min_area_ratio=minimum_area)
        )
        if quad is None and allow_relaxed_quad:
            quad = find_closed_edge_outer_quad(crop, min_area_ratio=minimum_area)
            quad_quality = (
                None
                if quad is None
                else validate_outer_quad(crop, quad, min_area_ratio=minimum_area)
            )
        check_deadline(deadline)
        if quad is not None and quad_quality is not None:
            return (
                warp_square(crop, quad, config.canonical_size),
                "fast-quad",
                {"method": "fast-quad", "fast_alignment": True, **quad_quality},
            )
        return (
            resize_square(crop, config.canonical_size),
            "fast-resize",
            {
                **resize_fallback_quality(
                    crop,
                    template_available=bool(config.template_path),
                    reason="fast_outer_quad_failed",
                ),
                "method": "fast-resize",
                "fast_alignment": True,
            },
        )

    if config.rectify_mode == "resize":
        return resize_square(crop, config.canonical_size), "resize", {"legacy_mode": True}

    if config.rectify_mode == "auto":
        quad = find_outer_quad(crop, min_area_ratio=0.40)
        quad_quality = None if quad is None else validate_outer_quad(crop, quad, min_area_ratio=0.40)
        if quad is not None and quad_quality is not None:
            return rectify_quad_crop(crop, config, quad, quad_quality, "auto-quad")

        if allow_relaxed_quad:
            quad = find_closed_edge_outer_quad(crop, min_area_ratio=0.40)
            quad_quality = None if quad is None else validate_outer_quad(crop, quad, min_area_ratio=0.40)
            if quad is not None and quad_quality is not None:
                return rectify_quad_crop(crop, config, quad, quad_quality, "closed-edge-quad")

        template = load_template_image(config)
        if template is not None:
            base = resize_square(crop, config.canonical_size)
            orb_result = align_by_orb_detailed(
                base,
                template,
                config,
                min_inliers=25,
                max_reprojection_error=2.0,
            )
            if orb_result is not None:
                aligned, orb_quality = orb_result
                return aligned, f"strict-orb:{orb_quality['inliers']}", {
                    "method": "strict-orb",
                    **orb_quality,
                }
            return template_align_after_geometry_failure(crop, config, template)
        return (
            resize_square(crop, config.canonical_size),
            "resize_fallback:no_template",
            resize_fallback_quality(
                crop,
                template_available=False,
                reason="outer_quad_failed",
            ),
        )

    base = resize_square(crop, config.canonical_size)
    template = load_template_image(config)
    if template is None:
        return base, "resize_fallback:no_template", {"legacy_mode": True}

    ecc_result = align_by_ecc(base, template, config)
    if ecc_result is not None:
        aligned, score = ecc_result
        return aligned, f"template-ecc:{score:.3f}", {"ecc_score": round(float(score), 6)}

    orb_result = align_by_orb(base, template, config)
    if orb_result is not None:
        aligned, inliers = orb_result
        return aligned, f"template-orb:{inliers}", {"orb_inliers": int(inliers)}

    return base, "resize_fallback:template_failed", {"legacy_mode": True}


def rectify_meter_crop(crop: np.ndarray, config: MeterConfig) -> tuple[np.ndarray | None, str]:
    image, method, _quality = rectify_meter_crop_detailed(crop, config)
    return image, method


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


def apply_pointer_search_mask(
    mask: np.ndarray,
    config: MeterConfig,
    max_radius_ratio: float = 0.68,
) -> np.ndarray:
    search_mask = np.zeros_like(mask)
    center = (int(round(config.center[0])), int(round(config.center[1])))
    max_radius = int(round(config.canonical_size * max_radius_ratio))
    cv2.circle(search_mask, center, max_radius, 255, thickness=-1)
    return cv2.bitwise_and(mask, search_mask)


def prepare_gray_pointer_mask(image: np.ndarray, config: MeterConfig) -> np.ndarray:
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

    return apply_pointer_search_mask(binary, config)


def prepare_color_pointer_mask(image: np.ndarray, config: MeterConfig) -> np.ndarray:
    """Prefer faint red/orange needle pixels before falling back to dark text masks."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red_low = cv2.inRange(hsv, np.array([0, 8, 55], dtype=np.uint8), np.array([22, 255, 255], dtype=np.uint8))
    red_high = cv2.inRange(hsv, np.array([165, 8, 55], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8))

    b_channel, g_channel, r_channel = cv2.split(image)
    red_delta = (
        r_channel.astype(np.int16)
        - ((g_channel.astype(np.int16) + b_channel.astype(np.int16)) // 2)
    )
    weak_red = np.zeros_like(r_channel, dtype=np.uint8)
    weak_red[(r_channel > 70) & (red_delta > 5)] = 255

    mask = cv2.bitwise_or(cv2.bitwise_or(red_low, red_high), weak_red)
    mask = cv2.medianBlur(mask, 3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8), iterations=1)
    return apply_pointer_search_mask(mask, config)


def prepare_edge_pointer_mask(image: np.ndarray, config: MeterConfig) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    enhanced = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    blurred = cv2.GaussianBlur(enhanced, (3, 3), 0)
    edges = cv2.Canny(blurred, 40, 120)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((2, 2), dtype=np.uint8), iterations=1)
    return apply_pointer_search_mask(edges, config)


def prepare_pointer_mask(image: np.ndarray, config: MeterConfig) -> np.ndarray:
    return prepare_gray_pointer_mask(image, config)


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


def sample_ray_strip_occupied(
    pointer_mask: np.ndarray,
    points: np.ndarray,
    strip_radius: int,
) -> np.ndarray:
    height, width = pointer_mask.shape[:2]
    occupied = np.zeros(points.shape[0], dtype=bool)
    radius = max(0, int(strip_radius))
    for index, (x_coord, y_coord) in enumerate(points):
        if x_coord < 0 or x_coord >= width or y_coord < 0 or y_coord >= height:
            continue
        x1 = max(0, int(x_coord) - radius)
        y1 = max(0, int(y_coord) - radius)
        x2 = min(width, int(x_coord) + radius + 1)
        y2 = min(height, int(y_coord) + radius + 1)
        occupied[index] = bool(np.any(pointer_mask[y1:y2, x1:x2] > 0))
    return occupied


def find_pointer_by_hough(
    pointer_mask: np.ndarray,
    config: MeterConfig,
    method: str = "hough",
    min_line_ratio: float = 0.15,
    near_center_ratio: float = 0.14,
    deadline: float | None = None,
) -> PointerCandidate | None:
    check_deadline(deadline)
    lines = cv2.HoughLinesP(
        pointer_mask,
        rho=1,
        theta=np.pi / 180.0,
        threshold=25,
        minLineLength=max(20, int(config.canonical_size * min_line_ratio)),
        maxLineGap=12,
    )
    if lines is None:
        return None

    center = np.array(config.center, dtype=np.float32)
    best_candidate = None
    best_score = -1.0

    for index, raw_line in enumerate(lines):
        if index % 32 == 0:
            check_deadline(deadline)
        line_values = np.asarray(raw_line).reshape(-1)
        if line_values.size < 4:
            continue
        x1, y1, x2, y2 = [int(value) for value in line_values[:4]]
        point_a = np.array([x1, y1], dtype=np.float32)
        point_b = np.array([x2, y2], dtype=np.float32)
        dist_a = float(np.linalg.norm(point_a - center))
        dist_b = float(np.linalg.norm(point_b - center))
        near_point, far_point = (point_a, point_b) if dist_a <= dist_b else (point_b, point_a)
        near_dist = min(dist_a, dist_b)
        far_dist = max(dist_a, dist_b)
        line_length = float(np.linalg.norm(point_a - point_b))

        if near_dist > config.canonical_size * near_center_ratio:
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
            method=method,
        )
        best_score = score

    check_deadline(deadline)
    return best_candidate


def find_pointer_by_ray(
    pointer_mask: np.ndarray,
    config: MeterConfig,
    method: str = "ray",
    start_radius_ratio: float = 0.05,
    end_radius_ratio: float = 0.65,
    step_deg: float = 0.5,
    strip_radius: int = 0,
    min_run_ratio: float = 0.10,
    min_hits: int = 0,
    max_first_hit_ratio: float = 1.0,
    deadline: float | None = None,
) -> PointerCandidate | None:
    check_deadline(deadline)
    height, width = pointer_mask.shape[:2]
    start_radius = config.canonical_size * start_radius_ratio
    end_radius = config.canonical_size * end_radius_ratio
    sample_count = max(80, int(config.canonical_size * 0.55))
    radius = max(0, int(strip_radius))
    sampled_mask = pointer_mask
    if radius:
        kernel_size = radius * 2 + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        sampled_mask = cv2.dilate(pointer_mask, kernel, iterations=1)
    best_candidate = None
    best_score = -1.0

    for index, angle in enumerate(build_angle_samples(config.zero_angle, config.full_angle, step_deg=step_deg)):
        if index % 8 == 0:
            check_deadline(deadline)
        ray_points = sample_ray_points(config.center, angle, start_radius, end_radius, sample_count)
        xs = ray_points[:, 0]
        ys = ray_points[:, 1]
        valid = (xs >= 0) & (xs < width) & (ys >= 0) & (ys < height)
        if not np.any(valid):
            continue

        xs = xs[valid]
        ys = ys[valid]
        occupied = sampled_mask[ys, xs] > 0
        if not np.any(occupied):
            continue

        run_length = longest_true_run(occupied.tolist())
        total_hits = int(np.count_nonzero(occupied))
        first_hit_index = int(np.flatnonzero(occupied)[0])
        farthest_index = int(np.flatnonzero(occupied)[-1])
        span_length = max(1, farthest_index - first_hit_index + 1)
        coverage = float(total_hits) / float(span_length)
        score = float(run_length * 5 + total_hits * 1.5 + span_length * 0.15 - first_hit_index * 0.20)

        if run_length < int(sample_count * min_run_ratio):
            continue
        if total_hits < int(min_hits):
            continue
        if first_hit_index > int(sample_count * max_first_hit_ratio):
            continue
        if coverage < 0.08:
            continue
        if score <= best_score:
            continue

        best_candidate = PointerCandidate(
            tip=(int(xs[farthest_index]), int(ys[farthest_index])),
            angle=angle,
            score=score,
            method=method,
        )
        best_score = score

    check_deadline(deadline)
    return best_candidate


def choose_pointer_candidate(*candidates: PointerCandidate | None) -> PointerCandidate | None:
    available = [candidate for candidate in candidates if candidate is not None]
    if not available:
        return None

    def priority(candidate: PointerCandidate) -> tuple[int, float]:
        method = candidate.method or ""
        if method.startswith("color"):
            return (3, candidate.score)
        if method.startswith("edge"):
            return (2, candidate.score)
        if method.startswith("gray-hough"):
            return (1, candidate.score)
        return (0, candidate.score)

    return max(available, key=priority)


def pointer_candidate_payload(candidate: PointerCandidate) -> dict:
    return {
        "method": candidate.method,
        "angle": round(float(candidate.angle), 6),
        "score": round(float(candidate.score), 6),
        "tip": [int(candidate.tip[0]), int(candidate.tip[1])],
    }


def angle_distance_deg(first: float, second: float) -> float:
    return abs(signed_delta_deg(first, second))


def consensus_pointer_candidate(
    candidates: Iterable[PointerCandidate | None],
    tolerance_deg: float = 1.5,
) -> tuple[PointerCandidate | None, int, list[dict]]:
    available = [candidate for candidate in candidates if candidate is not None]
    payloads = [pointer_candidate_payload(candidate) for candidate in available]
    if not available:
        return None, 0, payloads

    clusters = [
        [other for other in available if angle_distance_deg(candidate.angle, other.angle) <= tolerance_deg]
        for candidate in available
    ]
    cluster = max(clusters, key=lambda values: (len(values), sum(value.score for value in values)))
    if len(cluster) < 2:
        return choose_pointer_candidate(*available), 1, payloads

    radians = np.radians([candidate.angle for candidate in cluster])
    angle = normalize_angle(math.degrees(math.atan2(float(np.mean(np.sin(radians))), float(np.mean(np.cos(radians))))))
    representative = min(cluster, key=lambda candidate: angle_distance_deg(angle, candidate.angle))
    methods = "+".join(sorted({candidate.method for candidate in cluster}))
    return (
        PointerCandidate(
            tip=representative.tip,
            angle=angle,
            score=float(np.mean([candidate.score for candidate in cluster])),
            method=f"consensus:{methods}",
        ),
        len(cluster),
        payloads,
    )


def detect_pointer_center(image: np.ndarray, config: MeterConfig) -> tuple[MeterConfig, dict]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 0, 85)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
    contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    size = float(config.canonical_size)
    candidates: list[tuple[float, tuple[float, float], dict]] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not size * size * 0.002 <= area <= size * size * 0.05:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        aspect = float(width) / max(1.0, float(height))
        perimeter = float(cv2.arcLength(contour, True))
        circularity = 0.0 if perimeter <= 0.0 else 4.0 * math.pi * area / (perimeter * perimeter)
        center = (x + width * 0.5, y + height * 0.5)
        if not 0.55 <= aspect <= 1.55 or circularity < 0.25:
            continue
        if not size * 0.20 <= center[0] <= size * 0.80 or not size * 0.55 <= center[1] <= size * 0.98:
            continue
        distance = float(np.linalg.norm(np.asarray(center) - np.asarray(config.center)))
        if distance > size * 0.28:
            continue
        score = circularity + min(1.0, area / (size * size * 0.015)) - distance / size
        candidates.append(
            (
                score,
                center,
                {
                    "detected": True,
                    "center": [round(center[0], 3), round(center[1], 3)],
                    "circularity": round(circularity, 6),
                    "area_ratio": round(area / (size * size), 6),
                    "offset_px": round(distance, 3),
                },
            )
        )
    if not candidates:
        return config, {"detected": False, "center": [float(config.center[0]), float(config.center[1])]}

    _score, center, quality = max(candidates, key=lambda item: item[0])
    calibration_radius = size * 0.55

    def endpoint(angle: float) -> tuple[float, float]:
        radians = math.radians(angle)
        return (
            config.center[0] + calibration_radius * math.cos(radians),
            config.center[1] - calibration_radius * math.sin(radians),
        )

    adjusted = replace(
        config,
        center=center,
        zero_angle=angle_from_center(center, endpoint(config.zero_angle)),
        full_angle=angle_from_center(center, endpoint(config.full_angle)),
    )
    return adjusted, quality


def find_pointer_candidates(
    image: np.ndarray,
    config: MeterConfig,
    *,
    deadline: float | None = None,
) -> list[PointerCandidate | None]:
    check_deadline(deadline)
    color_mask = prepare_color_pointer_mask(image, config)
    check_deadline(deadline)
    edge_mask = prepare_edge_pointer_mask(image, config)
    check_deadline(deadline)
    gray_mask = prepare_gray_pointer_mask(image, config)
    candidates = [
        find_pointer_by_ray(
            color_mask,
            config,
            method="color-ray",
            start_radius_ratio=0.03,
            end_radius_ratio=0.62,
            strip_radius=2,
            min_run_ratio=0.015,
            min_hits=6,
            max_first_hit_ratio=0.75,
            deadline=deadline,
        ),
        find_pointer_by_hough(
            color_mask,
            config,
            method="color-hough",
            min_line_ratio=0.08,
            near_center_ratio=0.18,
            deadline=deadline,
        ),
        find_pointer_by_hough(
            edge_mask,
            config,
            method="edge-hough",
            min_line_ratio=0.12,
            near_center_ratio=0.10,
            deadline=deadline,
        ),
        find_pointer_by_hough(
            gray_mask,
            config,
            method="gray-hough",
            min_line_ratio=0.14,
            near_center_ratio=0.10,
            deadline=deadline,
        ),
        find_pointer_by_ray(
            gray_mask,
            config,
            method="gray-ray-fallback",
            start_radius_ratio=0.08,
            end_radius_ratio=0.62,
            strip_radius=1,
            min_run_ratio=0.16,
            min_hits=max(25, int(config.canonical_size * 0.03)),
            max_first_hit_ratio=0.30,
            deadline=deadline,
        ),
    ]
    check_deadline(deadline)
    return candidates


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
    *,
    allow_relaxed_quad: bool = False,
    fast_alignment: bool = False,
    deadline: float | None = None,
) -> MeterReadResult:
    check_deadline(deadline)
    if crop.size == 0:
        return MeterReadResult(value=None, det_conf=det_conf, status="crop_failed", bbox=bbox)

    canonical_image, rectify_method, rectify_quality = rectify_meter_crop_detailed(
        crop,
        config,
        allow_relaxed_quad=allow_relaxed_quad,
        fast_alignment=fast_alignment,
        deadline=deadline,
    )
    check_deadline(deadline)
    if canonical_image is None:
        return MeterReadResult(
            value=None,
            det_conf=det_conf,
            status=rectify_method,
            bbox=bbox,
            debug_image=crop,
            rectify_method=rectify_method,
            rectify_quality=rectify_quality,
        )

    pointer_config = config
    center_quality: dict = {"detected": False, "used": "calibrated"}
    geometry_fallback = bool(rectify_quality.get("geometry_fallback"))
    if config.rectify_mode == "auto" and not geometry_fallback:
        pointer_config, center_quality = detect_pointer_center(canonical_image, config)
    check_deadline(deadline)
    raw_candidates = find_pointer_candidates(canonical_image, pointer_config, deadline=deadline)
    if config.rectify_mode == "auto" and not geometry_fallback:
        candidate, consensus_count, candidate_payloads = consensus_pointer_candidate(
            raw_candidates,
            tolerance_deg=1.5,
        )
    else:
        candidate = choose_pointer_candidate(*raw_candidates)
        consensus_count = 1 if candidate is not None else 0
        candidate_payloads = [
            pointer_candidate_payload(value) for value in raw_candidates if value is not None
        ]
    if config.rectify_mode == "auto" and pointer_config is not config and consensus_count < 2:
        calibrated_candidate, calibrated_count, calibrated_payloads = consensus_pointer_candidate(
            find_pointer_candidates(canonical_image, config, deadline=deadline),
            tolerance_deg=1.5,
        )
        if calibrated_count > consensus_count:
            pointer_config = config
            candidate = calibrated_candidate
            consensus_count = calibrated_count
            candidate_payloads = calibrated_payloads
            center_quality = {**center_quality, "used": "calibrated_fallback"}
        else:
            center_quality = {**center_quality, "used": "detected"}
    elif pointer_config is not config:
        center_quality = {**center_quality, "used": "detected"}
    rectify_quality = {**rectify_quality, "pointer_center": center_quality}
    check_deadline(deadline)
    if candidate is None:
        return MeterReadResult(
            value=None,
            det_conf=det_conf,
            status="pointer_failed",
            bbox=bbox,
            canonical_image=canonical_image,
            debug_image=build_debug_image(
                canonical_image,
                pointer_config,
                None,
                "pointer_failed",
                None,
                rectify_method,
            ),
            rectify_method=rectify_method,
            pointer_candidates=candidate_payloads,
            rectify_quality=rectify_quality,
        )

    value = meter_value_from_angle(candidate.angle, pointer_config)
    check_deadline(deadline)
    debug_image = build_debug_image(canonical_image, pointer_config, candidate, "ok", value, rectify_method)
    geometry_score = float(rectify_quality.get("area_ratio", 0.0) or 0.0)
    quality_score = geometry_score * 2.0 + min(3, consensus_count) + float(det_conf or 0.0)
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
        rectify_method=rectify_method,
        pointer_candidates=candidate_payloads,
        pointer_consensus_count=consensus_count,
        rectify_quality=rectify_quality,
        quality_score=quality_score,
    )


def read_meter_from_box(
    image: np.ndarray,
    box: Iterable[float],
    config: MeterConfig,
    det_conf: float | None = None,
    margin_ratio: float = 0.12,
    *,
    allow_relaxed_quad: bool = False,
    fast_alignment: bool = False,
    deadline: float | None = None,
) -> MeterReadResult:
    check_deadline(deadline)
    expanded_box = expand_box(box, image.shape, margin_ratio=margin_ratio)
    crop = crop_box(image, expanded_box)
    return read_meter_from_crop(
        crop,
        config,
        det_conf=det_conf,
        bbox=expanded_box,
        allow_relaxed_quad=allow_relaxed_quad,
        fast_alignment=fast_alignment,
        deadline=deadline,
    )


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
    rectify_method: str | None = None,
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
    if rectify_method:
        summary = f"{summary} {rectify_method}"
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
