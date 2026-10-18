#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4

try:
    import cv2
    import numpy as np
    import requests
except Exception:  # pragma: no cover - board runtime dependency
    cv2 = None
    np = None
    requests = None

try:
    import rclpy
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
except Exception:  # pragma: no cover - board runtime dependency
    rclpy = None
    CancelGoal = None
    Twist = None
    Node = object
    LaserScan = None
    qos_profile_sensor_data = None

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    from camera_yolo_detect import (
        DEFAULT_CLASSES,
        DEFAULT_DIGIT_CLASSES,
        DEFAULT_DIGIT_MODEL,
        DEFAULT_EXTRA_CLASSES,
        DEFAULT_MODEL,
        DEFAULT_OUTPUT_DIR,
        apply_camera_controls,
        build_digit_readings,
        collect_model_specs,
        decode_detections,
        draw_measurement_summary,
        draw_results,
        extract_digit_detections,
        imgsz_for_model,
        label_for_model,
        letterbox,
        load_classes,
        meter_reading_to_cabinet_metric,
        run_meter_readings,
    )
except Exception:  # pragma: no cover - reported during runtime validation
    DEFAULT_MODEL = ROOT_DIR / "yolo_models" / "lightlabel_best.onnx"
    DEFAULT_CLASSES = ROOT_DIR / "yolo_models" / "classes.txt"
    DEFAULT_EXTRA_CLASSES = ROOT_DIR / "yolo_models" / "meter_digit_unified_best_classes.txt"
    DEFAULT_DIGIT_MODEL = ROOT_DIR / "yolo_models" / "digits_best.onnx"
    DEFAULT_DIGIT_CLASSES = ROOT_DIR / "yolo_models" / "digits_classes.txt"
    DEFAULT_OUTPUT_DIR = ROOT_DIR / "runtime" / "yolo_camera"
    apply_camera_controls = None
    build_digit_readings = None
    collect_model_specs = None
    decode_detections = None
    draw_measurement_summary = None
    draw_results = None
    extract_digit_detections = None
    imgsz_for_model = None
    label_for_model = None
    letterbox = None
    load_classes = None
    meter_reading_to_cabinet_metric = None
    run_meter_readings = None


DEFAULT_PROFILE = ROOT_DIR / "cabinet_inspection_profiles" / "default.yaml"
DEFAULT_STATUS_FILE = ROOT_DIR / "runtime" / "cabinet_nav_takeover" / "status.json"
DEFAULT_CAPTURE_DIR = ROOT_DIR / "runtime" / "cabinet_nav_takeover" / "captures"
DIGIT_LABELS = {str(value) for value in range(10)}
DEFAULT_TARGET_LABELS = {
    "dianya",
    "dianliu",
    "yakuang",
    "liukuang",
    "voltage",
    "current",
    "voltage_meter",
    "current_meter",
    "meter",
    "cabinet",
    "distribution_cabinet",
    "digital_display",
    "digit",
    "digits",
    "green_on",
    "green_off",
    "red_on",
    "red_off",
}
STAGE_LABELS = {
    "meters": {
        "dianya",
        "dianliu",
        "yakuang",
        "liukuang",
        "voltage",
        "current",
        "voltage_meter",
        "current_meter",
        "meter",
    },
    "digital_display": {"digital_display", "digit", "digits"} | DIGIT_LABELS,
    "status_lights": {"green_on", "green_off", "red_on", "red_off"},
}
STAGES = ("meters", "digital_display", "status_lights")


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def bool_from_env(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None or str(raw).strip() == "":
        return default
    return str(raw).strip().lower() in {"1", "true", "yes", "on", "enabled"}


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)


def detection_label(item: Any) -> str:
    if not isinstance(item, dict):
        return ""
    raw = str(item.get("class_name") or item.get("label") or item.get("display_name") or "").strip().lower()
    if ":" in raw:
        raw = raw.rsplit(":", 1)[-1].strip()
    return raw


def detection_box(item: Any) -> list[float] | None:
    if not isinstance(item, dict):
        return None
    box = item.get("box_xyxy") or item.get("bbox") or item.get("box")
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        return None
    try:
        x1, y1, x2, y2 = [float(value) for value in box]
    except (TypeError, ValueError):
        return None
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    if x2 <= x1 or y2 <= y1:
        return None
    return [x1, y1, x2, y2]


def union_bbox(boxes: list[list[float]]) -> list[float] | None:
    if not boxes:
        return None
    return [
        min(box[0] for box in boxes),
        min(box[1] for box in boxes),
        max(box[2] for box in boxes),
        max(box[3] for box in boxes),
    ]


def bbox_metrics(box: list[float], width: int, height: int) -> dict[str, float]:
    x1, y1, x2, y2 = box
    cx = ((x1 + x2) * 0.5) / max(1.0, float(width))
    cy = ((y1 + y2) * 0.5) / max(1.0, float(height))
    area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_ratio = area / max(1.0, float(width * height))
    return {
        "center_x": round(cx, 4),
        "center_y": round(cy, 4),
        "error_x": round(cx - 0.5, 4),
        "error_y": round(cy - 0.5, 4),
        "area_ratio": round(area_ratio, 6),
        "box_width_ratio": round(max(0.0, x2 - x1) / max(1.0, float(width)), 4),
        "box_height_ratio": round(max(0.0, y2 - y1) / max(1.0, float(height)), 4),
    }


def rotate_frame(frame: Any, degrees: int) -> Any:
    if degrees == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if degrees == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    if degrees == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame


def clip_text(text: str, limit: int = 1600) -> str:
    return text[-limit:] if len(text) > limit else text


def parse_device(device: str) -> int | str:
    return int(device) if str(device).isdigit() else str(device)


def selected_detections(result: dict[str, Any], labels: set[str]) -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    for source_key in ("detections", "digit_detections"):
        source = result.get(source_key, [])
        if not isinstance(source, list):
            continue
        for item in source:
            label = detection_label(item)
            if label in labels or label in DIGIT_LABELS and labels.intersection(DIGIT_LABELS):
                if detection_box(item) is not None:
                    items.append(item)
    return items


def target_bbox(result: dict[str, Any], labels: set[str]) -> tuple[list[float] | None, int, float]:
    boxes: list[list[float]] = []
    best_conf = 0.0
    for item in selected_detections(result, labels):
        box = detection_box(item)
        if box is None:
            continue
        boxes.append(box)
        try:
            best_conf = max(best_conf, float(item.get("confidence") or 0.0))
        except Exception:
            pass
    return union_bbox(boxes), len(boxes), best_conf


def stage_score(stage: str, result: dict[str, Any], width: int, height: int) -> tuple[float, list[dict[str, Any]]]:
    labels = set(STAGE_LABELS.get(stage, set()))
    matches = selected_detections(result, labels)
    if not matches:
        return 0.0, []
    area_sum = 0.0
    conf_sum = 0.0
    seen_labels: set[str] = set()
    for item in matches:
        box = detection_box(item)
        if box is None:
            continue
        x1, y1, x2, y2 = box
        area_sum += max(0.0, x2 - x1) * max(0.0, y2 - y1) / max(1.0, float(width * height))
        try:
            conf_sum += float(item.get("confidence") or 0.0)
        except Exception:
            pass
        seen_labels.add(detection_label(item))
    avg_conf = conf_sum / max(1, len(matches))
    score = avg_conf * 2.0 + min(3.0, area_sum * 18.0) + min(3.0, len(matches) * 0.35)
    if stage == "meters":
        if seen_labels.intersection({"dianya", "yakuang", "voltage", "voltage_meter"}):
            score += 0.8
        if seen_labels.intersection({"dianliu", "liukuang", "current", "current_meter"}):
            score += 0.8
    elif stage == "digital_display":
        digit_count = sum(1 for item in matches if detection_label(item) in DIGIT_LABELS)
        score += min(3.0, digit_count * 0.45)
    elif stage == "status_lights":
        score += min(2.0, len(seen_labels.intersection(STAGE_LABELS["status_lights"])) * 0.5)
    return round(score, 4), matches


def cabinet_data_from_result(stage: str, result: dict[str, Any]) -> dict[str, Any]:
    if stage == "meters":
        update: dict[str, Any] = {}
        for reading in result.get("meter_readings", []) if isinstance(result.get("meter_readings"), list) else []:
            if meter_reading_to_cabinet_metric is None:
                continue
            metric = meter_reading_to_cabinet_metric(reading)
            meter_type = str(reading.get("meter_type") or "")
            if isinstance(metric, dict) and meter_type:
                update[meter_type] = metric
        return update
    if stage == "digital_display":
        readings = result.get("digit_readings", {}) if isinstance(result.get("digit_readings"), dict) else {}
        text = str(readings.get("global_text") or "")
        rows = readings.get("rows", [])
        return {
            "digital_display": {
                "text": text,
                "rows": rows if isinstance(rows, list) else [],
                "status": "ok" if text else "unreadable",
                "captured_at": utc_now_iso(),
            }
        }
    if stage == "status_lights":
        detections = result.get("detections", []) if isinstance(result.get("detections"), list) else []
        lights = [
            {
                "label": detection_label(item),
                "confidence": item.get("confidence"),
                "box_xyxy": item.get("box_xyxy", []),
            }
            for item in detections
            if detection_label(item) in STAGE_LABELS["status_lights"]
        ]
        return {"status_lights": {"items": lights, "status": "ok" if lights else "unreadable", "captured_at": utc_now_iso()}}
    return {}


@dataclass
class BestFrame:
    score: float = 0.0
    image: Any = None
    result: dict[str, Any] = field(default_factory=dict)
    selected_count: int = 0
    updated_at: str = ""

    def snapshot(self) -> dict[str, Any]:
        return {
            "captured": self.image is not None,
            "score": round(float(self.score), 4),
            "selected_count": int(self.selected_count),
            "updated_at": self.updated_at,
        }


class StreamDetector:
    def __init__(self, args: argparse.Namespace):
        if cv2 is None or np is None:
            raise RuntimeError("OpenCV/numpy are unavailable")
        missing_helpers = [
            name
            for name, value in {
                "collect_model_specs": collect_model_specs,
                "load_classes": load_classes,
                "letterbox": letterbox,
                "decode_detections": decode_detections,
            }.items()
            if value is None
        ]
        if missing_helpers:
            raise RuntimeError(f"camera_yolo_detect helpers unavailable: {', '.join(missing_helpers)}")
        self.args = args
        self.models: list[dict[str, Any]] = []
        self.digit_model: dict[str, Any] | None = None
        self.model_warnings: list[str] = []
        self._load_models()

    def _load_backend(self, model_path: Path, backend: str) -> tuple[Any, str]:
        rknn_error: Exception | None = None
        if backend in ("auto", "rknn"):
            try:
                from rknnlite.api import RKNNLite

                rknn_path = Path(self.args.rknn_model_dir) / f"{model_path.stem}.rknn"
                if not rknn_path.is_file():
                    raise FileNotFoundError(f"RKNN model not found: {rknn_path}")
                runtime = RKNNLite()
                code = runtime.load_rknn(str(rknn_path))
                if code != 0:
                    raise RuntimeError(f"load_rknn failed with code {code}: {rknn_path}")
                code = runtime.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
                if code != 0:
                    runtime.release()
                    raise RuntimeError(f"init_runtime failed with code {code}: {rknn_path}")
                return runtime, "rknn"
            except Exception as exc:
                if backend == "rknn":
                    raise
                rknn_error = exc

        ort_error: Exception | None = None
        if backend in ("auto", "onnxruntime"):
            try:
                import onnxruntime as ort

                session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
                return session, "onnxruntime"
            except Exception as exc:
                if backend == "onnxruntime":
                    raise
                ort_error = exc
        try:
            net = cv2.dnn.readNetFromONNX(str(model_path))
            return net, "opencv"
        except Exception as exc:
            if ort_error is not None:
                raise RuntimeError(
                    f"rknn failed: {rknn_error}; onnxruntime failed: {ort_error}; "
                    f"opencv failed: {exc}"
                ) from exc
            raise

    def _load_models(self) -> None:
        spec_args = argparse.Namespace(
            model=self.args.model,
            classes=self.args.classes,
            extra_model=list(self.args.extra_model or []),
            extra_classes=list(self.args.extra_classes or []),
            imgsz=self.args.imgsz,
        )
        specs, warnings = collect_model_specs(spec_args)
        self.model_warnings.extend(warnings)
        for spec in specs:
            model_path = Path(spec["model"])
            classes = load_classes(Path(spec["classes"]))
            runtime, backend = self._load_backend(model_path, self.args.backend)
            if bool(self.args.require_rknn) and backend != "rknn":
                raise RuntimeError(f"RKNN backend required but unavailable for {model_path}")
            self.models.append(
                {
                    "model": model_path,
                    "classes": classes,
                    "classes_path": Path(spec["classes"]),
                    "label": str(spec["label"]),
                    "imgsz": int(spec["imgsz"]),
                    "runtime": runtime,
                    "backend": backend,
                }
            )

        if not bool(self.args.no_digits) and Path(self.args.digit_model).is_file():
            classes = load_classes(Path(self.args.digit_classes))
            runtime, backend = self._load_backend(Path(self.args.digit_model), self.args.backend)
            self.digit_model = {
                "model": Path(self.args.digit_model),
                "classes": classes,
                "classes_path": Path(self.args.digit_classes),
                "label": label_for_model(Path(self.args.digit_model)) if label_for_model else "digits",
                "imgsz": int(self.args.digit_imgsz)
                if self.args.digit_imgsz is not None
                else (
                    imgsz_for_model(Path(self.args.digit_model), self.args.imgsz)
                    if imgsz_for_model
                    else int(self.args.imgsz)
                ),
                "runtime": runtime,
                "backend": backend,
            }

    def _run_one(self, model: dict[str, Any], image: Any, conf: float) -> tuple[list[dict[str, Any]], float]:
        blob_image, scale, pad = letterbox(image, int(model["imgsz"]))
        started = time.perf_counter()
        if model["backend"] == "rknn":
            rgb = np.ascontiguousarray(
                np.expand_dims(cv2.cvtColor(blob_image, cv2.COLOR_BGR2RGB), axis=0)
            )
            outputs = model["runtime"].inference(inputs=[rgb])
            if not outputs:
                raise RuntimeError(f"RKNN returned no output for {model['model']}")
            if len(outputs) != 1:
                raise RuntimeError(
                    f"RKNN output contract mismatch for {model['model']}: expected 1 tensor, got {len(outputs)}"
                )
            output = outputs[0]
        else:
            blob = cv2.dnn.blobFromImage(
                blob_image,
                scalefactor=1.0 / 255.0,
                size=(int(model["imgsz"]), int(model["imgsz"])),
                swapRB=True,
            )
        if model["backend"] == "onnxruntime":
            input_name = model["runtime"].get_inputs()[0].name
            output = model["runtime"].run(None, {input_name: blob.astype(np.float32)})[0]
        elif model["backend"] == "opencv":
            model["runtime"].setInput(blob)
            output = model["runtime"].forward()
        infer_ms = (time.perf_counter() - started) * 1000.0
        detections = decode_detections(
            output,
            image.shape[:2],
            model["classes"],
            scale,
            pad,
            conf,
            float(self.args.iou),
        )
        return detections, infer_ms

    def detect(
        self,
        image: Any,
        *,
        include_digits: bool = True,
        model_indices: set[int] | None = None,
    ) -> dict[str, Any]:
        all_detections: list[dict[str, Any]] = []
        backend_parts: list[str] = []
        class_payload: list[dict[str, Any]] = []
        total_infer_ms = 0.0
        multi_model = len(self.models) > 1
        for model_index, model in enumerate(self.models):
            if model_indices is not None and model_index not in model_indices:
                continue
            detections, infer_ms = self._run_one(model, image, float(self.args.conf))
            for detection in detections:
                detection["model"] = str(model["label"])
                detection["source"] = "yolo"
                if multi_model:
                    detection["display_name"] = f'{model["label"]}:{detection["class_name"]}'
            all_detections.extend(detections)
            total_infer_ms += infer_ms
            backend_parts.append(f'{model["label"]}:{model["backend"]}')
            class_payload.append(
                {
                    "model": str(model["label"]),
                    "model_path": str(model["model"]),
                    "classes_path": str(model["classes_path"]),
                    "imgsz": int(model["imgsz"]),
                    "names": list(model["classes"]),
                }
            )

        digit_detections = extract_digit_detections(all_detections) if extract_digit_detections else []
        if include_digits and not digit_detections and self.digit_model is not None:
            detections, infer_ms = self._run_one(
                self.digit_model,
                image,
                float(self.args.digit_conf if self.args.digit_conf is not None else self.args.conf),
            )
            for detection in detections:
                detection["model"] = str(self.digit_model["label"])
                detection["source"] = "digit"
                detection["display_name"] = f'digit:{detection["class_name"]}'
            digit_detections = detections
            all_detections.extend(detections)
            total_infer_ms += infer_ms
            backend_parts.append(f'{self.digit_model["label"]}:{self.digit_model["backend"]}')
            class_payload.append(
                {
                    "model": str(self.digit_model["label"]),
                    "model_path": str(self.digit_model["model"]),
                    "classes_path": str(self.digit_model["classes_path"]),
                    "imgsz": int(self.digit_model["imgsz"]),
                    "names": list(self.digit_model["classes"]),
                }
            )

        all_detections.sort(key=lambda item: float(item.get("confidence") or 0.0), reverse=True)
        digit_detections.sort(key=lambda item: float(item.get("confidence") or 0.0), reverse=True)
        digit_readings = build_digit_readings(digit_detections, [], image.shape[:2]) if build_digit_readings else {}
        return {
            "model": ", ".join(entry["model_path"] for entry in class_payload),
            "models": [
                {
                    "label": entry["model"],
                    "model": entry["model_path"],
                    "classes": entry["classes_path"],
                    "imgsz": entry["imgsz"],
                }
                for entry in class_payload
            ],
            "classes": class_payload,
            "backend": ",".join(backend_parts),
            "inference_ms": round(total_infer_ms, 3),
            "model_warnings": list(self.model_warnings),
            "detections": all_detections,
            "digit_detections": digit_detections,
            "digit_readings": digit_readings,
            "cabinet_data": {},
        }

    def close(self) -> None:
        runtimes = [item.get("runtime") for item in self.models]
        if self.digit_model is not None:
            runtimes.append(self.digit_model.get("runtime"))
        for runtime in runtimes:
            release = getattr(runtime, "release", None)
            if callable(release):
                try:
                    release()
                except Exception:
                    pass


class TakeoverNode(Node):
    def __init__(self, cmd_vel_topic: str, scan_topic: str, front_angle_deg: float):
        super().__init__("cabinet_nav_takeover")
        self.cmd_vel_topic = cmd_vel_topic
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.front_min = float("inf")
        self.last_scan_at = 0.0
        self.front_angle_rad = math.radians(max(5.0, min(45.0, front_angle_deg)))
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg: LaserScan) -> None:
        front_ranges: list[float] = []
        for index, value in enumerate(msg.ranges):
            if not math.isfinite(float(value)):
                continue
            if value < msg.range_min or value > msg.range_max:
                continue
            angle = float(msg.angle_min) + float(index) * float(msg.angle_increment)
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= self.front_angle_rad:
                front_ranges.append(float(value))
        self.front_min = min(front_ranges) if front_ranges else float("inf")
        self.last_scan_at = time.time()

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop(self, repeats: int = 3) -> None:
        for _ in range(max(1, repeats)):
            self.publish_twist(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)


class CabinetNavTakeover:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.status_file = Path(args.status_file)
        self.capture_dir = Path(args.capture_dir)
        self.labels = {
            str(item).strip().lower()
            for item in str(args.target_labels).split(",")
            if str(item).strip()
        } or set(DEFAULT_TARGET_LABELS)
        self.state = "starting"
        self.target_seen_count = 0
        self.lost_count = 0
        self.takeover_active = False
        self.started_at = time.time()
        self.inspection_id = f"stream_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid4().hex[:6]}"
        self.last_detection: dict[str, Any] = {}
        self.last_error = ""
        self.result: dict[str, Any] = {}
        self.logs: list[str] = []
        self.node: TakeoverNode | None = None
        self.detector: StreamDetector | None = None
        self.camera: Any = None
        self.best_frames: dict[str, BestFrame] = {stage: BestFrame() for stage in STAGES}

    def log(self, message: str) -> None:
        entry = f"{utc_now_iso()} {message}"
        self.logs.append(entry)
        self.logs = self.logs[-80:]
        print(f"[cabinet-nav-takeover] {message}", file=sys.stderr, flush=True)

    def capture_progress(self) -> dict[str, Any]:
        return {stage: self.best_frames[stage].snapshot() for stage in STAGES}

    def update_status(self, **extra: Any) -> None:
        payload = {
            "state": self.state,
            "takeover_active": bool(self.takeover_active),
            "target_seen_count": int(self.target_seen_count),
            "lost_count": int(self.lost_count),
            "front_min": None
            if self.node is None or not math.isfinite(float(self.node.front_min))
            else round(float(self.node.front_min), 3),
            "capture_progress": self.capture_progress(),
            "inspection_id": self.inspection_id,
            "last_detection": dict(self.last_detection),
            "last_error": self.last_error,
            "result": dict(self.result),
            "logs": list(self.logs[-20:]),
            "pid": os.getpid(),
            "updated_at": utc_now_iso(),
        }
        payload.update(extra)
        atomic_write_json(self.status_file, payload)

    def open_camera(self) -> None:
        if apply_camera_controls is not None:
            report = apply_camera_controls(self.args.device, self.args.camera_profile, {})
            for error in report.get("errors", []):
                self.log(f"camera control warning: {error}")
        camera = cv2.VideoCapture(parse_device(self.args.device))
        if not camera.isOpened():
            raise RuntimeError(f"failed to open camera: {self.args.device}")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.args.preview_width))
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.args.preview_height))
        camera.set(cv2.CAP_PROP_FPS, int(self.args.fps))
        with contextlib_suppress():
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.camera = camera
        for _ in range(3):
            camera.read()

    def read_frame(self) -> Any:
        if self.camera is None:
            raise RuntimeError("camera is not open")
        ok, frame = self.camera.read()
        if not ok or frame is None:
            raise RuntimeError(f"failed to read frame from camera: {self.args.device}")
        return rotate_frame(frame, int(self.args.rotate))

    def detect_frame(self, frame: Any) -> dict[str, Any]:
        if self.detector is None:
            raise RuntimeError("detector is not initialized")
        return self.detector.detect(frame)

    def update_best_frames(self, frame: Any, result: dict[str, Any]) -> None:
        height, width = frame.shape[:2]
        for stage in STAGES:
            score, matches = stage_score(stage, result, width, height)
            if score <= 0.0:
                continue
            current = self.best_frames[stage]
            if score > current.score:
                self.best_frames[stage] = BestFrame(
                    score=score,
                    image=frame.copy(),
                    result=json.loads(json.dumps(result, ensure_ascii=False)),
                    selected_count=len(matches),
                    updated_at=utc_now_iso(),
                )

    def cancel_nav_goals(self) -> bool:
        if self.node is None or CancelGoal is None:
            return False
        client = self.node.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        if not client.wait_for_service(timeout_sec=2.0):
            self.last_error = "navigate_to_pose cancel service unavailable"
            return False
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        return future.done()

    def monitor_navigation(self) -> bool:
        self.state = "monitoring_navigation"
        self.update_status()
        deadline = time.time() + max(5.0, float(self.args.search_timeout_s))
        period = 1.0 / max(0.2, float(self.args.monitor_rate_hz))
        while time.time() < deadline:
            loop_started = time.time()
            if self.node is not None:
                rclpy.spin_once(self.node, timeout_sec=0.01)
            try:
                frame = self.read_frame()
                result = self.detect_frame(frame)
                self.update_best_frames(frame, result)
                box, count, confidence = target_bbox(result, self.labels)
            except Exception as exc:
                self.last_error = str(exc)
                box, count, confidence = None, 0, 0.0

            if box is None:
                self.target_seen_count = 0
                self.last_detection = {"target_found": False, "selected_count": 0}
            else:
                self.target_seen_count += 1
                height, width = frame.shape[:2]
                self.last_detection = {
                    "target_found": True,
                    "selected_count": count,
                    "confidence": round(confidence, 4),
                    "bbox": [round(value, 2) for value in box],
                    "metrics": bbox_metrics(box, width, height),
                }
            self.update_status()
            if self.target_seen_count >= int(self.args.confirm_frames):
                self.state = "target_confirmed"
                self.update_status()
                return True
            time.sleep(max(0.02, period - (time.time() - loop_started)))
        self.state = "idle_no_target"
        self.result = {"message": "no cabinet target confirmed"}
        self.update_status()
        return False

    def visual_approach(self) -> bool:
        if self.node is None:
            return False
        self.state = "visual_approach"
        self.update_status()
        period = 1.0 / max(1.0, float(self.args.control_rate_hz))
        ready_since = 0.0
        deadline = time.time() + max(10.0, float(self.args.approach_timeout_s))
        while time.time() < deadline:
            loop_started = time.time()
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if self.node.front_min < float(self.args.front_emergency_m):
                self.node.stop(repeats=5)
                self.state = "failed"
                self.result = {"message": "front obstacle too close", "front_min": round(self.node.front_min, 3)}
                self.update_status()
                return False

            try:
                frame = self.read_frame()
                result = self.detect_frame(frame)
                self.update_best_frames(frame, result)
                box, count, confidence = target_bbox(result, self.labels)
            except Exception as exc:
                self.last_error = str(exc)
                box, count, confidence = None, 0, 0.0

            if box is None:
                self.lost_count += 1
                self.last_detection = {"target_found": False, "selected_count": 0}
                self.update_status()
                self.node.stop(repeats=2)
                if self.lost_count >= int(self.args.max_lost_frames):
                    self.state = "failed"
                    self.result = {"message": "target lost during visual approach"}
                    self.update_status()
                    return False
                time.sleep(period)
                continue

            self.lost_count = 0
            height, width = frame.shape[:2]
            metrics = bbox_metrics(box, width, height)
            area = float(metrics["area_ratio"])
            error_x = float(metrics["error_x"])
            centered = abs(error_x) <= float(self.args.center_tolerance)
            near_enough = area >= float(self.args.target_area_min) or self.node.front_min <= float(self.args.front_caution_m)
            too_large = area >= float(self.args.target_area_max)
            self.last_detection = {
                "target_found": True,
                "selected_count": count,
                "confidence": round(confidence, 4),
                "bbox": [round(value, 2) for value in box],
                "metrics": metrics,
                "centered": centered,
                "near_enough": near_enough,
            }
            self.update_status()

            if centered and (near_enough or too_large):
                if ready_since <= 0.0:
                    ready_since = time.time()
                if time.time() - ready_since >= float(self.args.ready_hold_s):
                    self.node.stop(repeats=5)
                    self.state = "arrived"
                    self.update_status()
                    return True
            else:
                ready_since = 0.0

            angular = clamp(-float(self.args.angular_gain) * error_x, -float(self.args.max_angular), float(self.args.max_angular))
            linear = 0.0
            if not near_enough and self.node.front_min > float(self.args.front_caution_m):
                linear = float(self.args.max_linear)
            if too_large:
                linear = 0.0
            if abs(angular) < 0.025:
                angular = 0.0
            self.node.publish_twist(linear, angular)
            time.sleep(max(0.02, period - (time.time() - loop_started)))

        self.node.stop(repeats=5)
        self.state = "failed"
        self.result = {"message": "visual approach timed out"}
        self.update_status()
        return False

    def view_refine(self) -> None:
        if self.node is None:
            return
        self.state = "view_refine"
        self.update_status()
        period = 1.0 / max(1.0, float(self.args.control_rate_hz))
        deadline = time.time() + max(1.0, float(self.args.refine_duration_s))
        pattern = [1.0, 1.0, 0.0, -1.0, -1.0, 0.0]
        index = 0
        step_deadline = time.time()
        direction = 0.0
        while time.time() < deadline:
            loop_started = time.time()
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if time.time() >= step_deadline:
                direction = pattern[index % len(pattern)]
                index += 1
                step_deadline = time.time() + max(0.2, float(self.args.refine_step_s))
            try:
                frame = self.read_frame()
                result = self.detect_frame(frame)
                self.update_best_frames(frame, result)
                box, count, confidence = target_bbox(result, self.labels)
                if box is not None:
                    height, width = frame.shape[:2]
                    self.last_detection = {
                        "target_found": True,
                        "selected_count": count,
                        "confidence": round(confidence, 4),
                        "bbox": [round(value, 2) for value in box],
                        "metrics": bbox_metrics(box, width, height),
                    }
            except Exception as exc:
                self.last_error = str(exc)
            self.update_status()
            self.node.publish_twist(0.0, direction * float(self.args.refine_angular))
            time.sleep(max(0.02, period - (time.time() - loop_started)))
        self.node.stop(repeats=5)

    def finalize_stage_result(self, stage: str, best: BestFrame) -> dict[str, Any]:
        if best.image is None:
            return {"stage": stage, "status": "missing", "message": "no usable frame"}
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        raw_path = self.capture_dir / f"{self.inspection_id}_{stage}_raw.jpg"
        annotated_path = self.capture_dir / f"{self.inspection_id}_{stage}_det.jpg"
        result = json.loads(json.dumps(best.result, ensure_ascii=False))
        result["inspection_id"] = self.inspection_id
        result["inspection_stage"] = stage
        result["inspection_stage_label"] = stage
        result["board_id"] = self.args.board_id
        result["board_label"] = self.args.board_label
        result["source_image"] = str(raw_path)
        result["annotated_image"] = str(annotated_path)
        result["stream_best_score"] = round(float(best.score), 4)
        result["stream_selected_count"] = int(best.selected_count)
        result["captured_at"] = utc_now_iso()

        meter_readings: list[dict[str, Any]] = []
        meter_warnings: list[str] = []
        if stage == "meters" and run_meter_readings is not None:
            meter_args = argparse.Namespace(
                no_meter_readings=False,
                meter_margin=float(self.args.meter_margin),
                save_meter_debug=bool(self.args.save_meter_debug),
            )
            meter_readings, meter_warnings = run_meter_readings(
                best.image,
                result.get("detections", []),
                meter_args,
                self.capture_dir,
                f"{self.inspection_id}_{stage}",
            )
        result["meter_readings"] = meter_readings
        result["model_warnings"] = list(result.get("model_warnings", [])) + meter_warnings
        result["digit_readings"] = build_digit_readings(
            result.get("digit_detections", []),
            meter_readings,
            best.image.shape[:2],
        ) if build_digit_readings else result.get("digit_readings", {})
        cabinet_update = cabinet_data_from_result(stage, result)
        if cabinet_update:
            existing = result.get("cabinet_data", {})
            if not isinstance(existing, dict):
                existing = {}
            existing.update(cabinet_update)
            result["cabinet_data"] = existing

        if not cv2.imwrite(str(raw_path), best.image):
            raise RuntimeError(f"failed to write {raw_path}")
        annotated = draw_results(best.image, result.get("detections", []), float(result.get("inference_ms") or 0.0), str(result.get("backend") or "")) if draw_results else best.image.copy()
        if draw_measurement_summary is not None:
            annotated = draw_measurement_summary(annotated, meter_readings, result.get("digit_readings", {}))
        if not cv2.imwrite(str(annotated_path), annotated):
            raise RuntimeError(f"failed to write {annotated_path}")

        try:
            uploaded = self.upload_recognition(result)
            result["cloud_upload"] = uploaded
            result["status"] = "completed"
            result["message"] = "stage uploaded"
        except Exception as exc:
            result["status"] = "failed"
            result["message"] = f"recognition upload failed: {exc}"
        return result

    def upload_recognition(self, result: dict[str, Any]) -> dict[str, Any]:
        if requests is None:
            raise RuntimeError("requests is unavailable")
        raw_path = Path(str(result.get("source_image") or ""))
        det_path = Path(str(result.get("annotated_image") or ""))
        if not raw_path.exists() or not det_path.exists():
            raise RuntimeError("recognition output images are missing")
        url = f"{self.args.server_url.rstrip('/')}/api/upload/recognition"
        headers = {"X-Upload-Token": self.args.upload_token}
        with raw_path.open("rb") as raw_handle, det_path.open("rb") as det_handle:
            response = requests.post(
                url,
                headers=headers,
                data={"payload": json.dumps(result, ensure_ascii=False)},
                files={
                    "raw_image": (raw_path.name, raw_handle, "image/jpeg"),
                    "annotated_image": (det_path.name, det_handle, "image/jpeg"),
                },
                timeout=45,
            )
        response.raise_for_status()
        payload = response.json()
        return payload.get("recognition", {}) if isinstance(payload, dict) else {}

    def upload_inspection_summary(self, summary: dict[str, Any]) -> dict[str, Any]:
        if requests is None:
            raise RuntimeError("requests is unavailable")
        response = requests.post(
            f"{self.args.server_url.rstrip('/')}/api/upload/inspection",
            headers={"X-Upload-Token": self.args.upload_token},
            json=summary,
            timeout=30,
        )
        response.raise_for_status()
        payload = response.json()
        return payload if isinstance(payload, dict) else {}

    def upload_results(self) -> bool:
        self.state = "capture_best_frames"
        self.update_status()
        stage_results: list[dict[str, Any]] = []
        cabinet_update: dict[str, Any] = {}
        for stage in STAGES:
            try:
                result = self.finalize_stage_result(stage, self.best_frames[stage])
            except Exception as exc:
                result = {"stage": stage, "status": "failed", "message": str(exc)}
            stage_results.append(
                {
                    "stage": stage,
                    "label": stage,
                    "status": str(result.get("status") or "failed"),
                    "message": str(result.get("message") or ""),
                    "recognition": result.get("cloud_upload", {}),
                    "cabinet_update": result.get("cabinet_data", {}) if isinstance(result.get("cabinet_data"), dict) else {},
                    "adjustment": {
                        "stream_best_score": result.get("stream_best_score", 0.0),
                        "stream_selected_count": result.get("stream_selected_count", 0),
                    },
                }
            )
            if isinstance(result.get("cabinet_data"), dict):
                cabinet_update.update(result.get("cabinet_data", {}))
            self.update_status()

        completed_count = sum(1 for item in stage_results if item["status"] == "completed")
        missing = [item["stage"] for item in stage_results if item["status"] != "completed"]
        summary = {
            "inspection_id": self.inspection_id,
            "status": "completed" if completed_count == len(STAGES) else "partial" if completed_count > 0 else "failed",
            "profile": "stream_yolo_takeover",
            "board_id": self.args.board_id,
            "board_label": self.args.board_label,
            "started_at": utc_now_iso(),
            "completed_at": utc_now_iso(),
            "stages": stage_results,
            "missing_stages": missing,
            "cabinet_update": cabinet_update,
            "capture_progress": self.capture_progress(),
            "logs": list(self.logs[-80:]),
        }
        self.state = "upload_results"
        self.update_status()
        try:
            uploaded = self.upload_inspection_summary(summary)
            summary["cloud_upload"] = uploaded.get("inspection", uploaded) if isinstance(uploaded, dict) else uploaded
        except Exception as exc:
            summary["upload_error"] = str(exc)
            if summary["status"] == "completed":
                summary["status"] = "partial"
        self.result = summary
        self.state = "completed" if summary["status"] in {"completed", "partial"} else "failed"
        self.update_status()
        return self.state == "completed"

    def run(self) -> int:
        if rclpy is None or Twist is None or LaserScan is None:
            raise RuntimeError("ROS2 Python packages are unavailable")
        if not self.args.server_url:
            raise RuntimeError("missing server URL")
        if not self.args.upload_token:
            raise RuntimeError("missing upload token")

        self.detector = StreamDetector(self.args)
        self.open_camera()
        rclpy.init(args=None)
        self.node = TakeoverNode(self.args.cmd_vel_topic, self.args.scan_topic, self.args.front_angle_deg)
        try:
            if not self.monitor_navigation():
                return 0
            self.takeover_active = True
            self.state = "target_confirmed"
            self.update_status()
            self.cancel_nav_goals()
            self.node.stop(repeats=8)
            if not self.visual_approach():
                return 1
            self.view_refine()
            ok = self.upload_results()
            self.node.stop(repeats=8)
            return 0 if ok else 1
        finally:
            if self.node is not None:
                self.node.stop(repeats=3)
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if self.camera is not None:
                self.camera.release()


class contextlib_suppress:
    def __enter__(self):
        return None

    def __exit__(self, exc_type, exc, tb):
        return True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Streaming YOLO cabinet takeover during Nav2 navigation.")
    parser.add_argument("--server-url", default=os.environ.get("CAR_CLOUD_SERVER_URL", ""))
    parser.add_argument("--upload-token", default=os.environ.get("CAR_CLOUD_UPLOAD_TOKEN", ""))
    parser.add_argument("--board-id", default=os.environ.get("CAR_CLOUD_BOARD_ID", "rk3588-f103-board"))
    parser.add_argument("--board-label", default=os.environ.get("CAR_CLOUD_BOARD_LABEL", "RK3588 F103 Board"))
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE))
    parser.add_argument("--status-file", default=str(DEFAULT_STATUS_FILE))
    parser.add_argument("--capture-dir", default=str(DEFAULT_CAPTURE_DIR))
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--preview-width", type=int, default=1280)
    parser.add_argument("--preview-height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=180)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--backend", choices=("auto", "rknn", "onnxruntime", "opencv"), default="auto")
    parser.add_argument("--rknn-model-dir", type=Path, default=ROOT_DIR / "yolo_models" / "rknn")
    parser.add_argument("--require-rknn", action="store_true", default=False)
    parser.add_argument("--camera-profile", default="normal")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--classes", type=Path, default=DEFAULT_CLASSES)
    parser.add_argument("--extra-model", action="append", type=Path, default=[])
    parser.add_argument("--extra-classes", action="append", type=Path, default=[])
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--digit-model", type=Path, default=DEFAULT_DIGIT_MODEL)
    parser.add_argument("--digit-classes", type=Path, default=DEFAULT_DIGIT_CLASSES)
    parser.add_argument("--digit-conf", type=float, default=None)
    parser.add_argument("--digit-imgsz", type=int, default=None)
    parser.add_argument("--no-digits", action="store_true", default=False)
    parser.add_argument("--meter-margin", type=float, default=0.12)
    parser.add_argument("--save-meter-debug", action="store_true", default=False)
    parser.add_argument("--target-labels", default=",".join(sorted(DEFAULT_TARGET_LABELS)))
    parser.add_argument("--confirm-frames", type=int, default=3)
    parser.add_argument("--max-lost-frames", type=int, default=4)
    parser.add_argument("--monitor-rate-hz", type=float, default=2.0)
    parser.add_argument("--control-rate-hz", type=float, default=5.0)
    parser.add_argument("--preview-interval-s", type=float, default=1.2)
    parser.add_argument("--preview-timeout-s", type=float, default=45.0)
    parser.add_argument("--search-timeout-s", type=float, default=3600.0)
    parser.add_argument("--approach-timeout-s", type=float, default=90.0)
    parser.add_argument("--inspection-timeout-s", type=float, default=1200.0)
    parser.add_argument("--max-linear", type=float, default=0.04)
    parser.add_argument("--max-angular", type=float, default=0.16)
    parser.add_argument("--angular-gain", type=float, default=0.55)
    parser.add_argument("--center-tolerance", type=float, default=0.08)
    parser.add_argument("--target-area-min", type=float, default=0.08)
    parser.add_argument("--target-area-max", type=float, default=0.45)
    parser.add_argument("--ready-hold-s", type=float, default=1.0)
    parser.add_argument("--front-angle-deg", type=float, default=18.0)
    parser.add_argument("--front-caution-m", type=float, default=0.35)
    parser.add_argument("--front-emergency-m", type=float, default=0.25)
    parser.add_argument("--refine-duration-s", type=float, default=8.0)
    parser.add_argument("--refine-step-s", type=float, default=0.7)
    parser.add_argument("--refine-angular", type=float, default=0.10)
    parser.add_argument(
        "--allow-uncalibrated",
        action="store_true",
        default=bool_from_env("CAR_CLOUD_CABINET_TAKEOVER_ALLOW_UNCALIBRATED", True),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    runner = CabinetNavTakeover(args)
    try:
        return runner.run()
    except KeyboardInterrupt:
        runner.state = "stopped"
        runner.result = {"message": "stopped"}
        runner.update_status()
        return 130
    except Exception as exc:
        runner.state = "failed"
        runner.last_error = str(exc)
        runner.result = {"message": str(exc)}
        runner.update_status()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
