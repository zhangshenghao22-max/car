#!/usr/bin/env python3
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing
import os
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

ROOT = Path(
    os.environ.get("CAR2_BOARD_DEPLOY_ROOT")
    or os.environ.get("CAR_CLOUD_UPLOADER_ROOT")
    or Path(__file__).resolve().parents[1]
)
EXTRA_SITE = ROOT / "yolo_runtime" / "python"
if EXTRA_SITE.is_dir():
    sys.path.insert(0, str(EXTRA_SITE))

import cv2
import numpy as np


DEFAULT_MODEL = ROOT / "yolo_models" / "lightlabel_best.onnx"
DEFAULT_CLASSES = ROOT / "yolo_models" / "classes.txt"
DEFAULT_EXTRA_MODEL = ROOT / "yolo_models" / "meter_digit_unified_best.onnx"
DEFAULT_EXTRA_CLASSES = ROOT / "yolo_models" / "meter_digit_unified_best_classes.txt"
DEFAULT_DIGIT_MODEL = ROOT / "yolo_models" / "digits_best.onnx"
DEFAULT_DIGIT_CLASSES = ROOT / "yolo_models" / "digits_classes.txt"
DEFAULT_SWITCH_MODEL = ROOT / "yolo_models" / "switch_yolo_best.onnx"
DEFAULT_SWITCH_CLASSES = ROOT / "yolo_models" / "switch_yolo_classes.txt"
DEFAULT_METER_DIR = ROOT / "meter_reader"
DEFAULT_VOLTAGE_CONFIG = DEFAULT_METER_DIR / "meter_config_voltage.yaml"
DEFAULT_CURRENT_CONFIG = DEFAULT_METER_DIR / "meter_config_current.yaml"
DEFAULT_OUTPUT_DIR = ROOT / "runtime" / "yolo_camera"
DEFAULT_WORKER_SOCKET = ROOT / "runtime" / "recognition_worker" / "worker.sock"
UNIFIED_MODEL_IMGSZ = 1280

METER_TARGETS: Mapping[str, dict] = {
    "voltage": {
        "display_name": "voltage",
        "unit": "V",
        "labels": {"yakuang", "dianya", "voltage", "voltage_meter"},
        "config": DEFAULT_VOLTAGE_CONFIG,
    },
    "current": {
        "display_name": "current",
        "unit": "A",
        "labels": {"liukuang", "dianliu", "current", "current_meter"},
        "config": DEFAULT_CURRENT_CONFIG,
    },
}

DIGIT_LABELS = {str(value) for value in range(10)}


V4L2_CONTROL_IDS: Mapping[str, int] = {
    "brightness": 0x00980900,
    "contrast": 0x00980901,
    "saturation": 0x00980902,
    "hue": 0x00980903,
    "white_balance_temperature_auto": 0x0098090C,
    "gamma": 0x00980910,
    "gain": 0x00980913,
    "power_line_frequency": 0x00980918,
    "white_balance_temperature": 0x0098091A,
    "sharpness": 0x0098091B,
    "backlight_compensation": 0x0098091C,
    "exposure_auto": 0x009A0901,
    "exposure_absolute": 0x009A0902,
}


CAMERA_CONTROL_PROFILES: Mapping[str, Mapping[str, int]] = {
    "off": {},
    "reset": {
        "brightness": 0,
        "contrast": 34,
        "saturation": 40,
        "white_balance_temperature_auto": 1,
        "gamma": 110,
        "gain": 0,
        "power_line_frequency": 1,
        "sharpness": 26,
        "backlight_compensation": 0,
        "exposure_auto": 3,
    },
    "normal": {
        "brightness": 0,
        "contrast": 32,
        "saturation": 44,
        "white_balance_temperature_auto": 1,
        "gamma": 125,
        "gain": 0,
        "power_line_frequency": 1,
        "sharpness": 24,
        "backlight_compensation": 4,
        "exposure_auto": 3,
    },
    "bright": {
        "brightness": 0,
        "contrast": 32,
        "saturation": 48,
        "white_balance_temperature_auto": 1,
        "gamma": 145,
        "gain": 0,
        "power_line_frequency": 1,
        "sharpness": 24,
        "backlight_compensation": 16,
        "exposure_auto": 3,
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture one USB camera frame and run YOLO ONNX detection with OpenCV DNN."
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="ONNX model path.")
    parser.add_argument("--classes", type=Path, default=DEFAULT_CLASSES, help="Class-name file path.")
    parser.add_argument(
        "--extra-model",
        action="append",
        type=Path,
        default=[],
        help="Additional ONNX model path. Can be repeated.",
    )
    parser.add_argument(
        "--extra-classes",
        action="append",
        type=Path,
        default=[],
        help="Class-name file for the matching --extra-model. Can be repeated.",
    )
    parser.add_argument("--device", default="/dev/video0", help="V4L2 camera device.")
    parser.add_argument("--source", type=Path, default=None, help="Use an existing image instead of capturing.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="Output directory.")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO input size.")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold.")
    parser.add_argument("--iou", type=float, default=0.45, help="NMS IoU threshold.")
    parser.add_argument("--digit-model", type=Path, default=DEFAULT_DIGIT_MODEL, help="Digit YOLO ONNX model path.")
    parser.add_argument("--digit-classes", type=Path, default=DEFAULT_DIGIT_CLASSES, help="Digit class-name file path.")
    parser.add_argument("--digit-conf", type=float, default=None, help="Digit confidence threshold. Defaults to --conf.")
    parser.add_argument("--digit-imgsz", type=int, default=None, help="Digit model input size. Defaults to --imgsz.")
    parser.add_argument("--no-digits", action="store_true", help="Disable digit model detection.")
    parser.add_argument("--no-meter-readings", action="store_true", help="Disable analog meter pointer reading.")
    parser.add_argument("--meter-margin", type=float, default=0.12, help="Crop margin ratio around detected meter boxes.")
    parser.add_argument("--save-meter-debug", action="store_true", help="Save rectified analog meter debug images.")
    parser.add_argument("--width", type=int, default=3264, help="Capture width.")
    parser.add_argument("--height", type=int, default=2448, help="Capture height.")
    parser.add_argument("--fps", type=int, default=15, help="Capture framerate.")
    parser.add_argument(
        "--rotate",
        type=int,
        choices=(0, 90, 180, 270),
        default=180,
        help="Rotate the saved photo before detection. Default 180 matches the board camera mounting.",
    )
    parser.add_argument("--prefix", default=None, help="Output file prefix. Defaults to timestamp.")
    parser.add_argument(
        "--camera-profile",
        choices=tuple(CAMERA_CONTROL_PROFILES.keys()),
        default="normal",
        help="V4L2 hardware-control profile applied before capture. Use off to leave camera controls untouched.",
    )
    parser.add_argument(
        "--camera-control",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="Override one V4L2 control after the selected profile, for example gamma=130.",
    )
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=5,
        help="Discard this many camera frames before saving the photo so auto exposure/WB can settle.",
    )
    parser.add_argument(
        "--backend",
        choices=("auto", "rknn", "onnxruntime", "opencv"),
        default="auto",
        help="Inference backend. auto prefers the persistent RKNN worker and falls back to local ONNX.",
    )
    parser.add_argument(
        "--worker-socket",
        type=Path,
        default=Path(os.environ.get("CAR_CLOUD_RECOGNITION_SOCKET", DEFAULT_WORKER_SOCKET)),
        help="Unix socket used by the persistent recognition worker.",
    )
    parser.add_argument("--capture-only", action="store_true", help="Only capture the frame, skip detection.")
    return parser.parse_args()


def parse_camera_control_overrides(values: Sequence[str]) -> Dict[str, int]:
    overrides: Dict[str, int] = {}
    for raw in values:
        if "=" not in raw:
            raise ValueError(f"invalid --camera-control {raw!r}; expected NAME=VALUE")
        name, value = raw.split("=", 1)
        name = name.strip()
        if name not in V4L2_CONTROL_IDS:
            allowed = ", ".join(sorted(V4L2_CONTROL_IDS))
            raise ValueError(f"unknown camera control {name!r}; allowed: {allowed}")
        try:
            overrides[name] = int(value.strip())
        except ValueError as exc:
            raise ValueError(f"invalid value for camera control {name!r}: {value!r}") from exc
    return overrides


def build_camera_controls(profile: str, overrides: Mapping[str, int]) -> Dict[str, int]:
    if profile not in CAMERA_CONTROL_PROFILES:
        raise ValueError(f"unknown camera profile: {profile}")
    controls = dict(CAMERA_CONTROL_PROFILES[profile])
    controls.update(overrides)
    return controls


def apply_camera_controls(device: str, profile: str, overrides: Mapping[str, int]) -> dict:
    controls = build_camera_controls(profile, overrides)
    report = {"profile": profile, "requested": controls, "applied": {}, "errors": []}
    if not controls:
        return report

    try:
        import fcntl
        import os
        import struct
    except Exception as exc:
        report["errors"].append(f"v4l2 ioctl unavailable: {exc}")
        return report

    vidioc_s_ctrl = 0xC008561C
    try:
        fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
    except OSError as exc:
        report["errors"].append(f"open {device} failed: {exc}")
        return report

    try:
        for name, value in controls.items():
            ctrl_id = V4L2_CONTROL_IDS[name]
            try:
                fcntl.ioctl(fd, vidioc_s_ctrl, struct.pack("Ii", ctrl_id, int(value)))
                report["applied"][name] = int(value)
            except OSError as exc:
                report["errors"].append(f"{name}={value}: {exc}")
    finally:
        os.close(fd)
    return report


def load_classes(path: Path) -> List[str]:
    if not path.is_file():
        raise FileNotFoundError(f"class file not found: {path}")
    names = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not names:
        raise ValueError(f"class file is empty: {path}")
    return names


def label_for_model(path: Path) -> str:
    stem = path.stem.strip()
    return stem or "model"


def imgsz_for_model(model_path: Path, fallback: int) -> int:
    if model_path.stem.startswith("meter_digit_unified"):
        return UNIFIED_MODEL_IMGSZ
    return int(fallback)


def auto_classes_for_model(model_path: Path, fallback: Path) -> Path:
    candidates = [
        model_path.with_suffix(".classes.txt"),
        model_path.parent / f"{model_path.stem}_classes.txt",
        model_path.parent / f"{model_path.stem}.txt",
        fallback,
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return fallback


def collect_model_specs(args: argparse.Namespace) -> Tuple[List[dict], List[str]]:
    warnings: List[str] = []
    specs: List[dict] = []
    seen: set[str] = set()

    def add_model(model_path: Path, classes_path: Path, *, required: bool) -> None:
        resolved_key = str(model_path.resolve()) if model_path.exists() else str(model_path)
        if resolved_key in seen:
            return
        if not model_path.is_file():
            message = f"model not found: {model_path}"
            if required:
                raise FileNotFoundError(message)
            warnings.append(message)
            return
        class_path = auto_classes_for_model(model_path, classes_path)
        if not class_path.is_file():
            message = f"class file not found for {model_path}: {class_path}"
            if required:
                raise FileNotFoundError(message)
            warnings.append(message)
            return
        seen.add(resolved_key)
        specs.append(
            {
                "model": model_path,
                "classes": class_path,
                "label": label_for_model(model_path),
                "imgsz": imgsz_for_model(model_path, args.imgsz),
            }
        )

    explicit_extra_models = list(args.extra_model or [])
    has_extra = bool(explicit_extra_models or DEFAULT_EXTRA_MODEL.is_file())
    if args.model.is_file() or not has_extra:
        add_model(args.model, args.classes, required=not has_extra)

    for index, model_path in enumerate(explicit_extra_models):
        if index < len(args.extra_classes or []):
            classes_path = args.extra_classes[index]
        else:
            classes_path = auto_classes_for_model(model_path, DEFAULT_EXTRA_CLASSES)
        add_model(model_path, classes_path, required=True)

    if DEFAULT_EXTRA_MODEL.is_file():
        add_model(DEFAULT_EXTRA_MODEL, DEFAULT_EXTRA_CLASSES, required=False)

    if not specs:
        raise FileNotFoundError(
            f"no usable YOLO ONNX model found; checked {args.model} and {DEFAULT_EXTRA_MODEL}"
        )
    return specs, warnings


def capture_with_gstreamer(device: str, output: Path, width: int, height: int, fps: int) -> None:
    if shutil.which("gst-launch-1.0") is None:
        raise RuntimeError("gst-launch-1.0 not found")
    pipeline = [
        "gst-launch-1.0",
        "-q",
        "v4l2src",
        f"device={device}",
        "num-buffers=1",
        "!",
        f"image/jpeg,width={width},height={height},framerate={fps}/1",
        "!",
        "filesink",
        f"location={output}",
    ]
    subprocess.run(pipeline, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def warmup_with_gstreamer(device: str, frames: int, width: int, height: int, fps: int) -> None:
    if shutil.which("gst-launch-1.0") is None:
        raise RuntimeError("gst-launch-1.0 not found")
    pipeline = [
        "gst-launch-1.0",
        "-q",
        "v4l2src",
        f"device={device}",
        f"num-buffers={frames}",
        "!",
        f"image/jpeg,width={width},height={height},framerate={fps}/1",
        "!",
        "fakesink",
    ]
    timeout_s = max(5.0, frames / max(1, fps) + 5.0)
    subprocess.run(
        pipeline,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=timeout_s,
    )


def capture_with_opencv(device: str, output: Path, width: int, height: int, fps: int) -> None:
    index = int(device) if device.isdigit() else device
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera: {device}")
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        frame = None
        ok = False
        for _ in range(3):
            ok, frame = cap.read()
        if not ok or frame is None:
            raise RuntimeError(f"failed to read one frame from camera: {device}")
        if not cv2.imwrite(str(output), frame):
            raise RuntimeError(f"failed to write captured frame: {output}")
    finally:
        cap.release()


def rotate_frame(image: np.ndarray, degrees: int) -> np.ndarray:
    if degrees == 0:
        return image
    if degrees == 90:
        return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    if degrees == 180:
        return cv2.rotate(image, cv2.ROTATE_180)
    if degrees == 270:
        return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    raise ValueError(f"unsupported rotation: {degrees}")


def capture_warmed_frame(
    device: str,
    output: Path,
    width: int,
    height: int,
    fps: int,
    warmup_frames: int,
    rotate: int,
) -> None:
    index = int(device) if device.isdigit() else device
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera: {device}")
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        frame = None
        ok = False
        for _ in range(max(1, int(warmup_frames) + 1)):
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError(f"failed to read camera frame: {device}")
        output.parent.mkdir(parents=True, exist_ok=True)
        frame = rotate_frame(frame, rotate)
        if not cv2.imwrite(str(output), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
            raise RuntimeError(f"failed to save camera frame: {output}")
    finally:
        cap.release()


def warmup_with_opencv(device: str, frames: int, width: int, height: int, fps: int) -> None:
    index = int(device) if device.isdigit() else device
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera: {device}")
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        ok = False
        for _ in range(max(1, frames)):
            ok, _ = cap.read()
        if not ok:
            raise RuntimeError(f"failed to warm up camera: {device}")
    finally:
        cap.release()


def warmup_camera(device: str, frames: int, width: int, height: int, fps: int) -> List[str]:
    if frames <= 0:
        return []
    errors: List[str] = []
    for warmup_func in (warmup_with_gstreamer, warmup_with_opencv):
        try:
            warmup_func(device, frames, width, height, fps)
            return errors
        except Exception as exc:
            errors.append(f"{warmup_func.__name__}: {exc}")
    return errors


def capture_frame(device: str, output: Path, width: int, height: int, fps: int) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    errors: List[str] = []
    for capture_func in (capture_with_gstreamer, capture_with_opencv):
        try:
            capture_func(device, output, width, height, fps)
            if output.is_file() and output.stat().st_size > 0:
                return
        except Exception as exc:
            errors.append(f"{capture_func.__name__}: {exc}")
    raise RuntimeError("camera capture failed: " + " | ".join(errors))


def rotate_saved_image(path: Path, degrees: int) -> None:
    if degrees == 0:
        return
    image = cv2.imread(str(path))
    if image is None:
        raise RuntimeError(f"failed to read image for rotation: {path}")
    if degrees == 90:
        rotated = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    elif degrees == 180:
        rotated = cv2.rotate(image, cv2.ROTATE_180)
    elif degrees == 270:
        rotated = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    else:
        raise ValueError(f"unsupported rotation: {degrees}")
    if not cv2.imwrite(str(path), rotated):
        raise RuntimeError(f"failed to write rotated image: {path}")


def letterbox(image: np.ndarray, size: int) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    new_width, new_height = int(round(width * scale)), int(round(height * scale))
    resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    pad_x = (size - new_width) / 2.0
    pad_y = (size - new_height) / 2.0
    left, top = int(round(pad_x - 0.1)), int(round(pad_y - 0.1))
    canvas[top : top + new_height, left : left + new_width] = resized
    return canvas, scale, (left, top)


def normalize_output(output: np.ndarray) -> np.ndarray:
    output = np.squeeze(output)
    if output.ndim != 2:
        raise ValueError(f"unexpected model output shape after squeeze: {output.shape}")
    if output.shape[0] < output.shape[1]:
        output = output.T
    return output


def decode_detections(
    output: np.ndarray,
    image_shape: Tuple[int, int],
    classes: Sequence[str],
    scale: float,
    pad: Tuple[float, float],
    conf_threshold: float,
    iou_threshold: float,
) -> List[dict]:
    detections = normalize_output(output)
    image_h, image_w = image_shape
    pad_x, pad_y = pad
    class_scores = detections[:, 4 : 4 + len(classes)]
    if class_scores.shape[1] == 0:
        return []
    all_class_ids = np.argmax(class_scores, axis=1)
    all_scores = class_scores[np.arange(class_scores.shape[0]), all_class_ids]
    keep = all_scores >= conf_threshold
    if not np.any(keep):
        return []
    selected_rows = detections[keep, :4].astype(np.float64, copy=False)
    selected_scores = all_scores[keep]
    selected_class_ids = all_class_ids[keep]

    centers_x, centers_y, widths, heights = selected_rows.T
    x1 = np.rint((centers_x - widths / 2.0 - pad_x) / scale)
    y1 = np.rint((centers_y - heights / 2.0 - pad_y) / scale)
    x2 = np.rint((centers_x + widths / 2.0 - pad_x) / scale)
    y2 = np.rint((centers_y + heights / 2.0 - pad_y) / scale)
    x1 = np.clip(x1, 0, image_w - 1).astype(np.int32)
    y1 = np.clip(y1, 0, image_h - 1).astype(np.int32)
    x2 = np.clip(x2, 0, image_w - 1).astype(np.int32)
    y2 = np.clip(y2, 0, image_h - 1).astype(np.int32)
    valid = (x2 > x1) & (y2 > y1)
    boxes_array = np.column_stack((x1[valid], y1[valid], x2[valid] - x1[valid], y2[valid] - y1[valid]))
    scores = selected_scores[valid].astype(float).tolist()
    class_ids = selected_class_ids[valid].astype(int).tolist()
    boxes: List[List[int]] = boxes_array.astype(int).tolist()

    selected = cv2.dnn.NMSBoxes(boxes, scores, conf_threshold, iou_threshold)
    selected_indices = np.array(selected).reshape(-1).tolist() if len(selected) else []

    results: List[dict] = []
    for idx in selected_indices:
        x, y, w, h = boxes[idx]
        class_id = class_ids[idx]
        results.append(
            {
                "class_id": class_id,
                "class_name": classes[class_id] if class_id < len(classes) else str(class_id),
                "confidence": round(float(scores[idx]), 6),
                "box_xyxy": [int(x), int(y), int(x + w), int(y + h)],
            }
        )
    results.sort(key=lambda item: item["confidence"], reverse=True)
    return results


def run_detection_with_worker(
    worker_socket: Path,
    image_path: Path,
    model_path: Path,
    conf_threshold: float,
    iou_threshold: float,
) -> Tuple[List[dict], float, str, float]:
    request = {
        "action": "infer",
        "image_path": str(image_path.resolve()),
        "model_path": str(model_path.resolve()),
        "conf": float(conf_threshold),
        "iou": float(iou_threshold),
    }
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(15.0)
    try:
        client.connect(str(worker_socket))
        client.sendall((json.dumps(request, ensure_ascii=False) + "\n").encode("utf-8"))
        chunks: List[bytes] = []
        total = 0
        while True:
            chunk = client.recv(65536)
            if not chunk:
                break
            chunks.append(chunk)
            total += len(chunk)
            if total > 8 * 1024 * 1024:
                raise RuntimeError("recognition worker response is too large")
            if b"\n" in chunk:
                break
        raw = b"".join(chunks).split(b"\n", 1)[0]
        payload = json.loads(raw.decode("utf-8"))
        if not isinstance(payload, dict) or not payload.get("ok"):
            raise RuntimeError(str(payload.get("error") if isinstance(payload, dict) else payload))
        detections = payload.get("detections", [])
        if not isinstance(detections, list):
            raise RuntimeError("recognition worker returned invalid detections")
        return (
            detections,
            float(payload.get("inference_ms", 0.0) or 0.0),
            str(payload.get("backend") or "worker"),
            float(payload.get("queue_ms", 0.0) or 0.0),
        )
    finally:
        client.close()


def run_detection(
    model_path: Path,
    image: np.ndarray,
    classes: Sequence[str],
    imgsz: int,
    conf_threshold: float,
    iou_threshold: float,
    backend: str,
    worker_socket: Path | None = None,
    worker_image_path: Path | None = None,
    timing_sink: Dict[str, float] | None = None,
) -> Tuple[List[dict], float, str]:
    if not model_path.is_file():
        raise FileNotFoundError(f"model not found: {model_path}")

    worker_error: Optional[Exception] = None
    if backend in ("auto", "rknn") and worker_socket is not None and worker_image_path is not None:
        try:
            detections, infer_ms, worker_backend, queue_ms = run_detection_with_worker(
                worker_socket,
                worker_image_path,
                model_path,
                conf_threshold,
                iou_threshold,
            )
            if timing_sink is not None:
                timing_sink["worker_queue_ms"] = timing_sink.get("worker_queue_ms", 0.0) + queue_ms
            return detections, infer_ms, worker_backend
        except Exception as exc:
            worker_error = exc

    blob_image, scale, pad = letterbox(image, imgsz)
    blob = cv2.dnn.blobFromImage(blob_image, scalefactor=1.0 / 255.0, size=(imgsz, imgsz), swapRB=True)
    ort_error: Optional[Exception] = None

    if backend in ("auto", "rknn", "onnxruntime"):
        try:
            import onnxruntime as ort

            session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
            input_name = session.get_inputs()[0].name
            started = time.perf_counter()
            output = session.run(None, {input_name: blob.astype(np.float32)})[0]
            infer_ms = (time.perf_counter() - started) * 1000.0
            return (
                decode_detections(
                    output,
                    image.shape[:2],
                    classes,
                    scale,
                    pad,
                    conf_threshold,
                    iou_threshold,
                ),
                infer_ms,
                "onnxruntime",
            )
        except Exception as exc:
            if backend == "onnxruntime":
                raise
            ort_error = exc

    try:
        net = cv2.dnn.readNetFromONNX(str(model_path))
        net.setInput(blob)
        started = time.perf_counter()
        output = net.forward()
        infer_ms = (time.perf_counter() - started) * 1000.0
        return (
            decode_detections(
                output,
                image.shape[:2],
                classes,
                scale,
                pad,
                conf_threshold,
                iou_threshold,
            ),
            infer_ms,
            "opencv",
        )
    except Exception as exc:
        if ort_error is not None:
            raise RuntimeError(
                f"worker failed: {worker_error}; onnxruntime failed: {ort_error}; opencv failed: {exc}"
            ) from exc
        raise


def detection_label(det: Mapping[str, object]) -> str:
    return str(det.get("class_name") or "").strip().lower()


def detection_center(det: Mapping[str, object]) -> Tuple[float, float]:
    box = det.get("box_xyxy")
    if not isinstance(box, Sequence) or len(box) != 4:
        return (0.0, 0.0)
    x1, y1, x2, y2 = [float(value) for value in box]
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


def detection_height(det: Mapping[str, object]) -> float:
    box = det.get("box_xyxy")
    if not isinstance(box, Sequence) or len(box) != 4:
        return 0.0
    return abs(float(box[3]) - float(box[1]))


def expand_box_xyxy(
    box: Sequence[object],
    image_shape: Tuple[int, int],
    margin_ratio: float,
) -> List[int]:
    image_h, image_w = image_shape
    x1, y1, x2, y2 = [float(value) for value in box]
    width = max(1.0, x2 - x1)
    height = max(1.0, y2 - y1)
    margin = max(width, height) * max(0.0, margin_ratio)
    return [
        int(max(0, round(x1 - margin))),
        int(max(0, round(y1 - margin))),
        int(min(image_w - 1, round(x2 + margin))),
        int(min(image_h - 1, round(y2 + margin))),
    ]


def point_in_box(point: Tuple[float, float], box: Sequence[object]) -> bool:
    x, y = point
    x1, y1, x2, y2 = [float(value) for value in box]
    return x1 <= x <= x2 and y1 <= y <= y2


def best_detection_by_labels(detections: Sequence[dict], labels: Iterable[str]) -> Optional[dict]:
    wanted = {str(label).strip().lower() for label in labels}
    candidates = [det for det in detections if detection_label(det) in wanted]
    if not candidates:
        return None
    return max(candidates, key=lambda det: float(det.get("confidence") or 0.0))


def resolve_meter_detections(detections: Sequence[dict]) -> Dict[str, Tuple[Optional[dict], str]]:
    resolved: Dict[str, Tuple[Optional[dict], str]] = {
        meter_type: (best_detection_by_labels(detections, spec["labels"]), "model_label")
        for meter_type, spec in METER_TARGETS.items()
    }
    meter_labels = {
        str(label).strip().lower()
        for spec in METER_TARGETS.values()
        for label in spec["labels"]
    }
    candidates = [det for det in detections if detection_label(det) in meter_labels]
    if len(candidates) < 2:
        return resolved

    candidates.sort(
        key=lambda det: (
            float(det["box_xyxy"][0]) + float(det["box_xyxy"][2])
        ) * 0.5
    )
    left = candidates[0]
    right = candidates[-1]
    left_box = [float(value) for value in left["box_xyxy"]]
    right_box = [float(value) for value in right["box_xyxy"]]
    left_center = (left_box[0] + left_box[2]) * 0.5
    right_center = (right_box[0] + right_box[2]) * 0.5
    min_width = max(1.0, min(left_box[2] - left_box[0], right_box[2] - right_box[0]))
    if right_center - left_center < min_width * 0.5:
        return resolved

    labels = {detection_label(candidate) for candidate in candidates}
    voltage_detection, _ = resolved["voltage"]
    current_detection, _ = resolved["current"]
    if len(labels) == 1 or voltage_detection is None or current_detection is None:
        for detection, class_id, class_name, method in (
            (left, 0, "yakuang", "fixed_layout_left"),
            (right, 1, "liukuang", "fixed_layout_right"),
        ):
            original_class_name = detection_label(detection)
            if original_class_name != class_name:
                detection["model_class_id"] = detection.get("class_id")
                detection["model_class_name"] = original_class_name
                detection["class_id"] = class_id
                detection["class_name"] = class_name
                display_name = str(detection.get("display_name") or "")
                if ":" in display_name:
                    detection["display_name"] = f'{display_name.split(":", 1)[0]}:{class_name}'
            detection["meter_layout_assignment"] = method
        resolved["voltage"] = (left, "fixed_layout_left")
        resolved["current"] = (right, "fixed_layout_right")
    return resolved


def load_meter_tools():
    if not DEFAULT_METER_DIR.is_dir():
        raise FileNotFoundError(f"meter reader directory not found: {DEFAULT_METER_DIR}")
    meter_dir = str(DEFAULT_METER_DIR)
    if meter_dir not in sys.path:
        sys.path.insert(0, meter_dir)
    from meter_pipeline import load_meter_config, read_meter_from_box, save_image

    return load_meter_config, read_meter_from_box, save_image


def meter_reading_to_cabinet_metric(reading: Mapping[str, object]) -> Optional[dict]:
    value = reading.get("value")
    if value is None:
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    meter_type = str(reading.get("meter_type") or "")
    metric = {
        "value": numeric,
        "unit": str(reading.get("unit") or ""),
        "status": str(reading.get("status") or "ok"),
    }
    if meter_type == "voltage":
        metric.update({"min": 0, "max": 450, "raw_value": numeric})
    elif meter_type == "current":
        metric.update({"min": 0, "max": 1.0})
    return metric


def run_meter_reading_job(job: dict) -> Tuple[dict, List[str]]:
    started = time.perf_counter()
    warnings: List[str] = []
    cv2.setNumThreads(2)
    meter_dir = str(DEFAULT_METER_DIR)
    if meter_dir not in sys.path:
        sys.path.insert(0, meter_dir)
    from meter_pipeline import load_meter_config, read_meter_from_crop, save_image

    config = load_meter_config(Path(job["config_path"]))
    result = read_meter_from_crop(
        job["crop"],
        config,
        det_conf=float(job["confidence"]),
        bbox=tuple(int(value) for value in job["expanded_box"]),
    )
    fallback_attempt: dict = {}
    retry_crop = job.get("retry_crop")
    retry_box = job.get("retry_expanded_box")
    initial_quality = dict(getattr(result, "rectify_quality", None) or {})
    if (
        job["meter_type"] == "voltage"
        and (
            getattr(result, "value", None) is None
            or bool(initial_quality.get("geometry_fallback"))
        )
        and isinstance(retry_crop, np.ndarray)
        and retry_crop.size
        and isinstance(retry_box, list)
    ):
        fallback_attempt = {
            "initial_status": str(getattr(result, "status", "") or ""),
            "initial_value": getattr(result, "value", None),
            "initial_rectify_method": str(getattr(result, "rectify_method", "") or ""),
            "retry_margin": float(job.get("retry_margin") or 0.0),
        }
        retry_result = read_meter_from_crop(
            retry_crop,
            config,
            det_conf=float(job["confidence"]),
            bbox=tuple(int(value) for value in retry_box),
            allow_relaxed_quad=True,
        )
        fallback_attempt.update(
            {
                "retry_status": str(getattr(retry_result, "status", "") or ""),
                "retry_value": getattr(retry_result, "value", None),
                "retry_rectify_method": str(getattr(retry_result, "rectify_method", "") or ""),
                "accepted": getattr(retry_result, "value", None) is not None,
            }
        )
        if getattr(retry_result, "value", None) is not None:
            result = retry_result
    debug_path = ""
    if bool(job.get("save_debug")) and getattr(result, "debug_image", None) is not None:
        debug_file = Path(job["debug_file"])
        try:
            save_image(debug_file, result.debug_image)
            debug_path = str(debug_file)
        except Exception as exc:
            warnings.append(f"meter debug save failed for {job['meter_type']}: {exc}")

    raw_value = getattr(result, "value", None)
    payload = {
        "meter_type": job["meter_type"],
        "display_name": job["display_name"],
        "unit": job["unit"],
        "value": None if raw_value is None else round(float(raw_value), 6),
        "status": str(getattr(result, "status", "")),
        "confidence": job["confidence"],
        "box_xyxy": list(getattr(result, "bbox", None) or job["expanded_box"]),
        "pointer_angle": None
        if getattr(result, "pointer_angle", None) is None
        else round(float(result.pointer_angle), 6),
        "pointer_method": str(getattr(result, "pointer_method", "") or ""),
        "pointer_candidates": list(getattr(result, "pointer_candidates", None) or []),
        "pointer_consensus_count": int(getattr(result, "pointer_consensus_count", 0) or 0),
        "rectify_method": str(getattr(result, "rectify_method", "") or ""),
        "rectify_quality": dict(getattr(result, "rectify_quality", None) or {}),
        "quality_score": round(float(getattr(result, "quality_score", 0.0) or 0.0), 6),
        "debug_image": debug_path,
        "processing_ms": round((time.perf_counter() - started) * 1000.0, 3),
        "detected_class_name": job.get("detected_class_name", ""),
        "assignment_method": job.get("assignment_method", "model_label"),
    }
    if fallback_attempt:
        payload["fallback_attempt"] = fallback_attempt
    metric = meter_reading_to_cabinet_metric(payload)
    if metric is not None:
        payload["cabinet_metric"] = metric
    return payload, warnings


def run_meter_readings(
    image: np.ndarray,
    detections: Sequence[dict],
    args: argparse.Namespace,
    output_dir: Path,
    prefix: str,
) -> Tuple[List[dict], List[str]]:
    warnings: List[str] = []
    readings: List[dict] = []
    if args.no_meter_readings:
        return readings, warnings

    jobs: List[Tuple[int, dict]] = []
    ordered_payloads: List[dict | None] = []
    resolved_detections = resolve_meter_detections(detections)
    for meter_type, spec in METER_TARGETS.items():
        unit = str(spec["unit"])
        detection, assignment_method = resolved_detections[meter_type]
        config_path = Path(spec["config"])
        base_payload = {
            "meter_type": meter_type,
            "display_name": spec["display_name"],
            "unit": unit,
            "value": None,
            "status": "",
            "confidence": None,
            "box_xyxy": [],
            "pointer_angle": None,
            "pointer_method": "",
            "debug_image": "",
            "detected_class_name": "",
            "assignment_method": assignment_method,
        }

        if detection is None:
            payload = dict(base_payload)
            payload["status"] = "no_detection"
            ordered_payloads.append(payload)
            continue
        if not config_path.is_file():
            payload = dict(base_payload)
            payload["status"] = "config_missing"
            payload["confidence"] = detection.get("confidence")
            payload["box_xyxy"] = list(detection.get("box_xyxy") or [])
            ordered_payloads.append(payload)
            warnings.append(f"meter config missing for {meter_type}: {config_path}")
            continue
        expanded_box = expand_box_xyxy(
            detection["box_xyxy"],
            image.shape[:2],
            margin_ratio=float(args.meter_margin),
        )
        x1, y1, x2, y2 = expanded_box
        job = {
            "meter_type": meter_type,
            "display_name": spec["display_name"],
            "unit": unit,
            "config_path": str(config_path),
            "crop": image[y1:y2, x1:x2].copy(),
            "confidence": float(detection.get("confidence") or 0.0),
            "expanded_box": expanded_box,
            "save_debug": bool(args.save_meter_debug),
            "debug_file": str(output_dir / f"{prefix}_{meter_type}_meter_debug.jpg"),
            "detected_class_name": str(detection.get("model_class_name") or detection_label(detection)),
            "assignment_method": assignment_method,
        }
        if meter_type == "voltage":
            retry_margin = max(0.20, float(args.meter_margin))
            retry_box = expand_box_xyxy(
                detection["box_xyxy"],
                image.shape[:2],
                margin_ratio=retry_margin,
            )
            retry_x1, retry_y1, retry_x2, retry_y2 = retry_box
            job.update(
                {
                    "retry_crop": image[retry_y1:retry_y2, retry_x1:retry_x2].copy(),
                    "retry_expanded_box": retry_box,
                    "retry_margin": retry_margin,
                }
            )
        index = len(ordered_payloads)
        ordered_payloads.append(None)
        jobs.append((index, job))

    if jobs:
        max_workers = min(2, len(jobs))
        with ProcessPoolExecutor(
            max_workers=max_workers,
            mp_context=multiprocessing.get_context("spawn"),
        ) as executor:
            future_map = {
                executor.submit(run_meter_reading_job, job): (index, job)
                for index, job in jobs
            }
            for future in as_completed(future_map):
                index, job = future_map[future]
                try:
                    payload, job_warnings = future.result()
                    ordered_payloads[index] = payload
                    warnings.extend(job_warnings)
                except Exception as exc:
                    ordered_payloads[index] = {
                        "meter_type": job["meter_type"],
                        "display_name": job["display_name"],
                        "unit": job["unit"],
                        "value": None,
                        "status": f"read_failed: {exc}",
                        "confidence": job["confidence"],
                        "box_xyxy": list(job["expanded_box"]),
                        "pointer_angle": None,
                        "pointer_method": "",
                        "debug_image": "",
                    }

    readings.extend(payload for payload in ordered_payloads if isinstance(payload, dict))
    return readings, warnings


def run_digit_detection(
    image: np.ndarray,
    args: argparse.Namespace,
    worker_image_path: Path | None = None,
    timing_sink: Dict[str, float] | None = None,
) -> Tuple[List[dict], float, str, List[str], List[str]]:
    warnings: List[str] = []
    if args.no_digits:
        return [], 0.0, "", warnings, []
    if not args.digit_model.is_file():
        warnings.append(f"digit model not found: {args.digit_model}")
        return [], 0.0, "", warnings, []
    if not args.digit_classes.is_file():
        warnings.append(f"digit classes not found: {args.digit_classes}")
        return [], 0.0, "", warnings, []

    classes = load_classes(args.digit_classes)
    conf_threshold = args.conf if args.digit_conf is None else float(args.digit_conf)
    imgsz = imgsz_for_model(args.digit_model, args.imgsz) if args.digit_imgsz is None else int(args.digit_imgsz)
    detections, infer_ms, backend = run_detection(
        model_path=args.digit_model,
        image=image,
        classes=classes,
        imgsz=imgsz,
        conf_threshold=conf_threshold,
        iou_threshold=args.iou,
        backend=args.backend,
        worker_socket=args.worker_socket,
        worker_image_path=worker_image_path,
        timing_sink=timing_sink,
    )
    detections = [detection for detection in detections if is_digit_detection(detection)]
    for detection in detections:
        detection["model"] = label_for_model(args.digit_model)
        detection["source"] = "digit"
        detection["display_name"] = f'digit:{detection["class_name"]}'
    return detections, infer_ms, f"{label_for_model(args.digit_model)}:{backend}", warnings, digit_class_names(classes)


def is_digit_detection(detection: Mapping[str, object]) -> bool:
    return str(detection.get("class_name") or "").strip() in DIGIT_LABELS


def digit_class_names(classes: Sequence[str]) -> List[str]:
    return [str(name) for name in classes if str(name).strip() in DIGIT_LABELS]


def extract_digit_detections(detections: Sequence[dict]) -> List[dict]:
    digit_detections: List[dict] = []
    for detection in detections:
        if not is_digit_detection(detection):
            continue
        detection["source"] = "unified_digit"
        detection["display_name"] = f'digit:{detection["class_name"]}'
        digit_detections.append(detection)
    return digit_detections


def digit_text(detections: Sequence[dict]) -> str:
    items = sorted(detections, key=lambda det: detection_center(det)[0])
    return "".join(str(det.get("class_name") or "") for det in items)


def digit_rows(detections: Sequence[dict]) -> List[dict]:
    if not detections:
        return []
    ordered = sorted(detections, key=lambda det: detection_center(det)[1])
    median_h = float(np.median([max(1.0, detection_height(det)) for det in ordered]))
    tolerance = max(18.0, median_h * 0.65)
    rows: List[List[dict]] = []
    centers: List[float] = []
    for det in ordered:
        _, cy = detection_center(det)
        target_index = None
        for index, center_y in enumerate(centers):
            if abs(cy - center_y) <= tolerance:
                target_index = index
                break
        if target_index is None:
            rows.append([det])
            centers.append(cy)
        else:
            rows[target_index].append(det)
            centers[target_index] = (centers[target_index] * (len(rows[target_index]) - 1) + cy) / len(rows[target_index])

    payload: List[dict] = []
    for row_dets, center_y in sorted(zip(rows, centers), key=lambda item: item[1]):
        row_sorted = sorted(row_dets, key=lambda det: detection_center(det)[0])
        payload.append(
            {
                "text": digit_text(row_sorted),
                "count": len(row_sorted),
                "center_y": round(float(center_y), 2),
                "detections": row_sorted,
            }
        )
    return payload


def build_digit_readings(
    digit_detections: Sequence[dict],
    meter_readings: Sequence[dict],
    image_shape: Tuple[int, int],
) -> dict:
    rows = digit_rows(digit_detections)
    readings = {
        "global_text": "".join(str(row.get("text") or "") for row in rows),
        "rows": rows,
        "meters": [],
    }
    for meter in meter_readings:
        box = meter.get("box_xyxy")
        if not isinstance(box, Sequence) or len(box) != 4:
            continue
        expanded = expand_box_xyxy(box, image_shape, margin_ratio=0.08)
        selected = [det for det in digit_detections if point_in_box(detection_center(det), expanded)]
        if not selected:
            continue
        selected = sorted(selected, key=lambda det: detection_center(det)[0])
        readings["meters"].append(
            {
                "meter_type": meter.get("meter_type"),
                "unit": meter.get("unit"),
                "text": digit_text(selected),
                "count": len(selected),
                "box_xyxy": expanded,
                "detections": selected,
            }
        )
    return readings


def draw_measurement_summary(
    image: np.ndarray,
    meter_readings: Sequence[dict],
    digit_readings: Mapping[str, object],
) -> np.ndarray:
    annotated = image.copy()
    lines: List[str] = []
    for reading in meter_readings:
        value = reading.get("value")
        status = reading.get("status") or ""
        meter_type = reading.get("meter_type") or "meter"
        unit = reading.get("unit") or ""
        if value is None:
            lines.append(f"{meter_type}: {status}")
        else:
            lines.append(f"{meter_type}: {float(value):.2f}{unit} ({status})")
    global_text = str(digit_readings.get("global_text") or "") if isinstance(digit_readings, Mapping) else ""
    if global_text:
        lines.append(f"digits: {global_text}")
    if not lines:
        return annotated

    x = 12
    y = 62
    for line in lines[:5]:
        cv2.putText(annotated, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 4)
        cv2.putText(annotated, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (15, 23, 42), 2)
        y += 30
    return annotated


def draw_results(image: np.ndarray, detections: Iterable[dict], infer_ms: float, backend: str) -> np.ndarray:
    annotated = image.copy()
    palette = [
        (52, 211, 153),
        (96, 165, 250),
        (248, 113, 113),
        (251, 191, 36),
        (216, 180, 254),
    ]
    count = 0
    for count, det in enumerate(detections, start=1):
        x1, y1, x2, y2 = det["box_xyxy"]
        color = palette[det["class_id"] % len(palette)]
        label_name = det.get("display_name") or det["class_name"]
        label = f'{label_name} {det["confidence"]:.2f}'
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        text_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        top = max(0, y1 - text_size[1] - baseline - 6)
        cv2.rectangle(
            annotated,
            (x1, top),
            (x1 + text_size[0] + 8, top + text_size[1] + baseline + 6),
            color,
            -1,
        )
        cv2.putText(
            annotated,
            label,
            (x1 + 4, top + text_size[1] + 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )

    summary = f"detections={count} backend={backend} inference={infer_ms:.1f}ms"
    cv2.putText(annotated, summary, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 3)
    cv2.putText(annotated, summary, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 1)
    return annotated


def main() -> int:
    args = parse_args()
    total_started = time.perf_counter()
    if args.imgsz <= 0:
        print("ERROR: --imgsz must be positive", file=sys.stderr)
        return 2
    if not 0.0 <= args.conf <= 1.0:
        print("ERROR: --conf must be between 0 and 1", file=sys.stderr)
        return 2
    if args.digit_conf is not None and not 0.0 <= args.digit_conf <= 1.0:
        print("ERROR: --digit-conf must be between 0 and 1", file=sys.stderr)
        return 2
    if args.digit_imgsz is not None and args.digit_imgsz <= 0:
        print("ERROR: --digit-imgsz must be positive", file=sys.stderr)
        return 2
    if args.meter_margin < 0.0:
        print("ERROR: --meter-margin must be non-negative", file=sys.stderr)
        return 2
    if args.warmup_frames < 0:
        print("ERROR: --warmup-frames must be non-negative", file=sys.stderr)
        return 2

    try:
        camera_control_overrides = parse_camera_control_overrides(args.camera_control)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    prefix = args.prefix or time.strftime("%Y%m%d_%H%M%S")
    raw_image_path = output_dir / f"{prefix}_raw.jpg"
    annotated_path = output_dir / f"{prefix}_det.jpg"
    result_path = output_dir / f"{prefix}_det.json"

    try:
        timings_ms: Dict[str, object] = {}
        camera_report = {"profile": args.camera_profile, "requested": {}, "applied": {}, "errors": []}
        warmup_errors: List[str] = []
        if args.source is None:
            controls_started = time.perf_counter()
            camera_report = apply_camera_controls(args.device, args.camera_profile, camera_control_overrides)
            timings_ms["camera_controls"] = round((time.perf_counter() - controls_started) * 1000.0, 3)
            for error in camera_report.get("errors", []):
                print(f"WARN: camera control failed: {error}", file=sys.stderr)
            capture_started = time.perf_counter()
            try:
                capture_warmed_frame(
                    args.device,
                    raw_image_path,
                    args.width,
                    args.height,
                    args.fps,
                    args.warmup_frames,
                    args.rotate,
                )
                camera_report["capture_method"] = "opencv-single-session"
            except Exception as exc:
                warmup_errors.append(f"capture_warmed_frame: {exc}")
                warmup_errors.extend(warmup_camera(args.device, args.warmup_frames, args.width, args.height, args.fps))
                capture_frame(args.device, raw_image_path, args.width, args.height, args.fps)
                rotate_saved_image(raw_image_path, args.rotate)
                camera_report["capture_method"] = "legacy-fallback"
            timings_ms["camera_capture"] = round((time.perf_counter() - capture_started) * 1000.0, 3)
            for error in warmup_errors:
                print(f"WARN: camera warmup failed: {error}", file=sys.stderr)
        else:
            capture_started = time.perf_counter()
            if not args.source.is_file():
                raise FileNotFoundError(f"source image not found: {args.source}")
            shutil.copyfile(args.source, raw_image_path)
            rotate_saved_image(raw_image_path, args.rotate)
            camera_report["capture_method"] = "source-copy"
            timings_ms["camera_capture"] = round((time.perf_counter() - capture_started) * 1000.0, 3)

        if args.capture_only:
            timings_ms["total"] = round((time.perf_counter() - total_started) * 1000.0, 3)
            print(
                json.dumps(
                    {
                        "raw_image": str(raw_image_path),
                        "rotation": args.rotate,
                        "camera_controls": camera_report,
                        "camera_warmup_errors": warmup_errors,
                        "timings_ms": timings_ms,
                    },
                    ensure_ascii=False,
                    indent=2,
                )
            )
            return 0

        image = cv2.imread(str(raw_image_path))
        if image is None:
            raise RuntimeError(f"failed to read image: {raw_image_path}")

        model_specs, model_warnings = collect_model_specs(args)
        for warning in model_warnings:
            print(f"WARN: {warning}", file=sys.stderr)

        all_detections: List[dict] = []
        class_payload: List[dict] = []
        backend_parts: List[str] = []
        total_infer_ms = 0.0
        multi_model = len(model_specs) > 1
        detection_timing: Dict[str, float] = {}
        inference_started = time.perf_counter()

        for spec in model_specs:
            classes = load_classes(spec["classes"])
            detections, infer_ms, backend = run_detection(
                model_path=spec["model"],
                image=image,
                classes=classes,
                imgsz=int(spec["imgsz"]),
                conf_threshold=args.conf,
                iou_threshold=args.iou,
                backend=args.backend,
                worker_socket=args.worker_socket,
                worker_image_path=raw_image_path,
                timing_sink=detection_timing,
            )
            if args.no_digits:
                detections = [detection for detection in detections if not is_digit_detection(detection)]
            for detection in detections:
                detection["model"] = spec["label"]
                detection["source"] = "yolo"
                if multi_model:
                    detection["display_name"] = f'{spec["label"]}:{detection["class_name"]}'
            all_detections.extend(detections)
            class_payload.append(
                {
                    "model": spec["label"],
                    "model_path": str(spec["model"]),
                    "classes_path": str(spec["classes"]),
                    "imgsz": int(spec["imgsz"]),
                    "names": list(classes),
                }
            )
            total_infer_ms += infer_ms
            backend_parts.append(f'{spec["label"]}:{backend}')
        timings_ms["model_queue"] = round(detection_timing.get("worker_queue_ms", 0.0), 3)
        timings_ms["inference_wall"] = round((time.perf_counter() - inference_started) * 1000.0, 3)

        meter_started = time.perf_counter()
        meter_readings, meter_warnings = run_meter_readings(
            image=image,
            detections=all_detections,
            args=args,
            output_dir=output_dir,
            prefix=prefix,
        )
        timings_ms["meter_total"] = round((time.perf_counter() - meter_started) * 1000.0, 3)
        timings_ms["meters"] = {
            str(reading.get("meter_type") or "meter"): reading.get("processing_ms", 0.0)
            for reading in meter_readings
            if isinstance(reading, dict)
        }
        for warning in meter_warnings:
            print(f"WARN: {warning}", file=sys.stderr)

        digit_detections: List[dict] = []
        digit_infer_ms = 0.0
        digit_backend = ""
        digit_warnings: List[str] = []
        digit_classes: List[str] = []
        if not args.no_digits:
            digit_detections = extract_digit_detections(all_detections)
            if not digit_detections:
                digit_detections, digit_infer_ms, digit_backend, digit_warnings, digit_classes = run_digit_detection(
                    image=image,
                    args=args,
                    worker_image_path=raw_image_path,
                    timing_sink=detection_timing,
                )
        for warning in digit_warnings:
            print(f"WARN: {warning}", file=sys.stderr)
        if digit_classes:
            class_payload.append(
                {
                    "model": label_for_model(args.digit_model),
                    "model_path": str(args.digit_model),
                    "classes_path": str(args.digit_classes),
                    "imgsz": imgsz_for_model(args.digit_model, args.imgsz)
                    if args.digit_imgsz is None
                    else int(args.digit_imgsz),
                    "names": list(digit_classes),
                }
            )
        if digit_detections and digit_backend:
            all_detections.extend(digit_detections)
        if digit_backend:
            total_infer_ms += digit_infer_ms
            backend_parts.append(digit_backend)
        timings_ms["model_queue"] = round(detection_timing.get("worker_queue_ms", 0.0), 3)
        timings_ms["digit_inference"] = round(digit_infer_ms, 3)

        all_detections.sort(key=lambda item: item["confidence"], reverse=True)
        digit_detections.sort(key=lambda item: item["confidence"], reverse=True)
        digit_readings = build_digit_readings(digit_detections, meter_readings, image.shape[:2])
        backend_summary = ",".join(backend_parts)
        annotated = draw_results(image, all_detections, total_infer_ms, backend_summary)
        annotated = draw_measurement_summary(annotated, meter_readings, digit_readings)
        encode_started = time.perf_counter()
        if not cv2.imwrite(str(annotated_path), annotated):
            raise RuntimeError(f"failed to write annotated image: {annotated_path}")
        timings_ms["annotate_jpeg"] = round((time.perf_counter() - encode_started) * 1000.0, 3)

        cabinet_data: Dict[str, dict] = {}
        for reading in meter_readings:
            metric = reading.get("cabinet_metric")
            meter_type = str(reading.get("meter_type") or "")
            if isinstance(metric, dict) and meter_type:
                cabinet_data[meter_type] = metric
        model_entries = [
            {
                "label": spec["label"],
                "model": str(spec["model"]),
                "classes": str(spec["classes"]),
                "imgsz": int(spec["imgsz"]),
            }
            for spec in model_specs
        ]
        if digit_classes:
            model_entries.append(
                {
                    "label": label_for_model(args.digit_model),
                    "model": str(args.digit_model),
                    "classes": str(args.digit_classes),
                    "imgsz": imgsz_for_model(args.digit_model, args.imgsz)
                    if args.digit_imgsz is None
                    else int(args.digit_imgsz),
                }
            )

        payload = {
            "model": ", ".join(entry["model"] for entry in model_entries),
            "models": model_entries,
            "classes": class_payload,
            "source_image": str(raw_image_path),
            "annotated_image": str(annotated_path),
            "backend": backend_summary,
            "inference_ms": round(total_infer_ms, 3),
            "rotation": args.rotate,
            "camera_controls": camera_report,
            "camera_warmup_errors": warmup_errors,
            "model_warnings": model_warnings + meter_warnings + digit_warnings,
            "meter_readings": meter_readings,
            "digit_detections": digit_detections,
            "digit_readings": digit_readings,
            "cabinet_data": cabinet_data,
            "detections": all_detections,
            "timings_ms": timings_ms,
        }
        timings_ms["total"] = round((time.perf_counter() - total_started) * 1000.0, 3)
        result_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0
    except subprocess.CalledProcessError as exc:
        print(f"ERROR: capture command failed: {exc.stderr or exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
