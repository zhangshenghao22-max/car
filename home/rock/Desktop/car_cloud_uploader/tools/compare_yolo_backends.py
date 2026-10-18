#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


MODEL_SPECS = {
    "lightlabel_best": (640, "lightlabel_classes.txt"),
    "meter_digit_unified_best": (1280, "meter_digit_unified_best_classes.txt"),
    "digits_best": (640, "digits_classes.txt"),
    "switch_yolo_best": (640, "switch_yolo_classes.txt"),
}


def letterbox(image: np.ndarray, size: int) -> tuple[np.ndarray, float, tuple[int, int]]:
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    resized = cv2.resize(image, (round(width * scale), round(height * scale)))
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    left = round((size - resized.shape[1]) / 2 - 0.1)
    top = round((size - resized.shape[0]) / 2 - 0.1)
    canvas[top : top + resized.shape[0], left : left + resized.shape[1]] = resized
    return canvas, scale, (left, top)


def decode(
    output: np.ndarray,
    classes: list[str],
    image_shape: tuple[int, int],
    scale: float,
    pad: tuple[int, int],
    conf: float,
    iou: float,
) -> list[dict[str, Any]]:
    rows = np.squeeze(output)
    if rows.ndim != 2:
        raise RuntimeError(f"unexpected output shape: {output.shape}")
    if rows.shape[0] < rows.shape[1]:
        rows = rows.T
    boxes: list[list[int]] = []
    scores: list[float] = []
    class_ids: list[int] = []
    image_h, image_w = image_shape
    for row in rows:
        class_scores = row[4 : 4 + len(classes)]
        class_id = int(np.argmax(class_scores))
        score = float(class_scores[class_id])
        if score < conf:
            continue
        cx, cy, width, height = (float(value) for value in row[:4])
        x1 = max(0, min(image_w - 1, round((cx - width / 2 - pad[0]) / scale)))
        y1 = max(0, min(image_h - 1, round((cy - height / 2 - pad[1]) / scale)))
        x2 = max(0, min(image_w - 1, round((cx + width / 2 - pad[0]) / scale)))
        y2 = max(0, min(image_h - 1, round((cy + height / 2 - pad[1]) / scale)))
        if x2 <= x1 or y2 <= y1:
            continue
        boxes.append([x1, y1, x2 - x1, y2 - y1])
        scores.append(score)
        class_ids.append(class_id)
    selected = cv2.dnn.NMSBoxes(boxes, scores, conf, iou)
    results = []
    for index in np.asarray(selected).reshape(-1).tolist() if len(selected) else []:
        x, y, width, height = boxes[index]
        results.append(
            {
                "class_name": classes[class_ids[index]],
                "confidence": round(scores[index], 6),
                "box_xyxy": [x, y, x + width, y + height],
            }
        )
    return sorted(results, key=lambda item: item["confidence"], reverse=True)


def run(args: argparse.Namespace) -> int:
    runtimes: dict[str, Any] = {}
    selected_models = list(args.model or MODEL_SPECS)
    if args.backend == "onnx":
        import onnxruntime as ort

        for stem in selected_models:
            runtimes[stem] = ort.InferenceSession(
                str(args.model_dir / f"{stem}.onnx"), providers=["CPUExecutionProvider"]
            )
    else:
        from rknnlite.api import RKNNLite

        for stem in selected_models:
            runtime = RKNNLite()
            if runtime.load_rknn(str(args.model_dir / f"{stem}.rknn")) != 0:
                raise RuntimeError(f"failed to load {stem}")
            if runtime.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO) != 0:
                raise RuntimeError(f"failed to initialize {stem}")
            runtimes[stem] = runtime
    payload: dict[str, Any] = {"backend": args.backend, "images": {}}
    try:
        for image_path in args.images:
            image = cv2.imread(str(image_path))
            if image is None:
                raise RuntimeError(f"failed to read {image_path}")
            model_results: dict[str, Any] = {}
            for stem in selected_models:
                size, classes_name = MODEL_SPECS[stem]
                classes = [
                    line.strip()
                    for line in (args.classes_dir / classes_name).read_text(encoding="utf-8").splitlines()
                    if line.strip()
                ]
                resized, scale, pad = letterbox(image, size)
                if args.backend == "onnx":
                    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
                    tensor = np.transpose(rgb, (2, 0, 1))[None, ...]
                    session = runtimes[stem]
                    output = session.run(None, {session.get_inputs()[0].name: tensor})[0]
                else:
                    tensor = np.expand_dims(cv2.cvtColor(resized, cv2.COLOR_BGR2RGB), 0)
                    outputs = runtimes[stem].inference(inputs=[np.ascontiguousarray(tensor)])
                    output = outputs[0]
                model_results[stem] = decode(
                    output,
                    classes,
                    image.shape[:2],
                    scale,
                    pad,
                    args.conf,
                    args.iou,
                )
            payload["images"][image_path.name] = model_results
    finally:
        if args.backend == "rknn":
            for runtime in runtimes.values():
                runtime.release()
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0


def box_iou(left: list[float], right: list[float]) -> float:
    x1, y1 = max(left[0], right[0]), max(left[1], right[1])
    x2, y2 = min(left[2], right[2]), min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    right_area = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union > 0.0 else 0.0


def compare(args: argparse.Namespace) -> int:
    reference = json.loads(args.reference.read_text(encoding="utf-8"))
    candidate = json.loads(args.candidate.read_text(encoding="utf-8"))
    total = matched = 0
    ious: list[float] = []
    class_mismatches: list[str] = []
    for image_name, models in reference.get("images", {}).items():
        for model_name, detections in models.items():
            available = list(candidate.get("images", {}).get(image_name, {}).get(model_name, []))
            for detection in detections:
                if float(detection.get("confidence", 0.0)) < args.high_conf:
                    continue
                total += 1
                same_class = [
                    item for item in available if item.get("class_name") == detection.get("class_name")
                ]
                if not same_class:
                    class_mismatches.append(
                        f"{image_name}:{model_name}:{detection.get('class_name')}"
                    )
                    continue
                best = max(
                    same_class,
                    key=lambda item: box_iou(detection["box_xyxy"], item["box_xyxy"]),
                )
                value = box_iou(detection["box_xyxy"], best["box_xyxy"])
                available.remove(best)
                if value >= args.match_iou:
                    matched += 1
                    ious.append(value)
    retention = matched / total if total else 1.0
    mean_iou = sum(ious) / len(ious) if ious else 1.0
    result = {
        "reference_high_conf_count": total,
        "matched_count": matched,
        "retention": round(retention, 4),
        "mean_iou": round(mean_iou, 4),
        "class_mismatches": class_mismatches,
        "passed": retention >= 0.95 and mean_iou >= 0.90 and not class_mismatches,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 2


def main() -> int:
    parser = argparse.ArgumentParser(description="Run or compare ONNX/RKNN cabinet detections.")
    subparsers = parser.add_subparsers(dest="command", required=True)
    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("--backend", choices=("onnx", "rknn"), required=True)
    run_parser.add_argument("--model-dir", type=Path, required=True)
    run_parser.add_argument("--classes-dir", type=Path, required=True)
    run_parser.add_argument("--output", type=Path, required=True)
    run_parser.add_argument("--conf", type=float, default=0.25)
    run_parser.add_argument("--iou", type=float, default=0.45)
    run_parser.add_argument("--model", action="append", choices=tuple(MODEL_SPECS))
    run_parser.add_argument("images", type=Path, nargs="+")
    compare_parser = subparsers.add_parser("compare")
    compare_parser.add_argument("--reference", type=Path, required=True)
    compare_parser.add_argument("--candidate", type=Path, required=True)
    compare_parser.add_argument("--high-conf", type=float, default=0.50)
    compare_parser.add_argument("--match-iou", type=float, default=0.50)
    args = parser.parse_args()
    return run(args) if args.command == "run" else compare(args)


if __name__ == "__main__":
    raise SystemExit(main())
