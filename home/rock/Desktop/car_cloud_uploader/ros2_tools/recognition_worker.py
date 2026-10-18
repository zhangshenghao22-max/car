#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import socketserver
import threading
import time
from typing import Any

import cv2
import numpy as np

from camera_yolo_detect import (
    DEFAULT_CLASSES,
    DEFAULT_DIGIT_CLASSES,
    DEFAULT_DIGIT_MODEL,
    DEFAULT_EXTRA_CLASSES,
    DEFAULT_EXTRA_MODEL,
    DEFAULT_MODEL,
    DEFAULT_SWITCH_CLASSES,
    DEFAULT_SWITCH_MODEL,
    decode_detections,
    imgsz_for_model,
    letterbox,
    load_classes,
)


ROOT = Path(
    os.environ.get("CAR_CLOUD_UPLOADER_ROOT")
    or Path(__file__).resolve().parents[1]
)
DEFAULT_SOCKET = ROOT / "runtime" / "recognition_worker" / "worker.sock"
DEFAULT_RKNN_DIR = ROOT / "yolo_models" / "rknn"


class ModelRuntime:
    def __init__(self, model_path: Path, classes_path: Path, imgsz: int, rknn_dir: Path):
        self.model_path = model_path.resolve()
        self.classes_path = classes_path.resolve()
        self.classes = load_classes(self.classes_path)
        self.imgsz = int(imgsz)
        self.runtime: Any = None
        self.backend = ""
        self.load_error = ""
        self._load(rknn_dir)

    def _load(self, rknn_dir: Path) -> None:
        errors: list[str] = []
        rknn_path = rknn_dir / f"{self.model_path.stem}.rknn"
        try:
            from rknnlite.api import RKNNLite

            if not rknn_path.is_file():
                raise FileNotFoundError(rknn_path)
            runtime = RKNNLite()
            code = runtime.load_rknn(str(rknn_path))
            if code != 0:
                raise RuntimeError(f"load_rknn returned {code}")
            code = runtime.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
            if code != 0:
                runtime.release()
                raise RuntimeError(f"init_runtime returned {code}")
            self.runtime = runtime
            self.backend = "rknn"
            return
        except Exception as exc:
            errors.append(f"rknn: {exc}")

        try:
            import onnxruntime as ort

            self.runtime = ort.InferenceSession(
                str(self.model_path),
                providers=["CPUExecutionProvider"],
            )
            self.backend = "onnxruntime"
            self.load_error = "; ".join(errors)
            return
        except Exception as exc:
            errors.append(f"onnxruntime: {exc}")

        try:
            self.runtime = cv2.dnn.readNetFromONNX(str(self.model_path))
            self.backend = "opencv"
            self.load_error = "; ".join(errors)
        except Exception as exc:
            errors.append(f"opencv: {exc}")
            raise RuntimeError(f"cannot load {self.model_path}: {'; '.join(errors)}") from exc

    def infer(
        self,
        image: np.ndarray,
        conf_threshold: float,
        iou_threshold: float,
    ) -> tuple[list[dict[str, Any]], float]:
        blob_image, scale, pad = letterbox(image, self.imgsz)
        started = time.perf_counter()
        if self.backend == "rknn":
            rgb = np.ascontiguousarray(
                np.expand_dims(cv2.cvtColor(blob_image, cv2.COLOR_BGR2RGB), axis=0)
            )
            outputs = self.runtime.inference(inputs=[rgb])
            if not outputs or len(outputs) != 1:
                count = 0 if not outputs else len(outputs)
                raise RuntimeError(f"RKNN output contract mismatch: {count} tensors")
            output = outputs[0]
        else:
            blob = cv2.dnn.blobFromImage(
                blob_image,
                scalefactor=1.0 / 255.0,
                size=(self.imgsz, self.imgsz),
                swapRB=True,
            )
            if self.backend == "onnxruntime":
                input_name = self.runtime.get_inputs()[0].name
                output = self.runtime.run(None, {input_name: blob.astype(np.float32)})[0]
            else:
                self.runtime.setInput(blob)
                output = self.runtime.forward()
        infer_ms = (time.perf_counter() - started) * 1000.0
        detections = decode_detections(
            output,
            image.shape[:2],
            self.classes,
            scale,
            pad,
            conf_threshold,
            iou_threshold,
        )
        return detections, infer_ms

    def close(self) -> None:
        if self.backend == "rknn" and self.runtime is not None:
            try:
                self.runtime.release()
            except Exception:
                pass


class RecognitionEngine:
    def __init__(self, rknn_dir: Path):
        started = time.perf_counter()
        specs = (
            (DEFAULT_MODEL, DEFAULT_CLASSES, 640),
            (DEFAULT_EXTRA_MODEL, DEFAULT_EXTRA_CLASSES, imgsz_for_model(DEFAULT_EXTRA_MODEL, 640)),
            (DEFAULT_DIGIT_MODEL, DEFAULT_DIGIT_CLASSES, imgsz_for_model(DEFAULT_DIGIT_MODEL, 640)),
            (DEFAULT_SWITCH_MODEL, DEFAULT_SWITCH_CLASSES, imgsz_for_model(DEFAULT_SWITCH_MODEL, 640)),
        )
        self.models: dict[str, ModelRuntime] = {}
        for model_path, classes_path, imgsz in specs:
            runtime = ModelRuntime(model_path, classes_path, imgsz, rknn_dir)
            self.models[str(runtime.model_path)] = runtime
        self.lock = threading.Lock()
        self.loaded_at = time.time()
        self.load_ms = round((time.perf_counter() - started) * 1000.0, 3)

    def status(self) -> dict[str, Any]:
        return {
            "ok": True,
            "loaded_at": self.loaded_at,
            "load_ms": self.load_ms,
            "models": {
                runtime.model_path.stem: {
                    "backend": runtime.backend,
                    "imgsz": runtime.imgsz,
                    "load_warning": runtime.load_error,
                }
                for runtime in self.models.values()
            },
        }

    def execute(self, request: dict[str, Any]) -> dict[str, Any]:
        action = str(request.get("action") or "infer")
        if action == "health":
            return self.status()
        if action != "infer":
            raise ValueError(f"unsupported action: {action}")

        image_path = Path(str(request.get("image_path") or "")).resolve()
        model_path = Path(str(request.get("model_path") or "")).resolve()
        runtime = self.models.get(str(model_path))
        if runtime is None:
            raise ValueError(f"model is not preloaded: {model_path}")
        if not image_path.is_file():
            raise FileNotFoundError(f"image not found: {image_path}")
        image = cv2.imread(str(image_path))
        if image is None:
            raise RuntimeError(f"failed to read image: {image_path}")
        conf = max(0.0, min(1.0, float(request.get("conf", 0.25))))
        iou = max(0.0, min(1.0, float(request.get("iou", 0.45))))
        queued_at = time.perf_counter()
        with self.lock:
            queue_ms = (time.perf_counter() - queued_at) * 1000.0
            detections, infer_ms = runtime.infer(image, conf, iou)
        return {
            "ok": True,
            "backend": runtime.backend,
            "detections": detections,
            "inference_ms": round(infer_ms, 3),
            "queue_ms": round(queue_ms, 3),
        }

    def close(self) -> None:
        for runtime in self.models.values():
            runtime.close()


class RequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        raw = self.rfile.readline(1024 * 1024)
        try:
            request = json.loads(raw.decode("utf-8"))
            if not isinstance(request, dict):
                raise ValueError("request must be a JSON object")
            response = self.server.engine.execute(request)
        except Exception as exc:
            response = {"ok": False, "error": str(exc)}
        self.wfile.write((json.dumps(response, ensure_ascii=False) + "\n").encode("utf-8"))


class RecognitionServer(socketserver.UnixStreamServer):
    def __init__(self, socket_path: Path, engine: RecognitionEngine):
        self.engine = engine
        super().__init__(str(socket_path), RequestHandler)


def main() -> int:
    parser = argparse.ArgumentParser(description="Persistent RKNN/ONNX recognition worker")
    parser.add_argument("--socket", type=Path, default=Path(os.environ.get("CAR_CLOUD_RECOGNITION_SOCKET", DEFAULT_SOCKET)))
    parser.add_argument("--rknn-model-dir", type=Path, default=Path(os.environ.get("CAR_CLOUD_RKNN_MODEL_DIR", DEFAULT_RKNN_DIR)))
    args = parser.parse_args()
    args.socket.parent.mkdir(parents=True, exist_ok=True)
    if args.socket.exists():
        args.socket.unlink()

    engine = RecognitionEngine(args.rknn_model_dir)
    server = RecognitionServer(args.socket, engine)
    os.chmod(args.socket, 0o660)
    print(json.dumps(engine.status(), ensure_ascii=False), flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        server.server_close()
        engine.close()
        if args.socket.exists():
            args.socket.unlink()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
