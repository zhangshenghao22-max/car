#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
from rknnlite.api import RKNNLite


MODEL_SIZES = {
    "lightlabel_best.rknn": 640,
    "meter_digit_unified_best.rknn": 1280,
    "digits_best.rknn": 640,
    "switch_yolo_best.rknn": 640,
}


def benchmark(path: Path, size: int, repeats: int) -> dict[str, object]:
    runtime = RKNNLite()
    try:
        code = runtime.load_rknn(str(path))
        if code != 0:
            raise RuntimeError(f"load_rknn failed with code {code}: {path}")
        code = runtime.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
        if code != 0:
            raise RuntimeError(f"init_runtime failed with code {code}: {path}")
        image = np.full((1, size, size, 3), 114, dtype=np.uint8)
        outputs = runtime.inference(inputs=[image])
        if not outputs or len(outputs) != 1:
            raise RuntimeError(f"unexpected output count for {path}: {len(outputs or [])}")
        timings: list[float] = []
        for _ in range(max(1, repeats)):
            started = time.perf_counter()
            outputs = runtime.inference(inputs=[image])
            timings.append((time.perf_counter() - started) * 1000.0)
        return {
            "model": path.name,
            "input_shape": list(image.shape),
            "output_shape": list(outputs[0].shape),
            "output_dtype": str(outputs[0].dtype),
            "mean_inference_ms": round(sum(timings) / len(timings), 3),
            "max_inference_ms": round(max(timings), 3),
        }
    finally:
        runtime.release()


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke-test cabinet RKNN models on an RK3588 board.")
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=5)
    args = parser.parse_args()
    results = [
        benchmark(args.model_dir / filename, size, args.repeats)
        for filename, size in MODEL_SIZES.items()
    ]
    by_name = {str(item["model"]): item for item in results}
    light_ms = float(by_name["lightlabel_best.rknn"]["mean_inference_ms"])
    unified_ms = float(by_name["meter_digit_unified_best.rknn"]["mean_inference_ms"])
    switch_ms = float(by_name["switch_yolo_best.rknn"]["mean_inference_ms"])
    monitor_cycle_ms = light_ms * 2.0 + unified_ms
    payload = {
        "models": results,
        "monitor_schedule": ["lightlabel", "lightlabel", "meter_digit_unified"],
        "monitor_cycle_ms": round(monitor_cycle_ms, 3),
        "monitor_pipeline_fps": round(3000.0 / monitor_cycle_ms, 3)
        if monitor_cycle_ms > 0.0
        else 0.0,
        "switch_live_fps": round(1000.0 / switch_ms, 3) if switch_ms > 0.0 else 0.0,
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    if payload["monitor_pipeline_fps"] < 5.0:
        return 2
    if payload["switch_live_fps"] < 5.0:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
