#!/usr/bin/env python3
"""Capture a bounded batch of full-resolution camera frames.

This helper intentionally runs outside the ROS inspection process. The parent
can kill it if a V4L2 read blocks without leaving arm safety handling blocked.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any, Callable

import cv2


def capture_frames(
    *,
    device: str,
    output_dir: Path,
    prefix: str,
    width: int,
    height: int,
    fps: int,
    warmup_frames: int,
    candidate_count: int,
    jpeg_quality: int,
    rotate_180: bool,
    capture_factory: Callable[..., Any] = cv2.VideoCapture,
) -> list[Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    capture = capture_factory(device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    capture.set(cv2.CAP_PROP_FPS, int(fps))
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError("cannot open camera for high-resolution capture")

    paths: list[Path] = []
    try:
        for _ in range(max(0, int(warmup_frames))):
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("high-resolution camera warmup failed")
        for index in range(max(1, int(candidate_count))):
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("high-resolution camera capture failed")
            actual = (int(frame.shape[1]), int(frame.shape[0]))
            expected = (int(width), int(height))
            if actual != expected:
                raise RuntimeError(f"high-resolution camera size mismatch: {actual} != {expected}")
            if rotate_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            path = output_dir / f"{prefix}_candidate_{index + 1}.jpg"
            if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)]):
                raise RuntimeError("failed to save high-resolution candidate")
            paths.append(path)
    finally:
        capture.release()
    return paths


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture full-resolution inspection frames.")
    parser.add_argument("--device", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--warmup-frames", type=int, default=5)
    parser.add_argument("--candidate-count", type=int, default=3)
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--rotate-180", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    started_at = time.monotonic()
    try:
        paths = capture_frames(
            device=str(args.device),
            output_dir=Path(args.output_dir),
            prefix=str(args.prefix),
            width=int(args.width),
            height=int(args.height),
            fps=int(args.fps),
            warmup_frames=int(args.warmup_frames),
            candidate_count=int(args.candidate_count),
            jpeg_quality=int(args.jpeg_quality),
            rotate_180=bool(args.rotate_180),
        )
        payload = {
            "ok": True,
            "paths": [str(path) for path in paths],
            "elapsed_s": round(time.monotonic() - started_at, 3),
        }
        print(json.dumps(payload, ensure_ascii=False), flush=True)
        return 0
    except Exception as exc:
        payload = {
            "ok": False,
            "error": str(exc),
            "elapsed_s": round(time.monotonic() - started_at, 3),
        }
        print(json.dumps(payload, ensure_ascii=False), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
