#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import time
from pathlib import Path


def process_rss_kb(pattern: str) -> int:
    pid_text = subprocess.check_output(["pgrep", "-f", pattern], text=True).splitlines()[0]
    status = Path(f"/proc/{int(pid_text)}/status").read_text(encoding="utf-8")
    return next(
        int(line.split()[1]) for line in status.splitlines() if line.startswith("VmRSS:")
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor the cabinet camera overlay health file.")
    parser.add_argument(
        "--health",
        type=Path,
        default=Path("/run/user/1000/cabinet-camera/health.json"),
    )
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    samples: list[dict] = []
    started = time.time()
    deadline = started + max(1.0, args.duration)
    next_report = started + 60.0
    while time.time() < deadline:
        try:
            payload = json.loads(args.health.read_text(encoding="utf-8"))
            samples.append(
                {
                    "at": time.time(),
                    "age_s": max(0.0, time.time() - float(payload.get("updated_at", 0.0))),
                    "display_fps": float(payload.get("camera", {}).get("display_fps", 0.0)),
                    "inference_fps": float(payload.get("inference", {}).get("fps", 0.0)),
                    "inference_ms": float(
                        payload.get("inference", {}).get("inference_ms", 0.0)
                    ),
                    "error": str(payload.get("inference", {}).get("error") or ""),
                    "tag": payload.get("apriltag", {}).get("reference_id"),
                    "switches": len(payload.get("switches", [])),
                    "rss_kb": process_rss_kb("python3 .*cabinet_camera_overlay.py"),
                }
            )
        except Exception as exc:
            samples.append({"at": time.time(), "probe_error": str(exc)})
        if time.time() >= next_report:
            valid = [item for item in samples if "display_fps" in item]
            print(
                json.dumps(
                    {
                        "elapsed_s": round(time.time() - started),
                        "samples": len(samples),
                        "last_display_fps": valid[-1]["display_fps"] if valid else None,
                        "last_inference_fps": valid[-1]["inference_fps"] if valid else None,
                    }
                ),
                flush=True,
            )
            next_report += 60.0
        time.sleep(max(0.2, args.interval))

    valid = [item for item in samples if "display_fps" in item]
    summary = {
        "duration_s": round(time.time() - started, 3),
        "sample_count": len(samples),
        "valid_count": len(valid),
        "probe_error_count": len(samples) - len(valid),
        "health_age_max_s": round(max(item["age_s"] for item in valid), 3) if valid else None,
        "display_fps_mean": round(
            statistics.mean(item["display_fps"] for item in valid), 3
        )
        if valid
        else None,
        "display_fps_min": round(min(item["display_fps"] for item in valid), 3)
        if valid
        else None,
        "inference_fps_mean": round(
            statistics.mean(item["inference_fps"] for item in valid), 3
        )
        if valid
        else None,
        "inference_fps_min": round(min(item["inference_fps"] for item in valid), 3)
        if valid
        else None,
        "inference_ms_mean": round(
            statistics.mean(item["inference_ms"] for item in valid), 3
        )
        if valid
        else None,
        "runtime_error_count": sum(bool(item["error"]) for item in valid),
        "tag_present_ratio": round(
            sum(item["tag"] is not None for item in valid) / len(valid), 3
        )
        if valid
        else None,
        "switch_present_ratio": round(
            sum(item["switches"] > 0 for item in valid) / len(valid), 3
        )
        if valid
        else None,
        "rss_start_kb": valid[0]["rss_kb"] if valid else None,
        "rss_end_kb": valid[-1]["rss_kb"] if valid else None,
        "rss_growth_kb": valid[-1]["rss_kb"] - valid[0]["rss_kb"] if valid else None,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps({"summary": summary, "samples": samples}, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if valid and not summary["probe_error_count"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
