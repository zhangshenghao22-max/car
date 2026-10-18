from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_tools"))

from cabinet_apriltag_inspection import (  # noqa: E402
    CabinetAprilTagInspection,
    MeterProcessingTimeout,
)
from meter_pipeline import (  # noqa: E402
    MeterConfig,
    find_pointer_by_ray,
    rectify_meter_crop_detailed,
)


class MeterStageTimeoutTests(unittest.TestCase):
    def test_fast_alignment_bypasses_template_registration(self) -> None:
        config = MeterConfig(
            min_value=0.0,
            max_value=1.0,
            center=(512.0, 730.0),
            zero_angle=130.0,
            full_angle=50.0,
            canonical_size=1024,
            rectify_mode="template",
            template_path="unused.jpg",
        )
        crop = np.full((300, 400, 3), 255, dtype=np.uint8)
        with mock.patch("meter_pipeline.align_by_ecc") as ecc, mock.patch(
            "meter_pipeline.align_by_orb"
        ) as orb:
            image, method, quality = rectify_meter_crop_detailed(
                crop,
                config,
                fast_alignment=True,
            )
        self.assertEqual(image.shape, (1024, 1024, 3))
        self.assertTrue(method.startswith("fast-"))
        self.assertTrue(quality["fast_alignment"])
        ecc.assert_not_called()
        orb.assert_not_called()

    def test_expired_pointer_deadline_aborts_before_cpu_scan(self) -> None:
        config = MeterConfig(
            min_value=0.0,
            max_value=450.0,
            center=(512.0, 512.0),
            zero_angle=130.0,
            full_angle=40.0,
            canonical_size=1024,
        )
        mask = np.zeros((1024, 1024), dtype=np.uint8)
        with self.assertRaises(MeterProcessingTimeout):
            find_pointer_by_ray(mask, config, deadline=0.0)

    def test_meter_frame_timeout_returns_without_blocking_stage(self) -> None:
        runner = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
        runner.meter_configs = {"voltage": object()}
        runner.profile = {"recognition": {"meter_processing_timeout_s": 0.2}}
        runner.meter_processing_timeout_count = 0
        detections = [
            {
                "class_name": "yakuang",
                "confidence": 0.9,
                "box_xyxy": [10, 10, 100, 100],
            }
        ]
        with mock.patch(
            "cabinet_apriltag_inspection.read_meter_from_box",
            side_effect=MeterProcessingTimeout("test timeout"),
        ):
            result = runner.analyze_meter_frame(
                "voltage_meter",
                np.zeros((120, 120, 3), dtype=np.uint8),
                detections,
                0,
            )
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "meter_processing_timeout")
        self.assertEqual(runner.meter_processing_timeout_count, 1)


if __name__ == "__main__":
    unittest.main()
