from __future__ import annotations

import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
import sys
import yaml


TOOLS_DIR = Path(__file__).resolve().parents[1] / "ros2_tools"
sys.path.insert(0, str(TOOLS_DIR))

from cabinet_apriltag_inspection import (  # noqa: E402
    Candidate,
    CameraFrameTimeout,
    CabinetAprilTagInspection,
    ContinuousHighResolutionCamera,
    DeferredCapture,
    StreamRectifier,
    adapt_center_crop_intrinsics,
    assign_fixed_meter_layout,
    best_effort_digit_consensus,
    best_effort_light_consensus,
    best_effort_meter_fusion,
    digit_consensus,
    fuse_meter_readings,
    light_consensus,
    light_observations_by_stage,
    merge_cabinet_update,
    observation_action,
    prioritize_structured_inspection_payload,
    scale_box,
    scale_detections,
    stage_cabinet_update,
    switch_consensus,
    switch_observation,
    temperature_digit_candidates,
    temperature_digit_consensus,
)


def meter_reading(value: float, index: int, meter_type: str = "voltage") -> dict:
    return {
        "value": value,
        "raw_value": value,
        "pointer_angle": 30.0 + index,
        "quality_score": 1.0 + index / 10.0,
        "frame_quality_score": 0.8,
        "_observation_index": index,
        "meter_type": meter_type,
        "unit": "V" if meter_type == "voltage" else "A",
    }


class ContinuousHighResolutionInspectionTests(unittest.TestCase):
    @staticmethod
    def camera_config(**overrides: object) -> dict:
        config = {
            "backend": "v4l2",
            "width": 4,
            "height": 3,
            "fps": 15,
            "startup_timeout_s": 0.05,
            "startup_attempts": 2,
            "startup_retry_backoff_s": 0.0,
            "read_timeout_s": 1.0,
        }
        config.update(overrides)
        return config

    @staticmethod
    def failed_capture() -> mock.Mock:
        capture = mock.Mock()
        capture.read.return_value = (False, None)
        return capture

    @staticmethod
    def healthy_capture() -> mock.Mock:
        capture = mock.Mock()
        frame = np.zeros((3, 4, 3), dtype=np.uint8)
        capture.read.return_value = (True, frame)
        return capture

    def test_camera_open_recovers_after_first_reader_failure(self) -> None:
        camera = ContinuousHighResolutionCamera(
            {"device": "/dev/video-test", "rotate_180": False},
            self.camera_config(),
        )
        failed = self.failed_capture()
        healthy = self.healthy_capture()
        with mock.patch.object(camera, "_open_capture", side_effect=[failed, healthy]):
            camera.open()
        try:
            diagnostics = camera.diagnostics()
            self.assertEqual(diagnostics["startup_attempts_used"], 2)
            self.assertEqual(diagnostics["startup_recovery_count"], 1)
            self.assertIn("read failed", diagnostics["last_startup_error"])
            self.assertIsNotNone(camera.latest())
            failed.release.assert_called()
        finally:
            camera.close()

    def test_camera_open_recovers_after_delayed_first_frame_timeout(self) -> None:
        camera = ContinuousHighResolutionCamera(
            {"device": "/dev/video-test", "rotate_180": False},
            self.camera_config(startup_timeout_s=0.1),
        )
        delayed = self.healthy_capture()
        frame = np.zeros((3, 4, 3), dtype=np.uint8)

        def delayed_read() -> tuple[bool, np.ndarray]:
            time.sleep(0.2)
            return True, frame

        delayed.read.side_effect = delayed_read
        healthy = self.healthy_capture()
        with mock.patch.object(camera, "_open_capture", side_effect=[delayed, healthy]):
            camera.open()
        try:
            diagnostics = camera.diagnostics()
            self.assertEqual(diagnostics["startup_attempts_used"], 2)
            self.assertEqual(diagnostics["startup_recovery_count"], 1)
            self.assertIn("no fresh frame", diagnostics["last_startup_error"])
            self.assertIsNotNone(camera.latest())
            delayed.release.assert_called()
        finally:
            camera.close()

    def test_camera_open_fails_only_after_all_startup_attempts(self) -> None:
        camera = ContinuousHighResolutionCamera(
            {"device": "/dev/video-test", "rotate_180": False},
            self.camera_config(startup_attempts=3),
        )
        captures = [self.failed_capture() for _ in range(3)]
        with mock.patch.object(camera, "_open_capture", side_effect=captures):
            with self.assertRaisesRegex(CameraFrameTimeout, "startup failed after 3 attempts"):
                camera.open()
        self.assertEqual(camera.diagnostics()["startup_attempts_used"], 3)
        self.assertIsNone(camera.capture)
        for capture in captures:
            capture.release.assert_called()

    def test_camera_read_uses_configured_timeout_and_counts_failure(self) -> None:
        camera = ContinuousHighResolutionCamera(
            {"device": "/dev/video-test", "rotate_180": False},
            self.camera_config(read_timeout_s=1.0),
        )
        with mock.patch.object(
            camera,
            "wait_latest",
            side_effect=CameraFrameTimeout("temporary frame stall"),
        ) as wait_latest:
            with self.assertRaisesRegex(CameraFrameTimeout, "temporary frame stall"):
                camera.read()
        wait_latest.assert_called_once_with(0, timeout_s=1.0, copy_frame=True)
        self.assertEqual(camera.diagnostics()["read_timeout_count"], 1)

    def test_center_crop_intrinsics_expand_720_calibration_to_960_stream(self) -> None:
        camera_matrix = np.asarray(
            [[700.0, 0.0, 640.0], [0.0, 700.0, 300.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        adapted, adapted_new = adapt_center_crop_intrinsics(
            camera_matrix,
            camera_matrix.copy(),
            (1280, 720),
            (1280, 960),
        )
        self.assertAlmostEqual(adapted[0, 0], 700.0)
        self.assertAlmostEqual(adapted[1, 1], 700.0)
        self.assertAlmostEqual(adapted[0, 2], 640.0)
        self.assertAlmostEqual(adapted[1, 2], 420.0)
        np.testing.assert_allclose(adapted_new, adapted)

    def test_stream_rectifier_keeps_frame_and_point_coordinates_consistent(self) -> None:
        intrinsics = {
            "model": "fisheye",
            "image_size": [1280, 720],
            "camera_matrix": [[700.0, 0.0, 640.0], [0.0, 700.0, 300.0], [0.0, 0.0, 1.0]],
            "dist_coeffs": [0.0, 0.0, 0.0, 0.0],
            "new_camera_matrix": [[700.0, 0.0, 640.0], [0.0, 700.0, 300.0], [0.0, 0.0, 1.0]],
        }
        rectifier = StreamRectifier(intrinsics, (1280, 960))
        frame = np.zeros((960, 1280, 3), dtype=np.uint8)
        self.assertEqual(rectifier.rectify(frame).shape, frame.shape)
        points = np.asarray([[640.0, 420.0], [320.0, 240.0], [960.0, 720.0]])
        mapped = rectifier.rectify_points(points)
        self.assertEqual(mapped.shape, points.shape)
        self.assertTrue(np.isfinite(mapped).all())

    def test_high_resolution_box_maps_to_stream_resolution(self) -> None:
        self.assertEqual(
            scale_box([816, 612, 2448, 1836], (3264, 2448), (1280, 960)),
            [320, 240, 960, 720],
        )
        mapped = scale_detections(
            [{"class_name": "yakuang", "box_xyxy": [320, 240, 960, 720]}],
            (1280, 960),
            (3264, 2448),
        )
        self.assertEqual(mapped[0]["box_xyxy"], [816, 612, 2448, 1836])

    def test_meter_layout_forces_left_voltage_and_right_current(self) -> None:
        detections = [
            {"class_id": 0, "class_name": "yakuang", "confidence": 0.95, "box_xyxy": [100, 40, 300, 220]},
            {"class_id": 0, "class_name": "yakuang", "confidence": 0.78, "box_xyxy": [500, 35, 710, 225]},
            {"class_id": 3, "class_name": "1", "confidence": 0.90, "box_xyxy": [800, 40, 830, 100]},
        ]
        assigned = assign_fixed_meter_layout(detections)
        meters = [item for item in assigned if item.get("meter_layout_assignment")]
        self.assertEqual([item["class_name"] for item in meters], ["yakuang", "liukuang"])
        self.assertEqual(meters[0]["meter_layout_assignment"], "fixed_layout_left_voltage")
        self.assertEqual(meters[1]["meter_layout_assignment"], "fixed_layout_right_current")
        self.assertEqual(meters[1]["model_class_name"], "yakuang")

    def test_combined_meter_analysis_runs_both_readers_concurrently(self) -> None:
        runner = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
        runner.profile = {"recognition": {"meter_parallel_workers": 2}}
        barrier = threading.Barrier(2)

        def analyze(stage, _frame, _detections, index):
            barrier.wait(timeout=1.0)
            return {"valid": True, "meter_type": stage, "_observation_index": index}

        with mock.patch.object(runner, "analyze_meter_frame", side_effect=analyze):
            result = runner.analyze_meter_pair_frame(
                np.zeros((100, 100, 3), dtype=np.uint8),
                [],
                3,
            )
        self.assertEqual(result["meter_parallel_workers"], 2)
        self.assertEqual(result["meter_type"], "current_meter")
        self.assertEqual(result["voltage_analysis"]["meter_type"], "voltage_meter")

    def test_stage_uses_only_required_model(self) -> None:
        runner = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
        runner.light_model_paths = [Path("lightlabel_best.onnx")]
        runner.switch_model_paths = [Path("switch_yolo_best.onnx")]
        runner.meter_model_paths = [Path("meter_digit_unified_best.onnx")]
        runner.digit_model_paths = [Path("digits_best.onnx")]
        self.assertEqual(
            runner.model_paths_for_stage("upper_status_lights"),
            [*runner.light_model_paths, *runner.switch_model_paths],
        )
        self.assertEqual(
            runner.model_paths_for_stage("voltage_meter"),
            [*runner.meter_model_paths, *runner.digit_model_paths],
        )
        self.assertEqual(
            runner.model_paths_for_stage("temperature_controller"),
            [*runner.digit_model_paths, *runner.meter_model_paths],
        )
        self.assertEqual(runner.model_paths_for_stage("middle_transit"), [])

    def test_scan_profile_uses_combined_meter_and_light_waypoints(self) -> None:
        profile_path = Path(__file__).resolve().parents[1] / "cabinet_inspection_profiles" / "fixed_scan.yaml"
        profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
        waypoints = profile["scan"]["waypoints"]
        self.assertEqual(
            [waypoint["name"] for waypoint in waypoints],
            ["top_approach", "current_meter", "upper_status_lights"],
        )
        self.assertEqual(
            [waypoint["name"] for waypoint in waypoints if waypoint.get("capture")],
            ["current_meter", "upper_status_lights"],
        )
        self.assertNotIn("voltage_meter", [waypoint["name"] for waypoint in waypoints])
        self.assertNotIn("middle_transit", [waypoint["name"] for waypoint in waypoints])
        self.assertNotIn("temperature_controller", [waypoint["name"] for waypoint in waypoints])
        self.assertNotIn("lower_status_lights", [waypoint["name"] for waypoint in waypoints])
        self.assertIn("yolo_models/switch_yolo_best.onnx", profile["recognition"]["models"])
        self.assertTrue(profile["recognition"]["current_meter_fast_alignment"])
        self.assertEqual(profile["recognition"]["current_meter_fast_canonical_size"], 640)
        self.assertEqual(profile["best_effort"]["voltage_fallback"], 220.0)
        self.assertEqual(profile["best_effort"]["current_fallback"], 0.0)

    def test_meter_stage_collects_derived_temperature_consensus(self) -> None:
        runner = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
        runner.profile = {
            "quality": {
                "meter_min_valid_results": 3,
                "meter_min_inliers": 2,
                "voltage_unstable_delta": 6.0,
                "digit_min_valid_results": 3,
                "digit_min_winner_votes": 2,
                "digit_min_winner_ratio": 0.67,
            }
        }
        runner.stage_consensus_results = {}
        analyses = []
        for index in range(3):
            analyses.append(
                {
                    "analysis": {
                        "meter_reading": meter_reading(201.0 + index, index),
                        "temperature_analysis": {
                            "valid": True,
                            "upper_text": "137",
                            "quality_score": 0.8,
                            "selected_candidate": {"center": [0.5, 0.3], "model_support": 2},
                        },
                    }
                }
            )
        result = runner.stage_consensus("voltage_meter", analyses)
        self.assertEqual(result["status"], "ok")
        self.assertIn("temperature_controller", result["derived_consensus"])
        self.assertEqual(result["derived_consensus"]["temperature_controller"]["rows"], ["137"])

    def test_one_light_observation_produces_both_rows(self) -> None:
        detections = [
            {"class_name": "green_on", "confidence": 0.9, "box_xyxy": [100, 100, 120, 120]},
            {"class_name": "red_off", "confidence": 0.9, "box_xyxy": [150, 100, 170, 120]},
            {"class_name": "green_off", "confidence": 0.9, "box_xyxy": [100, 300, 120, 320]},
            {"class_name": "red_on", "confidence": 0.9, "box_xyxy": [150, 300, 170, 320]},
        ]
        observations = light_observations_by_stage(detections)
        self.assertEqual(observations["upper_status_lights"]["upper_status_lights:green"], "on")
        self.assertEqual(observations["upper_status_lights"]["upper_status_lights:red"], "off")
        self.assertEqual(observations["lower_status_lights"]["lower_status_lights:green"], "off")
        self.assertEqual(observations["lower_status_lights"]["lower_status_lights:red"], "on")

    def test_switches_are_numbered_top_to_bottom_and_fused(self) -> None:
        frames = []
        for upper_state, lower_state in (("kai", "guan"), ("kai", "guan"), ("kai", "guan")):
            frames.append(
                switch_observation(
                    [
                        {"class_name": lower_state, "confidence": 0.88, "box_xyxy": [400, 600, 520, 720]},
                        {"class_name": upper_state, "confidence": 0.92, "box_xyxy": [410, 180, 530, 300]},
                    ]
                )
            )
        self.assertEqual(frames[0]["switch_1"]["state"], "kai")
        self.assertEqual(frames[0]["switch_2"]["state"], "guan")
        result = switch_consensus(frames, required_votes=2, minimum_observations=3, minimum_ratio=0.67)
        self.assertEqual(result["positions"], {"switch_1": "kai", "switch_2": "guan"})
        self.assertTrue(result["reliable"])

    def test_current_source_creates_voltage_and_temperature_stages(self) -> None:
        runner = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
        runner.profile = {
            "quality": {
                "digit_min_valid_results": 3,
                "digit_min_winner_votes": 2,
                "digit_min_winner_ratio": 0.67,
            }
        }
        runner.stage_consensus_results = {}
        with tempfile.TemporaryDirectory() as directory:
            def source(stage: str, index: int) -> DeferredCapture:
                analysis = {
                    "_observation_index": index,
                    "meter_reading": meter_reading(0.31, index, "current"),
                    "voltage_analysis": {
                        "valid": True,
                        "meter_type": "voltage",
                        "meter_reading": meter_reading(201.0, index, "voltage"),
                        "_observation_index": index,
                    },
                    "temperature_analysis": {
                        "valid": True,
                        "upper_text": "137",
                        "quality_score": 0.8,
                        "selected_candidate": {"center": [0.5, 0.3], "model_support": 2},
                    },
                }
                candidate = Candidate(Path(directory) / f"{stage}.jpg", [], {"score": 1.0}, analysis)
                return DeferredCapture(
                    stage=stage,
                    scan_pose=[1640, 1500, 1500, 2350, 1500, 1500],
                    candidates=[candidate],
                    selected=candidate,
                    observation_metadata=[{"observation_index": index, "analysis": analysis}],
                    observation_elapsed_s=1.0,
                    source_stage=stage,
                )

            current = source("current_meter", 1)
            current.consensus = {
                "derived_consensus": {
                    "voltage_meter": fuse_meter_readings(
                        [meter_reading(201.0, 1, "voltage")],
                        meter_type="voltage",
                        minimum_valid=3,
                        required_votes=2,
                        max_spread=6.0,
                    ),
                    "temperature_controller": temperature_digit_consensus(
                        [current.observation_metadata[0]["analysis"]["temperature_analysis"]],
                        required_votes=2,
                        minimum_observations=3,
                        minimum_ratio=0.67,
                    ),
                }
            }
            expanded = runner.expand_combined_captures([current])
            self.assertEqual([item.stage for item in expanded], ["voltage_meter", "current_meter", "temperature_controller"])
            self.assertTrue(all(item.source_stage == "current_meter" for item in expanded))
            self.assertEqual(expanded[0].selected.analysis["meter_type"], "voltage")

    def test_meter_uses_inlier_median_after_five_valid_results(self) -> None:
        values = [201.0, 202.0, 203.0, 240.0, 241.0]
        result = fuse_meter_readings(
            [meter_reading(value, index) for index, value in enumerate(values)],
            meter_type="voltage",
            minimum_valid=5,
            required_votes=3,
            max_spread=6.0,
        )
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["reading"]["value"], 202.0)
        self.assertEqual(result["consensus_count"], 3)
        self.assertEqual(result["inlier_observation_indices"], [0, 1, 2])

    def test_meter_does_not_publish_single_frame_estimate(self) -> None:
        result = fuse_meter_readings(
            [meter_reading(value, index, "current") for index, value in enumerate([0.30, 0.31, 0.32, 0.60])],
            meter_type="current",
            minimum_valid=5,
            required_votes=3,
            max_spread=0.05,
        )
        self.assertEqual(result["status"], "unstable")
        self.assertIsNone(result["reading"])
        self.assertIsNotNone(result["best_estimate"])

    def test_best_effort_meter_promotes_single_frame_estimate(self) -> None:
        strict = fuse_meter_readings(
            [meter_reading(0.37, 0, "current")],
            meter_type="current",
            minimum_valid=5,
            required_votes=3,
            max_spread=0.05,
        )
        result = best_effort_meter_fusion(strict, meter_type="current", fallback_value=0.0)
        self.assertEqual(result["reading"]["value"], 0.37)
        self.assertFalse(result["reliable"])
        self.assertEqual(result["best_effort_source"], "best_observed_frame")

    def test_best_effort_meter_uses_configured_fallback_when_unreadable(self) -> None:
        result = best_effort_meter_fusion(
            {"status": "missing", "reading": None},
            meter_type="voltage",
            fallback_value=220.0,
        )
        self.assertEqual(result["reading"]["value"], 220.0)
        self.assertTrue(result["fallback_used"])
        self.assertEqual(result["reading"]["cabinet_metric"]["status"], "estimated")

    def test_meter_analysis_keeps_outer_observation_index(self) -> None:
        fake_result = mock.Mock(
            value=201.0,
            status="ok",
            pointer_angle=30.0,
            pointer_method="test",
            pointer_candidates=[],
            pointer_consensus_count=3,
            rectify_method="auto-quad",
            rectify_quality={},
            quality_score=2.0,
        )
        runner = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
        runner.meter_configs = {"voltage": object(), "current": object()}
        with mock.patch("cabinet_apriltag_inspection.read_meter_from_box", return_value=fake_result):
            result = runner.analyze_meter_frame(
                "voltage_meter",
                np.zeros((100, 100, 3), dtype=np.uint8),
                [{"class_name": "yakuang", "confidence": 0.9, "box_xyxy": [10, 10, 90, 90]}],
                7,
            )
        self.assertEqual(result["_observation_index"], 7)
        self.assertEqual(result["meter_reading"]["_observation_index"], 7)

    def test_missing_meter_detection_can_receive_observation_index(self) -> None:
        runner = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
        result = runner.analyze_meter_frame(
            "voltage_meter",
            np.zeros((100, 100, 3), dtype=np.uint8),
            [],
            4,
        )
        result.setdefault("_observation_index", 4)
        self.assertFalse(result["valid"])
        self.assertEqual(result["_observation_index"], 4)

    def test_digit_consensus_requires_five_samples_and_sixty_percent(self) -> None:
        stable = digit_consensus([["201"]] * 3 + [["202"]] * 2, 3, 5, 0.60)
        unstable = digit_consensus([["201"], ["202"], ["201"], ["202"], ["203"]], 3, 5, 0.60)
        self.assertEqual(stable["rows"], ["201"])
        self.assertTrue(stable["complete"])
        self.assertEqual(unstable["rows"], [])
        self.assertFalse(unstable["complete"])

    def test_best_effort_digit_uses_winner_and_never_configured_fallback(self) -> None:
        unstable = digit_consensus([["201"], ["202"]], 3, 5, 0.60)
        promoted = best_effort_digit_consensus(unstable)
        missing = best_effort_digit_consensus({})
        self.assertEqual(promoted["rows"], ["201"])
        self.assertEqual(promoted["source"], "best_observed_text")
        self.assertEqual(missing["rows"], [])
        self.assertEqual(missing["status"], "unreadable")
        self.assertFalse(missing["fallback_used"])

    def test_temperature_candidates_keep_models_separate_and_choose_upper_137(self) -> None:
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        detections = []
        for model, confidence in (("digits_best", 0.90), ("meter_digit_unified_best", 0.84)):
            for digit, x in zip("137", (500, 550, 600)):
                detections.append({
                    "class_name": digit,
                    "confidence": confidence,
                    "box_xyxy": [x, 220, x + 38, 290],
                    "model": model,
                })
        for digit, x in zip("25", (530, 575)):
            detections.append({
                "class_name": digit,
                "confidence": 0.95,
                "box_xyxy": [x, 360, x + 38, 430],
                "model": "digits_best",
            })
        candidates = temperature_digit_candidates(frame, detections)
        self.assertEqual(candidates[0]["text"], "137")
        self.assertEqual(candidates[0]["model_support"], 2)

    def test_temperature_consensus_prefers_stable_137(self) -> None:
        observations = [
            {
                "valid": True,
                "upper_text": text,
                "quality_score": score,
                "selected_candidate": {"center": [0.5, 0.3], "model_support": 2},
            }
            for text, score in (("137", 0.8), ("137", 0.9), ("137", 0.85), ("25", 0.95), ("25", 0.92))
        ]
        result = temperature_digit_consensus(observations, 3, 5, 0.60)
        self.assertEqual(result["rows"], ["137"])
        self.assertTrue(result["reliable"])

    def test_temperature_conflict_promotes_best_real_candidate(self) -> None:
        observations = [
            {
                "valid": True,
                "upper_text": text,
                "quality_score": score,
                "selected_candidate": {"center": [0.5, 0.3], "model_support": 1},
            }
            for text, score in (("137", 0.9), ("137", 0.85), ("25", 0.8), ("25", 0.75))
        ]
        strict = temperature_digit_consensus(observations, 3, 5, 0.60)
        promoted = best_effort_digit_consensus(strict)
        self.assertEqual(promoted["rows"], ["137"])
        self.assertEqual(promoted["status"], "estimated")

    def test_unreadable_temperature_is_uploaded_as_structured_null(self) -> None:
        update = stage_cabinet_update(
            "temperature_controller",
            {
                "temperature_controller": {
                    "status": "unreadable",
                    "upper_text": "",
                    "missing_reason": "no_digits",
                }
            },
            {"digits": best_effort_digit_consensus({})},
        )
        self.assertEqual(update["temperature_controller"]["status"], "unreadable")
        self.assertEqual(update["temperature_controller"]["display_text"], "")

    def test_light_consensus_is_independent_per_position(self) -> None:
        observations = [
            {"upper_status_lights:green": "on", "upper_status_lights:red": "off"}
            for _ in range(5)
        ]
        result = light_consensus(
            observations,
            required_votes=4,
            minimum_observations=5,
            minimum_ratio=0.70,
            expected_positions={"upper_status_lights:green", "upper_status_lights:red"},
        )
        self.assertTrue(result["complete"])
        self.assertEqual(result["positions"]["upper_status_lights:green"], "on")
        self.assertEqual(result["positions"]["upper_status_lights:red"], "off")

    def test_best_effort_lights_fill_every_position(self) -> None:
        result = best_effort_light_consensus(
            {
                "complete": False,
                "positions": {},
                "winner_positions": {"upper_status_lights:green": "on"},
            },
            {
                "upper_status_lights:green": "off",
                "upper_status_lights:red": "off",
            },
        )
        self.assertEqual(result["positions"]["upper_status_lights:green"], "on")
        self.assertEqual(result["positions"]["upper_status_lights:red"], "off")
        self.assertTrue(result["complete"])

    def test_structured_upload_prioritizes_stages_before_cloud_key_limit(self) -> None:
        summary = {f"extra_{index}": index for index in range(30)}
        summary.update(
            {
                "inspection_id": "inspection-1",
                "stages": [{"stage": "voltage_meter"}],
                "cabinet_update": {"voltage": {"value": 220.0}},
                "expected_stage_count": 5,
            }
        )
        payload = prioritize_structured_inspection_payload(summary)
        first_twenty = list(payload)[:20]
        self.assertIn("stages", first_twenty)
        self.assertIn("cabinet_update", first_twenty)
        self.assertIn("expected_stage_count", first_twenty)

    def test_cabinet_update_keeps_both_light_rows(self) -> None:
        merged = {}
        merge_cabinet_update(
            merged,
            {"status_lights": {"lights": [{"position": "upper_status_lights:green", "state": "on"}]}},
        )
        merge_cabinet_update(
            merged,
            {"status_lights": {"lights": [{"position": "lower_status_lights:red", "state": "off"}]}},
        )
        self.assertEqual(
            [item["position"] for item in merged["status_lights"]["lights"]],
            ["upper_status_lights:green", "lower_status_lights:red"],
        )

    def test_observation_extends_once_then_times_out(self) -> None:
        self.assertEqual(observation_action(2.49, True, False, minimum_s=2.5, primary_s=6, extension_s=3), "continue")
        self.assertEqual(observation_action(2.5, True, False, minimum_s=2.5, primary_s=6, extension_s=3), "complete")
        self.assertEqual(observation_action(6.0, False, False, minimum_s=2.5, primary_s=6, extension_s=3), "extend")
        self.assertEqual(observation_action(7.0, False, True, minimum_s=2.5, primary_s=6, extension_s=3), "continue")
        self.assertEqual(observation_action(9.0, False, True, minimum_s=2.5, primary_s=6, extension_s=3), "timeout")

    def test_fast_profile_uses_short_observation_window(self) -> None:
        profile_path = Path(__file__).resolve().parents[1] / "cabinet_inspection_profiles" / "fixed_scan.yaml"
        profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
        observation = profile["observation"]
        quality = profile["quality"]
        self.assertEqual(observation["primary_duration_s"], 2.0)
        self.assertEqual(observation["extension_duration_s"], 0.5)
        self.assertEqual(observation["minimum_duration_s"], 1.0)
        self.assertEqual(quality["meter_min_valid_results"], 3)
        self.assertEqual(quality["meter_min_inliers"], 2)
        self.assertEqual(quality["digit_min_valid_results"], 3)
        self.assertEqual(quality["digit_min_winner_votes"], 2)
        self.assertEqual(quality["light_min_valid_results"], 3)
        self.assertEqual(quality["light_min_winner_votes"], 2)

    def test_fast_observation_can_finish_after_one_second(self) -> None:
        self.assertEqual(
            observation_action(1.0, True, False, minimum_s=1.0, primary_s=2.0, extension_s=0.5),
            "complete",
        )
        self.assertEqual(
            observation_action(2.49, False, True, minimum_s=1.0, primary_s=2.0, extension_s=0.5),
            "continue",
        )
        self.assertEqual(
            observation_action(2.5, False, True, minimum_s=1.0, primary_s=2.0, extension_s=0.5),
            "timeout",
        )

    def test_fast_meter_consensus_accepts_three_valid_results_with_two_inliers(self) -> None:
        values = [201.0, 202.0, 240.0]
        result = fuse_meter_readings(
            [meter_reading(value, index) for index, value in enumerate(values)],
            meter_type="voltage",
            minimum_valid=3,
            required_votes=2,
            max_spread=6.0,
        )
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["reading"]["value"], 201.5)
        self.assertEqual(result["consensus_count"], 2)

    def test_fast_digit_and_light_consensus_accept_three_observations(self) -> None:
        digit_result = temperature_digit_consensus(
            [
                {"valid": True, "upper_text": "137", "quality_score": 0.8,
                 "selected_candidate": {"center": [0.5, 0.3], "model_support": 1}},
                {"valid": True, "upper_text": "137", "quality_score": 0.8,
                 "selected_candidate": {"center": [0.5, 0.3], "model_support": 1}},
                {"valid": True, "upper_text": "137", "quality_score": 0.8,
                 "selected_candidate": {"center": [0.5, 0.3], "model_support": 1}},
            ],
            required_votes=2,
            minimum_observations=3,
            minimum_ratio=0.67,
        )
        self.assertEqual(digit_result["rows"], ["137"])
        self.assertTrue(digit_result["reliable"])
        light_result = light_consensus(
            [
                {"upper_status_lights:green": "on", "upper_status_lights:red": "off"},
                {"upper_status_lights:green": "on", "upper_status_lights:red": "off"},
                {"upper_status_lights:green": "on", "upper_status_lights:red": "off"},
            ],
            required_votes=2,
            minimum_observations=3,
            minimum_ratio=0.67,
            expected_positions={"upper_status_lights:green", "upper_status_lights:red"},
        )
        self.assertTrue(light_result["complete"])
        self.assertEqual(light_result["positions"]["upper_status_lights:green"], "on")

    def test_best_meter_frame_is_selected_from_consensus_inliers(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            candidates = []
            for index, score in enumerate([0.6, 0.7, 0.9, 1.0]):
                candidates.append(
                    Candidate(
                        Path(directory) / f"{index}.jpg",
                        [],
                        {"score": score},
                        {"_observation_index": index},
                    )
                )
            runner = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
            selected = runner.select_consensus_candidate(
                "voltage_meter",
                candidates,
                {"inlier_observation_indices": [0, 1, 2]},
            )
            self.assertEqual(selected.analysis["_observation_index"], 2)


if __name__ == "__main__":
    unittest.main()
