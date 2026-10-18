from __future__ import annotations

import concurrent.futures
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "ros2_tools"
METER_READER = ROOT / "meter_reader"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))
if str(METER_READER) not in sys.path:
    sys.path.insert(0, str(METER_READER))

from arm_kinematics import ArmConfig, ArmKinematics
from arm_apriltag_s0_search import alignment_consensus, build_min_positioning_segments
from camera_highres_capture import capture_frames
import cabinet_apriltag_inspection as inspection_module
import meter_pipeline
from cabinet_apriltag_inspection import (
    CAPTURE_STAGES,
    UPLOAD_WORKERS,
    WAYPOINT_SEQUENCE,
    CabinetAprilTagInspection,
    build_stage_upload_payload,
    changed_servo_targets,
    digit_consensus,
    digit_text_rows,
    extension_scale_for_distance,
    frame_quality,
    fuse_voltage_readings,
    inspection_extension_scale,
    light_consensus,
    light_observation,
    meter_snapshot,
    navigation_scan_context_matches,
    plan_retakes,
    resolve_waypoint_pose,
    resolve_waypoint_motion,
    s0_motion_duration_ms,
    select_stage_detections,
    stage_cabinet_update,
    template_score,
    wait_capture_helper,
    validate_inference_elapsed,
    validate_trajectory,
)


def det(label: str, box: list[int], confidence: float = 0.9) -> dict:
    return {"class_name": label, "box_xyxy": box, "confidence": confidence}


class CabinetInspectionLogicTest(unittest.TestCase):
    def test_closed_edge_rectification_counts_as_geometric_quality(self) -> None:
        score = template_score(
            {
                "rectify_method": "closed-edge-quad",
                "rectify_quality": {"area_ratio": 0.605115},
            }
        )
        self.assertAlmostEqual(score, 0.9076725)

    def test_closed_edge_voltage_frames_use_median_consensus(self) -> None:
        readings = [
            {
                "value": value,
                "pointer_angle": angle,
                "pointer_consensus_count": 2,
                "rectify_method": "closed-edge-quad",
                "quality_score": quality,
            }
            for value, angle, quality in ((216.0, 90.0, 3.0), (217.0, 90.2, 4.0), (218.0, 90.4, 3.5))
        ]
        fused = fuse_voltage_readings(readings, required_votes=3, max_spread=6.0)
        self.assertEqual(fused["mode"], "median_consensus")
        self.assertEqual(fused["reading"]["value"], 217.0)
        self.assertEqual(fused["consensus_count"], 3)

    def test_auto_meter_geometry_failure_returns_marked_resize_fallback(self) -> None:
        config = meter_pipeline.MeterConfig(
            min_value=0.0,
            max_value=450.0,
            center=(32.0, 48.0),
            zero_angle=130.0,
            full_angle=40.0,
            canonical_size=64,
            rectify_mode="auto",
        )
        image, method, quality = meter_pipeline.rectify_meter_crop_detailed(
            np.zeros((40, 50, 3), dtype=np.uint8),
            config,
        )
        self.assertIsNotNone(image)
        self.assertEqual(image.shape[:2], (64, 64))
        self.assertEqual(method, "resize_fallback:no_template")
        self.assertTrue(quality["geometry_fallback"])

    def test_meter_geometry_fallback_prefers_color_pointer(self) -> None:
        config = meter_pipeline.MeterConfig(
            min_value=0.0,
            max_value=450.0,
            center=(32.0, 48.0),
            zero_angle=130.0,
            full_angle=40.0,
            canonical_size=64,
            rectify_mode="auto",
        )
        candidates = [
            meter_pipeline.PointerCandidate((32, 10), 90.0, 30.0, "color-ray"),
            meter_pipeline.PointerCandidate((50, 32), 35.0, 80.0, "edge-hough"),
            meter_pipeline.PointerCandidate((51, 33), 35.5, 70.0, "gray-hough"),
        ]
        with mock.patch.object(
            meter_pipeline,
            "rectify_meter_crop_detailed",
            return_value=(
                np.zeros((64, 64, 3), dtype=np.uint8),
                "resize_fallback:geometry_failed",
                {"geometry_fallback": True},
            ),
        ), mock.patch.object(meter_pipeline, "find_pointer_candidates", return_value=candidates):
            result = meter_pipeline.read_meter_from_crop(
                np.zeros((40, 50, 3), dtype=np.uint8),
                config,
                det_conf=0.9,
            )
        self.assertEqual(result.status, "ok")
        self.assertEqual(result.pointer_method, "color-ray")
        self.assertAlmostEqual(result.pointer_angle, 90.0)

    def test_alignment_consensus_tolerates_isolated_detection_drop(self) -> None:
        self.assertTrue(alignment_consensus([True, False, True, False, True], 3))
        self.assertFalse(alignment_consensus([True, True, False, True, False], 3))
        self.assertFalse(alignment_consensus([False, True, False, True, True], 4))

    def test_patrol_scan_context_requires_exact_inactive_session(self) -> None:
        status = {
            "task_running": True,
            "goal_active": False,
            "session_id": "nav-session",
            "task_phase": "scanning",
            "state": "scanning",
            "active_inspection_id": "inspection-1",
            "current_index": 2,
            "current_apriltag_id": 11,
        }
        args = {
            "session_id": "nav-session",
            "inspection_id": "inspection-1",
            "waypoint_index": 2,
            "expected_tag_id": 11,
        }
        self.assertTrue(navigation_scan_context_matches(status, **args))
        for key, value in (
            ("task_running", False),
            ("goal_active", True),
            ("session_id", "other-session"),
            ("task_phase", "returning"),
            ("state", "navigating"),
            ("active_inspection_id", "inspection-2"),
            ("current_index", 3),
            ("current_apriltag_id", 10),
        ):
            changed = dict(status)
            changed[key] = value
            self.assertFalse(navigation_scan_context_matches(changed, **args), key)

    def test_manual_scan_cannot_bypass_missing_navigation_status(self) -> None:
        self.assertFalse(
            navigation_scan_context_matches(
                {},
                session_id="",
                inspection_id="manual-inspection",
                waypoint_index=-1,
                expected_tag_id=None,
            )
        )

    def test_inspection_fallback_reads_verified_patrol_status(self) -> None:
        status = {
            "task_running": True,
            "goal_active": False,
            "session_id": "nav-session",
            "task_phase": "scanning",
            "state": "scanning",
            "active_inspection_id": "inspection-1",
            "current_index": 2,
            "current_apriltag_id": 11,
        }
        with tempfile.TemporaryDirectory() as temporary:
            status_path = Path(temporary) / "status.json"
            status_path.write_text(json.dumps(status), encoding="utf-8")
            inspection = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
            inspection.navigation_metadata = {
                "navigation_session_id": "nav-session",
                "waypoint_index": 2,
            }
            inspection.inspection_id = "inspection-1"
            inspection.expected_tag_id = 11
            inspection.navigation_status_fallback = ""
            with mock.patch.object(inspection_module, "NAV_COMMAND_STATUS_PATH", status_path):
                self.assertTrue(inspection.allow_missing_navigation_status())
            self.assertEqual(inspection.navigation_status_fallback, "verified_patrol_scan_context")

    def test_highres_capture_refreshes_ros_after_blocking_io(self) -> None:
        tracker = {"stale": False, "refreshes": 0, "opens": 0}
        frame = np.zeros((6, 8, 3), dtype=np.uint8)

        class FakePreviewCamera:
            def close(self):
                return None

            def open(self):
                tracker["opens"] += 1
                tracker["stale"] = True

            def read(self):
                tracker["stale"] = True
                return frame.copy()

        with tempfile.TemporaryDirectory() as temp_dir:
            inspection = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
            inspection.profile = {
                "capture": {
                    "width": 8,
                    "height": 6,
                    "fps": 15,
                    "warmup_frames": 2,
                    "candidate_count": 3,
                    "jpeg_quality": 90,
                }
            }
            inspection.config = {
                "camera": {"device": "fake", "rotate_180": False, "warmup_frames": 2},
                "safety": {"f103_state_timeout_s": 1.2},
            }
            inspection.capture_dir = Path(temp_dir)
            inspection.camera = FakePreviewCamera()
            inspection.latest_frame = None
            inspection.stop_requested = False
            inspection.preview_camera_reopened = True

            def capture_highres_paths(stage, capture_index, output_dir):
                paths = []
                for index in range(3):
                    path = output_dir / f"{stage}_{capture_index:02d}_candidate_{index + 1}.jpg"
                    self.assertTrue(cv2.imwrite(str(path), frame))
                    paths.append(path)
                return paths

            def wait_state(_timeout_s, *, require_home_axes=False):
                self.assertFalse(require_home_axes)
                tracker["stale"] = False
                tracker["refreshes"] += 1

            def assert_safe_state(*, require_home_axes=False):
                self.assertFalse(require_home_axes)
                self.assertFalse(tracker["stale"])

            def infer_file(_path):
                tracker["stale"] = True
                return []

            inspection._capture_highres_paths = capture_highres_paths
            inspection.wait_state = wait_state
            inspection.assert_safe_state = assert_safe_state
            inspection.infer_file = infer_file

            candidates = inspection.highres_candidates("voltage_meter", 1)

            self.assertEqual(len(candidates), 3)
            self.assertFalse(tracker["stale"])
            self.assertEqual(tracker["opens"], 1)
            self.assertGreaterEqual(tracker["refreshes"], 6)

    def test_highres_helper_captures_expected_files(self) -> None:
        frame = np.zeros((6, 8, 3), dtype=np.uint8)

        class FakeCapture:
            def __init__(self):
                self.released = False

            def set(self, *_args):
                return True

            def isOpened(self):
                return True

            def read(self):
                return True, frame.copy()

            def release(self):
                self.released = True

        fake = FakeCapture()
        with tempfile.TemporaryDirectory() as temp_dir:
            paths = capture_frames(
                device="fake",
                output_dir=Path(temp_dir),
                prefix="voltage_01",
                width=8,
                height=6,
                fps=15,
                warmup_frames=2,
                candidate_count=3,
                jpeg_quality=90,
                rotate_180=False,
                capture_factory=lambda *_args: fake,
            )
            self.assertEqual(len(paths), 3)
            self.assertTrue(all(path.is_file() for path in paths))
            self.assertTrue(fake.released)

    def test_highres_helper_timeout_terminates_then_kills(self) -> None:
        class HungProcess:
            def __init__(self):
                self.returncode = None
                self.terminated = False
                self.killed = False

            def poll(self):
                return self.returncode

            def terminate(self):
                self.terminated = True

            def kill(self):
                self.killed = True
                self.returncode = -9

            def wait(self, timeout=None):
                if self.returncode is None:
                    raise inspection_module.subprocess.TimeoutExpired("helper", timeout)
                return self.returncode

        process = HungProcess()
        with self.assertRaisesRegex(inspection_module.SearchFailure, "timed out"):
            wait_capture_helper(process, 0.1, lambda: None)
        self.assertTrue(process.terminated)
        self.assertTrue(process.killed)

    def test_highres_failure_reopens_preview_before_propagating(self) -> None:
        frame = np.zeros((6, 8, 3), dtype=np.uint8)

        class FakePreviewCamera:
            def __init__(self):
                self.open_count = 0

            def close(self):
                return None

            def open(self):
                self.open_count += 1

            def read(self):
                return frame.copy()

        inspection = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
        inspection.profile = {"capture": {"width": 8, "height": 6}}
        inspection.config = {
            "camera": {"warmup_frames": 1},
            "safety": {"f103_state_timeout_s": 1.0},
        }
        inspection.capture_dir = Path(tempfile.mkdtemp())
        inspection.camera = FakePreviewCamera()
        inspection.stop_requested = False
        inspection.latest_frame = None
        inspection.preview_camera_reopened = True
        inspection.highres_capture_state = "idle"
        inspection.highres_capture_error = ""
        inspection.write_status = lambda *_args, **_kwargs: None
        inspection.wait_state = lambda *_args, **_kwargs: None
        inspection._capture_highres_paths = mock.Mock(
            side_effect=inspection_module.SearchFailure("simulated V4L2 timeout")
        )

        with self.assertRaisesRegex(inspection_module.SearchFailure, "simulated V4L2 timeout"):
            inspection.highres_candidates("voltage_meter", 1)
        self.assertEqual(inspection.camera.open_count, 1)
        self.assertTrue(inspection.preview_camera_reopened)

    def test_skip_tag_search_goes_directly_to_home_scan_pose(self) -> None:
        inspection = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
        inspection.tag_search_skipped = True
        inspection.args = mock.Mock(pose_test="")
        inspection.scan_state = "INIT"
        inspection.scan_stage = ""
        inspection.home_pwm = [1640, 1500, 1500, 2250, 1500, 1500]
        inspection.config = {
            "arm": {
                "position_tolerance_pwm": 20,
                "return_home_duration_ms": 2000,
                "return_home_timeout_s": 4.0,
            },
            "safety": {"arm_command_retry_count": 1},
        }
        inspection.aligned = False
        inspection.aligned_tag_distance_mm = 220.0
        inspection.aligned_tag_distance_spread_mm = 2.0
        inspection.establish_other_axes_home = mock.Mock()
        inspection.assert_safe_state = mock.Mock()
        inspection.current_s0_pwm = mock.Mock(return_value=1640)
        inspection.move_s0_and_wait = mock.Mock()
        inspection.write_status = mock.Mock()

        self.assertEqual(inspection.search(), 0)
        self.assertTrue(inspection.aligned)
        self.assertEqual(inspection.final_s0_pwm, 1640)
        self.assertIsNone(inspection.aligned_tag_distance_mm)
        inspection.move_s0_and_wait.assert_not_called()
        inspection.write_status.assert_called_with(
            "ALIGNED",
            direct_scan=True,
            tag_search_skipped=True,
            direct_scan_home_pose=[1640, 1500, 1500, 2250, 1500, 1500],
        )

    def test_capture_failure_returns_arm_home(self) -> None:
        class HealthyNode:
            last_state = {"estop": 0}

            @staticmethod
            def state_age():
                return 0.0

        inspection = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
        inspection.args = mock.Mock(pose_test="")
        inspection.inspection_id = "inspection-timeout"
        inspection.final_s0_pwm = 1640
        inspection.home_pwm = [1640, 1500, 1500, 2250, 1500, 1500]
        inspection.current_scan_pose = list(inspection.home_pwm)
        inspection.run_fixed_scan = mock.Mock(
            side_effect=inspection_module.SearchFailure("high-resolution capture timed out after 12.0s")
        )
        inspection.reason = ""
        inspection.scan_state = "INIT"
        inspection.node = HealthyNode()
        inspection.config = {"safety": {"f103_state_timeout_s": 1.2}}
        inspection.emergency_stop = mock.Mock()
        inspection.safe_return_aligned_home = mock.Mock()
        inspection.navigation_metadata = {}
        inspection.write_status = mock.Mock()
        inspection.upload_executor = None

        with self.assertRaisesRegex(inspection_module.SearchFailure, "timed out"):
            inspection.after_alignment()
        inspection.emergency_stop.assert_called()
        inspection.safe_return_aligned_home.assert_called_once()

    def test_changed_targets_never_resend_s4_s5(self) -> None:
        start = [1640, 1500, 1500, 2250, 1500, 1500]
        middle = [1640, 1290, 1240, 2250, 1500, 1500]
        self.assertEqual(changed_servo_targets(start, middle), [(1, 1290), (2, 1240)])

    def test_waypoint_interpolation_keeps_s4_s5_fixed(self) -> None:
        waypoint = {
            "s0_offset_pwm": 100,
            "retracted_pose": [1640, 1500, 1500, 2350, 1500, 1500],
            "extended_pose": [1640, 1592, 1552, 2150, 1500, 1500],
        }
        self.assertEqual(
            resolve_waypoint_pose(waypoint, 1680, 0.5),
            [1780, 1546, 1526, 2250, 1500, 1500],
        )

    def test_trajectory_accepts_fixed_scan_and_rejects_s4(self) -> None:
        limits = [[500, 2500], [1000, 1600], [900, 1600], [2050, 2400], [1500, 1500], [1500, 1500]]
        kinematics = ArmKinematics(ArmConfig())
        validate_trajectory(
            [1640, 1500, 1500, 2250, 1500, 1500],
            [1640, 1290, 1240, 2250, 1500, 1500],
            limits,
            1640,
            250,
            kinematics,
        )
        with self.assertRaisesRegex(ValueError, "S4"):
            validate_trajectory(
                [1640, 1500, 1500, 2250, 1500, 1500],
                [1640, 1290, 1240, 2250, 1510, 1500],
                limits,
                1640,
                250,
                kinematics,
            )

    def test_digit_rows_and_two_of_three_consensus(self) -> None:
        detections = [
            det("5", [10, 10, 20, 30]),
            det("7", [25, 11, 35, 31]),
            det("4", [10, 60, 20, 80]),
            det("7", [25, 61, 35, 81]),
        ]
        self.assertEqual(digit_text_rows(detections), ["57", "47"])
        result = digit_consensus([["57", "47"], ["57", "47"], ["51", "49"]])
        self.assertTrue(result["complete"])
        self.assertEqual(result["rows"], ["57", "47"])

    def test_temperature_analysis_uses_digit_yolo_only(self) -> None:
        detections = [
            det("2", [10, 10, 20, 30], confidence=0.92),
            det("6", [25, 11, 35, 31], confidence=0.88),
        ]
        result = CabinetAprilTagInspection.analyze_temperature_frame(
            np.zeros((80, 120, 3), dtype=np.uint8),
            detections,
            observation_index=4,
        )
        self.assertTrue(result["valid"])
        self.assertEqual(result["upper_text"], "26")
        self.assertEqual(result["method"], "digit_yolo_incremental")
        self.assertEqual(result["verification_source"], "digit_yolo_incremental")
        self.assertNotIn("seven_segment_text", result)

    def test_temperature_stage_routes_only_to_digit_model(self) -> None:
        inspection = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
        inspection.light_model_paths = [Path("lightlabel_best.onnx")]
        inspection.meter_model_paths = [Path("meter_digit_unified_best.onnx")]
        inspection.digit_model_paths = [Path("digits_best.onnx")]
        self.assertEqual(
            inspection.model_paths_for_stage("temperature_controller"),
            [Path("digits_best.onnx")],
        )
        self.assertEqual(
            inspection.model_paths_for_stage("voltage_meter"),
            [Path("meter_digit_unified_best.onnx")],
        )

    def test_temperature_result_preserves_yolo_boxes(self) -> None:
        detections = [det("2", [10, 10, 20, 30]), det("6", [25, 10, 35, 30])]
        result = {"digit_detections": detections}
        reading = {"upper_text": "26", "source": "multi_frame_yolo_consensus", "consensus_count": 3}
        CabinetAprilTagInspection.apply_temperature_result(result, reading)
        self.assertEqual(result["digit_detections"], detections)
        self.assertEqual(result["yolo_digit_detections"], detections)
        self.assertEqual(result["digit_readings"]["global_text"], "26")
        self.assertEqual(result["digit_readings"]["rows"][0]["source"], "multi_frame_yolo_consensus")

    def test_fixed_scan_profile_loads_dedicated_digit_model(self) -> None:
        profile = yaml.safe_load(
            (ROOT / "cabinet_inspection_profiles" / "fixed_scan.yaml").read_text(encoding="utf-8")
        )
        self.assertIn("yolo_models/digits_best.onnx", profile["recognition"]["models"])
        self.assertEqual(profile["recognition"]["digit_conf"], 0.15)

    def test_light_consensus_requires_four_spatial_color_positions(self) -> None:
        middle = [det("green_on", [10, 10, 30, 30]), det("red_off", [60, 10, 80, 30])]
        lower = [det("green_off", [10, 10, 30, 30]), det("red_on", [60, 10, 80, 30])]
        observations = [
            light_observation("upper_status_lights", middle),
            light_observation("upper_status_lights", middle),
            light_observation("lower_status_lights", lower),
            light_observation("lower_status_lights", lower),
        ]
        result = light_consensus(observations)
        self.assertTrue(result["complete"])
        self.assertEqual(result["positions"]["upper_status_lights:red"], "off")
        self.assertEqual(result["positions"]["lower_status_lights:green"], "off")

    def test_meter_instability_is_not_averaged(self) -> None:
        first = {
            "meter_readings": [
                {"meter_type": "voltage", "value": 220.0, "rectify_method": "template-ecc:0.60"},
                {"meter_type": "current", "value": 1.00, "rectify_method": "template-ecc:0.70"},
            ]
        }
        second = {
            "meter_readings": [
                {"meter_type": "voltage", "value": 235.0, "rectify_method": "template-ecc:0.80"},
                {"meter_type": "current", "value": 1.02, "rectify_method": "template-ecc:0.65"},
            ]
        }
        result = meter_snapshot([first, second])
        self.assertEqual(result["voltage"]["status"], "unstable")
        self.assertEqual(result["voltage"]["value"], 235.0)
        self.assertEqual(result["current"]["status"], "ok")

    def test_retakes_are_capped_at_two(self) -> None:
        meters = {"voltage": {"status": "missing"}, "current": {"status": "missing"}}
        digits = {"complete": False}
        lights = {
            "missing": [
                "upper_status_lights:green",
                "upper_status_lights:red",
                "lower_status_lights:green",
                "lower_status_lights:red",
            ]
        }
        self.assertEqual(plan_retakes(meters, digits, lights, 2), ["voltage_meter", "current_meter"])

    def test_min_positioning_is_monotonic_and_finishes_at_500(self) -> None:
        config = {
            "arm": {
                "min_pwm": 500,
                "max_pwm": 2500,
                "move_to_min_duration_ms": 3000,
                "move_to_min_timeout_s": 5.5,
                "move_to_min_waypoints": [
                    {"target_pwm": 1200, "duration_ms": 1400, "timeout_s": 3.0},
                    {"target_pwm": 800, "duration_ms": 1100, "timeout_s": 2.7},
                    {"target_pwm": 500, "duration_ms": 1000, "timeout_s": 2.6},
                ],
            }
        }
        segments = build_min_positioning_segments(config)
        self.assertEqual([item.target_pwm for item in segments], [1200, 800, 500])

    def test_extension_scale_uses_distance_and_spread_guards(self) -> None:
        config = {"disable_below_mm": 200.0, "full_at_mm": 220.0, "max_distance_spread_mm": 20.0}
        self.assertEqual(extension_scale_for_distance(190.0, 2.0, config), (0.0, "cabinet_too_close"))
        self.assertEqual(extension_scale_for_distance(210.0, 2.0, config), (0.5, "scaled_extension"))
        self.assertEqual(extension_scale_for_distance(230.0, 2.0, config), (1.0, "full_extension"))
        self.assertEqual(extension_scale_for_distance(230.0, 25.0, config), (0.0, "tag_distance_unstable"))

    def test_direct_scan_keeps_previous_full_extension_motion(self) -> None:
        config = {
            "disable_below_mm": 200.0,
            "full_at_mm": 220.0,
            "max_distance_spread_mm": 20.0,
            "direct_scan_scale": 1.0,
        }
        self.assertEqual(
            inspection_extension_scale(None, None, config, tag_search_skipped=True),
            (1.0, "tag_search_skipped_configured_extension"),
        )

    def test_fixed_scan_accepts_observed_two_model_inference_latency(self) -> None:
        self.assertEqual(validate_inference_elapsed(0.855, {"max_inference_elapsed_s": 1.5}), 1.5)
        with self.assertRaisesRegex(inspection_module.SearchFailure, "inference elapsed"):
            validate_inference_elapsed(1.501, {"max_inference_elapsed_s": 1.5})

    def test_partition_sequence_has_five_captures_and_one_final_home(self) -> None:
        self.assertEqual(
            WAYPOINT_SEQUENCE,
            (
                "top_approach",
                "voltage_meter",
                "current_meter",
                "temperature_controller",
                "middle_transit",
                "upper_status_lights",
                "lower_status_lights",
            ),
        )
        self.assertEqual(len(CAPTURE_STAGES), 5)
        self.assertNotIn("home", WAYPOINT_SEQUENCE)

    def test_middle_transition_moves_all_three_pitch_joints_each_step(self) -> None:
        profile = yaml.safe_load((ROOT / "cabinet_inspection_profiles" / "fixed_scan.yaml").read_text(encoding="utf-8"))
        waypoint = next(item for item in profile["scan"]["waypoints"] if item["name"] == "middle_transit")
        motion = resolve_waypoint_motion(waypoint, 1720, 1.0, 1400)
        previous = [1560, 1592, 1552, 2150, 1500, 1500]
        self.assertEqual(len(motion), 3)
        for pose, duration_ms in motion:
            self.assertEqual(pose[0], 1560)
            self.assertEqual(pose[4:], [1500, 1500])
            self.assertEqual(duration_ms, 700)
            self.assertTrue(all(abs(pose[index] - previous[index]) >= 1 for index in (1, 2, 3)))
            previous = pose
        self.assertEqual(previous, [1560, 1537, 1401, 2193, 1500, 1500])

    def test_extended_middle_transition_descends_nearly_vertically(self) -> None:
        profile = yaml.safe_load((ROOT / "cabinet_inspection_profiles" / "fixed_scan.yaml").read_text(encoding="utf-8"))
        waypoint = next(item for item in profile["scan"]["waypoints"] if item["name"] == "middle_transit")
        kinematics = ArmKinematics(ArmConfig.from_dict(profile["kinematics"]))
        start = [1560, 1592, 1552, 2150, 1500, 1500]
        start_pose, _joints, _transform = kinematics.pose_from_pwm(start)
        motion = resolve_waypoint_motion(waypoint, 1720, 1.0, 1400)
        poses = [kinematics.pose_from_pwm(pose)[0] for pose, _duration_ms in motion]
        self.assertTrue(all(poses[index].z_mm < (start_pose if index == 0 else poses[index - 1]).z_mm for index in range(len(poses))))
        self.assertLess(abs(poses[-1].x_mm - start_pose.x_mm), 1.0)
        self.assertLess(abs(poses[-1].y_mm - start_pose.y_mm), 6.0)
        self.assertGreater(start_pose.z_mm - poses[-1].z_mm, 50.0)

    def test_profile_trajectory_is_valid_at_zero_and_full_extension(self) -> None:
        profile = yaml.safe_load((ROOT / "cabinet_inspection_profiles" / "fixed_scan.yaml").read_text(encoding="utf-8"))
        limits = profile["scan"]["servo_limits"]
        kinematics = ArmKinematics(ArmConfig.from_dict(profile["kinematics"]))
        for aligned_s0 in (1640, 1680):
            for scale in (0.0, 1.0):
                current = [aligned_s0, 1500, 1500, 2250, 1500, 1500]
                for waypoint in profile["scan"]["waypoints"]:
                    for target, _duration_ms in resolve_waypoint_motion(waypoint, aligned_s0, scale, 1400):
                        validate_trajectory(current, target, limits, aligned_s0, 250, kinematics)
                        current = target
                validate_trajectory(
                    current,
                    [aligned_s0, 1500, 1500, 2250, 1500, 1500],
                    limits,
                    aligned_s0,
                    250,
                    kinematics,
                )

    def test_stage_selection_targets_only_requested_partition(self) -> None:
        detections = [
            det("yakuang", [10, 10, 30, 30]),
            det("liukuang", [40, 10, 60, 30]),
            det("2", [80, 10, 90, 30]),
            det("green_on", [10, 80, 30, 100]),
            det("red_off", [60, 80, 80, 100]),
        ]
        self.assertEqual(len(select_stage_detections("voltage_meter", detections)), 1)
        self.assertEqual(len(select_stage_detections("current_meter", detections)), 1)
        self.assertEqual(len(select_stage_detections("temperature_controller", detections)), 1)
        self.assertEqual(len(select_stage_detections("upper_status_lights", detections)), 2)

    def test_stage_upload_filters_unrelated_meter_and_keeps_raw_voltage(self) -> None:
        source = {
            "meter_readings": [
                {"meter_type": "voltage", "value": 236.7, "cabinet_metric": {"value": 236.7, "unit": "V"}},
                {"meter_type": "current", "value": 0.42, "cabinet_metric": {"value": 0.42, "unit": "A"}},
            ],
            "digit_readings": {"global_text": "88"},
            "digit_detections": [{"class_name": "8"}],
        }
        payload = build_stage_upload_payload(
            source,
            inspection_id="insp-test",
            stage="voltage_meter",
            stage_index=1,
            profile="fixed_scan",
            started_at="2026-07-27T00:00:00Z",
            board_id="board",
            board_label="Board",
            coverage={},
        )
        self.assertEqual([item["meter_type"] for item in payload["meter_readings"]], ["voltage"])
        self.assertEqual(payload["cabinet_data"]["voltage"]["raw_value"], 236.7)
        self.assertNotIn("current", payload["cabinet_data"])
        self.assertEqual(payload["capture_key"], "insp-test:voltage_meter")
        self.assertEqual(payload["inspection_stage_count"], 5)
        self.assertEqual(UPLOAD_WORKERS, 2)

    def test_light_stage_update_contains_only_requested_row(self) -> None:
        coverage = {
            "status_lights": {
                "positions": {
                    "upper_status_lights:green": "off",
                    "upper_status_lights:red": "on",
                    "lower_status_lights:green": "on",
                    "lower_status_lights:red": "off",
                }
            }
        }
        update = stage_cabinet_update("lower_status_lights", {}, coverage)
        lights = update["status_lights"]["lights"]
        self.assertEqual(len(lights), 2)
        self.assertTrue(all(item["row"] == "lower" for item in lights))

    def test_full_resolution_uploads_use_two_worker_pipeline_without_reencoding(self) -> None:
        tracker = {"active": 0, "max_active": 0, "count": 0}
        tracker_lock = threading.Lock()
        raw_bytes = b"R" * 128
        annotated_bytes = b"A" * 256

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def do_POST(self):
                length = int(self.headers.get("Content-Length", "0"))
                self.rfile.read(length)
                with tracker_lock:
                    tracker["active"] += 1
                    tracker["max_active"] = max(tracker["max_active"], tracker["active"])
                    tracker["count"] += 1
                    item_id = tracker["count"]
                time.sleep(0.10)
                with tracker_lock:
                    tracker["active"] -= 1
                body = json.dumps(
                    {
                        "recognition": {
                            "id": f"rec-{item_id}",
                            "raw_image": {"size_bytes": len(raw_bytes)},
                            "annotated_image": {"size_bytes": len(annotated_bytes)},
                            "timings_ms": {"server_save": 1.0},
                        }
                    }
                ).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Connection", "close")
                self.end_headers()
                self.wfile.write(body)
                self.close_connection = True

            def log_message(self, _format, *_args):
                return

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with tempfile.TemporaryDirectory() as temp_dir:
                inspection = CabinetAprilTagInspection.__new__(CabinetAprilTagInspection)
                inspection.server_url = f"http://127.0.0.1:{server.server_port}"
                inspection.upload_token = "token"
                inspection.inspection_id = "insp-upload"
                inspection.inspection_started_at = "2026-07-27T00:00:00Z"
                inspection.board_id = "board"
                inspection.board_label = "Board"
                inspection.profile = {"profile_name": "fixed_scan"}
                inspection.profile_path = Path(temp_dir) / "fixed_scan.yaml"
                inspection._upload_thread_state = threading.local()
                records = []
                original_hashes = []
                for index, stage in enumerate(CAPTURE_STAGES, start=1):
                    raw_path = Path(temp_dir) / f"{stage}_raw.jpg"
                    annotated_path = Path(temp_dir) / f"{stage}_det.jpg"
                    json_path = Path(temp_dir) / f"{stage}.json"
                    raw_path.write_bytes(raw_bytes)
                    annotated_path.write_bytes(annotated_bytes)
                    json_path.write_text(json.dumps({"meter_readings": []}), encoding="utf-8")
                    records.append(
                        {
                            "stage": stage,
                            "upload_enqueued_at": time.monotonic(),
                            "paths": {
                                "raw": str(raw_path),
                                "annotated": str(annotated_path),
                                "json": str(json_path),
                            },
                        }
                    )
                    original_hashes.append((hashlib.sha256(raw_bytes).hexdigest(), hashlib.sha256(annotated_bytes).hexdigest()))

                started = time.monotonic()
                with concurrent.futures.ThreadPoolExecutor(max_workers=UPLOAD_WORKERS) as executor:
                    futures = [
                        executor.submit(inspection.upload_recognition, record, {}, index)
                        for index, record in enumerate(records, start=1)
                    ]
                    results = [future.result() for future in futures]
                elapsed = time.monotonic() - started
                self.assertEqual(tracker["max_active"], 2)
                self.assertLess(elapsed, 0.42)
                self.assertEqual(len(results), 5)
                for record, expected in zip(records, original_hashes):
                    self.assertEqual(hashlib.sha256(Path(record["paths"]["raw"]).read_bytes()).hexdigest(), expected[0])
                    self.assertEqual(hashlib.sha256(Path(record["paths"]["annotated"]).read_bytes()).hexdigest(), expected[1])
        finally:
            server.shutdown()
            server.server_close()

    def test_s0_duration_is_clamped_to_configured_range(self) -> None:
        config = {
            "s0_duration_per_pwm_ms": 4.0,
            "s0_min_duration_ms": 500,
            "s0_max_duration_ms": 1000,
        }
        self.assertEqual(s0_motion_duration_ms(1640, 1660, config), 500)
        self.assertEqual(s0_motion_duration_ms(1640, 1840, config), 800)
        self.assertEqual(s0_motion_duration_ms(1640, 2040, config), 1000)

    def test_quality_prefers_sharp_confident_centered_frame(self) -> None:
        sharp = np.zeros((200, 300, 3), dtype=np.uint8)
        for index in range(0, 300, 10):
            cv2.line(sharp, (index, 0), (index, 199), (200, 200, 200), 2)
        blurred = cv2.GaussianBlur(sharp, (31, 31), 0)
        detections = [det("green_on", [120, 70, 180, 130], 0.95)]
        self.assertGreater(frame_quality(sharp, detections)["score"], frame_quality(blurred, detections)["score"])


if __name__ == "__main__":
    unittest.main()
