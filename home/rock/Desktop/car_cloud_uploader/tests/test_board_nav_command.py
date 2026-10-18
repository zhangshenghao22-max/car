#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import unittest
from types import SimpleNamespace
from unittest import mock


MODULE_PATH = pathlib.Path(__file__).parents[1] / "board_nav_command.py"
SPEC = importlib.util.spec_from_file_location("board_nav_command", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class FakeNode:
    def __init__(self, outcomes, cleanups=None, arrival_poses=None):
        self.outcomes = list(outcomes)
        self.cleanups = list(cleanups or [])
        self.arrival_poses = list(
            arrival_poses or [{"x": 1.0, "y": 2.0, "yaw": 0.0}]
        )
        self.send_count = 0
        self.stop_count = 0
        self.cleanup_count = 0
        self.latest_map_pose = None
        self.last_arrival_confirmation = {}

    def send_goal(self, *_args, **_kwargs):
        self.send_count += 1
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, BaseException):
            raise outcome
        return outcome

    def publish_stop(self, _executor):
        self.stop_count += 1

    def cleanup_navigation_children(self, _executor, timeout_sec=3.0):
        self.cleanup_count += 1
        if self.cleanups:
            return self.cleanups.pop(0)
        return {"confirmed": True, "elapsed_sec": 0.1}

    def stabilizer_snapshot(self):
        return {"state": "TRACK"}

    def fresh_tf_map_pose(self, _max_age_sec):
        index = min(max(self.send_count - 1, 0), len(self.arrival_poses) - 1)
        return dict(self.arrival_poses[index])


class FakeExecutor:
    @staticmethod
    def spin_once(timeout_sec=0.1):
        del timeout_sec


class NavigationRetryTest(unittest.TestCase):
    def setUp(self):
        self.point = {"x": 1.0, "y": 2.0, "yaw": 0.0, "label": "C2"}
        self.patches = (
            mock.patch.object(MODULE, "merge_status", return_value={}),
            mock.patch.object(MODULE, "append_navigation_attempt"),
            mock.patch.object(MODULE, "wait_with_spin"),
            mock.patch.object(MODULE, "GOAL_ARRIVAL_CONFIRM_TIMEOUT_SEC", 0.01),
        )
        for patcher in self.patches:
            patcher.start()

    def tearDown(self):
        for patcher in reversed(self.patches):
            patcher.stop()

    def run_retries(self, node):
        return MODULE.navigate_with_retries(
            node,
            FakeExecutor(),
            {"stop": False},
            self.point,
            session_id="nav-test",
            point_index=1,
            point_count=3,
        )

    def test_retry_waits_for_cleanup_then_can_succeed(self):
        node = FakeNode(
            [
                {"kind": "result", "status": MODULE.GoalStatus.STATUS_ABORTED},
                {"kind": "result", "status": MODULE.GoalStatus.STATUS_SUCCEEDED},
            ]
        )

        reached, attempts, status = self.run_retries(node)

        self.assertTrue(reached)
        self.assertEqual(attempts, 2)
        self.assertEqual(status, MODULE.GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(node.send_count, 2)
        self.assertEqual(node.cleanup_count, 1)
        MODULE.wait_with_spin.assert_called_once()

    def test_cleanup_failure_prevents_the_next_goal(self):
        node = FakeNode(
            [{"kind": "result", "status": MODULE.GoalStatus.STATUS_ABORTED}],
            cleanups=[
                {
                    "confirmed": False,
                    "active_after": [
                        {
                            "action": "/follow_path",
                            "goal_id": "old-goal",
                            "status": MODULE.GoalStatus.STATUS_EXECUTING,
                        }
                    ],
                }
            ],
        )

        with self.assertRaises(MODULE.NavigationCleanupError):
            self.run_retries(node)

        self.assertEqual(node.send_count, 1)
        self.assertEqual(node.cleanup_count, 1)
        MODULE.wait_with_spin.assert_not_called()

    def test_cancel_records_cleanup_and_does_not_retry(self):
        node = FakeNode([KeyboardInterrupt()])

        with self.assertRaises(KeyboardInterrupt):
            self.run_retries(node)

        self.assertEqual(node.send_count, 1)
        self.assertEqual(node.cleanup_count, 1)
        MODULE.wait_with_spin.assert_not_called()

    def test_confirmed_parent_stall_is_cleaned_before_retry(self):
        cleanup = {
            "confirmed": True,
            "parent_cancel_confirmed": True,
            "stall_reason": "no child action",
        }
        node = FakeNode(
            [
                MODULE.NavigationStallError("no child action", cleanup),
                {"kind": "result", "status": MODULE.GoalStatus.STATUS_SUCCEEDED},
            ]
        )

        reached, attempts, status = self.run_retries(node)

        self.assertTrue(reached)
        self.assertEqual(attempts, 2)
        self.assertEqual(status, MODULE.GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(node.cleanup_count, 0)
        MODULE.wait_with_spin.assert_called_once()

    def test_unconfirmed_parent_stall_prevents_retry(self):
        cleanup = {
            "confirmed": True,
            "parent_cancel_confirmed": False,
            "stall_reason": "no child action",
        }
        node = FakeNode([MODULE.NavigationStallError("no child action", cleanup)])

        with self.assertRaises(MODULE.NavigationCleanupError):
            self.run_retries(node)

        self.assertEqual(node.send_count, 1)
        self.assertEqual(node.cleanup_count, 0)
        MODULE.wait_with_spin.assert_not_called()

    def test_rejects_false_success_until_pose_reaches_goal(self):
        node = FakeNode(
            [
                {"kind": "result", "status": MODULE.GoalStatus.STATUS_SUCCEEDED},
                {"kind": "result", "status": MODULE.GoalStatus.STATUS_SUCCEEDED},
            ],
            arrival_poses=[
                {"x": -1.0, "y": -1.0, "yaw": 0.8},
                {"x": 1.0, "y": 2.0, "yaw": 0.0},
            ],
        )

        reached, attempts, status = self.run_retries(node)

        self.assertTrue(reached)
        self.assertEqual(attempts, 2)
        self.assertEqual(status, MODULE.GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(node.send_count, 2)
        self.assertEqual(node.cleanup_count, 1)
        first_attempt = MODULE.append_navigation_attempt.call_args_list[0].args[0]
        self.assertTrue(first_attempt["reported_success_rejected"])
        self.assertFalse(first_attempt["arrival_confirmation"]["confirmed"])

    def test_rejects_success_when_heading_is_not_aligned(self):
        node = FakeNode(
            [
                {"kind": "result", "status": MODULE.GoalStatus.STATUS_SUCCEEDED},
                {"kind": "result", "status": MODULE.GoalStatus.STATUS_SUCCEEDED},
                {"kind": "result", "status": MODULE.GoalStatus.STATUS_SUCCEEDED},
            ],
            arrival_poses=[{"x": 1.0, "y": 2.0, "yaw": 0.3}],
        )

        reached, attempts, status = self.run_retries(node)

        self.assertFalse(reached)
        self.assertEqual(attempts, 3)
        self.assertEqual(status, MODULE.GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(node.send_count, 3)
        self.assertEqual(node.cleanup_count, 3)


class NavigationStallDetectionTest(unittest.TestCase):
    def valid_stabilizer(self):
        return {
            "status_age_sec": 0.1,
            "raw_cmd_age": 20.0,
            "goal_id": "goal-1",
            "action_active": True,
            "awaiting_goal_path": True,
            "path_goal_id": None,
        }

    def reason(self, **overrides):
        values = {
            "elapsed_sec": 12.0,
            "goal_id": "goal-1",
            "stabilizer": self.valid_stabilizer(),
            "active_children": [],
            "child_statuses_fresh": True,
        }
        values.update(overrides)
        return MODULE.navigation_stall_reason(**values)

    def test_detects_active_parent_without_path_or_child(self):
        self.assertIn("no child action", self.reason())

    def test_allows_initial_planning_grace_period(self):
        self.assertEqual(self.reason(elapsed_sec=9.9), "")

    def test_does_not_interrupt_an_active_child(self):
        child = {"action": "/compute_path_to_pose", "status": 2}
        self.assertEqual(self.reason(active_children=[child]), "")

    def test_requires_fresh_action_status(self):
        self.assertEqual(self.reason(child_statuses_fresh=False), "")

    def test_requires_the_current_goal(self):
        stabilizer = self.valid_stabilizer()
        stabilizer["goal_id"] = "older-goal"
        self.assertEqual(self.reason(stabilizer=stabilizer), "")


class ReturnPoseTest(unittest.TestCase):
    @staticmethod
    def transform(*, stamp_sec=100, stamp_nanosec=0, x=1.2, y=-0.4, yaw=0.5):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=stamp_sec, nanosec=stamp_nanosec)
            ),
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=x, y=y, z=0.0),
                rotation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=MODULE.math.sin(yaw / 2.0),
                    w=MODULE.math.cos(yaw / 2.0),
                ),
            ),
        )

    def test_accepts_fresh_map_to_base_transform(self):
        pose = MODULE.map_pose_from_transform(
            self.transform(),
            now_sec=100.5,
            max_age_sec=1.0,
        )

        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose["x"], 1.2)
        self.assertAlmostEqual(pose["y"], -0.4)
        self.assertAlmostEqual(pose["yaw"], 0.5)

    def test_rejects_stale_map_to_base_transform(self):
        pose = MODULE.map_pose_from_transform(
            self.transform(),
            now_sec=101.1,
            max_age_sec=1.0,
        )

        self.assertIsNone(pose)

    def test_capture_uses_tf_when_stationary_amcl_does_not_republish(self):
        class Node:
            latest_map_pose = None
            return_pose_source = ""

            @staticmethod
            def fresh_tf_map_pose(_max_age_sec):
                return {"x": 1.8, "y": -2.3, "yaw": 0.9}

        class Executor:
            @staticmethod
            def spin_once(timeout_sec=0.1):
                del timeout_sec

        node = Node()
        pose = MODULE.capture_fresh_return_pose(
            node,
            Executor(),
            {"stop": False},
            timeout_s=1.0,
        )

        self.assertEqual(pose["label"], "S")
        self.assertEqual(pose["kind"], "transit")
        self.assertAlmostEqual(pose["x"], 1.8)
        self.assertEqual(node.return_pose_source, "tf_map_base_link")

    def test_goal_yaw_error_wraps_across_pi(self):
        errors = MODULE.goal_pose_errors(
            {"x": 0.0, "y": 0.0, "yaw": 3.13},
            {"x": 0.0, "y": 0.0, "yaw": -3.13},
        )

        self.assertLess(errors["yaw_error_rad"], 0.03)


if __name__ == "__main__":
    unittest.main()
