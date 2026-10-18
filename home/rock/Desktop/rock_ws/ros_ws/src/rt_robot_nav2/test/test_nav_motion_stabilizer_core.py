#!/usr/bin/env python3
import importlib.util
import math
import pathlib
import sys
import unittest


MODULE_PATH = (
    pathlib.Path(__file__).parents[1] / "scripts" / "nav_motion_stabilizer_core.py"
)
SPEC = importlib.util.spec_from_file_location("nav_motion_stabilizer_core", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class NavMotionStabilizerCoreTest(unittest.TestCase):
    def test_normalize_angle_wraps_across_pi(self):
        error = MODULE.normalize_angle(math.radians(-179) - math.radians(179))
        self.assertAlmostEqual(error, math.radians(2), places=6)

    def test_path_target_uses_forward_lookahead(self):
        points = [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.3, 0.0, 0.0)]
        heading, goal_distance, remaining = MODULE.path_target(
            points, 0.0, 0.0, 0.25
        )
        self.assertAlmostEqual(heading, 0.0)
        self.assertAlmostEqual(goal_distance, 0.3)
        self.assertAlmostEqual(remaining, 0.3)

    def test_turn_target_is_frozen_and_direction_is_locked(self):
        controller = MODULE.TurnController()
        self.assertTrue(controller.begin("path", 0.0, math.pi / 2.0, 0.0))
        target = controller.target_yaw
        decision = controller.update(math.radians(10), 0.1, 0.5)
        self.assertEqual(controller.target_yaw, target)
        self.assertGreater(decision.angular_z, 0.0)
        self.assertEqual(controller.direction, 1)

    def test_crossing_target_stops_without_reverse(self):
        controller = MODULE.TurnController()
        controller.begin("path", 0.0, math.radians(30), 0.0)
        decision = controller.update(math.radians(31), 0.05, 1.0)
        self.assertEqual(decision.state, MODULE.SETTLE)
        self.assertEqual(decision.angular_z, 0.0)

    def test_goal_uses_five_degree_tolerance(self):
        controller = MODULE.TurnController()
        controller.begin("goal", 0.0, math.radians(20), 0.0)
        decision = controller.update(math.radians(16), 0.02, 1.0)
        self.assertEqual(decision.state, MODULE.SETTLE)

    def test_opposite_rotation_stops_and_recovers_without_latching_fault(self):
        controller = MODULE.TurnController()
        controller.begin("path", 0.0, math.radians(90), 0.0)
        decision = controller.update(math.radians(-6), -0.1, 0.5)
        self.assertFalse(decision.faulted)
        self.assertEqual(decision.state, MODULE.SETTLE)
        self.assertEqual(decision.reason, "opposite_rotation_stop")

        decision = controller.update(math.radians(-6), 0.0, 0.8)
        self.assertFalse(decision.completed)
        decision = controller.update(math.radians(-6), 0.0, 1.01)
        self.assertTrue(decision.completed)
        self.assertEqual(decision.reason, "opposite_rotation_recovered")
        self.assertEqual(decision.state, MODULE.TRACK)

    def test_no_progress_stops_for_recovery_after_six_seconds(self):
        controller = MODULE.TurnController()
        controller.begin("path", 0.0, math.radians(90), 0.0)
        decision = controller.update(0.0, 0.0, 6.1)
        self.assertFalse(decision.faulted)
        self.assertEqual(decision.state, MODULE.SETTLE)
        self.assertEqual(decision.reason, "no_heading_progress_stop")

    def test_data_pause_does_not_consume_progress_timeout(self):
        controller = MODULE.TurnController()
        controller.begin("path", 0.0, math.radians(90), 0.0)
        controller.pause_timers(1.0)

        decision = controller.update(0.0, 0.0, 5.1)
        self.assertFalse(decision.faulted)
        decision = controller.update(0.0, 0.0, 10.0)
        self.assertFalse(decision.faulted)
        decision = controller.update(0.0, 0.0, 10.2)
        self.assertEqual(decision.state, MODULE.SETTLE)
        self.assertEqual(decision.reason, "no_heading_progress_stop")

    def test_settle_requires_low_speed_window(self):
        controller = MODULE.TurnController()
        controller.begin("path", 0.0, math.radians(30), 0.0)
        controller.update(math.radians(30), 0.02, 1.0)
        decision = controller.update(math.radians(30), 0.02, 1.21)
        self.assertTrue(decision.completed)
        self.assertEqual(decision.state, MODULE.TRACK)

    def test_settle_timeout_completes_without_latching_fault(self):
        controller = MODULE.TurnController()
        controller.begin("path", 0.0, math.radians(30), 0.0)
        controller.update(math.radians(30), 0.10, 1.0)

        decision = controller.update(math.radians(30), 0.10, 2.01)

        self.assertTrue(decision.completed)
        self.assertFalse(decision.faulted)
        self.assertEqual(decision.reason, "settle_forced_complete")
        self.assertEqual(decision.state, MODULE.TRACK)

    def test_data_pause_does_not_consume_settle_timeout(self):
        controller = MODULE.TurnController()
        controller.begin("path", 0.0, math.radians(30), 0.0)
        controller.update(math.radians(30), 0.10, 1.0)
        controller.pause_timers(1.1)

        decision = controller.update(math.radians(30), 0.02, 4.1)
        self.assertFalse(decision.faulted)
        decision = controller.update(math.radians(30), 0.02, 4.31)
        self.assertTrue(decision.completed)
        self.assertEqual(decision.state, MODULE.TRACK)

    def test_goal_turn_can_reenter_to_correct_overshoot(self):
        controller = MODULE.TurnController()
        controller.begin("goal", 0.0, math.radians(20), 0.0)
        controller.update(math.radians(27), 0.0, 1.0)
        decision = controller.update(math.radians(27), 0.0, 1.21)
        self.assertTrue(decision.completed)

        self.assertTrue(
            controller.begin("goal", math.radians(27), math.radians(20), 1.3)
        )
        self.assertEqual(controller.direction, -1)
        decision = controller.update(math.radians(24), -0.05, 1.6)
        self.assertEqual(decision.state, MODULE.SETTLE)

    def test_post_turn_guard_holds_small_heading_changes(self):
        guard = MODULE.PostTurnGuard(
            hold_sec=1.2,
            hold_distance=0.05,
            reentry_error=math.radians(38),
            reentry_delay=0.35,
        )
        guard.begin(0.0, 0.0, 10.0)
        decision = guard.update(0.01, 0.0, math.radians(30), 10.5)
        self.assertTrue(decision.active)
        self.assertFalse(decision.allow_turn)

    def test_post_turn_guard_allows_a_confirmed_new_corner(self):
        guard = MODULE.PostTurnGuard(
            hold_sec=1.2,
            hold_distance=0.05,
            reentry_error=math.radians(38),
            reentry_delay=0.35,
        )
        guard.begin(0.0, 0.0, 10.0)
        decision = guard.update(0.01, 0.0, math.radians(45), 10.5)
        self.assertFalse(decision.active)
        self.assertTrue(decision.allow_turn)
        self.assertEqual(decision.reason, "new_corner")

    def test_post_turn_guard_finishes_after_forward_distance(self):
        guard = MODULE.PostTurnGuard(
            hold_sec=1.2,
            hold_distance=0.05,
            reentry_error=math.radians(38),
            reentry_delay=0.35,
        )
        guard.begin(0.0, 0.0, 10.0)
        decision = guard.update(0.051, 0.0, 0.0, 10.7)
        self.assertFalse(decision.active)
        self.assertEqual(decision.reason, "distance_complete")

    def test_path_freshness_uses_soft_warning_before_hard_stop(self):
        tracker = MODULE.PathFreshnessTracker(soft_timeout=2.5, hard_timeout=6.0)
        tracker.set_goal("goal-a", 0.0)
        self.assertTrue(tracker.accept_path("goal-a", 0.1))

        state, age = tracker.evaluate(3.0)
        self.assertEqual(state, MODULE.PATH_SOFT_STALE)
        self.assertAlmostEqual(age, 2.9)

        state, age = tracker.evaluate(6.2)
        self.assertEqual(state, MODULE.PATH_HARD_STALE)
        self.assertAlmostEqual(age, 6.1)
        self.assertEqual(tracker.soft_stale_count, 1)
        self.assertEqual(tracker.hard_stale_count, 1)

    def test_new_goal_requires_a_goal_local_path(self):
        tracker = MODULE.PathFreshnessTracker(soft_timeout=2.5, hard_timeout=6.0)
        tracker.set_goal("goal-a", 0.0)
        tracker.accept_path("goal-a", 0.1)

        self.assertTrue(tracker.set_goal("goal-b", 1.0))
        state, age = tracker.evaluate(1.1)
        self.assertEqual(state, MODULE.PATH_WAITING)
        self.assertIsNone(age)
        self.assertFalse(tracker.accept_path("goal-a", 1.2))
        self.assertTrue(tracker.accept_path("goal-b", 1.3))
        self.assertEqual(tracker.evaluate(1.4)[0], MODULE.PATH_FRESH)

    def test_fresh_path_closes_soft_stale_interval(self):
        tracker = MODULE.PathFreshnessTracker(soft_timeout=2.5, hard_timeout=6.0)
        tracker.set_goal("goal-a", 0.0)
        tracker.accept_path("goal-a", 0.0)
        tracker.evaluate(3.0)
        tracker.accept_path("goal-a", 4.0)

        snapshot = tracker.snapshot(4.0)
        self.assertEqual(snapshot["state"], MODULE.PATH_FRESH)
        self.assertAlmostEqual(snapshot["soft_stale_total_sec"], 1.0)


if __name__ == "__main__":
    unittest.main()
