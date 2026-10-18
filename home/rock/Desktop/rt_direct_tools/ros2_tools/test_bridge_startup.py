#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import time
import unittest


MODULE_PATH = pathlib.Path(__file__).with_name("f103_usb_ros2_bridge.py")
SPEC = importlib.util.spec_from_file_location("f103_usb_ros2_bridge", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class _Args:
    cmd_output_mode = "dcr"
    wheel_base_m = 0.205
    axle_base_m = 0.225
    max_linear_mm_s = 900
    max_angular_mrad_s = 200
    max_moving_angular_mrad_s = 100
    mapping_max_angular_mrad_s = 500
    mapping_max_moving_angular_mrad_s = 500
    firmware_vx_sign = -1.0
    firmware_vy_sign = 1.0
    firmware_wz_sign = 1.0
    inplace_startup_boost_wheel_mm_s = 45
    inplace_startup_boost_duration_sec = 0.8
    mapping_inplace_startup_boost_wheel_mm_s = 45
    mapping_inplace_startup_boost_duration_sec = 0.45
    inplace_startup_feedback_mm_s = 5
    inplace_startup_feedback_frames = 3
    inplace_startup_balance_ratio = 0.35
    inplace_reverse_confirm_sec = 0.8
    startup_boost_wheel_mm_s = 45
    startup_boost_duration_sec = 2.50
    startup_feedback_mm_s = 2
    startup_feedback_frames = 2
    zero_repeat_period = 0.5
    dcr_change_threshold_mm_s = 10
    dcr_max_update_hz = 3.0
    dcr_max_stale_sec = 0.0
    startup_retry_period = 1.2
    startup_retry_max = 1
    startup_retry_zero_sec = 0.15
    dcr_direction_blank_sec = 0.08


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass


def make_bridge():
    bridge = object.__new__(MODULE.F103UsbRos2Bridge)
    bridge.args = _Args()
    bridge.cmd_wheel_signs = (1, 1, 1, 1)
    bridge.odom_wheel_signs = (1, 1, -1, -1)
    bridge.inplace_wheel_min_mm_s = (30, 30, 30, 30)
    bridge.mapping_inplace_wheel_min_mm_s = (30, 30, 30, 30)
    bridge.active_command_source = "primary"
    bridge.active_inplace_wheel_min_mm_s = bridge.inplace_wheel_min_mm_s
    bridge.active_inplace_startup_boost_wheel_mm_s = 45
    bridge.mapping_inplace_startup_boost_duration_sec = 0.45
    bridge.active_inplace_startup_boost_duration_sec = 0.8
    bridge.last_sent = ("dcr", 0, 0, 0, 0)
    bridge.target_wheels = (0, 0, 0, 0)
    bridge.last_motion_write_time = 0.0
    bridge.last_zero_sent_at = 0.0
    bridge.startup_hold_active = False
    bridge.startup_hold_wheels = (0, 0, 0, 0)
    bridge.startup_pending_wheels = (0, 0, 0, 0)
    bridge.startup_feedback_frames = 0
    bridge.startup_feedback_threshold = bridge.args.startup_feedback_mm_s
    bridge.startup_feedback_required_frames = bridge.args.startup_feedback_frames
    bridge.startup_feedback_balance_ratio = 0.0
    bridge.startup_retry_count = 0
    bridge.startup_boost_deadline = 0.0
    bridge.dcr_blank_until = 0.0
    bridge.dcr_blank_pending_wheels = (0, 0, 0, 0)
    bridge.dcr_blank_reason = ""
    bridge.startup_rearm_pending = False
    bridge.inplace_direction_lock = 0
    bridge.inplace_reverse_pending_direction = 0
    bridge.inplace_reverse_pending_since = 0.0
    bridge.last_inplace_request_time = 0.0
    bridge.dcr_write_count = 0
    bridge.direction_blank_count = 0
    bridge.controlled_startup_retry_count = 0
    bridge.last_nonzero_command_time = 0.0
    bridge.target_vx = 0
    bridge.target_vy = 0
    bridge.target_wz = 0
    bridge.commands = []
    bridge._write_command = bridge.commands.append
    bridge.get_logger = lambda: _Logger()
    return bridge


class BridgeStartupTest(unittest.TestCase):
    def test_navigation_and_mapping_inplace_profiles_are_isolated(self):
        bridge = make_bridge()
        sent = []
        bridge._send_dcr_mm_s = lambda wheels, **kwargs: sent.append(
            (
                wheels,
                kwargs.get("startup_boost_override"),
                kwargs.get("startup_duration_override"),
            )
        )

        bridge._set_command_profile("primary")
        bridge._send_motion_mm_s(0, 0, 140)
        self.assertEqual(sent[-1], ((-30, -30, 30, 30), 45, 0.8))

        bridge._set_command_profile("mapping")
        bridge._send_motion_mm_s(0, 0, 140)
        self.assertEqual(sent[-1], ((-30, -30, 30, 30), 45, 0.45))

    def test_navigation_and_mapping_angular_limits_are_isolated(self):
        bridge = make_bridge()

        self.assertEqual(bridge._angular_limits_for_source("primary"), (200, 100))
        self.assertEqual(bridge._angular_limits_for_source("mapping"), (500, 500))

    def test_mapping_inplace_boost_uses_short_deadline(self):
        bridge = make_bridge()
        bridge._set_command_profile("mapping")
        bridge._send_dcr_mm_s(
            (30, 30, -30, -30),
            startup_boost_override=45,
            startup_duration_override=bridge.active_inplace_startup_boost_duration_sec,
        )

        remaining = bridge.startup_boost_deadline - time.monotonic()
        self.assertGreater(remaining, 0.0)
        self.assertLessEqual(remaining, 0.50)

    def test_low_target_gets_short_signed_boost_then_restores_target(self):
        bridge = make_bridge()
        bridge.args.startup_retry_max = 0
        bridge._send_dcr_mm_s((30, -30, 0, 60))

        self.assertEqual(bridge.commands, ["$DCR:45,-45,0,60!"])
        self.assertEqual(bridge.startup_pending_wheels, (30, -30, 0, 60))
        self.assertGreater(bridge.startup_boost_deadline, time.monotonic())

        bridge.startup_boost_deadline = time.monotonic() - 0.01
        bridge._update_startup_hold((0, 0, 0, 0))

        self.assertEqual(bridge.commands[-1], "$DCR:30,-30,0,60!")
        self.assertFalse(bridge.startup_hold_active)

    def test_feedback_releases_boost_before_timeout(self):
        bridge = make_bridge()
        bridge._send_dcr_mm_s((30, 30, -30, -30))
        bridge._update_startup_hold((3, 3, -3, -3))
        self.assertEqual(len(bridge.commands), 1)

        bridge._update_startup_hold((3, 3, -3, -3))
        self.assertEqual(bridge.commands[-1], "$DCR:30,30,-30,-30!")
        self.assertFalse(bridge.startup_hold_active)

    def test_matching_startup_target_is_not_repeated_on_release(self):
        bridge = make_bridge()
        bridge._send_dcr_mm_s(
            (45, 45, -45, -45),
            startup_boost_override=45,
            startup_feedback_override=5,
            startup_frames_override=3,
        )

        for _ in range(3):
            bridge._update_startup_hold((5, 6, -7, -8))

        self.assertEqual(bridge.commands, ["$DCR:45,45,-45,-45!"])
        self.assertFalse(bridge.startup_hold_active)

    def test_inplace_boost_releases_after_six_balanced_feedback_frames(self):
        bridge = make_bridge()
        bridge._send_dcr_mm_s(
            (30, 30, -30, -30),
            startup_boost_override=45,
            startup_feedback_override=12,
            startup_frames_override=6,
            startup_balance_ratio=0.45,
        )

        self.assertEqual(bridge.commands, ["$DCR:45,45,-45,-45!"])
        for _ in range(5):
            bridge._update_startup_hold((12, 13, -14, -15))
        self.assertEqual(len(bridge.commands), 1)
        self.assertTrue(bridge.startup_hold_active)

        bridge._update_startup_hold((12, 13, -14, -15))
        self.assertEqual(bridge.commands[-1], "$DCR:30,30,-30,-30!")
        self.assertFalse(bridge.startup_hold_active)

    def test_inplace_imbalanced_feedback_resets_confirmation_frames(self):
        bridge = make_bridge()
        bridge._send_dcr_mm_s(
            (30, 30, -30, -30),
            startup_boost_override=45,
            startup_feedback_override=12,
            startup_frames_override=2,
            startup_balance_ratio=0.45,
        )

        bridge._update_startup_hold((12, 13, -14, -15))
        bridge._update_startup_hold((12, 40, -14, -15))
        bridge._update_startup_hold((12, 13, -14, -15))
        self.assertEqual(len(bridge.commands), 1)

        bridge._update_startup_hold((12, 13, -14, -15))
        self.assertEqual(bridge.commands[-1], "$DCR:30,30,-30,-30!")

    def test_stop_cancels_boost_immediately(self):
        bridge = make_bridge()
        bridge._send_dcr_mm_s((30, 30, 30, 30))
        bridge._send_dcr_mm_s((0, 0, 0, 0))

        self.assertEqual(bridge.commands[-1], "$DCR:0,0,0,0!")
        self.assertFalse(bridge.startup_hold_active)
        self.assertEqual(bridge.startup_boost_deadline, 0.0)

    def test_small_changed_target_is_not_flushed_when_stale_updates_disabled(self):
        bridge = make_bridge()
        bridge.last_sent = ("dcr", 60, 60, 40, 40)
        bridge.target_wheels = (60, 60, 40, 40)
        bridge.last_motion_write_time = time.monotonic() - 1.1

        bridge._send_dcr_mm_s((56, 56, 44, 44))

        self.assertEqual(bridge.commands, [])
        self.assertEqual(bridge.target_wheels, (60, 60, 40, 40))

    def test_direction_change_has_real_zero_speed_blank(self):
        bridge = make_bridge()
        bridge.last_sent = ("dcr", 75, 75, -75, -75)
        bridge.target_wheels = (75, 75, -75, -75)

        bridge._send_dcr_mm_s((172, 172, 128, 128))

        self.assertEqual(bridge.commands, ["$DCR:0,0,0,0!"])
        self.assertGreater(bridge.dcr_blank_until, time.monotonic())
        self.assertEqual(bridge.dcr_blank_reason, "direction_change")

        bridge._send_dcr_mm_s((172, 172, 128, 128))
        self.assertEqual(len(bridge.commands), 1)

        bridge.dcr_blank_until = time.monotonic() - 0.01
        bridge._send_dcr_mm_s((172, 172, 128, 128))
        self.assertEqual(bridge.commands[-1], "$DCR:172,172,128,128!")
        self.assertTrue(bridge.startup_hold_active)

    def test_no_feedback_uses_one_controlled_zero_speed_restart(self):
        bridge = make_bridge()
        bridge.target_vx = 150
        bridge.target_vy = 0
        bridge.target_wz = 0
        bridge.last_wheel_mm_s = (0, 0, 0, 0)
        bridge._send_dcr_mm_s((172, 172, 128, 128))
        bridge.startup_boost_deadline = time.monotonic() - 0.01
        bridge.last_motion_write_time = time.monotonic() - 3.0

        bridge._update_startup_hold((0, 0, 0, 0))
        self.assertTrue(bridge.startup_hold_active)
        bridge._retry_stalled_startup(time.monotonic())

        self.assertEqual(bridge.commands[-1], "$DCR:0,0,0,0!")
        self.assertEqual(bridge.startup_retry_count, 1)
        self.assertEqual(bridge.controlled_startup_retry_count, 1)
        self.assertEqual(bridge.dcr_blank_reason, "startup_no_feedback")

        bridge.dcr_blank_until = time.monotonic() - 0.01
        bridge._send_dcr_mm_s((172, 172, 128, 128))
        self.assertEqual(bridge.commands[-1], "$DCR:172,172,128,128!")
        self.assertEqual(bridge.startup_retry_count, 1)

    def test_stop_cancels_pending_direction_blank(self):
        bridge = make_bridge()
        bridge.last_sent = ("dcr", 75, 75, -75, -75)
        bridge._send_dcr_mm_s((172, 172, 128, 128))

        bridge._send_dcr_mm_s((0, 0, 0, 0), force=True)

        self.assertEqual(bridge.commands[-1], "$DCR:0,0,0,0!")
        self.assertEqual(bridge.dcr_blank_until, 0.0)
        self.assertEqual(bridge.dcr_blank_pending_wheels, (0, 0, 0, 0))

    def test_wheel_saturation_scales_all_wheels_and_preserves_curvature(self):
        bridge = make_bridge()
        bridge.args.max_linear_mm_s = 300

        wheels = bridge._body_to_wheels_mm_s(300, 0, 100)

        self.assertEqual(wheels, (260, 260, 300, 300))

    def test_navigation_inplace_reverse_requires_stable_confirmation(self):
        bridge = make_bridge()
        bridge._send_motion_mm_s(0, 0, 140)
        self.assertEqual(bridge.commands[-1], "$DCR:-45,-45,45,45!")

        bridge._send_motion_mm_s(0, 0, -140)
        self.assertEqual(bridge.commands[-1], "$DCR:0,0,0,0!")
        self.assertEqual(bridge.inplace_reverse_pending_direction, -1)

        bridge.inplace_reverse_pending_since = time.monotonic() - 0.9
        bridge._send_motion_mm_s(0, 0, -140)
        self.assertEqual(bridge.commands[-1], "$DCR:45,45,-45,-45!")
        self.assertEqual(bridge.inplace_direction_lock, -1)

    def test_forced_stop_is_immediate_and_clears_direction_guard(self):
        bridge = make_bridge()
        bridge._send_motion_mm_s(0, 0, 140)

        bridge._send_motion_mm_s(0, 0, 0, force=True)

        self.assertEqual(bridge.commands[-1], "$DCR:0,0,0,0!")
        self.assertEqual(bridge.inplace_direction_lock, 0)
        self.assertEqual(bridge.inplace_reverse_pending_direction, 0)

    def test_mapping_inplace_reverse_is_not_delayed(self):
        bridge = make_bridge()
        bridge._set_command_profile("mapping")

        bridge._send_motion_mm_s(0, 0, 140)
        bridge._send_motion_mm_s(0, 0, -140)

        self.assertEqual(bridge.commands[-1], "$DCR:45,45,-45,-45!")

    def test_motion_mode_and_cumulative_direction_blank_count(self):
        bridge = make_bridge()
        self.assertEqual(bridge._current_motion_mode(), "stopped")

        bridge.target_vx = 60
        bridge.target_wz = 40
        self.assertEqual(bridge._current_motion_mode(), "forward_arc")

        bridge._begin_dcr_blank(
            (-45, -45, 45, 45),
            time.monotonic(),
            "direction_change",
        )
        self.assertEqual(bridge.direction_blank_count, 1)
        self.assertEqual(bridge._current_motion_mode(), "direction_blank")


if __name__ == "__main__":
    unittest.main()
