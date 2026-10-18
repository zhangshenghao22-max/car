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
    firmware_vx_sign = -1.0
    firmware_vy_sign = 1.0
    firmware_wz_sign = 1.0
    inplace_startup_boost_wheel_mm_s = 60
    mapping_inplace_startup_boost_wheel_mm_s = 45
    inplace_startup_feedback_mm_s = 12
    inplace_startup_feedback_frames = 6
    inplace_startup_balance_ratio = 0.45
    startup_boost_wheel_mm_s = 45
    startup_boost_duration_sec = 2.50
    startup_feedback_mm_s = 2
    startup_feedback_frames = 2
    zero_repeat_period = 0.5
    dcr_change_threshold_mm_s = 10
    dcr_max_update_hz = 2.0
    dcr_max_stale_sec = 1.0


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
    bridge.inplace_wheel_min_mm_s = (45, 45, 45, 45)
    bridge.mapping_inplace_wheel_min_mm_s = (30, 30, 30, 30)
    bridge.active_command_source = "primary"
    bridge.active_inplace_wheel_min_mm_s = bridge.inplace_wheel_min_mm_s
    bridge.active_inplace_startup_boost_wheel_mm_s = 60
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
    bridge.commands = []
    bridge._write_command = bridge.commands.append
    bridge.get_logger = lambda: _Logger()
    return bridge


class BridgeStartupTest(unittest.TestCase):
    def test_navigation_and_mapping_inplace_profiles_are_isolated(self):
        bridge = make_bridge()
        sent = []
        bridge._send_dcr_mm_s = lambda wheels, **kwargs: sent.append(
            (wheels, kwargs.get("startup_boost_override"))
        )

        bridge._set_command_profile("primary")
        bridge._send_motion_mm_s(0, 0, 140)
        self.assertEqual(sent[-1], ((-45, -45, 45, 45), 60))

        bridge._set_command_profile("mapping")
        bridge._send_motion_mm_s(0, 0, 140)
        self.assertEqual(sent[-1], ((-30, -30, 30, 30), 45))

    def test_low_target_gets_short_signed_boost_then_restores_target(self):
        bridge = make_bridge()
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

    def test_small_changed_target_is_flushed_after_stale_period(self):
        bridge = make_bridge()
        bridge.last_sent = ("dcr", 60, 60, 40, 40)
        bridge.target_wheels = (60, 60, 40, 40)
        bridge.last_motion_write_time = time.monotonic() - 1.1

        bridge._send_dcr_mm_s((56, 56, 44, 44))

        self.assertEqual(bridge.commands, ["$DCR:56,56,44,44!"])
        self.assertEqual(bridge.target_wheels, (56, 56, 44, 44))


if __name__ == "__main__":
    unittest.main()
