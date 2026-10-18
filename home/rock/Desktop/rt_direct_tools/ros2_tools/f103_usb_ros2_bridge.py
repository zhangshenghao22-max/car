#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import threading
import time
import csv
from dataclasses import dataclass
from typing import Optional

DEFAULT_FASTDDS_PROFILE = "/home/rock/Desktop/rock_ws/fastdds_no_shm.xml"
if not os.environ.get("RMW_IMPLEMENTATION"):
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
if not os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE") and os.path.exists(DEFAULT_FASTDDS_PROFILE):
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = DEFAULT_FASTDDS_PROFILE

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

try:
    from car2_arm_interfaces.srv import MoveArmPose
except Exception:  # pragma: no cover - optional until the interface package is built
    MoveArmPose = None

try:
    import serial
except Exception as exc:  # pragma: no cover - runtime dependency check
    raise SystemExit(f"pyserial is required: {exc}")


DEFAULT_PORT = "/dev/f103"
DEFAULT_BAUDRATE = 115200
WHEEL_BASE_M = 0.205
AXLE_BASE_M = 0.225

ODOM_RE = re.compile(
    r"@ODOM:SEQ=(?P<seq>\d+),DT_MS=(?P<dt>\d+),"
    r"EA=(?P<ea>-?\d+),EB=(?P<eb>-?\d+),EC=(?P<ec>-?\d+),ED=(?P<ed>-?\d+),"
    r"WA=(?P<wa>-?\d+),WB=(?P<wb>-?\d+),WC=(?P<wc>-?\d+),WD=(?P<wd>-?\d+)!"
)
TWIST_OK_RE = re.compile(r"@TWIST:SEQ=(?P<seq>\d+),OK!")
TWIST_ERR_RE = re.compile(r"@TWIST:SEQ=(?P<seq>\d+),ERR=(?P<err>[^,]+),MODE=(?P<mode>[^!]+)!")
FLOAT_RE = r"-?\d+(?:\.\d+)?"
IK6_OK_RE = re.compile(
    rf"@IK6:OK,SEQ=(?P<seq>\d+),"
    rf"J0=(?P<j0>{FLOAT_RE}),J1=(?P<j1>{FLOAT_RE}),J2=(?P<j2>{FLOAT_RE}),"
    rf"J3=(?P<j3>{FLOAT_RE}),J4=(?P<j4>{FLOAT_RE}),J5=(?P<j5>{FLOAT_RE}),"
    rf"P0=(?P<p0>-?\d+),P1=(?P<p1>-?\d+),P2=(?P<p2>-?\d+),"
    rf"P3=(?P<p3>-?\d+),P4=(?P<p4>-?\d+),P5=(?P<p5>-?\d+),T=(?P<t>\d+)!"
)
IK6_ERR_RE = re.compile(r"@IK6:ERR,CODE=(?P<code>-?\d+),REASON=(?P<reason>[^!]+)!")
ARM_DONE_RE = re.compile(r"@ARM:DONE,SEQ=(?P<seq>\d+)!")


@dataclass
class OdomFrame:
    seq: int
    dt_s: float
    encoder_ticks: tuple[int, int, int, int]
    wheel_mm_s: tuple[int, int, int, int]


@dataclass
class Ik6OkFrame:
    seq: int
    joint_deg: tuple[float, float, float, float, float, float]
    servo_pwm: tuple[int, int, int, int, int, int]
    actual_duration_s: float


class F103UsbRos2Bridge(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__(args.node_name)
        self.args = args
        self.args.port = self._resolve_serial_port(args.port)
        self.cmd_wheel_signs = self._parse_wheel_signs(args.cmd_wheel_signs, "--cmd-wheel-signs")
        self.odom_wheel_signs = self._parse_wheel_signs(args.odom_wheel_signs, "--odom-wheel-signs")
        self.inplace_wheel_min_mm_s = self._parse_wheel_minimums(
            args.inplace_wheel_min_mm_s,
            "--inplace-wheel-min-mm-s",
        )
        self.mapping_inplace_wheel_min_mm_s = self._parse_wheel_minimums(
            args.mapping_inplace_wheel_min_mm_s,
            "--mapping-inplace-wheel-min-mm-s",
        )
        self.active_command_source = "primary"
        self.active_inplace_wheel_min_mm_s = self.inplace_wheel_min_mm_s
        self.active_inplace_startup_boost_wheel_mm_s = (
            args.inplace_startup_boost_wheel_mm_s
        )
        self.mapping_inplace_startup_boost_duration_sec = (
            args.mapping_inplace_startup_boost_duration_sec
            if args.mapping_inplace_startup_boost_duration_sec >= 0.0
            else args.startup_boost_duration_sec
        )
        self.active_inplace_startup_boost_duration_sec = (
            args.inplace_startup_boost_duration_sec
        )
        self.serial_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.seq = int(time.time()) % 900000

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_seq: Optional[int] = None
        self.last_cmd_time = 0.0
        self.last_rx_time = 0.0
        self.last_state = ""
        self.last_twist_ack = ""
        self.last_encoder_ticks = (0, 0, 0, 0)
        self.last_wheel_mm_s = (0, 0, 0, 0)
        self.last_body_twist = (0.0, 0.0, 0.0)
        self.last_imu_wz = 0.0
        self.last_imu_time = 0.0
        self.motion_fault = ""
        self.motion_fault_time = 0.0
        self.motion_fault_zero_since = 0.0
        self.last_motion_warning = ""
        self.motion_warning_last_logged: dict[str, float] = {}
        self.wheel_fault_cycles = [0, 0, 0, 0]
        self.imu_fault_cycles = 0
        self.wheel_overspeed_cycles = [0, 0, 0, 0]
        self.rx_count = 0
        self.odom_count = 0
        self.state_count = 0
        self.twist_ok_count = 0
        self.twist_err_count = 0
        self.unknown_count = 0
        self.arm_cmd_count = 0
        self.arm_cmd_reject_count = 0
        self.last_arm_cmd = ""
        self.last_arm_cmd_time = 0.0
        self.arm_pose_request_count = 0
        self.arm_pose_success_count = 0
        self.arm_pose_error_count = 0
        self.last_arm_pose_command = ""
        self.last_arm_pose_result = ""
        self.last_ik6_ok: Optional[Ik6OkFrame] = None
        self.last_ik6_ok_time = 0.0
        self.last_ik6_error: Optional[tuple[int, str]] = None
        self.last_arm_done_seq: Optional[int] = None
        self._arm_pose_condition = threading.Condition()
        self._arm_pose_inflight = False
        self._pending_ik6_ok: Optional[Ik6OkFrame] = None
        self._pending_ik6_error: Optional[tuple[int, str]] = None
        self._pending_arm_done_seq: Optional[int] = None

        self.target_vx = 0
        self.target_vy = 0
        self.target_wz = 0
        self.target_wheels = (0, 0, 0, 0)
        self.requested_wheels = (0, 0, 0, 0)
        self.last_sent = (None, None, None)
        self.last_motion_write_time = 0.0
        self.last_zero_sent_at = 0.0
        self.startup_hold_active = False
        self.startup_hold_wheels = (0, 0, 0, 0)
        self.startup_pending_wheels = (0, 0, 0, 0)
        self.startup_feedback_frames = 0
        self.startup_feedback_threshold = max(1, args.startup_feedback_mm_s)
        self.startup_feedback_required_frames = max(1, args.startup_feedback_frames)
        self.startup_feedback_balance_ratio = 0.0
        self.startup_retry_count = 0
        self.startup_boost_deadline = 0.0
        self.dcr_blank_until = 0.0
        self.dcr_blank_pending_wheels = (0, 0, 0, 0)
        self.dcr_blank_reason = ""
        self.startup_rearm_pending = False
        self.inplace_direction_lock = 0
        self.inplace_reverse_pending_direction = 0
        self.inplace_reverse_pending_since = 0.0
        self.last_inplace_request_time = 0.0
        self.dcr_write_count = 0
        self.direction_blank_count = 0
        self.controlled_startup_retry_count = 0
        self.last_nonzero_command_time = 0.0
        self.cabinet_mode_active = False
        self.cabinet_mode_changed_at = 0.0
        self.last_cabinet_cmd_time = 0.0

        self.odom_pub = self.create_publisher(Odometry, args.odom_topic, 20)
        self.state_pub = self.create_publisher(String, args.state_topic, 20)
        self.fault_pub = self.create_publisher(String, args.motion_fault_topic, 20)

        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Twist, args.cmd_topic, self._on_twist, cmd_qos)
        imu_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Imu, args.imu_topic, self._on_imu, imu_qos)
        if args.extra_cmd_topic and args.extra_cmd_topic != args.cmd_topic:
            self.create_subscription(Twist, args.extra_cmd_topic, self._on_twist, cmd_qos)
        if args.mapping_cmd_topic and args.mapping_cmd_topic not in (
            args.cmd_topic,
            args.extra_cmd_topic,
        ):
            self.create_subscription(
                Twist,
                args.mapping_cmd_topic,
                self._on_mapping_twist,
                cmd_qos,
            )
        self.create_subscription(
            Twist,
            args.cabinet_cmd_topic,
            self._on_cabinet_twist,
            cmd_qos,
        )
        self.cabinet_mode_service = self.create_service(
            SetBool,
            args.cabinet_mode_service,
            self._on_cabinet_mode,
        )
        if args.arm_topic and not args.disable_arm_topic:
            arm_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(String, args.arm_topic, self._on_arm_cmd, arm_qos)

        self.arm_pose_service = None
        if not args.disable_arm_pose_service:
            if MoveArmPose is None:
                self.get_logger().warning(
                    "car2_arm_interfaces is not available; /arm/move_pose service is disabled"
                )
            else:
                self.arm_pose_service = self.create_service(
                    MoveArmPose,
                    args.arm_pose_service,
                    self._on_move_arm_pose,
                )

        self.serial = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            timeout=0.05,
            write_timeout=0.5,
        )
        with self.serial_lock:
            if not args.keep_input_buffer:
                self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

        self.reader_thread = threading.Thread(target=self._reader_loop, name="f103-usb-rx", daemon=True)
        self.reader_thread.start()

        self.command_timer = self.create_timer(1.0 / max(1.0, args.command_rate_hz), self._command_timer)
        self.status_timer = self.create_timer(max(0.02, args.status_period), self._publish_status)
        self.handshake_timer = self.create_timer(0.2, self._handshake_timer)
        self.handshake_done = False
        self.diag_file = None
        self.diag_writer = None
        if args.motion_diag_dir:
            os.makedirs(args.motion_diag_dir, exist_ok=True)
            path = os.path.join(args.motion_diag_dir, time.strftime("motion_%Y%m%d_%H%M%S.csv"))
            self.diag_file = open(path, "w", newline="", encoding="ascii")
            self.diag_writer = csv.writer(self.diag_file)
            self.diag_writer.writerow([
                "monotonic_s", "target_a", "target_b", "target_c", "target_d",
                "wheel_a", "wheel_b", "wheel_c", "wheel_d", "imu_wz",
                "odom_vx", "odom_vy", "odom_wz", "fault"
            ])
            self.diag_file.flush()

        self.get_logger().info(
            f"F103 USB bridge opened {args.port} @ {args.baudrate}; "
            f"subscribe {args.cmd_topic}"
            + (f" and {args.extra_cmd_topic}" if args.extra_cmd_topic else "")
            + (f"; mapping {args.mapping_cmd_topic}" if args.mapping_cmd_topic else "")
            + (f"; arm {args.arm_topic}" if args.arm_topic and not args.disable_arm_topic else "")
            + (f"; arm_pose_service {args.arm_pose_service}" if self.arm_pose_service is not None else "")
            + f"; publish {args.odom_topic}"
        )

    def destroy_node(self):
        self.stop_event.set()
        self._send_motion_mm_s(0, 0, 0, force=True)
        try:
            if self.reader_thread.is_alive():
                self.reader_thread.join(timeout=0.5)
        except Exception:
            pass
        try:
            self.serial.close()
        except Exception:
            pass
        if self.diag_file is not None:
            self.diag_file.close()
        super().destroy_node()

    def _on_imu(self, msg: Imu):
        self.last_imu_wz = float(msg.angular_velocity.z)
        self.last_imu_time = time.monotonic()

    def _report_motion_warning(self, reason: str):
        self.last_motion_warning = reason
        key = reason.split(":", 1)[0]
        now = time.monotonic()
        if now - self.motion_warning_last_logged.get(key, 0.0) < max(
            0.1, self.args.motion_warning_period_sec
        ):
            return
        self.motion_warning_last_logged[key] = now
        msg = String()
        msg.data = reason
        self.fault_pub.publish(msg)
        self.get_logger().warning(f"motion consistency warning: {reason}")

    def _send_handshake(self):
        for command in ("$PING!", "$STATUS!", "$REPORT:ON!", "$ODOM:ON!", "$MODE:NAVIGATION!", "$REPORT:ON!", "$ODOM:ON!"):
            self._write_command(command)
            time.sleep(max(0.03, self.args.handshake_interval))
        self._send_motion_mm_s(0, 0, 0, force=True)

    def _handshake_timer(self):
        if self.handshake_done:
            return
        self.handshake_done = True
        self.get_logger().info("sending F103 handshake")
        self._send_handshake()
        self.get_logger().info("F103 handshake sent")

    def _write_command(self, command: str):
        payload = command.strip()
        if not payload:
            return
        if not payload.endswith("\r\n"):
            payload += "\r\n"
        data = payload.encode("ascii", errors="ignore")
        with self.serial_lock:
            self.serial.write(data)
            if self.args.flush_serial_writes:
                self.serial.flush()
        if payload.startswith("$DCR:"):
            self.dcr_write_count += 1

    def _send_twist_mm_s(self, vx: int, vy: int, wz: int, *, force: bool = False):
        now = time.monotonic()
        if not force:
            if ("twist", vx, vy, wz) == self.last_sent:
                if (vx, vy, wz) != (0, 0, 0):
                    pass
                elif now - self.last_zero_sent_at < self.args.zero_repeat_period:
                    return
        self.seq = (self.seq % 999999) + 1
        self._write_command(f"$TWIST:SEQ={self.seq},VX={vx},VY={vy},WZ={wz}!")
        self.last_sent = ("twist", vx, vy, wz)
        if (vx, vy, wz) == (0, 0, 0):
            self.last_zero_sent_at = now

    def _send_dcr_mm_s(
        self,
        wheels: tuple[int, int, int, int],
        *,
        force: bool = False,
        startup_boost_override: Optional[int] = None,
        startup_duration_override: Optional[float] = None,
        startup_feedback_override: Optional[int] = None,
        startup_frames_override: Optional[int] = None,
        startup_balance_ratio: float = 0.0,
    ):
        now = time.monotonic()
        a, b, c, d = wheels
        stopped = wheels == (0, 0, 0, 0)

        if stopped:
            self.startup_hold_active = False
            self.startup_hold_wheels = (0, 0, 0, 0)
            self.startup_pending_wheels = (0, 0, 0, 0)
            self.startup_feedback_frames = 0
            self.startup_feedback_threshold = max(1, self.args.startup_feedback_mm_s)
            self.startup_feedback_required_frames = max(1, self.args.startup_feedback_frames)
            self.startup_feedback_balance_ratio = 0.0
            self.startup_retry_count = 0
            self.startup_boost_deadline = 0.0
            self.dcr_blank_until = 0.0
            self.dcr_blank_pending_wheels = (0, 0, 0, 0)
            self.dcr_blank_reason = ""
            self.startup_rearm_pending = False
            if not force and ("dcr", a, b, c, d) == self.last_sent:
                if now - self.last_zero_sent_at < self.args.zero_repeat_period:
                    return
            self._write_command(f"$DCR:{a},{b},{c},{d}!")
            self.last_sent = ("dcr", a, b, c, d)
            self.target_wheels = wheels
            self.last_motion_write_time = now
            self.last_zero_sent_at = now
            return

        if self.dcr_blank_until > 0.0:
            self.dcr_blank_pending_wheels = wheels
            if now < self.dcr_blank_until:
                return
            self.dcr_blank_until = 0.0
            self.dcr_blank_pending_wheels = (0, 0, 0, 0)
            self.dcr_blank_reason = ""

        last_wheels = self._last_sent_dcr_wheels()
        if self.startup_hold_active:
            self.startup_pending_wheels = wheels
            if not self._wheel_directions_changed(self.startup_hold_wheels, wheels):
                return

            if self.active_command_source == "primary":
                self._begin_dcr_blank(wheels, now, "direction_change")
                return

            self._write_command("$DCR:0,0,0,0!")
            self.last_sent = ("dcr", 0, 0, 0, 0)
            self.target_wheels = (0, 0, 0, 0)
            self.last_motion_write_time = now
            self.last_zero_sent_at = now
            self.startup_hold_active = False
            last_wheels = (0, 0, 0, 0)

        if (
            self.active_command_source == "primary"
            and last_wheels is not None
            and last_wheels != (0, 0, 0, 0)
            and self._wheel_directions_changed(last_wheels, wheels)
        ):
            self._begin_dcr_blank(wheels, now, "direction_change")
            return

        if not force and last_wheels is not None:
            max_delta = max(abs(value - previous) for value, previous in zip(wheels, last_wheels))
            if max_delta == 0:
                return
            if max_delta < max(1, self.args.dcr_change_threshold_mm_s):
                stale_period = float(self.args.dcr_max_stale_sec)
                if stale_period <= 0.0 or now - self.last_motion_write_time < stale_period:
                    return
            min_period = 1.0 / max(0.1, self.args.dcr_max_update_hz)
            if last_wheels != (0, 0, 0, 0) and now - self.last_motion_write_time < min_period:
                return

        starting = (
            last_wheels is None
            or last_wheels == (0, 0, 0, 0)
            or self._wheel_directions_changed(last_wheels, wheels)
        )
        command_wheels = wheels
        startup_minimum = max(
            0,
            int(
                self.args.startup_boost_wheel_mm_s
                if startup_boost_override is None
                else startup_boost_override
            ),
        )
        if starting and startup_minimum > 0:
            command_wheels = tuple(
                max(abs(value), startup_minimum) if value > 0 else -max(abs(value), startup_minimum)
                if value < 0 else 0
                for value in wheels
            )

        self._write_command("$DCR:{},{},{},{}!".format(*command_wheels))
        self.last_sent = ("dcr", *command_wheels)
        self.target_wheels = command_wheels
        self.last_motion_write_time = now

        if starting:
            self.startup_hold_active = True
            self.startup_hold_wheels = command_wheels
            self.startup_pending_wheels = wheels
            self.startup_feedback_frames = 0
            self.startup_feedback_threshold = max(
                1,
                int(
                    self.args.startup_feedback_mm_s
                    if startup_feedback_override is None
                    else startup_feedback_override
                ),
            )
            self.startup_feedback_required_frames = max(
                1,
                int(
                    self.args.startup_feedback_frames
                    if startup_frames_override is None
                    else startup_frames_override
                ),
            )
            self.startup_feedback_balance_ratio = max(0.0, startup_balance_ratio)
            if self.startup_rearm_pending:
                self.startup_rearm_pending = False
            else:
                self.startup_retry_count = 0
            startup_duration = (
                self.args.startup_boost_duration_sec
                if startup_duration_override is None
                else startup_duration_override
            )
            self.startup_boost_deadline = now + max(0.0, float(startup_duration))

    def _begin_dcr_blank(
        self,
        pending_wheels: tuple[int, int, int, int],
        now: float,
        reason: str,
    ) -> None:
        blank_sec = (
            self.args.startup_retry_zero_sec
            if reason == "startup_no_feedback"
            else self.args.dcr_direction_blank_sec
        )
        self._write_command("$DCR:0,0,0,0!")
        self.last_sent = ("dcr", 0, 0, 0, 0)
        self.target_wheels = (0, 0, 0, 0)
        self.last_motion_write_time = now
        self.last_zero_sent_at = now
        self.startup_hold_active = False
        self.startup_hold_wheels = (0, 0, 0, 0)
        self.startup_pending_wheels = pending_wheels
        self.startup_feedback_frames = 0
        self.startup_boost_deadline = 0.0
        self.dcr_blank_until = now + max(0.02, float(blank_sec))
        self.dcr_blank_pending_wheels = pending_wheels
        self.dcr_blank_reason = reason
        self.startup_rearm_pending = reason == "startup_no_feedback"
        if reason == "direction_change":
            self.direction_blank_count += 1
        self.get_logger().info(
            "DCR zero-speed blank reason={} duration={:.3f}s pending={}".format(
                reason,
                max(0.02, float(blank_sec)),
                pending_wheels,
            )
        )

    def _last_sent_dcr_wheels(self) -> Optional[tuple[int, int, int, int]]:
        if len(self.last_sent) != 5 or self.last_sent[0] != "dcr":
            return None
        return (
            int(self.last_sent[1]),
            int(self.last_sent[2]),
            int(self.last_sent[3]),
            int(self.last_sent[4]),
        )

    @staticmethod
    def _wheel_directions_changed(
        previous: tuple[int, int, int, int],
        current: tuple[int, int, int, int],
    ) -> bool:
        return any(old * new < 0 for old, new in zip(previous, current))

    def _update_startup_hold(self, wheel_mm_s: tuple[int, int, int, int]):
        if not self.startup_hold_active:
            return

        now = time.monotonic()
        threshold = self.startup_feedback_threshold
        active_feedback_present = all(
            target == 0 or abs(feedback) >= threshold
            for target, feedback in zip(self.startup_hold_wheels, wheel_mm_s)
        )
        active_magnitudes = [
            abs(feedback)
            for target, feedback in zip(self.startup_hold_wheels, wheel_mm_s)
            if target != 0
        ]
        if active_feedback_present and active_magnitudes:
            active_feedback_present = min(active_magnitudes) >= (
                max(active_magnitudes) * self.startup_feedback_balance_ratio
            )
        if active_feedback_present:
            self.startup_feedback_frames += 1
        else:
            self.startup_feedback_frames = 0

        feedback_confirmed = (
            self.startup_feedback_frames >= self.startup_feedback_required_frames
        )
        boost_expired = 0.0 < self.startup_boost_deadline <= now
        if not feedback_confirmed and not boost_expired:
            return

        if (
            boost_expired
            and not feedback_confirmed
            and self.active_command_source == "primary"
            and self.args.startup_retry_max > 0
        ):
            if self.startup_retry_count < self.args.startup_retry_max:
                return
            self.startup_boost_deadline = 0.0
            self.get_logger().warn(
                "DCR startup still has no wheel feedback after controlled retry; "
                "holding one stable command wheels={}".format(self.startup_hold_wheels)
            )
            return

        hold_wheels = self.startup_hold_wheels
        pending = self.startup_pending_wheels
        self.startup_hold_active = False
        self.startup_feedback_frames = 0
        self.startup_boost_deadline = 0.0
        self.get_logger().info(
            "DCR startup release reason={} wheels={} feedback={} target={}".format(
                "feedback" if feedback_confirmed else "boost_timeout",
                self.startup_hold_wheels,
                wheel_mm_s,
                pending,
            )
        )
        if pending != hold_wheels:
            self._send_dcr_mm_s(pending, force=True)

    def _retry_stalled_startup(self, now: float) -> None:
        if not self.startup_hold_active:
            return
        if self.active_command_source != "primary":
            return
        if self.target_vx == 0 and self.target_vy == 0 and self.target_wz == 0:
            return
        if self.startup_retry_count >= max(0, self.args.startup_retry_max):
            return
        if self.startup_boost_deadline <= 0.0 or now < self.startup_boost_deadline:
            return
        if now - self.last_motion_write_time < max(0.1, self.args.startup_retry_period):
            return

        threshold = max(1, self.args.startup_feedback_mm_s)
        any_active_feedback = any(
            target != 0 and abs(feedback) >= threshold
            for target, feedback in zip(self.startup_hold_wheels, self.last_wheel_mm_s)
        )
        if any_active_feedback:
            return

        wheels = self.startup_pending_wheels
        self.startup_retry_count += 1
        self.controlled_startup_retry_count += 1
        self._begin_dcr_blank(wheels, now, "startup_no_feedback")
        self.get_logger().warn(
            "DCR startup had no wheel feedback; controlled retry {}/{} wheels={}".format(
                self.startup_retry_count,
                self.args.startup_retry_max,
                wheels,
            )
        )

    def _body_to_wheels_mm_s(self, vx: int, vy: int, wz: int) -> tuple[int, int, int, int]:
        half_base = max(1e-6, (self.args.wheel_base_m + self.args.axle_base_m) * 0.5)
        rot = half_base * float(wz)
        wheels = (
            float(vx) + float(vy) - rot,
            float(vx) - float(vy) - rot,
            float(vx) + float(vy) + rot,
            float(vx) - float(vy) + rot,
        )
        peak = max(abs(value) for value in wheels)
        limit = max(0, int(self.args.max_linear_mm_s))
        scale = min(1.0, float(limit) / peak) if peak > 0.0 else 1.0
        signed = tuple(
            self._clamp(round(value * scale * sign), -limit, limit)
            for value, sign in zip(wheels, self.cmd_wheel_signs)
        )
        return signed  # type: ignore[return-value]

    def _reset_inplace_reverse_guard(self) -> None:
        self.inplace_direction_lock = 0
        self.inplace_reverse_pending_direction = 0
        self.inplace_reverse_pending_since = 0.0
        self.last_inplace_request_time = 0.0

    def _guard_navigation_inplace_reverse(
        self,
        vx: int,
        vy: int,
        wz: int,
        *,
        force: bool,
    ) -> tuple[int, int, int]:
        if self.active_command_source != "primary":
            self._reset_inplace_reverse_guard()
            return vx, vy, wz

        now = time.monotonic()
        if vx != 0 or vy != 0:
            self._reset_inplace_reverse_guard()
            return vx, vy, wz

        if wz == 0:
            if force or (
                self.last_inplace_request_time > 0.0
                and now - self.last_inplace_request_time
                >= max(0.0, self.args.inplace_reverse_confirm_sec)
            ):
                self._reset_inplace_reverse_guard()
            return 0, 0, 0

        direction = 1 if wz > 0 else -1
        self.last_inplace_request_time = now
        if self.inplace_direction_lock == 0:
            self.inplace_direction_lock = direction
            self.inplace_reverse_pending_direction = 0
            self.inplace_reverse_pending_since = 0.0
            return vx, vy, wz

        if direction == self.inplace_direction_lock:
            self.inplace_reverse_pending_direction = 0
            self.inplace_reverse_pending_since = 0.0
            return vx, vy, wz

        if self.inplace_reverse_pending_direction != direction:
            self.inplace_reverse_pending_direction = direction
            self.inplace_reverse_pending_since = now
            return 0, 0, 0

        confirm_sec = max(0.0, float(self.args.inplace_reverse_confirm_sec))
        if now - self.inplace_reverse_pending_since < confirm_sec:
            return 0, 0, 0

        self.inplace_direction_lock = direction
        self.inplace_reverse_pending_direction = 0
        self.inplace_reverse_pending_since = 0.0
        return vx, vy, wz

    def _send_motion_mm_s(self, vx: int, vy: int, wz: int, *, force: bool = False):
        if self.args.cmd_output_mode == "dcr":
            vx, vy, wz = self._guard_navigation_inplace_reverse(
                vx,
                vy,
                wz,
                force=force,
            )
            wheels = self._body_to_wheels_mm_s(vx, vy, wz)
            pure_inplace = vx == 0 and vy == 0 and wz != 0
            if pure_inplace:
                wheels = tuple(
                    (minimum if value > 0 else -minimum)
                    if value != 0 and minimum > 0 and abs(value) < minimum
                    else value
                    for value, minimum in zip(
                        wheels,
                        self.active_inplace_wheel_min_mm_s,
                    )
                )
            self.requested_wheels = wheels
            self._send_dcr_mm_s(
                wheels,
                force=force,
                startup_boost_override=(
                    self.active_inplace_startup_boost_wheel_mm_s
                    if pure_inplace
                    else None
                ),
                startup_duration_override=(
                    self.active_inplace_startup_boost_duration_sec
                    if pure_inplace
                    else None
                ),
                startup_feedback_override=(
                    self.args.inplace_startup_feedback_mm_s
                    if pure_inplace
                    else None
                ),
                startup_frames_override=(
                    self.args.inplace_startup_feedback_frames
                    if pure_inplace
                    else None
                ),
                startup_balance_ratio=(
                    self.args.inplace_startup_balance_ratio if pure_inplace else 0.0
                ),
            )
            return
        firmware_wheels = self._body_to_wheels_mm_s(
            round(vx * self.args.firmware_vx_sign),
            round(vy * self.args.firmware_vy_sign),
            round(wz * self.args.firmware_wz_sign),
        )
        self.target_wheels = tuple(
            value * sign for value, sign in zip(firmware_wheels, self.odom_wheel_signs)
        )
        self.requested_wheels = self.target_wheels
        self._send_twist_mm_s(vx, vy, wz, force=force)

    def _set_command_profile(self, source: str) -> None:
        if source == "mapping":
            if self.active_command_source != "mapping":
                self._reset_inplace_reverse_guard()
            self.active_command_source = "mapping"
            self.active_inplace_wheel_min_mm_s = (
                self.mapping_inplace_wheel_min_mm_s
            )
            self.active_inplace_startup_boost_wheel_mm_s = (
                self.args.mapping_inplace_startup_boost_wheel_mm_s
            )
            self.active_inplace_startup_boost_duration_sec = (
                self.mapping_inplace_startup_boost_duration_sec
            )
            return
        self.active_command_source = "primary"
        self.active_inplace_wheel_min_mm_s = self.inplace_wheel_min_mm_s
        self.active_inplace_startup_boost_wheel_mm_s = (
            self.args.inplace_startup_boost_wheel_mm_s
        )
        self.active_inplace_startup_boost_duration_sec = (
            self.args.inplace_startup_boost_duration_sec
        )

    def _on_twist(self, msg: Twist):
        if self.cabinet_mode_active:
            return
        self._handle_twist(msg, "primary")

    def _on_mapping_twist(self, msg: Twist):
        if self.cabinet_mode_active:
            return
        self._handle_twist(msg, "mapping")

    def _on_cabinet_twist(self, msg: Twist):
        if not self.cabinet_mode_active:
            return
        self.last_cabinet_cmd_time = time.monotonic()
        self._handle_twist(msg, "primary")

    def _on_cabinet_mode(self, request, response):
        now = time.monotonic()
        enabled = bool(request.data)
        self.cabinet_mode_active = enabled
        self.cabinet_mode_changed_at = now
        self.last_cabinet_cmd_time = now if enabled else 0.0
        self.last_cmd_time = 0.0
        self.target_vx = 0
        self.target_vy = 0
        self.target_wz = 0
        self._send_motion_mm_s(0, 0, 0, force=True)
        response.success = True
        response.message = (
            "cabinet control lease enabled"
            if enabled
            else "cabinet control lease released"
        )
        self.get_logger().warning(response.message)
        return response

    def _angular_limits_for_source(self, source: str) -> tuple[int, int]:
        angular_limit = abs(int(self.args.max_angular_mrad_s))
        moving_angular_limit = abs(int(self.args.max_moving_angular_mrad_s))
        if source == "mapping":
            if self.args.mapping_max_angular_mrad_s >= 0:
                angular_limit = abs(int(self.args.mapping_max_angular_mrad_s))
            if self.args.mapping_max_moving_angular_mrad_s >= 0:
                moving_angular_limit = abs(
                    int(self.args.mapping_max_moving_angular_mrad_s)
                )
        return angular_limit, moving_angular_limit

    def _handle_twist(self, msg: Twist, source: str):
        self._set_command_profile(source)
        vx = self._clamp(
            round(msg.linear.x * 1000.0 * self.args.cmd_linear_scale * self.args.cmd_vx_sign),
            -self.args.max_linear_mm_s,
            self.args.max_linear_mm_s,
        )
        vy = self._clamp(
            round(msg.linear.y * 1000.0 * self.args.cmd_linear_scale * self.args.cmd_vy_sign),
            -self.args.max_linear_mm_s,
            self.args.max_linear_mm_s,
        )
        wz_raw = round(msg.angular.z * 1000.0 * self.args.cmd_angular_scale * self.args.cmd_wz_sign)
        if self.args.cmd_wz_trim_per_vx != 0.0 and vx != 0:
            wz_raw += round(vx * self.args.cmd_wz_trim_per_vx)
        angular_limit, moving_angular_limit = self._angular_limits_for_source(source)
        if (vx != 0 or vy != 0) and moving_angular_limit > 0:
            angular_limit = min(angular_limit, moving_angular_limit)
        wz = self._clamp(
            wz_raw,
            -angular_limit,
            angular_limit,
        )
        if vx == 0 and vy == 0 and wz != 0:
            minimum = min(
                abs(int(self.args.min_inplace_angular_mrad_s)),
                angular_limit,
            )
            if abs(wz) < minimum:
                wz = minimum if wz > 0 else -minimum
        self.target_vx = vx
        self.target_vy = vy
        self.target_wz = wz
        self.last_cmd_time = time.monotonic()
        if vx != 0 or vy != 0 or wz != 0:
            self.last_nonzero_command_time = self.last_cmd_time
        self._send_motion_mm_s(vx, vy, wz)

    def _on_arm_cmd(self, msg: String):
        now = time.monotonic()
        if now - self.last_arm_cmd_time < max(0.0, self.args.arm_min_interval):
            return
        payload = self._sanitize_arm_payload(str(msg.data))
        if not payload:
            self.arm_cmd_reject_count += 1
            self.get_logger().warning("rejected empty or invalid /arm_cmd payload")
            return
        self._write_command(payload)
        self.arm_cmd_count += 1
        self.last_arm_cmd = payload[:160]
        self.last_arm_cmd_time = now

    def _on_move_arm_pose(self, request, response):
        del response
        result = MoveArmPose.Response()
        self.arm_pose_request_count += 1

        values = (
            float(request.x),
            float(request.y),
            float(request.z),
            float(request.rx),
            float(request.ry),
            float(request.rz),
            float(request.duration),
        )
        if not all(math.isfinite(value) for value in values):
            self.arm_pose_error_count += 1
            result.success = False
            result.error_code = -2
            result.message = "request contains non-finite value"
            return result

        duration_s = float(request.duration)
        if duration_s <= 0.0:
            duration_s = self.args.arm_pose_default_duration
        duration_ms = self._clamp(round(duration_s * 1000.0), 20, 10000)
        command = (
            f"$IK6:{request.x:.2f},{request.y:.2f},{request.z:.2f},"
            f"{request.rx:.2f},{request.ry:.2f},{request.rz:.2f},{duration_ms}!"
        )

        with self._arm_pose_condition:
            if self._arm_pose_inflight:
                self.arm_pose_error_count += 1
                result.success = False
                result.error_code = 5
                result.message = "another arm pose request is in flight"
                return result
            self._arm_pose_inflight = True
            self._pending_ik6_ok = None
            self._pending_ik6_error = None
            self._pending_arm_done_seq = None

        try:
            self.target_vx = 0
            self.target_vy = 0
            self.target_wz = 0
            self._send_motion_mm_s(0, 0, 0, force=True)
            self._write_command(command)
            self.last_arm_pose_command = command
            self.last_arm_cmd = command[:160]
            self.last_arm_cmd_time = time.monotonic()

            accepted = self._wait_for_ik6_result(timeout_s=max(0.2, self.args.arm_pose_accept_timeout))
            if accepted is None:
                self.arm_pose_error_count += 1
                result.success = False
                result.error_code = -1
                result.message = "timeout waiting for F103 @IK6 reply"
                self.last_arm_pose_result = result.message
                return result

            ok_frame, error_frame = accepted
            if error_frame is not None:
                code, reason = error_frame
                self.arm_pose_error_count += 1
                result.success = False
                result.error_code = int(code)
                result.message = reason
                self.last_arm_pose_result = f"ERR {code} {reason}"
                return result

            assert ok_frame is not None
            result.joint_deg = list(ok_frame.joint_deg)
            result.servo_pwm = list(ok_frame.servo_pwm)
            result.actual_duration = float(ok_frame.actual_duration_s)

            if bool(request.wait_done):
                done_timeout = max(
                    self.args.arm_pose_accept_timeout,
                    ok_frame.actual_duration_s + max(0.0, self.args.arm_pose_done_timeout_padding),
                )
                if not self._wait_for_arm_done(ok_frame.seq, timeout_s=done_timeout):
                    self.arm_pose_error_count += 1
                    result.success = False
                    result.error_code = -3
                    result.message = f"timeout waiting for @ARM:DONE seq={ok_frame.seq}"
                    self.last_arm_pose_result = result.message
                    return result

            self.arm_pose_success_count += 1
            result.success = True
            result.error_code = 0
            result.message = "done" if bool(request.wait_done) else "accepted"
            self.last_arm_pose_result = f"OK seq={ok_frame.seq} {result.message}"
            return result
        except Exception as exc:
            self.arm_pose_error_count += 1
            result.success = False
            result.error_code = -99
            result.message = f"bridge exception: {exc}"
            self.last_arm_pose_result = result.message
            self.get_logger().error(result.message)
            return result
        finally:
            with self._arm_pose_condition:
                self._arm_pose_inflight = False
                self._arm_pose_condition.notify_all()

    def _wait_for_ik6_result(self, *, timeout_s: float) -> Optional[tuple[Optional[Ik6OkFrame], Optional[tuple[int, str]]]]:
        deadline = time.monotonic() + max(0.0, timeout_s)
        with self._arm_pose_condition:
            while True:
                if self._pending_ik6_error is not None:
                    return None, self._pending_ik6_error
                if self._pending_ik6_ok is not None:
                    return self._pending_ik6_ok, None
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._arm_pose_condition.wait(timeout=remaining)

    def _wait_for_arm_done(self, seq: int, *, timeout_s: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_s)
        with self._arm_pose_condition:
            while True:
                if self._pending_ik6_error is not None:
                    return False
                if self._pending_arm_done_seq == seq:
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._arm_pose_condition.wait(timeout=remaining)

    @staticmethod
    def _sanitize_arm_payload(payload: str) -> str:
        payload = str(payload or "").strip()
        if not payload:
            return ""
        if len(payload) > 512:
            return ""
        if any(ord(ch) < 32 for ch in payload):
            return ""
        if payload[0] not in ("#", "$", "{"):
            return ""
        if not payload.endswith("!") and not payload.endswith("}"):
            return ""
        if not all(32 <= ord(ch) < 127 for ch in payload):
            return ""
        return payload

    def _command_timer(self):
        now = time.monotonic()
        if self.cabinet_mode_active:
            if now - self.last_cabinet_cmd_time > self.args.cabinet_cmd_timeout:
                self.target_vx = 0
                self.target_vy = 0
                self.target_wz = 0
            self._retry_stalled_startup(now)
            self._send_motion_mm_s(self.target_vx, self.target_vy, self.target_wz)
            return
        if self.last_cmd_time <= 0.0 or now - self.last_cmd_time > self.args.cmd_timeout:
            self.target_vx = 0
            self.target_vy = 0
            self.target_wz = 0
        self._retry_stalled_startup(now)
        self._send_motion_mm_s(self.target_vx, self.target_vy, self.target_wz)

    def _reader_loop(self):
        buffer = b""
        while not self.stop_event.is_set():
            try:
                chunk = self.serial.read(256)
            except Exception as exc:
                self.get_logger().error(f"serial read failed: {exc}")
                time.sleep(0.2)
                continue
            if not chunk:
                continue
            buffer += chunk
            while b"!" in buffer:
                bang_index = buffer.find(b"!")
                frame_bytes = buffer[: bang_index + 1]
                buffer = buffer[bang_index + 1 :]
                text = frame_bytes.decode("ascii", errors="ignore").strip()
                at_index = text.rfind("@")
                if at_index >= 0:
                    self._handle_frame(text[at_index:])
                elif text:
                    self.unknown_count += 1
            if len(buffer) > 2048:
                buffer = buffer[-256:]

    def _handle_frame(self, line: str):
        self.rx_count += 1
        self.last_rx_time = time.monotonic()

        match = ODOM_RE.fullmatch(line)
        if match:
            frame = OdomFrame(
                seq=int(match.group("seq")),
                dt_s=self._safe_dt(int(match.group("dt")) / 1000.0),
                encoder_ticks=(
                    int(match.group("ea")),
                    int(match.group("eb")),
                    int(match.group("ec")),
                    int(match.group("ed")),
                ),
                wheel_mm_s=(
                    int(match.group("wa")),
                    int(match.group("wb")),
                    int(match.group("wc")),
                    int(match.group("wd")),
                ),
            )
            self._handle_odom(frame)
            return

        if line.startswith("@STATE:") or line.startswith("@MODE:") or line == "@PONG!":
            self.last_state = line
            self.state_count += 1
            self._publish_status()
            return

        if TWIST_OK_RE.fullmatch(line):
            self.last_twist_ack = line
            self.twist_ok_count += 1
            return

        if TWIST_ERR_RE.fullmatch(line):
            self.last_twist_ack = line
            self.twist_err_count += 1
            self.get_logger().warning(f"F103 rejected twist: {line}")
            return

        match = IK6_OK_RE.fullmatch(line)
        if match:
            frame = Ik6OkFrame(
                seq=int(match.group("seq")),
                joint_deg=(
                    float(match.group("j0")),
                    float(match.group("j1")),
                    float(match.group("j2")),
                    float(match.group("j3")),
                    float(match.group("j4")),
                    float(match.group("j5")),
                ),
                servo_pwm=(
                    int(match.group("p0")),
                    int(match.group("p1")),
                    int(match.group("p2")),
                    int(match.group("p3")),
                    int(match.group("p4")),
                    int(match.group("p5")),
                ),
                actual_duration_s=int(match.group("t")) / 1000.0,
            )
            self.last_ik6_ok = frame
            self.last_ik6_ok_time = time.monotonic()
            self.last_arm_pose_result = f"IK6_OK seq={frame.seq}"
            with self._arm_pose_condition:
                self._pending_ik6_ok = frame
                self._arm_pose_condition.notify_all()
            return

        match = IK6_ERR_RE.fullmatch(line)
        if match:
            error = (int(match.group("code")), match.group("reason"))
            self.last_ik6_error = error
            self.last_arm_pose_result = f"IK6_ERR code={error[0]} reason={error[1]}"
            with self._arm_pose_condition:
                self._pending_ik6_error = error
                self._arm_pose_condition.notify_all()
            self.get_logger().warning(f"F103 rejected IK6: {line}")
            return

        match = ARM_DONE_RE.fullmatch(line)
        if match:
            seq = int(match.group("seq"))
            self.last_arm_done_seq = seq
            self.last_arm_pose_result = f"ARM_DONE seq={seq}"
            with self._arm_pose_condition:
                self._pending_arm_done_seq = seq
                self._arm_pose_condition.notify_all()
            return

        self.unknown_count += 1
        self.get_logger().debug(f"ignored F103 frame: {line}")

    def _handle_odom(self, frame: OdomFrame):
        if self.last_odom_seq is not None and frame.seq < self.last_odom_seq:
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
        self.last_odom_seq = frame.seq
        if self.odom_count == 0:
            self.get_logger().info(f"first F103 @ODOM received: seq={frame.seq}")

        vx, vy, wz = self._compute_body_twist(frame.wheel_mm_s)
        self._update_startup_hold(frame.wheel_mm_s)
        self._check_motion_consistency(frame.wheel_mm_s, wz)
        dt = frame.dt_s
        self.last_encoder_ticks = frame.encoder_ticks
        self.last_wheel_mm_s = frame.wheel_mm_s
        self.last_body_twist = (vx, vy, wz)

        if self.diag_writer is not None:
            self.diag_writer.writerow([
                f"{time.monotonic():.3f}", *self.target_wheels, *frame.wheel_mm_s,
                f"{self.last_imu_wz:.5f}", f"{vx:.5f}", f"{vy:.5f}", f"{wz:.5f}",
                self.last_motion_warning,
            ])
            if self.odom_count % 10 == 0:
                self.diag_file.flush()

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt
        self.yaw = self._normalize_angle(self.yaw + wz * dt)
        self.odom_count += 1

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.args.odom_frame
        message.child_frame_id = self.args.base_frame
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.position.z = 0.0
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = vx
        message.twist.twist.linear.y = vy
        message.twist.twist.angular.z = wz
        message.pose.covariance = self._pose_covariance()
        message.twist.covariance = self._twist_covariance()
        self.odom_pub.publish(message)
        if self.odom_count % 10 == 0:
            self._publish_status()
        if self.odom_count == 1:
            self.get_logger().info(f"first {self.args.odom_topic} published from F103 @ODOM")

    def _check_motion_consistency(self, wheel_mm_s: tuple[int, int, int, int], wheel_wz: float):
        for index, (target, raw_actual) in enumerate(zip(self.target_wheels, wheel_mm_s)):
            actual = raw_actual * self.odom_wheel_signs[index]
            moving = abs(target) >= self.args.wheel_fault_target_mm_s
            missing = moving and abs(actual) < self.args.wheel_fault_feedback_mm_s
            reversed_feedback = moving and abs(actual) >= self.args.wheel_fault_feedback_mm_s and target * actual < 0
            if missing or reversed_feedback:
                self.wheel_fault_cycles[index] += 1
            else:
                self.wheel_fault_cycles[index] = 0
            if self.wheel_fault_cycles[index] >= self.args.wheel_fault_frames:
                kind = "missing" if missing else "reversed"
                self._report_motion_warning(
                    f"wheel_{'ABCD'[index]}_{kind}:target={target},feedback={actual},raw={raw_actual}"
                )
            overspeed_limit = max(
                self.args.wheel_overspeed_min_mm_s,
                abs(target) * self.args.wheel_overspeed_ratio,
            )
            if moving and abs(actual) > overspeed_limit:
                self.wheel_overspeed_cycles[index] += 1
            else:
                self.wheel_overspeed_cycles[index] = 0
            if self.wheel_overspeed_cycles[index] >= self.args.wheel_overspeed_frames:
                self._report_motion_warning(
                    f"wheel_{'ABCD'[index]}_overspeed:target={target},feedback={actual},raw={raw_actual}"
                )

        imu_fresh = time.monotonic() - self.last_imu_time <= self.args.imu_timeout
        rotating = abs(wheel_wz) >= self.args.angular_fault_min_rad_s
        imu_rotating = abs(self.last_imu_wz) >= self.args.angular_fault_min_rad_s
        if imu_fresh and rotating and imu_rotating and wheel_wz * self.last_imu_wz < 0:
            self.imu_fault_cycles += 1
        else:
            self.imu_fault_cycles = 0
        if self.imu_fault_cycles >= self.args.angular_fault_frames:
            self._report_motion_warning(
                f"wheel_imu_direction_mismatch:wheel_wz={wheel_wz:.4f},imu_wz={self.last_imu_wz:.4f}"
            )

    def _compute_body_twist(self, wheel_mm_s: tuple[int, int, int, int]) -> tuple[float, float, float]:
        v_a = wheel_mm_s[0] * self.odom_wheel_signs[0] / 1000.0
        v_b = wheel_mm_s[1] * self.odom_wheel_signs[1] / 1000.0
        v_c = wheel_mm_s[2] * self.odom_wheel_signs[2] / 1000.0
        v_d = wheel_mm_s[3] * self.odom_wheel_signs[3] / 1000.0
        base = max(1e-6, self.args.wheel_base_m + self.args.axle_base_m)

        vx = (v_a + v_b + v_c + v_d) / 4.0
        vy = (v_a - v_b + v_c - v_d) / 4.0
        wz = (-v_a - v_b + v_c + v_d) / (2.0 * base)

        vx *= self.args.odom_linear_scale * self.args.odom_vx_sign
        vy *= self.args.odom_linear_scale * self.args.odom_vy_sign
        wz *= self.args.odom_angular_scale * self.args.odom_wz_sign
        return vx, vy, wz

    def _current_motion_mode(self, now: Optional[float] = None) -> str:
        now = time.monotonic() if now is None else now
        if self.dcr_blank_until > now:
            if self.dcr_blank_reason == "startup_no_feedback":
                return "startup_retry_blank"
            return "direction_blank"
        if self.startup_hold_active:
            return "startup_hold"
        if self.target_vx == 0 and self.target_vy == 0 and self.target_wz == 0:
            return "stopped"
        if self.target_vx == 0 and self.target_vy == 0:
            return "inplace_rotate"
        if self.target_vy != 0:
            return "lateral_arc" if self.target_wz != 0 else "lateral"
        if self.target_vx < 0:
            return "reverse_arc" if self.target_wz != 0 else "reverse"
        return "forward_arc" if self.target_wz != 0 else "forward"

    def _publish_status(self):
        now = time.monotonic()
        msg = String()
        msg.data = json.dumps(
            {
                "port": self.args.port,
                "rx_count": self.rx_count,
                "odom_count": self.odom_count,
                "state_count": self.state_count,
                "twist_ok_count": self.twist_ok_count,
                "twist_err_count": self.twist_err_count,
                "unknown_count": self.unknown_count,
                "arm_cmd_count": self.arm_cmd_count,
                "arm_cmd_reject_count": self.arm_cmd_reject_count,
                "arm_pose_request_count": self.arm_pose_request_count,
                "arm_pose_success_count": self.arm_pose_success_count,
                "arm_pose_error_count": self.arm_pose_error_count,
                "cmd_output_mode": self.args.cmd_output_mode,
                "active_command_source": self.active_command_source,
                "motion_mode": self._current_motion_mode(now),
                "cmd_wheel_signs": list(self.cmd_wheel_signs),
                "odom_wheel_signs": list(self.odom_wheel_signs),
                "active_inplace_wheel_min_mm_s": list(
                    self.active_inplace_wheel_min_mm_s
                ),
                "active_inplace_startup_boost_duration_sec": round(
                    self.active_inplace_startup_boost_duration_sec,
                    3,
                ),
                "last_arm_cmd": self.last_arm_cmd,
                "last_arm_cmd_age_s": None
                if self.last_arm_cmd_time <= 0.0
                else round(time.monotonic() - self.last_arm_cmd_time, 3),
                "last_arm_pose_command": self.last_arm_pose_command,
                "last_arm_pose_result": self.last_arm_pose_result,
                "last_ik6_ok": None
                if self.last_ik6_ok is None
                else {
                    "seq": self.last_ik6_ok.seq,
                    "joint_deg": [round(value, 3) for value in self.last_ik6_ok.joint_deg],
                    "servo_pwm": list(self.last_ik6_ok.servo_pwm),
                    "actual_duration_s": round(self.last_ik6_ok.actual_duration_s, 3),
                },
                "last_ik6_ok_age_s": None
                if self.last_ik6_ok_time <= 0.0
                else round(time.monotonic() - self.last_ik6_ok_time, 3),
                "last_ik6_error": None
                if self.last_ik6_error is None
                else {"code": self.last_ik6_error[0], "reason": self.last_ik6_error[1]},
                "last_arm_done_seq": self.last_arm_done_seq,
                "last_state": self.last_state,
                "last_twist_ack": self.last_twist_ack,
                "last_odom_seq": self.last_odom_seq,
                "last_rx_age_s": None if self.last_rx_time <= 0.0 else round(time.monotonic() - self.last_rx_time, 3),
                "target_vx_mm_s": self.target_vx,
                "target_vy_mm_s": self.target_vy,
                "target_wz_mrad_s": self.target_wz,
                "target_wheels_mm_s": list(self.target_wheels),
                "cabinet_mode_active": self.cabinet_mode_active,
                "cabinet_cmd_age_s": None
                if self.last_cabinet_cmd_time <= 0.0
                else round(now - self.last_cabinet_cmd_time, 3),
                "requested_wheels_mm_s": list(self.requested_wheels),
                "startup_hold_active": self.startup_hold_active,
                "startup_hold_wheels_mm_s": list(self.startup_hold_wheels),
                "startup_retry_count": self.startup_retry_count,
                "controlled_startup_retry_count": self.controlled_startup_retry_count,
                "startup_boost_remaining_sec": max(
                    0.0, self.startup_boost_deadline - time.monotonic()
                ),
                "dcr_blank_remaining_sec": max(
                    0.0, self.dcr_blank_until - time.monotonic()
                ),
                "dcr_blank_pending_wheels_mm_s": list(self.dcr_blank_pending_wheels),
                "dcr_blank_reason": self.dcr_blank_reason,
                "direction_blank_count": self.direction_blank_count,
                "dcr_write_count": self.dcr_write_count,
                "last_nonzero_command_age_s": None
                if self.last_nonzero_command_time <= 0.0
                else round(now - self.last_nonzero_command_time, 3),
                "inplace_direction_lock": self.inplace_direction_lock,
                "inplace_reverse_pending_direction": self.inplace_reverse_pending_direction,
                "inplace_reverse_pending_age_sec": None
                if self.inplace_reverse_pending_since <= 0.0
                else round(time.monotonic() - self.inplace_reverse_pending_since, 3),
                "last_encoder_ticks": list(self.last_encoder_ticks),
                "last_wheel_mm_s": list(self.last_wheel_mm_s),
                "last_body_twist": {
                    "vx": round(self.last_body_twist[0], 4),
                    "vy": round(self.last_body_twist[1], 4),
                    "wz": round(self.last_body_twist[2], 4),
                },
                "pose_x": round(self.x, 4),
                "pose_y": round(self.y, 4),
                "yaw": round(self.yaw, 4),
                "motion_fault": self.motion_fault,
                "odom_trusted": not bool(self.motion_fault),
                "last_motion_warning": self.last_motion_warning,
                "wheel_fault_cycles": list(self.wheel_fault_cycles),
                "wheel_overspeed_cycles": list(self.wheel_overspeed_cycles),
                "last_imu_wz": round(self.last_imu_wz, 5),
            },
            ensure_ascii=True,
        )
        self.state_pub.publish(msg)

    @staticmethod
    def _pose_covariance() -> list[float]:
        cov = [0.0] * 36
        cov[0] = 0.04
        cov[7] = 0.04
        cov[14] = 999.0
        cov[21] = 999.0
        cov[28] = 999.0
        cov[35] = 0.08
        return cov

    @staticmethod
    def _twist_covariance() -> list[float]:
        cov = [0.0] * 36
        cov[0] = 0.05
        cov[7] = 0.05
        cov[14] = 999.0
        cov[21] = 999.0
        cov[28] = 999.0
        cov[35] = 0.08
        return cov

    @staticmethod
    def _safe_dt(dt_s: float) -> float:
        if dt_s < 0.001 or dt_s > 0.2:
            return 0.02
        return dt_s

    @staticmethod
    def _normalize_angle(value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    @staticmethod
    def _clamp(value: int, low: int, high: int) -> int:
        return max(low, min(high, int(value)))

    @staticmethod
    def _parse_wheel_signs(value: str, name: str) -> tuple[int, int, int, int]:
        parts = [part.strip() for part in str(value or "").split(",")]
        if len(parts) != 4:
            raise SystemExit(f"{name} must contain four comma-separated signs, for example 1,1,-1,-1")
        signs: list[int] = []
        for part in parts:
            try:
                sign = int(part)
            except ValueError as exc:
                raise SystemExit(f"{name} contains non-integer sign: {part}") from exc
            if sign not in (-1, 1):
                raise SystemExit(f"{name} only accepts -1 or 1, got: {part}")
            signs.append(sign)
        return (signs[0], signs[1], signs[2], signs[3])

    @staticmethod
    def _parse_wheel_minimums(value: str, name: str) -> tuple[int, int, int, int]:
        parts = [part.strip() for part in str(value or "").split(",")]
        if len(parts) != 4:
            raise SystemExit(f"{name} must contain four comma-separated non-negative values")
        try:
            minimums = [max(0, int(part)) for part in parts]
        except ValueError as exc:
            raise SystemExit(f"{name} contains a non-integer value") from exc
        return (minimums[0], minimums[1], minimums[2], minimums[3])

    @staticmethod
    def _resolve_serial_port(port: str) -> str:
        if port and os.path.exists(port):
            return port
        detected = F103UsbRos2Bridge._detect_f103_port()
        if detected:
            return detected
        raise SystemExit(
            f"F103 serial port not found: {port}. "
            "Connect the F103 USB serial or pass --port explicitly. "
            "The auto detector excludes /dev/rplidar and /dev/myimu."
        )

    @staticmethod
    def _detect_f103_port() -> str:
        excluded: set[str] = set()
        for link in ("/dev/laser", "/dev/imu", "/dev/rplidar", "/dev/myimu"):
            if os.path.exists(link):
                try:
                    excluded.add(os.path.realpath(link))
                except Exception:
                    pass

        candidates_by_real: dict[str, str] = {}
        for pattern in ("/dev/f103", "/dev/f103_usb", "/dev/serial/by-path/*", "/dev/ttyUSB*", "/dev/ttyACM*"):
            for candidate in glob.glob(pattern):
                try:
                    real = os.path.realpath(candidate)
                except Exception:
                    continue
                if real in excluded:
                    continue
                candidates_by_real.setdefault(real, candidate)
        candidates = list(candidates_by_real.values())
        return candidates[0] if len(candidates) == 1 else ""


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Linux USB serial bridge from ROS2 Twist to F103; publishes internal F103 odom on /odom_raw."
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--node-name", default="f103_usb_ros2_bridge")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--extra-cmd-topic", default="/cmd_vel_cmd")
    parser.add_argument("--mapping-cmd-topic", default="")
    parser.add_argument("--cabinet-cmd-topic", default="/cmd_vel_cabinet")
    parser.add_argument(
        "--cabinet-mode-service",
        default="/f103_usb_ros2_bridge/cabinet_mode",
    )
    parser.add_argument("--cabinet-cmd-timeout", type=float, default=0.35)
    parser.add_argument("--arm-topic", default="/arm_cmd")
    parser.add_argument("--disable-arm-topic", action="store_true")
    parser.add_argument("--arm-min-interval", type=float, default=0.02)
    parser.add_argument("--arm-pose-service", default="/arm/move_pose")
    parser.add_argument("--disable-arm-pose-service", action="store_true")
    parser.add_argument("--arm-pose-default-duration", type=float, default=1.5)
    parser.add_argument("--arm-pose-accept-timeout", type=float, default=2.0)
    parser.add_argument("--arm-pose-done-timeout-padding", type=float, default=1.0)
    parser.add_argument("--odom-topic", default="/odom_raw")
    parser.add_argument("--state-topic", default="/f103_state")
    parser.add_argument("--motion-fault-topic", default="/f103_motion_fault")
    parser.add_argument("--imu-topic", default="/imu/data_calibrated")
    parser.add_argument("--motion-diag-dir", default="/home/rock/Desktop/rt_direct_tools/logs/navigation_motion")
    parser.add_argument("--wheel-fault-target-mm-s", type=int, default=40)
    parser.add_argument("--wheel-fault-feedback-mm-s", type=int, default=3)
    parser.add_argument("--wheel-fault-frames", type=int, default=25)
    parser.add_argument("--wheel-overspeed-ratio", type=float, default=3.0)
    parser.add_argument("--wheel-overspeed-min-mm-s", type=int, default=120)
    parser.add_argument("--wheel-overspeed-frames", type=int, default=3)
    parser.add_argument("--angular-fault-min-rad-s", type=float, default=0.04)
    parser.add_argument("--angular-fault-frames", type=int, default=15)
    parser.add_argument("--imu-timeout", type=float, default=0.5)
    parser.add_argument("--motion-fault-clear-zero-duration", type=float, default=1.0)
    parser.add_argument("--motion-warning-period-sec", type=float, default=1.0)
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--command-rate-hz", type=float, default=10.0)
    # The Nav2 controller runs at 3 Hz and can slip by one cycle during global
    # replanning. Keep immediate explicit zero handling, but tolerate that jitter.
    parser.add_argument("--cmd-timeout", type=float, default=1.0)
    parser.add_argument("--zero-repeat-period", type=float, default=0.5)
    parser.add_argument("--dcr-change-threshold-mm-s", type=int, default=10)
    parser.add_argument("--dcr-max-update-hz", type=float, default=2.0)
    parser.add_argument("--dcr-max-stale-sec", type=float, default=1.0)
    parser.add_argument("--startup-feedback-mm-s", type=int, default=2)
    parser.add_argument("--startup-feedback-frames", type=int, default=2)
    parser.add_argument("--startup-boost-wheel-mm-s", type=int, default=0)
    parser.add_argument("--startup-boost-duration-sec", type=float, default=0.3)
    parser.add_argument("--inplace-startup-boost-wheel-mm-s", type=int, default=0)
    parser.add_argument("--inplace-startup-boost-duration-sec", type=float, default=0.8)
    parser.add_argument(
        "--mapping-inplace-startup-boost-wheel-mm-s",
        type=int,
        default=0,
    )
    parser.add_argument(
        "--mapping-inplace-startup-boost-duration-sec",
        type=float,
        default=-1.0,
    )
    parser.add_argument("--inplace-startup-feedback-mm-s", type=int, default=12)
    parser.add_argument("--inplace-startup-feedback-frames", type=int, default=6)
    parser.add_argument("--inplace-startup-balance-ratio", type=float, default=0.45)
    parser.add_argument("--inplace-reverse-confirm-sec", type=float, default=0.0)
    parser.add_argument("--startup-retry-period", type=float, default=1.5)
    parser.add_argument("--startup-retry-max", type=int, default=0)
    parser.add_argument("--startup-retry-zero-sec", type=float, default=0.15)
    parser.add_argument("--dcr-direction-blank-sec", type=float, default=0.15)
    parser.add_argument("--status-period", type=float, default=1.0)
    parser.add_argument("--handshake-interval", type=float, default=0.08)
    parser.add_argument("--keep-input-buffer", action="store_true", default=True)
    parser.add_argument("--clear-input-buffer", dest="keep_input_buffer", action="store_false")
    parser.add_argument("--flush-serial-writes", action="store_true", default=False)
    parser.add_argument("--max-linear-mm-s", type=int, default=900)
    parser.add_argument("--max-angular-mrad-s", type=int, default=900)
    parser.add_argument("--max-moving-angular-mrad-s", type=int, default=0)
    parser.add_argument("--mapping-max-angular-mrad-s", type=int, default=-1)
    parser.add_argument("--mapping-max-moving-angular-mrad-s", type=int, default=-1)
    parser.add_argument("--min-inplace-angular-mrad-s", type=int, default=0)
    parser.add_argument("--wheel-base-m", type=float, default=WHEEL_BASE_M)
    parser.add_argument("--axle-base-m", type=float, default=AXLE_BASE_M)
    parser.add_argument("--cmd-output-mode", choices=("twist", "dcr"), default="twist")
    parser.add_argument("--cmd-wheel-signs", default="1,1,1,1")
    parser.add_argument("--inplace-wheel-min-mm-s", default="0,0,0,0")
    parser.add_argument(
        "--mapping-inplace-wheel-min-mm-s",
        default="0,0,0,0",
    )
    parser.add_argument("--odom-wheel-signs", default="1,1,1,1")
    parser.add_argument("--cmd-linear-scale", type=float, default=1.0)
    parser.add_argument("--cmd-angular-scale", type=float, default=1.0)
    parser.add_argument("--cmd-vx-sign", type=float, default=1.0)
    parser.add_argument("--cmd-vy-sign", type=float, default=1.0)
    parser.add_argument("--cmd-wz-sign", type=float, default=1.0)
    parser.add_argument("--cmd-wz-trim-per-vx", type=float, default=0.0)
    parser.add_argument("--firmware-vx-sign", type=float, default=-1.0)
    parser.add_argument("--firmware-vy-sign", type=float, default=1.0)
    parser.add_argument("--firmware-wz-sign", type=float, default=1.0)
    parser.add_argument("--odom-linear-scale", type=float, default=1.0)
    parser.add_argument("--odom-angular-scale", type=float, default=1.0)
    parser.add_argument("--odom-vx-sign", type=float, default=1.0)
    parser.add_argument("--odom-vy-sign", type=float, default=1.0)
    parser.add_argument("--odom-wz-sign", type=float, default=1.0)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    rclpy.init()
    node = None
    try:
        node = F103UsbRos2Bridge(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
