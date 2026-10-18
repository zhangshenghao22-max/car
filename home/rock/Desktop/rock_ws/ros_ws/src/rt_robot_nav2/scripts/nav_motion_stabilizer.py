#!/usr/bin/env python3
import json
import math
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from nav_motion_stabilizer_core import (
    FAULT,
    PATH_HARD_STALE,
    PATH_SOFT_STALE,
    PATH_WAITING,
    SETTLE,
    TRACK,
    TURN_GOAL,
    TURN_PATH,
    PathFreshnessTracker,
    PostTurnGuard,
    TurnController,
    normalize_angle,
    path_target,
    quaternion_yaw,
)


class NavMotionStabilizer(Node):
    def __init__(self) -> None:
        super().__init__("nav_motion_stabilizer")
        self.declare_parameter("raw_cmd_topic", "/cmd_vel_nav_raw")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter(
            "action_status_topic", "/navigate_to_pose/_action/status"
        )
        self.declare_parameter(
            "status_topic", "/nav_motion_stabilizer/status"
        )
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("path_lookahead_m", 0.25)
        self.declare_parameter("path_turn_enter_deg", 25.0)
        self.declare_parameter("goal_distance_m", 0.20)
        self.declare_parameter("post_turn_hold_sec", 1.20)
        self.declare_parameter("post_turn_hold_distance_m", 0.05)
        self.declare_parameter("post_turn_reentry_deg", 38.0)
        self.declare_parameter("post_turn_reentry_delay_sec", 0.35)
        self.declare_parameter("post_turn_forward_m_s", 0.05)
        self.declare_parameter("post_turn_max_angular_rad_s", 0.06)
        self.declare_parameter("post_turn_disable_goal_distance_m", 0.35)
        self.declare_parameter("raw_timeout_sec", 0.60)
        self.declare_parameter("tf_timeout_sec", 0.80)
        self.declare_parameter("odom_timeout_sec", 0.80)
        self.declare_parameter("path_timeout_sec", 2.50)
        self.declare_parameter("path_hard_timeout_sec", 6.00)

        self.raw_topic = str(self.get_parameter("raw_cmd_topic").value)
        self.output_topic = str(self.get_parameter("output_cmd_topic").value)
        self.path_lookahead = float(self.get_parameter("path_lookahead_m").value)
        self.path_turn_enter = math.radians(
            float(self.get_parameter("path_turn_enter_deg").value)
        )
        self.goal_distance = float(self.get_parameter("goal_distance_m").value)
        self.post_turn_forward = float(
            self.get_parameter("post_turn_forward_m_s").value
        )
        self.post_turn_max_angular = float(
            self.get_parameter("post_turn_max_angular_rad_s").value
        )
        self.post_turn_disable_goal_distance = float(
            self.get_parameter("post_turn_disable_goal_distance_m").value
        )
        self.raw_timeout = float(self.get_parameter("raw_timeout_sec").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
        self.path_timeout = float(self.get_parameter("path_timeout_sec").value)
        self.path_hard_timeout = float(
            self.get_parameter("path_hard_timeout_sec").value
        )
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.state_lock = threading.RLock()
        self.path_freshness = PathFreshnessTracker(
            self.path_timeout, self.path_hard_timeout
        )

        self.control_node = Node(
            self.get_name() + "_control", use_global_arguments=False
        )
        self.cmd_pub = self.control_node.create_publisher(
            Twist, self.output_topic, 10
        )
        self.status_pub = self.control_node.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            Twist,
            self.raw_topic,
            self._raw_callback,
            20,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("path_topic").value),
            self._path_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            20,
        )
        self.create_subscription(
            GoalStatusArray,
            str(self.get_parameter("action_status_topic").value),
            self._action_status_callback,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.controller = TurnController()
        self.post_turn_guard = PostTurnGuard(
            hold_sec=float(self.get_parameter("post_turn_hold_sec").value),
            hold_distance=float(
                self.get_parameter("post_turn_hold_distance_m").value
            ),
            reentry_error=math.radians(
                float(self.get_parameter("post_turn_reentry_deg").value)
            ),
            reentry_delay=float(
                self.get_parameter("post_turn_reentry_delay_sec").value
            ),
        )
        self.latest_raw = Twist()
        self.raw_received_at = -1.0
        self.path_received_at = -1.0
        self.odom_received_at = -1.0
        self.path_frame = "map"
        self.path_points: List[Tuple[float, float, float]] = []
        self.angular_speed = 0.0
        self.active_goal_id: Optional[str] = None
        self.action_seen = False
        self.stop_reason = "waiting_for_goal"
        self.current_yaw: Optional[float] = None
        self.current_target_yaw: Optional[float] = None
        self.current_error: Optional[float] = None
        self.last_status_at = -1.0
        self.control_started = False
        self.status_started = False
        self.status_publish_count = 0
        self.data_hold_reason = ""
        self.data_hold_started_at = -1.0
        self.data_hold_count = 0
        self.data_hold_total_sec = 0.0
        self.post_turn_output = "inactive"
        self.control_period = 1.0 / float(
            self.get_parameter("control_rate_hz").value
        )
        self.control_timer = self.control_node.create_timer(
            self.control_period, self._control_tick
        )
        self.control_executor = SingleThreadedExecutor()
        self.control_executor.add_node(self.control_node)
        self.control_thread = threading.Thread(
            target=self.control_executor.spin,
            name="nav_motion_executor",
            daemon=True,
        )
        self.control_thread.start()
        self.get_logger().info(
            "ready: %s -> %s at %.1f Hz"
            % (self.raw_topic, self.output_topic, 1.0 / self.control_period)
        )

    def _now(self) -> float:
        return time.monotonic()

    @staticmethod
    def _is_zero(cmd: Twist) -> bool:
        return (
            abs(cmd.linear.x) < 1e-6
            and abs(cmd.linear.y) < 1e-6
            and abs(cmd.angular.z) < 1e-6
        )

    @staticmethod
    def _is_spin(cmd: Twist) -> bool:
        return (
            abs(cmd.linear.x) <= 0.01
            and abs(cmd.linear.y) <= 0.01
            and abs(cmd.angular.z) >= 0.05
        )

    def _publish_zero(self, reason: str) -> None:
        self.stop_reason = reason
        self.cmd_pub.publish(Twist())

    def _post_turn_command(self, path_error: float) -> Twist:
        command = Twist()
        command.linear.x = self.post_turn_forward
        command.angular.z = max(
            -self.post_turn_max_angular,
            min(self.post_turn_max_angular, 0.8 * path_error),
        )
        return command

    def _hold_for_data(self, reason: str, now: float) -> None:
        self.controller.pause_timers(now)
        if not self.data_hold_reason:
            self.data_hold_started_at = now
            self.data_hold_count += 1
            self.get_logger().warning("motion paused: %s" % reason)
        elif reason != self.data_hold_reason:
            self.get_logger().warning(
                "motion pause reason changed: %s -> %s"
                % (self.data_hold_reason, reason)
            )
        self.data_hold_reason = reason
        self._publish_zero(reason)

    def _resume_from_data_hold(self, now: float) -> None:
        if self.data_hold_reason:
            self.data_hold_total_sec += max(0.0, now - self.data_hold_started_at)
            self.get_logger().info(
                "motion resumed after %.3fs hold (%s)"
                % (now - self.data_hold_started_at, self.data_hold_reason)
            )
            self.data_hold_reason = ""
            self.data_hold_started_at = -1.0
        self.controller.resume_timers(now)

    def _raw_callback(self, msg: Twist) -> None:
        with self.state_lock:
            self.latest_raw = msg
            self.raw_received_at = self._now()

    def _path_callback(self, msg: Path) -> None:
        points = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
                quaternion_yaw(
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ),
            )
            for pose in msg.poses
        ]
        with self.state_lock:
            now = self._now()
            if not self.path_freshness.accept_path(self.active_goal_id, now):
                self.get_logger().debug("ignored path without an active matching goal")
                return
            self.path_frame = msg.header.frame_id or "map"
            self.path_points = points
            self.path_received_at = now

    def _odom_callback(self, msg: Odometry) -> None:
        with self.state_lock:
            self.angular_speed = msg.twist.twist.angular.z
            self.odom_received_at = self._now()

    @staticmethod
    def _goal_id(status) -> str:
        return bytes(status.goal_info.goal_id.uuid).hex()

    def _action_status_callback(self, msg: GoalStatusArray) -> None:
        with self.state_lock:
            self.action_seen = True
            active = [
                status
                for status in msg.status_list
                if status.status
                in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
            ]
            active.sort(
                key=lambda status: (
                    status.goal_info.stamp.sec,
                    status.goal_info.stamp.nanosec,
                )
            )
            new_goal_id = self._goal_id(active[-1]) if active else None
            if new_goal_id != self.active_goal_id:
                now = self._now()
                if self.data_hold_started_at >= 0.0:
                    self.data_hold_total_sec += max(
                        0.0, now - self.data_hold_started_at
                    )
                self.controller.reset()
                self.post_turn_guard.cancel("goal_changed")
                self.post_turn_output = "inactive"
                self.current_target_yaw = None
                self.current_error = None
                self.data_hold_reason = ""
                self.data_hold_started_at = -1.0
                self.active_goal_id = new_goal_id
                self.path_freshness.set_goal(new_goal_id, now)
                self.path_points = []
                self.path_received_at = -1.0

    def _lookup_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.path_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException as exc:
            self.get_logger().debug("TF unavailable: %s" % exc)
            return None

        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        if stamp > 0.0 and ros_now - stamp > self.tf_timeout:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        return translation.x, translation.y, yaw

    def _publish_status(self, now: float) -> None:
        if self.last_status_at >= 0.0 and now - self.last_status_at < 0.20:
            return
        self.last_status_at = now
        path_status = self.path_freshness.snapshot(now)
        data_hold_total = self.data_hold_total_sec
        if self.data_hold_started_at >= 0.0:
            data_hold_total += max(0.0, now - self.data_hold_started_at)
        payload = {
            "state": self.controller.state,
            "turn_type": self.controller.turn_type,
            "locked_direction": self.controller.direction,
            "target_yaw": self.current_target_yaw,
            "current_yaw": self.current_yaw,
            "heading_error": self.current_error,
            "raw_cmd_age": None
            if self.raw_received_at < 0.0
            else now - self.raw_received_at,
            "path_age": path_status["path_age"],
            "path_freshness_state": path_status["state"],
            "path_goal_id": path_status["path_goal_id"],
            "awaiting_goal_path": path_status["state"] == PATH_WAITING,
            "path_soft_timeout_sec": path_status["soft_timeout_sec"],
            "path_hard_timeout_sec": path_status["hard_timeout_sec"],
            "path_accepted_count": path_status["accepted_count"],
            "path_rejected_count": path_status["rejected_count"],
            "path_soft_stale_count": path_status["soft_stale_count"],
            "path_hard_stale_count": path_status["hard_stale_count"],
            "path_soft_stale_total_sec": path_status["soft_stale_total_sec"],
            "odom_age": None
            if self.odom_received_at < 0.0
            else now - self.odom_received_at,
            "action_active": self.active_goal_id is not None,
            "goal_id": self.active_goal_id,
            "stop_reason": self.stop_reason,
            "fault_reason": self.controller.fault_reason,
            "tf_timeout_sec": self.tf_timeout,
            "odom_timeout_sec": self.odom_timeout,
            "data_hold_reason": self.data_hold_reason,
            "data_hold_age": None
            if self.data_hold_started_at < 0.0
            else now - self.data_hold_started_at,
            "data_hold_count": self.data_hold_count,
            "data_hold_total_sec": data_hold_total,
            "turn_timers_paused": self.controller.paused_at is not None,
            "turn_recovery_reason": self.controller.last_recovery_reason,
            "post_turn_active": self.post_turn_guard.active,
            "post_turn_reason": self.post_turn_guard.last_reason,
            "post_turn_age": self.post_turn_guard.elapsed,
            "post_turn_distance": self.post_turn_guard.distance,
            "post_turn_output": self.post_turn_output,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.status_pub.publish(message)
        self.status_publish_count += 1
        if not self.status_started:
            self.status_started = True
            self.get_logger().info("status publishing started")
        elif self.status_publish_count % 500 == 0:
            self.get_logger().info(
                "status heartbeat count=%d state=%s"
                % (self.status_publish_count, self.controller.state)
            )

    def _control_tick(self) -> None:
        with self.state_lock:
            now = self._now()
            if not self.control_started:
                self.control_started = True
                self.get_logger().info("control loop started")
            if not self.action_seen or self.active_goal_id is None:
                self.post_turn_guard.cancel("action_inactive")
                self.post_turn_output = "inactive"
                self._publish_zero("action_inactive")
                self._publish_status(now)
                return
            if self.raw_received_at < 0.0 or now - self.raw_received_at > self.raw_timeout:
                self._hold_for_data("raw_timeout", now)
                self._publish_status(now)
                return

            previous_path_state = self.path_freshness.state
            path_state, path_age = self.path_freshness.evaluate(now)
            if path_state != previous_path_state:
                if path_state == PATH_SOFT_STALE:
                    self.get_logger().warning(
                        "path is soft-stale at %.3fs; continuing with the current goal path"
                        % float(path_age or 0.0)
                    )
                elif path_state == PATH_HARD_STALE:
                    self.get_logger().error(
                        "path is hard-stale at %.3fs; stopping motion"
                        % float(path_age or 0.0)
                    )
                elif previous_path_state in (PATH_SOFT_STALE, PATH_HARD_STALE):
                    self.get_logger().info("fresh path received; path hold cleared")
            if path_state == PATH_WAITING:
                self._hold_for_data("waiting_for_goal_path", now)
                self._publish_status(now)
                return
            if path_state == PATH_HARD_STALE:
                self._hold_for_data("path_hard_stale", now)
                self._publish_status(now)
                return
            if self._is_zero(self.latest_raw):
                self._resume_from_data_hold(now)
                self.controller.pause_timers(now)
                self._publish_zero("raw_zero")
                self._publish_status(now)
                return

            pose = self._lookup_pose()
            if pose is None:
                self._hold_for_data("tf_stale", now)
                self._publish_status(now)
                return
            robot_x, robot_y, current_yaw = pose
            self.current_yaw = current_yaw
            target_info = path_target(
                self.path_points, robot_x, robot_y, self.path_lookahead
            )
            if target_info is None:
                self._hold_for_data("path_invalid", now)
                self._publish_status(now)
                return
            path_yaw, goal_distance, _ = target_info
            path_error = normalize_angle(path_yaw - current_yaw)

            self._resume_from_data_hold(now)

            if goal_distance <= self.post_turn_disable_goal_distance:
                self.post_turn_guard.cancel("goal_proximity")
                self.post_turn_output = "inactive"

            if self.controller.state == FAULT:
                self._publish_zero(self.controller.fault_reason)
                self._publish_status(now)
                return

            if self.controller.state == TRACK:
                guard = self.post_turn_guard.update(
                    robot_x, robot_y, path_error, now
                )
                if guard.active:
                    self.current_target_yaw = path_yaw
                    self.current_error = path_error
                    if self.latest_raw.linear.x < -0.01:
                        self.post_turn_output = "reverse_blocked"
                        self._publish_zero("post_turn_reverse_blocked")
                    elif self.latest_raw.linear.x <= 0.01:
                        self.post_turn_output = "forward_override"
                        self.stop_reason = ""
                        self.cmd_pub.publish(self._post_turn_command(path_error))
                    else:
                        self.post_turn_output = "nav_forward"
                        self.stop_reason = ""
                        self.cmd_pub.publish(self.latest_raw)
                    self._publish_status(now)
                    return

                self.post_turn_output = "inactive"
                if self._is_spin(self.latest_raw):
                    final_turn = goal_distance <= self.goal_distance
                    target_yaw = self.path_points[-1][2] if final_turn else path_yaw
                    error = normalize_angle(target_yaw - current_yaw)
                    enter = abs(error) > (
                        math.radians(5.0) if final_turn else self.path_turn_enter
                    )
                    if enter and self.controller.begin(
                        "goal" if final_turn else "path",
                        current_yaw,
                        target_yaw,
                        now,
                    ):
                        self.current_target_yaw = target_yaw
                        self.current_error = error

                if self.controller.state == TRACK:
                    self.stop_reason = ""
                    self.cmd_pub.publish(self.latest_raw)
                    self._publish_status(now)
                    return

            self.current_target_yaw = self.controller.target_yaw
            self.current_error = normalize_angle(
                self.controller.target_yaw - current_yaw
            )
            if self.controller.state == SETTLE and (
                self.odom_received_at < 0.0
                or now - self.odom_received_at > self.odom_timeout
            ):
                self._hold_for_data("odom_stale", now)
                self._publish_status(now)
                return

            previous_state = self.controller.state
            previous_turn_type = self.controller.turn_type
            decision = self.controller.update(current_yaw, self.angular_speed, now)
            if decision.faulted and previous_state != FAULT:
                self.get_logger().error(
                    "turn fault: reason=%s type=%s direction=%d error=%.3f"
                    % (
                        decision.reason,
                        self.controller.turn_type,
                        self.controller.direction,
                        self.current_error,
                    )
                )
            if decision.state in (TURN_PATH, TURN_GOAL):
                command = Twist()
                command.angular.z = decision.angular_z
                self.stop_reason = ""
                self.cmd_pub.publish(command)
            else:
                self._publish_zero(decision.reason)
                if (
                    decision.completed
                    and decision.reason == "turn_complete"
                    and previous_turn_type == "path"
                    and goal_distance > self.post_turn_disable_goal_distance
                ):
                    self.post_turn_guard.begin(robot_x, robot_y, now)
                    self.post_turn_output = "armed"
            self._publish_status(now)

    def stop(self) -> None:
        self.control_timer.cancel()
        for _ in range(5):
            self._publish_zero("shutdown")
            time.sleep(0.02)
        self.control_executor.shutdown()
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
        self.control_node.destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavMotionStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
