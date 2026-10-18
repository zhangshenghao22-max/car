#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple


TRACK = "TRACK"
TURN_PATH = "TURN_PATH"
TURN_GOAL = "TURN_GOAL"
SETTLE = "SETTLE"
FAULT = "FAULT"

PATH_INACTIVE = "inactive"
PATH_WAITING = "waiting_for_goal_path"
PATH_FRESH = "fresh"
PATH_SOFT_STALE = "soft_stale"
PATH_HARD_STALE = "hard_stale"


class PathFreshnessTracker:
    def __init__(self, soft_timeout: float, hard_timeout: float) -> None:
        if soft_timeout <= 0.0 or hard_timeout <= soft_timeout:
            raise ValueError("path freshness timeouts must satisfy 0 < soft < hard")
        self.soft_timeout = float(soft_timeout)
        self.hard_timeout = float(hard_timeout)
        self.goal_id: Optional[str] = None
        self.path_goal_id: Optional[str] = None
        self.path_received_at = -1.0
        self.state = PATH_INACTIVE
        self.accepted_count = 0
        self.rejected_count = 0
        self.soft_stale_count = 0
        self.hard_stale_count = 0
        self.soft_stale_started_at = -1.0
        self.soft_stale_total_sec = 0.0

    def _finish_soft_stale(self, now: float) -> None:
        if self.soft_stale_started_at >= 0.0:
            self.soft_stale_total_sec += max(0.0, now - self.soft_stale_started_at)
            self.soft_stale_started_at = -1.0

    def set_goal(self, goal_id: Optional[str], now: float) -> bool:
        if goal_id == self.goal_id:
            return False
        self._finish_soft_stale(now)
        self.goal_id = goal_id
        self.path_goal_id = None
        self.path_received_at = -1.0
        self.state = PATH_WAITING if goal_id else PATH_INACTIVE
        return True

    def accept_path(self, goal_id: Optional[str], now: float) -> bool:
        if not goal_id or goal_id != self.goal_id:
            self.rejected_count += 1
            return False
        self._finish_soft_stale(now)
        self.path_goal_id = goal_id
        self.path_received_at = now
        self.state = PATH_FRESH
        self.accepted_count += 1
        return True

    def evaluate(self, now: float) -> Tuple[str, Optional[float]]:
        if not self.goal_id:
            next_state = PATH_INACTIVE
            age = None
        elif self.path_goal_id != self.goal_id or self.path_received_at < 0.0:
            next_state = PATH_WAITING
            age = None
        else:
            age = max(0.0, now - self.path_received_at)
            if age > self.hard_timeout:
                next_state = PATH_HARD_STALE
            elif age > self.soft_timeout:
                next_state = PATH_SOFT_STALE
            else:
                next_state = PATH_FRESH

        previous_state = self.state
        if next_state != previous_state:
            if next_state == PATH_SOFT_STALE:
                self.soft_stale_count += 1
                self.soft_stale_started_at = now
            elif next_state == PATH_HARD_STALE:
                self.hard_stale_count += 1
                if self.soft_stale_started_at < 0.0:
                    self.soft_stale_started_at = now
            elif next_state not in (PATH_SOFT_STALE, PATH_HARD_STALE):
                self._finish_soft_stale(now)
            self.state = next_state
        return self.state, age

    def snapshot(self, now: float) -> Dict[str, object]:
        stale_total = self.soft_stale_total_sec
        if self.soft_stale_started_at >= 0.0:
            stale_total += max(0.0, now - self.soft_stale_started_at)
        return {
            "state": self.state,
            "goal_id": self.goal_id,
            "path_goal_id": self.path_goal_id,
            "path_age": None
            if self.path_received_at < 0.0
            else max(0.0, now - self.path_received_at),
            "soft_timeout_sec": self.soft_timeout,
            "hard_timeout_sec": self.hard_timeout,
            "accepted_count": self.accepted_count,
            "rejected_count": self.rejected_count,
            "soft_stale_count": self.soft_stale_count,
            "hard_stale_count": self.hard_stale_count,
            "soft_stale_total_sec": stale_total,
        }


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def path_target(
    points: Sequence[Tuple[float, float, float]],
    robot_x: float,
    robot_y: float,
    lookahead: float,
) -> Optional[Tuple[float, float, float]]:
    """Return path tangent yaw, Euclidean goal distance and remaining path length."""
    if len(points) < 2:
        return None

    nearest = min(
        range(len(points)),
        key=lambda index: (points[index][0] - robot_x) ** 2
        + (points[index][1] - robot_y) ** 2,
    )
    target_index = nearest
    walked = 0.0
    for index in range(nearest + 1, len(points)):
        walked += math.hypot(
            points[index][0] - points[index - 1][0],
            points[index][1] - points[index - 1][1],
        )
        target_index = index
        if walked >= lookahead:
            break

    start_x, start_y, _ = points[nearest]
    target_x, target_y, _ = points[target_index]
    if math.hypot(target_x - start_x, target_y - start_y) < 1e-4:
        tangent_yaw = points[-1][2]
    else:
        tangent_yaw = math.atan2(target_y - start_y, target_x - start_x)

    remaining = 0.0
    for index in range(nearest + 1, len(points)):
        remaining += math.hypot(
            points[index][0] - points[index - 1][0],
            points[index][1] - points[index - 1][1],
        )

    goal_x, goal_y, _ = points[-1]
    goal_distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
    return tangent_yaw, goal_distance, remaining


@dataclass
class TurnDecision:
    angular_z: float
    state: str
    reason: str
    completed: bool = False
    faulted: bool = False


@dataclass
class PostTurnDecision:
    active: bool
    allow_turn: bool
    reason: str
    elapsed: float
    distance: float


class PostTurnGuard:
    def __init__(
        self,
        hold_sec: float,
        hold_distance: float,
        reentry_error: float,
        reentry_delay: float,
    ) -> None:
        self.hold_sec = hold_sec
        self.hold_distance = hold_distance
        self.reentry_error = reentry_error
        self.reentry_delay = reentry_delay
        self.active = False
        self.started_at = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.elapsed = 0.0
        self.distance = 0.0
        self.last_reason = "inactive"

    def begin(self, robot_x: float, robot_y: float, now: float) -> None:
        self.active = True
        self.started_at = now
        self.start_x = robot_x
        self.start_y = robot_y
        self.elapsed = 0.0
        self.distance = 0.0
        self.last_reason = "post_turn_started"

    def cancel(self, reason: str) -> None:
        self.active = False
        self.last_reason = reason

    def update(
        self,
        robot_x: float,
        robot_y: float,
        heading_error: float,
        now: float,
    ) -> PostTurnDecision:
        if not self.active:
            return PostTurnDecision(
                False, True, self.last_reason, self.elapsed, self.distance
            )

        self.elapsed = max(0.0, now - self.started_at)
        self.distance = math.hypot(robot_x - self.start_x, robot_y - self.start_y)

        if (
            self.elapsed >= self.reentry_delay
            and abs(heading_error) >= self.reentry_error
        ):
            self.cancel("new_corner")
            return PostTurnDecision(
                False, True, self.last_reason, self.elapsed, self.distance
            )
        if self.distance >= self.hold_distance:
            self.cancel("distance_complete")
            return PostTurnDecision(
                False, True, self.last_reason, self.elapsed, self.distance
            )
        if self.elapsed >= self.hold_sec:
            self.cancel("time_complete")
            return PostTurnDecision(
                False, True, self.last_reason, self.elapsed, self.distance
            )

        self.last_reason = "holding_forward"
        return PostTurnDecision(
            True, False, self.last_reason, self.elapsed, self.distance
        )


class TurnController:
    def __init__(self) -> None:
        self.state = TRACK
        self.turn_type = "none"
        self.target_yaw = 0.0
        self.direction = 0
        self.tolerance = 0.0
        self.started_at = 0.0
        self.last_yaw = 0.0
        self.signed_progress = 0.0
        self.opposite_progress = 0.0
        self.progress_checkpoint = 0.0
        self.progress_checked_at = 0.0
        self.settle_started_at = 0.0
        self.settle_completion_reason = "turn_complete"
        self.low_speed_since: Optional[float] = None
        self.paused_at: Optional[float] = None
        self.fault_reason = ""
        self.last_recovery_reason = ""

    def reset(self) -> None:
        self.__init__()

    def begin(self, turn_type: str, current_yaw: float, target_yaw: float, now: float) -> bool:
        error = normalize_angle(target_yaw - current_yaw)
        if abs(error) < 1e-6:
            return False
        self.state = TURN_GOAL if turn_type == "goal" else TURN_PATH
        self.turn_type = turn_type
        self.target_yaw = normalize_angle(target_yaw)
        self.direction = 1 if error > 0.0 else -1
        self.tolerance = math.radians(5.0 if turn_type == "goal" else 12.0)
        self.started_at = now
        self.last_yaw = current_yaw
        self.signed_progress = 0.0
        self.opposite_progress = 0.0
        self.progress_checkpoint = 0.0
        self.progress_checked_at = now
        self.fault_reason = ""
        self.last_recovery_reason = ""
        return True

    def _fault(self, reason: str) -> TurnDecision:
        self.state = FAULT
        self.fault_reason = reason
        return TurnDecision(0.0, self.state, reason, faulted=True)

    def _complete(self, reason: str) -> TurnDecision:
        self.state = TRACK
        self.turn_type = "none"
        self.direction = 0
        self.fault_reason = ""
        return TurnDecision(0.0, self.state, reason, completed=True)

    def _recover(self, reason: str, angular_speed: float, now: float) -> TurnDecision:
        self.state = SETTLE
        self.settle_started_at = now
        self.settle_completion_reason = reason + "_recovered"
        self.low_speed_since = now if abs(angular_speed) < 0.03 else None
        self.last_recovery_reason = reason
        return TurnDecision(0.0, self.state, reason + "_stop")

    def pause_timers(self, now: float) -> None:
        if self.state in (TURN_PATH, TURN_GOAL, SETTLE) and self.paused_at is None:
            self.paused_at = now

    def resume_timers(self, now: float) -> None:
        if self.paused_at is None:
            return

        paused_for = max(0.0, now - self.paused_at)
        self.started_at += paused_for
        self.progress_checked_at += paused_for
        if self.state == SETTLE:
            self.settle_started_at += paused_for
            if self.low_speed_since is not None:
                self.low_speed_since += paused_for
        self.paused_at = None

    @staticmethod
    def _speed_for_error(error_abs: float) -> float:
        if error_abs >= math.radians(45.0):
            return 0.22
        if error_abs >= math.radians(20.0):
            return 0.16
        return 0.12

    def update(self, current_yaw: float, angular_speed: float, now: float) -> TurnDecision:
        self.resume_timers(now)
        if self.state == FAULT:
            return TurnDecision(0.0, self.state, self.fault_reason, faulted=True)

        if self.state == SETTLE:
            if abs(angular_speed) < 0.03:
                if self.low_speed_since is None:
                    self.low_speed_since = now
                elif now - self.low_speed_since >= 0.20:
                    return self._complete(self.settle_completion_reason)
            else:
                self.low_speed_since = None

            if now - self.settle_started_at > 1.0:
                if self.settle_completion_reason == "turn_complete":
                    return self._complete("settle_forced_complete")
                return self._complete(self.settle_completion_reason)
            return TurnDecision(0.0, self.state, "settling")

        if self.state not in (TURN_PATH, TURN_GOAL):
            return TurnDecision(0.0, self.state, "tracking")

        yaw_step = normalize_angle(current_yaw - self.last_yaw)
        self.last_yaw = current_yaw
        directed_step = yaw_step * self.direction
        self.signed_progress += directed_step
        if directed_step < 0.0:
            self.opposite_progress += -directed_step

        if self.opposite_progress > math.radians(5.0):
            return self._recover("opposite_rotation", angular_speed, now)
        if now - self.started_at > 20.0:
            return self._recover("turn_timeout", angular_speed, now)

        if self.signed_progress - self.progress_checkpoint >= math.radians(3.0):
            self.progress_checkpoint = self.signed_progress
            self.progress_checked_at = now
        elif now - self.progress_checked_at > 6.0:
            return self._recover("no_heading_progress", angular_speed, now)

        error = normalize_angle(self.target_yaw - current_yaw)
        directed_error = error * self.direction
        if abs(error) <= self.tolerance or directed_error <= 0.0:
            self.state = SETTLE
            self.settle_started_at = now
            self.settle_completion_reason = "turn_complete"
            self.low_speed_since = now if abs(angular_speed) < 0.03 else None
            return TurnDecision(0.0, self.state, "target_reached")

        speed = self._speed_for_error(abs(error))
        return TurnDecision(speed * self.direction, self.state, "turning")
