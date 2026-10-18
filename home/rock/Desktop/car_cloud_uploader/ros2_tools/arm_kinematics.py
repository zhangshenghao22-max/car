from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np


KINEMATIC_JOINT_COUNT = 5
SERVO_COUNT = 6
KINEMATIC_MODEL_VERSION = "arm5_camera_before_s4_v2"
CAMERA_MOUNT_FRAME = "camera_mount_before_s4"


@dataclass(frozen=True)
class ArmConfig:
    l0_mm: float = 100.0
    l1_mm: float = 105.0
    l2_mm: float = 88.0
    tool_length_mm: float = 155.0
    home_pwm: tuple[int, int, int, int, int, int] = (1600, 1500, 1500, 1500, 1500, 1500)
    direction: tuple[int, int, int, int, int, int] = (-1, 1, 1, 1, 1, 1)
    pwm_per_deg: tuple[float, float, float, float, float, float] = (
        2000.0 / 270.0,
        2000.0 / 270.0,
        2000.0 / 270.0,
        2000.0 / 270.0,
        2000.0 / 270.0,
        2000.0 / 270.0,
    )
    min_pwm: int = 500
    max_pwm: int = 2500
    min_deg: tuple[float, float, float, float, float, float] = (-90.0,) * 6
    max_deg: tuple[float, float, float, float, float, float] = (135.0,) * 6

    @staticmethod
    def from_dict(data: dict | None) -> "ArmConfig":
        data = data or {}

        def tuple_n(name: str, default: Iterable, cast):
            values = list(data.get(name, default))
            if len(values) < SERVO_COUNT:
                values.extend(list(default)[len(values) : SERVO_COUNT])
            return tuple(cast(value) for value in values[:SERVO_COUNT])

        default = ArmConfig()
        return ArmConfig(
            l0_mm=float(data.get("l0_mm", default.l0_mm)),
            l1_mm=float(data.get("l1_mm", default.l1_mm)),
            l2_mm=float(data.get("l2_mm", default.l2_mm)),
            tool_length_mm=float(data.get("tool_length_mm", default.tool_length_mm)),
            home_pwm=tuple_n("home_pwm", default.home_pwm, int),
            direction=tuple_n("direction", default.direction, int),
            pwm_per_deg=tuple_n("pwm_per_deg", default.pwm_per_deg, float),
            min_pwm=int(data.get("min_pwm", default.min_pwm)),
            max_pwm=int(data.get("max_pwm", default.max_pwm)),
            min_deg=tuple_n("min_deg", default.min_deg, float),
            max_deg=tuple_n("max_deg", default.max_deg, float),
        )


@dataclass(frozen=True)
class ArmPose:
    x_mm: float
    y_mm: float
    z_mm: float
    base_joint_deg: float
    tool_pitch_deg: float
    tool_roll_deg: float

    def to_dict(self) -> dict[str, float]:
        return {
            "x_mm": float(self.x_mm),
            "y_mm": float(self.y_mm),
            "z_mm": float(self.z_mm),
            "base_joint_deg": float(self.base_joint_deg),
            "tool_pitch_deg": float(self.tool_pitch_deg),
            "tool_roll_deg": float(self.tool_roll_deg),
        }


@dataclass(frozen=True)
class IK5Solution:
    reachable: bool
    reason: str
    joint_deg: tuple[float, ...] = ()
    servo_pwm: tuple[int, ...] = ()
    base_branch_deg: float | None = None
    signed_radial_mm: float | None = None
    wrist_distance_mm: float | None = None


@dataclass(frozen=True)
class VerticalAdjustmentSolution:
    reachable: bool
    reason: str
    requested_delta_z_mm: float = 0.0
    start_pose: ArmPose | None = None
    target_pose: ArmPose | None = None
    achieved_pose: ArmPose | None = None
    joint_deg: tuple[float, ...] = ()
    servo_pwm: tuple[int, ...] = ()
    wrist_distance_mm: float | None = None


class ArmKinematics:
    """Five-axis arm model with the camera mount fixed before S4."""

    def __init__(self, config: ArmConfig | None = None):
        self.config = config or ArmConfig()

    def pwm_to_joint_deg(self, pwm_values: Iterable[int]) -> list[float]:
        values = list(pwm_values)
        if len(values) < SERVO_COUNT:
            values.extend(self.config.home_pwm[len(values) : SERVO_COUNT])
        joints: list[float] = []
        for index in range(KINEMATIC_JOINT_COUNT):
            direction = self.config.direction[index] or 1
            scale = self.config.pwm_per_deg[index] or (2000.0 / 270.0)
            joints.append((float(values[index]) - float(self.config.home_pwm[index])) / (float(direction) * scale))
        return joints

    def joint_deg_to_pwm(self, joint_values: Iterable[float], gripper_pwm: int | None = None) -> list[int]:
        values = list(joint_values)
        if len(values) < KINEMATIC_JOINT_COUNT:
            values.extend([0.0] * (KINEMATIC_JOINT_COUNT - len(values)))
        pwm: list[int] = []
        for index in range(KINEMATIC_JOINT_COUNT):
            value = (
                float(self.config.home_pwm[index])
                + float(self.config.direction[index] or 1)
                * float(self.config.pwm_per_deg[index] or (2000.0 / 270.0))
                * float(values[index])
            )
            pwm.append(int(round(max(self.config.min_pwm, min(self.config.max_pwm, value)))))
        gripper = self.config.home_pwm[5] if gripper_pwm is None else int(gripper_pwm)
        pwm.append(max(self.config.min_pwm, min(self.config.max_pwm, gripper)))
        return pwm

    def _chain_geometry(self, joint_values: Iterable[float]) -> tuple[list[float], float, float, float, float]:
        joint = list(joint_values)
        if len(joint) < KINEMATIC_JOINT_COUNT:
            joint.extend([0.0] * (KINEMATIC_JOINT_COUNT - len(joint)))

        base = math.radians(joint[0])
        shoulder_abs = math.radians(joint[1] + 90.0)
        elbow = math.radians(joint[2])
        wrist_pitch = math.radians(joint[3])
        forearm_pitch = shoulder_abs - elbow
        tool_pitch = forearm_pitch + wrist_pitch

        radial = (
            self.config.l1_mm * math.cos(shoulder_abs)
            + self.config.l2_mm * math.cos(forearm_pitch)
            + self.config.tool_length_mm * math.cos(tool_pitch)
        )
        z = (
            self.config.l0_mm
            + self.config.l1_mm * math.sin(shoulder_abs)
            + self.config.l2_mm * math.sin(forearm_pitch)
            + self.config.tool_length_mm * math.sin(tool_pitch)
        )

        return joint, base, tool_pitch, radial, z

    @staticmethod
    def _build_transform(base: float, tool_pitch: float, radial: float, z: float) -> np.ndarray:
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation_z(-base) @ rotation_x(tool_pitch)
        transform[:3, 3] = [radial * math.sin(base), radial * math.cos(base), z]
        return transform

    def camera_mount_transform(self, joint_values: Iterable[float]) -> np.ndarray:
        """Return T_base_camera_mount; S4 and S5 cannot affect this frame."""
        _, base, tool_pitch, radial, z = self._chain_geometry(joint_values)
        return self._build_transform(base, tool_pitch, radial, z)

    def forward_transform(self, joint_values: Iterable[float]) -> np.ndarray:
        """Return the full tool transform, including S4 rotation about local +Y."""
        joint, base, tool_pitch, radial, z = self._chain_geometry(joint_values)
        transform = self._build_transform(base, tool_pitch, radial, z)
        transform[:3, :3] = transform[:3, :3] @ rotation_y(math.radians(joint[4]))
        return transform

    def forward_kinematics(self, joint_values: Iterable[float]) -> ArmPose:
        joint = list(joint_values)
        if len(joint) < KINEMATIC_JOINT_COUNT:
            joint.extend([0.0] * (KINEMATIC_JOINT_COUNT - len(joint)))
        transform = self.forward_transform(joint)
        tool_pitch = 90.0 + joint[1] - joint[2] + joint[3]
        return ArmPose(
            x_mm=float(transform[0, 3]),
            y_mm=float(transform[1, 3]),
            z_mm=float(transform[2, 3]),
            base_joint_deg=float(joint[0]),
            tool_pitch_deg=float(tool_pitch),
            tool_roll_deg=float(joint[4]),
        )

    def pose_from_pwm(self, pwm_values: Iterable[int]) -> tuple[ArmPose, list[float], np.ndarray]:
        joints = self.pwm_to_joint_deg(pwm_values)
        return self.forward_kinematics(joints), joints, self.forward_transform(joints)

    def transforms_from_pwm(
        self, pwm_values: Iterable[int]
    ) -> tuple[ArmPose, list[float], np.ndarray, np.ndarray]:
        joints = self.pwm_to_joint_deg(pwm_values)
        return (
            self.forward_kinematics(joints),
            joints,
            self.forward_transform(joints),
            self.camera_mount_transform(joints),
        )

    def solve_vertical_adjustment(
        self,
        current_pwm: Iterable[int],
        delta_z_mm: float,
    ) -> VerticalAdjustmentSolution:
        """Move only S1-S3 while preserving XY, base, pitch, S4, and S5."""
        try:
            delta_z = float(delta_z_mm)
            current_values = [float(value) for value in current_pwm]
        except (TypeError, ValueError):
            return VerticalAdjustmentSolution(False, "BAD_ARGS")
        if (
            not math.isfinite(delta_z)
            or abs(delta_z) < 1e-9
            or len(current_values) < SERVO_COUNT
            or not all(math.isfinite(value) for value in current_values[:SERVO_COUNT])
        ):
            return VerticalAdjustmentSolution(False, "BAD_ARGS")

        current = [int(round(value)) for value in current_values[:SERVO_COUNT]]
        if any(value < self.config.min_pwm or value > self.config.max_pwm for value in current):
            return VerticalAdjustmentSolution(False, "BAD_STATE")

        current_joints = self.pwm_to_joint_deg(current)
        for index in (0, 4):
            if not self.config.min_deg[index] <= current_joints[index] <= self.config.max_deg[index]:
                return VerticalAdjustmentSolution(False, "BAD_STATE")

        start_pose = self.forward_kinematics(current_joints)
        _, _, pitch_rad, signed_radial, start_z = self._chain_geometry(current_joints)
        target_z = start_z + delta_z
        target_pose = ArmPose(
            x_mm=start_pose.x_mm,
            y_mm=start_pose.y_mm,
            z_mm=target_z,
            base_joint_deg=start_pose.base_joint_deg,
            tool_pitch_deg=start_pose.tool_pitch_deg,
            tool_roll_deg=start_pose.tool_roll_deg,
        )

        wrist_radial = signed_radial - self.config.tool_length_mm * math.cos(pitch_rad)
        wrist_height = target_z - self.config.l0_mm - self.config.tool_length_mm * math.sin(pitch_rad)
        distance = math.hypot(wrist_radial, wrist_height)
        minimum_distance = abs(self.config.l1_mm - self.config.l2_mm)
        maximum_distance = self.config.l1_mm + self.config.l2_mm
        if distance < 1.0 or not minimum_distance <= distance <= maximum_distance:
            return VerticalAdjustmentSolution(
                False,
                "UNREACHABLE",
                delta_z,
                start_pose,
                target_pose,
                wrist_distance_mm=distance,
            )

        distance_sq = distance * distance
        cos_elbow = (
            distance_sq - self.config.l1_mm**2 - self.config.l2_mm**2
        ) / (2.0 * self.config.l1_mm * self.config.l2_mm)
        cos_shoulder = (
            distance_sq + self.config.l1_mm**2 - self.config.l2_mm**2
        ) / (2.0 * self.config.l1_mm * distance)
        if cos_elbow < -1.002 or cos_elbow > 1.002 or cos_shoulder < -1.002 or cos_shoulder > 1.002:
            return VerticalAdjustmentSolution(
                False,
                "UNREACHABLE",
                delta_z,
                start_pose,
                target_pose,
                wrist_distance_mm=distance,
            )

        elbow_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos_elbow))))
        shoulder_base_deg = math.degrees(math.atan2(wrist_height, wrist_radial))
        shoulder_offset_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos_shoulder))))
        pitch_deg = math.degrees(pitch_rad)
        candidates: list[tuple[int, list[float], list[int], ArmPose]] = []
        for shoulder_abs_deg, elbow_value in (
            (shoulder_base_deg + shoulder_offset_deg, elbow_deg),
            (shoulder_base_deg - shoulder_offset_deg, -elbow_deg),
        ):
            joints = list(current_joints)
            joints[1] = shoulder_abs_deg - 90.0
            joints[2] = elbow_value
            joints[3] = pitch_deg - shoulder_abs_deg + elbow_value
            pwm = list(current)
            valid = True
            for index in range(1, 4):
                raw_pwm = int(round(
                    self.config.home_pwm[index]
                    + self.config.direction[index]
                    * self.config.pwm_per_deg[index]
                    * joints[index]
                ))
                if (
                    joints[index] < self.config.min_deg[index]
                    or joints[index] > self.config.max_deg[index]
                    or raw_pwm < self.config.min_pwm
                    or raw_pwm > self.config.max_pwm
                ):
                    valid = False
                    break
                pwm[index] = raw_pwm
            if not valid:
                continue
            quantized_joints = self.pwm_to_joint_deg(pwm)
            achieved_pose = self.forward_kinematics(quantized_joints)
            score = sum(abs(pwm[index] - current[index]) for index in range(1, 4))
            candidates.append((score, quantized_joints, pwm, achieved_pose))

        if not candidates:
            return VerticalAdjustmentSolution(
                False,
                "LIMIT",
                delta_z,
                start_pose,
                target_pose,
                wrist_distance_mm=distance,
            )

        _, joints, pwm, achieved_pose = min(candidates, key=lambda item: item[0])
        return VerticalAdjustmentSolution(
            True,
            "OK",
            delta_z,
            start_pose,
            target_pose,
            achieved_pose,
            tuple(float(value) for value in joints),
            tuple(int(value) for value in pwm),
            distance,
        )

    def solve_ik5(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        tool_pitch_deg: float,
        tool_roll_deg: float,
        current_pwm: Iterable[int] | None = None,
    ) -> IK5Solution:
        """Mirror the F103 IK5 solver, including the folded negative-radial branch."""
        values = (x_mm, y_mm, z_mm, tool_pitch_deg, tool_roll_deg)
        if not all(math.isfinite(float(value)) for value in values):
            return IK5Solution(False, "BAD_ARGS")
        current = list(current_pwm) if current_pwm is not None else list(self.config.home_pwm)
        if len(current) < SERVO_COUNT:
            current.extend(self.config.home_pwm[len(current) : SERVO_COUNT])

        target_radial = math.hypot(float(x_mm), float(y_mm))
        if target_radial < 0.001:
            return IK5Solution(False, "SINGULAR")
        primary_base = math.degrees(math.atan2(float(x_mm), float(y_mm)))
        secondary_base = normalize_deg(primary_base + 180.0)
        pitch_rad = math.radians(float(tool_pitch_deg))
        height = float(z_mm) - self.config.l0_mm - self.config.tool_length_mm * math.sin(pitch_rad)
        candidates: list[tuple[int, list[float], list[int], float, float, float]] = []
        geometric_branch_found = False
        wrist_distances: list[float] = []

        for base_deg, signed_radial in (
            (primary_base, target_radial),
            (secondary_base, -target_radial),
        ):
            wrist_radial = signed_radial - self.config.tool_length_mm * math.cos(pitch_rad)
            distance = math.hypot(wrist_radial, height)
            wrist_distances.append(distance)
            if distance < 1.0:
                continue
            if not (
                abs(self.config.l1_mm - self.config.l2_mm)
                <= distance
                <= self.config.l1_mm + self.config.l2_mm
            ):
                continue
            geometric_branch_found = True
            distance_sq = distance * distance
            cos_elbow = (
                distance_sq - self.config.l1_mm**2 - self.config.l2_mm**2
            ) / (2.0 * self.config.l1_mm * self.config.l2_mm)
            cos_shoulder = (
                distance_sq + self.config.l1_mm**2 - self.config.l2_mm**2
            ) / (2.0 * self.config.l1_mm * distance)
            if cos_elbow < -1.002 or cos_elbow > 1.002 or cos_shoulder < -1.002 or cos_shoulder > 1.002:
                continue
            elbow_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos_elbow))))
            shoulder_base_deg = math.degrees(math.atan2(height, wrist_radial))
            shoulder_offset_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos_shoulder))))
            for shoulder_abs_deg, elbow_value in (
                (shoulder_base_deg + shoulder_offset_deg, elbow_deg),
                (shoulder_base_deg - shoulder_offset_deg, -elbow_deg),
            ):
                joints = [
                    normalize_deg(base_deg),
                    shoulder_abs_deg - 90.0,
                    elbow_value,
                    normalize_deg(float(tool_pitch_deg) - shoulder_abs_deg + elbow_value),
                    normalize_deg(float(tool_roll_deg)),
                ]
                pwm: list[int] = []
                valid = True
                for index, joint in enumerate(joints):
                    raw_pwm = int(round(
                        self.config.home_pwm[index]
                        + self.config.direction[index]
                        * self.config.pwm_per_deg[index]
                        * joint
                    ))
                    if (
                        joint < self.config.min_deg[index]
                        or joint > self.config.max_deg[index]
                        or raw_pwm < self.config.min_pwm
                        or raw_pwm > self.config.max_pwm
                    ):
                        valid = False
                        break
                    pwm.append(raw_pwm)
                if not valid:
                    continue
                pwm.append(int(max(self.config.min_pwm, min(self.config.max_pwm, current[5]))))
                score = sum(abs(pwm[index] - int(current[index])) for index in range(KINEMATIC_JOINT_COUNT))
                candidates.append((score, joints, pwm, base_deg, signed_radial, distance))

        if not candidates:
            reason = "LIMIT" if geometric_branch_found else "UNREACHABLE"
            nearest = min(wrist_distances) if wrist_distances else None
            return IK5Solution(False, reason, wrist_distance_mm=nearest)
        _, joints, pwm, base_deg, signed_radial, distance = min(candidates, key=lambda item: item[0])
        return IK5Solution(
            True,
            "OK",
            tuple(float(value) for value in joints),
            tuple(int(value) for value in pwm),
            float(base_deg),
            float(signed_radial),
            float(distance),
        )

    def reachable_pitch_ranges(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        tool_roll_deg: float = 0.0,
        current_pwm: Iterable[int] | None = None,
        step_deg: int = 1,
    ) -> list[tuple[int, int]]:
        reachable = [
            pitch
            for pitch in range(-180, 181, max(1, int(step_deg)))
            if self.solve_ik5(
                x_mm, y_mm, z_mm, float(pitch), tool_roll_deg, current_pwm
            ).reachable
        ]
        if not reachable:
            return []
        ranges: list[tuple[int, int]] = []
        start = previous = reachable[0]
        step = max(1, int(step_deg))
        for value in reachable[1:]:
            if value != previous + step:
                ranges.append((start, previous))
                start = value
            previous = value
        ranges.append((start, previous))
        return ranges


def rotation_x(angle_rad: float) -> np.ndarray:
    cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]], dtype=np.float64)


def rotation_y(angle_rad: float) -> np.ndarray:
    cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]], dtype=np.float64)


def rotation_z(angle_rad: float) -> np.ndarray:
    cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def normalize_deg(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def invert_transform(matrix: np.ndarray) -> np.ndarray:
    inv = np.eye(4, dtype=np.float64)
    rot = matrix[:3, :3]
    trans = matrix[:3, 3]
    inv[:3, :3] = rot.T
    inv[:3, 3] = -rot.T @ trans
    return inv


def base_point_to_ik5(point_base_mm: Iterable[float], z_offset_mm: float = 0.0) -> tuple[float, float, float]:
    """Map the verified base frame directly to IK5: +X left, -Y forward, +Z up."""
    values = [float(value) for value in point_base_mm]
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("Base point must contain three finite coordinates")
    return values[0], values[1], values[2] + float(z_offset_mm)


def matrix_to_readable(matrix: np.ndarray) -> dict[str, object]:
    return {
        "matrix": [[float(value) for value in row] for row in matrix.tolist()],
        "translation_mm": [float(value) for value in matrix[:3, 3].tolist()],
    }
