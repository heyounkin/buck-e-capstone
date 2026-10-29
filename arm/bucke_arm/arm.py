"""Real and simulated Buck-E arms with the same application methods."""

from dataclasses import dataclass
from importlib import metadata
import math
import time
from typing import Tuple

STANDBY_ANGLES = (0, 0, 90, 90, 90, -135)
READY_ANGLES = (90, 0, 0, 0, 90, -135)
POSE_TOLERANCE = 3.0
POSE_SPEED = 5
MOVE_TIMEOUT = 120
SETTLE_TIMEOUT = 5
SPEC_LIMITS = ((-168, 168), (-135, 135), (-150, 150),
               (-145, 145), (-165, 165), (-180, 180))


def _pose(values):
    if (not isinstance(values, (list, tuple)) or len(values) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
        raise ValueError("Expected six finite numeric angles/coordinates")
    return tuple(values)


def _matches(current, target):
    return all(abs(a - b) <= POSE_TOLERANCE for a, b in zip(current, target))


def _speed(value):
    if type(value) is not int or not 1 <= value <= 100:
        raise ValueError("Speed must be an integer from 1 to 100")


def _joint_target(joint, target):
    if type(joint) is not int or not 1 <= joint <= 6:
        raise ValueError("Joint must be 1 through 6")
    if type(target) not in (int, float) or not math.isfinite(target):
        raise ValueError("Target must be a finite number")
    low, high = SPEC_LIMITS[joint - 1]
    if not low <= target <= high:
        raise ValueError("J%d target outside joint limits" % joint)


@dataclass(frozen=True)
class Telemetry:
    source: str
    joint_angles_deg: Tuple[float, ...]
    tool_pose_mm_deg: Tuple[float, ...]

    def validate(self):
        if self.source not in ("simulated", "hardware"):
            raise ValueError("Telemetry source must be simulated or hardware")
        _pose(self.joint_angles_deg)
        _pose(self.tool_pose_mm_deg)


class RealArm:
    """Connect to the Pi SDK, or accept an injected SDK client for tests."""

    def __init__(self, client=None):
        if client is None:
            try:
                version = metadata.version("pymycobot")
            except metadata.PackageNotFoundError as error:
                raise RuntimeError("pymycobot is missing from this Python environment") from error
            if version != "4.0.6":
                raise RuntimeError("Expected verified pymycobot 4.0.6; found " + version)
            from pymycobot import MyCobot280, PI_PORT, PI_BAUD
            if PI_PORT != "/dev/ttyAMA0" or PI_BAUD != 1000000:
                raise RuntimeError("Library Pi constants differ from the verified settings")
            client = MyCobot280(PI_PORT, PI_BAUD)
        self._client = client

    def get_angles(self):
        return _pose(self._client.get_angles())

    def read_status(self):
        telemetry = Telemetry("hardware", self.get_angles(), _pose(self._client.get_coords()))
        telemetry.validate()
        return telemetry

    def read_fk_sample(self):
        """Bracket a coordinate query with stationary angles and reference reads.

        Queries are sequential, not atomic. No settings or motion are changed.
        """
        def references():
            result = {"reference_frame": self._client.get_reference_frame(),
                      "end_type": self._client.get_end_type(),
                      "tool_reference": _pose(self._client.get_tool_reference()),
                      "world_reference": _pose(self._client.get_world_reference())}
            for key in ("reference_frame", "end_type"):
                if type(result[key]) is not int or result[key] != 0:
                    raise RuntimeError("FK capture requires base frame and flange end type: " + repr(result))
            if any(result[key] != (0,) * 6 for key in ("tool_reference", "world_reference")):
                raise RuntimeError("FK capture requires zero tool/world references: " + repr(result))
            return result

        self._require_stationary()
        before_reference = references()
        before = self.get_angles()
        coords = _pose(self._client.get_coords())
        after = self.get_angles()
        after_reference = references()
        self._require_stationary()
        return {"angles_before_deg": before, "angles_after_deg": after,
                "reported_coords_mm_deg": coords,
                "references_before": before_reference, "references_after": after_reference}

    def _motion_status(self):
        moving = self._client.is_moving()
        error = self._client.get_error_information()
        if type(moving) is not int or moving not in (0, 1) or type(error) is not int or error != 0:
            raise RuntimeError("Invalid motion status or controller fault: moving=%r, error=%r" % (moving, error))
        return moving

    def _require_stationary(self):
        if self._motion_status() != 0:
            raise RuntimeError("Arm must be stationary before a command")

    def _check_target(self, joint, target):
        _joint_target(joint, target)
        low, high = SPEC_LIMITS[joint - 1]
        try:
            robot_low = self._client.get_joint_min_angle(joint)
            robot_high = self._client.get_joint_max_angle(joint)
        except (AttributeError, OSError):
            return  # Retain the manual controller's documented-limit fallback.
        if (type(robot_low) in (int, float) and type(robot_high) in (int, float)
                and math.isfinite(robot_low) and math.isfinite(robot_high) and robot_low < robot_high):
            low, high = max(low, robot_low), min(high, robot_high)
        if not low <= target <= high:
            raise ValueError("J%d target outside controller limits" % joint)

    def move_joint(self, joint, target, speed=10):
        _speed(speed)
        self._check_target(joint, target)
        self._require_stationary()
        self._client.send_angle(joint, target, speed)

    def open_gripper(self, speed=20):
        _speed(speed)
        self._require_stationary()
        self._client.set_gripper_state(0, speed)

    def close_gripper(self, speed=20):
        _speed(speed)
        self._require_stationary()
        self._client.set_gripper_state(1, speed)

    def move_to_ready(self):
        return self._move_between(STANDBY_ANGLES, READY_ANGLES)

    def move_to_standby(self):
        return self._move_between(READY_ANGLES, STANDBY_ANGLES)

    def _move_between(self, start, target):
        self._require_stationary()
        current = self.get_angles()
        if _matches(current, target):
            return True
        if not _matches(current, start):
            raise RuntimeError("Arm is not at the required starting pose: " + repr(start))
        for joint, angle in enumerate(target, 1):
            self._check_target(joint, angle)
        # Recheck after the limit queries, immediately before transmission.
        self._require_stationary()
        if not _matches(self.get_angles(), start):
            raise RuntimeError("Starting pose changed before movement")
        try:
            # SDK 4.0.6 returns 1 even on timeout, so verify telemetry afterward.
            # The blocking SDK call pauses our own fault polling until it returns.
            self._client.sync_send_angles(list(target), POSE_SPEED, timeout=MOVE_TIMEOUT)
            self._wait_for_target(target)
        except BaseException:
            try:
                self.stop()
            except Exception:
                pass  # Preserve the original failure if stop also fails.
            raise
        return True

    def _wait_for_target(self, target):
        started = time.monotonic()
        settled = 0
        while time.monotonic() - started < SETTLE_TIMEOUT:
            moving = self._motion_status()
            current = self.get_angles()
            settled = settled + 1 if moving == 0 and _matches(current, target) else 0
            if settled >= 3:
                return
            time.sleep(0.2)
        raise RuntimeError("Target not confirmed within pose tolerance")

    def stop(self):
        try:
            self._client.stop()
        finally:
            self._client.gripper_stop()

    def close(self):
        self._client._serial_port.close()


class FakeArm:
    """Immediate state changes with nominal flange FK; no motion physics."""

    def __init__(self, angles=STANDBY_ANGLES):
        self._angles = _pose(angles)
        self.gripper_open = False

    def get_angles(self):
        return self._angles

    def read_status(self):
        from .kinematics import forward_kinematics
        return Telemetry("simulated", self._angles,
                         forward_kinematics(self._angles).pose_mm_deg)

    def move_joint(self, joint, target, speed=10):
        _speed(speed)
        _joint_target(joint, target)
        updated = list(self._angles)
        updated[joint - 1] = target
        self._angles = tuple(updated)

    def open_gripper(self, speed=20):
        _speed(speed)
        self.gripper_open = True

    def close_gripper(self, speed=20):
        _speed(speed)
        self.gripper_open = False

    def move_to_ready(self):
        return self._move_between(STANDBY_ANGLES, READY_ANGLES)

    def move_to_standby(self):
        return self._move_between(READY_ANGLES, STANDBY_ANGLES)

    def _move_between(self, start, target):
        if _matches(self._angles, target):
            return True
        if not _matches(self._angles, start):
            raise RuntimeError("Arm is not at the required starting pose: " + repr(start))
        self._angles = target
        return True

    def stop(self):
        pass

    def close(self):
        pass
