"""Real and simulated Buck-E arms with the same application methods."""

from dataclasses import dataclass
from importlib import metadata
import math
import time
from typing import Tuple

# Saved configurations list J1-J6 in degrees. The operator confirmed both
# synchronous standby/ready transitions on October 28, 2026.
STANDBY_ANGLES = (0, 0, 90, 90, 90, -135)
READY_ANGLES = (-90, 0, 0, 0, 90, -135)
POSE_TOLERANCE = 3.0
POSE_SPEED = 5
DEFAULT_JOINT_SPEED = 10
MOVE_TIMEOUT = 120
SETTLE_TIMEOUT = 5
# Buck-E uses the MyCobot280 SDK bounds confirmed by the Pi readback, with
# tighter operator-specified caps for J2 (±135 degrees) and J4 (±145 degrees).
# Keep the offline preview defaults in config/ready_limits.provisional.json in sync.
SPEC_LIMITS = ((-168, 168), (-135, 135), (-150, 150),
               (-145, 145), (-155, 160), (-180, 180))


def _pose(values):
    if (not isinstance(values, (list, tuple)) or len(values) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
        raise ValueError("Expected six finite numeric angles/coordinates")
    return tuple(values)


def _matches(current, target, tolerance=POSE_TOLERANCE):
    return all(abs(a - b) <= tolerance for a, b in zip(current, target))


def validate_speed(value):
    """Require an integer SDK speed setting from 1 to 100."""
    if type(value) is not int or not 1 <= value <= 100:
        raise ValueError("Speed must be an integer from 1 to 100")


def _spec_limits(joint):
    if type(joint) is not int or not 1 <= joint <= 6:
        raise ValueError("Joint must be 1 through 6")
    return SPEC_LIMITS[joint - 1]


def _joint_target(joint, target):
    low, high = _spec_limits(joint)
    if type(target) not in (int, float) or not math.isfinite(target):
        raise ValueError("Target must be a finite number")
    if not low <= target <= high:
        raise ValueError(f"J{joint} target outside Buck-E limits [{low:g}, {high:g}] degrees")


@dataclass(frozen=True)
class Telemetry:
    """Joint angles and the current coordinate reply, with their source."""

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
        """Read joint angles and coordinates in the current controller frames."""
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
        """Allow commands only while stationary and fault-free."""
        if self._motion_status() != 0:
            raise RuntimeError("Arm must be stationary before a command")

    def joint_limits(self, joint):
        """Return the range permitted by Buck-E and the controller."""
        allowed, _ = self.joint_limit_ranges(joint)
        return allowed

    def joint_limit_ranges(self, joint):
        """Return allowed bounds and the controller's raw bounds, if available."""
        low, high = _spec_limits(joint)
        try:
            robot_low = self._client.get_joint_min_angle(joint)
            robot_high = self._client.get_joint_max_angle(joint)
        except (AttributeError, OSError):
            # Use the local bounds when controller limits are unavailable.
            return (low, high), None
        if (type(robot_low) in (int, float) and type(robot_high) in (int, float)
                and math.isfinite(robot_low) and math.isfinite(robot_high) and robot_low < robot_high):
            # Live settings can narrow the Buck-E range on either side.
            allowed = max(low, robot_low), min(high, robot_high)
            return allowed, (robot_low, robot_high)
        return (low, high), None

    def _check_target(self, joint, target):
        _joint_target(joint, target)
        low, high = self.joint_limits(joint)
        if not low <= target <= high:
            raise ValueError(f"J{joint} target outside controller limits [{low:g}, {high:g}] degrees")

    def move_joint(self, joint, target, speed=DEFAULT_JOINT_SPEED):
        """Send a checked joint command. Observe arrival separately."""
        validate_speed(speed)
        self._check_target(joint, target)
        self._require_stationary()
        self._client.send_angle(joint, target, speed)

    def open_gripper(self, speed=20):
        validate_speed(speed)
        self._require_stationary()
        self._client.set_gripper_state(0, speed)

    def close_gripper(self, speed=20):
        validate_speed(speed)
        self._require_stationary()
        self._client.set_gripper_state(1, speed)

    def move_to_ready(self):
        """Move from saved standby and verify arrival at ready."""
        return self._move_between(STANDBY_ANGLES, READY_ANGLES)

    def move_to_standby(self):
        """Move from saved ready and verify arrival at standby."""
        return self._move_between(READY_ANGLES, STANDBY_ANGLES)

    def check_hover_start(self, resume_lift3=False, from_hover=False):
        """Require three stable readings at the selected checkpoint or hover path."""
        from .resume_hover import LIFT3_READBACK, return_route
        expected = LIFT3_READBACK if resume_lift3 else READY_ANGLES
        tolerance = 0.5 if resume_lift3 else POSE_TOLERANCE
        reference = None
        for _ in range(3):
            self._require_stationary()
            actual = self.get_angles()
            if from_hover:
                return_route(actual)  # Refuse poses outside the padded route segments.
            elif not _matches(actual, expected, tolerance):
                name = "recorded Lift 3 checkpoint" if resume_lift3 else "saved ready"
                raise RuntimeError("Hover requires " + name + "; read " + repr(actual))
            if reference is not None and not _matches(actual, reference, 0.5):
                raise RuntimeError("Starting telemetry is not stable")
            reference = actual if reference is None else reference
            time.sleep(0.2)
        return reference

    def move_to_hover(self, resume_lift3=False, synchronous=False, continuous=False):
        return self._move_hover_route(resume_lift3, synchronous, continuous)

    def move_from_hover_to_ready(self, synchronous=True, continuous=False):
        raise RuntimeError("Hover return disabled after reported self-contact. "
                           "A measured forward trajectory must be recorded and reviewed first.")

    def _record_forward_hover(self, target, tolerance):
        """Send one forward command and capture actual motion instead of blocking SDK waits."""
        from .hover_trajectory import save_forward_trace
        samples, complete, failure = [], False, None
        started = time.monotonic()

        def capture():
            moving = self._motion_status()
            actual = self.get_angles()
            samples.append({"elapsed_s": time.monotonic() - started,
                            "angles_deg": list(actual), "moving": moving, "error": 0})
            return moving, actual

        try:
            moving, actual = capture()
            if moving != 0 or not _matches(actual, READY_ANGLES):
                raise RuntimeError("Recording must start stationary at saved ready")
            acknowledgement = self._client.send_angles(list(target), POSE_SPEED)
            if type(acknowledgement) is not int or acknowledgement != 1:
                raise RuntimeError("Forward command acknowledgement failed")
            settled = 0
            while time.monotonic() - started < MOVE_TIMEOUT:
                time.sleep(0.1)
                moving, actual = capture()
                reached = _matches(actual, target, tolerance)
                settled = settled + 1 if moving == 0 and reached else 0
                if settled >= 3:
                    complete = True
                    return True
            raise RuntimeError("Forward capture timed out before hover was confirmed")
        except BaseException as exc:
            failure = str(exc) or type(exc).__name__
            try:
                self.stop()
            except Exception:
                pass
            raise
        finally:
            try:
                path = save_forward_trace(samples, complete, failure)
                print("Measured forward recording:", path, flush=True)
            except OSError as exc:
                if complete:
                    raise RuntimeError("Hover reached but recording could not be saved: " + str(exc)) from exc
                print("Recording could not be saved:", exc, flush=True)

    def _move_hover_route(self, resume_lift3, synchronous, continuous):
        """Replay the supervised approach, then bounded final hover corrections.

        Requires the original physical setup and a cleared sweep. Joint-space
        waypoints and telemetry checks do not provide collision detection.
        """
        from .resume_hover import HOVER, LIFT3_READBACK, route

        def require_pose(expected, tolerance):
            self._require_stationary()
            actual = self.get_angles()
            if not _matches(actual, expected, tolerance):
                raise RuntimeError("Unexpected starting pose: " + repr(actual))
            return actual

        def move(label, target, target_tolerance, expected, tolerance):
            for joint, angle in enumerate(target, 1):
                self._check_target(joint, angle)
            # Recheck the entry pose after querying controller limits.
            actual = require_pose(expected, tolerance)
            print(label, flush=True)
            if synchronous:
                # The SDK can return 1 on timeout. Always verify independently.
                # This call blocks our application fault polling until it returns.
                self._client.sync_send_angles(list(target), POSE_SPEED, timeout=8)
            else:
                self._client.send_angles(list(target), POSE_SPEED)
            started, settled = time.monotonic(), 0
            while time.monotonic() - started < 8:
                moving, actual = self._motion_status(), self.get_angles()
                reached = _matches(actual, target, target_tolerance)
                settled = settled + 1 if moving == 0 and reached else 0
                if settled >= 3:
                    time.sleep(2.0)
                    require_pose(target, target_tolerance)
                    return
                time.sleep(0.2)
            raise RuntimeError("Timed out at " + label + "; read " + repr(actual))

        self.check_hover_start(resume_lift3=resume_lift3)
        waypoints = route(resume_lift3=resume_lift3)
        # Validate the whole route before sending its first command.
        for joint in range(1, 7):
            values = [target[joint - 1] for _, target, _ in waypoints]
            self._check_target(joint, min(values))
            self._check_target(joint, max(values))
        self.check_hover_start(resume_lift3=resume_lift3)
        expected = LIFT3_READBACK if resume_lift3 else READY_ANGLES
        tolerance = 0.5 if resume_lift3 else POSE_TOLERANCE
        try:
            if continuous:
                require_pose(expected, tolerance)
                if resume_lift3:
                    raise RuntimeError("Continuous recording requires a full forward run from ready")
                print("Single forward move to hover; recording measured joint trajectory.", flush=True)
                self._record_forward_hover(HOVER, 1.5)
                print("Final angles:", self.get_angles(), flush=True)
                return True
            time.sleep(2.0)
            for label, target, next_tolerance in waypoints:
                require_pose(expected, tolerance)
                move(label, target, next_tolerance, expected, tolerance)
                expected, tolerance = target, next_tolerance

            # Bound the compensation for the observed endpoint error.
            actual = require_pose(HOVER, 2.0)
            error = max(abs(a - b) for a, b in zip(actual, HOVER))
            for attempt in range(1, 3):
                if error <= 1.5:
                    break
                target = tuple(round(goal + max(-0.5, min(0.5, goal - measured)), 2)
                               if abs(goal - measured) > 1.5 else goal
                               for goal, measured in zip(HOVER, actual))
                move(f"Hover trim {attempt}/2", target, 2.0, expected, tolerance)
                actual = require_pose(HOVER, 2.0)
                improved = max(abs(a - b) for a, b in zip(actual, HOVER))
                if improved >= error - 0.05:
                    raise RuntimeError("Hover trim did not improve joint error")
                error = improved
            if error > 1.5:
                raise RuntimeError("Final hover error exceeds 1.5 degrees")
            print("Final angles:", actual, flush=True)
            return True
        except BaseException:
            try:
                self.stop()
            except Exception:
                pass
            raise

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
            # An SDK exception can occur after a command has been transmitted.
            try:
                self.stop()
            except Exception:
                pass  # Preserve the original failure if stop also fails.
            raise
        return True

    def _wait_for_target(self, target, tolerance=POSE_TOLERANCE):
        """Confirm arrival across three consecutive stationary, fault-free samples."""
        started = time.monotonic()
        settled = 0
        while time.monotonic() - started < SETTLE_TIMEOUT:
            moving = self._motion_status()
            current = self.get_angles()
            reached = _matches(current, target, tolerance)
            settled = settled + 1 if moving == 0 and reached else 0
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
        """Close the serial connection while leaving holding torque unchanged."""
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

    def joint_limits(self, joint):
        """Use the Buck-E command bounds for simulated joints."""
        return _spec_limits(joint)

    def joint_limit_ranges(self, joint):
        """Return the allowed bounds with no controller readback in simulation."""
        return self.joint_limits(joint), None

    def move_joint(self, joint, target, speed=DEFAULT_JOINT_SPEED):
        validate_speed(speed)
        _joint_target(joint, target)
        updated = list(self._angles)
        updated[joint - 1] = target
        self._angles = tuple(updated)

    def open_gripper(self, speed=20):
        validate_speed(speed)
        self.gripper_open = True

    def close_gripper(self, speed=20):
        validate_speed(speed)
        self.gripper_open = False

    def move_to_ready(self):
        return self._move_between(STANDBY_ANGLES, READY_ANGLES)

    def move_to_standby(self):
        return self._move_between(READY_ANGLES, STANDBY_ANGLES)

    def move_to_hover(self, resume_lift3=False, synchronous=False, continuous=False):
        from .resume_hover import HOVER, LIFT3_READBACK
        if resume_lift3:
            if any(abs(a - b) > 0.5 for a, b in zip(self._angles, LIFT3_READBACK)):
                raise RuntimeError("Hover requires recorded Lift 3 checkpoint")
            self._angles = HOVER
            return True
        return self._move_between(READY_ANGLES, HOVER)

    def move_from_hover_to_ready(self, synchronous=True, continuous=False):
        raise RuntimeError("Hover return disabled after reported self-contact")

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
