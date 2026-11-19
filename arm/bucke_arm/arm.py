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
# Shared contract for raised-approach arrival and entry into the grasp phase.
APPROACH_TOLERANCE = 1.5
POSE_SPEED = 5
DEFAULT_JOINT_SPEED = 10
MOVE_TIMEOUT = 120
SETTLE_TIMEOUT = 5
# Buck-E uses the MyCobot280 SDK bounds confirmed by the Pi readback, with
# tighter operator-specified caps for J2 (±135 degrees) and J4 (±145 degrees).
# Keep the offline preview defaults in config/ready_limits.provisional.json in sync.
SPEC_LIMITS = ((-168, 168), (-135, 135), (-150, 150),
               (-145, 145), (-155, 160), (-180, 180))


def _pose(values, *, source=None):
    if (not isinstance(values, (list, tuple)) or len(values) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
        message = "Expected six finite numeric angles/coordinates"
        if source is not None:
            message += f"; {source} returned {values!r}"
        raise ValueError(message)
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
        return _pose(self._client.get_angles(), source="get_angles()")

    def read_status(self):
        """Read joint angles and coordinates in the current controller frames."""
        # Keep the failing SDK query and raw reply visible when telemetry is
        # malformed; an angle failure and a coordinate failure need different diagnosis.
        telemetry = Telemetry("hardware", self.get_angles(),
                              _pose(self._client.get_coords(), source="get_coords()"))
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

    def move_joint(self, joint, target, speed=DEFAULT_JOINT_SPEED, *, wait=False):
        """Send a checked joint command and optionally return settled angles."""
        validate_speed(speed)
        self._check_target(joint, target)
        if wait:
            expected = list(self.get_angles())
            expected[joint - 1] = target
        self._require_stationary()
        try:
            self._client.send_angle(joint, target, speed)
            if wait:
                return self._wait_for_target(expected, timeout=MOVE_TIMEOUT)
        except BaseException:
            if wait:
                try:
                    self.stop()
                except Exception:
                    pass  # Preserve the original failure if stop also fails.
            raise

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

    def move_to_hover(self, resume_lift3=False, synchronous=False, continuous=None):
        # Ordinary HOVER is one monitored endpoint command. The fixed historical
        # recovery and explicitly requested waypoint mode remain separate.
        if continuous is None:
            continuous = not (resume_lift3 or synchronous)
        return self._move_hover_route(resume_lift3, synchronous, continuous)

    def _check_stopped_pose(self, expected, tolerance, label):
        """Require stable, fault-free readings at a fixed diagnostic checkpoint."""
        reference = None
        for _ in range(3):
            self._require_stationary()
            actual = self.get_angles()
            if not _matches(actual, expected, tolerance):
                raise RuntimeError(label + "; read " + repr(actual))
            if reference is not None and not _matches(actual, reference, 0.25):
                raise RuntimeError("Diagnostic starting telemetry is unstable")
            reference = actual if reference is None else reference
            time.sleep(0.2)
        return actual

    def _check_raised_approach_start(self):
        """Require the known stopped diagnostic pose before the small raised test."""
        from .raised_approach import START_ANGLES, START_TOLERANCE
        self._check_stopped_pose(START_ANGLES, START_TOLERANCE,
                                 "Raised approach requires recorded J2-test pose")

    def _sample_local_approach_start(self):
        """Capture stable stopped telemetry only inside the observed local region."""
        from .local_approach import known_start, STABILITY_TOLERANCE
        first = None
        for _ in range(3):
            self._require_stationary()
            actual = self.get_angles()
            if not known_start(actual):
                raise RuntimeError("Local approach requires the observed post-alignment region; read " + repr(actual))
            if first is not None and not _matches(actual,first,STABILITY_TOLERANCE):
                raise RuntimeError("Local approach start is not stable")
            first = actual if first is None else first
            time.sleep(0.2)
        return actual

    def move_to_local_approach(self):
        """One measured-reference positioning trial from a checked live start."""
        live=self._sample_local_approach_start()
        return self._move_local_approach_from_live(live)

    def _move_local_approach_from_live(self,live):
        """Execute from a live pose already checked by its owning approach stage."""
        from .local_approach import STABILITY_TOLERANCE,desired_target,command_target
        desired,command=desired_target(live),command_target(live)
        for joint in range(1,7):
            self._check_target(joint,desired[joint-1])
            self._check_target(joint,command[joint-1])
        self._check_stopped_pose(live,STABILITY_TOLERANCE,
                                 "Local approach start changed during limit checks")
        print("Checked live start:",live,flush=True)
        print("Planned J6 (held at captured start):",command[5],flush=True)
        self._record_forward_hover(command,APPROACH_TOLERANCE,
                                   direction="measured_hover_to_raised_approach",
                                   expected_start=live,
                                   start_tolerance=STABILITY_TOLERANCE,timeout=20,
                                   arrival_target=desired)
        return {"start":live,"desired":desired,"command":command,"actual":self.get_angles()}

    def _sample_grasp_start(self):
        """Require stable readings near the physically verified raised pose."""
        from .grasp import grasp_start_problem
        first=None
        for _ in range(3):
            self._require_stationary()
            actual=self.get_angles()
            problem=grasp_start_problem(actual)
            if problem is not None:
                raise RuntimeError("Grasp requires the verified centered raised pose; read "+repr(actual)+"; "+problem)
            if first is not None and not _matches(actual,first,0.25):
                raise RuntimeError("Grasp start is not stable")
            first=actual if first is None else first
            time.sleep(0.2)
        return actual

    def grasp_block(self):
        """One short verified-relative descent followed by jaw closure only."""
        from .grasp import descent_plan,require_descent_result
        live=self._sample_grasp_start()
        desired,command=descent_plan(live)
        for joint in range(1,7):
            self._check_target(joint,desired[joint-1])
            self._check_target(joint,command[joint-1])
        checked=self._sample_grasp_start()
        if not _matches(checked,live,0.25):
            raise RuntimeError("Grasp start changed during limit checks")
        self._record_forward_hover(command,1.5,direction="raised_approach_to_grasp",
                                   expected_start=live,start_tolerance=0.25,
                                   timeout=20,arrival_target=desired)
        try:
            self._require_stationary()
            actual=self.get_angles()
            require_descent_result(live,actual,desired)
            self.close_gripper()
            return {"start":live,"desired":desired,"command":command,"actual":actual}
        except BaseException:
            try:
                self.stop()
            except Exception:
                pass
            raise

    def _sample_return_checkpoint(self, expected=None):
        """Stable known drop entry or previous completed single-joint endpoint."""
        from .return_ready import known_drop
        first=None
        for _ in range(3):
            self._require_stationary()
            actual=self.get_angles()
            allowed=known_drop(actual) if expected is None else _matches(actual,expected,2.0)
            if not allowed:
                raise RuntimeError("Return requires the observed drop/return checkpoint; read "+repr(actual))
            if first is not None and not _matches(actual,first,.25):
                raise RuntimeError("Return checkpoint is not stable")
            first=actual if first is None else first
            time.sleep(.2)
        return actual

    def return_to_ready_after_drop(self,confirm_stage=None):
        """Checked return with automatic progression and paired final J5/J1.

        The operator explicitly removed return confirmations. An optional
        callback remains for diagnostic --step use; checks and stop behavior
        are retained for both modes. The old direct hover return stays locked.
        """
        if confirm_stage is not None and not callable(confirm_stage):
            raise ValueError("Return callback must be callable or None")
        from .return_ready import STAGES,stage_speed,assignments
        current=self._sample_return_checkpoint()
        # Check the entire route before its first motion, without changing
        # power, reference frames, holding mode or jaw position.
        for _,joint,value in STAGES:
            for j,v in assignments(joint,value):
                self._check_target(j,v)
        self._sample_return_checkpoint()
        expected=None
        try:
            for index,(label,joint,value) in enumerate(STAGES,1):
                current=self._sample_return_checkpoint(expected)
                # In the paired final command, reassert existing READY goals
                # for the other axes. Using measured following-error angles as
                # new targets could change their holding goals unnecessarily.
                target=list(current) if isinstance(joint,int) else list(READY_ANGLES)
                for j,v in assignments(joint,value):
                    target[j-1]=v
                target=tuple(target)
                if confirm_stage is not None and not confirm_stage(index,label,joint,current,target):
                    raise RuntimeError("Cancelled before return stage %d"%index)
                final_targets=assignments(joint,value) if isinstance(joint,int) else tuple(enumerate(READY_ANGLES,1))
                for j,v in final_targets:
                    self._check_target(j,v)
                # Recheck after limits and an optional diagnostic pause, then
                # preserve the stable measured joints outside this stage.
                live=self._sample_return_checkpoint(expected)
                target=list(live) if isinstance(joint,int) else list(READY_ANGLES)
                for j,v in assignments(joint,value):
                    target[j-1]=v
                target=tuple(target)
                self._record_forward_hover(target,2.0,
                                           direction="drop_return_stage_%d"%index,
                                           expected_start=live,start_tolerance=.5,
                                           timeout=25,joint=joint if isinstance(joint,int) else None,
                                           speed=stage_speed(joint))
                current=self.get_angles()
                expected=target
                print("Return stage %d stopped at:"%index,current,flush=True)
            self._check_stopped_pose(READY_ANGLES,POSE_TOLERANCE,
                                     "Final READY not confirmed")
            return self.get_angles()
        except BaseException:
            try:
                self.stop()
            except Exception:
                pass
            raise

    def run_ready_cycle(self,confirm_pickup,confirm_return=None):
        """Compose the demonstrated phase motions once from READY and back.

        Block placement is deferred until the raised approach, as in the
        observed run. No direct low READY-to-grasp shortcut or automatic retry.
        """
        if not callable(confirm_pickup):
            raise RuntimeError("Full hardware cycle requires block-placement confirmation")
        if confirm_return is not None and not callable(confirm_return):
            raise ValueError("Return callback must be callable or None")
        from .full_cycle import POST_HOVER_OBSERVED,POST_HOVER_COMMAND
        from .resume_hover import HOVER
        from .local_approach import desired_target,command_target
        from .grasp import VERIFIED_RAISED_ANGLES,descent_plan
        from .return_ready import STAGES,assignments
        self.check_hover_start()
        # Preflight reference plans for all phases, including return, before
        # opening jaws. Dynamic live-J6 plans are also checked when calculated.
        descent_desired,descent_command=descent_plan(VERIFIED_RAISED_ANGLES)
        for pose in (HOVER,POST_HOVER_COMMAND,POST_HOVER_OBSERVED,
                     desired_target(),command_target(),descent_desired,descent_command):
            for joint,angle in enumerate(pose,1):
                self._check_target(joint,angle)
        for joint,value in ((2,-10.0),(1,91.0)):
            self._check_target(joint,value)
        for _,joint,value in STAGES:
            for j,v in assignments(joint,value):
                self._check_target(j,v)
        self.check_hover_start()
        try:
            self.open_gripper()
            self._wait_for_gripper()
            print("Phase: single-command high HOVER",flush=True)
            self.move_to_hover(continuous=True)
            high=self._check_stopped_pose(HOVER,1.5,"High HOVER checkpoint changed")
            # Replay the command that produced the measured post-alignment
            # checkpoint. Keep the current signed wrist angle; never spin it
            # back to the stale value embedded in the historical snapshot.
            desired=POST_HOVER_OBSERVED[:4]+(high[4],high[5])
            command=POST_HOVER_COMMAND[:4]+(high[4],high[5])
            for joint,angle in enumerate(command,1):
                self._check_target(joint,angle)
            self._check_stopped_pose(high,.25,"High HOVER changed before taught transfer")
            print("Phase: taught high-hover transfer",flush=True)
            self._record_forward_hover(command,1.0,direction="high_hover_to_taught_checkpoint",
                                       expected_start=high,start_tolerance=.25,
                                       timeout=20,arrival_target=desired)
            post=self._check_stopped_pose(desired,1.0,"Taught transfer checkpoint not confirmed")
            print("Phase: measured-reference raised approach",flush=True)
            approach=self._move_local_approach_from_live(post)
            raised=self._sample_grasp_start()
            if not confirm_pickup(raised):
                raise RuntimeError("Cancelled at raised pickup; no descent or closure")
            print("Phase: grasp, lift and sweep/drop",flush=True)
            cycle=self.pick_and_place(jaws_already_open=True)
            print("Phase: staged empty-gripper return",flush=True)
            ready=self.return_to_ready_after_drop(confirm_return)
            return {"approach":approach,"cycle":cycle,"ready":ready}
        except BaseException:
            try:
                self.stop()
            except Exception:
                pass
            raise

    def _wait_for_gripper(self, timeout=10, *, allow_contact=False):
        """Wait for stopped jaws, or stable partial position after loaded closure.

        A held block can prevent fully closed arrival. For closure only, accept
        valid partial-position telemetry stable within one unit for at least
        one second, after a two-second minimum, even if moving remains 1.
        This detects stationary contact, not object identity or holding force.
        Opening still requires the normal stopped flag; no blind-delay fallback.
        """
        started,settled=time.monotonic(),0
        stable_since=None
        anchor=None
        last_moving=last_value=None
        while time.monotonic()-started<timeout:
            self._require_stationary()
            moving=self._client.is_gripper_moving()
            last_moving=moving
            if type(moving) is not int or moving not in (0,1):
                raise RuntimeError("Invalid gripper motion status: "+repr(moving))
            now=time.monotonic()
            if allow_contact:
                value=self._client.get_gripper_value(1)
                last_value=value
                if type(value) is not int or not 0<=value<=100:
                    raise RuntimeError("Invalid adaptive-gripper position: "+repr(value))
                if anchor is None or abs(value-anchor)>1:
                    anchor,stable_since=value,now
                if value>=95:
                    settled=0  # A closure that remains open must not permit lift.
                else:
                    settled=settled+1 if moving==0 else 0
                contact=(0<value<95 and stable_since is not None and now-stable_since>=1)
            else:
                settled=settled+1 if moving==0 else 0
                contact=False
            if now-started>=2 and (settled>=3 or contact):
                if allow_contact:
                    print("Gripper closure ready: moving=%r, stable position=%r"%(moving,last_value),flush=True)
                return
            time.sleep(0.2)
        raise RuntimeError("Gripper did not settle; no next motion; last moving=%r, value=%r"%(last_moving,last_value))

    def _sample_held_pickup_start(self):
        """Validate a stopped grasped-block pose without opening or reclosing."""
        from .pick_and_place import known_held_start
        first=None
        for _ in range(3):
            self._require_stationary()
            actual=self.get_angles()
            if not known_held_start(actual):
                raise RuntimeError("Resume requires the known post-grasp pose; read "+repr(actual))
            if first is not None and not _matches(actual,first,.25):
                raise RuntimeError("Post-grasp pose is not stable")
            first=actual if first is None else first
            time.sleep(.2)
        return actual

    def finish_held_pickup(self):
        """Resume lift/sweep/release from a held grasp; never open before lifting."""
        from .pick_and_place import LIFT_J2_COMMAND,SWEEP_J1_COMMAND
        grasped=self._sample_held_pickup_start()
        self._check_target(2,LIFT_J2_COMMAND)
        self._check_target(1,SWEEP_J1_COMMAND)
        self._check_stopped_pose(grasped,.25,"Post-grasp pose changed during limit checks")
        try:
            self._wait_for_gripper(allow_contact=True)
            return self._finish_grasped_cycle(grasped)
        except BaseException:
            try:
                self.stop()
            except Exception:
                pass
            raise

    def pick_and_place(self,*,jaws_already_open=False):
        """One local grasp, J2-only lift, J1-only sweep and release; no return.

        Only the separate demonstrated phase motions are known from hardware.
        The single lift replaces two monotonic J2 steps over the same interval;
        a single sweep replaces the observed monotonic J1 steps. This combined
        sequence and gripper polling still need a supervised hardware run.
        """
        from .pick_and_place import (LIFT_J2_COMMAND,LIFT_J2_OBSERVED,
                                    SWEEP_J1_COMMAND,SWEEP_J1_OBSERVED)
        from .grasp import descent_plan
        from .kinematics import gripper_midpoint_mat
        start=self._sample_grasp_start()  # Reject drop/ready before opening jaws.
        desired,command=descent_plan(start)
        # Preflight every planned joint target before any command, including
        # future lift/sweep limits. Individual phases recheck before execution.
        for joint in range(1,7):
            self._check_target(joint,desired[joint-1])
            self._check_target(joint,command[joint-1])
        self._check_target(2,LIFT_J2_COMMAND)
        self._check_target(1,SWEEP_J1_COMMAND)
        self._check_stopped_pose(start,0.25,"Pick-and-place starting pose changed")
        try:
            if not jaws_already_open:
                self.open_gripper()
                self._wait_for_gripper()
            grasp=self.grasp_block()
            self._wait_for_gripper(allow_contact=True)  # Wait for stopped/steady loaded closure.
            at_grasp=grasp['actual']
            return self._finish_grasped_cycle(at_grasp)
        except BaseException:
            try:
                self.stop()
            except Exception:
                pass
            raise

    def _finish_grasped_cycle(self,at_grasp):
        """Shared guarded lift/sweep/release after closure or explicit resume."""
        from .pick_and_place import (LIFT_J2_COMMAND,LIFT_J2_OBSERVED,
                                    SWEEP_J1_COMMAND,SWEEP_J1_OBSERVED)
        from .kinematics import gripper_midpoint_mat
        self._check_stopped_pose(at_grasp,0.5,"Grasp pose changed before lift")
        self._check_target(2,LIFT_J2_COMMAND)
        lift_command=list(at_grasp)
        lift_command[1]=LIFT_J2_COMMAND
        lift_goal=list(at_grasp)
        lift_goal[1]=LIFT_J2_OBSERVED
        self._record_forward_hover(tuple(lift_command),1.5,
                                   direction="grasp_to_lifted",expected_start=at_grasp,
                                   start_tolerance=0.5,timeout=20,joint=2,
                                   arrival_target=tuple(lift_goal))
        lifted=self.get_angles()
        rise=(gripper_midpoint_mat(lifted)[2]-gripper_midpoint_mat(at_grasp)[2])/25.4
        if abs(lifted[1]-LIFT_J2_OBSERVED)>0.4 or rise<0.5:
            raise RuntimeError("Lift clearance endpoint not confirmed; no sweep or release")
        self._check_stopped_pose(lifted,0.5,"Lifted pose changed before sweep")
        self._check_target(1,SWEEP_J1_COMMAND)
        sweep_command=list(lifted)
        sweep_command[0]=SWEEP_J1_COMMAND
        sweep_goal=list(lifted)
        sweep_goal[0]=SWEEP_J1_OBSERVED
        self._record_forward_hover(tuple(sweep_command),1.5,
                                   direction="lifted_to_drop",expected_start=lifted,
                                   start_tolerance=0.5,timeout=20,joint=1,
                                   arrival_target=tuple(sweep_goal))
        self._require_stationary()
        drop=self.get_angles()
        height_change=abs(gripper_midpoint_mat(drop)[2]-gripper_midpoint_mat(lifted)[2])/25.4
        if abs(drop[0]-SWEEP_J1_OBSERVED)>1 or height_change>0.125:
            raise RuntimeError("Drop endpoint not confirmed; jaws left holding")
        self.open_gripper()
        self._wait_for_gripper()
        return {"grasp":at_grasp,"lifted":lifted,"drop":drop}

    def square_hover(self):
        """One high-hover alignment test; no automatic trims, descent or grasp."""
        from .hover_alignment import START_ANGLES, START_TOLERANCE, alignment_target
        label = "Hover alignment requires the recorded high-hover pose"
        target = alignment_target()
        self._check_stopped_pose(START_ANGLES, START_TOLERANCE, label)
        for joint, angle in enumerate(target, 1):
            self._check_target(joint, angle)
        self._check_stopped_pose(START_ANGLES, START_TOLERANCE, label)
        self._record_forward_hover(target, 1.5, direction="high_hover_alignment",
                                   expected_start=START_ANGLES,
                                   start_tolerance=START_TOLERANCE, timeout=20)
        return self.get_angles()

    def move_to_raised_approach(self):
        """Raise J2 only from the known pose; never command the wrist or jaws."""
        from .raised_approach import START_ANGLES, START_TOLERANCE, raised_target
        target = raised_target()
        self._check_raised_approach_start()
        for joint, angle in enumerate(target, 1):
            self._check_target(joint, angle)
        self._check_raised_approach_start()
        # J6 is already aligned near +161 degrees. Never reuse this route from
        # ready, where J6 is -135 and the previous late rotation occurred.
        self._record_forward_hover(target, 1.5, direction="stopped_to_raised_approach",
                                   expected_start=START_ANGLES,
                                   start_tolerance=START_TOLERANCE, timeout=20, joint=2)
        return self.get_angles()

    def pick_up_block(self, grasp_height_inches=0.625):
        """Approach (7, 2) once from ready, then command jaw closure.

        Requires the operator-confirmed clearance and jaws already open. Arrival
        is checked before closure; no correction, lift or return is attempted.
        Closure is commanded, not a sensed confirmation that a block is held.
        """
        from .pickup import pickup_target, require_grasp_position, require_pickup_execution
        require_pickup_execution()
        target = pickup_target(grasp_height_inches)
        self.check_hover_start()
        for joint, angle in enumerate(target, 1):
            self._check_target(joint, angle)
        self.check_hover_start()  # Recheck after querying live joint limits.
        try:
            self._record_forward_hover(target, 1.5, direction="ready_to_pickup")
            self._require_stationary()
            actual = self.get_angles()
            if not _matches(actual, target, 1.5):
                raise RuntimeError("Pickup endpoint changed before jaw closure")
            require_grasp_position(actual)
            self.close_gripper()
            return actual
        except BaseException:
            try:
                self.stop()
            except Exception:
                pass  # Keep the original motion or closure failure visible.
            raise

    def move_from_hover_to_ready(self, synchronous=True, continuous=False):
        raise RuntimeError("Hover return disabled after reported self-contact. "
                           "A measured forward trajectory must be recorded and reviewed first.")

    def _record_forward_hover(self, target, tolerance, *, direction="ready_to_hover",
                              expected_start=READY_ANGLES, start_tolerance=POSE_TOLERANCE,
                              timeout=MOVE_TIMEOUT, joint=None, arrival_target=None, speed=POSE_SPEED):
        """Send one forward command and capture actual motion instead of blocking SDK waits."""
        from .hover_trajectory import save_forward_trace
        validate_speed(speed)
        samples, complete, failure = [], False, None
        expected_arrival = target if arrival_target is None else arrival_target
        started = time.monotonic()

        def capture():
            moving = self._motion_status()
            actual = self.get_angles()
            samples.append({"elapsed_s": time.monotonic() - started,
                            "angles_deg": list(actual), "moving": moving, "error": 0})
            return moving, actual

        try:
            moving, actual = capture()
            if moving != 0 or not _matches(actual, expected_start, start_tolerance):
                raise RuntimeError("Recording must start stationary at the required pose")
            if joint is None:
                acknowledgement = self._client.send_angles(list(target), speed)
            else:
                acknowledgement = self._client.send_angle(joint, target[joint - 1], speed)
            if type(acknowledgement) is not int or acknowledgement != 1:
                raise RuntimeError("Forward command acknowledgement failed")
            settled = 0
            while time.monotonic() - started < timeout:
                time.sleep(0.1)
                moving, actual = capture()
                reached = _matches(actual, expected_arrival, tolerance)
                settled = settled + 1 if moving == 0 and reached else 0
                if settled >= 3:
                    complete = True
                    return True
            errors = tuple(round(a-b,3) for a,b in zip(actual,expected_arrival))
            raise RuntimeError("Forward capture timed out before target was confirmed; "
                               "last angles=%r; errors=%r" % (actual,errors))
        except BaseException as exc:
            failure = str(exc) or type(exc).__name__
            try:
                self.stop()
            except Exception:
                pass
            raise
        finally:
            try:
                options={}
                if direction!="ready_to_hover":
                    options['direction']=direction
                if arrival_target is not None:
                    options['command_angles']=target
                    options['arrival_angles']=expected_arrival
                if speed!=POSE_SPEED:
                    options['speed']=speed
                path=save_forward_trace(samples,complete,failure,**options)
                print("Measured forward recording:", path, flush=True)
            except OSError as exc:
                if complete:
                    raise RuntimeError("Target reached but recording could not be saved: " + str(exc)) from exc
                print("Recording could not be saved:", exc, flush=True)

    def _move_hover_route(self, resume_lift3, synchronous, continuous):
        """Run a monitored direct hover or the explicitly selected historical route.

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
        if continuous and (resume_lift3 or synchronous):
            raise ValueError("Continuous hover cannot use Lift 3 recovery or synchronous waypoints")
        waypoints = [("Continuous hover", HOVER, 1.5)] if continuous else route(resume_lift3=resume_lift3)
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

            # Observe the final endpoint without issuing corrective movements.
            actual = require_pose(HOVER, 1.5)
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

    def _wait_for_target(self, target, tolerance=POSE_TOLERANCE, *, timeout=SETTLE_TIMEOUT):
        """Confirm arrival across three consecutive stationary, fault-free samples."""
        started = time.monotonic()
        settled = 0
        while time.monotonic() - started < timeout:
            moving = self._motion_status()
            current = self.get_angles()
            reached = _matches(current, target, tolerance)
            settled = settled + 1 if moving == 0 and reached else 0
            if settled >= 3:
                return current
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

    def move_joint(self, joint, target, speed=DEFAULT_JOINT_SPEED, *, wait=False):
        validate_speed(speed)
        _joint_target(joint, target)
        updated = list(self._angles)
        updated[joint - 1] = target
        self._angles = tuple(updated)
        if wait:
            return self._angles

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

    def move_to_hover(self, resume_lift3=False, synchronous=False, continuous=None):
        from .resume_hover import HOVER, LIFT3_READBACK
        if continuous and (resume_lift3 or synchronous):
            raise ValueError("Continuous hover cannot use Lift 3 recovery or synchronous waypoints")
        if resume_lift3:
            if any(abs(a - b) > 0.5 for a, b in zip(self._angles, LIFT3_READBACK)):
                raise RuntimeError("Hover requires recorded Lift 3 checkpoint")
            self._angles = HOVER
            return True
        return self._move_between(READY_ANGLES, HOVER)

    def move_to_local_approach(self):
        from .local_approach import known_start, desired_target, command_target
        if not known_start(self._angles):
            raise RuntimeError("Local approach requires the observed post-alignment region")
        live=self._angles
        desired,command=desired_target(live),command_target(live)
        self._angles=desired
        return {"start":live,"desired":desired,"command":command,"actual":self._angles}

    def grasp_block(self):
        from .grasp import known_grasp_start,descent_plan,require_descent_result
        if not known_grasp_start(self._angles):
            raise RuntimeError("Grasp requires the verified centered raised pose")
        live=self._angles
        desired,command=descent_plan(live)
        self._angles=desired
        require_descent_result(live,self._angles,desired)
        self.close_gripper()
        return {"start":live,"desired":desired,"command":command,"actual":self._angles}

    def run_ready_cycle(self,confirm_pickup=None,confirm_return=None):
        """Replay the observed phase checkpoints; not simulated tracking physics."""
        from .full_cycle import HIGH_HOVER_OBSERVED,POST_HOVER_OBSERVED
        from .grasp import VERIFIED_RAISED_ANGLES
        if not _matches(self._angles,READY_ANGLES):
            raise RuntimeError("Full cycle requires saved READY")
        self.open_gripper()
        self._angles=HIGH_HOVER_OBSERVED
        self._angles=POST_HOVER_OBSERVED
        # Replay the physically observed raised pose rather than asserting
        # that the instantaneous fake models the real actuator following error.
        self._angles=VERIFIED_RAISED_ANGLES
        if confirm_pickup is not None and not confirm_pickup(self._angles):
            raise RuntimeError("Cancelled at raised pickup")
        cycle=self.pick_and_place(jaws_already_open=True)
        ready=self.return_to_ready_after_drop(confirm_return)
        return {"approach":{"actual":VERIFIED_RAISED_ANGLES},"cycle":cycle,"ready":ready}

    def return_to_ready_after_drop(self,confirm_stage=None):
        from .return_ready import known_drop,STAGES,assignments
        if not known_drop(self._angles):
            raise RuntimeError("Return requires observed drop pose")
        for index,(label,joint,value) in enumerate(STAGES,1):
            target=list(self._angles)
            for j,v in assignments(joint,value):
                target[j-1]=v
            target=tuple(target)
            if confirm_stage is not None and not confirm_stage(index,label,joint,self._angles,target):
                raise RuntimeError("Cancelled before return stage %d"%index)
            self._angles=target
        return self._angles

    def finish_held_pickup(self):
        from .pick_and_place import known_held_start,LIFT_J2_OBSERVED,SWEEP_J1_OBSERVED
        if not known_held_start(self._angles):
            raise RuntimeError("Resume requires known post-grasp pose")
        grasp=self._angles
        lifted=list(grasp)
        lifted[1]=LIFT_J2_OBSERVED
        drop=lifted.copy()
        drop[0]=SWEEP_J1_OBSERVED
        self._angles=tuple(drop)
        self.open_gripper()
        return {"grasp":grasp,"lifted":tuple(lifted),"drop":self._angles}

    def pick_and_place(self,*,jaws_already_open=False):
        """Nominal instant phase simulation; no load or grasp sensing."""
        from .grasp import known_grasp_start
        from .pick_and_place import LIFT_J2_OBSERVED,SWEEP_J1_OBSERVED
        if not known_grasp_start(self._angles):
            raise RuntimeError("Pick-and-place requires verified raised pose")
        if not jaws_already_open:
            self.open_gripper()
        grasp=self.grasp_block()['actual']
        lifted=list(grasp)
        lifted[1]=LIFT_J2_OBSERVED
        self._angles=tuple(lifted)
        drop=lifted.copy()
        drop[0]=SWEEP_J1_OBSERVED
        self._angles=tuple(drop)
        self.open_gripper()
        return {"grasp":grasp,"lifted":tuple(lifted),"drop":self._angles}

    def square_hover(self):
        from .hover_alignment import START_ANGLES, START_TOLERANCE, alignment_target
        if not _matches(self._angles, START_ANGLES, START_TOLERANCE):
            raise RuntimeError("Hover alignment requires the recorded high-hover pose")
        self._angles = alignment_target()
        return self._angles

    def move_to_raised_approach(self):
        from .raised_approach import START_ANGLES, START_TOLERANCE, raised_target
        if not _matches(self._angles, START_ANGLES, START_TOLERANCE):
            raise RuntimeError("Raised approach requires recorded J2-test pose")
        self._angles = raised_target()
        return self._angles

    def pick_up_block(self, grasp_height_inches=0.625):
        """Simulate the same ready-only approach and closure without physics."""
        from .pickup import pickup_target, require_grasp_position
        target = pickup_target(grasp_height_inches)
        if not _matches(self._angles, READY_ANGLES):
            raise RuntimeError("Pickup requires saved ready")
        self._angles = target
        require_grasp_position(self._angles)
        self.close_gripper()
        return self._angles

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
