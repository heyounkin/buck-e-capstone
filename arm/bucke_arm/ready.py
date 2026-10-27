"""Offline ready-pose preview. No robot connection or movement commands."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

from .ready_config import JointLimits, ReadyRoute, angles, load_limits


# User-corrected ready target, in J1-J6 order (October 21, 2026).
# This is a joint target, not a reviewed route from an arbitrary starting pose.
READY_ANGLES_DEG = (-90.0, 0.0, 0.0, 0.0, 90.0, -135.0)


@dataclass(frozen=True)
class JointChange:
    joint: str
    current_deg: float
    target_deg: float
    delta_deg: float


@dataclass(frozen=True)
class WaypointPreview:
    step: int
    from_angles_deg: Tuple[float, ...]
    to_angles_deg: Tuple[float, ...]
    joint_changes: Tuple[JointChange, ...]


@dataclass(frozen=True)
class ReadyPreview:
    mode: str
    current_angles_deg: Tuple[float, ...]
    target_angles_deg: Tuple[float, ...]
    joint_changes: Tuple[JointChange, ...]
    exactly_at_target: bool
    configured_limits: JointLimits
    route: Optional[ReadyRoute]
    waypoint_steps: Tuple[WaypointPreview, ...]
    hardware_execution_enabled: bool = False
    route_reviewed: bool = False
    joint_limits_verified: bool = False
    notes: Tuple[str, ...] = (
        "Supplied state is not live telemetry; no robot connection is opened.",
        "Overall joint changes are listed in J1-J6 order, not an execution sequence.",
        "Signed differences are not wrapped modulo 360 degrees.",
        "Limit verification is supplied configuration metadata, not a hardware query.",
        "Configured bounds do not establish collision clearance or controller interpolation.",
        "Waypoint steps describe an unreviewed candidate route, never movement commands.",
        "With no selected route, waypoint_steps is empty; no path has been chosen.",
        "Physical clearance, controller limits, speed, and endpoint tolerance "
        "must be established before hardware execution.",
        "An exact numeric match does not establish physical accuracy.",
    )


def move_to_ready(
    current_angles_deg: Sequence[float], *, moving: int, error_information: int,
    limits: Optional[JointLimits] = None, route: Optional[ReadyRoute] = None
) -> ReadyPreview:
    """Describe changes to ready without moving or choosing a physical route.

    Both status fields must be supplied explicitly. Zero values describe the
    caller's input; this function cannot verify the arm's actual health. A
    faulted or moving input is rejected rather than turned into a movement plan.
    Bounds are checked against the supplied limits or the provisional defaults.
    An explicitly selected route must support the starting pose, and its entire
    definition is validated before any preview is returned. No route is selected
    automatically and physical clearance is never certified.
    """
    if type(moving) is not int or moving not in (0, 1):
        raise ValueError("moving must be integer 0 (stationary) or 1 (moving)")
    if type(error_information) is not int or error_information < 0:
        raise ValueError("error_information must be a nonnegative integer")
    if error_information != 0:
        raise ValueError(
            "Controller error %d: ready preview refused; diagnose the error first"
            % error_information
        )
    if moving != 0:
        raise ValueError("Arm is moving: ready preview requires a stationary input")
    current = angles(current_angles_deg, "Current pose")
    if limits is None:
        limits = load_limits()
    if not isinstance(limits, JointLimits):
        raise ValueError("limits must be a JointLimits configuration")
    limits.check(current, "Current pose")
    limits.check(READY_ANGLES_DEG, "Ready target")

    steps = []
    if route is not None:
        if not isinstance(route, ReadyRoute):
            raise ValueError("route must be a ReadyRoute configuration")
        limits.check(route.start_min_deg, "Route start minimum")
        limits.check(route.start_max_deg, "Route start maximum")
        for index, waypoint in enumerate(route.waypoints_deg, 1):
            limits.check(waypoint, "Waypoint %d" % index)
        route.check_start(current)
        previous = current
        # Always finish at the configured ready target, skipping numeric no-ops.
        # Validate every waypoint above, including any later segment, first.
        if current != READY_ANGLES_DEG:
            for waypoint in route.waypoints_deg + (READY_ANGLES_DEG,):
                if waypoint != previous:
                    steps.append(WaypointPreview(
                        len(steps) + 1, previous, waypoint, joint_changes(previous, waypoint)
                    ))
                    previous = waypoint
    return ReadyPreview(
        mode="preview_only",
        current_angles_deg=current,
        target_angles_deg=READY_ANGLES_DEG,
        joint_changes=joint_changes(current, READY_ANGLES_DEG),
        exactly_at_target=current == READY_ANGLES_DEG,
        configured_limits=limits,
        joint_limits_verified=limits.verified,
        route=route,
        waypoint_steps=tuple(steps),
    )


def joint_changes(current, target):
    changes = tuple(
        JointChange("J%d" % (index + 1), angle, goal, goal - angle)
        for index, (angle, goal) in enumerate(zip(current, target))
    )
    if any(not math.isfinite(change.delta_deg) for change in changes):
        raise ValueError("Joint angle differences must be finite")
    return changes
