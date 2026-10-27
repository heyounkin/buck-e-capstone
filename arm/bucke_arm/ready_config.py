"""Validated offline limits and route definitions; no hardware access."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Tuple


DEFAULT_LIMITS_PATH = Path(__file__).resolve().parent.parent / "config" / "ready_limits.provisional.json"


def angles(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(label + " must contain six angles in J1-J6 order")
    if any(isinstance(x, bool) or not isinstance(x, (int, float))
           or not math.isfinite(x) for x in value):
        raise ValueError(label + " must contain six finite numbers, not booleans")
    return tuple(float(x) for x in value)


def text_field(value, label):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(label + " must be a nonempty string")


def ordered_bounds(minimum, maximum, label):
    for index, (low, high) in enumerate(zip(minimum, maximum), 1):
        if low > high:
            raise ValueError("%s J%d minimum exceeds maximum" % (label, index))


@dataclass(frozen=True)
class JointLimits:
    minimum_deg: Tuple[float, ...]
    maximum_deg: Tuple[float, ...]
    source: str
    verified: bool = False

    def __post_init__(self):
        object.__setattr__(self, "minimum_deg", angles(self.minimum_deg, "Limit minimum"))
        object.__setattr__(self, "maximum_deg", angles(self.maximum_deg, "Limit maximum"))
        ordered_bounds(self.minimum_deg, self.maximum_deg, "Limits")
        text_field(self.source, "Limit source")
        if type(self.verified) is not bool:
            raise ValueError("Limit verified must be a boolean")

    def check(self, pose, label):
        for index, (angle, low, high) in enumerate(
            zip(angles(pose, label), self.minimum_deg, self.maximum_deg), 1
        ):
            if not low <= angle <= high:
                raise ValueError(
                    "%s J%d angle %g is outside configured limits [%g, %g]"
                    % (label, index, angle, low, high)
                )


@dataclass(frozen=True)
class ReadyRoute:
    name: str
    description: str
    start_min_deg: Tuple[float, ...]
    start_max_deg: Tuple[float, ...]
    waypoints_deg: Tuple[Tuple[float, ...], ...]

    def __post_init__(self):
        text_field(self.name, "Route name")
        text_field(self.description, "Route description")
        object.__setattr__(self, "start_min_deg", angles(self.start_min_deg, "Start minimum"))
        object.__setattr__(self, "start_max_deg", angles(self.start_max_deg, "Start maximum"))
        ordered_bounds(self.start_min_deg, self.start_max_deg, "Starting range")
        if not isinstance(self.waypoints_deg, (list, tuple)):
            raise ValueError("waypoints_deg must be a list of six-angle poses")
        object.__setattr__(self, "waypoints_deg", tuple(
            angles(pose, "Waypoint %d" % index)
            for index, pose in enumerate(self.waypoints_deg, 1)
        ))

    def check_start(self, pose):
        for index, (angle, low, high) in enumerate(
            zip(angles(pose, "Current pose"), self.start_min_deg, self.start_max_deg), 1
        ):
            if not low <= angle <= high:
                raise ValueError(
                    "Route %s does not support starting J%d angle %g; expected [%g, %g]"
                    % (self.name, index, angle, low, high)
                )


def load_config(path, cls):
    # Reject unknown fields so misspelled bounds cannot silently be ignored.
    with Path(path).open(encoding="utf-8") as stream:
        data = json.load(stream)
    required = set(cls.__dataclass_fields__)
    if not isinstance(data, dict) or set(data) != required:
        raise ValueError("%s configuration must contain exactly: %s"
                         % (cls.__name__, ", ".join(sorted(required))))
    return cls(**data)


def load_limits(path=DEFAULT_LIMITS_PATH):
    return load_config(path, JointLimits)


def load_route(path):
    return load_config(path, ReadyRoute)
