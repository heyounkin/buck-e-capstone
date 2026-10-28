"""Validated offline limits and route definitions; no hardware access."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Tuple


# Resolve relative to this package rather than the terminal's working directory.
# This keeps the default configuration usable when Python is launched elsewhere.
DEFAULT_LIMITS_PATH = Path(__file__).resolve().parent.parent / "config" / "ready_limits.provisional.json"


def angles(value, label):
    # Require one finite number per joint, rejecting booleans, text, NaN, and
    # infinity. A tuple copies caller data into the frozen configuration records.
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(label + " must contain six angles in J1-J6 order")

    if any(isinstance(x, bool) or not isinstance(x, (int, float))
           or not math.isfinite(x) for x in value):
        raise ValueError(label + " must contain six finite numbers, not booleans")

    return tuple(float(x) for x in value)


def text_field(value, label):
    # A name or provenance field containing only spaces is not useful metadata.
    if not isinstance(value, str) or not value.strip():
        raise ValueError(label + " must be a nonempty string")


def ordered_bounds(minimum, maximum, label):
    # Equal bounds are allowed: a range can constrain a joint to one exact angle.
    for index, (low, high) in enumerate(zip(minimum, maximum), 1):
        if low > high:
            raise ValueError("%s J%d minimum exceeds maximum" % (label, index))


@dataclass(frozen=True)
class JointLimits:
    # These are numeric planning bounds with a source, not physical guarantees.
    minimum_deg: Tuple[float, ...]
    maximum_deg: Tuple[float, ...]
    source: str
    verified: bool = False

    def __post_init__(self):
        # Dataclasses call this after their generated constructor. Frozen objects
        # need object.__setattr__ here to normalize lists into immutable tuples.
        object.__setattr__(self, "minimum_deg", angles(self.minimum_deg, "Limit minimum"))
        object.__setattr__(self, "maximum_deg", angles(self.maximum_deg, "Limit maximum"))
        ordered_bounds(self.minimum_deg, self.maximum_deg, "Limits")
        text_field(self.source, "Limit source")
        if type(self.verified) is not bool:
            raise ValueError("Limit verified must be a boolean")

    def check(self, pose, label):
        # Check inclusive bounds without clamping the angle or wrapping its sign.
        # Include the pose label and joint number to locate a configuration error.
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
    # Each joint has an independent allowed starting interval. Together they
    # form a six-dimensional box, not a collision model or calibrated tolerance.
    name: str
    description: str
    start_min_deg: Tuple[float, ...]
    start_max_deg: Tuple[float, ...]
    waypoints_deg: Tuple[Tuple[float, ...], ...]

    def __post_init__(self):
        # Validate direct Python construction as well as data loaded from JSON.
        text_field(self.name, "Route name")
        text_field(self.description, "Route description")
        object.__setattr__(self, "start_min_deg", angles(self.start_min_deg, "Start minimum"))
        object.__setattr__(self, "start_max_deg", angles(self.start_max_deg, "Start maximum"))
        ordered_bounds(self.start_min_deg, self.start_max_deg, "Starting range")
        if not isinstance(self.waypoints_deg, (list, tuple)):
            raise ValueError("waypoints_deg must be a list of six-angle poses")

        # An empty waypoint list is valid; the planner still appends ready.
        # Freeze the inner poses too, so later list edits cannot change a route.
        object.__setattr__(self, "waypoints_deg", tuple(
            angles(pose, "Waypoint %d" % index)
            for index, pose in enumerate(self.waypoints_deg, 1)
        ))

    def check_start(self, pose):
        # Every joint must match its range. Do not silently select another route.
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

    # The dataclass's field names define the exact JSON schema for this loader.
    required = set(cls.__dataclass_fields__)
    if not isinstance(data, dict) or set(data) != required:
        raise ValueError("%s configuration must contain exactly: %s"
                         % (cls.__name__, ", ".join(sorted(required))))

    # ** expands JSON keys into constructor arguments; __post_init__ validates
    # their values. JSON syntax and file errors propagate to the CLI handler.
    return cls(**data)


def load_limits(path=DEFAULT_LIMITS_PATH):
    # Callers may supply a separate controller-derived file without editing code.
    return load_config(path, JointLimits)


def load_route(path):
    # A route is always explicitly selected; there is no default physical route.
    return load_config(path, ReadyRoute)
