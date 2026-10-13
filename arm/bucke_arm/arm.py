"""Read-only interface and explicitly simulated telemetry."""

from dataclasses import dataclass
import math
from typing import Protocol, Tuple


# A dataclass generates the constructor for a simple data container.
# frozen=True prevents assigning new field values after it is created.
@dataclass(frozen=True)
class Telemetry:
    # Type hints describe the expected data; validate() checks it at runtime.
    # source distinguishes test data from future physical robot readings.
    source: str
    # Six angles, ordered by joint number, expressed in degrees.
    joint_angles_deg: Tuple[float, ...]
    # Intended format: x, y, z in mm, followed by three orientation values
    # in degrees. The real device's frame and orientation convention still
    # need verification. Tuple[float, ...] means a tuple of numeric values;
    # its length is checked below rather than enforced by the type hint.
    tool_pose_mm_deg: Tuple[float, ...]

    def validate(self) -> None:
        # Reject data whose origin is not explicitly identified.
        if self.source not in ("simulated", "hardware"):
            raise ValueError("Telemetry source must be simulated or hardware")
        # Apply the same shape and number checks to both sets of readings.
        for name, values in (
            ("joint angles", self.joint_angles_deg),
            ("tool pose", self.tool_pose_mm_deg),
        ):
            if len(values) != 6:
                raise ValueError("Expected six values for " + name)
            # any() is true if even one value fails these checks.
            # Reject booleans explicitly: Python treats bool as a kind of int.
            # isfinite() rejects NaN (not a number) and positive/negative infinity.
            if any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                for value in values
            ):
                raise ValueError("Expected finite numeric values for " + name)


# A Protocol describes a common interface for type checkers. A fake or real
# implementation can satisfy it by providing read_status(); inheritance is
# not required. It does not enforce hardware safety at runtime.
class ReadOnlyArm(Protocol):
    def read_status(self) -> Telemetry:
        """Return telemetry without issuing motion commands."""
        # This is an interface declaration, not a working implementation.
        ...

    def close(self) -> None:
        """Close the connection without issuing motion commands."""
        ...


class FakeArm:
    """Deterministic software fixture, not a physical robot model."""

    def read_status(self) -> Telemetry:
        # These fixed values let us develop on a laptop. They are not safe
        # motion targets, calibration, measured readings, or a physics model.
        telemetry = Telemetry(
            source="simulated",
            joint_angles_deg=(10.0, -20.0, 30.0, -40.0, 50.0, -60.0),
            tool_pose_mm_deg=(100.0, 20.0, 150.0, 0.0, 0.0, 0.0),
        )
        # Check the fake readings using the same rules as future real data.
        telemetry.validate()
        return telemetry

    def close(self) -> None:
        # The fake owns no connection, so cleanup does nothing.
        pass
