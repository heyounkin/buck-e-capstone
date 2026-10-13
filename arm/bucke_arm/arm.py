"""Read-only interface and explicitly simulated telemetry."""

from dataclasses import dataclass
import math
from typing import Protocol, Tuple


@dataclass(frozen=True)
class Telemetry:
    source: str
    joint_angles_deg: Tuple[float, ...]
    tool_pose_mm_deg: Tuple[float, ...]

    def validate(self) -> None:
        if self.source not in ("simulated", "hardware"):
            raise ValueError("Telemetry source must be simulated or hardware")
        for name, values in (
            ("joint angles", self.joint_angles_deg),
            ("tool pose", self.tool_pose_mm_deg),
        ):
            if len(values) != 6:
                raise ValueError("Expected six values for " + name)
            if any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                for value in values
            ):
                raise ValueError("Expected finite numeric values for " + name)


class ReadOnlyArm(Protocol):
    def read_status(self) -> Telemetry:
        """Return telemetry without issuing motion commands."""
        ...


class FakeArm:
    """Deterministic software fixture, not a physical robot model."""

    def read_status(self) -> Telemetry:
        telemetry = Telemetry(
            source="simulated",
            joint_angles_deg=(10.0, -20.0, 30.0, -40.0, 50.0, -60.0),
            tool_pose_mm_deg=(100.0, 20.0, 150.0, 0.0, 0.0, 0.0),
        )
        telemetry.validate()
        return telemetry
