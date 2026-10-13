"""Simulated status CLI. Hardware execution is unavailable."""

# argparse reads terminal arguments; asdict and json format the output.
# sys supplies the error stream and lets us return an exit code to the shell.
import argparse
from dataclasses import asdict
import json
import sys
from typing import Optional, Sequence

# The leading dot imports from this same package. No robot SDK is loaded.
from .arm import FakeArm


def main(argv: Optional[Sequence[str]] = None) -> int:
    # argv=None uses the terminal arguments. Tests can pass a list instead.
    # The function returns 0 on success and 1 for a status failure.
    parser = argparse.ArgumentParser(description="Buck-E read-only arm status")
    # Only status is supported; this starter has no motion commands.
    parser.add_argument("command", choices=("status",))
    # A backend is the source of the data. Default to the laptop fake.
    parser.add_argument("--backend", choices=("fake", "hardware"), default="fake")
    args = parser.parse_args(argv)
    # Obtain readings and validate them before printing a successful result.
    try:
        if args.backend == "hardware":
            # Import only when explicitly selected. The current placeholder
            # raises an error; it cannot connect to the robot.
            from .hardware import create_read_only_arm
            arm = create_read_only_arm()
        else:
            arm = FakeArm()
        telemetry = arm.read_status()
        # Recheck here so a future adapter cannot silently return bad data.
        telemetry.validate()
    except (RuntimeError, ValueError) as error:
        # Send failures to stderr (the error stream), with no fake fallback.
        print("Status failed: " + str(error), file=sys.stderr)
        return 1
    # Convert the dataclass to a dictionary, then print readable JSON.
    print(json.dumps(asdict(telemetry), indent=2))
    return 0


# python -m bucke_arm runs this block. Importing this module does not run it.
if __name__ == "__main__":
    sys.exit(main())
