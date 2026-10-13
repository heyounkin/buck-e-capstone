"""Simulated status CLI. Hardware execution is unavailable."""

import argparse
from dataclasses import asdict
import json
import sys
from typing import Optional, Sequence

from .arm import FakeArm


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Buck-E read-only arm status")
    parser.add_argument("command", choices=("status",))
    parser.add_argument("--backend", choices=("fake", "hardware"), default="fake")
    args = parser.parse_args(argv)
    try:
        if args.backend == "hardware":
            from .hardware import create_read_only_arm
            arm = create_read_only_arm()
        else:
            arm = FakeArm()
        telemetry = arm.read_status()
        telemetry.validate()
    except (RuntimeError, ValueError) as error:
        print("Status failed: " + str(error), file=sys.stderr)
        return 1
    print(json.dumps(asdict(telemetry), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
