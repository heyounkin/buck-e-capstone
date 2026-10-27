"""Read-only status and connection-free ready-pose preview CLI."""

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
    # The function returns 0 on success and 1 for a status or preview failure.

    parser = argparse.ArgumentParser(description="Buck-E arm status and offline preview")
    commands = parser.add_subparsers(dest="command", required=True)
    status = commands.add_parser("status", help="Read simulated or physical telemetry")
    # A backend is the source of the data. Default to the laptop fake.

    status.add_argument("--backend", choices=("fake", "hardware"), default="fake")
    ready = commands.add_parser(
        "move-to-ready", help="Preview joint differences; never connect or move"
    )
    ready.add_argument("--angles", nargs=6, type=float, required=True,
                       metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    ready.add_argument("--moving", type=int, choices=(0, 1), required=True,
                       help="Supplied movement status, not a live query")
    ready.add_argument("--error", type=int, required=True,
                       help="Supplied controller error code; nonzero refuses preview")
    ready.add_argument("--limits", metavar="JSON",
                       help="Joint-limit configuration; default is provisional offline bounds")
    ready.add_argument("--route", metavar="JSON",
                       help="Explicit candidate route with starting ranges and waypoints")
    args = parser.parse_args(argv)

    if args.command == "move-to-ready":
        from .ready import move_to_ready
        from .ready_config import load_limits, load_route

        try:
            preview = move_to_ready(
                args.angles, moving=args.moving, error_information=args.error,
                limits=load_limits(args.limits) if args.limits else None,
                route=load_route(args.route) if args.route else None,
            )
        except (ValueError, OSError) as error:
            print("Ready preview failed: " + str(error), file=sys.stderr)
            return 1
        print(json.dumps(asdict(preview), indent=2, allow_nan=False))
        return 0
    # Obtain readings and validate them before printing a successful result.

    try:
        if args.backend == "hardware":
            # Import only when explicitly selected, then open the verified
            # Pi connection. The adapter has no power or motion commands.

            from .hardware import create_read_only_arm
            arm = create_read_only_arm()
        else:
            arm = FakeArm()
        try:
            telemetry = arm.read_status()
            # Validate before reporting success, even with a future adapter.
            telemetry.validate()
        finally:
            # Close on successful reads and on failures. Fake cleanup is a no-op.
            arm.close()
    except (RuntimeError, ValueError, OSError) as error:
        # Send failures to stderr (the error stream), with no fake fallback.

        print("Status failed: " + str(error), file=sys.stderr)
        return 1

    # Convert the dataclass to a dictionary, then print readable JSON.
    print(json.dumps(asdict(telemetry), indent=2))
    return 0


# python -m bucke_arm runs this block. Importing this module does not run it.
if __name__ == "__main__":
    sys.exit(main())
