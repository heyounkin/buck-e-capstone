"""Read-only status, offline forward kinematics and ready-pose preview CLI."""

# argparse reads terminal arguments; asdict and json format the output.
# sys supplies the error stream and lets us return an exit code to the shell.
import argparse
from dataclasses import asdict
import json
import sys
from typing import Optional, Sequence

# The leading dot imports from this same package. No robot SDK is loaded.
from .arm import FakeArm, RealArm


def main(argv: Optional[Sequence[str]] = None) -> int:
    # argv=None uses the terminal arguments. Tests can pass a list instead.
    # The function returns 0 on success and 1 for a status or preview failure.

    parser = argparse.ArgumentParser(description="Buck-E arm status, offline FK and preview")

    # Subcommands keep hardware-only flags on status and offline inputs on ready.
    commands = parser.add_subparsers(dest="command", required=True)
    status = commands.add_parser("status", help="Read simulated or physical telemetry")

    # A backend is the source of the data. Default to the laptop fake.

    status.add_argument("--backend", choices=("fake", "real", "hardware"), default="fake")
    fk = commands.add_parser("fk", help="Offline nominal flange FK and optional XYZ comparison")
    fk.add_argument("--angles", nargs=6, type=float, required=True,
                    metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    fk.add_argument("--reported-xyz", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="Supplied base-frame flange coordinates in mm; no live query")
    fk.add_argument("--tool-offset-mm", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="Optional flange-local pickup-point offset; no hardware setting is changed")
    ready = commands.add_parser(
        "move-to-ready", help="Preview joint differences; never connect or move"
    )

    # nargs=6 requires one value for each joint. Explicit status inputs avoid
    # silently assuming that a supplied pose was stationary and error-free.
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

    if args.command == "fk":
        from .kinematics import forward_kinematics, compare_xyz, MODEL_URL
        try:
            pose = forward_kinematics(args.angles)
            result = {"source": "offline_model", "frame": "base_to_flange",
                      "model_url": MODEL_URL, "angles_deg": args.angles,
                      "xyz_mm": pose.xyz_mm, "rotation_matrix": pose.rotation_matrix,
                      "rpy_deg": pose.rpy_deg,
                      "orientation_convention": "Rz(yaw) Ry(pitch) Rx(roll); SDK convention unverified",
                      "transform_mm": pose.transform_mm}
            if args.reported_xyz is not None:
                result["comparison"] = compare_xyz(args.angles, args.reported_xyz)
            if args.tool_offset_mm is not None:
                result["tool_offset_flange_mm"] = args.tool_offset_mm
                result["pickup_point_base_mm"] = pose.point_in_base(args.tool_offset_mm)
        except ValueError as error:
            print("FK failed: " + str(error), file=sys.stderr)
            return 1
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0

    if args.command == "move-to-ready":
        # Complete the offline branch before reaching any RealArm construction.
        from .ready import move_to_ready
        from .ready_config import load_limits, load_route

        try:
            preview = move_to_ready(
                args.angles, moving=args.moving, error_information=args.error,
                limits=load_limits(args.limits) if args.limits else None,
                route=load_route(args.route) if args.route else None,
            )
        except (ValueError, OSError) as error:
            # Invalid poses/configurations produce an error and no partial JSON.
            print("Ready preview failed: " + str(error), file=sys.stderr)
            return 1

        # Nested dataclasses become ordinary dictionaries; reject nonfinite JSON.
        print(json.dumps(asdict(preview), indent=2, allow_nan=False))
        return 0

    # Obtain readings and validate them before printing a successful result.

    try:
        arm = FakeArm() if args.backend == "fake" else RealArm()

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
