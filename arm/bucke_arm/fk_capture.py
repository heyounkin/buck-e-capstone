"""Read-only Pi FK comparison: python3 -m bucke_arm.fk_capture --pose standby."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

from .arm import RealArm, READY_ANGLES, STANDBY_ANGLES, POSE_TOLERANCE
from .kinematics import compare_xyz, MODEL_URL


# Sampling consistency only; this is not a positioning accuracy requirement.
MAX_SAMPLE_SPREAD_DEG = 0.25


def collect(arm, pose_name):
    """Take up to three samples and retain completed samples on a later failure."""
    if pose_name not in ("standby", "ready"):
        raise ValueError("Pose must be standby or ready")
    expected = STANDBY_ANGLES if pose_name == "standby" else READY_ANGLES
    report = {"schema_version": 1, "source": "hardware", "pose": pose_name,
              "created_utc": datetime.now(timezone.utc).isoformat(),
              "model_url": MODEL_URL, "expected_angles_deg": expected,
              "pose_tolerance_deg": POSE_TOLERANCE,
              "max_sample_spread_deg": MAX_SAMPLE_SPREAD_DEG,
              "samples": [], "capture_complete": False,
              "note": "Sequential readings; XYZ agreement does not establish physical accuracy."}
    all_angles = []
    try:
        for index in range(3):
            sample = arm.read_fk_sample()
            sample["captured_utc"] = datetime.now(timezone.utc).isoformat()
            report["samples"].append(sample)
            angles = [sample["angles_before_deg"], sample["angles_after_deg"]]
            if any(abs(value - target) > POSE_TOLERANCE
                   for values in angles for value, target in zip(values, expected)):
                raise RuntimeError("Reported angles do not match the selected saved pose")
            all_angles.extend(angles)
            spread = max(max(values[j] for values in all_angles)
                         - min(values[j] for values in all_angles) for j in range(6))
            sample["max_observed_spread_deg"] = spread
            if spread > MAX_SAMPLE_SPREAD_DEG:
                raise RuntimeError("Joint readings changed by more than 0.25 degrees during capture")
            sample["comparison"] = compare_xyz(angles[0], sample["reported_coords_mm_deg"][:3])
            if index < 2:
                time.sleep(0.5)
        errors = [sample["comparison"]["position_error_mm"] for sample in report["samples"]]
        report["position_error_mm"] = {"min": min(errors), "max": max(errors),
                                       "mean": sum(errors) / len(errors)}
        report["capture_complete"] = True
    except (RuntimeError, ValueError, OSError, AttributeError, KeyboardInterrupt) as error:
        report["failure"] = str(error) or "Capture interrupted"
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="Read-only FK capture at an existing saved pose; never moves the arm")
    parser.add_argument("--pose", choices=("standby", "ready"), required=True)
    parser.add_argument("--output", type=Path, help="New JSON report path; existing files are never overwritten")
    args = parser.parse_args(argv)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = args.output or Path("fk_reports") / ("fk_%s_%s.json" % (args.pose, stamp))
    try:
        output.parent.mkdir(parents=True, exist_ok=True)
        # Reserve the destination before connecting, detecting file errors early.
        with output.open("x", encoding="utf-8") as handle:
            report = {"source": "hardware", "pose": args.pose,
                      "created_utc": datetime.now(timezone.utc).isoformat(),
                      "model_url": MODEL_URL, "capture_complete": False}
            try:
                arm = RealArm()
                try:
                    report = collect(arm, args.pose)
                finally:
                    arm.close()
            except (RuntimeError, ValueError, OSError, AttributeError, KeyboardInterrupt) as error:
                report["capture_complete"] = False
                report["failure"] = str(error) or "Capture interrupted"
            json.dump(report, handle, indent=2, allow_nan=False)
            handle.write("\n")
        print("Saved FK report: " + str(output.resolve()))
        if not report["capture_complete"]:
            print("Capture incomplete: " + report["failure"], file=sys.stderr)
            return 1
        errors = report["position_error_mm"]
        print("%s: 3 stationary samples; XYZ error %.3f to %.3f mm (mean %.3f mm)" %
              (args.pose, errors["min"], errors["max"], errors["mean"]))
        return 0
    except OSError as error:
        print("FK report failed: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
