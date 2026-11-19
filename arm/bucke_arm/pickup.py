"""Continuous approach and jaw closure for the block at mat (7, 2).

Preview is offline. Hardware execution is disabled after the operator
reported an excessively low approach and late gripper rotation.
"""

import argparse
import math

from .arm import SPEC_LIMITS, RealArm
from .kinematics import gripper_midpoint_mat
from .resume_hover import HOVER, print_hover_estimate


# Operator measurements: block height 1 inch, width slightly over 1 inch,
# and lowest finger point approximately 1/2 inch below the pad midpoint
# with the gripper pointing down. This scalar check is nominal; it does not
# describe the open-finger swept volume or account for calibration error.
BLOCK_HEIGHT_INCHES = 1.0
FINGER_BELOW_PAD_INCHES = 0.5
DEFAULT_GRASP_HEIGHT_INCHES = 0.625


def require_pickup_execution():
    """Fail closed after the reported low approach; no runtime bypass is offered."""
    raise RuntimeError("Pickup execution disabled after reported low approach and gripper rotation; "
                       "height calibration and coordinated orientation require review")


def pickup_target(grasp_height_inches=DEFAULT_GRASP_HEIGHT_INCHES):
    """Solve XYZ near HOVER, retaining wrist settings and arm bend orientation.

    A side grasp defaults to 5/8 inch, giving nominal 1/8-inch tip clearance.
    The former 1/2-inch midpoint would place the measured tips on the table.
    J2+J3+J4 stays fixed, preserving the near-downward flange tilt; J5 and J6
    stay fixed. J1 can change the jaw heading slightly as it changes mat XY.
    This local inverse calculation uses the updated FK, not SDK coordinates.
    """
    if (type(grasp_height_inches) not in (int, float)
            or not math.isfinite(grasp_height_inches)
            or not FINGER_BELOW_PAD_INCHES < grasp_height_inches < BLOCK_HEIGHT_INCHES):
        raise ValueError("Grasp midpoint must be above 1/2 inch for nominal finger clearance and below the 1-inch block top")
    return _solve_local_midpoint((7 * 25.4, 2 * 25.4, grasp_height_inches * 25.4),
                                 HOVER, sum(HOVER[1:4]), HOVER[4])


def _solve_local_midpoint(target, seed, bend, j5):
    """Local nominal IK for a midpoint and bend; hold the seed's signed J6.

    Used by offline pickup and a separately gated high-hover alignment test.
    It supplies a model target, not physical calibration or collision checking.
    """
    def angles(values):
        j1, j2, j3 = values
        return (j1, j2, j3, bend - j2 - j3, j5, seed[5])

    values = list(seed[:3])
    for _ in range(40):
        current = gripper_midpoint_mat(angles(values))
        error = [goal - actual for goal, actual in zip(target, current)]
        if max(abs(value) for value in error) < 0.01:
            result = tuple(round(value, 4) for value in angles(values))
            if any(not low <= value <= high for value, (low, high) in zip(result, SPEC_LIMITS)):
                raise ValueError("Pickup candidate exceeds Buck-E joint limits")
            return result
        # Central differences in mm/degree; solve the three-variable Newton
        # step with pivoting. Cap each update to stay near the historical pose.
        columns = []
        for joint in range(3):
            plus, minus = values.copy(), values.copy()
            plus[joint] += 0.01
            minus[joint] -= 0.01
            columns.append([(a - b) / 0.02 for a, b in zip(
                gripper_midpoint_mat(angles(plus)), gripper_midpoint_mat(angles(minus)))])
        rows = [[columns[col][row] for col in range(3)] + [error[row]] for row in range(3)]
        for col in range(3):
            pivot = max(range(col, 3), key=lambda row: abs(rows[row][col]))
            rows[col], rows[pivot] = rows[pivot], rows[col]
            scale = rows[col][col]
            if abs(scale) < 1e-8:
                raise ValueError("Pickup candidate is locally singular")
            rows[col] = [value / scale for value in rows[col]]
            for row in range(3):
                if row != col:
                    factor = rows[row][col]
                    rows[row] = [a - factor * b for a, b in zip(rows[row], rows[col])]
        delta = [row[3] for row in rows]
        scale = max(1.0, max(abs(value) for value in delta) / 5)
        values = [value + change / scale for value, change in zip(values, delta)]
    raise ValueError("Pickup candidate did not converge")


def require_grasp_position(angles):
    """Refuse jaw closure if actual-angle FK misses the block or table margin.

    This is an additional endpoint check, not physical grasp sensing or a
    collision model. Keep the pad reference above the measured finger length.
    """
    x, y, z = (value / 25.4 for value in gripper_midpoint_mat(angles))
    if (abs(x - 7) > 0.125 or abs(y - 2) > 0.125
            or not FINGER_BELOW_PAD_INCHES < z < BLOCK_HEIGHT_INCHES):
        raise RuntimeError("Pickup FK endpoint outside grasp bounds; jaws left open: " + repr((x, y, z)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--grasp-height", type=float, default=DEFAULT_GRASP_HEIGHT_INCHES,
                        help="Pad midpoint height in inches; default 5/8 gives nominal 1/8-inch tip clearance")
    parser.add_argument("--execute", action="store_true",
                        help="Connect on the Pi and perform the approach and closure")
    args = parser.parse_args(argv)
    if args.execute:
        try:
            require_pickup_execution()
        except RuntimeError as exc:
            print(exc)
            return 1
    try:
        candidate = pickup_target(args.grasp_height)
    except ValueError as exc:
        print("Pickup planning refused:", exc)
        return 1
    print("One-inch block at mat (7, 2); candidate side-grasp midpoint height:", args.grasp_height)
    print("Measured finger extension below midpoint: 0.5 inch; block width slightly over 1 inch.")
    print("Nominal fingertip/table clearance (inches):", args.grasp_height - FINGER_BELOW_PAD_INCHES)
    print("This nominal clearance excludes tilt, open-finger geometry and FK/tracking error.")
    print("Candidate pickup joints (degrees):", candidate)
    print_hover_estimate(candidate)
    print("Sequence: start at saved ready with jaws open; one continuous approach; verify; close jaws.")
    print("No correction steps, lift, return, sweep or drop.")
    if not args.execute:
        print("Preview only. Hardware pickup is disabled pending motion review.")
        return 0
    arm = None
    try:
        arm = RealArm()
        arm.check_hover_start()
        print("Place the block at (7, 2). Open the jaws wider than the block before starting.")
        print("Use the confirmed base/mat/gripper setup and clear approach; keep an independent stop reachable.")
        if input("Type PICKUP to approach and close the jaws: ").strip() != "PICKUP":
            print("Cancelled.")
            return 0
        actual = arm.pick_up_block(args.grasp_height)
        print_hover_estimate(actual)
        print("Approach verified; jaw closure commanded. Check the physical grasp. Arm remains at grasp height.")
        return 0
    except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as exc:
        print("Pickup stopped:", str(exc) or type(exc).__name__, flush=True)
        return 1
    finally:
        if arm is not None:
            arm.close()


if __name__ == "__main__":
    raise SystemExit(main())
