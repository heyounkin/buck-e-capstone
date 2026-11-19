"""Square the current high hover without intentionally changing its pad midpoint.

This local test is only for the measured high hover, physically centered over
(7, 2). It is not a ready approach, descent, grasp or full workspace calibration.
"""

import argparse
import math

from .arm import RealArm
from .kinematics import gripper_midpoint_mat, forward_kinematics
from .pickup import _solve_local_midpoint

# Three identical stationary, error-free readings supplied after the initial
# guard refusal. Use this refreshed checkpoint; do not widen the pose gate.
START_ANGLES = (34.27, -13.35, -93.95, 13.0, -0.35, 160.66)
START_TOLERANCE = 0.5


def tilt_degrees(angles):
    """Angle between nominal flange +Z and downward; not a physical measurement."""
    cosine = -forward_kinematics(angles).rotation_matrix[2][2]
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def alignment_target():
    # Preserve the MODEL point at the observed physically aligned pose, rather
    # than aiming at model XY (7,2), which would introduce an unwanted shift.
    # With the pinned axes, J5=0 and J2+J3+J4=-90 put flange +Z downward.
    return _solve_local_midpoint(gripper_midpoint_mat(START_ANGLES),
                                 START_ANGLES, -90, 0)


def print_alignment(angles):
    reference = gripper_midpoint_mat(START_ANGLES)
    xyz = gripper_midpoint_mat(angles)
    delta = tuple((v-a)/25.4 for v,a in zip(xyz,reference))
    print('Joint snapshot:', tuple(angles))
    print('Modeled flange tilt from downward: %.3f degrees' % tilt_degrees(angles))
    print('Modeled midpoint change from observed hover (inches):', tuple(round(v,4) for v in delta))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='One local alignment command from the recorded high hover')
    args = parser.parse_args(argv)
    print('High-hover alignment only; keep the observed pad center over (7,2).')
    print('Required start:', START_ANGLES)
    print('Candidate:')
    print_alignment(alignment_target())
    print('J6 unchanged. Same nominal endpoint height; intermediate height may change.')
    print('Remove the block; no jaw commands, retries or return.')
    if not args.execute:
        print('Preview only. The physical midpoint and vertical direction need checking after the test.')
        return 0
    arm = None
    try:
        arm = RealArm()
        actual = arm.square_hover()
        print('Joint endpoint reached. Actual-angle model comparison:')
        print_alignment(actual)
        print('Check physical tilt and pad center; do not descend yet.')
        return 0
    except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as exc:
        print('Hover alignment stopped:', str(exc) or type(exc).__name__, flush=True)
        return 1
    finally:
        if arm is not None:
            arm.close()


if __name__ == '__main__':
    raise SystemExit(main())
