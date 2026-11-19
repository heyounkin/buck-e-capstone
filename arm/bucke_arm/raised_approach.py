"""Preview or execute a raised, approach-only test from the recorded J2-test pose.

No grasp, return or ready transition. The user requested one inch above the
former 0.625-inch target; this is 1.625 inches at the modeled pad midpoint.
"""

import argparse

from .arm import RealArm
from .kinematics import gripper_midpoint_mat
from .resume_hover import print_hover_estimate

# Latest three-dimensional configuration supplied during J2 diagnostics.
# Do not substitute READY or join from an arbitrary nearest pose.
START_ANGLES = (36.65, -19.16, -117.59, 44.47, 0.08, 160.75)
START_TOLERANCE = 0.5
HEIGHT_INCHES = 1.625


def raised_target():
    """Raise only J2 to the higher nominal height; preserve every other joint.

    This deliberately does not recenter XY or solve another all-joint pickup.
    Search only in the increasing-J2 direction already observed to raise the
    fingers, bounded by the +0.61-degree position previously reached by the user.
    """
    low, high = START_ANGLES[1], 0.61
    def pose(angle):
        return START_ANGLES[:1] + (angle,) + START_ANGLES[2:]
    if not gripper_midpoint_mat(pose(low))[2] < HEIGHT_INCHES * 25.4 < gripper_midpoint_mat(pose(high))[2]:
        raise ValueError("Raised height is outside the previously traversed J2 range")
    for _ in range(40):
        middle = (low + high) / 2
        if gripper_midpoint_mat(pose(middle))[2] < HEIGHT_INCHES * 25.4:
            low = middle
        else:
            high = middle
    return pose(round((low + high) / 2, 4))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='Perform one raised approach from the recorded stopped pose')
    args = parser.parse_args(argv)
    print('J2-only raised test: modeled pad midpoint height 1.625 inches; XY may shift.')
    print('Required start J1-J6:', START_ANGLES)
    print('Target J1-J6:', raised_target())
    print_hover_estimate(raised_target())
    print('One J2 command, speed 5. Wrist unchanged; no jaw closure or automatic return.')
    if not args.execute:
        print('Preview only. --execute requires the unchanged recorded start pose and a clear local sweep.')
        return 0
    arm = None
    try:
        arm = RealArm()
        actual = arm.move_to_raised_approach()
        print('Raised approach arrived within joint tolerance.')
        print_hover_estimate(actual)
        print('Measure actual fingertip clearance and mat XY before planning pickup.')
        return 0
    except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as exc:
        print('Raised approach stopped:', str(exc) or type(exc).__name__, flush=True)
        return 1
    finally:
        if arm is not None:
            arm.close()


if __name__ == '__main__':
    raise SystemExit(main())
