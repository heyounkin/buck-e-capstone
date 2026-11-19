"""One local raised approach referenced to the measured pad position.

The operator measured (6.625,1.75,2) inches at START_ANGLES. The goal is
(7,2,1.625), not a low grasp. Original pickup and return remain disabled.
"""

import argparse
from .arm import RealArm
from .kinematics import gripper_midpoint_mat
from .pickup import _solve_local_midpoint
from .hover_alignment import tilt_degrees

START_ANGLES = (34.18, -9.66, -104.23, 20.47, -0.08, 160.66)
# Two operator-supplied post-alignment observations define this local region.
# This permits observed settling differences, not arbitrary route joins.
SETTLED_OBSERVATION = (34.27, -9.66, -104.23, 21.0, -0.52, 159.96)
START_REGION = tuple((min(a,b)-0.25,max(a,b)+0.25)
                     for a,b in zip(START_ANGLES,SETTLED_OBSERVATION))
STABILITY_TOLERANCE = 0.25
MEASURED_XYZ_INCHES = (6.625, 1.75, 2.0)
GOAL_XYZ_INCHES = (7.0, 2.0, 1.625)
# A single bounded feedforward TRIAL, not a general J2 calibration. These
# offsets are opposite the observed reported-minus-commanded errors for
# comparable directions: J2 negative (-0.59), J3 negative (-0.368), and
# J4 positive (-1.634). There are no feedback trims or hardware parameter writes.
TRIAL_OFFSETS_DEG = (0, 0.59, 0.368, 1.634, 0, 0)


def known_start(angles):
    return all(low <= value <= high for value,(low,high) in zip(angles,START_REGION))


def desired_target(live_start=None):
    reference = gripper_midpoint_mat(START_ANGLES)
    delta = tuple((b-a)*25.4 for a,b in zip(MEASURED_XYZ_INCHES, GOAL_XYZ_INCHES))
    goal = tuple(a+b for a,b in zip(reference,delta))
    return _solve_local_midpoint(goal, START_ANGLES if live_start is None else live_start, -90, 0)


def command_target(live_start=None):
    return tuple(round(a+b,4) for a,b in zip(desired_target(live_start),TRIAL_OFFSETS_DEG))


def print_actual(angles, desired=None):
    reference = gripper_midpoint_mat(START_ANGLES)
    xyz = gripper_midpoint_mat(angles)
    estimate = tuple(anchor+(v-r)/25.4 for anchor,v,r in zip(MEASURED_XYZ_INCHES,xyz,reference))
    print('Reported joints:',tuple(angles))
    print('Local estimate from measured anchor (mat inches):',tuple(round(v,3) for v in estimate))
    print('Modeled flange tilt from downward: %.3f degrees' % tilt_degrees(angles))
    print('Joint errors relative to desired angles:',tuple(round(a-b,3) for a,b in zip(angles,desired_target() if desired is None else desired)))
    print('Local XYZ is still an estimate; verify the physical pad center and tilt.')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true',help='Send one local approach command from the recorded stopped pose')
    args=parser.parse_args(argv)
    print('Measured anchor (mat inches):', MEASURED_XYZ_INCHES)
    print('Raised approach goal (mat inches):',GOAL_XYZ_INCHES)
    print('Measured reference joints:',START_ANGLES)
    print('Allowed local start ranges:',START_REGION)
    print('Reference-plan desired joints (live plan recalculated before movement):',desired_target())
    print('One command with bounded trial offsets:',command_target())
    print('J6 unchanged. Remove block: the path can dip. No jaws, retries, return or parameter changes.')
    if not args.execute:
        print('Preview only. --execute performs the single local positioning test.')
        return 0
    arm=None
    try:
        arm=RealArm()
        result=arm.move_to_local_approach()
        print('Raised approach reached within joint tolerance.')
        print('Captured live start:',result['start'])
        print('Executed command:',result['command'])
        print('Desired reported joints:',result['desired'])
        print_actual(result['actual'],result['desired'])
        return 0
    except (RuntimeError,ValueError,OSError,KeyboardInterrupt) as exc:
        print('Local approach stopped:',str(exc) or type(exc).__name__,flush=True)
        return 1
    finally:
        if arm is not None:
            arm.close()


if __name__=='__main__':
    raise SystemExit(main())
