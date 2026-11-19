"""Repeat the demonstrated grasp/lift/sweep/drop from the verified raised pose.

This does not approach from ready. Optional checked return proceeds without confirmation.
It starts centered over
the block at (7,2); the observed J1=91 command dropped near (2.5,7) inches.
The consolidated routine itself has not yet been run on hardware.
"""

import argparse
from .arm import RealArm,FakeArm,APPROACH_TOLERANCE
from .grasp import VERIFIED_RAISED_ANGLES

LIFT_J2_COMMAND = -10.0
LIFT_J2_OBSERVED = -11.6
SWEEP_J1_COMMAND = 91.0
SWEEP_J1_OBSERVED = 90.08
OBSERVED_DROP_XY_INCHES = (2.5,7.0)


OBSERVED_GRASP_ANGLES=(34.27,-15.9,-110.39,36.73,-.35,160.22)


def known_held_start(angles):
    from .kinematics import gripper_midpoint_mat
    from .hover_alignment import tilt_degrees
    if any(abs(a-b)>APPROACH_TOLERANCE for joint,(a,b) in
           enumerate(zip(angles,OBSERVED_GRASP_ANGLES)) if joint!=4):
        return False
    reference=gripper_midpoint_mat(OBSERVED_GRASP_ANGLES)
    xyz=gripper_midpoint_mat(angles)
    return max(abs(a-b) for a,b in zip(xyz,reference))<=.2*25.4 and tilt_degrees(angles)<=2


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--execute',action='store_true',help='Run once on hardware from the verified raised pickup pose')
    mode.add_argument('--fake',action='store_true',help='Run simulated sequence without hardware or motion physics')
    parser.add_argument('--return-ready',action='store_true',help='After release, return to READY without confirmation')
    parser.add_argument('--resume-grasp',action='store_true',help='Resume an already held block: no approach, descent, or initial jaw opening')
    args=parser.parse_args(argv)
    if args.resume_grasp:
        print('Resume an already held block from known post-grasp pose; first motion is J2 lift.')
    else:
        print('Start centered over block (7,2), near raised joints:',VERIFIED_RAISED_ANGLES)
    print('J2 to -10; J1 to 91; release.' if args.resume_grasp else
          'Open; lower 0.25 inch; close and wait; J2 to -10; J1 to 91; open and wait.')
    print('Speed 5; observed drop about (2.5,7). No trims or automatic repeat.')
    if args.return_ready:
        print('After release: checked return to READY without prompts; final J5/J1 commanded together.')
    else:
        print('No return selected.')
    if not args.execute and not args.fake:
        print('Preview only. Current drop pose is not a valid start.')
        return 0
    arm=None
    try:
        arm=FakeArm(OBSERVED_GRASP_ANGLES if args.resume_grasp else VERIFIED_RAISED_ANGLES) if args.fake else RealArm()
        if args.resume_grasp:
            if args.execute and input('Confirm block is held between the jaws; Enter to resume lift, Q to stop: ').strip():
                print('Cancelled without opening or moving.')
                return 0
            result=arm.finish_held_pickup()
        else:
            result=arm.pick_and_place()
        print('Final reported joints:',result['drop'])
        print('Release sequence completed. Verify the block landed near (2.5,7).')
        if args.return_ready:
            actual=arm.return_to_ready_after_drop()
            print('Returned to saved READY:',actual)
        return 0
    except (RuntimeError,ValueError,OSError,KeyboardInterrupt) as exc:
        print('Pick-and-place stopped:',str(exc) or type(exc).__name__,flush=True)
        return 1
    finally:
        if arm is not None:
            arm.close()


if __name__=='__main__':
    raise SystemExit(main())
