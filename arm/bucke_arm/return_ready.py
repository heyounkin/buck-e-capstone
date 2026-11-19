"""Checked empty-gripper return from the observed drop pose to saved READY.

The operator confirmed the staged return worked at speed 5. J6 now uses the
requested speed 10; other joints retain speed 5. The old direct hover return
caused self-contact. Return now proceeds without confirmation by explicit
operator request, combining J5 and J1 in its final all-joint command.
Nominal table screening is not a self-collision model. No automatic retry.
"""

import argparse
import math
from .arm import RealArm,FakeArm,READY_ANGLES,APPROACH_TOLERANCE
from .kinematics import forward_kinematics,MAT_TRANSLATION_MM

DROP_ANGLES=(90.08,-11.6,-110.39,37.17,-1.14,160.04)
# The preceding sweep accepts this same joint endpoint tolerance. Retain
# near-zero J5 separately so a valid arrival is not rejected at stage handoff.
DROP_ALLOWANCE=(APPROACH_TOLERANCE,)*6
# The first four single-joint stages prevent the J4/table dip found in
# a direct all-joint reset. J6 is changed before folding J4/J5, but only after
# lifting the empty tool away from the dropped block.
STAGES=(
    ('Raise empty tool',2,0.0),
    ('Rotate wrist through its signed arc',6,-135.0),
    ('Raise elbow',3,0.0),
    ('Reset J4 after elbow is raised',4,0.0),
    ('Finish ready wrist and base together',(5,1),(90.0,-90.0)),
)


def assignments(joint,value):
    """Normalize a single-joint stage or the final paired J5/J1 stage."""
    return ((joint,value),) if isinstance(joint,int) else tuple(zip(joint,value))


def describe_motion(joint,current,target):
    joints=(joint,) if isinstance(joint,int) else joint
    return ', '.join('J%d %.2f -> %.2f'%(j,current[j-1],target[j-1]) for j in joints)


def stage_speed(joint):
    """Only the long return wrist rotation uses the operator-requested speed 10."""
    return 10 if joint==6 else 5


def known_drop(angles):
    # The down-facing wrist may settle just above or below zero. Use a
    # bounded near-zero J5 angle instead of a difference from one signed
    # historical reading. All other observed drop bounds remain unchanged.
    return (abs(angles[4])<=2.0 and
            all(abs(a-b)<=allow for joint,(a,b,allow) in
                enumerate(zip(angles,DROP_ANGLES,DROP_ALLOWANCE)) if joint!=4))


def targets(start=DROP_ANGLES):
    current=tuple(start)
    result=[]
    for label,joint,value in STAGES:
        target=list(current)
        for j,v in assignments(joint,value):
            target[j-1]=v
        target=tuple(target)
        result.append((label,joint,target))
        current=target
    return result


def provisional_tool_height(angles):
    """Lower edge of historical axial/radial tool cylinder, in mat mm.

    Axial [-10,137] mm, radius 75 mm; a provisional envelope, not actual open
    finger meshes, robot-link collision geometry, overshoot or a safety bound.
    """
    flange=forward_kinematics(angles)
    az=flange.rotation_matrix[2][2]
    return (MAT_TRANSLATION_MM[2]+flange.xyz_mm[2]+min(-10*az,137*az)
            -75*math.sqrt(max(0,1-az*az)))


def preview():
    current=DROP_ANGLES
    print('Required start near observed drop:',DROP_ANGLES)
    print('Destination saved READY:',READY_ANGLES)
    print('Empty, open jaws only. Clear the full tool, camera and cable sweep.')
    for index,(label,joint,target) in enumerate(targets(),1):
        minimum=min(provisional_tool_height(tuple(a+(b-a)*i/300 for a,b in zip(current,target)))
                    for i in range(301))
        print('%d. %s: %s; speed %d; sampled tool/table height %.1f mm'
              %(index,label,describe_motion(joint,current,target),stage_speed(joint),minimum))
        current=target
    print('No return confirmations. J5/J1 are issued together; exact simultaneous arrival is not guaranteed by the SDK.')
    print('Provisional table screen only; it does not prove robot self-clearance.')


def confirm_stage(index,label,joint,current,target):
    print('\nStage %d: %s. %s at speed %d.'
          %(index,label,describe_motion(joint,current,target),stage_speed(joint)),flush=True)
    print('Actual start:',current,flush=True)
    if index==1:
        print('Confirm the block has been released and jaws are empty and open.',flush=True)
    if joint==6:
        print('This is about a 295-degree signed wrist turn, not a wrapped 65-degree turn.',flush=True)
    answer=input('Inspect full sweep clearance; Enter to run this stage, Q to stop: ').strip().upper()
    return answer==''


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--execute',action='store_true',help='Return without confirmations; empty gripper required')
    mode.add_argument('--fake',action='store_true',help='Offline state simulation of the staged return')
    parser.add_argument('--step',action='store_true',help='Optional diagnostic mode: pause before each return stage')
    args=parser.parse_args(argv)
    preview()
    if not args.execute and not args.fake:
        print('Preview only. No connection or movement.')
        return 0
    arm=None
    try:
        arm=FakeArm(DROP_ANGLES) if args.fake else RealArm()
        actual=arm.return_to_ready_after_drop(confirm_stage if args.step and not args.fake else None)
        print('Saved READY reached within tolerance. Actual angles:',actual)
        return 0
    except (RuntimeError,ValueError,OSError,KeyboardInterrupt) as exc:
        print('Return stopped:',str(exc) or type(exc).__name__,flush=True)
        return 1
    finally:
        if arm is not None:
            arm.close()


if __name__=='__main__':
    raise SystemExit(main())
