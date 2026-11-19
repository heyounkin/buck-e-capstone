"""Short relative descent and jaw closure from the physically verified raised pose.

No ready approach, wrist spin, automatic trims, lift or return. The operator
confirmed jaws centered over the one-inch block, almost vertical, with slight
fingertip overlap. This step lowers a further quarter inch before closure.
"""

import argparse
from .arm import RealArm, APPROACH_TOLERANCE
from .kinematics import gripper_midpoint_mat
from .pickup import _solve_local_midpoint
from .hover_alignment import tilt_degrees

VERIFIED_RAISED_ANGLES = (34.27,-14.06,-108.36,32.34,-0.52,159.96)
# Match the preceding raised-approach arrival contract. Position, tilt,
# live stability and faults remain separately checked.
SETTLE_ALLOWANCE = (APPROACH_TOLERANCE,) * 6
LOWER_INCHES = 0.25
# Same bounded directional trial offsets as the successfully observed local
# approach; no global calibration, controller settings or automatic retries.
TRIAL_OFFSETS_DEG = (0,0.59,0.368,1.634,0,0)


def grasp_start_problem(angles):
    """Explain an invalid handoff without mistaking ordinary arrival error for failure."""
    from .local_approach import desired_target
    # J5's signed difference from one snapshot is not a tilt check. Retain
    # geometric near-vertical and local position gates for that joint.
    for joint,(a,b,allow) in enumerate(zip(angles,VERIFIED_RAISED_ANGLES,SETTLE_ALLOWANCE)):
        if joint!=4 and abs(a-b)>allow:
            return 'J%d differs %.3f degrees from the raised reference (limit %.3f)'%(joint+1,abs(a-b),allow)
    xyz=gripper_midpoint_mat(angles)
    # Use the same measured-reference midpoint goal as the preceding approach,
    # not the incidental model bias at one historic reported-angle snapshot.
    reference=gripper_midpoint_mat(desired_target())
    delta=tuple((a-b)/25.4 for a,b in zip(xyz,reference))
    if max(abs(v) for v in delta)>.15:
        return 'modeled midpoint delta %r inches exceeds the 0.15-inch gate'%(tuple(round(v,4) for v in delta),)
    tilt=tilt_degrees(angles)
    if tilt>2:
        return 'modeled flange tilt %.3f degrees exceeds the 2-degree gate'%tilt
    return None


def known_grasp_start(angles):
    return grasp_start_problem(angles) is None


def descent_plan(live_start):
    current=gripper_midpoint_mat(live_start)
    goal=current[:2]+(current[2]-LOWER_INCHES*25.4,)
    # Retain the actual almost-vertical bend and wrist settings; do not
    # attempt another simultaneous orientation change during the short descent.
    desired=_solve_local_midpoint(goal,live_start,sum(live_start[1:4]),live_start[4])
    command=tuple(round(a+b,4) for a,b in zip(desired,TRIAL_OFFSETS_DEG))
    return desired,command


def require_descent_result(start,actual,desired):
    if any(abs(a-b)>1.5 for a,b in zip(actual,desired)):
        raise RuntimeError('Descent joint endpoint changed; jaws left open')
    before=gripper_midpoint_mat(start)
    after=gripper_midpoint_mat(actual)
    dx,dy,dz=tuple((a-b)/25.4 for a,b in zip(after,before))
    if abs(dx)>.125 or abs(dy)>.125 or not -.375<=dz<=-.125 or tilt_degrees(actual)>2:
        raise RuntimeError('Descent relative-position/tilt check failed; jaws left open: '+repr((dx,dy,dz)))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true',help='One quarter-inch descent, verify, then command jaw closure')
    args=parser.parse_args(argv)
    desired,command=descent_plan(VERIFIED_RAISED_ANGLES)
    print('Grasp step: lower 0.25 inch from the verified raised pose, then close jaws.')
    print('Start near:',VERIFIED_RAISED_ANGLES)
    print('Reference desired joints:',desired)
    print('Reference command with trial offsets:',command)
    print('Live plan preserves J5/J6. No lift, sweep, retry or return.')
    if not args.execute:
        print('Preview only. --execute requires the centered open jaws and block already in position.')
        return 0
    arm=None
    try:
        arm=RealArm()
        result=arm.grasp_block()
        print('Captured start:',result['start'])
        print('Reported joints:',result['actual'])
        print('Descent verified; jaw closure commanded. Check physical grip before any lift.')
        return 0
    except (RuntimeError,ValueError,OSError,KeyboardInterrupt) as exc:
        print('Grasp step stopped:',str(exc) or type(exc).__name__,flush=True)
        return 1
    finally:
        if arm is not None:
            arm.close()


if __name__=='__main__':
    raise SystemExit(main())
