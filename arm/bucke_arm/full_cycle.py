"""One supervised READY-to-pickup/drop-to-READY run of the demonstrated phases.

By default, place the block at (7,2) before starting; there is no placement
pause. --place-at-pickup retains the old manual insertion checkpoint. Motion,
pose checks and the automatic return are unchanged. The pre-positioned-block
approach still needs its first supervised physical run.
"""

import argparse
from .arm import RealArm,FakeArm,READY_ANGLES

# Preserve the actual intermediate pose produced by the demonstrated high-hover
# alignment command. This is a taught checkpoint, not a claim of verticality.
POST_HOVER_OBSERVED=(34.18,-9.66,-104.23,20.47,-0.08,160.66)
POST_HOVER_COMMAND=(33.9587,-8.2415,-103.862,22.1035,0,160.66)
HIGH_HOVER_OBSERVED=(34.27,-13.35,-93.95,13.0,-0.35,160.66)


def confirm_pickup(angles):
    print('\nRaised pickup approach stopped at:',angles,flush=True)
    print('Place the one-inch block at (7,2). Verify open jaws centered over it, nearly vertical, with slight fingertip overlap.',flush=True)
    answer=input('Enter to descend, close, lift, sweep and release; Q to stop: ').strip().upper()
    return answer==''


def preplaced_pickup(angles):
    """Continue after the existing pose checks, assuming the block was preplaced.

    This is not object detection or a new motion/clearance check. The operator
    requested removing the mid-cycle placement step; physical setup is external.
    """
    print('Raised pickup pose verified; continuing with the block pre-positioned at (7,2).',flush=True)
    return True


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--execute',action='store_true',help='Run once from saved READY with the block already at (7,2)')
    mode.add_argument('--fake',action='store_true',help='Replay observed checkpoints offline; no motion physics')
    parser.add_argument('--place-at-pickup',action='store_true',
                        help='Use the old placement pause; keep block out during the initial approach')
    args=parser.parse_args(argv)
    print('Start: saved READY',READY_ANGLES)
    if args.place_at_pickup:
        print('Keep block out during approach; pause at pickup for placement. Jaws empty.')
    else:
        print('Place the one-inch block at (7,2) BEFORE starting. No placement or return prompts; jaws empty.')
    print('READY -> high HOVER -> taught transfer -> raised pickup -> grasp -> lift -> sweep/drop -> READY.')
    print('Arm speed 5; only return J6 uses 10. One command per stage; no trims or automatic repeats.')
    if not args.execute and not args.fake:
        print('Preview only. The combined hardware run still needs supervision.')
        return 0
    arm=None
    try:
        arm=FakeArm(READY_ANGLES) if args.fake else RealArm()
        if args.fake:
            result=arm.run_ready_cycle()
        else:
            result=arm.run_ready_cycle(confirm_pickup if args.place_at_pickup else preplaced_pickup)
        print('Cycle completed. Actual READY angles:',result['ready'])
        print('Confirm the block landed near the demonstrated drop point (2.5,7).')
        return 0
    except (RuntimeError,ValueError,OSError,KeyboardInterrupt) as exc:
        print('Full cycle stopped:',str(exc) or type(exc).__name__,flush=True)
        print('No automatic recovery or repeat was attempted.')
        return 1
    finally:
        if arm is not None:
            arm.close()


if __name__=='__main__':
    raise SystemExit(main())
