"""Restore the November 3 hover; no connection unless --execute is supplied."""

import argparse

from .arm import READY_ANGLES, RealArm, _pose

INITIAL_HOVER = (34.68, -4.11, -103.13, 17.24, 0.0, 162.84)
LIFT = (33.66, -2.13, -90.41, 0.35, 0.08, 161.63)
SHIFT = (34.75, -10.73, -80.70, -0.75, 0.08, 161.63)
HOVER = (34.75, -12.77, -93.56, 14.15, 0.08, 161.63)
LAST_READBACK = (33.66, -13.35, -94.13, 13.0, 0.0, 161.63)
# November 5: three identical stationary, error-free samples after Lift 3 dropout.
LIFT3_READBACK = (33.75, -4.48, -98.78, 9.58, 0.43, 161.71)


def route(resume_lift3=False):
    """Reuse the approach and lift/shift/lower stages from the previous run.

    The first stage now begins at saved ready, rather than the previous
    session's measured ready. Final accuracy remains a telemetry tolerance.
    """
    result = []
    start = READY_ANGLES
    for name, end, steps, tolerance in (
        ("Approach", INITIAL_HOVER, 30, 2.5),
        ("Lift", LIFT, 6, 2.0),
        ("Shift", SHIFT, 6, 2.0),
        ("Lower to corrected hover", HOVER, 6, 2.0),
    ):
        for step in range(1, steps + 1):
            target = tuple(round(a + (b - a) * step / steps, 2)
                           for a, b in zip(start, end))
            result.append(("%s %d/%d" % (name, step, steps), target, tolerance))
        start = end
    if resume_lift3:
        # The dropout was during Lift 3, so retry that waypoint first.
        lift3 = next(index for index, (label, _, _) in enumerate(result) if label == "Lift 3/6")
        return result[lift3:]
    return result


def return_route(current=None):
    """Analyze the retired waypoint return without issuing commands.

    Each joint may have progressed independently inside the segment's padded
    angle box. Joining its ready-side endpoint stays in that same box. Choose
    the closest segment, preferring the earlier segment for an exact tie.
    Signed J6 angles are never wrapped. No arbitrary nearest-route join is used.
    """
    forward = route()
    points = [READY_ANGLES] + [target for _, target, _ in forward]
    last = len(forward) - 1
    if current is not None:
        current = _pose(current)
        if max(abs(a - b) for a, b in zip(current, READY_ANGLES)) <= 3.0:
            return []
        matches = []
        for index, (_, end, tolerance) in enumerate(forward):
            start = points[index]
            padding = max(tolerance, forward[index - 1][2] if index else 3.0)
            if not all(min(a, b) - padding <= v <= max(a, b) + padding
                       for a, b, v in zip(start, end, current)):
                continue
            delta = [b - a for a, b in zip(start, end)]
            progress = max(0.0, min(1.0, sum((v - a) * d for v, a, d
                                            in zip(current, start, delta))
                                    / sum(d * d for d in delta)))
            distance = max(abs(v - (a + progress * d))
                           for v, a, d in zip(current, start, delta))
            matches.append((distance, index))
        if not matches:
            raise RuntimeError("Current pose is outside the hover path: " + repr(current))
        _, last = min(matches)
    count = last + 1
    return [("Return to ready %d/%d" % (step, count), points[index], forward[index][2])
            for step, index in enumerate(range(last, -1, -1), 1)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true",
                        help="Connect on the Pi and offer one local HOVER confirmation")
    parser.add_argument("--resume-lift3", action="store_true",
                        help="Resume Lift 3 from the November 5 measured stopped pose")
    parser.add_argument("--to-ready", action="store_true",
                        help="Retired hover return option (disabled after reported contact)")
    parser.add_argument("--synchronous", action="store_true",
                        help="Use sync_send_angles for each all-joint waypoint")
    parser.add_argument("--continuous", action="store_true",
                        help="Send one forward command and record the measured joint trajectory")
    parser.add_argument("--preview-reverse", metavar="RECORDING.json",
                        help="Validate a recorded forward trace and preview its reversed samples offline")
    args = parser.parse_args(argv)
    if args.preview_reverse:
        if args.execute:
            parser.error("--preview-reverse is offline and cannot use --execute")
        from .hover_trajectory import reverse_trace
        try:
            reverse = reverse_trace(args.preview_reverse)
        except (ValueError, OSError) as exc:
            print("Reverse preview refused:", exc)
            return 1
        print("Recorded samples in reverse order:", len(reverse))
        print("Start:", reverse[0]["angles_deg"])
        print("End:", reverse[-1]["angles_deg"])
        print("Duration:", reverse[-1]["elapsed_s"], "seconds")
        print("Preview only. Hardware return remains disabled pending trajectory review.")
        return 0
    if args.to_ready:
        print("Return disabled after reported gripper-to-arm contact. "
              "The previous forward path was not recorded; a measured trace is required.")
        return 1
    print("Saved ready:", READY_ANGLES)
    print("Corrected hover target:", HOVER)
    print("Previous observed endpoint:", LAST_READBACK)
    if not args.continuous:
        print("%d waypoints, speed 5; final tolerance 1.5 degrees."
              % len(route(resume_lift3=args.resume_lift3)))
    if args.resume_lift3:
        print("Required stopped pose:", LIFT3_READBACK)
    if args.continuous:
        print("One forward endpoint command, speed 5; measured joint-path recording enabled.")
    elif args.synchronous:
        print("Synchronous SDK waypoint calls; application fault polling pauses during each call.")
    if not args.execute:
        print("Preview only. Use --execute on the Pi to connect.")
        return 0
    arm = RealArm()
    try:
        arm.check_hover_start(resume_lift3=args.resume_lift3)
        print("Current angles:", arm.get_angles(), flush=True)
        print("Keep the original base/mat/gripper setup. Remove the cube,")
        print("open the jaws, clear the entire sweep, and keep an independent stop reachable.")
        if input("Type HOVER to move to hover: ").strip() != "HOVER":
            print("Cancelled.")
            return 0
        arm.move_to_hover(resume_lift3=args.resume_lift3, synchronous=args.synchronous,
                          continuous=args.continuous)
        print("Hover reached within tolerance. Check jaw midpoint over (7, 2) inches.")
        print("Leave the cube out. Next: side view along mat Y to check flange tilt.")
        return 0
    except (RuntimeError, ValueError, OSError) as exc:
        print("Stopped:", exc, flush=True)
        return 1
    finally:
        arm.close()


if __name__ == "__main__":
    raise SystemExit(main())
