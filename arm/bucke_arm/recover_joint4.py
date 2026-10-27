"""Supervised manual J4 recovery; no target-position or calibration commands.

Default invocation only displays instructions. Hardware use is deliberately
separate from the read-only status adapter and requires interactive consent.
"""

import argparse
from importlib import metadata
import math
import sys
import time


LIMITS = ((-168, 168), (-140, 140), (-150, 150),
          (-150, 150), (-155, 160), (-180, 180))
PLAN = """Manual joint-4 recovery for the observed 153.8-degree pose.
Close all other robot-control programs. Have a second person operate the
terminal while you support the links beyond J4, away from pinch points.
Release removes holding torque; re-engaging torque may also cause movement.
Only proceed if you can positively identify J4 and support its attached links.
The script releases only J4 after RELEASE J4 is typed. Gently reduce its angle
toward 145 degrees, stopping for resistance, cable tension, or interference.
Press Enter to refresh angles. HOLD J4 requests re-engagement only at 140-148
degrees. The window is a recovery guard, not a validated collision-free pose.
No automatic re-engagement occurs on interruption, error, or exit.
If interrupted after release, KEEP SUPPORTING THE ARM; torque may remain off.
Closing this script or the serial connection does not restore holding torque.
No limits, calibration, gripper settings, or error flags are changed.
"""

RESUME_PLAN = """Resume J4 holding after an interrupted release.
This mode NEVER releases a joint and never sends a target angle.
Keep supporting the links beyond J4; have a second person use the terminal.
Keep fingers clear of pinch points: enabling torque can cause movement.
The displayed angles are controller readings, not angles measured by eye.
This mode requires J4 within -149 to +149 degrees, other joints within their
documented limits, and stable readings before requesting holding torque.
These checks do not establish calibration or collision-free geometry.
No automatic action is taken on interruption. KEEP SUPPORTING THE ARM until
holding is acknowledged and you carefully verify it physically holds.
"""


def read_angles(client):
    values = client.get_angles()
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise RuntimeError("Expected six joint angles")
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) for v in values):
        raise RuntimeError("Invalid joint angles")
    for index, (value, (low, high)) in enumerate(zip(values, LIMITS)):
        if index != 3 and not low <= value <= high:
            raise RuntimeError("Another joint is outside its documented limits")
    print("Joint angles:", list(values), flush=True)
    return values


def recover(client, ask=input, pause=time.sleep):
    """Injected client/input allow testing all torque-changing paths offline."""
    torque_uncertain = False
    try:
        if client.get_system_version() != 7.3:
            raise RuntimeError("Expected the observed firmware 7.3")
        if client.is_moving() != 0:
            raise RuntimeError("Arm must report stationary before release")
        error = client.get_error_information()
        print("Initial error information:", error)
        if error != 4:
            raise RuntimeError("Expected the observed joint-4 limit error (4)")
        start = read_angles(client)
        if not 150 < start[3] <= 160:
            raise RuntimeError("J4 is outside this recovery's starting window")
        if ask("Support the arm. Type RELEASE J4 to release only J4: ") != "RELEASE J4":
            print("Cancelled; no torque command sent.")
            return 0
        # Set BEFORE transmission: missing acknowledgement does not prove
        # that the hardware ignored a command.
        torque_uncertain = True
        result = client.release_servo(4)
        print("Release response:", result, flush=True)
        if result != 1:
            raise RuntimeError("Release not acknowledged; do not force the joint")
        print("Keep supporting the links. Reposition gently toward 145 degrees.")
        while True:
            action = ask("Enter = read angles; HOLD J4 = re-engage; ABORT = exit: ")
            if action == "ABORT":
                return 1
            if action not in ("", "HOLD J4"):
                continue
            current = read_angles(client)
            if any(abs(current[i] - start[i]) > 2 for i in (0, 1, 2, 4, 5)):
                raise RuntimeError("Another joint shifted over 2 degrees; stop recovery")
            if not 135 <= current[3] <= 160:
                print("J4 is outside the original recovery window. Keep supporting "
                      "it. ABORT and use --hardware --resume-hold to request "
                      "holding at its current position after fresh checks.")
                continue
            if action != "HOLD J4":
                continue
            if not 140 <= current[3] <= 148:
                print("Re-engagement refused: J4 must read 140-148 degrees.")
                continue
            # The operator keeps the arm stationary and supported throughout.
            result = client.focus_servo(4)
            print("Re-engage response:", result, flush=True)
            if result != 1:
                raise RuntimeError("Re-engagement not acknowledged")
            pause(0.5)
            final = read_angles(client)
            if not 140 <= final[3] <= 148 or any(
                    abs(final[i] - current[i]) > 2 for i in range(6)):
                raise RuntimeError("Unexpected position after re-engagement")
            print("Final error information:", client.get_error_information())
            print("J4 re-engagement acknowledged. Carefully verify it holds before"
                  " removing support. Do not command further movement yet.")
            torque_uncertain = False
            return 0
    finally:
        if torque_uncertain:
            print("KEEP SUPPORTING THE ARM: J4 torque state is uncertain or off. "
                  "No automatic re-engagement was attempted. Do not leave it "
                  "unsupported; arrange stable physical support and assistance.",
                  file=sys.stderr, flush=True)


def resume_hold(client, ask=input, pause=time.sleep):
    """Only request J4 holding, with no release or target-position command."""
    verified = False
    try:
        if client.get_system_version() != 7.3:
            raise RuntimeError("Expected the observed firmware 7.3")
        if client.is_moving() != 0:
            raise RuntimeError("Arm must report stationary")
        error = client.get_error_information()
        print("Initial error information:", error)
        if error not in (0, 4):
            raise RuntimeError("Unexpected error; only no error or retained J4 error allowed")
        first = read_angles(client)
        if not -149 <= first[3] <= 149:
            raise RuntimeError("J4 must be inside -149 to +149 degrees for resume-hold")
        print("Request: holding torque ONLY for J4 at the current supported pose.",
              flush=True)
        if ask("Keep it stationary and supported. Type HOLD J4: ") != "HOLD J4":
            print("Cancelled; no torque command sent.")
            return 1
        current = read_angles(client)
        pause(0.5)
        stable = read_angles(client)
        if not all(-149 <= angles[3] <= 149 for angles in (current, stable)):
            raise RuntimeError("J4 left the allowed holding window")
        if any(abs(current[i] - first[i]) > 1 or
               abs(stable[i] - current[i]) > 1 for i in range(6)):
            raise RuntimeError("Readings changed over 1 degree; no holding request sent")
        if client.is_moving() != 0:
            raise RuntimeError("Arm no longer reports stationary")
        error = client.get_error_information()
        if error not in (0, 4):
            raise RuntimeError("Unexpected error before holding request")
        result = client.focus_servo(4)
        print("Re-engage response:", result, flush=True)
        if result != 1:
            raise RuntimeError("Re-engagement not acknowledged")
        pause(0.5)
        final = read_angles(client)
        if not -149 <= final[3] <= 149 or any(
                abs(final[i] - stable[i]) > 2 for i in range(6)):
            raise RuntimeError("Unexpected position after re-engagement")
        print("Final error information:", client.get_error_information())
        print("J4 re-engagement acknowledged. Carefully verify it holds before "
              "removing support. Do not command further movement yet.")
        verified = True
        return 0
    finally:
        if not verified:
            print("KEEP SUPPORTING THE ARM: holding has not been verified. "
                  "No release, target move, or automatic retry was sent. "
                  "Arrange stable physical support and assistance if needed.",
                  file=sys.stderr, flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hardware", action="store_true",
                        help="connect to the Pi and start interactive recovery")
    parser.add_argument("--resume-hold", action="store_true",
                        help="request J4 holding at its current pose; never release")
    args = parser.parse_args(argv)
    print(RESUME_PLAN if args.resume_hold else PLAN)
    if not args.hardware:
        print("Preview only. To connect: python3 -m bucke_arm.recover_joint4 --hardware")
        return 0
    client = None
    try:
        if not sys.stdin.isatty():
            raise RuntimeError("Run in an interactive terminal, without piping input")
        if metadata.version("pymycobot") != "4.0.6":
            raise RuntimeError("Expected the observed pymycobot 4.0.6")
        from pymycobot import MyCobot280, PI_PORT, PI_BAUD
        if PI_PORT != "/dev/ttyAMA0" or PI_BAUD != 1000000:
            raise RuntimeError("Library Pi constants differ from the tested settings")
        client = MyCobot280(PI_PORT, PI_BAUD)
        return resume_hold(client) if args.resume_hold else recover(client)
    except (Exception, KeyboardInterrupt) as error:
        print("Recovery stopped:", str(error) or type(error).__name__, file=sys.stderr)
        return 1
    finally:
        if client is not None:
            client._serial_port.close()


if __name__ == "__main__":
    sys.exit(main())
