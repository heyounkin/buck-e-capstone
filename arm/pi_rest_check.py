"""Record 30 resting samples. Queries only; no motion or setting changes."""

import csv
from datetime import datetime
import math
from pathlib import Path
import time
from pymycobot import MyCobot280, PI_PORT, PI_BAUD, __version__


def main():
    # Check the known Pi setup before constructing an SDK client and opening UART.
    if __version__ != "4.0.6":
        raise RuntimeError("Expected the tested pymycobot 4.0.6")

    if PI_PORT != "/dev/ttyAMA0" or PI_BAUD != 1000000:
        raise RuntimeError("Connection constants differ from the tested settings")

    # Use wall time for a recognizable file name; elapsed measurements below use
    # a monotonic clock so system-clock adjustments cannot change durations.
    filename = "pi_rest_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".csv"
    print("Keep the arm untouched, everyone clear, and other control programs closed.")
    print("Read-only test. Log:", Path(filename).resolve(), flush=True)
    arm = MyCobot280(PI_PORT, PI_BAUD)
    try:
        print("Firmware:", arm.get_system_version(), flush=True)
        print("Free mode (0 = off, 1 = on):", arm.is_free_mode(), flush=True)
        print("Enabled replies J1-J6:", [arm.is_servo_enable(j) for j in range(1, 7)],
              flush=True)
        print("Initial servo status:", arm.get_servo_status(), flush=True)
        print("Initial voltages:", arm.get_servo_voltages(), flush=True)
        print("Initial temperatures:", arm.get_servo_temps(), flush=True)

        # Keep completed readings for the final per-joint variation calculation.
        samples = []

        # Exclusive creation (x) refuses an existing filename instead of replacing it.
        with open(filename, "x", newline="") as log:
            writer = csv.writer(log)
            writer.writerow(["elapsed_s", "J1_deg", "J2_deg", "J3_deg", "J4_deg",
                             "J5_deg", "J6_deg", "moving", "error"])
            started = time.monotonic()
            for sample in range(1, 31):
                # Validate the six reported joint values before recording them.
                angles = arm.get_angles()
                if not isinstance(angles, (list, tuple)) or len(angles) != 6:
                    raise RuntimeError("Expected six joint angles; received " + repr(angles))

                if any(isinstance(a, bool) or not isinstance(a, (int, float))
                       or not math.isfinite(a) for a in angles):
                    raise RuntimeError("Invalid angles: " + repr(angles))

                # These are separate synchronous queries, not an atomic snapshot.
                moving = arm.is_moving()
                error = arm.get_error_information()
                writer.writerow([round(time.monotonic() - started, 3)] + list(angles)
                                + [moving, error])
                log.flush()  # Retain completed samples if interrupted.
                print(sample, "Angles:", angles, "Moving:", moving, "Error:", error,
                      flush=True)
                samples.append(list(angles))
                if moving != 0 or error != 0:
                    # End reads without sending stop, release, or error-clear commands.
                    raise RuntimeError("Unexpected motion/status/error reply; ending reads. "
                                       "If physically moving, use the accessible power switch.")

                if sample < 30:
                    # Query time adds to this pause; samples are not exactly 1 Hz.
                    time.sleep(1)

        # Peak-to-peak change describes drift; it is not a motion approval limit.
        print("Peak-to-peak angle changes, J1-J6 (degrees):", flush=True)
        print([round(max(s[j] for s in samples) - min(s[j] for s in samples), 3)
               for j in range(6)], flush=True)
        print("Final servo status:", arm.get_servo_status(), flush=True)
        print("Final temperatures:", arm.get_servo_temps(), flush=True)
        print("Saved:", Path(filename).resolve(), flush=True)
    finally:
        # Closing communication does not stop movement or restore torque.
        arm._serial_port.close()


if __name__ == "__main__":
    # Loading the module defines main; running it explicitly starts diagnostics.
    main()
