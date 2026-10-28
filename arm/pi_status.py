"""Run on the Pi: python3 pi_status.py. Reads the arm; never moves it."""

import time
from pymycobot import MyCobot280, PI_PORT, PI_BAUD, __version__


def main():
    # These constants come from pymycobot, not from our own robot driver.
    print("pymycobot version:", __version__)
    print("Serial port:", PI_PORT)
    print("Baud rate:", PI_BAUD)

    # Stop before connecting if these differ from the setup tested on your Pi.
    if __version__ != "4.0.6":
        raise RuntimeError("This setup was tested with pymycobot 4.0.6")

    if PI_PORT != "/dev/ttyAMA0" or PI_BAUD != 1000000:
        raise RuntimeError("The library constants differ from your tested settings")

    # This creates the library's robot object and opens the serial connection.
    arm = MyCobot280(PI_PORT, PI_BAUD)
    try:
        print("Firmware:", arm.get_system_version())
        print("Angle list order: J1, J2, J3, J4, J5, J6 (degrees)")

        # Read three times to see whether the reported angles change at rest.
        for sample in range(1, 4):
            print("\nSample", sample)
            print("Joint angles:", arm.get_angles())

            # Read status separately; angle, moving, and error replies are not
            # an atomic sample. These diagnostics intentionally preserve raw values.
            print("Moving (0 = no, 1 = yes):", arm.is_moving())
            print("Error (0 = none):", arm.get_error_information())
            if sample < 3:
                time.sleep(2)

        # Print raw diagnostic readings; do not guess at missing/invalid values.
        print("\nServo status:", arm.get_servo_status())
        print("Servo voltages:", arm.get_servo_voltages())
        print("Servo temperatures:", arm.get_servo_temps())
        
    finally:
        # This closes communication even if a reading fails. It does not stop
        # movement or change holding torque. This field was verified on the Pi.
        arm._serial_port.close()


# Importing the file does not connect; running it directly calls main().
if __name__ == "__main__":
    main()
