"""Reserved boundary for a future verified myCobot 280 Pi adapter.

No serial settings or API signatures are assumed, and no robot SDK is imported.
"""


def create_read_only_arm():
    raise RuntimeError(
        "Hardware status is not implemented. Verify Pi OS, Python, pymycobot "
        "version, serial port, baud rate, and read-only API behavior first."
    )
