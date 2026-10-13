"""Reserved boundary for a future verified myCobot 280 Pi adapter.

No serial settings or API signatures are assumed, and no robot SDK is imported.
"""


# Later this factory will construct the verified, read-only Pi adapter.
# Keeping device-specific code here lets laptop code remain independent.
def create_read_only_arm():
    # Fail explicitly until the actual connection settings and API are known.
    # This function currently opens no port and sends no commands.
    raise RuntimeError(
        "Hardware status is not implemented. Verify Pi OS, Python, pymycobot "
        "version, serial port, baud rate, and read-only API behavior first."
    )
