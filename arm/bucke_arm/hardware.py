"""Read-only adapter for the API and connection tested on Buck-E's Pi.

Importing this module does not import the robot SDK or open a serial port.
"""

from importlib import metadata

from .arm import Telemetry


class HardwareArm:
    """Wrap the SDK client with only telemetry and connection cleanup."""

    def __init__(self, client):
        self._client = client

    def read_status(self) -> Telemetry:
        # These are the two query methods verified on the actual Pi.
        angles = self._client.get_angles()
        coords = self._client.get_coords()
        if not isinstance(angles, (list, tuple)) or not isinstance(coords, (list, tuple)):
            raise RuntimeError("The arm did not return joint and coordinate lists")
        telemetry = Telemetry("hardware", tuple(angles), tuple(coords))
        telemetry.validate()
        return telemetry

    def close(self) -> None:
        # This private SDK field was confirmed in the installed constructor
        # and used successfully in the Pi telemetry test. Isolate it here.
        self._client._serial_port.close()


def create_read_only_arm() -> HardwareArm:
    # Scope this adapter to the SDK release inspected and tested on the Pi.
    # No installation or upgrade occurs automatically.
    try:
        version = metadata.version("pymycobot")
    except metadata.PackageNotFoundError as error:
        raise RuntimeError("pymycobot is missing from this Python environment") from error
    if version != "4.0.6":
        raise RuntimeError("Expected verified pymycobot 4.0.6; found " + version)
    try:
        from pymycobot import MyCobot280
    except ImportError as error:
        raise RuntimeError("Cannot import MyCobot280 from pymycobot") from error

    # Explicit settings from the user's successful three-sample Pi test.
    # Do not use the constructor's different default baud rate.
    return HardwareArm(MyCobot280("/dev/ttyAMA0", 1000000))
