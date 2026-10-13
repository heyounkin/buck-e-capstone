# Pi configuration and read-only verification

Recorded from the user's Pi command output and screenshots shared on October 13, 2026. These observations establish the environment and basic communication. They do not establish calibration or safe movement.

## Confirmed environment

| Item | Observed value |
| --- | --- |
| Arm model | Elephant Robotics myCobot 280 Pi, confirmed by the user |
| OS | Ubuntu 20.04.4 LTS, Focal Fossa |
| Architecture | aarch64 / ARM64 |
| Kernel | 5.4.0-1073-raspi |
| Python | 3.8.10, `/usr/bin/python3` |
| pymycobot | 4.0.6 |
| pyserial | 3.5 |
| Package directory | `/home/er/.local/lib/python3.8/site-packages` |
| Linux user | `er` |
| Serial device | `/dev/ttyAMA0` |
| Serial alias | `/dev/serial0` points to `ttyAMA0` |
| Device permissions | `crw-rw----`, owned by `root:dialout` |
| User access | `er` belongs to `dialout` |
| Tested baud rate | 1,000,000 |

No `/dev/ttyUSB*` or `/dev/ttyACM*` devices matched the inventory command. `/dev/serial1` points to `ttyS0`; this was not the tested arm connection.

## Installed API inspection

The installed `MyCobot280` constructor signature is:

```python
(port, baudrate='115200', timeout=0.1, debug=False, thread_lock=True)
```

The library exports `PI_PORT = '/dev/ttyAMA0'` and `PI_BAUD = 1000000`. Supply the tested baud explicitly because the constructor default differs.

The captured constructor opens a serial connection, sets RTS to false, and waits 1.5 seconds. It contains no explicit motor-power or movement call. A later screenshot showed that the parent `CommandGenerator` constructor initializes logging and configuration. The methods inspected in that screenshot use:

```python
get_angles(): self._mesg(ProtocolCode.GET_ANGLES, has_reply=True)
get_coords(): self._mesg(ProtocolCode.GET_COORDS, has_reply=True)
```

The coordinate method describes a base-referenced `[x, y, z, rx, ry, rz]` result. The orientation convention and configured tool offset have not been independently verified. This inspection covers the captured functions, not every SDK code path.

## Standalone telemetry test passed

The user ran a standalone Python script that opened the tested connection, requested angles and coordinates three times, checked for six finite numeric values in each list, and closed the serial port. All three samples were identical:

```text
Joint angles: [-2.63, 8.7, -26.36, 130.34, 99.14, 9.31]
Tool coordinates: [-54.8, -15.8, 304.4, 122.17, -74.02, 142.64]
PASS: three valid telemetry samples received.
```

These are historical readings transcribed from the user's screenshot. They are not home poses, calibration values, or approved motion targets. Identical samples demonstrate repeatable reported values over this short test; measurement accuracy is not established.

## Repository deployment remains pending

- Running `python3 -m bucke_arm status --backend hardware` from the Pi's home directory returned `No module named bucke_arm`. The repository had not yet been cloned onto the Pi.
- HTTPS cloning encountered authentication failure and later a 403 response containing `Write access to repository not granted`. Successful GitHub authentication on the Pi has not been confirmed. The exact credential or permission cause is unresolved.
- The repository now includes the read-only adapter based on the tested connection and SDK. It passes ten laptop tests using fake/mocked hardware. The packaged hardware command has not been confirmed on the Pi and remains **UNVERIFIED ON HARDWARE**.

Next session: transfer the repository by USB or resolve Git authentication, enter its `arm` directory, run the tests and simulated status, then run the explicit hardware status command. See the [arm README](../README.md#run-on-the-pi).

## Remaining unknowns

Hardware revision and firmware; gripper model and connection; mounting and power arrangement; tool offset; orientation convention; measured joint limits; working clearance; validated starting and transport poses; camera calibration; docking tolerances; and safe motion behavior remain unverified.

## Evidence

The four original text outputs are preserved under [evidence/2026-10-13](evidence/2026-10-13):

- `buck-e-pi-info.txt`: OS, kernel, Python, and installed packages.
- `buck-e-pi-info2.txt`: SDK location, constants, and method signatures.
- `buck-e-pi-info3.txt`: serial devices and user groups.
- `buck-e-pi-info4.txt`: installed arm constructor source.

Parent-constructor inspection, telemetry results, and deployment errors above were transcribed from screenshots provided in this chat. Their original text logs were not supplied. [hardware.observed.json](../config/hardware.observed.json) summarizes the observations for reference; the current adapter does not load that file.
