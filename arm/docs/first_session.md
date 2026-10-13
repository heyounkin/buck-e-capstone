# First arm session

Goal: verify the environment and obtain actual joint angles and tool coordinates. The model is confirmed as myCobot 280 Pi. The user has reported Ubuntu 20.04.4, Python 3.8.10, pymycobot 4.0.6, and pyserial 3.5, and successfully read three telemetry samples over `/dev/ttyAMA0` at 1,000,000 baud. Revision, gripper, mounting, and calibration remain unverified.

The standalone communication milestone is complete. See the [verification record](pi_verification.md) for evidence. The immediate next task is transferring the repository to the Pi and testing the packaged adapter; GitHub authentication is currently unresolved. The inventory steps below remain useful for rechecking the setup.

## Inventory

- Record the model label and hardware revision.
- Identify the installed gripper, connection, power supply, and mounting.
- Keep the AGV stationary. Identify emergency-stop access before later motion testing.
- Record the Pi environment using these inspection commands:

```sh
uname -a
cat /etc/os-release
python3 --version
python3 -m pip show pymycobot
ls -l /dev/ttyAMA* /dev/ttyUSB* /dev/ttyACM*
```

Missing package or device messages are findings to record, not reasons to guess settings. Do not put credentials or private keys into logs or Git.

## Read-only connection milestone

- Verify the correct installed SDK class, port, baud rate, and API against the Pi image and device documentation.
- Verify that connecting and reading telemetry do not require a motion or power-on command.
- Implement the hardware adapter only after those details are known.
- Check for six finite joint angles and six finite tool-pose values. Record actual units, orientation convention, reference frame, and any tool offset.
- Record errors and repeated readings; do not silently replace invalid readings with simulated data.

Success is repeatable real telemetry with a documented connection configuration. The standalone Pi test achieved this; next verify the packaged adapter using the arm README commands.

## Later sessions

Review measured joint limits, a clear working volume, speed limits, and a verified starting pose before adding supervised motion with explicit execution opt-in. Then address tool calibration, gripper testing, fixed pick-and-place, vision, and AGV interlocks in separate milestones.
