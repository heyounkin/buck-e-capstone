# Buck-E arm development

Laptop preparation for the Elephant Robotics myCobot 280 Pi. This starter supports simulated status only. It is UNVERIFIED ON HARDWARE.

## Run locally

From this directory, use Python 3.8 or newer. No third-party packages are required for simulation or tests.

```sh
python -m bucke_arm status
python -m unittest discover -s tests -v
```

On Linux, use `python3` if `python` is unavailable. The status output is labeled simulated; its numbers are test data, not calibration or robot measurements.

The default command creates only a fake arm. It never imports `pymycobot`, opens a serial port, powers motors, or commands motion. Hardware mode is deliberately unavailable until the Pi configuration and API are verified.

## Layout

- `bucke_arm/arm.py`: typed telemetry and read-only arm interface, plus fake implementation.
- `bucke_arm/hardware.py`: isolated placeholder for the verified hardware adapter.
- `bucke_arm/__main__.py`: status command and explicit backend selection.
- `config/hardware.example.json`: unknown configuration values to record on the Pi.
- `docs/first_session.md`: first hardware session checklist.
- `tests/test_status.py`: malformed telemetry, hardware isolation, and command checks.

## Next small milestone

Record the actual Pi environment and connection settings, then implement the read-only hardware adapter. Do not copy simulated values into motion configuration. Joint limits, tool calibration, gripper behavior, verified poses, vision, and AGV coordination remain future work.
