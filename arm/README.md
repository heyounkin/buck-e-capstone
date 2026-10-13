# Buck-E arm development

Development for the Elephant Robotics myCobot 280 Pi. Status defaults to simulation, with an explicit read-only hardware mode based on the user's successful Pi test. The packaged adapter is UNVERIFIED ON HARDWARE until it is run on the Pi.

See the [Pi verification record](docs/pi_verification.md) for the confirmed environment, API inspection, successful standalone readings, deployment blockers, and remaining unknowns.

## Run locally

From this directory, use Python 3.8 or newer. No third-party packages are required for simulation or tests.

```sh
python -m bucke_arm status
python -m unittest discover -s tests -v
```

On Linux, use `python3` if `python` is unavailable. The status output is labeled simulated; its numbers are test data, not calibration or robot measurements.

The default command creates only a fake arm. It never imports `pymycobot`, opens a serial port, powers motors, or commands motion.

## Run on the Pi

Python must run from the `arm` directory, which contains `bucke_arm`. A command run from the home directory cannot find the package unless it has been installed separately.

For a first clone (only if this destination does not already exist):

```sh
cd ~
git clone https://github.com/heyounkin/buck-e-capstone.git
cd ~/buck-e-capstone/arm
python3 -m bucke_arm status
python3 -m bucke_arm status --backend hardware
```

For an existing checkout at that location:

```sh
cd ~/buck-e-capstone
git pull --ff-only
cd arm
python3 -m bucke_arm status --backend hardware
```

If your checkout is elsewhere, substitute its actual directory. If GitHub is unreachable, transfer the repository files by USB and run from the copied `arm` directory.

Hardware mode is scoped to the reported Ubuntu 20.04.4/Python 3.8.10 environment with `pymycobot` 4.0.6 and `pyserial` 3.5. It explicitly uses `/dev/ttyAMA0` at 1,000,000 baud, queries angles and coordinates, validates them, then closes the serial port. It never powers motors or sends movement commands. Other SDK versions are refused before connection; do not upgrade the working Pi environment to run this code.

The user's standalone test returned three valid samples. This verifies basic communication, not tool calibration, orientation conventions, safe targets, or the packaged adapter itself. Keep other serial-control programs closed while checking status. Hardware failures produce an error and nonzero exit code, with no simulated fallback.

## Layout

- `bucke_arm/arm.py`: typed telemetry and read-only arm interface, plus fake implementation.
- `bucke_arm/hardware.py`: isolated read-only adapter for the tested SDK and connection.
- `bucke_arm/__main__.py`: status command and explicit backend selection.
- `config/hardware.example.json`: unknown configuration values to record on the Pi.
- `config/hardware.observed.json`: confirmed observations and historical telemetry; not executable motion configuration or an adapter input.
- `docs/pi_verification.md`: verification results and links to preserved command output.
- `docs/first_session.md`: first hardware session checklist.
- `tests/test_status.py` and `tests/test_hardware.py`: telemetry, hardware isolation, connection settings, and cleanup checks with mocked hardware.

## Next small milestone

Run the packaged hardware status command on the Pi and save its output. Do not copy telemetry or simulated values into motion configuration. Joint limits, tool calibration, gripper behavior, verified poses, vision, and AGV coordination remain future work.
