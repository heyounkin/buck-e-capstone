# Buck-E Capstone

Development workspace for Buck-E, an indoor mobile manipulation capstone using an Elephant Robotics myAGV Pi and myCobot 280 Pi arm.

## Arm preparation

The [arm starter](arm/README.md) runs on a laptop without the robot or third-party packages. It includes a read-only interface, fake telemetry, a simulated status CLI, tests, and a [first-session checklist](arm/docs/first_session.md).

The [Pi verification record](arm/docs/pi_verification.md) documents the confirmed Pi/API configuration and successful standalone telemetry test, with original command outputs and remaining integration work.

The arm package also includes a connection-free `move_to_ready()` preview using
the corrected target `[-90, 0, 0, 0, 90, -135]` degrees. It reports signed joint
changes from supplied state, checks configurable joint bounds, and previews
explicit candidate waypoint routes with supported starting ranges. Physical
route review and execution remain pending. See
the [offline preview instructions](arm/README.md#offline-move_to_ready-preview).

```sh
cd arm
python -m bucke_arm status
python -m unittest discover -s tests -v
```

The explicit read-only hardware status adapter was verified on the Pi on October 21, 2026. A subsequent J4 limit error was resolved through supervised manual recovery and one error clear. The one-degree J1 attempts did not reach their target. The user subsequently ran a five-degree J1 test to target 144.06 degrees at speed setting 10 and reported movement; final J1 was 144.93 degrees, stationary and error-free, so the helper timed out without confirming the target and requested stop. The user confirmed that only the physical J1 base joint moved; target accuracy and repeatability remain unresolved. Earlier unexpected J4/J5 changes remain unexplained. See the arm README for the test scope and verification record. Calibration, gripper support, pick-and-place, vision, and mobile-base coordination remain future milestones.
