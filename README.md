# Buck-E Capstone

Development workspace for Buck-E, an indoor mobile manipulation capstone using an Elephant Robotics myAGV Pi and myCobot 280 Pi arm.

## Arm preparation

The [arm starter](arm/README.md) runs on a laptop without the robot or third-party packages. It includes a read-only interface, fake telemetry, a simulated status CLI, tests, and a [first-session checklist](arm/docs/first_session.md).

```sh
cd arm
python -m bucke_arm status
python -m unittest discover -s tests -v
```

An explicit read-only hardware status adapter now uses the SDK and serial settings from the user's successful Pi telemetry test. The packaged adapter is UNVERIFIED ON HARDWARE until it is run on the Pi. See the arm README for setup and commands. Motion, calibration, gripper support, pick-and-place, vision, and mobile-base coordination remain future milestones.
