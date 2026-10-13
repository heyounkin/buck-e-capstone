# Buck-E Capstone

Development workspace for Buck-E, an indoor mobile manipulation capstone using an Elephant Robotics myAGV Pi and myCobot 280 Pi arm.

## Arm preparation

The [arm starter](arm/README.md) runs on a laptop without the robot or third-party packages. It includes a read-only interface, fake telemetry, a simulated status CLI, tests, and a [first-session checklist](arm/docs/first_session.md).

```sh
cd arm
python -m bucke_arm status
python -m unittest discover -s tests -v
```

Hardware integration is UNVERIFIED ON HARDWARE. The current starter cannot connect to or move the robot. Physical telemetry, motion, calibration, gripper support, pick-and-place, vision, and mobile-base coordination will be developed in later milestones.
