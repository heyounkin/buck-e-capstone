# Buck-E Capstone

Development workspace for Buck-E, an indoor mobile manipulation capstone using
an Elephant Robotics myAGV Pi and myCobot 280 Pi arm.

## Arm

The [arm package](arm/README.md) provides `RealArm` for the Pi and `FakeArm`
for laptop development. Both expose the same status, joint, gripper and saved-pose
methods. The user confirmed synchronous standby-to-ready and return movements
work safely on October 28, 2026.

`joint_control.py` is a small manual testing menu:

```sh
cd arm
python -m bucke_arm.joint_control          # RealArm on the Pi
python -m bucke_arm.joint_control --fake   # FakeArm on a laptop
```

Select R to go from standby to ready, or S to return; no second confirmation is required. Reusable
movement checks live in `RealArm`, outside the menu. The classes and offline
preview share the same saved ready pose.

```sh
python -m bucke_arm status
python -m unittest discover -s tests -v
```

Historical observations are preserved in the
[Pi verification record](arm/docs/pi_verification.md). Calibration, pick-and-place,
vision, and mobile-base coordination remain future milestones.
