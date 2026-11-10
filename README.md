# Buck-E Capstone

Buck-E is an indoor mobile manipulation project using an Elephant Robotics
myAGV Pi base and myCobot 280 Pi arm. This repository currently contains the
arm controller, simulator, and hardware validation records.

## Run the arm simulator

From the repository root:

```sh
cd arm
python -m bucke_arm status
python -m bucke_arm.joint_control --fake
python -m unittest discover -s tests
```

See the [arm guide](arm/README.md) for Pi setup, command behavior, and offline
tools. The [Pi verification record](arm/docs/pi_verification.md) documents
hardware observations and earlier experiments.
