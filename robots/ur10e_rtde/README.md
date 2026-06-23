# UR10e RTDE Teleoperation Backend

Submodule-ready UR10e controller for the root `teleoperation` package.

It subscribes to AVP target poses published by:

```bash
teleop-publish-avp-pose --topic Target_Pose
```

Then it controls the UR10e with RTDE:

```bash
python -m pip install -e robots/ur10e_rtde
teleop-ur10e-rtde --robot-ip 192.168.56.101 --topic Target_Pose
```

The first measured TCP pose becomes the robot zero frame. Incoming `Target_Pose` messages are treated as relative teleoperation deltas and are composed onto that initial TCP pose before `servoL`.

Required external setup:

- ROS2 Python environment sourced.
- Python UR RTDE modules installed in the active environment.
- Network access to the robot controller.
- Robot safety mode and teach pendant state allow external control.
