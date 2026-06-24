# UR10e RTDE Teleoperation Backend

Submodule-ready UR10e controller for the root `teleoperation` package.

It subscribes to AVP target poses published by:

```bash
teleop-publish-avp-pose --topic Target_Pose
```

For first hardware bring-up, use simulated AVP output instead of Vision Pro:

```bash
teleop-sim-avp-output \
  --topic Target_Pose \
  --gripper-topic Gripper_Command \
  --pattern line-x \
  --amplitude 0.02 \
  --duration 20
```

Then it controls the UR10e with RTDE:

```bash
python -m pip install -e robots/ur10e_rtde
teleop-ur10e-rtde --robot-ip 192.168.56.101 --topic Target_Pose
```

The first measured TCP pose becomes the robot zero frame. Incoming `Target_Pose` messages are treated as relative teleoperation deltas and are composed onto that initial TCP pose before `servoL`.
If pose messages stop arriving for `--stale-after` seconds, the backend calls `servoStop`.

Suggested test order:

1. Start `teleop-ur10e-rtde --robot-ip <ip> --topic Target_Pose` with gripper disabled.
2. Run `teleop-sim-avp-output --amplitude 0.02 --duration 20` and verify a small servo motion.
3. Restart the backend with `--enable-gripper`.
4. Run `teleop-sim-avp-output --toggle-gripper --duration 20` and verify open/close.
5. Run `teleop-check-avp-output --duration 20 --require-motion 0.005 --require-gripper-change` before replacing the simulator with live AVP publishing.

To control a Robotiq 2F85 from AVP finger closing/pinch:

```bash
teleop-publish-avp-pose --topic Target_Pose --gripper-topic Gripper_Command
teleop-ur10e-rtde \
  --robot-ip 192.168.56.101 \
  --topic Target_Pose \
  --enable-gripper \
  --gripper-topic Gripper_Command
```

`Gripper_Command` is `std_msgs/Float32`: `0.0` means fully open and `1.0` means fully closed.
The backend sends this to the Robotiq URCap socket on port `63352` by default.

Required external setup:

- ROS2 Python environment sourced.
- Python UR RTDE modules installed in the active environment.
- Network access to the robot controller.
- Robot safety mode and teach pendant state allow external control.
- For gripper control, the Robotiq gripper URCap exposes the socket port configured by `--gripper-port`.
