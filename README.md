# Teleoperation

Standalone Vision Pro teleoperation core.

The default package publishes AVP hand-motion target poses without bundling robot SDKs. Robot-specific control lives under `robots/` and is intended to be split into Git submodules.

## AVP pose publisher

Install the core AVP path:

```bash
python -m pip install -e ".[avp,image]"
```

Publish AVP right-hand motion as ROS2 `geometry_msgs/PoseStamped`:

```bash
teleop-publish-avp-pose --topic Target_Pose --cert ./cert.pem --key ./key.pem
```

The publisher:

- starts a Vuer server for Vision Pro browser input,
- waits for the first valid right-hand matrix,
- treats that first matrix as the teleoperation zero frame,
- publishes relative target poses in `base_link`.

## UR10e backend

The UR10e controller is isolated under `robots/ur10e_rtde`:

```bash
python -m pip install -e robots/ur10e_rtde
teleop-ur10e-rtde --robot-ip 192.168.56.101 --topic Target_Pose
```

This backend requires ROS2 and the Python `ur_rtde` modules to be installed in the active environment.
