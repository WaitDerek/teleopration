# Teleoperation

Standalone Vision Pro to UR10e teleoperation core.

Default runtime environment:

```bash
mamba activate fast
```

Run commands from the repository root. The AVP tools read `PUBLIC_HOST` from `config/teleop.env` and derive the local Vuer client URL automatically, so `--client-url` is normally not needed. After switching Wi-Fi, update `PUBLIC_HOST` in `config/teleop.env`.

Edit the runtime config after switching Wi-Fi:

```text
config/teleop.env
```

Current fields:

```bash
PUBLIC_HOST=192.168.41.27
ROBOT_IP=192.168.56.2
CAMERA_INDEX=4
FRAME_CALIBRATION_FILE=recordings/avp_forward_xyz_calibration.json
PORT=8012
```

`client-url` is derived automatically from `PUBLIC_HOST` and `PORT`:

```text
https://<PUBLIC_HOST>:<PORT>
```

## Readiness check

```bash
./scripts/check_teleop_readiness.sh
```

Expected final state:

```text
teleop_session_ready=true
```

## Coordinate calibration

The default calibration file is:

```text
recordings/avp_forward_xyz_calibration.json
```

Create or overwrite it:

```bash
python -m teleoperation.cli.calibrate_avp_frame --enter-to-stop
```

Open the printed Vision Pro URL, enter the Vuer session, then follow the terminal prompts:

```text
+X: press Enter, move hand forward, press Enter to stop
+Y: press Enter, move hand right, press Enter to stop
+Z: press Enter, move hand up, press Enter to stop
```

With this convention:

```text
hand forward -> delta +X
hand right   -> delta +Y
hand up      -> delta +Z
```

If left should be positive Y, move left during the +Y calibration step.

## Check calibrated hand delta

This does not publish ROS messages and does not move the robot:

```bash
python -m teleoperation.cli.check_avp_calibrated_delta
```

It prints the calibrated hand displacement relative to the zero pose:

```text
raw_delta_m=x=... y=... z=...
scaled_delta_m=x=... y=... z=...
```

The default scale is `0.08`, so `scaled_delta_m = raw_delta_m * 0.08`.

## Start Vision Pro publisher

```bash
python -m teleoperation.cli.publish_avp_pose --debug-avp
```

Defaults used by this command:

```text
topic=Target_Pose
position_scale=0.08
orientation_scale=0.0
image_opacity=1.0
show_hands=true
frame_calibration_file=recordings/avp_forward_xyz_calibration.json
```

If you need to run without coordinate calibration:

```bash
python -m teleoperation.cli.publish_avp_pose --no-frame-calibration --debug-avp
```

## Start USB camera image stream

Use the camera index reported by the readiness check. In the current setup, index `4` has worked:

```bash
python -m teleoperation.cli.stream_usb_camera --camera-index 4 --wait-for-shm --backend-v4l2
```

If the image is mirrored incorrectly:

```bash
python -m teleoperation.cli.stream_usb_camera --camera-index 4 --wait-for-shm --backend-v4l2 --no-mirror
```

## Start UR10e backend

```bash
teleop-ur10e-rtde --robot-ip 192.168.56.2 --topic Target_Pose --enable-gripper
```

The backend uses incremental target deltas and limits each position step to `0.03 m` by default.

## Typical startup order

1. `mamba activate fast`
2. Run the readiness check.
3. Start `teleop-ur10e-rtde`.
4. Start `python -m teleoperation.cli.publish_avp_pose --debug-avp`.
5. Start `python -m teleoperation.cli.stream_usb_camera --camera-index 4 --wait-for-shm --backend-v4l2`.
6. Open the printed Vision Pro URL and enter the Vuer session.
