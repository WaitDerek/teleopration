# Design

## Source of truth
- Status: Draft
- Last refreshed: 2026-06-15
- Primary product surfaces:
  - A standalone Vision Pro teleoperation repository based on the added `TeleVision` core.
  - Optional robot driver repositories attached as Git submodules.
  - Example applications for AVP hand-pose inspection, ROS2 pose publishing, image streaming, replay, and real robot operation.
- Evidence reviewed:
  - `third_party/Wongtsai_arm/teleop_core/README.md`
  - `third_party/Wongtsai_arm/teleop_core/TeleVision/teleop.py`
  - `third_party/Wongtsai_arm/teleop_core/TeleVision/teleop/TeleVision.py`
  - `third_party/Wongtsai_arm/teleop_core/TeleVision/teleop/Preprocessor.py`
  - `third_party/Wongtsai_arm/teleop_core/TeleVision/teleop/constants_vuer.py`
  - `third_party/Wongtsai_arm/teleop_core/TeleVision/teleop/motion_utils.py`
  - `third_party/Wongtsai_arm/teleop_core/TeleVision/teleop/webrtc/*`
  - `third_party/Wongtsai_arm/teleop_core/TeleVision/teleop/dynamixel/*`
  - `third_party/Wongtsai_arm/teleop_core/TeleVision/teleop/teleop_active_cam.py`
  - `third_party/Wongtsai_arm/teleop_core/TeleVision/teleop/teleop_hand.py`
  - `third_party/Wongtsai_arm/teleop_core/TeleVision/requirements.txt`
  - `third_party/M3DexLab/README.md`
  - `third_party/M3DexLab/.gitmodules`
  - `third_party/M3DexLab/teleopration/oop/CMakeLists.txt`
  - `third_party/M3DexLab/teleopration/oop/app/main.cpp`
  - `third_party/M3DexLab/teleopration/oop/app/replay.cpp`
  - `third_party/M3DexLab/teleopration/oop/app/ur10e_main.cpp`
  - `third_party/M3DexLab/teleopration/oop/include/trajectory.hpp`
  - `third_party/M3DexLab/teleopration/oop/include/common.hpp`
  - `third_party/M3DexLab/teleopration/oop/src/odom_teleop/engine.cpp`
  - `third_party/M3DexLab/teleopration/oop/thirdparty/ros2_utils/LiOdometry/*`

## Brand
- Personality:
  - Research-grade, direct, modular, and safe by default.
  - The project should feel like a small robotics infrastructure layer, not a monolithic demo.
- Trust signals:
  - Explicit dependency boundaries.
  - Clear safety limits before hardware motion.
  - Minimal default install path.
  - Reproducible examples for simulation and replay before real robot execution.
- Avoid:
  - Hiding robot-specific SDK dependencies in the core package.
  - Requiring large robot assets, MuJoCo, ROS2, or vendor SDKs for users who only need teleoperation input.
  - Hard-coded robot IP addresses, joint names, site names, or gripper behavior in the core layer.

## Product goals
- Goals:
  - Extract the `TeleVision` AVP teleoperation core into a standalone Python-first repository.
  - Keep the default clone focused on AVP hand/head pose capture, image streaming, pose preprocessing, calibration, safety, and optional ROS2 pose publishing.
  - Let each robot backend live in an optional submodule with its own dependencies and build flags.
  - Provide a stable Python interface so new arms, grippers, cameras, and simulators can be added without changing the AVP teleoperation core.
- Non-goals:
  - Do not ship every robot controller in the core repository by default.
  - Do not make MuJoCo/oomj a mandatory runtime dependency.
  - Do not make Dynamixel, ZED SDK, Isaac Gym, FoundationPose, or dex-retargeting mandatory dependencies.
  - Do not make UR10e or Robotiq the implicit architecture center.
  - Do not preserve the current misspelled `teleopration` name in new public paths.
- Success signals:
  - A user can clone the core repository and run AVP pose inspection without initializing robot submodules.
  - A user can initialize only one robot submodule and build only that backend.
  - Vision Pro hand/head pose input can be replayed or inspected without connecting real robot hardware.
  - Robot-specific code is removable without breaking the core build.

## Personas and jobs
- Primary personas:
  - Teleoperation users who only need Vision Pro pose streaming and replay.
  - Robotics researchers adding one hardware backend.
  - Maintainers who need to update robot SDK integrations independently.
- User jobs:
  - Receive AVP hand, head, landmark, and pinch events through Vuer/WebRTC.
  - Display stereo or side-by-side camera streams inside the AVP session.
  - Calibrate AVP hand pose deltas to a robot TCP/base frame.
  - Filter and rate-limit target motion.
  - Publish target poses to ROS2 or route them to a selected simulator or real robot driver.
  - Add a new robot backend without forcing other users to install that backend.
- Key contexts of use:
  - Local development without hardware.
  - Simulation validation.
  - Lab hardware teleoperation with explicit safety gates.

## Information architecture
- Primary navigation:
  - `teleoperation/avp/` for Vuer/TeleVision server, event capture, and shared AVP state.
  - `teleoperation/preprocessing/` for frame conversion, hand-pose normalization, and pose-delta extraction.
  - `teleoperation/streaming/` for shared-memory images and optional WebRTC video.
  - `teleoperation/ros2/` for optional ROS2 publishers, subscribers, and action clients.
  - `teleoperation/recording/` for dataset recording utilities.
  - `robots/` for optional Git submodules and backend adapters.
  - `examples/` for runnable flows.
  - `docs/` for migration, integration, and safety notes.
- Core routes/screens:
  - Not applicable; this is a robotics/library repository, not a frontend surface.
- Content hierarchy:
  - README starts with default no-hardware usage.
  - Robot backend documentation is linked from optional submodule sections.
  - Real hardware execution instructions must appear after replay/simulation validation.

## Design principles
- Minimal core:
  - Core teleoperation must not depend on robot vendor SDKs, MuJoCo assets, Isaac Gym, Dynamixel hardware, ZED SDK, FoundationPose, or a specific arm.
- Dependency inversion:
  - The core defines AVP event sources, pose preprocessing, safety filters, and robot driver interfaces.
  - Submodules implement those interfaces.
- Hardware safety first:
  - Real robot examples must require explicit driver selection, connection configuration, workspace limits, and an emergency stop path.
- Replay before hardware:
  - Any pose-processing change should be testable with recorded data before running on a real robot.
- Tradeoffs:
  - Interface stability is more important than exposing every vendor-specific control knob in the core.
  - Optional submodules add setup steps for hardware users, but they keep the default teleoperation path lightweight.

## Visual language
- Color:
  - Not applicable for the core library.
- Typography:
  - Documentation should use concise technical prose, command blocks, and tables for backend capabilities.
- Spacing/layout rhythm:
  - Keep README and docs scan-friendly with short sections.
- Shape/radius/elevation:
  - Not applicable.
- Motion:
  - Motion terminology must distinguish device pose, filtered target pose, robot command, and measured robot state.
- Imagery/iconography:
  - Prefer diagrams for dataflow and frame transforms over screenshots unless documenting a Vision Pro UI.

## Components
- Existing components to reuse:
  - `OpenTeleVision` from `TeleVision.py` and `teleop.py` as the AVP/Vuer session server.
  - `VuerPreprocessor` from `Preprocessor.py` as the starting point for frame conversion and hand-pose normalization.
  - `constants_vuer.py` and `motion_utils.py` as frame-transform and matrix utility seeds.
  - `streaming_process` patterns from `teleop.py` as the side-by-side AVP image stream path after camera-specific code is isolated.
  - `PosePublisherNode`, ROS2 listeners, and `PinchActionClient` from `teleop.py` as optional ROS2 adapters.
  - Recording helpers from `teleop.py` and `script/*` as optional dataset tooling.
  - `OdometryReader` behavior from `thirdparty/ros2_utils/LiOdometry` as an optional ROS2 transport.
  - Position/orientation filtering logic from `include/trajectory.hpp`, after removing `oomj` coupling.
  - UR10e RTDE control flow from `app/ur10e_main.cpp` as the first robot driver submodule.
  - MuJoCo/oomj simulation flow from `app/main.cpp`, `common.hpp`, `engine.cpp`, and `oscontrol.cpp` as an optional simulation backend.
- New/changed components:
  - `teleoperation.types.Pose`
  - `teleoperation.avp.OpenTeleVision`
  - `teleoperation.avp.AvpState`
  - `teleoperation.preprocessing.VuerPreprocessor`
  - `teleoperation.session.TeleopSession`
  - `teleoperation.safety.SafetyLimiter`
  - `teleoperation.calibration.Calibration`
  - `teleoperation.filter.MotionFilter`
  - `teleoperation.ros2.TargetPosePublisher`
  - `teleoperation.robot.RobotDriver`
- Variants and states:
  - AVP source states: disconnected, waiting_for_browser, streaming, stale, error.
  - Image stream states: disabled, waiting_for_frame, streaming, stale, error.
  - Robot driver states: disconnected, connected, enabled, faulted, stopped.
  - Session states: idle, calibrating, armed, running, paused, emergency_stopped.
- Token/component ownership:
  - Core owns interfaces and common pose-processing utilities.
  - Each robot submodule owns vendor SDK usage, robot-specific configuration, assets, and hardware examples.

## Accessibility
- Target standard:
  - Not applicable as a UI standard, but CLI and logs should be usable in terminal-only lab environments.
- Keyboard/focus behavior:
  - Hardware examples must support keyboard or command-line stop controls where applicable.
- Contrast/readability:
  - Logs should clearly distinguish warnings, safety stops, stale input, and robot faults.
- Screen-reader semantics:
  - Not applicable.
- Reduced motion and sensory considerations:
  - Motion smoothing and rate limiting are safety controls, not visual preferences.

## Responsive behavior
- Supported breakpoints/devices:
  - Not applicable.
- Layout adaptations:
  - Not applicable.
- Touch/hover differences:
  - Not applicable.

## Interaction states
- Loading:
  - Waiting for AVP browser session.
  - Waiting for first valid hand matrix.
  - Waiting for image stream shared memory.
  - Waiting for robot connection.
- Empty:
  - No robot submodules initialized.
  - No hand/head pose samples received.
  - No camera frames received.
- Error:
  - Missing optional dependency.
  - Missing robot submodule.
  - Stale AVP event stream.
  - Invalid or singular hand matrix.
  - Shared-memory allocation/cleanup failure.
  - Robot driver fault.
- Success:
  - Pose source streaming.
  - Calibration captured.
  - Robot driver armed.
  - Commands accepted inside safety limits.
- Disabled:
  - Real robot command output disabled until explicit arm/enable step.
- Offline/slow network, if applicable:
  - Treat stale Vision Pro/odometry input as a stop condition, not as a command hold forever.

## Content voice
- Tone:
  - Direct, safety-aware, and implementation-focused.
- Terminology:
  - Use `teleoperation` for new public names.
  - Refer to the current source directory as legacy `teleopration` only when pointing to migration inputs.
  - Use `Vision Pro pose source` for device input, not a specific robot command path.
- Microcopy rules:
  - Hardware commands must say which robot and network endpoint they affect.
  - Optional dependencies must be marked optional in docs and CMake flags.

## Implementation constraints
- Framework/styling system:
  - Python-first package for the AVP teleoperation core.
  - Vuer/WebRTC support is part of the AVP core path.
  - ROS2 support is optional and isolated under `teleoperation/ros2`.
  - C++/CMake is only needed for optional legacy/simulation/robot backends.
  - Robot backends are optional submodules under `robots/`.
- Design-token constraints:
  - Not applicable.
- Performance constraints:
  - AVP event processing should support 30-60 Hz hand/head pose updates.
  - Image streaming should tolerate 30 Hz side-by-side frames.
  - Real robot drivers may impose lower control rates; adapters must declare their expected rate.
- Compatibility constraints:
  - Core must install without `ur_rtde`, MuJoCo, oomj, Isaac Gym, ZED SDK, Dynamixel SDK, FoundationPose, dex-retargeting, or robot assets.
  - OpenCV and ROS2 should be extras unless required by the selected streaming/ROS2 mode.
  - Optional transports and robot submodules may add those dependencies behind build flags.
- Test/screenshot expectations:
  - Core tests cover matrix validity handling, frame transforms, AVP hand-pose delta extraction, filtering, stale-input detection, and safety limits.
  - Optional robot drivers provide compile tests or mocked driver tests when hardware is unavailable.

## Open questions
- [x] Where is the actual Vision Pro app or streaming server code located? Answer: `third_party/Wongtsai_arm/teleop_core/TeleVision` contains the Vuer/WebRTC AVP teleoperation server and should seed the new core.
- [ ] Which first robot backends are required besides UR10e + Robotiq? Impact: shapes the first `RobotDriver` interface capabilities.
- [ ] Should ROS2 be a default extra for this lab workflow or an optional adapter only? Impact: default installation weight and run command shape.
- [ ] What safety policy is mandatory for lab hardware use? Impact: default workspace limits, enable sequence, and stop behavior.
- [ ] Should the new repository be a pure Python package with ROS2 optional modules, or a ROS2 workspace package? Impact: install, launch, and dependency management.
