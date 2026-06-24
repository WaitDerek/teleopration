#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

ROBOT_IP="${ROBOT_IP:-192.168.56.2}"

mamba run -n fast teleop-check-ur10e-state \
  --robot-ip "${ROBOT_IP}" \
  --require-gripper

mamba run -n fast teleop-ur10e-rtde \
  --robot-ip "${ROBOT_IP}" \
  --topic Target_Pose \
  --gripper-topic Gripper_Command \
  --enable-gripper \
  --speed 0.02 \
  --acceleration 0.1 \
  --max-position-delta 0.05 \
  --max-angular-delta 0.10 \
  --max-target-step 0.015 \
  --max-target-speed 0.20 \
  --max-angular-step 0.05 \
  --max-angular-speed 0.50 \
  --stale-after 0.25 \
  --gripper-speed 0.5 \
  --gripper-force 0.5 \
  --gripper-deadband 0.05 \
  --gripper-min-interval 0.1
