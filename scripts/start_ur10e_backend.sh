#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

ROBOT_IP="${ROBOT_IP:-192.168.56.2}"
MAX_POSITION_DELTA="${MAX_POSITION_DELTA:-0.05}"
MAX_ANGULAR_DELTA="${MAX_ANGULAR_DELTA:-0.25}"
MAX_TARGET_STEP="${MAX_TARGET_STEP:-0.015}"
MAX_TARGET_SPEED="${MAX_TARGET_SPEED:-0.20}"
MAX_ANGULAR_STEP="${MAX_ANGULAR_STEP:-0.08}"
MAX_ANGULAR_SPEED="${MAX_ANGULAR_SPEED:-1.00}"

echo "UR10e safety: max_position=${MAX_POSITION_DELTA}m, max_angle=${MAX_ANGULAR_DELTA}rad, max_step=${MAX_TARGET_STEP}m, max_speed=${MAX_TARGET_SPEED}m/s, max_angular_step=${MAX_ANGULAR_STEP}rad, max_angular_speed=${MAX_ANGULAR_SPEED}rad/s"

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
  --max-position-delta "${MAX_POSITION_DELTA}" \
  --max-angular-delta "${MAX_ANGULAR_DELTA}" \
  --max-target-step "${MAX_TARGET_STEP}" \
  --max-target-speed "${MAX_TARGET_SPEED}" \
  --max-angular-step "${MAX_ANGULAR_STEP}" \
  --max-angular-speed "${MAX_ANGULAR_SPEED}" \
  --stale-after 0.25 \
  --gripper-speed 0.5 \
  --gripper-force 0.5 \
  --gripper-deadband 0.05 \
  --gripper-min-interval 0.1
