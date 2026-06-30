#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

ROBOT_IP="${ROBOT_IP:-192.168.56.2}"
MAX_POSITION_DELTA="${MAX_POSITION_DELTA:-0.05}"
MAX_ANGULAR_DELTA="${MAX_ANGULAR_DELTA:-0.25}"
MAX_TARGET_STEP="${MAX_TARGET_STEP:-0.03}"
MAX_TARGET_SPEED="${MAX_TARGET_SPEED:-0.0}"
MAX_ANGULAR_STEP="${MAX_ANGULAR_STEP:-0.08}"
MAX_ANGULAR_SPEED="${MAX_ANGULAR_SPEED:-0.0}"
MAX_TCP_X="${MAX_TCP_X:--0.5}"
SPEED="${SPEED:-0.02}"
MAX_SPEED="${MAX_SPEED:-0.05}"
ACCELERATION="${ACCELERATION:-0.1}"
MAX_ACCELERATION="${MAX_ACCELERATION:-0.5}"

echo "UR10e safety: speed=${SPEED}m/s, max_speed=${MAX_SPEED}m/s, acceleration=${ACCELERATION}m/s^2, max_acceleration=${MAX_ACCELERATION}m/s^2, max_position=${MAX_POSITION_DELTA}m, max_angle=${MAX_ANGULAR_DELTA}rad, max_step=${MAX_TARGET_STEP}m, max_target_speed=${MAX_TARGET_SPEED}m/s, max_angular_step=${MAX_ANGULAR_STEP}rad, max_angular_speed=${MAX_ANGULAR_SPEED}rad/s, max_tcp_x=${MAX_TCP_X}m"

mamba run -n fast teleop-check-ur10e-state \
  --robot-ip "${ROBOT_IP}" \
  --require-gripper

mamba run -n fast teleop-ur10e-rtde \
  --robot-ip "${ROBOT_IP}" \
  --topic Target_Pose \
  --gripper-topic Gripper_Command \
  --enable-gripper \
  --speed "${SPEED}" \
  --max-speed "${MAX_SPEED}" \
  --acceleration "${ACCELERATION}" \
  --max-acceleration "${MAX_ACCELERATION}" \
  --max-position-delta "${MAX_POSITION_DELTA}" \
  --max-angular-delta "${MAX_ANGULAR_DELTA}" \
  --max-target-step "${MAX_TARGET_STEP}" \
  --max-target-speed "${MAX_TARGET_SPEED}" \
  --max-angular-step "${MAX_ANGULAR_STEP}" \
  --max-angular-speed "${MAX_ANGULAR_SPEED}" \
  --max-tcp-x "${MAX_TCP_X}" \
  --stale-after 0.25 \
  --gripper-speed 0.5 \
  --gripper-force 0.5 \
  --gripper-deadband 0.05 \
  --gripper-min-interval 0.1
