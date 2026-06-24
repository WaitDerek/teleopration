#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

PUBLIC_HOST="${PUBLIC_HOST:-172.20.10.6}"
POSITION_SCALE="${POSITION_SCALE:-0.05}"
ORIENTATION_SCALE="${ORIENTATION_SCALE:-0.25}"
MAX_POSITION_NORM="${MAX_POSITION_NORM:-0.05}"
MAX_ANGULAR_NORM="${MAX_ANGULAR_NORM:-0.25}"

echo "Open on Vision Pro: https://${PUBLIC_HOST}:8012/go"
echo "AVP scaling: position=${POSITION_SCALE}, orientation=${ORIENTATION_SCALE}, max_position=${MAX_POSITION_NORM}m, max_angle=${MAX_ANGULAR_NORM}rad"

mamba run -n fast teleop-publish-avp-pose \
  --cert "${REPO_ROOT}/cert.pem" \
  --key "${REPO_ROOT}/key.pem" \
  --public-host "${PUBLIC_HOST}" \
  --topic Target_Pose \
  --gripper-topic Gripper_Command \
  --calibration-delay-samples 30 \
  --position-scale "${POSITION_SCALE}" \
  --orientation-scale "${ORIENTATION_SCALE}" \
  --max-position-norm "${MAX_POSITION_NORM}" \
  --max-angular-norm "${MAX_ANGULAR_NORM}" \
  --image-opacity 0.05 \
  --rate 30
