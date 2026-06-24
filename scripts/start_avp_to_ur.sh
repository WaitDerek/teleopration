#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

PUBLIC_HOST="${PUBLIC_HOST:-172.20.10.6}"

echo "Open on Vision Pro: https://${PUBLIC_HOST}:8012/go"

mamba run -n fast teleop-publish-avp-pose \
  --cert "${REPO_ROOT}/cert.pem" \
  --key "${REPO_ROOT}/key.pem" \
  --public-host "${PUBLIC_HOST}" \
  --topic Target_Pose \
  --gripper-topic Gripper_Command \
  --calibration-delay-samples 30 \
  --position-scale 0.05 \
  --max-position-norm 0.05 \
  --image-opacity 0.05 \
  --rate 30
