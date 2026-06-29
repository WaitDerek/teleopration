#!/usr/bin/env bash
set -u -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

CONFIG_FILE="${CONFIG_FILE:-${REPO_ROOT}/config/teleop.env}"
if [[ -f "${CONFIG_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${CONFIG_FILE}"
  set +a
fi

ROBOT_IP="${ROBOT_IP:-192.168.56.2}"
PUBLIC_HOST="${PUBLIC_HOST:-}"
PORT="${PORT:-8012}"
CAMERA_INDEX="${CAMERA_INDEX:-0}"
CAMERA_SCAN_MAX="${CAMERA_SCAN_MAX:-8}"
CAPTURE_WIDTH="${CAPTURE_WIDTH:-1280}"
CAPTURE_HEIGHT="${CAPTURE_HEIGHT:-720}"
CERT_FILE="${CERT_FILE:-${REPO_ROOT}/cert.pem}"
KEY_FILE="${KEY_FILE:-${REPO_ROOT}/key.pem}"
CHECK_GRIPPER="${CHECK_GRIPPER:-1}"
CHECK_CAMERA="${CHECK_CAMERA:-1}"
USE_V4L2="${USE_V4L2:-1}"

status=0

ok() {
  printf '[OK] %s: %s\n' "$1" "$2"
}

warn() {
  printf '[WARN] %s: %s\n' "$1" "$2"
}

fail() {
  printf '[FAIL] %s: %s\n' "$1" "$2"
  status=1
}

run_and_capture() {
  local __resultvar="$1"
  shift
  local output
  output="$("$@" 2>&1)"
  local rc=$?
  printf -v "${__resultvar}" '%s' "${output}"
  return "${rc}"
}

echo "Teleoperation readiness check"
echo "repo=${REPO_ROOT}"
echo "robot_ip=${ROBOT_IP}"
echo "public_host=${PUBLIC_HOST:-<unset>}"
echo "camera_index=${CAMERA_INDEX}"

if run_and_capture env_check mamba run -n fast python -V; then
  ok "mamba env" "${env_check}"
else
  fail "mamba env" "could not run python inside env 'fast'"
fi

if [[ -z "${PUBLIC_HOST}" ]]; then
  fail "public host" "set PUBLIC_HOST to the computer IP opened from Vision Pro"
else
  local_ips="$(hostname -I 2>/dev/null || true)"
  if [[ " ${local_ips} " == *" ${PUBLIC_HOST} "* ]]; then
    ok "public host" "matches local interface list: ${PUBLIC_HOST}"
  else
    fail "public host" "not found on this machine. local_ips=${local_ips}"
  fi
fi

if [[ -f "${CERT_FILE}" && -f "${KEY_FILE}" ]]; then
  ok "tls files" "cert=$(basename "${CERT_FILE}") key=$(basename "${KEY_FILE}")"
else
  fail "tls files" "missing cert/key: cert=${CERT_FILE} key=${KEY_FILE}"
fi

if run_and_capture avp_deps_check mamba run -n fast python -c "import aiohttp, cv2, vuer"; then
  ok "avp deps" "aiohttp, cv2, vuer importable"
else
  fail "avp deps" "${avp_deps_check}"
fi

port_output="$(ss -ltnp 2>/dev/null | awk -v p=":${PORT}" '$4 ~ (p "$") {print}')"
if [[ -z "${port_output}" ]]; then
  ok "port ${PORT}" "free"
else
  warn "port ${PORT}" "already listening: ${port_output}"
fi

if [[ "${CHECK_CAMERA}" == "1" ]]; then
  if run_and_capture camera_output env CAMERA_INDEX="${CAMERA_INDEX}" CAMERA_SCAN_MAX="${CAMERA_SCAN_MAX}" CAPTURE_WIDTH="${CAPTURE_WIDTH}" CAPTURE_HEIGHT="${CAPTURE_HEIGHT}" USE_V4L2="${USE_V4L2}" mamba run -n fast python - <<'PY'
import os
import sys

import cv2

camera_index = os.environ["CAMERA_INDEX"]
scan_max = int(os.environ["CAMERA_SCAN_MAX"])
width = int(os.environ["CAPTURE_WIDTH"])
height = int(os.environ["CAPTURE_HEIGHT"])
use_v4l2 = os.environ.get("USE_V4L2", "1") == "1"
backend = cv2.CAP_V4L2 if use_v4l2 and hasattr(cv2, "CAP_V4L2") else cv2.CAP_ANY

def open_camera(index: int):
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened() and backend != cv2.CAP_ANY:
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None
    return cap, frame

if camera_index == "auto":
    for idx in range(scan_max):
        opened = open_camera(idx)
        if opened is None:
            continue
        cap, frame = opened
        frame_h, frame_w = frame.shape[:2]
        cap.release()
        print(f"camera_ready index={idx} frame={frame_w}x{frame_h}")
        sys.exit(0)
    print(f"camera_auto_scan_failed range=0..{scan_max - 1}")
    sys.exit(1)

index = int(camera_index)
opened = open_camera(index)
if opened is None:
    print(f"camera_open_failed index={index}")
    sys.exit(1)

cap, frame = opened
frame_h, frame_w = frame.shape[:2]
cap.release()
print(f"camera_ready index={index} frame={frame_w}x{frame_h}")
PY
  then
    ok "usb camera" "${camera_output}"
  else
    fail "usb camera" "${camera_output}"
  fi
else
  warn "usb camera" "skipped"
fi

ur_args=(mamba run -n fast teleop-check-ur10e-state --robot-ip "${ROBOT_IP}")
if [[ "${CHECK_GRIPPER}" != "1" ]]; then
  ur_args+=(--skip-gripper)
fi

if run_and_capture ur_output "${ur_args[@]}"; then
  printf '%s\n' "${ur_output}"
else
  printf '%s\n' "${ur_output}"
fi

if [[ "${CHECK_GRIPPER}" == "1" ]]; then
  if grep -q "teleop_with_gripper_ready=true" <<<"${ur_output}"; then
    ok "ur backend" "arm and gripper ready"
  else
    fail "ur backend" "arm or gripper not ready"
  fi
else
  if grep -q "teleop_without_gripper_ready=true" <<<"${ur_output}"; then
    ok "ur backend" "arm ready"
  else
    fail "ur backend" "arm not ready"
  fi
fi

if [[ "${status}" -eq 0 ]]; then
  echo "teleop_session_ready=true"
else
  echo "teleop_session_ready=false"
fi

exit "${status}"
