#!/usr/bin/env bash
# tmux launcher for balloon-race: sim → heartbeat → image + camera + control.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP="${PYTHON_ROOT}/flightSetup.json"
SESSION="balloon_race"
MODE="synth"
VIZ=0
GZ=0
GZ_MODEL="rc_cessna"
MODEL_SET=0
NO_SIM=0
CONTAINER_NAME="${PX4_JSBSIM_DOCKER_NAME:-px4-noble-jsbsim-rascal}"
# Balloon race needs distinct UDP feeds for control + image-source.
MAVLINK_FANOUT="${MAVLINK_FANOUT:-1}"
MAVLINK_CONTROL_PORT="${MAVLINK_CONTROL_PORT:-14540}"
MAVLINK_IMAGE_PORT="${MAVLINK_IMAGE_PORT:-14541}"
# Wait for PX4 heartbeat on control UDP after fan-out before starting peers.
HEARTBEAT_TIMEOUT_S="${HEARTBEAT_TIMEOUT_S:-120}"

usage() {
  echo "Usage: $0 [--viz] [--gz] [--model rc_cessna|advanced_plane] [--setup PATH] [--session NAME] [--no-sim]"
  echo "  --viz     FG viz sim + fg image capture (default: headless synth)"
  echo "  --gz      Gazebo plane + onboard camera (--mode gz); exclusive with --viz"
  echo "  --model   gz model (requires --gz); default rc_cessna"
  echo "  --no-sim  control connects to existing sim"
  echo "  Enables mavlink-server fan-out by default (MAVLINK_FANOUT=1);"
  echo "  aborts image/control if fan-out is not running."
  echo "  After sim/fan-out: wait for HEARTBEAT on UDP ${MAVLINK_CONTROL_PORT}"
  echo "  (timeout HEARTBEAT_TIMEOUT_S=${HEARTBEAT_TIMEOUT_S}s, fail if none)."
  exit 0
}

mavlink_fanout_up() {
  if pgrep -f '[m]avlink-server' >/dev/null 2>&1; then
    return 0
  fi
  if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}-mavlink"; then
    return 0
  fi
  if [[ "${GZ}" -eq 1 ]] && docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}-mavlink"; then
    return 0
  fi
  return 1
}

# Bind control UDP briefly, wait for vehicle HEARTBEAT, then release the port.
wait_control_heartbeat() {
  local port="$1"
  local timeout_s="$2"
  echo "Waiting for MAVLink heartbeat on UDP ${port} (timeout ${timeout_s}s)..."
  if ! python3 - "$port" "$timeout_s" <<'PY'
import sys
import time

from pymavlink import mavutil

port = int(sys.argv[1])
timeout_s = float(sys.argv[2])
master = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}")
deadline = time.time() + timeout_s
try:
    while time.time() < deadline:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is None:
            continue
        src = msg.get_srcSystem()
        comp = msg.get_srcComponent()
        if src in (0, 255):
            continue
        autopilot = int(getattr(msg, "autopilot", mavutil.mavlink.MAV_AUTOPILOT_INVALID))
        if comp == 191 and autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            continue
        if autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID and int(
            getattr(msg, "type", 0)
        ) in (
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_TYPE_GENERIC,
        ):
            continue
        print(f"Heartbeat from sys={src} comp={comp}")
        sys.exit(0)
finally:
    try:
        master.close()
    except Exception:
        pass
print(f"No MAVLink heartbeat on UDP {port} within {timeout_s:.0f}s", file=sys.stderr)
sys.exit(1)
PY
  then
    echo "Error: no heartbeat on control UDP ${port} within ${timeout_s}s; not launching image/camera/control" >&2
    return 1
  fi
  return 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --viz) VIZ=1; MODE="fg" ;;
    --gz) GZ=1; MODE="gz" ;;
    --model) MODEL_SET=1; GZ_MODEL="$2"; shift ;;
    --no-sim) NO_SIM=1 ;;
    --setup) SETUP="$2"; shift ;;
    --session) SESSION="$2"; shift ;;
    -h|--help) usage ;;
    *) echo "Unknown: $1" >&2; exit 1 ;;
  esac
  shift
done

if [[ "${VIZ}" -eq 1 && "${GZ}" -eq 1 ]]; then
  echo "Error: --viz and --gz are mutually exclusive" >&2
  exit 2
fi
if [[ "${MODEL_SET}" -eq 1 && "${GZ}" -eq 0 ]]; then
  echo "Error: --model requires --gz" >&2
  exit 2
fi

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux required" >&2
  exit 1
fi

# Prefer the active conda env's interpreter (user prompt is often (pigeon)).
PYTHON="${PYTHON:-python3}"

# Do not unset conda LD_LIBRARY_PATH: pyzmq/OpenCV need those libs. Unsetting
# mixed libzmq and aborted the camera pane (Assertion failed: !_more src/fq.cpp).
# tmux/bash "libtinfo no version information" lines are cosmetic.

if ! "${PYTHON}" -c "import cv2, numpy, zmq, pymavlink" >/dev/null 2>&1; then
  echo "Error: balloon-race Python deps missing in $("${PYTHON}" -c 'import sys; print(sys.executable)')." >&2
  echo "  Install: ${PYTHON} -m pip install -r ${PYTHON_ROOT}/requirements.txt" >&2
  exit 1
fi

# Quiet when no tmux server yet (fresh machine / after kill-server).
tmux kill-session -t "${SESSION}" 2>/dev/null || true

if [[ "${GZ}" -eq 1 ]]; then
  CONTAINER_NAME="${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}"
  SIM_CMD="MAVLINK_FANOUT=${MAVLINK_FANOUT} bash ${PYTHON_ROOT}/scripts/runSimGzPlane.sh --mavlink-server --setup ${SETUP} --model ${GZ_MODEL}"
else
  SIM_CMD="MAVLINK_FANOUT=${MAVLINK_FANOUT} bash ${PYTHON_ROOT}/scripts/runSimJsbsimRascal.sh"
  if [[ "${MAVLINK_FANOUT}" == "1" ]]; then
    SIM_CMD+=" --mavlink-server"
  fi
  if [[ "${VIZ}" -eq 1 ]]; then
    SIM_CMD+=" --viz"
  fi
fi

IMG_CMD="PYTHONUNBUFFERED=1 ${PYTHON} -u ${PYTHON_ROOT}/run_balloon_image_source.py --mode ${MODE} --setup ${SETUP} --udp ${MAVLINK_IMAGE_PORT}"
CAM_CMD="PYTHONUNBUFFERED=1 ${PYTHON} -u ${PYTHON_ROOT}/run_balloon_camera.py --setup ${SETUP}"
# Headless e2e / no GUI: BALLOON_CAMERA_NO_DISPLAY=1 or --no-display
if [[ "${BALLOON_CAMERA_NO_DISPLAY:-0}" == "1" ]]; then
  CAM_CMD+=" --no-display"
fi
CTL_CMD="PYTHONUNBUFFERED=1 ${PYTHON} -u ${PYTHON_ROOT}/run_balloon_control.py --setup ${SETUP} --udp ${MAVLINK_CONTROL_PORT} --no-plot"
# Optional fixed CSV path for verification (BALLOON_RACE_CSV=/path/to.csv)
if [[ -n "${BALLOON_RACE_CSV:-}" ]]; then
  CTL_CMD+=" --csv ${BALLOON_RACE_CSV}"
fi
# MAVLink ports with mavlink-server fan-out (started by runSimJsbsimRascal.sh):
#   14550 GCS/QGC, 14540 control, 14541 image-source
# Control always attaches: race owns sim, or user passed --no-sim for an existing sim.
CTL_CMD+=" --no-sim"
if [[ "${VIZ}" -eq 1 ]]; then
  CTL_CMD+=" --viz --spawn-fg-balloons"
fi
if [[ "${GZ}" -eq 1 ]]; then
  IMG_CMD+=" --container ${CONTAINER_NAME}"
  CTL_CMD+=" --gz --spawn-gz-balloons"
  CTL_CMD+=" --gz-container ${CONTAINER_NAME}"
fi

# One window; peers are split panes (tiled) after heartbeat.
# remain-on-exit: if sim/fan-out dies, keep the pane so the real error is visible
# (otherwise the session vanishes and the next tmux call prints "no server running").
if [[ "${NO_SIM}" -eq 0 ]]; then
  tmux new-session -d -s "${SESSION}" -n race "${SIM_CMD}"
  tmux set-option -t "${SESSION}" remain-on-exit on
  sleep 8
else
  tmux new-session -d -s "${SESSION}" -n race "echo 'NO_SIM: using existing sim'; bash"
  tmux set-option -t "${SESSION}" remain-on-exit on
fi

if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "Error: tmux session '${SESSION}' exited during sim startup." >&2
  echo "  Usual cause: mavlink-server fan-out failed (missing python/bin/mavlink-server)." >&2
  echo "  Fix: python/scripts/fetch_mavlink_server.sh   # then re-run ./run_balloon_race.sh" >&2
  echo "  Or inspect a prior sim log if remain-on-exit caught it." >&2
  exit 1
fi
tmux select-pane -t "${SESSION}:0.0" -T sim

if [[ "${MAVLINK_FANOUT}" == "1" ]] && ! mavlink_fanout_up; then
  echo "Error: mavlink-server fan-out failed to start; not launching image/camera/control" >&2
  echo "  Fix: python/scripts/fetch_mavlink_server.sh" >&2
  echo "  Or rebuild Noble image / set MAVLINK_FANOUT=0 (not recommended for race)." >&2
  echo "  Attach to see sim pane: tmux attach -t ${SESSION}" >&2
  exit 1
fi

if ! wait_control_heartbeat "${MAVLINK_CONTROL_PORT}" "${HEARTBEAT_TIMEOUT_S}"; then
  tmux kill-session -t "${SESSION}" 2>/dev/null || true
  exit 1
fi

# Control first so OFFBOARD engages before the plane burns remaining AGL.
# -d keeps focus on sim; title panes by id (tiled layout may renumber indices).
_ctl_pane="$(tmux split-window -d -t "${SESSION}:0.0" -P -F '#{pane_id}' "${CTL_CMD}")"
tmux select-pane -t "${_ctl_pane}" -T control
_img_pane="$(tmux split-window -d -t "${SESSION}:0.0" -P -F '#{pane_id}' "${IMG_CMD}")"
tmux select-pane -t "${_img_pane}" -T image
_cam_pane="$(tmux split-window -d -t "${SESSION}:0.0" -P -F '#{pane_id}' "${CAM_CMD}")"
tmux select-pane -t "${_cam_pane}" -T camera
tmux select-layout -t "${SESSION}:0" tiled

echo "tmux session '${SESSION}' started (attach: tmux attach -t ${SESSION})"
echo "panes (window race):"
tmux list-panes -t "${SESSION}:0" -F '  #{session_name}:#{window_index}.#{pane_index} #{pane_title}'
